"""ROS 2 adapter from private Gazebo bridge topics to public run contracts."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import copy
from dataclasses import dataclass

from .aggregation import AggregationFault, NativeOdometry
from .epoch import OutputEpochGate
from .live import LiveAdapter
from .model import (
    AdapterFault,
    AdapterSummary,
    NativeImage,
    PublicFrame,
    PublicGroundTruth,
    RangeSequence,
)
from .payload import (
    PayloadTracker,
    PublicPayloadState,
    parse_joint_state,
    payload_state_message,
)
from .topics import (
    PAYLOAD_IDS,
    camera_topics_for_world,
    contact_topic_for_world,
    private_command_topics_for_world,
    private_payload_topic,
    private_publisher_topics_for_world,
)


@dataclass(frozen=True)
class _PublicRange:
    sim_timestamp_ns: int
    message: object


def _nanoseconds(stamp) -> int:
    sec = stamp.sec
    nanosec = stamp.nanosec
    if type(sec) is not int or type(nanosec) is not int or sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise AdapterFault("ROS timestamp is malformed")
    return sec * 1_000_000_000 + nanosec


def _set_stamp(stamp, timestamp_ns: int) -> None:
    stamp.sec, stamp.nanosec = divmod(timestamp_ns, 1_000_000_000)


def _vector3(value) -> tuple[float, float, float]:
    return (value.x, value.y, value.z)


def _quaternion(value) -> tuple[float, float, float, float]:
    return (value.x, value.y, value.z, value.w)


def _qos(depth: int, *, reliable: bool):
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=depth,
        reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
        durability=DurabilityPolicy.VOLATILE,
    )


def _node_base():
    from rclpy.node import Node

    return Node


class GazeboAdapterNode(_node_base()):
    """Own the single public Gazebo ROS publisher set for one run."""

    def __init__(
        self,
        *,
        run_id: str,
        expected_frames: int,
        public_epoch_native_ns: int,
        world_name: str = "phase3_foundation",
        width_px: int = 320,
        height_px: int = 240,
        on_completed: Callable[[AdapterSummary], None] | None = None,
        on_fault: Callable[[str], None] | None = None,
    ) -> None:
        from nav_msgs.msg import Odometry
        from ros_gz_interfaces.msg import Contacts
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, LaserScan
        from simulation_interfaces.msg import FrameMetadata, GroundTruth, PayloadState

        super().__init__("drone_sim_gazebo_adapter")
        self._run_id = run_id
        self._world_name = world_name
        self._competition = world_name == "competition_mission"
        self._expected_frames = expected_frames
        self._contact_topic = contact_topic_for_world(world_name)
        self._live = LiveAdapter(
            run_id=run_id,
            expected_frames=expected_frames,
            width_px=width_px,
            height_px=height_px,
        )
        self._on_completed = on_completed or (lambda _summary: None)
        self._on_fault = on_fault or (lambda _reason: None)
        self._completion_reported = False
        self._faulted = False
        self._output_active = False
        self._output_epoch = OutputEpochGate(
            expected_frames=expected_frames,
            public_epoch_native_ns=public_epoch_native_ns,
        )
        self._pre_zero_outputs: deque[object] = deque()
        self._last_public_clock_ns: int | None = None
        self._clock_type = Clock
        self._image_type = Image
        self._metadata_type = FrameMetadata
        self._ground_truth_type = GroundTruth
        self._payload_state_type = PayloadState
        self._image_publishers = {
            stream: self.create_publisher(
                Image, f"/camera/{stream}/image_raw", _qos(100, reliable=True)
            )
            for stream in ("onboard", "observer")
        }
        self._metadata_publishers = {
            stream: self.create_publisher(
                FrameMetadata,
                f"/camera/{stream}/frame_metadata",
                _qos(100, reliable=True),
            )
            for stream in ("onboard", "observer")
        }
        self._ground_truth_publisher = self.create_publisher(
            GroundTruth, "/simulation/ground_truth", _qos(10, reliable=True)
        )
        self._payload_state_publisher = None
        self._range_publisher = None
        self._payload_trackers: dict[int, PayloadTracker] = {}
        self._range_sequence: RangeSequence | None = None
        if self._competition:
            from geometry_msgs.msg import PoseArray
            from std_msgs.msg import String

            self._payload_state_publisher = self.create_publisher(
                PayloadState,
                "/simulation/payload_state",
                _qos(100, reliable=True),
            )
            self._range_publisher = self.create_publisher(
                LaserScan,
                "/competition/range/downward",
                _qos(100, reliable=True),
            )
            self._payload_trackers = {
                aruco_id: PayloadTracker(
                    run_id=run_id,
                    aruco_id=aruco_id,
                    interval_ns=50_000_000,
                )
                for aruco_id in PAYLOAD_IDS
            }
            self._range_sequence = RangeSequence(
                expected_samples=expected_frames,
                interval_ns=50_000_000,
            )
            self.create_subscription(
                LaserScan,
                "/gazebo/private/range/downward",
                self._accept_range,
                _qos(10, reliable=False),
            )
            for aruco_id in PAYLOAD_IDS:
                self.create_subscription(
                    PoseArray,
                    private_payload_topic(aruco_id, "pose"),
                    lambda message, aruco_id=aruco_id: self._accept_payload_pose(
                        aruco_id, message
                    ),
                    _qos(10, reliable=True),
                )
                self.create_subscription(
                    Contacts,
                    private_payload_topic(aruco_id, "contact_state"),
                    lambda message, aruco_id=aruco_id: self._accept_payload_contact(
                        aruco_id, message
                    ),
                    _qos(10, reliable=True),
                )
                self.create_subscription(
                    String,
                    private_payload_topic(aruco_id, "joint_state"),
                    lambda message, aruco_id=aruco_id: self._accept_payload_attachment(
                        aruco_id, message
                    ),
                    _qos(10, reliable=True),
                )
        self._clock_publisher = self.create_publisher(
            Clock, "/clock", _qos(1000, reliable=True)
        )
        self.create_subscription(
            Clock,
            "/gazebo/private/clock",
            self._accept_clock,
            _qos(1000, reliable=True),
        )
        self.create_subscription(
            Odometry,
            "/gazebo/private/iris/odometry",
            self._accept_odometry,
            _qos(10, reliable=False),
        )
        self.create_subscription(
            Contacts,
            self._contact_topic,
            self._accept_contacts,
            _qos(10, reliable=False),
        )

    def transport_ready(self) -> bool:
        """Return true once every private bridge publisher is visible."""
        publishers_ready = all(
            self.count_publishers(topic) == 1
            for topic in private_publisher_topics_for_world(self._world_name)
        )
        commands_ready = all(
            self.count_subscribers(topic) == 1
            for topic in private_command_topics_for_world(self._world_name)
        )
        return publishers_ready and commands_ready

    def recorders_ready(self) -> bool:
        """Require at least one archival consumer for every physical sample."""
        topics = (
            "/camera/onboard/image_raw",
            "/camera/onboard/frame_metadata",
            "/camera/observer/image_raw",
            "/camera/observer/frame_metadata",
            "/simulation/ground_truth",
        )
        if self._competition:
            topics = (
                *topics,
                "/simulation/payload_state",
                "/competition/range/downward",
            )
        return all(self.count_subscribers(topic) >= 1 for topic in topics)

    def freeze_output(self) -> None:
        self._faulted = True
        self._output_active = False

    def activate_output(self) -> None:
        """Arm output before the configured native public epoch."""
        if self._faulted or self._output_active or self._completion_reported:
            return
        try:
            self._output_epoch.request_activation()
        except AdapterFault as error:
            self._fail(error)

    def _publish_clock(self, timestamp_ns: int, message=None) -> None:
        if (
            self._last_public_clock_ns is not None
            and timestamp_ns <= self._last_public_clock_ns
        ):
            return
        if message is None:
            message = self._clock_type()
            _set_stamp(message.clock, timestamp_ns)
        self._clock_publisher.publish(message)
        self._last_public_clock_ns = timestamp_ns

    def _fail(self, error: Exception) -> None:
        if not self._faulted:
            self._faulted = True
            self._output_active = False
            self._on_fault(str(error))

    def _accept_clock(self, message) -> None:
        if self._faulted or self._completion_reported:
            return
        try:
            timestamp_ns = _nanoseconds(message.clock)
            public_timestamp_ns = self._output_epoch.accept_clock(timestamp_ns)
            if public_timestamp_ns is not None:
                if public_timestamp_ns == 0 and not self._output_active:
                    for stream, topic in zip(
                        ("onboard", "observer"),
                        camera_topics_for_world(self._world_name),
                        strict=True,
                    ):
                        self.create_subscription(
                            self._image_type,
                            topic,
                            lambda message, stream=stream: self._accept_image(
                                stream, message
                            ),
                            _qos(20, reliable=True),
                        )
                    self._output_active = True
                    self._publish_clock(0)
                    queued = tuple(self._pre_zero_outputs)
                    self._pre_zero_outputs.clear()
                    self._publish(queued)
                elif self._output_active:
                    self._publish_clock(public_timestamp_ns)
        except (AdapterFault, ValueError, TypeError) as error:
            self._fail(error)

    def _accept_image(self, stream: str, message) -> None:
        if self._faulted or self._completion_reported:
            return
        if not self._output_active and not self._output_epoch.activation_pending:
            return
        try:
            timestamp_ns = _nanoseconds(message.header.stamp)
            public_timestamp_ns = self._output_epoch.accept_camera(
                stream, timestamp_ns
            )
            if public_timestamp_ns is None:
                return
            output = self._live.accept_image(
                stream,
                NativeImage(
                    sim_timestamp_ns=public_timestamp_ns,
                    width=message.width,
                    height=message.height,
                    encoding=message.encoding,
                    step=message.step,
                    data=bytes(message.data),
                ),
            )
            self._emit(output)
        except (AdapterFault, AggregationFault, ValueError, TypeError) as error:
            self._fail(error)

    def _accept_odometry(self, message) -> None:
        if (
            self._faulted
            or not self._output_active
            and not self._output_epoch.activation_pending
        ):
            return
        try:
            timestamp_ns = _nanoseconds(message.header.stamp)
            public_timestamp_ns = self._output_epoch.rebase_sample(timestamp_ns)
            if public_timestamp_ns is None:
                return
            pose = message.pose.pose
            twist = message.twist.twist
            output = self._live.accept_odometry(
                NativeOdometry(
                    sim_timestamp_ns=public_timestamp_ns,
                    position_xyz=_vector3(pose.position),
                    orientation_xyzw=_quaternion(pose.orientation),
                    linear_velocity_xyz=_vector3(twist.linear),
                    angular_velocity_xyz=_vector3(twist.angular),
                )
            )
            self._emit(output)
        except (AdapterFault, AggregationFault, ValueError, TypeError) as error:
            self._fail(error)

    def _accept_contacts(self, message) -> None:
        if (
            self._faulted
            or not self._output_active
            and not self._output_epoch.activation_pending
        ):
            return
        try:
            timestamp_ns = _nanoseconds(message.header.stamp)
            public_timestamp_ns = self._output_epoch.rebase_sample(timestamp_ns)
            if public_timestamp_ns is None:
                return
            output = self._live.accept_contact(
                public_timestamp_ns, bool(message.contacts)
            )
            self._emit(output)
        except (AdapterFault, AggregationFault, ValueError, TypeError) as error:
            self._fail(error)

    def _sample_timestamp(self, message) -> int | None:
        timestamp_ns = _nanoseconds(message.header.stamp)
        return self._output_epoch.rebase_sample(timestamp_ns)

    def _accept_payload_contact(self, aruco_id: int, message) -> None:
        if (
            self._faulted
            or not self._output_active
            and not self._output_epoch.activation_pending
        ):
            return
        try:
            timestamp_ns = self._sample_timestamp(message)
            if timestamp_ns is not None:
                self._emit(
                    self._payload_trackers[aruco_id].accept_contact(
                        timestamp_ns,
                        bool(message.contacts),
                    )
                )
        except (AdapterFault, ValueError, TypeError) as error:
            self._fail(error)

    def _accept_payload_attachment(self, aruco_id: int, message) -> None:
        if (
            self._faulted
            or self._completion_reported
            or not self._output_active
            and not self._output_epoch.activation_pending
        ):
            return
        try:
            native_timestamp_ns, physical_state = parse_joint_state(message.data)
            timestamp_ns = self._output_epoch.rebase_sample(native_timestamp_ns)
            if timestamp_ns is not None:
                self._emit(
                    self._payload_trackers[aruco_id].accept_attachment(
                        timestamp_ns,
                        physical_state,
                    )
                )
        except (AdapterFault, ValueError, TypeError) as error:
            self._fail(error)

    def _accept_payload_pose(self, aruco_id: int, message) -> None:
        if (
            self._faulted
            or self._completion_reported
            or not self._output_active
            and not self._output_epoch.activation_pending
        ):
            return
        try:
            timestamp_ns = self._sample_timestamp(message)
            if timestamp_ns is None:
                return
            if len(message.poses) != 1:
                raise AdapterFault("payload pose vector must contain exactly one pose")
            tracker = self._payload_trackers[aruco_id]
            pose = message.poses[0]
            output = tracker.accept_pose(
                timestamp_ns,
                _vector3(pose.position),
                _quaternion(pose.orientation),
            )
            self._emit(output)
        except (AdapterFault, ValueError, TypeError) as error:
            self._fail(error)

    def _accept_range(self, message) -> None:
        if (
            self._faulted
            or self._completion_reported
            or not self._output_active
            and not self._output_epoch.activation_pending
        ):
            return
        try:
            timestamp_ns = self._sample_timestamp(message)
            if timestamp_ns is None:
                return
            assert self._range_sequence is not None
            self._range_sequence.accept(timestamp_ns)
            public = copy.deepcopy(message)
            _set_stamp(public.header.stamp, timestamp_ns)
            self._emit((_PublicRange(timestamp_ns, public),))
        except (AdapterFault, ValueError, TypeError) as error:
            self._fail(error)

    def _emit(self, output) -> None:
        if not output:
            return
        if self._output_active:
            self._publish(output)
            return
        limit = 14 if self._competition else 6
        if len(self._pre_zero_outputs) + len(output) > limit:
            raise AdapterFault("pre-zero output queue exceeded two public epochs")
        self._pre_zero_outputs.extend(output)

    def _publish(self, output) -> None:
        for value in output:
            if isinstance(value, PublicFrame):
                self._publish_clock(value.sim_timestamp_ns)
                image = self._image_type()
                _set_stamp(image.header.stamp, value.header_timestamp_ns)
                image.header.frame_id = f"camera/{value.stream}"
                image.height = value.height
                image.width = value.width
                image.encoding = value.encoding
                image.is_bigendian = False
                image.step = value.step
                image.data = value.data
                metadata = self._metadata_type()
                metadata.run_id = value.run_id
                _set_stamp(metadata.sim_timestamp, value.sim_timestamp_ns)
                metadata.frame_id = value.frame_id
                metadata.stream = value.stream
                self._image_publishers[value.stream].publish(image)
                self._metadata_publishers[value.stream].publish(metadata)
            elif isinstance(value, PublicGroundTruth):
                truth = self._ground_truth_type()
                truth.run_id = value.run_id
                _set_stamp(truth.sim_timestamp, value.sim_timestamp_ns)
                truth.vehicle_id = value.vehicle_id
                truth.pose.position.x, truth.pose.position.y, truth.pose.position.z = value.position_xyz
                (
                    truth.pose.orientation.x,
                    truth.pose.orientation.y,
                    truth.pose.orientation.z,
                    truth.pose.orientation.w,
                ) = value.orientation_xyzw
                (
                    truth.twist.linear.x,
                    truth.twist.linear.y,
                    truth.twist.linear.z,
                ) = value.linear_velocity_xyz
                (
                    truth.twist.angular.x,
                    truth.twist.angular.y,
                    truth.twist.angular.z,
                ) = value.angular_velocity_xyz
                truth.in_contact = value.in_contact
                self._ground_truth_publisher.publish(truth)
            elif isinstance(value, PublicPayloadState):
                self._publish_clock(value.sim_timestamp_ns)
                assert self._payload_state_publisher is not None
                self._payload_state_publisher.publish(
                    payload_state_message(value, self._payload_state_type)
                )
            elif isinstance(value, _PublicRange):
                self._publish_clock(value.sim_timestamp_ns)
                assert self._range_publisher is not None
                self._range_publisher.publish(value.message)
        self._maybe_complete()

    def _maybe_complete(self) -> None:
        competition_complete = (
            not self._competition
            or self._range_sequence is not None
            and self._range_sequence.accepted_samples == self._expected_frames
            and all(
                tracker.accepted_poses == self._expected_frames
                for tracker in self._payload_trackers.values()
            )
        )
        if (
            self._live.complete
            and competition_complete
            and not self._completion_reported
        ):
            self._completion_reported = True
            self._output_active = False
            self._on_completed(self._live.freeze())
