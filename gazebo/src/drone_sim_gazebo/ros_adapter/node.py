"""ROS 2 adapter from private Gazebo bridge topics to public run contracts."""

from __future__ import annotations

from collections.abc import Callable

from .aggregation import AggregationFault, NativeOdometry
from .live import LiveAdapter
from .model import AdapterFault, AdapterSummary, NativeImage, PublicFrame, PublicGroundTruth


_CONTACT_TOPIC = (
    "/world/phase3_foundation/model/ground_plane/link/ground_link/sensor/"
    "iris_ground_contact/contact"
)


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
        on_completed: Callable[[AdapterSummary], None] | None = None,
        on_fault: Callable[[str], None] | None = None,
    ) -> None:
        from nav_msgs.msg import Odometry
        from ros_gz_interfaces.msg import Contacts
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image
        from simulation_interfaces.msg import FrameMetadata, GroundTruth

        super().__init__("drone_sim_gazebo_adapter")
        self._run_id = run_id
        self._live = LiveAdapter(run_id=run_id, expected_frames=expected_frames)
        self._on_completed = on_completed or (lambda _summary: None)
        self._on_fault = on_fault or (lambda _reason: None)
        self._completion_reported = False
        self._faulted = False
        self._image_type = Image
        self._metadata_type = FrameMetadata
        self._ground_truth_type = GroundTruth
        self._image_publishers = {
            stream: self.create_publisher(
                Image, f"/camera/{stream}/image_raw", _qos(5, reliable=True)
            )
            for stream in ("onboard", "observer")
        }
        self._metadata_publishers = {
            stream: self.create_publisher(
                FrameMetadata,
                f"/camera/{stream}/frame_metadata",
                _qos(5, reliable=True),
            )
            for stream in ("onboard", "observer")
        }
        self._ground_truth_publisher = self.create_publisher(
            GroundTruth, "/simulation/ground_truth", _qos(10, reliable=False)
        )
        self._clock_publisher = self.create_publisher(
            Clock, "/clock", _qos(1, reliable=False)
        )
        self.create_subscription(
            Clock,
            "/gazebo/private/clock",
            self._accept_clock,
            _qos(1, reliable=False),
        )
        for stream in ("onboard", "observer"):
            self.create_subscription(
                Image,
                f"/gazebo/private/camera/{stream}/image",
                lambda message, stream=stream: self._accept_image(stream, message),
                _qos(5, reliable=False),
            )
        self.create_subscription(
            Odometry,
            "/gazebo/private/iris/odometry",
            self._accept_odometry,
            _qos(10, reliable=False),
        )
        self.create_subscription(
            Contacts,
            _CONTACT_TOPIC,
            self._accept_contacts,
            _qos(10, reliable=False),
        )

    def transport_ready(self) -> bool:
        """Return true once every private bridge publisher is visible."""
        topics = (
            "/gazebo/private/clock",
            "/gazebo/private/camera/onboard/image",
            "/gazebo/private/camera/observer/image",
            "/gazebo/private/iris/odometry",
            _CONTACT_TOPIC,
        )
        return all(self.count_publishers(topic) == 1 for topic in topics)

    def recorders_ready(self) -> bool:
        """Require at least one archival consumer for every physical sample."""
        topics = (
            "/camera/onboard/image_raw",
            "/camera/onboard/frame_metadata",
            "/camera/observer/image_raw",
            "/camera/observer/frame_metadata",
            "/simulation/ground_truth",
        )
        return all(self.count_subscribers(topic) >= 1 for topic in topics)

    def freeze_output(self) -> None:
        self._faulted = True

    def _fail(self, error: Exception) -> None:
        if not self._faulted:
            self._faulted = True
            self._on_fault(str(error))

    def _accept_clock(self, message) -> None:
        if not self._faulted:
            self._clock_publisher.publish(message)

    def _accept_image(self, stream: str, message) -> None:
        if self._faulted:
            return
        try:
            output = self._live.accept_image(
                stream,
                NativeImage(
                    sim_timestamp_ns=_nanoseconds(message.header.stamp),
                    width=message.width,
                    height=message.height,
                    encoding=message.encoding,
                    step=message.step,
                    data=bytes(message.data),
                ),
            )
            self._publish(output)
        except (AdapterFault, AggregationFault, ValueError, TypeError) as error:
            self._fail(error)

    def _accept_odometry(self, message) -> None:
        if self._faulted:
            return
        try:
            pose = message.pose.pose
            twist = message.twist.twist
            output = self._live.accept_odometry(
                NativeOdometry(
                    sim_timestamp_ns=_nanoseconds(message.header.stamp),
                    position_xyz=_vector3(pose.position),
                    orientation_xyzw=_quaternion(pose.orientation),
                    linear_velocity_xyz=_vector3(twist.linear),
                    angular_velocity_xyz=_vector3(twist.angular),
                )
            )
            self._publish(output)
        except (AdapterFault, AggregationFault, ValueError, TypeError) as error:
            self._fail(error)

    def _accept_contacts(self, message) -> None:
        if self._faulted:
            return
        try:
            output = self._live.accept_contact(
                _nanoseconds(message.header.stamp), bool(message.contacts)
            )
            self._publish(output)
        except (AdapterFault, AggregationFault, ValueError, TypeError) as error:
            self._fail(error)

    def _publish(self, output) -> None:
        for value in output:
            if isinstance(value, PublicFrame):
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
        if self._live.complete and not self._completion_reported:
            self._completion_reported = True
            self._on_completed(self._live.freeze())
