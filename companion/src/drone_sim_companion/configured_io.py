"""Live ROS I/O owned by one configured competition mission."""

from __future__ import annotations

import threading
from types import SimpleNamespace


_MISSION_EVENT_TOPIC = "/simulation/mission_events"
_MISSION_EVENT_TYPE = "simulation_interfaces/msg/MissionEvent"
_SCOREKEEPER_NODE = "drone_sim_scorekeeper"
_ROSBAG_NODE_PREFIX = "rosbag2_recorder_"
_MISSION_SEQUENCE = (
    ("FM1", "STARTED"),
    ("FM1", "COMPLETE"),
    ("FM2", "STARTED"),
    ("FM2", "COMPLETE"),
    ("FM3_3", "STARTED"),
    ("FM3_3", "COMPLETE"),
    ("FM3_4", "STARTED"),
    ("FM3_4", "COMPLETE"),
    ("HOME", "STARTED"),
    ("HOME", "DISARMED"),
    ("HOME", "COMPLETE"),
)
_SEARCH_DELIVERY_SEQUENCE = (
    ("SEARCH", "STARTED"),
    ("SEARCH", "COMPLETE"),
    ("DELIVERY", "STARTED"),
    ("DELIVERY", "COMPLETE"),
    ("HOME", "STARTED"),
    ("HOME", "DISARMED"),
    ("HOME", "COMPLETE"),
)
_SENSOR_FRESHNESS_NS = 500_000_000


def _load_live_dependencies() -> SimpleNamespace:
    """Keep ROS, OpenCV, and the nested flight package out of offline imports."""

    from drone.sensors.camera._camera_manager import CameraManager
    from drone.sensors.camera.camera import Camera
    from drone.sensors.lidar.clearance import (
        AttitudeSample,
        ClearanceCalibration,
        ClearanceUnavailableError,
        project_vertical_clearance,
    )
    from drone.sensors.lidar.lidar import LidarSample
    from rclpy.duration import Duration
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, LaserScan
    from simulation_interfaces.msg import MissionEvent, PayloadState
    from simulation_interfaces.srv import PayloadCommand

    from .comp2026_host import (
        RosFrameSource,
        RosLidar,
        SimulationClock,
        StaleSensorError,
    )
    from .configured_payload import ConfiguredPayloadClient
    from .runtime_node import _create_simulator_camera

    return SimpleNamespace(
        SimulationClock=SimulationClock,
        RosFrameSource=RosFrameSource,
        RosLidar=RosLidar,
        ConfiguredPayloadClient=ConfiguredPayloadClient,
        CameraManager=CameraManager,
        Camera=Camera,
        LidarSample=LidarSample,
        StaleSensorError=StaleSensorError,
        AttitudeSample=AttitudeSample,
        ClearanceCalibration=ClearanceCalibration,
        ClearanceUnavailableError=ClearanceUnavailableError,
        project_vertical_clearance=project_vertical_clearance,
        create_simulator_camera=_create_simulator_camera,
        Duration=Duration,
        QoSProfile=QoSProfile,
        ReliabilityPolicy=ReliabilityPolicy,
        DurabilityPolicy=DurabilityPolicy,
        Image=Image,
        LaserScan=LaserScan,
        MissionEvent=MissionEvent,
        PayloadState=PayloadState,
        PayloadCommand=PayloadCommand,
    )


def _timestamp_ns(stamp: object) -> int:
    seconds = getattr(stamp, "sec", None)
    nanoseconds = getattr(stamp, "nanosec", None)
    if (
        type(seconds) is not int
        or seconds < 0
        or type(nanoseconds) is not int
        or not 0 <= nanoseconds < 1_000_000_000
    ):
        raise ValueError("message source timestamp is malformed")
    return seconds * 1_000_000_000 + nanoseconds


class CompetitionIO:
    """Bridge configured competition operations to public simulation evidence."""

    def __init__(self, config: object, node: object, emit) -> None:
        if (
            getattr(config, "mission", None) != "configured"
            or getattr(config, "scenario", None) not in {"competition_v1", "search_delivery_v1"}
        ):
            raise ValueError("competition I/O requires a configured competition mission")
        run_id = getattr(config, "run_id", None)
        if not isinstance(run_id, str) or not run_id:
            raise ValueError("competition I/O requires a nonempty run_id")
        scenario_path = getattr(config, "scenario_path", None)
        if scenario_path is None:
            raise ValueError("competition I/O requires a resolved scenario path")
        if not callable(emit):
            raise TypeError("competition I/O emit callback must be callable")

        self._config = config
        self._mission_sequence = (
            _SEARCH_DELIVERY_SEQUENCE if config.scenario == "search_delivery_v1" else _MISSION_SEQUENCE
        )
        self._node = node
        self._emit = emit
        self._deps = _load_live_dependencies()
        self._lock = threading.RLock()
        self._clock = self._deps.SimulationClock()
        self._frame_source = self._deps.RosFrameSource(width_px=640, height_px=480)
        self._pending_images: dict[int, object] = {}
        self._closed = False
        self._next_event_id = 0
        self._last_event_timestamp_ns: int | None = None
        self._camera_started = False
        self._last_camera_sequence = 0
        self._cached_marker_id: int | None = None
        self._cached_marker_timestamp_ns: int | None = None
        self._cached_marker: object | None = None
        self._attitude_sequence = 0

        self._camera = self._deps.create_simulator_camera(
            self._deps.CameraManager,
            self._deps.Camera,
            self._frame_source,
            clock=self._clock,
            scenario_path=scenario_path,
        )
        self._lidar = self._deps.RosLidar(
            self._clock,
            sample_factory=self._deps.LidarSample,
        )
        self._clearance_calibration = self._deps.ClearanceCalibration(
            beam_direction_body_frd=(0.0, 0.0, 1.0),
            measured_reference_offset_body_frd_m=(0.0, 0.0, 0.1),
            lidar_mounting_offset_already_applied=True,
            max_tilt_rad=0.6,
            max_age_seconds=0.5,
            max_skew_seconds=0.1,
            locally_horizontal_planar_surface=True,
        )

        reliable = self._deps.ReliabilityPolicy.RELIABLE
        volatile_qos = self._deps.QoSProfile(
            depth=100,
            reliability=reliable,
            durability=self._deps.DurabilityPolicy.VOLATILE,
        )
        event_qos = self._deps.QoSProfile(
            depth=100,
            reliability=reliable,
            durability=self._deps.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        raw_payload_client = node.create_client(
            self._deps.PayloadCommand,
            "/simulation/payload_command",
        )
        payload_client_type = getattr(
            self._deps,
            "ConfiguredPayloadClient",
            None,
        )
        if payload_client_type is None:
            from .configured_payload import ConfiguredPayloadClient

            payload_client_type = ConfiguredPayloadClient
        self.payloads = payload_client_type(
            run_id,
            raw_payload_client,
            self._deps.PayloadCommand.Request,
        )
        self._subscriptions = (
            node.create_subscription(
                self._deps.Image,
                "/camera/onboard/image_raw",
                self._accept_image,
                volatile_qos,
            ),
            node.create_subscription(
                self._deps.LaserScan,
                "/competition/range/downward",
                self._accept_range,
                volatile_qos,
            ),
            node.create_subscription(
                self._deps.PayloadState,
                "/simulation/payload_state",
                self._accept_payload_state,
                volatile_qos,
            ),
        )
        self._mission_publisher = node.create_publisher(
            self._deps.MissionEvent,
            _MISSION_EVENT_TOPIC,
            event_qos,
        )

    @property
    def ready(self) -> bool:
        return self.payloads.ready and self._mission_event_consumers_ready()

    def accept_clock(self, timestamp_ns: int) -> None:
        with self._lock:
            if self._closed:
                raise RuntimeError("competition I/O is closed")
            self._clock.accept(timestamp_ns)
            eligible = sorted(
                stamp for stamp in self._pending_images if stamp <= timestamp_ns
            )
            for stamp in eligible:
                message = self._pending_images.pop(stamp)
                self._frame_source.accept_image(message)

    def marker(self, aruco_id: int):
        from .configured_precision import MarkerObservation

        if type(aruco_id) is not int or aruco_id < 0:
            raise ValueError("marker aruco_id must be a nonnegative integer")
        with self._lock:
            if self._closed:
                return None
            if not self._camera_started:
                self._camera.cm.start_acquisition(quality=4)
                self._camera_started = True
            try:
                observation = self._camera.cm.latest_observation(
                    after_sequence=self._last_camera_sequence,
                    timeout_s=0.001,
                )
            except TimeoutError:
                return self._fresh_cached_marker(aruco_id)

            sequence = observation.metadata.sequence
            timestamp_ns = observation.metadata.exposure_timestamp_ns
            if type(sequence) is not int or sequence <= self._last_camera_sequence:
                raise RuntimeError("camera observation sequence did not advance")
            if type(timestamp_ns) is not int or timestamp_ns < 0:
                raise RuntimeError("camera observation lacks a source timestamp")
            self._last_camera_sequence = sequence
            vector = self._camera.vector_from_observation_3d(observation, aruco_id)
            marker = None
            if vector is not None:
                marker = MarkerObservation(
                    timestamp_ns=timestamp_ns,
                    sequence=sequence,
                    aruco_id=aruco_id,
                    forward_m=float(vector.x),
                    right_m=float(vector.y),
                    down_m=float(vector.z),
                )
            self._cached_marker_id = aruco_id
            self._cached_marker_timestamp_ns = timestamp_ns
            self._cached_marker = marker
            return self._fresh_cached_marker(aruco_id)

    def clearance(self, state: dict, timestamp_ns: int) -> tuple[float, int] | None:
        if type(timestamp_ns) is not int or timestamp_ns < 0:
            raise ValueError("clearance timestamp must be a nonnegative integer")
        if not isinstance(state, dict):
            raise TypeError("clearance state must be a dict")
        with self._lock:
            if self._closed:
                return None
            try:
                observed_at = state["observed_at_ns"]
                if not isinstance(observed_at, dict):
                    return None
                attitude_timestamps = tuple(
                    observed_at[name] for name in ("roll_rad", "pitch_rad", "yaw_rad")
                )
                if any(type(stamp) is not int or stamp < 0 for stamp in attitude_timestamps):
                    return None
                attitude_observed_at_ns = min(attitude_timestamps)
                self._attitude_sequence += 1
                attitude = self._deps.AttitudeSample(
                    roll_rad=float(state["roll_rad"]),
                    pitch_rad=float(state["pitch_rad"]),
                    yaw_rad=float(state["yaw_rad"]),
                    sampled_at=attitude_observed_at_ns / 1_000_000_000,
                    sequence=self._attitude_sequence,
                )
                sample = self._lidar.get_sample()
                projected = self._deps.project_vertical_clearance(
                    sample,
                    attitude,
                    self._clearance_calibration,
                    now=timestamp_ns / 1_000_000_000,
                )
            except (
                KeyError,
                TypeError,
                ValueError,
                self._deps.ClearanceUnavailableError,
                self._deps.StaleSensorError,
            ):
                return None
            source_ns = min(
                int(round(projected.range_sample.sampled_at * 1_000_000_000)),
                attitude_observed_at_ns,
            )
            return float(projected.projected_clearance_m), source_ns

    def publish_event(self, phase: str, state: str, timestamp_ns: int) -> None:
        with self._lock:
            if self._closed:
                raise RuntimeError("competition I/O is closed")
            if self._next_event_id == len(self._mission_sequence):
                raise RuntimeError("mission event sequence is complete")
            expected = self._mission_sequence[self._next_event_id]
            if (phase, state) != expected:
                raise ValueError(
                    "next mission event must be "
                    f"{expected[0]} {expected[1]}, got {phase} {state}"
                )
            if type(timestamp_ns) is not int or timestamp_ns < 0:
                raise ValueError("mission event timestamp must be nonnegative")
            if (
                self._last_event_timestamp_ns is not None
                and timestamp_ns <= self._last_event_timestamp_ns
            ):
                raise ValueError("mission event timestamp must increase")
            if not self.ready:
                raise RuntimeError(
                    "mission event publication requires payload service, scorekeeper, "
                    "and rosbag readiness"
                )

            message = self._deps.MissionEvent()
            message.run_id = self._config.run_id
            seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
            message.sim_timestamp.sec = seconds
            message.sim_timestamp.nanosec = nanoseconds
            message.event_id = self._next_event_id
            message.phase = phase
            message.state = state
            message.detail = "automatic attempt"
            self._mission_publisher.publish(message)
            self._next_event_id += 1
            self._last_event_timestamp_ns = timestamp_ns

    def flush(self) -> None:
        with self._lock:
            if self._next_event_id != len(self._mission_sequence):
                raise RuntimeError("mission event sequence is incomplete")
        if not self._mission_event_consumers_ready():
            raise RuntimeError(
                "mission event delivery requires scorekeeper and rosbag subscribers"
            )
        timeout_s = min(5.0, float(self._config.finalization_wall_seconds))
        acknowledged = self._mission_publisher.wait_for_all_acked(
            timeout=self._deps.Duration(seconds=timeout_s)
        )
        if acknowledged is not True:
            raise RuntimeError("mission event delivery was not acknowledged")
        if not self._mission_event_consumers_ready():
            raise RuntimeError(
                "mission event consumers disappeared during delivery confirmation"
            )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self.payloads.cancel()
            self._frame_source.stop("configured competition I/O cleanup")
            stopped = self._camera.cm.stop_acquisition(timeout_s=2.0)
            if stopped is not True:
                raise RuntimeError("camera acquisition worker did not stop within 2 seconds")

    def _accept_image(self, message: object) -> None:
        timestamp_ns = _timestamp_ns(message.header.stamp)  # type: ignore[attr-defined]
        with self._lock:
            if self._closed:
                return
            clock_ns = self._clock.timestamp_ns
            if clock_ns is not None and timestamp_ns <= clock_ns:
                self._frame_source.accept_image(message)
            else:
                self._pending_images[timestamp_ns] = message

    def _accept_range(self, message: object) -> None:
        timestamp_ns = _timestamp_ns(message.header.stamp)  # type: ignore[attr-defined]
        with self._lock:
            if not self._closed:
                self._lidar.accept(message, timestamp_ns)

    def _accept_payload_state(self, message: object) -> None:
        with self._lock:
            if not self._closed:
                self.payloads.observe(message)

    def _fresh_cached_marker(self, aruco_id: int):
        if self._cached_marker_id != aruco_id:
            return None
        timestamp_ns = self._cached_marker_timestamp_ns
        now_ns = self._clock.timestamp_ns
        if timestamp_ns is None or now_ns is None:
            return None
        age_ns = now_ns - timestamp_ns
        if age_ns < 0 or age_ns > _SENSOR_FRESHNESS_NS:
            return None
        return self._cached_marker

    def _mission_event_consumers_ready(self) -> bool:
        deps = self._deps
        try:
            endpoints = self._node.get_subscriptions_info_by_topic(
                _MISSION_EVENT_TOPIC
            )
        except BaseException:
            return False
        valid_names = {
            endpoint.node_name
            for endpoint in endpoints
            if endpoint.node_namespace == "/"
            and endpoint.topic_type == _MISSION_EVENT_TYPE
            and endpoint.qos_profile.reliability
            == deps.ReliabilityPolicy.RELIABLE
            and endpoint.qos_profile.durability
            == deps.DurabilityPolicy.TRANSIENT_LOCAL
        }
        return _SCOREKEEPER_NODE in valid_names and any(
            name.startswith(_ROSBAG_NODE_PREFIX) for name in valid_names
        )


__all__ = ["CompetitionIO"]
