"""Explicit rosbag2 process control and fail-closed MCAP validation."""

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
import math
import os
from pathlib import Path
import signal
import stat
import subprocess
import time
from types import MappingProxyType
from typing import Any, Protocol
import uuid

from ..validation import ValidationResult, ValidationStatus, validate_tree


FIXED_TOPICS = (
    "/clock",
    "/simulation/run_state",
    "/simulation/artifact_status",
    "/simulation/ground_truth",
    "/simulation/scenario_events",
    "/simulation/score_events",
    "/camera/onboard/image_raw",
    "/camera/onboard/frame_metadata",
    "/camera/observer/image_raw",
    "/camera/observer/frame_metadata",
)

FIXED_TOPIC_TYPES: Mapping[str, str] = MappingProxyType(
    {
        "/clock": "rosgraph_msgs/msg/Clock",
        "/simulation/run_state": "simulation_interfaces/msg/RunState",
        "/simulation/artifact_status": "simulation_interfaces/msg/ArtifactStatus",
        "/simulation/ground_truth": "simulation_interfaces/msg/GroundTruth",
        "/simulation/scenario_events": "simulation_interfaces/msg/ScenarioEvent",
        "/simulation/score_events": "simulation_interfaces/msg/ScoreEvent",
        "/camera/onboard/image_raw": "sensor_msgs/msg/Image",
        "/camera/onboard/frame_metadata": "simulation_interfaces/msg/FrameMetadata",
        "/camera/observer/image_raw": "sensor_msgs/msg/Image",
        "/camera/observer/frame_metadata": "simulation_interfaces/msg/FrameMetadata",
    }
)

_QOS_OVERRIDES_PATH = "/etc/drone_sim/recording-qos.yaml"
_FRAME_INTERVAL_NS = 50_000_000
_DIRECTORY_OPEN_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_DIRECTORY", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_LOG_OPEN_FLAGS = (
    os.O_WRONLY
    | os.O_APPEND
    | os.O_CREAT
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
    | getattr(os, "O_NONBLOCK", 0)
)


class _Process(Protocol):
    returncode: int | None

    def poll(self) -> int | None: ...

    def wait(self, timeout: float) -> int: ...

    def send_signal(self, signum: int) -> None: ...


class _Graph(Protocol):
    def get_node_names_and_namespaces(self) -> Sequence[tuple[str, str]]: ...

    def get_subscriptions_info_by_topic(self, topic: str) -> Sequence[Any]: ...

    def get_publishers_info_by_topic(self, topic: str) -> Sequence[Any]: ...


@dataclass(frozen=True)
class RecorderFinalization:
    exited: bool
    returncode: int | None
    signals: tuple[str, ...]
    escalated: bool
    detail: str
    shutdown_requested: bool


@dataclass(frozen=True)
class BagTopicMetadata:
    name: str
    message_type: str
    message_count: int


@dataclass(frozen=True)
class BagMetadata:
    storage_id: str
    topics: tuple[BagTopicMetadata, ...]


@dataclass(frozen=True)
class BagMessage:
    topic: str
    message: Any
    recorded_timestamp_ns: int


@dataclass(frozen=True)
class GroundTruthEvidence:
    sim_timestamp_ns: int
    vehicle_id: str
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    in_contact: bool


@dataclass(frozen=True)
class ScoreEventEvidence:
    sim_timestamp_ns: int
    event_id: int
    event_type: str
    value: float
    evidence_ref: str


@dataclass(frozen=True)
class PhysicalBagEvidence:
    bag_sha256: str
    config_sha256: str
    lifecycle_states: tuple[str, ...]
    ground_truth: tuple[GroundTruthEvidence, ...]
    score_events: tuple[ScoreEventEvidence, ...]


@dataclass(frozen=True)
class RosbagTopicDiagnostic:
    name: str
    message_type: str
    message_count: int
    first_sim_timestamp_ns: int | None
    last_sim_timestamp_ns: int | None


@dataclass(frozen=True)
class RosbagValidationResult(ValidationResult):
    topics: tuple[RosbagTopicDiagnostic, ...] = ()
    physical_evidence: PhysicalBagEvidence | None = None


class BagBackend(Protocol):
    def read_metadata(self, bag_directory: Path) -> BagMetadata: ...

    def read_messages(
        self,
        bag_directory: Path,
        storage_id: str,
        topic_types: Mapping[str, str],
    ) -> Iterable[BagMessage]: ...


def _production_process_factory(command: Sequence[str], **kwargs: Any) -> _Process:
    return subprocess.Popen(command, **kwargs)


def _production_signal_sender(process: _Process, signum: int) -> None:
    process.send_signal(signum)


def _production_qos_compatible(offered: Any, requested: Any) -> bool:
    from rclpy.qos import QoSCompatibility, qos_check_compatible

    compatibility, _reason = qos_check_compatible(offered, requested)
    return compatibility != QoSCompatibility.ERROR


def _same_entry(first: os.stat_result, second: os.stat_result) -> bool:
    return (
        first.st_dev == second.st_dev
        and first.st_ino == second.st_ino
        and first.st_mode == second.st_mode
    )


def _unsafe_log_path(error: BaseException | None = None) -> RuntimeError:
    failure = RuntimeError("unsafe recorder log path")
    if error is not None:
        failure.__cause__ = error
    return failure


def _open_or_create_directory_at(parent_fd: int, name: str) -> tuple[int, os.stat_result]:
    try:
        os.mkdir(name, mode=0o755, dir_fd=parent_fd)
    except FileExistsError:
        pass
    except OSError as error:
        raise _unsafe_log_path(error)

    descriptor: int | None = None
    try:
        before = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        if stat.S_ISLNK(before.st_mode) or not stat.S_ISDIR(before.st_mode):
            raise _unsafe_log_path()
        descriptor = os.open(name, _DIRECTORY_OPEN_FLAGS, dir_fd=parent_fd)
        opened = os.fstat(descriptor)
    except RuntimeError:
        if descriptor is not None:
            os.close(descriptor)
        raise
    except OSError as error:
        if descriptor is not None:
            os.close(descriptor)
        raise _unsafe_log_path(error)
    if not _same_entry(before, opened):
        os.close(descriptor)
        raise _unsafe_log_path()
    return descriptor, opened


def _entry_still_matches(
    parent_fd: int,
    name: str,
    descriptor: int,
    identity: os.stat_result,
) -> bool:
    try:
        current = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        opened = os.fstat(descriptor)
    except OSError:
        return False
    return _same_entry(current, identity) and _same_entry(opened, identity)


class RosbagRecorder:
    """Own one explicit shell-free rosbag2 recorder process."""

    def __init__(
        self,
        run_directory: Path | str,
        *,
        topics: Sequence[str] = FIXED_TOPICS,
        process_factory: Callable[..., _Process] = _production_process_factory,
        monotonic: Callable[[], float] = time.monotonic,
        signal_sender: Callable[[_Process, int], None] = _production_signal_sender,
        uuid_factory: Callable[[], uuid.UUID] = uuid.uuid4,
        qos_compatible: Callable[[Any, Any], bool] = _production_qos_compatible,
    ) -> None:
        self.run_directory = Path(run_directory).resolve()
        self.topics = tuple(topics)
        if self.topics != FIXED_TOPICS:
            raise ValueError("rosbag topics must equal the frozen ten-topic inventory")
        self.node_name = f"rosbag2_recorder_{uuid_factory().hex}"
        self._process_factory = process_factory
        self._monotonic = monotonic
        self._signal_sender = signal_sender
        self._qos_compatible = qos_compatible
        self._process: _Process | None = None
        self._log_stream: Any | None = None

    @property
    def output_directory(self) -> Path:
        return self.run_directory / "rosbag"

    @property
    def log_path(self) -> Path:
        return self.run_directory / "logs/docker/rosbag2.log.partial"

    @property
    def is_alive(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def command(self) -> tuple[str, ...]:
        return (
            "ros2",
            "bag",
            "record",
            "--storage",
            "mcap",
            "--output",
            str(self.output_directory),
            "--disable-keyboard-controls",
            "--include-unpublished-topics",
            "--qos-profile-overrides-path",
            _QOS_OVERRIDES_PATH,
            "--node-name",
            self.node_name,
            "--topics",
            *self.topics,
        )

    def _open_log_for_append(self) -> Any:
        held_descriptors: list[int] = []
        retained_entries: list[tuple[int, str, int, os.stat_result]] = []
        root_fd: int | None = None
        log_descriptor: int | None = None
        try:
            try:
                root_before = os.stat(self.run_directory, follow_symlinks=False)
                if not stat.S_ISDIR(root_before.st_mode):
                    raise _unsafe_log_path()
                root_fd = os.open(self.run_directory, _DIRECTORY_OPEN_FLAGS)
                root_opened = os.fstat(root_fd)
            except RuntimeError:
                raise
            except OSError as error:
                if root_fd is not None:
                    os.close(root_fd)
                raise _unsafe_log_path(error)
            assert root_fd is not None
            if not _same_entry(root_before, root_opened):
                os.close(root_fd)
                raise _unsafe_log_path()
            held_descriptors.append(root_fd)

            parent_fd = root_fd
            for name in ("logs", "docker"):
                descriptor, identity = _open_or_create_directory_at(parent_fd, name)
                held_descriptors.append(descriptor)
                retained_entries.append((parent_fd, name, descriptor, identity))
                parent_fd = descriptor

            try:
                log_descriptor = os.open(
                    "rosbag2.log.partial",
                    _LOG_OPEN_FLAGS,
                    0o644,
                    dir_fd=parent_fd,
                )
                log_identity = os.fstat(log_descriptor)
                current_log = os.stat(
                    "rosbag2.log.partial",
                    dir_fd=parent_fd,
                    follow_symlinks=False,
                )
            except OSError as error:
                raise _unsafe_log_path(error)
            if (
                not stat.S_ISREG(log_identity.st_mode)
                or log_identity.st_nlink != 1
                or not _same_entry(current_log, log_identity)
            ):
                raise _unsafe_log_path()
            if not all(
                _entry_still_matches(*entry) for entry in retained_entries
            ):
                raise _unsafe_log_path()
            try:
                current_root = os.stat(self.run_directory, follow_symlinks=False)
            except OSError as error:
                raise _unsafe_log_path(error)
            if not _same_entry(current_root, root_opened):
                raise _unsafe_log_path()

            log_stream = os.fdopen(log_descriptor, "ab", buffering=0)
            log_descriptor = None
            return log_stream
        finally:
            if log_descriptor is not None:
                os.close(log_descriptor)
            for descriptor in reversed(held_descriptors):
                os.close(descriptor)

    def start(self) -> None:
        if self._process is not None:
            raise RuntimeError("rosbag recorder has already been started")
        output = self.output_directory
        if output.exists():
            if not output.is_dir():
                raise FileExistsError(
                    f"rosbag output path exists and is not a directory: {output}"
                )
            if any(output.iterdir()):
                raise FileExistsError(f"refusing nonempty rosbag output directory: {output}")
            output.rmdir()

        log_stream = self._open_log_for_append()
        try:
            process = self._process_factory(
                self.command(),
                stdout=log_stream,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                shell=False,
            )
        except BaseException:
            log_stream.close()
            raise
        self._log_stream = log_stream
        self._process = process

    def is_ready(self, graph: _Graph) -> bool:
        process = self._process
        if process is None or process.poll() is not None:
            return False
        if not any(
            node_name == self.node_name
            for node_name, _namespace in graph.get_node_names_and_namespaces()
        ):
            return False

        for topic in self.topics:
            expected_type = FIXED_TOPIC_TYPES[topic]
            recorder_endpoints = tuple(
                endpoint
                for endpoint in graph.get_subscriptions_info_by_topic(topic)
                if endpoint.node_name == self.node_name
            )
            if not recorder_endpoints or any(
                getattr(endpoint, "topic_type", None) != expected_type
                for endpoint in recorder_endpoints
            ):
                return False
            publishers = tuple(graph.get_publishers_info_by_topic(topic))
            if any(
                getattr(publisher, "topic_type", None) != expected_type
                for publisher in publishers
            ):
                return False
            for publisher in publishers:
                if not any(
                    self._qos_compatible(
                        publisher.qos_profile,
                        subscription.qos_profile,
                    )
                    for subscription in recorder_endpoints
                ):
                    return False
        return True

    def _close_log(self) -> None:
        if self._log_stream is not None:
            self._log_stream.close()
            self._log_stream = None

    def finalize(self, deadline: float) -> RecorderFinalization:
        process = self._process
        if process is None:
            raise RuntimeError("rosbag recorder has not been started")

        signals_sent: list[str] = []
        if process.poll() is not None:
            self._close_log()
            return RecorderFinalization(
                True,
                process.returncode,
                (),
                False,
                f"recorder already exited with return code {process.returncode}",
                False,
            )

        shutdown_requested = False
        shutdown_signals = (signal.SIGINT, signal.SIGTERM, signal.SIGKILL)
        for index, signum in enumerate(shutdown_signals):
            if process.poll() is not None:
                break
            shutdown_requested = True
            try:
                self._signal_sender(process, signum)
            except ProcessLookupError:
                if process.poll() is not None:
                    break
                raise
            signals_sent.append(signal.Signals(signum).name)
            remaining = max(0.0, deadline - self._monotonic())
            phase_count = len(shutdown_signals) - index
            timeout = remaining / phase_count
            try:
                process.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                continue

        returncode = process.poll()
        exited = returncode is not None
        self._close_log()
        detail = (
            f"recorder exited with return code {returncode} after {', '.join(signals_sent)}"
            if exited
            else f"recorder did not exit before deadline after {', '.join(signals_sent)}"
        )
        return RecorderFinalization(
            exited,
            returncode,
            tuple(signals_sent),
            len(signals_sent) > 1,
            detail,
            shutdown_requested,
        )


class _Rosbag2Backend:
    def read_metadata(self, bag_directory: Path) -> BagMetadata:
        import rosbag2_py

        metadata = rosbag2_py.Info().read_metadata(str(bag_directory), "")
        return BagMetadata(
            storage_id=metadata.storage_identifier,
            topics=tuple(
                BagTopicMetadata(
                    item.topic_metadata.name,
                    item.topic_metadata.type,
                    item.message_count,
                )
                for item in metadata.topics_with_message_count
            ),
        )

    def read_messages(
        self,
        bag_directory: Path,
        storage_id: str,
        topic_types: Mapping[str, str],
    ) -> Iterable[BagMessage]:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(
            uri=str(bag_directory), storage_id=storage_id
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="", output_serialization_format=""
        )
        reader.open(storage_options, converter_options)
        message_classes = {
            topic: get_message(message_type)
            for topic, message_type in topic_types.items()
        }
        try:
            while reader.has_next():
                topic, serialized, recorded_timestamp_ns = reader.read_next()
                yield BagMessage(
                    topic,
                    deserialize_message(serialized, message_classes[topic]),
                    recorded_timestamp_ns,
                )
        finally:
            reader.close()


def _timestamp_ns(stamp: Any) -> int:
    seconds = stamp.sec
    nanoseconds = stamp.nanosec
    if not isinstance(seconds, int) or not isinstance(nanoseconds, int):
        raise ValueError("simulation timestamp fields must be integers")
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("simulation timestamp is outside the ROS Time range")
    return seconds * 1_000_000_000 + nanoseconds


def _ground_truth_evidence(message: Any, timestamp_ns: int) -> GroundTruthEvidence:
    position = message.pose.position
    orientation = message.pose.orientation
    linear = message.twist.linear
    angular = message.twist.angular
    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        linear.x,
        linear.y,
        linear.z,
        angular.x,
        angular.y,
        angular.z,
    )
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        for value in values
    ):
        raise ValueError("ground truth contains nonfinite vector fields")
    if sum(float(value) ** 2 for value in values[3:7]) == 0.0:
        raise ValueError("ground truth orientation has zero norm")
    if not isinstance(message.vehicle_id, str) or not message.vehicle_id:
        raise ValueError("ground truth vehicle_id is invalid")
    if not isinstance(message.in_contact, bool):
        raise ValueError("ground truth contact flag is invalid")
    return GroundTruthEvidence(
        timestamp_ns,
        message.vehicle_id,
        tuple(float(value) for value in values[:3]),
        tuple(float(value) for value in values[3:7]),
        tuple(float(value) for value in values[7:10]),
        tuple(float(value) for value in values[10:13]),
        message.in_contact,
    )


class RosbagValidator:
    """Validate a quiescent MCAP bag without changing it."""

    def __init__(
        self,
        run_id: str,
        *,
        backend: BagBackend | None = None,
        expected_camera_frames: int | None = None,
        physical_run: bool = False,
        config_sha256: str | None = None,
    ) -> None:
        if not run_id:
            raise ValueError("run_id must not be empty")
        if expected_camera_frames is not None and (
            not isinstance(expected_camera_frames, int)
            or isinstance(expected_camera_frames, bool)
            or expected_camera_frames <= 0
        ):
            raise ValueError("expected_camera_frames must be a positive integer or None")
        if not isinstance(physical_run, bool):
            raise TypeError("physical_run must be a boolean")
        if physical_run and (
            not isinstance(config_sha256, str)
            or len(config_sha256) != 64
            or any(character not in "0123456789abcdef" for character in config_sha256)
        ):
            raise ValueError("physical_run requires a lowercase config SHA-256")
        self.run_id = run_id
        self._backend = backend or _Rosbag2Backend()
        self.expected_camera_frames = expected_camera_frames
        self.physical_run = physical_run
        self.config_sha256 = config_sha256

    @staticmethod
    def _result(
        filesystem: ValidationResult,
        status: ValidationStatus,
        detail: str,
        topics: tuple[RosbagTopicDiagnostic, ...] = (),
        physical_evidence: PhysicalBagEvidence | None = None,
    ) -> RosbagValidationResult:
        return RosbagValidationResult(
            status,
            filesystem.size_bytes,
            filesystem.sha256,
            detail,
            topics,
            physical_evidence,
        )

    def validate(
        self, run_directory: Path | str, relative_path: Path | str = "rosbag"
    ) -> RosbagValidationResult:
        before = validate_tree(run_directory, relative_path)
        if before.status is not ValidationStatus.VALID:
            return self._result(before, before.status, before.detail)

        semantic_result = self._validate_semantics(
            run_directory,
            relative_path,
            before,
        )
        after = validate_tree(run_directory, relative_path)
        if (
            after.status is not ValidationStatus.VALID
            or after.size_bytes != before.size_bytes
            or after.sha256 != before.sha256
        ):
            return RosbagValidationResult(
                ValidationStatus.INVALID,
                None,
                None,
                "rosbag changed during semantic validation",
            )
        return semantic_result

    def _validate_semantics(
        self,
        run_directory: Path | str,
        relative_path: Path | str,
        filesystem: ValidationResult,
    ) -> RosbagValidationResult:

        bag_directory = Path(run_directory) / relative_path
        if not (bag_directory / "metadata.yaml").is_file():
            return self._result(
                filesystem,
                ValidationStatus.INVALID,
                "rosbag metadata.yaml is missing",
            )
        try:
            metadata = self._backend.read_metadata(bag_directory)
        except Exception as error:
            return self._result(
                filesystem,
                ValidationStatus.INVALID,
                f"rosbag metadata could not be read: {error}",
            )

        if metadata.storage_id != "mcap":
            return self._result(
                filesystem,
                ValidationStatus.INVALID,
                f"rosbag storage must be MCAP, got {metadata.storage_id!r}",
            )
        metadata_names = tuple(topic.name for topic in metadata.topics)
        if len(set(metadata_names)) != len(metadata_names) or set(metadata_names) != set(
            FIXED_TOPICS
        ):
            return self._result(
                filesystem,
                ValidationStatus.INVALID,
                "rosbag topic inventory differs from the frozen ten-topic inventory",
            )
        topics_by_name = {topic.name: topic for topic in metadata.topics}
        for topic in FIXED_TOPICS:
            topic_metadata = topics_by_name[topic]
            if topic_metadata.message_type != FIXED_TOPIC_TYPES[topic]:
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    f"rosbag topic {topic} has wrong type {topic_metadata.message_type!r}",
                )
            if topic_metadata.message_count <= 0:
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    f"rosbag topic {topic} has zero messages",
                )

        counts = {topic: 0 for topic in FIXED_TOPICS}
        timestamps: dict[str, list[int]] = {topic: [] for topic in FIXED_TOPICS}
        frame_ids: dict[str, list[int]] = {"onboard": [], "observer": []}
        artifact_statuses: list[tuple[int, Any]] = []
        run_states: list[Any] = []
        ground_truth_samples: list[GroundTruthEvidence] = []
        scenario_events: list[Any] = []
        score_events: list[Any] = []
        first_clock_index: int | None = None
        try:
            for record_index, record in enumerate(
                self._backend.read_messages(
                    bag_directory, metadata.storage_id, FIXED_TOPIC_TYPES
                )
            ):
                if record.topic not in counts:
                    return self._result(
                        filesystem,
                        ValidationStatus.INVALID,
                        f"serialized message uses unexpected topic {record.topic!r}",
                    )
                message = record.message
                counts[record.topic] += 1
                if record.topic == "/clock":
                    sim_timestamp_ns = _timestamp_ns(message.clock)
                    if first_clock_index is None:
                        first_clock_index = record_index
                elif record.topic.endswith("/image_raw"):
                    if not message.data:
                        return self._result(
                            filesystem,
                            ValidationStatus.INVALID,
                            f"rosbag topic {record.topic} contains a missing image payload",
                        )
                    if self.physical_run and (
                        message.height != 240
                        or message.width != 320
                        or message.encoding != "rgb8"
                        or message.step != 960
                        or len(message.data) != 320 * 240 * 3
                    ):
                        return self._result(
                            filesystem,
                            ValidationStatus.INVALID,
                            "rosbag topic "
                            f"{record.topic} has invalid physical image shape or payload",
                        )
                    sim_timestamp_ns = _timestamp_ns(message.header.stamp)
                else:
                    if message.run_id != self.run_id:
                        return self._result(
                            filesystem,
                            ValidationStatus.INVALID,
                            f"rosbag topic {record.topic} contains wrong run_id {message.run_id!r}",
                        )
                    sim_timestamp_ns = _timestamp_ns(message.sim_timestamp)
                    if timestamps[record.topic] and sim_timestamp_ns < timestamps[
                        record.topic
                    ][-1]:
                        return self._result(
                            filesystem,
                            ValidationStatus.INVALID,
                            f"rosbag topic {record.topic} has nonmonotonic "
                            "custom simulation timestamps",
                        )
                    if record.topic.endswith("/frame_metadata"):
                        stream = record.topic.split("/")[2]
                        if message.stream != stream:
                            return self._result(
                                filesystem,
                                ValidationStatus.INVALID,
                                f"rosbag topic {record.topic} contains wrong stream "
                                f"{message.stream!r}",
                            )
                        frame_ids[stream].append(message.frame_id)
                    elif record.topic == "/simulation/artifact_status":
                        artifact_statuses.append((record_index, message))
                    elif record.topic == "/simulation/run_state":
                        run_states.append(message)
                    elif record.topic == "/simulation/ground_truth" and self.physical_run:
                        ground_truth_samples.append(
                            _ground_truth_evidence(message, sim_timestamp_ns)
                        )
                    elif record.topic == "/simulation/scenario_events":
                        scenario_events.append(message)
                    elif record.topic == "/simulation/score_events":
                        score_events.append(message)
                timestamps[record.topic].append(sim_timestamp_ns)
        except Exception as error:
            return self._result(
                filesystem,
                ValidationStatus.INVALID,
                f"rosbag contains an unreadable serialized message: {error}",
            )

        for topic in FIXED_TOPICS:
            if counts[topic] != topics_by_name[topic].message_count:
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    f"rosbag topic {topic} metadata count does not match readable messages",
                )

        expected_artifact_statuses = (
            {
                "ready": False,
                "complete": False,
                "missing": ["onboard", "observer", "rosbag"],
                "manifest_path": "",
            },
            {
                "ready": True,
                "complete": False,
                "missing": [],
                "manifest_path": "",
            },
        )
        if len(artifact_statuses) != 2 or first_clock_index is None:
            return self._result(
                filesystem,
                ValidationStatus.INVALID,
                "rosbag artifact status startup sequence is incomplete",
            )
        for (record_index, message), expected in zip(
            artifact_statuses, expected_artifact_statuses, strict=True
        ):
            if (
                record_index >= first_clock_index
                or _timestamp_ns(message.sim_timestamp) != 0
                or message.ready is not expected["ready"]
                or message.complete is not expected["complete"]
                or list(message.missing) != expected["missing"]
                or message.manifest_path != expected["manifest_path"]
            ):
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    "rosbag artifact status startup sequence is invalid",
                )

        for stream in ("onboard", "observer"):
            image_topic = f"/camera/{stream}/image_raw"
            metadata_topic = f"/camera/{stream}/frame_metadata"
            if counts[image_topic] != counts[metadata_topic]:
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    f"{stream} image/metadata count mismatch",
                )
            if timestamps[image_topic] != timestamps[metadata_topic]:
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    f"{stream} image/metadata timestamps do not pair exactly",
                )
            if frame_ids[stream] != list(range(counts[metadata_topic])):
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    f"{stream} frame IDs are not contiguous from zero",
                )
            stream_timestamps = timestamps[metadata_topic]
            if any(
                current - previous != _FRAME_INTERVAL_NS
                for previous, current in zip(stream_timestamps, stream_timestamps[1:])
            ):
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    f"{stream} frame timestamps are not exactly 50 ms apart",
                )

        if self.physical_run:
            expected_states = (1, 2, 3, 4)
            if len(run_states) != len(expected_states) or any(
                message.state != state
                or message.config_sha256 != self.config_sha256
                or (state != 4 and message.reason != "")
                or (state == 4 and not message.reason)
                for message, state in zip(run_states, expected_states, strict=True)
            ):
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    "rosbag physical lifecycle is not exact or config-bound",
                )
            clock_timestamps = timestamps["/clock"]
            if any(
                current < previous
                for previous, current in zip(clock_timestamps, clock_timestamps[1:])
            ):
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    "rosbag physical clock timestamps are nonmonotonic",
                )
            frame_timestamps = set(timestamps["/camera/onboard/frame_metadata"])
            frame_timestamps.update(timestamps["/camera/observer/frame_metadata"])
            if not frame_timestamps.issubset(set(clock_timestamps)):
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    "rosbag physical clock does not cover every frame timestamp",
                )

            if len(scenario_events) != 1:
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    "rosbag physical scenario initialization is incomplete",
                )
            scenario = scenario_events[0]
            if (
                scenario.event_id != 0
                or scenario.magnet_id != "descent-v1-magnet"
                or scenario.state != "INACTIVE"
            ):
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    "rosbag physical scenario initialization is not truthful INACTIVE",
                )

            expected_event_types = (
                "descent.airborne_then_contact",
                "descent.touchdown_precision",
                "descent.safe_preimpact_speed",
                "descent.stable_contact",
                "score.finalized",
            )
            if len(score_events) != len(expected_event_types) or any(
                event.event_id != index
                or event.event_type != event_type
                or event.evidence_ref != f"scoring/events.jsonl#event-{index}"
                or isinstance(event.value, bool)
                or not isinstance(event.value, (int, float))
                or not math.isfinite(event.value)
                for index, (event, event_type) in enumerate(
                    zip(score_events, expected_event_types, strict=True)
                )
            ):
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    "rosbag physical score events are not exactly ordered and contiguous",
                )

        if self.expected_camera_frames is not None:
            exact_topics = (
                "/camera/onboard/image_raw",
                "/camera/onboard/frame_metadata",
                "/camera/observer/image_raw",
                "/camera/observer/frame_metadata",
                "/simulation/ground_truth",
            )
            if any(counts[topic] != self.expected_camera_frames for topic in exact_topics):
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    "rosbag camera or ground-truth count differs from configured frame count",
                )
            ground_truth_timestamps = timestamps["/simulation/ground_truth"]
            if (
                ground_truth_timestamps
                != timestamps["/camera/onboard/frame_metadata"]
                or ground_truth_timestamps
                != timestamps["/camera/observer/frame_metadata"]
            ):
                return self._result(
                    filesystem,
                    ValidationStatus.INVALID,
                    "rosbag ground truth is not aligned to both configured camera streams",
                )

        diagnostics = tuple(
            RosbagTopicDiagnostic(
                topic,
                topics_by_name[topic].message_type,
                counts[topic],
                timestamps[topic][0],
                timestamps[topic][-1],
            )
            for topic in FIXED_TOPICS
        )
        physical_evidence = None
        if self.physical_run:
            assert filesystem.sha256 is not None
            assert self.config_sha256 is not None
            physical_evidence = PhysicalBagEvidence(
                filesystem.sha256,
                self.config_sha256,
                ("STARTING", "READY", "RUNNING", "FINALIZING"),
                tuple(ground_truth_samples),
                tuple(
                    ScoreEventEvidence(
                        _timestamp_ns(event.sim_timestamp),
                        event.event_id,
                        event.event_type,
                        float(event.value),
                        event.evidence_ref,
                    )
                    for event in score_events
                ),
            )
        return self._result(
            filesystem,
            ValidationStatus.VALID,
            "valid MCAP rosbag with frozen topic and frame correlation contracts",
            diagnostics,
            physical_evidence,
        )


__all__ = [
    "FIXED_TOPIC_TYPES",
    "FIXED_TOPICS",
    "BagBackend",
    "BagMessage",
    "BagMetadata",
    "BagTopicMetadata",
    "GroundTruthEvidence",
    "PhysicalBagEvidence",
    "RecorderFinalization",
    "RosbagRecorder",
    "RosbagTopicDiagnostic",
    "RosbagValidationResult",
    "RosbagValidator",
    "ScoreEventEvidence",
]
