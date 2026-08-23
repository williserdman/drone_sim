"""Explicit rosbag2 process control and fail-closed MCAP validation."""

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
import signal
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
class RosbagTopicDiagnostic:
    name: str
    message_type: str
    message_count: int
    first_sim_timestamp_ns: int | None
    last_sim_timestamp_ns: int | None


@dataclass(frozen=True)
class RosbagValidationResult(ValidationResult):
    topics: tuple[RosbagTopicDiagnostic, ...] = ()


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

        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        log_stream = self.log_path.open("ab")
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
            recorder_subscriptions = tuple(
                endpoint
                for endpoint in graph.get_subscriptions_info_by_topic(topic)
                if endpoint.node_name == self.node_name
            )
            if not recorder_subscriptions:
                return False
            for publisher in graph.get_publishers_info_by_topic(topic):
                if not any(
                    self._qos_compatible(
                        publisher.qos_profile,
                        subscription.qos_profile,
                    )
                    for subscription in recorder_subscriptions
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
            )

        shutdown_signals = (signal.SIGINT, signal.SIGTERM, signal.SIGKILL)
        for index, signum in enumerate(shutdown_signals):
            if process.poll() is not None:
                break
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


class RosbagValidator:
    """Validate a quiescent MCAP bag without changing it."""

    def __init__(self, run_id: str, *, backend: BagBackend | None = None) -> None:
        if not run_id:
            raise ValueError("run_id must not be empty")
        self.run_id = run_id
        self._backend = backend or _Rosbag2Backend()

    @staticmethod
    def _result(
        filesystem: ValidationResult,
        status: ValidationStatus,
        detail: str,
        topics: tuple[RosbagTopicDiagnostic, ...] = (),
    ) -> RosbagValidationResult:
        return RosbagValidationResult(
            status,
            filesystem.size_bytes,
            filesystem.sha256,
            detail,
            topics,
        )

    def validate(
        self, run_directory: Path | str, relative_path: Path | str = "rosbag"
    ) -> RosbagValidationResult:
        filesystem = validate_tree(run_directory, relative_path)
        if filesystem.status is not ValidationStatus.VALID:
            return self._result(filesystem, filesystem.status, filesystem.detail)

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
        try:
            for record in self._backend.read_messages(
                bag_directory, metadata.storage_id, FIXED_TOPIC_TYPES
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
                elif record.topic.endswith("/image_raw"):
                    if not message.data:
                        return self._result(
                            filesystem,
                            ValidationStatus.INVALID,
                            f"rosbag topic {record.topic} contains a missing image payload",
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
        return self._result(
            filesystem,
            ValidationStatus.VALID,
            "valid MCAP rosbag with frozen topic and frame correlation contracts",
            diagnostics,
        )


__all__ = [
    "FIXED_TOPIC_TYPES",
    "FIXED_TOPICS",
    "BagBackend",
    "BagMessage",
    "BagMetadata",
    "BagTopicMetadata",
    "RecorderFinalization",
    "RosbagRecorder",
    "RosbagTopicDiagnostic",
    "RosbagValidationResult",
    "RosbagValidator",
]
