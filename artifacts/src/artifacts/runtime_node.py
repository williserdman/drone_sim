"""Aggregate ROS bag and video ownership for the Phase 2 runtime."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable, Mapping, Protocol

from ._adapters.rosbag import RosbagRecorder, RosbagValidator
from ._adapters.video import VideoStreamRecorder, VideoValidator
from .recorder_node import VideoRecorderNode
from .runtime_configuration import resolve_recording_runtime_config
from .runtime_protocol import RuntimeProtocol, canonical_run_id
from .structured_log import StructuredEvent, write_event
from .validation import ValidationStatus


_FAULTS = frozenset({"", "clock_stall_after_5", "observer_encoder_after_5"})
_STARTUP_MISSING = ["onboard", "observer", "rosbag"]


class _Protocol(Protocol):
    def write_status(self, name: str, document: Mapping[str, Any]) -> Any: ...
    def read_status(self, name: str) -> dict[str, Any] | None: ...
    def read_terminal_committed(self) -> dict[str, Any] | None: ...
    def read_manifest_status(self) -> dict[str, Any]: ...


class FaultAwareRecorder:
    """Disable only observer input after its fifth complete paired frame."""

    def __init__(
        self,
        delegate: Any,
        *,
        stream: str,
        fault: str,
        failure: Callable[[str, list[str]], None],
        pair_buffer_limit: int = 0,
        paired: Callable[[str, int, int], None] = lambda _stream, _frame, _stamp: None,
        received: Callable[[str, str, Any], None] = lambda _stream, _kind, _message: None,
    ) -> None:
        self._delegate = delegate
        self.stream = stream
        self._fault = fault
        self._failure = failure
        self._accepting = True
        self.failed_by_fault = False
        if (
            not isinstance(pair_buffer_limit, int)
            or isinstance(pair_buffer_limit, bool)
            or pair_buffer_limit < 0
        ):
            raise ValueError("pair_buffer_limit must be a nonnegative integer")
        self._pair_buffer_limit = pair_buffer_limit
        self._paired = paired
        self._received = received
        self._images_by_timestamp: dict[int, Any] = {}
        self._metadata_by_frame_id: dict[int, Any] = {}

    def __getattr__(self, name: str) -> Any:
        return getattr(self._delegate, name)

    def _accept(self, kind: str, message: Any) -> None:
        if not self._accepting:
            return
        self._received(self.stream, kind, message)
        if self._pair_buffer_limit:
            self._buffer(kind, message)
            self._drain_pairs()
        else:
            getattr(self._delegate, f"accept_{kind}")(message)
        if (
            self.stream == "observer"
            and self._fault == "observer_encoder_after_5"
            and self._delegate.frame_count >= 5
        ):
            self._accepting = False
            self._images_by_timestamp.clear()
            self._metadata_by_frame_id.clear()
            self.failed_by_fault = True
            self._failure(
                "observer encoder fault injected after frame 4",
                ["logs/docker/ffmpeg-observer.log.partial"],
            )

    @staticmethod
    def _timestamp_ns(stamp: Any) -> int:
        seconds = stamp.sec
        nanoseconds = stamp.nanosec
        if (
            not isinstance(seconds, int)
            or isinstance(seconds, bool)
            or not isinstance(nanoseconds, int)
            or isinstance(nanoseconds, bool)
            or seconds < 0
            or not 0 <= nanoseconds < 1_000_000_000
        ):
            raise ValueError("simulation timestamp is outside the ROS Time range")
        return seconds * 1_000_000_000 + nanoseconds

    def _buffer_failure(self, detail: str) -> None:
        self._accepting = False
        self._failure(detail, [f"logs/docker/ffmpeg-{self.stream}.log.partial"])
        raise ValueError(detail)

    def _buffer(self, kind: str, message: Any) -> None:
        try:
            if kind == "image":
                key = self._timestamp_ns(message.header.stamp)
                pending = self._images_by_timestamp
            else:
                key = message.frame_id
                if not isinstance(key, int) or isinstance(key, bool) or key < 0:
                    raise ValueError("metadata frame_id must be a nonnegative integer")
                pending = self._metadata_by_frame_id
        except (AttributeError, ValueError) as error:
            self._buffer_failure(f"invalid buffered {kind}: {error}")
        if key in pending:
            self._buffer_failure(f"duplicate buffered {kind}")
        if len(pending) >= self._pair_buffer_limit:
            self._buffer_failure(f"{kind} pair buffer exceeded its fixed limit")
        pending[key] = message

    def _drain_pairs(self) -> None:
        while True:
            frame_id = self._delegate.frame_count
            metadata = self._metadata_by_frame_id.get(frame_id)
            if metadata is None:
                return
            timestamp_ns = self._timestamp_ns(metadata.sim_timestamp)
            image = self._images_by_timestamp.get(timestamp_ns)
            if image is None:
                return
            del self._metadata_by_frame_id[frame_id]
            del self._images_by_timestamp[timestamp_ns]
            self._delegate.accept_image(image)
            self._delegate.accept_metadata(metadata)
            self._paired(self.stream, frame_id, timestamp_ns)

    def accept_image(self, message: Any) -> None:
        self._accept("image", message)

    def accept_metadata(self, message: Any) -> None:
        self._accept("metadata", message)

    def freeze(self) -> None:
        self._accepting = False
        if self._images_by_timestamp or self._metadata_by_frame_id:
            self._failure(
                "finalization found unmatched buffered camera input",
                [f"logs/docker/ffmpeg-{self.stream}.log.partial"],
            )
        self._images_by_timestamp.clear()
        self._metadata_by_frame_id.clear()


class AggregateArtifactsRuntime:
    """Injected aggregate recorder coordinator with one finalization deadline."""

    def __init__(
        self,
        run_directory: Path | str,
        run_id: str,
        *,
        protocol: _Protocol,
        bag_recorder: Any,
        video_node: Any,
        video_validators: Mapping[str, Any],
        bag_validator: Any,
        publish: Callable[[dict[str, Any]], None],
        fault: str = "",
        monotonic: Callable[[], float] = time.monotonic,
        backpressure_ready: Callable[[Any], bool] = lambda _graph: True,
        lifecycle_ready: Callable[[], bool] = lambda: True,
        initial_status_delivered: Callable[[], bool] = lambda: True,
        expected_camera_frames: int = 40,
    ) -> None:
        self.run_directory = Path(run_directory)
        self.run_id = canonical_run_id(run_id)
        if fault not in _FAULTS:
            raise ValueError("SIM_PHASE2_FAULT is not one of the accepted Phase 2 faults")
        if set(video_validators) != {"onboard", "observer"}:
            raise ValueError("video validators must contain onboard and observer")
        if (
            not isinstance(expected_camera_frames, int)
            or isinstance(expected_camera_frames, bool)
            or expected_camera_frames <= 0
        ):
            raise ValueError("expected_camera_frames must be a positive integer")
        self.protocol = protocol
        self.bag_recorder = bag_recorder
        self.video_node = video_node
        self.video_validators = dict(video_validators)
        self.bag_validator = bag_validator
        self.publish = publish
        self.fault = fault
        self.monotonic = monotonic
        self.backpressure_ready = backpressure_ready
        self.lifecycle_ready = lifecycle_ready
        self.initial_status_delivered = initial_status_delivered
        self.expected_camera_frames = expected_camera_frames
        self.started = False
        self.ready = False
        self._initial_status_published = False
        self._startup_deadline: float | None = None
        self.final_report: dict[str, Any] | None = None
        self._failure_written = False
        self._terminal = False
        self.finalization_started = False
        self.finalization_blocked = False
        self._premature_bag_exit = False

    def report_failure(self, reason: str, diagnostic_paths: list[str]) -> None:
        if self._failure_written:
            return
        self.protocol.write_status(
            "runtime-failure",
            {
                "run_id": self.run_id,
                "module": "artifacts",
                "reason": reason,
                "diagnostic_paths": diagnostic_paths,
            },
        )
        self._failure_written = True

    def report_video_diagnostic(
        self, diagnostic: Any, *, structured: Callable[[], None]
    ) -> None:
        self.report_failure(
            f"{diagnostic.stream} {diagnostic.event}: {diagnostic.detail}",
            [f"logs/docker/ffmpeg-{diagnostic.stream}.log.partial"],
        )
        structured()

    def check_health(self) -> bool:
        if not self.started or self.finalization_blocked:
            return False
        if not self.bag_recorder.is_alive:
            self._report_premature_bag_exit(
                "rosbag recorder exited after recorder readiness"
            )
            return False
        for stream in ("onboard", "observer"):
            if not self.video_node.recorders[stream].is_ready:
                self.report_failure(
                    f"{stream} FFmpeg exited after recorder readiness",
                    [f"logs/docker/ffmpeg-{stream}.log.partial"],
                )
                return False
        return True

    def _report_premature_bag_exit(self, reason: str) -> None:
        self._premature_bag_exit = True
        self.report_failure(reason, ["logs/docker/rosbag2.log.partial"])

    def start(self, *, deadline: float) -> bool:
        if self.started:
            raise RuntimeError("artifact runtime has already started")
        self.started = True
        self._startup_deadline = deadline
        try:
            self.bag_recorder.start()
            self.video_node.start(deadline=deadline)
        except Exception as error:
            paths = ["logs/docker/rosbag2.log.partial"]
            if "video" in type(error).__name__.lower() or self.bag_recorder is not None:
                paths = list(dict.fromkeys(paths))
            self.report_failure(
                f"recorder startup failed: {type(error).__name__}: {error}", paths
            )
            return False
        return True

    def check_ready(self, graph: Any) -> bool:
        if not self.started or self._failure_written or self.ready:
            return self.ready
        if not self.check_health():
            return False
        if not self.bag_recorder.is_ready(graph):
            return False
        if not self._initial_status_published:
            self.publish(
                {
                    "run_id": self.run_id,
                    "ready": False,
                    "complete": False,
                    "missing": list(_STARTUP_MISSING),
                    "manifest_path": "",
                }
            )
            self._initial_status_published = True
            return False
        if not self.initial_status_delivered():
            if (
                self._startup_deadline is not None
                and self.monotonic() >= self._startup_deadline
            ):
                self.report_failure(
                    "initial artifact status delivery was not acknowledged "
                    "before startup deadline",
                    ["logs/docker/rosbag2.log.partial"],
                )
            return False
        if (
            not self.video_node.is_ready
            or not self.backpressure_ready(graph)
            or not self.lifecycle_ready()
        ):
            return False
        document = {
            "run_id": self.run_id,
            "ready": True,
            "complete": False,
            "missing": [],
            "manifest_path": "",
        }
        self.publish(document)
        self.protocol.write_status(
            "artifacts-ready", {"run_id": self.run_id, "ready": True}
        )
        self.ready = True
        return True

    @staticmethod
    def _video_semantic(result: Any) -> dict[str, Any]:
        return {
            "codec_name": getattr(result, "codec_name", None),
            "pix_fmt": getattr(result, "pix_fmt", None),
            "avg_frame_rate": getattr(result, "avg_frame_rate", None),
            "width": getattr(result, "width", None),
            "height": getattr(result, "height", None),
            "frame_count": getattr(result, "frame_count", None),
            "diagnostics": [
                {
                    "event": getattr(item, "event", "validator"),
                    "detail": getattr(item, "detail", str(item)),
                }
                for item in getattr(result, "diagnostics", ())
            ],
        }

    @staticmethod
    def _bag_semantic(result: Any) -> dict[str, Any]:
        return {
            "storage_id": "mcap" if result.status is ValidationStatus.VALID else None,
            "topics": [
                {
                    "name": item.name,
                    "message_type": item.message_type,
                    "message_count": item.message_count,
                    "first_sim_timestamp_ns": item.first_sim_timestamp_ns,
                    "last_sim_timestamp_ns": item.last_sim_timestamp_ns,
                }
                for item in getattr(result, "topics", ())
            ],
        }

    def _invalid_result(self, detail: str) -> Any:
        return type(
            "InvalidValidation",
            (),
            {
                "status": ValidationStatus.INVALID,
                "size_bytes": None,
                "sha256": None,
                "detail": detail,
                "diagnostics": (),
                "topics": (),
            },
        )()

    def _bounded_bag_validation(self, deadline: float) -> Any:
        completed = threading.Event()
        results: list[Any] = []

        def validate() -> None:
            try:
                results.append(self.bag_validator.validate(self.run_directory, "rosbag"))
            except Exception as error:
                results.append(
                    self._invalid_result(
                        f"bag validation failed: {type(error).__name__}: {error}"
                    )
                )
            finally:
                completed.set()

        threading.Thread(target=validate, daemon=True).start()
        completed.wait(max(0.0, deadline - self.monotonic()))
        if not completed.is_set() or self.monotonic() >= deadline:
            return self._invalid_result("bag validation exceeded finalization deadline")
        return results[0]

    def _video_result(self, stream: str, outcome: str, deadline: float) -> Any:
        try:
            result = self.video_validators[stream].validate(
                self.run_directory,
                f"video/{stream}.mp4",
                expected_frame_count=self.expected_camera_frames,
                outcome=outcome,
                deadline=deadline,
            )
        except Exception as error:
            result = self._invalid_result(
                f"{stream} video validation failed: {type(error).__name__}: {error}"
            )
        recorder = self.video_node.recorders[stream]
        if stream == "observer" and getattr(recorder, "failed_by_fault", False):
            proxy = self._invalid_result("observer encoder fault injected after frame 4")
            proxy.size_bytes = result.size_bytes
            proxy.sha256 = result.sha256
            proxy.codec_name = getattr(result, "codec_name", None)
            proxy.pix_fmt = getattr(result, "pix_fmt", None)
            proxy.avg_frame_rate = getattr(result, "avg_frame_rate", None)
            proxy.width = getattr(result, "width", None)
            proxy.height = getattr(result, "height", None)
            proxy.frame_count = getattr(result, "frame_count", None)
            proxy.diagnostics = getattr(result, "diagnostics", ())
            return proxy
        return result

    @staticmethod
    def _record(path: str, result: Any, semantic: dict[str, Any]) -> dict[str, Any]:
        return {
            "relative_path": path,
            "status": result.status.value,
            "detail": result.detail,
            "size_bytes": result.size_bytes,
            "sha256": result.sha256,
            "semantic": semantic,
        }

    def finalize(self, outcome: str, *, deadline: float) -> dict[str, Any] | None:
        if self.final_report is not None:
            return self.final_report
        if self.finalization_blocked:
            return None
        if self.protocol.read_status("runtime-frozen") is None:
            return None
        self.finalization_started = True
        if not self.bag_recorder.is_alive:
            self._report_premature_bag_exit(
                "rosbag recorder exited before shutdown was requested"
            )
        for recorder in self.video_node.recorders.values():
            freeze = getattr(recorder, "freeze", None)
            if freeze is not None:
                freeze()
        for stream in ("onboard", "observer"):
            try:
                self.video_node.recorders[stream].finalize(deadline, outcome=outcome)
            except Exception as error:
                self.report_failure(
                    f"{stream} recorder finalization failed: {type(error).__name__}: {error}",
                    [f"logs/docker/ffmpeg-{stream}.log.partial"],
                )
        if self._premature_bag_exit:
            self.finalization_blocked = True
            return None
        try:
            bag_finalization = self.bag_recorder.finalize(deadline)
        except Exception as error:
            self.report_failure(
                f"bag recorder finalization failed: {type(error).__name__}: {error}",
                ["logs/docker/rosbag2.log.partial"],
            )
            self.finalization_blocked = True
            return None
        if not bag_finalization.exited:
            self.report_failure(
                f"bag recorder finalization failed: {bag_finalization.detail}",
                ["logs/docker/rosbag2.log.partial"],
            )
            self.finalization_blocked = True
            return None
        if not bag_finalization.shutdown_requested:
            self._report_premature_bag_exit(
                "bag recorder finalization failed: recorder exited before shutdown was requested"
            )
            self.finalization_blocked = True
            return None

        video_results = {
            stream: self._video_result(stream, outcome, deadline)
            for stream in ("onboard", "observer")
        }
        bag_result = self._bounded_bag_validation(deadline)
        records = [
            self._record(
                f"video/{stream}.mp4",
                video_results[stream],
                self._video_semantic(video_results[stream]),
            )
            for stream in ("onboard", "observer")
        ]
        records.append(self._record("rosbag", bag_result, self._bag_semantic(bag_result)))
        report = {
            "run_id": self.run_id,
            "complete": all(item["status"] == "valid" for item in records),
            "records": records,
        }
        self.protocol.write_status("artifacts-final", report)
        self.final_report = report
        return report

    def poll_terminal(self) -> bool:
        if self._terminal:
            return True
        if self.final_report is None:
            return False
        committed = self.protocol.read_terminal_committed()
        if committed is None:
            return False
        manifest_status = self.protocol.read_manifest_status()
        if (
            committed.get("run_id") != self.run_id
            or committed.get("manifest_path") != "manifest.json"
            or manifest_status.get("run_id") != self.run_id
            or manifest_status.get("manifest_path") != "manifest.json"
        ):
            raise RuntimeError("terminal manifest authority is inconsistent")
        self.publish(
            {
                "run_id": self.run_id,
                "ready": self.ready,
                "complete": manifest_status["complete"],
                "missing": sorted(manifest_status["missing"]),
                "manifest_path": "manifest.json",
            }
        )
        self.protocol.write_status(
            "terminal-notified", {"run_id": self.run_id, "notified": True}
        )
        self._terminal = True
        return True


def _assign_stamp(stamp: Any, timestamp_ns: int) -> None:
    stamp.sec = timestamp_ns // 1_000_000_000
    stamp.nanosec = timestamp_ns % 1_000_000_000


def publish_artifact_status(
    publisher: Any,
    message_type: Any,
    document: Mapping[str, Any],
    *,
    timestamp_ns: int,
) -> None:
    """Publish one mapped ArtifactStatus without touching frozen file/stdout evidence."""
    message = message_type()
    message.run_id = document["run_id"]
    _assign_stamp(message.sim_timestamp, timestamp_ns)
    message.ready = document["ready"]
    message.complete = document["complete"]
    message.missing = document["missing"]
    message.manifest_path = document["manifest_path"]
    publisher.publish(message)


def main() -> None:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from simulation_interfaces.msg import ArtifactStatus, FrameMetadata, RunState

    run_id = os.environ["SIM_RUN_ID"]
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"])
    config = json.loads(Path(os.environ["SIM_CONFIG_PATH"]).read_text(encoding="utf-8"))
    recording_contract = resolve_recording_runtime_config(config)
    fault = os.environ.get("SIM_PHASE2_FAULT", "")
    finalization_seconds = float(config["finalization_wall_seconds"])
    protocol = RuntimeProtocol(run_directory, run_id)
    rclpy.init()
    node = Node("artifacts_runtime")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(ArtifactStatus, "/simulation/artifact_status", qos)
    pair_ack_publisher = None
    if recording_contract.synthetic_camera_ack:
        pair_ack_publisher = node.create_publisher(
            FrameMetadata,
            "/simulation/camera_pair_ack",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def camera_recorder_qos() -> QoSProfile:
        return QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

    last_sim_timestamp_ns = 0
    requested_outcome: str | None = None
    finalization_deadline: float | None = None
    saw_current_run_starting = False
    quiescent = False

    def emit_log(event: str, **fields: Any) -> None:
        if quiescent:
            return
        write_event(
            sys.stdout,
            StructuredEvent(
                run_id=run_id,
                module="artifacts",
                severity="INFO",
                event=event,
                sim_timestamp=last_sim_timestamp_ns / 1e9,
                wall_timestamp=datetime.now(timezone.utc),
                fields=fields,
            ),
        )

    def publish(document: dict[str, Any]) -> None:
        publish_artifact_status(
            publisher,
            ArtifactStatus,
            document,
            timestamp_ns=last_sim_timestamp_ns,
        )

    runtime_ref: list[AggregateArtifactsRuntime] = []
    paired_frames = {"onboard": -1, "observer": -1}
    last_pair_ack = -1

    def paired(stream: str, frame_id: int, timestamp_ns: int) -> None:
        nonlocal last_pair_ack
        emit_log(
            "camera_stream_pair_drained",
            stream=stream,
            frame_id=frame_id,
            sim_timestamp_ns=timestamp_ns,
        )
        paired_frames[stream] = frame_id
        if pair_ack_publisher is None:
            return
        confirmed = min(paired_frames.values())
        if confirmed <= last_pair_ack:
            return
        acknowledgement = FrameMetadata()
        acknowledgement.run_id = run_id
        _assign_stamp(acknowledgement.sim_timestamp, timestamp_ns)
        acknowledgement.frame_id = confirmed
        acknowledgement.stream = "aggregate"
        pair_ack_publisher.publish(acknowledgement)
        emit_log(
            "camera_pair_ack_published",
            frame_id=confirmed,
            sim_timestamp_ns=timestamp_ns,
        )
        last_pair_ack = confirmed

    def report_failure(reason: str, paths: list[str]) -> None:
        runtime_ref[0].report_failure(reason, paths)
        emit_log("recorder_failure", reason=reason, diagnostic_paths=paths)

    def recorder_factory(
        run: Path, configured_run_id: str, stream: str, errors: Callable[[Any], None]
    ) -> FaultAwareRecorder:
        delegate = VideoStreamRecorder(
            run,
            run_id=configured_run_id,
            stream=stream,
            expected_frame_count=recording_contract.expected_camera_frames,
            diagnostic_sink=errors,
        )
        return FaultAwareRecorder(
            delegate,
            stream=stream,
            fault=fault,
            failure=report_failure,
            pair_buffer_limit=recording_contract.expected_camera_frames,
            paired=paired,
            received=lambda configured_stream, kind, message: emit_log(
                "camera_message_received",
                stream=configured_stream,
                kind=kind,
                frame_id=(
                    message.frame_id
                    if kind == "metadata"
                    else int(message.header.stamp.sec) * 20
                    + int(message.header.stamp.nanosec) // 50_000_000
                    - 1
                ),
            ),
        )

    video_node = VideoRecorderNode(
        run_directory,
        run_id,
        expected_frame_count=recording_contract.expected_camera_frames,
        node_backend=node,
        recorder_factory=recorder_factory,
        qos_factory=camera_recorder_qos,
        error_sink=lambda event: runtime_ref[0].report_video_diagnostic(
            event,
            structured=lambda: write_event(sys.stdout, event) if not quiescent else None,
        ),
    )
    runtime = AggregateArtifactsRuntime(
        run_directory,
        run_id,
        protocol=protocol,
        bag_recorder=RosbagRecorder(run_directory),
        video_node=video_node,
        video_validators={"onboard": VideoValidator(), "observer": VideoValidator()},
        bag_validator=RosbagValidator(
            run_id,
            expected_camera_frames=recording_contract.expected_camera_frames,
            physical_run=recording_contract.physical_run,
            config_sha256=(config["config_sha256"] if recording_contract.physical_run else None),
        ),
        publish=publish,
        fault=fault,
        backpressure_ready=(
            (lambda graph: graph.count_subscribers("/simulation/camera_pair_ack") >= 1)
            if recording_contract.synthetic_camera_ack
            else (lambda _graph: True)
        ),
        lifecycle_ready=lambda: saw_current_run_starting,
        initial_status_delivered=lambda: publisher.wait_for_all_acked(
            timeout=Duration(nanoseconds=0)
        ),
        expected_camera_frames=recording_contract.expected_camera_frames,
    )
    runtime_ref.append(runtime)

    def state_callback(message: Any) -> None:
        nonlocal requested_outcome, finalization_deadline, last_sim_timestamp_ns
        nonlocal saw_current_run_starting
        if message.run_id != run_id:
            return
        if message.state == RunState.STARTING:
            saw_current_run_starting = True
        last_sim_timestamp_ns = int(message.sim_timestamp.sec) * 1_000_000_000 + int(
            message.sim_timestamp.nanosec
        )
        if message.state == RunState.FINALIZING and requested_outcome is None:
            request = protocol.read_finalize_request()
            if request is not None:
                requested_outcome = request["requested_terminal"]
                finalization_deadline = time.monotonic() + finalization_seconds

    node.create_subscription(RunState, "/simulation/run_state", state_callback, qos)
    emit_log("starting")
    startup_deadline = time.monotonic() + float(config["startup_wall_seconds"])
    runtime.start(deadline=startup_deadline)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not runtime.ready:
                runtime.check_ready(node)
                if runtime.ready:
                    emit_log("ready")
            elif not runtime.finalization_started:
                runtime.check_health()
            if requested_outcome is not None and finalization_deadline is not None:
                report = runtime.finalize(requested_outcome, deadline=finalization_deadline)
                if report is not None or runtime.finalization_started:
                    quiescent = True
            if runtime.poll_terminal():
                break
    finally:
        protocol.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


__all__ = [
    "AggregateArtifactsRuntime",
    "FaultAwareRecorder",
    "main",
    "publish_artifact_status",
]
