"""ROS-facing ownership and readiness surface for the two video recorders."""

from collections.abc import Callable, Mapping
from datetime import datetime, timezone
from pathlib import Path
from types import MappingProxyType
from typing import Any

from ._adapters.video import STREAMS, VideoDiagnostic, VideoStreamRecorder
from .structured_log import StructuredEvent


try:
    from rclpy.node import Node as _RosNode
except ImportError:
    class _RosNode:  # type: ignore[no-redef]
        """Import-only host fallback; real construction requires ROS 2."""

        def __init__(self, *_args: Any, **_kwargs: Any) -> None:
            raise RuntimeError("VideoRecorderNode requires rclpy or an injected node_backend")


VIDEO_TOPICS = (
    "/camera/onboard/image_raw",
    "/camera/onboard/frame_metadata",
    "/camera/observer/image_raw",
    "/camera/observer/frame_metadata",
)


def _default_message_types() -> tuple[type[Any], type[Any]]:
    from sensor_msgs.msg import Image
    from simulation_interfaces.msg import FrameMetadata

    return Image, FrameMetadata


def _default_qos() -> Any:
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class VideoRecorderNode(_RosNode):
    """Own exactly two pipelines and four fixed best-effort subscriptions."""

    def __init__(
        self,
        run_directory: Path | str,
        run_id: str,
        *,
        expected_frame_count: int = 40,
        node_backend: Any | None = None,
        recorder_factory: Callable[[Path, str, str, Callable[[VideoDiagnostic], None]], Any] | None = None,
        message_types: tuple[type[Any], type[Any]] | None = None,
        qos_factory: Callable[[], Any] = _default_qos,
        error_sink: Callable[[StructuredEvent], None] | None = None,
        wall_clock: Callable[[], datetime] | None = None,
    ) -> None:
        if not run_id:
            raise ValueError("run_id must not be empty")
        if (
            not isinstance(expected_frame_count, int)
            or isinstance(expected_frame_count, bool)
            or expected_frame_count <= 0
        ):
            raise ValueError("expected_frame_count must be a positive integer")
        self.run_directory = Path(run_directory).resolve()
        self.run_id = run_id
        self._expected_frame_count = expected_frame_count
        if node_backend is None:
            super().__init__("video_recorder")
            self._node = self
        else:
            self._node = node_backend
        self._error_sink = error_sink or (lambda _event: None)
        self._wall_clock = wall_clock or (lambda: datetime.now(timezone.utc))
        image_type, metadata_type = message_types or _default_message_types()
        if recorder_factory is None:
            recorder_factory = lambda run, configured_run_id, stream, errors: VideoStreamRecorder(
                run,
                run_id=configured_run_id,
                stream=stream,
                expected_frame_count=self.expected_frame_count,
                diagnostic_sink=errors,
            )
        self.recorders: Mapping[str, Any] = MappingProxyType(
            {
                stream: recorder_factory(
                    self.run_directory,
                    self.run_id,
                    stream,
                    self.report_error,
                )
                for stream in STREAMS
            }
        )
        subscriptions = []
        for stream in STREAMS:
            image_topic = f"/camera/{stream}/image_raw"
            metadata_topic = f"/camera/{stream}/frame_metadata"
            subscriptions.append(
                self._node.create_subscription(
                    image_type,
                    image_topic,
                    self._callback(stream, "image"),
                    qos_factory(),
                )
            )
            subscriptions.append(
                self._node.create_subscription(
                    metadata_type,
                    metadata_topic,
                    self._callback(stream, "metadata"),
                    qos_factory(),
                )
            )
        self._owned_subscriptions = tuple(subscriptions)

    @property
    def subscription_handles(self) -> tuple[Any, ...]:
        return self._owned_subscriptions

    @property
    def expected_frame_count(self) -> int:
        return self._expected_frame_count

    def _callback(self, stream: str, kind: str) -> Callable[[Any], None]:
        method = getattr(self.recorders[stream], f"accept_{kind}")

        def contained_callback(message: Any) -> None:
            try:
                method(message)
            except Exception as error:
                self._safe_report(
                    VideoDiagnostic(
                        stream,
                        "recorder_callback_failed",
                        f"{kind} callback failed: {type(error).__name__}: {error}",
                    )
                )

        return contained_callback

    def start(self, *, deadline: float) -> None:
        started: list[str] = []
        for stream in STREAMS:
            try:
                self.recorders[stream].start(deadline=deadline)
                started.append(stream)
            except Exception as error:
                self._safe_report(
                    VideoDiagnostic(
                        stream,
                        "video_start_failed",
                        f"video recorder start failed: {type(error).__name__}: {error}",
                    )
                )
                rollback_streams = (stream, *reversed(started))
                for started_stream in rollback_streams:
                    try:
                        self.recorders[started_stream].finalize(
                            deadline, outcome="FAILED"
                        )
                    except Exception as rollback_error:
                        self._safe_report(
                            VideoDiagnostic(
                                started_stream,
                                "video_start_rollback_failed",
                                "video recorder rollback failed: "
                                f"{type(rollback_error).__name__}: {rollback_error}",
                            )
                        )
                raise

    def _safe_report(self, diagnostic: VideoDiagnostic) -> None:
        try:
            self.report_error(diagnostic)
        except Exception:
            pass

    @property
    def discovered_subscription_counts(self) -> dict[str, int]:
        return {topic: self._node.count_subscribers(topic) for topic in VIDEO_TOPICS}

    @property
    def is_ready(self) -> bool:
        return all(recorder.is_ready for recorder in self.recorders.values()) and all(
            count >= 1 for count in self.discovered_subscription_counts.values()
        )

    def report_error(self, diagnostic: VideoDiagnostic) -> None:
        sim_timestamp = (
            diagnostic.sim_timestamp_ns / 1_000_000_000
            if diagnostic.sim_timestamp_ns is not None
            else None
        )
        self._error_sink(
            StructuredEvent(
                run_id=self.run_id,
                module="artifacts",
                severity="ERROR",
                event=diagnostic.event,
                sim_timestamp=sim_timestamp,
                wall_timestamp=self._wall_clock(),
                fields={"stream": diagnostic.stream, "detail": diagnostic.detail},
            )
        )

    def destroy_node(self) -> None:
        if self._node is self:
            super().destroy_node()
        else:
            self._node.destroy_node()


__all__ = ["VIDEO_TOPICS", "VideoRecorderNode"]
