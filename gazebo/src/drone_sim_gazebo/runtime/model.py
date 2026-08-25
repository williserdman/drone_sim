"""Pure, bounded transition model for one paused-first Gazebo run."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import PurePosixPath
import re
from typing import TypeAlias
from uuid import UUID

from ..ros_adapter import AdapterSummary
from ..server import NativeArtifactSummary


_FRAME_INTERVAL_NS = 50_000_000
_LIFECYCLE_STATES = frozenset(
    {
        "CREATED",
        "STARTING",
        "READY",
        "RUNNING",
        "FINALIZING",
        "COMPLETED",
        "FAILED",
        "ABORTED",
    }
)
_TERMINAL_STATES = frozenset({"COMPLETED", "FAILED", "ABORTED"})
_MAX_REASON_BYTES = 1024
_MAX_ENDPOINT_BYTES = 256
_MAX_CHILD_NAME_BYTES = 64
_MAX_DIAGNOSTIC_PATH_BYTES = 512
_MAX_DIAGNOSTIC_PATHS = 8
_SERVER_DIAGNOSTIC = ("gazebo/server.log.partial",)
_SERVER_EXIT_DIAGNOSTICS = ("gazebo/server.log.partial", "gazebo/state")
_CHILD_NAME = re.compile(r"[a-z][a-z0-9_]*\Z")


class RuntimeModelError(RuntimeError):
    """A caller attempted to use the pure model outside its frozen contract."""


def _canonical_run_id(value: object) -> str:
    if not isinstance(value, str):
        raise ValueError("run_id must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (ValueError, TypeError, AttributeError) as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise ValueError("run_id must be a canonical UUID")
    return value


def _positive_integer(value: object, *, field: str) -> int:
    if type(value) is not int or value <= 0:
        raise ValueError(f"{field} must be a positive integer")
    return value


def _integer(value: object, *, field: str) -> int:
    if type(value) is not int:
        raise TypeError(f"{field} must be an integer")
    return value


def _deadline(value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError("deadline_monotonic must be a positive finite number")
    return float(value)


def _bounded_text(value: object, *, field: str, maximum: int) -> str:
    if (
        not isinstance(value, str)
        or not value
        or len(value.encode("utf-8")) > maximum
    ):
        raise ValueError(f"{field} must be nonempty and at most {maximum} UTF-8 bytes")
    return value


def _terminal(value: object) -> str:
    if value not in _TERMINAL_STATES or not isinstance(value, str):
        raise ValueError("requested_terminal must be COMPLETED, FAILED, or ABORTED")
    return value


def _diagnostic_paths(value: object) -> tuple[str, ...]:
    if type(value) is not tuple or len(value) > _MAX_DIAGNOSTIC_PATHS:
        raise TypeError("diagnostic_paths must be a bounded immutable tuple")
    if len(set(value)) != len(value):
        raise ValueError("diagnostic_paths must not contain duplicates")
    for item in value:
        if (
            not isinstance(item, str)
            or not item
            or "\\" in item
            or len(item.encode("utf-8")) > _MAX_DIAGNOSTIC_PATH_BYTES
        ):
            raise ValueError("diagnostic paths must be bounded relative POSIX paths")
        path = PurePosixPath(item)
        if path.is_absolute() or ".." in path.parts or path.parts in ((), (".",)):
            raise ValueError("diagnostic paths must be bounded relative POSIX paths")
    return value


def _child_name(value: object) -> str:
    if (
        not isinstance(value, str)
        or len(value.encode("utf-8")) > _MAX_CHILD_NAME_BYTES
        or _CHILD_NAME.fullmatch(value) is None
    ):
        raise ValueError(
            "child_name must be canonical lowercase snake case and at most "
            f"{_MAX_CHILD_NAME_BYTES} UTF-8 bytes"
        )
    return value


@dataclass(frozen=True)
class PublishGazeboReady:
    pass


@dataclass(frozen=True)
class RequestSteps:
    count: int

    def __post_init__(self) -> None:
        _positive_integer(self.count, field="count")


@dataclass(frozen=True)
class SetPaused:
    paused: bool

    def __post_init__(self) -> None:
        if type(self.paused) is not bool:
            raise TypeError("paused must be a boolean")


@dataclass(frozen=True)
class ActivateOutput:
    pass


@dataclass(frozen=True)
class WriteSourceFinished:
    sim_timestamp_ns: int

    def __post_init__(self) -> None:
        _positive_integer(self.sim_timestamp_ns, field="sim_timestamp_ns")


@dataclass(frozen=True)
class WriteRuntimeFailure:
    reason: str
    diagnostic_paths: tuple[str, ...]

    def __post_init__(self) -> None:
        _bounded_text(self.reason, field="reason", maximum=_MAX_REASON_BYTES)
        _diagnostic_paths(self.diagnostic_paths)


@dataclass(frozen=True)
class BeginFinalization:
    requested_terminal: str
    reason: str

    def __post_init__(self) -> None:
        _terminal(self.requested_terminal)
        _bounded_text(self.reason, field="reason", maximum=_MAX_REASON_BYTES)


@dataclass(frozen=True)
class StopServer:
    deadline_monotonic: float

    def __post_init__(self) -> None:
        _deadline(self.deadline_monotonic)


@dataclass(frozen=True)
class WriteQuiescence:
    native_artifacts: NativeArtifactSummary

    def __post_init__(self) -> None:
        if not isinstance(self.native_artifacts, NativeArtifactSummary):
            raise TypeError("native_artifacts must be a NativeArtifactSummary")


RuntimeAction: TypeAlias = (
    PublishGazeboReady
    | RequestSteps
    | SetPaused
    | ActivateOutput
    | WriteSourceFinished
    | WriteRuntimeFailure
    | BeginFinalization
    | StopServer
    | WriteQuiescence
)


@dataclass(frozen=True)
class ArtifactsReady:
    run_id: str

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)


@dataclass(frozen=True)
class GazeboReady:
    run_id: str

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)


@dataclass(frozen=True)
class RunStateEvent:
    run_id: str
    state: str

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)
        if self.state not in _LIFECYCLE_STATES or not isinstance(self.state, str):
            raise ValueError("state is not part of the run lifecycle")


@dataclass(frozen=True)
class AdapterCompleted:
    run_id: str
    summary: AdapterSummary

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)
        if not isinstance(self.summary, AdapterSummary):
            raise TypeError("summary must be an AdapterSummary")


@dataclass(frozen=True)
class ChildExited:
    run_id: str
    child_name: str
    returncode: int

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)
        _child_name(self.child_name)
        _integer(self.returncode, field="returncode")


@dataclass(frozen=True)
class EndpointTimeout:
    run_id: str
    endpoint: str

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)
        _bounded_text(
            self.endpoint,
            field="endpoint",
            maximum=_MAX_ENDPOINT_BYTES,
        )


@dataclass(frozen=True)
class FinalizationRequested:
    run_id: str
    requested_terminal: str
    reason: str
    deadline_monotonic: float

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)
        _terminal(self.requested_terminal)
        _bounded_text(self.reason, field="reason", maximum=_MAX_REASON_BYTES)
        _deadline(self.deadline_monotonic)


@dataclass(frozen=True)
class ServerStopped:
    run_id: str
    native_artifacts: NativeArtifactSummary

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)
        if not isinstance(self.native_artifacts, NativeArtifactSummary):
            raise TypeError("native_artifacts must be a NativeArtifactSummary")


@dataclass(frozen=True)
class ServerStopFailed:
    run_id: str
    reason: str
    diagnostic_paths: tuple[str, ...]

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)
        _bounded_text(self.reason, field="reason", maximum=_MAX_REASON_BYTES)
        if not self.diagnostic_paths:
            raise ValueError("server stop failure requires a named diagnostic path")
        _diagnostic_paths(self.diagnostic_paths)


RuntimeEvent: TypeAlias = (
    ArtifactsReady
    | GazeboReady
    | RunStateEvent
    | AdapterCompleted
    | ChildExited
    | EndpointTimeout
    | FinalizationRequested
    | ServerStopFailed
    | ServerStopped
)


class RuntimeModel:
    """Return ordered side-effect values for one canonical run."""

    def __init__(self, *, run_id: str, expected_frames: int) -> None:
        self._run_id = _canonical_run_id(run_id)
        self._expected_frames = _positive_integer(
            expected_frames, field="expected_frames"
        )
        self._artifacts_ready = False
        self._gazebo_ready = False
        self._gazebo_ready_published = False
        self._warmup_started = False
        self._lifecycle_state = "STARTING"
        self._paused = True
        self._source_summary: AdapterSummary | None = None
        self._failure_reason: str | None = None
        self._finalization_preempted = False
        self._begin_finalization_emitted = False
        self._stop_requested = False
        self._finalization_request: FinalizationRequested | None = None
        self._server_stop_failed = False
        self._native_artifacts: NativeArtifactSummary | None = None
        self._frozen = False

    def _failure(
        self,
        reason: str,
        diagnostic_paths: tuple[str, ...] = _SERVER_DIAGNOSTIC,
    ) -> tuple[RuntimeAction, ...]:
        if self._failure_reason is not None:
            return ()
        _bounded_text(reason, field="reason", maximum=_MAX_REASON_BYTES)
        _diagnostic_paths(diagnostic_paths)
        self._failure_reason = reason
        self._finalization_preempted = True
        actions: list[RuntimeAction] = [
            WriteRuntimeFailure(reason, diagnostic_paths)
        ]
        if not self._paused:
            self._paused = True
            actions.append(SetPaused(True))
        if not self._begin_finalization_emitted:
            self._begin_finalization_emitted = True
            actions.append(BeginFinalization("FAILED", reason))
        return tuple(actions)

    def _ready(self) -> tuple[RuntimeAction, ...]:
        if (
            self._artifacts_ready
            and self._gazebo_ready
            and not self._gazebo_ready_published
        ):
            self._gazebo_ready_published = True
            return (PublishGazeboReady(),)
        return ()

    def _native_stop_failure(self, reason: str) -> tuple[RuntimeAction, ...]:
        actions = self._failure(reason, _SERVER_EXIT_DIAGNOSTICS)
        self._server_stop_failed = True
        return actions

    def _accept_run_state(self, event: RunStateEvent) -> tuple[RuntimeAction, ...]:
        state = event.state
        if state == "STARTING" and self._lifecycle_state == "STARTING":
            return ()
        if state == "READY":
            if self._lifecycle_state == "READY" and self._warmup_started:
                return ()
            if not self._gazebo_ready_published:
                return self._failure("READY received before Gazebo readiness publication")
            if self._lifecycle_state != "STARTING":
                return self._failure(
                    f"invalid lifecycle transition {self._lifecycle_state} -> READY"
                )
            self._lifecycle_state = "READY"
            self._warmup_started = True
            if self._paused:
                self._paused = False
                return (SetPaused(False),)
            return ()
        if state == "RUNNING":
            if self._lifecycle_state == "RUNNING":
                return ()
            if not self._warmup_started or self._lifecycle_state != "READY":
                return self._failure(
                    "RUNNING received before the controlled readiness step"
                )
            self._lifecycle_state = "RUNNING"
            return (ActivateOutput(),)
        if state == "FINALIZING":
            self._lifecycle_state = "FINALIZING"
            self._finalization_preempted = True
            if not self._paused:
                self._paused = True
                return (SetPaused(True),)
            return ()
        if state in _TERMINAL_STATES:
            return self._failure(
                f"premature terminal lifecycle state received: {state}"
            )
        return self._failure(
            f"invalid lifecycle transition {self._lifecycle_state} -> {state}"
        )

    def _validated_completion(self, summary: AdapterSummary) -> str | None:
        counts = (
            summary.onboard_frames,
            summary.observer_frames,
            summary.paired_frames,
            summary.ground_truth_samples,
        )
        if any(type(value) is not int or value < 0 for value in counts):
            return "adapter summary counts must be nonnegative integers"
        if any(value > self._expected_frames for value in counts):
            return "adapter summary exceeds the configured frame count"
        if len(set(counts)) != 1:
            return "adapter summary requires aligned camera and ground-truth counts"
        if counts[0] != self._expected_frames:
            return f"adapter completion requires exactly {self._expected_frames} aligned samples"
        first = summary.first_sim_timestamp_ns
        last = summary.last_sim_timestamp_ns
        if (
            type(first) is not int
            or type(last) is not int
            or first != _FRAME_INTERVAL_NS
            or last != self._expected_frames * _FRAME_INTERVAL_NS
        ):
            return "adapter completion timestamp does not match the public frame cadence"
        return None

    def _accept_completion(
        self, event: AdapterCompleted
    ) -> tuple[RuntimeAction, ...]:
        if self._source_summary is not None:
            if event.summary == self._source_summary:
                return ()
            return self._failure("adapter completion changed after source-finished")
        if self._failure_reason is not None or self._finalization_preempted:
            return ()
        if self._lifecycle_state != "RUNNING":
            return self._failure("adapter completed before the run reached RUNNING")
        invalid = self._validated_completion(event.summary)
        if invalid is not None:
            return self._failure(invalid)
        assert event.summary.last_sim_timestamp_ns is not None
        self._source_summary = event.summary
        actions: list[RuntimeAction] = []
        if not self._paused:
            self._paused = True
            actions.append(SetPaused(True))
        actions.append(WriteSourceFinished(event.summary.last_sim_timestamp_ns))
        return tuple(actions)

    def _accept_finalization(
        self, event: FinalizationRequested
    ) -> tuple[RuntimeAction, ...]:
        if self._stop_requested:
            return ()
        actions: list[RuntimeAction] = []
        if (
            event.requested_terminal == "COMPLETED"
            and self._source_summary is None
            and self._failure_reason is None
        ):
            actions.extend(
                self._failure(
                    "COMPLETED finalization requested before exact source-finished"
                )
            )
        requested_terminal = (
            "FAILED" if self._failure_reason is not None else event.requested_terminal
        )
        reason = self._failure_reason or event.reason
        self._finalization_preempted = True
        self._lifecycle_state = "FINALIZING"
        if not self._paused:
            self._paused = True
            actions.append(SetPaused(True))
        if not self._begin_finalization_emitted:
            self._begin_finalization_emitted = True
            actions.append(BeginFinalization(requested_terminal, reason))
        self._finalization_request = event
        self._stop_requested = True
        actions.append(StopServer(event.deadline_monotonic))
        return tuple(actions)

    def _accept_stopped(self, event: ServerStopped) -> tuple[RuntimeAction, ...]:
        if self._native_artifacts is not None:
            if event.native_artifacts == self._native_artifacts:
                return ()
            raise RuntimeModelError("native artifact summary changed after quiescence")
        if self._server_stop_failed:
            return ()
        native = event.native_artifacts
        try:
            validated = NativeArtifactSummary(
                native.server_log_path,
                native.state_log_path,
                native.server_returncode,
                native.graceful,
            )
        except (TypeError, ValueError):
            return self._native_stop_failure("native artifact summary is malformed")
        if validated != native:
            return self._native_stop_failure("native artifact summary is inconsistent")
        if not self._stop_requested:
            return self._native_stop_failure(
                "Gazebo server stopped before finalization requested"
            )
        run_root = event.native_artifacts.server_log_path.parent.parent
        if run_root.name != self._run_id:
            return self._native_stop_failure(
                "native artifact summary belongs to another run"
            )
        self._native_artifacts = event.native_artifacts
        self._frozen = True
        return (WriteQuiescence(event.native_artifacts),)

    def _is_frozen_duplicate(self, event: object) -> bool:
        return (
            isinstance(event, ServerStopped)
            and event.native_artifacts == self._native_artifacts
        ) or (
            isinstance(event, RunStateEvent)
            and event.state in _TERMINAL_STATES | {"FINALIZING"}
        ) or (
            isinstance(event, FinalizationRequested)
            and event == self._finalization_request
        )

    def accept(self, event: object) -> tuple[RuntimeAction, ...]:
        """Accept one typed fact and return its ordered, immutable effects."""
        runtime_types = (
            ArtifactsReady,
            GazeboReady,
            RunStateEvent,
            AdapterCompleted,
            ChildExited,
            EndpointTimeout,
            FinalizationRequested,
            ServerStopFailed,
            ServerStopped,
        )
        if not isinstance(event, runtime_types):
            if self._frozen:
                raise RuntimeModelError("runtime model is frozen")
            return self._failure("unsupported runtime input")
        if event.run_id != self._run_id:
            return ()
        if self._frozen:
            if self._is_frozen_duplicate(event):
                return ()
            raise RuntimeModelError("runtime model is frozen")
        if isinstance(event, FinalizationRequested):
            return self._accept_finalization(event)
        if isinstance(event, ServerStopFailed):
            actions = self._failure(event.reason, event.diagnostic_paths)
            self._server_stop_failed = True
            return actions
        if isinstance(event, ServerStopped):
            return self._accept_stopped(event)
        if isinstance(event, ChildExited):
            return self._failure(
                f"runtime child {event.child_name} exited unexpectedly "
                f"with return code {event.returncode}",
                _SERVER_EXIT_DIAGNOSTICS,
            )
        if isinstance(event, EndpointTimeout):
            if self._finalization_preempted:
                return ()
            return self._failure(
                f"Gazebo endpoint readiness timed out: {event.endpoint}"
            )
        if self._failure_reason is not None:
            return ()
        if isinstance(event, RunStateEvent):
            return self._accept_run_state(event)
        if self._finalization_preempted:
            return ()
        if isinstance(event, ArtifactsReady):
            if self._artifacts_ready:
                return ()
            self._artifacts_ready = True
            return self._ready()
        if isinstance(event, GazeboReady):
            if self._gazebo_ready:
                return ()
            self._gazebo_ready = True
            return self._ready()
        assert isinstance(event, AdapterCompleted)
        return self._accept_completion(event)


__all__ = [
    "AdapterCompleted",
    "ArtifactsReady",
    "BeginFinalization",
    "ChildExited",
    "EndpointTimeout",
    "FinalizationRequested",
    "GazeboReady",
    "PublishGazeboReady",
    "RequestSteps",
    "RunStateEvent",
    "RuntimeAction",
    "RuntimeEvent",
    "RuntimeModel",
    "RuntimeModelError",
    "ServerStopFailed",
    "ServerStopped",
    "SetPaused",
    "ActivateOutput",
    "StopServer",
    "WriteQuiescence",
    "WriteRuntimeFailure",
    "WriteSourceFinished",
]
