"""Foreground host policy for one deterministic simulation run."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import time
from typing import Any, TextIO
from uuid import UUID, uuid4

from artifacts import (
    ArtifactSession,
    ConfigurationRecord,
    DockerLogCapture,
    DockerLogCaptureError,
    FinalizationInput,
    FinalizationResult,
    ImageDigest,
    SourceRevision,
    StructuredEvent,
    ValidationResult,
    ValidationStatus,
    read_regular_file_bytes,
    validate_regular_file,
    validate_tree,
)
from artifacts.score_validation import (
    ScoreValidationError,
    validate_descent_score_outputs,
)
from ._adapters.compose import ComposeCommandResult, ComposeRuntime
from .config import (
    RunConfig,
    RuntimeTopology,
    resolve_run_config,
    write_resolved_config,
)
from .lifecycle import LifecycleEvent, LifecycleState, RunLifecycle
from .status_store import (
    OperatorStatus,
    ProtocolFileError,
    StatusStore,
    TerminalCause,
)


_REPORT_PATHS = ("video/onboard.mp4", "video/observer.mp4", "rosbag")
_REPORT_KEYS = {
    "relative_path",
    "status",
    "detail",
    "size_bytes",
    "sha256",
    "semantic",
}
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
_COMPOSE_PS_ATTEMPT_SECONDS = 5.0
_FLIGHT_EXCHANGE_KEYS = frozenset(
    {
        "online",
        "servo_packets_received",
        "motor_updates",
        "duplicate_servo_packets",
        "servo_frame_gaps",
        "json_states_sent",
        "json_send_errors",
        "last_servo_frame",
        "last_json_sim_time_ns",
    }
)


class ControllerError(RuntimeError):
    """Controlled operator-facing failure."""


@dataclass(frozen=True)
class RunResult:
    run_id: str
    state: str
    reason: str
    manifest_path: str | None

    @property
    def exit_code(self) -> int:
        return {"COMPLETED": 0, "FAILED": 1, "ABORTED": 130}.get(self.state, 0)

    def to_dict(self) -> dict[str, str | None]:
        return {
            "result_type": "run_result",
            "run_id": self.run_id,
            "state": self.state,
            "reason": self.reason,
            "manifest_path": self.manifest_path,
        }


@dataclass(frozen=True)
class _ArtifactReportRecord:
    relative_path: str
    status: ValidationStatus
    detail: str
    size_bytes: int | None
    sha256: str | None
    semantic: Mapping[str, Any]


@dataclass(frozen=True)
class ArtifactFinalReport:
    run_id: str
    complete: bool
    records: tuple[_ArtifactReportRecord, ...]

    @classmethod
    def parse(
        cls,
        run_id: str,
        document: Any,
        deadline_check: Callable[[], None] | None = None,
    ) -> "ArtifactFinalReport":
        if deadline_check is not None:
            deadline_check()
        if not isinstance(document, dict) or set(document) != {"run_id", "complete", "records"}:
            raise ControllerError("artifacts-final report has invalid top-level keys")
        if document["run_id"] != run_id:
            raise ControllerError("artifacts-final report has the wrong run_id")
        if not isinstance(document["complete"], bool):
            raise ControllerError("artifacts-final report complete must be boolean")
        values = document["records"]
        if not isinstance(values, list) or len(values) != len(_REPORT_PATHS):
            raise ControllerError("artifacts-final report has missing or extra records")
        records: list[_ArtifactReportRecord] = []
        seen: set[str] = set()
        for value in values:
            if deadline_check is not None:
                deadline_check()
            if not isinstance(value, dict) or set(value) != _REPORT_KEYS:
                raise ControllerError("artifacts-final record has missing or extra keys")
            relative_path = value["relative_path"]
            if relative_path not in _REPORT_PATHS or relative_path in seen:
                raise ControllerError("artifacts-final report has missing, extra, or duplicate paths")
            seen.add(relative_path)
            try:
                status_value = ValidationStatus(value["status"])
            except (TypeError, ValueError) as exc:
                raise ControllerError("artifacts-final record status is invalid") from exc
            detail = value["detail"]
            if not isinstance(detail, str) or not detail:
                raise ControllerError("artifacts-final record detail must be nonempty")
            semantic = value["semantic"]
            if not isinstance(semantic, dict) or not semantic:
                raise ControllerError("artifacts-final record semantic must be a nonempty object")
            size_bytes = value["size_bytes"]
            sha256 = value["sha256"]
            if status_value is ValidationStatus.VALID:
                if (
                    isinstance(size_bytes, bool)
                    or not isinstance(size_bytes, int)
                    or size_bytes < 0
                    or not isinstance(sha256, str)
                    or _SHA256_PATTERN.fullmatch(sha256) is None
                ):
                    raise ControllerError("valid artifacts-final records require size and checksum")
            elif (size_bytes is None) != (sha256 is None) or (
                size_bytes is not None
                and (
                    isinstance(size_bytes, bool)
                    or not isinstance(size_bytes, int)
                    or size_bytes < 0
                    or not isinstance(sha256, str)
                    or _SHA256_PATTERN.fullmatch(sha256) is None
                )
            ):
                raise ControllerError(
                    "non-valid artifacts-final record size/checksum must both be null or valid"
                )
            records.append(
                _ArtifactReportRecord(
                    relative_path,
                    status_value,
                    detail,
                    size_bytes,
                    sha256,
                    semantic,
                )
            )
        if set(seen) != set(_REPORT_PATHS):
            raise ControllerError("artifacts-final report has missing records")
        ordered = tuple(sorted(records, key=lambda item: _REPORT_PATHS.index(item.relative_path)))
        expected_complete = all(record.status is ValidationStatus.VALID for record in ordered)
        if document["complete"] != expected_complete:
            raise ControllerError("artifacts-final aggregate complete disagrees with records")
        if deadline_check is not None:
            deadline_check()
        return cls(run_id, document["complete"], ordered)

    def first_failure(self) -> str | None:
        for record in self.records:
            if record.status is not ValidationStatus.VALID:
                return record.detail
        return None

    def validators(
        self, deadline_check: Callable[[], None] | None = None
    ) -> dict[str, Callable[[Path, str], ValidationResult]]:
        return {
            record.relative_path: self._validator(record, deadline_check)
            for record in self.records
        }

    @staticmethod
    def _validator(
        record: _ArtifactReportRecord,
        deadline_check: Callable[[], None] | None = None,
    ) -> Callable[[Path, str], ValidationResult]:
        def validate(run_directory: Path, relative_path: str) -> ValidationResult:
            if deadline_check is not None:
                deadline_check()
            if relative_path != record.relative_path:
                return ValidationResult(
                    ValidationStatus.INVALID,
                    None,
                    None,
                    "artifacts-final validator path mismatch",
                )
            if record.status is not ValidationStatus.VALID:
                return ValidationResult(record.status, None, None, record.detail)
            host_result = (
                validate_tree(
                    run_directory,
                    relative_path,
                    deadline_check=deadline_check,
                )
                if relative_path == "rosbag"
                else validate_regular_file(
                    run_directory,
                    relative_path,
                    deadline_check=deadline_check,
                )
            )
            if deadline_check is not None:
                deadline_check()
            if host_result.status is not ValidationStatus.VALID:
                return host_result
            if host_result.size_bytes != record.size_bytes:
                return ValidationResult(
                    ValidationStatus.INVALID,
                    None,
                    None,
                    f"artifacts-final size mismatch for {relative_path}",
                )
            if host_result.sha256 != record.sha256:
                return ValidationResult(
                    ValidationStatus.INVALID,
                    None,
                    None,
                    f"artifacts-final checksum mismatch for {relative_path}",
                )
            return host_result

        return validate


class _HostEventLog:
    """One exclusively-owned append stream consumed by Task 5 capture."""

    def __init__(
        self,
        run_directory: Path,
        run_id: str,
        stream: TextIO,
        utcnow: Callable[[], datetime],
    ) -> None:
        self._run_directory = run_directory
        self._run_id = run_id
        self._stream = stream
        self._utcnow = utcnow
        self._closed = False
        self._file_failed = False
        self._stream_failed = False
        logs = run_directory / "logs"
        try:
            logs.mkdir(mode=0o755)
        except FileExistsError:
            pass
        metadata = logs.lstat()
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
            raise ControllerError("logs path is not a safe directory")
        self._logs_fd = os.open(
            logs,
            os.O_RDONLY
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_DIRECTORY", 0)
            | getattr(os, "O_NOFOLLOW", 0),
        )
        try:
            os.mkdir("docker", 0o755, dir_fd=self._logs_fd)
            os.fsync(self._logs_fd)
            self._descriptor = os.open(
                "orchestration-host.jsonl.partial",
                os.O_WRONLY
                | os.O_APPEND
                | os.O_CREAT
                | os.O_EXCL
                | getattr(os, "O_CLOEXEC", 0)
                | getattr(os, "O_NOFOLLOW", 0),
                0o600,
                dir_fd=self._logs_fd,
            )
            os.fsync(self._logs_fd)
        except BaseException:
            os.close(self._logs_fd)
            raise

    def emit(self, event: str, *, severity: str = "INFO", **fields: Any) -> None:
        if self._closed:
            raise RuntimeError("host event log is closed")
        if self._file_failed:
            return
        timestamp = self._utcnow()
        if timestamp.tzinfo is None or timestamp.utcoffset() is None:
            raise ControllerError("host event wall clock must be timezone-aware")
        structured = StructuredEvent(
            run_id=self._run_id,
            module="orchestration",
            severity=severity,
            event=event,
            sim_timestamp=None,
            wall_timestamp=timestamp.astimezone(timezone.utc),
            fields=fields,
        )
        line = structured.to_json_line()
        payload = line.encode("utf-8")
        written = 0
        try:
            while written < len(payload):
                count = os.write(self._descriptor, payload[written:])
                if count <= 0:
                    raise OSError("host event append made no progress")
                written += count
            os.fsync(self._descriptor)
        except Exception:
            self._file_failed = True
            raise
        if not self._stream_failed:
            try:
                self._stream.write(line)
                self._stream.flush()
            except Exception:
                self._stream_failed = True
                raise

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        failure: Exception | None = None
        try:
            os.fsync(self._descriptor)
        except Exception as exc:
            failure = exc
        try:
            os.close(self._descriptor)
        except Exception as exc:
            failure = failure or exc
        try:
            os.fsync(self._logs_fd)
        except Exception as exc:
            failure = failure or exc
        try:
            os.close(self._logs_fd)
        except Exception as exc:
            failure = failure or exc
        if failure is not None:
            raise failure


class _UnavailableCompose:
    """Diagnostic-only boundary used when adapter construction itself fails."""

    def __init__(self, run_id: str, detail: str) -> None:
        self.project_name = f"drone-sim-{run_id.replace('-', '')}"
        self.detail = detail

    def logs(self, _command: list[str], _timeout: float):
        from artifacts import DockerLogCommandResult

        return DockerLogCommandResult(1, self.detail.encode("utf-8"))

    def image_digests(self, _timeout: float) -> tuple[ImageDigest, ...]:
        return ()


class RunController:
    """Own the host lifecycle policy; dependencies remain injectable boundaries."""

    def __init__(
        self,
        *,
        project_directory: Path | str | None = None,
        status_store_factory: Callable[[Path], StatusStore] = StatusStore,
        compose_factory: Callable[[RunConfig, Path], Any] | None = None,
        log_capture_factory: Callable[..., Any] = DockerLogCapture,
        artifact_session_factory: Callable[..., Any] | None = ArtifactSession,
        config_writer: Callable[[Path, RunConfig], Path] = write_resolved_config,
        uuid_factory: Callable[[], UUID] = uuid4,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        utcnow: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        event_stream: TextIO | None = None,
        poll_interval: float = 0.1,
        source_runner: Callable[..., Any] = subprocess.run,
    ) -> None:
        self.project_directory = Path(
            project_directory
            if project_directory is not None
            else Path(__file__).resolve().parents[3]
        ).resolve()
        self.status_store_factory = status_store_factory
        self.compose_factory = compose_factory or self._default_compose
        self.log_capture_factory = log_capture_factory
        self.artifact_session_factory = artifact_session_factory or ArtifactSession
        self.config_writer = config_writer
        self.uuid_factory = uuid_factory
        self.monotonic = monotonic
        self.sleep = sleep
        self.utcnow = utcnow
        self.event_stream = event_stream or sys.stdout
        if isinstance(poll_interval, bool) or poll_interval <= 0:
            raise ValueError("poll_interval must be positive")
        self.poll_interval = float(poll_interval)
        self.source_runner = source_runner

    def _default_compose(self, config: RunConfig, run_directory: Path) -> ComposeRuntime:
        return ComposeRuntime(
            project_directory=self.project_directory,
            run_id=config.run_id,
            run_directory=run_directory,
            config_path=run_directory / "configuration/run.json",
            topology=config.topology,
            monotonic=self.monotonic,
        )

    @staticmethod
    def _remaining(deadline: float, monotonic: Callable[[], float]) -> float:
        return max(0.0, deadline - monotonic())

    def _deadline_check(self, deadline: float) -> Callable[[], None]:
        def check() -> None:
            if self.monotonic() >= deadline:
                raise TimeoutError("finalization_deadline")

        return check

    @staticmethod
    def _cause_for_request(document: Mapping[str, Any]) -> TerminalCause:
        terminal = document["requested_terminal"]
        return TerminalCause(
            "operator_abort" if terminal == "ABORTED" else "requested_terminal",
            document["reason"],
        )

    @staticmethod
    def _validate_ready(document: Mapping[str, Any]) -> None:
        if set(document) != {"run_id", "ready"} or document["ready"] is not True:
            raise ProtocolFileError("artifacts-ready status is invalid")

    @staticmethod
    def _validate_gazebo_ready(document: Mapping[str, Any]) -> None:
        exchange = document.get("flight_exchange")
        if (
            set(document) != {"run_id", "ready", "flight_exchange"}
            or document["ready"] is not True
            or not isinstance(exchange, dict)
            or set(exchange) != _FLIGHT_EXCHANGE_KEYS
            or exchange["online"] is not True
            or any(
                type(exchange[key]) is not int or exchange[key] < 0
                for key in _FLIGHT_EXCHANGE_KEYS - {"online"}
            )
            or exchange["servo_packets_received"] < 1
            or exchange["motor_updates"] < 1
            or exchange["json_states_sent"] < 1
            or exchange["servo_frame_gaps"] != 0
            or exchange["json_send_errors"] != 0
        ):
            raise ProtocolFileError("gazebo-ready status is invalid")

    @staticmethod
    def _validate_ardupilot_ready(document: Mapping[str, Any]) -> None:
        if (
            set(document)
            != {"run_id", "ready", "json_exchange", "mavlink_endpoint"}
            or document["ready"] is not True
            or document["json_exchange"] is not True
            or document["mavlink_endpoint"] != "tcp://ardupilot-sitl:5760"
        ):
            raise ProtocolFileError("ardupilot-ready status is invalid")

    @staticmethod
    def _validate_companion_ready(document: Mapping[str, Any]) -> None:
        if (
            set(document)
            != {
                "run_id",
                "ready",
                "mavlink_endpoint",
                "mavlink_transport_connected",
            }
            or document["ready"] is not True
            or document["mavlink_endpoint"] != "tcp://ardupilot-sitl:5760"
            or document["mavlink_transport_connected"] is not True
        ):
            raise ProtocolFileError("companion-ready status is invalid")

    @staticmethod
    def _validate_mission_ready(document: Mapping[str, Any]) -> None:
        if (
            set(document)
            != {
                "run_id",
                "ready",
                "heartbeat_observed",
                "prearm_checks_healthy",
            }
            or document["ready"] is not True
            or document["heartbeat_observed"] is not True
            or document["prearm_checks_healthy"] is not True
        ):
            raise ProtocolFileError("mission-ready status is invalid")

    @staticmethod
    def _validate_running(document: Mapping[str, Any]) -> int:
        if set(document) != {"run_id", "state", "sim_timestamp_ns"}:
            raise ProtocolFileError("runtime-running status is invalid")
        stamp = document["sim_timestamp_ns"]
        if (
            document["state"] != "RUNNING"
            or isinstance(stamp, bool)
            or not isinstance(stamp, int)
            or stamp < 0
        ):
            raise ProtocolFileError("runtime-running status is invalid")
        return stamp

    @staticmethod
    def _source_stamp(document: Mapping[str, Any]) -> int | None:
        stamp = document.get("sim_timestamp_ns")
        if stamp is None:
            return None
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp < 0:
            raise ProtocolFileError("source-finished simulation timestamp is invalid")
        return stamp

    @staticmethod
    def _mission_stamp(document: Mapping[str, Any]) -> int:
        if set(document) != {"run_id", "finished", "sim_timestamp_ns", "outcome"}:
            raise ProtocolFileError("mission-finished status is invalid")
        stamp = document["sim_timestamp_ns"]
        if (
            document["finished"] is not True
            or document["outcome"] != "LANDED"
            or isinstance(stamp, bool)
            or not isinstance(stamp, int)
            or stamp < 0
        ):
            raise ProtocolFileError("mission-finished status is invalid")
        return stamp

    @staticmethod
    def _score_stamp(document: Mapping[str, Any]) -> int:
        if set(document) != {"run_id", "finished", "sim_timestamp_ns"}:
            raise ProtocolFileError("score-finished status is invalid")
        stamp = document["sim_timestamp_ns"]
        if (
            document["finished"] is not True
            or isinstance(stamp, bool)
            or not isinstance(stamp, int)
            or stamp < 0
        ):
            raise ProtocolFileError("score-finished status is invalid")
        return stamp

    @staticmethod
    def _ps_cause(
        result: ComposeCommandResult, topology: RuntimeTopology
    ) -> TerminalCause | None:
        if result.returncode != 0:
            return TerminalCause("child_process", "compose_ps_failed")
        try:
            decoded = result.output.decode("utf-8")
        except UnicodeDecodeError:
            return TerminalCause("child_process", "compose_ps_invalid")
        try:
            value = json.loads(decoded)
            rows = value if isinstance(value, list) else [value]
        except json.JSONDecodeError:
            try:
                rows = [json.loads(line) for line in decoded.splitlines() if line]
            except json.JSONDecodeError:
                return TerminalCause("child_process", "compose_ps_invalid")
        if not rows or any(
            not isinstance(row, dict)
            or not isinstance(row.get("Service"), str)
            or not row["Service"]
            or not isinstance(row.get("State"), str)
            for row in rows
        ):
            return TerminalCause("child_process", "compose_ps_invalid")
        if any(
            row["State"].lower() not in {"running", "restarting"}
            or str(row.get("Health", "")).lower() == "unhealthy"
            for row in rows
        ):
            return TerminalCause("child_process", "compose_child_exited")
        services = [row["Service"] for row in rows]
        required_services = {service for service, _module in topology.ownership}
        if len(services) != len(set(services)) or set(services) != required_services:
            return TerminalCause("child_process", "compose_child_set_invalid")
        return None

    def _observed_cause(
        self,
        store: StatusStore,
        run_id: str,
        compose: Any,
        topology: RuntimeTopology,
        deadline: float,
        deadline_check: Callable[[], None],
    ) -> TerminalCause | None:
        request = store.read_finalize_request(run_id)
        if request is not None:
            return self._cause_for_request(request)
        deadline_check()
        failure = store.read_runtime_status(
            run_id,
            "runtime-failure",
            deadline_check=deadline_check,
        )
        deadline_check()
        if failure is not None:
            reason = failure.get("reason")
            module = failure.get("module")
            if not isinstance(reason, str) or not reason:
                return TerminalCause("runtime_failure", "invalid_runtime_failure")
            return TerminalCause(
                "runtime_failure",
                reason,
                module if isinstance(module, str) and module else None,
            )
        remaining = self._remaining(deadline, self.monotonic)
        try:
            deadline_check()
            result = compose.ps(min(remaining, _COMPOSE_PS_ATTEMPT_SECONDS))
            deadline_check()
            return self._ps_cause(result, topology)
        except subprocess.TimeoutExpired:
            return None
        except TimeoutError:
            raise
        except Exception as exc:
            return TerminalCause("child_process", f"compose_ps_exception:{type(exc).__name__}")

    def _wait_for(
        self,
        store: StatusStore,
        run_id: str,
        compose: Any,
        topology: RuntimeTopology,
        name: str,
        deadline: float,
        deadline_cause: TerminalCause,
        *,
        observe_causes: bool = True,
        deadline_check: Callable[[], None] | None = None,
    ) -> tuple[dict[str, Any] | None, TerminalCause | None]:
        checker = deadline_check or self._deadline_check(deadline)
        while True:
            try:
                checker()
                if observe_causes:
                    cause = self._observed_cause(
                        store,
                        run_id,
                        compose,
                        topology,
                        deadline,
                        checker,
                    )
                    if cause is not None:
                        return None, cause
                checker()
                document = store.read_runtime_status(
                    run_id,
                    name,
                    deadline_check=checker,
                )
                checker()
            except TimeoutError:
                return None, deadline_cause
            if document is not None:
                return document, None
            remaining = self._remaining(deadline, self.monotonic)
            if remaining <= 0:
                return None, deadline_cause
            self.sleep(min(self.poll_interval, remaining))

    @staticmethod
    def _status(
        lifecycle: RunLifecycle,
        *,
        manifest_path: str | None = None,
        primary: TerminalCause | None = None,
        diagnostics: tuple[TerminalCause, ...] = (),
    ) -> OperatorStatus:
        return OperatorStatus(
            lifecycle.run_id,
            lifecycle.state.value,
            lifecycle.reason,
            manifest_path,
            primary,
            diagnostics,
        )

    @staticmethod
    def _terminal_event(requested: str) -> LifecycleEvent:
        return {
            "COMPLETED": LifecycleEvent.COMPLETE,
            "FAILED": LifecycleEvent.FAIL,
            "ABORTED": LifecycleEvent.ABORT,
        }[requested]

    def _source_revisions(self, deadline: float) -> tuple[SourceRevision, ...]:
        deadline_check = self._deadline_check(deadline)
        records: list[SourceRevision] = []
        try:
            for name, repository in (
                ("drone_sim", self.project_directory),
                ("comp2026", self.project_directory / "companion/comp2026"),
            ):
                deadline_check()
                revision = self.source_runner(
                    ["git", "-C", str(repository), "rev-parse", "HEAD"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    check=False,
                    shell=False,
                    timeout=self._remaining(deadline, self.monotonic),
                )
                deadline_check()
                dirty = self.source_runner(
                    [
                        "git",
                        "-C",
                        str(repository),
                        "status",
                        "--porcelain=v1",
                        "--untracked-files=normal",
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    check=False,
                    shell=False,
                    timeout=self._remaining(deadline, self.monotonic),
                )
                deadline_check()
                if revision.returncode != 0 or dirty.returncode != 0:
                    raise ControllerError("source_revision_unavailable")
                try:
                    value = revision.stdout.decode("ascii").strip()
                except UnicodeDecodeError as exc:
                    raise ControllerError("source_revision_invalid") from exc
                if not value:
                    raise ControllerError("source_revision_invalid")
                records.append(SourceRevision(name, value, bool(dirty.stdout)))
        except TimeoutError:
            raise
        except ControllerError:
            raise
        except Exception as exc:
            raise ControllerError("source_revision_unavailable") from exc
        return tuple(records)

    def _score_metadata(
        self,
        run_directory: Path,
        deadline_check: Callable[[], None] | None = None,
    ) -> tuple[float | None, float | None, str | None, tuple[str, ...]]:
        try:
            validation, payload = read_regular_file_bytes(
                run_directory,
                "scoring/result.json",
                deadline_check=deadline_check,
            )
            if validation.status is not ValidationStatus.VALID or payload is None:
                return None, None, None, ()
            if deadline_check is not None:
                deadline_check()
            document = json.loads(payload.decode("utf-8"))
            if deadline_check is not None:
                deadline_check()
        except TimeoutError:
            raise
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
            return None, None, None, ()
        if not isinstance(document, dict):
            return None, None, None, ()
        achieved = document.get("achieved_score")
        maximum = document.get("maximum_available_score")
        if (
            isinstance(achieved, bool)
            or not isinstance(achieved, (int, float))
            or not math.isfinite(achieved)
            or isinstance(maximum, bool)
            or not isinstance(maximum, (int, float))
            or not math.isfinite(maximum)
        ):
            return None, None, None, ()
        evidence = document.get("evidence_paths", [])
        scoring_checksum = document.get("scoring_checksum")
        if (
            not isinstance(scoring_checksum, str)
            or _SHA256_PATTERN.fullmatch(scoring_checksum) is None
            or not isinstance(evidence, list)
            or any(not isinstance(item, str) for item in evidence)
            or len(evidence) != len(set(evidence))
        ):
            return None, None, None, ()
        for item in evidence:
            relative_value = item.split("#", 1)[0]
            relative = Path(relative_value)
            if (
                not relative_value
                or relative.is_absolute()
                or ".." in relative.parts
                or relative.as_posix() != relative_value
            ):
                return None, None, None, ()
        return (
            achieved,
            maximum,
            scoring_checksum,
            tuple(evidence),
        )

    @staticmethod
    def _first_invalid(
        run_directory: Path,
        validators: Mapping[str, Callable[[Path, str], ValidationResult]],
        deadline_check: Callable[[], None] | None = None,
    ) -> str | None:
        for path in _REPORT_PATHS:
            if deadline_check is not None:
                deadline_check()
            result = validators[path](run_directory, path)
            if result.status is not ValidationStatus.VALID:
                return result.detail
        return None

    @staticmethod
    def _remove_captured_host_events(run_directory: Path) -> None:
        """Remove the closed host source after its structured log is published."""
        logs_fd = os.open(
            run_directory / "logs",
            os.O_RDONLY
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_DIRECTORY", 0)
            | getattr(os, "O_NOFOLLOW", 0),
        )
        descriptor: int | None = None
        try:
            name = "orchestration-host.jsonl.partial"
            before = os.stat(name, dir_fd=logs_fd, follow_symlinks=False)
            if (
                stat.S_ISLNK(before.st_mode)
                or not stat.S_ISREG(before.st_mode)
                or before.st_nlink != 1
            ):
                raise ControllerError("captured host event source is unsafe")
            descriptor = os.open(
                name,
                os.O_RDONLY
                | getattr(os, "O_CLOEXEC", 0)
                | getattr(os, "O_NOFOLLOW", 0),
                dir_fd=logs_fd,
            )
            opened = os.fstat(descriptor)
            if (before.st_dev, before.st_ino, before.st_size) != (
                opened.st_dev,
                opened.st_ino,
                opened.st_size,
            ):
                raise ControllerError("captured host event source changed while opening")
            os.close(descriptor)
            descriptor = None
            os.unlink(name, dir_fd=logs_fd)
            os.fsync(logs_fd)
        finally:
            if descriptor is not None:
                os.close(descriptor)
            os.close(logs_fd)

    def start(self, config_path: Path | str) -> RunResult:
        try:
            config = resolve_run_config(config_path, run_id_factory=self.uuid_factory)
        except (OSError, ValueError) as exc:
            raise ControllerError(str(exc)) from exc
        topology = config.topology
        store = self.status_store_factory(config.output_root)
        run_directory = store.allocate(config.run_id)
        lifecycle = RunLifecycle.created(config.run_id)
        store.write_operator_status(self._status(lifecycle))
        diagnostics: list[TerminalCause] = []
        event_log = _HostEventLog(
            run_directory, config.run_id, self.event_stream, self.utcnow
        )

        def retain_observability(operation: str, exc: BaseException) -> None:
            diagnostic = TerminalCause(
                "observability",
                f"host_event_{operation}_failed:{type(exc).__name__}",
            )
            if diagnostic not in diagnostics:
                diagnostics.append(diagnostic)

        def emit_event(event: str, **fields: Any) -> None:
            try:
                event_log.emit(event, **fields)
            except Exception as exc:
                retain_observability("emit", exc)

        def close_event_log() -> None:
            try:
                event_log.close()
            except Exception as exc:
                retain_observability("close", exc)

        wall_started = self.utcnow()
        mono_started = self.monotonic()
        compose: Any | None = None
        compose_started = False
        compose_attempted = False
        primary: TerminalCause | None = None
        sim_start_ns: int | None = None
        sim_end_ns: int | None = None
        manifest_path: Path | None = None
        source_revisions: tuple[SourceRevision, ...] = ()
        try:
            self.config_writer(run_directory, config)
            lifecycle = lifecycle.apply(LifecycleEvent.START)
            store.write_operator_status(self._status(lifecycle))
            emit_event("run_starting", config_sha256=config.config_sha256)
            try:
                compose = self.compose_factory(config, run_directory)
            except Exception as exc:
                primary = TerminalCause(
                    "compose_factory",
                    f"compose_factory_exception:{type(exc).__name__}",
                )
                compose = _UnavailableCompose(config.run_id, str(exc))
            startup_deadline = min(
                mono_started + config.startup_wall_seconds,
                mono_started + config.max_wall_seconds,
            )
            if primary is None:
                try:
                    source_revisions = self._source_revisions(startup_deadline)
                    if config.runtime_profile == "phase3":
                        compose.bind_source_revisions(
                            source_revisions,
                            self._remaining(startup_deadline, self.monotonic),
                        )
                except TimeoutError:
                    primary = TerminalCause("provenance", "startup_deadline")
                except Exception as exc:
                    primary = TerminalCause(
                        "provenance",
                        str(exc)
                        if isinstance(exc, ControllerError)
                        else "source_revision_binding_failed",
                    )
            if primary is None:
                try:
                    compose_attempted = True
                    up_result = compose.up(
                        self._remaining(startup_deadline, self.monotonic)
                    )
                    if up_result.returncode != 0:
                        emit_event(
                            "compose_up_failed",
                            exit_code=up_result.returncode,
                            output=up_result.output.decode("utf-8", errors="replace"),
                        )
                        primary = TerminalCause("compose_start", "compose_up_failed")
                    else:
                        compose_started = True
                except KeyboardInterrupt:
                    primary = TerminalCause("operator_interrupt", "operator_interrupt")
                except Exception as exc:
                    primary = TerminalCause(
                        "compose_start",
                        f"compose_up_exception:{type(exc).__name__}",
                    )

            if compose_started and primary is None:
                try:
                    ready, primary = self._wait_for(
                        store,
                        config.run_id,
                        compose,
                        topology,
                        "artifacts-ready",
                        startup_deadline,
                        TerminalCause("startup_deadline", "startup_deadline"),
                    )
                    if ready is not None:
                        self._validate_ready(ready)
                    if primary is None and config.runtime_profile == "phase3":
                        gazebo_ready, primary = self._wait_for(
                            store,
                            config.run_id,
                            compose,
                            topology,
                            "gazebo-ready",
                            startup_deadline,
                            TerminalCause("startup_deadline", "gazebo_readiness_stall"),
                        )
                        if gazebo_ready is not None:
                            self._validate_gazebo_ready(gazebo_ready)
                    if primary is None:
                        lifecycle = lifecycle.apply(LifecycleEvent.MODULES_READY)
                        store.write_operator_status(self._status(lifecycle))

                    if primary is None and config.runtime_profile == "phase3":
                        ardupilot_ready, primary = self._wait_for(
                            store,
                            config.run_id,
                            compose,
                            topology,
                            "ardupilot-ready",
                            startup_deadline,
                            TerminalCause("startup_deadline", "ardupilot_readiness_stall"),
                        )
                        if ardupilot_ready is not None:
                            self._validate_ardupilot_ready(ardupilot_ready)
                    if primary is None and config.runtime_profile == "phase3":
                        companion_ready, primary = self._wait_for(
                            store,
                            config.run_id,
                            compose,
                            topology,
                            "companion-ready",
                            startup_deadline,
                            TerminalCause("startup_deadline", "companion_readiness_stall"),
                        )
                        if companion_ready is not None:
                            self._validate_companion_ready(companion_ready)
                    overall_deadline = mono_started + config.max_wall_seconds
                    if primary is None and config.runtime_profile == "phase3":
                        mission_ready, primary = self._wait_for(
                            store,
                            config.run_id,
                            compose,
                            topology,
                            "mission-ready",
                            overall_deadline,
                            TerminalCause("mission_stall", "mission_readiness_stall"),
                        )
                        if mission_ready is not None:
                            self._validate_mission_ready(mission_ready)
                    if primary is None:
                        running, primary = self._wait_for(
                            store,
                            config.run_id,
                            compose,
                            topology,
                            "runtime-running",
                            overall_deadline,
                            TerminalCause("clock_stall", "clock_source_stall"),
                        )
                        if running is not None:
                            sim_start_ns = self._validate_running(running)
                            lifecycle = lifecycle.apply(LifecycleEvent.CLOCK_STARTED)
                            store.write_operator_status(self._status(lifecycle))
                    if primary is None:
                        finished, primary = self._wait_for(
                            store,
                            config.run_id,
                            compose,
                            topology,
                            "source-finished",
                            overall_deadline,
                            TerminalCause("clock_stall", "clock_source_stall"),
                        )
                        if finished is not None:
                            sim_end_ns = self._source_stamp(finished)
                    if primary is None and config.runtime_profile == "phase3":
                        mission_finished, primary = self._wait_for(
                            store,
                            config.run_id,
                            compose,
                            topology,
                            "mission-finished",
                            overall_deadline,
                            TerminalCause("mission_stall", "mission_completion_stall"),
                        )
                        if mission_finished is not None:
                            mission_stamp = self._mission_stamp(mission_finished)
                            if sim_end_ns is not None and mission_stamp > sim_end_ns:
                                raise ProtocolFileError(
                                    "mission-finished timestamp exceeds source-finished"
                                )
                    if primary is None and config.runtime_profile == "phase3":
                        score_finished, primary = self._wait_for(
                            store,
                            config.run_id,
                            compose,
                            topology,
                            "score-finished",
                            overall_deadline,
                            TerminalCause("score_stall", "score_completion_stall"),
                        )
                        if score_finished is not None:
                            score_stamp = self._score_stamp(score_finished)
                            if sim_end_ns is None or score_stamp != sim_end_ns:
                                raise ProtocolFileError(
                                    "score-finished timestamp must equal source-finished"
                                )
                except KeyboardInterrupt:
                    primary = TerminalCause("operator_interrupt", "operator_interrupt")
                except (ProtocolFileError, ControllerError) as exc:
                    primary = TerminalCause("protocol", str(exc))

            requested = "COMPLETED" if primary is None else (
                "ABORTED"
                if primary.kind in {"operator_abort", "operator_interrupt"}
                else "FAILED"
            )
            reason = "mission_complete" if primary is None else primary.reason

            if compose is not None:
                persisted = store.request_finalization(config.run_id, requested, reason)
                requested = persisted["requested_terminal"]
                reason = persisted["reason"]
                persisted_cause = self._cause_for_request(persisted)
                if primary is None or persisted_cause.kind == "operator_abort":
                    primary = persisted_cause if requested != "COMPLETED" else None
                lifecycle = lifecycle.apply(self._terminal_event(requested), reason=reason)
                store.write_operator_status(
                    self._status(lifecycle, primary=primary, diagnostics=tuple(diagnostics))
                )
                emit_event(
                    "run_finalizing",
                    requested_terminal=requested,
                    reason=reason,
                )

                final_deadline = self.monotonic() + config.finalization_wall_seconds
                teardown_reserve = min(5.0, config.finalization_wall_seconds / 5.0)
                manifest_reserve = min(5.0, config.finalization_wall_seconds / 5.0)
                manifest_deadline = final_deadline - teardown_reserve
                work_deadline = manifest_deadline - manifest_reserve
                work_deadline_check = self._deadline_check(work_deadline)
                manifest_deadline_check = self._deadline_check(manifest_deadline)

                if compose_started:
                    for status_name in ("runtime-frozen", "artifacts-final"):
                        try:
                            document, cause = self._wait_for(
                                store,
                                config.run_id,
                                compose,
                                topology,
                                status_name,
                                work_deadline,
                                TerminalCause("finalization_deadline", "finalization_deadline"),
                                observe_causes=False,
                                deadline_check=work_deadline_check,
                            )
                        except KeyboardInterrupt:
                            document = None
                            cause = TerminalCause("operator_interrupt", "operator_interrupt")
                        if cause is not None:
                            if primary is None:
                                primary = cause
                            elif cause != primary:
                                diagnostics.append(cause)
                            if requested == "COMPLETED":
                                requested = "FAILED"
                                reason = cause.reason
                            break

                emit_event("log_capture_starting")
                close_event_log()
                try:
                    capture = self.log_capture_factory(
                        run_directory=run_directory,
                        project_name=compose.project_name,
                        ownership=topology.ownership,
                        run_id=config.run_id,
                        command_runner=lambda command: compose.logs(
                            command,
                            self._remaining(work_deadline, self.monotonic),
                        ),
                        host_events=True,
                        deadline_check=work_deadline_check,
                    )
                    capture.capture()
                    self._remove_captured_host_events(run_directory)
                except KeyboardInterrupt:
                    requested = "ABORTED"
                    reason = "operator_interrupt"
                    primary = primary or TerminalCause("operator_interrupt", reason)
                except DockerLogCaptureError as exc:
                    deadline_exhausted = any(
                        item.kind == "deadline" for item in exc.result.diagnostics
                    )
                    diagnostic = TerminalCause(
                        "docker_log_capture",
                        "finalization_deadline"
                        if deadline_exhausted
                        else "docker_log_capture_failed",
                    )
                    if primary is None:
                        primary = diagnostic
                    else:
                        diagnostics.append(diagnostic)
                    if requested == "COMPLETED":
                        requested = "FAILED"
                        reason = diagnostic.reason
                    for item in exc.result.diagnostics:
                        diagnostics.append(
                            TerminalCause("docker_log_diagnostic", item.detail, item.module)
                        )
                except Exception as exc:
                    diagnostic = TerminalCause(
                        "docker_log_capture",
                        f"docker_log_capture_exception:{type(exc).__name__}",
                    )
                    primary = primary or diagnostic
                    if requested == "COMPLETED":
                        requested = "FAILED"
                        reason = diagnostic.reason

                try:
                    work_deadline_check()
                    report_document = store.read_runtime_status(
                        config.run_id,
                        "artifacts-final",
                        deadline_check=work_deadline_check,
                    )
                    work_deadline_check()
                    if report_document is None:
                        raise ControllerError("artifacts-final report is missing")
                    report = ArtifactFinalReport.parse(
                        config.run_id,
                        report_document,
                        work_deadline_check,
                    )
                    validators = report.validators(work_deadline_check)
                    report_failure = report.first_failure() or self._first_invalid(
                        run_directory,
                        validators,
                        work_deadline_check,
                    )
                except (ControllerError, ProtocolFileError, TimeoutError) as exc:
                    report = None
                    validators = {
                        path: (
                            lambda detail: lambda _root, _path: ValidationResult(
                                ValidationStatus.INVALID, None, None, detail
                            )
                        )(str(exc))
                        for path in _REPORT_PATHS
                    }
                    report_failure = str(exc)
                if report_failure is not None and requested == "COMPLETED":
                    requested = "FAILED"
                    reason = report_failure
                    primary = primary or TerminalCause("artifact_validation", reason)

                try:
                    work_deadline_check()
                    image_digests = tuple(
                        compose.image_digests(
                            self._remaining(work_deadline, self.monotonic)
                        )
                    )
                    if not image_digests:
                        raise ControllerError("image_digest_unavailable")
                    work_deadline_check()
                except Exception as exc:
                    image_digests = ()
                    if requested == "COMPLETED":
                        requested = "FAILED"
                        reason = (
                            "finalization_deadline"
                            if isinstance(exc, TimeoutError)
                            else "image_digest_unavailable"
                        )
                        primary = primary or TerminalCause("provenance", reason)
                try:
                    if (
                        requested == "COMPLETED"
                        and config.runtime_profile == "phase3"
                        and config.scenario != "competition_v1"
                    ):
                        score = validate_descent_score_outputs(
                            run_directory,
                            run_id=config.run_id,
                            rules_path=(
                                self.project_directory
                                / "scorekeeper/rules/descent_v1.json"
                            ),
                            deadline_check=work_deadline_check,
                        )
                        achieved = score.achieved_score
                        maximum = score.maximum_available_score
                        scoring_checksum = score.scoring_checksum
                        evidence = score.evidence_paths
                    else:
                        achieved, maximum, scoring_checksum, evidence = self._score_metadata(
                            run_directory,
                            work_deadline_check,
                        )
                except TimeoutError:
                    achieved, maximum, scoring_checksum, evidence = (None, None, None, ())
                    if requested == "COMPLETED":
                        requested = "FAILED"
                        reason = "finalization_deadline"
                        primary = primary or TerminalCause("provenance", reason)
                except ScoreValidationError:
                    achieved, maximum, scoring_checksum, evidence = (None, None, None, ())
                if (
                    requested == "COMPLETED"
                    and (
                        achieved is None
                        or maximum is None
                        or scoring_checksum is None
                    )
                ):
                    requested = "FAILED"
                    reason = "scoring_provenance_invalid"
                    primary = primary or TerminalCause("provenance", reason)
                if (sim_start_ns is None) != (sim_end_ns is None):
                    sim_start_ns = None
                    sim_end_ns = None
                request = FinalizationInput(
                    run_id=config.run_id,
                    requested_terminal=requested,
                    reason=reason,
                    sim_start_ns=sim_start_ns,
                    sim_end_ns=sim_end_ns,
                    wall_started_at=wall_started,
                    wall_ended_at=self.utcnow(),
                    source_revisions=source_revisions,
                    image_digests=image_digests,
                    configuration_records=(
                        ConfigurationRecord("configuration/run.json", config.config_sha256),
                    ),
                    achieved_score=achieved,
                    maximum_available_score=maximum,
                    scoring_checksum=scoring_checksum,
                    evidence_paths=evidence,
                )
                committed: FinalizationResult | None = None
                try:
                    session = self.artifact_session_factory(
                        run_directory,
                        validators=validators,
                        deadline_check=work_deadline_check,
                        commit_deadline_check=manifest_deadline_check,
                        physical_gazebo=config.runtime_profile == "phase3",
                    )
                    committed = session.finalize_with_result(request)
                    if not isinstance(committed, FinalizationResult):
                        raise TypeError("finalize_with_result returned an invalid result")
                    if committed.run_id != config.run_id:
                        raise ControllerError("committed manifest run_id mismatch")
                    manifest_path = committed.path
                except Exception as exc:
                    requested = "FAILED"
                    reason = f"artifact_finalization_failed:{exc}"
                    primary = primary or TerminalCause("artifact_finalization", reason)
                    manifest_path = None

                effective = requested
                if committed is not None:
                    # Returning from finalize_with_result is the publication boundary.
                    # These frozen facts are authoritative even when every subsequent
                    # observability or notification operation fails.
                    effective = committed.terminal_status
                    reason = committed.reason
                    if (
                        effective == "FAILED"
                        and lifecycle.pending_terminal is LifecycleState.COMPLETED
                    ):
                        lifecycle = lifecycle.apply(
                            LifecycleEvent.FINALIZATION_FAILED,
                            reason=reason,
                        )
                    else:
                        lifecycle = lifecycle.apply(LifecycleEvent.ARTIFACTS_FINALIZED)

                    try:
                        validated_result = store.validated_manifest_result(
                            config.run_id,
                            manifest_deadline_check,
                        )
                        if (
                            validated_result is None
                            or validated_result != committed
                        ):
                            raise ControllerError("committed manifest verification mismatch")
                    except Exception as exc:
                        diagnostics.append(
                            TerminalCause(
                                "manifest_verification",
                                "post_commit_manifest_verification_failed:"
                                f"{type(exc).__name__}",
                            )
                        )

                    try:
                        store.write_terminal_committed(
                            config.run_id,
                            {
                                "run_id": config.run_id,
                                "terminal_status": effective,
                                "reason": reason,
                                "manifest_path": "manifest.json",
                            },
                        )
                    except Exception as exc:
                        diagnostics.append(
                            TerminalCause(
                                "terminal_notification",
                                f"terminal_commit_write_failed:{type(exc).__name__}",
                            )
                        )
                    if compose_started:
                        try:
                            _notified, notify_cause = self._wait_for(
                                store,
                                config.run_id,
                                compose,
                                topology,
                                "terminal-notified",
                                manifest_deadline,
                                TerminalCause(
                                    "finalization_deadline",
                                    "terminal_notification_deadline",
                                ),
                                observe_causes=False,
                                deadline_check=manifest_deadline_check,
                            )
                            if notify_cause is not None:
                                diagnostics.append(notify_cause)
                        except Exception as exc:
                            diagnostics.append(
                                TerminalCause(
                                    "terminal_notification",
                                    f"terminal_notification_failed:{type(exc).__name__}",
                                )
                            )
                else:
                    effective = "FAILED"
                    lifecycle = replace(
                        lifecycle,
                        state=LifecycleState.FAILED,
                        pending_terminal=None,
                        reason=reason,
                    )

                later = None
                try:
                    manifest_deadline_check()
                    later = store.read_runtime_status(
                        config.run_id,
                        "runtime-failure",
                        deadline_check=manifest_deadline_check,
                    )
                    manifest_deadline_check()
                except TimeoutError:
                    diagnostics.append(
                        TerminalCause(
                            "runtime_failure_observation",
                            "post_commit_runtime_failure_deadline",
                        )
                    )
                except Exception as exc:
                    diagnostics.append(
                        TerminalCause(
                            "runtime_failure_observation",
                            f"post_commit_runtime_failure_failed:{type(exc).__name__}",
                        )
                    )
                if later is not None:
                    later_reason = later.get("reason")
                    if isinstance(later_reason, str) and later_reason:
                        later_cause = TerminalCause(
                            "runtime_failure",
                            later_reason,
                            later.get("module")
                            if isinstance(later.get("module"), str)
                            else None,
                        )
                        if later_cause != primary and later_cause not in diagnostics:
                            diagnostics.append(later_cause)
                final_status = OperatorStatus(
                    config.run_id,
                    effective,
                    reason,
                    "manifest.json" if manifest_path is not None else None,
                    primary,
                    tuple(diagnostics),
                )
                try:
                    store.write_operator_status(final_status)
                except Exception:
                    # The typed manifest result is already authoritative.  A
                    # mutable operator-cache failure cannot erase foreground
                    # terminal facts; later commands validate the manifest.
                    pass
                return RunResult(
                    config.run_id,
                    effective,
                    reason,
                    final_status.manifest_path,
                )

            lifecycle = replace(
                lifecycle,
                state=LifecycleState.FAILED,
                pending_terminal=None,
                reason=(primary.reason if primary else "compose_start_failed"),
            )
            status = self._status(lifecycle, primary=primary)
            store.write_operator_status(status)
            return RunResult(config.run_id, status.state, status.reason, None)
        finally:
            close_event_log()
            if compose_attempted and compose is not None:
                teardown_failure: str | None = None
                try:
                    # The finalization branch defines final_deadline. Startup failures
                    # retain a small bounded teardown attempt.
                    deadline = locals().get("final_deadline", self.monotonic() + 1.0)
                    down_result = compose.down(
                        self._remaining(deadline, self.monotonic)
                    )
                    if down_result.returncode != 0:
                        teardown_failure = (
                            f"compose_down_failed:{down_result.returncode}"
                        )
                except BaseException as exc:
                    teardown_failure = f"compose_down_exception:{type(exc).__name__}"
                if teardown_failure is not None:
                    try:
                        existing = store.read_operator_status(config.run_id)
                        diagnostic = TerminalCause("teardown", teardown_failure)
                        if diagnostic not in existing.diagnostics:
                            store.write_operator_status(
                                replace(
                                    existing,
                                    diagnostics=(*existing.diagnostics, diagnostic),
                                )
                            )
                    except Exception:
                        pass

    @staticmethod
    def _absolute_output_root(output_root: Path | str) -> Path:
        path = Path(output_root)
        if not path.is_absolute():
            raise ControllerError("output root must be absolute")
        return path

    def status(self, run_id: str, output_root: Path | str) -> RunResult:
        store = self.status_store_factory(self._absolute_output_root(output_root))
        committed = store.validated_manifest_result(run_id)
        if committed is not None:
            return RunResult(
                committed.run_id,
                committed.terminal_status,
                committed.reason,
                "manifest.json",
            )
        status = store.read_operator_status(run_id)
        committed = store.validated_manifest_result(run_id)
        if committed is not None:
            return RunResult(
                committed.run_id,
                committed.terminal_status,
                committed.reason,
                "manifest.json",
            )
        return RunResult(status.run_id, status.state, status.reason, status.manifest_path)

    def abort(self, run_id: str, output_root: Path | str) -> RunResult:
        store = self.status_store_factory(self._absolute_output_root(output_root))
        committed = store.validated_manifest_result(run_id)
        if committed is not None:
            return RunResult(
                committed.run_id,
                committed.terminal_status,
                committed.reason,
                "manifest.json",
            )
        store.request_finalization(run_id, "ABORTED", "operator_abort")
        committed = store.validated_manifest_result(run_id)
        if committed is not None:
            return RunResult(
                committed.run_id,
                committed.terminal_status,
                committed.reason,
                "manifest.json",
            )
        status = store.read_operator_status(run_id)
        return RunResult(status.run_id, status.state, status.reason, status.manifest_path)

    def collect_results(self, run_id: str, output_root: Path | str) -> RunResult:
        store = self.status_store_factory(self._absolute_output_root(output_root))
        committed = store.validated_manifest_result(run_id)
        if committed is None:
            raise ProtocolFileError("manifest.json is missing")
        return RunResult(
            committed.run_id,
            committed.terminal_status,
            committed.reason,
            "manifest.json",
        )


__all__ = [
    "ArtifactFinalReport",
    "ControllerError",
    "RunController",
    "RunResult",
]
