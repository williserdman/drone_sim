"""Live Gazebo transport boundary and finalization deadline latch."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time
from uuid import UUID, uuid4

from artifacts.runtime_status import RuntimeFailureStatus, SourceFinishedStatus
from .model import (
    ActivateOutput,
    BeginFinalization,
    PublishGazeboReady,
    RequestSteps,
    ServerStopFailed,
    ServerStopped,
    SetPaused,
    StopServer,
    WriteQuiescence,
    WriteRuntimeFailure,
    WriteSourceFinished,
)
from ..ros_adapter.topics import gazebo_topics_for_world


_FLIGHT_STATUS_SERVICE = "/model/iris/ardupilot/status"
_FLIGHT_STATUS_KEYS = frozenset(
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
class TransportError(RuntimeError):
    """Gazebo Transport discovery or world control failed."""


class TransportUnavailable(TransportError):
    """A Gazebo Transport command could not reach its endpoint."""


def _validate_flight_status(value: object) -> dict[str, bool | int]:
    if not isinstance(value, dict) or set(value) != _FLIGHT_STATUS_KEYS:
        raise TransportError("ArduPilot status has an invalid field inventory")
    if type(value["online"]) is not bool:
        raise TransportError("ArduPilot online status must be boolean")
    for key in _FLIGHT_STATUS_KEYS - {"online"}:
        if type(value[key]) is not int or value[key] < 0:
            raise TransportError(f"ArduPilot status field {key} must be nonnegative")
    return dict(value)


def _flight_status_ready(status: Mapping[str, bool | int]) -> bool:
    return bool(
        status["online"]
        and status["servo_packets_received"] >= 1
        and status["motor_updates"] >= 1
        and status["json_states_sent"] >= 1
        and status["servo_frame_gaps"] == 0
        and status["json_send_errors"] == 0
    )


class GazeboReadyStatus:
    """Publish the one module-specific readiness fact without replacing evidence."""

    def __init__(self, run_directory: Path, run_id: str) -> None:
        if str(UUID(run_id)) != run_id or run_directory.name != run_id:
            raise ValueError("run directory and canonical run_id must agree")
        self._directory = Path(run_directory) / ".status"
        self._target = self._directory / "gazebo-ready.json"
        self._run_id = run_id
        self._flight_exchange: dict[str, bool | int] | None = None
        self._payload: bytes | None = None

    def record_flight_exchange(self, status: Mapping[str, bool | int]) -> None:
        validated = _validate_flight_status(dict(status))
        if not _flight_status_ready(validated):
            raise RuntimeError("cannot record an unready ArduPilot exchange")
        if self._payload is not None:
            raise RuntimeError("gazebo-ready payload is already frozen")
        if self._flight_exchange is not None and self._flight_exchange != validated:
            raise RuntimeError("flight exchange readiness was already latched")
        self._flight_exchange = validated

    def _frozen_payload(self) -> bytes:
        if self._payload is None:
            document: dict[str, object] = {"run_id": self._run_id, "ready": True}
            if self._flight_exchange is not None:
                document["flight_exchange"] = self._flight_exchange
            self._payload = (
                json.dumps(document, sort_keys=True, separators=(",", ":")) + "\n"
            ).encode()
        return self._payload

    def write_gazebo_ready(self) -> Path:
        payload = self._frozen_payload()
        if self._target.exists():
            if self._target.read_bytes() != payload:
                raise RuntimeError("gazebo-ready fact conflicts with existing evidence")
            return self._target
        temporary = self._directory / f".gazebo-ready.{uuid4().hex}.tmp"
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0),
            0o644,
        )
        try:
            os.write(descriptor, payload)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        try:
            os.link(temporary, self._target)
        except FileExistsError:
            if self._target.read_bytes() != payload:
                raise RuntimeError("gazebo-ready fact conflicts with existing evidence")
        finally:
            temporary.unlink(missing_ok=True)
        directory_fd = os.open(self._directory, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        return self._target


class GazeboTransport:
    def __init__(
        self,
        *,
        environment: Mapping[str, str],
        world_name: str = "phase3_foundation",
        run: Callable[..., object] = subprocess.run,
    ) -> None:
        if world_name not in {
            "phase3_foundation",
            "vertical_descent",
            "competition_mission",
        }:
            raise ValueError("world_name must identify an approved local world")
        self._environment = dict(environment)
        self._flight = world_name in {"vertical_descent", "competition_mission"}
        self._topics = gazebo_topics_for_world(world_name)
        self._control_service = f"/world/{world_name}/control"
        self._stats_topic = f"/world/{world_name}/stats"
        self._run = run

    def _command(self, argv: tuple[str, ...], *, timeout: float = 5.0):
        try:
            result = self._run(
                argv,
                env=self._environment,
                shell=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise TransportUnavailable(
                f"Gazebo Transport command failed: {error}"
            ) from error
        if result.returncode != 0:
            raise TransportUnavailable(
                f"Gazebo Transport command failed: {result.stderr.strip()}"
            )
        return result

    def assert_ready(self) -> None:
        topics = set(self._command(("gz", "topic", "-l")).stdout.splitlines())
        for topic in self._topics:
            if topic not in topics:
                raise TransportError(f"required Gazebo topic is missing: {topic}")
        services = set(self._command(("gz", "service", "-l")).stdout.splitlines())
        if self._control_service not in services:
            raise TransportError(
                f"required Gazebo service is missing: {self._control_service}"
            )
        if self._flight and _FLIGHT_STATUS_SERVICE not in services:
            raise TransportError(
                f"required Gazebo service is missing: {_FLIGHT_STATUS_SERVICE}"
            )

    def flight_exchange_status(self, *, timeout: float = 2.0) -> dict[str, bool | int]:
        if not self._flight:
            raise TransportError("the passive world has no ArduPilot exchange")
        result = self._command(
            (
                "gz",
                "service",
                "-s",
                _FLIGHT_STATUS_SERVICE,
                "--reqtype",
                "gz.msgs.Empty",
                "--reptype",
                "gz.msgs.StringMsg",
                "--timeout",
                "1000",
                "--req",
                "",
            ),
            timeout=timeout,
        )
        output = result.stdout.strip()
        if not output.startswith('data: "'):
            raise TransportError("ArduPilot status service returned malformed data")
        try:
            encoded = json.loads(output.removeprefix("data: "))
            document = json.loads(encoded)
        except (TypeError, json.JSONDecodeError) as error:
            raise TransportError("ArduPilot status service returned malformed JSON") from error
        return _validate_flight_status(document)

    def flight_exchange_ready(self, *, timeout: float = 2.0) -> bool:
        return self.ready_flight_exchange(timeout=timeout) is not None

    def ready_flight_exchange(
        self, *, timeout: float = 2.0
    ) -> dict[str, bool | int] | None:
        try:
            status = self.flight_exchange_status(timeout=timeout)
        except TransportUnavailable:
            return None
        return status if _flight_status_ready(status) else None

    def _control(self, request: str) -> None:
        result = self._command(
            (
                "gz",
                "service",
                "-s",
                self._control_service,
                "--reqtype",
                "gz.msgs.WorldControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "60000",
                "--req",
                request,
            ),
            timeout=62.0,
        )
        if "true" not in result.stdout.lower():
            raise TransportError(f"Gazebo world control rejected request: {request}")

    def request_steps(self, count: int) -> None:
        if type(count) is not int or count <= 0:
            raise ValueError("step count must be a positive integer")
        self._control(f"pause: true, multi_step: {count}")

    def run_to_sim_time(self, target_ns: int) -> None:
        if type(target_ns) is not int or target_ns <= 0:
            raise ValueError("target simulation time must be a positive integer")
        seconds, nanoseconds = divmod(target_ns, 1_000_000_000)
        self._control(
            f"run_to_sim_time {{ sec: {seconds} nsec: {nanoseconds} }}"
        )

    def paused_sim_time_ns(self) -> int | None:
        result = self._command(
            ("gz", "topic", "-e", "-t", self._stats_topic, "-n", "1"),
            timeout=5.0,
        )
        output = result.stdout
        paused = re.search(r"^paused:\s*(true|false)\s*$", output, re.MULTILINE)
        sim_time = re.search(r"sim_time\s*\{(?P<body>.*?)\}", output, re.DOTALL)
        if sim_time is None:
            raise TransportError("Gazebo world statistics were malformed")
        # Gazebo's protobuf text output omits scalar fields at their default;
        # an absent boolean therefore means paused=false.
        if paused is None or paused.group(1) != "true":
            return None
        body = sim_time.group("body")
        seconds_match = re.search(r"^\s*sec:\s*(\d+)\s*$", body, re.MULTILINE)
        nanoseconds_match = re.search(r"^\s*nsec:\s*(\d+)\s*$", body, re.MULTILINE)
        seconds = int(seconds_match.group(1)) if seconds_match is not None else 0
        nanoseconds = (
            int(nanoseconds_match.group(1)) if nanoseconds_match is not None else 0
        )
        if nanoseconds >= 1_000_000_000:
            raise TransportError("Gazebo world statistics contained an invalid time")
        return seconds * 1_000_000_000 + nanoseconds

    def set_paused(self, paused: bool) -> None:
        if type(paused) is not bool:
            raise TypeError("paused must be a boolean")
        self._control(f"pause: {'true' if paused else 'false'}")


class FinalizationDeadlineLatch:
    """Convert durable finalization intent to one non-restarting absolute deadline."""

    def __init__(
        self,
        finalization_wall_seconds: int,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if type(finalization_wall_seconds) is not int or finalization_wall_seconds <= 0:
            raise ValueError("finalization wall seconds must be positive")
        self._seconds = finalization_wall_seconds
        self._monotonic = monotonic
        self._intent: dict | None = None
        self._deadline: float | None = None

    def deadline_for(self, intent: Mapping) -> float:
        document = dict(intent)
        if self._intent is not None:
            if document != self._intent:
                raise ValueError("finalization intent changed after deadline was latched")
            assert self._deadline is not None
            return self._deadline
        now = float(self._monotonic())
        if not math.isfinite(now):
            raise ValueError("monotonic clock is invalid")
        self._intent = document
        self._deadline = now + self._seconds
        return self._deadline


class ActionExecutor:
    """Apply ordered pure-model actions at the live module boundary."""

    def __init__(
        self,
        *,
        run_id: str,
        protocol,
        status,
        transport: GazeboTransport,
        children,
        server,
        activate_output: Callable[[], None],
        start_warmup: Callable[[], None] | None = None,
        observe: Callable[[object], None] | None = None,
    ) -> None:
        self._run_id = run_id
        self._protocol = protocol
        self._status = status
        self._transport = transport
        self._children = children
        self._server = server
        self._activate_output = activate_output
        self._start_warmup = start_warmup
        self._observe = observe or (lambda _action: None)

    def apply(self, actions: tuple[object, ...]) -> tuple[object, ...]:
        followups: list[object] = []
        for action in actions:
            self._observe(action)
            if isinstance(action, PublishGazeboReady):
                self._status.write_gazebo_ready()
            elif isinstance(action, RequestSteps):
                self._transport.request_steps(action.count)
            elif isinstance(action, SetPaused):
                if not action.paused and self._start_warmup is not None:
                    self._start_warmup()
                else:
                    self._transport.set_paused(action.paused)
            elif isinstance(action, ActivateOutput):
                self._activate_output()
            elif isinstance(action, WriteSourceFinished):
                self._protocol.write_status(
                    SourceFinishedStatus(self._run_id, action.sim_timestamp_ns)
                )
            elif isinstance(action, WriteRuntimeFailure):
                self._protocol.write_status(
                    RuntimeFailureStatus(
                        self._run_id,
                        "gazebo",
                        action.reason,
                        action.diagnostic_paths,
                    )
                )
            elif isinstance(action, BeginFinalization):
                continue
            elif isinstance(action, StopServer):
                stop_error: Exception | None = None
                try:
                    self._children.stop(action.deadline_monotonic)
                except Exception as error:
                    stop_error = error
                try:
                    summary = self._server.stop(action.deadline_monotonic)
                except Exception as error:
                    if stop_error is None:
                        stop_error = error
                if stop_error is not None:
                    followups.append(
                        ServerStopFailed(
                            self._run_id,
                            str(stop_error) or type(stop_error).__name__,
                            ("gazebo/server.log.partial", "gazebo/state"),
                        )
                    )
                else:
                    followups.append(ServerStopped(self._run_id, summary))
            elif isinstance(action, WriteQuiescence):
                self._protocol.write_quiescence("gazebo")
            else:
                raise TypeError(f"unsupported runtime action: {type(action).__name__}")
        return tuple(followups)
