"""Live Gazebo transport boundary and finalization deadline latch."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import json
import math
import os
from pathlib import Path
import subprocess
import time
from uuid import UUID, uuid4

from .model import (
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


_TOPICS = (
    "/clock",
    "/gazebo/private/camera/onboard/image",
    "/gazebo/private/camera/observer/image",
    "/gazebo/private/iris/odometry",
    "/world/phase3_foundation/model/ground_plane/link/ground_link/sensor/iris_ground_contact/contact",
)
_CONTROL = "/world/phase3_foundation/control"


class TransportError(RuntimeError):
    """Gazebo Transport discovery or world control failed."""


class GazeboReadyStatus:
    """Publish the one module-specific readiness fact without replacing evidence."""

    def __init__(self, run_directory: Path, run_id: str) -> None:
        if str(UUID(run_id)) != run_id or run_directory.name != run_id:
            raise ValueError("run directory and canonical run_id must agree")
        self._directory = Path(run_directory) / ".status"
        self._target = self._directory / "gazebo-ready.json"
        self._payload = (
            json.dumps(
                {"run_id": run_id, "ready": True},
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n"
        ).encode()

    def write_gazebo_ready(self) -> Path:
        if self._target.exists():
            if self._target.read_bytes() != self._payload:
                raise RuntimeError("gazebo-ready fact conflicts with existing evidence")
            return self._target
        temporary = self._directory / f".gazebo-ready.{uuid4().hex}.tmp"
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0),
            0o644,
        )
        try:
            os.write(descriptor, self._payload)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        try:
            os.link(temporary, self._target)
        except FileExistsError:
            if self._target.read_bytes() != self._payload:
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
        run: Callable[..., object] = subprocess.run,
    ) -> None:
        self._environment = dict(environment)
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
            raise TransportError(f"Gazebo Transport command failed: {error}") from error
        if result.returncode != 0:
            raise TransportError(
                f"Gazebo Transport command failed: {result.stderr.strip()}"
            )
        return result

    def assert_ready(self) -> None:
        topics = set(self._command(("gz", "topic", "-l")).stdout.splitlines())
        for topic in _TOPICS:
            if topic not in topics:
                raise TransportError(f"required Gazebo topic is missing: {topic}")
        services = set(self._command(("gz", "service", "-l")).stdout.splitlines())
        if _CONTROL not in services:
            raise TransportError(f"required Gazebo service is missing: {_CONTROL}")

    def _control(self, request: str) -> None:
        result = self._command(
            (
                "gz",
                "service",
                "-s",
                _CONTROL,
                "--reqtype",
                "gz.msgs.WorldControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                request,
            ),
            timeout=7.0,
        )
        if "true" not in result.stdout.lower():
            raise TransportError(f"Gazebo world control rejected request: {request}")

    def request_steps(self, count: int) -> None:
        if type(count) is not int or count <= 0:
            raise ValueError("step count must be a positive integer")
        self._control(f"pause: true, multi_step: {count}")

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
        observe: Callable[[object], None] | None = None,
    ) -> None:
        self._run_id = run_id
        self._protocol = protocol
        self._status = status
        self._transport = transport
        self._children = children
        self._server = server
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
                self._transport.set_paused(action.paused)
            elif isinstance(action, WriteSourceFinished):
                self._protocol.write_status(
                    "source-finished",
                    {
                        "run_id": self._run_id,
                        "finished": True,
                        "sim_timestamp_ns": action.sim_timestamp_ns,
                    },
                )
            elif isinstance(action, WriteRuntimeFailure):
                self._protocol.write_status(
                    "runtime-failure",
                    {
                        "run_id": self._run_id,
                        "module": "gazebo",
                        "reason": action.reason,
                        "diagnostic_paths": list(action.diagnostic_paths),
                    },
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
