"""Operator start admission and bounded flight-recovery supervision."""

from __future__ import annotations

from dataclasses import dataclass
import hmac
import math
import time
from typing import Callable, Literal, Protocol

from artifacts.operator_protocol import (
    FlightAbortRequest,
    FlightAbortStatus,
    MissionStartRequest,
    MissionStartReadiness,
    PreflightStatus,
    StartAdmission,
)


_FRESH_WALL_NS = 2_000_000_000


def dronekit_heartbeat_is_fresh(last_heartbeat: object) -> bool:
    """Accept DroneKit's heartbeat age only within the preflight snapshot window."""
    return (
        isinstance(last_heartbeat, (int, float))
        and not isinstance(last_heartbeat, bool)
        and math.isfinite(last_heartbeat)
        and 0.0 <= last_heartbeat <= _FRESH_WALL_NS / 1_000_000_000
    )


class OperatorProtocol(Protocol):
    def write_preflight(self, status: PreflightStatus) -> object: ...
    def read_preflight(self, component: Literal["companion"]) -> PreflightStatus | None: ...
    def read_start_request(self) -> MissionStartRequest | None: ...
    def write_start_readiness(self, status: MissionStartReadiness) -> object: ...
    def read_start_admission(self) -> StartAdmission | None: ...
    def read_flight_abort_request(self) -> FlightAbortRequest | None: ...
    def write_flight_abort_status(self, status: FlightAbortStatus) -> object: ...
    def read_flight_abort_statuses(self) -> tuple[FlightAbortStatus, ...]: ...


@dataclass(frozen=True)
class StartDecision:
    command_allowed: bool
    admitted_now: bool = False
    admission_sha256: str | None = None


@dataclass(frozen=True)
class RunningStateBinding:
    """Identity carried by one observed RUNNING state transition."""

    run_id: str
    config_sha256: str
    admission_sha256: str

    def matches(
        self,
        *,
        run_id: str,
        config_sha256: str,
        admission_sha256: str,
    ) -> bool:
        return (
            hmac.compare_digest(self.run_id, run_id)
            and hmac.compare_digest(self.config_sha256, config_sha256)
            and hmac.compare_digest(self.admission_sha256, admission_sha256)
        )


def running_state_binding(
    message: object,
    *,
    running_state: int,
) -> RunningStateBinding | None:
    """Retain the identity fields needed to authorize a RUNNING transition."""
    if getattr(message, "state", None) != running_state:
        return None

    def text(field: str) -> str:
        value = getattr(message, field, "")
        return value if isinstance(value, str) else ""

    return RunningStateBinding(
        run_id=text("run_id"),
        config_sha256=text("config_sha256"),
        admission_sha256=text("admission_sha256"),
    )


class CompanionOperatorControl:
    """Bind one companion host to live preflight, admission, and recovery files."""

    def __init__(
        self,
        *,
        protocol: OperatorProtocol,
        run_id: str,
        config_sha256: str,
        launch_mode: str,
        preparation_wall_seconds: float,
        mission_wall_seconds: float,
        recovery_wall_seconds: float,
        wall_time_ns: Callable[[], int] = time.time_ns,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if launch_mode not in {"unattended", "operator"}:
            raise ValueError("launch_mode must be unattended or operator")
        self._protocol = protocol
        self._run_id = run_id
        self._config_sha256 = config_sha256
        self._launch_mode = launch_mode
        self._mission_wall_ns = int(mission_wall_seconds * 1_000_000_000)
        self._recovery_wall_ns = int(recovery_wall_seconds * 1_000_000_000)
        self._wall_time_ns = wall_time_ns
        self._monotonic_ns = monotonic_ns
        previous_preflight = (
            protocol.read_preflight("companion") if launch_mode == "operator" else None
        )
        self._preflight_sequence = (
            0 if previous_preflight is None else previous_preflight.sequence + 1
        )
        self._last_preflight_write_ns: int | None = None
        self._last_preflight_values: tuple[object | None, ...] | None = None
        existing_abort_statuses = (
            protocol.read_flight_abort_statuses()
            if launch_mode == "operator"
            else ()
        )
        self._abort_sequence = len(existing_abort_statuses)
        self._values: dict[str, object] = {}
        self._observed_at: dict[str, int] = {}
        self._native_timestamp_ns: int | None = None
        self._readiness_requests: set[str] = set()
        self._admission_sha256: str | None = None
        self._admitted_request_sha256: str | None = None
        existing_abort_request = (
            protocol.read_flight_abort_request()
            if existing_abort_statuses
            else None
        )
        self._abort_request_sha256 = (
            existing_abort_request.sha256
            if existing_abort_request is not None
            else None
        )
        self._abort_terminal = bool(
            existing_abort_statuses
            and existing_abort_statuses[-1].state
            in {"landed_disarmed", "rejected", "failed", "interrupted"}
        )
        self._recovery_started_monotonic_ns = (
            self._monotonic_ns()
            if existing_abort_statuses
            and existing_abort_statuses[-1].state == "recovering"
            else None
        )
        self._recovery_land_observed = False
        self.wall_deadline_ns = (
            self._monotonic_ns() + int(preparation_wall_seconds * 1_000_000_000)
        )

    @property
    def operator_mode(self) -> bool:
        return self._launch_mode == "operator"

    @property
    def admission_sha256(self) -> str | None:
        return self._admission_sha256

    @property
    def recovery_active(self) -> bool:
        return self._abort_request_sha256 is not None and not self._abort_terminal

    @property
    def abort_observed(self) -> bool:
        return self._abort_request_sha256 is not None

    def observe_vehicle(
        self,
        *,
        native_timestamp_ns: int | None,
        heartbeat_live: bool | None = None,
        prearm_checks_healthy: bool | None = None,
        armed: bool | None = None,
        landed: bool | None = None,
        mode: str | None = None,
    ) -> None:
        now = self._wall_time_ns()
        for name, value in (
            ("heartbeat_live", heartbeat_live),
            ("prearm_checks_healthy", prearm_checks_healthy),
            ("armed", armed),
            ("landed", landed),
            ("mode", mode),
        ):
            if value is not None:
                self._values[name] = value
                self._observed_at[name] = now
        if native_timestamp_ns is not None:
            self._native_timestamp_ns = native_timestamp_ns
        if not self.operator_mode:
            return
        values = (
            self._fresh_bool("heartbeat_live", now),
            self._fresh_bool("prearm_checks_healthy", now),
            self._fresh_bool("armed", now),
        )
        ready = values[0] is True and values[1] is True and values[2] is False
        if (
            values == self._last_preflight_values
            and self._last_preflight_write_ns is not None
            and now - self._last_preflight_write_ns < 250_000_000
        ):
            return
        self._protocol.write_preflight(
            PreflightStatus(
                run_id=self._run_id,
                config_sha256=self._config_sha256,
                component="companion",
                sequence=self._preflight_sequence,
                observed_wall_time_ns=now,
                native_timestamp_ns=self._native_timestamp_ns,
                heartbeat_live=values[0],
                prearm_checks_healthy=values[1],
                armed=values[2],
                ready=ready,
                detail=(
                    "fresh heartbeat and prearm checks confirm a disarmed vehicle"
                    if ready
                    else "companion telemetry is incomplete, stale, unhealthy, or armed"
                ),
            )
        )
        self._preflight_sequence += 1
        self._last_preflight_write_ns = now
        self._last_preflight_values = values

    def poll_start(self, *, run_state: RunningStateBinding | None) -> StartDecision:
        if not self.operator_mode:
            return StartDecision(
                command_allowed=(
                    run_state is not None
                    and run_state.matches(
                        run_id=self._run_id,
                        config_sha256=self._config_sha256,
                        admission_sha256="",
                    )
                )
            )

        now = self._wall_time_ns()
        request = self._protocol.read_start_request()
        if request is None:
            return StartDecision(False)
        request_sha256 = str(request.sha256)
        if request_sha256 not in self._readiness_requests:
            ready, detail = self._readiness(now)
            self._protocol.write_start_readiness(
                MissionStartReadiness(
                    run_id=self._run_id,
                    config_sha256=self._config_sha256,
                    request_id=request.request_id,
                    request_sha256=request_sha256,
                    observed_wall_time_ns=now,
                    heartbeat_live=(
                        self._fresh_bool("heartbeat_live", now) is True
                    ),
                    prearm_checks_healthy=(
                        self._fresh_bool("prearm_checks_healthy", now) is True
                    ),
                    armed=self._fresh_bool("armed", now),
                    ready=ready,
                    detail=detail,
                )
            )
            self._readiness_requests.add(request_sha256)

        admission = self._protocol.read_start_admission()
        if (
            admission is None
            or run_state is None
        ):
            return StartDecision(False)
        admission_sha256 = str(admission.sha256)
        if (
            admission.run_id != self._run_id
            or admission.config_sha256 != self._config_sha256
            or admission.request_id != request.request_id
            or admission.request_sha256 != request_sha256
            or not run_state.matches(
                run_id=self._run_id,
                config_sha256=self._config_sha256,
                admission_sha256=admission_sha256,
            )
        ):
            return StartDecision(False)
        admitted_now = self._admission_sha256 is None
        if admitted_now:
            self._admission_sha256 = admission_sha256
            self._admitted_request_sha256 = request_sha256
            self.wall_deadline_ns = self._monotonic_ns() + self._mission_wall_ns
        return StartDecision(
            True,
            admitted_now=admitted_now,
            admission_sha256=self._admission_sha256,
        )

    def poll_abort(
        self,
        *,
        finalizing: bool,
        begin_recovery: Callable[[str], bool],
    ) -> None:
        if not self.operator_mode:
            return
        request = self._protocol.read_flight_abort_request()
        if request is None:
            return
        request_sha256 = str(request.sha256)
        if self._abort_request_sha256 is None:
            if (
                self._admission_sha256 is None
                or request.admission_sha256 != self._admission_sha256
            ):
                self._abort_request_sha256 = request_sha256
                self._abort_terminal = True
                self._write_abort(request, "rejected", "flight abort has no matching active admission")
                return
            self._abort_request_sha256 = request_sha256
            self._write_abort(request, "accepted", request.reason)
            if finalizing:
                self._abort_terminal = True
                self._write_abort(
                    request,
                    "interrupted",
                    "simulation finalization interrupted flight recovery",
                )
                return
            now = self._wall_time_ns()
            fresh_heartbeat = self._fresh_bool("heartbeat_live", now)
            fresh_armed = self._fresh_bool("armed", now)
            fresh_landed = self._fresh_bool("landed", now)
            fresh_mode = self._fresh_value("mode", now)
            if fresh_armed is False and fresh_landed is True:
                self._abort_terminal = True
                self._write_abort(
                    request,
                    "landed_disarmed",
                    "fresh telemetry already confirms landed and disarmed",
                )
                return
            if (
                fresh_heartbeat is not True
                or fresh_armed is not True
                or not isinstance(fresh_mode, str)
                or not fresh_mode
            ):
                self._abort_terminal = True
                self._write_abort(
                    request,
                    "failed",
                    "flight recovery cannot start with missing or stale airborne telemetry",
                )
                return
            try:
                recovery_started = begin_recovery(request.reason)
            except Exception as error:
                self._abort_terminal = True
                self._write_abort(request, "failed", f"recovery command failed: {error}")
                return
            if not recovery_started:
                self._abort_terminal = True
                self._write_abort(request, "failed", "recovery command was rejected")
                return
            self._recovery_started_monotonic_ns = self._monotonic_ns()
            self._write_abort(request, "recovering", "bounded flight recovery is active")
        elif request_sha256 != self._abort_request_sha256 or self._abort_terminal:
            return

        if finalizing:
            self._abort_terminal = True
            self._write_abort(
                request,
                "interrupted",
                "simulation finalization interrupted flight recovery",
            )
            return
        now = self._wall_time_ns()
        fresh_mode = self._fresh_value("mode", now)
        if fresh_mode == "LAND":
            self._recovery_land_observed = True
        elif (
            self._recovery_land_observed
            and self._fresh_bool("armed", now) is True
            and isinstance(fresh_mode, str)
        ):
            self._abort_terminal = True
            self._write_abort(
                request,
                "failed",
                f"recovery authority was replaced by observed mode {fresh_mode}",
            )
            return
        if (
            self._fresh_bool("armed", now) is False
            and self._fresh_bool("landed", now) is True
        ):
            self._abort_terminal = True
            self._write_abort(
                request,
                "landed_disarmed",
                "fresh telemetry confirms landed and disarmed",
            )
            return
        assert self._recovery_started_monotonic_ns is not None
        if (
            self._monotonic_ns() - self._recovery_started_monotonic_ns
            >= self._recovery_wall_ns
        ):
            self._abort_terminal = True
            fresh_heartbeat = self._fresh_bool("heartbeat_live", now)
            detail = (
                "flight recovery timed out with missing or stale telemetry"
                if fresh_heartbeat is not True
                else "flight recovery timed out before landed and disarmed confirmation"
            )
            self._write_abort(request, "failed", detail)

    def fail_recovery(self, detail: str) -> None:
        """Record an asynchronous mission-owned recovery rejection once."""
        if self._abort_terminal or self._abort_request_sha256 is None:
            return
        request = self._protocol.read_flight_abort_request()
        if request is None or str(request.sha256) != self._abort_request_sha256:
            return
        self._abort_terminal = True
        self._write_abort(request, "failed", detail or "flight recovery failed")

    def _fresh_value(self, name: str, now: int) -> object | None:
        observed_at = self._observed_at.get(name)
        if observed_at is None or now < observed_at or now - observed_at > _FRESH_WALL_NS:
            return None
        return self._values.get(name)

    def _fresh_bool(self, name: str, now: int) -> bool | None:
        value = self._fresh_value(name, now)
        return value if type(value) is bool else None

    def _readiness(self, now: int) -> tuple[bool, str]:
        heartbeat = self._fresh_bool("heartbeat_live", now)
        prearm = self._fresh_bool("prearm_checks_healthy", now)
        armed = self._fresh_bool("armed", now)
        if armed is True:
            return False, "vehicle is armed"
        if heartbeat is not True or prearm is not True or armed is not False:
            return False, "vehicle telemetry is missing or stale"
        return True, "fresh heartbeat and prearm checks confirm a disarmed vehicle"

    def _write_abort(
        self,
        request: FlightAbortRequest,
        state: Literal[
            "accepted",
            "recovering",
            "landed_disarmed",
            "rejected",
            "failed",
            "interrupted",
        ],
        detail: str,
    ) -> None:
        self._protocol.write_flight_abort_status(
            FlightAbortStatus(
                run_id=self._run_id,
                config_sha256=self._config_sha256,
                request_id=request.request_id,
                admission_sha256=self._admission_sha256
                or request.admission_sha256,
                sequence=self._abort_sequence,
                state=state,
                observed_wall_time_ns=self._wall_time_ns(),
                observed_native_ns=self._native_timestamp_ns,
                detail=detail,
            )
        )
        self._abort_sequence += 1


__all__ = [
    "CompanionOperatorControl",
    "RunningStateBinding",
    "StartDecision",
    "dronekit_heartbeat_is_fresh",
    "running_state_binding",
]
