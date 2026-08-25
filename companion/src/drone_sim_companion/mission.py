"""Pure, simulation-time-driven descent mission policy."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math


class CommandKind(str, Enum):
    SET_GUIDED = "SET_GUIDED"
    ARM = "ARM"
    TAKEOFF = "TAKEOFF"
    LAND = "LAND"


class MissionPhase(str, Enum):
    WAIT_HEARTBEAT = "WAIT_HEARTBEAT"
    WAIT_GUIDED_ACK = "WAIT_GUIDED_ACK"
    WAIT_GUIDED_MODE = "WAIT_GUIDED_MODE"
    WAIT_PREARM_READY = "WAIT_PREARM_READY"
    WAIT_ARM_ACK = "WAIT_ARM_ACK"
    WAIT_ARMED = "WAIT_ARMED"
    WAIT_TAKEOFF_ACK = "WAIT_TAKEOFF_ACK"
    WAIT_ALTITUDE = "WAIT_ALTITUDE"
    WAIT_LAND_ACK = "WAIT_LAND_ACK"
    DESCENDING = "DESCENDING"
    WAIT_DISARM = "WAIT_DISARM"
    LANDED = "LANDED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class Ack:
    command: CommandKind
    accepted: bool
    result: int


@dataclass(frozen=True)
class Telemetry:
    timestamp_ns: int
    heartbeat: bool = False
    mode: str | None = None
    armed: bool | None = None
    relative_altitude_m: float | None = None
    vertical_speed_m_s: float | None = None
    landed: bool | None = None
    ack: Ack | None = None
    status_text: str | None = None
    status_severity: int | None = None
    prearm_checks_healthy: bool | None = None


@dataclass(frozen=True)
class MissionCommand:
    kind: CommandKind
    timestamp_ns: int
    altitude_m: float | None = None


@dataclass(frozen=True)
class MissionEvent:
    name: str
    timestamp_ns: int
    fields: tuple[tuple[str, object], ...] = ()


@dataclass(frozen=True)
class MissionState:
    phase: MissionPhase
    last_timestamp_ns: int | None = None
    expected_ack: CommandKind | None = None
    peak_altitude_m: float = 0.0
    descent_observed: bool = False
    landing_observed: bool = False
    failure_reason: str = ""

    @classmethod
    def initial(cls) -> "MissionState":
        return cls(phase=MissionPhase.WAIT_HEARTBEAT)


@dataclass(frozen=True)
class Transition:
    state: MissionState
    commands: tuple[MissionCommand, ...] = ()
    events: tuple[MissionEvent, ...] = ()


_ACK_PHASE = {
    MissionPhase.WAIT_GUIDED_ACK: CommandKind.SET_GUIDED,
    MissionPhase.WAIT_ARM_ACK: CommandKind.ARM,
    MissionPhase.WAIT_TAKEOFF_ACK: CommandKind.TAKEOFF,
    MissionPhase.WAIT_LAND_ACK: CommandKind.LAND,
}


def _fail(state: MissionState, stamp: int, reason: str) -> Transition:
    failed = replace(
        state,
        phase=MissionPhase.FAILED,
        last_timestamp_ns=max(stamp, state.last_timestamp_ns or 0),
        expected_ack=None,
        failure_reason=reason,
    )
    return Transition(failed, events=(MissionEvent("mission_failed", stamp, (("reason", reason),)),))


def _validate(event: Telemetry) -> str | None:
    if (
        not isinstance(event.timestamp_ns, int)
        or isinstance(event.timestamp_ns, bool)
        or event.timestamp_ns < 0
    ):
        return "invalid simulation timestamp"
    for name, value in (
        ("relative altitude", event.relative_altitude_m),
        ("vertical speed", event.vertical_speed_m_s),
    ):
        if value is not None and (not isinstance(value, (int, float)) or not math.isfinite(value)):
            return f"invalid {name}"
    if event.prearm_checks_healthy is not None and not isinstance(
        event.prearm_checks_healthy, bool
    ):
        return "invalid prearm status"
    return None


def _command(state: MissionState, event: Telemetry, kind: CommandKind, phase: MissionPhase) -> Transition:
    altitude = 1.5 if kind is CommandKind.TAKEOFF else None
    next_state = replace(
        state,
        phase=phase,
        last_timestamp_ns=event.timestamp_ns,
        expected_ack=kind,
    )
    return Transition(
        next_state,
        commands=(MissionCommand(kind, event.timestamp_ns, altitude),),
        events=(MissionEvent("command_issued", event.timestamp_ns, (("command", kind.value),)),),
    )


def advance(state: MissionState, event: Telemetry) -> Transition:
    """Apply one ordered observation without reading either host or wall time."""
    if state.phase in {MissionPhase.FAILED, MissionPhase.LANDED}:
        return Transition(state)
    invalid = _validate(event)
    if invalid is not None:
        return _fail(state, 0 if isinstance(event.timestamp_ns, bool) else max(int(event.timestamp_ns), 0), invalid)
    stamp = event.timestamp_ns
    if state.last_timestamp_ns is not None and stamp < state.last_timestamp_ns:
        return _fail(state, stamp, "simulation timestamp regressed")

    expected = _ACK_PHASE.get(state.phase)
    if event.ack is not None:
        if expected is None or event.ack.command is not expected:
            return _fail(state, stamp, "unexpected acknowledgement")
        if not event.ack.accepted:
            return _fail(
                state,
                stamp,
                f"negative acknowledgement for {expected.value}: result {event.ack.result}",
            )
        next_phase = {
            CommandKind.SET_GUIDED: MissionPhase.WAIT_GUIDED_MODE,
            CommandKind.ARM: MissionPhase.WAIT_ARMED,
            CommandKind.TAKEOFF: MissionPhase.WAIT_ALTITUDE,
            CommandKind.LAND: MissionPhase.DESCENDING,
        }[expected]
        return Transition(
            replace(state, phase=next_phase, last_timestamp_ns=stamp, expected_ack=None),
            events=(MissionEvent("command_acknowledged", stamp, (("command", expected.value),)),),
        )

    current = replace(state, last_timestamp_ns=stamp)
    if state.phase is MissionPhase.WAIT_HEARTBEAT:
        if not event.heartbeat:
            return Transition(current)
        return _command(current, event, CommandKind.SET_GUIDED, MissionPhase.WAIT_GUIDED_ACK)

    guided_established = state.phase not in {
        MissionPhase.WAIT_GUIDED_ACK,
        MissionPhase.WAIT_GUIDED_MODE,
    }
    if guided_established and event.mode is not None and event.mode not in {"GUIDED", "LAND"}:
        return _fail(current, stamp, f"mode changed inconsistently to {event.mode}")

    if state.phase is MissionPhase.WAIT_GUIDED_MODE:
        if event.mode != "GUIDED":
            return Transition(current)
        return Transition(replace(current, phase=MissionPhase.WAIT_PREARM_READY))
    if state.phase is MissionPhase.WAIT_PREARM_READY:
        if event.prearm_checks_healthy is not True:
            return Transition(current)
        return _command(current, event, CommandKind.ARM, MissionPhase.WAIT_ARM_ACK)
    if state.phase is MissionPhase.WAIT_ARMED:
        if event.mode is not None and event.mode != "GUIDED":
            return _fail(current, stamp, f"mode changed inconsistently to {event.mode}")
        if event.armed is not True:
            return Transition(current)
        return _command(current, event, CommandKind.TAKEOFF, MissionPhase.WAIT_TAKEOFF_ACK)
    if state.phase is MissionPhase.WAIT_ALTITUDE:
        altitude = event.relative_altitude_m
        peak = max(state.peak_altitude_m, altitude or 0.0)
        current = replace(current, peak_altitude_m=peak)
        if altitude is None or altitude < 1.35:
            return Transition(current)
        return _command(current, event, CommandKind.LAND, MissionPhase.WAIT_LAND_ACK)
    if state.phase is MissionPhase.DESCENDING:
        altitude = event.relative_altitude_m
        peak = max(state.peak_altitude_m, altitude or 0.0)
        descending = state.descent_observed or (
            event.vertical_speed_m_s is not None and event.vertical_speed_m_s < -0.05
        ) or (altitude is not None and peak - altitude >= 0.05)
        current = replace(current, peak_altitude_m=peak, descent_observed=descending)
        events: list[MissionEvent] = []
        if descending and not state.descent_observed:
            events.append(MissionEvent("descent_observed", stamp))
        if event.landed is True:
            if not descending:
                return _fail(current, stamp, "landing preceded descent")
            events.append(MissionEvent("touchdown_observed", stamp))
            return Transition(
                replace(current, phase=MissionPhase.WAIT_DISARM, landing_observed=True),
                events=tuple(events),
            )
        return Transition(current, events=tuple(events))
    if state.phase is MissionPhase.WAIT_DISARM:
        if event.landed is False:
            return _fail(current, stamp, "landing state became inconsistent after touchdown")
        if event.armed is not False or event.landed is not True:
            return Transition(current)
        return Transition(
            replace(current, phase=MissionPhase.LANDED),
            events=(MissionEvent("vehicle_disarmed", stamp), MissionEvent("mission_landed", stamp)),
        )
    return Transition(current)


__all__ = [
    "Ack",
    "CommandKind",
    "MissionCommand",
    "MissionEvent",
    "MissionPhase",
    "MissionState",
    "Telemetry",
    "Transition",
    "advance",
]
