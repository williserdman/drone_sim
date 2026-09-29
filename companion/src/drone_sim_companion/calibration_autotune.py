"""Pure all-axis AutoTune policy and its MAVLink action adapter."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math
from pathlib import Path
from typing import Any, Callable, Mapping

from pymavlink import mavutil
from artifacts.calibration import CALIBRATION_PARAMETERS, PRESERVED_PARAMETERS


GAIN_PARAMETERS = CALIBRATION_PARAMETERS


class Phase(str, Enum):
    WAIT_READY = "WAIT_READY"
    WAIT_GUIDED = "WAIT_GUIDED"
    WAIT_ARMED = "WAIT_ARMED"
    WAIT_ALTITUDE = "WAIT_ALTITUDE"
    WAIT_PRE_TUNE_LOITER = "WAIT_PRE_TUNE_LOITER"
    WAIT_AUTOTUNE = "WAIT_AUTOTUNE"
    TUNING = "TUNING"
    WAIT_POST_TUNE_LOITER = "WAIT_POST_TUNE_LOITER"
    WAIT_GAIN_ACTIVATION = "WAIT_GAIN_ACTIVATION"
    SETTLING = "SETTLING"
    LANDING = "LANDING"
    POST_DISARM_READBACK = "POST_DISARM_READBACK"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


class ActionKind(str, Enum):
    SET_PARAMETER = "SET_PARAMETER"
    SET_MODE = "SET_MODE"
    ARM = "ARM"
    TAKEOFF = "TAKEOFF"
    NEUTRAL_OVERRIDE = "NEUTRAL_OVERRIDE"
    CLEAR_OVERRIDES = "CLEAR_OVERRIDES"
    AUX_FUNCTION = "AUX_FUNCTION"
    REQUEST_PARAMETERS = "REQUEST_PARAMETERS"
    EXPORT = "EXPORT"


@dataclass(frozen=True)
class Action:
    kind: ActionKind
    name: str = ""
    value: float | None = None
    second_value: float | None = None

    @classmethod
    def parameter(cls, name: str, value: float) -> "Action": return cls(ActionKind.SET_PARAMETER, name, value)
    @classmethod
    def mode(cls, name: str) -> "Action": return cls(ActionKind.SET_MODE, name)
    @classmethod
    def arm(cls) -> "Action": return cls(ActionKind.ARM)
    @classmethod
    def takeoff(cls, altitude_m: float) -> "Action": return cls(ActionKind.TAKEOFF, value=altitude_m)
    @classmethod
    def neutral_override(cls) -> "Action": return cls(ActionKind.NEUTRAL_OVERRIDE)
    @classmethod
    def clear_overrides(cls) -> "Action": return cls(ActionKind.CLEAR_OVERRIDES)
    @classmethod
    def aux_function(cls, function: int, position: int) -> "Action": return cls(ActionKind.AUX_FUNCTION, value=float(function), second_value=float(position))
    @classmethod
    def request_parameters(cls) -> "Action": return cls(ActionKind.REQUEST_PARAMETERS)
    @classmethod
    def export(cls) -> "Action": return cls(ActionKind.EXPORT)


@dataclass(frozen=True)
class Observation:
    timestamp_ns: int
    heartbeat: bool = False
    prearm_checks_healthy: bool | None = None
    mode: str | None = None
    armed: bool | None = None
    landed: bool | None = None
    relative_altitude_m: float | None = None
    status_text: str | None = None
    aux_ack: bool = False
    parameters: Mapping[str, float] | None = None
    parameter_generation: int = 0
    telemetry_timestamp_ns: int | None = None
    horizontal_speed_m_s: float | None = None
    vertical_speed_m_s: float | None = None
    roll_rad: float | None = None
    pitch_rad: float | None = None


@dataclass(frozen=True)
class AllAxisState:
    phase: Phase
    phase_started_ns: int
    public_deadline_ns: int
    last_timestamp_ns: int | None = None
    aux_acknowledged: bool = False
    testing_status_observed: bool = False
    saved_status_observed: bool = False
    activated_parameters: tuple[tuple[str, float], ...] = ()
    preserved_parameters: tuple[tuple[str, float], ...] = ()
    baseline_generation: int = 0
    activation_generation: int = 0
    settle_started_ns: int | None = None
    failure_reason: str = ""

    @classmethod
    def initial(cls, *, public_deadline_ns: int) -> "AllAxisState":
        return cls(Phase.WAIT_READY, 0, public_deadline_ns)


@dataclass(frozen=True)
class Transition:
    state: AllAxisState
    actions: tuple[Action, ...] = ()


def _enter(state: AllAxisState, phase: Phase, stamp: int, **changes: Any) -> AllAxisState:
    return replace(state, phase=phase, phase_started_ns=stamp, last_timestamp_ns=stamp, **changes)


def _failed(state: AllAxisState, stamp: int, reason: str) -> Transition:
    return Transition(_enter(state, Phase.FAILED, stamp, failure_reason=reason))


def _matching_profile(parameters: Mapping[str, float] | None) -> tuple[tuple[str, float], ...]:
    if parameters is None or not all(name in parameters for name in GAIN_PARAMETERS):
        return ()
    values = tuple((name, float(parameters[name])) for name in GAIN_PARAMETERS)
    if not all(math.isfinite(value) and value > 0 for _name, value in values):
        return ()
    return values


def advance(state: AllAxisState, observation: Observation) -> Transition:
    if state.phase in {Phase.COMPLETE, Phase.FAILED}:
        return Transition(state)
    stamp = observation.timestamp_ns
    if state.last_timestamp_ns is not None and stamp < state.last_timestamp_ns:
        return _failed(state, stamp, "simulation timestamp regressed")
    current = replace(state, last_timestamp_ns=stamp)
    if observation.status_text and observation.status_text.startswith("AutoTune: Failed"):
        return _failed(current, stamp, observation.status_text)
    if stamp >= state.public_deadline_ns - 60_000_000_000 and state.phase not in {Phase.LANDING}:
        return _failed(current, stamp, "AutoTune reserved landing window reached")
    wait_limit = 10_000_000_000
    if state.phase in {Phase.WAIT_GUIDED, Phase.WAIT_PRE_TUNE_LOITER, Phase.WAIT_AUTOTUNE, Phase.WAIT_POST_TUNE_LOITER, Phase.WAIT_GAIN_ACTIVATION} and stamp - state.phase_started_ns > wait_limit:
        return _failed(current, stamp, f"{state.phase.value} timed out")

    if state.phase is Phase.WAIT_READY:
        if not observation.heartbeat or observation.prearm_checks_healthy is not True:
            return Transition(current)
        return Transition(_enter(current, Phase.WAIT_GUIDED, stamp), (
            Action.parameter("ATC_RATE_FF_ENAB", 1.0), Action.parameter("AUTOTUNE_AXES", 7.0),
            Action.mode("GUIDED"), Action.request_parameters(),
        ))
    if state.phase is Phase.WAIT_GUIDED:
        profile = observation.parameters or {}
        if observation.mode != "GUIDED" or profile.get("ATC_RATE_FF_ENAB") != 1.0 or profile.get("AUTOTUNE_AXES") != 7.0 or not all(name in profile for name in (*GAIN_PARAMETERS, *PRESERVED_PARAMETERS)):
            return Transition(current)
        preserved = tuple((name, float(profile[name])) for name in PRESERVED_PARAMETERS)
        return Transition(_enter(current, Phase.WAIT_ARMED, stamp, preserved_parameters=preserved, baseline_generation=observation.parameter_generation), (Action.arm(),))
    if state.phase is Phase.WAIT_ARMED:
        if observation.mode != "GUIDED": return _failed(current, stamp, "left GUIDED before arming")
        if observation.armed is not True: return Transition(current)
        return Transition(_enter(current, Phase.WAIT_ALTITUDE, stamp), (Action.takeoff(5.0),))
    if state.phase is Phase.WAIT_ALTITUDE:
        if observation.mode != "GUIDED": return _failed(current, stamp, "left GUIDED before takeoff completed")
        if observation.relative_altitude_m is None or observation.relative_altitude_m < 4.5: return Transition(current)
        return Transition(_enter(current, Phase.WAIT_PRE_TUNE_LOITER, stamp), (Action.neutral_override(), Action.mode("LOITER")))
    if state.phase is Phase.WAIT_PRE_TUNE_LOITER:
        if observation.mode not in {"GUIDED", "LOITER"}: return _failed(current, stamp, "unexpected mode before tuning LOITER")
        if observation.mode != "LOITER": return Transition(current)
        return Transition(_enter(current, Phase.WAIT_AUTOTUNE, stamp), (Action.mode("AUTOTUNE"),))
    if state.phase is Phase.WAIT_AUTOTUNE:
        if observation.mode not in {"LOITER", "AUTOTUNE"}: return _failed(current, stamp, "unexpected mode while entering AUTOTUNE")
        if observation.mode != "AUTOTUNE": return Transition(current)
        return Transition(_enter(current, Phase.TUNING, stamp))
    if state.phase is Phase.TUNING:
        if observation.mode != "AUTOTUNE": return _failed(current, stamp, "left AUTOTUNE before success")
        if observation.status_text != "AutoTune: Success": return Transition(current)
        return Transition(_enter(current, Phase.WAIT_POST_TUNE_LOITER, stamp), (Action.mode("LOITER"),))
    if state.phase is Phase.WAIT_POST_TUNE_LOITER:
        if observation.mode not in {"AUTOTUNE", "LOITER"}: return _failed(current, stamp, "unexpected mode while leaving AUTOTUNE")
        if observation.mode != "LOITER": return Transition(current)
        return Transition(_enter(current, Phase.WAIT_GAIN_ACTIVATION, stamp), (Action.aux_function(180, 2), Action.request_parameters()))
    if state.phase is Phase.WAIT_GAIN_ACTIVATION:
        if observation.mode != "LOITER": return _failed(current, stamp, "left LOITER before gain activation")
        acknowledged = state.aux_acknowledged or observation.aux_ack
        testing = state.testing_status_observed or observation.status_text == "AutoTune: Pilot Testing gains for Roll Pitch Yaw(E)"
        profile = _matching_profile(observation.parameters)
        current = replace(current, aux_acknowledged=acknowledged, testing_status_observed=testing)
        preserved = tuple((name, float((observation.parameters or {}).get(name, math.nan))) for name in PRESERVED_PARAMETERS)
        if not (acknowledged and testing and profile and observation.parameter_generation > state.baseline_generation): return Transition(current)
        if preserved != state.preserved_parameters:
            return _failed(current, stamp, "preserved parameters changed during AutoTune")
        return Transition(_enter(current, Phase.SETTLING, stamp, activated_parameters=profile, activation_generation=observation.parameter_generation))
    if state.phase is Phase.SETTLING:
        if observation.mode != "LOITER": return _failed(current, stamp, "left LOITER while settling")
        if stamp - state.phase_started_ns > 20_000_000_000: return _failed(current, stamp, "settling timed out")
        values = (observation.horizontal_speed_m_s, observation.vertical_speed_m_s, observation.roll_rad, observation.pitch_rad)
        fresh = observation.telemetry_timestamp_ns is not None and 0 <= stamp - observation.telemetry_timestamp_ns <= 500_000_000
        stable = fresh and all(value is not None and math.isfinite(value) for value in values) and abs(observation.horizontal_speed_m_s or 0) <= .2 and abs(observation.vertical_speed_m_s or 0) <= .2 and abs(observation.roll_rad or 0) <= math.radians(5) and abs(observation.pitch_rad or 0) <= math.radians(5)
        if not stable: return Transition(replace(current, settle_started_ns=None))
        started = state.settle_started_ns if state.settle_started_ns is not None else stamp
        current = replace(current, settle_started_ns=started)
        if stamp - started < 2_000_000_000: return Transition(current)
        return Transition(_enter(current, Phase.LANDING, stamp), (Action.clear_overrides(), Action.mode("LAND")))
    if state.phase is Phase.LANDING:
        if observation.mode != "LAND": return _failed(current, stamp, "left native LAND before disarm")
        if stamp - state.phase_started_ns > 45_000_000_000: return _failed(current, stamp, "native LAND timed out")
        saved = state.saved_status_observed or observation.status_text == "AutoTune: Saved gains for Roll Pitch Yaw(E)"
        current = replace(current, saved_status_observed=saved)
        if not (saved and observation.armed is False and observation.landed is True): return Transition(current)
        return Transition(_enter(current, Phase.POST_DISARM_READBACK, stamp), (Action.request_parameters(),))
    if state.phase is Phase.POST_DISARM_READBACK:
        profile = _matching_profile(observation.parameters)
        if observation.parameter_generation <= state.activation_generation:
            return Transition(current)
        preserved = tuple((name, float((observation.parameters or {}).get(name, math.nan))) for name in PRESERVED_PARAMETERS)
        if preserved != state.preserved_parameters:
            return _failed(current, stamp, "preserved parameters changed before saved readback")
        if not profile or profile != state.activated_parameters: return _failed(current, stamp, "post-disarm gain readback does not match tested gains")
        return Transition(_enter(current, Phase.COMPLETE, stamp), (Action.export(),))
    return Transition(current)


class StatusTextAssembler:
    """Reassemble MAVLink 2 STATUSTEXT chunks by id and sequence."""
    def __init__(self) -> None:
        self._pending: dict[int, tuple[int, str]] = {}

    def push(self, message_id: int, chunk_seq: int, text: str) -> str | None:
        normalized = text.rstrip("\x00")
        if not message_id:
            return normalized.strip() or None
        if chunk_seq == 0:
            self._pending[message_id] = (1, normalized)
        else:
            expected, prior = self._pending.get(message_id, (-1, ""))
            if chunk_seq != expected:
                self._pending.pop(message_id, None)
                return None
            self._pending[message_id] = (expected + 1, prior + normalized)
        if len(text) >= 50 and "\x00" not in text:
            return None
        _next, complete = self._pending.pop(message_id)
        return complete.strip() or None


def execute_actions(vehicle: Any, actions: tuple[Action, ...], *, mode_factory: Callable[[str], Any], export: Callable[[], None]) -> None:
    master = vehicle._master
    for action in actions:
        if action.kind is ActionKind.SET_PARAMETER:
            master.mav.param_set_send(master.target_system, master.target_component, action.name.encode("ascii"), float(action.value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        elif action.kind is ActionKind.SET_MODE: vehicle.mode = mode_factory(action.name)
        elif action.kind is ActionKind.ARM: vehicle.armed = True
        elif action.kind is ActionKind.TAKEOFF: vehicle.simple_takeoff(float(action.value))
        elif action.kind is ActionKind.NEUTRAL_OVERRIDE: vehicle.channels.overrides = {str(i): 1500 for i in range(1, 5)}
        elif action.kind is ActionKind.CLEAR_OVERRIDES: vehicle.channels.overrides = {}
        elif action.kind is ActionKind.AUX_FUNCTION:
            master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION, 0, float(action.value), float(action.second_value), 0, 0, 0, 0, 0)
        elif action.kind is ActionKind.REQUEST_PARAMETERS: master.mav.param_request_list_send(master.target_system, master.target_component)
        elif action.kind is ActionKind.EXPORT: export()
