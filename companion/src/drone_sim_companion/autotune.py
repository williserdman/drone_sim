"""Roll-only ArduPilot AutoTune mission support."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Callable, Mapping
from uuid import UUID

from pymavlink import mavutil


ROLL_GAIN_PARAMETERS = (
    "ATC_ANG_RLL_P",
    "ATC_RAT_RLL_P",
    "ATC_RAT_RLL_I",
    "ATC_RAT_RLL_D",
    "ATC_ACC_R_MAX",
)


class Phase(str, Enum):
    WAIT_READY = "WAIT_READY"
    WAIT_GUIDED = "WAIT_GUIDED"
    WAIT_ARMED = "WAIT_ARMED"
    WAIT_ALTITUDE = "WAIT_ALTITUDE"
    WAIT_ALT_HOLD = "WAIT_ALT_HOLD"
    WAIT_AUTOTUNE = "WAIT_AUTOTUNE"
    TUNING = "TUNING"
    LANDING_TO_SAVE = "LANDING_TO_SAVE"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


class ActionKind(str, Enum):
    SET_PARAMETER = "SET_PARAMETER"
    SET_MODE = "SET_MODE"
    ARM = "ARM"
    TAKEOFF = "TAKEOFF"
    OVERRIDE_THROTTLE = "OVERRIDE_THROTTLE"
    SNAPSHOT_PARAMETERS = "SNAPSHOT_PARAMETERS"


@dataclass(frozen=True)
class Action:
    kind: ActionKind
    name: str = ""
    value: float | None = None

    @classmethod
    def parameter(cls, name: str, value: float) -> "Action":
        return cls(ActionKind.SET_PARAMETER, name, value)

    @classmethod
    def mode(cls, name: str) -> "Action":
        return cls(ActionKind.SET_MODE, name)

    @classmethod
    def arm(cls) -> "Action":
        return cls(ActionKind.ARM)

    @classmethod
    def takeoff(cls, altitude_m: float) -> "Action":
        return cls(ActionKind.TAKEOFF, value=altitude_m)

    @classmethod
    def override(cls, throttle_pwm: int) -> "Action":
        return cls(ActionKind.OVERRIDE_THROTTLE, value=float(throttle_pwm))

    @classmethod
    def snapshot(cls) -> "Action":
        return cls(ActionKind.SNAPSHOT_PARAMETERS)


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


@dataclass(frozen=True)
class RollAutoTuneState:
    phase: Phase
    last_timestamp_ns: int | None = None
    gains_saved: bool = False
    failure_reason: str = ""

    @classmethod
    def initial(cls) -> "RollAutoTuneState":
        return cls(Phase.WAIT_READY)


@dataclass(frozen=True)
class Transition:
    state: RollAutoTuneState
    actions: tuple[Action, ...] = ()


def _failed(state: RollAutoTuneState, stamp: int, reason: str) -> Transition:
    return Transition(
        replace(
            state,
            phase=Phase.FAILED,
            last_timestamp_ns=stamp,
            failure_reason=reason,
        )
    )


def advance(state: RollAutoTuneState, observation: Observation) -> Transition:
    """Advance the roll AutoTune policy using ordered simulation observations."""
    if state.phase in {Phase.COMPLETE, Phase.FAILED}:
        return Transition(state)
    stamp = observation.timestamp_ns
    if state.last_timestamp_ns is not None and stamp < state.last_timestamp_ns:
        return _failed(state, stamp, "simulation timestamp regressed")
    current = replace(state, last_timestamp_ns=stamp)
    if observation.status_text is not None and observation.status_text.startswith(
        "AutoTune: Failed"
    ):
        return _failed(current, stamp, observation.status_text)

    if state.phase is Phase.WAIT_READY:
        if not observation.heartbeat or observation.prearm_checks_healthy is not True:
            return Transition(current)
        return Transition(
            replace(current, phase=Phase.WAIT_GUIDED),
            (Action.mode("GUIDED"),),
        )
    if state.phase is Phase.WAIT_GUIDED:
        if observation.mode != "GUIDED":
            return Transition(current)
        return Transition(
            replace(current, phase=Phase.WAIT_ARMED),
            (
                Action.parameter("ATC_RAT_RLL_P", 0.0675),
                Action.parameter("ATC_RAT_RLL_I", 0.0675),
                Action.parameter("ATC_RAT_RLL_D", 0.0005),
                Action.parameter("AUTOTUNE_AXES", 1.0),
                Action.parameter("AUTOTUNE_AGGR", 0.05),
                Action.arm(),
            ),
        )
    if state.phase is Phase.WAIT_ARMED:
        if observation.armed is not True:
            return Transition(current)
        return Transition(
            replace(current, phase=Phase.WAIT_ALTITUDE),
            (Action.takeoff(5.0),),
        )
    if state.phase is Phase.WAIT_ALTITUDE:
        if observation.relative_altitude_m is None or observation.relative_altitude_m < 4.5:
            return Transition(current)
        return Transition(
            replace(current, phase=Phase.WAIT_ALT_HOLD),
            (Action.override(1500), Action.mode("ALT_HOLD")),
        )
    if state.phase is Phase.WAIT_ALT_HOLD:
        if observation.mode != "ALT_HOLD":
            return Transition(current)
        return Transition(
            replace(current, phase=Phase.WAIT_AUTOTUNE),
            (Action.mode("AUTOTUNE"),),
        )
    if state.phase is Phase.WAIT_AUTOTUNE:
        if observation.mode != "AUTOTUNE":
            return Transition(current)
        return Transition(replace(current, phase=Phase.TUNING))
    if state.phase is Phase.TUNING:
        if observation.status_text != "AutoTune: Success":
            return Transition(current)
        return Transition(
            replace(current, phase=Phase.LANDING_TO_SAVE),
            (Action.override(1300),),
        )
    if state.phase is Phase.LANDING_TO_SAVE:
        saved = state.gains_saved or observation.status_text == "AutoTune: Saved gains for Roll"
        current = replace(current, gains_saved=saved)
        if saved and observation.armed is False and observation.landed is True:
            return Transition(replace(current, phase=Phase.COMPLETE), (Action.snapshot(),))
        return Transition(current)
    return Transition(current)


def write_roll_gain_artifact(
    run_directory: Path,
    run_id: str,
    values: Mapping[str, float],
) -> Path:
    try:
        parsed_run_id = UUID(run_id)
    except ValueError as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if str(parsed_run_id) != run_id:
        raise ValueError("run_id must be a canonical UUID")
    if set(values) != set(ROLL_GAIN_PARAMETERS):
        raise ValueError("saved gains must contain exactly the roll AutoTune parameters")
    normalized: dict[str, float] = {}
    for name in ROLL_GAIN_PARAMETERS:
        value = values[name]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"saved gain {name} must be finite and positive")
        normalized[name] = float(value)
    if not math.isclose(
        normalized["ATC_RAT_RLL_I"],
        normalized["ATC_RAT_RLL_P"],
        rel_tol=1e-9,
        abs_tol=1e-12,
    ):
        raise ValueError("saved roll I gain must equal the roll P gain")

    target = run_directory / "ardupilot_sitl/autotune-roll.parm"
    target.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Roll gains saved by ArduPilot AutoTune",
        f"# run_id {run_id}",
        *(f"{name} {normalized[name]:g}" for name in ROLL_GAIN_PARAMETERS),
    ]
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=target.parent,
            prefix=f".{target.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write("\n".join(lines) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(0o644)
        os.replace(temporary, target)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return target


def read_saved_roll_gains_from_dataflash(
    run_directory: Path,
    *,
    connection_factory: Callable[[str], Any] = mavutil.mavlink_connection,
) -> dict[str, float]:
    """Read the latest coherent saved-gain PARM group from DataFlash."""
    logs = tuple((run_directory / "ardupilot_sitl/logs").glob("*.BIN"))
    if not logs:
        raise ValueError("ArduPilot DataFlash log is missing")
    log = max(logs, key=lambda path: path.stat().st_mtime_ns)
    connection = connection_factory(str(log))
    latest: dict[str, tuple[int, float]] = {}
    while True:
        message = connection.recv_match(type="PARM", blocking=False)
        if message is None:
            break
        row = message.to_dict()
        name = row.get("Name")
        if isinstance(name, bytes):
            name = name.decode("ascii").rstrip("\0")
        if name not in ROLL_GAIN_PARAMETERS:
            continue
        timestamp = row.get("TimeUS")
        value = row.get("Value")
        if type(timestamp) is not int or isinstance(value, bool) or not isinstance(
            value, (int, float)
        ):
            raise ValueError("saved DataFlash parameter record is invalid")
        latest[name] = (timestamp, float(value))
    if set(latest) != set(ROLL_GAIN_PARAMETERS):
        raise ValueError("saved DataFlash roll gains are incomplete")
    if len({timestamp for timestamp, _value in latest.values()}) != 1:
        raise ValueError("saved DataFlash roll gains do not come from one save epoch")
    return {name: latest[name][1] for name in ROLL_GAIN_PARAMETERS}


def execute_actions(
    vehicle: Any,
    actions: tuple[Action, ...],
    *,
    run_directory: Path,
    run_id: str,
    mode_factory: Callable[[str], Any],
    snapshot_reader: Callable[[Path], Mapping[str, float]] = (
        read_saved_roll_gains_from_dataflash
    ),
) -> None:
    for action in actions:
        if action.kind is ActionKind.SET_PARAMETER:
            assert action.value is not None
            master = vehicle._master
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                action.name.encode("ascii"),
                float(action.value),
                9,  # MAV_PARAM_TYPE_REAL32
            )
        elif action.kind is ActionKind.SET_MODE:
            vehicle.mode = mode_factory(action.name)
        elif action.kind is ActionKind.ARM:
            vehicle.armed = True
        elif action.kind is ActionKind.TAKEOFF:
            assert action.value is not None
            vehicle.simple_takeoff(action.value)
        elif action.kind is ActionKind.OVERRIDE_THROTTLE:
            assert action.value is not None
            vehicle.channels.overrides = {
                "1": 1500,
                "2": 1500,
                "3": int(action.value),
                "4": 1500,
            }
        elif action.kind is ActionKind.SNAPSHOT_PARAMETERS:
            values = snapshot_reader(run_directory)
            write_roll_gain_artifact(run_directory, run_id, values)


class RollAutoTuneDriver:
    """Apply the pure AutoTune policy to a DroneKit-compatible vehicle."""

    def __init__(
        self,
        vehicle: Any,
        *,
        run_directory: Path,
        run_id: str,
        mode_factory: Callable[[str], Any],
    ) -> None:
        self.vehicle = vehicle
        self.run_directory = run_directory
        self.run_id = run_id
        self.mode_factory = mode_factory
        self.state = RollAutoTuneState.initial()

    def _execute(self, actions: tuple[Action, ...]) -> None:
        execute_actions(
            self.vehicle,
            actions,
            run_directory=self.run_directory,
            run_id=self.run_id,
            mode_factory=self.mode_factory,
        )

    def observe(self, observation: Observation) -> Transition:
        transition = advance(self.state, observation)
        self._execute(transition.actions)
        self.state = transition.state
        return transition

    def refresh_override(self) -> None:
        if self.state.phase is Phase.WAIT_ARMED:
            self._execute((Action.arm(),))
        elif self.state.phase in {
            Phase.WAIT_ALT_HOLD,
            Phase.WAIT_AUTOTUNE,
            Phase.TUNING,
        }:
            self._execute((Action.override(1500),))
        elif self.state.phase is Phase.LANDING_TO_SAVE:
            self._execute((Action.override(1300),))


__all__ = ["ROLL_GAIN_PARAMETERS", "RollAutoTuneDriver", "write_roll_gain_artifact"]
