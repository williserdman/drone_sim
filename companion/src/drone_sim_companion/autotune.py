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

from .mavlink_adapter import MavlinkAdapter


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
    WAIT_POST_TUNE_LOITER = "WAIT_POST_TUNE_LOITER"
    WAIT_GAIN_ACTIVATION = "WAIT_GAIN_ACTIVATION"
    SETTLING = "SETTLING"
    WAIT_RETURN_GUIDED = "WAIT_RETURN_GUIDED"
    RETURNING = "RETURNING"
    WAIT_LAND = "WAIT_LAND"
    LANDING_TO_SAVE = "LANDING_TO_SAVE"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


class ActionKind(str, Enum):
    SET_PARAMETER = "SET_PARAMETER"
    SET_MODE = "SET_MODE"
    ARM = "ARM"
    TAKEOFF = "TAKEOFF"
    OVERRIDE_THROTTLE = "OVERRIDE_THROTTLE"
    CLEAR_OVERRIDES = "CLEAR_OVERRIDES"
    AUX_FUNCTION = "AUX_FUNCTION"
    REQUEST_PARAMETERS = "REQUEST_PARAMETERS"
    WAYPOINT = "WAYPOINT"
    SNAPSHOT_PARAMETERS = "SNAPSHOT_PARAMETERS"


@dataclass(frozen=True)
class Action:
    kind: ActionKind
    name: str = ""
    value: float | None = None
    second_value: float | None = None
    destination: tuple[float, float, float] | None = None
    expected_parameters: tuple[tuple[str, float], ...] = ()

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
    def clear_overrides(cls) -> "Action":
        return cls(ActionKind.CLEAR_OVERRIDES)

    @classmethod
    def aux_function(cls, function: int, position: int) -> "Action":
        return cls(
            ActionKind.AUX_FUNCTION,
            value=float(function),
            second_value=float(position),
        )

    @classmethod
    def request_parameters(cls) -> "Action":
        return cls(ActionKind.REQUEST_PARAMETERS)

    @classmethod
    def waypoint(
        cls, latitude_deg: float, longitude_deg: float, altitude_m: float
    ) -> "Action":
        return cls(
            ActionKind.WAYPOINT,
            destination=(latitude_deg, longitude_deg, altitude_m),
        )

    @classmethod
    def snapshot(cls, expected: Mapping[str, float]) -> "Action":
        return cls(
            ActionKind.SNAPSHOT_PARAMETERS,
            expected_parameters=tuple(
                (name, float(expected[name])) for name in ROLL_GAIN_PARAMETERS
            ),
        )


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
    latitude_deg: float | None = None
    longitude_deg: float | None = None
    position_timestamp_ns: int | None = None


@dataclass(frozen=True)
class RollAutoTuneState:
    phase: Phase
    phase_started_ns: int = 0
    public_deadline_ns: int = 120_000_000_000
    last_timestamp_ns: int | None = None
    gains_saved: bool = False
    aux_acknowledged: bool = False
    testing_status_observed: bool = False
    activated_parameters: tuple[tuple[str, float], ...] = ()
    settle_started_ns: int | None = None
    landing_started_ns: int | None = None
    home_latitude_deg: float | None = None
    home_longitude_deg: float | None = None
    failure_reason: str = ""

    @classmethod
    def initial(
        cls, *, public_deadline_ns: int = 120_000_000_000
    ) -> "RollAutoTuneState":
        return cls(Phase.WAIT_READY, public_deadline_ns=public_deadline_ns)


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


def _enter(
    state: RollAutoTuneState, phase: Phase, stamp: int, **changes: Any
) -> RollAutoTuneState:
    return replace(
        state,
        phase=phase,
        phase_started_ns=stamp,
        last_timestamp_ns=stamp,
        **changes,
    )


def _matching_profile(
    parameters: Mapping[str, float] | None,
) -> tuple[tuple[str, float], ...]:
    if parameters is None or not all(name in parameters for name in ROLL_GAIN_PARAMETERS):
        return ()
    values = tuple((name, float(parameters[name])) for name in ROLL_GAIN_PARAMETERS)
    if not all(math.isfinite(value) and value > 0 for _name, value in values):
        return ()
    return values


def _fresh_position(observation: Observation) -> bool:
    values = (
        observation.latitude_deg,
        observation.longitude_deg,
        observation.relative_altitude_m,
    )
    return bool(
        observation.position_timestamp_ns is not None
        and 0
        <= observation.timestamp_ns - observation.position_timestamp_ns
        <= 500_000_000
        and all(
            type(value) in (int, float) and math.isfinite(value) for value in values
        )
        and -90 <= observation.latitude_deg <= 90
        and -180 <= observation.longitude_deg <= 180
    )


def _stable_sample(observation: Observation) -> bool:
    values = (
        observation.horizontal_speed_m_s,
        observation.vertical_speed_m_s,
        observation.roll_rad,
        observation.pitch_rad,
    )
    return bool(
        observation.telemetry_timestamp_ns is not None
        and 0
        <= observation.timestamp_ns - observation.telemetry_timestamp_ns
        <= 500_000_000
        and all(
            value is not None and math.isfinite(value) for value in values
        )
        and abs(observation.horizontal_speed_m_s or 0) <= 0.2
        and abs(observation.vertical_speed_m_s or 0) <= 0.2
        and abs(observation.roll_rad or 0) <= math.radians(5)
        and abs(observation.pitch_rad or 0) <= math.radians(5)
    )


def _arrived_home(state: RollAutoTuneState, observation: Observation) -> bool:
    if not _fresh_position(observation) or not _stable_sample(observation):
        return False
    if state.home_latitude_deg is None or state.home_longitude_deg is None:
        return False
    north = (
        math.radians(observation.latitude_deg - state.home_latitude_deg) * 6_371_000
    )
    longitude_delta = (
        observation.longitude_deg - state.home_longitude_deg + 180
    ) % 360 - 180
    east = (
        math.radians(longitude_delta)
        * 6_371_000
        * math.cos(math.radians(state.home_latitude_deg))
    )
    return (
        math.hypot(north, east) <= 0.5
        and abs(observation.relative_altitude_m - 5.0) <= 0.5
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
    if stamp >= state.public_deadline_ns:
        return _failed(current, stamp, "roll AutoTune public deadline reached")
    if state.phase in {
        Phase.WAIT_ALTITUDE,
        Phase.WAIT_ALT_HOLD,
        Phase.WAIT_AUTOTUNE,
        Phase.TUNING,
        Phase.WAIT_POST_TUNE_LOITER,
        Phase.WAIT_GAIN_ACTIVATION,
        Phase.SETTLING,
        Phase.WAIT_RETURN_GUIDED,
        Phase.RETURNING,
        Phase.WAIT_LAND,
    } and observation.armed is False:
        return _failed(current, stamp, "vehicle disarmed before native LAND")
    if state.phase in {
        Phase.WAIT_GUIDED,
        Phase.WAIT_ALT_HOLD,
        Phase.WAIT_AUTOTUNE,
        Phase.WAIT_POST_TUNE_LOITER,
        Phase.WAIT_GAIN_ACTIVATION,
        Phase.WAIT_RETURN_GUIDED,
        Phase.WAIT_LAND,
    } and stamp - state.phase_started_ns > 10_000_000_000:
        return _failed(current, stamp, f"{state.phase.value} timed out")

    if state.phase is Phase.WAIT_READY:
        if not observation.heartbeat or observation.prearm_checks_healthy is not True:
            return Transition(current)
        return Transition(
            _enter(current, Phase.WAIT_GUIDED, stamp),
            (Action.mode("GUIDED"),),
        )
    if state.phase is Phase.WAIT_GUIDED:
        if (
            observation.mode != "GUIDED"
            or observation.armed is not False
            or not _fresh_position(observation)
            or abs(observation.relative_altitude_m) > 0.3
        ):
            return Transition(current)
        return Transition(
            _enter(
                current,
                Phase.WAIT_ARMED,
                stamp,
                home_latitude_deg=observation.latitude_deg,
                home_longitude_deg=observation.longitude_deg,
            ),
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
            _enter(current, Phase.WAIT_ALTITUDE, stamp),
            (Action.takeoff(5.0),),
        )
    if state.phase is Phase.WAIT_ALTITUDE:
        if observation.relative_altitude_m is None or observation.relative_altitude_m < 4.5:
            return Transition(current)
        return Transition(
            _enter(current, Phase.WAIT_ALT_HOLD, stamp),
            (Action.override(1500), Action.mode("ALT_HOLD")),
        )
    if state.phase is Phase.WAIT_ALT_HOLD:
        if observation.mode != "ALT_HOLD":
            return Transition(current)
        return Transition(
            _enter(current, Phase.WAIT_AUTOTUNE, stamp),
            (Action.mode("AUTOTUNE"),),
        )
    if state.phase is Phase.WAIT_AUTOTUNE:
        if observation.mode != "AUTOTUNE":
            return Transition(current)
        return Transition(_enter(current, Phase.TUNING, stamp))
    if state.phase is Phase.TUNING:
        if observation.mode != "AUTOTUNE":
            return _failed(current, stamp, "left AUTOTUNE before success")
        if observation.status_text != "AutoTune: Success":
            return Transition(current)
        return Transition(
            _enter(current, Phase.WAIT_POST_TUNE_LOITER, stamp),
            (Action.mode("LOITER"),),
        )
    if state.phase is Phase.WAIT_POST_TUNE_LOITER:
        if observation.mode not in {"AUTOTUNE", "LOITER"}:
            return _failed(current, stamp, "unexpected mode while leaving AUTOTUNE")
        if observation.mode != "LOITER":
            return Transition(current)
        return Transition(
            _enter(current, Phase.WAIT_GAIN_ACTIVATION, stamp),
            (Action.aux_function(180, 2), Action.request_parameters()),
        )
    if state.phase is Phase.WAIT_GAIN_ACTIVATION:
        if observation.mode != "LOITER":
            return _failed(current, stamp, "left LOITER before gain activation")
        acknowledged = state.aux_acknowledged or observation.aux_ack
        testing = (
            state.testing_status_observed
            or observation.status_text == "AutoTune: Pilot Testing gains for Roll"
        )
        profile = _matching_profile(observation.parameters)
        current = replace(
            current,
            aux_acknowledged=acknowledged,
            testing_status_observed=testing,
        )
        if not (
            acknowledged
            and testing
            and profile
            and observation.parameter_generation > 0
        ):
            return Transition(current)
        return Transition(
            _enter(
                current,
                Phase.SETTLING,
                stamp,
                activated_parameters=profile,
                settle_started_ns=None,
            )
        )
    if state.phase is Phase.SETTLING:
        if observation.mode != "LOITER":
            return _failed(current, stamp, "left LOITER while settling")
        if stamp - state.phase_started_ns > 20_000_000_000:
            return _failed(current, stamp, "settling timed out")
        if not _stable_sample(observation):
            return Transition(replace(current, settle_started_ns=None))
        started = state.settle_started_ns if state.settle_started_ns is not None else stamp
        current = replace(current, settle_started_ns=started)
        if stamp - started < 2_000_000_000:
            return Transition(current)
        return Transition(
            _enter(current, Phase.WAIT_RETURN_GUIDED, stamp, settle_started_ns=None),
            (Action.clear_overrides(), Action.mode("GUIDED")),
        )
    if state.phase is Phase.WAIT_RETURN_GUIDED:
        if observation.mode not in {"LOITER", "GUIDED"}:
            return _failed(current, stamp, "unexpected mode while entering return GUIDED")
        if observation.mode != "GUIDED":
            return Transition(current)
        if state.home_latitude_deg is None or state.home_longitude_deg is None:
            return _failed(current, stamp, "return position was not captured before arming")
        return Transition(
            _enter(current, Phase.RETURNING, stamp),
            (
                Action.waypoint(
                    state.home_latitude_deg, state.home_longitude_deg, 5.0
                ),
            ),
        )
    if state.phase is Phase.RETURNING:
        if observation.mode != "GUIDED":
            return _failed(current, stamp, "left GUIDED while returning to the landing zone")
        if stamp - state.phase_started_ns > 60_000_000_000:
            return _failed(current, stamp, "return to landing zone timed out")
        if not _arrived_home(state, observation):
            return Transition(replace(current, settle_started_ns=None))
        started = state.settle_started_ns if state.settle_started_ns is not None else stamp
        current = replace(current, settle_started_ns=started)
        if stamp - started < 2_000_000_000:
            return Transition(current)
        return Transition(
            _enter(current, Phase.WAIT_LAND, stamp, landing_started_ns=stamp),
            (Action.clear_overrides(), Action.mode("LAND")),
        )
    if state.phase is Phase.WAIT_LAND:
        if observation.mode not in {"GUIDED", "LAND"}:
            return _failed(current, stamp, "unexpected mode while entering native LAND")
        if observation.mode != "LAND":
            return Transition(current)
        return Transition(_enter(current, Phase.LANDING_TO_SAVE, stamp))
    if state.phase is Phase.LANDING_TO_SAVE:
        if observation.mode != "LAND":
            return _failed(current, stamp, "left native LAND before disarm")
        landing_started = (
            state.landing_started_ns
            if state.landing_started_ns is not None
            else state.phase_started_ns
        )
        if stamp - landing_started > 45_000_000_000:
            return _failed(current, stamp, "native LAND timed out")
        saved = state.gains_saved or observation.status_text == "AutoTune: Saved gains for Roll"
        current = replace(current, gains_saved=saved)
        if saved and observation.armed is False and observation.landed is True:
            return Transition(
                _enter(current, Phase.COMPLETE, stamp),
                (Action.snapshot(dict(state.activated_parameters)),),
            )
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
    request_parameters: Callable[[], None] | None = None,
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
        elif action.kind is ActionKind.CLEAR_OVERRIDES:
            vehicle.channels.overrides = {}
        elif action.kind is ActionKind.AUX_FUNCTION:
            master = vehicle._master
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION,
                0,
                float(action.value),
                float(action.second_value),
                0,
                0,
                0,
                0,
                0,
            )
        elif action.kind is ActionKind.REQUEST_PARAMETERS:
            if request_parameters is not None:
                request_parameters()
            else:
                master = vehicle._master
                master.mav.param_request_list_send(
                    master.target_system, master.target_component
                )
        elif action.kind is ActionKind.WAYPOINT:
            if action.destination is None:
                raise ValueError("waypoint requires a destination")
            MavlinkAdapter(vehicle._master, mavutil).send_waypoint(*action.destination)
        elif action.kind is ActionKind.SNAPSHOT_PARAMETERS:
            values = snapshot_reader(run_directory)
            expected = dict(action.expected_parameters)
            if set(values) != set(expected) or any(
                not math.isclose(
                    float(values[name]), expected[name], rel_tol=1e-5, abs_tol=1e-7
                )
                for name in values
            ):
                raise ValueError(
                    "saved DataFlash roll gains do not match activated gains"
                )
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
        public_deadline_ns: int = 120_000_000_000,
        request_parameters: Callable[[], None] | None = None,
    ) -> None:
        self.vehicle = vehicle
        self.run_directory = run_directory
        self.run_id = run_id
        self.mode_factory = mode_factory
        self.request_parameters = request_parameters
        self.state = RollAutoTuneState.initial(public_deadline_ns=public_deadline_ns)

    def _execute(self, actions: tuple[Action, ...]) -> None:
        execute_actions(
            self.vehicle,
            actions,
            run_directory=self.run_directory,
            run_id=self.run_id,
            mode_factory=self.mode_factory,
            request_parameters=self.request_parameters,
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
            Phase.WAIT_POST_TUNE_LOITER,
            Phase.WAIT_GAIN_ACTIVATION,
            Phase.SETTLING,
        }:
            self._execute((Action.override(1500),))


__all__ = ["ROLL_GAIN_PARAMETERS", "RollAutoTuneDriver", "write_roll_gain_artifact"]
