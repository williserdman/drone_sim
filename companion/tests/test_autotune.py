from __future__ import annotations

import importlib
import math
from pathlib import Path
import stat
from types import SimpleNamespace

import pytest


RUN_ID = "00000000-0000-4000-8000-000000000001"


def test_roll_gain_artifact_records_only_the_saved_autotune_values(
    tmp_path: Path,
) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")

    target = autotune.write_roll_gain_artifact(
        tmp_path,
        RUN_ID,
        {
            "ATC_RAT_RLL_P": 0.041,
            "ATC_RAT_RLL_I": 0.041,
            "ATC_RAT_RLL_D": 0.0011,
            "ATC_ANG_RLL_P": 4.25,
            "ATC_ACC_R_MAX": 72000.0,
        },
    )

    assert target == tmp_path / "ardupilot_sitl/autotune-roll.parm"
    assert target.read_text(encoding="utf-8") == (
        "# Roll gains saved by ArduPilot AutoTune\n"
        f"# run_id {RUN_ID}\n"
        "ATC_ANG_RLL_P 4.25\n"
        "ATC_RAT_RLL_P 0.041\n"
        "ATC_RAT_RLL_I 0.041\n"
        "ATC_RAT_RLL_D 0.0011\n"
        "ATC_ACC_R_MAX 72000\n"
    )
    assert stat.S_IMODE(target.stat().st_mode) == 0o644


def test_roll_autotune_stays_active_through_landing_and_saved_disarm() -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    state = autotune.RollAutoTuneState.initial()

    transition = autotune.advance(
        state,
        autotune.Observation(0, heartbeat=True, prearm_checks_healthy=True),
    )
    assert [(action.kind.value, action.name, action.value) for action in transition.actions] == [
        ("SET_MODE", "GUIDED", None),
    ]

    transition = autotune.advance(
        transition.state, autotune.Observation(1, mode="GUIDED", armed=False)
    )
    assert [(action.kind.value, action.name, action.value) for action in transition.actions] == [
        ("SET_PARAMETER", "ATC_RAT_RLL_P", 0.0675),
        ("SET_PARAMETER", "ATC_RAT_RLL_I", 0.0675),
        ("SET_PARAMETER", "ATC_RAT_RLL_D", 0.0005),
        ("SET_PARAMETER", "AUTOTUNE_AXES", 1.0),
        ("SET_PARAMETER", "AUTOTUNE_AGGR", 0.05),
        ("ARM", "", None),
    ]
    transition = autotune.advance(
        transition.state, autotune.Observation(2, mode="GUIDED", armed=True)
    )
    assert transition.actions == (autotune.Action.takeoff(5.0),)
    transition = autotune.advance(
        transition.state,
        autotune.Observation(3, mode="GUIDED", armed=True, relative_altitude_m=4.5),
    )
    assert transition.actions == (
        autotune.Action.override(1500),
        autotune.Action.mode("ALT_HOLD"),
    )
    transition = autotune.advance(
        transition.state, autotune.Observation(4, mode="ALT_HOLD", armed=True)
    )
    assert transition.actions == (autotune.Action.mode("AUTOTUNE"),)
    transition = autotune.advance(
        transition.state, autotune.Observation(5, mode="AUTOTUNE", armed=True)
    )
    assert transition.state.phase is autotune.Phase.TUNING

    transition = autotune.advance(
        transition.state,
        autotune.Observation(
            6,
            mode="AUTOTUNE",
            armed=True,
            status_text="AutoTune: Success",
        ),
    )
    assert transition.state.phase is autotune.Phase.LANDING_TO_SAVE
    assert transition.actions == (autotune.Action.override(1300),)
    assert all(action != autotune.Action.mode("LAND") for action in transition.actions)

    transition = autotune.advance(
        transition.state,
        autotune.Observation(
            7,
            mode="AUTOTUNE",
            armed=True,
            status_text="AutoTune: Saved gains for Roll",
        ),
    )
    assert transition.state.phase is autotune.Phase.LANDING_TO_SAVE
    transition = autotune.advance(
        transition.state,
        autotune.Observation(8, mode="AUTOTUNE", armed=False, landed=True),
    )
    assert transition.state.phase is autotune.Phase.COMPLETE
    assert transition.actions == (autotune.Action.snapshot(),)


def test_roll_autotune_fails_on_detailed_ardupilot_failure_text() -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    state = autotune.RollAutoTuneState(autotune.Phase.TUNING)

    transition = autotune.advance(
        state,
        autotune.Observation(
            1,
            mode="AUTOTUNE",
            armed=True,
            status_text="AutoTune: Failed to level, please tune manually",
        ),
    )

    assert transition.state.phase is autotune.Phase.FAILED
    assert transition.state.failure_reason == (
        "AutoTune: Failed to level, please tune manually"
    )


def test_action_executor_writes_the_post_disarm_parameters(tmp_path: Path) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")

    class Vehicle:
        def __init__(self) -> None:
            self.parameters: dict[str, float] = {
                "ATC_ANG_RLL_P": 4.25,
                "ATC_RAT_RLL_P": 0.041,
                "ATC_RAT_RLL_I": 0.041,
                "ATC_RAT_RLL_D": 0.0011,
                "ATC_ACC_R_MAX": 72000.0,
            }
            self.mode = "STABILIZE"
            self.armed = False
            self.channels = SimpleNamespace(overrides={})
            self.takeoffs: list[float] = []
            parameters = self.parameters

            class Mav:
                def param_set_send(
                    self,
                    _system,
                    _component,
                    name,
                    value,
                    parameter_type,
                ) -> None:
                    assert parameter_type == 9
                    parameters[name.decode("ascii")] = value

            self._master = SimpleNamespace(
                target_system=1,
                target_component=1,
                mav=Mav(),
            )

        def simple_takeoff(self, altitude_m: float) -> None:
            self.takeoffs.append(altitude_m)

    vehicle = Vehicle()
    actions = (
        autotune.Action.parameter("AUTOTUNE_AXES", 1.0),
        autotune.Action.mode("GUIDED"),
        autotune.Action.arm(),
        autotune.Action.takeoff(5.0),
        autotune.Action.override(1500),
        autotune.Action.snapshot(),
    )

    autotune.execute_actions(
        vehicle,
        actions,
        run_directory=tmp_path,
        run_id=RUN_ID,
        mode_factory=lambda name: name,
        snapshot_reader=lambda _run_directory: {
            "ATC_ANG_RLL_P": 4.25,
            "ATC_RAT_RLL_P": 0.041,
            "ATC_RAT_RLL_I": 0.041,
            "ATC_RAT_RLL_D": 0.0011,
            "ATC_ACC_R_MAX": 72000.0,
        },
    )

    assert vehicle.parameters["AUTOTUNE_AXES"] == 1.0
    assert vehicle.mode == "GUIDED"
    assert vehicle.armed is True
    assert vehicle.takeoffs == [5.0]
    assert vehicle.channels.overrides == {
        "1": 1500,
        "2": 1500,
        "3": 1500,
        "4": 1500,
    }
    assert (tmp_path / "ardupilot_sitl/autotune-roll.parm").is_file()


def test_saved_gains_come_from_one_coherent_dataflash_parameter_record(
    tmp_path: Path,
) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    log = tmp_path / "ardupilot_sitl/logs/00000001.BIN"
    log.parent.mkdir(parents=True)
    log.touch()

    class Message:
        def __init__(self, time_us: int, name: str, value: float) -> None:
            self._value = {"TimeUS": time_us, "Name": name, "Value": value}

        def to_dict(self) -> dict[str, object]:
            return self._value

    rows = [
        Message(1, "ATC_ANG_RLL_P", 4.5),
        Message(1, "ATC_RAT_RLL_P", 0.0675),
        Message(1, "ATC_RAT_RLL_I", 0.0675),
        Message(1, "ATC_RAT_RLL_D", 0.0018),
        Message(1, "ATC_ACC_R_MAX", 1100.0),
        Message(2, "ATC_ANG_RLL_P", 13.197),
        Message(2, "ATC_RAT_RLL_P", 0.0477),
        Message(2, "ATC_RAT_RLL_I", 0.0477),
        Message(2, "ATC_RAT_RLL_D", 0.000375),
        Message(2, "ATC_ACC_R_MAX", 2483.7),
    ]

    class Connection:
        def recv_match(self, *, type: str, blocking: bool):
            assert (type, blocking) == ("PARM", False)
            return rows.pop(0) if rows else None

    values = autotune.read_saved_roll_gains_from_dataflash(
        tmp_path,
        connection_factory=lambda path: Connection(),
    )

    assert values == {
        "ATC_ANG_RLL_P": 13.197,
        "ATC_RAT_RLL_P": 0.0477,
        "ATC_RAT_RLL_I": 0.0477,
        "ATC_RAT_RLL_D": 0.000375,
        "ATC_ACC_R_MAX": 2483.7,
    }


def test_dataflash_snapshot_rejects_parameters_from_different_save_epochs(
    tmp_path: Path,
) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    log = tmp_path / "ardupilot_sitl/logs/00000001.BIN"
    log.parent.mkdir(parents=True)
    log.touch()
    rows = [
        SimpleNamespace(
            to_dict=lambda name=name: {"TimeUS": 1, "Name": name, "Value": 1.0}
        )
        for name in autotune.ROLL_GAIN_PARAMETERS[:-1]
    ]
    rows.append(
        SimpleNamespace(
            to_dict=lambda: {
                "TimeUS": 2,
                "Name": autotune.ROLL_GAIN_PARAMETERS[-1],
                "Value": 1.0,
            }
        )
    )

    class Connection:
        def recv_match(self, **_kwargs):
            return rows.pop(0) if rows else None

    with pytest.raises(ValueError, match="one save epoch"):
        autotune.read_saved_roll_gains_from_dataflash(
            tmp_path,
            connection_factory=lambda path: Connection(),
        )


def test_parameter_write_uses_nonblocking_mavlink_without_dronekit_cache(
    tmp_path: Path,
) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    calls = []

    class Vehicle:
        @property
        def parameters(self):
            raise AssertionError("DroneKit parameter cache must not be accessed")

    vehicle = Vehicle()
    vehicle._master = SimpleNamespace(
        target_system=7,
        target_component=1,
        mav=SimpleNamespace(
            param_set_send=lambda *args: calls.append(args),
        ),
    )

    autotune.execute_actions(
        vehicle,
        (autotune.Action.parameter("AUTOTUNE_AXES", 1.0),),
        run_directory=tmp_path,
        run_id=RUN_ID,
        mode_factory=lambda name: name,
    )

    assert calls == [(7, 1, b"AUTOTUNE_AXES", 1.0, 9)]


@pytest.mark.parametrize(
    "changes",
    [
        {"ATC_RAT_RLL_P": math.nan},
        {"ATC_RAT_PIT_P": 0.1},
        {"ATC_RAT_RLL_D": None},
    ],
)
def test_roll_gain_artifact_rejects_unpromotable_values(
    tmp_path: Path,
    changes: dict[str, float | None],
) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    values: dict[str, float] = {
        "ATC_ANG_RLL_P": 4.25,
        "ATC_RAT_RLL_P": 0.041,
        "ATC_RAT_RLL_I": 0.041,
        "ATC_RAT_RLL_D": 0.0011,
        "ATC_ACC_R_MAX": 72000.0,
    }
    for name, value in changes.items():
        if value is None:
            values.pop(name)
        else:
            values[name] = value

    with pytest.raises(ValueError):
        autotune.write_roll_gain_artifact(tmp_path, RUN_ID, values)

    assert not (tmp_path / "ardupilot_sitl/autotune-roll.parm").exists()


def test_driver_refreshes_the_throttle_override_without_leaving_autotune(
    tmp_path: Path,
) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    vehicle = SimpleNamespace(
        parameters={},
        mode="AUTOTUNE",
        armed=True,
        channels=SimpleNamespace(overrides={}),
        simple_takeoff=lambda _altitude: None,
    )
    driver = autotune.RollAutoTuneDriver(
        vehicle,
        run_directory=tmp_path,
        run_id=RUN_ID,
        mode_factory=lambda name: name,
    )
    driver.state = autotune.RollAutoTuneState(autotune.Phase.TUNING)

    driver.observe(
        autotune.Observation(
            10,
            mode="AUTOTUNE",
            armed=True,
            status_text="AutoTune: Success",
        )
    )
    vehicle.channels.overrides = {}
    driver.refresh_override()

    assert vehicle.mode == "AUTOTUNE"
    assert vehicle.channels.overrides == {
        "1": 1500,
        "2": 1500,
        "3": 1300,
        "4": 1500,
    }


def test_driver_retries_arm_while_waiting_for_transient_prearm_check(
    tmp_path: Path,
) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    vehicle = SimpleNamespace(
        parameters={},
        mode="GUIDED",
        armed=False,
        channels=SimpleNamespace(overrides={}),
        simple_takeoff=lambda _altitude: None,
    )
    driver = autotune.RollAutoTuneDriver(
        vehicle,
        run_directory=tmp_path,
        run_id=RUN_ID,
        mode_factory=lambda name: name,
    )
    driver.state = autotune.RollAutoTuneState(autotune.Phase.WAIT_ARMED)

    driver.refresh_override()

    assert vehicle.armed is True
