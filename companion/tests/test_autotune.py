from __future__ import annotations

import importlib
import math
from pathlib import Path
import stat
from types import SimpleNamespace

import pytest
import drone_sim_companion.runtime_node as runtime_node


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


def test_roll_autotune_activates_returns_and_uses_native_land_before_export() -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    state = autotune.RollAutoTuneState.initial(public_deadline_ns=120_000_000_000)

    def observe(stamp: int, **changes):
        nonlocal state
        transition = autotune.advance(
            state, autotune.Observation(timestamp_ns=stamp, **changes)
        )
        state = transition.state
        return transition

    transition = observe(0, heartbeat=True, prearm_checks_healthy=True)
    assert [(action.kind.value, action.name, action.value) for action in transition.actions] == [
        ("SET_MODE", "GUIDED", None),
    ]

    transition = observe(
        1,
        mode="GUIDED",
        armed=False,
        relative_altitude_m=0.0,
        latitude_deg=37.4,
        longitude_deg=-122.08,
        position_timestamp_ns=1,
    )
    assert [(action.kind.value, action.name, action.value) for action in transition.actions] == [
        ("SET_PARAMETER", "ATC_RAT_RLL_P", 0.0675),
        ("SET_PARAMETER", "ATC_RAT_RLL_I", 0.0675),
        ("SET_PARAMETER", "ATC_RAT_RLL_D", 0.0005),
        ("SET_PARAMETER", "AUTOTUNE_AXES", 1.0),
        ("SET_PARAMETER", "AUTOTUNE_AGGR", 0.05),
        ("ARM", "", None),
    ]
    transition = observe(2, mode="GUIDED", armed=True)
    assert transition.actions == (autotune.Action.takeoff(5.0),)
    transition = observe(3, mode="GUIDED", armed=True, relative_altitude_m=4.5)
    assert transition.actions == (
        autotune.Action.override(1500),
        autotune.Action.mode("ALT_HOLD"),
    )
    transition = observe(4, mode="ALT_HOLD", armed=True)
    assert transition.actions == (autotune.Action.mode("AUTOTUNE"),)
    transition = observe(5, mode="AUTOTUNE", armed=True)
    assert transition.state.phase is autotune.Phase.TUNING

    transition = observe(
        6,
        mode="AUTOTUNE",
        armed=True,
        status_text="AutoTune: Success",
    )
    assert transition.state.phase is autotune.Phase.WAIT_POST_TUNE_LOITER
    assert transition.actions == (autotune.Action.mode("LOITER"),)
    assert all(action != autotune.Action.mode("LAND") for action in transition.actions)

    transition = observe(7, mode="LOITER", armed=True)
    assert transition.actions == (
        autotune.Action.aux_function(180, 2),
        autotune.Action.request_parameters(),
    )
    tuned = {
        "ATC_ANG_RLL_P": 13.197,
        "ATC_RAT_RLL_P": 0.0477,
        "ATC_RAT_RLL_I": 0.0477,
        "ATC_RAT_RLL_D": 0.000375,
        "ATC_ACC_R_MAX": 2483.7,
    }
    transition = observe(8, mode="LOITER", armed=True, aux_ack=True)
    assert transition.state.phase is autotune.Phase.WAIT_GAIN_ACTIVATION
    transition = observe(
        9,
        mode="LOITER",
        armed=True,
        status_text="AutoTune: Pilot Testing gains for Roll",
        parameters=tuned,
        parameter_generation=1,
    )
    assert transition.state.phase is autotune.Phase.SETTLING
    stable = dict(
        mode="LOITER",
        armed=True,
        horizontal_speed_m_s=0.1,
        vertical_speed_m_s=0.1,
        roll_rad=0.01,
        pitch_rad=-0.01,
    )
    observe(9_000_000_000, telemetry_timestamp_ns=9_000_000_000, **stable)
    transition = observe(
        11_000_000_000,
        telemetry_timestamp_ns=11_000_000_000,
        **stable,
    )
    assert transition.actions == (
        autotune.Action.clear_overrides(),
        autotune.Action.mode("GUIDED"),
    )
    transition = observe(12_000_000_000, mode="GUIDED", armed=True)
    assert transition.actions == (autotune.Action.waypoint(37.4, -122.08, 5.0),)
    arrived = dict(
        mode="GUIDED",
        armed=True,
        horizontal_speed_m_s=0.1,
        vertical_speed_m_s=0.1,
        roll_rad=0.01,
        pitch_rad=-0.01,
        latitude_deg=37.4,
        longitude_deg=-122.08,
        relative_altitude_m=5.0,
    )
    observe(
        13_000_000_000,
        telemetry_timestamp_ns=13_000_000_000,
        position_timestamp_ns=13_000_000_000,
        **arrived,
    )
    transition = observe(
        15_000_000_000,
        telemetry_timestamp_ns=15_000_000_000,
        position_timestamp_ns=15_000_000_000,
        **arrived,
    )
    assert transition.actions == (
        autotune.Action.clear_overrides(),
        autotune.Action.mode("LAND"),
    )
    observe(16_000_000_000, mode="LAND", armed=True)
    observe(
        17_000_000_000,
        mode="LAND",
        armed=True,
        status_text="AutoTune: Saved gains for Roll",
    )
    transition = observe(18_000_000_000, mode="LAND", armed=False, landed=True)
    assert transition.state.phase is autotune.Phase.COMPLETE
    assert transition.actions == (autotune.Action.snapshot(tuned),)


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
        autotune.Action.snapshot(vehicle.parameters),
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


def test_roll_action_executor_activates_clears_returns_and_requests_parameters(
    tmp_path: Path,
) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    command_calls: list[tuple[object, ...]] = []
    waypoint_calls: list[tuple[object, ...]] = []
    requests: list[bool] = []
    vehicle = SimpleNamespace(
        channels=SimpleNamespace(overrides={"3": 1500}),
        _master=SimpleNamespace(
            target_system=7,
            target_component=1,
            mav=SimpleNamespace(
                command_long_send=lambda *args: command_calls.append(args),
                set_position_target_global_int_send=lambda *args: waypoint_calls.append(
                    args
                ),
            ),
        ),
    )

    autotune.execute_actions(
        vehicle,
        (
            autotune.Action.clear_overrides(),
            autotune.Action.aux_function(180, 2),
            autotune.Action.request_parameters(),
            autotune.Action.waypoint(37.4, -122.08, 5.0),
        ),
        run_directory=tmp_path,
        run_id=RUN_ID,
        mode_factory=lambda name: name,
        request_parameters=lambda: requests.append(True),
    )

    assert vehicle.channels.overrides == {}
    assert command_calls[0][0:2] == (7, 1)
    assert command_calls[0][4:6] == (180.0, 2.0)
    assert requests == [True]
    assert waypoint_calls[0][5:8] == (374_000_000, -1_220_800_000, 5.0)


def test_dataflash_snapshot_rejects_gains_that_do_not_match_activated_profile(
    tmp_path: Path,
) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    activated = {
        "ATC_ANG_RLL_P": 13.197,
        "ATC_RAT_RLL_P": 0.0477,
        "ATC_RAT_RLL_I": 0.0477,
        "ATC_RAT_RLL_D": 0.000375,
        "ATC_ACC_R_MAX": 2483.7,
    }
    saved = {**activated, "ATC_RAT_RLL_D": 0.0005}

    with pytest.raises(ValueError, match="do not match activated gains"):
        autotune.execute_actions(
            SimpleNamespace(),
            (autotune.Action.snapshot(activated),),
            run_directory=tmp_path,
            run_id=RUN_ID,
            mode_factory=lambda name: name,
            snapshot_reader=lambda _path: saved,
        )

    assert not (tmp_path / "ardupilot_sitl/autotune-roll.parm").exists()


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


def test_driver_keeps_neutral_override_through_post_tune_activation(
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

    assert vehicle.mode == "LOITER"
    assert vehicle.channels.overrides == {
        "1": 1500,
        "2": 1500,
        "3": 1500,
        "4": 1500,
    }


@pytest.mark.parametrize(
    "changes",
    [
        {"aux_ack": False},
        {"status_text": "AutoTune: Pilot Testing gains for Roll Pitch Yaw(E)"},
        {"parameter_generation": 0},
        {"parameters": {"ATC_RAT_RLL_P": 0.0477}},
    ],
)
def test_roll_gain_activation_requires_ack_exact_status_and_fresh_complete_profile(
    changes,
) -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    tuned = {
        "ATC_ANG_RLL_P": 13.197,
        "ATC_RAT_RLL_P": 0.0477,
        "ATC_RAT_RLL_I": 0.0477,
        "ATC_RAT_RLL_D": 0.000375,
        "ATC_ACC_R_MAX": 2483.7,
    }
    observation = {
        "mode": "LOITER",
        "armed": True,
        "aux_ack": True,
        "status_text": "AutoTune: Pilot Testing gains for Roll",
        "parameters": tuned,
        "parameter_generation": 1,
    }
    observation.update(changes)

    transition = autotune.advance(
        autotune.RollAutoTuneState(
            autotune.Phase.WAIT_GAIN_ACTIVATION,
            public_deadline_ns=120_000_000_000,
        ),
        autotune.Observation(1, **observation),
    )

    assert transition.state.phase is autotune.Phase.WAIT_GAIN_ACTIVATION
    assert transition.actions == ()


def test_roll_return_requires_fresh_stable_sample_at_launch_point() -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    state = autotune.RollAutoTuneState(
        autotune.Phase.RETURNING,
        public_deadline_ns=120_000_000_000,
        home_latitude_deg=37.4,
        home_longitude_deg=-122.08,
    )
    observation = autotune.Observation(
        2_000_000_000,
        mode="GUIDED",
        armed=True,
        relative_altitude_m=5.0,
        latitude_deg=37.4,
        longitude_deg=-122.08,
        position_timestamp_ns=2_000_000_000,
        telemetry_timestamp_ns=1_000_000_000,
        horizontal_speed_m_s=0.1,
        vertical_speed_m_s=0.1,
        roll_rad=0.01,
        pitch_rad=-0.01,
    )

    transition = autotune.advance(state, observation)

    assert transition.state.phase is autotune.Phase.RETURNING
    assert transition.state.settle_started_ns is None
    assert transition.actions == ()


def test_roll_autotune_deadline_is_the_original_120_seconds() -> None:
    autotune = importlib.import_module("drone_sim_companion.autotune")
    state = autotune.RollAutoTuneState(
        autotune.Phase.RETURNING,
        public_deadline_ns=120_000_000_000,
        home_latitude_deg=37.4,
        home_longitude_deg=-122.08,
    )

    transition = autotune.advance(
        state,
        autotune.Observation(120_000_000_000, mode="GUIDED", armed=True),
    )

    assert transition.state.phase is autotune.Phase.FAILED
    assert transition.state.failure_reason == "roll AutoTune public deadline reached"


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


def test_all_axis_autotune_returns_to_launch_then_lands_and_exports_saved_gains() -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    state = calibration.AllAxisState.initial(public_deadline_ns=600_000_000_000)

    def observe(stamp: int, **changes):
        nonlocal state
        transition = calibration.advance(
            state, calibration.Observation(timestamp_ns=stamp, **changes)
        )
        state = transition.state
        return transition

    assert observe(0, heartbeat=True, prearm_checks_healthy=True).actions == (
        calibration.Action.parameter("AUTOTUNE_AXES", 7.0),
        calibration.Action.mode("GUIDED"),
        calibration.Action.request_parameters(),
    )
    baseline = {name: float(index + 1) for index, name in enumerate((*calibration.GAIN_PARAMETERS, *calibration.PRESERVED_PARAMETERS))}
    baseline["ATC_RATE_FF_ENAB"] = 1.0
    assert observe(
        1, mode="GUIDED", armed=False,
        parameters={**baseline, "AUTOTUNE_AXES": 7.0},
        latitude_deg=37.4, longitude_deg=-122.08,
        relative_altitude_m=0.0, position_timestamp_ns=1,
    ).actions == (calibration.Action.arm(),)
    assert observe(2, mode="GUIDED", armed=True).actions == (
        calibration.Action.takeoff(5.0),
    )
    assert observe(3, mode="GUIDED", armed=True, relative_altitude_m=4.5).actions == (
        calibration.Action.neutral_override(), calibration.Action.mode("LOITER")
    )
    stable = dict(
        mode="LOITER", armed=True, horizontal_speed_m_s=0.1,
        vertical_speed_m_s=0.1, roll_rad=0.01, pitch_rad=-0.01,
    )
    assert observe(
        4_000_000_000, telemetry_timestamp_ns=4_000_000_000, **stable
    ).actions == ()
    assert observe(
        6_000_000_000, telemetry_timestamp_ns=6_000_000_000, **stable
    ).actions == (calibration.Action.mode("AUTOTUNE"),)
    observe(7_000_000_000, mode="AUTOTUNE", armed=True)
    assert observe(8_000_000_000, mode="AUTOTUNE", armed=True, status_text="AutoTune: Success").actions == (
        calibration.Action.mode("LOITER"),
    )
    assert observe(9_000_000_000, mode="LOITER", armed=True).actions == (
        calibration.Action.aux_function(180, 2), calibration.Action.request_parameters()
    )
    # ACK alone cannot activate or land.
    assert observe(9_100_000_000, mode="LOITER", armed=True, aux_ack=True).actions == ()
    tuned = {name: float(index + 1) for index, name in enumerate(calibration.GAIN_PARAMETERS)}
    transition = observe(
        9_200_000_000, mode="LOITER", armed=True, aux_ack=True,
        status_text="AutoTune: Pilot Testing gains for Roll Pitch Yaw(E)",
        parameters={**baseline, **tuned}, parameter_generation=1,
    )
    assert transition.state.phase is calibration.Phase.SETTLING
    stable = dict(
        mode="LOITER", armed=True, horizontal_speed_m_s=0.1,
        vertical_speed_m_s=0.1, roll_rad=0.01, pitch_rad=-0.01,
    )
    observe(10_000_000_000, telemetry_timestamp_ns=10_000_000_000, **stable)
    assert observe(
        12_000_000_000, telemetry_timestamp_ns=12_000_000_000, **stable
    ).actions == (calibration.Action.clear_overrides(), calibration.Action.mode("GUIDED"))
    transition = observe(12_100_000_000, mode="LOITER", armed=True)
    assert transition.state.phase is calibration.Phase.WAIT_RETURN_GUIDED
    transition = observe(12_200_000_000, mode="GUIDED", armed=True)
    assert transition.state.phase is calibration.Phase.RETURNING
    assert transition.actions == (calibration.Action.waypoint(37.4, -122.08, 5.0),)
    arrived = dict(
        **{**stable, "mode": "GUIDED"},
        latitude_deg=37.4, longitude_deg=-122.08, relative_altitude_m=5.0,
    )
    assert observe(
        13_000_000_000, telemetry_timestamp_ns=13_000_000_000,
        position_timestamp_ns=13_000_000_000, **arrived,
    ).actions == ()
    assert observe(
        15_000_000_000, telemetry_timestamp_ns=15_000_000_000,
        position_timestamp_ns=15_000_000_000, **arrived,
    ).actions == (calibration.Action.clear_overrides(), calibration.Action.mode("LAND"))
    transition = observe(15_100_000_000, mode="GUIDED", armed=True)
    assert transition.state.phase is calibration.Phase.WAIT_LAND
    transition = observe(15_200_000_000, mode="LAND", armed=True)
    assert transition.state.phase is calibration.Phase.LANDING
    assert observe(
        16_000_000_000, mode="LAND", armed=True,
        status_text="AutoTune: Saved gains for Roll Pitch Yaw(E)",
    ).actions == ()
    transition = observe(
        17_000_000_000, mode="LAND", armed=False, landed=True, parameters={**baseline, **tuned},
        parameter_generation=1,
    )
    assert transition.actions == (calibration.Action.request_parameters(),)
    transition = observe(
        18_000_000_000, mode="LAND", armed=False, landed=True, parameters={**baseline, **tuned},
        parameter_generation=2,
    )
    assert transition.state.phase is calibration.Phase.COMPLETE
    assert transition.actions == (calibration.Action.export(),)


@pytest.mark.parametrize("changed", [
    {"latitude_deg": 37.4001},  # More than 11 m north of the launch point.
    {"longitude_deg": -122.0799},
    {"relative_altitude_m": 4.0},
    {"horizontal_speed_m_s": 0.3},
    {"position_timestamp_ns": 1_000_000_000},
    {"latitude_deg": math.nan},
])
def test_all_axis_return_requires_fresh_stationary_arrival(changed) -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    state = calibration.AllAxisState(
        phase=calibration.Phase.RETURNING, phase_started_ns=0,
        public_deadline_ns=600_000_000_000,
        home_latitude_deg=37.4, home_longitude_deg=-122.08,
        settle_started_ns=1_000_000_000,
    )
    observation = dict(
        timestamp_ns=3_000_000_000, mode="GUIDED", armed=True,
        latitude_deg=37.4, longitude_deg=-122.08, relative_altitude_m=5.0,
        position_timestamp_ns=3_000_000_000, telemetry_timestamp_ns=3_000_000_000,
        horizontal_speed_m_s=0.0, vertical_speed_m_s=0.0, roll_rad=0.0, pitch_rad=0.0,
    )
    transition = calibration.advance(state, calibration.Observation(**{**observation, **changed}))
    assert transition.actions == ()
    assert transition.state.phase is calibration.Phase.RETURNING
    assert transition.state.settle_started_ns is None


@pytest.mark.parametrize("changed", [
    {"timestamp_ns": 60_000_000_001}, {"mode": "LOITER"}, {"armed": False},
])
def test_all_axis_return_failure_cannot_command_successful_landing(changed) -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    state = calibration.AllAxisState(
        phase=calibration.Phase.RETURNING, phase_started_ns=0,
        public_deadline_ns=600_000_000_000,
        home_latitude_deg=37.4, home_longitude_deg=-122.08,
    )
    observation = {"timestamp_ns": 1, "mode": "GUIDED", "armed": True, **changed}
    transition = calibration.advance(state, calibration.Observation(**observation))
    assert transition.state.phase is calibration.Phase.FAILED
    assert transition.actions == ()


@pytest.mark.parametrize("changed", [
    {"latitude_deg": None}, {"latitude_deg": math.nan}, {"longitude_deg": 181.0},
    {"position_timestamp_ns": 0}, {"relative_altitude_m": 5.0},
])
def test_all_axis_does_not_arm_without_valid_ground_return_position(changed) -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    state = calibration.AllAxisState(
        phase=calibration.Phase.WAIT_GUIDED, phase_started_ns=0,
        public_deadline_ns=600_000_000_000,
    )
    parameters = {name: 1.0 for name in (*calibration.GAIN_PARAMETERS, *calibration.PRESERVED_PARAMETERS)}
    parameters["AUTOTUNE_AXES"] = 7.0
    observation = dict(
        timestamp_ns=1_000_000_000, mode="GUIDED", armed=False, parameters=parameters,
        latitude_deg=37.4, longitude_deg=-122.08, relative_altitude_m=0.0,
        position_timestamp_ns=1_000_000_000,
    )
    transition = calibration.advance(state, calibration.Observation(**{**observation, **changed}))
    assert transition.state.phase is calibration.Phase.WAIT_GUIDED
    assert transition.actions == ()


def test_all_axis_return_waypoint_uses_global_relative_altitude_frame() -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    commands = []
    vehicle = SimpleNamespace(_master=SimpleNamespace(
        target_system=7, target_component=1,
        mav=SimpleNamespace(set_position_target_global_int_send=lambda *args: commands.append(args)),
    ))
    calibration.execute_actions(
        vehicle, (calibration.Action.waypoint(37.4, -122.08, 5.0),),
        mode_factory=lambda name: name, export=lambda: None,
    )
    assert commands == [(0, 7, 1, 6, 3576, 374000000, -1220800000, 5.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]


def test_all_axis_autotune_stale_settle_sample_resets_window() -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    state = calibration.AllAxisState(
        phase=calibration.Phase.SETTLING,
        phase_started_ns=0,
        public_deadline_ns=600_000_000_000,
        settle_started_ns=1_000_000_000,
    )
    transition = calibration.advance(
        state,
        calibration.Observation(
            timestamp_ns=2_000_000_000,
            mode="LOITER", armed=True, telemetry_timestamp_ns=1_000_000_000,
            horizontal_speed_m_s=0.0, vertical_speed_m_s=0.0,
            roll_rad=0.0, pitch_rad=0.0,
        ),
    )
    assert transition.state.settle_started_ns is None
    assert transition.actions == ()


def test_statustext_reassembles_mavlink_two_chunk_message() -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    assembler = calibration.StatusTextAssembler()
    assert assembler.push(22, 0, "AutoTune: Pilot Testing gains for Roll Pitch Yaw(E") is None
    assert assembler.push(22, 1, ")") == (
        "AutoTune: Pilot Testing gains for Roll Pitch Yaw(E)"
    )


def test_all_axis_action_adapter_encodes_aux_and_clears_overrides() -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    commands: list[tuple[object, ...]] = []
    requests: list[tuple[object, ...]] = []
    vehicle = SimpleNamespace(
        _master=SimpleNamespace(
            target_system=7,
            target_component=1,
            mav=SimpleNamespace(
                command_long_send=lambda *args: commands.append(args),
                param_request_list_send=lambda *args: requests.append(args),
            ),
        ),
        channels=SimpleNamespace(overrides={"3": 1500}),
    )

    calibration.execute_actions(
        vehicle,
        (
            calibration.Action.aux_function(180, 2),
            calibration.Action.clear_overrides(),
            calibration.Action.request_parameters(),
        ),
        mode_factory=lambda name: name,
        export=lambda: None,
    )

    assert commands == [(7, 1, 218, 0, 180.0, 2.0, 0, 0, 0, 0, 0)]
    assert requests == [(7, 1)]
    assert vehicle.channels.overrides == {}


def test_all_axis_autotune_rejects_disabled_feedforward_without_writing_it() -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    state = calibration.AllAxisState.initial(public_deadline_ns=600_000_000_000)
    first = calibration.advance(
        state, calibration.Observation(0, heartbeat=True, prearm_checks_healthy=True)
    )
    assert all(action.name != "ATC_RATE_FF_ENAB" for action in first.actions)
    parameters = {
        name: 1.0
        for name in (*calibration.GAIN_PARAMETERS, *calibration.PRESERVED_PARAMETERS)
    }
    parameters.update({"ATC_RATE_FF_ENAB": 0.0, "AUTOTUNE_AXES": 7.0})
    rejected = calibration.advance(
        first.state,
        calibration.Observation(1, mode="GUIDED", armed=False, parameters=parameters),
    )
    assert rejected.state.phase is calibration.Phase.FAILED
    assert "feedforward" in rejected.state.failure_reason


def test_failed_airborne_autotune_uses_native_land_recovery_for_at_most_45_seconds() -> None:
    assert not runtime_node.autotune_failure_recovery_complete(
        recovery_started_ns=10, timestamp_ns=45_000_000_009, armed=True
    )
    assert runtime_node.autotune_failure_recovery_complete(
        recovery_started_ns=10, timestamp_ns=45_000_000_010, armed=True
    )
    assert runtime_node.autotune_failure_recovery_complete(
        recovery_started_ns=10, timestamp_ns=11, armed=False
    )


def test_parameter_readback_generation_requires_every_requested_parameter() -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    names = (*calibration.GAIN_PARAMETERS, *calibration.PRESERVED_PARAMETERS)
    values = {name: 1.0 for name in names}
    generations = {name: index + 1 for index, name in enumerate(names)}
    assert calibration.parameter_readback_generation(values, generations, names) == 1

    values.clear()
    generations.clear()
    for index, name in enumerate(names[:-1]):
        values[name] = 2.0
        generations[name] = 100 + index
    assert calibration.parameter_readback_generation(values, generations, names) == 0


def test_parameter_request_clears_session_before_mavlink_send() -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    session = {"old": 1.0}
    observed: list[dict[str, float]] = []
    vehicle = SimpleNamespace(_master=SimpleNamespace())

    def request() -> None:
        session.clear()
        observed.append(dict(session))

    calibration.execute_actions(
        vehicle,
        (calibration.Action.request_parameters(),),
        mode_factory=lambda name: name,
        export=lambda: None,
        request_parameters=request,
    )

    assert observed == [{}]


@pytest.mark.parametrize(
    "phase",
    [
        "WAIT_PRE_TUNE_LOITER",
        "WAIT_AUTOTUNE",
        "TUNING",
        "WAIT_POST_TUNE_LOITER",
        "WAIT_GAIN_ACTIVATION",
        "SETTLING",
    ],
)
def test_all_axis_autotune_keeps_neutral_rc_override_alive(phase: str) -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    assert calibration.neutral_override_required(calibration.Phase[phase])


@pytest.mark.parametrize(
    "status_text",
    [
        "AutoTune: Failed",
        "AutoTune: Rate D Gain Determination Failed",
        "AutoTune: Rate P Gain Determination Failed",
        "AutoTune: Angle P Gain Determination Failed",
    ],
)
def test_all_axis_autotune_fails_on_native_terminal_status(status_text: str) -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    state = calibration.AllAxisState(
        phase=calibration.Phase.TUNING,
        phase_started_ns=1,
        public_deadline_ns=600_000_000_000,
    )
    transition = calibration.advance(
        state,
        calibration.Observation(
            44_000_000_000, mode="AUTOTUNE", armed=True, status_text=status_text
        ),
    )
    assert transition.state.phase is calibration.Phase.FAILED
    assert transition.state.failure_reason == status_text
    assert transition.actions == ()


def test_all_axis_autotune_fails_immediately_on_unexpected_disarm() -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    state = calibration.AllAxisState(
        phase=calibration.Phase.TUNING,
        phase_started_ns=1,
        public_deadline_ns=600_000_000_000,
    )
    transition = calibration.advance(
        state,
        calibration.Observation(2, mode="AUTOTUNE", armed=False),
    )
    assert transition.state.phase is calibration.Phase.FAILED
    assert transition.state.failure_reason == "vehicle disarmed before native LAND"


def test_runtime_refreshes_neutral_override_twice_per_wall_second() -> None:
    calibration = importlib.import_module("drone_sim_companion.calibration_autotune")
    assert not runtime_node.autotune_neutral_refresh_due(
        phase=calibration.Phase.TUNING, last_refresh_wall=10.0, wall_now=10.49
    )
    assert runtime_node.autotune_neutral_refresh_due(
        phase=calibration.Phase.TUNING, last_refresh_wall=10.0, wall_now=10.5
    )
    assert not runtime_node.autotune_neutral_refresh_due(
        phase=calibration.Phase.WAIT_LAND, last_refresh_wall=10.0, wall_now=11.0
    )
    assert not runtime_node.autotune_neutral_refresh_due(
        phase=calibration.Phase.FAILED, last_refresh_wall=10.0, wall_now=11.0
    )
    last = 0.0
    refreshes = 0
    for wall_now in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5):
        if runtime_node.autotune_neutral_refresh_due(
            phase=calibration.Phase.TUNING,
            last_refresh_wall=last,
            wall_now=wall_now,
        ):
            refreshes += 1
            last = wall_now
    assert refreshes == 7
