from __future__ import annotations

from pathlib import Path

import pytest

from drone_sim_ardupilot.config import RuntimeConfig, resolve_gazebo_address


RUN_ID = "123e4567-e89b-42d3-a456-426614174000"


def _descent_parameters() -> dict[str, str]:
    parameter_file = Path(__file__).parents[1] / "params/descent.parm"
    return {
        name: value
        for line in parameter_file.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
        for name, value in (line.split(),)
    }


def test_descent_parameters_disable_rc_flight_mode_override() -> None:
    assert _descent_parameters()["FLTMODE_CH"] == "0"


def test_descent_parameters_enable_passive_extended_status_readiness() -> None:
    assert _descent_parameters()["MAV1_EXT_STAT"] == "1"


def test_descent_parameters_enable_mavlink_precision_landing() -> None:
    parameters = _descent_parameters()

    assert parameters["PLND_ENABLED"] == "1"
    assert parameters["PLND_TYPE"] == "1"


def test_descent_parameters_compensate_measured_simulator_camera_latency() -> None:
    assert float(_descent_parameters()["PLND_LAG"]) == pytest.approx(0.08)


def test_descent_parameters_use_each_accurate_stationary_target_measurement() -> None:
    assert _descent_parameters()["PLND_EST_TYPE"] == "0"


def test_descent_parameters_use_verified_competition_roll_rate_gains() -> None:
    parameters = _descent_parameters()

    assert parameters["ATC_RAT_RLL_P"] == "0.0675"
    assert parameters["ATC_RAT_RLL_I"] == "0.0675"
    assert parameters["ATC_RAT_RLL_D"] == "0.0018"


def test_descent_parameters_use_verified_competition_pitch_rate_gains() -> None:
    parameters = _descent_parameters()

    assert parameters["ATC_RAT_PIT_P"] == "0.0675"
    assert parameters["ATC_RAT_PIT_I"] == "0.0675"
    assert parameters["ATC_RAT_PIT_D"] == "0.0018"


def test_descent_parameters_use_precise_final_landing_speed() -> None:
    parameters = _descent_parameters()

    assert "LAND_SPD_MS" in parameters
    final_descent_speed_mps = float(parameters["LAND_SPD_MS"])

    # Keep final touchdown deliberately slow while retaining ample headroom
    # below the 1.0 m/s competition limit.
    assert final_descent_speed_mps == pytest.approx(0.10)


def test_descent_parameters_mark_sitl_accelerometers_calibrated() -> None:
    parameters = _descent_parameters()

    assert {
        name: parameters[name]
        for name in (
            "INS_ACCOFFS_X",
            "INS_ACCOFFS_Y",
            "INS_ACCOFFS_Z",
            "INS_ACCSCAL_X",
            "INS_ACCSCAL_Y",
            "INS_ACCSCAL_Z",
            "INS_ACC2OFFS_X",
            "INS_ACC2OFFS_Y",
            "INS_ACC2OFFS_Z",
            "INS_ACC2SCAL_X",
            "INS_ACC2SCAL_Y",
            "INS_ACC2SCAL_Z",
        )
    } == {
        "INS_ACCOFFS_X": "0.001",
        "INS_ACCOFFS_Y": "0.001",
        "INS_ACCOFFS_Z": "0.001",
        "INS_ACCSCAL_X": "1.001",
        "INS_ACCSCAL_Y": "1.001",
        "INS_ACCSCAL_Z": "1.001",
        "INS_ACC2OFFS_X": "0.001",
        "INS_ACC2OFFS_Y": "0.001",
        "INS_ACC2OFFS_Z": "0.001",
        "INS_ACC2SCAL_X": "1.001",
        "INS_ACC2SCAL_Y": "1.001",
        "INS_ACC2SCAL_Z": "1.001",
    }


def test_runtime_config_builds_lockstep_json_and_network_only_mavlink_argv(tmp_path: Path) -> None:
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        executable=Path("/opt/ardupilot/bin/arducopter"),
        parameter_file=Path("/opt/drone_sim/ardupilot/params/descent.parm"),
    )

    assert config.argv == (
        "/opt/ardupilot/bin/arducopter",
        "--model",
        "JSON",
        "--speedup",
        "1",
        "--sim-address",
        "gazebo-runtime",
        "--sim-port-in",
        "9003",
        "--sim-port-out",
        "9002",
        "--serial0",
        "tcp:0.0.0.0:5760",
        "--defaults",
        "/opt/drone_sim/ardupilot/params/descent.parm",
        "--home",
        "37.4003371,-122.0800351,0,0",
        "--wipe",
    )
    assert "--no-lockstep" not in config.argv


@pytest.mark.parametrize(
    "replacement",
    [
        {"run_id": "not-a-uuid"},
        {"run_id": "123E4567-E89B-42D3-A456-426614174000"},
        {"gazebo_host": ""},
        {"gazebo_port": 0},
        {"mavlink_port": 65536},
    ],
)
def test_runtime_config_rejects_ambiguous_identity_and_endpoints(
    tmp_path: Path, replacement: dict[str, object]
) -> None:
    values: dict[str, object] = {"run_id": RUN_ID, "run_directory": tmp_path}
    values.update(replacement)

    with pytest.raises(ValueError):
        RuntimeConfig(**values)  # type: ignore[arg-type]


def test_gazebo_service_name_is_resolved_for_upstream_numeric_only_socket() -> None:
    observed: list[str] = []

    def resolve(name: str) -> str:
        observed.append(name)
        return "172.23.0.2"

    assert resolve_gazebo_address("gazebo-runtime", resolver=resolve) == "172.23.0.2"
    assert observed == ["gazebo-runtime"]


def test_gazebo_resolution_rejects_non_ipv4_result() -> None:
    with pytest.raises(ValueError, match="IPv4"):
        resolve_gazebo_address("gazebo-runtime", resolver=lambda _name: "not-an-address")
