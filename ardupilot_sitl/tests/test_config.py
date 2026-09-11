from __future__ import annotations

import json
import re
from pathlib import Path

import pytest

from drone_sim_ardupilot.config import LaunchOrigin, RuntimeConfig, resolve_gazebo_address


RUN_ID = "123e4567-e89b-42d3-a456-426614174000"


def _launch_origin() -> LaunchOrigin:
    return LaunchOrigin(
        latitude_deg=37.4003371,
        longitude_deg=-122.0800351,
        amsl_m=12.5,
        heading_deg=270,
    )


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
    parameters = _descent_parameters()

    assert float(parameters["SR0_EXT_STAT"]) == pytest.approx(1.0)
    assert "MAV1_EXT_STAT" not in parameters


def test_descent_parameters_enable_mavlink_precision_landing() -> None:
    parameters = _descent_parameters()

    assert parameters["PLND_ENABLED"] == "1"
    assert parameters["PLND_TYPE"] == "1"


def test_descent_parameters_compensate_measured_simulator_camera_latency() -> None:
    assert float(_descent_parameters()["PLND_LAG"]) == pytest.approx(0.08)


def test_descent_parameters_use_each_accurate_stationary_target_measurement() -> None:
    assert _descent_parameters()["PLND_EST_TYPE"] == "0"


def test_descent_parameters_use_promoted_roll_autotune_gains() -> None:
    parameters = _descent_parameters()

    assert {
        name: parameters[name]
        for name in (
            "ATC_ANG_RLL_P",
            "ATC_RAT_RLL_P",
            "ATC_RAT_RLL_I",
            "ATC_RAT_RLL_D",
            "ATC_ACCEL_R_MAX",
        )
    } == {
        "ATC_ANG_RLL_P": "13.1974",
        "ATC_RAT_RLL_P": "0.0503722",
        "ATC_RAT_RLL_I": "0.0503722",
        "ATC_RAT_RLL_D": "0.000375",
        "ATC_ACCEL_R_MAX": "254776",
    }


def test_descent_parameters_use_verified_competition_pitch_rate_gains() -> None:
    parameters = _descent_parameters()

    assert parameters["ATC_RAT_PIT_P"] == "0.0675"
    assert parameters["ATC_RAT_PIT_I"] == "0.0675"
    assert parameters["ATC_RAT_PIT_D"] == "0.0018"


def test_descent_parameters_use_faster_final_landing_speed() -> None:
    parameters = _descent_parameters()

    assert "LAND_SPD_MS" not in parameters
    final_descent_speed_mps = float(parameters["LAND_SPEED"]) / 100.0

    assert final_descent_speed_mps == pytest.approx(0.50)


def test_descent_parameters_preserve_roll_acceleration_in_target_units() -> None:
    parameters = _descent_parameters()

    assert "ATC_ACC_R_MAX" not in parameters
    roll_acceleration_degrees_per_second_squared = (
        float(parameters["ATC_ACCEL_R_MAX"]) / 100.0
    )

    assert roll_acceleration_degrees_per_second_squared == pytest.approx(2547.76)


def test_descent_parameters_disable_precision_landing_final_slowdown() -> None:
    assert int(_descent_parameters()["PLND_OPTIONS"]) & 4


def test_descent_parameters_use_fast_guarded_precision_landing_profile() -> None:
    parameters = _descent_parameters()

    assert {
        name: parameters[name]
        for name in (
            "PLND_ENABLED",
            "PLND_TYPE",
            "PLND_EST_TYPE",
            "PLND_STRICT",
            "PLND_RET_MAX",
            "PLND_OPTIONS",
        )
    } == {
        "PLND_ENABLED": "1",
        "PLND_TYPE": "1",
        "PLND_EST_TYPE": "0",
        "PLND_STRICT": "2",
        "PLND_RET_MAX": "1",
        "PLND_OPTIONS": "4",
    }
    assert {
        name: float(parameters[name])
        for name in (
            "PLND_LAG",
            "PLND_XY_DIST_MAX",
            "PLND_TIMEOUT",
            "PLND_ALT_MIN",
            "PLND_ALT_MAX",
        )
    } == pytest.approx(
        {
            "PLND_LAG": 0.08,
            "PLND_XY_DIST_MAX": 0.50,
            "PLND_TIMEOUT": 0.50,
            "PLND_ALT_MIN": 0.75,
            "PLND_ALT_MAX": 8.0,
        }
    )


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
        launch_origin=_launch_origin(),
        executable=Path("/opt/ardupilot/bin/arducopter"),
        parameter_file=Path("/opt/drone_sim/ardupilot/params/descent.parm"),
        mavlink_port=14550,
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
        "tcp:14550",
        "--defaults",
        "/opt/drone_sim/ardupilot/params/descent.parm",
        "--home",
        "37.4003371,-122.0800351,12.5,270",
        "--wipe",
    )
    assert "--no-lockstep" not in config.argv


def test_launch_origin_formats_boundary_values_for_ardupilot_home() -> None:
    origin = LaunchOrigin(
        latitude_deg=-90,
        longitude_deg=180,
        amsl_m=-3.25,
        heading_deg=359.5,
    )

    assert origin.ardupilot_home == "-90,180,-3.25,359.5"


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("latitude_deg", True),
        ("latitude_deg", float("nan")),
        ("latitude_deg", -90.0000001),
        ("latitude_deg", 90.0000001),
        ("longitude_deg", False),
        ("longitude_deg", float("inf")),
        ("longitude_deg", -180.0000001),
        ("longitude_deg", 180.0000001),
        ("amsl_m", "0"),
        ("amsl_m", float("-inf")),
        ("heading_deg", True),
        ("heading_deg", -0.0000001),
        ("heading_deg", 360),
    ],
)
def test_launch_origin_rejects_non_numeric_nonfinite_and_out_of_range_values(
    field: str, value: object
) -> None:
    values: dict[str, object] = {
        "latitude_deg": 0,
        "longitude_deg": 0,
        "amsl_m": 0,
        "heading_deg": 0,
    }
    values[field] = value

    with pytest.raises(ValueError):
        LaunchOrigin(**values)


def test_build_and_provenance_pin_exact_official_copter_release() -> None:
    module_root = Path(__file__).parents[1]
    dockerfile = (module_root / "Dockerfile").read_text(encoding="utf-8")
    provenance = json.loads(
        (module_root / "provenance/ardupilot.json").read_text(encoding="utf-8")
    )
    revision_match = re.search(
        r"^ARG ARDUPILOT_COMMIT=([0-9a-f]{40})$", dockerfile, re.MULTILINE
    )

    assert revision_match is not None
    assert revision_match.group(1) == "2a3dc4b7bf2507120f7378a7b2fde73185e0c325"
    assert provenance["tag"] == "Copter-4.5.7"
    assert provenance["revision"] == revision_match.group(1)
    assert (
        provenance["build_base"]
        == "ardupilot/ardupilot-dev-base@sha256:576cd622957308469d6e72528befe5de5862457b264e61d60aaa4b8f29de85b6"
    )
    assert 'grep -Fqx \'#define THISFIRMWARE "ArduCopter V4.5.7"\'' in dockerfile
    assert (
        'org.opencontainers.image.revision="2a3dc4b7bf2507120f7378a7b2fde73185e0c325"'
        in dockerfile
    )


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
    values: dict[str, object] = {
        "run_id": RUN_ID,
        "run_directory": tmp_path,
        "launch_origin": _launch_origin(),
    }
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
