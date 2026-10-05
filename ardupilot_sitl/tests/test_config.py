from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from drone_sim_ardupilot.config import (
    CALIBRATION_PARAMETERS,
    RuntimeConfig,
    parameter_files_from_environment,
    resolve_gazebo_address,
)


RUN_ID = "123e4567-e89b-42d3-a456-426614174000"


def _with_config_sha256(document: dict[str, object]) -> dict[str, object]:
    encoded = json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
    return {**document, "config_sha256": hashlib.sha256(encoded).hexdigest()}


def _descent_parameters() -> dict[str, str]:
    parameter_file = Path(__file__).parents[1] / "params/descent.parm"
    return {
        name: value
        for line in parameter_file.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
        for name, value in (line.split(),)
    }


def _moving_parameters() -> dict[str, str]:
    parameter_file = Path(__file__).parents[1] / "params/moving-pad.parm"
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


def test_descent_parameters_use_promoted_roll_autotune_gains() -> None:
    parameters = _descent_parameters()

    assert {
        name: parameters[name]
        for name in (
            "ATC_ANG_RLL_P",
            "ATC_RAT_RLL_P",
            "ATC_RAT_RLL_I",
            "ATC_RAT_RLL_D",
            "ATC_ACC_R_MAX",
        )
    } == {
        "ATC_ANG_RLL_P": "13.1974",
        "ATC_RAT_RLL_P": "0.0503722",
        "ATC_RAT_RLL_I": "0.0503722",
        "ATC_RAT_RLL_D": "0.000375",
        "ATC_ACC_R_MAX": "2547.76",
    }


def test_descent_parameters_use_verified_competition_pitch_rate_gains() -> None:
    parameters = _descent_parameters()

    assert parameters["ATC_RAT_PIT_P"] == "0.0675"
    assert parameters["ATC_RAT_PIT_I"] == "0.0675"
    assert parameters["ATC_RAT_PIT_D"] == "0.0018"


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
            "LAND_SPD_MS",
            "PLND_LAG",
            "PLND_XY_DIST_MAX",
            "PLND_TIMEOUT",
            "PLND_ALT_MIN",
            "PLND_ALT_MAX",
        )
    } == pytest.approx(
        {
            "LAND_SPD_MS": 0.50,
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
        "--serial1",
        "tcp:0.0.0.0:5762",
        "--defaults",
        "/opt/drone_sim/ardupilot/params/descent.parm",
        "--home",
        "37.4003371,-122.0800351,0,0",
        "--wipe",
    )
    assert "--no-lockstep" not in config.argv


def test_operator_port_preserves_the_primary_endpoint(tmp_path: Path) -> None:
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        gazebo_host="127.0.0.1",
    )

    assert config.argv[config.argv.index("--serial0") + 1] == "tcp:0.0.0.0:5760"
    assert config.argv[config.argv.index("--serial1") + 1] == "tcp:0.0.0.0:5762"


def test_operator_port_must_be_distinct_from_the_primary_endpoint(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="operator_mavlink_port must differ"):
        RuntimeConfig(
            run_id=RUN_ID,
            run_directory=tmp_path,
            mavlink_port=5760,
            operator_mavlink_port=5760,
        )


def test_moving_profile_overlays_ekf3_and_native_precision_estimator(tmp_path: Path) -> None:
    overlay = Path("/opt/drone_sim/ardupilot/params/moving-pad.parm")
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        parameter_overlay_file=overlay,
    )

    defaults = config.argv[config.argv.index("--defaults") + 1]
    assert defaults == "/opt/drone_sim/ardupilot/params/descent.parm,/opt/drone_sim/ardupilot/params/moving-pad.parm"
    assert _moving_parameters() == {
        "AHRS_EKF_TYPE": "3",
        "PLND_OPTIONS": "5",
        "PLND_EST_TYPE": "1",
        "PLND_LAG": "0.04",
        "PSC_NE_POS_P": "1",
    }


def test_frozen_calibration_is_verified_and_loaded_after_scenario(tmp_path: Path) -> None:
    values = {name: index / 1000 for index, name in enumerate(CALIBRATION_PARAMETERS, 1)}
    calibration = "".join(f"{name} {value}\n" for name, value in values.items()).encode()
    configuration = tmp_path / "configuration"
    configuration.mkdir()
    calibration_path = configuration / "calibration.parm"
    calibration_path.write_bytes(calibration)
    config_path = configuration / "run.json"
    config_path.write_text(json.dumps(_with_config_sha256({
        "run_id": RUN_ID, "scenario": "moving_pad_v1",
        "calibration_json": {"schema_version": 1, "source_run_id": "source",
            "source_manifest_sha256": "0" * 64,
            "source_artifact_sha256": hashlib.sha256(calibration).hexdigest(),
            "gains": values, "profile": {"baseline_parameters": {}}},
    })), encoding="utf-8")

    overlay, frozen = parameter_files_from_environment(
        {"SIM_CONFIG_PATH": str(config_path)}, run_id=RUN_ID, run_directory=tmp_path,
    )
    runtime = RuntimeConfig(run_id=RUN_ID, run_directory=tmp_path,
                            parameter_overlay_file=overlay, calibration_file=frozen)

    assert runtime.argv[runtime.argv.index("--defaults") + 1].split(",") == [
        "/opt/drone_sim/ardupilot/params/descent.parm",
        "/opt/drone_sim/ardupilot/params/moving-pad.parm",
        str(calibration_path),
    ]


def test_frozen_calibration_rejects_changed_bytes(tmp_path: Path) -> None:
    configuration = tmp_path / "configuration"
    configuration.mkdir()
    (configuration / "calibration.parm").write_text("changed\n", encoding="utf-8")
    config_path = configuration / "run.json"
    config_path.write_text(json.dumps(_with_config_sha256({
        "run_id": RUN_ID,
        "calibration_json": {"schema_version": 1, "source_run_id": "source",
            "source_manifest_sha256": "1" * 64, "source_artifact_sha256": "0" * 64,
            "gains": {name: 0.1 for name in CALIBRATION_PARAMETERS},
            "profile": {"baseline_parameters": {}}},
    })), encoding="utf-8")

    with pytest.raises(ValueError, match="calibration artifact checksum"):
        parameter_files_from_environment(
            {"SIM_CONFIG_PATH": str(config_path)}, run_id=RUN_ID, run_directory=tmp_path,
        )


def test_frozen_calibration_rejects_changed_run_configuration(tmp_path: Path) -> None:
    values = {name: 0.1 for name in CALIBRATION_PARAMETERS}
    calibration = "".join(f"{name} {value}\n" for name, value in values.items()).encode()
    configuration = tmp_path / "configuration"
    configuration.mkdir()
    (configuration / "calibration.parm").write_bytes(calibration)
    document = _with_config_sha256({
        "run_id": RUN_ID,
        "calibration_json": {
            "schema_version": 1, "source_run_id": "source",
            "source_manifest_sha256": "1" * 64,
            "source_artifact_sha256": hashlib.sha256(calibration).hexdigest(),
            "gains": values, "profile": {"baseline_parameters": {}},
        },
    })
    document["scenario"] = "changed-after-freeze"
    config_path = configuration / "run.json"
    config_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="config_sha256"):
        parameter_files_from_environment(
            {"SIM_CONFIG_PATH": str(config_path)}, run_id=RUN_ID, run_directory=tmp_path,
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
