from __future__ import annotations

from dataclasses import FrozenInstanceError, replace
import hashlib
import json
from pathlib import Path
import shutil
import stat
import sys
import threading
import time
import tomllib

import pytest


ROOT = Path(__file__).parents[2]
NESTED_SOURCE = ROOT / "companion/comp2026/src"
NESTED_TESTS = ROOT / "companion/comp2026/tests"
sys.path.insert(0, str(NESTED_SOURCE))
sys.path.insert(0, str(NESTED_TESTS))
sys.path.insert(0, str(ROOT / "companion/tests"))

from drone import timebase
from drone.control.attempt_setup import initialize_ledger, prepare_attempt
from drone.control.listener import construct_after_full_validation
from drone.control.listener_runtime import (
    InjectedComponentConfig,
    VisionConfig,
    shared_monotonic_ns,
)
from drone.control.mission_supervisor import FM1, FM2, FM3, MissionSupervisor
from drone.common_types import GPSCoord, MissionHome
from drone.mock_mission import PrecisionMissionPolicy
from test_mission_supervisor import RecoveryController
from test_prepare_attempt import profile_data

import drone_sim_companion.qgc_runtime_config as qgc_runtime_config
from drone_sim_companion.qgc_runtime_config import ResolvedQGCInputs
from drone_sim_companion.runtime_node import RuntimeConfig
from test_qgc_runtime_policy import full_policy, valid_policy


RUN_ID = "00000000-0000-4000-8000-000000000001"


def _write(path: Path, content: bytes) -> str:
    path.write_bytes(content)
    return hashlib.sha256(content).hexdigest()


def prepared_config(tmp_path: Path) -> tuple[RuntimeConfig, Path, dict[str, object]]:
    run_directory = tmp_path / "runs" / RUN_ID
    configuration = run_directory / "configuration"
    configuration.mkdir(parents=True)
    course = (ROOT / "config/course.yaml").read_bytes()
    scenario = (ROOT / "config/scenario.yaml").read_bytes()
    course_digest = _write(configuration / "course.yaml", course)
    scenario_digest = _write(configuration / "scenario.yaml", scenario)

    profile_path = configuration / "deployment-profile.json"
    deployment_profile = profile_data()
    deployment_profile["observation"]["mode_mapping"].update(  # type: ignore[index]
        {"6": "RTL", "9": "LAND"}
    )
    profile_digest = _write(
        profile_path,
        json.dumps(deployment_profile, separators=(",", ":")).encode(),
    )
    state_root = tmp_path / "attempt-state"
    state_directory = state_root / f"sha256-{profile_digest}"
    state_directory.mkdir(parents=True)
    ledger_path = state_directory / "attempt-ledger.json"
    initialize_ledger(ledger_path)
    session_path = configuration / "listener-session.json"
    actions_path = configuration / "qgc-actions.json"
    prepare_attempt(
        profile_path,
        ledger_path,
        session_path,
        actions_path,
        acknowledge_on_ground=True,
    )

    policy_data = valid_policy()
    policy_data["bindings"] = {
        "course_sha256": course_digest,
        "scenario_sha256": scenario_digest,
        "ardupilot_commit": "2a3dc4b7bf2507120f7378a7b2fde73185e0c325",
    }
    policy_path = configuration / "qgc-runtime.json"
    policy_digest = _write(
        policy_path,
        json.dumps(policy_data, separators=(",", ":")).encode(),
    )
    qgc = ResolvedQGCInputs(
        deployment_profile_path=profile_path,
        deployment_profile_sha256=profile_digest,
        listener_session_path=session_path,
        listener_session_sha256=hashlib.sha256(session_path.read_bytes()).hexdigest(),
        qgc_actions_path=actions_path,
        qgc_actions_sha256=hashlib.sha256(actions_path.read_bytes()).hexdigest(),
        runtime_policy_path=policy_path,
        runtime_policy_sha256=policy_digest,
        attempt_state_id=f"sha256-{profile_digest}",
        course_sha256=course_digest,
        scenario_sha256=scenario_digest,
    )
    return (
        RuntimeConfig(
            run_id=RUN_ID,
            run_directory=run_directory,
            mission="comp2026_auto",
            course_path=configuration / "course.yaml",
            scenario_path=configuration / "scenario.yaml",
            qgc=qgc,
            output_root=tmp_path / "outputs",
        ),
        state_root,
        policy_data,
    )


def prepared_full_config(
    tmp_path: Path,
) -> tuple[RuntimeConfig, Path, dict[str, object]]:
    config, state_root, limited = prepared_config(tmp_path)
    policy = full_policy()
    policy["bindings"] = limited["bindings"]
    digest = _write(
        config.qgc.runtime_policy_path,
        json.dumps(policy, separators=(",", ":")).encode(),
    )
    config = replace(
        config,
        qgc=replace(config.qgc, runtime_policy_sha256=digest),
    )
    return config, state_root, policy


def _resolved_run_document(config: RuntimeConfig) -> dict[str, object]:
    assert config.qgc is not None
    qgc = config.qgc
    return {
        "run_id": config.run_id,
        "startup_wall_seconds": 120,
        "max_wall_seconds": 3600,
        "finalization_wall_seconds": 120,
        "mission": "comp2026_auto",
        "output_root": str(config.output_root),
        "runtime_profile": "phase3",
        "simulation": {
            "seed": 2026,
            "duration_sim_seconds": 600.0,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0.25,
        },
        "competition": {
            "course": "course.yaml",
            "scenario": "scenario.yaml",
            "course_sha256": qgc.course_sha256,
            "scenario_sha256": qgc.scenario_sha256,
        },
        "qgc": {
            "deployment_profile": "deployment-profile.json",
            "deployment_profile_sha256": qgc.deployment_profile_sha256,
            "listener_session": "listener-session.json",
            "listener_session_sha256": qgc.listener_session_sha256,
            "qgc_actions": "qgc-actions.json",
            "qgc_actions_sha256": qgc.qgc_actions_sha256,
            "runtime_policy": "qgc-runtime.json",
            "runtime_policy_sha256": qgc.runtime_policy_sha256,
            "attempt_state_id": qgc.attempt_state_id,
        },
    }


def _replace_competition_bytes(
    config: RuntimeConfig,
    policy: dict[str, object],
    *,
    label: str,
    content: bytes,
) -> RuntimeConfig:
    digest = _write(config.run_directory / f"configuration/{label}.yaml", content)
    policy["bindings"][f"{label}_sha256"] = digest  # type: ignore[index]
    policy_digest = _write(
        config.qgc.runtime_policy_path,  # type: ignore[union-attr]
        json.dumps(policy, separators=(",", ":")).encode(),
    )
    return replace(
        config,
        qgc=replace(
            config.qgc,
            **{
                f"{label}_sha256": digest,
                "runtime_policy_sha256": policy_digest,
            },
        ),
    )


def test_runtime_environment_rejects_competition_hash_mismatch(tmp_path: Path) -> None:
    config, _state_root, _ = prepared_config(tmp_path)
    config_path = config.run_directory / "configuration/run.json"
    config_path.write_text(json.dumps(_resolved_run_document(config)))
    config.course_path.write_bytes(config.course_path.read_bytes() + b"\n")

    with pytest.raises(ValueError, match="course.*SHA-256"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": config.run_id,
                "SIM_RUN_DIRECTORY": str(config.run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


def test_runtime_environment_loads_only_canonical_resolved_qgc_inputs(
    tmp_path: Path,
) -> None:
    expected, _state_root, _ = prepared_config(tmp_path)
    config_path = expected.run_directory / "configuration/run.json"
    config_path.write_text(json.dumps(_resolved_run_document(expected)))

    loaded = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": expected.run_id,
            "SIM_RUN_DIRECTORY": str(expected.run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )

    assert loaded.qgc == expected.qgc
    assert loaded.course_path == expected.course_path
    assert loaded.scenario_path == expected.scenario_path
    assert loaded.output_root == expected.output_root


@pytest.mark.parametrize(
    ("runtime_profile", "include_simulation"),
    [(None, False), ("phase2", False), ("phase3", False)],
    ids=["missing-profile", "phase2", "phase3-without-simulation"],
)
def test_runtime_environment_rejects_qgc_outside_explicit_phase3(
    tmp_path: Path, runtime_profile: str | None, include_simulation: bool
) -> None:
    expected, _state_root, _ = prepared_config(tmp_path)
    document = _resolved_run_document(expected)
    if runtime_profile is None:
        document.pop("runtime_profile")
    else:
        document["runtime_profile"] = runtime_profile
    if not include_simulation:
        document.pop("simulation")
    config_path = expected.run_directory / "configuration/run.json"
    config_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="QGC.*phase3"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": expected.run_id,
                "SIM_RUN_DIRECTORY": str(expected.run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


def test_runtime_rejects_qgc_structure_before_reading_competition_sources(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    expected, _state_root, _ = prepared_config(tmp_path)
    document = _resolved_run_document(expected)
    document["qgc"] = {}
    config_path = expected.run_directory / "configuration/run.json"
    config_path.write_text(json.dumps(document), encoding="utf-8")
    referenced_reads: list[Path] = []
    original_read = qgc_runtime_config._read_regular_file

    def reject_referenced_read(path: Path, *, label: str):
        if path != config_path:
            referenced_reads.append(path)
            raise AssertionError("referenced sources must not be read")
        return original_read(path, label=label)

    monkeypatch.setattr(qgc_runtime_config, "_read_regular_file", reject_referenced_read)

    with pytest.raises(ValueError, match="QGC"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": expected.run_id,
                "SIM_RUN_DIRECTORY": str(expected.run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )
    assert referenced_reads == []


def test_projects_complete_inert_fm1_fm2_runtime(tmp_path: Path) -> None:
    config, state_root, policy_data = prepared_config(tmp_path)
    epoch = time.time()

    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=epoch, test_only_state_root=state_root
    )

    runtime = projection.runtime_configuration
    assert projection.validated_listener_artifacts.ledger.path == (
        state_root / config.qgc.attempt_state_id / "attempt-ledger.json"
    )
    assert runtime.connection.source_identity == projection.flight_profile.companion_target
    assert runtime.connection.target_identity == projection.flight_profile.flight_controller
    assert runtime.autopilot_version.flight_sw_version == 0x040507FF
    assert runtime.autopilot_version.flight_custom_version == bytes.fromhex(
        policy_data["autopilot_version"]["flight_custom_version"]
    )
    assert isinstance(runtime.components, InjectedComponentConfig)
    assert config.qgc.course_sha256 in runtime.components.evidence_reference
    assert config.qgc.scenario_sha256 in runtime.components.evidence_reference
    assert runtime.enabled_phases == frozenset((FM1, FM2))
    assert runtime.vision is None
    assert runtime.precision_policy is None
    assert runtime.cruise_altitude_m == runtime.desired_drop_height_m == 10.0
    assert runtime.attempt_timeout_s == 600.0
    assert projection.payload_delay_wall_timeout_s == policy_data[
        "payload_delay_wall_timeout_s"
    ]
    assert set(json.loads(runtime.waypoint_path.read_text())) == {"L", "TARGET"}

    launch = json.loads(projection.sim_launch_origin_json)
    assert launch == policy_data["simulator_launch_origin"]
    waypoints = json.loads(runtime.waypoint_path.read_text())
    assert waypoints["L"]["coords"]["alt"] == 10.0
    assert waypoints["TARGET"]["coords"]["alt"] == 10.0
    assert waypoints["L"]["loaded_at"] == epoch
    waypoint_check = runtime.operating_site.waypoint_check
    assert waypoint_check(
        "L",
        GPSCoord(
            lat=waypoints["L"]["coords"]["lat"],
            long=waypoints["L"]["coords"]["long"],
            alt=10.0,
        ),
    )
    assert not waypoint_check(
        "H", GPSCoord(launch["latitude_deg"], launch["longitude_deg"], 10.0)
    )
    home = MissionHome(
        launch["latitude_deg"], launch["longitude_deg"], launch["amsl_m"]
    )
    runtime.operating_site.mission_home_check(home)
    runtime.operating_site.recovery_check("RETURN", home, home.amsl_m + 5.0)
    with pytest.raises(ValueError):
        runtime.operating_site.recovery_check("FM3", home, 5.0)
    with pytest.raises(ValueError):
        runtime.operating_site.recovery_check("LOCAL_LAND", home, 31.0)
    with pytest.raises(FrozenInstanceError):
        projection.sim_launch_origin_json = "{}"


def test_full_projection_freezes_exact_l_target_wa_wm1_waypoints(
    tmp_path: Path,
) -> None:
    config, state_root, _policy = prepared_full_config(tmp_path)
    epoch = time.time()

    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=epoch, test_only_state_root=state_root
    )

    runtime = projection.runtime_configuration
    assert runtime.enabled_phases == frozenset((31000, 31001, 31002))
    waypoints = json.loads(runtime.waypoint_path.read_text())
    assert tuple(sorted(waypoints)) == ("L", "TARGET", "WA", "WM1")
    expected_coordinates = {
        "L": (52.0, 13.248665793575334, 10.0),
        "TARGET": (52.0, 13.247776322625556, 10.0),
        "WA": (51.99991785805042, 13.249332896787667, 10.0),
        "WM1": (52.00008214194958, 13.249332896787667, 10.0),
    }
    for name, expected in expected_coordinates.items():
        coords = waypoints[name]["coords"]
        assert (coords["lat"], coords["long"], coords["alt"]) == pytest.approx(
            expected
        )
        assert waypoints[name]["loaded_at"] == epoch


@pytest.mark.parametrize("mutation", ["missing", "reordered"])
def test_full_projection_rejects_missing_or_reordered_fm3_cycles(
    tmp_path: Path, mutation: str
) -> None:
    import yaml

    config, state_root, policy = prepared_full_config(tmp_path)
    scenario = yaml.safe_load(config.scenario_path.read_text())
    cycles = scenario["mission"]["fm3_cycles"]
    scenario["mission"]["fm3_cycles"] = (
        cycles[:1] if mutation == "missing" else list(reversed(cycles))
    )
    config = _replace_competition_bytes(
        config,
        policy,
        label="scenario",
        content=yaml.safe_dump(scenario, sort_keys=False).encode(),
    )

    with pytest.raises(ValueError, match="competition contract"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )


def test_full_projection_builds_version_bound_vision_and_precision_policy(
    tmp_path: Path,
) -> None:
    config, state_root, _policy = prepared_full_config(tmp_path)

    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )

    runtime = projection.runtime_configuration
    assert isinstance(runtime.vision, VisionConfig)
    assert isinstance(runtime.precision_policy, PrecisionMissionPolicy)
    assert runtime.vision.marker_size_mm == 100.0
    assert runtime.vision.calibration_path.name == "gazebo_camera_calibration.json"
    assert runtime.vision.mounting_path.name == "gazebo_camera_mounting.json"
    assert runtime.vision.receipt_clock_ns is shared_monotonic_ns
    assert runtime.vision.max_exposure_age_ns == 500_000_000
    assert (
        runtime.precision_policy.clearance_calibration
        is runtime.clearance_calibration
    )
    assert runtime.precision_policy.clock is timebase.monotonic
    expected_precision = {
        "max_exposure_age_s": 0.5,
        "max_image_attitude_skew_s": 0.05,
        "max_image_location_skew_s": 0.05,
        "max_attitude_transport_latency_s": 0.05,
        "max_location_transport_latency_s": 0.05,
        "acquisition_timeout_s": 5.0,
        "frame_timeout_s": 0.5,
        "observation_period_s": 0.05,
        "target_hover_height_m": 4.572,
        "hover_tolerance_m": 0.2,
        "centered_tolerance_m": 0.075,
        "correction_gain": 0.5,
        "cruise_altitude_m": 10.0,
        "desired_drop_height_m": 10.0,
        "landing_timeout_s": 60.0,
        "target_loss_timeout_s": 0.5,
    }
    for field, expected in expected_precision.items():
        assert getattr(runtime.precision_policy, field) == expected
    assert runtime.precision_policy.reacquisition_count == 5
    assert runtime.autopilot_version.flight_custom_version == bytes.fromhex(
        "3261336463346237"
    )


def test_full_projection_camera_resources_are_installed_package_data() -> None:
    project = tomllib.loads((ROOT / "companion/pyproject.toml").read_text())

    assert set(project["tool"]["setuptools"]["package-data"]["drone_sim_companion"]) >= {
        "gazebo_camera_calibration.json",
        "gazebo_camera_mounting.json",
    }


def test_projected_recovery_authorizes_return_and_local_fallback(tmp_path: Path) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    origin = json.loads(projection.sim_launch_origin_json)
    home = MissionHome(origin["latitude_deg"], origin["longitude_deg"], origin["amsl_m"])

    class SiteRecoveryController(RecoveryController):
        def climb(self, target_alt, *, timeout):
            self._permit()
            self.events.append(("climb", target_alt, timeout))
            self.amsl_m = self.mission_home.amsl_m + target_alt
            self._after("climb")
            return self.results.get("climb")

    def recover(*, fail_return: bool) -> str:
        controller = SiteRecoveryController()
        controller.mission_home = home
        controller.amsl_m = home.amsl_m + 3.0
        if fail_return:
            controller.results["return"] = -1
        supervisor = MissionSupervisor(
            attempt_id=7,
            admission_check=lambda _request: None,
            attempt_consumer=lambda _attempt_id: None,
            permission_check=lambda: None,
            recovery_policy=projection.runtime_configuration.recovery_policy,
        )
        controller.permission_guard = supervisor.check_permission
        supervisor.request_abort("test recovery")
        return supervisor.recover(controller, home, 10.0)

    assert recover(fail_return=False) == "HOME_LANDED"
    assert recover(fail_return=True) == "LOCAL_LANDED"


def test_projected_recovery_authorizes_local_land_for_invalid_return_datum(
    tmp_path: Path,
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    origin = json.loads(projection.sim_launch_origin_json)
    requested_home = MissionHome(
        origin["latitude_deg"], origin["longitude_deg"], origin["amsl_m"]
    )
    controller = RecoveryController()
    controller.mission_home = MissionHome(41.0, -81.0, 100.0)
    supervisor = MissionSupervisor(
        attempt_id=7,
        admission_check=lambda _request: None,
        attempt_consumer=lambda _attempt_id: None,
        permission_check=lambda: None,
        recovery_policy=projection.runtime_configuration.recovery_policy,
    )
    controller.permission_guard = supervisor.check_permission
    supervisor.request_abort("invalid return datum")

    assert supervisor.recover(controller, requested_home, 10.0) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == ["land", "disarm"]


def test_longitude_wrap_treats_dateline_endpoints_as_same_site() -> None:
    first = GPSCoord(10.0, 180.0, 0.0)
    second = GPSCoord(10.0, -180.0, 0.0)
    assert qgc_runtime_config._distance_m(first, second) == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("latitude", "longitude"),
    [(90.0, 13.25), (89.9995, 13.25), (52.0, -179.9999)],
)
def test_rejects_singular_or_out_of_range_derived_coordinates_before_output(
    tmp_path: Path, latitude: float, longitude: float
) -> None:
    config, state_root, policy = prepared_config(tmp_path)
    policy["simulator_launch_origin"]["latitude_deg"] = latitude
    policy["simulator_launch_origin"]["longitude_deg"] = longitude
    policy_path = config.qgc.runtime_policy_path
    digest = _write(policy_path, json.dumps(policy, separators=(",", ":")).encode())
    config = replace(config, qgc=replace(config.qgc, runtime_policy_sha256=digest))

    with pytest.raises(ValueError, match="coordinate|origin"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert not (config.run_directory / ".control").exists()


def test_attempt_state_cannot_overlap_resolved_output_root(tmp_path: Path) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    config = replace(config, output_root=state_root)

    with pytest.raises(ValueError, match="overlap"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert not (config.run_directory / ".control").exists()


def test_projection_is_idempotent_and_never_overwrites_waypoints(tmp_path: Path) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    epoch = time.time()
    first = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=epoch, test_only_state_root=state_root
    )
    before = first.runtime_configuration.waypoint_path.read_bytes()

    second = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=epoch, test_only_state_root=state_root
    )
    assert second.runtime_configuration.waypoint_path.read_bytes() == before

    first.runtime_configuration.waypoint_path.write_text("{}")
    with pytest.raises(ValueError, match="overwrite"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=epoch, test_only_state_root=state_root
        )


def test_failure_does_not_create_control_output(tmp_path: Path) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    config.qgc.runtime_policy_path.write_bytes(b"{}")

    with pytest.raises(ValueError):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=1_800_000_000.0, test_only_state_root=state_root
        )
    assert not (config.run_directory / ".control").exists()


@pytest.mark.parametrize(
    ("label", "needle", "replacement"),
    [
        ("course", b"units: meters", b"units: feet\nunits: meters"),
        (
            "scenario",
            b"  fm2_drop_zone: F2",
            b"  fm2_drop_zone: F1\n  fm2_drop_zone: F2",
        ),
    ],
)
def test_duplicate_yaml_keys_fail_before_output(
    tmp_path: Path, label: str, needle: bytes, replacement: bytes
) -> None:
    config, state_root, policy = prepared_config(tmp_path)
    source = (config.run_directory / f"configuration/{label}.yaml").read_bytes()
    assert source.count(needle) == 1
    config = _replace_competition_bytes(
        config,
        policy,
        label=label,
        content=source.replace(needle, replacement),
    )

    with pytest.raises(ValueError, match="YAML"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert not (config.run_directory / ".control").exists()


def test_constructor_failure_retains_forensic_run_output(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    import drone.control.listener_runtime as listener_runtime

    class ConstructorFailure:
        def __init__(self, **_kwargs: object) -> None:
            raise RuntimeError("injected constructor failure")

    monkeypatch.setattr(listener_runtime, "RuntimeConfiguration", ConstructorFailure)
    original_fsync = qgc_runtime_config.os.fsync
    fsync_calls = 0

    def fail_cleanup_fsync(descriptor: int) -> None:
        nonlocal fsync_calls
        fsync_calls += 1
        if fsync_calls == 4:
            raise OSError("injected cleanup fsync failure")
        original_fsync(descriptor)

    monkeypatch.setattr(qgc_runtime_config.os, "fsync", fail_cleanup_fsync)
    with pytest.raises(RuntimeError, match="injected constructor failure"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert (config.run_directory / ".control/comp2026-waypoints.json").is_file()


def test_failure_cleanup_never_renames_unlinks_or_removes_namespace_entries(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    import drone.control.listener_runtime as listener_runtime

    class ConstructorFailure:
        def __init__(self, **_kwargs: object) -> None:
            raise RuntimeError("injected constructor failure")

    monkeypatch.setattr(listener_runtime, "RuntimeConfiguration", ConstructorFailure)
    calls: list[str] = []

    def forbidden(*_args: object, **_kwargs: object) -> None:
        calls.append("namespace mutation")
        raise AssertionError("failure cleanup mutated the namespace")

    monkeypatch.setattr(qgc_runtime_config.os, "rename", forbidden)
    monkeypatch.setattr(qgc_runtime_config.os, "unlink", forbidden)
    monkeypatch.setattr(qgc_runtime_config.os, "rmdir", forbidden)
    with pytest.raises(RuntimeError, match="constructor failure"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert calls == []
    assert (config.run_directory / ".control/comp2026-waypoints.json").is_file()


def test_constructor_race_never_removes_replacement_control_entry(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    import drone.control.listener_runtime as listener_runtime

    replacement = b"replacement owned by another actor"

    class ReplacingConstructor:
        def __init__(self, **_kwargs: object) -> None:
            control = config.run_directory / ".control"
            control.rename(config.run_directory / "moved-control")
            control.mkdir()
            (control / "comp2026-waypoints.json").write_bytes(replacement)
            raise RuntimeError("injected replacement race")

    monkeypatch.setattr(listener_runtime, "RuntimeConfiguration", ReplacingConstructor)
    with pytest.raises(RuntimeError, match="replacement race"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )

    assert (
        config.run_directory / ".control/comp2026-waypoints.json"
    ).read_bytes() == replacement
    assert (
        config.run_directory / "moved-control/comp2026-waypoints.json"
    ).exists()


def test_rollback_does_not_unlink_waypoint_replacement_after_identity_check(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    import drone.control.listener_runtime as listener_runtime

    class ReplacingConstructor:
        def __init__(self, **_kwargs: object) -> None:
            control = config.run_directory / ".control"
            (control / "comp2026-waypoints.json").rename(control / "owned-moved")
            (control / "comp2026-waypoints.json").write_bytes(replacement)
            raise RuntimeError("injected constructor failure")

    monkeypatch.setattr(listener_runtime, "RuntimeConfiguration", ReplacingConstructor)
    replacement = b"replacement owned by another actor"
    with pytest.raises(RuntimeError, match="constructor failure"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )

    assert (
        config.run_directory / ".control/comp2026-waypoints.json"
    ).read_bytes() == replacement


def test_repeated_constructor_failure_does_not_leak_descriptors(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import drone.control.listener_runtime as listener_runtime

    class ConstructorFailure:
        def __init__(self, **_kwargs: object) -> None:
            raise RuntimeError("injected constructor failure")

    monkeypatch.setattr(listener_runtime, "RuntimeConfiguration", ConstructorFailure)
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    for index in range(4):
        case = tmp_path / str(index)
        config, state_root, _ = prepared_config(case)
        with pytest.raises(RuntimeError, match="constructor failure"):
            qgc_runtime_config.project_qgc_runtime(
                config, epoch_seconds=time.time(), test_only_state_root=state_root
            )
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_waypoint_fsync_failure_retains_forensic_output(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    original_fsync = qgc_runtime_config.os.fsync
    calls = 0

    def fail_first_fsync(descriptor: int) -> None:
        nonlocal calls
        calls += 1
        if calls == 1:
            raise OSError("injected fsync failure")
        original_fsync(descriptor)

    monkeypatch.setattr(qgc_runtime_config.os, "fsync", fail_first_fsync)
    with pytest.raises(OSError, match="injected fsync failure"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert (config.run_directory / ".control").is_dir()


def test_waypoint_write_failure_retains_forensic_output(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    original_fdopen = qgc_runtime_config.os.fdopen

    class FailingDestination:
        def __init__(self, destination: object) -> None:
            self.destination = destination

        def __enter__(self) -> "FailingDestination":
            self.destination.__enter__()
            return self

        def __exit__(self, *args: object) -> object:
            return self.destination.__exit__(*args)

        def write(self, _content: bytes) -> None:
            raise OSError("injected write failure")

    def fdopen(descriptor: int, mode: str):
        opened = original_fdopen(descriptor, mode)
        return FailingDestination(opened) if mode == "wb" else opened

    monkeypatch.setattr(qgc_runtime_config.os, "fdopen", fdopen)
    with pytest.raises(OSError, match="injected write failure"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert (config.run_directory / ".control/comp2026-waypoints.json").is_file()


def test_waypoint_fdopen_failure_closes_raw_descriptor_and_rolls_back(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    original_fdopen = qgc_runtime_config.os.fdopen
    before = len(tuple(Path("/proc/self/fd").iterdir()))

    def fail_write_fdopen(descriptor: int, mode: str):
        if mode == "wb":
            raise OSError("injected fdopen failure")
        return original_fdopen(descriptor, mode)

    monkeypatch.setattr(qgc_runtime_config.os, "fdopen", fail_write_fdopen)
    with pytest.raises(OSError, match="fdopen failure"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert (config.run_directory / ".control/comp2026-waypoints.json").is_file()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_waypoint_fstat_failure_tracks_raw_descriptor_and_rolls_back(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    original_fstat = qgc_runtime_config.os.fstat
    failed = False
    before = len(tuple(Path("/proc/self/fd").iterdir()))

    def fail_created_file(descriptor: int):
        nonlocal failed
        metadata = original_fstat(descriptor)
        target = qgc_runtime_config.os.readlink(f"/proc/self/fd/{descriptor}")
        if (
            not failed
            and target.endswith("/.control/comp2026-waypoints.json")
            and metadata.st_size == 0
            and stat.S_ISREG(metadata.st_mode)
        ):
            failed = True
            raise OSError("injected file fstat failure")
        return metadata

    monkeypatch.setattr(qgc_runtime_config.os, "fstat", fail_created_file)
    with pytest.raises(OSError, match="file fstat failure"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert (config.run_directory / ".control/comp2026-waypoints.json").is_file()
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_mkdir_open_replacement_is_not_adopted_or_deleted(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    original_open = qgc_runtime_config.os.open
    replaced = False

    def replace_before_open(path: object, flags: int, *args: object, **kwargs: object):
        nonlocal replaced
        if path == ".control" and not replaced:
            replaced = True
            control = config.run_directory / ".control"
            control.rename(config.run_directory / "displaced-control")
            control.mkdir()
            (control / "replacement").write_text("keep")
        return original_open(path, flags, *args, **kwargs)

    monkeypatch.setattr(qgc_runtime_config.os, "open", replace_before_open)
    with pytest.raises(ValueError, match="control directory"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert (config.run_directory / ".control/replacement").read_text() == "keep"


def test_waypoint_output_close_clears_slots_before_close_error_and_never_retries(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_fd = qgc_runtime_config.os.open(tmp_path, qgc_runtime_config.os.O_RDONLY)
    control_fd = qgc_runtime_config.os.open(tmp_path, qgc_runtime_config.os.O_RDONLY)
    metadata = qgc_runtime_config.os.fstat(control_fd)
    output = qgc_runtime_config._WaypointOutput(
        path=tmp_path / "unused",
        run_fd=run_fd,
        control_fd=control_fd,
        control_identity=(metadata.st_dev, metadata.st_ino),
        file_identity=None,
        file_fd=-1,
        created_file=False,
        created_directory=False,
    )
    original_close = qgc_runtime_config.os.close

    replacement_fd = -1
    close_calls: list[int] = []

    def fail_control(descriptor: int) -> None:
        nonlocal replacement_fd
        close_calls.append(descriptor)
        if descriptor == control_fd:
            original_close(descriptor)
            replacement_fd = qgc_runtime_config.os.open(
                "/dev/null", qgc_runtime_config.os.O_RDONLY
            )
            assert replacement_fd == control_fd
            raise OSError("injected close failure")
        original_close(descriptor)

    monkeypatch.setattr(qgc_runtime_config.os, "close", fail_control)
    with pytest.raises(OSError, match="close failure"):
        output.close()
    assert (output.file_fd, output.control_fd, output.run_fd) == (-1, -1, -1)
    with pytest.raises(OSError):
        qgc_runtime_config.os.fstat(run_fd)
    calls_after_failure = tuple(close_calls)
    output.close()
    assert tuple(close_calls) == calls_after_failure
    assert qgc_runtime_config.os.fstat(replacement_fd).st_ino >= 0
    original_close(replacement_fd)


def test_waypoint_traversal_close_failure_does_not_leak_successor_fd(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_directory = tmp_path / "a" / "b"
    run_directory.mkdir(parents=True)
    original_close = qgc_runtime_config.os.close
    failed = False
    failed_fd = -1
    before = len(tuple(Path("/proc/self/fd").iterdir()))

    def fail_predecessor_once(descriptor: int) -> None:
        nonlocal failed, failed_fd
        target = qgc_runtime_config.os.readlink(f"/proc/self/fd/{descriptor}")
        if not failed and target == str(tmp_path / "a"):
            failed = True
            failed_fd = descriptor
            raise OSError("injected predecessor close failure")
        original_close(descriptor)

    monkeypatch.setattr(qgc_runtime_config.os, "close", fail_predecessor_once)
    with pytest.raises(OSError, match="predecessor close failure"):
        qgc_runtime_config._write_run_owned_waypoints(run_directory, b"{}\n")
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before + 1
    original_close(failed_fd)


def test_waypoint_traversal_never_retries_close_after_descriptor_reuse(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_directory = tmp_path / "a" / "b"
    run_directory.mkdir(parents=True)
    original_close = qgc_runtime_config.os.close
    failed = False
    replacement_fd = -1

    def close_reuse_then_fail(descriptor: int) -> None:
        nonlocal failed, replacement_fd
        target = qgc_runtime_config.os.readlink(f"/proc/self/fd/{descriptor}")
        if not failed and target == str(tmp_path / "a"):
            failed = True
            original_close(descriptor)
            replacement_fd = qgc_runtime_config.os.open(
                "/dev/null", qgc_runtime_config.os.O_RDONLY
            )
            assert replacement_fd == descriptor
            raise OSError("injected reused predecessor close failure")
        original_close(descriptor)

    monkeypatch.setattr(qgc_runtime_config.os, "close", close_reuse_then_fail)
    with pytest.raises(OSError, match="reused predecessor close failure"):
        qgc_runtime_config._write_run_owned_waypoints(run_directory, b"{}\n")

    assert qgc_runtime_config.os.fstat(replacement_fd).st_ino >= 0
    original_close(replacement_fd)


def test_rejects_double_slash_file_alias() -> None:
    with pytest.raises(ValueError, match="canonical"):
        qgc_runtime_config._read_regular_file(
            Path("//tmp/qgc-runtime.json"), label="QGC runtime policy"
        )


def test_rejects_symlinked_artifact_before_run_output(tmp_path: Path) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    policy = config.qgc.runtime_policy_path
    real_policy = policy.with_name("saved-policy.json")
    policy.rename(real_policy)
    policy.symlink_to(real_policy)

    with pytest.raises(ValueError, match="symlink"):
        qgc_runtime_config.project_qgc_runtime(
            config, epoch_seconds=time.time(), test_only_state_root=state_root
        )
    assert not (config.run_directory / ".control").exists()


def test_two_runs_share_external_attempt_state_without_reading_or_changing_ledger(
    tmp_path: Path,
) -> None:
    first, state_root, _ = prepared_config(tmp_path)
    assert first.qgc is not None
    second_directory = tmp_path / "runs" / "00000000-0000-4000-8000-000000000002"
    shutil.copytree(
        first.run_directory / "configuration", second_directory / "configuration"
    )
    second_configuration = second_directory / "configuration"
    second_qgc = replace(
        first.qgc,
        deployment_profile_path=second_configuration / "deployment-profile.json",
        listener_session_path=second_configuration / "listener-session.json",
        qgc_actions_path=second_configuration / "qgc-actions.json",
        runtime_policy_path=second_configuration / "qgc-runtime.json",
    )
    second = replace(
        first,
        run_id="00000000-0000-4000-8000-000000000002",
        run_directory=second_directory,
        course_path=second_configuration / "course.yaml",
        scenario_path=second_configuration / "scenario.yaml",
        qgc=second_qgc,
    )
    ledger_path = state_root / first.qgc.attempt_state_id / "attempt-ledger.json"
    before = (ledger_path.read_bytes(), ledger_path.stat().st_mtime_ns)

    first_projection = qgc_runtime_config.project_qgc_runtime(
        first, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    second_projection = qgc_runtime_config.project_qgc_runtime(
        second, epoch_seconds=time.time(), test_only_state_root=state_root
    )

    assert first_projection.attempt_state == second_projection.attempt_state
    assert (ledger_path.read_bytes(), ledger_path.stat().st_mtime_ns) == before


def test_validated_listener_bundle_survives_source_path_replacement(
    tmp_path: Path,
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    original_session = config.qgc.listener_session_path.read_bytes()
    original_actions = config.qgc.qgc_actions_path.read_bytes()
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )

    alternate = tmp_path / "alternate"
    alternate.mkdir()
    alternate_ledger = alternate / "attempt-ledger.json"
    initialize_ledger(alternate_ledger)
    prepare_attempt(
        config.qgc.deployment_profile_path,
        alternate_ledger,
        alternate / "listener-session.json",
        alternate / "qgc-actions.json",
        acknowledge_on_ground=True,
    )
    config.qgc.listener_session_path.write_bytes(
        (alternate / "listener-session.json").read_bytes()
    )
    config.qgc.qgc_actions_path.write_bytes((alternate / "qgc-actions.json").read_bytes())
    config.qgc.deployment_profile_path.write_bytes(b"{}")
    calls = []

    result = construct_after_full_validation(
        projection.validated_listener_artifacts,
        projection.runtime_configuration,
        component_factory=lambda artifacts, runtime: calls.append(
            (artifacts, runtime)
        )
        or artifacts.prepared_attempt.attempt_id,
    )

    artifacts = projection.validated_listener_artifacts
    assert result == 1
    assert artifacts.session_sha256 == hashlib.sha256(original_session).hexdigest()
    assert artifacts.actions_sha256 == hashlib.sha256(original_actions).hexdigest()
    assert calls == [(artifacts, projection.runtime_configuration)]


def test_projection_uses_secured_bytes_if_paths_change_before_nested_parse(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    import drone.control.listener as listener

    original_loader = listener.load_listener_artifacts_bytes
    original_hashes = (
        config.qgc.deployment_profile_sha256,
        config.qgc.listener_session_sha256,
        config.qgc.qgc_actions_sha256,
    )

    def replace_paths_then_load(*args: object, **kwargs: object):
        config.qgc.deployment_profile_path.write_bytes(b"{}")
        config.qgc.listener_session_path.write_bytes(b"{}")
        config.qgc.qgc_actions_path.write_bytes(b"{}")
        return original_loader(*args, **kwargs)

    monkeypatch.setattr(listener, "load_listener_artifacts_bytes", replace_paths_then_load)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )

    artifacts = projection.validated_listener_artifacts
    assert (
        artifacts.profile_sha256,
        artifacts.session_sha256,
        artifacts.actions_sha256,
    ) == original_hashes


def test_validated_listener_bundle_rechecks_mutable_ledger_before_factory(
    tmp_path: Path,
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    prepare_attempt(
        config.qgc.deployment_profile_path,
        projection.validated_listener_artifacts.ledger.path,
        tmp_path / "later-session.json",
        tmp_path / "later-actions.json",
        acknowledge_on_ground=True,
    )
    calls: list[object] = []

    with pytest.raises(Exception, match="attempt"):
        construct_after_full_validation(
            projection.validated_listener_artifacts,
            projection.runtime_configuration,
            component_factory=lambda artifacts, runtime: calls.append(
                (artifacts, runtime)
            ),
        )
    assert calls == []


def _alternate_prepared_ledger(config: RuntimeConfig, directory: Path) -> Path:
    directory.mkdir()
    ledger = directory / "attempt-ledger.json"
    initialize_ledger(ledger)
    prepare_attempt(
        config.qgc.deployment_profile_path,
        ledger,
        directory / "session.json",
        directory / "actions.json",
        acknowledge_on_ground=True,
    )
    return ledger


def test_listener_admission_rejects_ledger_symlink_substitution(
    tmp_path: Path,
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    artifacts = projection.validated_listener_artifacts
    ledger = artifacts.ledger.path
    alternate = _alternate_prepared_ledger(config, tmp_path / "alternate-ledger")
    preserved = ledger.with_name("preserved-ledger")
    ledger.rename(preserved)
    ledger.symlink_to(alternate)
    calls: list[object] = []

    with pytest.raises(Exception, match="ledger|identity|symlink"):
        construct_after_full_validation(
            artifacts,
            projection.runtime_configuration,
            component_factory=lambda snapshot, runtime: calls.append((snapshot, runtime)),
        )
    assert calls == []
    artifacts.close()


def test_listener_admission_rejects_lock_replacement(tmp_path: Path) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    artifacts = projection.validated_listener_artifacts
    lock_path = artifacts.ledger.path.with_name("attempt-ledger.json.lock")
    lock_path.rename(lock_path.with_name("preserved-lock"))
    lock_path.write_bytes(b"")

    with pytest.raises(Exception, match="lock|identity"):
        construct_after_full_validation(
            artifacts,
            projection.runtime_configuration,
            component_factory=lambda *_args: pytest.fail("factory must not run"),
        )
    artifacts.close()


def test_listener_snapshot_rejects_mutated_ledger_path(tmp_path: Path) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    artifacts = projection.validated_listener_artifacts
    alternate = _alternate_prepared_ledger(config, tmp_path / "alternate-ledger")
    object.__setattr__(artifacts, "_ledger_path", alternate)

    with pytest.raises(Exception, match="altered|ledger"):
        construct_after_full_validation(
            artifacts,
            projection.runtime_configuration,
            component_factory=lambda *_args: pytest.fail("factory must not run"),
        )
    artifacts.close()


def test_pinned_ledger_consumption_preserves_inode(tmp_path: Path) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    artifacts = projection.validated_listener_artifacts
    ledger_path = artifacts.ledger.path
    before = ledger_path.stat()

    artifacts.ledger.consume(artifacts.prepared_attempt.attempt_id)

    after = ledger_path.stat()
    assert (after.st_dev, after.st_ino) == (before.st_dev, before.st_ino)
    assert json.loads(ledger_path.read_text())["consumed_through_attempt_id"] == 1
    artifacts.close()


def test_post_admission_ledger_swap_never_redirects_consumption(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    artifacts = projection.validated_listener_artifacts
    ledger_path = artifacts.ledger.path
    alternate = _alternate_prepared_ledger(config, tmp_path / "alternate-ledger")
    alternate_before = alternate.read_bytes()
    moved = ledger_path.with_name("pinned-ledger-moved")
    import drone.control.mission_supervisor as mission_supervisor

    original_pread = mission_supervisor.os.pread
    swapped = False

    def swap_then_read(*args: object, **kwargs: object):
        nonlocal swapped
        if not swapped:
            swapped = True
            ledger_path.rename(moved)
            shutil.copyfile(alternate, ledger_path)
        return original_pread(*args, **kwargs)

    monkeypatch.setattr(mission_supervisor.os, "pread", swap_then_read)
    artifacts.ledger.consume(artifacts.prepared_attempt.attempt_id)

    assert alternate.read_bytes() == alternate_before
    assert json.loads(moved.read_text())["consumed_through_attempt_id"] == 1
    assert ledger_path.read_bytes() == alternate_before
    artifacts.close()


def test_listener_snapshot_close_releases_all_pinned_descriptors(tmp_path: Path) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    assert len(tuple(Path("/proc/self/fd").iterdir())) > before

    projection.validated_listener_artifacts.close()

    assert len(tuple(Path("/proc/self/fd").iterdir())) == before


def test_same_pinned_ledger_serializes_concurrent_consumers(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    artifacts = projection.validated_listener_artifacts
    ledger = artifacts.ledger
    import drone.control.mission_supervisor as mission_supervisor

    original_read = mission_supervisor._read_ledger_descriptor
    simultaneous_reads = threading.Barrier(2)
    start_barrier = threading.Barrier(2)
    results: list[str] = []

    def synchronized_read(descriptor: int):
        try:
            simultaneous_reads.wait(timeout=0.2)
        except threading.BrokenBarrierError:
            pass
        return original_read(descriptor)

    monkeypatch.setattr(
        mission_supervisor, "_read_ledger_descriptor", synchronized_read
    )

    def consume() -> None:
        start_barrier.wait()
        try:
            ledger.consume(1)
        except Exception as error:
            results.append(str(error))
        else:
            results.append("success")

    threads = [threading.Thread(target=consume) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert results.count("success") == 1
    assert sum("already consumed" in result for result in results) == 1
    projection.validated_listener_artifacts.close()


@pytest.mark.parametrize("slot", ["_directory_fd", "_ledger_fd", "_lock_fd"])
def test_pinned_ledger_rejects_mutated_descriptor_slot_before_consumption(
    tmp_path: Path, slot: str
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    artifacts = projection.validated_listener_artifacts
    ledger = artifacts.ledger
    original_fd = getattr(ledger, slot)
    replacement_fd = qgc_runtime_config.os.open("/dev/null", qgc_runtime_config.os.O_RDWR)
    object.__setattr__(ledger, slot, replacement_fd)

    with pytest.raises(Exception, match="identity|regular|directory"):
        ledger.consume(1)

    assert json.loads(ledger.path.read_text())["consumed_through_attempt_id"] == 0
    artifacts.close()
    qgc_runtime_config.os.close(original_fd)


def test_pinned_ledger_close_clears_slots_before_close_error_and_never_retries(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    artifacts = projection.validated_listener_artifacts
    ledger = artifacts.ledger
    failed_fd = ledger._ledger_fd
    original_close = qgc_runtime_config.os.close
    replacement_fd = -1
    close_calls: list[int] = []

    def close_then_fail(descriptor: int) -> None:
        nonlocal replacement_fd
        close_calls.append(descriptor)
        original_close(descriptor)
        if descriptor == failed_fd:
            replacement_fd = qgc_runtime_config.os.open(
                "/dev/null", qgc_runtime_config.os.O_RDONLY
            )
            assert replacement_fd == failed_fd
            raise OSError("injected pinned close failure")

    monkeypatch.setattr("drone.control.mission_supervisor.os.close", close_then_fail)
    with pytest.raises(OSError, match="pinned close failure"):
        artifacts.close()
    assert (ledger._directory_fd, ledger._ledger_fd, ledger._lock_fd) == (-1, -1, -1)
    calls_after_failure = tuple(close_calls)
    artifacts.close()
    assert tuple(close_calls) == calls_after_failure
    assert qgc_runtime_config.os.fstat(replacement_fd).st_ino >= 0
    original_close(replacement_fd)


def test_pinned_ledger_constructor_final_fstat_failure_closes_all_descriptors_once(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    assert config.qgc is not None
    ledger_path = (
        state_root
        / config.qgc.attempt_state_id
        / "attempt-ledger.json"
    )
    import drone.control.mission_supervisor as mission_supervisor

    original_fstat = mission_supervisor.os.fstat
    original_close = mission_supervisor.os.close
    before = len(tuple(Path("/proc/self/fd").iterdir()))
    regular_fstats = 0
    closed_targets: list[str] = []

    def fail_final_directory_fstat(descriptor: int):
        nonlocal regular_fstats
        metadata = original_fstat(descriptor)
        if stat.S_ISREG(metadata.st_mode):
            regular_fstats += 1
        elif regular_fstats >= 2 and stat.S_ISDIR(metadata.st_mode):
            raise OSError("injected final directory fstat failure")
        return metadata

    def record_close(descriptor: int) -> None:
        closed_targets.append(
            mission_supervisor.os.readlink(f"/proc/self/fd/{descriptor}")
        )
        original_close(descriptor)

    monkeypatch.setattr(mission_supervisor.os, "fstat", fail_final_directory_fstat)
    monkeypatch.setattr(mission_supervisor.os, "close", record_close)
    with pytest.raises(OSError, match="final directory fstat failure"):
        mission_supervisor.PinnedAttemptLedger(ledger_path)
    assert len(tuple(Path("/proc/self/fd").iterdir())) == before
    state_directory = str(ledger_path.parent)
    assert closed_targets.count(str(ledger_path)) == 1
    assert closed_targets.count(f"{ledger_path}.lock") == 1
    assert closed_targets.count(state_directory) == 1


def test_pinned_ledger_rejects_mutated_path_slot_before_consumption(
    tmp_path: Path,
) -> None:
    config, state_root, _ = prepared_config(tmp_path)
    projection = qgc_runtime_config.project_qgc_runtime(
        config, epoch_seconds=time.time(), test_only_state_root=state_root
    )
    artifacts = projection.validated_listener_artifacts
    ledger = artifacts.ledger
    object.__setattr__(ledger, "_path", tmp_path / "alternate/attempt-ledger.json")

    with pytest.raises(Exception, match="directory|unavailable|unsafe"):
        ledger.consume(1)

    assert json.loads(
        (state_root / config.qgc.attempt_state_id / "attempt-ledger.json").read_text()
    )["consumed_through_attempt_id"] == 0
    artifacts.close()
