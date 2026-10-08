import hashlib
import json
from pathlib import Path
import shutil
import sys

import pytest

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / "companion/comp2026/src"))

from drone import timebase
from drone.common_types import MissionHome
from drone.control.flight_state import SourceIdentity
from drone.control.listener_runtime import AutopilotVersionContract
from drone_sim_companion.comp2026_policy import build_competition_policy


def resolved_config(tmp_path, **changes):
    configuration = tmp_path / "configuration"
    configuration.mkdir()
    for name in ("course.yaml", "scenario.yaml"):
        shutil.copyfile(ROOT / "config" / name, configuration / name)
    document = json.loads((ROOT / "config/default-run.json").read_text())
    document.update(changes)
    document["competition"] = {
        "course": "course.yaml",
        "scenario": "scenario.yaml",
        "course_sha256": hashlib.sha256((configuration / "course.yaml").read_bytes()).hexdigest(),
        "scenario_sha256": hashlib.sha256((configuration / "scenario.yaml").read_bytes()).hexdigest(),
    }
    path = configuration / "run.json"
    path.write_text(json.dumps(document))
    return path


def test_builds_exact_pinned_simulation_policy(tmp_path):
    policy = build_competition_policy(resolved_config(tmp_path), clock=timebase.monotonic)

    assert policy.flight_profile.companion_target == SourceIdentity(1, 191)
    assert policy.flight_profile.flight_controller == SourceIdentity(1, 1)
    assert policy.flight_profile.firmware == "ArduCopter 4.7.0"
    assert isinstance(policy.autopilot_version_contract, AutopilotVersionContract)
    assert policy.autopilot_version_contract.flight_sw_version == 0x040700FF
    assert policy.autopilot_version_contract.flight_custom_version == b"1511f271"
    assert policy.precision_policy.clearance_calibration is policy.clearance_calibration
    assert policy.precision_policy is not None
    assert policy.recovery_policy.clock is timebase.monotonic
    assert policy.parameter_expectations["PLND_TIMEOUT"] == 0.5
    assert policy.parameter_expectations["FLTMODE_CH"] == 7
    assert policy.parameter_expectations["FLTMODE1"] == 0
    assert policy.parameter_expectations["FLTMODE4"] == 4
    assert policy.parameter_expectations["FLTMODE6"] == 5
    assert policy.parameter_expectations["RNGFND1_TYPE"] == 100
    assert policy.parameter_expectations["RNGFND1_MIN"] == 0.05
    assert policy.parameter_expectations["RNGFND1_MAX"] == 40
    assert [(band.minimum_pwm, band.maximum_pwm, band.mode) for band in policy.flight_profile.rc_mode_bands] == [
        (801, 1230, "STABILIZE"),
        (1491, 1620, "GUIDED"),
        (1750, 2199, "LOITER"),
    ]
    intervals = {request.message_id: request.interval_us for request in policy.flight_profile.telemetry_requests}
    assert intervals[0] == 1_000_000
    assert set(intervals.values()) == {50_000, 1_000_000}
    assert policy.flight_profile.freshness_bounds["heartbeat"] == 2.0
    assert policy.flight_profile.freshness_bounds["mode"] == 2.0
    assert policy.flight_profile.freshness_bounds["armed"] == 2.0
    assert policy.telemetry_policy.command_ack_timeout_s == 60.0
    assert policy.telemetry_policy.collection_timeout_s == 120.0
    assert policy.startup_timeout_s == 1800.0
    assert len(policy.evidence_sha256) == 64


def test_evidence_freezes_competition_parameter_overlay(tmp_path):
    path = resolved_config(tmp_path)
    policy = build_competition_policy(path, clock=timebase.monotonic)
    evidence = hashlib.sha256()
    for frozen in (
        path.parent / "course.yaml",
        path.parent / "scenario.yaml",
        ROOT / "ardupilot_sitl/provenance/ardupilot.json",
        ROOT / "ardupilot_sitl/params/descent.parm",
        ROOT / "ardupilot_sitl/params/competition.parm",
        ROOT / "gazebo/resources/models/iris_competition/model.sdf",
        ROOT / "gazebo/resources/worlds/competition_mission.sdf",
    ):
        evidence.update(frozen.read_bytes())
    assert policy.evidence_sha256 == evidence.hexdigest()


@pytest.mark.parametrize(
    ("field", "value"),
    [("mission", "configured"), ("world", "phase3_foundation"),
     ("vehicle", "iris_flight"), ("scenario", "descent_v1")],
)
def test_rejects_wrong_simulation_binding(tmp_path, field, value):
    with pytest.raises(ValueError, match=field):
        build_competition_policy(resolved_config(tmp_path, **{field: value}), clock=timebase.monotonic)


def test_accepts_realtime_world_selector(tmp_path):
    policy = build_competition_policy(
        resolved_config(tmp_path, world="competition_mission_1x"),
        clock=timebase.monotonic,
    )
    assert policy.world == "competition_mission_1x"


def test_rejects_tampered_frozen_competition_input(tmp_path):
    path = resolved_config(tmp_path)
    (path.parent / "course.yaml").write_text("tampered\n")
    with pytest.raises(ValueError, match="course.*hash"):
        build_competition_policy(path, clock=timebase.monotonic)


def test_rejects_modified_input_even_when_manifest_hash_is_rewritten(tmp_path):
    path = resolved_config(tmp_path)
    scenario = path.parent / "scenario.yaml"
    scenario.write_text(scenario.read_text().replace("seed: 2026", "seed: 2027"))
    document = json.loads(path.read_text())
    document["competition"]["scenario_sha256"] = hashlib.sha256(scenario.read_bytes()).hexdigest()
    path.write_text(json.dumps(document))
    with pytest.raises(ValueError, match="checked-in scenario"):
        build_competition_policy(path, clock=timebase.monotonic)


def test_rejects_non_shared_or_nonfinite_clock(tmp_path):
    with pytest.raises(ValueError, match="shared timebase"):
        build_competition_policy(resolved_config(tmp_path), clock=lambda: float("nan"))


def test_home_check_rejects_outside_course_home_and_nonfinite(tmp_path):
    policy = build_competition_policy(resolved_config(tmp_path), clock=timebase.monotonic)
    assert policy.mission_home_check(policy.home) is None
    assert policy.mission_home_check(MissionHome(policy.home.lat, policy.home.lon, 0.1956)) is None
    with pytest.raises(ValueError, match="home zone"):
        policy.mission_home_check(MissionHome(policy.home.lat + 0.001, policy.home.lon, 0.0))
    with pytest.raises(ValueError, match="finite"):
        policy.mission_home_check(MissionHome(float("nan"), policy.home.lon, 0.0))
