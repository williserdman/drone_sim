import hashlib
import json
from pathlib import Path
import shutil
import sys

import pytest

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / "companion/comp2026/src"))

from drone import timebase
from drone_sim_companion.comp2026_policy import MissionHome
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

    assert policy.flight_sw_version == 0x040603FF
    assert policy.flight_custom_version == b"92b0cd78"
    assert policy.parameter_expectations["PLND_TIMEOUT"] == 0.5
    assert policy.parameter_expectations["FLTMODE_CH"] == 7
    assert policy.parameter_expectations["FLTMODE1"] == 0
    assert policy.parameter_expectations["FLTMODE4"] == 4
    assert policy.parameter_expectations["FLTMODE6"] == 5
    assert policy.parameter_expectations["RNGFND1_TYPE"] == 100
    assert policy.parameter_expectations["RNGFND1_MIN_CM"] == 5
    assert policy.parameter_expectations["RNGFND1_MAX_CM"] == 4000
    assert dict(policy.telemetry_intervals_us)[0] == 100_000
    assert policy.startup_timeout_s == 1800.0
    assert len(policy.evidence_sha256) == 64
    assert not hasattr(policy, "precision_policy")


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
