from __future__ import annotations

import hashlib
import json
import math
from dataclasses import replace

import pytest


RUN_ID = "01234567-89ab-cdef-0123-456789abcdef"
ROOT = __import__("pathlib").Path(__file__).parents[2]


def test_autonomy_ignores_disarmed_preflight_mode_but_not_failsafe():
    from artifacts._adapters.rosbag import (
        MissionEventEvidence,
        PhysicalBagEvidence,
        VehicleStatusEvidence,
    )
    from artifacts.vfs_score_validation import _autonomous

    evidence = PhysicalBagEvidence(
        "a" * 64,
        "b" * 64,
        (),
        (),
        (),
        mission_events=(MissionEventEvidence(0, 0, "FM1", "STARTED", ""),),
        vehicle_statuses=(
            VehicleStatusEvidence(1, 0, "iris", False, "STABILIZE", False, ""),
            VehicleStatusEvidence(2, 1, "iris", True, "GUIDED", False, ""),
        ),
    )

    assert _autonomous("FM1", evidence, 3)
    failed = replace(
        evidence,
        vehicle_statuses=(
            replace(
                evidence.vehicle_statuses[0],
                failsafe_active=True,
                failsafe_reason="EKF",
            ),
            evidence.vehicle_statuses[1],
        ),
    )
    assert not _autonomous("FM1", failed, 3)


def test_tilted_payload_projection_does_not_forge_circle_contact():
    from artifacts._adapters.rosbag import PayloadStateEvidence
    from artifacts.vfs_contract import load_vfs_course, load_vfs_scenario
    from artifacts.vfs_score_validation import _payload_contacts_target

    course = load_vfs_course(ROOT / "config/course-2027.yaml")
    scenario = load_vfs_scenario(ROOT / "config/scenario-2027.yaml")
    target = course.waypoints["C"]
    half_angle = math.pi / 8
    tilt = 2 * half_angle
    center_z = (
        math.sin(tilt) * scenario.payload_geometry.size_m[0] / 2
        + math.cos(tilt) * scenario.payload_geometry.size_m[2] / 2
    )
    sample = PayloadStateEvidence(
        1,
        2,
        (target.x + target.radius + 0.05, target.y, center_z),
        (0.0, math.sin(half_angle), 0.0, math.cos(half_angle)),
        (0.0, 0.0, 0.0),
        True,
        False,
    )

    assert not _payload_contacts_target(sample, course, scenario)


def test_vfs_score_dispatch_requires_independently_decoded_physical_evidence(tmp_path):
    from artifacts.score_validation import ScoreValidationError, validate_score_outputs

    scoring = tmp_path / "scoring"
    scoring.mkdir()
    (scoring / "result.json").write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "ruleset_id": "competition_2027_v1",
                "complete": True,
                "achieved_score": 365.0,
                "maximum_available_score": 380.0,
                "scoring_checksum": "0" * 64,
                "evidence_paths": [],
                "rule_results": [],
                "diagnostic": None,
            }
        )
    )
    rules = tmp_path / "rules.json"
    rules.write_text("{}")

    with pytest.raises(ScoreValidationError, match="decoded VFS physical evidence"):
        validate_score_outputs(tmp_path, run_id=RUN_ID, rules_path=rules)


def test_vfs_rules_checksum_is_verified_before_score_is_trusted(tmp_path):
    from artifacts._adapters.rosbag import PhysicalBagEvidence
    from artifacts.score_validation import ScoreValidationError, validate_score_outputs

    scoring = tmp_path / "scoring"
    scoring.mkdir()
    rules = tmp_path / "rules.json"
    rules.write_bytes((ROOT / "scorekeeper/rules/competition_2027_v1.json").read_bytes())
    (scoring / "result.json").write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "ruleset_id": "competition_2027_v1",
                "complete": True,
                "achieved_score": 365.0,
                "maximum_available_score": 380.0,
                "scoring_checksum": hashlib.sha256(b"different").hexdigest(),
                "evidence_paths": [],
                "rule_results": [],
                "diagnostic": None,
            }
        )
    )
    evidence = PhysicalBagEvidence("a" * 64, "b" * 64, (), (), ())

    with pytest.raises(ScoreValidationError, match="checksum"):
        validate_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=rules,
            physical_evidence=evidence,
        )


def test_independent_replay_uses_configured_a_b_c_targets(tmp_path):
    import yaml
    from artifacts._adapters.rosbag import (
        JudgeDecisionEvidence,
        GroundTruthEvidence,
        MissionEventEvidence,
        PayloadEventEvidence,
        PayloadStateEvidence,
        PhysicalBagEvidence,
        VehicleStatusEvidence,
    )
    from artifacts.score_validation import ScoreValidationError
    from artifacts.vfs_score_validation import _independent_values

    second = 1_000_000_000

    configuration = tmp_path / "configuration"
    configuration.mkdir()
    course_payload = (ROOT / "config/course-2027.yaml").read_bytes()
    scenario = yaml.safe_load((ROOT / "config/scenario-2027.yaml").read_bytes())
    scenario["mission"]["targets"] = {2: "A", 3: "B", 4: "C"}
    scenario_payload = yaml.safe_dump(scenario, sort_keys=False).encode()
    (configuration / "course.yaml").write_bytes(course_payload)
    (configuration / "scenario.yaml").write_bytes(scenario_payload)
    (configuration / "run.json").write_text(
        json.dumps(
            {
                "competition": {
                    "course_sha256": hashlib.sha256(course_payload).hexdigest(),
                    "scenario_sha256": hashlib.sha256(scenario_payload).hexdigest(),
                }
            }
        )
    )
    mission = tuple(
        MissionEventEvidence(stamp, index, phase, "STARTED", "")
        for index, (stamp, phase) in enumerate(
            ((1 * second, "FM1"), (11 * second, "FM2"), (21 * second, "FM3")), 1
        )
    )
    decisions = (
        JudgeDecisionEvidence(10 * second, 0, "r0", 3, "FM1", "H", 1, (), 8 * second, "ok", ("m#3",)),
        JudgeDecisionEvidence(20 * second, 1, "r1", 4, "FM2", "", 1, (2, 3, 4), 17_100_000_000, "ok", ("m#4",)),
        JudgeDecisionEvidence(30 * second, 2, "r2", 5, "FM3", "", 1, (), 25_100_000_000, "ok", ("m#5",)),
    )
    statuses = (
        VehicleStatusEvidence(1_500_000_000, 0, "iris", False, "STABILIZE", False, ""),
        VehicleStatusEvidence(5 * second, 1, "iris", True, "GUIDED", False, ""),
        VehicleStatusEvidence(15 * second, 2, "iris", True, "GUIDED", False, ""),
        VehicleStatusEvidence(19_100_000_000, 3, "iris", False, "LAND", False, ""),
        VehicleStatusEvidence(25 * second, 4, "iris", True, "GUIDED", False, ""),
        VehicleStatusEvidence(30_100_000_000, 5, "iris", False, "LAND", False, ""),
    )
    targets = {2: (-86.106, 3.048), 3: (-86.106, 0.0), 4: (-86.106, -3.048)}
    payloads = tuple(
        PayloadStateEvidence(stamp, marker, (*targets[marker], 0.0381), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), True, False)
        for marker in (2, 3, 4)
        for stamp in (18 * second, 19_950_000_000)
    ) + tuple(
        PayloadStateEvidence(30 * second, marker, (-30.48, float(marker - 3), 0.0381), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), True, False)
        for marker in (2, 3, 4)
    )
    evidence = PhysicalBagEvidence(
        "a" * 64,
        "b" * 64,
        (),
        (
            GroundTruthEvidence(
                0, "iris", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), True,
            ),
            GroundTruthEvidence(
                2 * second, "iris", (0.0, 0.0, 10.0), (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False,
            ),
            GroundTruthEvidence(
                10 * second, "iris", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), True,
            ),
            GroundTruthEvidence(
                12 * second, "iris", (0.0, 0.0, 9.5), (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False,
            ),
            GroundTruthEvidence(
                19 * second, "iris", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), True,
            ),
            GroundTruthEvidence(
                22 * second, "iris", (0.0, 0.0, 10.0), (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False,
            ),
            GroundTruthEvidence(
                30 * second, "iris", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), True,
            ),
        ),
        (),
        payload_states=payloads,
        payload_events=(PayloadEventEvidence(12 * second, 0, 2, "release-2", "release", "detached", "OK"),),
        mission_events=mission,
        vehicle_statuses=statuses,
        judge_decisions=decisions,
    )
    rules = json.loads((ROOT / "scorekeeper/rules/competition_2027_v1.json").read_text())

    assert _independent_values(tmp_path, evidence, rules) == (
        65.0, 45.0, 90.0, 1.5, 15.0, 15.0, 15.0, 0.0,
    )

    forged = replace(
        evidence,
        payload_states=tuple(
            sample
            for sample in evidence.payload_states
            if not (sample.aruco_id == 4 and sample.sim_timestamp_ns < 20 * second)
        ),
    )
    with pytest.raises(ScoreValidationError, match="eligibility"):
        _independent_values(tmp_path, forged, rules)

    forged_clock = replace(
        evidence,
        judge_decisions=(
            decisions[0],
            decisions[1],
            replace(decisions[2], active_elapsed_ns=25_200_000_000),
        ),
    )
    with pytest.raises(ScoreValidationError, match="active clock"):
        _independent_values(tmp_path, forged_clock, rules)
