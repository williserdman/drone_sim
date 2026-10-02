from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import pytest

from artifacts._adapters.rosbag import GroundTruthEvidence, PhysicalBagEvidence, ScoreEventEvidence
from artifacts.score_validation import ScoreValidationError, validate_score_outputs
from artifacts.validation import validate_tree


RUN_ID = "00000000-0000-4000-8000-000000000406"
RULES = Path(__file__).parents[2] / "scorekeeper/rules/calibration_v1.json"


def _evidence(run_directory: Path, *, speed=0.5):
    bag = run_directory / "rosbag"
    bag.mkdir()
    (bag / "data.mcap").write_bytes(b"calibration")
    samples = [GroundTruthEvidence(0, "iris", (25.0, -12.0, 0.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False)]
    samples.append(GroundTruthEvidence(50_000_000, "iris", (25.0, -12.0, 1.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False))
    samples.append(GroundTruthEvidence(100_000_000, "iris", (25.0, -12.0, 0.1), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, -speed), (0.0, 0.0, 0.0), False))
    samples.extend(GroundTruthEvidence(i * 50_000_000, "iris", (25.0, -12.0, 0.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), True) for i in range(3, 14))
    awarded = (20.0, 40.0 if speed <= 1.0 else 0.0, 40.0)
    types = ("calibration.airborne_then_contact", "calibration.safe_preimpact_speed", "calibration.stable_contact", "score.finalized")
    events = tuple(ScoreEventEvidence(650_000_000, i, kind, value, f"scoring/events.jsonl#event-{i}") for i, (kind, value) in enumerate(zip(types, (*awarded, sum(awarded)), strict=True)))
    digest = validate_tree(run_directory, "rosbag").sha256
    return PhysicalBagEvidence(digest, "a" * 64, ("STARTING", "READY", "RUNNING", "FINALIZING"), tuple(samples), events)


def _write_score(run_directory: Path, evidence: PhysicalBagEvidence):
    scoring = run_directory / "scoring"
    scoring.mkdir()
    points = (20.0, 40.0, 40.0)
    values = tuple(event.value for event in evidence.score_events[:3])
    ids = ("airborne_then_contact", "safe_preimpact_speed", "stable_contact")
    rows = [{"run_id": RUN_ID, "sim_timestamp_ns": event.sim_timestamp_ns, "event_id": event.event_id, "event_type": event.event_type, "value": event.value, "evidence_ref": event.evidence_ref} for event in evidence.score_events]
    (scoring / "events.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    result = {
        "run_id": RUN_ID, "ruleset_id": "calibration_v1", "complete": True,
        "achieved_score": sum(values), "maximum_available_score": 100.0,
        "scoring_checksum": hashlib.sha256(RULES.read_bytes()).hexdigest(),
        "evidence_paths": [*(f"scoring/events.jsonl#event-{i}" for i in range(4)), "rosbag#/simulation/ground_truth"],
        "rule_results": [{"rule_id": rule_id, "passed": value > 0, "awarded_points": value, "available_points": available, "evidence_ref": f"rosbag#/simulation/ground_truth:{rule_id}"} for rule_id, value, available in zip(ids, values, points, strict=True)],
        "diagnostic": None,
    }
    (scoring / "result.json").write_text(json.dumps(result))


@pytest.mark.parametrize(("speed", "score"), [(0.5, 100.0), (1.052, 60.0)])
def test_dispatcher_independently_accepts_calibration_scores(tmp_path, speed, score):
    evidence = _evidence(tmp_path, speed=speed)
    _write_score(tmp_path, evidence)
    assert validate_score_outputs(tmp_path, run_id=RUN_ID, rules_path=RULES, physical_evidence=evidence).achieved_score == score


def test_missing_contact_evidence_rejects_persisted_maximum(tmp_path):
    evidence = _evidence(tmp_path)
    _write_score(tmp_path, evidence)
    evidence = PhysicalBagEvidence(evidence.bag_sha256, evidence.config_sha256, evidence.lifecycle_states, tuple(sample for sample in evidence.ground_truth if not sample.in_contact), evidence.score_events)
    with pytest.raises(ScoreValidationError, match="independently"):
        validate_score_outputs(tmp_path, run_id=RUN_ID, rules_path=RULES, physical_evidence=evidence)


def test_calibration_rotates_recorded_body_velocity_to_world_for_tilted_vehicle():
    from artifacts._adapters.rosbag import _rotate_body_to_world

    half = math.sqrt(0.5)
    world = _rotate_body_to_world((0.0, half, 0.0, half), (1.052, 0.0, 0.0))
    assert world == pytest.approx((0.0, 0.0, -1.052))
