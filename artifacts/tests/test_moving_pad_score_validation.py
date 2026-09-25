from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from artifacts._adapters.rosbag import (
    GroundTruthEvidence,
    LandingPadStateEvidence,
    MissionEventEvidence,
    PhysicalBagEvidence,
    ScoreEventEvidence,
)
from artifacts.score_validation import ScoreValidationError, validate_score_outputs
from artifacts.validation import validate_tree


RUN_ID = "00000000-0000-0000-0000-000000000007"
RULES_PATH = Path(__file__).parents[2] / "scorekeeper/rules/moving_pad_v1.json"


def _write_score(run_directory: Path, *, passed: bool = True) -> None:
    scoring = run_directory / "scoring"
    scoring.mkdir()
    checksum = hashlib.sha256(RULES_PATH.read_bytes()).hexdigest()
    awarded = 100.0 if passed else 0.0
    event_rows = (
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 100_000_000,
            "event_id": 0,
            "event_type": "moving_pad.touchdown_offset_m",
            "value": 0.1,
            "evidence_ref": "scoring/events.jsonl#event-0",
        },
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 100_000_000,
            "event_id": 1,
            "event_type": "moving_pad.touchdown_relative_velocity_mps",
            "value": 0.0,
            "evidence_ref": "scoring/events.jsonl#event-1",
        },
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 2_100_000_000,
            "event_id": 2,
            "event_type": "moving_pad.physical_landing",
            "value": awarded,
            "evidence_ref": "scoring/events.jsonl#event-2",
        },
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 2_100_000_000,
            "event_id": 3,
            "event_type": "score.finalized",
            "value": awarded,
            "evidence_ref": "scoring/events.jsonl#event-3",
        },
    )
    evidence_paths = [row["evidence_ref"] for row in event_rows] + [
        "rosbag#/simulation/ground_truth",
        "rosbag#/simulation/landing_pad_state",
        "rosbag#/simulation/mission_events",
    ]
    result = {
        "run_id": RUN_ID,
        "ruleset_id": "moving_pad_v1",
        "complete": True,
        "achieved_score": awarded,
        "maximum_available_score": 100.0,
        "scoring_checksum": checksum,
        "evidence_paths": evidence_paths,
        "rule_results": [
            {
                "rule_id": "physical_landing",
                "passed": passed,
                "awarded_points": awarded,
                "available_points": 100.0,
                "evidence_ref": "rosbag#moving_pad:physical_landing",
            }
        ],
        "diagnostic": None,
    }
    (scoring / "result.json").write_text(json.dumps(result), encoding="utf-8")
    (scoring / "events.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in event_rows), encoding="utf-8"
    )


def _physical_evidence(run_directory: Path, *, outside_deck: bool = False):
    bag = run_directory / "rosbag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("metadata\n", encoding="utf-8")
    (bag / "bag.mcap").write_bytes(b"moving-pad-evidence")
    digest = validate_tree(run_directory, "rosbag").sha256
    assert digest is not None
    ground_truth = []
    pad_truth = []
    for index in range(42):
        timestamp = (index + 1) * 50_000_000
        pad_x = 10.0 + 0.5 * timestamp / 1_000_000_000
        airborne = index == 0
        offset = 2.0 if outside_deck else 0.1
        ground_truth.append(
            GroundTruthEvidence(
                timestamp,
                "iris",
                (pad_x + offset, 0.0, 5.0 if airborne else 0.3),
                (0.0, 0.0, 0.0, 1.0),
                (0.5, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                not airborne,
            )
        )
        pad_truth.append(
            LandingPadStateEvidence(
                timestamp,
                7,
                (pad_x, 0.0, 0.2),
                (0.0, 0.0, 0.0, 1.0),
                (0.5, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                not airborne,
            )
        )
    score_events = (
        ScoreEventEvidence(
            100_000_000,
            0,
            "moving_pad.touchdown_offset_m",
            0.1,
            "scoring/events.jsonl#event-0",
        ),
        ScoreEventEvidence(
            100_000_000,
            1,
            "moving_pad.touchdown_relative_velocity_mps",
            0.0,
            "scoring/events.jsonl#event-1",
        ),
        ScoreEventEvidence(
            2_100_000_000,
            2,
            "moving_pad.physical_landing",
            100.0,
            "scoring/events.jsonl#event-2",
        ),
        ScoreEventEvidence(
            2_100_000_000,
            3,
            "score.finalized",
            100.0,
            "scoring/events.jsonl#event-3",
        ),
    )
    return PhysicalBagEvidence(
        digest,
        "a" * 64,
        ("STARTING", "READY", "RUNNING", "FINALIZING"),
        tuple(ground_truth),
        score_events,
        mission_events=(
            MissionEventEvidence(0, 0, "MOVING_PAD", "ARMED", "observed"),
            MissionEventEvidence(
                100_000_000, 1, "MOVING_PAD", "DISARMED", "observed"
            ),
        ),
        landing_pad_states=tuple(pad_truth),
    )


def _host_rosbag(run_directory: Path) -> None:
    bag = run_directory / "rosbag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("metadata\n", encoding="utf-8")
    (bag / "bag.mcap").write_bytes(b"host-validated-moving-pad-evidence")


def test_host_metadata_validation_accepts_complete_logical_score_without_ros_decoder(
    tmp_path,
):
    _write_score(tmp_path)
    _host_rosbag(tmp_path)

    result = validate_score_outputs(
        tmp_path,
        run_id=RUN_ID,
        rules_path=RULES_PATH,
    )

    assert result.achieved_score == 100.0
    assert result.maximum_available_score == 100.0


def test_host_metadata_validation_rejects_retrospective_touchdown_events(tmp_path):
    _write_score(tmp_path)
    _host_rosbag(tmp_path)
    events_path = tmp_path / "scoring/events.jsonl"
    rows = [json.loads(line) for line in events_path.read_text().splitlines()]
    rows = [rows[index] for index in (2, 0, 1, 3)]
    for index, row in enumerate(rows):
        row["event_id"] = index
        row["evidence_ref"] = f"scoring/events.jsonl#event-{index}"
    events_path.write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
    )

    with pytest.raises(ScoreValidationError, match="logically ordered"):
        validate_score_outputs(tmp_path, run_id=RUN_ID, rules_path=RULES_PATH)


def test_host_metadata_validation_rejects_score_event_value_mismatch(tmp_path):
    _write_score(tmp_path)
    _host_rosbag(tmp_path)
    events_path = tmp_path / "scoring/events.jsonl"
    rows = [json.loads(line) for line in events_path.read_text().splitlines()]
    rows[-1]["value"] = 0.0
    events_path.write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
    )

    with pytest.raises(ScoreValidationError, match="logical"):
        validate_score_outputs(tmp_path, run_id=RUN_ID, rules_path=RULES_PATH)


def test_host_metadata_validation_rejects_pass_without_touchdown_diagnostics(tmp_path):
    _write_score(tmp_path)
    _host_rosbag(tmp_path)
    events_path = tmp_path / "scoring/events.jsonl"
    rows = [json.loads(line) for line in events_path.read_text().splitlines()]
    rows = (
        {**rows[2], "event_id": 0, "evidence_ref": "scoring/events.jsonl#event-0"},
        {**rows[-1], "event_id": 1, "evidence_ref": "scoring/events.jsonl#event-1"},
    )
    events_path.write_text(
        "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
    )
    result_path = tmp_path / "scoring/result.json"
    result = json.loads(result_path.read_text())
    result["evidence_paths"] = [
        "scoring/events.jsonl#event-0",
        "scoring/events.jsonl#event-1",
        *result["evidence_paths"][-3:],
    ]
    result_path.write_text(json.dumps(result), encoding="utf-8")

    with pytest.raises(ScoreValidationError, match="touchdown diagnostics"):
        validate_score_outputs(tmp_path, run_id=RUN_ID, rules_path=RULES_PATH)


def test_host_metadata_validation_rejects_unsafe_rosbag(tmp_path):
    _write_score(tmp_path)
    outside = tmp_path.parent / "outside-rosbag"
    outside.mkdir()
    (outside / "data.mcap").write_bytes(b"outside")
    (tmp_path / "rosbag").symlink_to(outside, target_is_directory=True)

    with pytest.raises(ScoreValidationError, match="unsafe"):
        validate_score_outputs(tmp_path, run_id=RUN_ID, rules_path=RULES_PATH)


def test_dispatcher_accepts_independently_recomputed_moving_pad_physical_pass(tmp_path):
    _write_score(tmp_path)
    evidence = _physical_evidence(tmp_path)

    result = validate_score_outputs(
        tmp_path,
        run_id=RUN_ID,
        rules_path=RULES_PATH,
        physical_evidence=evidence,
    )

    assert result.achieved_score == 100.0
    assert result.maximum_available_score == 100.0
    assert result.evidence_paths[-3:] == (
        "rosbag#/simulation/ground_truth",
        "rosbag#/simulation/landing_pad_state",
        "rosbag#/simulation/mission_events",
    )


def test_moving_pad_validation_rejects_persisted_pass_for_floor_landing(tmp_path):
    _write_score(tmp_path)
    evidence = _physical_evidence(tmp_path, outside_deck=True)

    with pytest.raises(ScoreValidationError, match="independently"):
        validate_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=RULES_PATH,
            physical_evidence=evidence,
        )


def test_moving_pad_validation_rejects_missing_observed_disarm(tmp_path):
    _write_score(tmp_path)
    evidence = _physical_evidence(tmp_path)
    evidence = PhysicalBagEvidence(
        evidence.bag_sha256,
        evidence.config_sha256,
        evidence.lifecycle_states,
        evidence.ground_truth,
        evidence.score_events,
        landing_pad_states=evidence.landing_pad_states,
    )

    with pytest.raises(ScoreValidationError, match="ARMED/DISARMED"):
        validate_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=RULES_PATH,
            physical_evidence=evidence,
        )
