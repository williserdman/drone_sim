from __future__ import annotations

import json
from pathlib import Path

import pytest

from drone_sim_scorekeeper.descent import DescentScorer, GroundTruthSample, load_descent_rules
from drone_sim_scorekeeper.output import persist_score_outputs


RUN_ID = "11111111-1111-4111-8111-111111111111"
RULES = Path(__file__).parents[1] / "rules/descent_v1.json"


def _zero_result():
    scorer = DescentScorer(
        RUN_ID, load_descent_rules(RULES), expected_ground_truth_samples=11
    )
    for index in range(11):
        scorer.accept(
            GroundTruthSample(
                RUN_ID,
                index * 50_000_000,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                False,
            )
        )
    return scorer.finalize()


def test_persisted_result_matches_manifest_scoring_contract(tmp_path):
    """Renaming a summary field would make the manifest silently lose score provenance."""
    result = _zero_result()

    paths = persist_score_outputs(tmp_path, result)
    document = json.loads(paths.result.read_text(encoding="utf-8"))
    events = [json.loads(line) for line in paths.events.read_text(encoding="utf-8").splitlines()]

    assert document["ruleset_id"] == "descent_v1"
    assert document["complete"] is True
    assert document["achieved_score"] == 0.0
    assert document["maximum_available_score"] == 100.0
    assert document["scoring_checksum"] == result.scoring_checksum
    assert document["evidence_paths"] == [
        "scoring/events.jsonl#event-0",
        "scoring/events.jsonl#event-1",
        "scoring/events.jsonl#event-2",
        "scoring/events.jsonl#event-3",
        "scoring/events.jsonl#event-4",
        "rosbag#/simulation/ground_truth",
    ]
    assert len(events) == 5
    assert events[-1]["event_type"] == "score.finalized"
    assert result.finished_status() == {
        "run_id": RUN_ID,
        "complete": True,
        "ruleset_id": "descent_v1",
        "achieved_score": 0.0,
        "maximum_available_score": 100.0,
        "scoring_checksum": result.scoring_checksum,
        "result_path": "scoring/result.json",
    }


def test_score_outputs_never_overwrite_existing_evidence(tmp_path):
    """A retry must not replace evidence already eligible for manifest hashing."""
    result = _zero_result()
    persist_score_outputs(tmp_path, result)

    with pytest.raises(FileExistsError):
        persist_score_outputs(tmp_path, result)
