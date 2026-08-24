from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest


RUN_ID = "00000000-0000-4000-8000-000000000606"
RULES_PATH = Path(__file__).parents[2] / "scorekeeper/rules/descent_v1.json"


def _write_valid_partial_score(run_directory: Path) -> None:
    scoring = run_directory / "scoring"
    scoring.mkdir(parents=True)
    (run_directory / "rosbag").mkdir()
    (run_directory / "rosbag/data.mcap").write_bytes(b"mcap")
    rule_ids = (
        "airborne_then_contact",
        "touchdown_precision",
        "safe_preimpact_speed",
        "stable_contact",
    )
    event_types = tuple(f"descent.{rule_id}" for rule_id in rule_ids)
    points = (20.0, 40.0, 20.0, 20.0)
    awarded = (20.0, 40.0, 0.0, 0.0)
    events = [
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 2_000_000_000,
            "event_id": index,
            "event_type": event_type,
            "value": value,
            "evidence_ref": f"scoring/events.jsonl#event-{index}",
        }
        for index, (event_type, value) in enumerate(zip(event_types, awarded, strict=True))
    ]
    events.append(
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 2_000_000_000,
            "event_id": 4,
            "event_type": "score.finalized",
            "value": 60.0,
            "evidence_ref": "scoring/events.jsonl#event-4",
        }
    )
    (scoring / "events.jsonl").write_text(
        "".join(json.dumps(event, sort_keys=True) + "\n" for event in events),
        encoding="utf-8",
    )
    checksum = hashlib.sha256(RULES_PATH.read_bytes()).hexdigest()
    result = {
        "run_id": RUN_ID,
        "ruleset_id": "descent_v1",
        "complete": True,
        "achieved_score": 60.0,
        "maximum_available_score": 100.0,
        "scoring_checksum": checksum,
        "evidence_paths": [
            *(f"scoring/events.jsonl#event-{index}" for index in range(5)),
            "rosbag#/simulation/ground_truth",
        ],
        "rule_results": [
            {
                "rule_id": rule_id,
                "passed": value > 0,
                "awarded_points": value,
                "available_points": available,
                "evidence_ref": f"rosbag#/simulation/ground_truth:{rule_id}",
            }
            for rule_id, value, available in zip(rule_ids, awarded, points, strict=True)
        ],
        "diagnostic": None,
    }
    (scoring / "result.json").write_text(json.dumps(result), encoding="utf-8")


def test_valid_complete_partial_descent_score_is_accepted(tmp_path):
    from artifacts.score_validation import validate_descent_score_outputs

    _write_valid_partial_score(tmp_path)

    metadata = validate_descent_score_outputs(
        tmp_path,
        run_id=RUN_ID,
        rules_path=RULES_PATH,
    )

    assert metadata.achieved_score == 60.0
    assert metadata.maximum_available_score == 100.0
    assert metadata.scoring_checksum == hashlib.sha256(RULES_PATH.read_bytes()).hexdigest()
    assert metadata.evidence_paths == (
        "scoring/events.jsonl#event-0",
        "scoring/events.jsonl#event-1",
        "scoring/events.jsonl#event-2",
        "scoring/events.jsonl#event-3",
        "scoring/events.jsonl#event-4",
        "rosbag#/simulation/ground_truth",
    )


def _mutate_result(run_directory: Path, mutation) -> None:
    path = run_directory / "scoring/result.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    mutation(document)
    path.write_text(json.dumps(document), encoding="utf-8")


@pytest.mark.parametrize(
    "mutation",
    [
        lambda document: document.update(run_id="another-run"),
        lambda document: document.update(ruleset_id="descent_v2"),
        lambda document: document.update(complete=False),
        lambda document: document.update(maximum_available_score=99.0),
        lambda document: document.update(scoring_checksum="0" * 64),
        lambda document: document.update(rule_results=document["rule_results"][:3]),
        lambda document: document["evidence_paths"].pop(),
    ],
    ids=(
        "wrong-run",
        "wrong-ruleset",
        "incomplete",
        "wrong-maximum",
        "wrong-rules-checksum",
        "missing-rule-result",
        "missing-safe-evidence-reference",
    ),
)
def test_invalid_descent_score_result_is_rejected(tmp_path, mutation):
    from artifacts.score_validation import ScoreValidationError, validate_descent_score_outputs

    _write_valid_partial_score(tmp_path)
    _mutate_result(tmp_path, mutation)

    with pytest.raises(ScoreValidationError):
        validate_descent_score_outputs(tmp_path, run_id=RUN_ID, rules_path=RULES_PATH)


@pytest.mark.parametrize(
    "mutation",
    [
        lambda events: events.pop(),
        lambda events: events[1].update(event_id=7),
        lambda events: events[1].update(event_type="score.unknown"),
        lambda events: events[1].update(run_id="another-run"),
    ],
    ids=("missing-event", "noncontiguous-event-id", "wrong-order", "wrong-run"),
)
def test_invalid_descent_score_events_are_rejected(tmp_path, mutation):
    from artifacts.score_validation import ScoreValidationError, validate_descent_score_outputs

    _write_valid_partial_score(tmp_path)
    path = tmp_path / "scoring/events.jsonl"
    events = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    mutation(events)
    path.write_text(
        "".join(json.dumps(event) + "\n" for event in events), encoding="utf-8"
    )

    with pytest.raises(ScoreValidationError):
        validate_descent_score_outputs(tmp_path, run_id=RUN_ID, rules_path=RULES_PATH)
