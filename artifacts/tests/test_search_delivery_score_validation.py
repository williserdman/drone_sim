from __future__ import annotations

from dataclasses import replace
import hashlib
import json
from pathlib import Path

import pytest

from artifacts._adapters.rosbag import (
    GroundTruthEvidence,
    MissionEventEvidence,
    PayloadEventEvidence,
    PayloadStateEvidence,
    PhysicalBagEvidence,
    ScoreEventEvidence,
)
from artifacts.validation import validate_tree


RUN_ID = "11111111-1111-4111-8111-111111111111"
RULES_PATH = Path(__file__).parents[2] / "scorekeeper/rules/search_delivery_v1.json"
DT = 50_000_000
WA = (18.0, 8.0)
F2 = (6.0, 20.0)
RULE_IDS = ("search", "pickup", "delivery", "home")


def _mission_events() -> tuple[MissionEventEvidence, ...]:
    rows = (
        (0, "SEARCH", "STARTED"),
        (1_500_000_000, "SEARCH", "COMPLETE"),
        (1_550_000_000, "DELIVERY", "STARTED"),
        (7_000_000_000, "DELIVERY", "COMPLETE"),
        (7_050_000_000, "HOME", "STARTED"),
        (7_150_000_000, "HOME", "DISARMED"),
        (7_200_000_000, "HOME", "COMPLETE"),
    )
    return tuple(
        MissionEventEvidence(stamp, event_id, phase, state, "automatic attempt")
        for event_id, (stamp, phase, state) in enumerate(rows)
    )


def _payload_events() -> tuple[PayloadEventEvidence, ...]:
    return (
        PayloadEventEvidence(
            1_450_000_000, 0, 3, "run:3:attach:0", "attach", "attached", "OK"
        ),
        PayloadEventEvidence(
            4_000_000_000, 1, 3, "run:3:release:1", "release", "detached", "OK"
        ),
    )


def _physical_evidence(
    run_directory: Path, *, first_sample_ns: int = 0
) -> PhysicalBagEvidence:
    ground_truth: list[GroundTruthEvidence] = []
    payload_states: list[PayloadStateEvidence] = []
    for stamp in range(first_sample_ns, 7_200_000_000 + DT, DT):
        if stamp < 500_000_000:
            vehicle_xy, vehicle_z, contact = (0.0, 0.0), 0.0, True
        elif stamp < 600_000_000:
            vehicle_xy, vehicle_z, contact = (16.0, 8.0), 4.572, False
        elif stamp < 1_050_000_000:
            vehicle_xy, vehicle_z, contact = (16.8, 8.0), 4.572, False
        elif stamp <= 1_500_000_000:
            vehicle_xy, vehicle_z, contact = WA, 0.0, True
        elif stamp < 2_000_000_000:
            vehicle_xy, vehicle_z, contact = WA, 1.0, False
        elif stamp <= 7_050_000_000:
            vehicle_xy, vehicle_z, contact = F2, 10.0, False
        else:
            vehicle_xy, vehicle_z, contact = (0.0, 0.0), 0.0, True
        ground_truth.append(
            GroundTruthEvidence(
                stamp,
                "iris_search_delivery",
                (*vehicle_xy, vehicle_z),
                (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                contact,
            )
        )

        attached = 1_450_000_000 <= stamp <= 4_000_000_000
        if stamp < 1_550_000_000:
            payload_xy, payload_z, grounded = WA, 0.0254, not attached
        elif stamp <= 4_000_000_000:
            payload_xy, payload_z, grounded = vehicle_xy, vehicle_z, False
        elif stamp < 4_500_000_000:
            payload_xy, payload_z, grounded = F2, 1.0, False
        else:
            payload_xy, payload_z, grounded = F2, 0.0254, True
        payload_states.append(
            PayloadStateEvidence(
                stamp,
                3,
                (*payload_xy, payload_z),
                (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0),
                grounded,
                attached,
            )
        )

    end = 7_200_000_000
    score_events = tuple(
        ScoreEventEvidence(
            end,
            index,
            f"search_delivery.{rule_id}",
            25.0,
            f"scoring/events.jsonl#event-{index}",
        )
        for index, rule_id in enumerate(RULE_IDS)
    ) + (
        ScoreEventEvidence(
            end, 4, "score.finalized", 100.0, "scoring/events.jsonl#event-4"
        ),
    )
    bag_sha = validate_tree(run_directory, "rosbag").sha256
    assert bag_sha is not None
    return PhysicalBagEvidence(
        bag_sha,
        "a" * 64,
        ("STARTING", "READY", "RUNNING", "FINALIZING"),
        tuple(ground_truth),
        score_events,
        tuple(payload_states),
        _payload_events(),
        _mission_events(),
    )


def _fixture(tmp_path: Path, *, first_sample_ns: int = 0) -> PhysicalBagEvidence:
    (tmp_path / "rosbag").mkdir()
    (tmp_path / "rosbag/data.mcap").write_bytes(b"search delivery evidence")
    scoring = tmp_path / "scoring"
    scoring.mkdir()
    checksum = hashlib.sha256(RULES_PATH.read_bytes()).hexdigest()
    events = [
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 7_200_000_000,
            "event_id": index,
            "event_type": f"search_delivery.{rule_id}",
            "value": 25.0,
            "evidence_ref": f"scoring/events.jsonl#event-{index}",
        }
        for index, rule_id in enumerate(RULE_IDS)
    ]
    events.append(
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 7_200_000_000,
            "event_id": 4,
            "event_type": "score.finalized",
            "value": 100.0,
            "evidence_ref": "scoring/events.jsonl#event-4",
        }
    )
    (scoring / "events.jsonl").write_text(
        "".join(json.dumps(event) + "\n" for event in events), encoding="utf-8"
    )
    evidence_paths = [
        *(f"scoring/events.jsonl#event-{index}" for index in range(5)),
        "rosbag#/simulation/ground_truth",
        "rosbag#/simulation/payload_state",
        "rosbag#/simulation/payload_events",
        "rosbag#/simulation/mission_events",
    ]
    result = {
        "run_id": RUN_ID,
        "ruleset_id": "search_delivery_v1",
        "complete": True,
        "achieved_score": 100.0,
        "maximum_available_score": 100.0,
        "scoring_checksum": checksum,
        "evidence_paths": evidence_paths,
        "rule_results": [
            {
                "rule_id": rule_id,
                "passed": True,
                "awarded_points": 25.0,
                "available_points": 25.0,
                "evidence_ref": f"rosbag#search_delivery:{rule_id}",
            }
            for rule_id in RULE_IDS
        ],
        "diagnostic": None,
    }
    (scoring / "result.json").write_text(json.dumps(result), encoding="utf-8")
    return _physical_evidence(tmp_path, first_sample_ns=first_sample_ns)


def test_independent_search_delivery_validation_accepts_valid_bundle(tmp_path):
    """The artifact validator must derive 100/100 without importing the live scorer."""
    from artifacts.search_delivery_score_validation import (
        validate_search_delivery_score_outputs,
    )

    metadata = validate_search_delivery_score_outputs(
        tmp_path,
        run_id=RUN_ID,
        rules_path=RULES_PATH,
        physical_evidence=_fixture(tmp_path),
    )

    assert metadata.achieved_score == metadata.maximum_available_score == 100.0
    assert metadata.elapsed_simulated_ns == 7_200_000_000


def test_independent_validation_accepts_physical_grid_starting_at_50_ms(tmp_path):
    """The public 20 Hz grid is (0, duration], after SEARCH starts at zero."""
    from artifacts.search_delivery_score_validation import (
        validate_search_delivery_score_outputs,
    )

    metadata = validate_search_delivery_score_outputs(
        tmp_path,
        run_id=RUN_ID,
        rules_path=RULES_PATH,
        physical_evidence=_fixture(tmp_path, first_sample_ns=DT),
    )

    assert metadata.achieved_score == metadata.maximum_available_score == 100.0


def test_independent_validation_rejects_first_physical_sample_at_100_ms(tmp_path):
    """Allowing the normal first tick must not hide a missing 50 ms sample."""
    from artifacts.search_delivery_score_validation import (
        SearchDeliveryScoreValidationError,
        validate_search_delivery_score_outputs,
    )

    with pytest.raises(SearchDeliveryScoreValidationError, match="independent"):
        validate_search_delivery_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=RULES_PATH,
            physical_evidence=_fixture(tmp_path, first_sample_ns=2 * DT),
        )


def test_independent_validation_rejects_internal_physical_grid_gap(tmp_path):
    """A missing later tick remains invalid after admitting the 50 ms first tick."""
    from artifacts.search_delivery_score_validation import (
        SearchDeliveryScoreValidationError,
        validate_search_delivery_score_outputs,
    )

    evidence = _fixture(tmp_path, first_sample_ns=DT)
    missing_stamp = 2_500_000_000
    ground_truth = tuple(
        row for row in evidence.ground_truth if row.sim_timestamp_ns != missing_stamp
    )

    with pytest.raises(SearchDeliveryScoreValidationError, match="independent"):
        validate_search_delivery_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=RULES_PATH,
            physical_evidence=replace(evidence, ground_truth=ground_truth),
        )


def test_independent_search_delivery_validation_rejects_missing_search_motion(tmp_path):
    """Persisted 25 search points cannot replace staging and eastward motion."""
    from artifacts.search_delivery_score_validation import (
        SearchDeliveryScoreValidationError,
        validate_search_delivery_score_outputs,
    )

    evidence = _fixture(tmp_path)
    without_staging = tuple(
        replace(sample, position_xyz=(15.6, 8.0, sample.position_xyz[2]))
        if 500_000_000 <= sample.sim_timestamp_ns < 600_000_000
        else sample
        for sample in evidence.ground_truth
    )

    with pytest.raises(SearchDeliveryScoreValidationError, match="independent"):
        validate_search_delivery_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=RULES_PATH,
            physical_evidence=replace(evidence, ground_truth=without_staging),
        )


def test_score_dispatch_selects_search_delivery_validator(tmp_path):
    """A new result must not fall through to the unsupported-ruleset branch."""
    from artifacts.score_validation import validate_score_outputs

    metadata = validate_score_outputs(
        tmp_path,
        run_id=RUN_ID,
        rules_path=RULES_PATH,
        physical_evidence=_fixture(tmp_path),
    )

    assert metadata.achieved_score == 100.0
