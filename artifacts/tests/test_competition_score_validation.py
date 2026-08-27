from __future__ import annotations

from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import pytest

import artifacts._adapters.rosbag as rosbag_module
from artifacts._adapters.rosbag import (
    GroundTruthEvidence,
    PhysicalBagEvidence,
    ScoreEventEvidence,
)
from artifacts.validation import validate_tree


RUN_ID = "00000000-0000-4000-8000-000000000606"
RULES_PATH = Path(__file__).parents[2] / "scorekeeper/rules/competition_v1.json"
DT = 50_000_000
START = 900_000_000_000
F2 = (-152.40, 0.0)
WA = (-45.72, -9.144)
WM = (-45.72, 9.144)
POINTS = (20.0, 30.0, 10.0, 20.0, 15.0, 50.0, 5.0)
RULE_IDS = (
    "fm1_landing",
    "fm1_autonomy",
    "payload_2",
    "fm2_autonomy",
    "payload_3",
    "fm3_autonomy",
    "payload_4",
)


def _at(seconds: float) -> int:
    return START + int(seconds * 1_000_000_000)


def _mission_events() -> tuple[object, ...]:
    rows = (
        (0.0, "FM1", "STARTED"),
        (1.0, "FM1", "COMPLETE"),
        (2.0, "FM2", "STARTED"),
        (5.05, "FM2", "COMPLETE"),
        (8.0, "FM3_3", "STARTED"),
        (12.05, "FM3_3", "COMPLETE"),
        (15.0, "FM3_4", "STARTED"),
        (19.05, "FM3_4", "COMPLETE"),
        (22.0, "HOME", "STARTED"),
        (23.05, "HOME", "DISARMED"),
        (23.10, "HOME", "COMPLETE"),
    )
    return tuple(
        rosbag_module.MissionEventEvidence(
            _at(seconds), event_id, phase, state, "automatic attempt"
        )
        for event_id, (seconds, phase, state) in enumerate(rows)
    )


def _payload_events() -> tuple[object, ...]:
    rows = (
        (5.0, 2, "release", "detached"),
        (9.0, 3, "attach", "attached"),
        (12.0, 3, "release", "detached"),
        (16.0, 4, "attach", "attached"),
        (19.0, 4, "release", "detached"),
    )
    return tuple(
        rosbag_module.PayloadEventEvidence(
            _at(seconds),
            event_id,
            marker,
            f"run:{marker}:{action}:{event_id}",
            action,
            state,
            "OK",
        )
        for event_id, (seconds, marker, action, state) in enumerate(rows)
    )


def _vehicle_fact(seconds: float):
    xy = (0.0, 0.0)
    z = 10.0
    velocity = (0.0, 0.0, 0.0)
    contact = False
    if math.isclose(seconds, 1.0):
        xy, z, contact = (-91.44, 0.0), 0.0, True
    for start, end in ((3.0, 5.0), (10.0, 12.0), (17.0, 19.0)):
        if start <= seconds <= end:
            xy, z = F2, 10.0
    if math.isclose(seconds, 9.0):
        xy, z, contact = WA, 0.0, True
    if math.isclose(seconds, 16.0):
        xy, z, contact = WM, 0.0, True
    if seconds >= 23.0:
        xy, z, contact = (0.0, 0.0), 0.0, True
    return xy, z, velocity, contact


def _payload_fact(seconds: float, marker: int):
    if marker == 2:
        xy, attached, grounded = (0.0, 0.0), seconds <= 5.0, False
        if 3.0 <= seconds <= 5.0:
            xy = F2
        if seconds >= 6.0:
            xy, grounded = F2, True
    elif marker == 3:
        xy, attached, grounded = WA, 9.0 <= seconds <= 12.0, seconds < 9.0
        if 10.0 <= seconds <= 12.0:
            xy = F2
        if seconds >= 13.0:
            xy, grounded = F2, True
        elif attached:
            grounded = math.isclose(seconds, 9.0)
    else:
        xy, attached, grounded = WM, 16.0 <= seconds <= 19.0, seconds < 16.0
        if 17.0 <= seconds <= 19.0:
            xy = F2
        if seconds >= 20.0:
            xy, grounded = F2, True
        elif attached:
            grounded = math.isclose(seconds, 16.0)
    return xy, attached, grounded


def _perfect_physical_evidence(run_directory: Path) -> PhysicalBagEvidence:
    end = _at(23.10)
    ground_truth = []
    payload_states = []
    downward_ranges = []
    timestamp = START
    while timestamp <= end:
        seconds = (timestamp - START) / 1_000_000_000
        xy, z, velocity, contact = _vehicle_fact(seconds)
        ground_truth.append(
            GroundTruthEvidence(
                timestamp,
                "iris",
                (xy[0], xy[1], z),
                (0.0, 0.0, 0.0, 1.0),
                velocity,
                (0.0, 0.0, 0.0),
                contact,
            )
        )
        for marker in (2, 3, 4):
            payload_xy, attached, grounded = _payload_fact(seconds, marker)
            payload_states.append(
                rosbag_module.PayloadStateEvidence(
                    timestamp,
                    marker,
                    (payload_xy[0], payload_xy[1], 0.0254),
                    (0.0, 0.0, 0.0, 1.0),
                    (0.0, 0.0, 0.0),
                    grounded,
                    attached,
                )
            )
        downward_ranges.append(
            rosbag_module.DownwardRangeEvidence(timestamp, max(0.05, z))
        )
        timestamp += DT

    score_events = tuple(
        ScoreEventEvidence(
            end,
            index,
            f"competition.{rule_id}",
            value,
            f"scoring/events.jsonl#event-{index}",
        )
        for index, (rule_id, value) in enumerate(zip(RULE_IDS, POINTS, strict=True))
    ) + (
        ScoreEventEvidence(
            end,
            7,
            "score.finalized",
            150.0,
            "scoring/events.jsonl#event-7",
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
        tuple(downward_ranges),
    )


def _write_score_outputs(run_directory: Path, *, complete: bool = True) -> None:
    scoring = run_directory / "scoring"
    scoring.mkdir(parents=True)
    checksum = hashlib.sha256(RULES_PATH.read_bytes()).hexdigest()
    end = _at(23.10)
    events = [
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": end,
            "event_id": index,
            "event_type": f"competition.{rule_id}",
            "value": value,
            "evidence_ref": f"scoring/events.jsonl#event-{index}",
        }
        for index, (rule_id, value) in enumerate(zip(RULE_IDS, POINTS, strict=True))
    ]
    events.append(
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": end,
            "event_id": 7,
            "event_type": "score.finalized",
            "value": 150.0,
            "evidence_ref": "scoring/events.jsonl#event-7",
        }
    )
    (scoring / "events.jsonl").write_text(
        "".join(json.dumps(event) + "\n" for event in events),
        encoding="utf-8",
    )
    evidence_paths = [
        *(f"scoring/events.jsonl#event-{index}" for index in range(8)),
        "rosbag#/simulation/ground_truth",
        "rosbag#/simulation/payload_state",
        "rosbag#/simulation/payload_events",
        "rosbag#/simulation/mission_events",
    ]
    result = {
        "run_id": RUN_ID,
        "ruleset_id": "competition_v1",
        "complete": complete,
        "achieved_score": 150.0,
        "maximum_available_score": 150.0,
        "scoring_checksum": checksum,
        "evidence_paths": evidence_paths,
        "rule_results": [
            {
                "rule_id": rule_id,
                "passed": True,
                "awarded_points": value,
                "available_points": value,
                "evidence_ref": f"rosbag#competition:{rule_id}",
            }
            for rule_id, value in zip(RULE_IDS, POINTS, strict=True)
        ],
        "diagnostic": None,
    }
    (scoring / "result.json").write_text(json.dumps(result), encoding="utf-8")


def _fixture(tmp_path: Path, *, complete: bool = True) -> PhysicalBagEvidence:
    (tmp_path / "rosbag").mkdir()
    (tmp_path / "rosbag/data.mcap").write_bytes(b"physical competition evidence")
    _write_score_outputs(tmp_path, complete=complete)
    return _perfect_physical_evidence(tmp_path)


def test_independent_competition_validation_recomputes_150_from_physical_evidence(
    tmp_path,
):
    """Changing any physical component or point allocation must break 150/150."""
    from artifacts.competition_score_validation import (
        validate_competition_score_outputs,
    )

    evidence = _fixture(tmp_path)
    metadata = validate_competition_score_outputs(
        tmp_path,
        run_id=RUN_ID,
        rules_path=RULES_PATH,
        physical_evidence=evidence,
    )

    assert metadata.achieved_score == metadata.maximum_available_score == 150.0
    assert metadata.cumulative_checkpoints == (80.0, 145.0, 150.0)
    assert metadata.elapsed_simulated_ns == 23_100_000_000


def test_independent_competition_validation_uses_mission_relative_deadline(tmp_path):
    """A 900-second absolute epoch must not consume a 23.1-second attempt deadline."""
    from artifacts.competition_score_validation import (
        validate_competition_score_outputs,
    )

    evidence = _fixture(tmp_path)
    metadata = validate_competition_score_outputs(
        tmp_path,
        run_id=RUN_ID,
        rules_path=RULES_PATH,
        physical_evidence=evidence,
    )

    assert metadata.elapsed_simulated_ns < 600_000_000_000


def test_independent_competition_validation_rejects_scorekeeper_complete_claim(
    tmp_path,
):
    """A persisted result boolean cannot override independently complete physics."""
    from artifacts.competition_score_validation import (
        CompetitionScoreValidationError,
        validate_competition_score_outputs,
    )

    evidence = _fixture(tmp_path, complete=False)

    with pytest.raises(CompetitionScoreValidationError, match="independent"):
        validate_competition_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=RULES_PATH,
            physical_evidence=evidence,
        )


def test_independent_competition_validation_rejects_release_speed_over_threshold(
    tmp_path,
):
    """A 0.100001 m/s sample must reset the continuous two-second release window."""
    from artifacts.competition_score_validation import (
        CompetitionScoreValidationError,
        validate_competition_score_outputs,
    )

    evidence = _fixture(tmp_path)
    mutated_ground_truth = tuple(
        replace(sample, linear_velocity_xyz=(0.100001, 0.0, 0.0))
        if sample.sim_timestamp_ns == _at(4.0)
        else sample
        for sample in evidence.ground_truth
    )

    with pytest.raises(CompetitionScoreValidationError, match="independent"):
        validate_competition_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=RULES_PATH,
            physical_evidence=replace(evidence, ground_truth=mutated_ground_truth),
        )


def test_independent_competition_validation_rejects_rotated_payload_outside_f2(
    tmp_path,
):
    """A centered-only containment check must not accept a rotated overhanging box."""
    from artifacts.competition_score_validation import (
        CompetitionScoreValidationError,
        validate_competition_score_outputs,
    )

    evidence = _fixture(tmp_path)
    yaw = math.pi / 4.0
    mutated_states = tuple(
        replace(
            sample,
            position_xyz=(F2[0] + 0.36, F2[1], sample.position_xyz[2]),
            orientation_xyzw=(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
        )
        if sample.aruco_id == 2 and sample.sim_timestamp_ns >= _at(6.0)
        else sample
        for sample in evidence.payload_states
    )

    with pytest.raises(CompetitionScoreValidationError, match="independent"):
        validate_competition_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=RULES_PATH,
            physical_evidence=replace(evidence, payload_states=mutated_states),
        )


def test_independent_competition_validation_requires_pre_attach_payload_in_source(
    tmp_path,
):
    """Crossing the WA boundary during attach must not prove a source pickup."""
    from artifacts.competition_score_validation import (
        CompetitionScoreValidationError,
        validate_competition_score_outputs,
    )

    evidence = _fixture(tmp_path)
    outside_x = WA[0] + 6.096 / 2.0 + 0.01
    inside_x = WA[0] + 6.096 / 2.0 - 0.01
    mutated_states = tuple(
        replace(sample, position_xyz=(outside_x, WA[1], sample.position_xyz[2]))
        if sample.aruco_id == 3 and sample.sim_timestamp_ns == _at(8.95)
        else replace(sample, position_xyz=(inside_x, WA[1], sample.position_xyz[2]))
        if sample.aruco_id == 3 and sample.sim_timestamp_ns == _at(9.0)
        else sample
        for sample in evidence.payload_states
    )
    mutated_ground_truth = tuple(
        replace(sample, position_xyz=(inside_x, WA[1], sample.position_xyz[2]))
        if sample.sim_timestamp_ns == _at(9.0)
        else sample
        for sample in evidence.ground_truth
    )

    with pytest.raises(CompetitionScoreValidationError, match="independent"):
        validate_competition_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=RULES_PATH,
            physical_evidence=replace(
                evidence,
                ground_truth=mutated_ground_truth,
                payload_states=mutated_states,
            ),
        )


def test_independent_competition_validation_rejects_unexplained_attachment_swap(
    tmp_path,
):
    """Capacity one cannot hide an eventless reattach/detach before marker 3."""
    from artifacts.competition_score_validation import (
        CompetitionScoreValidationError,
        validate_competition_score_outputs,
    )

    evidence = _fixture(tmp_path)
    mutated_states = tuple(
        replace(sample, attached=True)
        if sample.aruco_id == 2
        and _at(8.0) <= sample.sim_timestamp_ns < _at(9.0)
        else sample
        for sample in evidence.payload_states
    )

    with pytest.raises(CompetitionScoreValidationError, match="independent"):
        validate_competition_score_outputs(
            tmp_path,
            run_id=RUN_ID,
            rules_path=RULES_PATH,
            physical_evidence=replace(evidence, payload_states=mutated_states),
        )
