from __future__ import annotations

from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from drone_sim_scorekeeper.descent import DescentScorer, GroundTruthSample, load_descent_rules


RUN_ID = "11111111-1111-4111-8111-111111111111"
RULES = Path(__file__).parents[1] / "rules/descent_v1.json"
DT = 50_000_000


def sample(
    index: int,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    vz: float = 0.0,
    speed_x: float = 0.0,
    contact: bool = False,
    orientation=(0.0, 0.0, 0.0, 1.0),
) -> GroundTruthSample:
    return GroundTruthSample(
        run_id=RUN_ID,
        sim_timestamp_ns=index * DT,
        position_xyz=(x, y, z),
        orientation_xyzw=orientation,
        linear_velocity_xyz=(speed_x, 0.0, vz),
        angular_velocity_xyz=(0.0, 0.0, 0.0),
        in_contact=contact,
    )


def perfect_descent() -> list[GroundTruthSample]:
    values = [sample(index) for index in range(31)]
    values[10] = sample(10, z=0.6)
    for index in range(11, 20):
        values[index] = sample(index, x=0.1, z=0.6 - (index - 10) * 0.06, vz=-0.8)
    for index in range(20, 31):
        values[index] = sample(index, x=0.1, z=0.0, speed_x=0.05, contact=True)
    return values


def score(values):
    scorer = DescentScorer(
        RUN_ID, load_descent_rules(RULES), expected_ground_truth_samples=len(values)
    )
    for value in values:
        scorer.accept(value)
    return scorer.finalize()


def test_ground_truth_and_result_are_immutable():
    """Mutation after acceptance could change an already-audited score."""
    value = sample(0)
    result = score(perfect_descent())

    with pytest.raises(FrozenInstanceError):
        value.in_contact = True
    with pytest.raises(FrozenInstanceError):
        result.achieved_score = 0.0


def test_contiguous_nonflight_scores_zero_with_five_ordered_events():
    """A landed-only trace must not receive any descent points."""
    result = score([sample(index) for index in range(11)])

    assert result.complete is True
    assert (result.achieved_score, result.maximum_available_score) == (0.0, 100.0)
    assert [event.event_type for event in result.events] == [
        "descent.airborne_then_contact",
        "descent.touchdown_precision",
        "descent.safe_preimpact_speed",
        "descent.stable_contact",
        "score.finalized",
    ]
    assert [event.event_id for event in result.events] == [0, 1, 2, 3, 4]
    assert [event.value for event in result.events] == [0.0, 0.0, 0.0, 0.0, 0.0]


def test_off_marker_descent_scores_partial_sixty():
    """Touchdown precision must fail independently of the other three rules."""
    values = perfect_descent()
    for index in range(11, len(values)):
        original = values[index]
        values[index] = GroundTruthSample(
            run_id=original.run_id,
            sim_timestamp_ns=original.sim_timestamp_ns,
            position_xyz=(0.75, original.position_xyz[1], original.position_xyz[2]),
            orientation_xyzw=original.orientation_xyzw,
            linear_velocity_xyz=original.linear_velocity_xyz,
            angular_velocity_xyz=original.angular_velocity_xyz,
            in_contact=original.in_contact,
        )

    result = score(values)

    assert result.complete is True
    assert result.achieved_score == 60.0
    assert [rule.awarded_points for rule in result.rule_results] == [20.0, 0.0, 20.0, 20.0]


def test_safe_contiguous_descent_scores_exact_maximum():
    """The approved vertical slice needs one independently calculable 100/100 trace."""
    result = score(perfect_descent())

    assert result.complete is True
    assert (result.achieved_score, result.maximum_available_score) == (100.0, 100.0)
    assert [rule.passed for rule in result.rule_results] == [True, True, True, True]
    assert result.events[-1].value == 100.0
    assert result.ruleset_id == "descent_v1"
    assert len(result.scoring_checksum) == 64


def test_initial_ground_contact_does_not_hide_later_airborne_touchdown():
    """The Iris begins landed; scoring must select contact after takeoff."""
    values = perfect_descent()
    for index in range(10):
        values[index] = sample(index, contact=True)

    result = score(values)

    assert result.complete is True
    assert result.achieved_score == 100.0
    assert [rule.passed for rule in result.rule_results] == [True, True, True, True]


def test_first_gap_latches_incomplete_and_later_samples_cannot_repair_it():
    """Accepting a later contiguous suffix could falsely certify incomplete evidence."""
    scorer = DescentScorer(
        RUN_ID, load_descent_rules(RULES), expected_ground_truth_samples=31
    )
    scorer.accept(sample(0))
    scorer.accept(sample(2))
    scorer.accept(sample(1))
    for value in perfect_descent()[3:]:
        scorer.accept(value)

    result = scorer.finalize()

    assert result.complete is False
    assert result.achieved_score == 0.0
    assert result.diagnostic == "ground_truth_timestamp_gap"
    assert all(event.value == 0.0 for event in result.events)


def test_contiguous_truncated_trace_cannot_claim_complete_score():
    """A missing tail is data loss even when every received delta is contiguous."""
    scorer = DescentScorer(
        RUN_ID, load_descent_rules(RULES), expected_ground_truth_samples=31
    )
    for value in perfect_descent()[:-1]:
        scorer.accept(value)

    result = scorer.finalize()

    assert result.complete is False
    assert result.achieved_score == 0.0
    assert result.diagnostic == "ground_truth_sample_count_mismatch"


def test_preimpact_speed_uses_last_noncontact_sample():
    """Using the stopped contact sample would hide an unsafe impact."""
    values = perfect_descent()
    values[19] = sample(19, x=0.1, z=0.05, vz=-1.01)

    result = score(values)

    assert result.achieved_score == 80.0
    assert result.rule_results[2].passed is False


def test_stability_requires_full_half_second_on_exact_grid():
    """Ten 50 ms gaps span only 450 ms and must not earn settled-contact points."""
    result = score(perfect_descent()[:-1])

    assert result.achieved_score == 80.0
    assert result.rule_results[3].passed is False
