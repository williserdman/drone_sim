from __future__ import annotations

from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from drone_sim_scorekeeper.descent import DescentScorer, GroundTruthSample, load_descent_rules
from drone_sim_scorekeeper.models import RuleResult, ScoreEvent, ScoreResult


RUN_ID = "11111111-1111-4111-8111-111111111111"
RULES = Path(__file__).parents[1] / "rules/descent_v1.json"
LEGACY_RULES = Path(__file__).parents[1] / "rules/descent_v1_legacy.json"
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


def score_with_rules(values, rules_path):
    scorer = DescentScorer(
        RUN_ID,
        load_descent_rules(rules_path),
        expected_ground_truth_samples=len(values),
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


def test_v2_impact_transient_can_settle_before_deadline():
    values = perfect_descent() + [sample(31, x=0.1, contact=True)]
    values[20] = sample(20, x=0.1, contact=True, vz=-0.199)

    result = score(values)

    assert result.achieved_score == 100.0
    assert result.rule_results[3].passed is True


def test_v2_settling_interval_may_complete_exactly_at_deadline():
    values = perfect_descent() + [sample(index, x=0.1, contact=True) for index in range(31, 41)]
    for index in range(20, 30):
        values[index] = sample(index, x=0.1, contact=True, speed_x=0.11)

    result = score(values)

    assert result.achieved_score == 100.0
    assert result.rule_results[3].passed is True


def test_v2_settling_interval_completed_after_deadline_fails():
    values = perfect_descent() + [sample(index, x=0.1, contact=True) for index in range(31, 42)]
    for index in range(20, 31):
        values[index] = sample(index, x=0.1, contact=True, speed_x=0.11)

    result = score(values)

    assert result.achieved_score == 80.0
    assert result.rule_results[3].passed is False


def test_v2_contact_loss_before_qualification_latches_failure():
    values = perfect_descent() + [sample(index, x=0.1, contact=True) for index in range(31, 42)]
    values[22] = sample(22, x=0.1, contact=False)

    result = score(values)

    assert result.achieved_score == 80.0
    assert result.rule_results[3].passed is False


def test_v2_tilt_violation_before_qualification_latches_failure():
    values = perfect_descent() + [sample(index, x=0.1, contact=True) for index in range(31, 42)]
    values[22] = sample(
        22,
        x=0.1,
        contact=True,
        orientation=(0.1305261922, 0.0, 0.0, 0.9914448614),
    )

    result = score(values)

    assert result.achieved_score == 80.0
    assert result.rule_results[3].passed is False


def test_v2_speed_oscillation_never_forms_qualifying_interval():
    values = perfect_descent() + [sample(index, x=0.1, contact=True) for index in range(31, 42)]
    for index in range(20, 41):
        values[index] = sample(
            index,
            x=0.1,
            contact=True,
            speed_x=0.11 if index % 2 == 0 else 0.05,
        )

    result = score(values)

    assert result.achieved_score == 80.0
    assert result.rule_results[3].passed is False


def test_v2_insufficient_samples_before_qualification_fails():
    values = perfect_descent()[:25]
    values[20] = sample(20, x=0.1, contact=True, speed_x=0.11)

    result = score(values)

    assert result.achieved_score == 80.0
    assert result.rule_results[3].passed is False


def test_legacy_rules_keep_exact_first_contact_window_semantics():
    values = perfect_descent()
    values[20] = sample(20, x=0.1, contact=True, vz=-0.199)

    result = score_with_rules(values, LEGACY_RULES)

    assert result.achieved_score == 80.0
    assert result.rule_results[3].passed is False


def test_v2_rules_reject_off_grid_or_short_settling_deadline(tmp_path):
    import json

    document = json.loads(RULES.read_text())
    for deadline in (499_999_999, 525_000_000):
        document["settled_deadline_ns"] = deadline
        path = tmp_path / f"rules-{deadline}.json"
        path.write_text(json.dumps(document))
        with pytest.raises(ValueError, match="deadline"):
            load_descent_rules(path)


def test_descent_exports_the_scorer_neutral_result_models_unchanged():
    """Extracting shared models must not fork descent serialization types."""
    result = score(perfect_descent())

    assert isinstance(result, ScoreResult)
    assert all(isinstance(rule, RuleResult) for rule in result.rule_results)
    assert all(isinstance(event, ScoreEvent) for event in result.events)
