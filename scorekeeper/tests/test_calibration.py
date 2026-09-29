from pathlib import Path

from drone_sim_scorekeeper.calibration import CalibrationScorer, load_calibration_rules
from drone_sim_scorekeeper.descent import GroundTruthSample


RUN_ID = "00000000-0000-4000-8000-000000000404"
RULES = Path(__file__).parents[1] / "rules/calibration_v1.json"


def _sample(index, *, z=0.0, velocity=(0.0, 0.0, 0.0), contact=False):
    return GroundTruthSample(
        RUN_ID, index * 50_000_000, (25.0, -12.0, z), (0.0, 0.0, 0.0, 1.0),
        velocity, (0.0, 0.0, 0.0), contact,
    )


def _score(preimpact_speed=0.5):
    samples = [_sample(0), _sample(1, z=1.0)]
    samples.append(_sample(2, velocity=(0.0, 0.0, -preimpact_speed)))
    samples.extend(_sample(index, contact=True) for index in range(3, 14))
    scorer = CalibrationScorer(RUN_ID, load_calibration_rules(RULES), expected_ground_truth_samples=14)
    for sample in samples:
        scorer.accept(sample)
    return scorer.finalize()


def test_far_safe_stable_landing_scores_100_without_position_rule():
    result = _score()
    assert result.achieved_score == 100.0
    assert tuple(row.rule_id for row in result.rule_results) == (
        "airborne_then_contact", "safe_preimpact_speed", "stable_contact"
    )


def test_1_052_mps_landing_scores_60():
    result = _score(1.052)
    assert result.achieved_score == 60.0
    assert tuple(row.passed for row in result.rule_results) == (True, False, True)


def test_missing_contact_and_sample_grid_fail_closed():
    rules = load_calibration_rules(RULES)
    no_contact = CalibrationScorer(RUN_ID, rules, expected_ground_truth_samples=2)
    no_contact.accept(_sample(0))
    no_contact.accept(_sample(1, z=1.0))
    assert no_contact.finalize().achieved_score == 0.0

    gap = CalibrationScorer(RUN_ID, rules, expected_ground_truth_samples=2)
    gap.accept(_sample(0))
    gap.accept(_sample(2, contact=True))
    assert gap.finalize().complete is False
    assert gap.finalize().diagnostic == "ground_truth_timestamp_gap"
