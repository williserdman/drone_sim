from pathlib import Path

from drone_sim_scorekeeper.calibration import CalibrationScorer, load_calibration_rules
from drone_sim_scorekeeper.calibration_runtime import CalibrationScorekeeperRuntime
from drone_sim_scorekeeper.descent import GroundTruthSample


RUN_ID = "00000000-0000-4000-8000-000000000405"
RULES = Path(__file__).parents[1] / "rules/calibration_v1.json"


class Protocol:
    def write_status(self, status): return status
    def write_quiescence(self, module): return module


def test_runtime_finalizes_ground_truth_without_scenario_event(tmp_path):
    published = []
    scorer = CalibrationScorer(RUN_ID, load_calibration_rules(RULES), expected_ground_truth_samples=1)
    runtime = CalibrationScorekeeperRuntime(
        RUN_ID, scorer, run_directory=tmp_path, protocol=Protocol(),
        publish=published.append, flush=lambda: None,
    )
    runtime.accept_ground_truth(GroundTruthSample(
        RUN_ID, 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False,
    ))
    result = runtime.accept_source_finished(0)
    assert result.complete is True
    assert len(published) == 4
