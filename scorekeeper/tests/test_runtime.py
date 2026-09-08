from __future__ import annotations

import json
from pathlib import Path

from artifacts.runtime_status import RuntimeFailureStatus
from drone_sim_scorekeeper.descent import (
    DescentScorer,
    GroundTruthSample,
    load_descent_rules,
)
from drone_sim_scorekeeper.runtime import ScenarioSample, ScorekeeperRuntime


RUN_ID = "11111111-1111-4111-8111-111111111111"
RULES = Path(__file__).parents[1] / "rules/descent_v1.json"
DT = 50_000_000


class ProtocolRecorder:
    def __init__(self) -> None:
        self.statuses = []
        self.quiescence: list[str] = []

    def write_status(self, status) -> None:
        self.statuses.append(status)

    def write_quiescence(self, module: str) -> None:
        self.quiescence.append(module)


def _sample(index: int, *, contact: bool = False) -> GroundTruthSample:
    if index == 10:
        return GroundTruthSample(
            RUN_ID, index * DT, (0.0, 0.0, 0.6), (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False,
        )
    if 11 <= index < 20:
        return GroundTruthSample(
            RUN_ID, index * DT, (0.1, 0.0, 0.6 - (index - 10) * 0.06),
            (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, -0.8),
            (0.0, 0.0, 0.0), False,
        )
    if index >= 20 or contact:
        return GroundTruthSample(
            RUN_ID, index * DT, (0.1, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
            (0.05, 0.0, 0.0), (0.0, 0.0, 0.0), True,
        )
    return GroundTruthSample(
        RUN_ID, index * DT, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), False,
    )


def _runtime(tmp_path, *, expected=31):
    protocol = ProtocolRecorder()
    operations: list[object] = []
    runtime = ScorekeeperRuntime(
        RUN_ID,
        DescentScorer(
            RUN_ID,
            load_descent_rules(RULES),
            expected_ground_truth_samples=expected,
        ),
        run_directory=tmp_path,
        protocol=protocol,
        publish=lambda event: operations.append(("publish", event.event_id)),
        flush=lambda: operations.append(("flush",)),
        write_finished=lambda document: operations.append(("finished", document)),
    )
    return runtime, protocol, operations


def test_complete_score_persists_and_flushes_five_events_before_finished(tmp_path):
    """Writing score-finished early could let orchestration close an incomplete bag."""
    runtime, protocol, operations = _runtime(tmp_path)
    runtime.accept_scenario(
        ScenarioSample(RUN_ID, DT, 0, "landing_pad", "INACTIVE")
    )
    for index in range(31):
        runtime.accept_ground_truth(_sample(index))

    runtime.accept_source_finished(30 * DT)

    result = json.loads((tmp_path / "scoring/result.json").read_text())
    assert result["complete"] is True
    assert result["achieved_score"] == 100.0
    assert operations[:5] == [("publish", index) for index in range(5)]
    assert operations[5] == ("flush",)
    assert operations[6] == (
        "finished",
        {"run_id": RUN_ID, "finished": True, "sim_timestamp_ns": 30 * DT},
    )
    assert protocol.statuses == []


def test_duplicate_ground_truth_writes_failure_and_never_score_finished(tmp_path):
    """A duplicate sample must not be hidden by a later contiguous suffix."""
    runtime, protocol, operations = _runtime(tmp_path)
    runtime.accept_scenario(
        ScenarioSample(RUN_ID, DT, 0, "landing_pad", "INACTIVE")
    )
    runtime.accept_ground_truth(_sample(0))
    runtime.accept_ground_truth(_sample(0))

    runtime.accept_source_finished(30 * DT)

    result = json.loads((tmp_path / "scoring/result.json").read_text())
    assert result["complete"] is False
    assert result["diagnostic"] == "ground_truth_timestamp_duplicate"
    assert all(operation[0] != "finished" for operation in operations)
    assert protocol.statuses == [
        RuntimeFailureStatus(
            RUN_ID,
            "scorekeeper",
            "ground_truth_timestamp_duplicate",
            ("scoring/events.jsonl", "scoring/result.json"),
        )
    ]


def test_finalization_of_truncated_input_fails_then_becomes_silent(tmp_path):
    """Quiescence must not certify a truncated score or permit later output."""
    runtime, protocol, operations = _runtime(tmp_path)
    runtime.accept_scenario(
        ScenarioSample(RUN_ID, DT, 0, "landing_pad", "INACTIVE")
    )
    for index in range(30):
        runtime.accept_ground_truth(_sample(index))

    runtime.begin_finalization()
    runtime.accept_ground_truth(_sample(30))

    result = json.loads((tmp_path / "scoring/result.json").read_text())
    assert result["complete"] is False
    assert result["diagnostic"] == "ground_truth_sample_count_mismatch"
    assert protocol.quiescence == ["scorekeeper"]
    assert runtime.quiescent is True
    assert all(operation[0] != "finished" for operation in operations)


def test_active_scenario_fails_closed_even_with_perfect_ground_truth(tmp_path):
    """A score for the inactive-only ruleset cannot survive an ACTIVE event."""
    runtime, protocol, operations = _runtime(tmp_path)
    runtime.accept_scenario(
        ScenarioSample(RUN_ID, DT, 0, "landing_pad", "ACTIVE")
    )
    for index in range(31):
        runtime.accept_ground_truth(_sample(index))

    runtime.accept_source_finished(30 * DT)

    result = json.loads((tmp_path / "scoring/result.json").read_text())
    assert result["complete"] is False
    assert result["diagnostic"] == "scenario_not_inactive"
    assert all(operation[0] != "finished" for operation in operations)
    assert type(protocol.statuses[0]) is RuntimeFailureStatus


def test_existing_global_failure_does_not_prevent_scorekeeper_quiescence(tmp_path):
    """The shared first-wins failure slot must not deadlock aggregate freeze."""
    protocol = ProtocolRecorder()
    runtime = ScorekeeperRuntime(
        RUN_ID,
        DescentScorer(
            RUN_ID,
            load_descent_rules(RULES),
            expected_ground_truth_samples=31,
        ),
        run_directory=tmp_path,
        protocol=protocol,
        publish=lambda _event: None,
        flush=lambda: None,
        write_finished=lambda _document: None,
    )

    runtime.begin_finalization()

    assert runtime.result is not None
    assert runtime.result.complete is False
    assert runtime.quiescent is True
    assert protocol.quiescence == ["scorekeeper"]
    assert type(protocol.statuses[0]) is RuntimeFailureStatus
