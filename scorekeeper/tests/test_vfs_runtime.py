from __future__ import annotations

from types import SimpleNamespace

from drone_sim_scorekeeper.runtime_node import (
    attempt_status_message,
    judge_decision_message,
    vehicle_status_from_message,
    load_runtime_settings,
    rules_path_for_scenario,
)
from drone_sim_scorekeeper.vfs import AttemptState, AttemptStatus, JudgeDecision, JudgeOutcome
from drone_sim_scorekeeper.vfs_runtime import VfsScorekeeperRuntime


RUN_ID = "00000000-0000-4000-8000-000000002027"


def stamp(seconds: int = 0):
    return SimpleNamespace(sec=seconds, nanosec=0)


def message_type():
    return SimpleNamespace(sim_timestamp=stamp())


def test_vehicle_status_ros_conversion_preserves_observed_facts():
    message = SimpleNamespace(
        run_id=RUN_ID, sim_timestamp=stamp(2), sequence_id=7, vehicle_id="iris",
        armed=True, mode="GUIDED", failsafe_active=False, failsafe_reason="",
    )
    sample = vehicle_status_from_message(message)
    assert (sample.sim_timestamp_ns, sample.sequence_id, sample.armed, sample.mode) == (
        2_000_000_000, 7, True, "GUIDED"
    )


def test_judge_and_attempt_ros_outputs_preserve_correlation_and_budget():
    decision = JudgeDecision(
        RUN_ID, 3_000_000_000, 9, "request-2", 4, "FM2", "",
        JudgeOutcome.APPROVED, (2, 4), 2_000_000_000, "eligible_payloads",
        ("payload-state#2",),
    )
    output = judge_decision_message(decision, message_type)
    assert (output.request_id, output.request_event_id, output.outcome) == ("request-2", 4, 1)
    attempt = AttemptStatus(RUN_ID, 3_000_000_000, 10, AttemptState.APPROVED, 2_000_000_000, 3_000_000_000, True)
    attempt_output = attempt_status_message(attempt, message_type)
    assert (attempt_output.state, attempt_output.active_elapsed_ns, attempt_output.started) == (3, 2_000_000_000, True)


def test_vfs_runtime_publishes_attempt_status_and_correlated_decision():
    from scorekeeper.tests.test_vfs_score import COURSE, Scenario, payload, request, status, truth

    decisions = []
    attempts = []
    runtime = VfsScorekeeperRuntime(
        RUN_ID,
        __import__("drone_sim_scorekeeper.vfs", fromlist=["VfsJudge"]).VfsJudge(RUN_ID, COURSE, Scenario()),
        scorer=None,
        publish_decision=decisions.append,
        publish_attempt=attempts.append,
    )
    runtime.accept_ground_truth(truth(1, 0, 0, 0, True))
    runtime.accept_vehicle_status(status(1, 0, armed=False))
    for marker, x in ((2, -86.106), (3, -80), (4, -80)):
        runtime.accept_payload_state(payload(0, marker, x, -3.048, grounded=True))
        runtime.accept_payload_state(payload(1, marker, x, -3.048, grounded=True))
    runtime.accept_mission_event(request(1, 3, "FM2"))
    assert attempts[-1].state is AttemptState.WAITING
    assert decisions[-1].request_event_id == 3
    assert decisions[-1].eligible_aruco_ids == (2,)


def test_vfs_runtime_source_readiness_requires_truth_payload_and_status_tails(tmp_path):
    from scorekeeper.tests.test_vfs_score import COURSE, Scenario, payload, status, truth
    from drone_sim_scorekeeper.vfs import VfsJudge, VfsScorer, load_vfs_rules

    class Protocol:
        def write_status(self, _status): return None
        def write_quiescence(self, _module): return None

    scorer = VfsScorer(
        RUN_ID, load_vfs_rules("scorekeeper/rules/competition_2027_v1.json"),
        COURSE, Scenario(), unloaded_weight_lb=9.0,
    )
    runtime = VfsScorekeeperRuntime(
        RUN_ID, VfsJudge(RUN_ID, COURSE, Scenario()), scorer,
        publish_decision=lambda _value: None, publish_attempt=lambda _value: None,
        run_directory=tmp_path, protocol=Protocol(), publish_score=lambda _value: None,
        flush=lambda: None,
    )
    runtime.accept_ground_truth(truth(1, 0, 0, 0, True))
    runtime.accept_vehicle_status(status(0.6, 0, armed=False))
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(1, marker, -80, 0, grounded=True))
    assert runtime.source_inputs_observed_through(1_000_000_000)


def test_runtime_selects_2027_profile_and_rules(tmp_path):
    import json
    config = tmp_path / "run.json"
    config.write_text(json.dumps({
        "run_id": RUN_ID, "scenario": "competition_2027_v1",
        "recording": {"fps": 20}, "simulation": {"duration_sim_seconds": 780},
    }))
    assert load_runtime_settings(config, RUN_ID).scenario == "competition_2027_v1"
    assert rules_path_for_scenario("scorekeeper/rules", "competition_2027_v1").name == "competition_2027_v1.json"
