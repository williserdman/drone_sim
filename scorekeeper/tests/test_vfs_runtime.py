from __future__ import annotations

from types import SimpleNamespace

from drone_sim_scorekeeper.competition import MissionEventSample
from drone_sim_scorekeeper.runtime_node import (
    attempt_status_message,
    judge_decision_message,
    vehicle_status_from_message,
    load_runtime_settings,
    rules_path_for_scenario,
)
from drone_sim_scorekeeper.vfs import (
    AttemptState,
    AttemptStatus,
    JudgeDecision,
    JudgeOutcome,
    VfsJudge,
)
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
    for marker, x in ((2, -86.106), (3, -80), (4, -80)):
        runtime.accept_payload_state(payload(0, marker, x, -3.048, grounded=True))
    runtime.accept_ground_truth(truth(0, 0, 0, 0, True))
    runtime.accept_vehicle_status(status(0, 0, armed=False))
    runtime.accept_ground_truth(truth(1, 0, 0, 0, True))
    runtime.accept_vehicle_status(status(1, 1, armed=False))
    for marker, x in ((2, -86.106), (3, -80), (4, -80)):
        runtime.accept_payload_state(payload(1, marker, x, -3.048, grounded=True))
    runtime.accept_mission_event(request(1, 3, "FM2"))
    assert attempts[-1].state is AttemptState.WAITING
    assert decisions[-1].request_event_id == 3
    assert decisions[-1].eligible_aruco_ids == (2,)


def test_check_waits_for_exact_truth_epoch_and_excludes_future_status():
    from scorekeeper.tests.test_vfs_score import COURSE, Scenario, payload, request, status, truth
    decisions = []
    runtime = VfsScorekeeperRuntime(
        RUN_ID, VfsJudge(RUN_ID, COURSE, Scenario()), scorer=None,
        publish_decision=decisions.append, publish_attempt=lambda _value: None,
    )
    runtime.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM1", "STARTED", ""))
    for seconds, x, z, contact, armed in (
        (0.0, 0.0, 0.0, True, False),
        (1.0, 0.0, 10.0, False, True),
        (2.0, -91.44, 0.0, True, False),
    ):
        runtime.accept_ground_truth(truth(seconds, x, 0, z, contact))
        runtime.accept_vehicle_status(status(seconds, int(seconds), armed=armed))
        for marker in (2, 3, 4):
            runtime.accept_payload_state(payload(seconds, marker, -80, 0, grounded=True))
    runtime.accept_mission_event(request(2, 1, "FM1", "D"))
    assert decisions[-1].outcome is JudgeOutcome.APPROVED
    decisions.clear()
    runtime.accept_ground_truth(truth(3, 0, 0, 10, False))
    runtime.accept_vehicle_status(status(3, 3, armed=True))
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(3, marker, -80, 0, grounded=True))
    runtime.accept_ground_truth(truth(96.2, 0, 0, 0, True))
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(96.2, marker, -80, 0, grounded=True))
    runtime.accept_vehicle_status(status(96.2, 4, armed=False))

    check = request(96.3, 2, "FM1", "H")
    runtime.accept_mission_event(check)
    runtime.accept_ground_truth(truth(96.25, 0, 0, 0, True))
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(96.25, marker, -80, 0, grounded=True))
    runtime.accept_vehicle_status(status(96.3, 5, armed=False))
    runtime.accept_vehicle_status(status(96.4, 6, armed=True))
    assert decisions == []
    runtime.accept_ground_truth(truth(96.3, 0, 0, 0, True))
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(96.3, marker, -80, 0, grounded=True))

    assert len(decisions) == 1
    assert decisions[0].outcome is JudgeOutcome.APPROVED
    assert decisions[0].sim_timestamp_ns == 96_300_000_000
    assert "groundtruth#96300000000" in decisions[0].evidence_refs
    assert "vehicle-status#5" in decisions[0].evidence_refs
    runtime.accept_mission_event(check)
    assert decisions[-1] == decisions[0]


def test_status_before_truth_pauses_fm2_clock_on_the_exact_epoch():
    from scorekeeper.tests.test_vfs_score import COURSE, Scenario, payload, status, truth
    attempts = []
    runtime = VfsScorekeeperRuntime(
        RUN_ID, VfsJudge(RUN_ID, COURSE, Scenario()), scorer=None,
        publish_decision=lambda _value: None, publish_attempt=attempts.append,
    )
    runtime.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM2", "STARTED", ""))
    for seconds, contact, armed in ((0.0, True, False), (1.0, False, True)):
        runtime.accept_ground_truth(truth(seconds, 0, 0, 10 if not contact else 0, contact))
        runtime.accept_vehicle_status(status(seconds, int(seconds), armed=armed))
        for marker in (2, 3, 4):
            runtime.accept_payload_state(payload(seconds, marker, -80, 0, grounded=True))

    runtime.accept_vehicle_status(status(10, 2, armed=False))
    runtime.accept_ground_truth(truth(10, 0, 0, 0, True))
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(10, marker, -80, 0, grounded=True))

    assert attempts[-1].state is AttemptState.PAUSED
    assert attempts[-1].active_elapsed_ns == 9_000_000_000


def test_check_fails_closed_without_exact_epoch_and_never_uses_future_evidence():
    from scorekeeper.tests.test_vfs_score import COURSE, Scenario, payload, request, status, truth
    decisions = []
    runtime = VfsScorekeeperRuntime(
        RUN_ID, VfsJudge(RUN_ID, COURSE, Scenario()), scorer=None,
        publish_decision=decisions.append, publish_attempt=lambda _value: None,
    )
    runtime.accept_ground_truth(truth(0, 0, 0, 0, True))
    runtime.accept_vehicle_status(status(0, 0, armed=False))
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(0, marker, -80, 0, grounded=True))
    runtime.accept_mission_event(request(1, 1, "FM1", "D"))

    runtime.accept_vehicle_status(status(1.6, 1, armed=False))

    assert len(decisions) == 1
    assert decisions[0].outcome is JudgeOutcome.ERROR
    assert decisions[0].reason_code == "evidence_stale_or_missing"
    assert decisions[0].evidence_refs == ("mission-events#1",)


def test_missing_exact_epoch_cannot_approve_from_fresh_prior_physical_facts():
    from scorekeeper.tests.test_vfs_score import COURSE, Scenario, payload, request, status, truth

    decisions = []
    runtime = VfsScorekeeperRuntime(
        RUN_ID, VfsJudge(RUN_ID, COURSE, Scenario()), scorer=None,
        publish_decision=decisions.append, publish_attempt=lambda _value: None,
    )
    runtime.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM1", "STARTED", ""))
    sequence = 0
    for seconds, x, z, contact, armed in (
        (0.0, 0.0, 0.0, True, False),
        (1.0, 0.0, 10.0, False, True),
        (2.0, -91.44, 0.0, True, False),
    ):
        runtime.accept_ground_truth(truth(seconds, x, 0, z, contact))
        runtime.accept_vehicle_status(status(seconds, sequence, armed=armed))
        sequence += 1
        for marker in (2, 3, 4):
            runtime.accept_payload_state(payload(seconds, marker, -80, 0, grounded=True))
    runtime.accept_mission_event(request(2, 1, "FM1", "D"))
    decisions.clear()
    runtime.accept_ground_truth(truth(3, 0, 0, 10, False))
    runtime.accept_vehicle_status(status(3, sequence, armed=True))
    sequence += 1
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(3, marker, -80, 0, grounded=True))
    runtime.accept_ground_truth(truth(96.2, 0, 0, 0, True))
    runtime.accept_vehicle_status(status(96.2, sequence, armed=False))
    sequence += 1
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(96.2, marker, -80, 0, grounded=True))
    runtime.accept_ground_truth(truth(96.25, 0, 0, 0, True))
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(96.25, marker, -80, 0, grounded=True))
    runtime.accept_vehicle_status(status(96.3, sequence, armed=False))
    sequence += 1
    check = request(96.3, 2, "FM1", "H")
    runtime.accept_mission_event(check)

    try:
        for tenth in range(4, 10):
            runtime.accept_vehicle_status(status(96 + tenth / 10, sequence, armed=False))
            sequence += 1
    except ValueError as error:
        assert "remained incomplete" in str(error)

    assert len(decisions) == 1
    assert decisions[0].outcome is JudgeOutcome.ERROR
    assert decisions[0].reason_code == "evidence_stale_or_missing"
    assert decisions[0].request_id == "request-2"
    assert decisions[0].request_event_id == 2
    assert decisions[0].active_elapsed_ns == 95_300_000_000
    runtime.accept_mission_event(check)
    assert decisions[-1] == decisions[0]


def test_first_fifty_ms_attempt_epoch_publishes_when_status_grid_begins_at_100_ms():
    from scorekeeper.tests.test_vfs_score import COURSE, Scenario, payload, status, truth
    attempts = []
    runtime = VfsScorekeeperRuntime(
        RUN_ID, VfsJudge(RUN_ID, COURSE, Scenario()), scorer=None,
        publish_decision=lambda _value: None, publish_attempt=attempts.append,
    )
    runtime.accept_ground_truth(truth(0.05, 0, 0, 0, True))
    for marker in (2, 3, 4):
        runtime.accept_payload_state(payload(0.05, marker, -80, 0, grounded=True))
    assert attempts == []

    runtime.accept_vehicle_status(status(0.1, 0, armed=False))

    assert attempts[-1].sim_timestamp_ns == 50_000_000
    assert attempts[-1].state is AttemptState.WAITING


def test_incomplete_physical_join_is_bounded_and_duplicate_input_fails_closed():
    from scorekeeper.tests.test_vfs_score import COURSE, Scenario, payload, status, truth

    runtime = VfsScorekeeperRuntime(
        RUN_ID, VfsJudge(RUN_ID, COURSE, Scenario()), scorer=None,
        publish_decision=lambda _value: None, publish_attempt=lambda _value: None,
    )
    runtime.accept_ground_truth(truth(1, 0, 0, 0, True))
    runtime.accept_vehicle_status(status(1, 0, armed=False))
    runtime.accept_payload_state(payload(1, 2, -80, 0, grounded=True))
    runtime.accept_payload_state(payload(1, 3, -80, 0, grounded=True))
    try:
        runtime.accept_payload_state(payload(1, 3, -80, 0, grounded=True))
    except ValueError as error:
        assert "timestamps must increase" in str(error)
    else:
        raise AssertionError("duplicate payload input was accepted")

    try:
        runtime.accept_vehicle_status(status(1.6, 1, armed=False))
    except ValueError as error:
        assert "remained incomplete" in str(error)
    else:
        raise AssertionError("incomplete physical join exceeded its bound")


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
    runtime.accept_vehicle_status(status(1, 0, armed=False))
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
