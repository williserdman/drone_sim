from __future__ import annotations

import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

from artifacts.runtime_status import RuntimeFailureStatus
from drone_sim_scorekeeper.competition import CompetitionScorer, load_competition_rules
from drone_sim_scorekeeper.competition_runtime import CompetitionScorekeeperRuntime
from drone_sim_scorekeeper.runtime_node import (
    _create_ros_boundary,
    load_runtime_settings,
    mission_event_from_message,
    payload_event_from_message,
    payload_state_from_message,
    rules_path_for_scenario,
)

from .test_competition_score import AttemptTrace, RUN_ID, RULES, new_trace, perfect_trace


class ProtocolRecorder:
    def __init__(self) -> None:
        self.statuses = []
        self.quiescence: list[str] = []

    def write_status(self, status) -> None:
        self.statuses.append(status)

    def write_quiescence(self, module: str) -> None:
        self.quiescence.append(module)


def runtime_for(tmp_path: Path, scorer: CompetitionScorer):
    protocol = ProtocolRecorder()
    operations: list[object] = []
    runtime = CompetitionScorekeeperRuntime(
        RUN_ID,
        scorer,
        run_directory=tmp_path,
        protocol=protocol,
        publish=lambda event: operations.append(("publish", event.event_id)),
        flush=lambda: operations.append(("flush",)),
        write_finished=lambda document: operations.append(("finished", document)),
    )
    return runtime, protocol, operations


def replay(source: AttemptTrace, runtime: CompetitionScorekeeperRuntime) -> None:
    for kind, sample in source.accepted_inputs:
        getattr(runtime, f"accept_{kind}")(sample)


def test_complete_score_persists_and_flushes_eight_events_before_finished(tmp_path):
    """The Home completion must precede durable publication and score-finished."""
    source = perfect_trace()
    runtime, protocol, operations = runtime_for(
        tmp_path,
        CompetitionScorer(RUN_ID, load_competition_rules(RULES)),
    )
    replay(source, runtime)

    result = runtime.accept_source_finished(source.scorer.last_sim_timestamp_ns)

    assert result.complete is True
    assert result.achieved_score == 150.0
    assert operations[:8] == [("publish", index) for index in range(8)]
    assert operations[8] == ("flush",)
    assert operations[9] == (
        "finished",
        {
            "run_id": RUN_ID,
            "finished": True,
            "sim_timestamp_ns": source.scorer.last_sim_timestamp_ns,
        },
    )
    assert protocol.statuses == []
    assert len((tmp_path / "scoring/events.jsonl").read_text().splitlines()) == 8
    assert json.loads((tmp_path / "scoring/result.json").read_text())[
        "maximum_available_score"
    ] == 150.0


def test_valid_home_persists_and_finishes_partial_score(tmp_path):
    """A valid terminal sequence must durably finalize honest missed points."""
    source = new_trace()
    source.fm1()
    source.drop(2, phase="FM2")
    source.drop(3, phase="FM3_3")
    source.drop(4, phase="FM3_4", release_speed=0.100001)
    source.home()
    runtime, protocol, operations = runtime_for(
        tmp_path,
        CompetitionScorer(RUN_ID, load_competition_rules(RULES)),
    )
    replay(source, runtime)

    result = runtime.accept_source_finished(source.scorer.last_sim_timestamp_ns)

    assert result.complete is True
    assert result.achieved_score == 145.0
    assert operations[:8] == [("publish", index) for index in range(8)]
    assert operations[8] == ("flush",)
    assert operations[9][0] == "finished"
    assert protocol.statuses == []
    persisted = json.loads((tmp_path / "scoring/result.json").read_text())
    assert persisted["complete"] is True
    assert persisted["achieved_score"] == 145.0
    event_rows = [
        json.loads(row)
        for row in (tmp_path / "scoring/events.jsonl").read_text().splitlines()
    ]
    assert [row["event_id"] for row in event_rows] == list(range(8))
    assert event_rows[6]["value"] == 0.0
    assert event_rows[7]["value"] == 145.0


def test_source_finish_without_valid_home_never_writes_score_finished(tmp_path):
    """Payload checkpoints alone must not complete the scoring lifecycle."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    runtime, protocol, operations = runtime_for(
        tmp_path,
        CompetitionScorer(RUN_ID, load_competition_rules(RULES)),
    )
    replay(trace, runtime)

    result = runtime.accept_source_finished(trace.scorer.last_sim_timestamp_ns)

    assert result.complete is False
    assert all(operation[0] != "finished" for operation in operations)
    assert type(protocol.statuses[0]) is RuntimeFailureStatus


def test_source_readiness_waits_for_payload_and_home_event_tail(tmp_path):
    """The durable source marker can race ahead of queued transient-local events."""
    trace = perfect_trace()
    runtime, _protocol, _operations = runtime_for(
        tmp_path,
        CompetitionScorer(RUN_ID, load_competition_rules(RULES)),
    )
    final_payload = next(
        sample
        for kind, sample in reversed(trace.accepted_inputs)
        if kind == "payload_event"
    )
    final_mission = next(
        sample
        for kind, sample in reversed(trace.accepted_inputs)
        if kind == "mission_event"
    )
    for kind, sample in trace.accepted_inputs:
        if sample in {final_payload, final_mission}:
            continue
        getattr(runtime, f"accept_{kind}")(sample)

    source_timestamp_ns = trace.scorer.last_sim_timestamp_ns
    assert runtime.source_inputs_observed_through(source_timestamp_ns) is False
    runtime.accept_payload_event(final_payload)
    assert runtime.source_inputs_observed_through(source_timestamp_ns) is False
    runtime.accept_mission_event(final_mission)
    assert runtime.source_inputs_observed_through(source_timestamp_ns) is True


def test_source_readiness_requires_distinct_home_disarmed_event(tmp_path):
    """HOME/COMPLETE alone cannot let the runtime finalize an armed attempt."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home(disarmed=False)
    runtime, _protocol, _operations = runtime_for(
        tmp_path,
        CompetitionScorer(RUN_ID, load_competition_rules(RULES)),
    )
    replay(trace, runtime)

    assert runtime.source_inputs_observed_through(
        trace.scorer.last_sim_timestamp_ns
    ) is False


def test_begin_finalization_persists_failure_then_becomes_quiescent(tmp_path):
    """Finalization cannot make a missing mission start or Home completion valid."""
    runtime, protocol, operations = runtime_for(
        tmp_path,
        CompetitionScorer(RUN_ID, load_competition_rules(RULES)),
    )

    runtime.begin_finalization()

    assert runtime.result is not None
    assert runtime.result.complete is False
    assert runtime.quiescent is True
    assert protocol.quiescence == ["scorekeeper"]
    assert all(operation[0] != "finished" for operation in operations)


def stamp(timestamp_ns: int):
    return SimpleNamespace(
        sec=timestamp_ns // 1_000_000_000,
        nanosec=timestamp_ns % 1_000_000_000,
    )


def test_resolved_competition_scenario_selects_only_competition_rules(tmp_path):
    """Selecting rules from the old fixed descent path would score the wrong policy."""
    config = tmp_path / "run.json"
    config.write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "scenario": "competition_v1",
                "recording": {"fps": 20},
                "simulation": {"duration_sim_seconds": 600.0},
            }
        )
    )

    settings = load_runtime_settings(config, RUN_ID)

    assert settings.scenario == "competition_v1"
    assert settings.expected_ground_truth_samples == 12_000
    assert rules_path_for_scenario(tmp_path / "rules", settings.scenario) == (
        tmp_path / "rules/competition_v1.json"
    )
    assert rules_path_for_scenario(
        tmp_path / "rules/descent_v1.json", settings.scenario
    ) == (tmp_path / "rules/competition_v1.json")


def test_competition_ros_messages_preserve_physical_and_ordered_evidence():
    """Dropping a physical field or event identity would let transport change score."""
    pose = SimpleNamespace(
        position=SimpleNamespace(x=-152.4, y=0.1, z=0.0254),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    twist = SimpleNamespace(
        linear=SimpleNamespace(x=0.01, y=0.02, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
    )

    state = payload_state_from_message(
        SimpleNamespace(
            run_id=RUN_ID,
            sim_timestamp=stamp(1_250_000_000),
            aruco_id=3,
            pose=pose,
            twist=twist,
            grounded=True,
            attached=False,
        )
    )
    payload_event = payload_event_from_message(
        SimpleNamespace(
            run_id=RUN_ID,
            sim_timestamp=stamp(1_300_000_000),
            event_id=4,
            aruco_id=3,
            command_id="run:3:release:1",
            action="release",
            state="detached",
            code="OK",
        )
    )
    mission_event = mission_event_from_message(
        SimpleNamespace(
            run_id=RUN_ID,
            sim_timestamp=stamp(1_350_000_000),
            event_id=5,
            phase="FM3_3",
            state="COMPLETE",
            detail="automatic attempt",
        )
    )

    assert state.position_xyz == (-152.4, 0.1, 0.0254)
    assert state.linear_velocity_xyz == (0.01, 0.02, 0.0)
    assert (state.grounded, state.attached) == (True, False)
    assert (payload_event.event_id, payload_event.command_id) == (
        4,
        "run:3:release:1",
    )
    assert (mission_event.event_id, mission_event.phase, mission_event.state) == (
        5,
        "FM3_3",
        "COMPLETE",
    )


def test_competition_ros_boundary_is_reliable_read_only_and_event_durable(monkeypatch):
    """A best-effort state input or actuator publisher would violate scoring authority."""
    class ReliabilityPolicy:
        RELIABLE = "reliable"
        BEST_EFFORT = "best_effort"

    class DurabilityPolicy:
        VOLATILE = "volatile"
        TRANSIENT_LOCAL = "transient_local"

    class QoSProfile:
        def __init__(self, *, depth, reliability, durability=DurabilityPolicy.VOLATILE):
            self.depth = depth
            self.reliability = reliability
            self.durability = durability

    class Publisher:
        def publish(self, _message):
            return None

    class Node:
        last = None

        def __init__(self, name):
            self.name = name
            self.publishers = []
            self.subscriptions = []
            Node.last = self

        def create_publisher(self, message_type, topic, qos):
            self.publishers.append((message_type, topic, qos))
            return Publisher()

        def create_subscription(self, message_type, topic, callback, qos):
            self.subscriptions.append((message_type, topic, callback, qos))
            return object()

    node_module = ModuleType("rclpy.node")
    node_module.Node = Node
    qos_module = ModuleType("rclpy.qos")
    qos_module.DurabilityPolicy = DurabilityPolicy
    qos_module.QoSProfile = QoSProfile
    qos_module.ReliabilityPolicy = ReliabilityPolicy
    clock_module = ModuleType("rosgraph_msgs.msg")
    clock_module.Clock = type("Clock", (), {})
    interface_module = ModuleType("simulation_interfaces.msg")
    for name in (
        "GroundTruth",
        "MissionEvent",
        "PayloadEvent",
        "PayloadState",
        "RunState",
        "ScenarioEvent",
        "ScoreEvent",
    ):
        setattr(interface_module, name, type(name, (), {}))
    monkeypatch.setitem(sys.modules, "rclpy.node", node_module)
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos_module)
    monkeypatch.setitem(sys.modules, "rosgraph_msgs.msg", clock_module)
    monkeypatch.setitem(sys.modules, "simulation_interfaces.msg", interface_module)

    _create_ros_boundary(
        RUN_ID,
        [SimpleNamespace()],
        SimpleNamespace(emit=lambda *_args, **_kwargs: None),
        scenario="competition_v1",
    )

    node = Node.last
    assert [topic for _type, topic, _qos in node.publishers] == [
        "/simulation/score_events"
    ]
    subscriptions = {
        topic: qos for _type, topic, _callback, qos in node.subscriptions
    }
    assert set(subscriptions) == {
        "/simulation/ground_truth",
        "/simulation/payload_state",
        "/simulation/payload_events",
        "/simulation/mission_events",
        "/clock",
        "/simulation/run_state",
    }
    for topic in (
        "/simulation/ground_truth",
        "/simulation/payload_state",
    ):
        assert subscriptions[topic].reliability == ReliabilityPolicy.RELIABLE
        assert subscriptions[topic].durability == DurabilityPolicy.VOLATILE
    for topic in ("/simulation/payload_events", "/simulation/mission_events"):
        assert subscriptions[topic].reliability == ReliabilityPolicy.RELIABLE
        assert subscriptions[topic].durability == DurabilityPolicy.TRANSIENT_LOCAL
