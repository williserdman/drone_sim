from dataclasses import dataclass

import pytest

from orchestration.runtime_node import (
    OrchestrationRuntime,
    RunStateSubscriber,
    RunStateTransportBarrier,
    RuntimeStateEvent,
    start_runtime_after_transport_barrier,
)


RUN_ID = "11111111-1111-4111-8111-111111111111"


class FakeProtocol:
    def __init__(self):
        self.finalize = None
        self.terminal = None
        self.statuses = []
        self.readable_statuses = {}
        self.quiescence = {}
        self.quiescence_writes = []

    def read_finalize_request(self):
        return self.finalize

    def read_terminal_committed(self):
        return self.terminal

    def write_status(self, name, document):
        self.statuses.append((name, document))

    def read_status(self, name):
        return self.readable_statuses.get(name)

    def write_quiescence(self, module):
        self.quiescence_writes.append(module)
        self.quiescence[module] = {
            "run_id": RUN_ID,
            "module": module,
            "quiescent": True,
        }

    def read_quiescence(self, module):
        return self.quiescence.get(module)


def _runtime():
    protocol = FakeProtocol()
    published = []
    diagnostics = []
    runtime = OrchestrationRuntime(
        RUN_ID,
        "a" * 64,
        protocol=protocol,
        publish=published.append,
        diagnostic=diagnostics.append,
    )
    return runtime, protocol, published, diagnostics


def _phase3_runtime():
    protocol = FakeProtocol()
    published = []
    diagnostics = []
    runtime = OrchestrationRuntime(
        RUN_ID,
        "a" * 64,
        protocol=protocol,
        publish=published.append,
        diagnostic=diagnostics.append,
        required_durable_readiness=("ardupilot-ready", "companion-ready"),
    )
    return runtime, protocol, published, diagnostics


def _subscriber(name, *, topic_type="simulation_interfaces/msg/RunState", reliable=True,
                transient_local=True):
    return RunStateSubscriber(name, topic_type, reliable, transient_local)


def _required_subscribers():
    return [
        _subscriber("artifacts_runtime"),
        _subscriber("synthetic_companion"),
        _subscriber("synthetic_ardupilot_sitl"),
        _subscriber("synthetic_gazebo"),
        _subscriber("synthetic_electromagnet"),
        _subscriber("synthetic_scorekeeper"),
        _subscriber("rosbag2_recorder_deadbeef"),
    ]


def test_run_state_transport_barrier_requires_all_seven_intended_subscribers():
    failures = []
    barrier = RunStateTransportBarrier(deadline=10.0, failure=failures.append)

    assert barrier.poll(_required_subscribers()[:6], now=1.0, finalizing=False) is False
    assert barrier.poll(_required_subscribers(), now=2.0, finalizing=False) is True
    assert barrier.ready is True
    assert failures == []


def test_phase3_transport_barrier_requires_only_real_ros_consumers_and_recorder():
    failures = []
    required_nodes = {
        "artifacts_runtime",
        "drone_sim_companion",
        "gazebo_runtime",
        "drone_sim_electromagnet",
        "drone_sim_scorekeeper",
    }
    barrier = RunStateTransportBarrier(
        deadline=10.0,
        failure=failures.append,
        required_nodes=required_nodes,
    )
    subscribers = [
        _subscriber(name) for name in sorted(required_nodes)
    ] + [_subscriber("rosbag2_recorder_deadbeef")]

    missing_real_companion = [
        _subscriber("synthetic_companion")
        if item.node_name == "drone_sim_companion"
        else item
        for item in subscribers
    ]
    assert barrier.poll(missing_real_companion, now=1.0, finalizing=False) is False
    assert barrier.poll(subscribers, now=2.0, finalizing=False) is True
    assert failures == []

    # ArduPilot is not a ROS node; durable ardupilot-ready remains its gate.
    assert "ardupilot_sitl" not in required_nodes


def test_run_state_transport_barrier_ignores_duplicates_stale_qos_type_and_extras():
    failures = []
    required = _required_subscribers()
    barrier = RunStateTransportBarrier(deadline=10.0, failure=failures.append)
    invalid = [
        required[0],
        required[0],
        _subscriber("stale", topic_type="std_msgs/msg/String"),
        _subscriber("synthetic_scorekeeper", reliable=False),
        _subscriber("unrelated_debugger"),
    ]
    assert barrier.poll(invalid, now=1.0, finalizing=False) is False

    assert barrier.poll(required + [_subscriber("unrelated_debugger")], now=2.0,
                        finalizing=False) is True
    assert failures == []


def test_run_state_transport_barrier_durable_fails_once_at_deadline():
    failures = []
    barrier = RunStateTransportBarrier(deadline=3.0, failure=failures.append)

    assert barrier.poll(_required_subscribers()[:6], now=3.0, finalizing=False) is False
    assert barrier.failed is True
    assert failures == ["run-state transport discovery deadline expired"]
    assert barrier.poll(_required_subscribers(), now=4.0, finalizing=False) is False
    assert failures == ["run-state transport discovery deadline expired"]


def test_run_state_transport_barrier_finalize_preempts_without_failure():
    failures = []
    barrier = RunStateTransportBarrier(deadline=10.0, failure=failures.append)

    assert barrier.poll([], now=1.0, finalizing=True) is False
    assert barrier.preempted is True
    assert barrier.poll(_required_subscribers(), now=2.0, finalizing=False) is False
    assert failures == []


def test_main_path_does_not_start_runtime_after_transport_timeout():
    failures = []
    started = []
    barrier = RunStateTransportBarrier(deadline=1.0, failure=failures.append)

    result = start_runtime_after_transport_barrier(
        barrier=barrier,
        subscribers=lambda: _required_subscribers()[:6],
        finalize_requested=lambda: False,
        monotonic=lambda: 1.0,
        spin_once=lambda: None,
        runtime_start=lambda: started.append("STARTING"),
        runtime_ok=lambda: True,
    )

    assert result is False
    assert started == []
    assert failures == ["run-state transport discovery deadline expired"]


def test_main_path_does_not_start_runtime_after_finalize_preemption():
    started = []
    barrier = RunStateTransportBarrier(deadline=10.0, failure=lambda _reason: None)

    result = start_runtime_after_transport_barrier(
        barrier=barrier,
        subscribers=_required_subscribers,
        finalize_requested=lambda: True,
        monotonic=lambda: 1.0,
        spin_once=lambda: None,
        runtime_start=lambda: started.append("STARTING"),
        runtime_ok=lambda: True,
    )

    assert result is False
    assert started == []
    assert barrier.preempted is True


def test_main_path_starts_once_only_after_exact_transport_readiness():
    started = []
    barrier = RunStateTransportBarrier(deadline=10.0, failure=lambda _reason: None)

    result = start_runtime_after_transport_barrier(
        barrier=barrier,
        subscribers=_required_subscribers,
        finalize_requested=lambda: False,
        monotonic=lambda: 1.0,
        spin_once=lambda: None,
        runtime_start=lambda: started.append("STARTING"),
        runtime_ok=lambda: True,
    )

    assert result is True
    assert started == ["STARTING"]


def test_starting_ready_running_order_and_exact_first_clock_stamp():
    runtime, protocol, published, _ = _runtime()
    runtime.start()
    runtime.accept_clock(99)
    runtime.accept_artifact_status("stale", True)
    runtime.accept_artifact_status(RUN_ID, False)
    runtime.accept_artifact_status(RUN_ID, True)
    runtime.accept_clock(0)
    runtime.accept_clock(50_000_000)

    assert [(item.state, item.sim_timestamp_ns) for item in published] == [
        ("STARTING", 0),
        ("READY", 0),
        ("RUNNING", 0),
    ]
    assert protocol.statuses == [
        (
            "runtime-running",
            {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": 0},
        )
    ]
    assert runtime.last_sim_timestamp_ns == 50_000_000


def test_phase3_first_clock_remains_ready_until_durable_flight_peers_are_ready():
    runtime, protocol, published, _ = _phase3_runtime()
    runtime.start()
    runtime.accept_artifact_status(RUN_ID, True)
    runtime.accept_clock(0)

    assert [item.state for item in published] == ["STARTING", "READY"]
    assert protocol.statuses == []

    protocol.readable_statuses["ardupilot-ready"] = {
        "run_id": RUN_ID,
        "ready": True,
        "json_exchange": True,
        "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
    }
    assert runtime.poll() is False
    assert [item.state for item in published] == ["STARTING", "READY"]

    protocol.readable_statuses["companion-ready"] = {
        "run_id": RUN_ID,
        "ready": True,
        "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
        "heartbeat_sim_timestamp_ns": 0,
    }
    assert runtime.poll() is False

    assert [(item.state, item.sim_timestamp_ns) for item in published] == [
        ("STARTING", 0),
        ("READY", 0),
        ("RUNNING", 0),
    ]
    assert protocol.statuses == [
        (
            "runtime-running",
            {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": 0},
        )
    ]


@pytest.mark.parametrize("preterminal", ["STARTING", "READY", "RUNNING"])
@pytest.mark.parametrize("terminal", ["COMPLETED", "FAILED", "ABORTED"])
def test_finalize_from_every_preterminal_state_leaves_live_notification_to_artifacts(
    preterminal, terminal
):
    runtime, protocol, published, _ = _runtime()
    runtime.start()
    if preterminal in {"READY", "RUNNING"}:
        runtime.accept_artifact_status(RUN_ID, True)
    if preterminal == "RUNNING":
        runtime.accept_clock(123)
    protocol.finalize = {
        "run_id": RUN_ID,
        "requested_terminal": terminal,
        "reason": "operator requested",
    }
    assert runtime.poll() is False
    assert published[-1] == RuntimeStateEvent(
        RUN_ID, "FINALIZING", 123 if preterminal == "RUNNING" else 0, "operator requested", "a" * 64
    )
    assert protocol.quiescence_writes == ["orchestration"]
    assert not any(name == "runtime-frozen" for name, _document in protocol.statuses)
    for module in ("companion", "ardupilot_sitl", "gazebo", "electromagnet", "scorekeeper"):
        protocol.quiescence[module] = {
            "run_id": RUN_ID,
            "module": module,
            "quiescent": True,
        }
    assert runtime.poll() is False
    assert protocol.statuses[-1] == (
        "runtime-frozen",
        {"run_id": RUN_ID, "frozen": True},
    )
    protocol.terminal = {
        "run_id": RUN_ID,
        "terminal_status": terminal,
        "reason": "committed reason",
        "manifest_path": "manifest.json",
    }
    assert runtime.poll() is True
    assert published[-1].state == "FINALIZING"
    assert runtime.state == terminal
    assert protocol.statuses[-1] == (
        "runtime-frozen",
        {"run_id": RUN_ID, "frozen": True},
    )
    assert not any(name == "terminal-notified" for name, _ in protocol.statuses)
    count = len(published)
    assert runtime.poll() is True
    assert len(published) == count


def test_aggregate_freeze_waits_for_all_six_quiescence_owners_and_writes_once():
    runtime, protocol, published, _ = _runtime()
    runtime.start()
    protocol.finalize = {
        "run_id": RUN_ID,
        "requested_terminal": "FAILED",
        "reason": "recorder failed",
    }

    assert runtime.poll() is False
    assert published[-1].state == "FINALIZING"
    assert protocol.quiescence_writes == ["orchestration"]

    for module in ("companion", "ardupilot_sitl", "gazebo", "electromagnet"):
        protocol.quiescence[module] = {
            "run_id": RUN_ID,
            "module": module,
            "quiescent": True,
        }
        assert runtime.poll() is False
        assert not any(name == "runtime-frozen" for name, _document in protocol.statuses)

    protocol.quiescence["scorekeeper"] = {
        "run_id": RUN_ID,
        "module": "scorekeeper",
        "quiescent": True,
    }
    assert runtime.poll() is False
    assert [item for item in protocol.statuses if item[0] == "runtime-frozen"] == [
        ("runtime-frozen", {"run_id": RUN_ID, "frozen": True})
    ]
    runtime.poll()
    assert protocol.quiescence_writes == ["orchestration"]
    assert [item for item in protocol.statuses if item[0] == "runtime-frozen"] == [
        ("runtime-frozen", {"run_id": RUN_ID, "frozen": True})
    ]


def test_stale_ids_are_diagnosed_before_quiescence_and_ignored():
    runtime, _, published, diagnostics = _runtime()
    runtime.start()
    runtime.accept_artifact_status("22222222-2222-4222-8222-222222222222", True)
    assert [item.state for item in published] == ["STARTING"]
    assert diagnostics == ["ignored stale artifact status"]


def test_clock_requires_nonnegative_integer_and_readiness():
    runtime, _, _, _ = _runtime()
    runtime.start()
    with pytest.raises(ValueError):
        runtime.accept_clock(-1)
    with pytest.raises(TypeError):
        runtime.accept_clock(True)
