from __future__ import annotations

import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from artifacts.runtime_protocol import RuntimeProtocol
from artifacts.runtime_status import (
    RuntimeFailureStatus,
    ScoreFinishedStatus,
    SourceFinishedStatus,
)
import drone_sim_scorekeeper.runtime_node as runtime_node
from drone_sim_scorekeeper.runtime_node import (
    _RosBoundary,
    ScorekeeperDriver,
    ground_truth_from_message,
    load_runtime_settings,
    scenario_from_message,
    score_event_message,
)
from drone_sim_scorekeeper.descent import DescentScorer, load_descent_rules
from drone_sim_scorekeeper.runtime import ScenarioSample, ScorekeeperRuntime

from .test_competition_score import new_trace


RUN_ID = "11111111-1111-4111-8111-111111111111"
RULES = Path(__file__).parents[1] / "rules/descent_v1.json"


def _stamp(ns: int):
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _ground_truth_message(run_id: str, *, position_x: float = 0.0):
    return SimpleNamespace(
        run_id=run_id,
        sim_timestamp=_stamp(0),
        vehicle_id="iris",
        pose=SimpleNamespace(
            position=SimpleNamespace(x=position_x, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        in_contact=False,
    )


def _payload_state_message(run_id: str, *, position_x: float = 0.0):
    return SimpleNamespace(
        run_id=run_id,
        sim_timestamp=_stamp(0),
        aruco_id=2,
        pose=SimpleNamespace(
            position=SimpleNamespace(x=position_x, y=0.0, z=0.0254),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0, z=0.0)),
        grounded=False,
        attached=True,
    )


def _message_from_competition_sample(kind: str, sample: object):
    if kind == "ground_truth":
        return SimpleNamespace(
            run_id=sample.run_id,
            sim_timestamp=_stamp(sample.sim_timestamp_ns),
            pose=SimpleNamespace(
                position=SimpleNamespace(
                    x=sample.position_xyz[0],
                    y=sample.position_xyz[1],
                    z=sample.position_xyz[2],
                ),
                orientation=SimpleNamespace(
                    x=sample.orientation_xyzw[0],
                    y=sample.orientation_xyzw[1],
                    z=sample.orientation_xyzw[2],
                    w=sample.orientation_xyzw[3],
                ),
            ),
            twist=SimpleNamespace(
                linear=SimpleNamespace(
                    x=sample.linear_velocity_xyz[0],
                    y=sample.linear_velocity_xyz[1],
                    z=sample.linear_velocity_xyz[2],
                ),
                angular=SimpleNamespace(
                    x=sample.angular_velocity_xyz[0],
                    y=sample.angular_velocity_xyz[1],
                    z=sample.angular_velocity_xyz[2],
                ),
            ),
            in_contact=sample.in_contact,
        )
    if kind == "payload_state":
        return SimpleNamespace(
            run_id=sample.run_id,
            sim_timestamp=_stamp(sample.sim_timestamp_ns),
            aruco_id=sample.aruco_id,
            pose=SimpleNamespace(
                position=SimpleNamespace(
                    x=sample.position_xyz[0],
                    y=sample.position_xyz[1],
                    z=sample.position_xyz[2],
                ),
                orientation=SimpleNamespace(
                    x=sample.orientation_xyzw[0],
                    y=sample.orientation_xyzw[1],
                    z=sample.orientation_xyzw[2],
                    w=sample.orientation_xyzw[3],
                ),
            ),
            twist=SimpleNamespace(
                linear=SimpleNamespace(
                    x=sample.linear_velocity_xyz[0],
                    y=sample.linear_velocity_xyz[1],
                    z=sample.linear_velocity_xyz[2],
                )
            ),
            grounded=sample.grounded,
            attached=sample.attached,
        )
    fields = vars(sample).copy()
    fields["sim_timestamp"] = _stamp(fields.pop("sim_timestamp_ns"))
    return SimpleNamespace(**fields)


def _install_fake_ros(monkeypatch, *, spin=None):
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

        def wait_for_all_acked(self, timeout):
            return True

    class Node:
        last = None

        def __init__(self, _name):
            self.subscriptions = []
            self.destroyed = False
            Node.last = self

        def create_publisher(self, _message_type, _topic, _qos):
            return Publisher()

        def create_subscription(self, message_type, topic, callback, qos):
            self.subscriptions.append((message_type, topic, callback, qos))
            return object()

        def destroy_node(self):
            self.destroyed = True

    class Duration:
        def __init__(self, *, seconds=0, nanoseconds=0):
            self.seconds = seconds
            self.nanoseconds = nanoseconds

    rclpy = ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    rclpy.spin_once = lambda node, timeout_sec: spin(node) if spin else None
    node_module = ModuleType("rclpy.node")
    node_module.Node = Node
    qos_module = ModuleType("rclpy.qos")
    qos_module.DurabilityPolicy = DurabilityPolicy
    qos_module.QoSProfile = QoSProfile
    qos_module.ReliabilityPolicy = ReliabilityPolicy
    duration_module = ModuleType("rclpy.duration")
    duration_module.Duration = Duration
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
    for name, module in {
        "rclpy": rclpy,
        "rclpy.duration": duration_module,
        "rclpy.node": node_module,
        "rclpy.qos": qos_module,
        "rosgraph_msgs.msg": clock_module,
        "simulation_interfaces.msg": interface_module,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    return Node


def test_runtime_settings_derive_six_hundred_samples_from_resolved_config(tmp_path):
    """A production node must not retain the Phase 2 hard-coded 40 samples."""
    config = tmp_path / "run.json"
    config.write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "scenario": "descent_v1",
                "recording": {"fps": 20},
                "simulation": {"duration_sim_seconds": 30.0},
            }
        ),
        encoding="utf-8",
    )

    settings = load_runtime_settings(config, RUN_ID)

    assert settings.expected_ground_truth_samples == 600


@pytest.mark.parametrize("duration", [0, 1.001, True, "30"])
def test_runtime_settings_reject_non_grid_duration(tmp_path, duration):
    """Rounding duration would permit a truncated trace to look complete."""
    config = tmp_path / "run.json"
    config.write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "scenario": "descent_v1",
                "recording": {"fps": 20},
                "simulation": {"duration_sim_seconds": duration},
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError):
        load_runtime_settings(config, RUN_ID)


def test_ros_ground_truth_conversion_preserves_exact_simulation_values():
    """Using receipt or wall time would change score inputs under host slowdown."""
    message = SimpleNamespace(
        run_id=RUN_ID,
        sim_timestamp=_stamp(1_250_000_000),
        vehicle_id="iris",
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.1, y=0.2, z=-0.8),
            angular=SimpleNamespace(x=0.01, y=0.02, z=0.03),
        ),
        in_contact=True,
    )

    sample = ground_truth_from_message(message)

    assert sample.run_id == RUN_ID
    assert sample.sim_timestamp_ns == 1_250_000_000
    assert sample.position_xyz == (1.0, 2.0, 3.0)
    assert sample.linear_velocity_xyz == (0.1, 0.2, -0.8)
    assert sample.angular_velocity_xyz == (0.01, 0.02, 0.03)
    assert sample.in_contact is True


def test_ros_scenario_conversion_and_score_event_output_are_exact():
    """Changing event identity or timestamp would break bag/evidence correlation."""
    scenario = scenario_from_message(
        SimpleNamespace(
            run_id=RUN_ID,
            sim_timestamp=_stamp(50_000_000),
            event_id=7,
            magnet_id="landing_pad",
            state="INACTIVE",
        )
    )
    event = SimpleNamespace(
        run_id=RUN_ID,
        sim_timestamp_ns=1_500_000_000,
        event_id=4,
        event_type="score.finalized",
        value=100.0,
        evidence_ref="scoring/events.jsonl#event-4",
    )

    message = score_event_message(event, SimpleNamespace)

    assert scenario.event_id == 7
    assert scenario.state == "INACTIVE"
    assert message.run_id == RUN_ID
    assert (message.sim_timestamp.sec, message.sim_timestamp.nanosec) == (1, 500_000_000)
    assert message.event_id == 4
    assert message.event_type == "score.finalized"
    assert message.value == 100.0
    assert message.evidence_ref == "scoring/events.jsonl#event-4"


def test_ros_boundary_flush_uses_jazzy_duration_timeout(monkeypatch):
    """Jazzy accepts ``timeout=Duration(...)``, not ``timeout_sec=...``."""
    class Duration:
        def __init__(self, *, seconds=0, nanoseconds=0):
            self.seconds = seconds
            self.nanoseconds = nanoseconds

    duration_module = ModuleType("rclpy.duration")
    duration_module.Duration = Duration
    monkeypatch.setitem(sys.modules, "rclpy.duration", duration_module)

    class Publisher:
        def __init__(self):
            self.timeout = None

        def wait_for_all_acked(self, timeout):
            self.timeout = timeout
            return True

    publisher = Publisher()

    _RosBoundary(node=None, publisher=publisher, errors=[]).flush()

    assert publisher.timeout.seconds == 5.0
    assert publisher.timeout.nanoseconds == 0


def test_driver_observes_source_and_finalize_once_then_waits_for_terminal(tmp_path):
    """Polling duplicates must not republish score or let the service exit before commit."""
    class Protocol:
        def __init__(self):
            self.finalize = None
            self.terminal = None
            self.quiescence = []
            self.statuses = []

        def read_status(self, status_type):
            assert status_type is SourceFinishedStatus
            return SourceFinishedStatus(RUN_ID, 0)

        def read_finalize_request(self):
            return self.finalize

        def read_terminal_committed(self):
            return self.terminal

        def write_status(self, status):
            self.statuses.append(status)

        def write_quiescence(self, module):
            self.quiescence.append(module)

    protocol = Protocol()
    scorer = DescentScorer(
        RUN_ID, load_descent_rules(RULES), expected_ground_truth_samples=1
    )
    ground_truth = ground_truth_from_message(SimpleNamespace(
        run_id=RUN_ID,
        sim_timestamp=_stamp(0),
        vehicle_id="iris",
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        in_contact=False,
    ))
    published = []
    runtime = ScorekeeperRuntime(
        RUN_ID,
        scorer,
        run_directory=tmp_path,
        protocol=protocol,
        publish=published.append,
        flush=lambda: None,
    )
    runtime.accept_scenario(ScenarioSample(RUN_ID, 0, 0, "landing_pad", "INACTIVE"))
    runtime.accept_ground_truth(ground_truth)
    driver = ScorekeeperDriver(RUN_ID, runtime, protocol)

    assert driver.poll() is False
    assert driver.poll() is False
    assert len(published) == 5
    assert protocol.statuses == [ScoreFinishedStatus(RUN_ID, 0)]
    protocol.finalize = {
        "run_id": RUN_ID,
        "requested_terminal": "COMPLETED",
        "reason": "mission_complete",
    }
    assert driver.poll() is False
    assert protocol.quiescence == ["scorekeeper"]
    protocol.terminal = {
        "run_id": RUN_ID,
        "terminal_status": "COMPLETED",
        "reason": "mission_complete",
        "manifest_path": "manifest.json",
    }
    assert driver.poll() is True


def test_driver_drains_ros_ground_truth_through_source_timestamp_before_scoring(tmp_path):
    """The durable source marker can race ahead of still-queued ROS samples."""
    class Protocol:
        def __init__(self):
            self.statuses = []

        def read_status(self, status_type):
            assert status_type is SourceFinishedStatus
            return SourceFinishedStatus(RUN_ID, 0)

        def read_finalize_request(self):
            return None

        def read_terminal_committed(self):
            return None

        def write_status(self, status):
            self.statuses.append(status)

        def write_quiescence(self, _module):
            raise AssertionError("finalization was not requested")

    scorer = DescentScorer(
        RUN_ID, load_descent_rules(RULES), expected_ground_truth_samples=1
    )
    protocol = Protocol()
    runtime = ScorekeeperRuntime(
        RUN_ID,
        scorer,
        run_directory=tmp_path,
        protocol=protocol,
        publish=lambda _event: None,
        flush=lambda: None,
    )
    driver = ScorekeeperDriver(RUN_ID, runtime, protocol)

    assert driver.poll() is False
    assert runtime.result is None
    runtime.accept_ground_truth(ground_truth_from_message(SimpleNamespace(
        run_id=RUN_ID,
        sim_timestamp=_stamp(0),
        vehicle_id="iris",
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        in_contact=False,
    )))
    assert driver.poll() is False
    assert runtime.result is None
    runtime.accept_scenario(ScenarioSample(RUN_ID, 0, 0, "landing_pad", "INACTIVE"))
    assert driver.poll() is False
    assert runtime.result is not None
    assert runtime.result.complete is True
    assert protocol.statuses == [ScoreFinishedStatus(RUN_ID, 0)]


@pytest.mark.parametrize(
    ("scenario", "topic", "converter_name", "message_factory"),
    [
        (
            "descent_v1",
            "/simulation/ground_truth",
            "ground_truth_from_message",
            _ground_truth_message,
        ),
        (
            "competition_v1",
            "/simulation/payload_state",
            "payload_state_from_message",
            _payload_state_message,
        ),
    ],
)
def test_malformed_same_run_callback_latches_original_input_failure(
    monkeypatch, scenario, topic, converter_name, message_factory
):
    """A callback must latch scoring failure without replacing its exception."""
    Node = _install_fake_ros(monkeypatch)
    original_error = ValueError("original callback error")

    def reject_message(message):
        assert message.pose.position.x != message.pose.position.x
        raise original_error

    monkeypatch.setattr(runtime_node, converter_name, reject_message)

    class Runtime:
        failure_reason = None

        def fail(self, reason):
            if self.failure_reason is None:
                self.failure_reason = reason

        def accept_ground_truth(self, _sample):
            raise AssertionError("malformed evidence reached runtime acceptance")

        def accept_payload_state(self, _sample):
            raise AssertionError("malformed evidence reached runtime acceptance")

    runtime = Runtime()
    boundary = runtime_node._create_ros_boundary(
        RUN_ID,
        [runtime],
        SimpleNamespace(emit=lambda *_args, **_kwargs: None),
        scenario=scenario,
    )
    callback = {
        subscription_topic: callback
        for _type, subscription_topic, callback, _qos in Node.last.subscriptions
    }[topic]

    callback(message_factory("22222222-2222-4222-8222-222222222222", position_x=float("nan")))
    assert boundary.errors == []
    assert runtime.failure_reason is None

    callback(message_factory(RUN_ID, position_x=float("nan")))
    assert boundary.errors == [original_error]
    assert runtime.failure_reason == "ros_evidence_invalid"


@pytest.mark.parametrize(
    ("scenario", "topic", "accept_method", "message"),
    [
        (
            "descent_v1",
            "/simulation/scenario_events",
            "accept_scenario",
            SimpleNamespace(
                run_id=RUN_ID,
                sim_timestamp=_stamp(50_000_000),
                event_id=0,
                magnet_id="landing_pad",
                state="INACTIVE",
            ),
        ),
        (
            "competition_v1",
            "/simulation/payload_events",
            "accept_payload_event",
            SimpleNamespace(
                run_id=RUN_ID,
                sim_timestamp=_stamp(100_000_000),
                event_id=0,
                aruco_id=2,
                command_id="run:2:release:0",
                action="release",
                state="detached",
                code="OK",
            ),
        ),
        (
            "competition_v1",
            "/simulation/mission_events",
            "accept_mission_event",
            SimpleNamespace(
                run_id=RUN_ID,
                sim_timestamp=_stamp(150_000_000),
                event_id=0,
                phase="FM1",
                state="STARTED",
                detail="automatic attempt",
            ),
        ),
    ],
)
def test_logging_callback_error_does_not_latch_input_failure(
    monkeypatch, scenario, topic, accept_method, message
):
    """Logging failure after accepted evidence must not change scoring state."""
    Node = _install_fake_ros(monkeypatch)
    original_error = RuntimeError("original logging error")

    class Runtime:
        failure_reason = None

        def __init__(self):
            self.accepted = []

        def fail(self, reason):
            if self.failure_reason is None:
                self.failure_reason = reason

        def accept_scenario(self, sample):
            self.accepted.append(("accept_scenario", sample))

        def accept_payload_event(self, sample):
            self.accepted.append(("accept_payload_event", sample))

        def accept_mission_event(self, sample):
            self.accepted.append(("accept_mission_event", sample))

    class Logger:
        def emit(self, *_args, **_kwargs):
            raise original_error

    runtime = Runtime()
    boundary = runtime_node._create_ros_boundary(
        RUN_ID,
        [runtime],
        Logger(),
        scenario=scenario,
    )
    callback = {
        subscription_topic: callback
        for _type, subscription_topic, callback, _qos in Node.last.subscriptions
    }[topic]

    callback(message)

    assert boundary.errors == [original_error]
    assert runtime.failure_reason is None
    assert len(runtime.accepted) == 1
    assert runtime.accepted[0][0] == accept_method
    assert runtime.accepted[0][1].run_id == RUN_ID


@pytest.mark.parametrize("scenario", ["descent_v1", "competition_v1"])
def test_main_malformed_same_run_evidence_never_writes_score_finished(
    tmp_path, monkeypatch, scenario
):
    """Emergency finalization must persist a latched callback failure."""
    config = tmp_path / "configuration/run.json"
    config.parent.mkdir()
    (tmp_path / ".status/quiescence").mkdir(parents=True)
    config.write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "scenario": scenario,
                "recording": {"fps": 20},
                "simulation": {"duration_sim_seconds": 0.05},
            }
        ),
        encoding="utf-8",
    )
    spin_called = False

    if scenario == "competition_v1":
        trace = new_trace()
        trace.fm1()
        trace.drop(2, phase="FM2")
        trace.drop(3, phase="FM3_3")
        trace.drop(4, phase="FM3_4")
        trace.home()

    def spin(node):
        nonlocal spin_called
        assert spin_called is False
        spin_called = True
        callbacks = {
            topic: callback
            for _type, topic, callback, _qos in node.subscriptions
        }
        if scenario == "descent_v1":
            callbacks["/simulation/scenario_events"](
                SimpleNamespace(
                    run_id=RUN_ID,
                    sim_timestamp=_stamp(0),
                    event_id=0,
                    magnet_id="landing_pad",
                    state="INACTIVE",
                )
            )
            callbacks["/simulation/ground_truth"](_ground_truth_message(RUN_ID))
            callbacks["/simulation/ground_truth"](
                _ground_truth_message(RUN_ID, position_x=float("nan"))
            )
            return
        topics = {
            "ground_truth": "/simulation/ground_truth",
            "payload_state": "/simulation/payload_state",
            "payload_event": "/simulation/payload_events",
            "mission_event": "/simulation/mission_events",
        }
        for kind, sample in trace.accepted_inputs:
            callbacks[topics[kind]](_message_from_competition_sample(kind, sample))
        callbacks["/simulation/payload_state"](
            _payload_state_message(RUN_ID, position_x=float("nan"))
        )

    _install_fake_ros(monkeypatch, spin=spin)
    monkeypatch.setenv("SIM_RUN_ID", RUN_ID)
    monkeypatch.setenv("SIM_RUN_DIRECTORY", str(tmp_path))
    monkeypatch.setenv("SIM_CONFIG_PATH", str(config))
    monkeypatch.setenv("SIM_SCORE_RULES_PATH", str(RULES.parent))

    assert runtime_node.main() == 1

    result = json.loads((tmp_path / "scoring/result.json").read_text())
    assert result["complete"] is False
    assert result["diagnostic"] == "ros_evidence_invalid"
    protocol = RuntimeProtocol(tmp_path, RUN_ID)
    try:
        failure = protocol.read_status(RuntimeFailureStatus)
        assert failure == RuntimeFailureStatus(
            RUN_ID,
            "scorekeeper",
            "ros_evidence_invalid",
            ("scoring/events.jsonl", "scoring/result.json"),
        )
        assert protocol.read_status(ScoreFinishedStatus) is None
        assert protocol.read_quiescence("scorekeeper") == {
            "run_id": RUN_ID,
            "module": "scorekeeper",
            "quiescent": True,
        }
    finally:
        protocol.close()
