"""Bounded offline image check for policy, durable files, and ROS endpoints."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile

from artifacts.runtime_protocol import RuntimeProtocol

from .descent import DescentScorer, GroundTruthSample, load_descent_rules
from .runtime import ScenarioSample, ScorekeeperRuntime
from .runtime_node import _StructuredLogger, _create_ros_boundary
from .runtime_node import rules_path_for_scenario


RUN_ID = "11111111-1111-4111-8111-111111111111"
DT = 50_000_000


def _sample(index: int) -> GroundTruthSample:
    if index == 10:
        z, vz, contact, vx = 0.6, 0.0, False, 0.0
    elif 11 <= index < 20:
        z, vz, contact, vx = 0.6 - (index - 10) * 0.06, -0.8, False, 0.0
    elif index >= 20:
        z, vz, contact, vx = 0.0, 0.0, True, 0.05
    else:
        z, vz, contact, vx = 0.0, 0.0, True, 0.0
    return GroundTruthSample(
        RUN_ID,
        index * DT,
        (0.1 if index >= 11 else 0.0, 0.0, z),
        (0.0, 0.0, 0.0, 1.0),
        (vx, 0.0, vz),
        (0.0, 0.0, 0.0),
        contact,
    )


def _runtime_check(rules_path: Path) -> None:
    with tempfile.TemporaryDirectory(prefix="scorekeeper-self-test-") as temporary:
        run = Path(temporary)
        (run / ".status/quiescence").mkdir(parents=True)
        (run / ".control").mkdir()
        published = []
        with RuntimeProtocol(run, RUN_ID) as protocol:
            runtime = ScorekeeperRuntime(
                RUN_ID,
                DescentScorer(
                    RUN_ID,
                    load_descent_rules(rules_path),
                    expected_ground_truth_samples=31,
                ),
                run_directory=run,
                protocol=protocol,
                publish=published.append,
                flush=lambda: None,
            )
            runtime.accept_scenario(
                ScenarioSample(RUN_ID, DT, 0, "landing_pad", "INACTIVE")
            )
            for index in range(31):
                runtime.accept_ground_truth(_sample(index))
            result = runtime.accept_source_finished(30 * DT)
            assert result.complete and result.achieved_score == 100.0
            assert [event.event_id for event in published] == [0, 1, 2, 3, 4]
            assert json.loads(
                (run / ".status/score-finished.json").read_text(encoding="utf-8")
            ) == {
                "run_id": RUN_ID,
                "finished": True,
                "sim_timestamp_ns": 30 * DT,
            }
            runtime.begin_finalization()
            assert protocol.read_quiescence("scorekeeper") is not None


def _ros_check() -> None:
    import time

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from simulation_interfaces.msg import ScenarioEvent

    class ScenarioCapture:
        def __init__(self) -> None:
            self.samples = []

        def accept_scenario(self, sample) -> None:
            self.samples.append(sample)

    rclpy.init()
    publisher_node = rclpy.create_node("scorekeeper_scenario_history_publisher")
    publisher = publisher_node.create_publisher(
        ScenarioEvent,
        "/simulation/scenario_events",
        QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    message = ScenarioEvent()
    message.run_id = RUN_ID
    message.event_id = 0
    message.magnet_id = "descent-v1-magnet"
    message.state = "INACTIVE"
    publisher.publish(message)
    capture = ScenarioCapture()
    boundary = _create_ros_boundary(RUN_ID, [capture], _StructuredLogger(RUN_ID))
    try:
        deadline = time.monotonic() + 5.0
        while not capture.samples and time.monotonic() < deadline:
            rclpy.spin_once(publisher_node, timeout_sec=0.01)
            rclpy.spin_once(boundary.node, timeout_sec=0.05)
        assert len(capture.samples) == 1
        assert capture.samples[0].event_id == 0
        assert capture.samples[0].state == "INACTIVE"
        assert boundary.node.get_name() == "drone_sim_scorekeeper"
        publishers = boundary.node.get_publishers_info_by_topic(
            "/simulation/score_events"
        )
        assert len(publishers) == 1
        assert publishers[0].topic_type == "simulation_interfaces/msg/ScoreEvent"
        assert publishers[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert boundary.publisher.qos_profile.depth == 100
        expected = {
            "/simulation/ground_truth": (ReliabilityPolicy.BEST_EFFORT, 10, None),
            "/simulation/scenario_events": (
                ReliabilityPolicy.RELIABLE,
                100,
                DurabilityPolicy.TRANSIENT_LOCAL,
            ),
            "/clock": (ReliabilityPolicy.BEST_EFFORT, 1, None),
            "/simulation/run_state": (
                ReliabilityPolicy.RELIABLE,
                1,
                DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        }
        for topic, (reliability, depth, durability) in expected.items():
            endpoints = boundary.node.get_subscriptions_info_by_topic(topic)
            assert len(endpoints) == 1
            assert endpoints[0].qos_profile.reliability == reliability
            subscription = next(
                item for item in boundary.node.subscriptions if item.topic_name == topic
            )
            assert subscription.qos_profile.depth == depth
            if durability is not None:
                assert endpoints[0].qos_profile.durability == durability
    finally:
        boundary.node.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    rules = rules_path_for_scenario(
        __import__("os").environ.get(
            "SIM_SCORE_RULES_PATH",
            "/opt/drone_sim/scorekeeper/rules",
        ),
        "descent_v1",
    )
    _runtime_check(rules)
    _ros_check()
    print(json.dumps({"result": "ok", "ruleset_id": "descent_v1", "score": 100}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
