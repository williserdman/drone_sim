#!/usr/bin/env python3
"""Run a deterministic synthetic ROS lifecycle for foundation verification."""

from contextlib import ExitStack
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from simulation_interfaces.msg import RunState

from artifacts import StructuredEvent, write_event


MODULE = "foundation"
SIMULATED_TIMES = ((0, 0), (0, 50_000_000), (0, 100_000_000))
DELIVERY_TIMEOUT_SECONDS = 15.0


def _set_time(message, seconds: int, nanoseconds: int) -> None:
    message.sec = seconds
    message.nanosec = nanoseconds


class FoundationNode(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_foundation")
        self.clock_publisher = self.create_publisher(
            Clock,
            "/clock",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.run_state_publisher = self.create_publisher(
            RunState,
            "/simulation/run_state",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def publish_clock(self, seconds: int, nanoseconds: int) -> None:
        message = Clock()
        _set_time(message.clock, seconds, nanoseconds)
        self.clock_publisher.publish(message)

    def publish_run_state(
        self,
        run_id: str,
        state: int,
        seconds: int,
        nanoseconds: int,
    ) -> None:
        message = RunState()
        message.run_id = run_id
        _set_time(message.sim_timestamp, seconds, nanoseconds)
        message.state = state
        message.reason = ""
        message.config_sha256 = ""
        self.run_state_publisher.publish(message)

    def wait_for_subscribers(self) -> None:
        deadline = time.monotonic() + DELIVERY_TIMEOUT_SECONDS
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.clock_publisher.get_subscription_count() > 0
                and self.run_state_publisher.get_subscription_count() > 0
            ):
                return
        raise RuntimeError("timed out waiting for foundation topic subscribers")


def _read_observation(observation_path: Path) -> dict[str, object] | None:
    try:
        report = json.loads(observation_path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return None
    if report.get("error"):
        raise RuntimeError(report["error"])
    return report


def _wait_for_observer_discovery(observation_path: Path) -> None:
    deadline = time.monotonic() + DELIVERY_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        report = _read_observation(observation_path)
        if report is not None and report.get("discovered"):
            return
        time.sleep(0.02)
    raise RuntimeError("timed out waiting for observer-side DDS discovery")


def _publish_and_wait_for_observation(
    observation_path: Path,
    publish,
    *,
    clock_count: int,
    run_state_count: int,
) -> None:
    publish()
    deadline = time.monotonic() + DELIVERY_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        report = _read_observation(observation_path)
        if report is not None and (
            len(report.get("clock", [])) >= clock_count
            and len(report.get("run_state", [])) >= run_state_count
        ):
            return
        time.sleep(0.02)
    raise RuntimeError(
        "timed out waiting for DDS delivery acknowledgment: "
        f"clocks={clock_count}, run_states={run_state_count}"
    )


def main() -> None:
    run_id = os.environ["SIM_RUN_ID"]
    output_root = Path(os.environ["SIM_OUTPUT_ROOT"])
    log_path = output_root / "logs" / "foundation.jsonl"
    observation_path = output_root / "foundation-observation.json"
    log_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = FoundationNode()
    try:
        with ExitStack() as stack:
            log_stream = stack.enter_context(log_path.open("w", encoding="utf-8"))

            def emit(name: str, sim_timestamp: float | None) -> None:
                event = StructuredEvent(
                    run_id=run_id,
                    module=MODULE,
                    severity="INFO",
                    event=name,
                    sim_timestamp=sim_timestamp,
                    wall_timestamp=datetime.now(timezone.utc),
                )
                write_event(sys.stdout, event)
                write_event(log_stream, event)

            emit("starting", None)
            node.wait_for_subscribers()
            _wait_for_observer_discovery(observation_path)

            _publish_and_wait_for_observation(
                observation_path,
                lambda: node.publish_clock(*SIMULATED_TIMES[0]),
                clock_count=1,
                run_state_count=0,
            )
            _publish_and_wait_for_observation(
                observation_path,
                lambda: node.publish_run_state(run_id, RunState.STARTING, *SIMULATED_TIMES[0]),
                clock_count=1,
                run_state_count=1,
            )

            Path("/run/foundation").mkdir(parents=True, exist_ok=True)
            Path("/run/foundation/ready").touch()
            _publish_and_wait_for_observation(
                observation_path,
                lambda: node.publish_run_state(run_id, RunState.READY, *SIMULATED_TIMES[0]),
                clock_count=1,
                run_state_count=2,
            )
            emit("ready", 0.0)

            _publish_and_wait_for_observation(
                observation_path,
                lambda: node.publish_clock(*SIMULATED_TIMES[1]),
                clock_count=2,
                run_state_count=2,
            )
            _publish_and_wait_for_observation(
                observation_path,
                lambda: node.publish_run_state(run_id, RunState.RUNNING, *SIMULATED_TIMES[1]),
                clock_count=2,
                run_state_count=3,
            )
            emit("clock_started", 0.05)

            _publish_and_wait_for_observation(
                observation_path,
                lambda: node.publish_clock(*SIMULATED_TIMES[2]),
                clock_count=3,
                run_state_count=3,
            )
            _publish_and_wait_for_observation(
                observation_path,
                lambda: node.publish_run_state(run_id, RunState.FINALIZING, *SIMULATED_TIMES[2]),
                clock_count=3,
                run_state_count=4,
            )
            emit("finalizing", 0.10)

            _publish_and_wait_for_observation(
                observation_path,
                lambda: node.publish_run_state(run_id, RunState.COMPLETED, *SIMULATED_TIMES[2]),
                clock_count=3,
                run_state_count=5,
            )
            emit("completed", 0.10)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
