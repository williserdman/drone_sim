#!/usr/bin/env python3
"""Run a deterministic synthetic ROS lifecycle for foundation verification."""

from contextlib import ExitStack
from datetime import datetime, timezone
import os
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from simulation_interfaces.msg import RunState

from artifacts import StructuredEvent, write_event


MODULE = "foundation"
SIMULATED_TIMES = ((0, 0), (0, 50_000_000), (0, 100_000_000))


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


def main() -> None:
    run_id = os.environ["SIM_RUN_ID"]
    output_root = Path(os.environ["SIM_OUTPUT_ROOT"])
    log_path = output_root / "logs" / "foundation.jsonl"
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
            node.publish_clock(*SIMULATED_TIMES[0])
            node.publish_run_state(run_id, RunState.STARTING, *SIMULATED_TIMES[0])

            Path("/run/foundation").mkdir(parents=True, exist_ok=True)
            Path("/run/foundation/ready").touch()
            node.publish_run_state(run_id, RunState.READY, *SIMULATED_TIMES[0])
            emit("ready", 0.0)

            node.publish_clock(*SIMULATED_TIMES[1])
            node.publish_run_state(run_id, RunState.RUNNING, *SIMULATED_TIMES[1])
            emit("clock_started", 0.05)

            node.publish_clock(*SIMULATED_TIMES[2])
            node.publish_run_state(run_id, RunState.FINALIZING, *SIMULATED_TIMES[2])
            emit("finalizing", 0.10)

            node.publish_run_state(run_id, RunState.COMPLETED, *SIMULATED_TIMES[2])
            emit("completed", 0.10)
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
