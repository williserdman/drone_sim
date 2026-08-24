"""ROS 2 process adapter for the permanently inactive descent_v1 scenario."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import sys
from typing import Any

from artifacts.runtime_protocol import RuntimeProtocol, canonical_run_id

from .controller import ScenarioController
from .scenario import InactiveScenarioEvent, ScenarioPolicy


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def assign_stamp(stamp: Any, timestamp_ns: int) -> None:
    stamp.sec = timestamp_ns // 1_000_000_000
    stamp.nanosec = timestamp_ns % 1_000_000_000


def main() -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import RunState, ScenarioEvent

    run_id = canonical_run_id(os.environ["SIM_RUN_ID"])
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"])
    protocol = RuntimeProtocol(run_directory, run_id)
    rclpy.init()
    node = Node("drone_sim_electromagnet")
    publisher = node.create_publisher(
        ScenarioEvent,
        "/simulation/scenario_events",
        QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )

    def publish(event: InactiveScenarioEvent) -> None:
        message = ScenarioEvent()
        message.run_id = event.run_id
        assign_stamp(message.sim_timestamp, event.timestamp_ns)
        message.event_id = event.event_id
        message.magnet_id = event.magnet_id
        message.state = event.state
        publisher.publish(message)

    controller = ScenarioController(
        run_id=run_id,
        policy=ScenarioPolicy(run_id=run_id),
        publish=publish,
        protocol=protocol,
        stream=sys.stdout,
    )
    finalizing = False
    requested_stop = False
    last_timestamp_ns: int | None = None
    failure: str | None = None

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal requested_stop
        requested_stop = True

    def clock_callback(message: Any) -> None:
        nonlocal last_timestamp_ns, failure
        value = stamp_ns(message.clock)
        try:
            controller.observe_clock(value)
            last_timestamp_ns = value
        except Exception as error:
            failure = str(error)

    def state_callback(message: Any) -> None:
        nonlocal finalizing
        if message.run_id == run_id and message.state == RunState.FINALIZING:
            finalizing = True

    node.create_subscription(
        Clock,
        "/clock",
        clock_callback,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    node.create_subscription(
        RunState,
        "/simulation/run_state",
        state_callback,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    controller.mark_ready()
    exit_code = 0
    try:
        while rclpy.ok() and not finalizing and not requested_stop:
            rclpy.spin_once(node, timeout_sec=0.05)
            if failure is not None:
                controller.fail(last_timestamp_ns, failure)
                exit_code = 1
                break
            if protocol.read_finalize_request() is not None:
                finalizing = True
    finally:
        controller.finalize(last_timestamp_ns)
        node.destroy_node()
        protocol.close()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["assign_stamp", "main", "stamp_ns"]
