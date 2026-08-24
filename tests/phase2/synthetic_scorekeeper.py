#!/usr/bin/env python3
"""Synthetic score-event publisher; not mission-scoring evidence."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any


def scoring_result(run_id: str, scoring_config: Path) -> dict[str, Any]:
    return {
        "run_id": run_id,
        "fixture": True,
        "validity": "synthetic infrastructure evidence only",
        "achieved_score": 0.0,
        "maximum_available_score": 0.0,
        "scoring_checksum": hashlib.sha256(scoring_config.read_bytes()).hexdigest(),
        "evidence_paths": ["scoring/events.jsonl"],
    }


def _assign_stamp(stamp: Any, timestamp_ns: int) -> None:
    stamp.sec = timestamp_ns // 1_000_000_000
    stamp.nanosec = timestamp_ns % 1_000_000_000


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import RunState, ScoreEvent
    from artifacts.runtime_protocol import RuntimeProtocol
    from artifacts.structured_log import StructuredEvent, write_event
    from module_stub import write_bytes_atomic

    run_id = os.environ["SIM_RUN_ID"]
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"])
    scoring_config = Path(__file__).with_name("scoring.json")
    result_path = run_directory / "scoring/result.json"
    events_path = run_directory / "scoring/events.jsonl"
    write_bytes_atomic(
        result_path,
        (json.dumps(scoring_result(run_id, scoring_config), sort_keys=True, separators=(",", ":")) + "\n").encode(),
    )
    protocol = RuntimeProtocol(run_directory, run_id)
    rclpy.init()
    node = Node("synthetic_scorekeeper")
    publisher = node.create_publisher(
        ScoreEvent,
        "/simulation/score_events",
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
    )
    running = False
    published = False
    finalizing = False
    last_stamp = 0

    def emit(event: str) -> None:
        write_event(
            sys.stdout,
            StructuredEvent(
                run_id=run_id, module="scorekeeper", severity="INFO", event=event,
                sim_timestamp=last_stamp / 1e9, wall_timestamp=datetime.now(timezone.utc),
                fields={"fixture": True},
            ),
        )

    def state_callback(message: Any) -> None:
        nonlocal running, finalizing
        if message.run_id != run_id:
            return
        if message.state == RunState.RUNNING:
            running = True
        elif message.state == RunState.FINALIZING and not finalizing:
            emit("finalizing")
            finalizing = True

    def clock_callback(message: Any) -> None:
        nonlocal published, last_stamp
        stamp = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
        last_stamp = stamp
        if not running or finalizing or published or stamp < 1_000_000_000:
            return
        event = ScoreEvent()
        event.run_id = run_id
        _assign_stamp(event.sim_timestamp, 1_000_000_000)
        event.event_id = 0
        event.event_type = "synthetic_fixture"
        event.value = 0.0
        event.evidence_ref = "scoring/events.jsonl"
        publisher.publish(event)
        write_bytes_atomic(
            events_path,
            (json.dumps({
                "run_id": run_id, "fixture": True, "event_id": 0,
                "sim_timestamp_ns": 1_000_000_000, "event_type": "synthetic_fixture", "value": 0.0,
            }, sort_keys=True, separators=(",", ":")) + "\n").encode(),
        )
        published = True

    state_qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(RunState, "/simulation/run_state", state_callback, state_qos)
    node.create_subscription(
        Clock, "/clock", clock_callback,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    emit("starting")
    emit("ready")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if finalizing and protocol.read_terminal_committed() is not None:
                break
    finally:
        node.destroy_node()
        protocol.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
