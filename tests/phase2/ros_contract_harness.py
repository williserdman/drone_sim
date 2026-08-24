#!/usr/bin/env python3
"""Isolated real-ROS contract harness for the synthetic Gazebo source."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from simulation_interfaces.msg import FrameMetadata, GroundTruth, RunState


RUN_ID = "11111111-1111-4111-8111-111111111111"
TIMEOUT = 30.0


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _atomic_json(path: Path, document: dict[str, Any]) -> None:
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(document), encoding="utf-8")
    os.replace(temporary, path)


class Observer(Node):
    def __init__(self) -> None:
        super().__init__("phase2_ros_contract_observer")
        self.clocks: list[int] = []
        self.images: dict[str, list[tuple[int, str]]] = {"onboard": [], "observer": []}
        self.metadata: dict[str, list[tuple[int, int]]] = {"onboard": [], "observer": []}
        self.ground_truth: list[int] = []
        self.last_pair_ack = -1
        self.state_publisher = self.create_publisher(
            RunState,
            "/simulation/run_state",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.pair_ack_publisher = self.create_publisher(
            FrameMetadata,
            "/simulation/camera_pair_ack",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            Clock,
            "/clock",
            lambda message: self.clocks.append(_stamp_ns(message.clock)),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_subscription(
            GroundTruth,
            "/simulation/ground_truth",
            lambda message: self.ground_truth.append(_stamp_ns(message.sim_timestamp)),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        for stream in ("onboard", "observer"):
            self.create_subscription(
                Image,
                f"/camera/{stream}/image_raw",
                lambda message, stream=stream: self.images[stream].append(
                    (_stamp_ns(message.header.stamp), hashlib.sha256(bytes(message.data)).hexdigest())
                ),
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
            )
            self.create_subscription(
                FrameMetadata,
                f"/camera/{stream}/frame_metadata",
                lambda message, stream=stream: self.metadata[stream].append(
                    (int(message.frame_id), _stamp_ns(message.sim_timestamp))
                ),
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
            )
            self.create_subscription(
                Image,
                f"/camera/{stream}/image_raw",
                lambda _message: None,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
            )
            self.create_subscription(
                FrameMetadata,
                f"/camera/{stream}/frame_metadata",
                lambda _message: None,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE),
            )

    def publish_state(self, state: int) -> None:
        message = RunState()
        message.run_id = RUN_ID
        message.state = state
        self.state_publisher.publish(message)

    def publish_pending_pair_ack(self) -> None:
        next_id = self.last_pair_ack + 1
        if any(len(self.images[stream]) <= next_id for stream in ("onboard", "observer")):
            return
        if any(len(self.metadata[stream]) <= next_id for stream in ("onboard", "observer")):
            return
        stamps = {
            self.images[stream][next_id][0] for stream in ("onboard", "observer")
        } | {
            self.metadata[stream][next_id][1] for stream in ("onboard", "observer")
        }
        assert len(stamps) == 1
        message = FrameMetadata()
        message.run_id = RUN_ID
        stamp = stamps.pop()
        message.sim_timestamp.sec = stamp // 1_000_000_000
        message.sim_timestamp.nanosec = stamp % 1_000_000_000
        message.frame_id = next_id
        message.stream = "aggregate"
        self.pair_ack_publisher.publish(message)
        self.last_pair_ack = next_id


def _spin_until(node: Observer, condition, label: str) -> None:
    deadline = time.monotonic() + TIMEOUT
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        node.publish_pending_pair_ack()
        if condition():
            return
    raise AssertionError(f"timed out waiting for {label}")


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="phase2-ros-contract-") as temporary:
        run = Path(temporary) / RUN_ID
        (run / ".control").mkdir(parents=True)
        (run / ".status").mkdir()
        (run / ".status/quiescence").mkdir()
        (run / "configuration").mkdir()
        _atomic_json(
            run / "configuration/run.json",
            {"startup_wall_seconds": TIMEOUT},
        )
        environment = {
            **os.environ,
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run),
            "SIM_CONFIG_PATH": str(run / "configuration/run.json"),
            "SIM_PHASE2_FAULT": "",
            "SIM_SYNTHETIC_WALL_DELAY_MS": "5",
        }
        process = subprocess.Popen(
            [sys.executable, str(Path(__file__).with_name("synthetic_gazebo.py"))],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=environment,
        )
        rclpy.init()
        node = Observer()
        try:
            _spin_until(
                node,
                lambda: node.count_publishers("/clock") == 1
                and node.count_publishers("/camera/onboard/image_raw") == 1
                and node.count_publishers("/camera/observer/image_raw") == 1,
                "publisher discovery",
            )
            deadline = time.monotonic() + 0.25
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            assert node.clocks == [], "synthetic source published before READY"

            while not node.clocks:
                node.publish_state(RunState.READY)
                rclpy.spin_once(node, timeout_sec=0.05)
            assert node.clocks == [0]
            while len(node.clocks) < 41:
                node.publish_state(RunState.RUNNING)
                rclpy.spin_once(node, timeout_sec=0.02)
                node.publish_pending_pair_ack()
            _spin_until(
                node,
                lambda: all(len(values) == 40 for values in node.images.values())
                and all(len(values) == 40 for values in node.metadata.values())
                and len(node.ground_truth) == 40,
                "all frames and ground truth",
            )
            assert node.clocks == list(range(0, 2_000_000_001, 50_000_000))
            expected_stamps = list(range(50_000_000, 2_000_000_001, 50_000_000))
            for stream in ("onboard", "observer"):
                assert [value[0] for value in node.images[stream]] == expected_stamps
                assert node.metadata[stream] == list(zip(range(40), expected_stamps, strict=True))
                assert len(set(value[1] for value in node.images[stream])) == 40
            assert node.ground_truth == expected_stamps
            source_finished = run / ".status/source-finished.json"
            _spin_until(node, source_finished.exists, "durable source-finished status")
            status = json.loads(source_finished.read_text())
            assert status == {
                "run_id": RUN_ID,
                "finished": True,
                "sim_timestamp_ns": 2_000_000_000,
            }

            gazebo_marker = run / ".status/quiescence/gazebo.json"
            while not gazebo_marker.exists():
                node.publish_state(RunState.FINALIZING)
                rclpy.spin_once(node, timeout_sec=0.05)
            assert json.loads(gazebo_marker.read_text()) == {
                "run_id": RUN_ID,
                "module": "gazebo",
                "quiescent": True,
            }
            assert not (run / ".status/runtime-frozen.json").exists()
            frozen_counts = (
                len(node.clocks),
                *(len(node.images[stream]) for stream in ("onboard", "observer")),
                *(len(node.metadata[stream]) for stream in ("onboard", "observer")),
                len(node.ground_truth),
            )
            node.publish_state(RunState.RUNNING)
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            assert frozen_counts == (
                len(node.clocks),
                *(len(node.images[stream]) for stream in ("onboard", "observer")),
                *(len(node.metadata[stream]) for stream in ("onboard", "observer")),
                len(node.ground_truth),
            )
            _atomic_json(
                run / ".control/terminal-committed.json",
                {
                    "run_id": RUN_ID,
                    "terminal_status": "COMPLETED",
                    "reason": "isolated ROS contract",
                    "manifest_path": "manifest.json",
                },
            )
            output, _ = process.communicate(timeout=TIMEOUT)
            assert process.returncode == 0, output
            assert "SYNTHETIC PHASE 2 FIXTURE" in (run / "gazebo/server.log").read_text()
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
