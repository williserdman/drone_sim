from __future__ import annotations

import importlib
import time

import pytest


RUN_ID = "11111111-1111-4111-8111-111111111111"


def test_depth_two_recorder_cache_retains_both_statuses_before_late_take():
    rclpy = pytest.importorskip("rclpy")
    messages = pytest.importorskip("simulation_interfaces.msg")
    duration_type = importlib.import_module("rclpy.duration").Duration
    qos_module = importlib.import_module("rclpy.qos")
    runtime_node = importlib.import_module("artifacts.runtime_node")

    rclpy.init()
    publisher_node = rclpy.create_node("artifact_status_startup_publisher")
    subscriber_node = rclpy.create_node("artifact_status_startup_recorder")
    publisher_qos = qos_module.QoSProfile(
        depth=1,
        reliability=qos_module.ReliabilityPolicy.RELIABLE,
        durability=qos_module.DurabilityPolicy.TRANSIENT_LOCAL,
    )
    recorder_qos = qos_module.QoSProfile(
        depth=2,
        reliability=qos_module.ReliabilityPolicy.RELIABLE,
        durability=qos_module.DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = publisher_node.create_publisher(
        messages.ArtifactStatus,
        "/phase2_test/artifact_status_startup",
        publisher_qos,
    )
    received = []
    subscriber_node.create_subscription(
        messages.ArtifactStatus,
        "/phase2_test/artifact_status_startup",
        lambda message: received.append(message.ready),
        recorder_qos,
    )
    deadline = time.monotonic() + 10.0
    try:
        while publisher.get_subscription_count() != 1 and time.monotonic() < deadline:
            rclpy.spin_once(publisher_node, timeout_sec=0.01)
        assert publisher.get_subscription_count() == 1

        runtime_node.publish_artifact_status(
            publisher,
            messages.ArtifactStatus,
            {
                "run_id": RUN_ID,
                "ready": False,
                "complete": False,
                "missing": ["onboard", "observer", "rosbag"],
                "manifest_path": "",
            },
            timestamp_ns=0,
        )
        acknowledged = False
        while not acknowledged and time.monotonic() < deadline:
            rclpy.spin_once(publisher_node, timeout_sec=0.01)
            acknowledged = publisher.wait_for_all_acked(
                timeout=duration_type(nanoseconds=0)
            )
        assert acknowledged is True

        runtime_node.publish_artifact_status(
            publisher,
            messages.ArtifactStatus,
            {
                "run_id": RUN_ID,
                "ready": True,
                "complete": False,
                "missing": [],
                "manifest_path": "",
            },
            timestamp_ns=0,
        )
        while len(received) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(publisher_node, timeout_sec=0.01)
            rclpy.spin_once(subscriber_node, timeout_sec=0.05)

        assert received == [False, True]
    finally:
        subscriber_node.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown()


def test_post_manifest_artifact_status_is_live_and_transient_local_for_late_subscriber():
    rclpy = pytest.importorskip("rclpy")
    messages = pytest.importorskip("simulation_interfaces.msg")
    qos_module = importlib.import_module("rclpy.qos")
    runtime_node = importlib.import_module("artifacts.runtime_node")
    publish_artifact_status = getattr(runtime_node, "publish_artifact_status", None)
    assert publish_artifact_status is not None, "production ArtifactStatus mapper is missing"

    rclpy.init()
    publisher_node = rclpy.create_node("artifact_status_late_publisher")
    subscriber_node = rclpy.create_node("artifact_status_late_subscriber")
    qos = qos_module.QoSProfile(
        depth=1,
        reliability=qos_module.ReliabilityPolicy.RELIABLE,
        durability=qos_module.DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = publisher_node.create_publisher(
        messages.ArtifactStatus,
        "/phase2_test/artifact_status",
        qos,
    )
    received = []
    try:
        publish_artifact_status(
            publisher,
            messages.ArtifactStatus,
            {
                "run_id": RUN_ID,
                "ready": True,
                "complete": False,
                "missing": ["rosbag", "video/observer.mp4"],
                "manifest_path": "manifest.json",
            },
            timestamp_ns=2_000_000_000,
        )
        subscriber_node.create_subscription(
            messages.ArtifactStatus,
            "/phase2_test/artifact_status",
            received.append,
            qos,
        )
        deadline = time.monotonic() + 10.0
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(publisher_node, timeout_sec=0.01)
            rclpy.spin_once(subscriber_node, timeout_sec=0.05)

        assert len(received) == 1
        status = received[0]
        assert status.run_id == RUN_ID
        assert (status.sim_timestamp.sec, status.sim_timestamp.nanosec) == (2, 0)
        assert status.ready is True
        assert status.complete is False
        assert list(status.missing) == ["rosbag", "video/observer.mp4"]
        assert status.manifest_path == "manifest.json"
    finally:
        subscriber_node.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown()
