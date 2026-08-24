from __future__ import annotations

import time

import pytest


RUN_ID = "11111111-1111-4111-8111-111111111111"


def test_live_ros_node_offers_exact_public_topics_qos_and_no_ack_subscription():
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("simulation_interfaces.msg")
    from rclpy.qos import ReliabilityPolicy
    from drone_sim_gazebo.ros_adapter.node import GazeboAdapterNode

    rclpy.init()
    adapter = GazeboAdapterNode(run_id=RUN_ID, expected_frames=2)
    graph = rclpy.create_node("gazebo_adapter_contract_observer")
    expected = {
        "/clock": (ReliabilityPolicy.BEST_EFFORT, 1),
        "/camera/onboard/image_raw": (ReliabilityPolicy.RELIABLE, 5),
        "/camera/onboard/frame_metadata": (ReliabilityPolicy.RELIABLE, 5),
        "/camera/observer/image_raw": (ReliabilityPolicy.RELIABLE, 5),
        "/camera/observer/frame_metadata": (ReliabilityPolicy.RELIABLE, 5),
        "/simulation/ground_truth": (ReliabilityPolicy.BEST_EFFORT, 10),
    }
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if all(graph.get_publishers_info_by_topic(topic) for topic in expected):
                break
            rclpy.spin_once(graph, timeout_sec=0.05)
        for topic, (reliability, depth) in expected.items():
            endpoints = graph.get_publishers_info_by_topic(topic)
            assert len(endpoints) == 1, topic
            assert endpoints[0].qos_profile.reliability is reliability
            local = [publisher for publisher in adapter.publishers
                     if publisher.topic_name == topic]
            assert len(local) == 1, topic
            assert local[0].qos_profile.depth == depth
        subscriptions = {name for name, _types in graph.get_topic_names_and_types()
                         if graph.get_subscriptions_info_by_topic(name)}
        assert "/simulation/camera_pair_ack" not in subscriptions
        contact_topic = (
            "/world/phase3_foundation/model/ground_plane/link/ground_link/sensor/"
            "iris_ground_contact/contact"
        )
        assert len(graph.get_subscriptions_info_by_topic(contact_topic)) == 1
        adapter.freeze_output()
        rclpy.spin_once(graph, timeout_sec=0.05)
        assert len(graph.get_subscriptions_info_by_topic(contact_topic)) == 1
    finally:
        graph.destroy_node()
        adapter.destroy_node()
        rclpy.shutdown()
