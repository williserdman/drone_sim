from __future__ import annotations

import time

import pytest


RUN_ID = "11111111-1111-4111-8111-111111111111"


def _set_stamp(stamp, timestamp_ns):
    stamp.sec, stamp.nanosec = divmod(timestamp_ns, 1_000_000_000)


def _spin_until(node, predicate, *, timeout=5.0):
    rclpy = pytest.importorskip("rclpy")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        rclpy.spin_once(node, timeout_sec=0.05)
    assert predicate()


def test_live_ros_node_offers_exact_public_topics_qos_and_no_ack_subscription():
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("simulation_interfaces.msg")
    from rclpy.qos import ReliabilityPolicy
    from drone_sim_gazebo.ros_adapter.node import GazeboAdapterNode

    rclpy.init()
    adapter = GazeboAdapterNode(run_id=RUN_ID, expected_frames=2)
    graph = rclpy.create_node("gazebo_adapter_contract_observer")
    expected = {
        "/clock": (ReliabilityPolicy.RELIABLE, 1000),
        "/camera/onboard/image_raw": (ReliabilityPolicy.RELIABLE, 5),
        "/camera/onboard/frame_metadata": (ReliabilityPolicy.RELIABLE, 5),
        "/camera/observer/image_raw": (ReliabilityPolicy.RELIABLE, 5),
        "/camera/observer/frame_metadata": (ReliabilityPolicy.RELIABLE, 5),
        "/simulation/ground_truth": (ReliabilityPolicy.RELIABLE, 10),
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


def test_live_ros_node_hides_warmup_then_rebases_every_public_stamp_at_activation():
    """Warmup time must not leak into the fixed public mission interval."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("simulation_interfaces.msg")
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ros_gz_interfaces.msg import Contact, Contacts
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    from simulation_interfaces.msg import GroundTruth
    from drone_sim_gazebo.ros_adapter.node import GazeboAdapterNode

    faults = []
    completions = []
    clocks = []
    images = []
    truths = []
    rclpy.init()
    adapter = GazeboAdapterNode(
        run_id=RUN_ID,
        expected_frames=1,
        on_completed=completions.append,
        on_fault=faults.append,
    )
    observer = rclpy.create_node("gazebo_adapter_activation_observer")
    best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    observer.create_subscription(
        Clock,
        "/clock",
        lambda message: clocks.append(
            message.clock.sec * 1_000_000_000 + message.clock.nanosec
        ),
        best_effort,
    )
    for topic in ("/camera/onboard/image_raw", "/camera/observer/image_raw"):
        observer.create_subscription(
            Image,
            topic,
            lambda message: images.append(
                message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec
            ),
            best_effort,
        )
    observer.create_subscription(
        GroundTruth,
        "/simulation/ground_truth",
        lambda message: truths.append(
            message.sim_timestamp.sec * 1_000_000_000
            + message.sim_timestamp.nanosec
        ),
        best_effort,
    )

    def image(timestamp_ns):
        message = Image()
        _set_stamp(message.header.stamp, timestamp_ns)
        message.height = 240
        message.width = 320
        message.encoding = "rgb8"
        message.step = 960
        message.data = bytes(320 * 240 * 3)
        return message

    def odometry(timestamp_ns):
        message = Odometry()
        _set_stamp(message.header.stamp, timestamp_ns)
        message.pose.pose.orientation.w = 1.0
        return message

    def contacts(timestamp_ns, *, in_contact=False):
        message = Contacts()
        _set_stamp(message.header.stamp, timestamp_ns)
        if in_contact:
            message.contacts.append(Contact())
        return message

    try:
        _spin_until(
            observer,
            lambda: all(
                adapter.count_subscribers(topic) >= 1
                for topic in (
                    "/clock",
                    "/camera/onboard/image_raw",
                    "/camera/observer/image_raw",
                    "/simulation/ground_truth",
                )
            ),
        )

        warmup_clock = Clock()
        _set_stamp(warmup_clock.clock, 49_025_000_000)
        adapter._accept_clock(warmup_clock)
        adapter._accept_image("onboard", image(49_000_000_000))
        adapter._accept_image("observer", image(49_000_000_000))
        adapter._accept_odometry(odometry(49_000_000_000))
        adapter._accept_contacts(contacts(49_000_000_000))
        rclpy.spin_once(observer, timeout_sec=0.1)

        assert clocks == []
        assert images == []
        assert truths == []
        assert faults == []
        assert completions == []

        adapter.activate_output()
        _spin_until(observer, lambda: clocks == [0])
        # Samples at the floored epoch may already be queued in a different
        # DDS reader and execute after the RUNNING callback.
        adapter._accept_image("onboard", image(49_000_000_000))
        adapter._accept_image("observer", image(49_000_000_000))
        adapter._accept_odometry(odometry(49_000_000_000))
        adapter._accept_contacts(contacts(49_000_000_000))
        rclpy.spin_once(observer, timeout_sec=0.1)
        assert images == []
        assert truths == []
        assert faults == []
        assert completions == []

        adapter._accept_image("onboard", image(49_050_000_000))
        adapter._accept_image("observer", image(49_050_000_000))
        adapter._accept_contacts(contacts(49_050_000_000, in_contact=True))
        adapter._accept_odometry(odometry(49_050_000_000))
        _spin_until(observer, lambda: len(images) == 2 and truths == [50_000_000])

        assert images == [50_000_000, 50_000_000]
        _spin_until(observer, lambda: clocks == [0, 50_000_000])
        delayed_clock = Clock()
        _set_stamp(delayed_clock.clock, 49_025_000_000)
        adapter._accept_clock(delayed_clock)
        _set_stamp(delayed_clock.clock, 49_050_000_000)
        adapter._accept_clock(delayed_clock)
        rclpy.spin_once(observer, timeout_sec=0.1)
        assert clocks == [0, 50_000_000]
        assert faults == []
        assert len(completions) == 1

        # DDS may already have delivered the next physical samples to the
        # executor when the exact final frame completes the adapter.  Those
        # queued callbacks belong after the completed run boundary and must
        # be discarded rather than mutating the frozen adapter.
        adapter._accept_image("onboard", image(49_100_000_000))
        adapter._accept_image("observer", image(49_100_000_000))
        adapter._accept_contacts(contacts(49_100_000_000))
        adapter._accept_odometry(odometry(49_100_000_000))
        rclpy.spin_once(observer, timeout_sec=0.1)

        assert faults == []
        assert len(completions) == 1
        assert images == [50_000_000, 50_000_000]
        assert truths == [50_000_000]

        adapter.freeze_output()
        frozen_clock = Clock()
        _set_stamp(frozen_clock.clock, 49_100_000_000)
        adapter._accept_clock(frozen_clock)
        adapter._accept_image("onboard", image(49_100_000_000))
        rclpy.spin_once(observer, timeout_sec=0.1)

        assert clocks == [0, 50_000_000]
        assert images == [50_000_000, 50_000_000]
        assert truths == [50_000_000]
    finally:
        observer.destroy_node()
        adapter.destroy_node()
        rclpy.shutdown()
