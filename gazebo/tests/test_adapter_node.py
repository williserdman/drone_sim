from __future__ import annotations

import time

import pytest

from drone_sim_gazebo.ros_adapter import AdapterFault
from drone_sim_gazebo.ros_adapter.payload import PublicPayloadState


RUN_ID = "11111111-1111-4111-8111-111111111111"
PUBLIC_EPOCH_NATIVE_NS = 90_000_000_000


def _set_stamp(stamp, timestamp_ns):
    stamp.sec, stamp.nanosec = divmod(timestamp_ns, 1_000_000_000)


def _spin_until(node, predicate, *, timeout=5.0):
    rclpy = pytest.importorskip("rclpy")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        rclpy.spin_once(node, timeout_sec=0.05)
    assert predicate()


def test_payload_state_message_preserves_all_physical_fields():
    """A ROS conversion must not replace Gazebo-owned pose or joint facts."""
    from drone_sim_gazebo.ros_adapter.payload import payload_state_message

    class Value:
        pass

    class Message:
        def __init__(self):
            self.sim_timestamp = Value()
            self.pose = Value()
            self.pose.position = Value()
            self.pose.orientation = Value()
            self.twist = Value()
            self.twist.linear = Value()
            self.twist.angular = Value()

    value = PublicPayloadState(
        run_id=RUN_ID,
        sim_timestamp_ns=50_000_000,
        aruco_id=3,
        position_xyz=(1.0, 2.0, 3.0),
        orientation_xyzw=(0.1, 0.2, 0.3, 0.9),
        linear_velocity_xyz=(4.0, 5.0, 6.0),
        grounded=True,
        attached=False,
    )

    message = payload_state_message(value, Message)

    assert message.run_id == RUN_ID
    assert (message.sim_timestamp.sec, message.sim_timestamp.nanosec) == (0, 50_000_000)
    assert message.aruco_id == 3
    assert (message.pose.position.x, message.pose.position.y, message.pose.position.z) == (1.0, 2.0, 3.0)
    assert (
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ) == (0.1, 0.2, 0.3, 0.9)
    assert (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z) == (4.0, 5.0, 6.0)
    assert message.grounded is True
    assert message.attached is False


def test_joint_truth_wire_preserves_authoritative_native_timestamp():
    """The adapter must use plugin sample time, never callback receipt time."""
    from drone_sim_gazebo.ros_adapter.payload import parse_joint_state

    assert parse_joint_state(
        "payload-joint-state-v1|90050000000|detached"
    ) == (90_050_000_000, "detached")
    with pytest.raises(AdapterFault, match="joint state"):
        parse_joint_state("attached")


def test_range_sequence_rejects_a_missing_first_public_tick():
    """Range cannot shift its count window after losing public 50 ms."""
    from drone_sim_gazebo.ros_adapter.model import RangeSequence

    sequence = RangeSequence(expected_samples=2, interval_ns=50_000_000)

    with pytest.raises(AdapterFault, match="first range sample.*50000000 ns"):
        sequence.accept(100_000_000)


def test_live_ros_node_offers_exact_public_topics_qos_and_no_ack_subscription():
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("simulation_interfaces.msg")
    from rclpy.qos import ReliabilityPolicy
    from drone_sim_gazebo.ros_adapter.node import GazeboAdapterNode

    rclpy.init()
    adapter = GazeboAdapterNode(
        run_id=RUN_ID,
        expected_frames=2,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
    )
    graph = rclpy.create_node("gazebo_adapter_contract_observer")
    expected = {
        "/clock": (ReliabilityPolicy.RELIABLE, 1000),
        "/camera/onboard/image_raw": (ReliabilityPolicy.RELIABLE, 100),
        "/camera/onboard/frame_metadata": (ReliabilityPolicy.RELIABLE, 100),
        "/camera/observer/image_raw": (ReliabilityPolicy.RELIABLE, 100),
        "/camera/observer/frame_metadata": (ReliabilityPolicy.RELIABLE, 100),
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


def test_competition_node_creates_exact_reliable_camera_sources_at_public_zero_once():
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("simulation_interfaces.msg")
    from rclpy.qos import ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from drone_sim_gazebo.ros_adapter.node import GazeboAdapterNode

    rclpy.init()
    adapter = GazeboAdapterNode(
        run_id=RUN_ID,
        expected_frames=2,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
        world_name="competition_mission",
        width_px=640,
        height_px=480,
    )
    graph = rclpy.create_node("competition_camera_source_observer")
    selected = "/gazebo/private/camera/competition_onboard/image"
    observer = "/gazebo/private/camera/observer/image"
    inherited = "/gazebo/private/camera/onboard/image"
    try:
        assert graph.get_subscriptions_info_by_topic(selected) == []
        assert graph.get_subscriptions_info_by_topic(observer) == []
        assert graph.get_subscriptions_info_by_topic(inherited) == []
        assert len(
            graph.get_publishers_info_by_topic("/camera/onboard/image_raw")
        ) == 1

        adapter.activate_output()
        warmup_clock = Clock()
        _set_stamp(warmup_clock.clock, PUBLIC_EPOCH_NATIVE_NS - 50_000_000)
        adapter._accept_clock(warmup_clock)
        rclpy.spin_once(graph, timeout_sec=0.1)
        for topic in (selected, observer):
            local = [
                subscription
                for subscription in adapter.subscriptions
                if subscription.topic_name == topic
            ]
            assert local == [], topic
            assert graph.get_subscriptions_info_by_topic(topic) == []

        epoch_clock = Clock()
        _set_stamp(epoch_clock.clock, PUBLIC_EPOCH_NATIVE_NS)
        adapter._accept_clock(epoch_clock)
        _spin_until(
            graph,
            lambda: all(
                len(graph.get_subscriptions_info_by_topic(topic)) == 1
                for topic in (selected, observer)
            ),
        )
        assert graph.get_subscriptions_info_by_topic(inherited) == []
        for topic in (selected, observer):
            endpoints = graph.get_subscriptions_info_by_topic(topic)
            assert endpoints[0].qos_profile.reliability is ReliabilityPolicy.RELIABLE
            local = [
                subscription
                for subscription in adapter.subscriptions
                if subscription.topic_name == topic
            ]
            assert len(local) == 1, topic
            assert local[0].qos_profile.depth == 10

        adapter.activate_output()
        adapter._accept_clock(epoch_clock)
        rclpy.spin_once(graph, timeout_sec=0.1)
        for topic in (selected, observer):
            assert len(graph.get_subscriptions_info_by_topic(topic)) == 1
        assert len(
            graph.get_publishers_info_by_topic("/camera/onboard/image_raw")
        ) == 1
    finally:
        graph.destroy_node()
        adapter.destroy_node()
        rclpy.shutdown()


def test_competition_node_consumes_only_reliable_recurrent_contact_truth():
    """Physical contact truth is recurrent, reliable, and private to the adapter."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("simulation_interfaces.msg")
    from rclpy.qos import ReliabilityPolicy
    from drone_sim_gazebo.ros_adapter.node import GazeboAdapterNode

    rclpy.init()
    adapter = GazeboAdapterNode(
        run_id=RUN_ID,
        expected_frames=2,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
        world_name="competition_mission",
        width_px=640,
        height_px=480,
    )
    graph = rclpy.create_node("competition_contact_source_observer")
    try:
        for aruco_id in (2, 3, 4):
            selected = f"/gazebo/private/payload_{aruco_id}/contact_state"
            legacy = f"/gazebo/private/payload_{aruco_id}/contacts"
            endpoints = graph.get_subscriptions_info_by_topic(selected)
            assert len(endpoints) == 1, selected
            assert endpoints[0].qos_profile.reliability is ReliabilityPolicy.RELIABLE
            local = [
                subscription
                for subscription in adapter.subscriptions
                if subscription.topic_name == selected
            ]
            assert len(local) == 1
            assert local[0].qos_profile.depth == 10
            assert graph.get_subscriptions_info_by_topic(legacy) == []
    finally:
        graph.destroy_node()
        adapter.destroy_node()
        rclpy.shutdown()


def test_competition_node_consumes_every_payload_pose_reliably():
    """A missed private pose sample must not create a false 50 ms grid fault."""
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("simulation_interfaces.msg")
    from rclpy.qos import ReliabilityPolicy
    from drone_sim_gazebo.ros_adapter.node import GazeboAdapterNode

    rclpy.init()
    adapter = GazeboAdapterNode(
        run_id=RUN_ID,
        expected_frames=2,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
        world_name="competition_mission",
        width_px=640,
        height_px=480,
    )
    graph = rclpy.create_node("competition_payload_pose_source_observer")
    try:
        for aruco_id in (2, 3, 4):
            topic = f"/gazebo/private/payload_{aruco_id}/pose"
            endpoints = graph.get_subscriptions_info_by_topic(topic)
            assert len(endpoints) == 1, topic
            assert endpoints[0].qos_profile.reliability is ReliabilityPolicy.RELIABLE
            local = [
                subscription
                for subscription in adapter.subscriptions
                if subscription.topic_name == topic
            ]
            assert len(local) == 1, topic
            assert local[0].qos_profile.depth == 10
    finally:
        graph.destroy_node()
        adapter.destroy_node()
        rclpy.shutdown()


def test_live_ros_node_faults_when_pre_zero_outputs_exceed_two_public_epochs():
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("simulation_interfaces.msg")
    from drone_sim_gazebo.ros_adapter import AdapterFault
    from drone_sim_gazebo.ros_adapter.node import GazeboAdapterNode

    rclpy.init()
    adapter = GazeboAdapterNode(
        run_id=RUN_ID,
        expected_frames=3,
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
    )
    try:
        adapter._emit(tuple(object() for _ in range(6)))

        with pytest.raises(AdapterFault, match="two public epochs"):
            adapter._emit((object(),))
    finally:
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
        public_epoch_native_ns=PUBLIC_EPOCH_NATIVE_NS,
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
        _set_stamp(warmup_clock.clock, PUBLIC_EPOCH_NATIVE_NS - 50_000_000)
        adapter._accept_clock(warmup_clock)
        adapter._accept_image("onboard", image(PUBLIC_EPOCH_NATIVE_NS - 50_000_000))
        adapter._accept_image("observer", image(PUBLIC_EPOCH_NATIVE_NS - 50_000_000))
        adapter._accept_odometry(odometry(PUBLIC_EPOCH_NATIVE_NS - 50_000_000))
        adapter._accept_contacts(contacts(PUBLIC_EPOCH_NATIVE_NS - 50_000_000))
        rclpy.spin_once(observer, timeout_sec=0.1)

        assert clocks == []
        assert images == []
        assert truths == []
        assert faults == []
        assert completions == []

        adapter.activate_output()
        adapter._accept_image("onboard", image(PUBLIC_EPOCH_NATIVE_NS - 50_000_000))
        adapter._accept_image("observer", image(PUBLIC_EPOCH_NATIVE_NS - 50_000_000))
        rclpy.spin_once(observer, timeout_sec=0.1)
        assert clocks == []
        assert images == []
        assert truths == []
        assert faults == []
        assert completions == []

        epoch_clock = Clock()
        _set_stamp(epoch_clock.clock, PUBLIC_EPOCH_NATIVE_NS)
        adapter._accept_clock(epoch_clock)
        _spin_until(observer, lambda: clocks == [0])

        first_public_native_ns = PUBLIC_EPOCH_NATIVE_NS + 50_000_000
        adapter._accept_image("onboard", image(first_public_native_ns))
        adapter._accept_image("observer", image(first_public_native_ns))
        adapter._accept_contacts(contacts(first_public_native_ns, in_contact=True))
        adapter._accept_odometry(odometry(first_public_native_ns))
        _spin_until(
            observer,
            lambda: len(images) == 2 and truths == [50_000_000],
        )

        assert images == [50_000_000, 50_000_000]
        delayed_clock = Clock()
        _set_stamp(delayed_clock.clock, PUBLIC_EPOCH_NATIVE_NS)
        adapter._accept_clock(delayed_clock)
        _set_stamp(delayed_clock.clock, first_public_native_ns)
        adapter._accept_clock(delayed_clock)
        rclpy.spin_once(observer, timeout_sec=0.1)
        assert clocks == [0, 50_000_000]
        assert faults == []
        assert len(completions) == 1

        # DDS may already have delivered the next physical samples to the
        # executor when the exact final frame completes the adapter.  Those
        # queued callbacks belong after the completed run boundary and must
        # be discarded rather than mutating the frozen adapter.
        adapter._accept_image("onboard", image(49_150_000_000))
        adapter._accept_image("observer", image(49_150_000_000))
        adapter._accept_contacts(contacts(49_150_000_000))
        adapter._accept_odometry(odometry(49_150_000_000))
        post_completion_clock = Clock()
        _set_stamp(post_completion_clock.clock, PUBLIC_EPOCH_NATIVE_NS + 100_000_000)
        adapter._accept_clock(post_completion_clock)
        rclpy.spin_once(observer, timeout_sec=0.1)

        assert faults == []
        assert len(completions) == 1
        assert images == [50_000_000, 50_000_000]
        assert truths == [50_000_000]

        adapter.freeze_output()
        frozen_clock = Clock()
        _set_stamp(frozen_clock.clock, PUBLIC_EPOCH_NATIVE_NS + 150_000_000)
        adapter._accept_clock(frozen_clock)
        adapter._accept_image("onboard", image(PUBLIC_EPOCH_NATIVE_NS + 100_000_000))
        rclpy.spin_once(observer, timeout_sec=0.1)

        assert clocks == [0, 50_000_000]
        assert images == [50_000_000, 50_000_000]
        assert truths == [50_000_000]
    finally:
        observer.destroy_node()
        adapter.destroy_node()
        rclpy.shutdown()
