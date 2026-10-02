from types import SimpleNamespace
import time

import pytest

from drone_sim_gazebo.ros_adapter import AdapterFault
from drone_sim_gazebo.ros_adapter.aggregation import NativeOdometry
from drone_sim_gazebo.ros_adapter.landing_pad import (
    LandingPadTracker,
    landing_pad_state_message,
    vehicle_on_deck,
)


RUN_ID = "11111111-1111-4111-8111-111111111111"
STAMP = 50_000_000


def _odometry(stamp=STAMP):
    return NativeOdometry(
        sim_timestamp_ns=stamp,
        position_xyz=(10.0, 0.0, 0.1),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        linear_velocity_xyz=(0.5, 0.0, 0.0),
        angular_velocity_xyz=(0.0, 0.0, 0.0),
    )


def _contact(collision1, collision2):
    return SimpleNamespace(
        collision1=SimpleNamespace(name=collision1),
        collision2=SimpleNamespace(name=collision2),
    )


@pytest.mark.parametrize(
    "contacts",
    (
        [_contact("moving_pad::deck::deck_collision", "ground_plane::ground_link::ground_collision")],
        [_contact("iris::airframe::base_link::front_left_leg_collision", "ground_plane::ground_link::ground_collision")],
        [_contact("moving_pad::deck::deck_collision", "iris::airframe::base_link::base_collision")],
        [],
    ),
)
def test_only_iris_leg_and_deck_collision_pair_is_contact(contacts):
    assert vehicle_on_deck(contacts) is False


def test_reversed_iris_leg_and_deck_pair_is_contact():
    contacts = [
        _contact(
            "world::iris::airframe::base_link::rear_right_leg_collision",
            "world::moving_pad::deck::deck_collision",
        )
    ]

    assert vehicle_on_deck(contacts) is True


def test_pad_truth_requires_matching_twenty_hertz_odometry_and_contact():
    tracker = LandingPadTracker(run_id=RUN_ID, marker_id=7)

    assert tracker.accept_odometry(_odometry()) is None
    state = tracker.accept_contact(STAMP, False)

    assert state.run_id == RUN_ID
    assert state.sim_timestamp_ns == STAMP
    assert state.marker_id == 7
    assert state.position_xyz == (10.0, 0.0, 0.1)
    assert state.linear_velocity_xyz == (0.5, 0.0, 0.0)
    assert state.vehicle_in_contact is False


def test_pad_truth_aggregates_physics_rate_contact_identity_for_each_public_tick():
    tracker = LandingPadTracker(run_id=RUN_ID, marker_id=7)

    for stamp in range(1_000_000, STAMP, 1_000_000):
        assert tracker.accept_contact(stamp, stamp == 25_000_000) is None
    assert tracker.accept_odometry(_odometry()) is None
    state = tracker.accept_contact(STAMP, False)

    assert state.sim_timestamp_ns == STAMP
    assert state.vehicle_in_contact is True


def test_pad_truth_rejects_absent_and_regressing_evidence():
    tracker = LandingPadTracker(run_id=RUN_ID, marker_id=7)
    tracker.accept_odometry(_odometry())

    assert tracker.accepted_samples == 0
    with pytest.raises(AdapterFault, match="odometry timestamps must advance"):
        tracker.accept_odometry(_odometry())


def test_pad_message_preserves_measured_pose_twist_and_contact():
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

    tracker = LandingPadTracker(run_id=RUN_ID, marker_id=7)
    tracker.accept_contact(STAMP, True)
    state = tracker.accept_odometry(_odometry())

    message = landing_pad_state_message(state, Message)

    assert message.run_id == RUN_ID
    assert (message.sim_timestamp.sec, message.sim_timestamp.nanosec) == (0, STAMP)
    assert message.marker_id == 7
    assert (message.pose.position.x, message.pose.position.y, message.pose.position.z) == (10.0, 0.0, 0.1)
    assert (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z) == (0.5, 0.0, 0.0)
    assert message.vehicle_in_contact is True


def test_moving_pad_node_excludes_warmup_then_publishes_measured_public_tick():
    rclpy = pytest.importorskip("rclpy")
    messages = pytest.importorskip("simulation_interfaces.msg")
    LandingPadState = getattr(messages, "LandingPadState", None)
    if LandingPadState is None:
        pytest.skip("LandingPadState has not been rebuilt on this host")
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ros_gz_interfaces.msg import Contact, Contacts
    from rosgraph_msgs.msg import Clock
    from drone_sim_gazebo.ros_adapter.node import GazeboAdapterNode

    native_epoch = 90_000_000_000
    received = []

    def set_stamp(stamp, timestamp_ns):
        stamp.sec, stamp.nanosec = divmod(timestamp_ns, 1_000_000_000)

    def odometry(timestamp_ns, x):
        message = Odometry()
        set_stamp(message.header.stamp, timestamp_ns)
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        message.twist.twist.linear.x = 0.5
        return message

    def contacts(timestamp_ns):
        message = Contacts()
        set_stamp(message.header.stamp, timestamp_ns)
        contact = Contact()
        contact.collision1.name = "moving_pad::deck::deck_collision"
        contact.collision2.name = "iris::airframe::base_link::front_left_leg_collision"
        message.contacts.append(contact)
        return message

    rclpy.init()
    adapter = GazeboAdapterNode(
        run_id=RUN_ID,
        expected_frames=1,
        public_epoch_native_ns=native_epoch,
        world_name="moving_pad_landing",
        width_px=640,
        height_px=480,
    )
    observer = rclpy.create_node("landing_pad_state_observer")
    observer.create_subscription(
        LandingPadState,
        "/simulation/landing_pad_state",
        received.append,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
    )
    try:
        adapter.prepare_output_epoch()
        adapter._accept_landing_pad_odometry(
            odometry(native_epoch - 50_000_000, 9.975)
        )
        adapter._accept_landing_pad_contacts(
            contacts(native_epoch - 50_000_000)
        )
        assert adapter._landing_pad_tracker.accepted_samples == 0

        adapter.activate_output()
        clock = Clock()
        set_stamp(clock.clock, native_epoch)
        adapter._accept_clock(clock)
        adapter._accept_landing_pad_contacts(contacts(native_epoch + STAMP))
        adapter._accept_landing_pad_odometry(
            odometry(native_epoch + STAMP, 10.025)
        )
        deadline = time.monotonic() + 2.0
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.05)

        assert len(received) == 1
        assert (received[0].sim_timestamp.sec, received[0].sim_timestamp.nanosec) == (0, STAMP)
        assert received[0].pose.position.x == pytest.approx(10.025)
        assert received[0].twist.linear.x == pytest.approx(0.5)
        assert received[0].vehicle_in_contact is True
    finally:
        observer.destroy_node()
        adapter.destroy_node()
        rclpy.shutdown()
