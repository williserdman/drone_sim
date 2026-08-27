from dataclasses import FrozenInstanceError
import math

import pytest

from drone_sim_gazebo.ros_adapter import AdapterFault
from drone_sim_gazebo.ros_adapter.payload import PayloadTracker, PublicPayloadState


RUN_ID = "00000000-0000-4000-8000-000000000404"
INTERVAL_NS = 50_000_000
UNIT_QUATERNION = (0.0, 0.0, 0.0, 1.0)


def ready_tracker(*, aruco_id: int = 3, stamp_ns: int = INTERVAL_NS):
    tracker = PayloadTracker(
        run_id=RUN_ID,
        aruco_id=aruco_id,
        interval_ns=INTERVAL_NS,
    )
    tracker.accept_contact(stamp_ns, True)
    tracker.accept_attachment(stamp_ns, False)
    return tracker


def test_payload_tracker_emits_one_complete_sample_per_pose_tick():
    """Dropping a pose tick would make public physical truth incomplete."""
    tracker = ready_tracker()

    first = tracker.accept_pose(
        INTERVAL_NS,
        (1.0, 2.0, 0.0254),
        UNIT_QUATERNION,
    )
    tracker.accept_contact(2 * INTERVAL_NS, False)
    second = tracker.accept_pose(
        2 * INTERVAL_NS,
        (1.005, 2.0, 0.0254),
        UNIT_QUATERNION,
    )

    assert first == PublicPayloadState(
        run_id=RUN_ID,
        sim_timestamp_ns=INTERVAL_NS,
        aruco_id=3,
        position_xyz=(1.0, 2.0, 0.0254),
        orientation_xyzw=UNIT_QUATERNION,
        linear_velocity_xyz=(0.0, 0.0, 0.0),
        grounded=True,
        attached=False,
    )
    assert second.linear_velocity_xyz == pytest.approx((0.1, 0.0, 0.0))
    assert second.grounded is False
    assert tracker.accepted_poses == 2


def test_payload_tracker_uses_only_explicit_joint_state_for_attachment():
    """Payload proximity or grounded state must never synthesize attachment."""
    tracker = PayloadTracker(
        run_id=RUN_ID,
        aruco_id=2,
        interval_ns=INTERVAL_NS,
    )
    tracker.accept_contact(INTERVAL_NS, True)

    with pytest.raises(AdapterFault, match="attachment state"):
        tracker.accept_pose(INTERVAL_NS, (0.0, 0.0, 0.065), UNIT_QUATERNION)


def test_payload_tracker_accepts_only_known_physical_joint_states_and_latches():
    """An unknown plugin state must freeze publication instead of becoming false."""
    tracker = PayloadTracker(
        run_id=RUN_ID,
        aruco_id=4,
        interval_ns=INTERVAL_NS,
    )

    with pytest.raises(AdapterFault, match="attached.*detached") as rejected:
        tracker.accept_attachment(INTERVAL_NS, "unknown")
    with pytest.raises(AdapterFault) as replacement:
        tracker.accept_attachment(INTERVAL_NS, "attached")

    assert str(replacement.value) == str(rejected.value)


def test_payload_tracker_rejects_contact_older_than_one_pose_sample():
    """A stale grounded bit must not be joined to a current pose."""
    tracker = ready_tracker(stamp_ns=INTERVAL_NS)
    tracker.accept_pose(INTERVAL_NS, (0.0, 0.0, 0.0254), UNIT_QUATERNION)
    tracker.accept_pose(2 * INTERVAL_NS, (0.0, 0.0, 0.0254), UNIT_QUATERNION)

    with pytest.raises(AdapterFault, match="contact.*older than one sample"):
        tracker.accept_pose(
            3 * INTERVAL_NS,
            (0.0, 0.0, 0.0254),
            UNIT_QUATERNION,
        )


@pytest.mark.parametrize(
    "second_stamp",
    [INTERVAL_NS, INTERVAL_NS - 1, 2 * INTERVAL_NS + 1],
    ids=["duplicate", "regression", "gap"],
)
def test_payload_tracker_latches_pose_timestamp_regressions_and_gaps(second_stamp):
    """Every payload pose must remain on the exact 50 ms current-run grid."""
    tracker = ready_tracker()
    tracker.accept_pose(INTERVAL_NS, (0.0, 0.0, 0.0254), UNIT_QUATERNION)

    with pytest.raises(AdapterFault, match="exactly 50000000 ns") as rejected:
        tracker.accept_pose(second_stamp, (0.0, 0.0, 0.0254), UNIT_QUATERNION)
    with pytest.raises(AdapterFault) as replacement:
        tracker.accept_pose(
            2 * INTERVAL_NS,
            (0.0, 0.0, 0.0254),
            UNIT_QUATERNION,
        )

    assert str(replacement.value) == str(rejected.value)


@pytest.mark.parametrize(
    ("position", "orientation"),
    [
        ((math.nan, 0.0, 0.0), UNIT_QUATERNION),
        ((0.0, 0.0, 0.0), (0.0, math.inf, 0.0, 1.0)),
        ([0.0, 0.0, 0.0], UNIT_QUATERNION),
    ],
    ids=["nonfinite-position", "nonfinite-orientation", "mutable-position"],
)
def test_payload_tracker_rejects_malformed_physical_pose(position, orientation):
    """Malformed Gazebo poses must fail closed before a public sample exists."""
    tracker = ready_tracker()

    with pytest.raises(AdapterFault):
        tracker.accept_pose(INTERVAL_NS, position, orientation)


def test_public_payload_state_is_immutable():
    tracker = ready_tracker()
    state = tracker.accept_pose(
        INTERVAL_NS,
        (0.0, 0.0, 0.0254),
        UNIT_QUATERNION,
    )

    with pytest.raises(FrozenInstanceError):
        state.attached = True
