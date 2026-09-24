from __future__ import annotations

from dataclasses import replace

import pytest

from drone_sim_companion.moving_precision import (
    MovingObservation,
    MovingPrecisionLanding,
    PrecisionStatus,
)
from drone_sim_companion.mission import CommandKind, Telemetry
from drone_sim_companion.operations import DroneOperations


def observation(
    timestamp_ns: int,
    *,
    sequence: int = 1,
    marker_id: int = 7,
    range_m: float = 5.0,
) -> MovingObservation:
    return MovingObservation(
        camera_timestamp_ns=timestamp_ns,
        camera_sequence=sequence,
        marker_id=marker_id,
        target_body_frd=(0.2, -0.1, range_m),
        range_m=range_m,
        range_timestamp_ns=timestamp_ns,
        attitude_rpy_rad=(0.01, -0.02, 0.0),
        attitude_timestamp_ns=timestamp_ns,
    )


def vehicle(*, speed: float = 0.0, clearance: float = 5.0, landed=False, armed=True):
    return {
        "horizontal_speed_m_s": speed,
        "relative_altitude_m": clearance,
        "landed": landed,
        "armed": armed,
        "mode": "GUIDED",
    }


def settled(policy: MovingPrecisionLanding) -> None:
    policy.tick(0, vehicle())
    policy.tick(500_000_000, vehicle())


def test_two_seconds_of_fresh_unique_tracking_requests_land_once() -> None:
    policy = MovingPrecisionLanding()
    policy.start(7, 45_000_000_000, 60_000_000_000)
    settled(policy)

    requested = []
    for sequence, stamp in enumerate(range(500_000_000, 2_500_000_001, 100_000_000), 1):
        policy.observe(observation(stamp, sequence=sequence))
        status = policy.tick(stamp, vehicle())
        if status.requested_mode is not None:
            requested.append(status.requested_mode)

    assert requested == ["LAND"]


def test_stale_wrong_marker_and_duplicate_frames_cannot_authorize_land() -> None:
    policy = MovingPrecisionLanding()
    policy.start(7, 45_000_000_000, 60_000_000_000)
    settled(policy)

    policy.observe(observation(500_000_000, marker_id=8))
    assert policy.tick(500_000_000, vehicle()).target_body_frd is None
    policy.observe(observation(500_000_000, sequence=2))
    assert policy.tick(800_000_001, vehicle()).target_body_frd is None
    policy.observe(observation(900_000_000, sequence=3))
    assert policy.tick(900_000_000, vehicle()).target_body_frd is not None
    policy.observe(observation(900_000_000, sequence=3))
    assert policy.tick(2_900_000_000, vehicle()).requested_mode is None


def test_settlement_must_be_slow_for_half_second_by_deadline() -> None:
    policy = MovingPrecisionLanding()
    policy.start(7, 1_000_000_000, 2_000_000_000)

    policy.tick(0, vehicle(speed=0.21))
    policy.tick(600_000_000, vehicle(speed=0.0))
    status = policy.tick(1_000_000_001, vehicle(speed=0.0))

    assert status.state == "failed"
    assert "settle" in status.error


def test_tracking_loss_above_clearance_requests_guided_and_fails() -> None:
    policy = MovingPrecisionLanding()
    policy.start(7, 45_000_000_000, 60_000_000_000)
    settled(policy)
    for sequence, stamp in enumerate(range(500_000_000, 2_500_000_001, 100_000_000), 1):
        policy.observe(observation(stamp, sequence=sequence))
        policy.tick(stamp, vehicle())

    status = policy.tick(3_000_000_001, vehicle(clearance=1.0))

    assert status.state == "failed"
    assert status.requested_mode == "GUIDED"
    assert "tracking" in status.error


def test_near_deck_handoff_keeps_targets_and_requires_touchdown_within_three_seconds() -> None:
    policy = MovingPrecisionLanding()
    policy.start(7, 45_000_000_000, 60_000_000_000)
    settled(policy)
    for sequence, stamp in enumerate(range(500_000_000, 2_500_000_001, 100_000_000), 1):
        policy.observe(observation(stamp, sequence=sequence, range_m=0.7))
        policy.tick(stamp, vehicle(clearance=0.7))

    policy.observe(observation(3_000_000_000, sequence=30, range_m=0.5))
    assert policy.tick(3_000_000_000, vehicle(clearance=0.5)).target_body_frd is not None
    assert policy.tick(5_500_000_000, vehicle(clearance=0.5)).state == "running"
    assert policy.tick(5_500_000_001, vehicle(clearance=0.5)).state == "failed"


def test_touchdown_and_disarm_succeeds_during_final_handoff() -> None:
    policy = MovingPrecisionLanding()
    policy.start(7, 45_000_000_000, 60_000_000_000)
    settled(policy)
    for sequence, stamp in enumerate(range(500_000_000, 2_500_000_001, 100_000_000), 1):
        policy.observe(observation(stamp, sequence=sequence, range_m=0.7))
        policy.tick(stamp, vehicle(clearance=0.7))

    status = policy.tick(
        3_000_000_000,
        vehicle(clearance=0.2, landed=True, armed=False),
    )
    assert status.state == "succeeded"


def test_touchdown_by_final_deadline_can_disarm_after_deadline() -> None:
    policy = MovingPrecisionLanding()
    policy.start(7, 45_000_000_000, 60_000_000_000)
    settled(policy)
    for sequence, stamp in enumerate(range(500_000_000, 2_500_000_001, 100_000_000), 1):
        policy.observe(observation(stamp, sequence=sequence, range_m=0.7))
        policy.tick(stamp, vehicle(clearance=5.0))

    touchdown = policy.tick(5_500_000_000, vehicle(clearance=5.0, landed=True, armed=True))
    disarmed = policy.tick(6_000_000_000, vehicle(clearance=5.0, landed=True, armed=False))

    assert touchdown.state == "running"
    assert disarmed.state == "succeeded"


def test_drone_operations_alone_applies_precision_target_and_mode_effects() -> None:
    class Vehicle:
        def __init__(self) -> None:
            self.commands = []
            self.targets = []

        def send(self, command, altitude_m) -> None:
            self.commands.append((command, altitude_m))

        def send_landing_target(self, forward, right, down) -> None:
            self.targets.append((forward, right, down))

    vehicle_adapter = Vehicle()
    operations = DroneOperations(
        vehicle_adapter,
        moving_precision=MovingPrecisionLanding(),
    )
    operations.observe(Telemetry(
        0, heartbeat=True, mode="GUIDED", armed=True, landed=False,
        horizontal_speed_m_s=0.0,
    ))
    operation_id = operations.start("precision_land", {
        "marker_id": 7,
        "settle_by_sim_s": 45,
        "acquire_by_sim_s": 60,
    }, timeout_sim_s=45)
    for sequence, stamp in enumerate(range(500_000_000, 2_500_000_001, 100_000_000), 1):
        operations.observe(Telemetry(
            stamp, heartbeat=True, mode="GUIDED", armed=True, landed=False,
            horizontal_speed_m_s=0.0,
        ))
        operations.observe_precision(observation(stamp, sequence=sequence))
        operations.tick(stamp)

    assert vehicle_adapter.targets[-1] == (0.2, -0.1, 5.0)
    assert vehicle_adapter.commands == [(CommandKind.LAND, None)]
    assert operations.operation_status(operation_id).state == "running"


def test_moving_profile_holds_north_yaw_during_eastbound_waypoint() -> None:
    class Vehicle:
        def __init__(self): self.waypoints = []
        def send_waypoint(self, latitude, longitude, altitude, *, yaw_rad=None):
            self.waypoints.append((latitude, longitude, altitude, yaw_rad))

    vehicle_adapter = Vehicle()
    operations = DroneOperations(
        vehicle_adapter,
        moving_precision=MovingPrecisionLanding(),
    )
    operations.observe(Telemetry(
        0, heartbeat=True, mode="GUIDED", armed=True, landed=False,
    ))

    operations.start("goto_waypoint", {
        "latitude_deg": 37.4003371,
        "longitude_deg": -122.079639322083,
        "altitude_m": 5.0,
    })

    assert vehicle_adapter.waypoints == [
        (37.4003371, -122.079639322083, 5.0, 0.0),
    ]


def test_guided_hold_failure_cannot_be_immediately_overwritten_by_recovery_land() -> None:
    class FailedPolicy:
        def start(self, *_args): pass
        def tick(self, *_args):
            return PrecisionStatus(
                "failed",
                "precision target tracking was lost above final clearance",
                requested_mode="GUIDED",
            )
        def abort(self, _reason): pass

    class Vehicle:
        def __init__(self): self.commands = []
        def send(self, command, altitude): self.commands.append((command, altitude))

    vehicle_adapter = Vehicle()
    operations = DroneOperations(vehicle_adapter, moving_precision=FailedPolicy())
    operations.observe(Telemetry(
        0, heartbeat=True, mode="GUIDED", armed=True, landed=False,
        horizontal_speed_m_s=0.0,
    ))
    operations.start("precision_land", {
        "marker_id": 7, "settle_by_sim_s": 45, "acquire_by_sim_s": 60,
    })

    assert operations.recover_land() is None
    assert vehicle_adapter.commands == [(CommandKind.SET_GUIDED, None)]


@pytest.mark.parametrize(
    ("marker_id", "settle", "acquire"),
    [(True, 1, 2), (-1, 1, 2), (7, 0, 2), (7, 2, 2), (7, 3, 2)],
)
def test_start_rejects_invalid_contract(marker_id, settle, acquire) -> None:
    with pytest.raises(ValueError):
        MovingPrecisionLanding().start(marker_id, settle, acquire)
