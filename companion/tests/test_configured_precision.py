from __future__ import annotations

import pytest

from drone_sim_companion.configured_precision import MarkerObservation, PrecisionLanding
from drone_sim_companion.mission import Ack, CommandKind


class Vehicle:
    def __init__(self) -> None:
        self.commands: list[tuple] = []

    def send(self, command: CommandKind, altitude_m: float | None) -> None:
        self.commands.append((command, altitude_m))

    def send_waypoint(
        self, latitude_deg: float, longitude_deg: float, altitude_m: float
    ) -> None:
        self.commands.append(
            ("waypoint", latitude_deg, longitude_deg, altitude_m)
        )

    def send_landing_target(
        self, forward_m: float, right_m: float, down_m: float
    ) -> None:
        self.commands.append(("landing_target", forward_m, right_m, down_m))


def vehicle_state(
    stamp: int,
    *,
    mode: str = "GUIDED",
    armed: bool = True,
    landed: bool = False,
    latitude_deg: float = 37.4,
    longitude_deg: float = -122.08,
    relative_altitude_m: float = 4.572,
) -> dict:
    names = (
        "mode",
        "armed",
        "landed",
        "latitude_deg",
        "longitude_deg",
        "relative_altitude_m",
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "horizontal_speed_m_s",
        "vertical_speed_m_s",
        "clearance_m",
    )
    return {
        "mode": mode,
        "armed": armed,
        "landed": landed,
        "latitude_deg": latitude_deg,
        "longitude_deg": longitude_deg,
        "relative_altitude_m": relative_altitude_m,
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
        "yaw_rad": 0.0,
        "horizontal_speed_m_s": 0.0,
        "vertical_speed_m_s": 0.0,
        "observed_at_ns": {name: stamp for name in names},
    }


def marker(
    stamp: int,
    sequence: int,
    *,
    aruco_id: int = 3,
    forward: float = 0.03,
) -> MarkerObservation:
    return MarkerObservation(
        timestamp_ns=stamp,
        sequence=sequence,
        aruco_id=aruco_id,
        forward_m=forward,
        right_m=0.02,
        down_m=4.5,
    )


def advance_to_search(
    policy: PrecisionLanding, vehicle: Vehicle
) -> None:
    assert policy.tick(0, vehicle_state(0, relative_altitude_m=10.0), None, 10.0) is False
    assert vehicle.commands == [("waypoint", 37.4, -122.08, 4.572)]
    assert policy.tick(50_000_000, vehicle_state(50_000_000), None, 4.572) is False


def acquire_anchor(policy: PrecisionLanding, vehicle: Vehicle) -> None:
    advance_to_search(policy, vehicle)
    for sequence in range(1, 6):
        stamp = sequence * 100_000_000
        assert policy.tick(stamp, vehicle_state(stamp), marker(stamp, sequence), 4.572) is False
    assert vehicle.commands[-1] == (CommandKind.LAND, None)


def test_centered_anchor_starts_land_and_completes_only_after_touchdown_disarm() -> None:
    vehicle = Vehicle()
    policy = PrecisionLanding(vehicle, aruco_id=3, started_at_ns=0, timeout_sim_s=20)
    acquire_anchor(policy, vehicle)
    policy.observe_ack(Ack(CommandKind.LAND, True, 0))

    assert policy.tick(
        600_000_000,
        vehicle_state(600_000_000, mode="LAND"),
        marker(600_000_000, 6),
        3.0,
    ) is False
    assert vehicle.commands[-1] == ("landing_target", 0.03, 0.02, 4.5)

    assert policy.tick(
        700_000_000,
        vehicle_state(700_000_000, mode="LAND", armed=True, landed=True),
        None,
        0.1,
    ) is False
    assert policy.tick(
        800_000_000,
        vehicle_state(800_000_000, mode="LAND", armed=False, landed=True),
        None,
        0.1,
    ) is True


def test_stale_marker_cannot_start_land_or_finish() -> None:
    vehicle = Vehicle()
    policy = PrecisionLanding(vehicle, aruco_id=3, started_at_ns=0, timeout_sim_s=1)
    advance_to_search(policy, vehicle)

    stale = marker(0, 1)
    for stamp in (600_000_000, 900_000_000):
        assert policy.tick(stamp, vehicle_state(stamp), stale, 4.572) is False

    with pytest.raises(TimeoutError, match="precision landing"):
        policy.tick(1_000_000_000, vehicle_state(1_000_000_000), stale, 4.572)

    assert (CommandKind.LAND, None) not in vehicle.commands


def test_negative_land_ack_fails_the_operation() -> None:
    vehicle = Vehicle()
    policy = PrecisionLanding(vehicle, aruco_id=3, started_at_ns=0, timeout_sim_s=20)
    acquire_anchor(policy, vehicle)

    with pytest.raises(RuntimeError, match="LAND command rejected"):
        policy.observe_ack(Ack(CommandKind.LAND, False, 4))


def test_land_mode_requires_a_postcommand_observation() -> None:
    vehicle = Vehicle()
    policy = PrecisionLanding(vehicle, aruco_id=3, started_at_ns=0, timeout_sim_s=20)
    acquire_anchor(policy, vehicle)
    policy.observe_ack(Ack(CommandKind.LAND, True, 0))

    old_mode = vehicle_state(600_000_000, mode="LAND")
    old_mode["observed_at_ns"]["mode"] = 400_000_000
    assert policy.tick(600_000_000, old_mode, marker(600_000_000, 6), 3.0) is False
    assert not any(command[0] == "landing_target" for command in vehicle.commands)

    assert policy.tick(
        700_000_000,
        vehicle_state(700_000_000, mode="LAND"),
        marker(700_000_000, 7),
        3.0,
    ) is False
    assert vehicle.commands[-1][0] == "landing_target"


def test_lost_target_holds_in_guided_until_five_fresh_frames_reacquire() -> None:
    vehicle = Vehicle()
    policy = PrecisionLanding(vehicle, aruco_id=3, started_at_ns=0, timeout_sim_s=20)
    acquire_anchor(policy, vehicle)
    policy.observe_ack(Ack(CommandKind.LAND, True, 0))
    policy.tick(
        600_000_000,
        vehicle_state(600_000_000, mode="LAND"),
        marker(600_000_000, 6),
        3.0,
    )

    assert policy.tick(
        1_100_000_000,
        vehicle_state(1_100_000_000, mode="LAND"),
        None,
        3.0,
    ) is False
    assert vehicle.commands[-1] == (CommandKind.SET_GUIDED, None)

    policy.observe_ack(Ack(CommandKind.SET_GUIDED, True, 0))
    assert policy.tick(
        1_200_000_000,
        vehicle_state(1_200_000_000),
        None,
        3.0,
    ) is False
    assert vehicle.commands[-1][0] == "waypoint"

    for sequence in range(7, 12):
        stamp = 1_200_000_000 + (sequence - 6) * 100_000_000
        policy.tick(stamp, vehicle_state(stamp), marker(stamp, sequence), 3.0)

    assert vehicle.commands[-1] == (CommandKind.LAND, None)


@pytest.mark.parametrize("bad_frame", ["off_center", "stale", "wrong_id"])
def test_hold_ignores_duplicate_callbacks_but_bad_frame_resets_reacquisition(
    bad_frame: str,
) -> None:
    vehicle = Vehicle()
    policy = PrecisionLanding(vehicle, aruco_id=3, started_at_ns=0, timeout_sim_s=20)
    acquire_anchor(policy, vehicle)
    policy.observe_ack(Ack(CommandKind.LAND, True, 0))
    policy.tick(
        600_000_000,
        vehicle_state(600_000_000, mode="LAND"),
        marker(600_000_000, 6),
        3.0,
    )
    policy.tick(
        1_100_000_000,
        vehicle_state(1_100_000_000, mode="LAND"),
        None,
        3.0,
    )
    policy.observe_ack(Ack(CommandKind.SET_GUIDED, True, 0))
    policy.tick(1_200_000_000, vehicle_state(1_200_000_000), None, 3.0)

    def observe_with_duplicate(stamp: int, sequence: int, *, forward: float = 0.03) -> None:
        observation = marker(stamp, sequence, forward=forward)
        policy.tick(stamp, vehicle_state(stamp), observation, 3.0)
        duplicate_stamp = stamp + 20_000_000
        policy.tick(
            duplicate_stamp,
            vehicle_state(duplicate_stamp),
            observation,
            3.0,
        )

    observe_with_duplicate(1_300_000_000, 7)
    observe_with_duplicate(1_400_000_000, 8)
    bad_observation = {
        "off_center": marker(1_500_000_000, 9, forward=0.10),
        "stale": marker(1_000_000_000, 9),
        "wrong_id": marker(1_500_000_000, 9, aruco_id=99),
    }[bad_frame]
    policy.tick(
        1_500_000_000,
        vehicle_state(1_500_000_000),
        bad_observation,
        3.0,
    )
    policy.tick(
        1_520_000_000,
        vehicle_state(1_520_000_000),
        bad_observation,
        3.0,
    )

    for sequence in range(10, 14):
        observe_with_duplicate(
            1_600_000_000 + (sequence - 10) * 100_000_000,
            sequence,
        )
    assert vehicle.commands.count((CommandKind.LAND, None)) == 1

    observe_with_duplicate(2_000_000_000, 14)
    assert vehicle.commands.count((CommandKind.LAND, None)) == 2
