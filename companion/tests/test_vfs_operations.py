from __future__ import annotations

from collections import deque
from types import SimpleNamespace

import pytest

from drone_sim_companion.configured_vision import MarkerObservation, RangeObservation
from drone_sim_companion.mission import CommandKind, Telemetry
from drone_sim_companion.operations import DroneOperations
from drone_sim_companion.precision_landing import PRECISION_PARAMETERS, offset_gps


class Vehicle:
    def __init__(self) -> None:
        self.commands: list[tuple] = []

    def send(self, command, altitude_m) -> None:
        self.commands.append((command, altitude_m))

    def send_waypoint(self, latitude_deg, longitude_deg, altitude_m) -> None:
        self.commands.append(("waypoint", latitude_deg, longitude_deg, altitude_m))

    def send_mode(self, mode: str) -> None:
        self.commands.append(("mode", mode))

    def send_landing_target(self, vector) -> None:
        self.commands.append(("landing_target", vector))


class Vision:
    def __init__(self, observations=()) -> None:
        self.observations = deque(observations)
        self.requested: list[tuple[int, int]] = []
        self.cancelled = False

    def request(self, aruco_id: int, after_sequence: int) -> None:
        self.requested.append((aruco_id, after_sequence))

    def poll(self) -> MarkerObservation | None:
        return self.observations.popleft() if self.observations else None

    def cancel(self) -> None:
        self.cancelled = True

    def add(self, *observations: MarkerObservation) -> None:
        self.observations.extend(observations)


class Rangefinder:
    def __init__(self, distance_m: float = 4.572) -> None:
        self.distance_m = distance_m
        self.sequence = 0

    def latest(self, timestamp_ns: int) -> RangeObservation:
        self.sequence += 1
        return RangeObservation(self.sequence, timestamp_ns, self.distance_m)


class Payloads:
    def __init__(self) -> None:
        self.requests = []
        self.confirmations = {}
        self.cancelled = False

    def request(self, action, aruco_id, request_id, delivery_mode) -> None:
        self.requests.append((action, aruco_id, request_id, delivery_mode))

    def confirmation(self, request_id):
        return self.confirmations.get(request_id)

    def cancel(self) -> None:
        self.cancelled = True


class Phases:
    def __init__(self) -> None:
        self.events = []
        self.decisions = {}

    def begin(self, phase, request_id, stamp_ns) -> None:
        self.events.append(("begin", phase, "", request_id, stamp_ns))

    def check(self, phase, checkpoint, request_id, stamp_ns) -> None:
        self.events.append(("check", phase, checkpoint, request_id, stamp_ns))

    def decision(self, request_id):
        return self.decisions.get(request_id)


class Attempt:
    active_elapsed_ns = 0
    approval_timestamp_ns = 0
    state = 0


def course() -> SimpleNamespace:
    waypoint = lambda x, y: SimpleNamespace(center_xy_m=(x, y))
    return SimpleNamespace(
        waypoints={"H": waypoint(0.0, 0.0), "D": waypoint(0.0, 0.0)},
        attempt=SimpleNamespace(
            acquisition_agl_m=4.572,
            minimum_agl_m=9.144,
            recovery_timeout_s=60.0,
        ),
    )


def scenario() -> SimpleNamespace:
    return SimpleNamespace(payload=lambda _aruco_id: SimpleNamespace(
        delivery="C", release_offset_xy_m=(0.0, 0.28)))


def observe_flight(operations: DroneOperations, stamp: int, **changes) -> None:
    fields = dict(
        heartbeat=True,
        mode="GUIDED",
        armed=True,
        landed=False,
        relative_altitude_m=10.0,
        latitude_deg=41.0,
        longitude_deg=-81.0,
        horizontal_velocity_north_m_s=0.0,
        horizontal_velocity_east_m_s=0.0,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.0,
        home_latitude_deg=41.0,
        home_longitude_deg=-81.0,
        failsafe=False,
    )
    fields.update(changes)
    operations.observe(Telemetry(stamp, **fields))


def test_course_waypoint_requires_one_second_below_horizontal_speed_limit() -> None:
    vehicle = Vehicle()
    operations = DroneOperations(vehicle, course=course(), scenario=scenario())
    observe_flight(operations, 0)

    operation_id = operations.start(
        "goto_course_waypoint",
        {"waypoint": "D", "altitude_m": 10.0, "tolerance_m": 0.5},
    )
    assert vehicle.commands and vehicle.commands[0][0] == "waypoint"

    observe_flight(
        operations,
        50_000_000,
        horizontal_velocity_north_m_s=0.10,
    )
    observe_flight(operations, 100_000_000)
    observe_flight(operations, 1_099_999_999)
    assert operations.operation_status(operation_id).state == "running"
    observe_flight(operations, 1_100_000_000)
    assert operations.operation_status(operation_id).state == "succeeded"


@pytest.mark.parametrize(
    ("target_args", "east_m", "north_m"),
    [
        ({"waypoint": "H"}, 0.0, 0.0),
        ({"payload_target": 2}, 11.0, 22.0),
    ],
)
def test_course_navigation_keeps_first_complete_home_as_enu_origin(
    target_args, east_m, north_m
) -> None:
    waypoint = lambda x, y: SimpleNamespace(center_xy_m=(x, y))
    frozen_course = SimpleNamespace(
        waypoints={"H": waypoint(0.0, 0.0), "P": waypoint(10.0, 20.0)},
        attempt=SimpleNamespace(),
    )
    frozen_scenario = SimpleNamespace(payload=lambda _aruco_id: SimpleNamespace(
        delivery="P", release_offset_xy_m=(1.0, 2.0)
    ))
    vehicle = Vehicle()
    operations = DroneOperations(
        vehicle, course=frozen_course, scenario=frozen_scenario
    )
    observe_flight(
        operations, 0, home_latitude_deg=41.0, home_longitude_deg=-81.0
    )
    observe_flight(
        operations, 1, home_latitude_deg=41.001, home_longitude_deg=-81.002
    )

    operation_id = operations.start(
        "goto_course_waypoint", {**target_args, "altitude_m": 10.0}
    )

    assert operations.operation_status(operation_id).state == "running"
    expected = offset_gps(41.0, -81.0, north_m, east_m)
    assert vehicle.commands[-1][1:3] == pytest.approx(expected)
    state = operations.read_vehicle_state()
    assert state["home_latitude_deg"] == pytest.approx(41.001)
    assert state["home_longitude_deg"] == pytest.approx(-81.002)


def test_course_navigation_without_first_home_origin_sends_no_command() -> None:
    vehicle = Vehicle()
    operations = DroneOperations(vehicle, course=course(), scenario=scenario())
    observe_flight(
        operations, 0, home_latitude_deg=None, home_longitude_deg=None
    )

    operation_id = operations.start(
        "goto_course_waypoint", {"waypoint": "H", "altitude_m": 10.0}
    )

    assert operations.operation_status(operation_id).state == "failed"
    assert vehicle.commands == []


def test_find_marker_needs_five_new_consistent_requested_id_frames() -> None:
    observations = [
        MarkerObservation(2, 1, 0, 0, 0.0, 0.0, 4.572),
        MarkerObservation(3, 2, 50_000_000, 50_000_000, 0.0, 0.0, 4.572),
        MarkerObservation(2, 2, 100_000_000, 100_000_000, 0.0, 0.0, 4.572),
        MarkerObservation(2, 3, 150_000_000, 150_000_000, 0.0, 0.0, 4.572),
        MarkerObservation(2, 4, 200_000_000, 200_000_000, 0.0, 0.0, 4.572),
        MarkerObservation(2, 5, 250_000_000, 250_000_000, 0.0, 0.0, 4.572),
        MarkerObservation(2, 6, 300_000_000, 300_000_000, 0.0, 0.0, 4.572),
    ]
    vision = Vision(observations)
    vehicle = Vehicle()
    operations = DroneOperations(
        vehicle,
        course=course(),
        scenario=scenario(),
        vision=vision,
        rangefinder=Rangefinder(),
    )
    observe_flight(operations, 0, relative_altitude_m=4.572)
    operation_id = operations.start("find_marker", {"aruco_id": 2})

    for stamp in range(50_000_000, 350_000_000, 50_000_000):
        observe_flight(operations, stamp, relative_altitude_m=4.572)

    assert operations.operation_status(operation_id).state == "succeeded"
    assert vision.requested == [(2, 0)]
    assert vehicle.commands == []


def test_find_marker_abort_cancels_vision_without_output() -> None:
    vision = Vision()
    vehicle = Vehicle()
    operations = DroneOperations(
        vehicle,
        course=course(),
        scenario=scenario(),
        vision=vision,
        rangefinder=Rangefinder(),
    )
    observe_flight(operations, 0, relative_altitude_m=4.572)
    operation_id = operations.start("find_marker", {"aruco_id": 2})

    operations.abort("operator abort")

    assert operations.operation_status(operation_id).state == "cancelled"
    assert vision.cancelled
    assert vehicle.commands == []


def acquire_marker(operations: DroneOperations, vision: Vision) -> None:
    operation_id = operations.start("find_marker", {"aruco_id": 2})
    for sequence in range(1, 6):
        vision.add(MarkerObservation(
            2, sequence, sequence * 50_000_000, sequence * 50_000_000,
            0.0, 0.0, 4.572,
        ))
        observe_flight(operations, sequence * 50_000_000, relative_altitude_m=4.572)
    assert operations.operation_status(operation_id).state == "succeeded"


def test_precision_land_rejects_live_profile_mismatch_before_output() -> None:
    vision = Vision()
    vehicle = Vehicle()
    operations = DroneOperations(
        vehicle, course=course(), scenario=scenario(), vision=vision,
        rangefinder=Rangefinder(),
    )
    observe_flight(operations, 0, relative_altitude_m=4.572)
    acquire_marker(operations, vision)
    vehicle.commands.clear()

    operation_id = operations.start("precision_land", {"aruco_id": 2})

    assert operations.operation_status(operation_id).state == "failed"
    assert "profile" in operations.operation_status(operation_id).error
    assert vehicle.commands == []


def test_precision_land_uses_only_post_start_frame_and_holds_on_health_loss() -> None:
    vision = Vision()
    vehicle = Vehicle()
    operations = DroneOperations(
        vehicle, course=course(), scenario=scenario(), vision=vision,
        rangefinder=Rangefinder(),
    )
    observe_flight(operations, 0, relative_altitude_m=4.572,
                   precision_parameters=dict(PRECISION_PARAMETERS))
    acquire_marker(operations, vision)
    vehicle.commands.clear()
    operation_id = operations.start("precision_land", {"aruco_id": 2})
    assert vehicle.commands == []
    vision.add(MarkerObservation(2, 5, 300_000_000, 300_000_000,
                                 0.0, 0.0, 4.572))
    observe_flight(operations, 300_000_000, relative_altitude_m=4.572)
    assert vehicle.commands == []  # repeated acquisition frame cannot start LAND
    vision.add(MarkerObservation(2, 6, 350_000_000, 350_000_000,
                                 0.0, 0.0, 4.572))
    observe_flight(operations, 350_000_000, relative_altitude_m=4.572)
    assert vehicle.commands == [("mode", "LAND")]
    vision.add(MarkerObservation(2, 7, 400_000_000, 400_000_000,
                                 0.0, 0.0, 4.572))
    observe_flight(operations, 400_000_000, mode="LAND", relative_altitude_m=4.572)
    assert vehicle.commands[-1][0] == "landing_target"

    operations.tick(900_000_000)
    assert vehicle.commands[-1] == ("mode", "GUIDED")
    assert operations.operation_status(operation_id).state == "running"


def test_precision_land_near_ground_marker_loss_waits_for_touchdown_and_disarm() -> None:
    vision = Vision()
    vehicle = Vehicle()
    rangefinder = Rangefinder(0.70)
    operations = DroneOperations(
        vehicle, course=course(), scenario=scenario(), vision=vision,
        rangefinder=rangefinder,
    )
    rangefinder.distance_m = 4.572
    observe_flight(operations, 0, relative_altitude_m=4.572,
                   precision_parameters=dict(PRECISION_PARAMETERS))
    acquire_marker(operations, vision)
    operation_id = operations.start("precision_land", {"aruco_id": 2})
    vision.add(MarkerObservation(2, 6, 300_000_000, 300_000_000,
                                 0.0, 0.0, 4.572))
    observe_flight(operations, 300_000_000, relative_altitude_m=0.7)
    rangefinder.distance_m = 0.70
    operations.tick(900_000_000)
    assert ("mode", "GUIDED") not in vehicle.commands

    observe_flight(operations, 950_000_000, mode="LAND", armed=False, landed=True,
                   relative_altitude_m=0.0)
    assert operations.operation_status(operation_id).state == "succeeded"


def test_precision_land_reacquires_five_frames_before_resuming_land() -> None:
    vision = Vision()
    vehicle = Vehicle()
    operations = DroneOperations(
        vehicle, course=course(), scenario=scenario(), vision=vision,
        rangefinder=Rangefinder(),
    )
    observe_flight(operations, 0, relative_altitude_m=4.572,
                   precision_parameters=dict(PRECISION_PARAMETERS))
    acquire_marker(operations, vision)
    operation_id = operations.start("precision_land", {"aruco_id": 2})
    vision.add(MarkerObservation(2, 6, 300_000_000, 300_000_000,
                                 0.0, 0.0, 4.572))
    observe_flight(operations, 300_000_000, relative_altitude_m=4.572)
    operations.tick(800_000_000)
    assert vehicle.commands[-1] == ("mode", "GUIDED")

    for sequence in range(7, 10):
        stamp = 800_000_000 + (sequence - 6) * 100_000_000
        vision.add(MarkerObservation(2, sequence, stamp, stamp,
                                     0.0, 0.0, 4.572))
        observe_flight(operations, stamp, relative_altitude_m=4.572)
        operations.tick(stamp + 50_000_000)

    invalid_stamp = 1_150_000_000
    vision.add(MarkerObservation(2, 10, invalid_stamp, invalid_stamp,
                                 1.0, 0.0, 4.572))
    observe_flight(operations, invalid_stamp, relative_altitude_m=4.572)

    for sequence in range(11, 16):
        stamp = 1_150_000_000 + (sequence - 10) * 100_000_000
        vision.add(MarkerObservation(2, sequence, stamp, stamp,
                                     0.0, 0.0, 4.572))
        observe_flight(operations, stamp, relative_altitude_m=4.572)
        if sequence < 15:
            operations.tick(stamp + 50_000_000)
            assert vehicle.commands[-1] != ("mode", "LAND")
    assert vehicle.commands[-1] == ("mode", "LAND")
    assert operations.operation_status(operation_id).state == "running"


def test_precision_land_is_bounded_to_sixty_simulated_seconds_and_abort_cancels() -> None:
    vision = Vision()
    operations = DroneOperations(
        Vehicle(), course=course(), scenario=scenario(), vision=vision,
        rangefinder=Rangefinder(),
    )
    observe_flight(operations, 0, relative_altitude_m=4.572,
                   precision_parameters=dict(PRECISION_PARAMETERS))
    acquire_marker(operations, vision)
    operation_id = operations.start(
        "precision_land", {"aruco_id": 2}, timeout_sim_s=120
    )
    operations.tick(60_250_000_000)
    assert operations.operation_status(operation_id).state == "failed"
    assert "timeout" in operations.operation_status(operation_id).error
    assert vision.cancelled


def test_payload_completion_requires_correlated_physical_confirmation() -> None:
    payloads = Payloads()
    operations = DroneOperations(Vehicle(), course=course(), payloads=payloads)
    observe_flight(operations, 0, armed=False, landed=True)
    operations._last_precision_touchdown_id = 2
    operation_id = operations.start("attach_payload", {"aruco_id": 2})
    request_id = payloads.requests[0][2]

    payloads.confirmations[request_id] = SimpleNamespace(
        request_id="foreign", confirmed=True, reason="")
    operations.tick(1)
    assert operations.operation_status(operation_id).state == "running"
    payloads.confirmations[request_id] = SimpleNamespace(
        request_id=request_id, confirmed=True, reason="")
    operations.tick(2)
    assert operations.operation_status(operation_id).state == "succeeded"


def test_airborne_release_refuses_below_minimum_agl_without_payload_request() -> None:
    payloads = Payloads()
    operations = DroneOperations(Vehicle(), course=course(), payloads=payloads)
    observe_flight(operations, 0, relative_altitude_m=9.143)

    operation_id = operations.start(
        "release_payload", {"aruco_id": 2, "delivery_mode": "airborne"}
    )

    assert operations.operation_status(operation_id).state == "failed"
    assert payloads.requests == []


def test_phase_check_ignores_foreign_decision_then_accepts_exact_correlation() -> None:
    phases = Phases()
    operations = DroneOperations(Vehicle(), phases=phases)
    operation_id = operations.start("check_phase", {"phase": "FM1", "checkpoint": "D"})
    request_id = phases.events[0][3]
    phases.decisions[request_id] = SimpleNamespace(
        request_id="foreign", phase="FM1", checkpoint="D", approved=True, reason="")
    operations.tick(1)
    assert operations.operation_status(operation_id).state == "running"
    phases.decisions[request_id] = SimpleNamespace(
        request_id=request_id, phase="FM1", checkpoint="D", approved=True, reason="")
    operations.tick(2)
    assert operations.operation_status(operation_id).state == "succeeded"


def test_attempt_expiry_cancels_payload_and_transfers_owner_to_home_recovery() -> None:
    payloads = Payloads()
    attempt = Attempt()
    vehicle = Vehicle()
    operations = DroneOperations(
        vehicle, course=course(), payloads=payloads, attempt=attempt
    )
    observe_flight(operations, 0, relative_altitude_m=10.0)
    operation_id = operations.start(
        "release_payload", {"aruco_id": 2, "delivery_mode": "airborne"}
    )
    attempt.active_elapsed_ns = 600_000_000_000

    operations.tick(1)

    assert operations.operation_status(operation_id).state == "failed"
    assert payloads.cancelled
    assert vehicle.commands[-1][0] == "waypoint"
    assert operations._recovery_id is not None


def test_attempt_expiry_with_stale_telemetry_sends_no_recovery_command() -> None:
    payloads = Payloads()
    attempt = Attempt()
    vehicle = Vehicle()
    operations = DroneOperations(
        vehicle, course=course(), payloads=payloads, attempt=attempt
    )
    observe_flight(operations, 0, relative_altitude_m=10.0)
    operation_id = operations.start(
        "release_payload", {"aruco_id": 2, "delivery_mode": "airborne"}
    )
    vehicle.commands.clear()
    attempt.active_elapsed_ns = 600_000_000_000

    operations.tick(3_000_000_001)

    assert operations.operation_status(operation_id).state == "failed"
    assert vehicle.commands == []


def test_approval_to_takeoff_deadline_fails_waiting_payload_work() -> None:
    payloads = Payloads()
    attempt = Attempt()
    attempt.state = 3
    attempt.approval_timestamp_ns = 1
    operations = DroneOperations(
        Vehicle(), course=course(), payloads=payloads, attempt=attempt
    )
    observe_flight(operations, 0, relative_altitude_m=10.0)
    operation_id = operations.start(
        "release_payload", {"aruco_id": 2, "delivery_mode": "airborne"},
        timeout_sim_s=120,
    )

    operations.tick(60_000_000_001)

    status = operations.operation_status(operation_id)
    assert status.state == "failed"
    assert status.error == "approval-to-takeoff deadline expired"


@pytest.mark.parametrize(
    ("tool", "args", "forbidden"),
    [
        ("set_mode", {"mode": "GUIDED"}, CommandKind.SET_GUIDED),
        ("arm", {}, CommandKind.ARM),
        ("takeoff", {"altitude_m": 10.0}, CommandKind.TAKEOFF),
    ],
)
def test_finished_pending_approval_expiry_blocks_flight_command_before_dispatch(
    tool, args, forbidden
) -> None:
    attempt = Attempt()
    attempt.state = 4  # FINISHED after the pending approval expired.
    attempt.approval_timestamp_ns = 1
    vehicle = Vehicle()
    operations = DroneOperations(vehicle, course=course(), attempt=attempt)
    observe_flight(
        operations,
        60_000_000_001,
        prearm_checks_healthy=True,
    )

    operation_id = operations.start(tool, args)

    status = operations.operation_status(operation_id)
    assert status.state == "failed"
    assert status.error == "approval-to-takeoff deadline expired"
    assert all(command[0] is not forbidden for command in vehicle.commands)


def test_finished_attempt_without_pending_approval_does_not_invent_expiry() -> None:
    attempt = Attempt()
    attempt.state = 4
    attempt.approval_timestamp_ns = 0
    operations = DroneOperations(Vehicle(), course=course(), attempt=attempt)
    observe_flight(operations, 60_000_000_001)

    operation_id = operations.start("hold", {"duration_sim_s": 1.0})

    assert operations.operation_status(operation_id).state == "running"
