from __future__ import annotations

import pytest

from drone_sim_companion.mission import Ack, CommandKind, Telemetry
from drone_sim_companion.mission_plan import parse_mission_plan
from drone_sim_companion.operations import DroneOperations
from drone_sim_companion.configured_mission import ConfiguredMission


class Vehicle:
    def __init__(self):
        self.commands = []

    def send(self, command, altitude_m):
        self.commands.append((command, altitude_m))

    def send_waypoint(self, latitude_deg, longitude_deg, altitude_m):
        self.commands.append(("waypoint", latitude_deg, longitude_deg, altitude_m))


def ready(operations, stamp=0, *, armed=False, mode="GUIDED", landed=True):
    operations.observe(Telemetry(stamp, heartbeat=True, mode=mode, armed=armed,
                                 landed=landed, prearm_checks_healthy=True))


def ack(operations, command, stamp=0, *, accepted=True):
    operations.observe(Telemetry(stamp, ack=Ack(command, accepted, 0 if accepted else 4)))


def plan(*steps):
    return parse_mission_plan({"schema_version": 1, "steps": list(steps)})


@pytest.mark.parametrize("bad", [
    {"tool": "read_camera", "args": {}},
    {"tool": "arm", "args": {"force": True}},
    {"tool": "takeoff", "args": {"altitude_m": True}},
    {"tool": "takeoff", "args": {"altitude_m": float("nan")}},
    {"tool": "takeoff", "args": {"altitude_m": 0.1, "tolerance_m": 0.2}},
    {"tool": "goto_waypoint", "args": {"latitude_deg": 91, "longitude_deg": 0, "altitude_m": 2}},
    {"tool": "wait_for_state", "args": {}},
    {"tool": "land", "args": {}, "timeout_sim_s": 0},
])
def test_whole_plan_is_validated_before_any_operation(bad):
    with pytest.raises(ValueError):
        plan({"tool": "arm", "args": {}}, bad)


def test_operator_wait_is_passive_and_takeoff_never_arms_or_changes_mode():
    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    mission = ConfiguredMission(plan(
        {"tool": "wait_for_state", "args": {"armed": True, "mode": "GUIDED"}},
        {"tool": "takeoff", "args": {"altitude_m": 2}},
    ), operations)
    ready(operations, mode="STABILIZE")
    mission.tick(0)
    ready(operations, 1_000_000_000, armed=True, mode="STABILIZE")
    mission.tick(1_000_000_000)
    assert vehicle.commands == []
    ready(operations, 2_000_000_000, armed=True)
    mission.tick(2_000_000_000)
    assert vehicle.commands == [(CommandKind.TAKEOFF, 2)]
    assert operations.armed_at_ns == 1_000_000_000


def test_takeoff_rejects_missing_arm_without_sending_commands():
    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    ready(operations)
    operation_id = operations.start("takeoff", {"altitude_m": 2})
    assert operations.operation_status(operation_id).state == "failed"
    assert vehicle.commands == []


def test_ack_alone_does_not_finish_action_and_state_reads_remain_available():
    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    ready(operations, armed=True)
    operation_id = operations.start("takeoff", {"altitude_m": 2})
    ack(operations, CommandKind.TAKEOFF)
    assert operations.operation_status(operation_id).state == "running"
    assert operations.read_vehicle_state()["armed"] is True
    with pytest.raises(RuntimeError, match="active"):
        operations.start("land", {})
    operations.observe(Telemetry(100_000_000, relative_altitude_m=1))
    assert operations.operation_status(operation_id).state == "running"
    operations.observe(Telemetry(200_000_000, relative_altitude_m=2))
    assert operations.operation_status(operation_id).state == "succeeded"


def test_failure_stops_sequence_and_cannot_be_repaired_by_late_success():
    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    mission = ConfiguredMission(plan(
        {"tool": "arm", "args": {}},
        {"tool": "takeoff", "args": {"altitude_m": 2}},
    ), operations)
    ready(operations)
    mission.tick(0)
    ack(operations, CommandKind.ARM, accepted=False)
    mission.tick(0)
    assert mission.state == "failed"
    ready(operations, 100_000_000, armed=True)
    ack(operations, CommandKind.ARM, 100_000_000)
    mission.tick(100_000_000)
    assert mission.state == "failed"
    assert vehicle.commands == [(CommandKind.ARM, None)]


def test_simulation_deadline_fails_even_without_new_telemetry():
    operations = DroneOperations(Vehicle())
    operation_id = operations.start("wait_for_state", {"armed": True}, timeout_sim_s=1)
    operations.tick(999_999_999)
    assert operations.operation_status(operation_id).state == "running"
    operations.tick(1_000_000_000)
    assert operations.operation_status(operation_id).state == "failed"
    assert "timeout" in operations.operation_status(operation_id).error


def test_waypoint_requires_observed_position_and_altitude():
    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    ready(operations, armed=True, landed=False)
    operation_id = operations.start("goto_waypoint", {
        "latitude_deg": 37.4, "longitude_deg": -122.08, "altitude_m": 3,
    })
    assert vehicle.commands == [("waypoint", 37.4, -122.08, 3)]
    operations.observe(Telemetry(100_000_000, latitude_deg=37.4,
                                 longitude_deg=-122.08, relative_altitude_m=1))
    assert operations.operation_status(operation_id).state == "running"
    operations.observe(Telemetry(200_000_000, latitude_deg=37.4,
                                 longitude_deg=-122.08, relative_altitude_m=3))
    assert operations.operation_status(operation_id).state == "succeeded"


def test_land_waits_for_touchdown_and_disarm_then_finishes_sequence():
    operations = DroneOperations(Vehicle())
    ready(operations, armed=True, landed=False)
    mission = ConfiguredMission(plan({"tool": "land", "args": {}}), operations)
    mission.tick(0)
    ack(operations, CommandKind.LAND)
    ready(operations, 100_000_000, armed=True, mode="LAND", landed=True)
    mission.tick(100_000_000)
    assert mission.state == "running"
    ready(operations, 200_000_000, armed=False, mode="LAND", landed=True)
    mission.tick(200_000_000)
    assert mission.state == "succeeded"


def test_abort_leaves_no_later_step_commands():
    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    ready(operations, armed=True)
    mission = ConfiguredMission(plan(
        {"tool": "takeoff", "args": {"altitude_m": 2}},
        {"tool": "land", "args": {}},
    ), operations)
    mission.tick(0)
    mission.abort("operator abort")
    ack(operations, CommandKind.TAKEOFF)
    operations.observe(Telemetry(100_000_000, relative_altitude_m=2))
    mission.tick(100_000_000)
    assert mission.state == "cancelled"
    assert vehicle.commands == [(CommandKind.TAKEOFF, 2)]


def test_full_sequence_waits_for_each_ack_and_observed_completion():
    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    mission = ConfiguredMission(plan(
        {"tool": "set_mode", "args": {"mode": "GUIDED"}},
        {"tool": "arm", "args": {}},
        {"tool": "takeoff", "args": {"altitude_m": 2}},
        {"tool": "hold", "args": {"duration_sim_s": 1}},
        {"tool": "land", "args": {}},
    ), operations)

    ready(operations, mode="STABILIZE")
    mission.tick(0)
    assert vehicle.commands == [(CommandKind.SET_GUIDED, None)]

    ack(operations, CommandKind.SET_GUIDED, 50_000_000)
    mission.tick(50_000_000)
    assert vehicle.commands == [(CommandKind.SET_GUIDED, None)]
    ready(operations, 100_000_000)
    mission.tick(100_000_000)
    assert vehicle.commands[-1] == (CommandKind.ARM, None)

    ack(operations, CommandKind.ARM, 150_000_000)
    ready(operations, 200_000_000, armed=True, landed=False)
    mission.tick(200_000_000)
    assert vehicle.commands[-1] == (CommandKind.TAKEOFF, 2)

    ack(operations, CommandKind.TAKEOFF, 250_000_000)
    operations.observe(Telemetry(300_000_000, relative_altitude_m=2))
    mission.tick(300_000_000)
    assert operations.armed_at_ns == 200_000_000
    assert vehicle.commands == [
        (CommandKind.SET_GUIDED, None),
        (CommandKind.ARM, None),
        (CommandKind.TAKEOFF, 2),
    ]

    mission.tick(1_299_999_999)
    assert mission.state == "running"
    mission.tick(1_300_000_000)
    assert vehicle.commands[-1] == (CommandKind.LAND, None)

    ack(operations, CommandKind.LAND, 1_350_000_000)
    ready(operations, 1_400_000_000, armed=True, mode="LAND", landed=True)
    mission.tick(1_400_000_000)
    assert mission.state == "running"
    ready(operations, 1_500_000_000, armed=False, mode="LAND", landed=True)
    mission.tick(1_500_000_000)

    assert mission.state == "succeeded"


def test_mode_loss_during_hold_fails_and_never_starts_land():
    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    mission = ConfiguredMission(plan(
        {"tool": "hold", "args": {"duration_sim_s": 2}},
        {"tool": "land", "args": {}},
    ), operations)
    ready(operations, armed=True, landed=False)
    mission.tick(0)

    ready(operations, 100_000_000, armed=True, mode="STABILIZE", landed=False)
    mission.tick(100_000_000)

    assert mission.state == "failed"
    assert "GUIDED" in mission.error
    assert vehicle.commands == []


def test_hold_fails_when_heartbeat_age_exceeds_three_simulated_seconds():
    operations = DroneOperations(Vehicle())
    ready(operations, armed=True, landed=False)
    operation_id = operations.start(
        "hold", {"duration_sim_s": 5}, timeout_sim_s=10
    )

    operations.tick(3_000_000_000)
    assert operations.operation_status(operation_id).state == "running"
    operations.tick(3_000_000_001)

    status = operations.operation_status(operation_id)
    assert status.state == "failed"
    assert status.error == "vehicle heartbeat became stale"


def test_parsed_plan_copies_and_freezes_operation_arguments():
    document = {
        "schema_version": 1,
        "steps": [{"tool": "takeoff", "args": {"altitude_m": 2}}],
    }

    parsed = parse_mission_plan(document)
    document["steps"][0]["args"]["altitude_m"] = 8

    assert parsed.steps[0].args["altitude_m"] == 2
    with pytest.raises(TypeError):
        parsed.steps[0].args["altitude_m"] = 3
