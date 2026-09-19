from io import StringIO

from artifacts.runtime_status import RuntimeStatus, status_document, status_name
from drone_sim_companion.configured_runtime import ConfiguredHost
from drone_sim_companion.lifecycle import CompanionLifecycle
from drone_sim_companion.mission import Ack, CommandKind, Telemetry
from drone_sim_companion.mission_plan import parse_mission_plan


RUN_ID = "00000000-0000-4000-8000-000000000001"


class Protocol:
    def __init__(self):
        self.statuses = {}
        self.quiescent = False

    def write_status(self, status: RuntimeStatus):
        self.statuses[status_name(type(status))] = status_document(status)

    def write_quiescence(self, module):
        self.quiescent = True


class Vehicle:
    def __init__(self):
        self.commands = []

    def send(self, command, altitude_m):
        self.commands.append((command, altitude_m))


def host_for(steps):
    protocol = Protocol()
    vehicle = Vehicle()
    lifecycle = CompanionLifecycle(run_id=RUN_ID, protocol=protocol, stream=StringIO())
    host = ConfiguredHost(parse_mission_plan({"schema_version": 1, "steps": steps}),
                          vehicle, lifecycle, protocol, RUN_ID)
    return host, vehicle, protocol


def test_operator_plan_releases_clock_without_commanding_mode_or_arm():
    host, vehicle, protocol = host_for([
        {"tool": "wait_for_state", "args": {"armed": True, "mode": "GUIDED"}},
        {"tool": "takeoff", "args": {"altitude_m": 2}},
    ])
    host.observe(Telemetry(0, heartbeat=True, mode="STABILIZE", armed=False,
                           landed=True, prearm_checks_healthy=True))
    host.tick(None, mission_running=True)
    host.tick(0, mission_running=False)
    assert "mission-execution-ready" not in protocol.statuses
    host.tick(0, mission_running=True)
    assert protocol.statuses["mission-execution-ready"] == {
        "run_id": RUN_ID, "ready": True, "sim_timestamp_ns": 0,
    }
    assert vehicle.commands == []
    host.observe(Telemetry(100_000_000, heartbeat=True, mode="GUIDED", armed=True))
    host.tick(100_000_000, mission_running=True)
    assert vehicle.commands == [(CommandKind.TAKEOFF, 2)]


def test_completed_steps_without_landing_do_not_publish_mission_success():
    host, vehicle, protocol = host_for([
        {"tool": "wait_for_state", "args": {"armed": True, "mode": "GUIDED"}},
    ])
    host.observe(Telemetry(0, heartbeat=True, mode="GUIDED", armed=True,
                           landed=False, prearm_checks_healthy=True))
    host.tick(0, mission_running=True)
    host.tick(0, mission_running=True)
    assert host.error
    assert "mission-finished" not in protocol.statuses
    assert vehicle.commands == [(CommandKind.LAND, None)]


def test_failed_takeoff_attempts_one_land_and_keeps_failure_after_recovery():
    host, vehicle, protocol = host_for([
        {"tool": "takeoff", "args": {"altitude_m": 2}},
        {"tool": "hold", "args": {"duration_sim_s": 2}},
    ])
    host.observe(Telemetry(0, heartbeat=True, mode="GUIDED", armed=True,
                           landed=False, prearm_checks_healthy=True))
    host.tick(0, mission_running=True)
    host.observe(Telemetry(100_000_000, ack=Ack(CommandKind.TAKEOFF, False, 4)))
    host.tick(100_000_000, mission_running=True)
    assert vehicle.commands == [(CommandKind.TAKEOFF, 2), (CommandKind.LAND, None)]
    host.fail("second failure")
    host.observe(Telemetry(200_000_000, ack=Ack(CommandKind.LAND, True, 0)))
    host.observe(Telemetry(300_000_000, heartbeat=True, mode="LAND", armed=False, landed=True))
    host.tick(300_000_000, mission_running=True)
    host.finalize()
    assert host.error and "rejected" in host.error
    assert host.recovery_pending is False
    assert "mission-finished" not in protocol.statuses
    assert protocol.statuses["runtime-failure"]["reason"] == host.error
    assert protocol.quiescent
    assert len(vehicle.commands) == 2


def test_operator_mode_change_never_forces_land_over_operator_control():
    host, vehicle, protocol = host_for([
        {"tool": "takeoff", "args": {"altitude_m": 2}},
    ])
    host.observe(Telemetry(0, heartbeat=True, mode="GUIDED", armed=True,
                           landed=False, prearm_checks_healthy=True))
    host.tick(0, mission_running=True)
    host.observe(Telemetry(100_000_000, heartbeat=True, mode="LOITER", armed=True))
    host.tick(100_000_000, mission_running=True)
    assert host.error
    assert vehicle.commands == [(CommandKind.TAKEOFF, 2)]


def test_low_takeoff_target_cannot_succeed_at_ground_altitude():
    host, _, _ = host_for([
        {"tool": "takeoff", "args": {"altitude_m": 0.1}},
        {"tool": "land", "args": {}},
    ])
    host.observe(Telemetry(0, heartbeat=True, mode="GUIDED", armed=True,
                           landed=True, prearm_checks_healthy=True))
    host.tick(0, mission_running=True)
    operation_id = host.mission.operation_id
    host.observe(Telemetry(100_000_000, ack=Ack(CommandKind.TAKEOFF, True, 0)))
    host.observe(Telemetry(200_000_000, relative_altitude_m=0.0))
    assert host.operations.operation_status(operation_id).state == "running"
