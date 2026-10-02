from io import StringIO
import json
from pathlib import Path

import pytest

from artifacts.runtime_status import RuntimeStatus, status_document, status_name
from drone_sim_companion.configured_runtime import ConfiguredHost
from drone_sim_companion.configured_runtime import MOVING_PRECISION_PARAMETERS
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

    def send_waypoint(self, latitude, longitude, altitude, *, yaw_rad=None):
        self.commands.append(("waypoint", latitude, longitude, altitude, yaw_rad))


def host_for(steps, *, calibration_parameters=None):
    protocol = Protocol()
    vehicle = Vehicle()
    lifecycle = CompanionLifecycle(run_id=RUN_ID, protocol=protocol, stream=StringIO())
    host = ConfiguredHost(parse_mission_plan({"schema_version": 1, "steps": steps}),
                          vehicle, lifecycle, protocol, RUN_ID,
                          calibration_parameters=calibration_parameters)
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


@pytest.mark.parametrize(
    ("parameter_name", "actual", "expected"),
    [
        ("PLND_OPTIONS", 4.0, 5.0),
        ("AHRS_EKF_TYPE", 10.0, 3.0),
        ("PLND_EST_TYPE", 0.0, 1.0),
        ("PSC_NE_POS_P", 4.0, 1.0),
    ],
)
def test_moving_profile_mismatch_emits_zero_flight_commands(parameter_name, actual, expected):
    host, vehicle, protocol = host_for([
        {"tool": "set_mode", "args": {"mode": "GUIDED"}},
        {"tool": "precision_land", "args": {
            "marker_id": 7, "settle_by_sim_s": 45, "acquire_by_sim_s": 60,
        }, "timeout_sim_s": 45},
    ])
    host.observe(Telemetry(
        0, heartbeat=True, mode="STABILIZE", armed=False, landed=True,
        prearm_checks_healthy=True,
    ))
    host.observe(Telemetry(
        0, parameter_name=parameter_name, parameter_value=actual,
    ))
    host.tick(0, mission_running=True)

    assert host.error == f"effective precision parameter {parameter_name} is {actual}, expected {expected}"
    assert vehicle.commands == []
    assert "mission-execution-ready" not in protocol.statuses


def test_ordinary_configured_plan_ignores_moving_profile_parameter_values():
    host, vehicle, _protocol = host_for([
        {"tool": "wait_for_state", "args": {"armed": True, "mode": "GUIDED"}},
    ])

    host.observe(Telemetry(
        0, parameter_name="PLND_EST_TYPE", parameter_value=1.0,
    ))

    assert host.error is None
    assert vehicle.commands == []


def test_calibration_readback_blocks_first_flight_command_until_all_values_match():
    expected = {"ATC_RAT_RLL_P": 0.041, "ATC_RAT_YAW_FLTE": 2.0}
    host, vehicle, protocol = host_for(
        [{"tool": "set_mode", "args": {"mode": "GUIDED"}}],
        calibration_parameters=expected,
    )
    host.observe(Telemetry(0, heartbeat=True, mode="STABILIZE", armed=False,
                           landed=True, prearm_checks_healthy=True))
    host.observe(Telemetry(0, parameter_name="ATC_RAT_RLL_P", parameter_value=0.041))
    host.tick(0, mission_running=True)
    assert vehicle.commands == []
    assert "mission-execution-ready" not in protocol.statuses

    host.observe(Telemetry(1, parameter_name="ATC_RAT_YAW_FLTE", parameter_value=2.0))
    host.tick(1, mission_running=True)
    assert vehicle.commands == [(CommandKind.SET_GUIDED, None)]


def test_calibration_readback_mismatch_emits_zero_flight_commands():
    host, vehicle, protocol = host_for(
        [{"tool": "arm", "args": {}}],
        calibration_parameters={"ATC_RAT_RLL_P": 0.041},
    )
    host.observe(Telemetry(0, heartbeat=True, mode="GUIDED", armed=False,
                           landed=True, prearm_checks_healthy=True))
    host.observe(Telemetry(0, parameter_name="ATC_RAT_RLL_P", parameter_value=0.05))
    host.tick(0, mission_running=True)

    assert host.error == "effective required parameter ATC_RAT_RLL_P is 0.05, expected 0.041"
    assert vehicle.commands == []
    assert "mission-execution-ready" not in protocol.statuses


def test_calibration_readback_accepts_float32_rounding_and_emits_verified_event_once():
    expected = {"ATC_ACC_R_MAX": 123456.789}
    host, vehicle, protocol = host_for(
        [{"tool": "set_mode", "args": {"mode": "GUIDED"}}],
        calibration_parameters=expected,
    )
    host.observe(Telemetry(0, heartbeat=True, mode="STABILIZE", armed=False,
                           landed=True, prearm_checks_healthy=True))
    host.observe(Telemetry(
        0, parameter_name="ATC_ACC_R_MAX", parameter_value=123456.79,
    ))
    host.tick(0, mission_running=True)
    host.tick(1, mission_running=True)

    events = [json.loads(line) for line in host.lifecycle._stream.getvalue().splitlines()]
    verified = [event for event in events
                if event["event"] == "calibration_parameters_verified"]
    assert len(verified) == 1
    assert verified[0]["fields"] == {
        "stage": "pre_arm", "parameters": {"ATC_ACC_R_MAX": 123456.79},
    }
    assert vehicle.commands == [(CommandKind.SET_GUIDED, None)]
    assert protocol.statuses["mission-execution-ready"]["ready"] is True


def test_moving_profile_gate_needs_no_image_before_first_flight_command():
    host, vehicle, protocol = host_for([
        {"tool": "set_mode", "args": {"mode": "GUIDED"}},
        {"tool": "precision_land", "args": {
            "marker_id": 7, "settle_by_sim_s": 45, "acquire_by_sim_s": 60,
        }, "timeout_sim_s": 45},
    ])
    host.observe(Telemetry(
        0, heartbeat=True, mode="STABILIZE", armed=False, landed=True,
        prearm_checks_healthy=True,
    ))
    for name, value in MOVING_PRECISION_PARAMETERS.items():
        host.observe(Telemetry(0, parameter_name=name, parameter_value=value))

    host.tick(0, mission_running=True)

    assert protocol.statuses["mission-execution-ready"]["ready"] is True
    assert vehicle.commands == [(CommandKind.SET_GUIDED, None)]


def test_companion_expected_profile_matches_effective_base_and_moving_overlay():
    root = Path(__file__).parents[2]
    effective = {}
    for relative in ("ardupilot_sitl/params/descent.parm", "ardupilot_sitl/params/moving-pad.parm"):
        for line in (root / relative).read_text(encoding="utf-8").splitlines():
            if line and not line.startswith("#"):
                name, value = line.split()
                effective[name] = float(value)

    assert {name: effective[name] for name in MOVING_PRECISION_PARAMETERS} == {
        "AHRS_EKF_TYPE": 3.0,
        "LAND_SPD_MS": 0.50,
        "PLND_ENABLED": 1.0,
        "PLND_TYPE": 1.0,
        "PLND_LAG": 0.04,
        "PLND_EST_TYPE": 1.0,
        "PLND_XY_DIST_MAX": 0.50,
        "PLND_STRICT": 2.0,
        "PLND_RET_MAX": 1.0,
        "PLND_TIMEOUT": 0.50,
        "PLND_ALT_MIN": 0.75,
        "PLND_ALT_MAX": 8.0,
        "PLND_OPTIONS": 5.0,
        "PSC_NE_POS_P": 1.0,
    }
    assert {name: effective[name] for name in MOVING_PRECISION_PARAMETERS} == MOVING_PRECISION_PARAMETERS


def test_moving_plan_publishes_observed_arm_and_disarm_events():
    published = []
    protocol = Protocol()
    vehicle = Vehicle()
    lifecycle = CompanionLifecycle(run_id=RUN_ID, protocol=protocol, stream=StringIO())
    host = ConfiguredHost(
        parse_mission_plan({"schema_version": 1, "steps": [{
            "tool": "precision_land",
            "args": {"marker_id": 7, "settle_by_sim_s": 45, "acquire_by_sim_s": 60},
        }]}),
        vehicle,
        lifecycle,
        protocol,
        RUN_ID,
        publish_mission_event=published.append,
    )

    host.observe(Telemetry(1, heartbeat=True, armed=True, mode="GUIDED"))
    host.observe(Telemetry(2, heartbeat=True, armed=False, mode="LAND", landed=True))

    assert [(event.phase, event.state, event.sim_timestamp_ns) for event in published] == [
        ("MOVING_PAD", "ARMED", 1),
        ("MOVING_PAD", "DISARMED", 2),
    ]


def test_moving_approach_fails_if_waypoint_is_still_active_after_absolute_settle_deadline():
    host, vehicle, _protocol = host_for([
        {"tool": "goto_waypoint", "args": {
            "latitude_deg": 37.4003371,
            "longitude_deg": -122.079639322083,
            "altitude_m": 5,
        }},
        {"tool": "precision_land", "args": {
            "marker_id": 7,
            "settle_by_sim_s": 45,
            "acquire_by_sim_s": 60,
        }, "timeout_sim_s": 45},
    ])
    host.observe(Telemetry(
        0, heartbeat=True, mode="GUIDED", armed=True, landed=False,
        prearm_checks_healthy=True,
    ))
    for name, value in MOVING_PRECISION_PARAMETERS.items():
        host.observe(Telemetry(0, parameter_name=name, parameter_value=value))
    host.tick(0, mission_running=True)
    assert vehicle.commands == [
        ("waypoint", 37.4003371, -122.079639322083, 5, 0.0),
    ]

    late = 45_000_000_001
    host.observe(Telemetry(
        late, heartbeat=True, mode="GUIDED", armed=True, landed=False,
        latitude_deg=37.4003371, longitude_deg=-122.0800351,
        relative_altitude_m=5.0,
    ))
    host.tick(late, mission_running=True)

    assert host.error == "moving-pad approach did not reach precision settlement by 45 simulated seconds"
