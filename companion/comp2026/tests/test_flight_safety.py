import math
import queue
import threading
import time as wall_time
from types import MappingProxyType, SimpleNamespace

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from drone.common_types import GPSCoord, MissionHome, RelPosComplete
from drone import timebase
from drone.control import drone_control
from dronekit.mavlink import MAVWriter
from drone.control.drone_control import DroneControl
from drone.control.listener import (
    TelemetryStartupEvidence,
    _TelemetryStartupVerification,
)
from drone.control.flight_state import FlightState, RCModeBand
from drone.control.mission_supervisor import AuthorityLost, FlightOperationError, MissionAbort
from drone.control.stability import ReleaseStabilityConfig
from drone.sensors.lidar.clearance import ClearanceCalibration
from drone.sensors.lidar.lidar import LidarSample


BOUNDS = {
    "heartbeat": 1.5,
    "mode": 1.5,
    "location": 0.5,
    "velocity": 0.5,
    "attitude": 0.5,
    "landed_state": 1.5,
    "armed": 1.5,
    "home": 1.5,
    "rc_input": 0.5,
    "range": 0.5,
    "failsafe": 1.5,
}


def wall_time_wait(predicate, timeout=0.2):
    deadline = wall_time.monotonic() + timeout
    while wall_time.monotonic() < deadline:
        if predicate():
            return True
        wall_time.sleep(0.001)
    return predicate()


class FakeClock:
    def __init__(self):
        self.now_value = 10.0
        self.events = []

    def now(self):
        return self.now_value

    def after(self, delay, callback):
        self.events.append((self.now_value + delay, callback))

    def sleep(self, seconds):
        self.now_value += seconds
        due = [event for event in self.events if event[0] <= self.now_value]
        self.events = [event for event in self.events if event[0] > self.now_value]
        for _, callback in due:
            callback()


class FakeMode:
    def __init__(self, name):
        self.name = name

    def __eq__(self, other):
        return self.name == getattr(other, "name", None)


class FakeMav:
    def __init__(self):
        self.waypoints = []
        self.mission_result = mavutil.mavlink.MAV_MISSION_ACCEPTED
        self.mission_callback = lambda _result: None

    def mission_item_int_send(self, *fields):
        self.waypoints.append(fields)
        if self.mission_result is not None:
            self.mission_callback(self.mission_result)


class FakeFactory:
    def __init__(self):
        self.relative = []
        self.landing_targets = []

    def set_position_target_local_ned_encode(self, *fields):
        self.relative.append(fields)
        return fields

    def landing_target_encode(self, *fields):
        self.landing_targets.append(fields)
        return fields

    def command_long_encode(self, *fields):
        return ("command-long", fields)


class FakeVehicle:
    def __init__(self):
        self._mode = FakeMode("GUIDED")
        self.mode_requests = []
        self.arm_requests = []
        self.takeoffs = []
        self.gotos = []
        self.sent = []
        self.flushes = 0
        self.groundspeed = None
        self.is_armable = True
        self.message_factory = FakeFactory()
        self._master = SimpleNamespace(mav=FakeMav())
        self.command_results = {}
        self.command_callback = lambda _command, _result: None
        self.before_arm_output = lambda _value: None
        self._mode_mapping = {"GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        self.mode_requests.append(value.name)
        self._mode = value
        self._emit_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE)

    @property
    def groundspeed(self):
        return self._groundspeed

    @groundspeed.setter
    def groundspeed(self, value):
        self._groundspeed = value
        if hasattr(self, "command_results"):
            self._emit_command(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED)

    @property
    def armed(self):
        return bool(self.arm_requests and self.arm_requests[-1])

    @armed.setter
    def armed(self, value):
        self.before_arm_output(value)
        self.arm_requests.append(value)
        self._emit_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

    def simple_takeoff(self, altitude):
        self.takeoffs.append(altitude)
        self._emit_command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    def simple_goto(self, location):
        self.gotos.append(location)

    def send_mavlink(self, message):
        if isinstance(message, tuple) and message[0] == "command-long":
            fields = message[1]
            command = fields[2]
            if command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                mode_id = int(fields[5])
                name = next(name for name, value in self._mode_mapping.items() if value == mode_id)
                self.mode_requests.append(name)
                self._mode = FakeMode(name)
            elif command == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
                self._groundspeed = fields[5]
            elif command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                armed = fields[4] == 1.0
                self.before_arm_output(armed)
                self.arm_requests.append(armed)
            elif command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                self.takeoffs.append(fields[10])
            else:
                self.sent.append(message)
            self._emit_command(command)
            return
        self.sent.append(message)

    def flush(self):
        self.flushes += 1

    def _emit_command(self, command):
        result = self.command_results.get(
            command, mavutil.mavlink.MAV_RESULT_ACCEPTED
        )
        if result is not None:
            self.command_callback(command, result)


class Harness:
    def __init__(self, monkeypatch, *, install_complete_telemetry=True):
        self.clock = FakeClock()
        monkeypatch.setattr(drone_control.time, "monotonic", self.clock.now)
        monkeypatch.setattr(drone_control.time, "sleep", self.clock.sleep)
        self.state = FlightState(
            source_system=1,
            source_component=1,
            freshness_bounds=BOUNDS,
            rc_channel=7,
            rc_mode_mapping=(
                RCModeBand("manual", 900, 1200, "STABILIZE"),
                RCModeBand("companion", 1400, 1600, "GUIDED"),
                RCModeBand("pilot", 1800, 2100, "LOITER"),
            ),
            clock=self.clock.now,
        )
        self.sequence = 0
        self.vehicle = FakeVehicle()
        self.guard_calls = 0
        self.controller = object.__new__(DroneControl)
        self.controller.vehicle = self.vehicle
        self.controller.source_identity = SimpleNamespace(system_id=1, component_id=191)
        self.controller.wire_protocol = "2.0"
        self.controller.flight_controller_target = self.state.source
        self.controller.flight_state = self.state
        self.controller.permission_guard = self.guard
        self.controller.install_output_transactions(
            dependency_transaction=lambda operation: operation(),
            supervisor_transaction=lambda operation: operation(),
            transport_transaction=lambda operation, enqueue_check: (
                enqueue_check(), operation()
            )[1],
        )
        if install_complete_telemetry:
            self.controller.install_startup_telemetry_verifier(None, verified=True)
        self.controller.mission_home_check = lambda home: None
        self.controller.fc_home_position_tolerance_m = 2.0
        self.controller.fc_home_altitude_tolerance_m = 20.0
        self.controller.clearance_calibration = ClearanceCalibration(
            beam_direction_body_frd=(0.0, 0.0, 1.0),
            measured_reference_offset_body_frd_m=(0.0, 0.0, 0.0),
            lidar_mounting_offset_already_applied=True,
            max_tilt_rad=0.3,
            max_age_seconds=0.5,
            max_skew_seconds=0.2,
            locally_horizontal_planar_surface=True,
        )
        self.controller.release_stability_config = ReleaseStabilityConfig(
            hold_seconds=0.1,
            timeout_seconds=0.5,
            max_horizontal_speed_m_s=0.3,
            max_vertical_speed_m_s=0.2,
            max_roll_rad=0.2,
            max_pitch_rad=0.2,
            horizontal_position_tolerance_m=0.5,
            vertical_position_tolerance_m=0.5,
            max_observation_skew_seconds=0.2,
            max_observation_gap_seconds=0.2,
            poll_interval_seconds=0.1,
            waypoint_reissue_interval_seconds=0.2,
        )
        self.controller._release_hold_confirmation = None
        self.controller._mission_home = None
        self.controller._last_arm_boundary_sequence = None
        self.controller.cruise_alt = 10.0
        self.controller.boot_time = self.clock.now()
        tracker_type = getattr(drone_control, "CommandAckTracker", None)
        if tracker_type is not None:
            self.controller._command_ack_tracker = tracker_type(
                wire_protocol="2.0",
                source_system=1,
                source_component=1,
                target_system=1,
                target_component=191,
            )
            self.vehicle.command_callback = self.observe_ack
        self.controller._mission_ack_tracker = drone_control.MissionAckTracker(
            source_system=1,
            source_component=1,
            target_system=1,
            target_component=191,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        self.controller._mission_ack_transaction_lock = threading.Lock()
        self.vehicle._master.mav.mission_callback = self.observe_mission_ack
        self.publish(
            heartbeat="alive",
            mode="GUIDED",
            landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
            armed=False,
            location=(410000000, -810000000, 250000, 0),
            failsafe=(False, "test clear"),
        )
        self.sequence += 1
        assert self.state.observe_rc_input(
            channel=7,
            pwm=1500,
            signal_healthy=True,
            received_at=self.clock.now(),
            sequence=self.sequence,
            source_system=1,
            source_component=1,
        )
        assert self.state.acquire_initial_companion_authority(now=self.clock.now())
        self.controller.set_mission_home(MissionHome(41.0, -81.0, 250.0))

    def guard(self):
        self.guard_calls += 1

    def publish(self, **values):
        self.sequence += 1
        assert self.state.update_many(
            values,
            received_at=self.clock.now(),
            sequence=self.sequence,
            source_system=1,
            source_component=1,
        )

    def observe_mode(self, mode):
        self.sequence += 1
        assert self.state.observe_mode(
            mode,
            received_at=self.clock.now(),
            sequence=self.sequence,
            source_system=1,
            source_component=1,
        )

    def observe_ack(self, command, result, *, system=1, component=1):
        self.controller._command_ack_tracker.observe(
            command=command,
            result=result,
            source_system=system,
            source_component=component,
            target_system=1,
            target_component=191,
        )

    def observe_mission_ack(
        self,
        result,
        *,
        system=1,
        component=1,
        target_system=1,
        target_component=191,
        mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
    ):
        message = SimpleNamespace(
            type=result,
            mission_type=mission_type,
            target_system=target_system,
            target_component=target_component,
            get_srcSystem=lambda: system,
            get_srcComponent=lambda: component,
        )
        self.controller._mission_ack_tracker.observe_message(message)

    def establish_post_arm_home(self, amsl_mm=250000):
        self.controller._last_arm_boundary_sequence = self.sequence
        self.publish(home=(410000000, -810000000, amsl_mm))


def test_missing_altitude_cannot_confirm_arrival(monkeypatch):
    ticks = iter([0.0, 0.1, 1.1])
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: next(ticks))
    monkeypatch.setattr(drone_control.time, "sleep", lambda _seconds: None)
    vehicle = SimpleNamespace(
        location=SimpleNamespace(
            global_relative_frame=SimpleNamespace(lat=41.0, lon=-81.0, alt=None)
        )
    )

    assert not drone_control.wait_pos(
        vehicle, 41.0, -81.0, alt_m=10.0, timeout=1.0
    )


def test_missing_guard_and_flight_state_fail_closed_before_output():
    controller = object.__new__(DroneControl)
    controller.vehicle = FakeVehicle()
    controller.flight_state = None
    controller.permission_guard = None

    with pytest.raises(AuthorityLost):
        controller.set_guided_mode(timeout=0.1)

    assert controller.vehicle.mode_requests == []


@pytest.mark.parametrize("guard_result", [False, True, object()])
def test_non_none_guard_result_fails_closed_before_output(monkeypatch, guard_result):
    harness = Harness(monkeypatch)
    harness.controller.permission_guard = lambda: guard_result

    with pytest.raises(AuthorityLost, match="success contract"):
        harness.controller.set_guided_mode(timeout=0.1)

    assert harness.vehicle.mode_requests == []


@pytest.mark.parametrize(
    "operation",
    [
        lambda controller: controller.goto_waypoint(GPSCoord(41.0, -81.0, 10.0)),
        lambda controller: controller.guide_move_relative_frame(
            RelPosComplete(1.0, 0.0, 0.0)
        ),
        lambda controller: controller.simple_land(),
        lambda controller: controller.takeoff(10.0),
        lambda controller: controller.arm(),
        lambda controller: controller.disarm(),
        lambda controller: controller.climb(12.0),
        lambda controller: controller.land_send_landing_target(
            RelPosComplete(0.0, 0.0, 1.0)
        ),
    ],
)
def test_every_flight_operation_fails_closed_without_state_and_guard(operation):
    controller = object.__new__(DroneControl)
    controller.vehicle = FakeVehicle()
    controller.flight_state = None
    controller.permission_guard = None
    controller.cruise_alt = 10.0

    with pytest.raises(AuthorityLost):
        operation(controller)

    assert controller.vehicle.mode_requests == []
    assert controller.vehicle.arm_requests == []
    assert controller.vehicle.takeoffs == []
    assert controller.vehicle.gotos == []
    assert controller.vehicle.sent == []
    assert controller.vehicle._master.mav.waypoints == []


def test_denied_guided_sends_once_then_raises(monkeypatch):
    harness = Harness(monkeypatch)

    with pytest.raises(FlightOperationError, match="GUIDED"):
        harness.controller.set_guided_mode(timeout=1.0)

    assert harness.vehicle.mode_requests == ["GUIDED"]
    assert harness.clock.now_value == pytest.approx(11.0)


def test_denied_mode_ack_fails_even_when_mode_state_confirms(monkeypatch):
    assert hasattr(drone_control, "CommandAckTracker")
    harness = Harness(monkeypatch)
    harness.vehicle.command_results[mavutil.mavlink.MAV_CMD_DO_SET_MODE] = (
        mavutil.mavlink.MAV_RESULT_DENIED
    )
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))

    with pytest.raises(FlightOperationError, match="acknowledgement"):
        harness.controller.set_guided_mode(timeout=0.5)


def test_stale_unrelated_and_wrong_source_acks_do_not_confirm_mode(monkeypatch):
    assert hasattr(drone_control, "CommandAckTracker")
    harness = Harness(monkeypatch)
    harness.vehicle.command_results[mavutil.mavlink.MAV_CMD_DO_SET_MODE] = None
    harness.observe_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, system=44)
    harness.observe_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0)
    harness.observe_ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0)
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))

    with pytest.raises(FlightOperationError, match="acknowledgement"):
        harness.controller.set_guided_mode(timeout=0.3)


def test_ack_arriving_during_permission_checks_is_stale_for_transaction(monkeypatch):
    harness = Harness(monkeypatch)
    harness.vehicle.command_results[mavutil.mavlink.MAV_CMD_DO_SET_MODE] = None
    calls = [0]

    def emit_ack_before_transmission():
        calls[0] += 1
        if calls[0] == 2:
            harness.observe_ack(
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
            )

    harness.controller.permission_guard = emit_ack_before_transmission
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))

    with pytest.raises(FlightOperationError, match="acknowledgement"):
        harness.controller.set_guided_mode(timeout=0.3)


def test_command_ack_packed_before_actual_writer_enqueue_is_stale(monkeypatch):
    harness = Harness(monkeypatch)
    harness.vehicle.command_results[mavutil.mavlink.MAV_CMD_DO_SET_MODE] = None
    outbound = queue.Queue()
    guarded_writer = drone_control._OutputGuardedWriter(MAVWriter(outbound))
    encoder = mavlink2.MAVLink(
        guarded_writer, srcSystem=1, srcComponent=191
    )
    harness.vehicle.message_factory = encoder
    harness.vehicle._master.mav = encoder
    harness.vehicle.send_mavlink = encoder.send
    harness.controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=guarded_writer.transaction,
    )
    message_type = mavlink2.MAVLink_command_long_message
    original_pack = message_type.pack

    def pack_with_old_ack(message, mav, *args, **kwargs):
        packet = original_pack(message, mav, *args, **kwargs)
        harness.observe_ack(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_RESULT_ACCEPTED,
        )
        return packet

    monkeypatch.setattr(message_type, "pack", pack_with_old_ack)
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))

    with pytest.raises(FlightOperationError, match="acknowledgement"):
        harness.controller.set_guided_mode(timeout=0.3)

    assert outbound.qsize() == 1


def test_denied_groundspeed_ack_prevents_waypoint_setpoint(monkeypatch):
    assert hasattr(drone_control, "CommandAckTracker")
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.vehicle.command_results[mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED] = (
        mavutil.mavlink.MAV_RESULT_FAILED
    )

    with pytest.raises(FlightOperationError, match="acknowledgement"):
        harness.controller.goto_waypoint(
            GPSCoord(41.0, -81.0, 10.0), timeout=0.5
        )

    assert harness.vehicle._master.mav.waypoints == []


def test_rejected_mission_ack_fails_waypoint_before_motion_wait(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.vehicle._master.mav.mission_result = mavutil.mavlink.MAV_MISSION_DENIED

    with pytest.raises(FlightOperationError, match="MISSION_ACK rejected"):
        harness.controller.goto_waypoint(
            GPSCoord(41.0, -81.0, 10.0), timeout=0.5
        )

    assert len(harness.vehicle._master.mav.waypoints) == 1
    assert harness.clock.now_value == pytest.approx(10.0)


def test_mission_ack_transactions_serialize_until_each_terminal_ack(monkeypatch):
    harness = Harness(monkeypatch)
    outputs = []
    wake = threading.Event()
    monkeypatch.setattr(drone_control.time, "sleep", lambda _seconds: wake.wait(0.01))
    results = []

    def transact(label):
        results.append(
            harness.controller._send_mission_acknowledged(
                lambda: outputs.append(label), deadline=11.0
            )
        )

    first = threading.Thread(target=transact, args=("first",), daemon=True)
    second = threading.Thread(target=transact, args=("second",), daemon=True)
    first.start()
    assert wall_time_wait(lambda: outputs == ["first"])
    second.start()
    assert not wall_time_wait(lambda: len(outputs) == 2, timeout=0.03)

    harness.observe_mission_ack(mavutil.mavlink.MAV_MISSION_ACCEPTED)
    wake.set()
    assert wall_time_wait(lambda: outputs == ["first", "second"])
    wake.clear()
    harness.observe_mission_ack(mavutil.mavlink.MAV_MISSION_ACCEPTED)
    wake.set()
    first.join(0.2)
    second.join(0.2)

    assert not first.is_alive() and not second.is_alive()
    assert len(results) == 2


def test_mission_ack_timeout_latches_out_later_guided_mission_output(monkeypatch):
    harness = Harness(monkeypatch)
    harness.vehicle._master.mav.mission_callback = None
    outputs = []

    with pytest.raises(drone_control.MissionAckTimeout):
        harness.controller._send_mission_acknowledged(
            lambda: outputs.append("ambiguous"), deadline=10.2
        )

    with pytest.raises(drone_control.MissionAckError, match="further GUIDED"):
        harness.controller._send_mission_acknowledged(
            lambda: outputs.append("retry"), deadline=10.4
        )

    assert outputs == ["ambiguous"]


def test_abort_during_mission_ack_wait_latches_out_recovery_mission_output(monkeypatch):
    harness = Harness(monkeypatch)
    harness.vehicle._master.mav.mission_callback = None
    outputs = []
    abort_after_enqueue = [False]
    abort = MissionAbort("QGC abort during MISSION_ACK wait")

    def guard():
        if abort_after_enqueue[0]:
            abort_after_enqueue[0] = False
            raise abort
        harness.guard()

    def ambiguous_output():
        outputs.append("initial")
        abort_after_enqueue[0] = True

    harness.controller.permission_guard = guard
    with pytest.raises(MissionAbort) as raised:
        harness.controller._send_mission_acknowledged(
            ambiguous_output, deadline=10.2
        )
    assert raised.value is abort

    with pytest.raises(drone_control.MissionAckError, match="further GUIDED"):
        harness.controller._send_mission_acknowledged(
            lambda: outputs.append("recovery"), deadline=10.4
        )

    assert outputs == ["initial"]


@pytest.mark.parametrize(
    ("operation", "command", "state"),
    [
        (
            lambda controller: controller.arm(timeout=0.5),
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            {"mode": "GUIDED", "armed": False, "landed_state": 1},
        ),
        (
            lambda controller: controller.takeoff(10.0, timeout=0.5),
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            {"mode": "GUIDED", "armed": True, "landed_state": 1},
        ),
        (
            lambda controller: controller.disarm(timeout=0.5),
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            {"mode": "LAND", "armed": True, "landed_state": 1},
        ),
    ],
)
def test_rejected_command_ack_fails_arm_takeoff_and_disarm(
    monkeypatch, operation, command, state
):
    harness = Harness(monkeypatch)
    harness.publish(**state)
    if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
        harness.establish_post_arm_home()
    harness.vehicle.command_results[command] = mavutil.mavlink.MAV_RESULT_DENIED

    with pytest.raises(FlightOperationError, match="acknowledgement"):
        operation(harness.controller)


def test_in_progress_ack_waits_for_accepted_ack_and_state_confirmation(monkeypatch):
    harness = Harness(monkeypatch)

    def acknowledge_in_progress_then_accept(command, _result):
        harness.observe_ack(command, mavutil.mavlink.MAV_RESULT_IN_PROGRESS)
        harness.clock.after(
            0.1,
            lambda: harness.observe_ack(command, mavutil.mavlink.MAV_RESULT_ACCEPTED),
        )

    harness.vehicle.command_callback = acknowledge_in_progress_then_accept
    harness.vehicle.command_results[mavutil.mavlink.MAV_CMD_DO_SET_MODE] = (
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS
    )
    harness.clock.after(0.2, lambda: harness.observe_mode("GUIDED"))

    assert harness.controller.set_guided_mode(timeout=0.5) == 0


def test_first_guided_output_delivery_callback_follows_transport_enqueue(monkeypatch):
    harness = Harness(monkeypatch)
    events = []
    original_send = harness.vehicle.send_mavlink

    def send(message):
        events.append("transport-enqueue")
        original_send(message)

    harness.vehicle.send_mavlink = send
    harness.controller.guided_output_delivery_callback = lambda: events.append(
        "guided-delivered"
    )
    harness.controller._guided_output_delivery_reported = False
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))

    assert harness.controller.set_guided_mode(timeout=0.5) == 0
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))
    assert harness.controller.set_guided_mode(timeout=0.5) == 0
    harness.clock.after(0.1, lambda: harness.observe_mode("LAND"))
    assert harness.controller.set_land_mode(timeout=0.5) == 0
    assert events == [
        "transport-enqueue",
        "guided-delivered",
        "transport-enqueue",
        "transport-enqueue",
    ]


def test_first_guided_delivery_rechecks_grounded_disarmed_at_enqueue(monkeypatch):
    harness = Harness(monkeypatch)
    deliveries = []
    harness.controller.guided_output_delivery_callback = lambda: deliveries.append(
        "guided-delivered"
    )
    harness.controller._guided_output_delivery_reported = False
    prepare = harness.controller._prepared_mode_command

    def prepare_then_become_airborne(mode):
        message = prepare(mode)
        harness.publish(
            armed=True,
            landed_state=mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
        )
        return message

    monkeypatch.setattr(
        harness.controller, "_prepared_mode_command", prepare_then_become_airborne
    )

    with pytest.raises(FlightOperationError, match="grounded and disarmed"):
        harness.controller.set_guided_mode(timeout=0.3)

    assert harness.vehicle.mode_requests == []
    assert deliveries == []


def test_guided_output_delivery_callback_rejects_false_delivery(monkeypatch):
    harness = Harness(monkeypatch)
    deliveries = []
    harness.controller.guided_output_delivery_callback = lambda: deliveries.append(
        "guided-delivered"
    )
    harness.controller._guided_output_delivery_reported = False
    harness.vehicle.send_mavlink = lambda _message: (_ for _ in ()).throw(
        RuntimeError("transport failed")
    )

    with pytest.raises(RuntimeError, match="transport failed"):
        harness.controller.set_guided_mode(timeout=0.5)

    assert deliveries == []


def test_recovery_waypoint_raises_target_to_fresh_enqueue_altitude(monkeypatch):
    harness = Harness(monkeypatch)
    harness.controller._mission_home = MissionHome(41.0, -81.0, 100.0)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
        location=(410000000, -810000000, 125000, 25000),
    )
    approvals = []
    original_ack = harness.vehicle.command_callback

    def acknowledge(command, result):
        if command == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
            harness.publish(location=(410000000, -810000000, 135000, 35000))
        original_ack(command, result)

    harness.vehicle.command_callback = acknowledge
    harness.clock.after(
        0.1,
        lambda: harness.publish(
            location=(410000000, -810000000, 135000, 35000)
        ),
    )

    assert harness.controller.goto_recovery_waypoint(
        GPSCoord(41.0, -81.0, 25.0),
        approve_target_amsl=approvals.append,
        timeout=0.5,
    ) == 0

    assert approvals == [135.0]
    waypoint = harness.vehicle._master.mav.waypoints[0]
    assert waypoint[:2] == (1, 1)
    assert waypoint[-1] == 135.0


def test_recovery_waypoint_rejects_raised_unapproved_altitude_before_enqueue(
    monkeypatch,
):
    harness = Harness(monkeypatch)
    harness.controller._mission_home = MissionHome(41.0, -81.0, 100.0)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
        location=(410000000, -810000000, 125000, 25000),
    )
    original_ack = harness.vehicle.command_callback

    def acknowledge(command, result):
        if command == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
            harness.publish(location=(410000000, -810000000, 135000, 35000))
        original_ack(command, result)

    harness.vehicle.command_callback = acknowledge

    with pytest.raises(FlightOperationError, match="raised altitude rejected"):
        harness.controller.goto_recovery_waypoint(
            GPSCoord(41.0, -81.0, 25.0),
            approve_target_amsl=lambda _amsl: (_ for _ in ()).throw(
                FlightOperationError("raised altitude rejected")
            ),
            timeout=0.5,
        )

    assert harness.vehicle._master.mav.waypoints == []


def test_recovery_waypoint_rejects_altitude_rise_after_packet_pack(monkeypatch):
    harness = Harness(monkeypatch)
    harness.controller._mission_home = MissionHome(41.0, -81.0, 100.0)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
        location=(410000000, -810000000, 125000, 25000),
    )
    outbound = queue.Queue()
    guarded_writer = drone_control._OutputGuardedWriter(MAVWriter(outbound))
    harness.vehicle._master.mav = mavutil.mavlink.MAVLink(guarded_writer)
    original_send = harness.vehicle.send_mavlink

    def send_with_real_speed_boundary(message):
        if isinstance(message, tuple) and message[0] == "command-long":
            guarded_writer.write(b"speed")
        return original_send(message)

    harness.vehicle.send_mavlink = send_with_real_speed_boundary
    harness.controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=guarded_writer.transaction,
    )
    message_type = mavutil.mavlink.MAVLink_mission_item_int_message
    original_pack = message_type.pack

    def pack_then_climb(message, mav, *args, **kwargs):
        packet = original_pack(message, mav, *args, **kwargs)
        harness.publish(location=(410000000, -810000000, 135000, 35000))
        return packet

    monkeypatch.setattr(message_type, "pack", pack_then_climb)
    approvals = []

    with pytest.raises(FlightOperationError, match="after packet preparation"):
        harness.controller.goto_recovery_waypoint(
            GPSCoord(41.0, -81.0, 25.0),
            approve_target_amsl=approvals.append,
            timeout=0.5,
        )

    assert approvals == []
    assert outbound.qsize() == 1
    assert outbound.get_nowait() == b"speed"


def test_ack_and_state_confirmation_share_one_operation_deadline(monkeypatch):
    harness = Harness(monkeypatch)

    def delay_ack(command, _result):
        harness.clock.after(
            0.4,
            lambda: harness.observe_ack(command, mavutil.mavlink.MAV_RESULT_ACCEPTED),
        )

    harness.vehicle.command_callback = delay_ack
    harness.clock.after(0.8, lambda: harness.observe_mode("GUIDED"))

    with pytest.raises(FlightOperationError, match="GUIDED"):
        harness.controller.set_guided_mode(timeout=0.6)

    assert harness.clock.now_value == pytest.approx(10.6)


def test_invalid_mode_timeout_is_rejected_before_output(monkeypatch):
    harness = Harness(monkeypatch)

    with pytest.raises(ValueError, match="timeout"):
        harness.controller.set_guided_mode(timeout=0.0)

    assert harness.vehicle.mode_requests == []


def test_denied_arming_is_bounded_and_never_force_arms(monkeypatch):
    harness = Harness(monkeypatch)
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))
    harness.controller.set_guided_mode(timeout=1.0)

    with pytest.raises(FlightOperationError, match="arm"):
        harness.controller.arm(timeout=1.0)

    assert harness.vehicle.arm_requests == [True]
    assert harness.clock.now_value == pytest.approx(11.1)


def test_force_arm_takeoff_confirms_guided_arm_and_ascent_in_order(monkeypatch):
    harness = Harness(monkeypatch)
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))
    harness.clock.after(0.2, lambda: harness.publish(armed=True))
    harness.clock.after(
        0.25,
        lambda: harness.publish(home=(410000000, -810000000, 250000)),
    )
    harness.clock.after(
        0.3,
        lambda: harness.publish(
            landed_state=2,
            location=(410000000, -810000000, 260000, 10000),
        ),
    )

    harness.controller.force_arm_takeoff(10.0)

    assert harness.vehicle.mode_requests == ["GUIDED"]
    assert harness.vehicle.arm_requests == [True]
    assert harness.vehicle.takeoffs == [10.0]


def test_invalid_force_takeoff_altitude_cannot_change_mode_or_arm(monkeypatch):
    harness = Harness(monkeypatch)

    with pytest.raises(FlightOperationError, match="altitude"):
        harness.controller.force_arm_takeoff(float("nan"))

    assert harness.vehicle.mode_requests == []
    assert harness.vehicle.arm_requests == []
    assert harness.vehicle.takeoffs == []


def test_staged_startup_verification_runs_once_before_arm_output(monkeypatch):
    harness = Harness(monkeypatch, install_complete_telemetry=False)
    harness.publish(mode="GUIDED", armed=False, landed_state=1)
    events = []

    evidence = TelemetryStartupEvidence(MappingProxyType({0: (10.5, 11.0, 11.5)}), 10.0)

    def collect_after_guided():
        events.append("verified")
        return evidence

    verification = _TelemetryStartupVerification(
        SimpleNamespace(verify_after_guided=collect_after_guided)
    )
    harness.controller.install_startup_telemetry_verifier(
        verification.verify_after_guided
    )
    harness.controller.guided_output_delivery_callback = lambda: events.append(
        "guided-delivered"
    )
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))
    harness.controller.set_guided_mode(timeout=1.0)
    harness.vehicle.before_arm_output = lambda _value: events.append("arm-output")
    harness.clock.after(0.1, lambda: harness.publish(armed=True))

    assert harness.controller.arm(timeout=1.0) == 0
    assert events == ["guided-delivered", "verified", "arm-output"]
    assert verification.evidence is evidence


@pytest.mark.parametrize(
    "original",
    [
        TimeoutError("mission deadline expired"),
        timebase.ClockError("simulation stopped"),
        MissionAbort("QGC aborted"),
        AuthorityLost("pilot takeover"),
    ],
)
def test_staged_startup_verification_preserves_control_flow_exception_and_latches(
    monkeypatch, original
):
    harness = Harness(monkeypatch, install_complete_telemetry=False)
    harness.publish(mode="GUIDED", armed=False, landed_state=1)
    attempts = []

    def fail_verification():
        attempts.append("attempt")
        raise original

    harness.controller.install_startup_telemetry_verifier(fail_verification)
    harness.controller.guided_output_delivery_callback = lambda: None
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))
    harness.controller.set_guided_mode(timeout=1.0)

    with pytest.raises(type(original)) as caught:
        harness.controller.arm(timeout=1.0)
    assert caught.value is original
    with pytest.raises(FlightOperationError, match="startup telemetry verification failed"):
        harness.controller.arm(timeout=1.0)

    assert attempts == ["attempt"]
    assert harness.vehicle.arm_requests == []


def test_takeoff_rejects_unverified_staged_startup_before_packet_creation(monkeypatch):
    harness = Harness(monkeypatch, install_complete_telemetry=False)
    harness.publish(mode="GUIDED", armed=True, landed_state=1)
    harness.establish_post_arm_home()
    harness.controller.install_startup_telemetry_verifier(lambda: None)

    with pytest.raises(FlightOperationError, match="startup telemetry is not verified"):
        harness.controller.takeoff(10.0, timeout=1.0)

    assert harness.vehicle.takeoffs == []


def test_missing_startup_telemetry_installation_denies_arm_without_output(monkeypatch):
    harness = Harness(monkeypatch, install_complete_telemetry=False)
    harness.publish(mode="GUIDED", armed=False, landed_state=1)

    with pytest.raises(FlightOperationError, match="startup telemetry"):
        harness.controller.arm(timeout=1.0)

    assert harness.vehicle.arm_requests == []


def test_pending_verifier_is_not_invoked_before_guarded_guided_delivery(monkeypatch):
    harness = Harness(monkeypatch, install_complete_telemetry=False)
    harness.publish(mode="GUIDED", armed=False, landed_state=1)
    attempts = []
    harness.controller.install_startup_telemetry_verifier(
        lambda: attempts.append("verify")
    )

    with pytest.raises(FlightOperationError, match="GUIDED delivery"):
        harness.controller.arm(timeout=1.0)

    assert attempts == []
    assert harness.vehicle.arm_requests == []


def test_takeoff_without_postcommand_ascent_raises(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=1)
    harness.establish_post_arm_home()

    with pytest.raises(FlightOperationError, match="ascent"):
        harness.controller.takeoff(10.0, timeout=1.0)

    assert harness.vehicle.takeoffs == [10.0]


def test_takeoff_cannot_succeed_while_still_on_ground(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=1)
    harness.establish_post_arm_home()
    harness.clock.after(
        0.1,
        lambda: harness.publish(
            armed=True,
            landed_state=1,
            location=(410000000, -810000000, 250000, 500),
        ),
    )

    with pytest.raises(FlightOperationError, match="ascent"):
        harness.controller.takeoff(0.5, timeout=0.3)


def test_takeoff_uses_final_transmission_snapshot_as_ascent_baseline(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=1,
        location=(410000000, -810000000, 250000, 0),
    )
    harness.establish_post_arm_home()
    calls = [0]

    def publish_target_altitude_during_final_guard():
        calls[0] += 1
        if calls[0] == 3:
            harness.publish(location=(410000000, -810000000, 260000, 10000))
            harness.clock.after(
                0.1,
                lambda: harness.publish(
                    armed=True,
                    landed_state=2,
                    location=(410000000, -810000000, 260000, 10000),
                ),
            )

    harness.controller.permission_guard = publish_target_altitude_during_final_guard

    with pytest.raises(FlightOperationError, match="above current"):
        harness.controller.takeoff(10.0, timeout=0.3)

    assert harness.vehicle.takeoffs == []


def test_takeoff_rejects_fc_home_change_after_packet_is_prepared(monkeypatch):
    """Break caught: packed takeoff altitude must match the final FC HOME datum."""
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=1,
        location=(410000000, -810000000, 250000, 0),
    )
    harness.establish_post_arm_home(250000)
    outbound = queue.Queue()
    guarded_writer = drone_control._OutputGuardedWriter(MAVWriter(outbound))
    harness.controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=guarded_writer.transaction,
    )

    class PackedTakeoff:
        def __init__(self, target_altitude):
            self.target_altitude = target_altitude

        def pack(self, _encoder, **_kwargs):
            harness.publish(home=(410000000, -810000000, 255000))
            return f"takeoff:{self.target_altitude}".encode()

    def encode(*fields):
        assert fields[2] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        return PackedTakeoff(fields[10])

    harness.vehicle.message_factory.command_long_encode = encode
    harness.vehicle.send_mavlink = lambda message: guarded_writer.write(
        message.pack(object())
    )

    with pytest.raises((AuthorityLost, FlightOperationError)):
        harness.controller.takeoff(10.0, timeout=0.3)

    assert outbound.empty()


def test_climb_requires_positive_airborne_state(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=0,
        location=(410000000, -810000000, 250000, 10000),
    )
    harness.establish_post_arm_home()

    with pytest.raises(FlightOperationError, match="airborne"):
        harness.controller.climb(10.5, timeout=0.3)

    assert harness.vehicle.gotos == []


def test_climb_uses_final_transmission_snapshot_as_ascent_baseline(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        location=(410000000, -810000000, 260000, 10000),
    )
    harness.establish_post_arm_home()
    def move_before_enqueue(operation, enqueue_check):
        harness.publish(location=(410000000, -810000000, 260500, 10500))
        harness.clock.after(
            0.1,
            lambda: harness.publish(
                location=(410000000, -810000000, 260500, 10500)
            ),
        )
        enqueue_check()
        return operation()

    harness.controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=move_before_enqueue,
    )

    with pytest.raises(FlightOperationError, match="above current"):
        harness.controller.climb(10.5, timeout=0.3)

    assert harness.vehicle.gotos == []


def test_climb_output_does_not_invert_receiver_admission_lock_order(monkeypatch):
    """Break caught: nested climb permission must not deadlock admission."""
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        location=(410000000, -810000000, 260000, 10000),
    )
    decoder_lock = threading.RLock()
    supervisor_state_lock = threading.Lock()
    output_about_to_run = threading.Event()
    receiver_holds_state = threading.Event()
    receiver_acquired_decoder = []
    errors = []

    def dependency_transaction(operation):
        with decoder_lock:
            return operation()

    def permission_guard():
        with supervisor_state_lock:
            return None

    def transport_transaction(operation, enqueue_check):
        enqueue_check()
        output_about_to_run.set()
        assert receiver_holds_state.wait(1.0)
        return operation()

    harness.controller.permission_guard = permission_guard
    harness.controller.install_output_transactions(
        dependency_transaction=dependency_transaction,
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=transport_transaction,
    )
    original_send = harness.vehicle._master.mav.mission_item_int_send

    def send_and_confirm(*fields):
        original_send(*fields)
        harness.clock.after(
            0.1,
            lambda: harness.publish(
                location=(fields[11], fields[12], int(fields[13] * 1000), 12000)
            ),
        )

    harness.vehicle._master.mav.mission_item_int_send = send_and_confirm

    def receiver():
        assert output_about_to_run.wait(1.0)
        with supervisor_state_lock:
            receiver_holds_state.set()
            acquired = decoder_lock.acquire(timeout=0.2)
            receiver_acquired_decoder.append(acquired)
            if acquired:
                decoder_lock.release()

    def climb():
        try:
            harness.controller.climb(12.0, timeout=0.5)
        except BaseException as error:
            errors.append(error)

    receiver_thread = threading.Thread(target=receiver)
    climb_thread = threading.Thread(target=climb)
    receiver_thread.start()
    climb_thread.start()
    receiver_thread.join(1.0)
    climb_thread.join(1.0)

    assert not receiver_thread.is_alive()
    assert not climb_thread.is_alive()
    assert errors == []
    assert receiver_acquired_decoder == [True]


def test_takeoff_rejects_airborne_request_without_output(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        location=(410000000, -810000000, 250000, 10000),
    )
    harness.establish_post_arm_home()

    with pytest.raises(FlightOperationError, match="already airborne"):
        harness.controller.takeoff(12.0, timeout=1.0)

    assert harness.vehicle.takeoffs == []


def test_takeoff_rejects_target_not_above_current_altitude_without_output(
    monkeypatch,
):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=1,
        location=(410000000, -810000000, 260000, 10000),
    )
    harness.establish_post_arm_home()

    with pytest.raises(FlightOperationError, match="above current"):
        harness.controller.takeoff(10.0, timeout=1.0)

    assert harness.vehicle.takeoffs == []


def test_climb_rejects_descent_without_output(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        location=(410000000, -810000000, 260000, 10000),
    )

    with pytest.raises(FlightOperationError, match="above"):
        harness.controller.climb(9.0, timeout=1.0)

    assert harness.vehicle.gotos == []


def test_stale_position_cannot_confirm_waypoint(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)

    with pytest.raises(FlightOperationError, match="waypoint"):
        harness.controller.goto_waypoint(GPSCoord(41.0, -81.0, 10.0), timeout=1.0)

    assert len(harness.vehicle._master.mav.waypoints) == 1


def test_waypoint_requires_fresh_postcommand_position_and_altitude(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.clock.after(
        0.2,
        lambda: harness.publish(
            location=(410000000, -810000000, 260000, 10000)
        ),
    )

    assert harness.controller.goto_waypoint(
        GPSCoord(41.0, -81.0, 10.0), timeout=1.0
    ) == 0


def test_malformed_postcommand_position_expires_as_operation_failure(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.clock.after(0.1, lambda: harness.publish(location=(410000000, -810000000)))

    with pytest.raises(FlightOperationError, match="waypoint"):
        harness.controller.goto_waypoint(GPSCoord(41.0, -81.0, 10.0), timeout=0.3)

    assert len(harness.vehicle._master.mav.waypoints) == 1


def test_low_relative_altitude_cannot_confirm_touchdown(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.clock.after(0.1, lambda: harness.observe_mode("LAND"))
    harness.clock.after(
        0.2,
        lambda: harness.publish(
            location=(410000000, -810000000, 250000, 100), armed=True
        ),
    )

    with pytest.raises(FlightOperationError, match="touchdown"):
        harness.controller.simple_land(timeout=1.0, mode_timeout=0.5)

    assert harness.vehicle.mode_requests == ["LAND"]


def test_denied_land_never_reports_touchdown(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)

    with pytest.raises(FlightOperationError, match="LAND"):
        harness.controller.simple_land(timeout=1.0, mode_timeout=0.5)

    assert harness.vehicle.mode_requests == ["LAND"]


def test_land_requires_new_explicit_on_ground_evidence(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.clock.after(0.1, lambda: harness.observe_mode("LAND"))
    harness.clock.after(0.2, lambda: harness.publish(landed_state=1, armed=True))

    assert harness.controller.simple_land(timeout=1.0, mode_timeout=0.5) == 0


@pytest.mark.parametrize(
    "final_landed_state",
    [
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        mavutil.mavlink.MAV_LANDED_STATE_LANDING,
    ],
)
def test_land_final_guard_preserves_ground_or_descent_without_mode_output(
    monkeypatch, final_landed_state
):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    calls = [0]

    def transition_during_final_guard():
        calls[0] += 1
        if calls[0] == 4:
            harness.publish(landed_state=final_landed_state)
            if final_landed_state == mavutil.mavlink.MAV_LANDED_STATE_LANDING:
                harness.clock.after(
                    0.1, lambda: harness.publish(landed_state=1, armed=True)
                )

    harness.controller.permission_guard = transition_during_final_guard

    assert harness.controller.simple_land(timeout=1.0, mode_timeout=0.5) == 0
    assert harness.vehicle.mode_requests == []


def test_land_final_guard_preserves_fresh_land_mode_without_reassertion(
    monkeypatch,
):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    calls = [0]

    def accept_land_during_final_guard():
        calls[0] += 1
        if calls[0] == 4:
            harness.observe_mode("LAND")
            harness.clock.after(
                0.1, lambda: harness.publish(landed_state=1, armed=True)
            )

    harness.controller.permission_guard = accept_land_during_final_guard

    assert harness.controller.simple_land(timeout=1.0, mode_timeout=0.5) == 0
    assert harness.vehicle.mode_requests == []


def test_cached_ground_state_from_before_land_cannot_confirm_touchdown(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=1)
    harness.clock.after(0.1, lambda: harness.observe_mode("LAND"))

    with pytest.raises(FlightOperationError, match="touchdown"):
        harness.controller.simple_land(timeout=0.4, mode_timeout=0.2)


def test_auto_disarm_after_land_confirms_touchdown_without_airborne_contradiction(
    monkeypatch,
):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.clock.after(0.1, lambda: harness.observe_mode("LAND"))
    harness.clock.after(
        0.2,
        lambda: harness.publish(
            armed=False,
            landed_state=mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED,
        ),
    )

    assert harness.controller.simple_land(timeout=1.0, mode_timeout=0.5) == 0


def test_auto_disarm_before_confirmed_land_transition_cannot_confirm_touchdown(
    monkeypatch,
):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.clock.after(0.1, lambda: harness.publish(armed=False))
    harness.clock.after(0.2, lambda: harness.observe_mode("LAND"))

    with pytest.raises(FlightOperationError, match="touchdown"):
        harness.controller.simple_land(timeout=0.4, mode_timeout=0.3)


def test_auto_disarm_cannot_override_postcommand_airborne_evidence(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.clock.after(0.1, lambda: harness.observe_mode("LAND"))
    harness.clock.after(0.2, lambda: harness.publish(armed=False, landed_state=2))

    with pytest.raises(FlightOperationError, match="touchdown"):
        harness.controller.simple_land(timeout=0.4, mode_timeout=0.2)


def test_landing_state_contradicts_auto_disarm_touchdown(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.clock.after(0.1, lambda: harness.observe_mode("LAND"))
    harness.clock.after(0.2, lambda: harness.publish(armed=False, landed_state=4))

    with pytest.raises(FlightOperationError, match="touchdown"):
        harness.controller.simple_land(timeout=0.4, mode_timeout=0.2)


def test_repeated_false_after_preland_disarm_is_not_auto_disarm_transition(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    harness.clock.after(0.1, lambda: harness.publish(armed=False))
    harness.clock.after(0.2, lambda: harness.observe_mode("LAND"))
    harness.clock.after(0.3, lambda: harness.publish(armed=False))

    with pytest.raises(FlightOperationError, match="touchdown"):
        harness.controller.simple_land(timeout=0.5, mode_timeout=0.3)


def test_disarm_requires_touchdown_and_delayed_confirmation(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="LAND", armed=True, landed_state=1)
    harness.clock.after(0.3, lambda: harness.publish(armed=False, landed_state=1))

    assert harness.controller.disarm(timeout=1.0) == 0
    assert harness.vehicle.arm_requests == [False]
    assert harness.clock.now_value == pytest.approx(10.3, abs=0.11)


@pytest.mark.parametrize("changed", ["landed", "armed", "mode", "armable"])
def test_arm_revalidates_every_precondition_at_output_boundary(monkeypatch, changed):
    """Break caught: an arm request must not outlive any pre-arm condition."""
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=False, landed_state=1)
    checks = [0]

    def change_before_output():
        checks[0] += 1
        if checks[0] != 3:
            return
        if changed == "landed":
            harness.publish(landed_state=2)
        elif changed == "armed":
            harness.publish(armed=True)
        elif changed == "mode":
            harness.observe_mode("LOITER")
        else:
            harness.vehicle.is_armable = False

    harness.controller.permission_guard = change_before_output

    with pytest.raises((AuthorityLost, FlightOperationError)):
        harness.controller.arm(timeout=0.3)

    assert harness.vehicle.arm_requests == []


def test_disarm_refuses_airborne_output(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)

    with pytest.raises(FlightOperationError, match="touchdown"):
        harness.controller.disarm(timeout=1.0)

    assert harness.vehicle.arm_requests == []


def test_disarm_revalidates_ground_at_final_output_boundary(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="LAND", armed=True, landed_state=1)
    calls = [0]

    def publish_airborne_before_send():
        calls[0] += 1
        if calls[0] == 3:
            harness.publish(landed_state=2)

    harness.controller.permission_guard = publish_airborne_before_send

    with pytest.raises(FlightOperationError, match="touchdown"):
        harness.controller.disarm(timeout=1.0)

    assert harness.vehicle.arm_requests == []


def test_disarm_ack_boundary_takeover_prevents_actual_output(monkeypatch):
    """Break caught: takeover at the ACK boundary must precede disarm enqueue."""
    harness = Harness(monkeypatch)
    harness.publish(mode="LAND", armed=True, landed_state=1)
    tracker = harness.controller._command_ack_tracker
    original_boundary = tracker.boundary

    def takeover_at_boundary():
        boundary = original_boundary()
        harness.sequence += 1
        assert harness.state.observe_rc_input(
            channel=7,
            pwm=1900,
            signal_healthy=True,
            received_at=harness.clock.now(),
            sequence=harness.sequence,
            source_system=1,
            source_component=1,
        )
        harness.sequence += 1
        assert harness.state.observe_mode(
            "LOITER",
            received_at=harness.clock.now(),
            sequence=harness.sequence,
            source_system=1,
            source_component=1,
        )
        return boundary

    tracker.boundary = takeover_at_boundary

    with pytest.raises(AuthorityLost):
        harness.controller.disarm(timeout=0.3)

    assert harness.state.snapshot().authority.name == "UNKNOWN"
    harness.sequence += 1
    assert harness.state.observe_mode(
        "LOITER",
        received_at=harness.clock.now(),
        sequence=harness.sequence,
        source_system=1,
        source_component=1,
    )
    assert harness.state.snapshot().authority.name == "PILOT"
    assert harness.vehicle.arm_requests == []


def test_same_thread_takeover_during_setter_preparation_prevents_enqueue(monkeypatch):
    """Break caught: reentrant setter preparation cannot emit after takeover."""
    harness = Harness(monkeypatch)
    harness.publish(mode="LAND", armed=True, landed_state=1)

    def takeover(_value):
        harness.sequence += 1
        assert harness.state.observe_rc_input(
            channel=7,
            pwm=1900,
            signal_healthy=True,
            received_at=harness.clock.now(),
            sequence=harness.sequence,
            source_system=1,
            source_component=1,
        )
        harness.sequence += 1
        harness.state.observe_mode(
            "LOITER",
            received_at=harness.clock.now(),
            sequence=harness.sequence,
            source_system=1,
            source_component=1,
        )

    harness.vehicle.before_arm_output = takeover

    with pytest.raises(AuthorityLost):
        harness.controller.disarm(timeout=0.3)

    assert harness.vehicle.arm_requests == []


def test_disarm_rejects_airborne_evidence_during_ack_wait(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="LAND", armed=True, landed_state=1)
    harness.vehicle.command_results[
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    ] = None
    calls = [0]

    def publish_airborne_during_ack_wait():
        calls[0] += 1
        if calls[0] == 4:
            harness.publish(landed_state=2)
            harness.clock.after(
                0.1,
                lambda: harness.publish(landed_state=1, armed=False),
            )
            harness.clock.after(
                0.1,
                lambda: harness.observe_ack(
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                ),
            )

    harness.controller.permission_guard = publish_airborne_during_ack_wait

    with pytest.raises(FlightOperationError, match="touchdown"):
        harness.controller.disarm(timeout=0.5)


def test_authority_loss_during_wait_stops_operation(monkeypatch):
    harness = Harness(monkeypatch)
    harness.clock.after(
        0.2,
        lambda: harness.state.invalidate_observation(
            "heartbeat", source_system=1, source_component=1
        ),
    )

    with pytest.raises(AuthorityLost):
        harness.controller.set_guided_mode(timeout=1.0)

    assert harness.vehicle.mode_requests == ["GUIDED"]


def test_authority_loss_between_permission_checks_prevents_output(monkeypatch):
    harness = Harness(monkeypatch)
    calls = [0]

    def revoke_at_boundary():
        calls[0] += 1
        if calls[0] == 2:
            harness.state.invalidate_observation(
                "heartbeat", source_system=1, source_component=1
            )

    harness.controller.permission_guard = revoke_at_boundary

    with pytest.raises(AuthorityLost):
        harness.controller.set_guided_mode(timeout=1.0)

    assert harness.vehicle.mode_requests == []


def test_pretransmission_position_does_not_confirm_waypoint(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    calls = [0]

    def publish_target_during_guard():
        calls[0] += 1
        if calls[0] == 2:
            harness.publish(location=(410000000, -810000000, 250000, 10000))

    harness.controller.permission_guard = publish_target_during_guard

    with pytest.raises(FlightOperationError, match="waypoint"):
        harness.controller.goto_waypoint(
            GPSCoord(41.0, -81.0, 10.0), timeout=0.3
        )

    assert len(harness.vehicle._master.mav.waypoints) == 1


def test_latest_guard_position_does_not_confirm_waypoint(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    calls = [0]

    def publish_target_during_waypoint_inner_guard():
        calls[0] += 1
        if calls[0] == 7:
            harness.publish(location=(410000000, -810000000, 250000, 10000))

    harness.controller.permission_guard = publish_target_during_waypoint_inner_guard

    with pytest.raises(FlightOperationError, match="waypoint"):
        harness.controller.goto_waypoint(
            GPSCoord(41.0, -81.0, 10.0), timeout=0.3
        )

    assert len(harness.vehicle._master.mav.waypoints) == 1


def test_relative_offset_uses_frd_once_and_requires_observed_completion(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        attitude=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        location=(410000000, -810000000, 250000, 10000),
    )
    north_degrees = 1.0 / 6378137.0 * 180.0 / math.pi
    harness.clock.after(
        0.2,
        lambda: harness.publish(
            location=(
                round((41.0 + north_degrees) * 1e7),
                -810000000,
                250000,
                10000,
            )
        ),
    )

    assert harness.controller.guide_move_relative_frame(
        RelPosComplete(1.0, 0.0, 0.0), timeout=1.0
    ) == 0

    assert len(harness.vehicle.message_factory.relative) == 1
    fields = harness.vehicle.message_factory.relative[0]
    assert fields[1:3] == (1, 1)
    assert fields[3:8] == (
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0x0DF8,
        1.0,
        0.0,
        0.0,
    )


def test_relative_encoder_authority_loss_prevents_transport(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        attitude=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        location=(410000000, -810000000, 250000, 10000),
    )
    original_encode = harness.vehicle.message_factory.set_position_target_local_ned_encode

    def revoke_while_encoding(*fields):
        message = original_encode(*fields)
        harness.state.invalidate_observation(
            "heartbeat", source_system=1, source_component=1
        )
        return message

    harness.vehicle.message_factory.set_position_target_local_ned_encode = revoke_while_encoding

    with pytest.raises(AuthorityLost):
        harness.controller.guide_move_relative_frame(
            RelPosComplete(1.0, 0.0, 0.0), timeout=1.0
        )

    assert harness.vehicle.sent == []


def test_relative_target_uses_send_boundary_pose_not_early_pose(monkeypatch):
    """Break caught: pre-send motion must not satisfy a new relative command."""
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        attitude=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        location=(410000000, -810000000, 250000, 10000),
    )
    north_degrees = 5.0 / 6378137.0 * 180.0 / math.pi
    shifted = (
        round((41.0 + north_degrees) * 1e7),
        -810000000,
        250000,
        10000,
    )
    original_encode = harness.vehicle.message_factory.set_position_target_local_ned_encode

    def move_during_encoding(*fields):
        harness.publish(location=shifted)
        harness.clock.after(0.1, lambda: harness.publish(location=shifted))
        return original_encode(*fields)

    harness.vehicle.message_factory.set_position_target_local_ned_encode = move_during_encoding

    with pytest.raises(FlightOperationError, match="relative movement"):
        harness.controller.guide_move_relative_frame(
            RelPosComplete(5.0, 0.0, 0.0), timeout=0.3
        )

    assert len(harness.vehicle.sent) == 1


def test_relative_target_uses_writer_boundary_pose_after_message_pack(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        attitude=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        location=(410000000, -810000000, 250000, 10000),
    )
    north_degrees = 5.0 / 6378137.0 * 180.0 / math.pi
    shifted = (
        round((41.0 + north_degrees) * 1e7),
        -810000000,
        250000,
        10000,
    )
    outbound = queue.Queue()
    guarded_writer = drone_control._OutputGuardedWriter(MAVWriter(outbound))
    harness.controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=guarded_writer.transaction,
    )

    class PackedRelative:
        def pack(self, _encoder, **_kwargs):
            harness.publish(location=shifted)
            harness.clock.after(0.1, lambda: harness.publish(location=shifted))
            return b"relative"

    harness.vehicle.message_factory.set_position_target_local_ned_encode = (
        lambda *_fields: PackedRelative()
    )
    harness.vehicle.send_mavlink = lambda message: guarded_writer.write(
        message.pack(object())
    )

    with pytest.raises(FlightOperationError, match="relative movement"):
        harness.controller.guide_move_relative_frame(
            RelPosComplete(5.0, 0.0, 0.0), timeout=0.3
        )

    assert outbound.qsize() == 1


def test_armability_change_during_message_pack_prevents_writer_enqueue(monkeypatch):
    harness = Harness(monkeypatch)
    outbound = queue.Queue()
    guarded_writer = drone_control._OutputGuardedWriter(MAVWriter(outbound))
    harness.controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=guarded_writer.transaction,
    )

    class PackedArm:
        def pack(self, _encoder, **_kwargs):
            harness.vehicle.is_armable = False
            return b"arm"

    harness.vehicle.message_factory.command_long_encode = lambda *_fields: PackedArm()
    harness.vehicle.send_mavlink = lambda message: guarded_writer.write(
        message.pack(object())
    )

    with pytest.raises(FlightOperationError, match="pre-arm"):
        harness.controller.arm(timeout=0.3)

    assert outbound.empty()


def test_nonfinite_landing_target_is_rejected_before_output(monkeypatch):
    harness = Harness(monkeypatch)

    with pytest.raises(FlightOperationError, match="finite"):
        harness.controller.land_send_landing_target(
            RelPosComplete(float("nan"), 0.0, 1.0)
        )

    assert harness.vehicle.sent == []


def test_landing_target_requires_current_land_mode(monkeypatch):
    harness = Harness(monkeypatch)

    with pytest.raises(FlightOperationError, match="LAND mode"):
        harness.controller.land_send_landing_target(RelPosComplete(0.0, 0.0, 1.0))

    assert harness.vehicle.sent == []


def test_pitch_stability_resets_after_out_of_limit_observation(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(attitude=(0.0, 0.1, 0.0, 0.0, 0.0, 0.0))
    harness.clock.after(
        0.2,
        lambda: harness.publish(attitude=(0.0, 0.5, 0.0, 0.0, 0.0, 0.0)),
    )
    harness.clock.after(
        0.4,
        lambda: harness.publish(attitude=(0.0, 0.1, 0.0, 0.0, 0.0, 0.0)),
    )

    assert harness.controller.wait_until_stable(
        vel_threshold=0.3, stable_duration=0.3, timeout=1.0
    )
    assert harness.clock.now_value >= 10.7


def test_stale_hold_evidence_returns_false_without_waypoint_output(monkeypatch):
    harness = Harness(monkeypatch)
    harness.clock.now_value = 11.0

    assert not harness.controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0),
        SimpleNamespace(get_distance=lambda: 10.0),
        required_agl_m=10.0,
    )
    assert harness.vehicle._master.mav.waypoints == []


def test_unimplemented_movement_apis_raise():
    controller = object.__new__(DroneControl)

    with pytest.raises(NotImplementedError):
        controller.move_relative_ned(object())
    with pytest.raises(NotImplementedError):
        controller.translate_relative(object())


def test_mission_home_requires_approved_fresh_ground_datum_and_is_immutable(monkeypatch):
    harness = Harness(monkeypatch)
    approved = []
    harness.controller._mission_home = None
    harness.controller.mission_home_check = approved.append
    home = MissionHome(41.0, -81.0, 250.0)

    harness.controller.set_mission_home(home)
    harness.controller.set_mission_home(home)

    assert harness.controller.mission_home is home
    assert approved == [home]
    with pytest.raises(FlightOperationError, match="cannot change"):
        harness.controller.set_mission_home(MissionHome(41.0, -81.0, 251.0))


def test_mission_home_fails_closed_without_operating_area_approval(monkeypatch):
    harness = Harness(monkeypatch)
    harness.controller._mission_home = None
    harness.controller.mission_home_check = None

    with pytest.raises(FlightOperationError, match="operating-area"):
        harness.controller.set_mission_home(MissionHome(41.0, -81.0, 250.0))


def test_mission_home_revalidates_ground_after_area_approval(monkeypatch):
    harness = Harness(monkeypatch)
    harness.controller._mission_home = None

    def approve_then_liftoff(_home):
        harness.publish(landed_state=2, armed=True)

    harness.controller.mission_home_check = approve_then_liftoff

    with pytest.raises(FlightOperationError, match="ON_GROUND"):
        harness.controller.set_mission_home(MissionHome(41.0, -81.0, 250.0))
    assert harness.controller.mission_home is None


def test_force_takeoff_rejects_missing_home_before_mode_or_arm(monkeypatch):
    harness = Harness(monkeypatch)
    harness.controller._mission_home = None

    with pytest.raises(FlightOperationError, match="pinned mission home"):
        harness.controller.force_arm_takeoff(10.0)
    assert harness.vehicle.mode_requests == []
    assert harness.vehicle.arm_requests == []


def test_get_current_gps_and_waypoint_use_pinned_home_and_amsl(monkeypatch):
    harness = Harness(monkeypatch)
    harness.controller.mission_home_check = lambda home: None
    harness.controller.set_mission_home(MissionHome(41.0, -81.0, 250.0))
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        location=(410000000, -810000000, 255000, 47000),
    )

    assert harness.controller.get_current_gps() == GPSCoord(41.0, -81.0, 5.0)
    harness.clock.after(
        0.2,
        lambda: harness.publish(
            location=(410000000, -810000000, 260000, 52000)
        ),
    )

    assert harness.controller.goto_waypoint(
        GPSCoord(41.0, -81.0, 10.0), timeout=1.0
    ) == 0
    fields = harness.vehicle._master.mav.waypoints[-1]
    assert fields[3] == mavutil.mavlink.MAV_FRAME_GLOBAL_INT
    assert fields[13] == 260.0


def test_takeoff_uses_fresh_post_arm_fc_home_after_rearm(monkeypatch):
    harness = Harness(monkeypatch)
    harness.controller.mission_home_check = lambda home: None
    harness.controller.set_mission_home(MissionHome(41.0, -81.0, 250.0))
    harness.clock.after(0.1, lambda: harness.observe_mode("GUIDED"))
    harness.clock.after(0.2, lambda: harness.publish(armed=True))
    harness.clock.after(
        0.25,
        lambda: harness.publish(home=(410000000, -810000000, 245000)),
    )
    harness.clock.after(
        0.3,
        lambda: harness.publish(
            landed_state=2,
            location=(410000000, -810000000, 260000, 15000),
        ),
    )

    harness.controller.force_arm_takeoff(10.0)

    assert harness.vehicle.takeoffs == [15.0]


def test_original_home_survives_multiple_rearms_with_new_fc_home_data(monkeypatch):
    harness = Harness(monkeypatch)

    harness.clock.after(0.1, lambda: harness.publish(armed=True))
    harness.controller.arm(timeout=1.0)
    harness.clock.after(
        0.1,
        lambda: harness.publish(home=(410000000, -810000000, 245000)),
    )
    harness.clock.after(
        0.2,
        lambda: harness.publish(
            landed_state=2,
            location=(410000000, -810000000, 260000, 15000),
        ),
    )
    harness.controller.takeoff(10.0, timeout=1.0)

    harness.publish(
        mode="GUIDED",
        armed=False,
        landed_state=1,
        location=(410000000, -810000000, 250000, 10000),
    )
    harness.clock.after(0.1, lambda: harness.publish(armed=True))
    harness.controller.arm(timeout=1.0)
    harness.clock.after(
        0.1,
        lambda: harness.publish(home=(410000000, -810000000, 240000)),
    )
    harness.clock.after(
        0.2,
        lambda: harness.publish(
            landed_state=2,
            location=(410000000, -810000000, 260000, 20000),
        ),
    )
    harness.controller.takeoff(10.0, timeout=1.0)

    assert harness.controller.mission_home == MissionHome(41.0, -81.0, 250.0)
    assert harness.vehicle.takeoffs == [15.0, 20.0]


def test_climb_encodes_original_home_target_as_global_amsl(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        location=(410000000, -810000000, 255000, 47000),
    )
    harness.clock.after(
        0.2,
        lambda: harness.publish(
            location=(410000000, -810000000, 260000, 52000)
        ),
    )

    harness.controller.climb(10.0, timeout=1.0)

    assert harness.vehicle.gotos == []
    fields = harness.vehicle._master.mav.waypoints[-1]
    assert fields[:2] == (1, 1)
    assert fields[3] == mavutil.mavlink.MAV_FRAME_GLOBAL_INT
    assert fields[13] == 260.0


def test_climb_uses_pinned_fc_ids_at_actual_writer_boundary(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        location=(410000000, -810000000, 255000, 47000),
    )
    harness.clock.after(
        0.2,
        lambda: harness.publish(
            location=(410000000, -810000000, 260000, 52000)
        ),
    )
    outbound = queue.Queue()
    guarded_writer = drone_control._OutputGuardedWriter(MAVWriter(outbound))
    harness.vehicle._master.mav = mavutil.mavlink.MAVLink(guarded_writer)
    harness.controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=guarded_writer.transaction,
    )
    harness.clock.after(
        0.1,
        lambda: harness.observe_mission_ack(mavutil.mavlink.MAV_MISSION_ACCEPTED),
    )

    harness.controller.climb(10.0, timeout=1.0)

    packet = outbound.get_nowait()
    message = mavutil.mavlink.MAVLink(None).decode(bytearray(packet))
    assert message.get_type() == "MISSION_ITEM_INT"
    assert (message.target_system, message.target_component) == (1, 1)


def test_mission_ack_packed_before_actual_writer_enqueue_is_stale(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
        location=(410000000, -810000000, 255000, 47000),
    )
    outbound = queue.Queue()
    guarded_writer = drone_control._OutputGuardedWriter(MAVWriter(outbound))
    encoder = mavlink2.MAVLink(guarded_writer, srcSystem=1, srcComponent=191)
    harness.vehicle._master.mav = encoder
    harness.controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=guarded_writer.transaction,
    )
    message_type = mavlink2.MAVLink_mission_item_int_message
    original_pack = message_type.pack

    def pack_with_old_ack(message, mav, *args, **kwargs):
        packet = original_pack(message, mav, *args, **kwargs)
        harness.observe_mission_ack(mavutil.mavlink.MAV_MISSION_ACCEPTED)
        return packet

    monkeypatch.setattr(message_type, "pack", pack_with_old_ack)
    harness.clock.after(
        0.1,
        lambda: harness.publish(
            location=(410000000, -810000000, 260000, 52000)
        ),
    )

    with pytest.raises(drone_control.MissionAckTimeout):
        harness.controller.climb(10.0, timeout=0.3)

    assert outbound.qsize() == 1


def test_confirm_landing_preserves_accepted_land_without_new_mode_output(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="LAND", armed=True, landed_state=4)
    harness.clock.after(0.2, lambda: harness.publish(landed_state=1, armed=True))

    assert harness.controller.confirm_landing(timeout=1.0) == 0
    assert harness.vehicle.mode_requests == []


def test_waypoint_hold_reissues_pinned_amsl_target(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        velocity=(0, 0, 0),
        attitude=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        location=(410000000, -810000000, 260000, 52000),
    )

    class CurrentLidar:
        def __init__(self):
            self.sequence = 0

        def get_sample(self):
            self.sequence += 1
            return LidarSample(
                10.0, harness.clock.now(), self.sequence, 0
            )

    def publish_fresh_observations():
        harness.publish(
            mode="GUIDED",
            armed=True,
            landed_state=2,
            velocity=(0, 0, 0),
            attitude=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            location=(410000000, -810000000, 260000, 52000),
        )
        harness.clock.after(0.1, publish_fresh_observations)

    harness.clock.after(0.1, publish_fresh_observations)

    assert harness.controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0),
        CurrentLidar(),
        required_agl_m=10.0,
    )
    fields = harness.vehicle._master.mav.waypoints[0]
    assert fields[3] == mavutil.mavlink.MAV_FRAME_GLOBAL_INT
    assert fields[13] == 260.0


def test_release_hold_does_not_retry_ambiguous_mission_ack_timeout(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        velocity=(0, 0, 0),
        attitude=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        location=(410000000, -810000000, 260000, 52000),
    )
    harness.vehicle._master.mav.mission_result = None

    class CurrentLidar:
        def __init__(self):
            self.sequence = 0

        def get_sample(self):
            self.sequence += 1
            return LidarSample(10.0, harness.clock.now(), self.sequence, 0)

    with pytest.raises(drone_control.MissionAckTimeout):
        harness.controller.hold_waypoint_until_stable(
            GPSCoord(41.0, -81.0, 10.0),
            CurrentLidar(),
            required_agl_m=10.0,
        )

    assert len(harness.vehicle._master.mav.waypoints) == 1


def test_takeoff_requires_home_newer_than_confirmed_armed_observation(monkeypatch):
    harness = Harness(monkeypatch)
    harness.clock.after(
        0.1,
        lambda: harness.publish(home=(410000000, -810000000, 245000)),
    )
    harness.clock.after(0.2, lambda: harness.publish(armed=True))
    harness.controller.arm(timeout=1.0)

    with pytest.raises(FlightOperationError, match="post-arm FC home"):
        harness.controller.takeoff(10.0, timeout=0.3)
    assert harness.vehicle.takeoffs == []


def test_climb_constructs_coordinates_from_actual_final_guard_snapshot(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(
        mode="GUIDED",
        armed=True,
        landed_state=2,
        location=(410000000, -810000000, 255000, 5000),
    )
    def move_during_final_transaction(operation):
        harness.publish(location=(410001000, -810000000, 255000, 5000))
        harness.clock.after(
            0.1,
            lambda: harness.publish(
                location=(410001000, -810000000, 260000, 10000)
            ),
        )
        return operation()

    harness.controller._dependency_output_transaction = move_during_final_transaction

    harness.controller.climb(10.0, timeout=1.0)

    fields = harness.vehicle._master.mav.waypoints[-1]
    assert fields[11] == 410001000


def test_waypoint_final_guard_ground_transition_prevents_transport(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=2)
    transactions = [0]

    def touch_down_at_waypoint_transaction(operation):
        transactions[0] += 1
        if transactions[0] == 2:
            harness.publish(landed_state=1)
        return operation()

    harness.controller._dependency_output_transaction = touch_down_at_waypoint_transaction

    with pytest.raises(FlightOperationError, match="airborne"):
        harness.controller.goto_waypoint(
            GPSCoord(41.0, -81.0, 10.0), timeout=0.3
        )
    assert harness.vehicle._master.mav.waypoints == []


def test_landing_confirmation_rejects_float_enum_lookalike(monkeypatch):
    harness = Harness(monkeypatch)
    harness.publish(mode="GUIDED", armed=True, landed_state=4.0)

    with pytest.raises(FlightOperationError, match="accepted LAND"):
        harness.controller.confirm_landing(timeout=0.3)
    assert harness.vehicle.mode_requests == []
