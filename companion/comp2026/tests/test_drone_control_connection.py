import math
import queue
import threading
import time as wall_time
from types import SimpleNamespace

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from drone.common_types import GPSCoord, MissionHome, RelPosComplete
from drone.control import drone_control
from dronekit.mavlink import MAVWriter
from drone.control.flight_state import (
    FailsafeEvidence,
    FlightState,
    RCModeBand,
    SourceIdentity,
)


def _round_trip_v2(message, *, source=(1, 1)):
    packets = queue.Queue()
    encoder = mavlink2.MAVLink(
        MAVWriter(packets), srcSystem=source[0], srcComponent=source[1]
    )
    encoder.send(message)
    return mavlink2.MAVLink(None).parse_char(packets.get_nowait())


def _command_ack(*, target=(1, 191), source=(1, 1), command=176, result=0):
    encoder = mavlink2.MAVLink(None, srcSystem=source[0], srcComponent=source[1])
    message = encoder.command_ack_encode(
        command,
        result,
        0,
        0,
        target[0],
        target[1],
    )
    return _round_trip_v2(message, source=source)


def _mission_ack(*, target=(1, 191), source=(1, 1), result=0, mission_type=0):
    encoder = mavlink2.MAVLink(None, srcSystem=source[0], srcComponent=source[1])
    message = encoder.mission_ack_encode(
        target[0], target[1], result, mission_type
    )
    return _round_trip_v2(message, source=source)


def test_command_ack_tracker_requires_v2_fc_source_and_companion_target():
    tracker = drone_control.CommandAckTracker(
        wire_protocol="2.0",
        source_system=1,
        source_component=1,
        target_system=1,
        target_component=191,
    )
    boundary = tracker.boundary()

    for message in (
        _command_ack(target=(200, 190)),
        _command_ack(source=(42, 1)),
        _command_ack(target=(0, 0)),
    ):
        assert tracker.observe(
            command=message.command,
            result=message.result,
            source_system=message.get_srcSystem(),
            source_component=message.get_srcComponent(),
            target_system=message.target_system,
            target_component=message.target_component,
        ) is False

    accepted = _command_ack()
    assert tracker.observe(
        command=accepted.command,
        result=accepted.result,
        source_system=accepted.get_srcSystem(),
        source_component=accepted.get_srcComponent(),
        target_system=accepted.target_system,
        target_component=accepted.target_component,
    ) is True
    assert tracker.result_after(command=176, boundary=boundary) == 0


@pytest.mark.parametrize(
    "command",
    [
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    ],
)
def test_wrong_qgc_target_cannot_satisfy_any_live_command_but_companion_can(command):
    tracker = drone_control.CommandAckTracker(
        wire_protocol="2.0",
        source_system=1,
        source_component=1,
        target_system=1,
        target_component=191,
    )
    boundary = tracker.boundary()
    wrong = _command_ack(command=command, target=(200, 190))
    assert tracker.observe(
        command=wrong.command,
        result=wrong.result,
        source_system=wrong.get_srcSystem(),
        source_component=wrong.get_srcComponent(),
        target_system=wrong.target_system,
        target_component=wrong.target_component,
    ) is False
    assert tracker.result_after(command=command, boundary=boundary) is None

    correct = _command_ack(command=command)
    assert tracker.observe(
        command=correct.command,
        result=correct.result,
        source_system=correct.get_srcSystem(),
        source_component=correct.get_srcComponent(),
        target_system=correct.target_system,
        target_component=correct.target_component,
    ) is True
    assert tracker.result_after(command=command, boundary=boundary) == 0


def test_mission_ack_tracker_filters_source_target_type_and_stale_packets():
    tracker = drone_control.MissionAckTracker(
        source_system=1,
        source_component=1,
        target_system=1,
        target_component=191,
        mission_type=mavlink2.MAV_MISSION_TYPE_MISSION,
    )
    stale = _mission_ack()
    assert tracker.observe_message(stale) is True
    boundary = tracker.boundary()

    for message in (
        _mission_ack(target=(200, 190)),
        _mission_ack(source=(42, 1)),
        _mission_ack(mission_type=mavlink2.MAV_MISSION_TYPE_FENCE),
    ):
        assert tracker.observe_message(message) is False

    assert tracker.result_after(boundary=boundary) is None
    rejected = _mission_ack(result=mavlink2.MAV_MISSION_DENIED)
    assert tracker.observe_message(rejected) is True
    assert tracker.result_after(boundary=boundary) == mavlink2.MAV_MISSION_DENIED


def _connection_identities():
    return {
        "source_identity": SourceIdentity(1, 191),
        "flight_controller_target": SourceIdentity(1, 1),
        "wire_protocol": "2.0",
    }


class _Vehicle:
    def __init__(self):
        self.encoded = None
        self.sent = None
        self.flushed = False
        self.message_factory = self
        self.listeners = {}
        self.mode = type("Mode", (), {"name": "GUIDED"})()
        self._handler = SimpleNamespace(target_system=1, target_component=1)
        self._master = SimpleNamespace(WIRE_PROTOCOL_VERSION="2.0")
        self.closed = False

    def close(self):
        self.closed = True

    def on_message(self, message_names):
        def register(callback):
            names = (
                message_names if isinstance(message_names, list) else [message_names]
            )
            for name in names:
                self.listeners[name] = callback
            return callback

        return register

    def landing_target_encode(self, *fields):
        self.encoded = fields
        return fields

    def send_mavlink(self, message):
        self.sent = message

    def flush(self):
        self.flushed = True


def test_drone_control_preserves_ready_wait_by_default(monkeypatch):
    captured = {}

    def connect(endpoint, **options):
        captured["endpoint"] = endpoint
        captured["options"] = options
        return _Vehicle()

    monkeypatch.setattr(drone_control, "connect", connect)

    drone_control.DroneControl(
        "tcp:ardupilot-sitl:5760", **_connection_identities()
    )

    assert captured == {
        "endpoint": "tcp:ardupilot-sitl:5760",
        "options": {
            "wait_ready": True,
            "heartbeat_timeout": 60,
            "timeout": 120,
            "source_system": 1,
            "source_component": 191,
        },
    }


def test_drone_control_can_defer_readiness_to_its_host_startup_budget(monkeypatch):
    captured = {}

    def connect(endpoint, **options):
        captured["endpoint"] = endpoint
        captured["options"] = options
        return _Vehicle()

    monkeypatch.setattr(drone_control, "connect", connect)

    drone_control.DroneControl(
        "tcp:ardupilot-sitl:5760",
        **_connection_identities(),
        wait_ready=False,
        heartbeat_timeout=120,
    )

    assert captured["options"]["wait_ready"] is False
    assert captured["options"]["heartbeat_timeout"] == 120


@pytest.mark.parametrize("source_identity", [SourceIdentity(0, 191), SourceIdentity(1, 0)])
def test_drone_control_rejects_zero_companion_source_identity_before_connect(
    monkeypatch, source_identity
):
    calls = []
    monkeypatch.setattr(
        drone_control, "connect", lambda *_args, **_kwargs: calls.append("connect")
    )

    with pytest.raises(ValueError, match="companion source identity must be nonzero"):
        drone_control.DroneControl(
            "offline",
            source_identity=source_identity,
            flight_controller_target=SourceIdentity(1, 1),
            wire_protocol="2.0",
        )

    assert calls == []


def test_drone_control_rejects_and_closes_mismatched_discovered_fc_system(monkeypatch):
    vehicle = _Vehicle()
    vehicle._handler.target_system = 42
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)

    with pytest.raises(ValueError, match="discovered flight controller system"):
        drone_control.DroneControl(
            "tcp:ardupilot-sitl:5760", **_connection_identities()
        )

    assert vehicle.closed is True


def test_dronekit_handler_without_target_component_accepts_pinned_system(monkeypatch):
    vehicle = _Vehicle()
    vehicle._handler = SimpleNamespace(target_system=1)
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)

    controller = drone_control.DroneControl(
        "offline", wait_ready=False, **_connection_identities()
    )

    assert controller.flight_controller_target == SourceIdentity(1, 1)


def test_flight_enabled_controller_rejects_actual_mavlink1_and_closes(monkeypatch):
    vehicle = _Vehicle()
    vehicle._master.WIRE_PROTOCOL_VERSION = "1.0"
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)

    with pytest.raises(ValueError, match="actual MAVLink 2"):
        drone_control.DroneControl(
            "offline",
            wait_ready=False,
            flight_state=_flight_state(),
            **_connection_identities(),
        )

    assert vehicle.closed is True


def test_actual_wire_validation_error_survives_vehicle_close_failure(monkeypatch):
    vehicle = _Vehicle()
    vehicle._master.WIRE_PROTOCOL_VERSION = "1.0"

    def close():
        raise RuntimeError("close failed")

    vehicle.close = close
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)

    with pytest.raises(ValueError, match="actual MAVLink 2"):
        drone_control.DroneControl(
            "offline",
            wait_ready=False,
            flight_state=_flight_state(),
            **_connection_identities(),
        )


def test_precision_landing_packet_uses_frd_and_reconstructs_literal_direction(
    monkeypatch,
):
    vehicle = _Vehicle()
    controller = _authorized_controller(monkeypatch, vehicle)
    direction = RelPosComplete(0.6, -0.4, 4.0)

    assert controller.land_send_landing_target(direction) == 0

    assert vehicle.encoded is not None
    _, _, frame, angle_x, angle_y, distance, size_x, size_y = vehicle.encoded
    assert frame == mavutil.mavlink.MAV_FRAME_BODY_FRD
    reconstructed = (
        -math.tan(angle_y) * direction.z,
        math.tan(angle_x) * direction.z,
        direction.z,
    )
    assert reconstructed == pytest.approx(
        (direction.x, direction.y, direction.z), abs=1e-12
    )
    assert distance == pytest.approx(math.sqrt(0.6**2 + (-0.4) ** 2 + 4.0**2))
    assert (size_x, size_y) == (0.0, 0.0)
    assert vehicle.sent == vehicle.encoded
    assert vehicle.flushed is True


def test_nonblocking_guided_hold_uses_integer_coordinates_and_pinned_home():
    packets = []
    controller = object.__new__(drone_control.DroneControl)
    controller.vehicle = SimpleNamespace(
        _master=SimpleNamespace(
            mav=SimpleNamespace(
                mission_item_int_send=lambda *fields: packets.append(fields)
            )
        )
    )
    controller.flight_controller_target = SourceIdentity(1, 1)
    controller._mission_home = MissionHome(41.0, -81.0, 100.0)
    current = SimpleNamespace(
        mode=SimpleNamespace(fresh=True, observation=SimpleNamespace(value="GUIDED")),
        armed=SimpleNamespace(fresh=True, observation=SimpleNamespace(value=True)),
        landed_state=SimpleNamespace(fresh=True, observation=SimpleNamespace(value=2)),
    )
    controller._send_guarded = lambda output, *, validate_snapshot: (
        validate_snapshot(current),
        output(),
    )[1]

    assert controller.send_guided_waypoint(GPSCoord(41.12345678, -81.87654321, 4.5)) == 0

    assert len(packets) == 1
    packet = packets[0]
    assert packet[0:2] == (1, 1)
    assert packet[3] == mavutil.mavlink.MAV_FRAME_GLOBAL_INT
    assert packet[4] == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
    assert packet[-3:] == (411234568, -818765432, 104.5)


def test_precision_landing_profile_requires_every_exact_runtime_parameter():
    assert drone_control.PRECISION_LANDING_PARAMETERS["LAND_SPEED"] == 50
    assert "LAND_SPD_MS" not in drone_control.PRECISION_LANDING_PARAMETERS
    controller = object.__new__(drone_control.DroneControl)
    controller.vehicle = SimpleNamespace(
        parameters=dict(drone_control.PRECISION_LANDING_PARAMETERS)
    )

    assert controller.require_precision_landing_profile() is True

    controller.vehicle.parameters["PLND_OPTIONS"] = 0
    assert controller.require_precision_landing_profile() is False


def _flight_state(clock=lambda: 10.0):
    bounds = {
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
    return FlightState(
        source_system=1,
        source_component=1,
        freshness_bounds=bounds,
        rc_channel=7,
        rc_mode_mapping=(
            RCModeBand("manual-low", 900, 1200, "STABILIZE"),
            RCModeBand("companion", 1400, 1600, "GUIDED"),
            RCModeBand("manual-high", 1800, 2100, "LOITER"),
        ),
        clock=clock,
    )


def _authorized_controller(monkeypatch, vehicle):
    state = _flight_state()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: 10.0)
    controller = drone_control.DroneControl(
        "offline",
        **_connection_identities(),
        wait_ready=False,
        flight_state=state,
        permission_guard=lambda: None,
    )
    assert state.update_many(
        {
            "heartbeat": "alive",
            "mode": "LAND",
            "landed_state": 1,
            "armed": False,
            "failsafe": FailsafeEvidence(active=False, reason="test clear"),
        },
        received_at=10.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )
    assert state.observe_rc_input(
        channel=7,
        pwm=1500,
        signal_healthy=True,
        received_at=10.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )
    assert state.acquire_initial_companion_authority(now=10.0)
    controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=lambda operation, enqueue_check: (
            enqueue_check(), operation()
        )[1],
    )
    return controller


class _Message:
    def __init__(self, system=1, component=1, **values):
        self._system = system
        self._component = component
        self.__dict__.update(values)

    def get_srcSystem(self):
        return self._system

    def get_srcComponent(self):
        return self._component


def test_drone_control_routes_source_identified_messages_to_flight_state(monkeypatch):
    vehicle = _Vehicle()
    state = _flight_state()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: 10.0)
    drone_control.DroneControl(
        "offline", wait_ready=False, flight_state=state, **_connection_identities()
    )

    vehicle.listeners["ATTITUDE"](
        vehicle,
        "ATTITUDE",
        _Message(roll=0.1, pitch=0.2, yaw=0.3, rollspeed=0.4, pitchspeed=0.5, yawspeed=0.6),
    )
    vehicle.listeners["ATTITUDE"](
        vehicle,
        "ATTITUDE",
        _Message(system=44, roll=9.0, pitch=9.0, yaw=9.0),
    )

    attitude = state.snapshot(now=10.0).attitude
    assert attitude is not None
    assert attitude.observation.value == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    assert attitude.source.system_id == 1


def test_command_ack_listener_requires_decoded_companion_target(
    monkeypatch,
):
    vehicle = _Vehicle()
    state = _flight_state()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    controller = drone_control.DroneControl(
        "offline", wait_ready=False, flight_state=state, **_connection_identities()
    )
    tracker = controller._command_ack_tracker
    boundary = tracker.boundary()

    vehicle.listeners["COMMAND_ACK"](
        vehicle,
        "COMMAND_ACK",
        _Message(system=44, command=mavutil.mavlink.MAV_CMD_DO_SET_MODE, result=0),
    )
    vehicle.listeners["COMMAND_ACK"](
        vehicle,
        "COMMAND_ACK",
        _Message(command=True, result=0),
    )

    assert tracker.boundary() == boundary

    vehicle.listeners["COMMAND_ACK"](
        vehicle,
        "COMMAND_ACK",
        _Message(
            command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            result=0,
            target_system=1,
            target_component=191,
        ),
    )

    assert tracker.result_after(
        command=mavutil.mavlink.MAV_CMD_DO_SET_MODE, boundary=boundary
    ) == mavutil.mavlink.MAV_RESULT_ACCEPTED


@pytest.mark.parametrize("wire_protocol", [None, "1.0"])
def test_flight_enabled_controller_requires_explicit_mavlink2_before_connect(
    monkeypatch, wire_protocol
):
    calls = []
    monkeypatch.setattr(
        drone_control, "connect", lambda *_args, **_kwargs: calls.append("connect")
    )
    options = _connection_identities()
    options["wire_protocol"] = wire_protocol

    with pytest.raises(ValueError, match="MAVLink 2"):
        drone_control.DroneControl(
            "offline", wait_ready=False, flight_state=_flight_state(), **options
        )

    assert calls == []


def test_startup_telemetry_ack_timeout_does_not_use_paused_mission_clock(monkeypatch):
    controller = object.__new__(drone_control.DroneControl)
    controller.flight_controller_target = SourceIdentity(1, 1)
    controller._command_ack_tracker = drone_control.CommandAckTracker(
        wire_protocol="2.0",
        source_system=1,
        source_component=1,
        target_system=1,
        target_component=191,
    )
    controller._telemetry_transaction_lock = threading.Lock()
    controller._startup_telemetry_open = True
    controller._transport_output_transaction = (
        lambda operation, enqueue_check: enqueue_check(operation)
    )
    controller.vehicle = SimpleNamespace(
        message_factory=SimpleNamespace(command_long_encode=lambda *_fields: object()),
        send_mavlink=lambda _message: None,
    )
    paused_sleep_entered = threading.Event()
    release_paused_sleep = threading.Event()

    def paused_sleep(_seconds):
        paused_sleep_entered.set()
        release_paused_sleep.wait()

    monkeypatch.setattr(drone_control.time, "monotonic", lambda: 0.0)
    monkeypatch.setattr(drone_control.time, "sleep", paused_sleep)
    outcome = []

    def invoke():
        try:
            controller._run_telemetry_command(
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
                100_000,
                timeout_s=0.03,
                poll_interval_s=0.01,
            )
        except BaseException as error:
            outcome.append(error)

    worker = threading.Thread(target=invoke, daemon=True)
    started = wall_time.monotonic()
    worker.start()
    worker.join(0.2)
    elapsed = wall_time.monotonic() - started
    release_paused_sleep.set()
    worker.join(0.2)

    assert not worker.is_alive()
    assert elapsed < 0.2
    assert len(outcome) == 1 and isinstance(outcome[0], TimeoutError)
    assert not paused_sleep_entered.is_set()


def test_startup_telemetry_shares_the_flight_command_ack_transaction_lock():
    controller = object.__new__(drone_control.DroneControl)
    controller.flight_controller_target = SourceIdentity(1, 1)
    controller._command_ack_tracker = drone_control.CommandAckTracker(
        wire_protocol="2.0",
        source_system=1,
        source_component=1,
        target_system=1,
        target_component=191,
    )
    controller._telemetry_transaction_lock = threading.Lock()
    controller._command_ack_transaction_lock = threading.Lock()
    controller._startup_telemetry_open = True
    controller._transport_output_transaction = (
        lambda operation, enqueue_check: enqueue_check(operation)
    )
    sent = threading.Event()
    controller.vehicle = SimpleNamespace(
        message_factory=SimpleNamespace(command_long_encode=lambda *_fields: object()),
        send_mavlink=lambda _message: sent.set(),
    )
    outcome = []

    def invoke():
        try:
            controller._run_telemetry_command(
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
                100_000,
                timeout_s=0.2,
                poll_interval_s=0.01,
            )
        except BaseException as error:
            outcome.append(error)

    controller._command_ack_transaction_lock.acquire()
    worker = threading.Thread(target=invoke, daemon=True)
    worker.start()
    assert not sent.wait(0.03)
    controller._command_ack_transaction_lock.release()
    assert sent.wait(0.2)
    controller._command_ack_tracker.observe(
        command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
        source_system=1,
        source_component=1,
        target_system=1,
        target_component=191,
    )
    worker.join(0.3)

    assert not worker.is_alive()
    assert outcome == []


def test_startup_ack_packed_before_actual_writer_enqueue_is_stale(monkeypatch):
    controller = object.__new__(drone_control.DroneControl)
    controller.flight_controller_target = SourceIdentity(1, 1)
    controller._command_ack_tracker = drone_control.CommandAckTracker(
        wire_protocol="2.0",
        source_system=1,
        source_component=1,
        target_system=1,
        target_component=191,
    )
    controller._telemetry_transaction_lock = threading.Lock()
    controller._startup_telemetry_open = True
    outbound = queue.Queue()
    guarded_writer = drone_control._OutputGuardedWriter(MAVWriter(outbound))
    encoder = mavlink2.MAVLink(guarded_writer, srcSystem=1, srcComponent=191)
    controller._transport_output_transaction = guarded_writer.transaction
    controller.vehicle = SimpleNamespace(
        message_factory=encoder,
        send_mavlink=encoder.send,
    )
    message_type = mavlink2.MAVLink_command_long_message
    original_pack = message_type.pack

    def pack_with_old_ack(message, mav, *args, **kwargs):
        packet = original_pack(message, mav, *args, **kwargs)
        controller._command_ack_tracker.observe(
            command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
            source_system=1,
            source_component=1,
            target_system=1,
            target_component=191,
        )
        return packet

    monkeypatch.setattr(message_type, "pack", pack_with_old_ack)

    with pytest.raises(TimeoutError, match="ACK timed out"):
        controller.set_telemetry_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT,
            100_000,
            timeout_s=0.03,
            poll_interval_s=0.01,
        )

    assert outbound.qsize() == 1


def test_rc_channels_callback_does_not_infer_signal_health(monkeypatch):
    vehicle = _Vehicle()
    state = _flight_state()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: 10.0)
    drone_control.DroneControl(
        "offline", wait_ready=False, flight_state=state, **_connection_identities()
    )

    vehicle.listeners["RC_CHANNELS"](
        vehicle,
        "RC_CHANNELS",
        _Message(chan7_raw=1500, rssi=255),
    )

    rc_input = state.snapshot(now=10.0).rc_input
    assert rc_input is not None
    assert rc_input.observation.value.healthy is False


def test_wrong_source_landed_state_does_not_change_filtered_controller_state(monkeypatch):
    vehicle = _Vehicle()
    state = _flight_state()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: 10.0)
    controller = drone_control.DroneControl(
        "offline", wait_ready=False, flight_state=state, **_connection_identities()
    )

    vehicle.listeners["EXTENDED_SYS_STATE"](
        vehicle,
        "EXTENDED_SYS_STATE",
        _Message(system=44, landed_state=1),
    )

    assert controller.is_landed() is False
    assert state.snapshot(now=10.0).landed_state is None


@pytest.mark.parametrize("landed_state", [0, 1, 2, 3, 4])
def test_landed_state_callback_preserves_exact_mav_enum(monkeypatch, landed_state):
    vehicle = _Vehicle()
    state = _flight_state()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: 10.0)
    drone_control.DroneControl(
        "offline", wait_ready=False, flight_state=state, **_connection_identities()
    )

    vehicle.listeners["EXTENDED_SYS_STATE"](
        vehicle, "EXTENDED_SYS_STATE", _Message(landed_state=landed_state)
    )

    observed = state.snapshot(now=10.0).landed_state
    assert observed is not None
    assert observed.observation.value == landed_state


@pytest.mark.parametrize("landed_state", [None, True, 1.0, -1, 5, "ground", []])
def test_trusted_invalid_landed_state_clears_old_ground_evidence(
    monkeypatch, landed_state
):
    vehicle, state = _controller_with_heartbeat_decoder(monkeypatch)

    vehicle.listeners["EXTENDED_SYS_STATE"](
        vehicle, "EXTENDED_SYS_STATE", _Message(landed_state=landed_state)
    )

    snapshot = state.snapshot(now=10.0)
    assert snapshot.landed_state is None
    assert snapshot.authority.name == "UNKNOWN"
    assert snapshot.commands_suspended


@pytest.mark.parametrize(
    ("decoded_health", "expected_health"),
    [(True, True), (False, False), (None, False)],
)
def test_configured_rc_health_decoder_supplies_explicit_evidence(
    monkeypatch, decoded_health, expected_health
):
    vehicle = _Vehicle()
    state = _flight_state()
    decoder_inputs = []

    def decode_health(message, channel, pwm):
        decoder_inputs.append((message, channel, pwm))
        return decoded_health

    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: 10.0)
    drone_control.DroneControl(
        "offline",
        **_connection_identities(),
        wait_ready=False,
        flight_state=state,
        rc_health_decoder=decode_health,
    )
    message = _Message(chan7_raw=1500, rssi=255)

    vehicle.listeners["RC_CHANNELS"](vehicle, "RC_CHANNELS", message)

    rc_input = state.snapshot(now=10.0).rc_input
    assert rc_input is not None
    assert rc_input.observation.value.healthy is expected_health
    assert decoder_inputs == [(message, 7, 1500)]


def test_stale_configured_rc_health_evidence_cannot_acquire_authority(monkeypatch):
    now = [10.0]
    vehicle = _Vehicle()
    state = _flight_state(clock=lambda: now[0])
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: now[0])
    drone_control.DroneControl(
        "offline",
        **_connection_identities(),
        wait_ready=False,
        flight_state=state,
        rc_health_decoder=lambda _message, _channel, _pwm: True,
    )
    state.update_many(
        {"heartbeat": "alive", "landed_state": 1, "armed": False},
        received_at=10.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )
    vehicle.listeners["RC_CHANNELS"](
        vehicle, "RC_CHANNELS", _Message(chan7_raw=1500)
    )

    now[0] = 11.0

    assert not state.acquire_initial_companion_authority(now=11.0)


def test_configured_failsafe_decoder_supplies_source_filtered_evidence(monkeypatch):
    vehicle = _Vehicle()
    state = _flight_state()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: 10.0)
    drone_control.DroneControl(
        "offline",
        **_connection_identities(),
        wait_ready=False,
        flight_state=state,
        failsafe_decoders={
            "SYS_STATUS": lambda message: FailsafeEvidence(
                active=message.sensor_failed,
                reason="configured sensor failure",
            )
        },
    )

    vehicle.listeners["SYS_STATUS"](
        vehicle,
        "SYS_STATUS",
        _Message(system=44, sensor_failed=True),
    )
    assert state.snapshot(now=10.0).failsafe is None

    vehicle.listeners["SYS_STATUS"](
        vehicle,
        "SYS_STATUS",
        _Message(sensor_failed=True),
    )

    failsafe = state.snapshot(now=10.0).failsafe
    assert failsafe is not None
    assert failsafe.observation.value == (True, "configured sensor failure")
    assert state.authority.name == "FC_FAILSAFE"


def test_failsafe_decoder_can_report_no_available_evidence(monkeypatch):
    vehicle = _Vehicle()
    state = _flight_state()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    drone_control.DroneControl(
        "offline",
        **_connection_identities(),
        wait_ready=False,
        flight_state=state,
        failsafe_decoders={"SYS_STATUS": lambda _message: None},
    )

    vehicle.listeners["SYS_STATUS"](
        vehicle, "SYS_STATUS", _Message(sensor_failed=False)
    )

    assert state.snapshot(now=10.0).failsafe is None


def _heartbeat(mode="GUIDED", *, system=1):
    return _Message(
        system=system,
        type=2,
        autopilot=3,
        base_mode=0,
        custom_mode=4,
        system_status=3,
        decoded_mode=mode,
    )


def _controller_with_heartbeat_decoder(monkeypatch):
    vehicle = _Vehicle()
    state = _flight_state()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: 10.0)
    drone_control.DroneControl(
        "offline",
        **_connection_identities(),
        wait_ready=False,
        flight_state=state,
        heartbeat_mode_decoder=lambda message: message.decoded_mode,
        rc_health_decoder=lambda _message, _channel, _pwm: True,
    )
    vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", _heartbeat())
    vehicle.listeners["EXTENDED_SYS_STATE"](
        vehicle, "EXTENDED_SYS_STATE", _Message(landed_state=1)
    )
    vehicle.listeners["RC_CHANNELS"](
        vehicle, "RC_CHANNELS", _Message(chan7_raw=1500)
    )
    assert state.acquire_initial_companion_authority(now=10.0)
    return vehicle, state


def test_repeated_unchanged_heartbeat_mode_preserves_companion_authority(monkeypatch):
    vehicle, state = _controller_with_heartbeat_decoder(monkeypatch)

    vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", _heartbeat())

    assert state.authority.name == "COMPANION"
    assert state.ordinary_commands_permitted()


def test_old_heartbeat_mode_does_not_consume_pending_transition(monkeypatch):
    vehicle, state = _controller_with_heartbeat_decoder(monkeypatch)
    state.register_expected_mode("LOITER")

    vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", _heartbeat("GUIDED"))

    snapshot = state.snapshot(now=10.0)
    assert snapshot.expected_mode == "LOITER"
    assert snapshot.authority.name == "COMPANION"


def test_foreign_heartbeat_cannot_supply_fresh_mode_evidence(monkeypatch):
    vehicle, state = _controller_with_heartbeat_decoder(monkeypatch)

    vehicle.listeners["HEARTBEAT"](
        vehicle, "HEARTBEAT", _heartbeat("LOITER", system=44)
    )

    snapshot = state.snapshot(now=10.0)
    assert snapshot.mode is not None
    assert snapshot.mode.observation.value == "GUIDED"
    assert snapshot.authority.name == "COMPANION"


def _active_controller_with_decoders(monkeypatch, **decoder_options):
    vehicle = _Vehicle()
    state = _flight_state()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_options: vehicle)
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: 10.0)
    drone_control.DroneControl(
        "offline",
        wait_ready=False,
        flight_state=state,
        **_connection_identities(),
        **decoder_options,
    )
    state.update_many(
        {
            "heartbeat": "alive",
            "mode": "GUIDED",
            "landed_state": 1,
            "armed": False,
        },
        received_at=10.0,
        sequence=0,
        source_system=1,
        source_component=1,
    )
    state.observe_rc_input(
        channel=7,
        pwm=1500,
        signal_healthy=True,
        received_at=10.0,
        sequence=0,
        source_system=1,
        source_component=1,
    )
    assert state.acquire_initial_companion_authority(now=10.0)
    return vehicle, state


@pytest.mark.parametrize("failure", ["none", "invalid", "exception"])
def test_heartbeat_decoder_failure_revokes_stale_mode_authority(monkeypatch, failure):
    def decode(message):
        if failure == "exception":
            raise ValueError("unsupported heartbeat")
        if failure == "invalid":
            return object()
        return None

    vehicle, state = _active_controller_with_decoders(
        monkeypatch, heartbeat_mode_decoder=decode
    )

    vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", _heartbeat())

    snapshot = state.snapshot(now=10.0)
    assert snapshot.mode is None
    assert snapshot.authority.name == "UNKNOWN"
    assert snapshot.commands_suspended


@pytest.mark.parametrize("failure", ["none", "exception"])
def test_rc_health_decoder_failure_revokes_stale_rc_authority(monkeypatch, failure):
    def decode(_message, _channel, _pwm):
        if failure == "exception":
            raise ValueError("unsupported RC health")
        return None

    vehicle, state = _active_controller_with_decoders(
        monkeypatch, rc_health_decoder=decode
    )

    vehicle.listeners["RC_CHANNELS"](
        vehicle, "RC_CHANNELS", _Message(chan7_raw=1500)
    )

    snapshot = state.snapshot(now=10.0)
    assert snapshot.authority.name == "UNKNOWN"
    assert snapshot.commands_suspended
    assert snapshot.rc_input is not None
    assert snapshot.rc_input.observation.value.healthy is False


@pytest.mark.parametrize("failure", ["none", "exception"])
def test_failsafe_decoder_failure_revokes_stale_authority(monkeypatch, failure):
    def decode(_message):
        if failure == "exception":
            raise ValueError("unsupported failsafe")
        return None

    vehicle, state = _active_controller_with_decoders(
        monkeypatch, failsafe_decoders={"SYS_STATUS": decode}
    )

    vehicle.listeners["SYS_STATUS"](
        vehicle, "SYS_STATUS", _Message(sensor_failed=False)
    )

    snapshot = state.snapshot(now=10.0)
    assert snapshot.failsafe is None
    assert snapshot.authority.name == "UNKNOWN"
    assert snapshot.commands_suspended


@pytest.mark.parametrize("bad_base_mode", [None, "armed", True])
def test_trusted_malformed_heartbeat_clears_old_authorizing_evidence(
    monkeypatch, bad_base_mode
):
    vehicle, state = _active_controller_with_decoders(
        monkeypatch, heartbeat_mode_decoder=lambda message: message.decoded_mode
    )
    message = _heartbeat()
    message.base_mode = bad_base_mode

    vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", message)

    snapshot = state.snapshot(now=10.0)
    assert snapshot.heartbeat is None
    assert snapshot.armed is None
    assert snapshot.mode is None
    assert snapshot.authority.name == "UNKNOWN"
    assert snapshot.commands_suspended


@pytest.mark.parametrize("bad_pwm", [None, "1500", True])
def test_trusted_malformed_rc_pwm_clears_old_edge_evidence(monkeypatch, bad_pwm):
    vehicle, state = _active_controller_with_decoders(
        monkeypatch, rc_health_decoder=lambda _message, _channel, _pwm: True
    )
    vehicle.listeners["RC_CHANNELS"](
        vehicle,
        "RC_CHANNELS",
        _Message(chan7_raw=2000),
    )
    message = _Message()
    if bad_pwm is not None:
        message.chan7_raw = bad_pwm

    vehicle.listeners["RC_CHANNELS"](vehicle, "RC_CHANNELS", message)
    assert state.observe_mode(
        "LOITER",
        received_at=10.0,
        sequence=2,
        source_system=1,
        source_component=1,
    )

    snapshot = state.snapshot(now=10.0)
    assert snapshot.rc_input is None
    assert snapshot.authority.name == "UNKNOWN"
    assert snapshot.commands_suspended


@pytest.mark.parametrize("decoder_failure", ["none", "exception"])
def test_unusable_trusted_heartbeat_revokes_when_other_payload_is_rejected(
    monkeypatch, decoder_failure
):
    def decode(_message):
        if decoder_failure == "exception":
            raise ValueError("mode unavailable")
        return None

    vehicle, state = _active_controller_with_decoders(
        monkeypatch, heartbeat_mode_decoder=decode
    )
    permission = state.command_permission()
    assert permission is not None
    message = _heartbeat()
    message.custom_mode = None

    vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", message)

    snapshot = state.snapshot(now=10.0)
    assert snapshot.heartbeat is None
    assert snapshot.armed is None
    assert snapshot.mode is None
    assert snapshot.authority.name == "UNKNOWN"
    assert snapshot.commands_suspended
    assert not state.permission_is_current(permission)


def test_foreign_rejected_heartbeat_cannot_invalidate_authorizing_evidence(monkeypatch):
    decoder_calls = []

    def decode(message):
        decoder_calls.append(message)
        return None

    vehicle, state = _active_controller_with_decoders(
        monkeypatch, heartbeat_mode_decoder=decode
    )
    permission = state.command_permission()
    assert permission is not None
    message = _heartbeat(system=44)
    message.custom_mode = None

    vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", message)

    snapshot = state.snapshot(now=10.0)
    assert snapshot.mode is not None
    assert snapshot.mode.observation.value == "GUIDED"
    assert snapshot.authority.name == "COMPANION"
    assert state.permission_is_current(permission)
    assert decoder_calls == []
