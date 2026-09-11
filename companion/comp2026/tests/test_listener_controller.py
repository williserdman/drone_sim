from types import SimpleNamespace
import queue

import pytest
from pymavlink import mavutil

from drone.control import drone_control
from dronekit.mavlink import MAVWriter
from drone.common_types import GPSCoord, MissionHome, RelPosComplete
from drone.control.flight_state import (
    FailsafeEvidence,
    FlightState,
    RCModeBand,
    SourceIdentity,
)
from drone.control.mission_supervisor import AuthorityLost


class InertVehicle:
    def __init__(self):
        self.listeners = {}
        self.sent = []
        self.message_factory = self
        self._handler = SimpleNamespace(target_system=1, target_component=1)
        self._master = SimpleNamespace(WIRE_PROTOCOL_VERSION="2.0")
        self.closed = False

    def close(self):
        self.closed = True

    def on_message(self, names):
        def register(callback):
            for name in names if isinstance(names, list) else [names]:
                self.listeners[name] = callback
            return callback

        return register

    def command_long_encode(self, *fields):
        return SimpleNamespace(fields=fields)

    def send_mavlink(self, message):
        self.sent.append(message)


def state(source=(1, 1)):
    return FlightState(
        source_system=source[0],
        source_component=source[1],
        freshness_bounds={name: 1.0 for name in (
            "heartbeat", "mode", "location", "velocity", "attitude",
            "landed_state", "armed", "home", "rc_input", "range", "failsafe",
        )},
        rc_channel=7,
        rc_mode_mapping=(
            RCModeBand("companion", 1400, 1600, "GUIDED"),
            RCModeBand("pilot", 1800, 2100, "LOITER"),
        ),
        clock=lambda: 10.0,
    )


def construct(monkeypatch, vehicle, **overrides):
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_kwargs: vehicle)
    options = {
        "connection_port": "offline",
        "wait_ready": False,
        "source_identity": SourceIdentity(1, 191),
        "flight_controller_target": SourceIdentity(1, 1),
        "wire_protocol": "2.0",
        "flight_state": state(),
    }
    options.update(overrides)
    vehicle._handler.target_system = options["flight_controller_target"].system_id
    controller = drone_control.DroneControl(**options)
    controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=lambda operation, enqueue_check: (
            enqueue_check(), operation()
        )[1],
    )
    return controller


def test_explicit_connection_identities_are_used_and_legacy_qgc_callback_is_absent(
    monkeypatch,
):
    vehicle = InertVehicle()
    captured = {}

    def connect(endpoint, **options):
        captured.update(endpoint=endpoint, options=options)
        vehicle._handler.target_system = 9
        return vehicle

    monkeypatch.setattr(drone_control, "connect", connect)

    drone_control.DroneControl(
        "offline",
        wait_ready=False,
        source_identity=SourceIdentity(42, 77),
        flight_controller_target=SourceIdentity(9, 1),
        wire_protocol="2.0",
        flight_state=state((9, 1)),
    )

    assert captured["options"]["source_system"] == 42
    assert captured["options"]["source_component"] == 77
    assert "COMMAND_LONG" not in vehicle.listeners
    assert "COMMAND_INT" not in vehicle.listeners


def test_missing_connection_identity_fails_before_connect(monkeypatch):
    calls = []
    monkeypatch.setattr(
        drone_control,
        "connect",
        lambda *_args, **_kwargs: calls.append("connect"),
    )

    with pytest.raises((TypeError, ValueError), match="source_identity"):
        drone_control.DroneControl(
            "offline",
            wait_ready=False,
            flight_controller_target=SourceIdentity(1, 1),
        )

    assert calls == []


def test_set_telemetry_interval_is_targeted_and_source_acknowledged(monkeypatch):
    vehicle = InertVehicle()
    controller = construct(monkeypatch, vehicle)

    def acknowledge(message):
        vehicle.sent.append(message)
        vehicle.listeners["COMMAND_ACK"](
            vehicle,
            "COMMAND_ACK",
            SimpleNamespace(
                command=mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                target_system=1,
                target_component=191,
                get_srcSystem=lambda: 1,
                get_srcComponent=lambda: 1,
            ),
        )

    vehicle.send_mavlink = acknowledge

    assert controller.set_telemetry_interval(
        33,
        100_000,
        timeout_s=1.0,
        poll_interval_s=0.01,
    ) is None

    fields = vehicle.sent[0].fields
    assert fields[:4] == (
        1,
        1,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
    )
    assert fields[4:] == (33.0, 100_000.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_startup_telemetry_request_times_out_without_matching_ack(monkeypatch):
    vehicle = InertVehicle()
    now = [0.0]
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(drone_control.time, "sleep", lambda seconds: now.__setitem__(0, now[0] + seconds))
    controller = construct(monkeypatch, vehicle)

    with pytest.raises(TimeoutError, match="telemetry command ACK"):
        controller.set_telemetry_interval(
            33,
            100_000,
            timeout_s=0.1,
            poll_interval_s=0.05,
        )


def test_startup_autopilot_version_request_is_narrow_and_fc_targeted(monkeypatch):
    vehicle = InertVehicle()
    controller = construct(monkeypatch, vehicle)

    def acknowledge(message):
        vehicle.sent.append(message)
        vehicle.listeners["COMMAND_ACK"](
            vehicle,
            "COMMAND_ACK",
            SimpleNamespace(
                command=mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                target_system=1,
                target_component=191,
                get_srcSystem=lambda: 1,
                get_srcComponent=lambda: 1,
            ),
        )

    vehicle.send_mavlink = acknowledge

    controller.request_autopilot_version(timeout_s=1.0, poll_interval_s=0.01)

    fields = vehicle.sent[0].fields
    assert fields[:4] == (
        1,
        1,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
    )
    assert fields[4:] == (
        float(mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    controller.close_startup_telemetry()
    with pytest.raises(Exception, match="closed"):
        controller.request_autopilot_version(timeout_s=1.0, poll_interval_s=0.01)


def test_home_request_requires_current_normal_permission(monkeypatch):
    vehicle = InertVehicle()
    controller = construct(
        monkeypatch,
        vehicle,
        permission_guard=lambda: (_ for _ in ()).throw(RuntimeError("no authority")),
    )

    with pytest.raises(RuntimeError, match="no authority"):
        controller.request_home_position(timeout_s=1.0, poll_interval_s=0.01)

    assert vehicle.sent == []


def test_home_request_rechecks_authority_after_serializing(monkeypatch):
    vehicle = InertVehicle()
    controller = construct(monkeypatch, vehicle)
    checks = []

    def permission():
        checks.append("check")
        if len(checks) == 2:
            raise RuntimeError("revoked before serialized send")

    controller.check_permission = permission

    with pytest.raises(RuntimeError, match="serialized send"):
        controller.request_home_position(timeout_s=1.0, poll_interval_s=0.01)

    assert vehicle.sent == []
    assert checks == ["check", "check"]


def test_home_request_rechecks_authority_while_waiting_for_ack(monkeypatch):
    vehicle = InertVehicle()
    controller = construct(monkeypatch, vehicle)
    checks = []

    def permission():
        checks.append("check")
        if len(checks) == 4:
            raise RuntimeError("revoked during ACK wait")

    controller.check_permission = permission
    controller._command_permission = lambda: (permission(), object())[1]
    controller._check_command_boundary = lambda _token: permission()
    controller.flight_state.permission_is_current = lambda _permission: True
    controller.flight_state.execute_command_output = lambda _permission, **values: (
        controller.flight_state.snapshot(),
        values["output"](),
    )
    controller._require_vital_output_snapshot = lambda _snapshot: None

    with pytest.raises(RuntimeError, match="ACK wait"):
        controller.request_home_position(timeout_s=1.0, poll_interval_s=0.01)

    assert len(vehicle.sent) == 1
    assert checks == ["check", "check", "check", "check"]


def test_internal_telemetry_helper_rejects_arbitrary_command_after_startup(monkeypatch):
    vehicle = InertVehicle()
    controller = construct(monkeypatch, vehicle)
    controller.close_startup_telemetry()

    with pytest.raises((ValueError, RuntimeError), match="allowlisted|closed"):
        controller._run_telemetry_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            1,
            0,
            timeout_s=1.0,
            poll_interval_s=0.01,
        )

    assert vehicle.sent == []


def test_home_request_does_not_send_when_encoding_revokes_authority(monkeypatch):
    vehicle = InertVehicle()
    controller = construct(monkeypatch, vehicle)
    revoked = [False]
    original_encode = vehicle.command_long_encode

    def encode(*fields):
        revoked[0] = True
        return original_encode(*fields)

    vehicle.command_long_encode = encode
    controller.check_permission = lambda: (
        (_ for _ in ()).throw(RuntimeError("revoked while encoding"))
        if revoked[0]
        else None
    )

    with pytest.raises(RuntimeError, match="while encoding"):
        controller.request_home_position(timeout_s=1.0, poll_interval_s=0.01)

    assert vehicle.sent == []


def test_heartbeat_failsafe_is_published_before_same_packet_heartbeat(monkeypatch):
    vehicle = InertVehicle()
    flight_state = state()
    order = []
    original_failsafe = flight_state.observe_failsafe
    original_heartbeat = flight_state.observe_heartbeat

    def observe_failsafe(*args, **kwargs):
        order.append("failsafe")
        return original_failsafe(*args, **kwargs)

    def observe_heartbeat(*args, **kwargs):
        order.append("heartbeat")
        return original_heartbeat(*args, **kwargs)

    flight_state.observe_failsafe = observe_failsafe
    flight_state.observe_heartbeat = observe_heartbeat
    controller = construct(
        monkeypatch,
        vehicle,
        flight_state=flight_state,
        heartbeat_mode_decoder=lambda _message: "GUIDED",
        failsafe_decoders={
            "HEARTBEAT": lambda _message: FailsafeEvidence(
                active=True,
                reason="FC heartbeat reports failsafe",
            )
        },
    )
    del controller
    message = SimpleNamespace(
        type=2,
        autopilot=3,
        base_mode=128,
        custom_mode=4,
        system_status=5,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
    )

    vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", message)

    assert order == ["failsafe", "heartbeat"]


def test_sys_status_uses_dedicated_observer_without_invalidating_failsafe(monkeypatch):
    vehicle = InertVehicle()
    flight_state = state()
    observed = []
    invalidated = []
    original = flight_state.invalidate_observation

    def record_invalidation(field, **metadata):
        invalidated.append(field)
        return original(field, **metadata)

    flight_state.invalidate_observation = record_invalidation
    construct(
        monkeypatch,
        vehicle,
        flight_state=flight_state,
        sys_status_observer=lambda message: observed.append(message),
    )
    message = SimpleNamespace(get_srcSystem=lambda: 1, get_srcComponent=lambda: 1)

    vehicle.listeners["SYS_STATUS"](vehicle, "SYS_STATUS", message)

    assert observed == [message]
    assert "failsafe" not in invalidated


@pytest.mark.parametrize(
    ("message_name", "valid_fields", "invalid_fields", "affected"),
    (
        (
            "ATTITUDE",
            dict(roll=0.0, pitch=0.0, yaw=0.0, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0),
            dict(roll=float("nan"), pitch=0.0, yaw=0.0, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0),
            ("attitude",),
        ),
        (
            "GLOBAL_POSITION_INT",
            dict(lat=1, lon=2, alt=3, relative_alt=4, vx=0, vy=0, vz=0),
            dict(lat=None, lon=2, alt=3, relative_alt=4, vx=0, vy=0, vz=0),
            ("location", "velocity"),
        ),
    ),
)
def test_trusted_invalid_navigation_packet_removes_evidence_and_marks_continuity(
    monkeypatch, message_name, valid_fields, invalid_fields, affected
):
    """Break caught: newer malformed telemetry must not leave old evidence usable."""
    vehicle = InertVehicle()
    flight_state = state()
    construct(monkeypatch, vehicle, flight_state=flight_state)

    def deliver(fields):
        message = SimpleNamespace(
            **fields,
            get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 1,
        )
        vehicle.listeners[message_name](vehicle, message_name, message)

    deliver(valid_fields)
    before = flight_state.snapshot()
    deliver(invalid_fields)
    invalid = flight_state.snapshot()
    deliver(valid_fields)
    recovered = flight_state.snapshot()

    for name in affected:
        assert getattr(before, name) is not None
        assert getattr(invalid, name) is None
        assert getattr(recovered, name).invalidation_generation == 1


def test_configured_arm_requests_a_new_home_after_confirmation():
    controller = object.__new__(drone_control.DroneControl)
    field = lambda value, sequence: SimpleNamespace(
        fresh=True,
        observation=SimpleNamespace(value=value, sequence=sequence),
    )
    before = SimpleNamespace(
        heartbeat=field(object(), 1),
        mode=field("GUIDED", 1),
        armed=field(False, 1),
        landed_state=field(1, 1),
        location=field((1, 2, 3, 4), 1),
    )
    confirmed = SimpleNamespace(armed=field(True, 2))
    snapshots = iter((before, confirmed))
    controller.flight_state = SimpleNamespace(snapshot=lambda: next(snapshots))
    controller.flight_controller_target = SourceIdentity(1, 1)
    controller.vehicle = SimpleNamespace(
        is_armable=True,
        message_factory=SimpleNamespace(command_long_encode=lambda *_fields: object()),
    )
    controller.home_request_timeout_s = 2.0
    controller.telemetry_poll_interval_s = 0.1
    controller.check_permission = lambda: None
    controller._require_fresh = lambda *_args: None
    controller._mode_name = lambda _snapshot: "GUIDED"
    controller._deadline = lambda timeout: timeout
    controller._send_acknowledged = lambda *_args, **_kwargs: before
    controller._wait_until = lambda *_args, **_kwargs: None
    requested = []
    controller.request_home_position = lambda **values: requested.append(values)
    controller.install_startup_telemetry_verifier(None, verified=True)

    assert controller.arm(timeout=5.0) == 0

    assert requested == [{"timeout_s": 2.0, "poll_interval_s": 0.1}]


def test_relative_move_timeout_includes_preparation_and_send(monkeypatch):
    now = [10.0]
    monkeypatch.setattr(drone_control.time, "monotonic", lambda: now[0])
    controller = object.__new__(drone_control.DroneControl)
    field = lambda value: SimpleNamespace(
        fresh=True,
        observation=SimpleNamespace(value=value, sequence=1),
    )
    snapshot = SimpleNamespace(
        mode=field("GUIDED"),
        armed=field(True),
        landed_state=field(mavutil.mavlink.MAV_LANDED_STATE_IN_AIR),
        location=field((10_000_000, 20_000_000, 100_000, 10_000)),
        attitude=field((0.0, 0.0, 0.0)),
    )
    controller.flight_state = SimpleNamespace(snapshot=lambda: snapshot)
    controller.flight_controller_target = SourceIdentity(1, 1)
    controller._mission_home = MissionHome(1.0, 2.0, 90.0)
    controller.check_permission = lambda: None
    controller._require_fresh = lambda *_args: None
    controller._mode_name = lambda _snapshot: "GUIDED"
    controller._location_value = lambda value: value.observation.value
    controller.get_location_metres = lambda start, _north, _east: GPSCoord(
        start.lat, start.long, start.alt
    )
    controller.vehicle = SimpleNamespace(
        message_factory=SimpleNamespace(
            set_position_target_local_ned_encode=lambda *_args: object()
        ),
        send_mavlink=lambda _message: None,
    )

    def send_guarded(send, *, validate_snapshot=None, **_kwargs):
        if validate_snapshot is not None:
            validate_snapshot(snapshot)
        send()
        now[0] = 10.6
        return snapshot

    controller._send_guarded = send_guarded
    controller._field_sequence = lambda _field: 1
    deadlines = []
    controller._wait_until = lambda deadline, _predicate, _failure: deadlines.append(
        deadline
    )

    assert controller.guide_move_relative_frame(
        RelPosComplete(1.0, 0.0, 0.0), timeout=1.0
    ) == 0

    assert deadlines == [11.0]


def test_installed_callback_revocation_during_encoding_prevents_transport(monkeypatch):
    """A swallowed callback during pack cannot reach MAVWriter.queue.put."""
    vehicle = InertVehicle()
    outbound = queue.Queue()
    vehicle._master = SimpleNamespace(
        mav=SimpleNamespace(file=MAVWriter(outbound)),
        WIRE_PROTOCOL_VERSION="2.0",
    )
    flight_state = state()
    controller = construct(
        monkeypatch,
        vehicle,
        flight_state=flight_state,
        permission_guard=lambda: None,
        rc_health_decoder=lambda _message, _channel, _pwm: True,
        heartbeat_mode_decoder=lambda _message: "LOITER",
    )
    controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=vehicle._master.mav.file.transaction,
    )
    now = 10.0
    metadata = dict(
        received_at=now,
        source_system=1,
        source_component=1,
    )
    assert flight_state.observe_heartbeat(
        (2, 3, 0, 4, 3),
        armed=False,
        mode="GUIDED",
        sequence=1,
        **metadata,
    )
    assert flight_state.update(
        "landed_state",
        mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        sequence=2,
        **metadata,
    )
    assert flight_state.observe_rc_input(
        channel=7,
        pwm=1500,
        signal_healthy=True,
        sequence=3,
        **metadata,
    )
    assert flight_state.acquire_initial_companion_authority(now=now)
    assert flight_state.update("armed", True, sequence=4, **metadata)

    # Advance the controller callback's own monotonic sequence beyond the
    # directly seeded fixture observations without changing trusted evidence.
    for _ in range(4):
        ignored = SimpleNamespace(
            chan7_raw=1500,
            get_srcSystem=lambda: 99,
            get_srcComponent=lambda: 1,
        )
        vehicle.listeners["RC_CHANNELS"](vehicle, "RC_CHANNELS", ignored)

    class PackedMessage:
        def pack(self, _encoder, **_kwargs):
            rc = SimpleNamespace(
                chan7_raw=1900,
                get_srcSystem=lambda: 1,
                get_srcComponent=lambda: 1,
            )
            try:
                vehicle.listeners["RC_CHANNELS"](vehicle, "RC_CHANNELS", rc)
                heartbeat = SimpleNamespace(
                    type=2,
                    autopilot=3,
                    base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
                    custom_mode=5,
                    system_status=3,
                    get_srcSystem=lambda: 1,
                    get_srcComponent=lambda: 1,
                )
                vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", heartbeat)
            except Exception:
                pass
            return b"would-be-flight-command"

    vehicle.command_long_encode = lambda *_fields: PackedMessage()

    def send_mavlink(message):
        packet = message.pack(vehicle._master.mav)
        vehicle._master.mav.file.write(packet)

    vehicle.send_mavlink = send_mavlink

    with pytest.raises(AuthorityLost):
        controller.disarm(timeout=0.1)

    assert outbound.empty()
