from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import threading
from types import SimpleNamespace

import pytest

from pymavlink.dialects.v20 import ardupilotmega as real_mavlink

from drone_sim_companion.flight_authority import FlightAuthorityGate, FlightAuthorityLost
from drone_sim_companion.mavlink_adapter import MavlinkAdapter
from drone_sim_companion.mission import CommandKind, Telemetry


class Message:
    def __init__(self, kind: str, *, source_system: int = 1, source_component: int = 1, **fields: object) -> None:
        self._kind = kind
        self._source_system = source_system
        self._source_component = source_component
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self) -> str:
        return self._kind

    def get_srcSystem(self) -> int:
        return self._source_system

    def get_srcComponent(self) -> int:
        return self._source_component


@dataclass
class FakeMav:
    calls: list[tuple[object, ...]] = field(default_factory=list)
    stream_calls: list[tuple[object, ...]] = field(default_factory=list)
    global_position_target_calls: list[tuple[object, ...]] = field(default_factory=list)
    parameter_calls: list[tuple[object, ...]] = field(default_factory=list)
    parameter_list_calls: list[tuple[object, ...]] = field(default_factory=list)
    landing_target_calls: list[tuple[object, ...]] = field(default_factory=list)

    def command_long_send(self, *arguments: object) -> None:
        self.calls.append(arguments)

    def request_data_stream_send(self, *arguments: object) -> None:
        self.stream_calls.append(arguments)

    def set_position_target_global_int_send(self, *arguments: object) -> None:
        self.global_position_target_calls.append(arguments)

    def param_request_read_send(self, *arguments: object) -> None:
        self.parameter_calls.append(arguments)

    def param_request_list_send(self, *arguments: object) -> None:
        self.parameter_list_calls.append(arguments)

    def landing_target_send(self, *arguments: object) -> None:
        self.landing_target_calls.append(arguments)


class FakeConnection:
    def __init__(self, messages: list[Message] | None = None) -> None:
        self.target_system = 1
        self.target_component = 1
        self.mav = FakeMav()
        self.messages = list(messages or [])

    def recv_match(self, *, blocking: bool) -> Message | None:
        assert blocking is False
        return self.messages.pop(0) if self.messages else None


def mavutil() -> SimpleNamespace:
    constants = SimpleNamespace(
        MAV_CMD_DO_SET_MODE=176,
        MAV_CMD_COMPONENT_ARM_DISARM=400,
        MAV_CMD_NAV_TAKEOFF=22,
        MAV_CMD_NAV_LAND=21,
        MAV_CMD_SET_MESSAGE_INTERVAL=511,
        MAVLINK_MSG_ID_HEARTBEAT=0,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1,
        MAV_MODE_FLAG_SAFETY_ARMED=128,
        MAV_RESULT_ACCEPTED=0,
        MAV_RESULT_IN_PROGRESS=5,
        MAV_LANDED_STATE_ON_GROUND=1,
        MAV_DATA_STREAM_ALL=0,
        MAV_SYS_STATUS_PREARM_CHECK=0x10000000,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT=6,
        MAV_FRAME_BODY_FRD=12,
        POSITION_TARGET_TYPEMASK_VX_IGNORE=8,
        POSITION_TARGET_TYPEMASK_VY_IGNORE=16,
        POSITION_TARGET_TYPEMASK_VZ_IGNORE=32,
        POSITION_TARGET_TYPEMASK_AX_IGNORE=64,
        POSITION_TARGET_TYPEMASK_AY_IGNORE=128,
        POSITION_TARGET_TYPEMASK_AZ_IGNORE=256,
        POSITION_TARGET_TYPEMASK_YAW_IGNORE=1024,
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE=2048,
    )
    return SimpleNamespace(
        mavlink=constants,
        mode_string_v10=lambda message: {0: "STABILIZE", 4: "GUIDED", 9: "LAND"}[
            message.custom_mode
        ],
    )


def real_heartbeat(*, source_system: int, source_component: int, armed: bool):
    encoder = real_mavlink.MAVLink(
        None, srcSystem=source_system, srcComponent=source_component
    )
    base_mode = real_mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    if armed:
        base_mode |= real_mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    message = encoder.heartbeat_encode(
        real_mavlink.MAV_TYPE_GCS
        if source_system == 255
        else real_mavlink.MAV_TYPE_QUADROTOR,
        real_mavlink.MAV_AUTOPILOT_INVALID
        if source_system == 255
        else real_mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        4,
        real_mavlink.MAV_STATE_ACTIVE,
    )
    message.pack(encoder)
    return message


def real_ack(*, source_system: int, source_component: int):
    encoder = real_mavlink.MAVLink(
        None, srcSystem=source_system, srcComponent=source_component
    )
    message = encoder.command_ack_encode(
        real_mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        real_mavlink.MAV_RESULT_ACCEPTED,
        0,
        0,
        0,
        0,
    )
    message.pack(encoder)
    return message


def test_commands_translate_to_exact_mavlink_long_commands() -> None:
    connection = FakeConnection()
    adapter = MavlinkAdapter(connection, mavutil())
    adapter.send(CommandKind.SET_GUIDED, None)
    adapter.send(CommandKind.ARM, None)
    adapter.send(CommandKind.TAKEOFF, 1.5)
    adapter.send(CommandKind.LAND, None)

    assert connection.mav.calls == [
        (1, 1, 176, 0, 1, 4, 0, 0, 0, 0, 0),
        (1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0),
        (1, 1, 22, 0, 0, 0, 0, 0, 0, 0, 1.5),
        (1, 1, 21, 0, 0, 0, 0, 0, 0, 0, 0),
    ]


def test_revoked_gate_blocks_all_raw_aircraft_outputs_but_allows_observation_setup() -> None:
    connection = FakeConnection()
    gate = FlightAuthorityGate()
    gate.observe(Telemetry(1, heartbeat=True, mode="STABILIZE", armed=False))
    gate.admit("a" * 64)
    gate.revoke("QGC selected LOITER")
    adapter = MavlinkAdapter(connection, mavutil(), authority_gate=gate)

    with pytest.raises(FlightAuthorityLost):
        adapter.send(CommandKind.SET_GUIDED, None)
    with pytest.raises(FlightAuthorityLost):
        adapter.send(CommandKind.ARM, None)
    with pytest.raises(FlightAuthorityLost):
        adapter.send_waypoint(37.0, -122.0, 5.0)
    with pytest.raises(FlightAuthorityLost):
        adapter.send_landing_target(0.0, 0.0, 1.0, exposure_timestamp_ns=1)

    adapter.request_parameters(("PLND_ENABLED",))
    adapter.request_telemetry()
    assert connection.mav.global_position_target_calls == []
    assert connection.mav.landing_target_calls == []
    assert connection.mav.parameter_list_calls == [(1, 1)]
    assert len(connection.mav.stream_calls) == 1


def test_raw_land_enqueue_owns_land_entry_and_eventual_disarm_separately() -> None:
    connection = FakeConnection()
    gate = FlightAuthorityGate()
    gate.observe(Telemetry(1, heartbeat=True, mode="GUIDED", armed=True))
    gate.admit("b" * 64)
    adapter = MavlinkAdapter(connection, mavutil(), authority_gate=gate)

    adapter.send(CommandKind.LAND, None)
    gate.observe(Telemetry(2, heartbeat=True, mode="LAND", armed=True))
    gate.observe(Telemetry(3, heartbeat=True, mode="LAND", armed=False))

    assert gate.revoked is False


def test_raw_finalization_after_action_selection_blocks_actual_adapter_write() -> None:
    connection = FakeConnection()
    gate = FlightAuthorityGate()
    gate.observe(Telemetry(1, heartbeat=True, mode="GUIDED", armed=True))
    gate.admit("b" * 64)
    adapter = MavlinkAdapter(connection, mavutil(), authority_gate=gate)
    selected = lambda: adapter.send(CommandKind.LAND, None)

    gate.begin_finalizing()

    with pytest.raises(FlightAuthorityLost, match="finalizing"):
        selected()
    assert connection.mav.calls == []


def test_waypoint_translates_to_relative_home_global_position_target() -> None:
    connection = FakeConnection()
    adapter = MavlinkAdapter(connection, mavutil())

    adapter.send_waypoint(37.4003371, -122.0800351, 12.5)

    assert connection.mav.global_position_target_calls == [
        (
            0,
            1,
            1,
            6,
            3576,
            374003371,
            -1220800351,
            12.5,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
    ]


def test_moving_waypoint_holds_north_yaw_for_long_camera_axis_along_east() -> None:
    connection = FakeConnection()
    adapter = MavlinkAdapter(connection, mavutil())

    adapter.send_waypoint(37.4003371, -122.079639322083, 5.0, yaw_rad=0.0)

    call = connection.mav.global_position_target_calls[0]
    assert call[4] == 2552
    assert call[-2:] == (0.0, 0.0)


@pytest.mark.parametrize(
    ("latitude_deg", "longitude_deg", "altitude_m"),
    [
        (float("nan"), 0.0, 10.0),
        (0.0, float("inf"), 10.0),
        (0.0, 0.0, float("-inf")),
        (-90.0000001, 0.0, 10.0),
        (90.0000001, 0.0, 10.0),
        (0.0, -180.0000001, 10.0),
        (0.0, 180.0000001, 10.0),
    ],
)
def test_waypoint_rejects_invalid_coordinates_before_output(
    latitude_deg: float, longitude_deg: float, altitude_m: float
) -> None:
    connection = FakeConnection()
    adapter = MavlinkAdapter(connection, mavutil())

    with pytest.raises(ValueError, match="waypoint"):
        adapter.send_waypoint(latitude_deg, longitude_deg, altitude_m)

    assert connection.mav.global_position_target_calls == []


def test_telemetry_request_keeps_generic_stream_and_requests_landed_state() -> None:
    connection = FakeConnection()
    adapter = MavlinkAdapter(connection, mavutil())
    adapter.request_telemetry(rate_hz=10)
    assert connection.mav.stream_calls == [(1, 1, 0, 10, 1)]
    assert connection.mav.calls == [
        (1, 1, 511, 0, 0, 100_000, 0, 0, 0, 0, 0),
        (1, 1, 511, 0, 245, 100_000, 0, 0, 0, 0, 0),
    ]


def test_foreign_and_gcs_heartbeats_cannot_change_vehicle_state() -> None:
    connection = FakeConnection(
        [
            real_heartbeat(source_system=255, source_component=190, armed=True),
            real_heartbeat(source_system=42, source_component=1, armed=True),
            real_heartbeat(source_system=1, source_component=1, armed=False),
        ]
    )
    gate = FlightAuthorityGate()
    gate.observe(Telemetry(1, heartbeat=True, mode="GUIDED", armed=False))
    gate.admit("a" * 64)
    adapter = MavlinkAdapter(connection, mavutil(), authority_gate=gate)

    vehicle = adapter.poll(3)

    assert vehicle is not None
    assert (vehicle.mode, vehicle.armed) == ("GUIDED", False)
    assert gate.revoked is False


def test_qgc_dispatch_does_not_starve_following_fc_telemetry() -> None:
    qgc = Message("COMMAND_LONG", source_system=255, source_component=190)
    fc = Message("HEARTBEAT", custom_mode=4, base_mode=0)
    connection = FakeConnection([qgc, fc])

    class QgcIO:
        def __init__(self) -> None:
            self.messages = []

        def observe_message(self, message) -> bool:
            self.messages.append(message)
            return message is qgc

    qgc_io = QgcIO()
    adapter = MavlinkAdapter(connection, mavutil(), qgc_io=qgc_io)

    telemetry = adapter.poll(1)

    assert qgc_io.messages == [qgc, fc]
    assert telemetry is not None and telemetry.heartbeat is True


def test_same_system_foreign_component_cannot_populate_fc_cache() -> None:
    connection = FakeConnection(
        [
            Message("HEARTBEAT", source_component=191, custom_mode=4, base_mode=128),
            Message("HEARTBEAT", source_component=1, custom_mode=4, base_mode=0),
        ]
    )
    adapter = MavlinkAdapter(connection, mavutil())

    telemetry = adapter.poll(1)

    assert telemetry is not None and telemetry.armed is False


def test_foreign_and_source_less_command_acknowledgements_are_ignored() -> None:
    source_less = Message("COMMAND_ACK", command=400, result=0)
    del source_less._source_system
    connection = FakeConnection(
        [
            real_ack(source_system=255, source_component=190),
            real_ack(source_system=42, source_component=1),
            source_less,
            real_ack(source_system=1, source_component=1),
        ]
    )
    adapter = MavlinkAdapter(connection, mavutil())

    vehicle = adapter.poll(4)

    assert vehicle is not None and vehicle.ack is not None
    assert vehicle.ack.command is CommandKind.ARM


def test_outputs_stay_pinned_to_vehicle_when_connection_target_is_contaminated() -> None:
    connection = FakeConnection()
    connection.target_system = 255
    connection.target_component = 190
    adapter = MavlinkAdapter(connection, mavutil())

    adapter.send(CommandKind.LAND, None)

    assert connection.mav.calls == [
        (1, 1, 21, 0, 0, 0, 0, 0, 0, 0, 0),
    ]


def test_vehicle_telemetry_interval_rejection_is_diagnostic() -> None:
    connection = FakeConnection(
        [Message("COMMAND_ACK", command=511, result=4)]
    )
    adapter = MavlinkAdapter(connection, mavutil())

    with pytest.raises(RuntimeError, match="telemetry interval.*result 4"):
        adapter.poll(1)


def test_required_parameters_use_one_complete_list_request_and_observations_are_exposed() -> None:
    connection = FakeConnection([
        Message("PARAM_VALUE", param_id=b"PLND_OPTIONS\x00", param_value=5.0),
    ])
    adapter = MavlinkAdapter(connection, mavutil())

    names = tuple(f"PARAM_{index:02d}" for index in range(34))
    adapter.request_parameters(names)
    telemetry = adapter.poll(1_000_000_000)

    assert connection.mav.parameter_list_calls == [(1, 1)]
    assert connection.mav.parameter_calls == []
    assert telemetry is not None
    assert (telemetry.parameter_name, telemetry.parameter_value) == ("PLND_OPTIONS", 5.0)


def test_attitude_and_horizontal_velocity_keep_public_exposure_timestamp() -> None:
    connection = FakeConnection([
        Message("ATTITUDE", roll=0.1, pitch=-0.2, yaw=0.3),
        Message("GLOBAL_POSITION_INT", lat=0, lon=0, relative_alt=1000, vx=30, vy=40, vz=0),
    ])
    adapter = MavlinkAdapter(connection, mavutil())

    attitude = adapter.poll(2_000_000_000)
    position = adapter.poll(2_050_000_000)

    assert attitude is not None
    assert attitude.attitude_rpy_rad == (0.1, -0.2, 0.3)
    assert attitude.attitude_timestamp_ns == 2_000_000_000
    assert position is not None and position.horizontal_speed_m_s == pytest.approx(0.5)


def test_landing_target_uses_exposure_time_and_position_valid_body_frd_encoding() -> None:
    connection = FakeConnection()
    adapter = MavlinkAdapter(connection, mavutil())

    adapter.send_landing_target(
        1.0,
        -2.0,
        4.0,
        exposure_timestamp_ns=12_345_678_901,
    )

    assert connection.mav.landing_target_calls == [(
        12_345_678, 0, 12,
        pytest.approx(0.2449786631), pytest.approx(-0.4636476090), pytest.approx(4.582575695),
        0.0, 0.0,
        1.0, -2.0, 4.0,
        (1.0, 0.0, 0.0, 0.0), 0, 1,
    )]


def test_mavlink_messages_are_stamped_with_current_simulation_time() -> None:
    connection = FakeConnection(
        [
            Message("HEARTBEAT", custom_mode=4, base_mode=128),
            Message("COMMAND_ACK", command=400, result=0),
            Message("GLOBAL_POSITION_INT", relative_alt=1234, vz=25),
            Message("EXTENDED_SYS_STATE", landed_state=1),
        ]
    )
    adapter = MavlinkAdapter(connection, mavutil())

    heartbeat = adapter.poll(1_000_000_000)
    acknowledgement = adapter.poll(1_050_000_000)
    altitude = adapter.poll(1_100_000_000)
    landed = adapter.poll(1_150_000_000)

    assert heartbeat is not None
    assert (heartbeat.timestamp_ns, heartbeat.heartbeat, heartbeat.mode, heartbeat.armed) == (
        1_000_000_000,
        True,
        "GUIDED",
        True,
    )
    assert acknowledgement is not None
    assert acknowledgement.ack is not None
    assert acknowledgement.ack.command is CommandKind.ARM
    assert acknowledgement.ack.accepted
    assert altitude is not None
    assert altitude.relative_altitude_m == 1.234
    assert altitude.vertical_speed_m_s == -0.25
    assert altitude.latitude_deg is None
    assert altitude.longitude_deg is None
    assert landed is not None and landed.landed is True


def test_global_position_exposes_decimal_degree_coordinates() -> None:
    connection = FakeConnection(
        [
            Message(
                "GLOBAL_POSITION_INT",
                lat=374003371,
                lon=-1220800351,
                relative_alt=12_500,
                vz=-25,
            )
        ]
    )
    adapter = MavlinkAdapter(connection, mavutil())

    position = adapter.poll(2_000_000_000)

    assert position is not None
    assert position.latitude_deg == pytest.approx(37.4003371)
    assert position.longitude_deg == pytest.approx(-122.0800351)


def test_in_progress_ack_waits_and_negative_ack_is_exposed() -> None:
    connection = FakeConnection(
        [
            Message("COMMAND_ACK", command=22, result=5),
            Message("COMMAND_ACK", command=22, result=4),
        ]
    )
    adapter = MavlinkAdapter(connection, mavutil())
    assert adapter.poll(10) is None
    rejected = adapter.poll(20)
    assert rejected is not None and rejected.ack is not None
    assert rejected.ack.command is CommandKind.TAKEOFF
    assert not rejected.ack.accepted


def test_landed_state_is_preserved_across_later_heartbeat() -> None:
    connection = FakeConnection(
        [
            Message("EXTENDED_SYS_STATE", landed_state=1),
            Message("HEARTBEAT", custom_mode=9, base_mode=0),
        ]
    )
    adapter = MavlinkAdapter(connection, mavutil())

    landed = adapter.poll(100)
    disarmed = adapter.poll(200)

    assert landed is not None and landed.landed is True
    assert disarmed is not None
    assert disarmed.armed is False and disarmed.landed is True


def test_operator_facts_only_record_new_exact_fc_ground_messages() -> None:
    wall = iter((100, 200))
    connection = FakeConnection(
        [
            Message("EXTENDED_SYS_STATE", landed_state=1),
            Message("HEARTBEAT", custom_mode=9, base_mode=0),
            Message("GLOBAL_POSITION_INT", relative_alt=0, vx=0, vy=0, vz=0),
        ]
    )
    adapter = MavlinkAdapter(connection, mavutil(), wall_time_ns=lambda: next(wall))

    adapter.poll(1)
    adapter.poll(2)
    observations = adapter.drain_operator_observations()
    adapter.poll(3)

    assert [(item.received_wall_time_ns, item.armed, item.landed) for item in observations] == [
        (100, None, True),
        (200, False, None),
    ]
    assert adapter.drain_operator_observations() == ()


def test_raw_operator_fact_drain_is_atomic_with_message_observation() -> None:
    adapter = MavlinkAdapter(FakeConnection(), mavutil(), wall_time_ns=lambda: 100)
    adapter._observe_operator_fact(
        Message("HEARTBEAT", custom_mode=0, base_mode=0),
        1,
    )
    iteration_started = threading.Event()
    allow_iteration = threading.Event()

    class PausingDeque(deque):
        def __iter__(self):
            iteration_started.set()
            assert allow_iteration.wait(1.0)
            return super().__iter__()

    adapter._operator_observations = PausingDeque(
        adapter._operator_observations,
        maxlen=64,
    )
    drained = []
    drain_finished = threading.Event()
    append_finished = threading.Event()
    drain = threading.Thread(
        target=lambda: (drained.extend(adapter.drain_operator_observations()), drain_finished.set())
    )
    append = threading.Thread(
        target=lambda: (
            adapter._observe_operator_fact(Message("EXTENDED_SYS_STATE", landed_state=1), 2),
            append_finished.set(),
        )
    )
    drain.start()
    assert iteration_started.wait(1.0)
    append.start()
    assert not append_finished.wait(0.02)
    allow_iteration.set()
    drain.join(timeout=1.0)
    append.join(timeout=1.0)

    assert len(drained) == 1
    remaining = adapter.drain_operator_observations()
    assert len(remaining) == 1
    assert remaining[0].landed is True


def test_statustext_is_preserved_as_simulation_stamped_diagnostics() -> None:
    connection = FakeConnection(
        [Message("STATUSTEXT", severity=3, text="PreArm: Compass not calibrated")]
    )
    adapter = MavlinkAdapter(connection, mavutil())

    diagnostic = adapter.poll(23_000_000_000)

    assert diagnostic is not None
    assert diagnostic.timestamp_ns == 23_000_000_000
    assert diagnostic.status_text == "PreArm: Compass not calibrated"
    assert diagnostic.status_severity == 3


def test_sys_status_requires_prearm_check_enabled_and_healthy() -> None:
    prearm = 0x10000000
    connection = FakeConnection(
        [
            Message(
                "SYS_STATUS",
                onboard_control_sensors_enabled=0,
                onboard_control_sensors_health=0,
            ),
            Message(
                "SYS_STATUS",
                onboard_control_sensors_enabled=prearm,
                onboard_control_sensors_health=0,
            ),
            Message(
                "SYS_STATUS",
                onboard_control_sensors_enabled=prearm,
                onboard_control_sensors_health=prearm,
            ),
        ]
    )
    adapter = MavlinkAdapter(connection, mavutil())

    assert adapter.poll(1).prearm_checks_healthy is False
    assert adapter.poll(2).prearm_checks_healthy is False
    assert adapter.poll(3).prearm_checks_healthy is True
