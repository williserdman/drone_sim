from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace

import pytest

from drone_sim_companion.mavlink_adapter import MavlinkAdapter
from drone_sim_companion.mission import CommandKind


class Message:
    def __init__(self, kind: str, **fields: object) -> None:
        self._kind = kind
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self) -> str:
        return self._kind


@dataclass
class FakeMav:
    calls: list[tuple[object, ...]] = field(default_factory=list)
    stream_calls: list[tuple[object, ...]] = field(default_factory=list)
    global_position_target_calls: list[tuple[object, ...]] = field(default_factory=list)

    def command_long_send(self, *arguments: object) -> None:
        self.calls.append(arguments)

    def request_data_stream_send(self, *arguments: object) -> None:
        self.stream_calls.append(arguments)

    def set_position_target_global_int_send(self, *arguments: object) -> None:
        self.global_position_target_calls.append(arguments)


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
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1,
        MAV_MODE_FLAG_SAFETY_ARMED=128,
        MAV_RESULT_ACCEPTED=0,
        MAV_RESULT_IN_PROGRESS=5,
        MAV_LANDED_STATE_ON_GROUND=1,
        MAV_DATA_STREAM_ALL=0,
        MAV_SYS_STATUS_PREARM_CHECK=0x10000000,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT=6,
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
        (1, 1, 511, 0, 245, 100_000, 0, 0, 0, 0, 0),
    ]


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
