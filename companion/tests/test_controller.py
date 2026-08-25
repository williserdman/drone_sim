from __future__ import annotations

from dataclasses import dataclass, field

from drone_sim_companion.controller import MissionController, process_telemetry
from drone_sim_companion.mission import Ack, CommandKind, MissionPhase, Telemetry


@dataclass
class FakeVehicle:
    sent: list[tuple[CommandKind, float | None]] = field(default_factory=list)

    def send(self, command: CommandKind, altitude_m: float | None) -> None:
        self.sent.append((command, altitude_m))


def test_pre_run_heartbeat_makes_runtime_ready_without_starting_mission() -> None:
    vehicle = FakeVehicle()
    records: list[tuple[str, int, dict[str, object]]] = []
    controller = MissionController(
        vehicle, lambda name, stamp, fields: records.append((name, stamp, fields))
    )

    process_telemetry(
        controller,
        Telemetry(0, heartbeat=True, mode="STABILIZE", armed=False),
        mission_running=False,
        public_clock_observed=False,
    )

    assert controller.ready
    assert controller.state.phase is MissionPhase.WAIT_HEARTBEAT
    assert vehicle.sent == []
    assert records == [("heartbeat_observed", 0, {})]

    process_telemetry(
        controller,
        Telemetry(50_000_000, heartbeat=True, mode="STABILIZE", armed=False),
        mission_running=True,
        public_clock_observed=True,
    )

    assert controller.state.phase is MissionPhase.WAIT_GUIDED_ACK
    assert vehicle.sent == [(CommandKind.SET_GUIDED, None)]
    assert [name for name, _stamp, _fields in records] == [
        "heartbeat_observed",
        "command_issued",
    ]


def test_pre_run_readiness_latches_heartbeat_and_prearm_health_independently() -> None:
    vehicle = FakeVehicle()
    records: list[tuple[str, int, dict[str, object]]] = []
    controller = MissionController(
        vehicle, lambda name, stamp, fields: records.append((name, stamp, fields))
    )

    process_telemetry(
        controller,
        Telemetry(10, prearm_checks_healthy=True),
        mission_running=False,
        public_clock_observed=False,
    )

    assert not controller.heartbeat_observed
    assert controller.prearm_checks_healthy
    assert not controller.mission_ready
    assert vehicle.sent == []

    process_telemetry(
        controller,
        Telemetry(20, heartbeat=True, mode="STABILIZE", armed=False),
        mission_running=False,
        public_clock_observed=False,
    )
    process_telemetry(
        controller,
        Telemetry(30, prearm_checks_healthy=False),
        mission_running=False,
        public_clock_observed=False,
    )

    assert controller.heartbeat_observed
    assert controller.prearm_checks_healthy
    assert controller.mission_ready
    assert vehicle.sent == []
    assert records == [
        ("prearm_checks_healthy", 10, {}),
        ("heartbeat_observed", 20, {}),
    ]


def test_mission_commands_require_running_and_the_first_public_clock() -> None:
    vehicle = FakeVehicle()
    controller = MissionController(vehicle, lambda *_record: None)
    heartbeat = Telemetry(0, heartbeat=True, mode="STABILIZE", armed=False)

    process_telemetry(
        controller,
        heartbeat,
        mission_running=True,
        public_clock_observed=False,
    )
    process_telemetry(
        controller,
        heartbeat,
        mission_running=False,
        public_clock_observed=True,
    )

    assert controller.state.phase is MissionPhase.WAIT_HEARTBEAT
    assert vehicle.sent == []

    process_telemetry(
        controller,
        heartbeat,
        mission_running=True,
        public_clock_observed=True,
    )

    assert controller.state.phase is MissionPhase.WAIT_GUIDED_ACK
    assert vehicle.sent == [(CommandKind.SET_GUIDED, None)]


def test_running_mavlink_status_text_is_emitted_without_changing_policy() -> None:
    records = []
    controller = MissionController(FakeVehicle(), lambda *record: records.append(record))

    controller.consume(
        Telemetry(
            23_000_000_000,
            status_text="PreArm: Compass not calibrated",
            status_severity=3,
        )
    )

    assert controller.state.phase is MissionPhase.WAIT_HEARTBEAT
    assert records == [
        (
            "mavlink_status_text",
            23_000_000_000,
            {"status_severity": 3, "text": "PreArm: Compass not calibrated"},
        )
    ]

def test_fake_vehicle_receives_the_complete_controlled_descent_sequence() -> None:
    vehicle = FakeVehicle()
    records: list[tuple[str, int, dict[str, object]]] = []
    controller = MissionController(vehicle, lambda name, stamp, fields: records.append((name, stamp, fields)))
    inputs = (
        Telemetry(0, heartbeat=True, mode="STABILIZE", armed=False),
        Telemetry(1, ack=Ack(CommandKind.SET_GUIDED, True, 0)),
        Telemetry(2, heartbeat=True, mode="GUIDED", armed=False),
        Telemetry(3, prearm_checks_healthy=True),
        Telemetry(4, ack=Ack(CommandKind.ARM, True, 0)),
        Telemetry(5, heartbeat=True, mode="GUIDED", armed=True),
        Telemetry(6, ack=Ack(CommandKind.TAKEOFF, True, 0)),
        Telemetry(7, mode="GUIDED", armed=True, relative_altitude_m=1.4),
        Telemetry(8, ack=Ack(CommandKind.LAND, True, 0)),
        Telemetry(
            9,
            mode="LAND",
            armed=True,
            relative_altitude_m=1.0,
            vertical_speed_m_s=-0.3,
            landed=False,
        ),
        Telemetry(
            10,
            mode="LAND",
            armed=True,
            relative_altitude_m=0.0,
            vertical_speed_m_s=-0.05,
            landed=True,
        ),
        Telemetry(
            11,
            mode="LAND",
            armed=False,
            relative_altitude_m=0.0,
            vertical_speed_m_s=0.0,
            landed=True,
        ),
    )

    for item in inputs:
        controller.consume(item)

    assert controller.ready
    assert controller.state.phase is MissionPhase.LANDED
    assert vehicle.sent == [
        (CommandKind.SET_GUIDED, None),
        (CommandKind.ARM, None),
        (CommandKind.TAKEOFF, 1.5),
        (CommandKind.LAND, None),
    ]
    assert [name for name, _stamp, _fields in records] == [
        "heartbeat_observed",
        "command_issued",
        "command_acknowledged",
        "mode_confirmed",
        "prearm_checks_healthy",
        "command_issued",
        "command_acknowledged",
        "armed_observed",
        "command_issued",
        "command_acknowledged",
        "altitude_observed",
        "command_issued",
        "command_acknowledged",
        "altitude_observed",
        "descent_observed",
        "altitude_observed",
        "touchdown_observed",
        "altitude_observed",
        "vehicle_disarmed",
        "mission_landed",
    ]


def test_failed_mission_emits_failure_once_and_never_sends_repair() -> None:
    vehicle = FakeVehicle()
    records: list[tuple[str, int, dict[str, object]]] = []
    controller = MissionController(vehicle, lambda name, stamp, fields: records.append((name, stamp, fields)))
    controller.consume(Telemetry(10, heartbeat=True, mode="STABILIZE", armed=False))
    controller.consume(Telemetry(11, ack=Ack(CommandKind.SET_GUIDED, False, 4)))
    controller.consume(Telemetry(12, ack=Ack(CommandKind.SET_GUIDED, True, 0)))

    assert controller.state.phase is MissionPhase.FAILED
    assert vehicle.sent == [(CommandKind.SET_GUIDED, None)]
    assert [record[0] for record in records].count("mission_failed") == 1


def test_command_is_not_logged_or_committed_when_vehicle_send_fails() -> None:
    class BrokenVehicle:
        def send(self, command: CommandKind, altitude_m: float | None) -> None:
            raise ConnectionError("MAVLink unavailable")

    records: list[tuple[str, int, dict[str, object]]] = []
    controller = MissionController(
        BrokenVehicle(), lambda name, stamp, fields: records.append((name, stamp, fields))
    )
    try:
        controller.consume(Telemetry(0, heartbeat=True, mode="STABILIZE", armed=False))
    except ConnectionError:
        pass
    else:
        raise AssertionError("broken MAVLink send unexpectedly succeeded")

    assert controller.state.phase is MissionPhase.WAIT_HEARTBEAT
    assert [record[0] for record in records] == ["heartbeat_observed"]
