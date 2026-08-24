from __future__ import annotations

from dataclasses import dataclass, field

from drone_sim_companion.controller import MissionController
from drone_sim_companion.mission import Ack, CommandKind, MissionPhase, Telemetry


@dataclass
class FakeVehicle:
    sent: list[tuple[CommandKind, float | None]] = field(default_factory=list)

    def send(self, command: CommandKind, altitude_m: float | None) -> None:
        self.sent.append((command, altitude_m))


def test_fake_vehicle_receives_the_complete_controlled_descent_sequence() -> None:
    vehicle = FakeVehicle()
    records: list[tuple[str, int, dict[str, object]]] = []
    controller = MissionController(vehicle, lambda name, stamp, fields: records.append((name, stamp, fields)))
    inputs = (
        Telemetry(0, heartbeat=True, mode="STABILIZE", armed=False),
        Telemetry(1, ack=Ack(CommandKind.SET_GUIDED, True, 0)),
        Telemetry(2, heartbeat=True, mode="GUIDED", armed=False),
        Telemetry(3, ack=Ack(CommandKind.ARM, True, 0)),
        Telemetry(4, heartbeat=True, mode="GUIDED", armed=True),
        Telemetry(5, ack=Ack(CommandKind.TAKEOFF, True, 0)),
        Telemetry(6, mode="GUIDED", armed=True, relative_altitude_m=1.4),
        Telemetry(7, ack=Ack(CommandKind.LAND, True, 0)),
        Telemetry(
            8,
            mode="LAND",
            armed=True,
            relative_altitude_m=1.0,
            vertical_speed_m_s=-0.3,
            landed=False,
        ),
        Telemetry(
            9,
            mode="LAND",
            armed=True,
            relative_altitude_m=0.0,
            vertical_speed_m_s=-0.05,
            landed=True,
        ),
        Telemetry(
            10,
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
