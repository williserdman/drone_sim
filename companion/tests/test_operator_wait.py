from __future__ import annotations

from dataclasses import dataclass

import drone_sim_companion.operator_wait as operator_wait
from drone_sim_companion.mission import Ack, CommandKind, Telemetry
from drone_sim_companion.operator_wait import (
    OperatorPhase,
    OperatorWaitActor,
    connect_operator_mavlink,
)


RUN_ID = "123e4567-e89b-42d3-a456-426614174000"


class Vehicle:
    def __init__(self) -> None:
        self.commands: list[CommandKind] = []

    def send(self, command: CommandKind, altitude_m: float | None) -> None:
        assert altitude_m is None
        self.commands.append(command)


@dataclass(frozen=True)
class ExecutionReady:
    run_id: str
    sim_timestamp_ns: int


@dataclass(frozen=True)
class WaitStarted:
    run_id: str
    operation_id: str
    sim_timestamp_ns: int


def actor():
    vehicle = Vehicle()
    events: list[tuple[str, int, dict[str, object]]] = []
    return OperatorWaitActor(vehicle, lambda name, stamp, fields: events.append((name, stamp, fields)), run_id=RUN_ID), vehicle, events


def ready(subject: OperatorWaitActor, stamp: int = 1_000_000_000) -> None:
    subject.observe_running(RUN_ID)
    subject.observe_execution_ready(ExecutionReady(RUN_ID, stamp - 2))
    subject.observe_wait_started(WaitStarted(RUN_ID, "operation-0", stamp - 1))
    subject.observe(Telemetry(stamp, heartbeat=True, mode="STABILIZE", armed=False))


def test_actor_requires_same_run_started_wait_and_fresh_heartbeat() -> None:
    subject, vehicle, _events = actor()
    stamp = 1_000_000_000

    subject.observe_running("00000000-0000-0000-0000-000000000000")
    subject.observe_execution_ready(ExecutionReady(RUN_ID, stamp - 2))
    subject.observe_wait_started(WaitStarted(RUN_ID, "operation-0", stamp - 1))
    subject.observe(Telemetry(stamp - 500_000_001, heartbeat=True, mode="STABILIZE", armed=False))
    subject.tick(stamp)
    assert vehicle.commands == []

    subject.observe_running(RUN_ID)
    subject.tick(stamp)
    assert vehicle.commands == []

    subject.observe(Telemetry(stamp, heartbeat=True, mode="STABILIZE", armed=False))
    subject.tick(stamp)
    assert vehicle.commands == [CommandKind.SET_GUIDED]


def test_actor_requires_observed_guided_after_ack_and_observed_arm_after_ack() -> None:
    subject, vehicle, events = actor()
    ready(subject)
    subject.tick(1_000_000_000)

    subject.observe(Telemetry(1_000_000_001, ack=Ack(CommandKind.SET_GUIDED, True, 0)))
    subject.tick(1_000_000_001)
    assert vehicle.commands == [CommandKind.SET_GUIDED]

    subject.observe(Telemetry(1_000_000_002, heartbeat=True, mode="GUIDED", armed=False))
    subject.tick(1_000_000_002)
    assert vehicle.commands == [CommandKind.SET_GUIDED, CommandKind.ARM]

    subject.observe(Telemetry(1_000_000_003, ack=Ack(CommandKind.ARM, True, 0)))
    subject.tick(1_000_000_003)
    assert subject.phase is OperatorPhase.WAIT_ARMED

    subject.observe(Telemetry(1_000_000_004, heartbeat=True, mode="GUIDED", armed=True))
    subject.tick(1_000_000_004)
    assert subject.phase is OperatorPhase.PASSIVE
    assert [event[0] for event in events] == [
        "operator_command",
        "operator_acknowledgement",
        "operator_observed_guided",
        "operator_command",
        "operator_acknowledgement",
        "operator_observed_armed",
    ]


def test_rejected_ack_fails_and_finalization_prevents_commands() -> None:
    subject, vehicle, _events = actor()
    ready(subject)
    subject.tick(1_000_000_000)
    subject.observe(Telemetry(1_000_000_001, ack=Ack(CommandKind.SET_GUIDED, False, 4)))
    subject.tick(1_000_000_001)
    assert subject.phase is OperatorPhase.FAILED

    another, another_vehicle, _ = actor()
    ready(another)
    another.stop_for_finalization()
    another.tick(1_000_000_000)
    assert another.phase is OperatorPhase.FINALIZED
    assert another_vehicle.commands == []


def test_incorrect_observed_mode_after_guided_ack_fails_closed() -> None:
    subject, vehicle, _events = actor()
    ready(subject)
    subject.tick(1_000_000_000)
    subject.observe(Telemetry(1_000_000_001, ack=Ack(CommandKind.SET_GUIDED, True, 0)))

    subject.observe(Telemetry(1_000_000_002, heartbeat=True, mode="LOITER", armed=False))

    assert subject.phase is OperatorPhase.FAILED
    assert vehicle.commands == [CommandKind.SET_GUIDED]


def test_operator_connection_retries_refusal_and_uses_distinct_source_system() -> None:
    attempts = []
    clock = iter((0.0, 0.1))
    connection = object()

    def factory(endpoint, **keywords):
        attempts.append((endpoint, keywords))
        if len(attempts) < 3:
            raise ConnectionRefusedError("SITL is still starting")
        return connection

    assert connect_operator_mavlink(
        factory,
        deadline=1.0,
        now=lambda: next(clock),
        pause=lambda _seconds: None,
    ) is connection
    assert attempts == [
        ("tcp:ardupilot-sitl:5762", {"autoreconnect": False, "source_system": 253}),
        ("tcp:ardupilot-sitl:5762", {"autoreconnect": False, "source_system": 253}),
        ("tcp:ardupilot-sitl:5762", {"autoreconnect": False, "source_system": 253}),
    ]


def test_operator_drains_private_warmup_without_delivering_telemetry() -> None:
    subject, command_vehicle, events = actor()

    class Connection:
        def __init__(self) -> None:
            self.received = []

        def recv_match(self, *, blocking: bool):
            self.received.append(blocking)
            return object()

    class TelemetryVehicle:
        def __init__(self) -> None:
            self.timestamps = []
            self.telemetry = iter((
                Telemetry(1_000_000_001, ack=Ack(CommandKind.SET_GUIDED, True, 0)),
                Telemetry(1_000_000_002, heartbeat=True, mode="GUIDED", armed=False),
            ))

        def poll(self, timestamp_ns: int):
            self.timestamps.append(timestamp_ns)
            return next(self.telemetry)

    connection = Connection()
    telemetry_vehicle = TelemetryVehicle()
    event_count = len(events)

    operator_wait.poll_operator_mavlink(
        connection, telemetry_vehicle, subject, public_timestamp_ns=None
    )

    assert connection.received == [False]
    assert telemetry_vehicle.timestamps == []
    assert len(events) == event_count
    assert command_vehicle.commands == []

    ready(subject)
    subject.tick(1_000_000_000)

    operator_wait.poll_operator_mavlink(
        connection, telemetry_vehicle, subject, public_timestamp_ns=1_000_000_001
    )
    operator_wait.poll_operator_mavlink(
        connection, telemetry_vehicle, subject, public_timestamp_ns=1_000_000_002
    )

    assert telemetry_vehicle.timestamps == [1_000_000_001, 1_000_000_002]
    assert command_vehicle.commands == [CommandKind.SET_GUIDED, CommandKind.ARM]
