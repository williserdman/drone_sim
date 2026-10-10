from __future__ import annotations

import collections
import collections.abc
from dataclasses import dataclass, field
import importlib
from pathlib import Path
import queue
from typing import Callable

import pytest

from artifacts.operator_protocol import OperatorProtocol
from artifacts.qgc_operator import (
    MavlinkIdentity,
    QGC_ABORT_COMMAND,
    QGC_ACTIONS_KIND,
    QGC_START_COMMAND,
    QgcCommandEnvelope,
    QgcCommandResponse,
    QgcOperatorDescriptor,
)
from drone_sim_companion.operator_control import CompanionOperatorControl
from pymavlink.dialects.v20 import ardupilotmega as mavlink2


RUN_ID = "00000000-0000-4000-8000-000000000001"
QGC_IDENTITY = MavlinkIdentity(255, 190)
COMPANION_IDENTITY = MavlinkIdentity(1, 191)


def _transport_module():
    if not hasattr(collections, "MutableMapping"):
        collections.MutableMapping = collections.abc.MutableMapping
    return importlib.import_module("drone_sim_companion.qgc_transport")


def _frame(message, *, source: MavlinkIdentity) -> bytes:
    encoder = mavlink2.MAVLink(
        None, srcSystem=source.system_id, srcComponent=source.component_id
    )
    return message.pack(encoder)


def _parse(frame: bytes):
    parser = mavlink2.MAVLink(None)
    decoded = None
    for byte in frame:
        value = parser.parse_char(bytes((byte,)))
        if value is not None:
            decoded = value
    assert decoded is not None
    return decoded


def _drain_queue(values: queue.Queue) -> list[bytes]:
    frames = []
    while True:
        try:
            frames.append(bytes(values.get_nowait()))
        except queue.Empty:
            return frames


@dataclass
class _RawConnection:
    messages: list[object] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.frames: list[bytes] = []
        self.incoming: collections.deque[object] = collections.deque(self.messages)
        self.read_count = 0
        self.closed = False
        self.mav = mavlink2.MAVLink(self, srcSystem=1, srcComponent=191)

    def write(self, value: bytes) -> None:
        self.frames.append(bytes(value))

    def recv_match(self, *, blocking: bool):
        assert blocking is False
        self.read_count += 1
        return self.incoming.popleft() if self.incoming else None

    def close(self) -> None:
        self.closed = True


@dataclass
class _AdapterSession:
    transport: object
    connection: object
    received: list[QgcCommandEnvelope]
    ingress: Callable[[object], None]
    drain_frames: Callable[[], list[bytes]]
    close: Callable[[], None]


def _open_adapter(adapter_kind: str, *, wall_time_ns: int = 123) -> _AdapterSession:
    module = _transport_module()
    received: list[QgcCommandEnvelope] = []

    if adapter_kind == "raw":
        from drone_sim_companion.mavlink_adapter import MavlinkAdapter
        from pymavlink import mavutil

        connection = _RawConnection()
        transport = module.PymavlinkQgcCommandIO(
            connection,
            mavlink2,
            wall_time_ns=lambda: wall_time_ns,
        )
        transport.install(received.append)
        owner = MavlinkAdapter(connection, mavutil, qgc_io=transport)

        def ingress(message: object) -> None:
            connection.incoming.append(message)
            owner.poll(0)

        def drain_frames() -> list[bytes]:
            frames = list(connection.frames)
            connection.frames.clear()
            return frames

        assert owner._connection is connection
        assert owner._qgc_io is transport
        assert transport._connection is connection
        return _AdapterSession(
            transport,
            connection,
            received,
            ingress,
            drain_frames,
            connection.close,
        )

    assert adapter_kind == "dronekit"
    from dronekit.mavlink import MAVConnection, MAVWriter
    from drone_sim_companion.dronekit_sim import SimulationVehicle

    module.configure_dronekit_mavlink2()
    connection = MAVConnection(
        "udpin:127.0.0.1:0",
        source_system=1,
        source_component=191,
    )
    vehicle = SimulationVehicle(connection)
    transport = module.DroneKitQgcCommandIO(
        vehicle,
        mavlink2,
        wall_time_ns=lambda: wall_time_ns,
    )
    transport.install(received.append)
    writer = getattr(
        connection.master.mav.file,
        "delegate",
        connection.master.mav.file,
    )
    assert isinstance(writer, MAVWriter)
    assert writer.queue is connection.out_queue
    assert transport._encoder is connection.master.mav

    def ingress(message: object) -> None:
        vehicle.notify_message_listeners(message.get_type(), message)

    def close() -> None:
        connection._alive = False
        connection.mavlink_thread_in = None
        connection.mavlink_thread_out = None
        connection.master.close()

    return _AdapterSession(
        transport,
        connection,
        received,
        ingress,
        lambda: _drain_queue(connection.out_queue),
        close,
    )


def _command_frame(*, target: MavlinkIdentity, command: int, token: int) -> bytes:
    return _frame(
        mavlink2.MAVLink_command_long_message(
            target.system_id,
            target.component_id,
            command,
            0,
            token,
            0,
            0,
            0,
            0,
            0,
            0,
        ),
        source=QGC_IDENTITY,
    )


@pytest.mark.parametrize("adapter_kind", ("raw", "dronekit"))
def test_adapters_decode_equivalent_commands_and_target_acks(
    adapter_kind: str,
) -> None:
    session = _open_adapter(adapter_kind)
    try:
        heartbeat_frames = session.drain_frames()
        assert len(heartbeat_frames) == 1
        route_heartbeat = _parse(heartbeat_frames[0])
        assert route_heartbeat.get_type() == "HEARTBEAT"
        assert route_heartbeat.get_msgbuf()[:1] == b"\xfd"
        assert (
            route_heartbeat.get_srcSystem(),
            route_heartbeat.get_srcComponent(),
        ) == (1, 191)

        command_bytes = _command_frame(
            target=COMPANION_IDENTITY,
            command=QGC_START_COMMAND,
            token=987654,
        )
        assert command_bytes[:1] == b"\xfd"
        session.ingress(_parse(command_bytes))
        session.ingress(_parse(command_bytes))

        assert len(session.received) == 2
        envelope = session.received[0]
        assert envelope.wire_version == 2
        assert envelope.source == QGC_IDENTITY
        assert envelope.target == COMPANION_IDENTITY
        assert envelope.command == QGC_START_COMMAND
        assert envelope.parameters == (
            987654.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        assert session.received[1] == envelope
        if adapter_kind == "raw":
            assert session.connection.read_count == 4

        session.transport.send_ack(
            QgcCommandResponse(
                QGC_START_COMMAND,
                QGC_IDENTITY,
                mavlink2.MAV_RESULT_IN_PROGRESS,
                25,
                "Start pending for token 987654",
            )
        )
        ack, status = map(_parse, session.drain_frames())
        assert ack.get_msgbuf()[:1] == b"\xfd"
        assert (ack.get_srcSystem(), ack.get_srcComponent()) == (1, 191)
        assert (ack.target_system, ack.target_component) == (255, 190)
        assert (ack.command, ack.result, ack.progress) == (
            QGC_START_COMMAND,
            mavlink2.MAV_RESULT_IN_PROGRESS,
            25,
        )
        assert (status.get_srcSystem(), status.get_srcComponent()) == (1, 191)
        assert status.text == "Start pending for token 987654"
    finally:
        session.close()


@pytest.mark.parametrize("adapter_kind", ("raw", "dronekit"))
def test_fc_targeted_aircraft_command_receives_no_companion_ack(
    adapter_kind: str,
) -> None:
    session = _open_adapter(adapter_kind)
    try:
        session.drain_frames()
        aircraft_command = _parse(
            _command_frame(
                target=MavlinkIdentity(1, 1),
                command=mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
                token=1,
            )
        )

        session.ingress(aircraft_command)

        assert session.received == []
        assert session.drain_frames() == []
        if adapter_kind == "raw":
            assert session.connection.read_count == 2
    finally:
        session.close()


@pytest.mark.parametrize("adapter_kind", ("raw", "dronekit"))
def test_adapter_accepts_command_after_disconnect_and_reconnect(
    adapter_kind: str,
) -> None:
    connections = []
    for token in (41, 42):
        session = _open_adapter(adapter_kind, wall_time_ns=token)
        connections.append(session.connection)
        try:
            session.drain_frames()
            session.ingress(
                _parse(
                    _command_frame(
                        target=COMPANION_IDENTITY,
                        command=QGC_ABORT_COMMAND,
                        token=token,
                    )
                )
            )
            assert len(session.received) == 1
            assert session.received[0].parameters[0] == float(token)
            assert session.received[0].received_wall_time_ns == token
        finally:
            session.close()

    assert connections[0] is not connections[1]
    if adapter_kind == "raw":
        assert all(connection.closed for connection in connections)


def _descriptor() -> QgcOperatorDescriptor:
    return QgcOperatorDescriptor(
        QGC_ACTIONS_KIND,
        QGC_START_COMMAND,
        QGC_ABORT_COMMAND,
        17,
        "00000000-0000-4000-8000-000000000002",
        "00000000-0000-4000-8000-000000000003",
        QGC_IDENTITY,
        COMPANION_IDENTITY,
        MavlinkIdentity(1, 1),
        "configuration/qgc-actions.json",
        "b" * 64,
    )


def test_raw_adapter_duplicate_ingress_stays_bounded(tmp_path: Path) -> None:
    from drone_sim_companion.mavlink_adapter import MavlinkAdapter
    from pymavlink import mavutil

    module = _transport_module()
    run_directory = tmp_path / RUN_ID
    for relative in (".control", ".status", "configuration"):
        (run_directory / relative).mkdir(parents=True, exist_ok=True)
    protocol = OperatorProtocol(run_directory, RUN_ID, "a" * 64)
    operator = CompanionOperatorControl(
        protocol=protocol,
        run_id=RUN_ID,
        config_sha256="a" * 64,
        launch_mode="operator",
        preparation_wall_seconds=1800,
        mission_wall_seconds=120,
        recovery_wall_seconds=10,
    )
    connection = _RawConnection()
    transport = module.PymavlinkQgcCommandIO(connection, mavlink2)
    owner = MavlinkAdapter(connection, mavutil, qgc_io=transport)
    host = module.QgcHostControl(
        protocol=protocol,
        operator=operator,
        descriptor=_descriptor(),
        io=transport,
        readiness_reasons=lambda: (),
        is_finalizing=lambda: False,
    )
    host.observe_vehicle(
        native_timestamp_ns=0,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=True,
        mode="STABILIZE",
    )
    command_bytes = _command_frame(
        target=COMPANION_IDENTITY,
        command=QGC_START_COMMAND,
        token=17,
    )

    try:
        for _ in range(80):
            connection.incoming.append(_parse(command_bytes))
            owner.poll(0)

        assert host.poll_start(run_state=None).command_allowed is False
        assert protocol.read_start_request() is not None
        assert len(protocol.read_qgc_command_statuses()) == 65
    finally:
        protocol.close()
        connection.close()
