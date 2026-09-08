import collections
import collections.abc
import queue
from types import SimpleNamespace

import pytest

if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping
from dronekit.mavlink import MAVWriter
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from drone.control.flight_state import SourceIdentity
from drone.control.listener import (
    ACK_ACCEPTED,
    CommandAck,
    DroneKitQGCAckTransport,
    FM1,
    QGCCommandListener,
)
from test_listener import Packet, profile, supervisor


class InertDroneKitVehicle:
    def __init__(self, dialect, *, wire_protocol, source=(1, 191), fc_system=1):
        self.out_queue = queue.Queue()
        encoder = dialect.MAVLink(
            MAVWriter(self.out_queue),
            srcSystem=source[0],
            srcComponent=source[1],
        )
        original_send = encoder.send

        def dronekit_send(message, *args, **kwargs):
            if hasattr(message, "target_system"):
                message.target_system = fc_system
            return original_send(message, *args, **kwargs)

        encoder.send = dronekit_send
        self._master = SimpleNamespace(
            mav=encoder,
            WIRE_PROTOCOL_VERSION=wire_protocol,
        )
        self.listeners = {}

    def add_message_listener(self, name, callback):
        self.listeners[name] = callback

    def send_mavlink(self, message):
        self._master.mav.send(message)


def decode_packet(dialect, packet):
    decoder = dialect.MAVLink(None)
    return decoder.parse_char(packet)


def test_v2_ack_uses_actual_dronekit_queue_without_fc_target_rewrite():
    vehicle = InertDroneKitVehicle(mavlink2, wire_protocol="2.0")
    transport = DroneKitQGCAckTransport(
        vehicle,
        expected_source=SourceIdentity(1, 191),
        wire_protocol="2.0",
    )

    transport.send_command_ack(
        CommandAck(
            command=FM1,
            result=ACK_ACCEPTED,
            progress=0,
            target_system=200,
            target_component=190,
        )
    )

    packet = vehicle.out_queue.get_nowait()
    decoded = decode_packet(mavlink2, packet)
    assert packet[0] == 0xFD
    assert (decoded.get_srcSystem(), decoded.get_srcComponent()) == (1, 191)
    assert (decoded.target_system, decoded.target_component) == (200, 190)
    assert decoded.command == FM1
    assert decoded.result == ACK_ACCEPTED


def test_installed_listener_queues_envelope_and_encodes_targeted_progress_ack():
    vehicle = InertDroneKitVehicle(mavlink2, wire_protocol="2.0")
    transport = DroneKitQGCAckTransport(
        vehicle,
        expected_source=SourceIdentity(1, 191),
        wire_protocol="2.0",
    )
    commands = queue.Queue()
    listener = QGCCommandListener(
        supervisor=supervisor(),
        profile=profile(),
        command_queue=commands,
        clock=lambda: 12.5,
        ack_transport=transport,
        wire_protocol="2.0",
    )
    listener.install()

    vehicle.listeners["COMMAND_LONG"](vehicle, "COMMAND_LONG", Packet())

    assert commands.get_nowait() == listener.supervisor.reserved_envelope(FM1)
    decoded = decode_packet(mavlink2, vehicle.out_queue.get_nowait())
    assert (decoded.get_srcSystem(), decoded.get_srcComponent()) == (1, 191)
    assert (decoded.target_system, decoded.target_component) == (200, 190)
    assert decoded.command == FM1
    assert decoded.result == 5


def test_harness_proves_plain_dronekit_send_would_retarget_ack_to_fc():
    vehicle = InertDroneKitVehicle(mavlink2, wire_protocol="2.0", fc_system=1)
    message = vehicle._master.mav.command_ack_encode(
        FM1,
        ACK_ACCEPTED,
        0,
        0,
        200,
        190,
    )

    vehicle.send_mavlink(message)

    decoded = decode_packet(mavlink2, vehicle.out_queue.get_nowait())
    assert (decoded.target_system, decoded.target_component) == (1, 190)


def test_v1_ack_has_actual_source_and_no_extension_targets():
    vehicle = InertDroneKitVehicle(mavlink1, wire_protocol="1.0")
    transport = DroneKitQGCAckTransport(
        vehicle,
        expected_source=SourceIdentity(1, 191),
        wire_protocol="1.0",
    )

    transport.send_command_ack(
        CommandAck(
            command=FM1,
            result=ACK_ACCEPTED,
            progress=0,
            target_system=None,
            target_component=None,
        )
    )

    packet = vehicle.out_queue.get_nowait()
    decoded = decode_packet(mavlink1, packet)
    assert packet[0] == 0xFE
    assert (decoded.get_srcSystem(), decoded.get_srcComponent()) == (1, 191)
    assert not hasattr(decoded, "target_system")


def test_transport_rejects_declared_source_or_wire_not_proved_by_encoder():
    vehicle = InertDroneKitVehicle(mavlink2, wire_protocol="2.0")

    with pytest.raises(ValueError, match="actual MAVLink source"):
        DroneKitQGCAckTransport(
            vehicle,
            expected_source=SourceIdentity(1, 192),
            wire_protocol="2.0",
        )
    with pytest.raises(ValueError, match="actual MAVLink wire protocol"):
        DroneKitQGCAckTransport(
            vehicle,
            expected_source=SourceIdentity(1, 191),
            wire_protocol="1.0",
        )


def test_transport_installs_dronekit_callback_adapter_without_io():
    vehicle = InertDroneKitVehicle(mavlink2, wire_protocol="2.0")
    transport = DroneKitQGCAckTransport(
        vehicle,
        expected_source=SourceIdentity(1, 191),
        wire_protocol="2.0",
    )
    received = []

    transport.install_message_callback(
        ("COMMAND_LONG", "COMMAND_INT"), received.append
    )
    packet = object()
    vehicle.listeners["COMMAND_LONG"](vehicle, "COMMAND_LONG", packet)

    assert received == [packet]
    assert vehicle.out_queue.empty()


def test_transport_requires_actual_dronekit_mavwriter_queue():
    vehicle = InertDroneKitVehicle(mavlink2, wire_protocol="2.0")
    vehicle._master.mav.file = SimpleNamespace(write=lambda _packet: None)

    with pytest.raises(ValueError, match="MAVWriter"):
        DroneKitQGCAckTransport(
            vehicle,
            expected_source=SourceIdentity(1, 191),
            wire_protocol="2.0",
        )
