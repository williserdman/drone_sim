from __future__ import annotations

import json
import queue
import socket
import threading
import time

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from orchestration._adapters.posix_host import _PosixDatagramTransport
from orchestration.gcs_bridge import GcsBridge
from orchestration.host_integration import HostTransportError


def _frame(message, *, system: int, component: int) -> bytes:
    return message.pack(
        mavlink.MAVLink(None, srcSystem=system, srcComponent=component)
    )


def _listener() -> socket.socket:
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen()
    listener.settimeout(3)
    return listener


def _wait_until(predicate, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("condition did not become true")
        time.sleep(0.005)


class QueuedTransport:
    def __init__(self) -> None:
        self.incoming: queue.Queue[bytes | BaseException] = queue.Queue()
        self.sent: list[bytes] = []
        self.closed = False

    def send(self, payload: bytes) -> None:
        if self.closed:
            raise HostTransportError("transport closed")
        self.sent.append(payload)

    def receive(self, timeout_s: float) -> bytes | None:
        if self.closed:
            raise HostTransportError("transport closed")
        try:
            item = self.incoming.get(timeout=timeout_s)
        except queue.Empty:
            return None
        if isinstance(item, BaseException):
            raise item
        return item

    def close(self) -> None:
        self.closed = True


class BarrierTransport(QueuedTransport):
    def __init__(self, payload: bytes) -> None:
        super().__init__()
        self.payload = payload
        self.receiving = threading.Event()
        self.release = threading.Event()
        self.delivered = False

    def receive(self, timeout_s: float) -> bytes | None:
        if self.delivered:
            return super().receive(timeout_s)
        self.receiving.set()
        if not self.release.wait(timeout_s):
            return None
        self.delivered = True
        return self.payload

    def close(self) -> None:
        super().close()
        self.release.set()


class FailThenReplayTransport(QueuedTransport):
    """Expose a command if the bridge touches a failed transport again."""

    def __init__(self) -> None:
        super().__init__()
        self.failed = threading.Event()
        self.receive_calls = 0

    def receive(self, timeout_s: float) -> bytes | None:
        self.receive_calls += 1
        if self.receive_calls == 1:
            self.failed.set()
            raise HostTransportError("helper EOF")
        try:
            item = self.incoming.get(timeout=timeout_s)
        except queue.Empty:
            return None
        assert isinstance(item, bytes)
        return item


def test_single_owner_relays_exact_frames_and_records_each_intervention_once(tmp_path):
    listener = _listener()
    qgc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    qgc.bind(("127.0.0.1", 0))
    qgc.settimeout(3)
    transport = _PosixDatagramTransport(qgc.getsockname()[1])
    bridge = GcsBridge(
        tmp_path,
        transport=transport,
        tcp_port=listener.getsockname()[1],
    )
    bridge.start()
    connection, _ = listener.accept()
    connection.settimeout(3)

    controller_frames = [
        _frame(
            mavlink.MAVLink_heartbeat_message(2, 3, 129, 4, 3, 3),
            system=1,
            component=1,
        ),
        _frame(
            mavlink.MAVLink_global_position_int_message(
                1, 0, 0, 0, 12_500, 0, 0, 0, 0
            ),
            system=1,
            component=1,
        ),
        _frame(
            mavlink.MAVLink_extended_sys_state_message(0, 2),
            system=1,
            component=1,
        ),
    ]
    first = controller_frames[0]
    connection.sendall(first[:5])
    qgc.settimeout(0.05)
    with pytest.raises(socket.timeout):
        qgc.recvfrom(4096)
    qgc.settimeout(3)
    connection.sendall(first[5:] + b"".join(controller_frames[1:]))
    relayed = [qgc.recvfrom(4096) for _frame_bytes in controller_frames]
    assert [payload for payload, _peer in relayed] == controller_frames
    peer = relayed[-1][1]

    _wait_until(lambda: "landed_state" in bridge.status())
    controller_status = bridge.status()
    assert controller_status["mode"] == "GUIDED"
    assert controller_status["armed"] is True
    assert controller_status["relative_altitude_m"] == 12.5
    assert controller_status["landed_state"] == 2
    controller_freshness = controller_status["last_telemetry_wall_time"]

    foreign_frames = []
    for system, component in ((42, 1), (1, 191)):
        foreign_frames.extend(
            (
                _frame(
                    mavlink.MAVLink_heartbeat_message(2, 3, 0, 0, 3, 3),
                    system=system,
                    component=component,
                ),
                _frame(
                    mavlink.MAVLink_global_position_int_message(
                        2, 0, 0, 0, 999_000, 0, 0, 0, 0
                    ),
                    system=system,
                    component=component,
                ),
                _frame(
                    mavlink.MAVLink_extended_sys_state_message(0, 1),
                    system=system,
                    component=component,
                ),
            )
        )
    foreign_observations = b"".join(foreign_frames)
    connection.sendall(foreign_observations)
    assert b"".join(
        qgc.recvfrom(4096)[0] for _ in foreign_frames
    ) == foreign_observations
    filtered_status = bridge.status()
    assert filtered_status["mode"] == "GUIDED"
    assert filtered_status["armed"] is True
    assert filtered_status["relative_altitude_m"] == 12.5
    assert filtered_status["landed_state"] == 2
    assert filtered_status["last_telemetry_wall_time"] == controller_freshness

    command = _frame(
        mavlink.MAVLink_command_long_message(
            1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0
        ),
        system=255,
        component=190,
    )
    manual = _frame(
        mavlink.MAVLink_manual_control_message(1, 0, 0, 0, 0, 0),
        system=255,
        component=190,
    )
    foreign = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    foreign.sendto(command, peer)
    qgc.sendto(command + manual, peer)
    received = b""
    while len(received) < len(command + manual):
        received += connection.recv(4096)
    assert received == command + manual

    intervention_path = tmp_path / "logs/docker/qgc-interventions.jsonl"
    _wait_until(lambda: intervention_path.exists())
    bridge.stop()
    records = [json.loads(line) for line in intervention_path.read_text().splitlines()]
    assert [record["message_type"] for record in records] == [
        "COMMAND_LONG",
        "MANUAL_CONTROL",
    ]
    assert [record["source_system"] for record in records] == [255, 255]
    assert bridge.status()["connected"] is False
    assert bridge.status()["qgc_connected"] is False

    foreign.close()
    connection.close()
    listener.close()
    qgc.close()


def test_finalize_discards_command_already_buffered_in_transport(tmp_path):
    listener = _listener()
    command = _frame(
        mavlink.MAVLink_command_long_message(
            1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0
        ),
        system=255,
        component=190,
    )
    transport = BarrierTransport(command)
    bridge = GcsBridge(
        tmp_path,
        transport=transport,
        tcp_port=listener.getsockname()[1],
    )
    bridge.start()
    connection, _ = listener.accept()
    connection.settimeout(0.2)

    assert transport.receiving.wait(2)
    finalization = tmp_path / ".control/finalize-request.json"
    finalization.parent.mkdir()
    finalization.write_text("{}", encoding="utf-8")
    transport.release.set()

    with pytest.raises(socket.timeout):
        connection.recv(4096)
    assert not (tmp_path / "logs/docker/qgc-interventions.jsonl").exists()

    bridge.stop()
    connection.close()
    listener.close()


def test_finalize_is_rechecked_immediately_before_tcp_write(tmp_path):
    listener = _listener()
    command = _frame(
        mavlink.MAVLink_command_long_message(
            1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0
        ),
        system=255,
        component=190,
    )
    transport = QueuedTransport()
    bridge = GcsBridge(
        tmp_path,
        transport=transport,
        tcp_port=listener.getsockname()[1],
    )
    persisted = threading.Event()
    release = threading.Event()
    original_record = bridge._record_intervention

    def record_then_wait(message) -> None:
        original_record(message)
        persisted.set()
        assert release.wait(2)

    bridge._record_intervention = record_then_wait
    bridge.start()
    connection, _ = listener.accept()
    connection.settimeout(0.2)
    transport.incoming.put(command)
    assert persisted.wait(2)

    finalization = tmp_path / ".control/finalize-request.json"
    finalization.parent.mkdir()
    finalization.write_text("{}", encoding="utf-8")
    release.set()

    with pytest.raises(socket.timeout):
        connection.recv(4096)
    bridge.stop()
    connection.close()
    listener.close()


def test_stop_discards_queued_command_and_transport_is_not_replayed_on_recovery(tmp_path):
    listener = _listener()
    stale_command = _frame(
        mavlink.MAVLink_command_long_message(
            1, 1, 176, 0, 1, 0, 0, 0, 0, 0, 0
        ),
        system=255,
        component=190,
    )
    stale = BarrierTransport(stale_command)
    first = GcsBridge(
        tmp_path,
        transport=stale,
        tcp_port=listener.getsockname()[1],
    )
    first.start()
    first_connection, _ = listener.accept()
    first_connection.settimeout(0.2)
    assert stale.receiving.wait(2)
    first.stop()
    with pytest.raises((socket.timeout, ConnectionError)):
        data = first_connection.recv(4096)
        if data == b"":
            raise ConnectionError("closed")
    assert stale.closed is True

    replacement = QueuedTransport()
    second = GcsBridge(
        tmp_path,
        transport=replacement,
        tcp_port=listener.getsockname()[1],
    )
    second.start()
    second_connection, _ = listener.accept()
    second_connection.settimeout(0.2)
    with pytest.raises(socket.timeout):
        second_connection.recv(4096)

    second.stop()
    first_connection.close()
    second_connection.close()
    listener.close()


def test_transport_failure_marks_qgc_disconnected_but_keeps_observing_sitl(tmp_path):
    listener = _listener()
    transport = QueuedTransport()
    transport.incoming.put(HostTransportError("helper EOF"))
    bridge = GcsBridge(
        tmp_path,
        transport=transport,
        tcp_port=listener.getsockname()[1],
    )
    bridge.start()
    connection, _ = listener.accept()

    first = _frame(
        mavlink.MAVLink_heartbeat_message(2, 3, 0, 0, 3, 3),
        system=1,
        component=1,
    )
    second = _frame(
        mavlink.MAVLink_heartbeat_message(2, 3, 128, 4, 3, 3),
        system=1,
        component=1,
    )
    connection.sendall(first)
    _wait_until(lambda: bridge.status()["qgc_connected"] is False)
    connection.sendall(second)
    _wait_until(lambda: bridge.status().get("armed") is True)

    status = bridge.status()
    assert status["connected"] is True
    assert status["qgc_connected"] is False
    assert "helper EOF" in status["qgc_detail"]

    bridge.stop()
    connection.close()
    listener.close()


def test_failed_transport_stays_disabled_across_sitl_reconnect(tmp_path):
    listener = _listener()
    transport = FailThenReplayTransport()
    bridge = GcsBridge(
        tmp_path,
        transport=transport,
        tcp_port=listener.getsockname()[1],
    )
    bridge.start()
    first_connection, _ = listener.accept()
    assert transport.failed.wait(2)
    assert bridge.status()["qgc_connected"] is False

    first_connection.close()
    replacement_connection, _ = listener.accept()
    replacement_connection.settimeout(0.2)
    command = _frame(
        mavlink.MAVLink_command_long_message(
            1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0
        ),
        system=255,
        component=190,
    )
    transport.incoming.put(command)

    with pytest.raises(socket.timeout):
        replacement_connection.recv(4096)
    assert transport.receive_calls == 1

    bridge.stop()
    replacement_connection.close()
    listener.close()
