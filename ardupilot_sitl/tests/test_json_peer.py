from __future__ import annotations

import json
import socket
import struct

import pytest

from drone_sim_ardupilot.json_peer import FrameSequenceError, JsonPeer


def _servo_packet(frame_count: int, *, frame_rate_hz: int = 400) -> bytes:
    return struct.pack(
        "<HHI16H", 18458, frame_rate_hz, frame_count, *([1000] * 16)
    )


def test_peer_exchanges_real_udp_packets_and_keeps_simulation_timestamp() -> None:
    with JsonPeer("127.0.0.1", 0) as peer, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.settimeout(1.0)
        client.sendto(_servo_packet(0), peer.address)

        frame = peer.exchange_once(timeout_seconds=1.0, sim_timestamp=0.0025)
        response = json.loads(client.recv(4096))

    assert frame.frame_count == 0
    assert frame.frame_rate_hz == 400
    assert frame.pwm == (1000,) * 16
    assert response == {
        "timestamp": 0.0025,
        "imu": {"gyro": [0.0, 0.0, 0.0], "accel_body": [0.0, 0.0, -9.80665]},
        "position": [0.0, 0.0, 0.0],
        "quaternion": [1.0, 0.0, 0.0, 0.0],
        "velocity": [0.0, 0.0, 0.0],
        "no_time_sync": True,
        "no_lockstep": False,
    }


def test_peer_rejects_frame_gap_instead_of_retiming_or_repairing() -> None:
    with JsonPeer("127.0.0.1", 0) as peer, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.sendto(_servo_packet(0), peer.address)
        peer.exchange_once(timeout_seconds=1.0, sim_timestamp=0.0)
        client.sendto(_servo_packet(2), peer.address)

        with pytest.raises(FrameSequenceError, match="expected 1, received 2"):
            peer.exchange_once(timeout_seconds=1.0, sim_timestamp=0.005)


def test_peer_replies_to_upstream_retransmission_without_advancing_sim_time() -> None:
    with JsonPeer("127.0.0.1", 0) as peer, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.settimeout(1.0)
        client.sendto(_servo_packet(0), peer.address)
        first = peer.exchange_once(timeout_seconds=1.0, sim_timestamp=0.0)
        client.recv(4096)
        client.sendto(_servo_packet(0), peer.address)

        retried = peer.exchange_once(timeout_seconds=1.0, sim_timestamp=99.0)
        response = json.loads(client.recv(4096))

    assert not first.retransmission
    assert retried.retransmission
    assert response["timestamp"] == 0.0


def test_peer_selects_timestamp_from_validated_frame_and_reuses_it_on_retry() -> None:
    selected_frames = []

    def select_timestamp(frame):
        selected_frames.append(frame)
        return 1.0 / frame.frame_rate_hz

    with JsonPeer("127.0.0.1", 0) as peer, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.settimeout(1.0)
        client.sendto(_servo_packet(0, frame_rate_hz=1200), peer.address)
        first = peer.exchange_once(timeout_seconds=1.0, sim_timestamp=select_timestamp)
        first_response = json.loads(client.recv(4096))
        client.sendto(_servo_packet(0, frame_rate_hz=1200), peer.address)
        retried = peer.exchange_once(timeout_seconds=1.0, sim_timestamp=select_timestamp)
        retry_response = json.loads(client.recv(4096))

    assert first.frame_rate_hz == 1200
    assert not first.retransmission
    assert retried.retransmission
    assert len(selected_frames) == 1
    assert first_response["timestamp"] == pytest.approx(1.0 / 1200)
    assert retry_response["timestamp"] == first_response["timestamp"]


@pytest.mark.parametrize("selected", [-1.0, float("inf"), float("nan"), "not-a-time"])
def test_peer_rejects_invalid_timestamp_selected_from_frame(selected: object) -> None:
    with JsonPeer("127.0.0.1", 0) as peer, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.sendto(_servo_packet(0, frame_rate_hz=1200), peer.address)

        with pytest.raises(ValueError, match="sim_timestamp"):
            peer.exchange_once(
                timeout_seconds=1.0,
                sim_timestamp=lambda _frame: selected,
            )


def test_peer_rejects_malformed_servo_datagram() -> None:
    with JsonPeer("127.0.0.1", 0) as peer, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.sendto(b"not-a-servo-frame", peer.address)

        with pytest.raises(ValueError, match="servo packet"):
            peer.exchange_once(timeout_seconds=1.0, sim_timestamp=0.0)
