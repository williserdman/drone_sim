"""Bounded real-UDP peer for proving ArduPilot's JSON lockstep seam."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import socket
import struct
from typing import Callable


class FrameSequenceError(RuntimeError):
    pass


class ResponseSendError(TimeoutError):
    pass


@dataclass(frozen=True)
class ServoFrame:
    frame_rate_hz: int
    frame_count: int
    pwm: tuple[int, ...]
    retransmission: bool


_PACKETS = {
    40: (18458, struct.Struct("<HHI16H")),
    72: (29569, struct.Struct("<HHI32H")),
}


def _validate_timestamp(value: object) -> None:
    try:
        invalid = not math.isfinite(value) or value < 0  # type: ignore[arg-type]
    except (TypeError, ValueError):
        invalid = True
    if invalid:
        raise ValueError("sim_timestamp must be nonnegative and finite")


class JsonPeer:
    """Accept servo frames and return one inert, lockstep sensor sample."""

    def __init__(self, host: str, port: int) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((host, port))
        self._next_frame = 0
        self._last_sim_timestamp: float | None = None

    @property
    def address(self) -> tuple[str, int]:
        host, port = self._socket.getsockname()
        return str(host), int(port)

    def __enter__(self) -> "JsonPeer":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()

    def close(self) -> None:
        self._socket.close()

    def exchange_once(
        self,
        *,
        timeout_seconds: float,
        sim_timestamp: float | Callable[[ServoFrame], float],
    ) -> ServoFrame:
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive and finite")
        if not callable(sim_timestamp):
            _validate_timestamp(sim_timestamp)
        self._socket.settimeout(timeout_seconds)
        payload, sender = self._socket.recvfrom(4096)
        packet = _PACKETS.get(len(payload))
        if packet is None:
            raise ValueError("unsupported servo packet size")
        expected_magic, layout = packet
        values = layout.unpack(payload)
        magic, frame_rate, frame_count = values[:3]
        if magic != expected_magic or frame_rate <= 0:
            raise ValueError("invalid servo packet header")
        retransmission = self._next_frame > 0 and frame_count == self._next_frame - 1
        if frame_count != self._next_frame and not retransmission:
            raise FrameSequenceError(
                f"noncontiguous servo frame: expected {self._next_frame}, received {frame_count}"
            )
        frame = ServoFrame(frame_rate, frame_count, tuple(values[3:]), retransmission)
        if retransmission:
            assert self._last_sim_timestamp is not None
            response_timestamp = self._last_sim_timestamp
        else:
            response_timestamp = (
                sim_timestamp(frame) if callable(sim_timestamp) else sim_timestamp
            )
            _validate_timestamp(response_timestamp)
        sensor = {
            "timestamp": response_timestamp,
            "imu": {"gyro": [0.0, 0.0, 0.0], "accel_body": [0.0, 0.0, -9.80665]},
            "position": [0.0, 0.0, 0.0],
            "quaternion": [1.0, 0.0, 0.0, 0.0],
            "velocity": [0.0, 0.0, 0.0],
            "no_time_sync": True,
            "no_lockstep": False,
        }
        encoded = json.dumps(sensor, allow_nan=False, separators=(",", ":")).encode() + b"\n"
        try:
            self._socket.sendto(encoded, sender)
        except OSError as error:
            raise ResponseSendError(f"JSON response send failed: {error}") from error
        if not retransmission:
            self._last_sim_timestamp = response_timestamp
            self._next_frame += 1
        return frame
