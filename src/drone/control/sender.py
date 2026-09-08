"""Explicit synthetic MAVLink STATUSTEXT sender for bench diagnostics."""

from __future__ import annotations

import argparse
import time
from collections.abc import Callable, Iterable
from typing import Any


SYNTHETIC_PREFIX = "[SYNTHETIC TEST]"


class ReceiverIdentityError(RuntimeError):
    """Raised when the heartbeat source is not the requested test receiver."""


def _load_mavutil():
    from pymavlink import mavutil

    return mavutil


def start_sender(
    *,
    endpoint: str,
    source_system: int,
    source_component: int,
    expected_receiver_system: int,
    expected_receiver_component: int,
    heartbeat_timeout_seconds: float,
    messages: Iterable[str] = ("bench connectivity pulse",),
    interval_seconds: float = 5.0,
    iterations: int | None = None,
    mavutil_module: Any = None,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """Send visibly synthetic text after validating a received heartbeat packet."""
    if not endpoint:
        raise ValueError("endpoint must be explicit and non-empty")
    if heartbeat_timeout_seconds <= 0:
        raise ValueError("heartbeat_timeout_seconds must be positive")
    if iterations is not None and iterations < 0:
        raise ValueError("iterations must be non-negative or None")

    mavutil = mavutil_module or _load_mavutil()
    transport = mavutil.mavlink_connection(
        endpoint,
        source_system=source_system,
        source_component=source_component,
    )
    try:
        heartbeat = transport.wait_heartbeat(timeout=heartbeat_timeout_seconds)
        if heartbeat is None:
            raise TimeoutError("timed out waiting for receiver heartbeat")
        actual = (heartbeat.get_srcSystem(), heartbeat.get_srcComponent())
        expected = (expected_receiver_system, expected_receiver_component)
        if actual != expected:
            raise ReceiverIdentityError(
                f"received heartbeat from {actual[0]}:{actual[1]}, "
                f"expected {expected[0]}:{expected[1]}"
            )

        payloads = tuple(
            f"{SYNTHETIC_PREFIX} {message}".encode("utf-8") for message in messages
        )
        completed = 0
        while iterations is None or completed < iterations:
            for payload in payloads:
                transport.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_INFO, payload
                )
            completed += 1
            if iterations is None or completed < iterations:
                sleep(interval_seconds)
    finally:
        transport.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", required=True)
    parser.add_argument("--source-system", required=True, type=int)
    parser.add_argument("--source-component", required=True, type=int)
    parser.add_argument("--receiver-system", required=True, type=int)
    parser.add_argument("--receiver-component", required=True, type=int)
    parser.add_argument("--heartbeat-timeout", required=True, type=float)
    parser.add_argument("--message", required=True, action="append")
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--iterations", type=int)
    args = parser.parse_args(argv)
    start_sender(
        endpoint=args.endpoint,
        source_system=args.source_system,
        source_component=args.source_component,
        expected_receiver_system=args.receiver_system,
        expected_receiver_component=args.receiver_component,
        heartbeat_timeout_seconds=args.heartbeat_timeout,
        messages=args.message,
        interval_seconds=args.interval,
        iterations=args.iterations,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
