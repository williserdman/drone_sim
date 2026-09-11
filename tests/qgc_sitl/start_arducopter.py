#!/usr/bin/env python3
"""Resolve the owned JSON peer once, record argv, then exec Copter 4.5.7."""

from __future__ import annotations

import ipaddress
import hashlib
import json
import os
from pathlib import Path
import socket
import time


def resolve_peer_ipv4(name: str, *, wall_timeout_s: float = 15) -> str:
    deadline = time.monotonic() + wall_timeout_s
    last_error: OSError | None = None
    while time.monotonic() < deadline:
        try:
            address = socket.gethostbyname(name)
            parsed = ipaddress.IPv4Address(address)
            if not parsed.is_private or parsed.is_unspecified or parsed.is_loopback:
                raise OSError("peer did not resolve to an owned-network IPv4 address")
            return str(parsed)
        except OSError as error:
            last_error = error
            time.sleep(0.1)
    raise TimeoutError(f"could not resolve {name} to IPv4: {last_error}")


def argv(peer_ipv4: str) -> tuple[str, ...]:
    ipaddress.IPv4Address(peer_ipv4)
    return (
        "/opt/ardupilot/bin/arducopter",
        "--model", "JSON",
        "--speedup", "1",
        "--sim-address", peer_ipv4,
        "--sim-port-in", "9003",
        "--sim-port-out", "9002",
        "--serial0", "tcp:5760",
        "--serial1", "tcp:5762",
        "--defaults", "/opt/qgc-sitl/listener.parm",
        "--home", "37.4003371,-122.0800351,0,0",
        "--wipe",
    )


def launcher_sha256() -> str:
    launcher = Path(__file__).resolve()
    return hashlib.sha256(launcher.read_bytes()).hexdigest()


def startup_evidence(peer_ipv4: str) -> dict[str, object]:
    return {
        "status": "exec",
        "argv": argv(peer_ipv4),
        "launcher_sha256": launcher_sha256(),
    }


def main() -> int:
    output = Path("/result/arducopter/startup.json")
    try:
        peer_ipv4 = resolve_peer_ipv4("listener-probe")
        command = argv(peer_ipv4)
        output.write_text(json.dumps(startup_evidence(peer_ipv4), indent=2) + "\n")
        os.execv(command[0], command)
    except BaseException as error:
        output.write_text(json.dumps({
            "status": "failed",
            "stage": "peer-resolution",
            "reason": f"{type(error).__name__}: {error}",
            "launcher_sha256": launcher_sha256(),
        }, indent=2) + "\n")
        return 1
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
