"""Validated runtime configuration and shell-free ArduCopter command."""

from __future__ import annotations

from dataclasses import dataclass
from ipaddress import IPv4Address
from pathlib import Path
import socket
from typing import Callable
from uuid import UUID


def _port(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 65535:
        raise ValueError(f"{name} must be an integer port")
    return value


def resolve_gazebo_address(
    service_name: str, *, resolver: Callable[[str], str] = socket.gethostbyname
) -> str:
    """Resolve Docker DNS for ArduPilot's numeric-only native UDP socket."""
    try:
        return str(IPv4Address(resolver(service_name)))
    except (OSError, ValueError) as error:
        raise ValueError("Gazebo service must resolve to an IPv4 address") from error


@dataclass(frozen=True)
class RuntimeConfig:
    run_id: str
    run_directory: Path
    executable: Path = Path("/opt/ardupilot/bin/arducopter")
    parameter_file: Path = Path("/opt/drone_sim/ardupilot/params/descent.parm")
    gazebo_host: str = "gazebo-runtime"
    gazebo_port: int = 9002
    gazebo_input_port: int = 9003
    mavlink_port: int = 5760
    home: str = "37.4003371,-122.0800351,0,0"

    def __post_init__(self) -> None:
        try:
            canonical = str(UUID(self.run_id))
        except (ValueError, AttributeError) as error:
            raise ValueError("run_id must be a canonical UUID") from error
        if self.run_id != canonical:
            raise ValueError("run_id must be a canonical UUID")
        if not isinstance(self.run_directory, Path):
            object.__setattr__(self, "run_directory", Path(self.run_directory))
        if not self.gazebo_host or any(character.isspace() for character in self.gazebo_host):
            raise ValueError("gazebo_host must be a nonempty DNS name or address")
        _port(self.gazebo_port, "gazebo_port")
        _port(self.gazebo_input_port, "gazebo_input_port")
        _port(self.mavlink_port, "mavlink_port")

    @property
    def argv(self) -> tuple[str, ...]:
        return (
            str(self.executable),
            "--model",
            "JSON",
            "--speedup",
            "1",
            "--sim-address",
            self.gazebo_host,
            "--sim-port-in",
            str(self.gazebo_input_port),
            "--sim-port-out",
            str(self.gazebo_port),
            "--serial0",
            f"tcp:0.0.0.0:{self.mavlink_port}",
            "--defaults",
            str(self.parameter_file),
            "--home",
            self.home,
            "--wipe",
        )
