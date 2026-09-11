"""Validated runtime configuration and shell-free ArduCopter command."""

from __future__ import annotations

from dataclasses import dataclass
from ipaddress import IPv4Address
import math
from pathlib import Path
import socket
from typing import Callable
from uuid import UUID


def _port(value: int, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 65535:
        raise ValueError(f"{name} must be an integer port")
    return value


def _finite_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


@dataclass(frozen=True)
class LaunchOrigin:
    latitude_deg: float
    longitude_deg: float
    amsl_m: float
    heading_deg: float

    def __post_init__(self) -> None:
        for name in ("latitude_deg", "longitude_deg", "amsl_m", "heading_deg"):
            object.__setattr__(self, name, _finite_number(getattr(self, name), name))
        if not -90.0 <= self.latitude_deg <= 90.0:
            raise ValueError("latitude_deg must be in [-90, 90]")
        if not -180.0 <= self.longitude_deg <= 180.0:
            raise ValueError("longitude_deg must be in [-180, 180]")
        if not 0.0 <= self.heading_deg < 360.0:
            raise ValueError("heading_deg must be in [0, 360)")

    @property
    def ardupilot_home(self) -> str:
        return ",".join(
            format(value, ".15g")
            for value in (
                self.latitude_deg,
                self.longitude_deg,
                self.amsl_m,
                self.heading_deg,
            )
        )


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
    launch_origin: LaunchOrigin
    executable: Path = Path("/opt/ardupilot/bin/arducopter")
    parameter_file: Path = Path("/opt/drone_sim/ardupilot/params/descent.parm")
    gazebo_host: str = "gazebo-runtime"
    gazebo_port: int = 9002
    gazebo_input_port: int = 9003
    mavlink_port: int = 5760

    def __post_init__(self) -> None:
        try:
            canonical = str(UUID(self.run_id))
        except (ValueError, AttributeError) as error:
            raise ValueError("run_id must be a canonical UUID") from error
        if self.run_id != canonical:
            raise ValueError("run_id must be a canonical UUID")
        if not isinstance(self.run_directory, Path):
            object.__setattr__(self, "run_directory", Path(self.run_directory))
        if not isinstance(self.launch_origin, LaunchOrigin):
            raise ValueError("launch_origin must be a LaunchOrigin")
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
            f"tcp:{self.mavlink_port}",
            "--defaults",
            str(self.parameter_file),
            "--home",
            self.launch_origin.ardupilot_home,
            "--wipe",
        )
