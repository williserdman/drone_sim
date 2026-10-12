"""Validated runtime configuration and shell-free ArduCopter command."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
from ipaddress import IPv4Address
from pathlib import Path
import socket
from typing import Callable
from uuid import UUID


MOVING_PAD_PARAMETERS = Path("/opt/drone_sim/ardupilot/params/moving-pad.parm")
COMPETITION_PARAMETERS = Path("/opt/drone_sim/ardupilot/params/competition.parm")
CALIBRATION_PARAMETERS = (
    "ATC_ANG_RLL_P",
    "ATC_RAT_RLL_P",
    "ATC_RAT_RLL_I",
    "ATC_RAT_RLL_D",
    "ATC_ACC_R_MAX",
    "ATC_ANG_PIT_P",
    "ATC_RAT_PIT_P",
    "ATC_RAT_PIT_I",
    "ATC_RAT_PIT_D",
    "ATC_ACC_P_MAX",
    "ATC_ANG_YAW_P",
    "ATC_RAT_YAW_P",
    "ATC_RAT_YAW_I",
    "ATC_RAT_YAW_FLTE",
    "ATC_ACC_Y_MAX",
)


def _verified_calibration(document: dict[str, object], run_directory: Path) -> Path | None:
    calibration = document.get("calibration_json")
    if calibration is None:
        return None
    if not isinstance(calibration, dict):
        raise ValueError("frozen calibration_json is malformed")
    gains = calibration.get("gains")
    digest = calibration.get("source_artifact_sha256")
    if not isinstance(gains, dict) or set(gains) != set(CALIBRATION_PARAMETERS):
        raise ValueError("frozen calibration must contain exactly 15 gain parameters")
    path = run_directory / "configuration/calibration.parm"
    try:
        contents = path.read_bytes()
    except OSError as error:
        raise ValueError("frozen calibration artifact is unreadable") from error
    if (
        not isinstance(digest, str)
        or hashlib.sha256(contents).hexdigest() != digest
    ):
        raise ValueError("calibration artifact checksum does not match frozen configuration")
    parsed: dict[str, float] = {}
    try:
        for raw_line in contents.decode("utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            name, raw_value = line.split()
            if name in parsed:
                raise ValueError("duplicate calibration parameter")
            parsed[name] = float(raw_value)
    except (UnicodeDecodeError, ValueError) as error:
        raise ValueError("frozen calibration artifact is malformed") from error
    if set(parsed) != set(CALIBRATION_PARAMETERS) or parsed != gains:
        raise ValueError("calibration artifact gains do not match frozen configuration")
    return path


def parameter_files_from_environment(
    environment: Mapping[str, str],
    *,
    run_id: str,
    run_directory: Path,
) -> tuple[Path | None, Path | None]:
    raw_path = environment.get("SIM_CONFIG_PATH")
    if raw_path is None:
        return None, None
    path = Path(raw_path)
    if path != run_directory / "configuration/run.json":
        raise ValueError("SIM_CONFIG_PATH must be the run's frozen configuration")
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError("SIM_CONFIG_PATH must contain readable JSON") from error
    if not isinstance(document, dict) or document.get("run_id") != run_id:
        raise ValueError("frozen configuration run_id does not match SIM_RUN_ID")
    if "calibration_json" in document:
        expected_checksum = document.get("config_sha256")
        without_checksum = {
            key: value for key, value in document.items() if key != "config_sha256"
        }
        actual_checksum = hashlib.sha256(
            json.dumps(
                without_checksum, sort_keys=True, separators=(",", ":")
            ).encode()
        ).hexdigest()
        if expected_checksum != actual_checksum:
            raise ValueError(
                "config_sha256 does not match the frozen configuration"
            )
    overlay = {
        "moving_pad_v1": MOVING_PAD_PARAMETERS,
        "competition_v1": COMPETITION_PARAMETERS,
    }.get(document.get("scenario"))
    return overlay, _verified_calibration(document, run_directory)


def parameter_overlay_from_environment(
    environment: Mapping[str, str],
    *,
    run_id: str,
    run_directory: Path,
) -> Path | None:
    return parameter_files_from_environment(
        environment, run_id=run_id, run_directory=run_directory,
    )[0]


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
    parameter_overlay_file: Path | None = None
    calibration_file: Path | None = None
    gazebo_host: str = "gazebo-runtime"
    gazebo_port: int = 9002
    gazebo_input_port: int = 9003
    mavlink_port: int = 5760
    operator_mavlink_port: int = 5762
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
        if not isinstance(self.parameter_file, Path):
            object.__setattr__(self, "parameter_file", Path(self.parameter_file))
        if self.parameter_overlay_file is not None and not isinstance(
            self.parameter_overlay_file, Path
        ):
            object.__setattr__(
                self, "parameter_overlay_file", Path(self.parameter_overlay_file)
            )
        if self.calibration_file is not None and not isinstance(
            self.calibration_file, Path
        ):
            object.__setattr__(self, "calibration_file", Path(self.calibration_file))
        if not self.gazebo_host or any(character.isspace() for character in self.gazebo_host):
            raise ValueError("gazebo_host must be a nonempty DNS name or address")
        _port(self.gazebo_port, "gazebo_port")
        _port(self.gazebo_input_port, "gazebo_input_port")
        _port(self.mavlink_port, "mavlink_port")
        _port(self.operator_mavlink_port, "operator_mavlink_port")
        if self.operator_mavlink_port == self.mavlink_port:
            raise ValueError("operator_mavlink_port must differ from mavlink_port")

    @property
    def argv(self) -> tuple[str, ...]:
        defaults = str(self.parameter_file)
        if self.parameter_overlay_file is not None:
            defaults += f",{self.parameter_overlay_file}"
        if self.calibration_file is not None:
            defaults += f",{self.calibration_file}"
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
            "--serial1",
            f"tcp:{self.operator_mavlink_port}",
            "--defaults",
            defaults,
            "--home",
            self.home,
            "--wipe",
        )
