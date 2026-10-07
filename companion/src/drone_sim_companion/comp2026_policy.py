"""Fail-closed policy derived from the checked-in Comp2026 simulation."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from types import MappingProxyType
from typing import Callable, Mapping

import yaml

from drone import timebase
from drone.common_types import MissionHome
from drone.control.flight_profile import FlightProfile, TelemetryRequest
from drone.control.flight_state import RCModeBand, SourceIdentity
from drone.control.listener_runtime import (
    AutopilotVersionContract,
    TelemetryStartupPolicy,
)
from drone.control.mission_supervisor import RecoveryPolicy
from drone.control.stability import ReleaseStabilityConfig
from drone.mock_mission import PrecisionMissionPolicy
from drone.sensors.lidar.clearance import ClearanceCalibration


ARDUPILOT_REVISION = "1511f27194f1dcc3728270883047bdf022b3fd53"
HOME = MissionHome(37.4003371, -122.0800351, 0.195)
EARTH_RADIUS_M = 6_378_137.0
EXPECTED_SELECTORS = {
    "mission": "comp2026_auto",
    "vehicle": "iris_competition",
    "scenario": "competition_v1",
}
EXPECTED_PARAMETERS = MappingProxyType(
    {
        "LAND_SPD_MS": 0.50,
        "PLND_ENABLED": 1,
        "PLND_TYPE": 1,
        "PLND_LAG": 0.08,
        "PLND_EST_TYPE": 0,
        "PLND_XY_DIST_MAX": 0.50,
        "PLND_STRICT": 2,
        "PLND_RET_MAX": 1,
        "PLND_TIMEOUT": 0.50,
        "PLND_ALT_MIN": 0.75,
        "PLND_ALT_MAX": 8.0,
        "PLND_OPTIONS": 4,
        "SERIAL0_PROTOCOL": 2,
        "FLTMODE_CH": 7,
        "FLTMODE1": 0,
        "FLTMODE4": 4,
        "FLTMODE6": 5,
    }
)
BASE_PARAMETER_NAMES = frozenset(EXPECTED_PARAMETERS) - {
    "FLTMODE_CH", "FLTMODE1", "FLTMODE4", "FLTMODE6"
}


class SimulationAutopilotVersionContract(AutopilotVersionContract):
    """Exact version contract for the pinned Copter 4.7 SITL image only."""

    def __post_init__(self) -> None:
        if self.firmware_label != "ArduCopter 4.7.0":
            raise ValueError("simulation firmware must be pinned ArduCopter 4.7.0")
        if self.flight_sw_version != 0x040700FF:
            raise ValueError("simulation flight_sw_version does not identify Copter 4.7.0")
        if self.flight_custom_version != ARDUPILOT_REVISION[:8].encode("ascii"):
            raise ValueError("simulation flight_custom_version does not match pinned source")
        if not isinstance(self.evidence_reference, str) or ARDUPILOT_REVISION not in self.evidence_reference:
            raise ValueError("simulation version evidence must name the pinned source")


@dataclass(frozen=True)
class CompetitionSimulationPolicy:
    world: str
    flight_profile: FlightProfile
    telemetry_policy: TelemetryStartupPolicy
    autopilot_version_contract: SimulationAutopilotVersionContract
    clearance_calibration: ClearanceCalibration
    release_stability: ReleaseStabilityConfig
    precision_policy: PrecisionMissionPolicy
    recovery_policy: RecoveryPolicy
    mission_home_check: Callable[[MissionHome], None]
    fc_home_position_tolerance_m: float
    fc_home_altitude_tolerance_m: float
    startup_timeout_s: float
    telemetry_poll_interval_s: float
    parameter_expectations: Mapping[str, float | int]
    home: MissionHome
    course_bounds: Mapping[str, tuple[float, float, float, float]]
    evidence_sha256: str


def _read_object(path: Path, label: str) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{label} is unreadable") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be an object")
    return value


def _sha256(path: Path) -> str:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError as error:
        raise ValueError(f"required simulation input is unreadable: {path}") from error


def _verify_frozen_input(configuration: Path, competition: dict, name: str) -> Path:
    if competition.get(name) != f"{name}.yaml":
        raise ValueError(f"competition {name} path is invalid")
    expected = competition.get(f"{name}_sha256")
    if not isinstance(expected, str) or re.fullmatch(r"[0-9a-f]{64}", expected) is None:
        raise ValueError(f"competition {name} hash is invalid")
    path = configuration / f"{name}.yaml"
    if _sha256(path) != expected:
        raise ValueError(f"competition {name} hash does not match frozen input")
    return path


def _load_yaml(path: Path, label: str) -> dict:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"{label} is unreadable") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _course_bounds(course: dict) -> Mapping[str, tuple[float, float, float, float]]:
    if course.get("schema_version") != 1 or course.get("origin") != "H":
        raise ValueError("course contract is unsupported")
    raw = course.get("waypoints")
    if not isinstance(raw, dict) or set(raw) != {"H", "L", "F2", "WA", "WM"}:
        raise ValueError("course waypoint contract is incomplete")
    result = {}
    for name, value in raw.items():
        if not isinstance(value, dict):
            raise ValueError("course waypoint is invalid")
        numbers = tuple(value.get(key) for key in ("x", "y", "width", "height"))
        if any(isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)) for item in numbers):
            raise ValueError("course waypoint contains a nonfinite value")
        x, y, width, height = map(float, numbers)
        if width <= 0 or height <= 0:
            raise ValueError("course waypoint dimensions must be positive")
        result[name] = (x, y, width, height)
    if result["H"][:2] != (0.0, 0.0):
        raise ValueError("course home must be the local origin")
    return MappingProxyType(result)


def _parameter_file(path: Path, expected_names: frozenset[str]) -> dict[str, float]:
    values: dict[str, float] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise ValueError("SITL parameter policy is unreadable") from error
    for line in lines:
        content = line.split("#", 1)[0].strip()
        if not content:
            continue
        name, raw = content.split()
        values[name] = float(raw)
    for name in expected_names:
        expected = EXPECTED_PARAMETERS[name]
        if name not in values or not math.isclose(values[name], float(expected), rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(f"SITL parameter {name} does not match competition policy")
    return values


def build_competition_policy(
    run_config_path: Path, *, clock: Callable[[], float]
) -> CompetitionSimulationPolicy:
    if clock is not timebase.monotonic:
        raise ValueError("competition policy requires the shared timebase monotonic clock")
    path = Path(run_config_path).resolve()
    document = _read_object(path, "resolved run configuration")
    startup_timeout = document.get("startup_wall_seconds")
    if isinstance(startup_timeout, bool) or not isinstance(startup_timeout, int) or startup_timeout <= 0:
        raise ValueError("resolved startup_wall_seconds must be a positive integer")
    for field, expected in EXPECTED_SELECTORS.items():
        if document.get(field) != expected:
            raise ValueError(f"resolved {field} does not select the competition simulation")
    world = document.get("world")
    if world not in {"competition_mission", "competition_mission_1x"}:
        raise ValueError("resolved world does not select the competition simulation")
    competition = document.get("competition")
    if not isinstance(competition, dict):
        raise ValueError("resolved competition inputs are missing")
    course_path = _verify_frozen_input(path.parent, competition, "course")
    scenario_path = _verify_frozen_input(path.parent, competition, "scenario")
    root = Path(__file__).resolve().parents[3]
    for name, frozen in (("course", course_path), ("scenario", scenario_path)):
        if _sha256(frozen) != _sha256(root / f"config/{name}.yaml"):
            raise ValueError(f"frozen {name} does not match the checked-in {name} contract")
    course = _load_yaml(course_path, "course")
    scenario = _load_yaml(scenario_path, "scenario")
    bounds = _course_bounds(course)
    if scenario.get("schema_version") != 1 or scenario.get("seed") != 2026:
        raise ValueError("scenario contract is unsupported")

    provenance_path = root / "ardupilot_sitl/provenance/ardupilot.json"
    provenance = _read_object(provenance_path, "ArduPilot provenance")
    if provenance.get("tag") != "Copter-4.7.0" or provenance.get("revision") != ARDUPILOT_REVISION:
        raise ValueError("ArduPilot provenance does not match competition policy")
    parameter_path = root / "ardupilot_sitl/params/descent.parm"
    _parameter_file(parameter_path, BASE_PARAMETER_NAMES)
    competition_parameter_path = root / "ardupilot_sitl/params/competition.parm"
    _parameter_file(
        competition_parameter_path,
        frozenset(("FLTMODE_CH", "FLTMODE1", "FLTMODE4", "FLTMODE6")),
    )
    model_path = root / "gazebo/resources/models/iris_competition/model.sdf"
    world_path = root / f"gazebo/resources/worlds/{world}.sdf"
    evidence = hashlib.sha256()
    for frozen in (
        course_path, scenario_path, provenance_path, parameter_path,
        competition_parameter_path, model_path, world_path,
    ):
        evidence.update(frozen.read_bytes())
    evidence_sha256 = evidence.hexdigest()

    def mission_home_check(home: MissionHome) -> None:
        if not isinstance(home, MissionHome) or not all(
            isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))
            for value in (getattr(home, "lat", None), getattr(home, "lon", None), getattr(home, "amsl_m", None))
        ):
            raise ValueError("mission home must contain finite values")
        north = math.radians(home.lat - HOME.lat) * EARTH_RADIUS_M
        east = math.radians(home.lon - HOME.lon) * EARTH_RADIUS_M * math.cos(math.radians(HOME.lat))
        _, _, width, height = bounds["H"]
        if abs(east) > width / 2 or abs(north) > height / 2 or not 0.0 <= home.amsl_m <= 0.5:
            raise ValueError("mission home is outside the configured home zone")

    def recovery_check(_operation: str, home: MissionHome, altitude: float | None) -> None:
        mission_home_check(home)
        if altitude is not None and (
            isinstance(altitude, bool) or not isinstance(altitude, (int, float))
            or not math.isfinite(float(altitude)) or not 0 <= float(altitude) <= 40
        ):
            raise ValueError("recovery altitude is outside the simulated sensor volume")

    freshness = MappingProxyType({
        "heartbeat": 2.0, "mode": 2.0, "location": 0.5, "velocity": 0.5,
        "attitude": 0.5, "landed_state": 0.5, "armed": 2.0, "home": 2.0,
        "rc_input": 0.5, "range": 0.5, "failsafe": 2.0,
    })
    identity = SourceIdentity(1, 191)
    fc = SourceIdentity(1, 1)
    requests = tuple(
        TelemetryRequest(message_id, 1_000_000 if message_id == 0 else 50_000)
        for message_id in (0, 1, 30, 33, 65, 132, 242, 245)
    )
    flight_profile = FlightProfile(
        profile_id="competition-sitl-copter-4.7-v1", raw_sha256=evidence_sha256,
        firmware="ArduCopter 4.7.0", qgc_source=identity, companion_target=identity,
        flight_controller=fc, freshness_bounds=freshness, rc_health_max_age=0.5,
        rc_channel=7, rc_mode_bands=(
            RCModeBand("stabilize", 801, 1230, "STABILIZE"),
            RCModeBand("companion", 1491, 1620, "GUIDED"),
            RCModeBand("pilot", 1750, 2199, "LOITER"),
        ),
        heartbeat_type=2, heartbeat_autopilot=3,
        copter_modes=MappingProxyType({0: "STABILIZE", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}),
        failsafe_active_statuses=frozenset((5,)), failsafe_clear_statuses=frozenset((3, 4)),
        decoder_contract_version=1, decoder_contract_evidence=f"sha256:{evidence_sha256}",
        decoder_contract_reference=f"ArduPilot {ARDUPILOT_REVISION} simulation telemetry",
        telemetry_requests=requests,
    )
    telemetry = TelemetryStartupPolicy(60.0, 120.0, 5.0, 0.05, 2, 0.25)
    version = SimulationAutopilotVersionContract(
        "ArduCopter 4.7.0", 0x040700FF, b"1511f271",
        f"pinned SITL source {ARDUPILOT_REVISION}; ArduCopter/version.h FIRMWARE_VERSION 4,7,0,OFFICIAL",
    )
    clearance = ClearanceCalibration((0.0, 0.0, 1.0), (0.3, 0.0, -0.1), True, math.radians(8), 0.5, 0.05, True)
    release = ReleaseStabilityConfig(2.0, 10.0, 0.10, 0.10, math.radians(8), math.radians(8), 0.15, 0.15, 0.05, 0.10, 0.05, 0.20)
    precision = PrecisionMissionPolicy(
        clearance, timebase.monotonic, 0.25, 0.05, 0.05, 0.05, 0.05,
        5.0, 0.10, 0.05, 4.572, 0.20, 0.20, 1.0, 10.0, 10.0,
    )
    recovery = RecoveryPolicy(recovery_check, 60.0, 10.0, timebase.monotonic)
    return CompetitionSimulationPolicy(
        world, flight_profile, telemetry, version, clearance, release, precision,
        recovery, mission_home_check, 2.286, 0.05, float(startup_timeout), 0.05,
        EXPECTED_PARAMETERS, HOME, bounds, evidence_sha256,
    )


__all__ = [
    "CompetitionSimulationPolicy",
    "SimulationAutopilotVersionContract",
    "build_competition_policy",
]
