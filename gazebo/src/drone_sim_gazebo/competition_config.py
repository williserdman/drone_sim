"""Strict immutable loader for the Comp2026 physical asset generator."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping

import yaml


INCH_TO_METERS = 0.0254
POUND_TO_KILOGRAMS = 0.45359237
MILLIMETER_TO_METERS = 0.001
WAYPOINT_NAMES = frozenset({"H", "L", "F2", "WA", "WM"})
WAYPOINT_ROLES = {
    "H": "home",
    "L": "landing",
    "F2": "fire",
    "WA": "autonomous_pickup",
    "WM": "manual_pickup",
}
PAYLOAD_COLORS = frozenset({"red", "yellow", "blue"})
PROFILES = frozenset({"competition", "search_delivery"})


class CompetitionConfigError(ValueError):
    """The committed course or scenario is not safe to generate."""


@dataclass(frozen=True)
class Waypoint:
    name: str
    x_m: float
    y_m: float
    width_m: float
    height_m: float
    role: str


@dataclass(frozen=True)
class Attempt:
    duration_seconds: int
    acquisition_agl_m: float
    transit_agl_m: float
    release_agl_m: float


@dataclass(frozen=True)
class CourseConfig:
    waypoints: Mapping[str, Waypoint]
    attempt: Attempt


@dataclass(frozen=True)
class Camera:
    width_px: int
    height_px: int
    update_rate_hz: float
    horizontal_fov_rad: float
    body_position_m: tuple[float, float, float]


@dataclass(frozen=True)
class RangeSensor:
    update_rate_hz: float


@dataclass(frozen=True)
class ImageGeometry:
    width_px: int
    height_px: int


@dataclass(frozen=True)
class PayloadInteraction:
    pickup_max_center_error_m: float
    settle_position_tolerance_m: float
    settle_time_s: float


@dataclass(frozen=True)
class PayloadGeometry:
    size_m: tuple[float, float, float]
    mass_kg: float
    marker_size_m: float


@dataclass(frozen=True)
class Payload:
    aruco_id: int
    color: str
    initial: str


@dataclass(frozen=True)
class MissionCycle:
    pickup_zone: str
    color: str
    marker_id: int
    drop_zone: str


@dataclass(frozen=True)
class SearchMission:
    aruco_id: int
    pickup_zone: str
    drop_zone: str
    search_start_xy: tuple[float, float]


@dataclass(frozen=True)
class ScenarioConfig:
    seed: int
    payload_capacity: int
    camera: Camera
    range_sensor: RangeSensor
    interaction: PayloadInteraction
    payload_geometry: PayloadGeometry
    payloads: tuple[Payload, ...]
    fm2_drop_zone: str | None
    fm3_cycles: tuple[MissionCycle, ...]
    search_mission: SearchMission | None = None
    observer_camera: ImageGeometry | None = None


def _profile(value: object) -> str:
    if value not in PROFILES:
        raise CompetitionConfigError(
            f"profile must be one of {sorted(PROFILES)}"
        )
    return str(value)


def _read_mapping(path: Path) -> dict[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise CompetitionConfigError(f"unable to read {path}: {error}") from error
    if not isinstance(value, dict):
        raise CompetitionConfigError(f"{path} root must be a mapping")
    return value


def _mapping(value: object, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise CompetitionConfigError(f"{label} must be a mapping")
    return value


def _require_keys(mapping: Mapping[str, object], expected: set[str], label: str) -> None:
    actual = set(mapping)
    if actual != expected:
        raise CompetitionConfigError(
            f"{label} keys must be exactly {sorted(expected)}; got {sorted(actual)}"
        )


def _finite(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise CompetitionConfigError(f"{label} must be numeric") from error
    if not math.isfinite(result):
        raise CompetitionConfigError(f"{label} must be finite")
    return result


def world_xy(course: CourseConfig, name: str) -> tuple[float, float]:
    """Return a waypoint in the single H-relative Gazebo world frame."""
    try:
        point = course.waypoints[name]
        home = course.waypoints["H"]
    except KeyError as error:
        raise CompetitionConfigError(f"unknown waypoint {name!r}") from error
    return point.x_m - home.x_m, point.y_m - home.y_m


def load_course(path: Path, *, profile: str = "competition") -> CourseConfig:
    profile = _profile(profile)
    raw = _read_mapping(path)
    _require_keys(raw, {"schema_version", "units", "origin", "waypoints", "attempt"}, "course")
    if raw["schema_version"] != 1:
        raise CompetitionConfigError("course schema_version must be 1")
    if raw["units"] != "meters":
        raise CompetitionConfigError("course units must be meters")
    if raw["origin"] != "H":
        raise CompetitionConfigError("course origin must be H")

    waypoint_raw = _mapping(raw["waypoints"], "waypoints")
    if set(waypoint_raw) != WAYPOINT_NAMES:
        raise CompetitionConfigError(
            f"waypoints must be exactly {sorted(WAYPOINT_NAMES)}"
        )
    waypoints: dict[str, Waypoint] = {}
    for name in sorted(WAYPOINT_NAMES):
        entry = _mapping(waypoint_raw[name], f"waypoint {name}")
        _require_keys(entry, {"x", "y", "width", "height", "role"}, f"waypoint {name}")
        point = Waypoint(
            name=name,
            x_m=_finite(entry["x"], f"waypoint {name} x"),
            y_m=_finite(entry["y"], f"waypoint {name} y"),
            width_m=_finite(entry["width"], f"waypoint {name} width"),
            height_m=_finite(entry["height"], f"waypoint {name} height"),
            role=str(entry["role"]),
        )
        if point.width_m <= 0 or point.height_m <= 0:
            raise CompetitionConfigError(f"waypoint {name} footprint must be positive")
        if point.role != WAYPOINT_ROLES[name]:
            raise CompetitionConfigError(
                f"waypoint {name} role must be {WAYPOINT_ROLES[name]}"
            )
        waypoints[name] = point
    if (waypoints["H"].x_m, waypoints["H"].y_m) != (0.0, 0.0):
        raise CompetitionConfigError("H must be the zero-valued source origin")

    attempt_raw = _mapping(raw["attempt"], "attempt")
    _require_keys(
        attempt_raw,
        {"duration_seconds", "acquisition_agl_m", "transit_agl_m", "release_agl_m"},
        "attempt",
    )
    try:
        duration_seconds = int(attempt_raw["duration_seconds"])
    except (TypeError, ValueError) as error:
        raise CompetitionConfigError("attempt duration_seconds must be an integer") from error
    attempt = Attempt(
        duration_seconds=duration_seconds,
        acquisition_agl_m=_finite(attempt_raw["acquisition_agl_m"], "acquisition AGL"),
        transit_agl_m=_finite(attempt_raw["transit_agl_m"], "transit AGL"),
        release_agl_m=_finite(attempt_raw["release_agl_m"], "release AGL"),
    )
    expected_duration = 240 if profile == "search_delivery" else 600
    if attempt.duration_seconds != expected_duration:
        raise CompetitionConfigError(
            f"attempt duration_seconds must be {expected_duration}"
        )
    if min(attempt.acquisition_agl_m, attempt.transit_agl_m, attempt.release_agl_m) <= 0:
        raise CompetitionConfigError("attempt AGL values must be positive")
    if profile == "search_delivery":
        expected_waypoints = {
            "H": (0.0, 0.0, 4.572, 4.572),
            "L": (-8.0, 6.0, 4.572, 4.572),
            "F2": (6.0, 20.0, 0.9144, 0.9144),
            "WA": (18.0, 8.0, 6.096, 6.096),
            "WM": (18.0, -8.0, 6.096, 6.096),
        }
        actual_waypoints = {
            name: (point.x_m, point.y_m, point.width_m, point.height_m)
            for name, point in waypoints.items()
        }
        if actual_waypoints != expected_waypoints:
            raise CompetitionConfigError(
                "search_delivery waypoints must match the approved compact course"
            )
        if (
            attempt.acquisition_agl_m,
            attempt.transit_agl_m,
            attempt.release_agl_m,
        ) != (4.572, 10.0, 10.0):
            raise CompetitionConfigError(
                "search_delivery attempt heights must match the approved profile"
            )
    return CourseConfig(MappingProxyType(waypoints), attempt)


def load_scenario(
    path: Path,
    course: CourseConfig,
    *,
    profile: str = "competition",
) -> ScenarioConfig:
    profile = _profile(profile)
    raw = _read_mapping(path)
    scenario_keys = {
            "schema_version",
            "seed",
            "vehicle",
            "camera",
            "range_sensor",
            "payload_interaction",
            "payload_geometry",
            "payloads",
            "mission",
    }
    if profile == "search_delivery":
        scenario_keys.add("observer_camera")
    _require_keys(raw, scenario_keys, "scenario")
    if raw["schema_version"] != 1:
        raise CompetitionConfigError("scenario schema_version must be 1")

    vehicle_raw = _mapping(raw["vehicle"], "vehicle")
    _require_keys(vehicle_raw, {"payload_capacity"}, "vehicle")
    try:
        payload_capacity = int(vehicle_raw["payload_capacity"])
    except (TypeError, ValueError) as error:
        raise CompetitionConfigError("payload capacity must be an integer") from error
    if payload_capacity != 1:
        raise CompetitionConfigError("payload capacity must be 1")

    camera_raw = _mapping(raw["camera"], "camera")
    _require_keys(
        camera_raw,
        {"width_px", "height_px", "update_rate_hz", "horizontal_fov_rad", "body_position_m"},
        "camera",
    )
    position = camera_raw["body_position_m"]
    if not isinstance(position, list) or len(position) != 3:
        raise CompetitionConfigError("camera body_position_m must have three values")
    try:
        width_px = int(camera_raw["width_px"])
        height_px = int(camera_raw["height_px"])
    except (TypeError, ValueError) as error:
        raise CompetitionConfigError("camera dimensions must be integers") from error
    camera = Camera(
        width_px=width_px,
        height_px=height_px,
        update_rate_hz=_finite(camera_raw["update_rate_hz"], "camera update rate"),
        horizontal_fov_rad=_finite(camera_raw["horizontal_fov_rad"], "camera horizontal FOV"),
        body_position_m=tuple(_finite(value, "camera body position") for value in position),
    )
    if (camera.width_px, camera.height_px, camera.update_rate_hz) != (640, 480, 20.0):
        raise CompetitionConfigError("competition camera must be 640x480 at 20 Hz")
    if not 0 < camera.horizontal_fov_rad < math.pi:
        raise CompetitionConfigError("camera horizontal FOV must be between zero and pi")

    observer_camera: ImageGeometry | None = None
    if profile == "search_delivery":
        observer_raw = _mapping(raw["observer_camera"], "observer_camera")
        _require_keys(observer_raw, {"width_px", "height_px"}, "observer_camera")
        try:
            observer_camera = ImageGeometry(
                int(observer_raw["width_px"]),
                int(observer_raw["height_px"]),
            )
        except (TypeError, ValueError) as error:
            raise CompetitionConfigError(
                "observer camera dimensions must be integers"
            ) from error
        if observer_camera != ImageGeometry(1280, 960):
            raise CompetitionConfigError(
                "search_delivery observer camera must be exactly 1280x960"
            )

    range_raw = _mapping(raw["range_sensor"], "range_sensor")
    _require_keys(range_raw, {"update_rate_hz"}, "range_sensor")
    range_sensor = RangeSensor(
        update_rate_hz=_finite(range_raw["update_rate_hz"], "range update rate")
    )
    if range_sensor.update_rate_hz != 20.0:
        raise CompetitionConfigError("competition range sensor must be 20 Hz")

    interaction_raw = _mapping(raw["payload_interaction"], "payload_interaction")
    _require_keys(
        interaction_raw,
        {"pickup_max_center_error_m", "settle_position_tolerance_m", "settle_time_s"},
        "payload_interaction",
    )
    interaction = PayloadInteraction(
        pickup_max_center_error_m=_finite(
            interaction_raw["pickup_max_center_error_m"], "pickup center error"
        ),
        settle_position_tolerance_m=_finite(
            interaction_raw["settle_position_tolerance_m"], "settle position tolerance"
        ),
        settle_time_s=_finite(interaction_raw["settle_time_s"], "settle time"),
    )
    if min(
        interaction.pickup_max_center_error_m,
        interaction.settle_position_tolerance_m,
        interaction.settle_time_s,
    ) <= 0:
        raise CompetitionConfigError("payload interaction values must be positive")

    geometry_raw = _mapping(raw["payload_geometry"], "payload_geometry")
    _require_keys(geometry_raw, {"size_in", "mass_lb", "marker_size_mm"}, "payload_geometry")
    size_in = geometry_raw["size_in"]
    if not isinstance(size_in, list) or len(size_in) != 3:
        raise CompetitionConfigError("payload size_in must have three values")
    dimensions = tuple(_finite(value, "payload dimension") for value in size_in)
    mass_lb = _finite(geometry_raw["mass_lb"], "payload mass")
    marker_size_mm = _finite(geometry_raw["marker_size_mm"], "marker size")
    if min(*dimensions, mass_lb, marker_size_mm) <= 0:
        raise CompetitionConfigError("payload geometry values must be positive")
    geometry = PayloadGeometry(
        size_m=tuple(round(value * INCH_TO_METERS, 12) for value in dimensions),
        mass_kg=round(mass_lb * POUND_TO_KILOGRAMS, 12),
        marker_size_m=round(marker_size_mm * MILLIMETER_TO_METERS, 12),
    )
    if geometry.marker_size_m > min(geometry.size_m[:2]):
        raise CompetitionConfigError("payload marker must fit on the top face")
    if profile == "search_delivery" and (
        camera
        != Camera(
            width_px=640,
            height_px=480,
            update_rate_hz=20.0,
            horizontal_fov_rad=0.60,
            body_position_m=(0.0, 0.0, -0.10),
        )
        or interaction.pickup_max_center_error_m != 0.075
        or geometry
        != PayloadGeometry(
            size_m=(0.1524, 0.1524, 0.0508),
            mass_kg=1.133980925,
            marker_size_m=0.1,
        )
    ):
        raise CompetitionConfigError(
            "search_delivery sensors and payload must match the approved calibration"
        )

    payload_raw = raw["payloads"]
    if not isinstance(payload_raw, list):
        raise CompetitionConfigError("payloads must be a list")
    payloads: list[Payload] = []
    for index, value in enumerate(payload_raw):
        entry = _mapping(value, f"payload {index}")
        _require_keys(entry, {"aruco_id", "color", "initial"}, f"payload {index}")
        try:
            marker_id = int(entry["aruco_id"])
        except (TypeError, ValueError) as error:
            raise CompetitionConfigError("payload ArUco ID must be an integer") from error
        payloads.append(Payload(marker_id, str(entry["color"]).lower(), str(entry["initial"])))
    payload_semantics = [(item.aruco_id, item.color, item.initial) for item in payloads]
    expected_payloads = (
        [(3, "yellow", "WA")]
        if profile == "search_delivery"
        else [
            (2, "red", "attached"),
            (3, "yellow", "WA"),
            (4, "blue", "WM"),
        ]
    )
    if payload_semantics != expected_payloads:
        raise CompetitionConfigError(
            "search_delivery payload inventory must be exact ID 3"
            if profile == "search_delivery"
            else "payload inventory must be exact IDs 2, 3, and 4"
        )
    if any(item.color not in PAYLOAD_COLORS for item in payloads):
        raise CompetitionConfigError("payload color is unsupported")

    mission_raw = _mapping(raw["mission"], "mission")
    cycles: list[MissionCycle] = []
    search_mission: SearchMission | None = None
    if profile == "search_delivery":
        _require_keys(
            mission_raw,
            {"aruco_id", "pickup_zone", "drop_zone", "search_start"},
            "mission",
        )
        search_start = _mapping(mission_raw["search_start"], "mission search_start")
        _require_keys(search_start, {"x", "y"}, "mission search_start")
        try:
            mission_aruco_id = int(mission_raw["aruco_id"])
        except (TypeError, ValueError) as error:
            raise CompetitionConfigError("mission aruco_id must be an integer") from error
        search_mission = SearchMission(
            aruco_id=mission_aruco_id,
            pickup_zone=str(mission_raw["pickup_zone"]),
            drop_zone=str(mission_raw["drop_zone"]),
            search_start_xy=(
                _finite(search_start["x"], "mission search_start x"),
                _finite(search_start["y"], "mission search_start y"),
            ),
        )
        if search_mission != SearchMission(3, "WA", "F2", (16.0, 8.0)):
            raise CompetitionConfigError(
                "search_delivery mission must target payload 3 from WA to F2"
            )
        fm2_drop_zone = None
    else:
        _require_keys(mission_raw, {"fm2_drop_zone", "fm3_cycles"}, "mission")
        fm2_drop_zone = str(mission_raw["fm2_drop_zone"])
        if fm2_drop_zone != "F2":
            raise CompetitionConfigError("FM2 drop zone must be F2")
        cycle_raw = mission_raw["fm3_cycles"]
        if not isinstance(cycle_raw, list):
            raise CompetitionConfigError("fm3_cycles must be a list")
        for index, value in enumerate(cycle_raw):
            entry = _mapping(value, f"FM3 cycle {index}")
            _require_keys(entry, {"pickup_zone", "color", "drop_zone"}, f"FM3 cycle {index}")
            pickup = str(entry["pickup_zone"])
            color = str(entry["color"]).lower()
            drop = str(entry["drop_zone"])
            matches = [item for item in payloads if item.initial == pickup and item.color == color]
            if len(matches) != 1 or drop not in course.waypoints:
                raise CompetitionConfigError(f"FM3 cycle {index} does not resolve one payload")
            cycles.append(MissionCycle(pickup, color, matches[0].aruco_id, drop))
        if [(item.pickup_zone, item.color, item.marker_id, item.drop_zone) for item in cycles] != [
            ("WA", "yellow", 3, "F2"),
            ("WM", "blue", 4, "F2"),
        ]:
            raise CompetitionConfigError("FM3 cycles must be WA/yellow then WM/blue")

    try:
        seed = int(raw["seed"])
    except (TypeError, ValueError) as error:
        raise CompetitionConfigError("scenario seed must be an integer") from error
    return ScenarioConfig(
        seed=seed,
        payload_capacity=payload_capacity,
        camera=camera,
        range_sensor=range_sensor,
        interaction=interaction,
        payload_geometry=geometry,
        payloads=tuple(payloads),
        fm2_drop_zone=fm2_drop_zone,
        fm3_cycles=tuple(cycles),
        search_mission=search_mission,
        observer_camera=observer_camera,
    )
