"""Frozen VFS 2027 configuration shared by runtime and evidence consumers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

import yaml


VFS_PROFILE = "competition_2027_v1"
VFS_PAYLOAD_IDS = (2, 3, 4)
XY = tuple[float, float]


@dataclass(frozen=True)
class VfsWaypoint:
    name: str
    x: float
    y: float
    role: str
    width: float | None = None
    height: float | None = None
    radius: float | None = None

    @property
    def center_xy_m(self) -> XY:
        return self.x, self.y

    def contains(self, xy: XY, margin: float = 0.0) -> bool:
        if self.radius is not None:
            return math.hypot(xy[0] - self.x, xy[1] - self.y) + margin <= self.radius + 1e-9
        if self.width is not None and self.height is not None:
            return (abs(xy[0] - self.x) + margin <= self.width / 2 + 1e-9
                    and abs(xy[1] - self.y) + margin <= self.height / 2 + 1e-9)
        return math.hypot(xy[0] - self.x, xy[1] - self.y) <= 1e-9 and margin == 0


@dataclass(frozen=True)
class VfsAttempt:
    duration_seconds: float
    acquisition_agl_m: float
    transit_agl_m: float
    release_agl_m: float
    minimum_agl_m: float
    inspection_timeout_s: float
    approval_timeout_s: float
    initial_wait_timeout_s: float
    recovery_timeout_s: float
    ground_contact_epsilon_m: float
    aircraft_radius_m: float


@dataclass(frozen=True)
class VfsCourse:
    waypoints: Mapping[str, VfsWaypoint]
    attempt: VfsAttempt
    ground_z_m: float = 0.0


@dataclass(frozen=True)
class VfsPayloadGeometry:
    size_m: tuple[float, float, float]
    water_mass_kg: float
    tare_mass_kg: float
    marker_size_m: float

    @property
    def total_mass_kg(self) -> float:
        return self.water_mass_kg + self.tare_mass_kg


@dataclass(frozen=True)
class VfsCamera:
    width_px: int
    height_px: int
    update_rate_hz: int
    horizontal_fov_rad: float
    body_position_m: tuple[float, float, float]


@dataclass(frozen=True)
class VfsRangeSensor:
    update_rate_hz: int


@dataclass(frozen=True)
class VfsPayloadInteraction:
    pickup_max_center_error_m: float
    settle_position_tolerance_m: float
    settle_time_s: float


@dataclass(frozen=True)
class VfsPayload:
    aruco_id: int
    color: str
    initial: str
    delivery: str
    release_offset_xy_m: XY


@dataclass(frozen=True)
class VfsScenario:
    seed: int
    aircraft_empty_mass_kg: float
    camera: VfsCamera
    range_sensor: VfsRangeSensor
    payload_interaction: VfsPayloadInteraction
    payload_geometry: VfsPayloadGeometry
    payloads: tuple[VfsPayload, ...]
    targets: Mapping[int, str]
    assumptions: Mapping[str, bool]
    payload_capacity: int = 1

    def payload(self, aruco_id: int) -> VfsPayload:
        for payload in self.payloads:
            if payload.aruco_id == aruco_id:
                return payload
        raise ValueError(f"unknown payload ID: {aruco_id}")


def _object(value: object, fields: set[str], name: str) -> dict:
    if not isinstance(value, dict) or set(value) != fields:
        raise ValueError(f"{name} has missing or unknown fields")
    return value


def _number(value: object, name: str, *, positive: bool = False) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    result = float(value)
    if positive and result <= 0:
        raise ValueError(f"{name} must be positive")
    return result


def _vector(value: object, name: str, length: int, *, positive: bool = False) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must have {length} coordinates")
    return tuple(_number(item, name, positive=positive) for item in value)


def _read(path: str | Path) -> dict:
    try:
        value = yaml.safe_load(Path(path).read_bytes())
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"invalid VFS configuration: {path}") from exc
    if not isinstance(value, dict):
        raise ValueError("VFS configuration must be an object")
    return value


def validate_vfs_course(document: object) -> VfsCourse:
    doc = _object(document, {"schema_version", "profile", "units", "origin", "waypoints", "attempt"}, "course")
    if type(doc["schema_version"]) is not int or doc["schema_version"] != 1 or doc["profile"] != VFS_PROFILE:
        raise ValueError("unsupported VFS course profile/version")
    if doc["units"] != "meters" or doc["origin"] != "H":
        raise ValueError("VFS course uses metres and H origin")
    raw = doc["waypoints"]
    required = {"H", "S", "D", "L", "A", "B", "C", "P2", "P3", "P4", "L2", "L3", "L4"}
    if not isinstance(raw, dict) or not required <= set(raw):
        raise ValueError("VFS course is missing required waypoints")
    waypoints: dict[str, VfsWaypoint] = {}
    for name, fields in raw.items():
        if not isinstance(name, str) or not name or not isinstance(fields, dict):
            raise ValueError("invalid VFS waypoint")
        extras = set(fields) - {"x", "y", "role"}
        if not {"x", "y", "role"} <= set(fields) or extras not in (set(), {"radius"}, {"width", "height"}):
            raise ValueError(f"invalid waypoint geometry: {name}")
        if not isinstance(fields["role"], str) or not fields["role"]:
            raise ValueError("waypoint role must be nonempty")
        waypoints[name] = VfsWaypoint(name, _number(fields["x"], name), _number(fields["y"], name),
                                     fields["role"],
                                     _number(fields["width"], name, positive=True) if "width" in fields else None,
                                     _number(fields["height"], name, positive=True) if "height" in fields else None,
                                     _number(fields["radius"], name, positive=True) if "radius" in fields else None)
    for name in ("H", "A", "B", "C"):
        if waypoints[name].radius is None:
            raise ValueError(f"{name} must be circular")
    for name in ("S", "D", "L"):
        if waypoints[name].width is None:
            raise ValueError(f"{name} must be rectangular")
    if waypoints["H"].center_xy_m != (0.0, 0.0) or not waypoints["S"].contains((0.0, 0.0), waypoints["H"].radius or 0):
        raise ValueError("H must be the origin and fully inside S")
    for name in ("A", "B", "C"):
        if not waypoints["D"].contains(waypoints[name].center_xy_m, waypoints[name].radius or 0):
            raise ValueError(f"{name} must be fully inside D")
    attempt_fields = set(VfsAttempt.__dataclass_fields__)
    attempt_doc = _object(doc["attempt"], attempt_fields, "attempt")
    attempt = VfsAttempt(**{name: _number(value, name, positive=True) for name, value in attempt_doc.items()})
    if attempt.duration_seconds != 600 or attempt.minimum_agl_m != 9.144:
        raise ValueError("VFS flight limit must be 600 s and minimum altitude 9.144 m")
    if min(attempt.transit_agl_m, attempt.release_agl_m) < attempt.minimum_agl_m:
        raise ValueError("transit and release heights must meet the minimum altitude")
    return VfsCourse(MappingProxyType(waypoints), attempt)


def validate_vfs_scenario(document: object) -> VfsScenario:
    doc = _object(document, {"schema_version", "profile", "seed", "vehicle", "camera", "range_sensor", "payload_interaction", "payload_geometry", "payloads", "mission", "assumptions"}, "scenario")
    if type(doc["schema_version"]) is not int or doc["schema_version"] != 1 or doc["profile"] != VFS_PROFILE:
        raise ValueError("unsupported VFS scenario profile/version")
    if type(doc["seed"]) is not int or not 0 <= doc["seed"] <= 0xFFFFFFFF:
        raise ValueError("scenario seed must be an unsigned 32-bit integer")
    vehicle = _object(doc["vehicle"], {"payload_capacity", "aircraft_empty_mass_kg"}, "vehicle")
    if type(vehicle["payload_capacity"]) is not int or vehicle["payload_capacity"] != 1:
        raise ValueError("VFS payload capacity must be one")
    camera_doc = _object(doc["camera"], set(VfsCamera.__dataclass_fields__), "camera")
    if any(type(camera_doc[key]) is not int or camera_doc[key] != expected
           for key, expected in (("width_px", 640), ("height_px", 480), ("update_rate_hz", 20))):
        raise ValueError("VFS camera must be 640x480 at 20 Hz")
    body = _vector(camera_doc["body_position_m"], "camera body position", 3)
    camera = VfsCamera(640, 480, 20, _number(camera_doc["horizontal_fov_rad"], "camera FOV", positive=True),
                       (body[0], body[1], body[2]))
    if camera.horizontal_fov_rad >= math.pi:
        raise ValueError("camera FOV must be below pi")
    range_doc = _object(doc["range_sensor"], {"update_rate_hz"}, "range sensor")
    if type(range_doc["update_rate_hz"]) is not int or range_doc["update_rate_hz"] != 20:
        raise ValueError("VFS range sensor must run at 20 Hz")
    interaction_doc = _object(doc["payload_interaction"], set(VfsPayloadInteraction.__dataclass_fields__), "payload interaction")
    interaction = VfsPayloadInteraction(**{name: _number(value, name, positive=True) for name, value in interaction_doc.items()})
    geometry_doc = _object(doc["payload_geometry"], set(VfsPayloadGeometry.__dataclass_fields__), "payload geometry")
    size = _vector(geometry_doc["size_m"], "payload size", 3, positive=True)
    geometry = VfsPayloadGeometry((size[0], size[1], size[2]),
                                  _number(geometry_doc["water_mass_kg"], "water mass", positive=True),
                                  _number(geometry_doc["tare_mass_kg"], "tare mass", positive=True),
                                  _number(geometry_doc["marker_size_m"], "marker size", positive=True))
    if geometry.water_mass_kg < 1.36077711 or max(size) > 1.2192:
        raise ValueError("payload must contain at least 3 lb water and fit inside 4 ft")
    if geometry.marker_size_m != 0.1 or geometry.marker_size_m > min(size[0], size[1]):
        raise ValueError("VFS payload marker must be 100 mm and fit the container")
    raw_payloads = doc["payloads"]
    if not isinstance(raw_payloads, list) or len(raw_payloads) != 3:
        raise ValueError("VFS requires three payloads")
    payloads = []
    for raw in raw_payloads:
        item = _object(raw, set(VfsPayload.__dataclass_fields__), "payload")
        aruco_id = item["aruco_id"]
        if type(aruco_id) is not int or aruco_id not in VFS_PAYLOAD_IDS:
            raise ValueError("VFS payload IDs must be 2, 3 and 4")
        if item["color"] not in {"red", "yellow", "blue"} or any(not isinstance(item[key], str) or not item[key] for key in ("initial", "delivery")):
            raise ValueError("invalid payload color or waypoint reference")
        offset = _vector(item["release_offset_xy_m"], "release offset", 2)
        payloads.append(VfsPayload(aruco_id, item["color"], item["initial"], item["delivery"], (offset[0], offset[1])))
    if sorted(p.aruco_id for p in payloads) != list(VFS_PAYLOAD_IDS):
        raise ValueError("VFS payload IDs must be unique")
    mission = _object(doc["mission"], {"targets"}, "mission")
    if not isinstance(mission["targets"], dict):
        raise ValueError("targets must be an object")
    targets: dict[int, str] = {}
    for key, value in mission["targets"].items():
        if type(key) is int:
            payload_id = key
        elif isinstance(key, str) and key in {"2", "3", "4"}:
            payload_id = int(key)
        else:
            raise ValueError("target keys must be payload IDs")
        if payload_id not in VFS_PAYLOAD_IDS or payload_id in targets or value not in {"A", "B", "C"}:
            raise ValueError("targets must assign A, B or C once to each payload")
        targets[payload_id] = value
    if set(targets) != set(VFS_PAYLOAD_IDS):
        raise ValueError("targets must include each payload")
    assumptions = _object(doc["assumptions"], {"rigid_containers", "liquid_and_breakage_simulated", "failed_payloads_physically_removed", "ground_mission_simulated"}, "assumptions")
    if any(type(value) is not bool for value in assumptions.values()) or assumptions != {
        "rigid_containers": True, "liquid_and_breakage_simulated": False,
        "failed_payloads_physically_removed": False, "ground_mission_simulated": False,
    }:
        raise ValueError("unsupported VFS physical assumptions")
    return VfsScenario(doc["seed"], _number(vehicle["aircraft_empty_mass_kg"], "empty aircraft mass", positive=True),
                       camera, VfsRangeSensor(20), interaction, geometry, tuple(payloads),
                       MappingProxyType(targets), MappingProxyType(dict(assumptions)))


def load_vfs_course(path: str | Path) -> VfsCourse:
    return validate_vfs_course(_read(path))


def load_vfs_scenario(path: str | Path) -> VfsScenario:
    return validate_vfs_scenario(_read(path))


def resolve_waypoint_xy(course: VfsCourse, scenario: VfsScenario, *,
                        waypoint: str | None = None, payload_target: int | None = None) -> XY:
    if (waypoint is None) == (payload_target is None):
        raise ValueError("choose exactly one waypoint or payload target")
    if payload_target is not None:
        payload = scenario.payload(payload_target)
        point = course.waypoints[scenario.targets[payload_target]]
        return point.x + payload.release_offset_xy_m[0], point.y + payload.release_offset_xy_m[1]
    assert waypoint is not None
    if waypoint not in course.waypoints:
        raise ValueError(f"unknown course waypoint: {waypoint}")
    return course.waypoints[waypoint].center_xy_m


def validate_vfs_configuration(course: VfsCourse, scenario: VfsScenario) -> None:
    margin = math.hypot(*scenario.payload_geometry.size_m[:2]) / 2
    for payload in scenario.payloads:
        if payload.initial not in course.waypoints or payload.delivery not in course.waypoints:
            raise ValueError("payload references an unknown waypoint")
        initial = course.waypoints[payload.initial].center_xy_m
        delivery = course.waypoints[payload.delivery].center_xy_m
        if not course.waypoints["S"].contains(initial, margin) or course.waypoints["H"].contains(initial, -margin):
            raise ValueError("initial payloads must fit in S outside H")
        if not course.waypoints["L"].contains(delivery, margin):
            raise ValueError("delivery points must fit in L")
        target = resolve_waypoint_xy(course, scenario, payload_target=payload.aruco_id)
        if not course.waypoints[scenario.targets[payload.aruco_id]].contains(target, margin):
            raise ValueError("release offset does not fit the selected target")
