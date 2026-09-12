"""Inert projection of resolved simulator inputs into the nested QGC runtime."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
from typing import Any, Mapping


_SHA256 = re.compile(r"[0-9a-f]{64}\Z")
_QGC_NAMES = {
    "deployment_profile": "deployment-profile.json",
    "listener_session": "listener-session.json",
    "qgc_actions": "qgc-actions.json",
    "runtime_policy": "qgc-runtime.json",
}


@dataclass(frozen=True, slots=True)
class ResolvedQGCInputs:
    deployment_profile_path: Path
    deployment_profile_sha256: str
    listener_session_path: Path
    listener_session_sha256: str
    qgc_actions_path: Path
    qgc_actions_sha256: str
    runtime_policy_path: Path
    runtime_policy_sha256: str
    attempt_state_id: str
    attempt_state_root: Path
    course_sha256: str
    scenario_sha256: str


@dataclass(frozen=True, slots=True)
class QGCRuntimeProjection:
    """All inert inputs required by the nested listener and ArduPilot."""

    runtime_configuration: Any
    validated_listener_artifacts: Any
    deployment_profile: Any
    flight_profile: Any
    attempt_state: Any
    sim_launch_origin_json: str
    payload_delay_wall_timeout_s: float


@dataclass(frozen=True, slots=True)
class _ReadRegularFile:
    content: bytes
    identity: tuple[int, int]


def _digest(value: object, label: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise ValueError(f"{label} must be a lowercase SHA-256")
    return value


def canonical_attempt_state_root(value: object) -> Path:
    if not isinstance(value, str) or "\0" in value or value.startswith("//"):
        raise ValueError("attempt state root must be absolute and canonical")
    path = Path(value)
    try:
        canonical = Path(os.path.abspath(value))
    except (OSError, ValueError) as error:
        raise ValueError("attempt state root must be absolute and canonical") from error
    if not path.is_absolute() or path != canonical or path == Path("/"):
        raise ValueError("attempt state root must be absolute and canonical")
    return canonical


def _read_regular_file(path: Path, *, label: str) -> _ReadRegularFile:
    """Read an absolute canonical path without following any symlink component."""
    raw_path = os.fspath(path)
    if (
        raw_path.startswith("//")
        or not path.is_absolute()
        or path != Path(os.path.abspath(path))
    ):
        raise ValueError(f"{label} path must be absolute and canonical")
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    directory_flags = flags | getattr(os, "O_DIRECTORY", 0)
    directory_descriptors: list[int] = []
    descriptor = -1
    try:
        directory_descriptors.append(os.open("/", directory_flags))
        for component in path.parts[1:-1]:
            directory_descriptors.append(
                os.open(component, directory_flags, dir_fd=directory_descriptors[-1])
            )
        descriptor = os.open(
            path.name,
            flags | getattr(os, "O_NONBLOCK", 0),
            dir_fd=directory_descriptors[-1],
        )
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError(f"{label} must be a regular non-symlink file")
        with os.fdopen(descriptor, "rb") as source:
            descriptor = -1
            return _ReadRegularFile(
                content=source.read(), identity=(metadata.st_dev, metadata.st_ino)
            )
    except ValueError:
        raise
    except OSError as error:
        raise ValueError(f"{label} must be a readable regular non-symlink file") from error
    finally:
        if descriptor >= 0:
            try:
                os.close(descriptor)
            except OSError:
                pass
        for directory_descriptor in reversed(directory_descriptors):
            try:
                os.close(directory_descriptor)
            except OSError:
                pass


def _json_object(content: bytes, *, label: str) -> dict[str, Any]:
    def reject_constant(value: str) -> None:
        raise ValueError(f"{label} contains a non-standard JSON number {value}")

    def unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"{label} contains duplicate key {key!r}")
            result[key] = value
        return result

    try:
        value = json.loads(
            content.decode("utf-8"),
            parse_constant=reject_constant,
            object_pairs_hook=unique_object,
        )
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"{label} must contain valid UTF-8 JSON") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain a JSON object")
    return value


def read_resolved_run_document(path: Path) -> dict[str, Any]:
    """Read the canonical run document with the same fail-closed file rules."""
    opened = _read_regular_file(path, label="resolved run configuration")
    return _json_object(opened.content, label="resolved run configuration")


def resolved_qgc_inputs(
    raw: object,
    *,
    configuration_directory: Path,
    competition: Mapping[str, object],
) -> ResolvedQGCInputs:
    """Validate the closed resolved-QGC document and its copied bytes."""
    expected_fields = {
        *(_QGC_NAMES.keys()),
        *(f"{name}_sha256" for name in _QGC_NAMES),
        "attempt_state_id",
        "attempt_state_root",
    }
    if not isinstance(raw, dict) or set(raw) != expected_fields:
        raise ValueError("resolved QGC configuration has missing or unknown fields")
    values: dict[str, object] = {}
    identities: set[tuple[int, int]] = set()
    for field, filename in _QGC_NAMES.items():
        if raw[field] != filename:
            raise ValueError("resolved QGC sources must use canonical artifact names")
        path = configuration_directory / filename
        opened = _read_regular_file(path, label=f"QGC {field}")
        content = opened.content
        digest = _digest(raw[f"{field}_sha256"], f"{field}_sha256")
        if hashlib.sha256(content).hexdigest() != digest:
            raise ValueError(f"{field} bytes do not match their SHA-256")
        _json_object(content, label=f"QGC {field}")
        if opened.identity in identities:
            raise ValueError("resolved QGC source files must be distinct")
        identities.add(opened.identity)
        values[f"{field}_path"] = path
        values[f"{field}_sha256"] = digest
    profile_digest = values["deployment_profile_sha256"]
    if raw["attempt_state_id"] != f"sha256-{profile_digest}":
        raise ValueError("attempt_state_id does not match the deployment profile digest")
    return ResolvedQGCInputs(
        **values,
        attempt_state_id=raw["attempt_state_id"],
        attempt_state_root=canonical_attempt_state_root(raw["attempt_state_root"]),
        course_sha256=_digest(competition.get("course_sha256"), "course_sha256"),
        scenario_sha256=_digest(
            competition.get("scenario_sha256"), "scenario_sha256"
        ),
    )


def validate_resolved_competition(
    raw: Mapping[str, object], *, configuration_directory: Path
) -> tuple[Path, Path]:
    """Validate canonical competition paths and the exact copied bytes."""
    course_path = configuration_directory / "course.yaml"
    scenario_path = configuration_directory / "scenario.yaml"
    for label, path in (("course", course_path), ("scenario", scenario_path)):
        digest = _digest(raw.get(f"{label}_sha256"), f"{label}_sha256")
        content = _read_regular_file(path, label=f"competition {label}").content
        if hashlib.sha256(content).hexdigest() != digest:
            raise ValueError(f"competition {label} bytes do not match their SHA-256")
        _yaml_document(
            content,
            expected=_EXPECTED_COURSE if label == "course" else _EXPECTED_SCENARIO,
            label=f"competition {label}",
        )
    return course_path, scenario_path


_EXPECTED_COURSE = {
    "schema_version": 1,
    "units": "meters",
    "origin": "H",
    "waypoints": {
        "H": {
            "x": 0.0,
            "y": 0.0,
            "width": 4.572,
            "height": 4.572,
            "role": "home",
        },
        "L": {
            "x": -91.44,
            "y": 0.0,
            "width": 4.572,
            "height": 4.572,
            "role": "landing",
        },
        "F2": {
            "x": -152.40,
            "y": 0.0,
            "width": 0.9144,
            "height": 0.9144,
            "role": "fire",
        },
        "WA": {
            "x": -45.72,
            "y": -9.144,
            "width": 6.096,
            "height": 6.096,
            "role": "autonomous_pickup",
        },
        "WM": {
            "x": -45.72,
            "y": 9.144,
            "width": 6.096,
            "height": 6.096,
            "role": "manual_pickup",
        },
    },
    "attempt": {
        "duration_seconds": 600,
        "acquisition_agl_m": 4.572,
        "transit_agl_m": 10.0,
        "release_agl_m": 10.0,
    },
}

_EXPECTED_SCENARIO = {
    "schema_version": 1,
    "seed": 2026,
    "vehicle": {"payload_capacity": 1},
    "camera": {
        "width_px": 640,
        "height_px": 480,
        "update_rate_hz": 20,
        "horizontal_fov_rad": 0.60,
        "body_position_m": [0.0, 0.0, -0.10],
    },
    "range_sensor": {"update_rate_hz": 20},
    "payload_interaction": {
        "pickup_max_center_error_m": 0.075,
        "settle_position_tolerance_m": 0.01,
        "settle_time_s": 1.0,
    },
    "payload_geometry": {
        "size_in": [6, 6, 2],
        "mass_lb": 2.5,
        "marker_size_mm": 100,
    },
    "payloads": [
        {"aruco_id": 2, "color": "red", "initial": "attached"},
        {"aruco_id": 3, "color": "yellow", "initial": "WA"},
        {"aruco_id": 4, "color": "blue", "initial": "WM"},
    ],
    "mission": {
        "fm2_drop_zone": "F2",
        "fm3_cycles": [
            {"pickup_zone": "WA", "color": "yellow", "drop_zone": "F2"},
            {"pickup_zone": "WM", "color": "blue", "drop_zone": "F2"},
        ],
    },
}


def _same_typed(value: object, expected: object) -> bool:
    if type(value) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(value) == set(expected) and all(  # type: ignore[arg-type]
            _same_typed(value[key], expected[key]) for key in expected  # type: ignore[index]
        )
    if isinstance(expected, list):
        return len(value) == len(expected) and all(  # type: ignore[arg-type]
            _same_typed(actual, wanted)
            for actual, wanted in zip(value, expected, strict=True)  # type: ignore[arg-type]
        )
    return value == expected


def _yaml_document(content: bytes, *, expected: object, label: str) -> object:
    import yaml

    class UniqueKeyLoader(yaml.SafeLoader):
        pass

    def construct_unique_mapping(loader: Any, node: Any, deep: bool = False) -> dict:
        loader.flatten_mapping(node)
        result: dict[Any, Any] = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node, deep=deep)
            try:
                duplicate = key in result
            except TypeError as error:
                raise yaml.constructor.ConstructorError(
                    "while constructing a mapping",
                    node.start_mark,
                    "found an unhashable key",
                    key_node.start_mark,
                ) from error
            if duplicate:
                raise yaml.constructor.ConstructorError(
                    "while constructing a mapping",
                    node.start_mark,
                    f"found duplicate key {key!r}",
                    key_node.start_mark,
                )
            result[key] = loader.construct_object(value_node, deep=deep)
        return result

    UniqueKeyLoader.add_constructor(
        yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_unique_mapping
    )

    try:
        value = yaml.load(content.decode("utf-8"), Loader=UniqueKeyLoader)
    except (UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"{label} must be valid UTF-8 YAML") from error
    if not _same_typed(value, expected):
        raise ValueError(f"{label} does not match the validated competition contract")
    return value


def _require_hashed(path: Path, digest: str, *, label: str) -> bytes:
    content = _read_regular_file(path, label=label).content
    if hashlib.sha256(content).hexdigest() != digest:
        raise ValueError(f"{label} bytes do not match their SHA-256")
    return content


def _distance_m(first: Any, second: Any) -> float:
    radius = 6_378_137.0
    first_lat = math.radians(first.lat)
    second_lat = math.radians(second.lat)
    north = second_lat - first_lat
    longitude_delta = (second.long - first.long + 180.0) % 360.0 - 180.0
    east = math.radians(longitude_delta) * math.cos(
        (first_lat + second_lat) / 2.0
    )
    return radius * math.hypot(east, north)


def _course_coordinate(
    origin: Any, entry: Mapping[str, object], coordinate_type: Any
) -> Any:
    radius = 6_378_137.0
    # This local tangent projection is validated only where longitude scale is
    # well-conditioned; competition sites outside this domain require geodesy.
    if abs(float(origin.lat)) > 85.0:
        raise ValueError("course origin is outside the validated latitude domain")
    x_m = float(entry["x"])
    y_m = float(entry["y"])
    latitude = origin.lat + math.degrees(y_m / radius)
    longitude = origin.long + math.degrees(
        x_m / (radius * math.cos(math.radians(origin.lat)))
    )
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise ValueError("derived course coordinate is outside WGS84 bounds")
    return coordinate_type(latitude, longitude, 10.0)


def _waypoint_bytes(waypoints: Mapping[str, Any], epoch_seconds: float) -> bytes:
    document = {
        name: {
            "coords": {"lat": point.lat, "long": point.long, "alt": point.alt},
            "loaded_at": epoch_seconds,
        }
        for name, point in waypoints.items()
    }
    return (json.dumps(document, sort_keys=True, separators=(",", ":")) + "\n").encode()


@dataclass(slots=True)
class _WaypointOutput:
    path: Path
    run_fd: int
    control_fd: int
    control_identity: tuple[int, int]
    file_identity: tuple[int, int] | None
    file_fd: int
    created_file: bool
    created_directory: bool

    def close(self) -> None:
        close_error: BaseException | None = None
        for name in ("file_fd", "control_fd", "run_fd"):
            descriptor = getattr(self, name)
            if descriptor >= 0:
                setattr(self, name, -1)
                try:
                    os.close(descriptor)
                except BaseException as error:
                    if close_error is None:
                        close_error = error
        if close_error is not None:
            raise close_error

    def rollback(self) -> None:
        """Close capabilities and retain namespace entries as forensic debris."""
        self.close()


def _write_run_owned_waypoints(run_directory: Path, content: bytes) -> _WaypointOutput:
    """Create or verify the fixed run-owned file and retain ownership descriptors."""
    raw_run_directory = os.fspath(run_directory)
    if (
        raw_run_directory.startswith("//")
        or not run_directory.is_absolute()
        or run_directory != Path(os.path.abspath(run_directory))
    ):
        raise ValueError("run directory must be absolute and canonical")
    directory_flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    current_fd = os.open("/", directory_flags)
    created_directory = False
    control_fd = -1
    output: _WaypointOutput | None = None
    try:
        for component in run_directory.parts[1:]:
            next_fd = os.open(component, directory_flags, dir_fd=current_fd)
            predecessor_fd = current_fd
            current_fd = -1
            try:
                os.close(predecessor_fd)
            except BaseException:
                successor_fd = next_fd
                next_fd = -1
                os.close(successor_fd)
                raise
            current_fd = next_fd
        try:
            os.mkdir(".control", mode=0o700, dir_fd=current_fd)
            created_directory = True
        except FileExistsError:
            pass
        control_fd = os.open(".control", directory_flags, dir_fd=current_fd)
        control_metadata = os.fstat(control_fd)
        control_identity = (control_metadata.st_dev, control_metadata.st_ino)
        if created_directory and os.listdir(control_fd):
            raise ValueError("newly created control directory was replaced before open")
        output = _WaypointOutput(
            path=run_directory / ".control/comp2026-waypoints.json",
            run_fd=current_fd,
            control_fd=control_fd,
            control_identity=control_identity,
            file_identity=None,
            file_fd=-1,
            created_file=False,
            created_directory=created_directory,
        )
        control_fd = -1
        current_fd = -1
        if created_directory:
            os.fsync(output.run_fd)
        else:
            try:
                descriptor = os.open(
                    output.path.name,
                    os.O_RDONLY
                    | getattr(os, "O_CLOEXEC", 0)
                    | getattr(os, "O_NOFOLLOW", 0)
                    | getattr(os, "O_NONBLOCK", 0),
                    dir_fd=output.control_fd,
                )
            except FileNotFoundError as error:
                raise ValueError(
                    "waypoint output cannot be created in a preexisting control directory"
                ) from error
            try:
                with os.fdopen(descriptor, "rb") as source:
                    descriptor = -1
                    if not stat.S_ISREG(os.fstat(source.fileno()).st_mode):
                        raise ValueError("waypoint output cannot overwrite a nonregular file")
                    if source.read() != content:
                        raise ValueError("waypoint output cannot overwrite existing bytes")
            finally:
                if descriptor >= 0:
                    os.close(descriptor)
            return output
        flags = (
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0)
        )
        try:
            descriptor = os.open(output.path.name, flags, 0o600, dir_fd=output.control_fd)
            output.created_file = True
            output.file_fd = descriptor
            metadata = os.fstat(descriptor)
            output.file_identity = (metadata.st_dev, metadata.st_ino)
        except FileExistsError as error:
            raise ValueError("waypoint output appeared during exclusive creation") from error
        write_descriptor = os.dup(descriptor)
        try:
            destination = os.fdopen(write_descriptor, "wb")
        except BaseException:
            os.close(write_descriptor)
            raise
        with destination:
            destination.write(content)
            destination.flush()
            os.fsync(destination.fileno())
        os.fsync(output.control_fd)
        return output
    except BaseException:
        if output is not None:
            try:
                output.rollback()
            except BaseException:
                pass
        raise
    finally:
        if control_fd >= 0:
            try:
                os.close(control_fd)
            except BaseException:
                pass
        if current_fd >= 0:
            try:
                os.close(current_fd)
            except BaseException:
                pass


def project_qgc_runtime(
    config: object,
    *,
    epoch_seconds: float,
    test_only_state_root: str | os.PathLike[str] | None = None,
) -> QGCRuntimeProjection:
    """Validate all offline facts, then create the sole run-owned projection."""
    if getattr(config, "mission", None) != "comp2026_auto":
        raise ValueError("QGC runtime projection requires mission comp2026_auto")
    qgc = getattr(config, "qgc", None)
    if not isinstance(qgc, ResolvedQGCInputs):
        raise ValueError("comp2026_auto requires resolved QGC inputs")
    if (
        isinstance(epoch_seconds, bool)
        or not isinstance(epoch_seconds, (int, float))
        or not math.isfinite(float(epoch_seconds))
        or float(epoch_seconds) <= 0
    ):
        raise ValueError("epoch_seconds must be finite and positive")
    epoch = float(epoch_seconds)

    run_directory = Path(getattr(config, "run_directory"))
    output_root = getattr(config, "output_root", None)
    if not isinstance(output_root, Path):
        raise ValueError("comp2026_auto requires a resolved output_root")
    raw_output_root = os.fspath(output_root)
    if (
        raw_output_root.startswith("//")
        or not output_root.is_absolute()
        or output_root != Path(os.path.abspath(output_root))
    ):
        raise ValueError("output_root must be absolute and canonical")
    configuration_directory = run_directory / "configuration"
    expected_paths = {
        "deployment_profile": configuration_directory / "deployment-profile.json",
        "listener_session": configuration_directory / "listener-session.json",
        "qgc_actions": configuration_directory / "qgc-actions.json",
        "runtime_policy": configuration_directory / "qgc-runtime.json",
    }
    artifact_bytes: dict[str, bytes] = {}
    for field, path in expected_paths.items():
        actual = getattr(qgc, f"{field}_path")
        if actual != path:
            raise ValueError(f"QGC {field} path is not the canonical run artifact")
        artifact_bytes[field] = _require_hashed(
            path, getattr(qgc, f"{field}_sha256"), label=f"QGC {field}"
        )

    course_path = Path(getattr(config, "course_path"))
    scenario_path = Path(getattr(config, "scenario_path"))
    if course_path != configuration_directory / "course.yaml":
        raise ValueError("course path is not the canonical run artifact")
    if scenario_path != configuration_directory / "scenario.yaml":
        raise ValueError("scenario path is not the canonical run artifact")
    course = _yaml_document(
        _require_hashed(course_path, qgc.course_sha256, label="competition course"),
        expected=_EXPECTED_COURSE,
        label="competition course",
    )
    scenario = _yaml_document(
        _require_hashed(scenario_path, qgc.scenario_sha256, label="competition scenario"),
        expected=_EXPECTED_SCENARIO,
        label="competition scenario",
    )

    from drone import timebase
    from drone.common_types import GPSCoord, MissionHome
    from drone.control.flight_state import SourceIdentity
    from drone.control.listener import load_listener_artifacts_bytes
    from drone.control.listener_runtime import (
        AutopilotVersionContract,
        ConnectionConfig,
        InjectedComponentConfig,
        OperatingSitePolicy,
        RuntimeConfiguration,
        TelemetryStartupPolicy,
        VisionConfig,
        shared_monotonic_ns,
    )
    from drone.control.mission_info import MAX_WAYPOINT_AGE_SECONDS
    from drone.control.mission_supervisor import RecoveryPolicy
    from drone.mock_mission import PrecisionMissionPolicy
    from drone.control.stability import ReleaseStabilityConfig
    from drone.sensors.lidar.clearance import ClearanceCalibration

    from .qgc_attempt_state import resolve_attempt_state
    from .qgc_runtime_policy import load_qgc_runtime_policy_bytes

    policy = load_qgc_runtime_policy_bytes(
        artifact_bytes["runtime_policy"], expected_sha256=qgc.runtime_policy_sha256
    )
    epoch_now = timebase.epoch()
    if epoch > epoch_now or epoch_now - epoch > MAX_WAYPOINT_AGE_SECONDS:
        raise ValueError("epoch_seconds must be a current persisted-data timestamp")
    if (
        policy.bindings.course_sha256 != qgc.course_sha256
        or policy.bindings.scenario_sha256 != qgc.scenario_sha256
        or policy.bindings.ardupilot_commit
        != "2a3dc4b7bf2507120f7378a7b2fde73185e0c325"
    ):
        raise ValueError("runtime policy bindings do not match the resolved run")
    origin_policy = policy.simulator_launch_origin
    origin = GPSCoord(origin_policy.latitude_deg, origin_policy.longitude_deg, 0.0)
    waypoints = course["waypoints"]  # type: ignore[index]
    fm2_zone = scenario["mission"]["fm2_drop_zone"]  # type: ignore[index]
    derived = {
        "L": _course_coordinate(origin, waypoints["L"], GPSCoord),
        "TARGET": _course_coordinate(origin, waypoints[fm2_zone], GPSCoord),
    }
    full_phase = policy.purpose == "drone-sim-comp2026-full"
    if full_phase:
        cycles = scenario["mission"]["fm3_cycles"]  # type: ignore[index]
        if cycles != [
            {"pickup_zone": "WA", "color": "yellow", "drop_zone": "F2"},
            {"pickup_zone": "WM", "color": "blue", "drop_zone": "F2"},
        ]:
            raise ValueError("full runtime requires both exact ordered FM3 cycles")
        derived.update(
            {
                "WA": _course_coordinate(origin, waypoints["WA"], GPSCoord),
                "WM1": _course_coordinate(origin, waypoints["WM"], GPSCoord),
            }
        )
    site = policy.operating_site

    def waypoint_check(name: str, coordinate: Any) -> bool:
        expected = derived.get(name)
        return (
            expected is not None
            and isinstance(coordinate, GPSCoord)
            and all(
                math.isfinite(float(value))
                for value in (coordinate.lat, coordinate.long, coordinate.alt)
            )
            and _distance_m(expected, coordinate) <= site.waypoint_tolerance_m
            and abs(float(coordinate.alt) - expected.alt) <= site.waypoint_tolerance_m
        )

    def mission_home_check(home: Any) -> None:
        if not isinstance(home, MissionHome):
            raise ValueError("mission home must be explicit")
        home_coordinate = GPSCoord(home.lat, home.lon, 0.0)
        if (
            not all(
                math.isfinite(float(value))
                for value in (home.lat, home.lon, home.amsl_m)
            )
            or _distance_m(origin, home_coordinate) > site.home_position_tolerance_m
            or abs(home.amsl_m - origin_policy.amsl_m) > site.home_altitude_tolerance_m
        ):
            raise ValueError("mission home is outside the validated operating site")

    def recovery_check(operation: str, home: Any, altitude_agl_m: float | None) -> None:
        if operation not in site.operations:
            raise ValueError("recovery operation is outside the validated operating site")
        if operation == "LOCAL_LAND":
            # This fallback is closed over the validated run/site and is used
            # specifically when the FC's return-home datum cannot be trusted.
            if altitude_agl_m is not None:
                raise ValueError("LOCAL_LAND does not accept a recovery altitude")
            return
        mission_home_check(home)
        if isinstance(altitude_agl_m, bool) or not isinstance(
            altitude_agl_m, (int, float)
        ):
            raise ValueError("recovery altitude is outside the validated corridor")
        altitude_amsl_m = float(altitude_agl_m)
        recovery_agl_m = altitude_amsl_m - home.amsl_m
        if (
            not math.isfinite(altitude_amsl_m)
            or not site.minimum_recovery_agl_m
            <= recovery_agl_m
            <= site.maximum_recovery_agl_m
        ):
            raise ValueError("recovery altitude is outside the validated corridor")

    operating_site = OperatingSitePolicy(
        waypoint_check=waypoint_check,
        mission_home_check=mission_home_check,
        recovery_check=recovery_check,
        evidence_reference=site.evidence_reference,
    )
    valid_home = MissionHome(origin.lat, origin.long, origin_policy.amsl_m)
    if not all(waypoint_check(name, point) for name, point in derived.items()):
        raise ValueError("derived waypoints failed the closed site callback")
    mission_home_check(valid_home)
    recovery_check(
        "RETURN",
        valid_home,
        valid_home.amsl_m + site.minimum_recovery_agl_m,
    )
    recovery_check("LOCAL_LAND", valid_home, None)

    attempt_state = resolve_attempt_state(
        qgc.deployment_profile_sha256,
        qgc.attempt_state_id,
        forbidden_paths=(
            run_directory,
            configuration_directory,
            output_root,
            run_directory / ".control",
            run_directory / ".control/comp2026-waypoints.json",
        ),
        test_only_state_root=test_only_state_root,
    )
    validated_listener_artifacts = load_listener_artifacts_bytes(
        artifact_bytes["deployment_profile"],
        artifact_bytes["listener_session"],
        artifact_bytes["qgc_actions"],
        ledger_path=attempt_state.ledger_path,
    )
    deployment = validated_listener_artifacts.deployment_profile
    flight = validated_listener_artifacts.flight_profile
    prepared = validated_listener_artifacts.prepared_attempt
    if (
        validated_listener_artifacts.profile_sha256
        != qgc.deployment_profile_sha256
        or validated_listener_artifacts.session_sha256
        != qgc.listener_session_sha256
        or validated_listener_artifacts.actions_sha256 != qgc.qgc_actions_sha256
    ):
        validated_listener_artifacts.close()
        raise ValueError("listener artifact snapshot does not match resolved hashes")
    if (
        deployment.firmware != "ArduCopter 4.5.7"
        or flight.firmware != "ArduCopter 4.5.7"
        or deployment.profile_id != flight.profile_id
        or deployment.raw_sha256 != flight.raw_sha256
    ):
        validated_listener_artifacts.close()
        raise ValueError("deployment and flight profile identities are inconsistent")
    if (
        flight.companion_target
        != SourceIdentity(deployment.target_system, deployment.target_component)
        or flight.flight_controller
        != SourceIdentity(
            deployment.flight_controller_system,
            deployment.flight_controller_component,
        )
        or prepared.profile_id != deployment.profile_id
    ):
        validated_listener_artifacts.close()
        raise ValueError("listener identities are inconsistent")
    launch_json = json.dumps(
        {
            "latitude_deg": origin_policy.latitude_deg,
            "longitude_deg": origin_policy.longitude_deg,
            "amsl_m": origin_policy.amsl_m,
            "heading_deg": origin_policy.heading_deg,
        },
        sort_keys=True,
        separators=(",", ":"),
    )
    launch_decoded = json.loads(launch_json)
    if (launch_decoded["latitude_deg"], launch_decoded["longitude_deg"]) != (
        origin.lat,
        origin.long,
    ):
        validated_listener_artifacts.close()
        raise ValueError("ArduPilot launch origin does not match waypoint origin")

    waypoint_content = _waypoint_bytes(derived, epoch)
    try:
        waypoint_output = _write_run_owned_waypoints(run_directory, waypoint_content)
    except BaseException:
        validated_listener_artifacts.close()
        raise
    try:
        evidence = (
            f"course-sha256:{qgc.course_sha256};"
            f"scenario-sha256:{qgc.scenario_sha256}"
        )
        clearance_calibration = ClearanceCalibration(
            **vars(policy.clearance_calibration)
        )
        vision = None
        precision_policy = None
        if full_phase:
            if policy.vision is None or policy.precision is None:
                raise ValueError(
                    "full runtime policy requires vision and precision configuration"
                )
            attempt = course["attempt"]  # type: ignore[index]
            marker_size_mm = scenario["payload_geometry"][  # type: ignore[index]
                "marker_size_mm"
            ]
            if (
                policy.vision.marker_size_mm != marker_size_mm
                or policy.precision.target_hover_height_m
                != attempt["acquisition_agl_m"]
                or policy.precision.cruise_altitude_m != attempt["transit_agl_m"]
                or policy.precision.desired_drop_height_m != attempt["release_agl_m"]
            ):
                raise ValueError(
                    "full runtime precision values do not match the competition contract"
                )
            vision = VisionConfig(
                marker_size_mm=policy.vision.marker_size_mm,
                calibration_path=Path(__file__).with_name(
                    policy.vision.calibration_path
                ),
                mounting_path=Path(__file__).with_name(policy.vision.mounting_path),
                receipt_clock_ns=shared_monotonic_ns,
                max_exposure_age_ns=policy.vision.max_exposure_age_ns,
            )
            precision_values = vars(policy.precision).copy()
            precision_values.pop("clearance_calibration")
            precision_values.pop("clock")
            precision_policy = PrecisionMissionPolicy(
                clearance_calibration=clearance_calibration,
                clock=timebase.monotonic,
                **precision_values,
            )
        runtime = RuntimeConfiguration(
            connection=ConnectionConfig(
                endpoint=getattr(config, "mavlink_endpoint"),
                source_identity=flight.companion_target,
                target_identity=flight.flight_controller,
                wire_protocol=deployment.mavlink_wire_protocol,
                wait_ready=policy.connection.wait_ready,
                heartbeat_timeout_s=policy.connection.heartbeat_timeout_s,
            ),
            waypoint_path=waypoint_output.path,
            operating_site=operating_site,
            recovery_policy=RecoveryPolicy(
                check=recovery_check,
                timeout_s=policy.recovery.timeout_s,
                local_land_reserve_s=policy.recovery.local_land_reserve_s,
                clock=timebase.monotonic,
            ),
            telemetry=TelemetryStartupPolicy(**vars(policy.telemetry_startup)),
            autopilot_version=AutopilotVersionContract(
                firmware_label=policy.autopilot_version.firmware_label,
                flight_sw_version=policy.autopilot_version.flight_sw_version,
                flight_custom_version=(
                    policy.autopilot_version.flight_custom_version_bytes
                ),
                evidence_reference=policy.autopilot_version.evidence_reference,
            ),
            vision=vision,
            components=InjectedComponentConfig(
                backend=policy.backend, evidence_reference=evidence
            ),
            clearance_calibration=clearance_calibration,
            release_stability=ReleaseStabilityConfig(
                **vars(policy.release_stability)
            ),
            enabled_phases=frozenset(policy.enabled_phases),
            precision_policy=precision_policy,
            cruise_altitude_m=(
                10.0 if precision_policy is None else precision_policy.cruise_altitude_m
            ),
            desired_drop_height_m=(
                10.0
                if precision_policy is None
                else precision_policy.desired_drop_height_m
            ),
            fc_home_position_tolerance_m=policy.fc_home_position_tolerance_m,
            fc_home_altitude_tolerance_m=policy.fc_home_altitude_tolerance_m,
            attempt_timeout_s=600.0,
            idle_poll_s=policy.idle_poll_s,
            cleanup_timeout_s=policy.cleanup_timeout_s,
        )
    except BaseException:
        try:
            waypoint_output.rollback()
        except BaseException:
            pass
        validated_listener_artifacts.close()
        raise
    try:
        waypoint_output.close()
    except BaseException:
        try:
            waypoint_output.rollback()
        except BaseException:
            pass
        validated_listener_artifacts.close()
        raise
    return QGCRuntimeProjection(
        runtime_configuration=runtime,
        validated_listener_artifacts=validated_listener_artifacts,
        deployment_profile=deployment,
        flight_profile=flight,
        attempt_state=attempt_state,
        sim_launch_origin_json=launch_json,
        payload_delay_wall_timeout_s=policy.payload_delay_wall_timeout_s,
    )
