"""Immutable operator-template resolution and run configuration persistence."""

from collections.abc import Callable
from dataclasses import dataclass, replace
from decimal import Decimal, InvalidOperation
import hashlib
import json
import os
from pathlib import Path
import stat
from typing import Any
from uuid import UUID, uuid4

import yaml


_TEMPLATE_FIELDS = {
    "world",
    "vehicle",
    "mission",
    "scenario",
    "output_root",
    "max_wall_seconds",
    "startup_wall_seconds",
    "finalization_wall_seconds",
    "recording",
}
_RESOLVED_FIELDS = _TEMPLATE_FIELDS | {"run_id", "config_sha256"}
_OPTIONAL_FIELDS = {"runtime_profile", "simulation", "competition", "qgc"}
_STRING_FIELDS = ("world", "vehicle", "mission", "scenario", "output_root")
_DEADLINE_FIELDS = (
    "max_wall_seconds",
    "startup_wall_seconds",
    "finalization_wall_seconds",
)
_RECORDING_FIELDS = {"width_px", "height_px", "fps", "encoding"}
_SIMULATION_FIELDS = {
    "seed",
    "duration_sim_seconds",
    "public_epoch_native_sim_seconds",
    "target_real_time_factor",
}
_TEMPLATE_COMPETITION_FIELDS = {"course", "scenario"}
_RESOLVED_COMPETITION_FIELDS = {
    "course",
    "scenario",
    "course_sha256",
    "scenario_sha256",
}
_TEMPLATE_QGC_FIELDS = {
    "deployment_profile",
    "listener_session",
    "qgc_actions",
    "runtime_policy",
    "attempt_state_root",
}
_QGC_ARTIFACT_NAMES = {
    "deployment_profile": "deployment-profile.json",
    "listener_session": "listener-session.json",
    "qgc_actions": "qgc-actions.json",
    "runtime_policy": "qgc-runtime.json",
}
_RESOLVED_QGC_FIELDS = {
    *_TEMPLATE_QGC_FIELDS,
    *(f"{field}_sha256" for field in _QGC_ARTIFACT_NAMES),
    "attempt_state_id",
}
CAMERA_INTERVAL_NS = 50_000_000
PUBLIC_EPOCH_DEFAULT_NS = 90_000_000_000

PHASE2_OWNERSHIP = (
    ("orchestration-runtime", "orchestration"),
    ("artifacts-runtime", "artifacts"),
    ("synthetic-companion", "companion"),
    ("synthetic-ardupilot-sitl", "ardupilot_sitl"),
    ("synthetic-gazebo", "gazebo"),
    ("synthetic-electromagnet", "electromagnet"),
    ("synthetic-scorekeeper", "scorekeeper"),
)
PHASE3_OWNERSHIP = (
    ("orchestration-runtime", "orchestration"),
    ("artifacts-runtime", "artifacts"),
    ("companion-runtime", "companion"),
    ("ardupilot-sitl", "ardupilot_sitl"),
    ("gazebo-runtime", "gazebo"),
    ("electromagnet-runtime", "electromagnet"),
    ("scorekeeper-runtime", "scorekeeper"),
)


@dataclass(frozen=True)
class RecordingConfig:
    width_px: int
    height_px: int
    fps: int
    encoding: str


@dataclass(frozen=True)
class CompetitionSources:
    course_source: Path
    scenario_source: Path
    course_sha256: str
    scenario_sha256: str


@dataclass(frozen=True)
class QGCSources:
    deployment_profile: bytes
    listener_session: bytes
    qgc_actions: bytes
    runtime_policy: bytes
    attempt_state_root: Path

    def __post_init__(self) -> None:
        for field, payload in self._payloads():
            if not isinstance(payload, bytes):
                raise TypeError(f"QGC {field} payload must be immutable bytes")
            _validate_json_object(payload, field)
        object.__setattr__(
            self,
            "attempt_state_root",
            _canonical_absolute_path(
                self.attempt_state_root, label="QGC attempt state root"
            ),
        )

    def _payloads(self) -> tuple[tuple[str, bytes], ...]:
        return tuple(
            (field, getattr(self, field)) for field in _QGC_ARTIFACT_NAMES
        )

    def _digest(self, field: str) -> str:
        return hashlib.sha256(getattr(self, field)).hexdigest()

    @property
    def attempt_state_id(self) -> str:
        return f"sha256-{self._digest('deployment_profile')}"


@dataclass(frozen=True)
class SimulationConfig:
    seed: int
    duration_ns: int
    target_real_time_factor: float
    public_epoch_native_ns: int = PUBLIC_EPOCH_DEFAULT_NS

    @property
    def expected_camera_frames(self) -> int:
        return self.duration_ns // CAMERA_INTERVAL_NS


@dataclass(frozen=True)
class RuntimeTopology:
    profile: str
    ownership: tuple[tuple[str, str], ...]

    def __post_init__(self) -> None:
        if self.profile == "phase2":
            expected = PHASE2_OWNERSHIP
        elif self.profile == "phase3":
            expected = PHASE3_OWNERSHIP
        else:
            raise ValueError("runtime topology profile must be phase2 or phase3")
        if type(self.ownership) is not tuple or self.ownership != expected:
            raise ValueError("runtime topology ownership does not match its profile")


PHASE2_TOPOLOGY = RuntimeTopology("phase2", PHASE2_OWNERSHIP)
PHASE3_TOPOLOGY = RuntimeTopology("phase3", PHASE3_OWNERSHIP)


@dataclass(frozen=True)
class RunTemplate:
    world: str
    vehicle: str
    mission: str
    scenario: str
    output_root: Path
    max_wall_seconds: int
    startup_wall_seconds: int
    finalization_wall_seconds: int
    recording: RecordingConfig
    runtime_profile: str
    simulation: SimulationConfig | None
    competition: CompetitionSources | None = None
    qgc: QGCSources | None = None


@dataclass(frozen=True)
class RunConfig:
    run_id: str
    world: str
    vehicle: str
    mission: str
    scenario: str
    output_root: Path
    max_wall_seconds: int
    startup_wall_seconds: int
    finalization_wall_seconds: int
    recording: RecordingConfig
    runtime_profile: str
    simulation: SimulationConfig | None
    config_sha256: str
    competition: CompetitionSources | None = None
    qgc: QGCSources | None = None

    @property
    def expected_camera_frames(self) -> int:
        if self.simulation is None:
            return 2 * self.recording.fps
        return self.simulation.expected_camera_frames

    @property
    def topology(self) -> RuntimeTopology:
        return PHASE3_TOPOLOGY if self.runtime_profile == "phase3" else PHASE2_TOPOLOGY


def _read_document_payload(payload: bytes, source: Path) -> dict[str, Any]:
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid run configuration: {source}") from exc
    if not isinstance(document, dict):
        raise ValueError("run configuration must be a JSON object")
    return document


def _canonical_absolute_path(value: Path | str, *, label: str) -> Path:
    try:
        raw = os.fspath(value)
    except TypeError as exc:
        raise ValueError(f"{label} must be canonical and absolute") from exc
    if not isinstance(raw, str) or "\0" in raw or raw.startswith("//"):
        raise ValueError(f"{label} must be canonical and absolute")
    path = Path(raw)
    try:
        canonical = Path(os.path.abspath(raw))
    except (OSError, ValueError) as exc:
        raise ValueError(f"{label} must be canonical and absolute") from exc
    if not path.is_absolute() or path != canonical or path == Path("/"):
        raise ValueError(f"{label} must be canonical and absolute")
    return canonical


def _validate_recording(document: Any, *, mission: str) -> RecordingConfig:
    if not isinstance(document, dict) or set(document) != _RECORDING_FIELDS:
        raise ValueError("recording configuration has missing or unknown keys")
    width = document["width_px"]
    height = document["height_px"]
    if (
        type(width) is not int
        or type(height) is not int
        or (width, height) not in {(320, 240), (640, 480)}
    ):
        raise ValueError("recording dimensions must be 320x240 or 640x480")
    required_dimensions = (
        (640, 480)
        if mission == "comp2026_auto"
        else (320, 240)
    )
    if (width, height) != required_dimensions:
        raise ValueError(
            f"{mission} recording dimensions must be exactly "
            f"{required_dimensions[0]}x{required_dimensions[1]}"
        )
    if document["fps"] != 20 or isinstance(document["fps"], bool):
        raise ValueError("recording fps must be 20")
    if document["encoding"] != "rgb8":
        raise ValueError("recording encoding must be rgb8")
    return RecordingConfig(width, height, document["fps"], document["encoding"])


def _validate_simulation(document: Any) -> SimulationConfig:
    if not isinstance(document, dict) or set(document) != _SIMULATION_FIELDS:
        raise ValueError("simulation configuration has missing or unknown keys")
    seed = document["seed"]
    if type(seed) is not int or not 0 <= seed <= 4_294_967_295:
        raise ValueError("simulation seed must be an unsigned 32-bit integer")
    duration = document["duration_sim_seconds"]
    if isinstance(duration, bool) or not isinstance(duration, (int, float)):
        raise ValueError("simulation duration must be a positive finite number")
    try:
        duration_decimal = Decimal(str(duration))
    except InvalidOperation as exc:
        raise ValueError("simulation duration must be a positive finite number") from exc
    if not duration_decimal.is_finite() or duration_decimal <= 0:
        raise ValueError("simulation duration must be a positive finite number")
    duration_numerator, duration_denominator = duration_decimal.as_integer_ratio()
    scaled_numerator = duration_numerator * 1_000_000_000
    if scaled_numerator % duration_denominator != 0:
        raise ValueError("simulation duration must resolve to exact integer nanoseconds")
    duration_ns = scaled_numerator // duration_denominator
    if duration_ns % CAMERA_INTERVAL_NS != 0:
        raise ValueError("simulation duration must contain an integral camera frame count")
    target = document["target_real_time_factor"]
    if isinstance(target, bool) or not isinstance(target, (int, float)):
        raise ValueError("target_real_time_factor must be exactly 0.1, 0.25, or 1.0")
    try:
        target_decimal = Decimal(str(target))
    except InvalidOperation as exc:
        raise ValueError("target_real_time_factor must be exactly 0.1, 0.25, or 1.0") from exc
    if not target_decimal.is_finite() or target_decimal not in {
        Decimal("0.1"),
        Decimal("0.25"),
        Decimal("1.0"),
    }:
        raise ValueError("target_real_time_factor must be exactly 0.1, 0.25, or 1.0")
    public_epoch = document["public_epoch_native_sim_seconds"]
    if isinstance(public_epoch, bool) or not isinstance(public_epoch, (int, float)):
        raise ValueError(
            "public_epoch_native_sim_seconds must be a positive finite number"
        )
    try:
        public_epoch_decimal = Decimal(str(public_epoch))
    except InvalidOperation as exc:
        raise ValueError(
            "public_epoch_native_sim_seconds must be a positive finite number"
        ) from exc
    if not public_epoch_decimal.is_finite() or public_epoch_decimal <= 0:
        raise ValueError(
            "public_epoch_native_sim_seconds must be a positive finite number"
        )
    epoch_numerator, epoch_denominator = public_epoch_decimal.as_integer_ratio()
    scaled_epoch_numerator = epoch_numerator * 1_000_000_000
    if scaled_epoch_numerator % epoch_denominator != 0:
        raise ValueError(
            "public_epoch_native_sim_seconds must resolve to exact integer nanoseconds"
        )
    public_epoch_native_ns = scaled_epoch_numerator // epoch_denominator
    if public_epoch_native_ns % CAMERA_INTERVAL_NS != 0:
        raise ValueError(
            "public_epoch_native_sim_seconds must be on the 50 ms public grid"
        )
    return SimulationConfig(
        seed,
        duration_ns,
        float(target_decimal),
        public_epoch_native_ns,
    )


def _duration_seconds(duration_ns: int) -> int | float:
    seconds, remainder_ns = divmod(duration_ns, 1_000_000_000)
    if remainder_ns == 0:
        return float(seconds) if seconds <= 2**53 else seconds
    return float(Decimal(duration_ns) / Decimal(1_000_000_000))


def _validate_common(
    document: dict[str, Any], required_fields: set[str]
) -> tuple[RecordingConfig, str, SimulationConfig | None]:
    fields = set(document)
    if not required_fields <= fields or not fields <= required_fields | _OPTIONAL_FIELDS:
        raise ValueError("run configuration has missing or unknown keys")
    if any(
        not isinstance(document[field], str) or not document[field]
        for field in _STRING_FIELDS
    ):
        raise ValueError("run configuration string fields must be non-empty")
    for field in _DEADLINE_FIELDS:
        value = document[field]
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise ValueError(f"{field} must be a positive integer")
    runtime_profile = document.get("runtime_profile", "phase2")
    if runtime_profile not in {"phase2", "phase3"}:
        raise ValueError("runtime_profile must be phase2 or phase3")
    if "qgc" in document and runtime_profile != "phase3":
        raise ValueError("QGC configuration requires runtime_profile phase3")
    if "qgc" in document and document["mission"] != "comp2026_auto":
        raise ValueError("QGC configuration requires mission comp2026_auto")
    if runtime_profile == "phase3":
        if "simulation" not in document:
            raise ValueError("phase3 requires simulation configuration")
        simulation = _validate_simulation(document["simulation"])
    else:
        if "simulation" in document:
            raise ValueError("simulation configuration requires runtime_profile phase3")
        simulation = None
    return (
        _validate_recording(document["recording"], mission=document["mission"]),
        runtime_profile,
        simulation,
    )


_EXPECTED_COURSE = {
    "schema_version": 1,
    "units": "meters",
    "origin": "H",
    "waypoints": {
        "H": {"x": 0.0, "y": 0.0, "width": 4.572, "height": 4.572, "role": "home"},
        "L": {"x": -91.44, "y": 0.0, "width": 4.572, "height": 4.572, "role": "landing"},
        "F2": {"x": -152.40, "y": 0.0, "width": 0.9144, "height": 0.9144, "role": "fire"},
        "WA": {"x": -45.72, "y": -9.144, "width": 6.096, "height": 6.096, "role": "autonomous_pickup"},
        "WM": {"x": -45.72, "y": 9.144, "width": 6.096, "height": 6.096, "role": "manual_pickup"},
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


def _require_source_file(path: Path) -> None:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"competition source must be a regular non-symlink file: {path}")


def _same_typed_document(actual: Any, expected: Any) -> bool:
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return actual.keys() == expected.keys() and all(
            _same_typed_document(actual[key], value) for key, value in expected.items()
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _same_typed_document(actual_value, expected_value)
            for actual_value, expected_value in zip(actual, expected, strict=True)
        )
    return actual == expected


def _read_validated_source(
    path: Path, expected: dict[str, Any], name: str
) -> bytes:
    try:
        payload = path.read_bytes()
        document = yaml.safe_load(payload)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f"invalid {name} configuration: {path}") from exc
    if not _same_typed_document(document, expected):
        raise ValueError(f"{name} configuration does not match the approved schema")
    return payload


def _template_source(template_dir: Path, value: Any, name: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"competition {name} source must be a relative path")
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"competition {name} source must be a safe relative path")
    return template_dir / relative


def _competition_from_template(
    document: Any, template_dir: Path
) -> CompetitionSources | None:
    if document is None:
        return None
    if not isinstance(document, dict) or set(document) != _TEMPLATE_COMPETITION_FIELDS:
        raise ValueError("competition configuration has missing or unknown keys")
    course = _template_source(template_dir, document["course"], "course")
    scenario = _template_source(template_dir, document["scenario"], "scenario")
    _require_source_file(course)
    _require_source_file(scenario)
    course_payload = _read_validated_source(course, _EXPECTED_COURSE, "course")
    scenario_payload = _read_validated_source(
        scenario, _EXPECTED_SCENARIO, "scenario"
    )
    return CompetitionSources(
        course,
        scenario,
        hashlib.sha256(course_payload).hexdigest(),
        hashlib.sha256(scenario_payload).hexdigest(),
    )


def _competition_from_resolved(
    document: Any, configuration_dir: Path
) -> CompetitionSources | None:
    if document is None:
        return None
    if not isinstance(document, dict) or set(document) != _RESOLVED_COMPETITION_FIELDS:
        raise ValueError("resolved competition configuration has missing or unknown keys")
    if document["course"] != "course.yaml" or document["scenario"] != "scenario.yaml":
        raise ValueError("resolved competition sources must use safe artifact names")
    for field in ("course_sha256", "scenario_sha256"):
        value = document[field]
        if (
            not isinstance(value, str)
            or len(value) != 64
            or any(character not in "0123456789abcdef" for character in value)
        ):
            raise ValueError(f"{field} must be a SHA-256 digest")
    course = configuration_dir / "course.yaml"
    scenario = configuration_dir / "scenario.yaml"
    _require_source_file(course)
    _require_source_file(scenario)
    course_payload = _read_validated_source(course, _EXPECTED_COURSE, "course")
    scenario_payload = _read_validated_source(
        scenario, _EXPECTED_SCENARIO, "scenario"
    )
    if hashlib.sha256(course_payload).hexdigest() != document["course_sha256"]:
        raise ValueError("course_sha256 does not match the copied configuration")
    if hashlib.sha256(scenario_payload).hexdigest() != document["scenario_sha256"]:
        raise ValueError("scenario_sha256 does not match the copied configuration")
    return CompetitionSources(
        course,
        scenario,
        document["course_sha256"],
        document["scenario_sha256"],
    )


def _validate_json_object(payload: bytes, name: str) -> None:
    def reject_constant(value: str) -> None:
        raise ValueError(f"non-standard JSON number {value}")

    def unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        document: dict[str, Any] = {}
        for key, value in pairs:
            if key in document:
                raise ValueError(f"duplicate JSON key {key!r}")
            document[key] = value
        return document

    try:
        document = json.loads(
            payload.decode("utf-8"),
            parse_constant=reject_constant,
            object_pairs_hook=unique_object,
        )
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError(f"QGC {name} source must contain valid UTF-8 JSON") from exc
    if not isinstance(document, dict):
        raise ValueError(f"QGC {name} source must contain a JSON object")


def _open_directory_nofollow(path: Path, name: str) -> int:
    absolute = Path(os.path.abspath(os.fspath(path)))
    flags = os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_CLOEXEC", 0)
    nofollow = getattr(os, "O_NOFOLLOW", 0)
    descriptor: int | None = None
    try:
        descriptor = os.open(absolute.anchor, flags)
        for part in absolute.parts[1:]:
            child = os.open(part, flags | nofollow, dir_fd=descriptor)
            os.close(descriptor)
            descriptor = child
        return descriptor
    except OSError as exc:
        if descriptor is not None:
            os.close(descriptor)
        raise ValueError(f"{name} must be a non-symlink directory") from exc


def _read_regular_relative(
    directory_fd: int, relative: Path, description: str
) -> tuple[bytes, tuple[int, int]]:
    directory_flags = os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_CLOEXEC", 0)
    nofollow = getattr(os, "O_NOFOLLOW", 0)
    descriptors: list[int] = []
    try:
        current = os.dup(directory_fd)
        descriptors.append(current)
        for part in relative.parts[:-1]:
            current = os.open(part, directory_flags | nofollow, dir_fd=current)
            descriptors.append(current)
        final = os.open(
            relative.parts[-1],
            os.O_RDONLY | nofollow | getattr(os, "O_NONBLOCK", 0),
            dir_fd=current,
        )
        descriptors.append(final)
        metadata = os.fstat(final)
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError(
                f"{description} must be a regular non-symlink file"
            )
        with os.fdopen(final, "rb", closefd=False) as stream:
            payload = stream.read()
        return payload, (metadata.st_dev, metadata.st_ino)
    except OSError as exc:
        raise ValueError(f"{description} must be a regular non-symlink file") from exc
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def _qgc_source(
    template_fd: int, value: Any, name: str
) -> tuple[bytes, tuple[int, int]]:
    if not isinstance(value, str) or not value:
        raise ValueError(f"QGC {name} source must be a relative path")
    path = Path(value)
    if path.is_absolute():
        absolute = _canonical_absolute_path(path, label=f"QGC {name} source")
        directory_fd = _open_directory_nofollow(
            absolute.parent, f"QGC {name} source directory"
        )
        try:
            payload, identity = _read_regular_relative(
                directory_fd, Path(absolute.name), f"QGC {name} source"
            )
        finally:
            os.close(directory_fd)
        _validate_json_object(payload, name)
        return payload, identity
    relative = path
    if (
        value.startswith("./")
        or "\x00" in value
        or "\n" in value
        or "\r" in value
        or relative == Path(".")
        or relative.is_absolute()
        or ".." in relative.parts
    ):
        raise ValueError(f"QGC {name} source must be a safe relative path")
    payload, identity = _read_regular_relative(
        template_fd, relative, f"QGC {name} source"
    )
    _validate_json_object(payload, name)
    return payload, identity


def _qgc_from_template(document: Any, template_fd: int) -> QGCSources:
    if document is None:
        raise ValueError("QGC configuration must be an object")
    if not isinstance(document, dict) or set(document) != _TEMPLATE_QGC_FIELDS:
        raise ValueError("QGC configuration has missing or unknown keys")
    if any(not isinstance(document[field], str) for field in _QGC_ARTIFACT_NAMES):
        raise ValueError("QGC source paths must be strings")
    relatives = tuple(Path(document[field]) for field in _QGC_ARTIFACT_NAMES)
    if len(set(relatives)) != len(_QGC_ARTIFACT_NAMES):
        raise ValueError("QGC source paths must be pairwise distinct")
    payloads = {}
    identities = set()
    for field in _QGC_ARTIFACT_NAMES:
        payload, identity = _qgc_source(template_fd, document[field], field)
        if identity in identities:
            raise ValueError("QGC source files must be pairwise distinct")
        identities.add(identity)
        payloads[field] = payload
    return QGCSources(
        **payloads,
        attempt_state_root=_canonical_absolute_path(
            document["attempt_state_root"], label="QGC attempt state root"
        ),
    )


def _qgc_document(qgc: QGCSources) -> dict[str, str]:
    document = {
        field: artifact_name for field, artifact_name in _QGC_ARTIFACT_NAMES.items()
    }
    document.update(
        {f"{field}_sha256": qgc._digest(field) for field in _QGC_ARTIFACT_NAMES}
    )
    document["attempt_state_id"] = qgc.attempt_state_id
    document["attempt_state_root"] = str(qgc.attempt_state_root)
    return document


def _sha256(value: Any, field: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError(f"{field} must be a lowercase SHA-256 digest")
    return value


def _qgc_from_resolved(document: Any, configuration_fd: int) -> QGCSources:
    if document is None:
        raise ValueError("resolved QGC configuration must be an object")
    if not isinstance(document, dict) or set(document) != _RESOLVED_QGC_FIELDS:
        raise ValueError("resolved QGC configuration has missing or unknown keys")
    payloads = {}
    identities = set()
    for field, artifact_name in _QGC_ARTIFACT_NAMES.items():
        if document[field] != artifact_name:
            raise ValueError("resolved QGC sources must use canonical artifact names")
        digest = _sha256(document[f"{field}_sha256"], f"{field}_sha256")
        payload, identity = _read_regular_relative(
            configuration_fd, Path(artifact_name), f"QGC {field} source"
        )
        if identity in identities:
            raise ValueError("resolved QGC source files must be pairwise distinct")
        identities.add(identity)
        _validate_json_object(payload, field)
        if hashlib.sha256(payload).hexdigest() != digest:
            raise ValueError(f"{field}_sha256 does not match the copied configuration")
        payloads[field] = payload
    qgc = QGCSources(
        **payloads,
        attempt_state_root=_canonical_absolute_path(
            document["attempt_state_root"], label="QGC attempt state root"
        ),
    )
    if document["attempt_state_id"] != qgc.attempt_state_id:
        raise ValueError("attempt_state_id does not match the deployment profile digest")
    return qgc


def _template_from_document(
    document: dict[str, Any], template_dir: Path, template_fd: int
) -> RunTemplate:
    recording, runtime_profile, simulation = _validate_common(
        document, _TEMPLATE_FIELDS
    )
    return RunTemplate(
        world=document["world"],
        vehicle=document["vehicle"],
        mission=document["mission"],
        scenario=document["scenario"],
        output_root=Path(document["output_root"]).resolve(),
        max_wall_seconds=document["max_wall_seconds"],
        startup_wall_seconds=document["startup_wall_seconds"],
        finalization_wall_seconds=document["finalization_wall_seconds"],
        recording=recording,
        runtime_profile=runtime_profile,
        simulation=simulation,
        competition=_competition_from_template(document.get("competition"), template_dir),
        qgc=(
            _qgc_from_template(document["qgc"], template_fd)
            if "qgc" in document
            else None
        ),
    )


def _document_without_checksum(config: RunConfig) -> dict[str, Any]:
    document = {
        "run_id": config.run_id,
        "world": config.world,
        "vehicle": config.vehicle,
        "mission": config.mission,
        "scenario": config.scenario,
        "output_root": str(config.output_root),
        "max_wall_seconds": config.max_wall_seconds,
        "startup_wall_seconds": config.startup_wall_seconds,
        "finalization_wall_seconds": config.finalization_wall_seconds,
        "recording": {
            "width_px": config.recording.width_px,
            "height_px": config.recording.height_px,
            "fps": config.recording.fps,
            "encoding": config.recording.encoding,
        },
        "runtime_profile": config.runtime_profile,
    }
    if config.simulation is not None:
        document["simulation"] = {
            "seed": config.simulation.seed,
            "duration_sim_seconds": _duration_seconds(config.simulation.duration_ns),
            "public_epoch_native_sim_seconds": _duration_seconds(
                config.simulation.public_epoch_native_ns
            ),
            "target_real_time_factor": config.simulation.target_real_time_factor,
        }
    if config.competition is not None:
        document["competition"] = {
            "course": "course.yaml",
            "scenario": "scenario.yaml",
            "course_sha256": config.competition.course_sha256,
            "scenario_sha256": config.competition.scenario_sha256,
        }
    if config.qgc is not None:
        document["qgc"] = _qgc_document(config.qgc)
    return document


def _checksum(document: dict[str, Any]) -> str:
    canonical = json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(canonical).hexdigest()


def resolve_run_config(
    path: str | Path, run_id_factory: Callable[[], UUID] = uuid4
) -> RunConfig:
    """Validate an operator template and bind it to one generated run identity."""
    source = Path(os.path.abspath(os.fspath(path)))
    template_fd = _open_directory_nofollow(source.parent, "template directory")
    try:
        template_payload, _identity = _read_regular_relative(
            template_fd, Path(source.name), "run template"
        )
        document = _read_document_payload(template_payload, source)
        template = _template_from_document(document, source.parent, template_fd)
    finally:
        os.close(template_fd)
    if template.mission == "comp2026_auto" and template.competition is None:
        raise ValueError("comp2026_auto requires competition source configuration")
    generated = run_id_factory()
    if not isinstance(generated, UUID):
        raise ValueError("run_id_factory must return a UUID")
    unresolved = RunConfig(
        run_id=str(generated),
        world=template.world,
        vehicle=template.vehicle,
        mission=template.mission,
        scenario=template.scenario,
        output_root=template.output_root,
        max_wall_seconds=template.max_wall_seconds,
        startup_wall_seconds=template.startup_wall_seconds,
        finalization_wall_seconds=template.finalization_wall_seconds,
        recording=template.recording,
        runtime_profile=template.runtime_profile,
        simulation=template.simulation,
        config_sha256="",
        competition=template.competition,
        qgc=template.qgc,
    )
    return replace(
        unresolved,
        config_sha256=_checksum(_document_without_checksum(unresolved)),
    )


def load_run_config(path: str | Path) -> RunConfig:
    """Load and verify a resolved immutable run configuration snapshot."""
    source = Path(os.path.abspath(os.fspath(path)))
    configuration_fd = _open_directory_nofollow(
        source.parent, "configuration directory"
    )
    try:
        resolved_payload, _identity = _read_regular_relative(
            configuration_fd, Path(source.name), "resolved run configuration"
        )
        document = _read_document_payload(resolved_payload, source)
        recording, runtime_profile, simulation = _validate_common(
            document, _RESOLVED_FIELDS
        )
        try:
            run_uuid = UUID(document["run_id"])
        except (ValueError, AttributeError, TypeError) as exc:
            raise ValueError("run_id must be a valid UUID") from exc
        if str(run_uuid) != document["run_id"]:
            raise ValueError("run_id must be a canonical UUID")
        checksum = document["config_sha256"]
        if not isinstance(checksum, str) or len(checksum) != 64:
            raise ValueError("config_sha256 must be a SHA-256 digest")
        without_checksum = {
            key: value for key, value in document.items() if key != "config_sha256"
        }
        if _checksum(without_checksum) != checksum:
            raise ValueError("config_sha256 does not match the resolved configuration")
        qgc = (
            _qgc_from_resolved(document["qgc"], configuration_fd)
            if "qgc" in document
            else None
        )
    finally:
        os.close(configuration_fd)
    competition = _competition_from_resolved(document.get("competition"), source.parent)
    if document["mission"] == "comp2026_auto" and competition is None:
        raise ValueError("comp2026_auto requires competition source configuration")
    return RunConfig(
        run_id=str(run_uuid),
        world=document["world"],
        vehicle=document["vehicle"],
        mission=document["mission"],
        scenario=document["scenario"],
        output_root=Path(document["output_root"]).resolve(),
        max_wall_seconds=document["max_wall_seconds"],
        startup_wall_seconds=document["startup_wall_seconds"],
        finalization_wall_seconds=document["finalization_wall_seconds"],
        recording=recording,
        runtime_profile=runtime_profile,
        simulation=simulation,
        config_sha256=checksum,
        competition=competition,
        qgc=qgc,
    )


def _destination_exists(directory_fd: int, name: str) -> bool:
    try:
        os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
    except FileNotFoundError:
        return False
    return True


def _write_exclusive(directory_fd: int, name: str, payload: bytes) -> None:
    descriptor = os.open(
        name,
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0),
        0o644,
        dir_fd=directory_fd,
    )
    try:
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    finally:
        os.close(descriptor)


def write_resolved_config(run_dir: str | Path, config: RunConfig) -> Path:
    """Durably create the resolved snapshot without overwriting an existing run."""
    configuration_dir = Path(run_dir) / "configuration"
    target = configuration_dir / "run.json"
    target_names = ["run.json"]
    if config.competition is not None:
        target_names.extend(("course.yaml", "scenario.yaml"))
    if config.qgc is not None:
        target_names.extend(_QGC_ARTIFACT_NAMES.values())

    try:
        run_uuid = UUID(config.run_id)
    except (ValueError, AttributeError, TypeError) as exc:
        raise ValueError("run_id must be a valid UUID") from exc
    if str(run_uuid) != config.run_id:
        raise ValueError("run_id must be a canonical UUID")
    document = _document_without_checksum(config)
    if _checksum(document) != config.config_sha256:
        raise ValueError("config_sha256 does not match the resolved configuration")
    persisted = {**document, "config_sha256": config.config_sha256}
    _validate_common(persisted, _RESOLVED_FIELDS)

    if configuration_dir.is_symlink():
        raise FileExistsError(configuration_dir)
    configuration_dir.mkdir(parents=True, exist_ok=True)
    directory_fd = _open_directory_nofollow(
        configuration_dir, "configuration directory"
    )
    try:
        for name in target_names:
            if _destination_exists(directory_fd, name):
                raise FileExistsError(configuration_dir / name)

        source_payloads: tuple[tuple[str, bytes], ...] = ()
        if config.competition is not None:
            _require_source_file(config.competition.course_source)
            _require_source_file(config.competition.scenario_source)
            course_payload = config.competition.course_source.read_bytes()
            scenario_payload = config.competition.scenario_source.read_bytes()
            if (
                hashlib.sha256(course_payload).hexdigest()
                != config.competition.course_sha256
            ):
                raise ValueError(
                    "course source changed after run configuration resolution"
                )
            if (
                hashlib.sha256(scenario_payload).hexdigest()
                != config.competition.scenario_sha256
            ):
                raise ValueError(
                    "scenario source changed after run configuration resolution"
                )
            source_payloads = (
                ("course.yaml", course_payload),
                ("scenario.yaml", scenario_payload),
            )
        if config.qgc is not None:
            source_payloads += tuple(
                (_QGC_ARTIFACT_NAMES[field], payload)
                for field, payload in config.qgc._payloads()
            )

        for destination, payload in source_payloads:
            _write_exclusive(directory_fd, destination, payload)
        run_payload = (
            json.dumps(persisted, sort_keys=True, indent=2) + "\n"
        ).encode("utf-8")
        _write_exclusive(directory_fd, "run.json", run_payload)
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
    return target
