"""Immutable operator-template resolution and run configuration persistence."""

from collections.abc import Callable
from dataclasses import dataclass, replace
from decimal import Decimal, InvalidOperation
import hashlib
import json
import os
from pathlib import Path
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
_OPTIONAL_FIELDS = {"runtime_profile", "simulation", "competition"}
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

    @property
    def expected_camera_frames(self) -> int:
        if self.simulation is None:
            return 2 * self.recording.fps
        return self.simulation.expected_camera_frames

    @property
    def topology(self) -> RuntimeTopology:
        return PHASE3_TOPOLOGY if self.runtime_profile == "phase3" else PHASE2_TOPOLOGY


def _read_document(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid run configuration: {source}") from exc
    if not isinstance(document, dict):
        raise ValueError("run configuration must be a JSON object")
    return document


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
    required_dimensions = (640, 480) if mission == "comp2026_auto" else (320, 240)
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


def _template_from_document(document: dict[str, Any], template_dir: Path) -> RunTemplate:
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
    return document


def _checksum(document: dict[str, Any]) -> str:
    canonical = json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(canonical).hexdigest()


def resolve_run_config(
    path: str | Path, run_id_factory: Callable[[], UUID] = uuid4
) -> RunConfig:
    """Validate an operator template and bind it to one generated run identity."""
    source = Path(path).resolve()
    document = _read_document(source)
    template = _template_from_document(document, source.parent)
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
    )
    return replace(
        unresolved,
        config_sha256=_checksum(_document_without_checksum(unresolved)),
    )


def load_run_config(path: str | Path) -> RunConfig:
    """Load and verify a resolved immutable run configuration snapshot."""
    source = Path(path).resolve()
    document = _read_document(source)
    recording, runtime_profile, simulation = _validate_common(
        document, _RESOLVED_FIELDS
    )
    try:
        run_uuid = UUID(document["run_id"])
    except (ValueError, AttributeError, TypeError) as exc:
        raise ValueError("run_id must be a valid UUID") from exc
    checksum = document["config_sha256"]
    if not isinstance(checksum, str) or len(checksum) != 64:
        raise ValueError("config_sha256 must be a SHA-256 digest")
    without_checksum = {key: value for key, value in document.items() if key != "config_sha256"}
    if _checksum(without_checksum) != checksum:
        raise ValueError("config_sha256 does not match the resolved configuration")
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
    )


def write_resolved_config(run_dir: str | Path, config: RunConfig) -> Path:
    """Durably create the resolved snapshot without overwriting an existing run."""
    configuration_dir = Path(run_dir) / "configuration"
    target = configuration_dir / "run.json"
    course_target = configuration_dir / "course.yaml"
    scenario_target = configuration_dir / "scenario.yaml"
    targets = [target]
    if config.competition is not None:
        targets.extend((course_target, scenario_target))
    for candidate in targets:
        if candidate.exists() or candidate.is_symlink():
            raise FileExistsError(candidate)

    try:
        UUID(config.run_id)
    except (ValueError, AttributeError, TypeError) as exc:
        raise ValueError("run_id must be a valid UUID") from exc
    document = _document_without_checksum(config)
    if _checksum(document) != config.config_sha256:
        raise ValueError("config_sha256 does not match the resolved configuration")
    persisted = {**document, "config_sha256": config.config_sha256}
    _validate_common(persisted, _RESOLVED_FIELDS)

    source_payloads: tuple[tuple[Path, bytes], ...] = ()
    if config.competition is not None:
        _require_source_file(config.competition.course_source)
        _require_source_file(config.competition.scenario_source)
        course_payload = config.competition.course_source.read_bytes()
        scenario_payload = config.competition.scenario_source.read_bytes()
        if hashlib.sha256(course_payload).hexdigest() != config.competition.course_sha256:
            raise ValueError("course source changed after run configuration resolution")
        if hashlib.sha256(scenario_payload).hexdigest() != config.competition.scenario_sha256:
            raise ValueError("scenario source changed after run configuration resolution")
        source_payloads = (
            (course_target, course_payload),
            (scenario_target, scenario_payload),
        )

    configuration_dir.mkdir(parents=True, exist_ok=True)
    for destination, payload in source_payloads:
        with destination.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    with target.open("x", encoding="utf-8") as stream:
        json.dump(persisted, stream, sort_keys=True, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    directory_fd = os.open(configuration_dir, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
    return target
