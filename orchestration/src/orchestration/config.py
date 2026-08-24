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
_PROFILE_FIELDS = {"runtime_profile", "simulation"}
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
    "target_real_time_factor",
}
CAMERA_INTERVAL_NS = 50_000_000

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
class SimulationConfig:
    seed: int
    duration_ns: int
    target_real_time_factor: float

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


def _validate_recording(document: Any) -> RecordingConfig:
    if not isinstance(document, dict) or set(document) != _RECORDING_FIELDS:
        raise ValueError("recording configuration has missing or unknown keys")
    width = document["width_px"]
    height = document["height_px"]
    if (
        type(width) is not int
        or type(height) is not int
        or (width, height) != (320, 240)
    ):
        raise ValueError("recording dimensions must be exactly 320x240")
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
        raise ValueError("target_real_time_factor must be exactly 0.1")
    try:
        target_decimal = Decimal(str(target))
    except InvalidOperation as exc:
        raise ValueError("target_real_time_factor must be exactly 0.1") from exc
    if not target_decimal.is_finite() or target_decimal != Decimal("0.1"):
        raise ValueError("target_real_time_factor must be exactly 0.1")
    return SimulationConfig(seed, duration_ns, float(target_decimal))


def _duration_seconds(duration_ns: int) -> int | float:
    seconds, remainder_ns = divmod(duration_ns, 1_000_000_000)
    if remainder_ns == 0:
        return float(seconds) if seconds <= 2**53 else seconds
    return float(Decimal(duration_ns) / Decimal(1_000_000_000))


def _validate_common(
    document: dict[str, Any], required_fields: set[str]
) -> tuple[RecordingConfig, str, SimulationConfig | None]:
    fields = set(document)
    if not required_fields <= fields or not fields <= required_fields | _PROFILE_FIELDS:
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
    return _validate_recording(document["recording"]), runtime_profile, simulation


def _template_from_document(document: dict[str, Any]) -> RunTemplate:
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
            "target_real_time_factor": config.simulation.target_real_time_factor,
        }
    return document


def _checksum(document: dict[str, Any]) -> str:
    canonical = json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(canonical).hexdigest()


def resolve_run_config(
    path: str | Path, run_id_factory: Callable[[], UUID] = uuid4
) -> RunConfig:
    """Validate an operator template and bind it to one generated run identity."""
    template = _template_from_document(_read_document(path))
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
    )
    return replace(
        unresolved,
        config_sha256=_checksum(_document_without_checksum(unresolved)),
    )


def load_run_config(path: str | Path) -> RunConfig:
    """Load and verify a resolved immutable run configuration snapshot."""
    document = _read_document(path)
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
    )


def write_resolved_config(run_dir: str | Path, config: RunConfig) -> Path:
    """Durably create the resolved snapshot without overwriting an existing run."""
    configuration_dir = Path(run_dir) / "configuration"
    target = configuration_dir / "run.json"
    if target.exists():
        raise FileExistsError(target)

    try:
        UUID(config.run_id)
    except (ValueError, AttributeError, TypeError) as exc:
        raise ValueError("run_id must be a valid UUID") from exc
    document = _document_without_checksum(config)
    if _checksum(document) != config.config_sha256:
        raise ValueError("config_sha256 does not match the resolved configuration")
    persisted = {**document, "config_sha256": config.config_sha256}
    _validate_common(persisted, _RESOLVED_FIELDS)

    configuration_dir.mkdir(parents=True, exist_ok=True)
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
