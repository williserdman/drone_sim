"""Immutable operator-template resolution and run configuration persistence."""

from collections.abc import Callable
from dataclasses import dataclass, replace
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
_STRING_FIELDS = ("world", "vehicle", "mission", "scenario", "output_root")
_DEADLINE_FIELDS = (
    "max_wall_seconds",
    "startup_wall_seconds",
    "finalization_wall_seconds",
)
_RECORDING_FIELDS = {"width_px", "height_px", "fps", "encoding"}


@dataclass(frozen=True)
class RecordingConfig:
    width_px: int
    height_px: int
    fps: int
    encoding: str


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
    config_sha256: str


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
    if any(
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < 2
        or value % 2
        for value in (width, height)
    ):
        raise ValueError("recording dimensions must be positive even integers")
    if document["fps"] != 20 or isinstance(document["fps"], bool):
        raise ValueError("recording fps must be 20")
    if document["encoding"] != "rgb8":
        raise ValueError("recording encoding must be rgb8")
    return RecordingConfig(width, height, document["fps"], document["encoding"])


def _validate_common(document: dict[str, Any], expected_fields: set[str]) -> RecordingConfig:
    if set(document) != expected_fields:
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
    return _validate_recording(document["recording"])


def _template_from_document(document: dict[str, Any]) -> RunTemplate:
    recording = _validate_common(document, _TEMPLATE_FIELDS)
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
    )


def _document_without_checksum(config: RunConfig) -> dict[str, Any]:
    return {
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
    }


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
        config_sha256="",
    )
    return replace(
        unresolved,
        config_sha256=_checksum(_document_without_checksum(unresolved)),
    )


def load_run_config(path: str | Path) -> RunConfig:
    """Load and verify a resolved immutable run configuration snapshot."""
    document = _read_document(path)
    recording = _validate_common(document, _RESOLVED_FIELDS)
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
        config_sha256=checksum,
    )


def write_resolved_config(run_dir: str | Path, config: RunConfig) -> Path:
    """Durably create the resolved snapshot without overwriting an existing run."""
    configuration_dir = Path(run_dir) / "configuration"
    target = configuration_dir / "run.json"
    if target.exists():
        raise FileExistsError(target)

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
