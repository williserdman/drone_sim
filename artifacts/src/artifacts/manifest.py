"""Immutable inventory and atomic persistence for a simulation run bundle."""

from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import tempfile

from .validation import ValidationStatus, validate_regular_file, validate_tree


MODULE_LOGS = (
    "logs/orchestration.jsonl",
    "logs/artifacts.jsonl",
    "logs/companion.jsonl",
    "logs/ardupilot_sitl.jsonl",
    "logs/gazebo.jsonl",
    "logs/electromagnet.jsonl",
    "logs/scorekeeper.jsonl",
)
REQUIRED_DIRECTORY_PATHS = frozenset({"configuration", "gazebo/state", "rosbag"})
REQUIRED_ARTIFACT_PATHS = (
    "configuration",
    "gazebo/server.log",
    "gazebo/state",
    "video/onboard.mp4",
    "video/observer.mp4",
    "rosbag",
    *MODULE_LOGS,
    "scoring/events.jsonl",
    "scoring/result.json",
)
TERMINAL_STATUSES = frozenset({"COMPLETED", "FAILED", "ABORTED"})
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
DeadlineCheck = Callable[[], None]


def _check_deadline(deadline_check: DeadlineCheck | None) -> None:
    if deadline_check is not None:
        deadline_check()


class FinalizationConflict(RuntimeError):
    """Raised when an existing manifest differs from a finalization request."""


@dataclass(frozen=True)
class SourceRevision:
    name: str
    revision: str
    dirty: bool

    def to_dict(self) -> dict[str, object]:
        return {"name": self.name, "revision": self.revision, "dirty": self.dirty}


@dataclass(frozen=True)
class ImageDigest:
    name: str
    digest: str

    def to_dict(self) -> dict[str, str]:
        return {"name": self.name, "digest": self.digest}


@dataclass(frozen=True)
class ConfigurationRecord:
    relative_path: str
    sha256: str

    def to_dict(self) -> dict[str, str]:
        return {"relative_path": self.relative_path, "sha256": self.sha256}


@dataclass(frozen=True)
class SimulationTiming:
    start_ns: int | None
    end_ns: int | None
    duration_ns: int | None

    def to_dict(self) -> dict[str, int | None]:
        return {
            "start_ns": self.start_ns,
            "end_ns": self.end_ns,
            "duration_ns": self.duration_ns,
        }


@dataclass(frozen=True)
class WallTiming:
    started_at: datetime
    ended_at: datetime
    duration_seconds: float

    def to_dict(self) -> dict[str, str | float]:
        return {
            "started_at": self.started_at.isoformat(),
            "ended_at": self.ended_at.isoformat(),
            "duration_seconds": self.duration_seconds,
        }


@dataclass(frozen=True)
class ArtifactRecord:
    relative_path: str
    size_bytes: int | None
    sha256: str | None
    validation: str
    detail: str

    def to_dict(self) -> dict[str, object]:
        return {
            "relative_path": self.relative_path,
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
            "validation": self.validation,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class RunManifest:
    run_id: str
    terminal_status: str
    reason: str
    simulation_timing: SimulationTiming
    wall_timing: WallTiming
    source_revisions: tuple[SourceRevision, ...]
    image_digests: tuple[ImageDigest, ...]
    configurations: tuple[ConfigurationRecord, ...]
    artifacts: tuple[ArtifactRecord, ...]
    incomplete_paths: tuple[str, ...]
    achieved_score: float | None = None
    maximum_available_score: float | None = None
    scoring_checksum: str | None = None
    evidence_paths: tuple[str, ...] = ()
    schema_version: int = 1

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "terminal_status": self.terminal_status,
            "reason": self.reason,
            "simulation_timing": self.simulation_timing.to_dict(),
            "wall_timing": self.wall_timing.to_dict(),
            "source_revisions": [record.to_dict() for record in self.source_revisions],
            "image_digests": [record.to_dict() for record in self.image_digests],
            "configurations": [record.to_dict() for record in self.configurations],
            "artifacts": [record.to_dict() for record in self.artifacts],
            "incomplete_paths": list(self.incomplete_paths),
            "scoring": {
                "achieved_score": self.achieved_score,
                "maximum_available_score": self.maximum_available_score,
                "scoring_checksum": self.scoring_checksum,
                "evidence_paths": list(self.evidence_paths),
            },
        }


def _require_finite_score(field_name: str, value: float | None) -> None:
    if value is not None and (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{field_name} must be finite or None for JSON compliant output")


def _require_digest(
    field_name: str, value: str | None, *, optional: bool = False
) -> None:
    if optional and value is None:
        return
    if not isinstance(value, str) or _SHA256_PATTERN.fullmatch(value) is None:
        raise ValueError(f"{field_name} must be an exact lowercase SHA-256 digest")


def _require_relative_path(field_name: str, value: str) -> None:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field_name} must be a nonempty relative path")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"{field_name} must be contained inside the run directory")


def _validate_timing(simulation: SimulationTiming, wall: WallTiming) -> None:
    simulation_values = (simulation.start_ns, simulation.end_ns)
    if (simulation.start_ns is None) != (simulation.end_ns is None):
        raise ValueError("simulation start and end must both be present or absent")
    if any(
        value is not None
        and (isinstance(value, bool) or not isinstance(value, int) or value < 0)
        for value in simulation_values
    ):
        raise ValueError("simulation times must be nonnegative integers or None")
    expected_duration = (
        None
        if simulation.start_ns is None
        else simulation.end_ns - simulation.start_ns  # type: ignore[operator]
    )
    if expected_duration is not None and expected_duration < 0:
        raise ValueError("simulation end must not precede start")
    if simulation.duration_ns != expected_duration:
        raise ValueError("simulation duration does not match start and end")

    try:
        aware = (
            wall.started_at.utcoffset() is not None
            and wall.ended_at.utcoffset() is not None
        )
    except (AttributeError, ValueError):
        aware = False
    if not aware:
        raise ValueError("wall timestamps must be timezone-aware datetimes")
    expected_wall_duration = (wall.ended_at - wall.started_at).total_seconds()
    if not math.isfinite(expected_wall_duration) or expected_wall_duration < 0:
        raise ValueError("wall end must not precede start")
    if (
        isinstance(wall.duration_seconds, bool)
        or not isinstance(wall.duration_seconds, (int, float))
        or not math.isfinite(wall.duration_seconds)
        or wall.duration_seconds < 0
        or wall.duration_seconds != expected_wall_duration
    ):
        raise ValueError("wall duration must be finite, nonnegative, and match timestamps")


def validate_manifest(
    manifest: RunManifest, *, deadline_check: DeadlineCheck | None = None
) -> None:
    """Validate metadata invariants shared by builders and session finalization."""
    _check_deadline(deadline_check)
    if manifest.schema_version != 1:
        raise ValueError("schema_version must be 1")
    if not isinstance(manifest.run_id, str) or not manifest.run_id:
        raise ValueError("run_id must be nonempty")
    if not isinstance(manifest.reason, str):
        raise ValueError("reason must be a string")
    if manifest.terminal_status not in TERMINAL_STATUSES:
        raise ValueError(f"terminal_status must be one of {sorted(TERMINAL_STATUSES)}")
    _validate_timing(manifest.simulation_timing, manifest.wall_timing)
    _require_finite_score("achieved_score", manifest.achieved_score)
    _require_finite_score("maximum_available_score", manifest.maximum_available_score)
    _require_digest("scoring_checksum", manifest.scoring_checksum, optional=True)

    source_names: set[str] = set()
    for source in manifest.source_revisions:
        _check_deadline(deadline_check)
        if (
            not isinstance(source.name, str)
            or not source.name
            or not isinstance(source.revision, str)
            or not source.revision
            or not isinstance(source.dirty, bool)
        ):
            raise ValueError("source revisions require a name, revision, and dirty flag")
        if source.name in source_names:
            raise ValueError("source revision names must be unique")
        source_names.add(source.name)

    image_names: set[str] = set()
    for image in manifest.image_digests:
        _check_deadline(deadline_check)
        if not isinstance(image.name, str) or not image.name:
            raise ValueError("image digest names must be nonempty")
        if image.name in image_names:
            raise ValueError("image digest names must be unique")
        image_names.add(image.name)
        _require_digest("image digest", image.digest)

    configuration_paths: set[str] = set()
    for configuration in manifest.configurations:
        _check_deadline(deadline_check)
        _require_relative_path("configuration path", configuration.relative_path)
        if configuration.relative_path in configuration_paths:
            raise ValueError("configuration paths must be unique")
        configuration_paths.add(configuration.relative_path)
        _require_digest("configuration sha256", configuration.sha256)

    artifact_paths: set[str] = set()
    valid_statuses = {status.value for status in ValidationStatus}
    for artifact in manifest.artifacts:
        _check_deadline(deadline_check)
        _require_relative_path("artifact path", artifact.relative_path)
        if artifact.relative_path in artifact_paths:
            raise ValueError("artifact paths must be unique")
        artifact_paths.add(artifact.relative_path)
        if artifact.validation not in valid_statuses:
            raise ValueError("artifact validation must be valid, missing, or invalid")
        if not isinstance(artifact.detail, str) or not artifact.detail:
            raise ValueError("artifact detail must be nonempty")
        if artifact.validation == ValidationStatus.VALID.value:
            if (
                isinstance(artifact.size_bytes, bool)
                or not isinstance(artifact.size_bytes, int)
                or artifact.size_bytes < 0
            ):
                raise ValueError("valid artifacts require a nonnegative size")
            _require_digest("artifact sha256", artifact.sha256)
        elif artifact.size_bytes is not None or artifact.sha256 is not None:
            raise ValueError("missing and invalid artifacts cannot carry size or sha256")

    if len(set(manifest.incomplete_paths)) != len(manifest.incomplete_paths):
        raise ValueError("incomplete_paths must be unique")
    for incomplete_path in manifest.incomplete_paths:
        _check_deadline(deadline_check)
        if incomplete_path not in REQUIRED_ARTIFACT_PATHS:
            raise ValueError("incomplete_paths must name required artifact paths")
    required_records = {
        artifact.relative_path: artifact
        for artifact in manifest.artifacts
        if artifact.relative_path in REQUIRED_ARTIFACT_PATHS
    }
    if set(required_records) != set(REQUIRED_ARTIFACT_PATHS):
        raise ValueError("artifacts must contain every required artifact path")
    expected_incomplete = tuple(
        path
        for path in REQUIRED_ARTIFACT_PATHS
        if required_records[path].validation != ValidationStatus.VALID.value
    )
    if manifest.incomplete_paths != expected_incomplete:
        raise ValueError("incomplete_paths must exactly match non-valid required artifacts")
    if manifest.terminal_status == "COMPLETED" and manifest.incomplete_paths:
        raise ValueError("COMPLETED manifests require every required artifact to be valid")
    if len(set(manifest.evidence_paths)) != len(manifest.evidence_paths):
        raise ValueError("evidence paths must be unique")
    for evidence_path in manifest.evidence_paths:
        _check_deadline(deadline_check)
        _require_relative_path("evidence path", evidence_path.split("#", 1)[0])
    _check_deadline(deadline_check)


def _record(run_directory: Path, relative_path: str) -> ArtifactRecord:
    validator = (
        validate_tree
        if relative_path in REQUIRED_DIRECTORY_PATHS
        else validate_regular_file
    )
    result = validator(run_directory, relative_path)
    return ArtifactRecord(
        relative_path,
        result.size_bytes,
        result.sha256,
        result.status.value,
        result.detail,
    )


def build_manifest(
    run_directory: Path | str,
    run_id: str,
    terminal_status: str,
    reason: str,
    *,
    achieved_score: float | None = None,
    maximum_available_score: float | None = None,
    scoring_checksum: str | None = None,
    evidence_paths: tuple[str, ...] = (),
) -> RunManifest:
    """Compatibility builder for the fixed required inventory.

    New controller code should use :class:`ArtifactSession`, which accepts the
    full finalization metadata and discovers optional diagnostics.
    """
    if terminal_status not in TERMINAL_STATUSES:
        raise ValueError(f"terminal_status must be one of {sorted(TERMINAL_STATUSES)}")
    records = tuple(_record(Path(run_directory), path) for path in REQUIRED_ARTIFACT_PATHS)
    incomplete = tuple(
        record.relative_path
        for record in records
        if record.validation != ValidationStatus.VALID.value
    )
    effective_terminal = (
        "FAILED" if terminal_status == "COMPLETED" and incomplete else terminal_status
    )
    epoch = datetime(1970, 1, 1, tzinfo=timezone.utc)
    manifest = RunManifest(
        run_id=run_id,
        terminal_status=effective_terminal,
        reason=reason,
        simulation_timing=SimulationTiming(None, None, None),
        wall_timing=WallTiming(epoch, epoch, 0.0),
        source_revisions=(),
        image_digests=(),
        configurations=(),
        artifacts=records,
        incomplete_paths=incomplete,
        achieved_score=achieved_score,
        maximum_available_score=maximum_available_score,
        scoring_checksum=scoring_checksum,
        evidence_paths=tuple(evidence_paths),
    )
    validate_manifest(manifest)
    return manifest


def canonical_manifest_bytes(
    manifest: RunManifest, *, deadline_check: DeadlineCheck | None = None
) -> bytes:
    validate_manifest(manifest, deadline_check=deadline_check)
    encoder = json.JSONEncoder(
        allow_nan=False,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    chunks: list[bytes] = []
    for chunk in encoder.iterencode(manifest.to_dict()):
        _check_deadline(deadline_check)
        chunks.append(chunk.encode("utf-8"))
    payload = b"".join(chunks)
    _check_deadline(deadline_check)
    return payload


def write_manifest_atomic(
    run_directory: Path | str,
    manifest: RunManifest,
    *,
    deadline_check: DeadlineCheck | None = None,
) -> Path:
    """Durably and idempotently commit ``manifest.json``."""
    directory = Path(run_directory)
    target = directory / "manifest.json"
    payload = canonical_manifest_bytes(manifest, deadline_check=deadline_check)
    _check_deadline(deadline_check)
    directory_fd = os.open(
        directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    )
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=".manifest.json.", suffix=".tmp", dir=directory
        )
        temporary = Path(temporary_name)
        published = False
        try:
            with os.fdopen(descriptor, "wb") as stream:
                view = memoryview(payload)
                written = 0
                while written < len(view):
                    _check_deadline(deadline_check)
                    count = stream.write(view[written : written + 1024 * 1024])
                    if count is None or count <= 0:
                        raise OSError("manifest write made no progress")
                    written += count
                _check_deadline(deadline_check)
                stream.flush()
                _check_deadline(deadline_check)
                os.fsync(stream.fileno())
                _check_deadline(deadline_check)
            try:
                _check_deadline(deadline_check)
                os.link(temporary, target, follow_symlinks=False)
                published = True
            except FileExistsError:
                existing = _read_existing_manifest(
                    directory_fd,
                    manifest.run_id,
                    deadline_check=None,
                )
                if existing == payload:
                    published = True
                    return target
                raise FinalizationConflict(
                    f"run {manifest.run_id!r} is already finalized differently"
                )
            except TimeoutError as timeout:
                # A cooperative checker may expire from inside an injected
                # publication boundary after the kernel has created the hard
                # link.  Resolve that exact ambiguous outcome without another
                # cooperative rejection: identical named bytes are authority.
                try:
                    existing = _read_existing_manifest(
                        directory_fd,
                        manifest.run_id,
                        deadline_check=None,
                    )
                except FinalizationConflict:
                    raise timeout
                if existing != payload:
                    raise FinalizationConflict(
                        f"run {manifest.run_id!r} is already finalized differently"
                    ) from timeout
                published = True
            return target
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
            if not published:
                _check_deadline(deadline_check)
            os.fsync(directory_fd)
            if not published:
                _check_deadline(deadline_check)
    finally:
        os.close(directory_fd)


def _read_existing_manifest(
    directory_fd: int,
    run_id: str,
    *,
    deadline_check: DeadlineCheck | None = None,
) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open("manifest.json", flags, dir_fd=directory_fd)
    except OSError as exc:
        raise FinalizationConflict(
            f"run {run_id!r} has an unreadable existing manifest"
        ) from exc
    try:
        chunks: list[bytes] = []
        while True:
            _check_deadline(deadline_check)
            chunk = os.read(descriptor, 1024 * 1024)
            _check_deadline(deadline_check)
            if not chunk:
                break
            chunks.append(chunk)
        return b"".join(chunks)
    finally:
        os.close(descriptor)
