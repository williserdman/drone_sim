"""Immutable inventory and atomic persistence for a simulation run bundle."""

from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path


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
_CHUNK_SIZE = 1024 * 1024


@dataclass(frozen=True)
class ArtifactRecord:
    relative_path: str
    size_bytes: int | None
    sha256: str | None
    validation: str

    def to_dict(self) -> dict[str, object]:
        return {
            "relative_path": self.relative_path,
            "size_bytes": self.size_bytes,
            "sha256": self.sha256,
            "validation": self.validation,
        }


@dataclass(frozen=True)
class RunManifest:
    run_id: str
    terminal_status: str
    reason: str
    artifacts: tuple[ArtifactRecord, ...]
    achieved_score: float | None = None
    maximum_available_score: float | None = None
    scoring_checksum: str | None = None
    evidence_paths: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        return {
            "run_id": self.run_id,
            "terminal_status": self.terminal_status,
            "reason": self.reason,
            "artifacts": [record.to_dict() for record in self.artifacts],
            "scoring": {
                "achieved_score": self.achieved_score,
                "maximum_available_score": self.maximum_available_score,
                "scoring_checksum": self.scoring_checksum,
                "evidence_paths": list(self.evidence_paths),
            },
        }


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(_CHUNK_SIZE):
            digest.update(chunk)
    return digest.hexdigest()


def _record(run_directory: Path, relative_path: str) -> ArtifactRecord:
    path = run_directory / relative_path
    expects_directory = relative_path in REQUIRED_DIRECTORY_PATHS
    if not path.exists():
        return ArtifactRecord(relative_path, None, None, "missing")
    if expects_directory:
        if not path.is_dir():
            return ArtifactRecord(relative_path, None, None, "invalid")
        return ArtifactRecord(relative_path, None, None, "present")
    if not path.is_file():
        return ArtifactRecord(relative_path, None, None, "invalid")
    return ArtifactRecord(relative_path, path.stat().st_size, _sha256(path), "present")


def _require_finite_score(field_name: str, value: float | None) -> None:
    if value is not None and not math.isfinite(value):
        raise ValueError(f"{field_name} must be finite or None")


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
    """Inventory all required bundle paths without hiding incomplete artifacts."""
    if terminal_status not in TERMINAL_STATUSES:
        raise ValueError(f"terminal_status must be one of {sorted(TERMINAL_STATUSES)}")
    _require_finite_score("achieved_score", achieved_score)
    _require_finite_score("maximum_available_score", maximum_available_score)
    records = tuple(_record(Path(run_directory), relative_path) for relative_path in REQUIRED_ARTIFACT_PATHS)
    if terminal_status == "COMPLETED" and any(record.validation != "present" for record in records):
        raise ValueError("COMPLETED manifests require every required artifact to be present")
    return RunManifest(
        run_id=run_id,
        terminal_status=terminal_status,
        reason=reason,
        artifacts=records,
        achieved_score=achieved_score,
        maximum_available_score=maximum_available_score,
        scoring_checksum=scoring_checksum,
        evidence_paths=tuple(evidence_paths),
    )


def write_manifest_atomic(run_directory: Path | str, manifest: RunManifest) -> Path:
    """Durably write ``manifest.json`` without exposing a partial JSON document."""
    directory = Path(run_directory)
    target = directory / "manifest.json"
    temporary = directory / "manifest.json.tmp"
    payload = json.dumps(
        manifest.to_dict(),
        allow_nan=False,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, target)
    return target
