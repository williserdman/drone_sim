"""Run-scoped validator registry and authoritative manifest finalization."""

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import stat

from .manifest import (
    ArtifactRecord,
    ConfigurationRecord,
    FinalizationConflict,
    ImageDigest,
    REQUIRED_ARTIFACT_PATHS,
    REQUIRED_DIRECTORY_PATHS,
    RunManifest,
    SimulationTiming,
    SourceRevision,
    WallTiming,
    validate_manifest,
    write_manifest_atomic,
)
from .validation import (
    ValidationResult,
    ValidationStatus,
    validate_regular_file,
    validate_tree,
)


ArtifactValidator = Callable[[Path, str], ValidationResult]


@dataclass(frozen=True)
class FinalizationInput:
    run_id: str
    requested_terminal: str
    reason: str
    sim_start_ns: int | None
    sim_end_ns: int | None
    wall_started_at: datetime
    wall_ended_at: datetime
    source_revisions: tuple[SourceRevision, ...]
    image_digests: tuple[ImageDigest, ...]
    configuration_records: tuple[ConfigurationRecord, ...]
    achieved_score: float | None
    maximum_available_score: float | None
    scoring_checksum: str | None
    evidence_paths: tuple[str, ...]


class ArtifactSession:
    """Validate one quiescent run directory and commit its manifest once."""

    def __init__(
        self,
        run_directory: Path | str,
        validators: Mapping[str, ArtifactValidator] | None = None,
    ) -> None:
        self.run_directory = Path(run_directory)
        registry: dict[str, ArtifactValidator] = {
            relative_path: (
                validate_tree
                if relative_path in REQUIRED_DIRECTORY_PATHS
                else validate_regular_file
            )
            for relative_path in REQUIRED_ARTIFACT_PATHS
        }
        if validators is not None:
            unknown = set(validators) - set(REQUIRED_ARTIFACT_PATHS)
            if unknown:
                raise ValueError(
                    f"validators contain unknown required paths: {sorted(unknown)}"
                )
            registry.update(validators)
        self._validators = registry

    @staticmethod
    def _artifact_record(relative_path: str, result: ValidationResult) -> ArtifactRecord:
        if not isinstance(result, ValidationResult):
            raise TypeError(f"validator for {relative_path!r} must return ValidationResult")
        return ArtifactRecord(
            relative_path=relative_path,
            size_bytes=result.size_bytes,
            sha256=result.sha256,
            validation=result.status.value,
            detail=result.detail,
        )

    def _required_records(self) -> tuple[ArtifactRecord, ...]:
        return tuple(
            self._artifact_record(
                relative_path,
                self._validators[relative_path](self.run_directory, relative_path),
            )
            for relative_path in REQUIRED_ARTIFACT_PATHS
        )

    def _optional_paths(self) -> tuple[str, ...]:
        discovered: set[str] = set()
        docker_logs = self.run_directory / "logs/docker"
        if docker_logs.exists():
            for directory, directory_names, file_names in os.walk(
                docker_logs, followlinks=False
            ):
                directory_path = Path(directory)
                directory_names[:] = sorted(directory_names)
                for name in sorted(file_names):
                    discovered.add(
                        (directory_path / name)
                        .relative_to(self.run_directory)
                        .as_posix()
                    )

        if self.run_directory.exists():
            for candidate in self.run_directory.rglob("*.partial"):
                relative = candidate.relative_to(self.run_directory)
                if relative.parts and relative.parts[0] in {".control", ".status"}:
                    continue
                try:
                    if stat.S_ISDIR(candidate.lstat().st_mode):
                        continue
                except OSError:
                    continue
                discovered.add(relative.as_posix())

        discovered.discard("manifest.json")
        return tuple(
            path
            for path in sorted(discovered)
            if path not in REQUIRED_ARTIFACT_PATHS
            and not path.startswith(".control/")
            and not path.startswith(".status/")
        )

    def _optional_records(self) -> tuple[ArtifactRecord, ...]:
        return tuple(
            self._artifact_record(
                relative_path,
                validate_regular_file(self.run_directory, relative_path),
            )
            for relative_path in self._optional_paths()
        )

    @staticmethod
    def _simulation_timing(request: FinalizationInput) -> SimulationTiming:
        duration = (
            None
            if request.sim_start_ns is None or request.sim_end_ns is None
            else request.sim_end_ns - request.sim_start_ns
        )
        return SimulationTiming(request.sim_start_ns, request.sim_end_ns, duration)

    @staticmethod
    def _wall_timing(request: FinalizationInput) -> WallTiming:
        try:
            duration = (request.wall_ended_at - request.wall_started_at).total_seconds()
        except (TypeError, AttributeError):
            duration = float("nan")
        return WallTiming(request.wall_started_at, request.wall_ended_at, duration)

    def finalize(self, request: FinalizationInput) -> Path:
        """Validate the bundle and durably commit the canonical manifest."""
        required_records = self._required_records()
        incomplete_paths = tuple(
            record.relative_path
            for record in required_records
            if record.validation != ValidationStatus.VALID.value
        )
        effective_terminal = (
            "FAILED"
            if request.requested_terminal == "COMPLETED" and incomplete_paths
            else request.requested_terminal
        )
        manifest = RunManifest(
            run_id=request.run_id,
            terminal_status=effective_terminal,
            reason=request.reason,
            simulation_timing=self._simulation_timing(request),
            wall_timing=self._wall_timing(request),
            source_revisions=tuple(request.source_revisions),
            image_digests=tuple(request.image_digests),
            configurations=tuple(request.configuration_records),
            artifacts=required_records + self._optional_records(),
            incomplete_paths=incomplete_paths,
            achieved_score=request.achieved_score,
            maximum_available_score=request.maximum_available_score,
            scoring_checksum=request.scoring_checksum,
            evidence_paths=tuple(request.evidence_paths),
        )
        validate_manifest(manifest)
        return write_manifest_atomic(self.run_directory, manifest)


__all__ = ["ArtifactSession", "FinalizationConflict", "FinalizationInput"]
