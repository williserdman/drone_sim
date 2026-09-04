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
    validate_gazebo_state,
    validate_nonempty_regular_file,
    validate_regular_file,
    validate_tree,
)


ArtifactValidator = Callable[[Path, str], ValidationResult]
DeadlineCheck = Callable[[], None]


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


@dataclass(frozen=True)
class FinalizationResult:
    """Authoritative terminal facts returned only after manifest publication."""

    path: Path
    run_id: str
    terminal_status: str
    reason: str

    def __post_init__(self) -> None:
        if not isinstance(self.path, Path):
            raise TypeError("path must be a Path")
        if not isinstance(self.run_id, str) or not self.run_id:
            raise ValueError("run_id must be nonempty")
        if self.terminal_status not in {"COMPLETED", "FAILED", "ABORTED"}:
            raise ValueError("terminal_status is invalid")
        if not isinstance(self.reason, str):
            raise TypeError("reason must be a string")


class ArtifactSession:
    """Validate one quiescent run directory and commit its manifest once."""

    def __init__(
        self,
        run_directory: Path | str,
        validators: Mapping[str, ArtifactValidator] | None = None,
        *,
        deadline_check: DeadlineCheck | None = None,
        commit_deadline_check: DeadlineCheck | None = None,
        physical_gazebo: bool = False,
    ) -> None:
        if not isinstance(physical_gazebo, bool):
            raise TypeError("physical_gazebo must be a boolean")
        self.run_directory = Path(run_directory)
        self._deadline_check = deadline_check
        self._commit_deadline_check = commit_deadline_check or deadline_check
        self._physical_gazebo = physical_gazebo
        registry: dict[str, ArtifactValidator] = {
            relative_path: self._default_validator(relative_path)
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

    def _check_deadline(self) -> None:
        if self._deadline_check is not None:
            self._deadline_check()

    def _default_validator(self, relative_path: str) -> ArtifactValidator:
        if self._physical_gazebo and relative_path == "gazebo/server.log":
            validator = validate_nonempty_regular_file
        elif self._physical_gazebo and relative_path == "gazebo/state":
            validator = validate_gazebo_state
        else:
            validator = (
                validate_tree
                if relative_path in REQUIRED_DIRECTORY_PATHS
                else validate_regular_file
            )
        return lambda root, path: validator(
            root, path, deadline_check=self._deadline_check
        )

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
        records: list[ArtifactRecord] = []
        exhausted = False
        for relative_path in REQUIRED_ARTIFACT_PATHS:
            if exhausted:
                result = ValidationResult(
                    ValidationStatus.INVALID,
                    None,
                    None,
                    "finalization deadline exhausted before validation",
                )
            else:
                try:
                    self._check_deadline()
                    result = self._validators[relative_path](
                        self.run_directory, relative_path
                    )
                    self._check_deadline()
                except TimeoutError:
                    exhausted = True
                    result = ValidationResult(
                        ValidationStatus.INVALID,
                        None,
                        None,
                        "finalization deadline exhausted during validation",
                    )
            records.append(self._artifact_record(relative_path, result))
        return tuple(records)

    def _optional_paths(self) -> tuple[str, ...]:
        discovered: set[str] = set()
        autotune_parameters = self.run_directory / "ardupilot_sitl/autotune-roll.parm"
        if autotune_parameters.exists():
            discovered.add("ardupilot_sitl/autotune-roll.parm")
        docker_logs = self.run_directory / "logs/docker"
        if docker_logs.exists():
            for directory, directory_names, file_names in os.walk(
                docker_logs, followlinks=False
            ):
                self._check_deadline()
                directory_path = Path(directory)
                directory_names[:] = sorted(directory_names)
                for name in sorted(file_names):
                    self._check_deadline()
                    discovered.add(
                        (directory_path / name)
                        .relative_to(self.run_directory)
                        .as_posix()
                    )

        if self.run_directory.exists():
            for candidate in self.run_directory.rglob("*.partial"):
                self._check_deadline()
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
        records: list[ArtifactRecord] = []
        try:
            paths = self._optional_paths()
            for relative_path in paths:
                self._check_deadline()
                result = validate_regular_file(
                    self.run_directory,
                    relative_path,
                    deadline_check=self._deadline_check,
                )
                records.append(self._artifact_record(relative_path, result))
        except TimeoutError:
            pass
        return tuple(records)

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

    def finalize_with_result(self, request: FinalizationInput) -> FinalizationResult:
        """Commit the manifest and return its immutable authoritative facts."""
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
        validate_manifest(manifest, deadline_check=self._commit_deadline_check)
        path = write_manifest_atomic(
            self.run_directory,
            manifest,
            deadline_check=self._commit_deadline_check,
        )
        return FinalizationResult(
            path,
            manifest.run_id,
            manifest.terminal_status,
            manifest.reason,
        )

    def finalize(self, request: FinalizationInput) -> Path:
        """Backward-compatible path-only finalization wrapper."""
        return self.finalize_with_result(request).path


__all__ = [
    "ArtifactSession",
    "FinalizationConflict",
    "FinalizationInput",
    "FinalizationResult",
]
