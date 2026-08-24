"""Descriptor-validated resolution of the immutable Phase 3 world."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import stat

from artifacts.validation import (
    ValidationResult,
    ValidationStatus,
    validate_regular_file,
    validate_tree,
)


@dataclass(frozen=True)
class WorldConfig:
    world: str
    vehicle: str


@dataclass(frozen=True)
class ResolvedWorld:
    path: Path
    world_name: str
    vehicle_id: str
    resource_path: Path
    world_sha256: str
    resource_sha256s: tuple[tuple[str, str], ...]


def _resource_error(result: ValidationResult) -> ValueError:
    detail = result.detail
    if "multiple hard links" in detail or "exactly one hard link" in detail:
        detail = "regular files must have exactly one hard link"
    elif "non-regular file" in detail or "not a regular file" in detail:
        detail = "resource tree must contain only regular files and directories"
    elif "changed during validation" in detail:
        detail = "resource changed during hashing"
    return ValueError(detail)


def _validated_tree(root: Path) -> ValidationResult:
    result = validate_tree(root.parent, root.name)
    if result.status is not ValidationStatus.VALID:
        raise _resource_error(result)
    return result


def _relative_resource_path(path: Path, *, root: Path) -> Path:
    try:
        relative = path.relative_to(root)
    except ValueError as exc:
        raise ValueError("resource path escapes the Gazebo resource root") from exc
    if not relative.parts or ".." in relative.parts:
        raise ValueError("resource path escapes the Gazebo resource root")
    return relative


def _sha256_regular(path: Path, *, root: Path) -> str:
    """Hash one resource through retained no-follow file descriptors."""
    relative = _relative_resource_path(path, root=root)
    result = validate_regular_file(root, relative)
    if result.status is not ValidationStatus.VALID or result.sha256 is None:
        raise _resource_error(result)
    return result.sha256


def _resource_files(root: Path) -> tuple[Path, ...]:
    """Enumerate names only; descriptor validation owns all trust decisions."""
    files: list[Path] = []
    try:
        entries = sorted(root.rglob("*"))
    except OSError as exc:
        raise ValueError("resource tree could not be read") from exc
    for entry in entries:
        try:
            metadata = entry.lstat()
        except OSError as exc:
            raise ValueError("resource tree changed during hashing") from exc
        if stat.S_ISLNK(metadata.st_mode):
            raise ValueError("symlinks are not allowed")
        if stat.S_ISDIR(metadata.st_mode):
            continue
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError(
                "resource tree must contain only regular files and directories"
            )
        files.append(entry)
    return tuple(files)


def _same_tree(first: ValidationResult, second: ValidationResult) -> bool:
    return (
        first.status is second.status
        and first.size_bytes == second.size_bytes
        and first.sha256 == second.sha256
    )


def resolve_world(
    config: WorldConfig, *, package_root: Path | None = None
) -> ResolvedWorld:
    """Resolve the fixed local Phase 3 fixture and its stable resource hashes."""
    if config != WorldConfig("phase3_foundation", "iris"):
        raise ValueError("Phase 3 supports only phase3_foundation/iris")

    requested_root = (
        Path(package_root)
        if package_root is not None
        else Path(__file__).resolve().parents[3] / "resources"
    )
    try:
        requested_metadata = requested_root.lstat()
    except FileNotFoundError:
        raise
    except OSError as exc:
        raise ValueError("Gazebo resource root could not be read") from exc
    if stat.S_ISLNK(requested_metadata.st_mode):
        raise ValueError("Gazebo resource root must not be a symlink")
    if not stat.S_ISDIR(requested_metadata.st_mode):
        raise ValueError("Gazebo resource root is not a directory")

    root = requested_root.resolve(strict=True)
    world_path = (root / "worlds/phase3_foundation.sdf").resolve(strict=True)
    if not world_path.is_relative_to(root):
        raise ValueError("world path escapes the Gazebo resource root")

    before = _validated_tree(root)
    resource_sha256s = tuple(
        (
            item.relative_to(root).as_posix(),
            _sha256_regular(item, root=root),
        )
        for item in _resource_files(root)
    )
    world_sha256 = _sha256_regular(world_path, root=root)
    after = _validated_tree(root)
    if not _same_tree(before, after):
        raise ValueError("resource tree changed during hashing")

    models = (root / "models").resolve(strict=True)
    if not models.is_relative_to(root) or not models.is_dir():
        raise ValueError("model path escapes the Gazebo resource root")
    return ResolvedWorld(
        path=world_path,
        world_name="phase3_foundation",
        vehicle_id="iris",
        resource_path=models,
        world_sha256=world_sha256,
        resource_sha256s=resource_sha256s,
    )
