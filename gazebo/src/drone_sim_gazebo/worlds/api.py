"""Descriptor-validated resolution of the immutable Phase 3 world."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import stat


_CHUNK_SIZE = 1024 * 1024
_DIRECTORY_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_DIRECTORY", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_FILE_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
    | getattr(os, "O_NONBLOCK", 0)
)
_IDENTITY_FIELDS = (
    "st_dev",
    "st_ino",
    "st_mode",
    "st_nlink",
    "st_size",
    "st_mtime_ns",
    "st_ctime_ns",
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


@dataclass(frozen=True)
class _RetainedDirectory:
    relative_path: str
    parent_fd: int | None
    name: str | None
    descriptor: int
    identity: os.stat_result
    inventory: tuple[str, ...]


@dataclass(frozen=True)
class _RetainedFile:
    relative_path: str
    parent_fd: int
    name: str
    descriptor: int
    identity: os.stat_result


@dataclass
class _ResourceSnapshot:
    root: Path
    directories: tuple[_RetainedDirectory, ...]
    files: tuple[_RetainedFile, ...]
    descriptors: tuple[int, ...]
    closed: bool = False

    def verify(self) -> None:
        if self.closed:
            raise ValueError("resource snapshot is already closed")
        if not all(
            _retained_directory_matches(directory, root=self.root)
            for directory in self.directories
        ):
            raise ValueError("resource tree changed during hashing")
        if not all(_retained_file_matches(item) for item in self.files):
            raise ValueError("resource tree changed during hashing")

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        _close_descriptors(self.descriptors)


def _same_snapshot(first: os.stat_result, second: os.stat_result) -> bool:
    return all(
        getattr(first, field) == getattr(second, field)
        for field in _IDENTITY_FIELDS
    )


def _close_descriptors(descriptors: tuple[int, ...] | list[int]) -> None:
    first_error: OSError | None = None
    for descriptor in reversed(descriptors):
        try:
            os.close(descriptor)
        except OSError as exc:
            if first_error is None:
                first_error = exc
    if first_error is not None:
        raise ValueError("resource descriptors could not be closed") from first_error


def _close_unretained(descriptor: int) -> None:
    try:
        os.close(descriptor)
    except OSError as exc:
        raise ValueError("resource descriptors could not be closed") from exc


def _named_stat(parent_fd: int, name: str) -> os.stat_result:
    try:
        return os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    except OSError as exc:
        raise ValueError("resource tree changed during hashing") from exc


def _retained_file_matches(item: _RetainedFile) -> bool:
    try:
        named = os.stat(item.name, dir_fd=item.parent_fd, follow_symlinks=False)
        opened = os.fstat(item.descriptor)
    except OSError:
        return False
    return _same_snapshot(item.identity, named) and _same_snapshot(
        item.identity, opened
    )


def _retained_directory_matches(
    directory: _RetainedDirectory, *, root: Path
) -> bool:
    try:
        named = (
            root.lstat()
            if directory.parent_fd is None
            else os.stat(
                directory.name,
                dir_fd=directory.parent_fd,
                follow_symlinks=False,
            )
        )
        opened = os.fstat(directory.descriptor)
        inventory = tuple(sorted(os.listdir(directory.descriptor)))
    except OSError:
        return False
    return (
        _same_snapshot(directory.identity, named)
        and _same_snapshot(directory.identity, opened)
        and directory.inventory == inventory
    )


def _open_directory_at(parent_fd: int, name: str) -> tuple[int, os.stat_result]:
    named = _named_stat(parent_fd, name)
    if stat.S_ISLNK(named.st_mode):
        raise ValueError("symlinks are not allowed")
    if not stat.S_ISDIR(named.st_mode):
        raise ValueError(
            "resource tree must contain only regular files and directories"
        )
    try:
        descriptor = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
    except OSError as exc:
        raise ValueError("resource tree changed during hashing") from exc
    try:
        opened = os.fstat(descriptor)
    except OSError as exc:
        _close_unretained(descriptor)
        raise ValueError("resource tree changed during hashing") from exc
    if not _same_snapshot(named, opened):
        error = ValueError("resource tree changed during hashing")
        _close_unretained(descriptor)
        raise error
    return descriptor, opened


def _open_file_at(parent_fd: int, name: str) -> tuple[int, os.stat_result]:
    named = _named_stat(parent_fd, name)
    if stat.S_ISLNK(named.st_mode):
        raise ValueError("symlinks are not allowed")
    if not stat.S_ISREG(named.st_mode):
        raise ValueError(
            "resource tree must contain only regular files and directories"
        )
    if named.st_nlink != 1:
        raise ValueError("regular files must have exactly one hard link")
    try:
        descriptor = os.open(name, _FILE_FLAGS, dir_fd=parent_fd)
    except OSError as exc:
        raise ValueError("resource tree changed during hashing") from exc
    try:
        opened = os.fstat(descriptor)
    except OSError as exc:
        _close_unretained(descriptor)
        raise ValueError("resource tree changed during hashing") from exc
    if not _same_snapshot(named, opened):
        error = ValueError("resource tree changed during hashing")
        _close_unretained(descriptor)
        raise error
    if not stat.S_ISREG(opened.st_mode):
        error = ValueError(
            "resource tree must contain only regular files and directories"
        )
        _close_unretained(descriptor)
        raise error
    if opened.st_nlink != 1:
        error = ValueError("regular files must have exactly one hard link")
        _close_unretained(descriptor)
        raise error
    return descriptor, opened


def _open_resource_snapshot(root: Path) -> _ResourceSnapshot:
    descriptors: list[int] = []
    directories: list[_RetainedDirectory] = []
    files: list[_RetainedFile] = []
    try:
        named_root = root.lstat()
        if stat.S_ISLNK(named_root.st_mode):
            raise ValueError("symlinks are not allowed")
        if not stat.S_ISDIR(named_root.st_mode):
            raise ValueError("Gazebo resource root is not a directory")
        root_fd = os.open(root, _DIRECTORY_FLAGS)
        descriptors.append(root_fd)
        opened_root = os.fstat(root_fd)
        if not _same_snapshot(named_root, opened_root):
            raise ValueError("resource tree changed during hashing")

        pending = [(root_fd, (), None, None, opened_root)]
        while pending:
            descriptor, prefix, parent_fd, name, identity = pending.pop()
            try:
                inventory = tuple(sorted(os.listdir(descriptor)))
            except OSError as exc:
                raise ValueError("resource tree could not be read") from exc
            relative_directory = "/".join(prefix)
            directories.append(
                _RetainedDirectory(
                    relative_directory,
                    parent_fd,
                    name,
                    descriptor,
                    identity,
                    inventory,
                )
            )
            for child_name in inventory:
                relative_parts = (*prefix, child_name)
                relative_path = "/".join(relative_parts)
                try:
                    relative_path.encode("utf-8")
                except UnicodeEncodeError as exc:
                    raise ValueError(
                        "resource tree contains a non-UTF-8 path"
                    ) from exc
                child = _named_stat(descriptor, child_name)
                if stat.S_ISLNK(child.st_mode):
                    raise ValueError("symlinks are not allowed")
                if stat.S_ISDIR(child.st_mode):
                    child_fd, child_identity = _open_directory_at(
                        descriptor, child_name
                    )
                    descriptors.append(child_fd)
                    pending.append(
                        (
                            child_fd,
                            relative_parts,
                            descriptor,
                            child_name,
                            child_identity,
                        )
                    )
                    continue
                if not stat.S_ISREG(child.st_mode):
                    raise ValueError(
                        "resource tree must contain only regular files and directories"
                    )
                file_fd, file_identity = _open_file_at(descriptor, child_name)
                descriptors.append(file_fd)
                files.append(
                    _RetainedFile(
                        relative_path,
                        descriptor,
                        child_name,
                        file_fd,
                        file_identity,
                    )
                )
        if not files:
            raise ValueError("resource tree is empty")
        return _ResourceSnapshot(
            root,
            tuple(directories),
            tuple(sorted(files, key=lambda item: item.relative_path)),
            tuple(descriptors),
        )
    except BaseException as error:
        try:
            _close_descriptors(descriptors)
        except ValueError as cleanup_error:
            raise cleanup_error from error
        raise


def _relative_resource_path(path: Path, *, root: Path) -> Path:
    try:
        relative = path.relative_to(root)
    except ValueError as exc:
        raise ValueError("resource path escapes the Gazebo resource root") from exc
    if not relative.parts or ".." in relative.parts:
        raise ValueError("resource path escapes the Gazebo resource root")
    return relative


def _sha256_regular(
    path: Path, *, root: Path, retained: _RetainedFile
) -> str:
    """Hash one resource through its snapshot-retained descriptor."""
    relative = _relative_resource_path(path, root=root)
    if relative.as_posix() != retained.relative_path:
        raise ValueError("resource path changed during hashing")
    if not _retained_file_matches(retained):
        raise ValueError("resource tree changed during hashing")
    try:
        os.lseek(retained.descriptor, 0, os.SEEK_SET)
        before = os.fstat(retained.descriptor)
    except OSError as exc:
        raise ValueError("resource file could not be read") from exc
    if not _same_snapshot(retained.identity, before):
        raise ValueError("resource changed during hashing")
    digest = hashlib.sha256()
    while True:
        try:
            chunk = os.read(retained.descriptor, _CHUNK_SIZE)
        except OSError as exc:
            raise ValueError("resource file could not be read") from exc
        if not chunk:
            break
        digest.update(chunk)
    after = os.fstat(retained.descriptor)
    if not _same_snapshot(retained.identity, after):
        raise ValueError("resource changed during hashing")
    if not _retained_file_matches(retained):
        raise ValueError("resource tree changed during hashing")
    return digest.hexdigest()


def resolve_world(
    config: WorldConfig, *, package_root: Path | None = None
) -> ResolvedWorld:
    """Resolve an approved local fixture and its stable resource hashes."""
    supported_worlds = {
        WorldConfig("phase3_foundation", "iris"): (
            "phase3_foundation",
            "iris",
        ),
        WorldConfig("vertical_descent", "iris_flight"): (
            "vertical_descent",
            "iris_flight",
        ),
    }
    try:
        world_name, vehicle_id = supported_worlds[config]
    except KeyError:
        raise ValueError(
            "Phase 3 supports only phase3_foundation/iris or "
            "vertical_descent/iris_flight"
        )

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
    world_relative_path = f"worlds/{world_name}.sdf"
    world_path = (root / world_relative_path).resolve(strict=True)
    if not world_path.is_relative_to(root):
        raise ValueError("world path escapes the Gazebo resource root")

    snapshot = _open_resource_snapshot(root)
    try:
        resource_sha256s = tuple(
            (
                item.relative_path,
                _sha256_regular(
                    root / item.relative_path,
                    root=root,
                    retained=item,
                ),
            )
            for item in snapshot.files
        )
        resource_hashes = dict(resource_sha256s)
        try:
            world_sha256 = resource_hashes[world_relative_path]
        except KeyError as exc:
            raise ValueError("Phase 3 world is missing from resource snapshot") from exc
        if "models" not in {
            directory.relative_path for directory in snapshot.directories
        }:
            raise ValueError("Phase 3 model resource directory is missing")
        snapshot.verify()
        return ResolvedWorld(
            path=world_path,
            world_name=world_name,
            vehicle_id=vehicle_id,
            resource_path=root / "models",
            world_sha256=world_sha256,
            resource_sha256s=resource_sha256s,
        )
    finally:
        snapshot.close()
