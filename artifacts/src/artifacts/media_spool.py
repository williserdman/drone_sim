"""Bounded publication from a native-Linux media spool into a run bundle."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import hashlib
import math
import os
from pathlib import Path
import secrets
import stat
import time
from uuid import UUID


_DIRECTORY_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_DIRECTORY", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_READ_FLAGS = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
_WRITE_FLAGS = (
    os.O_WRONLY
    | os.O_CREAT
    | os.O_EXCL
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_COPY_CHUNK_BYTES = 1024 * 1024
_STREAMS = ("onboard", "observer")


@dataclass(frozen=True)
class MediaSpoolFailure:
    relative_path: str
    detail: str


@dataclass(frozen=True)
class MediaSpoolPublication:
    published: tuple[str, ...]
    failures: tuple[MediaSpoolFailure, ...]


def _canonical_run_id(value: str) -> str:
    try:
        parsed = UUID(value)
    except (ValueError, TypeError, AttributeError) as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise ValueError("run_id must be a canonical UUID")
    return value


def _directory_identity(first: os.stat_result, second: os.stat_result) -> bool:
    return (
        stat.S_ISDIR(first.st_mode)
        and first.st_dev == second.st_dev
        and first.st_ino == second.st_ino
        and first.st_mode == second.st_mode
    )


def _snapshot_equal(first: os.stat_result, second: os.stat_result) -> bool:
    fields = (
        "st_dev",
        "st_ino",
        "st_mode",
        "st_nlink",
        "st_size",
        "st_mtime_ns",
        "st_ctime_ns",
    )
    return all(getattr(first, field) == getattr(second, field) for field in fields)


def _open_directory(path: Path) -> tuple[int, os.stat_result]:
    before = os.stat(path, follow_symlinks=False)
    descriptor = os.open(path, _DIRECTORY_FLAGS)
    opened = os.fstat(descriptor)
    if not _directory_identity(before, opened):
        os.close(descriptor)
        raise RuntimeError(f"unsafe media spool directory: {path}")
    return descriptor, opened


def _open_child_directory(parent_fd: int, name: str, *, create: bool) -> int:
    if create:
        try:
            os.mkdir(name, 0o755, dir_fd=parent_fd)
        except FileExistsError:
            pass
    before = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    descriptor = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
    opened = os.fstat(descriptor)
    if not _directory_identity(before, opened):
        os.close(descriptor)
        raise RuntimeError(f"unsafe media spool directory component: {name}")
    return descriptor


def _require_time(deadline: float, monotonic: Callable[[], float]) -> None:
    if monotonic() >= deadline:
        raise TimeoutError("media spool publication exceeded finalization deadline")


def _hash_descriptor(
    descriptor: int,
    *,
    deadline: float,
    monotonic: Callable[[], float],
) -> str:
    digest = hashlib.sha256()
    offset = 0
    while True:
        _require_time(deadline, monotonic)
        chunk = os.pread(descriptor, _COPY_CHUNK_BYTES, offset)
        if not chunk:
            return digest.hexdigest()
        digest.update(chunk)
        offset += len(chunk)


class MediaSpool:
    """Own one fresh native spool directory and publish its closed outputs."""

    def __init__(
        self,
        spool_root: Path,
        authoritative_directory: Path,
        run_id: str,
    ) -> None:
        self.spool_root = spool_root
        self.authoritative_directory = authoritative_directory
        self.run_id = run_id
        self.recording_directory = spool_root / run_id

    @classmethod
    def create(
        cls,
        spool_root: Path | str,
        authoritative_directory: Path | str,
        run_id: str,
    ) -> "MediaSpool":
        root = Path(spool_root)
        authority = Path(authoritative_directory)
        canonical = _canonical_run_id(run_id)
        root_fd, _identity = _open_directory(root)
        try:
            try:
                os.mkdir(canonical, 0o700, dir_fd=root_fd)
            except FileExistsError as error:
                raise FileExistsError(
                    f"refusing existing run-scoped media spool: {canonical}"
                ) from error
            run_fd = _open_child_directory(root_fd, canonical, create=False)
            os.close(run_fd)
        finally:
            os.close(root_fd)
        authority_fd, _authority_identity = _open_directory(authority)
        os.close(authority_fd)
        return cls(root.resolve(), authority.resolve(), canonical)

    def publish(
        self,
        *,
        deadline: float,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> MediaSpoolPublication:
        if (
            isinstance(deadline, bool)
            or not isinstance(deadline, (int, float))
            or not math.isfinite(deadline)
        ):
            raise ValueError("media spool deadline must be finite")
        candidates: list[str] = []
        failures: list[MediaSpoolFailure] = []
        for stream in _STREAMS:
            log = f"logs/docker/ffmpeg-{stream}.log.partial"
            if self._source_entry_exists(log):
                candidates.append(log)
            final = f"video/{stream}.mp4"
            partial = f"video/{stream}.mp4.partial"
            final_exists = self._source_entry_exists(final)
            partial_exists = self._source_entry_exists(partial)
            if final_exists and partial_exists:
                failures.append(
                    MediaSpoolFailure(
                        final,
                        "media spool contains conflicting final and partial outputs",
                    )
                )
            elif final_exists:
                candidates.append(final)
            elif partial_exists:
                candidates.append(partial)

        published: list[str] = []
        for relative in sorted(candidates):
            try:
                self._copy_one(relative, deadline=deadline, monotonic=monotonic)
            except Exception as error:
                failures.append(
                    MediaSpoolFailure(
                        relative,
                        f"{type(error).__name__}: {error}",
                    )
                )
            else:
                published.append(relative)
        return MediaSpoolPublication(tuple(published), tuple(failures))

    def _source_entry_exists(self, relative: str) -> bool:
        try:
            os.stat(
                self.recording_directory / relative,
                follow_symlinks=False,
            )
        except FileNotFoundError:
            return False
        return True

    @staticmethod
    def _open_relative_parent(root_fd: int, relative: str, *, create: bool) -> tuple[int, str]:
        parts = relative.split("/")
        if not parts or any(part in {"", ".", ".."} for part in parts):
            raise ValueError("media spool path is invalid")
        descriptor = os.dup(root_fd)
        try:
            for component in parts[:-1]:
                child = _open_child_directory(descriptor, component, create=create)
                os.close(descriptor)
                descriptor = child
            return descriptor, parts[-1]
        except BaseException:
            os.close(descriptor)
            raise

    def _copy_one(
        self,
        relative: str,
        *,
        deadline: float,
        monotonic: Callable[[], float],
    ) -> None:
        _require_time(deadline, monotonic)
        source_root_fd, _source_identity = _open_directory(self.recording_directory)
        destination_root_fd, _destination_identity = _open_directory(
            self.authoritative_directory
        )
        source_parent_fd: int | None = None
        destination_parent_fd: int | None = None
        source_fd: int | None = None
        temporary_fd: int | None = None
        verify_fd: int | None = None
        published_fd: int | None = None
        temporary_name: str | None = None
        temporary_identity: os.stat_result | None = None
        destination_linked = False
        publication_confirmed = False
        try:
            source_parent_fd, source_name = self._open_relative_parent(
                source_root_fd, relative, create=False
            )
            destination_parent_fd, destination_name = self._open_relative_parent(
                destination_root_fd, relative, create=True
            )
            try:
                os.stat(
                    destination_name,
                    dir_fd=destination_parent_fd,
                    follow_symlinks=False,
                )
            except FileNotFoundError:
                pass
            else:
                raise FileExistsError(
                    f"authoritative media output already exists: {relative}"
                )
            source_fd = os.open(source_name, _READ_FLAGS, dir_fd=source_parent_fd)
            source_before = os.fstat(source_fd)
            if (
                not stat.S_ISREG(source_before.st_mode)
                or source_before.st_nlink != 1
                or source_before.st_mode & 0o222
            ):
                raise RuntimeError("media spool source is not a closed read-only file")

            temporary_name = (
                f".media-spool-{self.run_id}-{secrets.token_hex(8)}.tmp"
            )
            temporary_fd = os.open(
                temporary_name,
                _WRITE_FLAGS,
                0o600,
                dir_fd=destination_parent_fd,
            )
            source_digest = hashlib.sha256()
            offset = 0
            while True:
                _require_time(deadline, monotonic)
                chunk = os.pread(source_fd, _COPY_CHUNK_BYTES, offset)
                if not chunk:
                    break
                source_digest.update(chunk)
                written = 0
                while written < len(chunk):
                    _require_time(deadline, monotonic)
                    count = os.write(temporary_fd, chunk[written:])
                    if count <= 0:
                        raise OSError("media spool copy made no progress")
                    written += count
                offset += len(chunk)
            _require_time(deadline, monotonic)
            os.fsync(temporary_fd)
            source_after = os.fstat(source_fd)
            if not _snapshot_equal(source_before, source_after):
                raise RuntimeError("media spool source changed during copy")
            if os.fstat(temporary_fd).st_size != source_before.st_size:
                raise RuntimeError("media spool copy size mismatch")
            os.fchmod(temporary_fd, 0o444)
            temporary_identity = os.fstat(temporary_fd)
            os.close(temporary_fd)
            temporary_fd = None

            verify_fd = os.open(
                temporary_name,
                _READ_FLAGS,
                dir_fd=destination_parent_fd,
            )
            destination_digest = _hash_descriptor(
                verify_fd,
                deadline=deadline,
                monotonic=monotonic,
            )
            if destination_digest != source_digest.hexdigest():
                raise RuntimeError("media spool copy hash mismatch")
            os.close(verify_fd)
            verify_fd = None
            _require_time(deadline, monotonic)
            os.link(
                temporary_name,
                destination_name,
                src_dir_fd=destination_parent_fd,
                dst_dir_fd=destination_parent_fd,
                follow_symlinks=False,
            )
            destination_linked = True
            os.fsync(destination_parent_fd)
            published_fd = os.open(
                destination_name,
                _READ_FLAGS,
                dir_fd=destination_parent_fd,
            )
            if _hash_descriptor(
                published_fd,
                deadline=deadline,
                monotonic=monotonic,
            ) != source_digest.hexdigest():
                raise RuntimeError("published media hash mismatch")
            os.unlink(temporary_name, dir_fd=destination_parent_fd)
            temporary_name = None
            os.fsync(destination_parent_fd)
            publication_confirmed = True
        finally:
            if (
                destination_linked
                and not publication_confirmed
                and destination_parent_fd is not None
                and temporary_identity is not None
            ):
                try:
                    published_identity = os.stat(
                        destination_name,
                        dir_fd=destination_parent_fd,
                        follow_symlinks=False,
                    )
                    if (
                        published_identity.st_dev == temporary_identity.st_dev
                        and published_identity.st_ino == temporary_identity.st_ino
                    ):
                        os.unlink(destination_name, dir_fd=destination_parent_fd)
                        os.fsync(destination_parent_fd)
                except (FileNotFoundError, OSError):
                    pass
            if temporary_name is not None and destination_parent_fd is not None:
                try:
                    os.unlink(temporary_name, dir_fd=destination_parent_fd)
                except FileNotFoundError:
                    pass
            for descriptor in (
                published_fd,
                verify_fd,
                temporary_fd,
                source_fd,
                destination_parent_fd,
                source_parent_fd,
                destination_root_fd,
                source_root_fd,
            ):
                if descriptor is not None:
                    try:
                        os.close(descriptor)
                    except OSError:
                        pass


__all__ = [
    "MediaSpool",
    "MediaSpoolFailure",
    "MediaSpoolPublication",
]
