"""Fail-closed filesystem validation for run-bundle artifacts."""

from dataclasses import dataclass
from enum import Enum
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


class ValidationStatus(str, Enum):
    VALID = "valid"
    MISSING = "missing"
    INVALID = "invalid"


@dataclass(frozen=True)
class ValidationResult:
    status: ValidationStatus
    size_bytes: int | None
    sha256: str | None
    detail: str


@dataclass(frozen=True)
class _RetainedEntry:
    parent_fd: int
    name: str
    descriptor: int
    identity: os.stat_result


class _ValidationFailure(Exception):
    def __init__(self, result: ValidationResult) -> None:
        super().__init__(result.detail)
        self.result = result


def _fail(status: ValidationStatus, detail: str) -> None:
    raise _ValidationFailure(ValidationResult(status, None, None, detail))


def _relative_parts(relative_path: Path | str) -> tuple[str, ...]:
    relative = Path(relative_path)
    if relative.is_absolute() or ".." in relative.parts:
        _fail(ValidationStatus.INVALID, "path escapes run directory")
    parts = tuple(part for part in relative.parts if part != ".")
    if not parts:
        _fail(ValidationStatus.INVALID, "path escapes run directory")
    return parts


def _open_run_directory(run_directory: Path | str) -> int:
    try:
        return os.open(run_directory, _DIRECTORY_FLAGS)
    except FileNotFoundError:
        _fail(ValidationStatus.MISSING, "path is missing")
    except PermissionError:
        _fail(ValidationStatus.INVALID, "run directory could not be read")
    except OSError:
        try:
            root_stat = Path(run_directory).lstat()
        except FileNotFoundError:
            _fail(ValidationStatus.MISSING, "path is missing")
        except OSError:
            _fail(ValidationStatus.INVALID, "run directory could not be read")
        if stat.S_ISLNK(root_stat.st_mode):
            _fail(ValidationStatus.INVALID, "symlinks are not allowed")
        _fail(ValidationStatus.INVALID, "run directory could not be read")


def _stat_at(parent_fd: int, name: str, unreadable_detail: str) -> os.stat_result:
    try:
        return os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
    except FileNotFoundError:
        _fail(ValidationStatus.MISSING, "path is missing")
    except OSError:
        _fail(ValidationStatus.INVALID, unreadable_detail)


def _open_directory_at(
    parent_fd: int,
    name: str,
    *,
    not_directory_detail: str,
    unreadable_detail: str,
) -> tuple[int, os.stat_result]:
    entry_stat = _stat_at(parent_fd, name, unreadable_detail)
    if stat.S_ISLNK(entry_stat.st_mode):
        _fail(ValidationStatus.INVALID, "symlinks are not allowed")
    if not stat.S_ISDIR(entry_stat.st_mode):
        _fail(ValidationStatus.INVALID, not_directory_detail)
    try:
        descriptor = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
    except FileNotFoundError:
        _fail(ValidationStatus.MISSING, "path is missing")
    except OSError:
        current_stat = _stat_at(parent_fd, name, unreadable_detail)
        if stat.S_ISLNK(current_stat.st_mode):
            _fail(ValidationStatus.INVALID, "symlinks are not allowed")
        _fail(ValidationStatus.INVALID, unreadable_detail)
    opened_stat = os.fstat(descriptor)
    if not _same_entry(entry_stat, opened_stat):
        os.close(descriptor)
        _fail(ValidationStatus.INVALID, "path changed during validation")
    return descriptor, opened_stat


def _same_entry(first: os.stat_result, second: os.stat_result) -> bool:
    return (
        first.st_dev == second.st_dev
        and first.st_ino == second.st_ino
        and first.st_mode == second.st_mode
    )


def _same_file_snapshot(first: os.stat_result, second: os.stat_result) -> bool:
    return all(getattr(first, field) == getattr(second, field) for field in _IDENTITY_FIELDS)


def _entry_still_matches(entry: _RetainedEntry) -> bool:
    try:
        current = os.stat(
            entry.name,
            dir_fd=entry.parent_fd,
            follow_symlinks=False,
        )
        opened = os.fstat(entry.descriptor)
    except OSError:
        return False
    return _same_entry(current, entry.identity) and _same_entry(opened, entry.identity)


def _open_directory_chain(
    root_fd: int,
    parts: tuple[str, ...],
    held_descriptors: list[int],
    retained_entries: list[_RetainedEntry],
    *,
    final_not_directory_detail: str,
    unreadable_detail: str,
) -> int:
    parent_fd = root_fd
    for index, name in enumerate(parts):
        detail = (
            final_not_directory_detail
            if index == len(parts) - 1
            else "path is not a directory"
        )
        descriptor, opened_stat = _open_directory_at(
            parent_fd,
            name,
            not_directory_detail=detail,
            unreadable_detail=unreadable_detail,
        )
        held_descriptors.append(descriptor)
        retained_entries.append(
            _RetainedEntry(parent_fd, name, descriptor, opened_stat)
        )
        parent_fd = descriptor
    return parent_fd


def _hash_regular_file_at(
    parent_fd: int,
    name: str,
    *,
    unreadable_detail: str,
) -> ValidationResult:
    entry_stat = _stat_at(parent_fd, name, unreadable_detail)
    if stat.S_ISLNK(entry_stat.st_mode):
        _fail(ValidationStatus.INVALID, "symlinks are not allowed")
    if not stat.S_ISREG(entry_stat.st_mode):
        _fail(ValidationStatus.INVALID, "path is not a regular file")
    try:
        descriptor = os.open(name, _FILE_FLAGS, dir_fd=parent_fd)
    except FileNotFoundError:
        _fail(ValidationStatus.MISSING, "path is missing")
    except PermissionError:
        _fail(ValidationStatus.INVALID, unreadable_detail)
    except OSError:
        current_stat = _stat_at(parent_fd, name, unreadable_detail)
        if stat.S_ISLNK(current_stat.st_mode):
            _fail(ValidationStatus.INVALID, "symlinks are not allowed")
        _fail(ValidationStatus.INVALID, unreadable_detail)

    try:
        before = os.fstat(descriptor)
        if not _same_entry(entry_stat, before):
            _fail(ValidationStatus.INVALID, "file changed during validation")
        if not stat.S_ISREG(before.st_mode):
            _fail(ValidationStatus.INVALID, "path is not a regular file")
        if before.st_nlink != 1:
            _fail(
                ValidationStatus.INVALID,
                "regular files must have exactly one hard link",
            )
        digest = hashlib.sha256()
        try:
            while chunk := os.read(descriptor, _CHUNK_SIZE):
                digest.update(chunk)
        except OSError:
            _fail(ValidationStatus.INVALID, unreadable_detail)
        after = os.fstat(descriptor)
        if not _same_file_snapshot(before, after):
            _fail(ValidationStatus.INVALID, "file changed during validation")
        retained = _RetainedEntry(parent_fd, name, descriptor, after)
        if not _entry_still_matches(retained):
            _fail(ValidationStatus.INVALID, "path changed during validation")
        return ValidationResult(
            ValidationStatus.VALID,
            after.st_size,
            digest.hexdigest(),
            "valid regular file",
        )
    finally:
        os.close(descriptor)


def _all_entries_still_match(entries: list[_RetainedEntry]) -> bool:
    return all(_entry_still_matches(entry) for entry in entries)


def validate_regular_file(
    run_directory: Path | str, relative_path: Path | str
) -> ValidationResult:
    """Validate a regular file through a retained, no-follow descriptor walk."""
    held_descriptors: list[int] = []
    retained_entries: list[_RetainedEntry] = []
    try:
        parts = _relative_parts(relative_path)
        root_fd = _open_run_directory(run_directory)
        held_descriptors.append(root_fd)
        parent_fd = _open_directory_chain(
            root_fd,
            parts[:-1],
            held_descriptors,
            retained_entries,
            final_not_directory_detail="path is not a directory",
            unreadable_detail="file could not be read",
        )
        result = _hash_regular_file_at(
            parent_fd,
            parts[-1],
            unreadable_detail="file could not be read",
        )
        if not _all_entries_still_match(retained_entries):
            _fail(ValidationStatus.INVALID, "path changed during validation")
        return result
    except _ValidationFailure as failure:
        return failure.result
    finally:
        for descriptor in reversed(held_descriptors):
            os.close(descriptor)


def _tree_file_failure(result: ValidationResult) -> ValidationResult:
    detail = {
        "regular files must have exactly one hard link": (
            "directory tree contains file with multiple hard links"
        ),
        "file changed during validation": "directory tree changed during validation",
        "path changed during validation": "path changed during validation",
    }.get(result.detail, "directory tree contains unreadable file")
    return ValidationResult(ValidationStatus.INVALID, None, None, detail)


def validate_tree(run_directory: Path | str, relative_path: Path | str) -> ValidationResult:
    """Validate a nonempty tree through retained, no-follow directory descriptors."""
    held_descriptors: list[int] = []
    retained_entries: list[_RetainedEntry] = []
    try:
        parts = _relative_parts(relative_path)
        root_fd = _open_run_directory(run_directory)
        held_descriptors.append(root_fd)
        tree_fd = _open_directory_chain(
            root_fd,
            parts,
            held_descriptors,
            retained_entries,
            final_not_directory_detail="path is not a directory",
            unreadable_detail="directory tree could not be read",
        )
        rows: list[tuple[str, int, str]] = []
        pending: list[tuple[int, tuple[str, ...]]] = [(tree_fd, ())]
        while pending:
            directory_fd, prefix = pending.pop()
            try:
                names = sorted(os.listdir(directory_fd))
            except OSError:
                _fail(ValidationStatus.INVALID, "directory tree could not be read")
            for name in names:
                relative_parts = (*prefix, name)
                relative_posix = "/".join(relative_parts)
                try:
                    relative_posix.encode("utf-8")
                except UnicodeEncodeError:
                    _fail(
                        ValidationStatus.INVALID,
                        "directory tree contains a non-UTF-8 path",
                    )
                entry_stat = _stat_at(
                    directory_fd,
                    name,
                    "directory tree could not be read",
                )
                if stat.S_ISLNK(entry_stat.st_mode):
                    _fail(ValidationStatus.INVALID, "symlinks are not allowed")
                if stat.S_ISDIR(entry_stat.st_mode):
                    child_fd, opened_stat = _open_directory_at(
                        directory_fd,
                        name,
                        not_directory_detail="directory tree contains a non-regular file",
                        unreadable_detail="directory tree could not be read",
                    )
                    held_descriptors.append(child_fd)
                    retained_entries.append(
                        _RetainedEntry(directory_fd, name, child_fd, opened_stat)
                    )
                    pending.append((child_fd, relative_parts))
                    continue
                if not stat.S_ISREG(entry_stat.st_mode):
                    _fail(
                        ValidationStatus.INVALID,
                        "directory tree contains a non-regular file",
                    )
                try:
                    result = _hash_regular_file_at(
                        directory_fd,
                        name,
                        unreadable_detail="directory tree contains unreadable file",
                    )
                except _ValidationFailure as failure:
                    raise _ValidationFailure(_tree_file_failure(failure.result)) from failure
                rows.append(
                    (
                        relative_posix,
                        result.size_bytes or 0,
                        result.sha256 or "",
                    )
                )

        if not rows:
            _fail(ValidationStatus.INVALID, "directory tree is empty")
        if not _all_entries_still_match(retained_entries):
            _fail(ValidationStatus.INVALID, "path changed during validation")
        digest = hashlib.sha256()
        size_bytes = 0
        for path, size, file_digest in sorted(rows):
            digest.update(f"{path}\0{size}\0{file_digest}\n".encode("utf-8"))
            size_bytes += size
        return ValidationResult(
            ValidationStatus.VALID,
            size_bytes,
            digest.hexdigest(),
            "valid directory tree",
        )
    except _ValidationFailure as failure:
        return failure.result
    finally:
        for descriptor in reversed(held_descriptors):
            os.close(descriptor)
