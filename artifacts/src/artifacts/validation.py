"""Fail-closed filesystem validation for run-bundle artifacts."""

from dataclasses import dataclass
from enum import Enum
import hashlib
import os
from pathlib import Path
import stat


_CHUNK_SIZE = 1024 * 1024


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


def _candidate(
    run_directory: Path | str, relative_path: Path | str
) -> tuple[Path, str | None]:
    root = Path(run_directory).absolute()
    relative = Path(relative_path)
    if relative.is_absolute() or ".." in relative.parts:
        return root, "path escapes run directory"
    candidate = (root / relative).absolute()
    try:
        candidate.relative_to(root)
    except ValueError:
        return candidate, "path escapes run directory"
    return candidate, None


def _symlink_in_path(root: Path, candidate: Path) -> bool:
    current = root
    for part in candidate.relative_to(root).parts:
        current = current / part
        try:
            if stat.S_ISLNK(current.lstat().st_mode):
                return True
        except FileNotFoundError:
            return False
        except OSError:
            return False
    return False


def _hash_regular_file(path: Path) -> ValidationResult:
    try:
        path_stat = path.lstat()
    except FileNotFoundError:
        return ValidationResult(ValidationStatus.MISSING, None, None, "path is missing")
    except PermissionError:
        return ValidationResult(ValidationStatus.INVALID, None, None, "file could not be read")
    except OSError:
        return ValidationResult(ValidationStatus.INVALID, None, None, "file could not be read")
    if stat.S_ISLNK(path_stat.st_mode):
        return ValidationResult(
            ValidationStatus.INVALID, None, None, "symlinks are not allowed"
        )
    if not stat.S_ISREG(path_stat.st_mode):
        return ValidationResult(
            ValidationStatus.INVALID, None, None, "path is not a regular file"
        )

    flags = os.O_RDONLY
    flags |= getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except FileNotFoundError:
        return ValidationResult(ValidationStatus.MISSING, None, None, "path is missing")
    except PermissionError:
        return ValidationResult(ValidationStatus.INVALID, None, None, "file could not be read")
    except OSError:
        return ValidationResult(ValidationStatus.INVALID, None, None, "file could not be read")

    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            return ValidationResult(
                ValidationStatus.INVALID, None, None, "path is not a regular file"
            )
        if before.st_nlink != 1:
            return ValidationResult(
                ValidationStatus.INVALID,
                None,
                None,
                "regular files must have exactly one hard link",
            )
        digest = hashlib.sha256()
        try:
            while chunk := os.read(descriptor, _CHUNK_SIZE):
                digest.update(chunk)
        except OSError:
            return ValidationResult(
                ValidationStatus.INVALID, None, None, "file could not be read"
            )
        after = os.fstat(descriptor)
        identity = (
            "st_dev",
            "st_ino",
            "st_mode",
            "st_nlink",
            "st_size",
            "st_mtime_ns",
            "st_ctime_ns",
        )
        if any(getattr(before, field) != getattr(after, field) for field in identity):
            return ValidationResult(
                ValidationStatus.INVALID, None, None, "file changed during validation"
            )
        return ValidationResult(
            ValidationStatus.VALID,
            after.st_size,
            digest.hexdigest(),
            "valid regular file",
        )
    finally:
        os.close(descriptor)


def validate_regular_file(
    run_directory: Path | str, relative_path: Path | str
) -> ValidationResult:
    """Validate and hash one required regular file without following symlinks."""
    root = Path(run_directory).absolute()
    candidate, invalid_detail = _candidate(root, relative_path)
    if invalid_detail is not None:
        return ValidationResult(ValidationStatus.INVALID, None, None, invalid_detail)
    if _symlink_in_path(root, candidate):
        return ValidationResult(
            ValidationStatus.INVALID, None, None, "symlinks are not allowed"
        )
    return _hash_regular_file(candidate)


def validate_tree(run_directory: Path | str, relative_path: Path | str) -> ValidationResult:
    """Validate a nonempty directory tree and hash its sorted regular-file inventory."""
    root = Path(run_directory).absolute()
    tree, invalid_detail = _candidate(root, relative_path)
    if invalid_detail is not None:
        return ValidationResult(ValidationStatus.INVALID, None, None, invalid_detail)
    if _symlink_in_path(root, tree):
        return ValidationResult(
            ValidationStatus.INVALID, None, None, "symlinks are not allowed"
        )
    try:
        tree_stat = tree.lstat()
    except FileNotFoundError:
        return ValidationResult(ValidationStatus.MISSING, None, None, "path is missing")
    except OSError:
        return ValidationResult(
            ValidationStatus.INVALID, None, None, "directory tree could not be read"
        )
    if not stat.S_ISDIR(tree_stat.st_mode):
        return ValidationResult(
            ValidationStatus.INVALID, None, None, "path is not a directory"
        )

    rows: list[tuple[str, int, str]] = []
    pending = [tree]
    while pending:
        directory = pending.pop()
        try:
            with os.scandir(directory) as iterator:
                entries = sorted(iterator, key=lambda entry: entry.name)
        except OSError:
            return ValidationResult(
                ValidationStatus.INVALID, None, None, "directory tree could not be read"
            )
        for entry in entries:
            entry_path = Path(entry.path)
            try:
                entry_stat = entry.stat(follow_symlinks=False)
            except OSError:
                return ValidationResult(
                    ValidationStatus.INVALID, None, None, "directory tree could not be read"
                )
            if stat.S_ISLNK(entry_stat.st_mode):
                return ValidationResult(
                    ValidationStatus.INVALID, None, None, "symlinks are not allowed"
                )
            if stat.S_ISDIR(entry_stat.st_mode):
                pending.append(entry_path)
                continue
            if not stat.S_ISREG(entry_stat.st_mode):
                return ValidationResult(
                    ValidationStatus.INVALID,
                    None,
                    None,
                    "directory tree contains a non-regular file",
                )
            result = _hash_regular_file(entry_path)
            if result.status is not ValidationStatus.VALID:
                detail = {
                    "regular files must have exactly one hard link": (
                        "directory tree contains file with multiple hard links"
                    ),
                    "file changed during validation": "directory tree changed during validation",
                }.get(result.detail, "directory tree contains unreadable file")
                return ValidationResult(ValidationStatus.INVALID, None, None, detail)
            rows.append(
                (
                    entry_path.relative_to(tree).as_posix(),
                    result.size_bytes or 0,
                    result.sha256 or "",
                )
            )

    if not rows:
        return ValidationResult(
            ValidationStatus.INVALID, None, None, "directory tree is empty"
        )
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
