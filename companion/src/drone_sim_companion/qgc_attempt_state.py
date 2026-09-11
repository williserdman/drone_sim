"""Resolve the fixed QGC attempt-state binding without mutating it.

The descriptor checks in this module target local filesystems. They validate the
objects opened during the call; callers must not treat the returned paths as a
capability or as protection against replacement after the function returns.
"""

from __future__ import annotations

from dataclasses import dataclass
import errno
import os
from pathlib import Path
import re
import stat
from typing import Iterable


ATTEMPT_STATE_ROOT = Path("/var/lib/drone-sim/comp2026-attempt-state")
_DIGEST_PATTERN = re.compile(r"[0-9a-f]{64}\Z")
_LEDGER_NAME = "attempt-ledger.json"
_LOCK_NAME = "attempt-ledger.json.lock"
_ALLOWED_STATE_ENTRIES = frozenset((_LEDGER_NAME, _LOCK_NAME))


class AttemptStateError(ValueError):
    """The canonical attempt-state binding is absent or unsafe."""


@dataclass(frozen=True, slots=True)
class AttemptStateBinding:
    """Canonical paths for one deployment profile's persistent attempt state."""

    attempt_state_id: str
    state_directory: Path
    ledger_path: Path
    lock_path: Path


def resolve_attempt_state(
    deployment_profile_digest: str,
    attempt_state_id: str,
    *,
    forbidden_paths: Iterable[str | os.PathLike[str]],
    test_only_state_root: str | os.PathLike[str] | None = None,
) -> AttemptStateBinding:
    """Validate and return the one canonical binding for a deployment profile.

    Production callers cannot choose the state root or either filename. Tests
    may supply ``test_only_state_root`` to exercise the same checks in a
    temporary local filesystem.
    """

    if not isinstance(deployment_profile_digest, str) or not _DIGEST_PATTERN.fullmatch(
        deployment_profile_digest
    ):
        raise AttemptStateError(
            "deployment profile digest must be exactly 64 lowercase hex characters"
        )
    expected_state_id = f"sha256-{deployment_profile_digest}"
    if attempt_state_id != expected_state_id:
        raise AttemptStateError(
            "attempt state ID must be sha256- followed by the deployment profile digest"
        )

    state_root = _canonical_absolute_path(
        ATTEMPT_STATE_ROOT
        if test_only_state_root is None
        else test_only_state_root,
        label="attempt state root",
    )
    if state_root == Path("/"):
        raise AttemptStateError("attempt state root cannot be the filesystem root")

    state_directory = state_root / expected_state_id
    ledger_path = state_directory / _LEDGER_NAME
    lock_path = state_directory / _LOCK_NAME
    _reject_forbidden_overlap(
        (state_root, state_directory, ledger_path, lock_path), forbidden_paths
    )

    root_fd = _open_directory_path(state_root, label="attempt state root")
    try:
        state_fd = _open_directory_at(
            root_fd, expected_state_id, label="attempt state directory"
        )
        try:
            entries = frozenset(os.listdir(state_fd))
            unexpected = entries - _ALLOWED_STATE_ENTRIES
            if unexpected:
                raise AttemptStateError(
                    "attempt state directory contains an unexpected filename"
                )
            _validate_regular_file_at(
                state_fd, _LEDGER_NAME, label="attempt ledger", required=True
            )
            _validate_regular_file_at(
                state_fd, _LOCK_NAME, label="attempt ledger lock", required=True
            )
        finally:
            os.close(state_fd)
    finally:
        os.close(root_fd)

    return AttemptStateBinding(
        attempt_state_id=expected_state_id,
        state_directory=state_directory,
        ledger_path=ledger_path,
        lock_path=lock_path,
    )


def _canonical_absolute_path(
    value: str | os.PathLike[str], *, label: str
) -> Path:
    try:
        raw = os.fspath(value)
    except TypeError as error:
        raise AttemptStateError(f"{label} must be a filesystem path") from error
    if not isinstance(raw, str):
        raise AttemptStateError(f"{label} must be a text filesystem path")
    if "\0" in raw:
        raise AttemptStateError(f"{label} must not contain NUL bytes")
    if raw.startswith("//"):
        raise AttemptStateError(f"{label} must use a single-slash filesystem anchor")
    try:
        path = Path(raw)
        canonical = Path(os.path.abspath(raw))
    except (OSError, ValueError) as error:
        raise AttemptStateError(f"{label} is malformed") from error
    if not path.is_absolute():
        raise AttemptStateError(f"{label} must be absolute")
    if path != canonical:
        raise AttemptStateError(f"{label} must not contain dot path components")
    return canonical


def _reject_forbidden_overlap(
    state_paths: tuple[Path, ...],
    forbidden_paths: Iterable[str | os.PathLike[str]],
) -> None:
    try:
        supplied_paths = tuple(forbidden_paths)
    except TypeError as error:
        raise AttemptStateError("forbidden paths must be an iterable") from error

    for supplied in supplied_paths:
        forbidden = _canonical_absolute_path(supplied, label="forbidden path")
        try:
            resolved_forbidden = forbidden.resolve(strict=False)
        except (OSError, RuntimeError, ValueError) as error:
            raise AttemptStateError("forbidden path cannot be resolved safely") from error
        for state_path in state_paths:
            if _overlaps(state_path, forbidden) or _overlaps(
                state_path, resolved_forbidden
            ):
                raise AttemptStateError(
                    "attempt state and forbidden output paths must not overlap"
                )


def _overlaps(first: Path, second: Path) -> bool:
    return first == second or first.is_relative_to(second) or second.is_relative_to(first)


def _directory_flags() -> int:
    return (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )


def _open_directory_path(path: Path, *, label: str) -> int:
    flags = _directory_flags()
    try:
        current_fd = os.open("/", flags)
    except (OSError, ValueError) as error:  # pragma: no cover - fixed path
        raise AttemptStateError("cannot inspect the local filesystem root") from error

    try:
        for component in path.parts[1:]:
            next_fd = os.open(component, flags, dir_fd=current_fd)
            os.close(current_fd)
            current_fd = next_fd
    except (OSError, ValueError) as error:
        os.close(current_fd)
        raise AttemptStateError(f"{label} must be an existing real directory") from error
    except BaseException:
        os.close(current_fd)
        raise
    return current_fd


def _open_directory_at(parent_fd: int, name: str, *, label: str) -> int:
    try:
        return os.open(name, _directory_flags(), dir_fd=parent_fd)
    except (OSError, ValueError) as error:
        raise AttemptStateError(f"{label} must be an existing real directory") from error


def _validate_regular_file_at(
    directory_fd: int, name: str, *, label: str, required: bool
) -> None:
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
        | getattr(os, "O_NONBLOCK", 0)
    )
    try:
        file_fd = os.open(name, flags, dir_fd=directory_fd)
    except (OSError, ValueError) as error:
        if (
            not required
            and isinstance(error, OSError)
            and error.errno == errno.ENOENT
        ):
            return
        expectation = "an existing regular non-symlink file"
        if not required:
            expectation = "absent or a regular non-symlink file"
        raise AttemptStateError(f"{label} must be {expectation}") from error
    try:
        if not stat.S_ISREG(os.fstat(file_fd).st_mode):
            expectation = "an existing regular non-symlink file"
            if not required:
                expectation = "absent or a regular non-symlink file"
            raise AttemptStateError(f"{label} must be {expectation}")
    finally:
        os.close(file_fd)
