"""Descriptor-relative, durable JSON protocol file operations."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from enum import Enum
import fcntl
import json
import os
import stat
from typing import Any
from uuid import uuid4


_FILE_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_TEMPORARY_FILE_FLAGS = (
    os.O_WRONLY
    | os.O_CREAT
    | os.O_EXCL
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_SNAPSHOT_FIELDS = (
    "st_dev",
    "st_ino",
    "st_mode",
    "st_nlink",
    "st_size",
    "st_mtime_ns",
    "st_ctime_ns",
)


class ProtocolIOError(RuntimeError):
    """A protocol file could not be encoded, inspected, read, or persisted."""


class WritePolicy(Enum):
    IDENTICAL = "identical"
    FIRST_WINS = "first_wins"
    REPLACE = "replace"


def _reject_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    document: dict[str, Any] = {}
    for key, value in pairs:
        if key in document:
            raise ValueError(f"duplicate JSON key: {key}")
        document[key] = value
    return document


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-standard JSON constant: {value}")


def canonical_json(document: Mapping[str, Any]) -> bytes:
    """Encode a mapping as strict, deterministic UTF-8 JSON."""
    try:
        return (
            json.dumps(
                dict(document),
                allow_nan=False,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError, OverflowError, RecursionError, UnicodeError) as error:
        raise ProtocolIOError(f"protocol document is not valid JSON: {error}") from error


def _same_snapshot(first: os.stat_result, second: os.stat_result) -> bool:
    return all(
        getattr(first, field) == getattr(second, field)
        for field in _SNAPSHOT_FIELDS
    )


def _inspect_existing(directory_fd: int, name: str) -> os.stat_result | None:
    try:
        metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
    except FileNotFoundError:
        return None
    except OSError as error:
        raise ProtocolIOError(
            f"could not inspect protocol file {name!r}: {error}"
        ) from error
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        raise ProtocolIOError(
            f"protocol file {name!r} must be regular with exactly one hard link"
        )
    return metadata


def _check_deadline(deadline_check: Callable[[], None] | None) -> None:
    if deadline_check is not None:
        deadline_check()


def read_json_object_at(
    directory_fd: int,
    name: str,
    *,
    deadline_check: Callable[[], None] | None = None,
    max_bytes: int = 4 * 1024 * 1024,
) -> dict[str, Any] | None:
    """Read one stable, single-link JSON object relative to a directory fd."""
    _check_deadline(deadline_check)
    before = _inspect_existing(directory_fd, name)
    if before is None:
        return None

    descriptor: int | None = None
    try:
        descriptor = os.open(name, _FILE_FLAGS, dir_fd=directory_fd)
        opened = os.fstat(descriptor)
        if not _same_snapshot(before, opened):
            raise ProtocolIOError(f"protocol file {name!r} changed while opening")
        if opened.st_size > max_bytes:
            raise ProtocolIOError(f"protocol file {name!r} is too large")

        chunks: list[bytes] = []
        remaining = max_bytes + 1
        while remaining > 0:
            _check_deadline(deadline_check)
            chunk = os.read(descriptor, min(65536, remaining))
            _check_deadline(deadline_check)
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        payload = b"".join(chunks)
        if len(payload) > max_bytes:
            raise ProtocolIOError(f"protocol file {name!r} is too large")

        after = os.fstat(descriptor)
        named_after = _inspect_existing(directory_fd, name)
        if (
            not _same_snapshot(opened, after)
            or named_after is None
            or not _same_snapshot(after, named_after)
        ):
            raise ProtocolIOError(f"protocol file {name!r} changed while reading")

        try:
            _check_deadline(deadline_check)
            document = json.loads(
                payload.decode("utf-8"),
                object_pairs_hook=_reject_duplicate_pairs,
                parse_constant=_reject_json_constant,
            )
            _check_deadline(deadline_check)
        except TimeoutError:
            raise
        except (
            UnicodeError,
            json.JSONDecodeError,
            ValueError,
            RecursionError,
            TypeError,
            OverflowError,
        ) as error:
            raise ProtocolIOError(
                f"protocol file {name!r} contains invalid JSON: {error}"
            ) from error
        if not isinstance(document, dict):
            raise ProtocolIOError(
                f"protocol file {name!r} must contain a JSON object"
            )
        return document
    except ProtocolIOError:
        raise
    except TimeoutError:
        raise
    except OSError as error:
        raise ProtocolIOError(
            f"could not read protocol file {name!r}: {error}"
        ) from error
    finally:
        if descriptor is not None:
            os.close(descriptor)


def write_json_object_at(
    directory_fd: int,
    name: str,
    document: Mapping[str, Any],
    *,
    mode: int,
    policy: WritePolicy,
) -> tuple[dict[str, Any], bool]:
    """Persist one JSON object under an exclusive directory lock."""
    temporary = f".{name}.{uuid4().hex}.tmp"
    descriptor: int | None = None
    locked = False
    temporary_created = False
    try:
        fcntl.flock(directory_fd, fcntl.LOCK_EX)
        locked = True

        existing = read_json_object_at(directory_fd, name)
        candidate = dict(document)
        if existing is not None:
            if policy is WritePolicy.IDENTICAL:
                if existing == candidate:
                    return existing, False
                raise ProtocolIOError(
                    f"protocol file {name!r} conflicts with existing value"
                )
            if policy is WritePolicy.FIRST_WINS:
                return existing, False

        try:
            payload = canonical_json(candidate)
        except ProtocolIOError as error:
            raise ProtocolIOError(
                f"protocol file {name!r} contains a document that is not valid JSON: "
                f"{error}"
            ) from error
        descriptor = os.open(
            temporary,
            _TEMPORARY_FILE_FLAGS,
            mode,
            dir_fd=directory_fd,
        )
        temporary_created = True
        os.fchmod(descriptor, mode)
        written = 0
        while written < len(payload):
            count = os.write(descriptor, payload[written:])
            if count <= 0:
                raise OSError("protocol write made no progress")
            written += count
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.replace(
            temporary,
            name,
            src_dir_fd=directory_fd,
            dst_dir_fd=directory_fd,
        )
        os.fsync(directory_fd)
        return candidate, True
    except ProtocolIOError:
        raise
    except (TypeError, ValueError, OverflowError, RecursionError, UnicodeError) as error:
        raise ProtocolIOError(
            f"protocol file {name!r} is not valid JSON: {error}"
        ) from error
    except OSError as error:
        raise ProtocolIOError(
            f"could not persist protocol file {name!r}: {error}"
        ) from error
    finally:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except OSError:
                pass
        if temporary_created:
            try:
                os.unlink(temporary, dir_fd=directory_fd)
            except OSError:
                pass
        if locked:
            fcntl.flock(directory_fd, fcntl.LOCK_UN)
