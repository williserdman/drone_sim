"""Descriptor-relative, durable JSON protocol file operations."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import ctypes
from enum import Enum
import errno
import fcntl
import json
import math
import os
import stat
from typing import Any
from uuid import uuid4


_FILE_FLAGS = os.O_RDONLY | os.O_CLOEXEC | os.O_NOFOLLOW | os.O_NONBLOCK
_DIRECTORY_FLAGS = (
    os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC | os.O_NOFOLLOW
)
_TEMPORARY_FILE_FLAGS = (
    os.O_RDWR
    | os.O_CREAT
    | os.O_EXCL
    | os.O_CLOEXEC
    | os.O_NOFOLLOW
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
_PUBLICATION_FIELDS = tuple(
    field for field in _SNAPSHOT_FIELDS if field != "st_ctime_ns"
)
_RENAME_NOREPLACE = 1
try:
    _LIBC = ctypes.CDLL(None, use_errno=True)
    _RENAMEAT2 = _LIBC.renameat2
except (AttributeError, OSError):
    _RENAMEAT2 = None
else:
    _RENAMEAT2.argtypes = (
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    )
    _RENAMEAT2.restype = ctypes.c_int


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


def _parse_finite_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"non-finite JSON number: {value}")
    return parsed


def canonical_json(document: Mapping[str, Any]) -> bytes:
    """Encode a mapping as strict, deterministic UTF-8 JSON."""
    try:
        payload = (
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
    _decode_json_object(payload, "protocol document")
    return payload


def _decode_json_object(payload: bytes, description: str) -> dict[str, Any]:
    try:
        document = json.loads(
            payload.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_pairs,
            parse_constant=_reject_json_constant,
            parse_float=_parse_finite_float,
        )
    except (
        UnicodeError,
        json.JSONDecodeError,
        ValueError,
        RecursionError,
        TypeError,
        OverflowError,
    ) as error:
        raise ProtocolIOError(
            f"{description} contains invalid JSON: {error}"
        ) from error
    if not isinstance(document, dict):
        raise ProtocolIOError(f"{description} must contain a JSON object")
    return document


def _same_snapshot(first: os.stat_result, second: os.stat_result) -> bool:
    return all(
        getattr(first, field) == getattr(second, field)
        for field in _SNAPSHOT_FIELDS
    )


def _same_publication_state(
    before_rename: os.stat_result,
    after_rename: os.stat_result,
) -> bool:
    # Renaming an inode updates its ctime; its identity, bytes, and mode must persist.
    return all(
        getattr(before_rename, field) == getattr(after_rename, field)
        for field in _PUBLICATION_FIELDS
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


def _attempt_cleanup(
    failures: list[tuple[str, Exception]],
    action: str,
    operation: Callable[[], None],
) -> None:
    try:
        operation()
    except Exception as error:
        failures.append((action, error))


def _report_cleanup_failures(
    name: str,
    pending_error: BaseException | None,
    failures: list[tuple[str, Exception]],
) -> None:
    if not failures:
        return

    notes = [
        f"protocol file {name!r} cleanup failed during {action}: {error}"
        for action, error in failures
    ]
    if pending_error is not None:
        for note in notes:
            pending_error.add_note(note)
        return

    cleanup_error = ProtocolIOError(notes[0])
    for note in notes[1:]:
        cleanup_error.add_note(note)
    raise cleanup_error from failures[0][1]


def _temporary_path_matches(
    directory_fd: int,
    temporary: str,
    identity: tuple[int, int],
) -> bool:
    try:
        metadata = os.stat(
            temporary,
            dir_fd=directory_fd,
            follow_symlinks=False,
        )
    except FileNotFoundError:
        return False
    except OSError as error:
        raise ProtocolIOError(
            f"could not inspect temporary protocol file {temporary!r}: {error}"
        ) from error
    return (
        stat.S_ISREG(metadata.st_mode)
        and metadata.st_nlink == 1
        and (metadata.st_dev, metadata.st_ino) == identity
    )


def _rename_noreplace_at(
    directory_fd: int,
    source: str,
    target: str,
) -> None:
    """Atomically rename source to target without replacing an existing path."""
    if _RENAMEAT2 is None:
        raise OSError(errno.ENOSYS, "renameat2 is unavailable")
    ctypes.set_errno(0)
    result = _RENAMEAT2(
        directory_fd,
        os.fsencode(source),
        directory_fd,
        os.fsencode(target),
        _RENAME_NOREPLACE,
    )
    if result != 0:
        error_number = ctypes.get_errno() or errno.EIO
        if error_number == errno.EEXIST:
            raise FileExistsError(
                error_number,
                os.strerror(error_number),
                target,
            )
        raise OSError(error_number, os.strerror(error_number), target)


def _named_path_matches_snapshot(
    directory_fd: int,
    name: str,
    snapshot: os.stat_result,
) -> bool:
    try:
        metadata = os.stat(
            name,
            dir_fd=directory_fd,
            follow_symlinks=False,
        )
    except FileNotFoundError:
        return False
    except OSError as error:
        raise ProtocolIOError(
            f"could not inspect protocol file {name!r}: {error}"
        ) from error
    return (
        stat.S_ISREG(metadata.st_mode)
        and metadata.st_nlink == 1
        and _same_snapshot(snapshot, metadata)
    )


def _read_json_object_and_metadata_at(
    directory_fd: int,
    name: str,
    *,
    deadline_check: Callable[[], None] | None = None,
    max_bytes: int = 4 * 1024 * 1024,
) -> tuple[dict[str, Any], os.stat_result] | None:
    _check_deadline(deadline_check)
    before = _inspect_existing(directory_fd, name)
    if before is None:
        return None

    descriptor: int | None = None
    pending_error: BaseException | None = None
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

        _check_deadline(deadline_check)
        document = _decode_json_object(payload, f"protocol file {name!r}")
        _check_deadline(deadline_check)
        decoded = os.fstat(descriptor)
        _check_deadline(deadline_check)
        decoded_payload = os.pread(descriptor, len(payload) + 1, 0)
        _check_deadline(deadline_check)
        verified = os.fstat(descriptor)
        named_decoded = _inspect_existing(directory_fd, name)
        if (
            not _same_snapshot(after, decoded)
            or decoded_payload != payload
            or not _same_snapshot(decoded, verified)
            or named_decoded is None
            or not _same_snapshot(verified, named_decoded)
        ):
            raise ProtocolIOError(f"protocol file {name!r} changed while reading")
        return document, verified
    except ProtocolIOError as error:
        pending_error = error
        raise
    except TimeoutError as error:
        pending_error = error
        raise
    except OSError as error:
        wrapped = ProtocolIOError(
            f"could not read protocol file {name!r}: {error}"
        )
        pending_error = wrapped
        raise wrapped from error
    except BaseException as error:
        pending_error = error
        raise
    finally:
        cleanup_failures: list[tuple[str, Exception]] = []
        if descriptor is not None:
            descriptor_to_close = descriptor
            descriptor = None
            _attempt_cleanup(
                cleanup_failures,
                "close read descriptor",
                lambda: os.close(descriptor_to_close),
            )
        _report_cleanup_failures(name, pending_error, cleanup_failures)


def read_json_object_at(
    directory_fd: int,
    name: str,
    *,
    deadline_check: Callable[[], None] | None = None,
    max_bytes: int = 4 * 1024 * 1024,
) -> dict[str, Any] | None:
    """Read one stable, single-link JSON object relative to a directory fd."""
    result = _read_json_object_and_metadata_at(
        directory_fd,
        name,
        deadline_check=deadline_check,
        max_bytes=max_bytes,
    )
    return None if result is None else result[0]


def _apply_nonreplacing_policy(
    directory_fd: int,
    name: str,
    existing_record: tuple[dict[str, Any], os.stat_result],
    payload: bytes,
    mode: int,
    policy: WritePolicy,
) -> tuple[dict[str, Any], bool]:
    existing, metadata = existing_record
    existing_mode = stat.S_IMODE(metadata.st_mode)
    if existing_mode != mode:
        raise ProtocolIOError(
            f"protocol file {name!r} has mode {existing_mode:#05o}; "
            f"required {mode:#05o}"
        )
    if (
        policy is WritePolicy.IDENTICAL
        and canonical_json(existing) != payload
    ):
        raise ProtocolIOError(
            f"protocol file {name!r} conflicts with existing value"
        )
    if not _named_path_matches_snapshot(directory_fd, name, metadata):
        raise ProtocolIOError(
            f"protocol file {name!r} changed while applying write policy"
        )
    return existing, False


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
    lock_descriptor: int | None = None
    locked = False
    temporary_created = False
    temporary_identity: tuple[int, int] | None = None
    pending_error: BaseException | None = None
    try:
        if not isinstance(policy, WritePolicy):
            raise ProtocolIOError("unsupported protocol write policy")
        try:
            payload = canonical_json(document)
        except ProtocolIOError as error:
            raise ProtocolIOError(
                f"protocol file {name!r} contains a document that is not valid JSON: "
                f"{error}"
            ) from error
        candidate = _decode_json_object(payload, f"protocol file {name!r}")

        lock_descriptor = os.open(
            ".",
            _DIRECTORY_FLAGS,
            dir_fd=directory_fd,
        )
        fcntl.flock(lock_descriptor, fcntl.LOCK_EX)
        locked = True

        existing_record = _read_json_object_and_metadata_at(directory_fd, name)
        nonreplacing = policy in (WritePolicy.IDENTICAL, WritePolicy.FIRST_WINS)
        if existing_record is not None and nonreplacing:
            return _apply_nonreplacing_policy(
                directory_fd,
                name,
                existing_record,
                payload,
                mode,
                policy,
            )

        descriptor = os.open(
            temporary,
            _TEMPORARY_FILE_FLAGS,
            mode,
            dir_fd=directory_fd,
        )
        temporary_created = True
        temporary_metadata = os.fstat(descriptor)
        temporary_identity = (
            temporary_metadata.st_dev,
            temporary_metadata.st_ino,
        )
        os.fchmod(descriptor, mode)
        written = 0
        while written < len(payload):
            count = os.write(descriptor, payload[written:])
            if count <= 0:
                raise OSError("protocol write made no progress")
            written += count
        os.fsync(descriptor)
        sealed = os.fstat(descriptor)
        if stat.S_IMODE(sealed.st_mode) != mode or sealed.st_size != len(payload):
            raise ProtocolIOError(
                f"temporary protocol file {temporary!r} changed before publication"
            )
        if not _named_path_matches_snapshot(
            directory_fd,
            temporary,
            sealed,
        ):
            raise ProtocolIOError(
                f"temporary protocol file {temporary!r} changed before publication"
            )
        if nonreplacing:
            try:
                _rename_noreplace_at(directory_fd, temporary, name)
            except FileExistsError:
                if not _temporary_path_matches(
                    directory_fd,
                    temporary,
                    temporary_identity,
                ):
                    raise ProtocolIOError(
                        f"temporary protocol file {temporary!r} changed "
                        "during publication"
                    )
                os.unlink(temporary, dir_fd=directory_fd)
                temporary_created = False
                os.fsync(directory_fd)
                winner = _read_json_object_and_metadata_at(directory_fd, name)
                if winner is None:
                    raise ProtocolIOError(
                        f"protocol file {name!r} changed while applying write policy"
                    )
                return _apply_nonreplacing_policy(
                    directory_fd,
                    name,
                    winner,
                    payload,
                    mode,
                    policy,
                )
            temporary_created = False
        else:
            os.replace(
                temporary,
                name,
                src_dir_fd=directory_fd,
                dst_dir_fd=directory_fd,
            )
            temporary_created = False
        os.fsync(directory_fd)
        published = os.fstat(descriptor)
        published_payload = os.pread(descriptor, len(payload) + 1, 0)
        verified = os.fstat(descriptor)
        if (
            not _same_publication_state(sealed, published)
            or published_payload != payload
            or not _same_snapshot(published, verified)
            or not _named_path_matches_snapshot(directory_fd, name, verified)
        ):
            raise ProtocolIOError(
                f"protocol file {name!r} changed during publication"
            )
        return candidate, True
    except ProtocolIOError as error:
        pending_error = error
        raise
    except (TypeError, ValueError, OverflowError, RecursionError, UnicodeError) as error:
        wrapped = ProtocolIOError(
            f"protocol file {name!r} is not valid JSON: {error}"
        )
        pending_error = wrapped
        raise wrapped from error
    except OSError as error:
        wrapped = ProtocolIOError(
            f"could not persist protocol file {name!r}: {error}"
        )
        pending_error = wrapped
        raise wrapped from error
    except BaseException as error:
        pending_error = error
        raise
    finally:
        cleanup_failures: list[tuple[str, Exception]] = []
        if temporary_created:
            if temporary_identity is None:
                cleanup_failures.append(
                    (
                        "verify temporary file ownership",
                        ProtocolIOError(
                            f"temporary protocol file {temporary!r} has no recorded identity"
                        ),
                    )
                )
            else:
                try:
                    remove_temporary = _temporary_path_matches(
                        directory_fd,
                        temporary,
                        temporary_identity,
                    )
                except Exception as error:
                    cleanup_failures.append(
                        ("verify temporary file ownership", error)
                    )
                else:
                    if remove_temporary:
                        try:
                            os.unlink(temporary, dir_fd=directory_fd)
                        except Exception as error:
                            cleanup_failures.append(
                                ("remove temporary file", error)
                            )
                        else:
                            _attempt_cleanup(
                                cleanup_failures,
                                "sync directory after temporary cleanup",
                                lambda: os.fsync(directory_fd),
                            )
                    else:
                        cleanup_failures.append(
                            (
                                "verify temporary file ownership",
                                ProtocolIOError(
                                    f"temporary protocol file {temporary!r} changed"
                                ),
                            )
                        )
        if descriptor is not None:
            descriptor_to_close = descriptor
            descriptor = None
            _attempt_cleanup(
                cleanup_failures,
                "close write descriptor",
                lambda: os.close(descriptor_to_close),
            )
        if locked and lock_descriptor is not None:
            _attempt_cleanup(
                cleanup_failures,
                "release directory lock",
                lambda: fcntl.flock(lock_descriptor, fcntl.LOCK_UN),
            )
        if lock_descriptor is not None:
            lock_descriptor_to_close = lock_descriptor
            lock_descriptor = None
            _attempt_cleanup(
                cleanup_failures,
                "close directory lock descriptor",
                lambda: os.close(lock_descriptor_to_close),
            )
        _report_cleanup_failures(name, pending_error, cleanup_failures)
