"""Descriptor-bounded durable status owned by the scorekeeper runtime."""

from __future__ import annotations

import json
import os
from pathlib import Path
import stat
from typing import Any, Mapping
from uuid import UUID, uuid4


_DIRECTORY_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_DIRECTORY", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)


def _canonical_run_id(value: object) -> str:
    if not isinstance(value, str):
        raise ValueError("run_id must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (TypeError, ValueError, AttributeError) as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise ValueError("run_id must be a canonical UUID")
    return value


def _validate(document: Mapping[str, Any], run_id: str) -> dict[str, Any]:
    value = dict(document)
    if (
        set(value) != {"run_id", "finished", "sim_timestamp_ns"}
        or value.get("run_id") != run_id
        or value.get("finished") is not True
        or not isinstance(value.get("sim_timestamp_ns"), int)
        or isinstance(value["sim_timestamp_ns"], bool)
        or value["sim_timestamp_ns"] < 0
    ):
        raise ValueError("score-finished status has an invalid schema")
    return value


def _open_directory(path: Path) -> int:
    before = path.lstat()
    descriptor = os.open(path, _DIRECTORY_FLAGS)
    opened = os.fstat(descriptor)
    if not stat.S_ISDIR(before.st_mode) or (before.st_dev, before.st_ino) != (
        opened.st_dev,
        opened.st_ino,
    ):
        os.close(descriptor)
        raise OSError(f"unsafe directory: {path}")
    return descriptor


def _existing(status_fd: int) -> dict[str, Any] | None:
    try:
        metadata = os.stat("score-finished.json", dir_fd=status_fd, follow_symlinks=False)
    except FileNotFoundError:
        return None
    if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
        raise FileExistsError("score-finished status path is unsafe")
    descriptor = os.open(
        "score-finished.json",
        os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
        dir_fd=status_fd,
    )
    try:
        with os.fdopen(descriptor, "rb", closefd=False) as stream:
            payload = stream.read(4097)
        if len(payload) > 4096:
            raise FileExistsError("score-finished status is too large")
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise FileExistsError("score-finished status is invalid") from error
    finally:
        os.close(descriptor)
    if not isinstance(value, dict):
        raise FileExistsError("score-finished status is invalid")
    return value


def write_score_finished(
    run_directory: Path | str,
    run_id: str,
    document: Mapping[str, Any],
) -> Path:
    """Atomically create the exact current-run completion fact without overwrite."""
    canonical = _canonical_run_id(run_id)
    value = _validate(document, canonical)
    payload = (
        json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":")) + "\n"
    ).encode("utf-8")
    run = Path(run_directory)
    run_fd = _open_directory(run)
    status_fd: int | None = None
    temporary = f".score-finished.{uuid4().hex}.tmp"
    temporary_fd: int | None = None
    try:
        status_fd = os.open(".status", _DIRECTORY_FLAGS, dir_fd=run_fd)
        existing = _existing(status_fd)
        if existing is not None:
            if existing == value:
                return run / ".status/score-finished.json"
            raise FileExistsError("score-finished status already exists differently")
        temporary_fd = os.open(
            temporary,
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0),
            0o644,
            dir_fd=status_fd,
        )
        os.fchmod(temporary_fd, 0o644)
        written = 0
        while written < len(payload):
            count = os.write(temporary_fd, payload[written:])
            if count <= 0:
                raise OSError("score-finished write made no progress")
            written += count
        os.fsync(temporary_fd)
        os.close(temporary_fd)
        temporary_fd = None
        # Hard-link publication supplies an atomic create-if-absent operation;
        # unlike rename it cannot overwrite a concurrent completion fact.
        os.link(
            temporary,
            "score-finished.json",
            src_dir_fd=status_fd,
            dst_dir_fd=status_fd,
            follow_symlinks=False,
        )
        os.unlink(temporary, dir_fd=status_fd)
        os.fsync(status_fd)
        return run / ".status/score-finished.json"
    finally:
        if temporary_fd is not None:
            os.close(temporary_fd)
        if status_fd is not None:
            try:
                os.unlink(temporary, dir_fd=status_fd)
            except FileNotFoundError:
                pass
            os.close(status_fd)
        os.close(run_fd)


__all__ = ["write_score_finished"]
