"""Durable allocation and no-clobber publication of run-bound QGC actions."""

from __future__ import annotations

from dataclasses import dataclass
import errno
import fcntl
import hashlib
import json
import os
from pathlib import Path
import stat
import threading
from typing import Any
from uuid import UUID, uuid4

from artifacts.qgc_operator import (
    MAX_ATTEMPT_TOKEN,
    MavlinkIdentity,
    QGC_ABORT_COMMAND,
    QGC_ACTIONS_BUNDLE_PATH,
    QGC_ACTIONS_KIND,
    QGC_START_COMMAND,
    QgcOperatorDescriptor,
)


_LEDGER_NAME = "attempt-ledger.json"
_MARKER_NAME = "initialized"
_LOCK_NAME = "allocation.lock"
_MARKER_BYTES = b"qgc_actions_v1\n"
_COMPANION = MavlinkIdentity(1, 191)
_FLIGHT_CONTROLLER = MavlinkIdentity(1, 1)
_allocation_mutex = threading.Lock()


@dataclass(frozen=True)
class QgcActionMaterial:
    descriptor: QgcOperatorDescriptor
    action_bytes: bytes
    published_name: str

    def __post_init__(self) -> None:
        if not isinstance(self.descriptor, QgcOperatorDescriptor):
            raise TypeError("descriptor must be a QgcOperatorDescriptor")
        if not isinstance(self.action_bytes, bytes) or not self.action_bytes:
            raise ValueError("action_bytes must be nonempty bytes")
        if (
            not isinstance(self.published_name, str)
            or not self.published_name.endswith(".json")
            or Path(self.published_name).name != self.published_name
        ):
            raise ValueError("published_name must be a JSON filename")


def _canonical_run_id(value: object) -> str:
    if not isinstance(value, str):
        raise ValueError("run_id must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (ValueError, TypeError, AttributeError) as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise ValueError("run_id must be a canonical UUID")
    return value


def _ledger_bytes(token: int) -> bytes:
    return (
        json.dumps(
            {"schema_version": 1, "last_allocated_token": token},
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    ).encode("utf-8")


def _render_actions(run_id: str, token: int) -> bytes:
    actions: list[dict[str, Any]] = []
    for label, description, command in (
        (
            "Start selected mission",
            f"Start the selected onboard mission for run {run_id}.",
            QGC_START_COMMAND,
        ),
        (
            "Abort and recover",
            f"Abort mission progression and request recovery for run {run_id}.",
            QGC_ABORT_COMMAND,
        ),
    ):
        action: dict[str, Any] = {
            "label": label,
            "description": description,
            "mavCmd": command,
            "compId": _COMPANION.component_id,
            "param1": token,
        }
        action.update({f"param{index}": 0 for index in range(2, 8)})
        actions.append(action)
    document = {
        "version": 1,
        "fileType": "MavlinkActions",
        "title": f"Drone Sim run {run_id}",
        "actions": actions,
    }
    return (json.dumps(document, ensure_ascii=False, indent=2) + "\n").encode("utf-8")


def _read_exact_file(directory_fd: int, name: str, *, label: str) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(name, flags, dir_fd=directory_fd)
    except FileNotFoundError as error:
        raise RuntimeError(f"QGC action {label} is missing") from error
    except OSError as error:
        raise RuntimeError(f"QGC action {label} is unsafe or unreadable") from error
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise RuntimeError(f"QGC action {label} must be a single-link regular file")
        if metadata.st_size > 4096:
            raise RuntimeError(f"QGC action {label} is corrupt")
        payload = bytearray()
        while len(payload) <= metadata.st_size:
            chunk = os.read(descriptor, metadata.st_size + 1 - len(payload))
            if not chunk:
                break
            payload.extend(chunk)
        if len(payload) != metadata.st_size:
            raise RuntimeError(f"QGC action {label} changed while being read")
        return bytes(payload)
    finally:
        os.close(descriptor)


def _create_file(directory_fd: int, name: str, payload: bytes) -> None:
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(name, flags, 0o600, dir_fd=directory_fd)
    try:
        written = 0
        while written < len(payload):
            count = os.write(descriptor, payload[written:])
            if count <= 0:
                raise OSError("QGC action state write made no progress")
            written += count
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _replace_file(directory_fd: int, name: str, payload: bytes) -> None:
    temporary = f".{name}.{uuid4().hex}.tmp"
    created = False
    try:
        _create_file(directory_fd, temporary, payload)
        created = True
        os.replace(
            temporary,
            name,
            src_dir_fd=directory_fd,
            dst_dir_fd=directory_fd,
        )
        created = False
        os.fsync(directory_fd)
    finally:
        if created:
            try:
                os.unlink(temporary, dir_fd=directory_fd)
                os.fsync(directory_fd)
            except FileNotFoundError:
                pass


def _read_ledger(directory_fd: int) -> int:
    payload = _read_exact_file(directory_fd, _LEDGER_NAME, label="ledger")
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError("QGC action ledger is corrupt") from error
    if not isinstance(document, dict) or set(document) != {
        "schema_version",
        "last_allocated_token",
    }:
        raise RuntimeError("QGC action ledger is corrupt")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        raise RuntimeError("QGC action ledger schema is corrupt")
    token = document["last_allocated_token"]
    if type(token) is not int or not 0 <= token <= MAX_ATTEMPT_TOKEN:
        raise RuntimeError("QGC action ledger token is corrupt")
    return token


class QgcActionStore:
    """Allocate attempt tokens and publish their immutable QGC action bytes."""

    def __init__(self, state_directory: Path, qgc_source: MavlinkIdentity) -> None:
        if not isinstance(state_directory, Path):
            raise TypeError("state_directory must be a Path")
        if not isinstance(qgc_source, MavlinkIdentity):
            raise TypeError("qgc_source must be a MavlinkIdentity")
        if qgc_source in {_COMPANION, _FLIGHT_CONTROLLER}:
            raise ValueError("QGC source must be distinct from companion and flight controller")
        self.state_directory = state_directory
        self.qgc_source = qgc_source

    def _open_state_directory(self) -> int:
        if self.state_directory.is_symlink():
            raise RuntimeError("QGC action state directory must not be a symlink")
        try:
            self.state_directory.mkdir(parents=True, mode=0o700, exist_ok=True)
        except OSError as error:
            raise RuntimeError("could not create QGC action state directory") from error
        flags = (
            os.O_RDONLY
            | getattr(os, "O_DIRECTORY", 0)
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0)
        )
        try:
            descriptor = os.open(self.state_directory, flags)
        except OSError as error:
            raise RuntimeError("QGC action state directory is unsafe") from error
        metadata = os.fstat(descriptor)
        if not stat.S_ISDIR(metadata.st_mode):
            os.close(descriptor)
            raise RuntimeError("QGC action state path must be a directory")
        return descriptor

    def _initialize_or_validate(self, directory_fd: int) -> None:
        try:
            marker = os.stat(_MARKER_NAME, dir_fd=directory_fd, follow_symlinks=False)
        except FileNotFoundError:
            marker = None
        try:
            ledger = os.stat(_LEDGER_NAME, dir_fd=directory_fd, follow_symlinks=False)
        except FileNotFoundError:
            ledger = None
        if marker is None and ledger is None:
            _create_file(directory_fd, _LEDGER_NAME, _ledger_bytes(0))
            os.fsync(directory_fd)
            _create_file(directory_fd, _MARKER_NAME, _MARKER_BYTES)
            os.fsync(directory_fd)
            return
        if marker is None:
            raise RuntimeError("QGC action initialization marker is missing")
        if ledger is None:
            raise RuntimeError("QGC action ledger is missing after initialization")
        if not stat.S_ISREG(marker.st_mode) or marker.st_nlink != 1:
            raise RuntimeError("QGC action initialization marker is unsafe")
        if not stat.S_ISREG(ledger.st_mode) or ledger.st_nlink != 1:
            raise RuntimeError("QGC action ledger is unsafe")
        if _read_exact_file(directory_fd, _MARKER_NAME, label="initialization marker") != _MARKER_BYTES:
            raise RuntimeError("QGC action initialization marker is corrupt")

    def _allocate_token(self) -> int:
        with _allocation_mutex:
            directory_fd = self._open_state_directory()
            lock_fd: int | None = None
            try:
                lock_flags = (
                    os.O_RDWR
                    | os.O_CREAT
                    | getattr(os, "O_CLOEXEC", 0)
                    | getattr(os, "O_NOFOLLOW", 0)
                )
                try:
                    lock_fd = os.open(_LOCK_NAME, lock_flags, 0o600, dir_fd=directory_fd)
                except OSError as error:
                    raise RuntimeError("QGC action allocation lock is unsafe") from error
                metadata = os.fstat(lock_fd)
                if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                    raise RuntimeError("QGC action allocation lock is unsafe")
                fcntl.flock(lock_fd, fcntl.LOCK_EX)
                self._initialize_or_validate(directory_fd)
                previous = _read_ledger(directory_fd)
                if previous >= MAX_ATTEMPT_TOKEN:
                    raise RuntimeError("QGC action attempt-token ledger is exhausted")
                token = previous + 1
                _replace_file(directory_fd, _LEDGER_NAME, _ledger_bytes(token))
                return token
            finally:
                if lock_fd is not None:
                    try:
                        fcntl.flock(lock_fd, fcntl.LOCK_UN)
                    finally:
                        os.close(lock_fd)
                os.close(directory_fd)

    def allocate(self, run_id: str) -> QgcActionMaterial:
        """Durably consume one token before constructing its private action bytes."""
        canonical_run_id = _canonical_run_id(run_id)
        token = self._allocate_token()
        start_request_id = str(uuid4())
        abort_request_id = str(uuid4())
        action_bytes = _render_actions(canonical_run_id, token)
        descriptor = QgcOperatorDescriptor(
            kind=QGC_ACTIONS_KIND,
            start_command=QGC_START_COMMAND,
            abort_command=QGC_ABORT_COMMAND,
            attempt_token=token,
            start_request_id=start_request_id,
            abort_request_id=abort_request_id,
            qgc_source=self.qgc_source,
            companion=_COMPANION,
            flight_controller=_FLIGHT_CONTROLLER,
            actions_path=QGC_ACTIONS_BUNDLE_PATH,
            actions_sha256=hashlib.sha256(action_bytes).hexdigest(),
        )
        return QgcActionMaterial(
            descriptor=descriptor,
            action_bytes=action_bytes,
            published_name=f"drone-sim-{canonical_run_id}.json",
        )

    def publish(self, material: QgcActionMaterial, save_directory: Path) -> Path:
        """Create a QGC file, or accept an existing byte-identical regular file."""
        if not isinstance(material, QgcActionMaterial):
            raise TypeError("material must be QgcActionMaterial")
        if not isinstance(save_directory, Path):
            raise TypeError("save_directory must be a Path")
        digest = hashlib.sha256(material.action_bytes).hexdigest()
        if digest != material.descriptor.actions_sha256:
            raise ValueError("QGC action bytes do not match their descriptor digest")
        if save_directory.is_symlink():
            raise RuntimeError("QGC action save directory must not be a symlink")
        save_directory.mkdir(parents=True, exist_ok=True)
        target = save_directory / material.published_name
        flags = (
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0)
        )
        try:
            descriptor = os.open(target, flags, 0o600)
        except OSError as error:
            if error.errno != errno.EEXIST:
                raise
            if target.is_symlink() or not target.is_file():
                raise FileExistsError(target) from error
            if target.read_bytes() != material.action_bytes:
                raise FileExistsError(target) from error
            return target
        try:
            written = 0
            while written < len(material.action_bytes):
                count = os.write(descriptor, material.action_bytes[written:])
                if count <= 0:
                    raise OSError("QGC action publication made no progress")
                written += count
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        directory_fd = os.open(
            save_directory,
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0),
        )
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        return target


__all__ = ["QgcActionMaterial", "QgcActionStore"]
