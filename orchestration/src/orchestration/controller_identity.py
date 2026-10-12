"""Durable Linux controller identity paired with the QGC token ledger."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import stat
from uuid import UUID, uuid4

from artifacts.qgc_operator import MavlinkIdentity

from .host_integration import ControllerIdentity, ControllerRegistration, HostLease
from .qgc_actions import QgcActionStore


_IDENTITY_NAME = "controller-identity.json"
_LEDGER_NAME = "attempt-ledger.json"
_MARKER_NAME = "initialized"


@dataclass(frozen=True, slots=True)
class ControllerEnrollment:
    identity: ControllerIdentity
    registration: ControllerRegistration


def _identity_bytes(identity: ControllerIdentity) -> bytes:
    return (
        json.dumps(
            {
                "installation_id": str(identity.installation_id),
                "ledger_id": str(identity.ledger_id),
                "schema_version": 1,
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    ).encode("utf-8")


def _write_identity(directory_fd: int, identity: ControllerIdentity) -> None:
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(_IDENTITY_NAME, flags, 0o600, dir_fd=directory_fd)
    payload = _identity_bytes(identity)
    try:
        offset = 0
        while offset < len(payload):
            written = os.write(descriptor, payload[offset:])
            if written <= 0:
                raise OSError("controller identity write made no progress")
            offset += written
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    os.fsync(directory_fd)


def _read_identity(directory_fd: int) -> ControllerIdentity:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(_IDENTITY_NAME, flags, dir_fd=directory_fd)
    except OSError as error:
        raise RuntimeError(
            "controller identity is missing or unsafe; explicit recovery is required"
        ) from error
    try:
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_nlink != 1
            or metadata.st_size > 1024
        ):
            raise RuntimeError(
                "controller identity is unsafe; explicit recovery is required"
            )
        payload = os.read(descriptor, metadata.st_size + 1)
        if len(payload) != metadata.st_size:
            raise RuntimeError(
                "controller identity changed while reading; explicit recovery is required"
            )
    finally:
        os.close(descriptor)
    try:
        document = json.loads(payload.decode("utf-8"))
        if not isinstance(document, dict) or set(document) != {
            "schema_version",
            "installation_id",
            "ledger_id",
        }:
            raise ValueError
        if type(document["schema_version"]) is not int or document["schema_version"] != 1:
            raise ValueError
        installation_id = UUID(document["installation_id"])
        ledger_id = UUID(document["ledger_id"])
        if (
            str(installation_id) != document["installation_id"]
            or str(ledger_id) != document["ledger_id"]
            or installation_id.int == 0
            or ledger_id.int == 0
        ):
            raise ValueError
    except (KeyError, TypeError, ValueError, UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError(
            "controller identity is corrupt; explicit recovery is required"
        ) from error
    return ControllerIdentity(installation_id, ledger_id)


def load_or_enroll_identity(
    state_directory: Path, lease: HostLease
) -> ControllerEnrollment:
    """Load one shared identity or enroll a wholly pristine ledger."""
    if not isinstance(state_directory, Path):
        raise TypeError("state_directory must be a Path")
    if state_directory.is_symlink():
        raise RuntimeError(
            "controller state directory is a symlink; explicit recovery is required"
        )
    state_directory.mkdir(parents=True, mode=0o700, exist_ok=True)
    flags = (
        os.O_RDONLY
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    directory_fd = os.open(state_directory, flags)
    try:
        metadata = os.fstat(directory_fd)
        if not stat.S_ISDIR(metadata.st_mode):
            raise RuntimeError("controller state path must be a directory")
        entries = set(os.listdir(directory_fd))
        required = {_IDENTITY_NAME, _LEDGER_NAME, _MARKER_NAME}
        pristine = not entries
        if not pristine and not required.issubset(entries):
            raise RuntimeError(
                "controller identity or token ledger is partial; explicit recovery is required"
            )
        if pristine:
            store = QgcActionStore(state_directory, MavlinkIdentity(255, 190))
            store.initialize()
            identity = ControllerIdentity(uuid4(), uuid4())
            _write_identity(directory_fd, identity)
        else:
            identity = _read_identity(directory_fd)
            QgcActionStore(state_directory, MavlinkIdentity(255, 190)).initialize()
        registration = lease.register_controller(identity, pristine=pristine)
        return ControllerEnrollment(identity, registration)
    finally:
        os.close(directory_fd)


__all__ = ["ControllerEnrollment", "load_or_enroll_identity"]
