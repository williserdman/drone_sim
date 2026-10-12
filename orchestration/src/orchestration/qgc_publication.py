"""Crash-aware, no-clobber publication for POSIX QGC action files."""

from __future__ import annotations

import errno
import hashlib
import os
from pathlib import Path
import stat

from .host_integration import PublicationReceipt
from .qgc_actions import QgcActionMaterial


def _open_directory(directory: Path) -> int:
    if directory.is_symlink():
        raise RuntimeError("QGC action save directory must not be a symlink")
    try:
        directory.mkdir(parents=True, exist_ok=True)
    except OSError as error:
        raise RuntimeError(f"could not create QGC action save directory {directory}") from error
    flags = (
        os.O_RDONLY
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        descriptor = os.open(directory, flags)
    except OSError as error:
        raise RuntimeError(f"QGC action save directory {directory} is unsafe") from error
    if not stat.S_ISDIR(os.fstat(descriptor).st_mode):
        os.close(descriptor)
        raise RuntimeError(f"QGC action save path {directory} is not a directory")
    return descriptor


def _read_target(directory_fd: int, name: str) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(name, flags, dir_fd=directory_fd)
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise RuntimeError("published QGC action must be a single-link regular file")
        payload = bytearray()
        while len(payload) <= before.st_size:
            chunk = os.read(descriptor, before.st_size + 1 - len(payload))
            if not chunk:
                break
            payload.extend(chunk)
        after = os.fstat(descriptor)
        identity_before = (before.st_dev, before.st_ino, before.st_size)
        identity_after = (after.st_dev, after.st_ino, after.st_size)
        if identity_before != identity_after or len(payload) != before.st_size:
            raise RuntimeError("published QGC action changed during readback")
        return bytes(payload)
    finally:
        os.close(descriptor)


def publish_posix(
    material: QgcActionMaterial, directory: Path
) -> PublicationReceipt:
    """Publish exact action bytes, accepting only an identical existing file."""

    if not isinstance(material, QgcActionMaterial):
        raise TypeError("material must be QgcActionMaterial")
    if not isinstance(directory, Path):
        raise TypeError("directory must be a Path")
    expected_hash = hashlib.sha256(material.action_bytes).hexdigest()
    if expected_hash != material.descriptor.actions_sha256:
        raise ValueError("QGC action bytes do not match their descriptor digest")

    directory_fd = _open_directory(directory)
    name = material.published_name
    created = False
    try:
        flags = (
            os.O_WRONLY
            | os.O_CREAT
            | os.O_EXCL
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0)
        )
        try:
            descriptor = os.open(name, flags, 0o600, dir_fd=directory_fd)
        except OSError as error:
            if error.errno != errno.EEXIST:
                raise
            try:
                existing = _read_target(directory_fd, name)
            except (OSError, RuntimeError) as unsafe:
                raise FileExistsError(directory / name) from unsafe
            if existing != material.action_bytes:
                raise FileExistsError(directory / name) from error
        else:
            created = True
            try:
                written = 0
                while written < len(material.action_bytes):
                    count = os.write(descriptor, material.action_bytes[written:])
                    if count <= 0:
                        raise OSError("QGC action publication made no progress")
                    written += count
                os.fsync(descriptor)
            except BaseException:
                os.close(descriptor)
                descriptor = -1
                try:
                    os.unlink(name, dir_fd=directory_fd)
                    os.fsync(directory_fd)
                except FileNotFoundError:
                    pass
                raise
            finally:
                if descriptor >= 0:
                    os.close(descriptor)
            os.fsync(directory_fd)

        readback = _read_target(directory_fd, name)
        if readback != material.action_bytes:
            if created:
                try:
                    os.unlink(name, dir_fd=directory_fd)
                    os.fsync(directory_fd)
                except FileNotFoundError:
                    pass
            raise RuntimeError("QGC action publication readback did not match source bytes")
        readback_hash = hashlib.sha256(readback).hexdigest()
        if readback_hash != expected_hash:
            raise RuntimeError("QGC action publication readback digest did not match descriptor")
        target = directory / name
        return PublicationReceipt(str(target), name, readback_hash)
    finally:
        os.close(directory_fd)


__all__ = ["publish_posix"]
