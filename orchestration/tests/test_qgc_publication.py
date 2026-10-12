from __future__ import annotations

import hashlib
import os
from pathlib import Path

import pytest

from artifacts.qgc_operator import MavlinkIdentity
import orchestration.qgc_publication as publication_module
from orchestration.qgc_actions import QgcActionStore
from orchestration.qgc_publication import publish_posix


RUN_ID = "11111111-1111-4111-8111-111111111111"
QGC_SOURCE = MavlinkIdentity(255, 190)


def _material(tmp_path: Path):
    return QgcActionStore(tmp_path / "state", QGC_SOURCE).allocate(RUN_ID)


def test_publish_posix_writes_exact_bytes_and_returns_readback_hash(
    tmp_path: Path,
) -> None:
    material = _material(tmp_path)
    directory = tmp_path / "QGroundControl" / "MavlinkActions"

    receipt = publish_posix(material, directory)

    published = directory / material.published_name
    assert published.read_bytes() == material.action_bytes
    assert receipt.native_path == str(published)
    assert receipt.filename == material.published_name
    assert receipt.sha256 == hashlib.sha256(published.read_bytes()).hexdigest()


def test_publish_posix_accepts_only_identical_regular_duplicate(tmp_path: Path) -> None:
    material = _material(tmp_path)
    directory = tmp_path / "MavlinkActions"
    directory.mkdir()
    unrelated = directory / "custom_missions.json"
    unrelated.write_bytes(b"user-owned\n")
    first = publish_posix(material, directory)

    assert publish_posix(material, directory) == first
    assert unrelated.read_bytes() == b"user-owned\n"

    target = directory / material.published_name
    target.write_bytes(b"foreign bytes\n")
    with pytest.raises(FileExistsError, match=material.published_name):
        publish_posix(material, directory)

    target.unlink()
    foreign = tmp_path / "foreign"
    foreign.write_bytes(material.action_bytes)
    target.symlink_to(foreign)
    with pytest.raises(FileExistsError, match=material.published_name):
        publish_posix(material, directory)


def test_publish_posix_rejects_symlink_directory(tmp_path: Path) -> None:
    material = _material(tmp_path)
    real = tmp_path / "real"
    real.mkdir()
    linked = tmp_path / "linked"
    linked.symlink_to(real, target_is_directory=True)

    with pytest.raises(RuntimeError, match="directory.*symlink"):
        publish_posix(material, linked)


def test_publish_posix_removes_partial_file_after_interrupted_write(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    material = _material(tmp_path)
    directory = tmp_path / "MavlinkActions"
    real_write = os.write
    calls = 0

    def interrupted_write(descriptor: int, payload: bytes) -> int:
        nonlocal calls
        calls += 1
        if calls == 1:
            return real_write(descriptor, payload[:10])
        raise OSError("injected interruption")

    monkeypatch.setattr(publication_module.os, "write", interrupted_write)
    with pytest.raises(OSError, match="injected interruption"):
        publish_posix(material, directory)

    assert not (directory / material.published_name).exists()


def test_publish_posix_fails_when_final_readback_differs(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    material = _material(tmp_path)
    directory = tmp_path / "MavlinkActions"
    monkeypatch.setattr(publication_module, "_read_target", lambda *_args: b"wrong")

    with pytest.raises(RuntimeError, match="readback"):
        publish_posix(material, directory)
