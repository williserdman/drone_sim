from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import stat
from threading import Barrier

import pytest

import artifacts.protocol_files as protocol_files
from artifacts.protocol_files import (
    ProtocolIOError,
    WritePolicy,
    canonical_json,
    read_json_object_at,
    write_json_object_at,
)


@pytest.fixture
def directory_fd(tmp_path: Path):
    descriptor = os.open(tmp_path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        yield descriptor
    finally:
        os.close(descriptor)


def _temporary_siblings(tmp_path: Path, name: str = "state.json") -> list[Path]:
    return list(tmp_path.glob(f".{name}.*.tmp"))


def test_canonical_json_is_strict_sorted_and_newline_terminated():
    assert canonical_json({"z": 1, "a": "x"}) == b'{"a":"x","z":1}\n'
    with pytest.raises(ProtocolIOError):
        canonical_json({"value": float("nan")})


@pytest.mark.parametrize(
    "payload",
    [b'{"x":1,"x":2}', b'{"x":NaN}', b"[]", b"\xff"],
)
def test_read_rejects_noncanonical_json_objects(
    directory_fd, tmp_path: Path, payload: bytes
):
    (tmp_path / "state.json").write_bytes(payload)
    with pytest.raises(ProtocolIOError):
        read_json_object_at(directory_fd, "state.json")


def test_read_calls_deadline_at_each_required_boundary(
    directory_fd, tmp_path: Path, monkeypatch
):
    (tmp_path / "state.json").write_text('{"run_id":"x"}')
    events: list[str] = []
    real_inspect = protocol_files._inspect_existing
    real_open = os.open
    real_read = os.read
    real_loads = protocol_files.json.loads

    def check():
        events.append("deadline")

    def recording_inspect(fd, name):
        events.append("inspect")
        return real_inspect(fd, name)

    def recording_open(path, *args, **kwargs):
        if path == "state.json":
            events.append("open")
        return real_open(path, *args, **kwargs)

    def recording_read(descriptor, size):
        events.append("read")
        return real_read(descriptor, size)

    def recording_loads(payload, *args, **kwargs):
        events.append("decode")
        return real_loads(payload, *args, **kwargs)

    monkeypatch.setattr(protocol_files, "_inspect_existing", recording_inspect)
    monkeypatch.setattr(os, "open", recording_open)
    monkeypatch.setattr(os, "read", recording_read)
    monkeypatch.setattr(protocol_files.json, "loads", recording_loads)

    assert read_json_object_at(
        directory_fd, "state.json", deadline_check=check
    ) == {"run_id": "x"}
    assert events[:3] == ["deadline", "inspect", "open"]
    read_indices = [index for index, event in enumerate(events) if event == "read"]
    assert read_indices
    for index in read_indices:
        assert events[index - 1 : index + 2] == ["deadline", "read", "deadline"]
    decode_index = events.index("decode")
    assert events[decode_index - 1 : decode_index + 2] == [
        "deadline",
        "decode",
        "deadline",
    ]
    assert events.index("open") < read_indices[0] <= read_indices[-1] < decode_index


def test_deadline_error_propagates_unchanged(directory_fd, tmp_path: Path):
    (tmp_path / "state.json").write_text('{"run_id":"x"}')
    marker = TimeoutError("budget expired")

    def check():
        raise marker

    with pytest.raises(TimeoutError) as raised:
        read_json_object_at(directory_fd, "state.json", deadline_check=check)
    assert raised.value is marker


def test_missing_file_returns_none(directory_fd):
    assert read_json_object_at(directory_fd, "missing.json") is None


def test_read_rejects_file_larger_than_default_limit(directory_fd, tmp_path: Path):
    (tmp_path / "state.json").write_bytes(b"x" * (4 * 1024 * 1024 + 1))

    with pytest.raises(ProtocolIOError, match="too large"):
        read_json_object_at(directory_fd, "state.json")


@pytest.mark.parametrize("kind", ["symlink", "hard_link", "fifo"])
def test_read_rejects_unsafe_file_types_without_opening(
    directory_fd, tmp_path: Path, kind: str, monkeypatch
):
    target = tmp_path / "state.json"
    if kind == "symlink":
        outside = tmp_path / "outside.json"
        outside.write_text("{}")
        target.symlink_to(outside)
    elif kind == "hard_link":
        outside = tmp_path / "outside.json"
        outside.write_text("{}")
        os.link(outside, target)
    else:
        os.mkfifo(target)

    real_open = os.open

    def reject_target_open(path, *args, **kwargs):
        if path == "state.json":
            pytest.fail("unsafe file was opened")
        return real_open(path, *args, **kwargs)

    monkeypatch.setattr(os, "open", reject_target_open)

    with pytest.raises(ProtocolIOError):
        read_json_object_at(directory_fd, "state.json")


def test_read_rejects_name_replaced_between_inspection_and_open(
    directory_fd, tmp_path: Path, monkeypatch
):
    target = tmp_path / "state.json"
    target.write_text('{"version":1}')
    replacement = tmp_path / "replacement.json"
    replacement.write_text('{"version":2}')
    real_inspect = protocol_files._inspect_existing
    replaced = False

    def inspect_then_replace(fd, name):
        nonlocal replaced
        metadata = real_inspect(fd, name)
        if not replaced:
            os.replace(replacement, target)
            replaced = True
        return metadata

    monkeypatch.setattr(protocol_files, "_inspect_existing", inspect_then_replace)

    with pytest.raises(ProtocolIOError, match="changed while opening"):
        read_json_object_at(directory_fd, "state.json")


def test_read_rejects_file_changed_during_read(
    directory_fd, tmp_path: Path, monkeypatch
):
    target = tmp_path / "state.json"
    target.write_text('{"payload":"' + "x" * 100_000 + '"}')
    real_read = os.read
    changed = False

    def read_then_change(descriptor, size):
        nonlocal changed
        chunk = real_read(descriptor, size)
        if chunk and not changed:
            target.write_text('{"payload":"changed"}')
            changed = True
        return chunk

    monkeypatch.setattr(os, "read", read_then_change)

    with pytest.raises(ProtocolIOError, match="changed while reading"):
        read_json_object_at(directory_fd, "state.json")


@pytest.mark.parametrize("payload", [b'{"ok":true}', b"{"])
def test_read_descriptor_closes_after_success_and_failure(
    directory_fd, tmp_path: Path, payload: bytes, monkeypatch
):
    (tmp_path / "state.json").write_bytes(payload)
    real_open = os.open
    opened_descriptor = None

    def recording_open(path, *args, **kwargs):
        nonlocal opened_descriptor
        descriptor = real_open(path, *args, **kwargs)
        if path == "state.json":
            opened_descriptor = descriptor
        return descriptor

    monkeypatch.setattr(os, "open", recording_open)

    if payload == b"{":
        with pytest.raises(ProtocolIOError):
            read_json_object_at(directory_fd, "state.json")
    else:
        assert read_json_object_at(directory_fd, "state.json") == {"ok": True}

    assert opened_descriptor is not None
    with pytest.raises(OSError):
        os.fstat(opened_descriptor)


def test_read_preserves_timeout_when_descriptor_close_fails(
    directory_fd, tmp_path: Path, monkeypatch
):
    (tmp_path / "state.json").write_text('{"run_id":"x"}')
    timeout = TimeoutError("budget expired")
    close_error = OSError("simulated close failure")
    deadline_calls = 0
    real_close = os.close

    def check():
        nonlocal deadline_calls
        deadline_calls += 1
        if deadline_calls == 2:
            raise timeout

    def close_then_fail(descriptor):
        real_close(descriptor)
        raise close_error

    monkeypatch.setattr(os, "close", close_then_fail)

    with pytest.raises(TimeoutError) as raised:
        read_json_object_at(directory_fd, "state.json", deadline_check=check)

    assert raised.value is timeout
    assert any("simulated close failure" in note for note in timeout.__notes__)


def test_read_preserves_protocol_error_when_descriptor_close_fails(
    directory_fd, tmp_path: Path, monkeypatch
):
    (tmp_path / "state.json").write_text("{")
    close_error = OSError("simulated close failure")
    real_close = os.close

    def close_then_fail(descriptor):
        real_close(descriptor)
        raise close_error

    monkeypatch.setattr(os, "close", close_then_fail)

    with pytest.raises(ProtocolIOError, match="invalid JSON") as raised:
        read_json_object_at(directory_fd, "state.json")

    assert raised.value.__cause__ is not close_error
    assert any(
        "simulated close failure" in note for note in raised.value.__notes__
    )


def test_read_reports_descriptor_close_failure_after_success(
    directory_fd, tmp_path: Path, monkeypatch
):
    (tmp_path / "state.json").write_text('{"run_id":"x"}')
    close_error = OSError("simulated close failure")
    real_close = os.close

    def close_then_fail(descriptor):
        real_close(descriptor)
        raise close_error

    monkeypatch.setattr(os, "close", close_then_fail)

    with pytest.raises(ProtocolIOError, match="close read descriptor") as raised:
        read_json_object_at(directory_fd, "state.json")

    assert raised.value.__cause__ is close_error


@pytest.mark.parametrize("mode", [0o600, 0o644])
def test_write_sets_exact_mode_and_fsyncs_file_and_directory(
    directory_fd, tmp_path: Path, mode: int, monkeypatch
):
    events: list[str] = []
    real_fsync = os.fsync
    real_replace = os.replace

    def recording_fsync(descriptor):
        events.append(
            "directory-fsync"
            if stat.S_ISDIR(os.fstat(descriptor).st_mode)
            else "file-fsync"
        )
        real_fsync(descriptor)

    def recording_replace(source, target, **kwargs):
        events.append("replace")
        return real_replace(source, target, **kwargs)

    monkeypatch.setattr(os, "fsync", recording_fsync)
    monkeypatch.setattr(os, "replace", recording_replace)
    document = {"run_id": "x", "sequence": 1}

    previous_umask = os.umask(0o077)
    try:
        persisted, created = write_json_object_at(
            directory_fd,
            "state.json",
            document,
            mode=mode,
            policy=WritePolicy.REPLACE,
        )
    finally:
        os.umask(previous_umask)

    target = tmp_path / "state.json"
    assert (persisted, created) == (document, True)
    assert target.read_bytes() == b'{"run_id":"x","sequence":1}\n'
    assert stat.S_IMODE(target.stat().st_mode) == mode
    assert events == ["file-fsync", "replace", "directory-fsync"]
    assert _temporary_siblings(tmp_path) == []


def test_identical_policy_allows_same_retry_and_rejects_conflict(
    directory_fd, tmp_path: Path
):
    original = {"run_id": "x", "value": 1}
    changed = {"run_id": "x", "value": 2}

    assert write_json_object_at(
        directory_fd,
        "state.json",
        original,
        mode=0o600,
        policy=WritePolicy.IDENTICAL,
    ) == (original, True)
    assert write_json_object_at(
        directory_fd,
        "state.json",
        original,
        mode=0o600,
        policy=WritePolicy.IDENTICAL,
    ) == (original, False)
    with pytest.raises(ProtocolIOError, match="conflicts"):
        write_json_object_at(
            directory_fd,
            "state.json",
            changed,
            mode=0o600,
            policy=WritePolicy.IDENTICAL,
        )

    assert read_json_object_at(directory_fd, "state.json") == original
    assert _temporary_siblings(tmp_path) == []


def test_first_wins_policy_returns_original_document(directory_fd, tmp_path: Path):
    original = {"run_id": "x", "reason": "first"}
    later = {"run_id": "x", "reason": "later"}

    assert write_json_object_at(
        directory_fd,
        "state.json",
        original,
        mode=0o600,
        policy=WritePolicy.FIRST_WINS,
    ) == (original, True)
    assert write_json_object_at(
        directory_fd,
        "state.json",
        later,
        mode=0o600,
        policy=WritePolicy.FIRST_WINS,
    ) == (original, False)
    assert read_json_object_at(directory_fd, "state.json") == original
    assert _temporary_siblings(tmp_path) == []


def test_replace_policy_atomically_replaces_document(
    directory_fd, tmp_path: Path, monkeypatch
):
    original = {"run_id": "x", "sequence": 1}
    replacement = {"run_id": "x", "sequence": 2}
    write_json_object_at(
        directory_fd,
        "state.json",
        original,
        mode=0o600,
        policy=WritePolicy.REPLACE,
    )
    real_replace = os.replace
    calls = []

    def recording_replace(source, target, **kwargs):
        calls.append((source, target, kwargs))
        return real_replace(source, target, **kwargs)

    monkeypatch.setattr(os, "replace", recording_replace)

    assert write_json_object_at(
        directory_fd,
        "state.json",
        replacement,
        mode=0o600,
        policy=WritePolicy.REPLACE,
    ) == (replacement, True)
    assert read_json_object_at(directory_fd, "state.json") == replacement
    assert len(calls) == 1
    assert calls[0][1] == "state.json"
    assert calls[0][2] == {
        "src_dir_fd": directory_fd,
        "dst_dir_fd": directory_fd,
    }
    assert _temporary_siblings(tmp_path) == []


def test_interrupted_write_removes_temporary_file(
    directory_fd, tmp_path: Path, monkeypatch
):
    marker = InterruptedError("simulated interruption")

    def interrupt_write(_descriptor, _payload):
        raise marker

    monkeypatch.setattr(os, "write", interrupt_write)

    with pytest.raises(ProtocolIOError) as raised:
        write_json_object_at(
            directory_fd,
            "state.json",
            {"run_id": "x"},
            mode=0o600,
            policy=WritePolicy.REPLACE,
        )

    assert raised.value.__cause__ is marker
    assert not (tmp_path / "state.json").exists()
    assert _temporary_siblings(tmp_path) == []


def test_write_preserves_primary_error_and_attempts_every_cleanup(
    directory_fd, tmp_path: Path, monkeypatch
):
    write_error = OSError("simulated write failure")
    cleanup_calls: list[str] = []
    real_close = os.close
    real_unlink = os.unlink
    real_flock = protocol_files.fcntl.flock

    def fail_write(_descriptor, _payload):
        raise write_error

    def close_then_fail(descriptor):
        cleanup_calls.append("close")
        real_close(descriptor)
        raise OSError("simulated close failure")

    def unlink_then_fail(path, **kwargs):
        cleanup_calls.append("unlink")
        real_unlink(path, **kwargs)
        raise OSError("simulated unlink failure")

    def unlock_then_fail(descriptor, operation):
        real_flock(descriptor, operation)
        if operation == protocol_files.fcntl.LOCK_UN:
            cleanup_calls.append("unlock")
            raise OSError("simulated unlock failure")

    monkeypatch.setattr(os, "write", fail_write)
    monkeypatch.setattr(os, "close", close_then_fail)
    monkeypatch.setattr(os, "unlink", unlink_then_fail)
    monkeypatch.setattr(protocol_files.fcntl, "flock", unlock_then_fail)

    with pytest.raises(ProtocolIOError, match="could not persist") as raised:
        write_json_object_at(
            directory_fd,
            "state.json",
            {"run_id": "x"},
            mode=0o600,
            policy=WritePolicy.REPLACE,
        )

    assert raised.value.__cause__ is write_error
    assert cleanup_calls == ["close", "unlink", "unlock"]
    assert len(raised.value.__notes__) == 3
    assert _temporary_siblings(tmp_path) == []


def test_write_reports_unlock_failure_after_success(
    directory_fd, tmp_path: Path, monkeypatch
):
    unlock_error = OSError("simulated unlock failure")
    real_flock = protocol_files.fcntl.flock

    def unlock_then_fail(descriptor, operation):
        real_flock(descriptor, operation)
        if operation == protocol_files.fcntl.LOCK_UN:
            raise unlock_error

    monkeypatch.setattr(protocol_files.fcntl, "flock", unlock_then_fail)

    with pytest.raises(ProtocolIOError, match="release directory lock") as raised:
        write_json_object_at(
            directory_fd,
            "state.json",
            {"run_id": "x"},
            mode=0o600,
            policy=WritePolicy.REPLACE,
        )

    assert raised.value.__cause__ is unlock_error


def test_replace_does_not_unlink_recreated_temporary_path(
    directory_fd, tmp_path: Path, monkeypatch
):
    real_replace = os.replace
    recreated: Path | None = None

    def replace_then_recreate(source, target, **kwargs):
        nonlocal recreated
        real_replace(source, target, **kwargs)
        recreated = tmp_path / source
        recreated.write_bytes(b"belongs to another actor")

    monkeypatch.setattr(os, "replace", replace_then_recreate)

    assert write_json_object_at(
        directory_fd,
        "state.json",
        {"run_id": "x"},
        mode=0o600,
        policy=WritePolicy.REPLACE,
    ) == ({"run_id": "x"}, True)

    assert recreated is not None
    assert recreated.read_bytes() == b"belongs to another actor"


def test_concurrent_first_wins_has_one_persisted_winner(tmp_path: Path):
    documents = [{"writer": index} for index in range(8)]
    barrier = Barrier(len(documents))

    def write(document):
        descriptor = os.open(tmp_path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            barrier.wait()
            return write_json_object_at(
                descriptor,
                "state.json",
                document,
                mode=0o600,
                policy=WritePolicy.FIRST_WINS,
            )
        finally:
            os.close(descriptor)

    with ThreadPoolExecutor(max_workers=len(documents)) as pool:
        results = tuple(pool.map(write, documents))

    winners = [persisted for persisted, created in results if created]
    assert len(winners) == 1
    assert all(persisted == winners[0] for persisted, _created in results)
    assert (tmp_path / "state.json").read_bytes() == canonical_json(winners[0])
    assert _temporary_siblings(tmp_path) == []
