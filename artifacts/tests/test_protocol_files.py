from __future__ import annotations

from collections.abc import Iterator, Mapping
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import stat
from threading import Barrier, Event, Lock

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


def test_write_rejects_keys_that_collapse_to_duplicate_json_names(
    directory_fd,
    tmp_path: Path,
):
    class DistinctTextKey(str):
        def __new__(cls, value: str, identity: int):
            instance = super().__new__(cls, value)
            instance.identity = identity
            return instance

        def __eq__(self, other):
            return self is other

        def __hash__(self):
            return hash((str(self), self.identity))

    document = {
        DistinctTextKey("value", 1): 1,
        DistinctTextKey("value", 2): 2,
    }
    assert len(document) == 2

    with pytest.raises(ProtocolIOError, match="duplicate JSON key"):
        canonical_json(document)

    with pytest.raises(ProtocolIOError, match="duplicate JSON key"):
        write_json_object_at(
            directory_fd,
            "state.json",
            document,
            mode=0o600,
            policy=WritePolicy.REPLACE,
        )

    assert not (tmp_path / "state.json").exists()
    assert _temporary_siblings(tmp_path) == []


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


@pytest.mark.parametrize("literal", ["1e999", "-1e999"])
def test_read_rejects_nested_exponent_overflow(
    directory_fd, tmp_path: Path, literal: str
):
    (tmp_path / "state.json").write_text(
        f'{{"nested":{{"value":{literal}}}}}'
    )

    with pytest.raises(ProtocolIOError, match="state.json"):
        read_json_object_at(directory_fd, "state.json")


def test_read_accepts_finite_exponents(directory_fd, tmp_path: Path):
    (tmp_path / "state.json").write_text(
        '{"nested":{"negative":-1.25e-3,"positive":6.02e23}}'
    )

    assert read_json_object_at(directory_fd, "state.json") == {
        "nested": {"negative": -0.00125, "positive": 6.02e23}
    }


def test_read_calls_deadline_at_each_required_boundary(
    directory_fd, tmp_path: Path, monkeypatch
):
    (tmp_path / "state.json").write_text('{"run_id":"x"}')
    events: list[str] = []
    real_inspect = protocol_files._inspect_existing
    real_open = os.open
    real_read = os.read
    real_pread = os.pread
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

    def recording_pread(descriptor, size, offset):
        events.append("pread")
        return real_pread(descriptor, size, offset)

    def recording_loads(payload, *args, **kwargs):
        events.append("decode")
        return real_loads(payload, *args, **kwargs)

    monkeypatch.setattr(protocol_files, "_inspect_existing", recording_inspect)
    monkeypatch.setattr(os, "open", recording_open)
    monkeypatch.setattr(os, "read", recording_read)
    monkeypatch.setattr(os, "pread", recording_pread)
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
    pread_index = events.index("pread")
    assert events[pread_index - 1 : pread_index + 2] == [
        "deadline",
        "pread",
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


def test_read_checks_deadline_after_final_named_path_validation(
    directory_fd, tmp_path: Path, monkeypatch
):
    (tmp_path / "state.json").write_text('{"run_id":"x"}')
    marker = TimeoutError("budget expired during final validation")
    real_inspect = protocol_files._inspect_existing
    inspection_count = 0
    expired = False

    def inspect_and_expire(fd, name):
        nonlocal inspection_count, expired
        metadata = real_inspect(fd, name)
        inspection_count += 1
        if inspection_count == 3:
            expired = True
        return metadata

    def check():
        if expired:
            raise marker

    monkeypatch.setattr(protocol_files, "_inspect_existing", inspect_and_expire)

    with pytest.raises(TimeoutError) as raised:
        read_json_object_at(directory_fd, "state.json", deadline_check=check)

    assert inspection_count == 3
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


def test_read_rejects_fifo_swapped_in_before_open_without_blocking(
    directory_fd, tmp_path: Path, monkeypatch
):
    target = tmp_path / "state.json"
    target.write_text('{"version":1}')
    real_inspect = protocol_files._inspect_existing
    real_open = os.open
    replaced = False

    def inspect_then_replace(fd, name):
        nonlocal replaced
        metadata = real_inspect(fd, name)
        if not replaced:
            target.unlink()
            os.mkfifo(target)
            replaced = True
        return metadata

    def reject_blocking_target_open(path, flags, *args, **kwargs):
        if path == "state.json" and not flags & os.O_NONBLOCK:
            pytest.fail("read attempted a blocking open after FIFO swap")
        return real_open(path, flags, *args, **kwargs)

    monkeypatch.setattr(protocol_files, "_inspect_existing", inspect_then_replace)
    monkeypatch.setattr(os, "open", reject_blocking_target_open)

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


@pytest.mark.parametrize("mutation", ["replace", "content", "mode"])
def test_read_rejects_file_changed_during_json_decoding(
    directory_fd,
    tmp_path: Path,
    monkeypatch,
    mutation: str,
):
    target = tmp_path / "state.json"
    target.write_text('{"version":1}')
    replacement = tmp_path / "replacement.json"
    replacement.write_text('{"version":2}')
    real_loads = protocol_files.json.loads

    def decode_then_change(*args, **kwargs):
        document = real_loads(*args, **kwargs)
        if mutation == "replace":
            os.replace(replacement, target)
        elif mutation == "content":
            before = target.stat()
            target.write_text('{"version":2}')
            os.utime(target, ns=(before.st_atime_ns, before.st_mtime_ns))
        else:
            target.chmod(0o666)
        return document

    monkeypatch.setattr(protocol_files.json, "loads", decode_then_change)

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


def test_write_returns_document_detached_before_publication(
    directory_fd,
    tmp_path: Path,
    monkeypatch,
):
    nested = [1]
    document = {"values": nested}
    real_fsync = os.fsync
    mutated = False

    def fsync_then_mutate_input(descriptor):
        nonlocal mutated
        real_fsync(descriptor)
        if not stat.S_ISDIR(os.fstat(descriptor).st_mode) and not mutated:
            nested.append(2)
            mutated = True

    monkeypatch.setattr(os, "fsync", fsync_then_mutate_input)

    assert write_json_object_at(
        directory_fd,
        "state.json",
        document,
        mode=0o600,
        policy=WritePolicy.REPLACE,
    ) == ({"values": [1]}, True)
    assert document == {"values": [1, 2]}
    assert read_json_object_at(directory_fd, "state.json") == {"values": [1]}
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


def test_identical_policy_uses_json_type_strict_equality(
    directory_fd,
    tmp_path: Path,
):
    original = {"value": True}
    write_json_object_at(
        directory_fd,
        "state.json",
        original,
        mode=0o600,
        policy=WritePolicy.IDENTICAL,
    )

    with pytest.raises(ProtocolIOError, match="conflicts"):
        write_json_object_at(
            directory_fd,
            "state.json",
            {"value": 1},
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


def test_write_rejects_unsupported_policy_before_replacement(
    directory_fd,
    tmp_path: Path,
):
    original = {"value": 1}
    write_json_object_at(
        directory_fd,
        "state.json",
        original,
        mode=0o600,
        policy=WritePolicy.REPLACE,
    )
    target = tmp_path / "state.json"
    original_metadata = target.stat()

    with pytest.raises(ProtocolIOError, match="write policy"):
        write_json_object_at(
            directory_fd,
            "state.json",
            {"value": 2},
            mode=0o600,
            policy="identical",
        )

    current_metadata = target.stat()
    assert (current_metadata.st_dev, current_metadata.st_ino) == (
        original_metadata.st_dev,
        original_metadata.st_ino,
    )
    assert target.read_bytes() == canonical_json(original)
    assert _temporary_siblings(tmp_path) == []


@pytest.mark.parametrize(
    ("policy", "original", "retry", "required_mode"),
    [
        (
            WritePolicy.IDENTICAL,
            {"run_id": "x", "value": 1},
            {"run_id": "x", "value": 1},
            0o644,
        ),
        (
            WritePolicy.FIRST_WINS,
            {"run_id": "x", "reason": "first"},
            {"run_id": "x", "reason": "later"},
            0o600,
        ),
    ],
)
def test_nonreplacing_policy_rejects_existing_file_with_wrong_mode(
    directory_fd,
    tmp_path: Path,
    policy: WritePolicy,
    original: dict[str, object],
    retry: dict[str, object],
    required_mode: int,
):
    write_json_object_at(
        directory_fd,
        "state.json",
        original,
        mode=required_mode,
        policy=policy,
    )
    target = tmp_path / "state.json"
    target.chmod(0o666)
    original_identity = (target.stat().st_dev, target.stat().st_ino)
    original_bytes = target.read_bytes()

    with pytest.raises(ProtocolIOError, match="mode"):
        write_json_object_at(
            directory_fd,
            "state.json",
            retry,
            mode=required_mode,
            policy=policy,
        )

    metadata = target.stat()
    assert (metadata.st_dev, metadata.st_ino) == original_identity
    assert stat.S_IMODE(metadata.st_mode) == 0o666
    assert target.read_bytes() == original_bytes
    assert _temporary_siblings(tmp_path) == []


@pytest.mark.parametrize("policy", [WritePolicy.IDENTICAL, WritePolicy.FIRST_WINS])
def test_nonreplacing_policy_revalidates_after_candidate_conversion(
    directory_fd,
    tmp_path: Path,
    policy: WritePolicy,
):
    original = {"value": 1}
    write_json_object_at(
        directory_fd,
        "state.json",
        original,
        mode=0o600,
        policy=policy,
    )
    target = tmp_path / "state.json"
    replacement = tmp_path / "replacement.json"
    replacement.write_bytes(b'{"value":2}\n')
    replacement.chmod(0o666)

    class ReplacingMapping(Mapping[str, int]):
        def __getitem__(self, key: str) -> int:
            if key != "value":
                raise KeyError(key)
            return 1

        def __iter__(self) -> Iterator[str]:
            os.replace(replacement, target)
            return iter(("value",))

        def __len__(self) -> int:
            return 1

    with pytest.raises(ProtocolIOError):
        write_json_object_at(
            directory_fd,
            "state.json",
            ReplacingMapping(),
            mode=0o600,
            policy=policy,
        )

    assert target.read_bytes() == b'{"value":2}\n'
    assert stat.S_IMODE(target.stat().st_mode) == 0o666
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


def test_failed_publication_cleanup_is_directory_synced(
    directory_fd,
    tmp_path: Path,
    monkeypatch,
):
    unlink_attempts = 0
    directory_fsyncs = 0
    real_unlink = os.unlink
    real_fsync = os.fsync

    def fail_publication(*_args, **_kwargs):
        raise OSError("simulated publication interruption")

    def recording_unlink(path, **kwargs):
        nonlocal unlink_attempts
        if str(path).startswith(".state.json."):
            unlink_attempts += 1
        return real_unlink(path, **kwargs)

    def recording_fsync(descriptor):
        nonlocal directory_fsyncs
        if stat.S_ISDIR(os.fstat(descriptor).st_mode):
            directory_fsyncs += 1
        return real_fsync(descriptor)

    monkeypatch.setattr(os, "replace", fail_publication)
    monkeypatch.setattr(os, "unlink", recording_unlink)
    monkeypatch.setattr(os, "fsync", recording_fsync)

    document = {"run_id": "x"}
    with pytest.raises(ProtocolIOError, match="could not persist"):
        write_json_object_at(
            directory_fd,
            "state.json",
            document,
            mode=0o600,
            policy=WritePolicy.REPLACE,
        )

    assert unlink_attempts == 1
    assert directory_fsyncs == 1
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
        cleanup_calls.append(
            "close-directory"
            if stat.S_ISDIR(os.fstat(descriptor).st_mode)
            else "close-file"
        )
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
    assert cleanup_calls == [
        "unlink",
        "close-file",
        "unlock",
        "close-directory",
    ]
    assert len(raised.value.__notes__) == 4
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


def test_concurrent_first_wins_serializes_shared_directory_descriptor(
    directory_fd,
    tmp_path: Path,
    monkeypatch,
):
    start = Barrier(2)
    first_holds_lock = Event()
    second_attempted_lock = Event()
    acquisition_guard = Lock()
    acquisition_count = 0
    second_contended: list[bool] = []
    real_flock = protocol_files.fcntl.flock

    def probing_flock(descriptor, operation):
        nonlocal acquisition_count
        if operation != protocol_files.fcntl.LOCK_EX:
            return real_flock(descriptor, operation)

        with acquisition_guard:
            acquisition_count += 1
            acquisition = acquisition_count
        if acquisition == 1:
            real_flock(
                descriptor,
                protocol_files.fcntl.LOCK_EX | protocol_files.fcntl.LOCK_NB,
            )
            first_holds_lock.set()
            if not second_attempted_lock.wait(timeout=2):
                raise AssertionError("second writer never attempted the directory lock")
            return None

        if not first_holds_lock.wait(timeout=2):
            raise AssertionError("first writer never acquired the directory lock")
        try:
            real_flock(
                descriptor,
                protocol_files.fcntl.LOCK_EX | protocol_files.fcntl.LOCK_NB,
            )
        except BlockingIOError:
            second_contended.append(True)
            second_attempted_lock.set()
            return real_flock(descriptor, operation)
        second_contended.append(False)
        second_attempted_lock.set()
        return None

    monkeypatch.setattr(protocol_files.fcntl, "flock", probing_flock)

    def write(document):
        start.wait()
        return write_json_object_at(
            directory_fd,
            "state.json",
            document,
            mode=0o600,
            policy=WritePolicy.FIRST_WINS,
        )

    documents = ({"writer": 1}, {"writer": 2})
    with ThreadPoolExecutor(max_workers=2) as pool:
        results = tuple(pool.map(write, documents))

    assert second_contended == [True]
    winners = [persisted for persisted, created in results if created]
    assert len(winners) == 1
    assert all(persisted == winners[0] for persisted, _created in results)
    assert (tmp_path / "state.json").read_bytes() == canonical_json(winners[0])
    assert _temporary_siblings(tmp_path) == []
