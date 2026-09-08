import json
import os
from pathlib import Path
import stat
from concurrent.futures import ThreadPoolExecutor

import pytest

from artifacts.protocol_files import ProtocolIOError
from artifacts.runtime_protocol import ProtocolError, RuntimeProtocol
from artifacts.runtime_status import (
    ArduPilotReadyStatus,
    ArtifactFinalRecord,
    ArtifactsFinalStatus,
    ArtifactsReadyStatus,
    CompanionReadyStatus,
    FlightExchange,
    GazeboReadyStatus,
    MissionCommandDeliveredStatus,
    MissionFinishedStatus,
    MissionReadyStatus,
    RuntimeFailureStatus,
    RuntimeFrozenStatus,
    RuntimeRunningStatus,
    ScoreFinishedStatus,
    SourceFinishedStatus,
    TerminalNotifiedStatus,
    status_document,
)
from artifacts.validation import ValidationStatus


RUN_ID = "11111111-1111-4111-8111-111111111111"
OTHER_RUN_ID = "22222222-2222-4222-8222-222222222222"
DIGEST = "a" * 64
FINAL_RECORDS = tuple(
    ArtifactFinalRecord(path, ValidationStatus.VALID, "valid", 1, DIGEST, {"ok": True})
    for path in ("video/onboard.mp4", "video/observer.mp4", "rosbag")
)
STATUSES = (
    ArtifactsReadyStatus(RUN_ID),
    GazeboReadyStatus(RUN_ID, FlightExchange(True, 1, 1, 0, 0, 1, 0, 0, 0)),
    ArduPilotReadyStatus(RUN_ID),
    CompanionReadyStatus(RUN_ID),
    MissionReadyStatus(RUN_ID),
    MissionCommandDeliveredStatus(RUN_ID, 50_000_000),
    RuntimeRunningStatus(RUN_ID, 1),
    SourceFinishedStatus(RUN_ID, 2),
    MissionFinishedStatus(RUN_ID, 3),
    ScoreFinishedStatus(RUN_ID, 4),
    RuntimeFailureStatus(RUN_ID, "gazebo", "exchange stopped", ("logs/gazebo.log",)),
    RuntimeFrozenStatus(RUN_ID),
    ArtifactsFinalStatus(RUN_ID, FINAL_RECORDS),
    TerminalNotifiedStatus(RUN_ID),
)


@pytest.fixture
def run_directory(tmp_path: Path) -> Path:
    run = tmp_path / RUN_ID
    (run / ".status").mkdir(parents=True)
    (run / ".status/quiescence").mkdir()
    (run / ".control").mkdir()
    return run


def _fail_directory_fstat(monkeypatch, target: Path) -> list[int]:
    expected = target.resolve()
    real_fstat = os.fstat
    failed: list[int] = []

    def fail_target(descriptor):
        try:
            opened_path = Path(os.readlink(f"/proc/self/fd/{descriptor}"))
        except OSError:
            return real_fstat(descriptor)
        if opened_path == expected:
            failed.append(descriptor)
            raise OSError("simulated directory fstat failure")
        return real_fstat(descriptor)

    monkeypatch.setattr(os, "fstat", fail_target)
    return failed


def _assert_failed_open_closed(failed: list[int]) -> None:
    assert len(failed) == 1
    assert not Path(f"/proc/self/fd/{failed[0]}").exists()


def test_constructor_translates_directory_fstat_failure_without_leak(
    run_directory, monkeypatch
):
    failed = _fail_directory_fstat(monkeypatch, run_directory)

    with pytest.raises(ProtocolError, match="run directory is unsafe") as raised:
        RuntimeProtocol(run_directory, RUN_ID)

    assert isinstance(raised.value.__cause__, ProtocolIOError)
    _assert_failed_open_closed(failed)


def test_child_open_translates_directory_fstat_failure_without_leak(
    run_directory, monkeypatch
):
    protocol = RuntimeProtocol(run_directory, RUN_ID)
    failed = _fail_directory_fstat(monkeypatch, run_directory / ".status")
    try:
        with pytest.raises(
            ProtocolError, match="protocol directory '.status'"
        ) as raised:
            protocol.read_status(ArtifactsReadyStatus)
        assert isinstance(raised.value.__cause__, ProtocolIOError)
        _assert_failed_open_closed(failed)
    finally:
        protocol.close()


@pytest.mark.parametrize("status", STATUSES, ids=lambda status: status.name)
def test_runtime_status_round_trips_as_typed_canonical_file(run_directory, status):
    protocol = RuntimeProtocol(run_directory, RUN_ID)

    path = protocol.write_status(status)

    expected = (
        json.dumps(
            status_document(status),
            allow_nan=False,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    ).encode()
    assert path.read_bytes() == expected
    assert stat.S_IMODE(path.stat().st_mode) == 0o644
    assert protocol.read_status(type(status)) == status


def test_identical_status_equal_retry_succeeds_and_different_retry_conflicts(run_directory):
    protocol = RuntimeProtocol(run_directory, RUN_ID)
    status = RuntimeRunningStatus(RUN_ID, 1)
    path = protocol.write_status(status)
    assert protocol.write_status(status) == path

    with pytest.raises(ProtocolError, match="conflict"):
        protocol.write_status(RuntimeRunningStatus(RUN_ID, 2))


def test_runtime_status_write_rejects_cross_run_value_before_publication(run_directory):
    path = run_directory / ".status/artifacts-ready.json"

    with pytest.raises(ProtocolError, match="wrong run_id"):
        RuntimeProtocol(run_directory, RUN_ID).write_status(
            ArtifactsReadyStatus(OTHER_RUN_ID)
        )

    assert not path.exists()


def test_runtime_failure_concurrent_distinct_writers_keep_one_valid_winner(run_directory):
    failures = tuple(
        RuntimeFailureStatus(RUN_ID, f"module-{index}", f"reason-{index}", ())
        for index in range(8)
    )
    protocol = RuntimeProtocol(run_directory, RUN_ID)
    with ThreadPoolExecutor(max_workers=len(failures)) as executor:
        paths = tuple(executor.map(protocol.write_status, failures))

    assert len(set(paths)) == 1
    assert protocol.read_status(RuntimeFailureStatus) in failures


def test_runtime_failure_rejects_malformed_persisted_winner_without_changing_it(run_directory):
    path = run_directory / ".status/runtime-failure.json"
    malformed = b'{"run_id":"wrong"}\n'
    path.write_bytes(malformed)
    path.chmod(0o644)

    with pytest.raises(ProtocolError):
        RuntimeProtocol(run_directory, RUN_ID).write_status(
            RuntimeFailureStatus(RUN_ID, "gazebo", "failed", ())
        )

    assert path.read_bytes() == malformed


def test_runtime_status_translates_malformed_document_to_protocol_error(run_directory):
    (run_directory / ".status/source-finished.json").write_bytes(
        b'{"finished":false,"run_id":"11111111-1111-4111-8111-111111111111","sim_timestamp_ns":1}\n'
    )

    with pytest.raises(ProtocolError) as raised:
        RuntimeProtocol(run_directory, RUN_ID).read_status(SourceFinishedStatus)

    assert type(raised.value) is ProtocolError


def test_host_controls_are_exact_and_first_observation_is_immutable(run_directory):
    finalize = {
        "run_id": RUN_ID,
        "requested_terminal": "COMPLETED",
        "reason": "source completed",
    }
    terminal = {
        "run_id": RUN_ID,
        "terminal_status": "COMPLETED",
        "reason": "validated",
        "manifest_path": "manifest.json",
    }
    (run_directory / ".control/finalize-request.json").write_text(json.dumps(finalize))
    protocol = RuntimeProtocol(run_directory, RUN_ID)
    assert protocol.read_finalize_request() == finalize
    assert protocol.read_finalize_request() == finalize
    (run_directory / ".control/finalize-request.json").write_text(
        json.dumps({**finalize, "reason": "rewritten"})
    )
    with pytest.raises(ProtocolError, match="changed"):
        protocol.read_finalize_request()

    (run_directory / ".control/terminal-committed.json").write_text(json.dumps(terminal))
    assert RuntimeProtocol(run_directory, RUN_ID).read_terminal_committed() == terminal


def test_manifest_status_is_read_descriptor_safely_after_terminal_commit(run_directory):
    manifest = {
        "schema_version": 1,
        "run_id": RUN_ID,
        "terminal_status": "FAILED",
        "artifacts": [
            {"relative_path": "rosbag", "validation": "missing"},
            {"relative_path": "video/onboard.mp4", "validation": "valid"},
            {"relative_path": "video/observer.mp4", "validation": "invalid"},
        ],
        "incomplete_paths": ["video/observer.mp4", "rosbag"],
    }
    (run_directory / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    protocol = RuntimeProtocol(run_directory, RUN_ID)
    read_manifest_status = getattr(protocol, "read_manifest_status", None)
    assert callable(read_manifest_status), "descriptor-safe manifest status reader is missing"
    assert read_manifest_status() == {
        "run_id": RUN_ID,
        "complete": False,
        "missing": ["rosbag", "video/observer.mp4"],
        "manifest_path": "manifest.json",
    }


@pytest.mark.parametrize("kind", ["symlink", "hardlink"])
def test_manifest_status_rejects_unsafe_manifest_path(run_directory, kind):
    outside = run_directory.parent / "outside-manifest.json"
    outside.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "run_id": RUN_ID,
                "terminal_status": "COMPLETED",
                "artifacts": [],
                "incomplete_paths": [],
            }
        ),
        encoding="utf-8",
    )
    target = run_directory / "manifest.json"
    if kind == "symlink":
        target.symlink_to(outside)
    else:
        os.link(outside, target)

    with pytest.raises(ProtocolError):
        RuntimeProtocol(run_directory, RUN_ID).read_manifest_status()


@pytest.mark.parametrize(
    "document",
    [
        {"run_id": RUN_ID, "requested_terminal": "READY", "reason": "x"},
        {"run_id": RUN_ID, "requested_terminal": "FAILED", "reason": ""},
        {"run_id": RUN_ID, "requested_terminal": "FAILED", "reason": "x", "extra": 1},
    ],
)
def test_host_control_rejects_wrong_schema(run_directory, document):
    (run_directory / ".control/finalize-request.json").write_text(json.dumps(document))
    with pytest.raises(ProtocolError):
        RuntimeProtocol(run_directory, RUN_ID).read_finalize_request()


@pytest.mark.parametrize("kind", ["symlink", "hardlink", "fifo"])
def test_protocol_rejects_unsafe_files(run_directory, kind):
    target = run_directory / "target"
    target.write_text('{"run_id":"x"}')
    path = run_directory / ".control/finalize-request.json"
    if kind == "symlink":
        path.symlink_to(target)
    elif kind == "hardlink":
        os.link(target, path)
    else:
        os.mkfifo(path)
    with pytest.raises(ProtocolError):
        RuntimeProtocol(run_directory, RUN_ID).read_finalize_request()


@pytest.mark.parametrize(
    "payload",
    [
        b'{"run_id":"x","run_id":"y"}',
        b'{"value":NaN}',
        b'{',
        b'[]',
        b'\xff',
        b'{' + b'"x":[' * 1100 + b'0' + b']' * 1100 + b'}',
    ],
)
def test_protocol_rejects_malformed_or_resource_host_json(run_directory, payload):
    (run_directory / ".control/finalize-request.json").write_bytes(payload)
    with pytest.raises(ProtocolError):
        RuntimeProtocol(run_directory, RUN_ID).read_finalize_request()


def test_runtime_protocol_translates_shared_io_errors(run_directory):
    (run_directory / ".control/finalize-request.json").write_bytes(b"{")
    protocol = RuntimeProtocol(run_directory, RUN_ID)

    with pytest.raises(ProtocolError, match="contains invalid JSON") as raised:
        protocol.read_finalize_request()
    assert type(raised.value) is ProtocolError
    assert isinstance(raised.value.__cause__, ProtocolIOError)


def test_protocol_rejects_oversized_host_file(run_directory):
    (run_directory / ".control/finalize-request.json").write_bytes(b" " * (4 * 1024 * 1024 + 1))
    with pytest.raises(ProtocolError, match="too large"):
        RuntimeProtocol(run_directory, RUN_ID).read_finalize_request()


def test_runtime_write_fsyncs_file_and_directory(monkeypatch, run_directory):
    calls = []
    real_fsync = os.fsync

    def tracked(descriptor):
        calls.append(os.fstat(descriptor).st_mode)
        return real_fsync(descriptor)

    monkeypatch.setattr(os, "fsync", tracked)
    RuntimeProtocol(run_directory, RUN_ID).write_status(ArtifactsReadyStatus(RUN_ID))
    assert len(calls) >= 2


def test_runtime_status_is_readable_by_the_non_root_host_controller(run_directory):
    previous = os.umask(0o077)
    try:
        path = RuntimeProtocol(run_directory, RUN_ID).write_status(
            ArtifactsReadyStatus(RUN_ID)
        )
    finally:
        os.umask(previous)

    assert stat.S_IMODE(path.stat().st_mode) == 0o644


def test_quiescence_markers_are_exact_descriptor_safe_and_host_readable(run_directory):
    protocol = RuntimeProtocol(run_directory, RUN_ID)
    previous = os.umask(0o077)
    try:
        path = protocol.write_quiescence("gazebo")
    finally:
        os.umask(previous)

    assert json.loads(path.read_text()) == {
        "run_id": RUN_ID,
        "module": "gazebo",
        "quiescent": True,
    }
    assert stat.S_IMODE(path.stat().st_mode) == 0o644
    assert protocol.read_quiescence("gazebo") == {
        "run_id": RUN_ID,
        "module": "gazebo",
        "quiescent": True,
    }
    assert protocol.write_quiescence("gazebo") == path


@pytest.mark.parametrize("module", ["artifacts", "../gazebo", "", "unknown"])
def test_quiescence_rejects_unowned_or_unsafe_module_names(run_directory, module):
    protocol = RuntimeProtocol(run_directory, RUN_ID)
    with pytest.raises(ValueError, match="quiescence module"):
        protocol.write_quiescence(module)
    with pytest.raises(ValueError, match="quiescence module"):
        protocol.read_quiescence(module)


@pytest.mark.parametrize(
    "payload",
    [
        b'{"run_id":"stale","module":"gazebo","quiescent":true}',
        b'{"run_id":"11111111-1111-4111-8111-111111111111","module":"scorekeeper","quiescent":true}',
        b'{"run_id":"11111111-1111-4111-8111-111111111111","module":"gazebo","quiescent":false}',
        b'{"run_id":"11111111-1111-4111-8111-111111111111","module":"gazebo","module":"gazebo","quiescent":true}',
    ],
)
def test_quiescence_read_rejects_stale_wrong_duplicate_or_invalid_schema(
    run_directory, payload
):
    (run_directory / ".status/quiescence/gazebo.json").write_bytes(payload)
    with pytest.raises(ProtocolError):
        RuntimeProtocol(run_directory, RUN_ID).read_quiescence("gazebo")


@pytest.mark.parametrize("kind", ["symlink", "hardlink", "fifo"])
def test_quiescence_rejects_unsafe_marker_paths(run_directory, kind):
    target = run_directory / "target-marker"
    target.write_text(
        json.dumps({"run_id": RUN_ID, "module": "gazebo", "quiescent": True})
    )
    path = run_directory / ".status/quiescence/gazebo.json"
    if kind == "symlink":
        path.symlink_to(target)
    elif kind == "hardlink":
        os.link(target, path)
    else:
        os.mkfifo(path)
    with pytest.raises(ProtocolError):
        RuntimeProtocol(run_directory, RUN_ID).read_quiescence("gazebo")


def test_constructor_rejects_noncanonical_id_and_unsafe_run_root(tmp_path):
    with pytest.raises(ValueError, match="canonical UUID"):
        RuntimeProtocol(tmp_path, "RUN")
    real = tmp_path / "real"
    real.mkdir()
    link = tmp_path / "link"
    link.symlink_to(real, target_is_directory=True)
    with pytest.raises(ProtocolError):
        RuntimeProtocol(link, RUN_ID)
