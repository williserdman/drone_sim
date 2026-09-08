from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import stat

import pytest

from artifacts.protocol_files import ProtocolIOError
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
    RuntimeStatusError,
    ScoreFinishedStatus,
    SourceFinishedStatus,
    TerminalNotifiedStatus,
    status_document,
)
from artifacts.validation import ValidationStatus
import orchestration.status_store as status_store_module
from orchestration.status_store import (
    OperatorStatus,
    ProtocolFileError,
    StatusStore,
)


RUN_ID = "00000000-0000-4000-8000-000000000606"
DIGEST_A = "a" * 64
DIGEST_C = "c" * 64

FLIGHT_EXCHANGE = FlightExchange(
    True,
    2,
    2,
    0,
    0,
    2,
    0,
    1,
    40_000_000,
)
FINAL_RECORDS = (
    ArtifactFinalRecord(
        "video/onboard.mp4",
        ValidationStatus.VALID,
        "valid video",
        10,
        DIGEST_A,
        {"codec": "h264"},
    ),
    ArtifactFinalRecord(
        "video/observer.mp4",
        ValidationStatus.INVALID,
        "invalid video",
        None,
        None,
        {"codec": "unknown"},
    ),
    ArtifactFinalRecord(
        "rosbag",
        ValidationStatus.VALID,
        "valid bag",
        30,
        DIGEST_C,
        {"topics": ["/clock"]},
    ),
)
FINAL_STATUS = ArtifactsFinalStatus(RUN_ID, FINAL_RECORDS)
FINAL_DOCUMENT = status_document(FINAL_STATUS)
RUNTIME_STATUSES = (
    ArtifactsReadyStatus(RUN_ID),
    GazeboReadyStatus(RUN_ID, FLIGHT_EXCHANGE),
    ArduPilotReadyStatus(RUN_ID),
    CompanionReadyStatus(RUN_ID),
    MissionReadyStatus(RUN_ID),
    MissionCommandDeliveredStatus(RUN_ID, 50_000_000),
    RuntimeRunningStatus(RUN_ID, 1),
    SourceFinishedStatus(RUN_ID, 2),
    MissionFinishedStatus(RUN_ID, 2),
    ScoreFinishedStatus(RUN_ID, 2),
    RuntimeFailureStatus(RUN_ID, "gazebo", "exchange stopped", ("logs/gazebo.log",)),
    RuntimeFrozenStatus(RUN_ID),
    FINAL_STATUS,
    TerminalNotifiedStatus(RUN_ID),
)


def _store(tmp_path: Path) -> StatusStore:
    return StatusStore(tmp_path.resolve())


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


def test_output_root_open_translates_fstat_failure_without_leak(
    tmp_path, monkeypatch
):
    output_root = tmp_path / "runs"
    output_root.mkdir()
    store = StatusStore(output_root.resolve())
    failed = _fail_directory_fstat(monkeypatch, output_root)

    with pytest.raises(ProtocolFileError, match="output root") as raised:
        store.cleanup(RUN_ID)

    assert isinstance(raised.value.__cause__, ProtocolIOError)
    _assert_failed_open_closed(failed)


@pytest.mark.parametrize("close_before_raising", (False, True))
def test_output_root_parent_close_failure_closes_owned_child(
    tmp_path, monkeypatch, close_before_raising
):
    output_root = tmp_path / "runs"
    output_root.mkdir()
    child_descriptors: list[int] = []
    parent_close_pending = False
    real_open_directory = status_store_module.open_directory
    real_close = os.close

    def recording_open_directory(path, *, dir_fd=None):
        nonlocal parent_close_pending
        descriptor = real_open_directory(path, dir_fd=dir_fd)
        opened_path = Path(os.readlink(f"/proc/self/fd/{descriptor}"))
        if opened_path == output_root.resolve():
            child_descriptors.append(descriptor)
            parent_close_pending = True
        return descriptor

    def failing_parent_close(descriptor):
        nonlocal parent_close_pending
        if parent_close_pending:
            parent_close_pending = False
            if close_before_raising:
                real_close(descriptor)
            raise OSError("simulated parent close failure")
        real_close(descriptor)

    monkeypatch.setattr(status_store_module, "open_directory", recording_open_directory)
    monkeypatch.setattr(status_store_module.os, "close", failing_parent_close)

    with pytest.raises(ProtocolFileError, match="output root is unsafe"):
        StatusStore(output_root.resolve()).cleanup(RUN_ID)

    assert len(child_descriptors) == 1
    assert not Path(f"/proc/self/fd/{child_descriptors[0]}").exists()


def test_run_open_translates_fstat_failure_without_leak(tmp_path, monkeypatch):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    failed = _fail_directory_fstat(monkeypatch, run_directory)

    with pytest.raises(
        ProtocolFileError, match="run directory is unsafe"
    ) as raised:
        store.cleanup(RUN_ID)

    assert isinstance(raised.value.__cause__, ProtocolIOError)
    _assert_failed_open_closed(failed)


def test_child_open_translates_fstat_failure_without_leak(tmp_path, monkeypatch):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    failed = _fail_directory_fstat(monkeypatch, run_directory / ".status")

    with pytest.raises(
        ProtocolFileError, match="protocol directory '.status'"
    ) as raised:
        store.read_operator_status(RUN_ID)

    assert isinstance(raised.value.__cause__, ProtocolIOError)
    _assert_failed_open_closed(failed)


def test_allocate_exclusively_creates_only_owned_protocol_directories(tmp_path):
    store = _store(tmp_path)

    run_directory = store.allocate(RUN_ID)

    assert run_directory == tmp_path / RUN_ID
    assert sorted(path.name for path in run_directory.iterdir()) == [
        ".control",
        ".status",
        "configuration",
    ]
    quiescence = run_directory / ".status/quiescence"
    assert quiescence.is_dir()
    assert stat.S_IMODE(quiescence.stat().st_mode) == 0o755
    with pytest.raises(FileExistsError):
        store.allocate(RUN_ID)


@pytest.mark.parametrize(
    "run_id",
    ["../escape", "not-a-uuid", "00000000-0000-4000-8000-000000000606/child"],
)
def test_run_ids_are_canonical_uuids_and_cannot_escape_output_root(tmp_path, run_id):
    with pytest.raises(ValueError):
        _store(tmp_path).run_directory(run_id)


def test_allocate_rejects_symlink_output_root(tmp_path):
    real = tmp_path / "real"
    real.mkdir()
    alias = tmp_path / "alias"
    alias.symlink_to(real, target_is_directory=True)

    with pytest.raises(ProtocolFileError, match="output root"):
        StatusStore(alias.absolute()).allocate(RUN_ID)


def test_allocate_rejects_symlink_in_output_root_ancestor(tmp_path):
    real = tmp_path / "real"
    real.mkdir()
    alias = tmp_path / "alias"
    alias.symlink_to(real, target_is_directory=True)

    with pytest.raises(ProtocolFileError, match="output root"):
        StatusStore((alias / "runs").absolute()).allocate(RUN_ID)


def test_allocate_does_not_create_outside_through_missing_path_below_symlink(tmp_path):
    outside = tmp_path / "outside"
    outside.mkdir()
    inside = tmp_path / "inside"
    inside.mkdir()
    (inside / "redirect").symlink_to(outside, target_is_directory=True)
    store = StatusStore((inside / "redirect" / "new" / "runs").absolute())

    with pytest.raises(ProtocolFileError, match="symlink"):
        store.allocate(RUN_ID)

    assert not (outside / "new").exists()


def test_operator_state_is_atomic_valid_json_and_leaves_no_temp_sibling(tmp_path):
    store = _store(tmp_path)
    store.allocate(RUN_ID)
    running = OperatorStatus(RUN_ID, "RUNNING", "", None)

    path = store.write_operator_status(running)

    assert path == tmp_path / RUN_ID / ".status/operator-state.json"
    assert json.loads(path.read_text(encoding="utf-8")) == running.to_dict()
    assert list(path.parent.glob(".operator-state.json.*.tmp")) == []
    assert store.read_operator_status(RUN_ID) == running


def test_status_store_translates_shared_io_errors(tmp_path):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    (run_directory / ".status/operator-state.json").write_text(
        "{", encoding="utf-8"
    )

    with pytest.raises(ProtocolFileError, match="contains invalid JSON") as raised:
        store.read_operator_status(RUN_ID)

    assert type(raised.value) is ProtocolFileError
    assert isinstance(raised.value.__cause__, ProtocolIOError)


def test_operator_state_replacement_fsyncs_file_then_status_directory(
    tmp_path, monkeypatch
):
    store = _store(tmp_path)
    store.allocate(RUN_ID)
    events: list[str] = []
    real_fsync = os.fsync
    real_replace = os.replace

    def recording_fsync(descriptor):
        events.append(
            "directory-fsync"
            if os.path.isdir(f"/proc/self/fd/{descriptor}")
            else "file-fsync"
        )
        real_fsync(descriptor)

    def recording_replace(source, target, **kwargs):
        events.append("replace")
        return real_replace(source, target, **kwargs)

    monkeypatch.setattr(os, "fsync", recording_fsync)
    monkeypatch.setattr(os, "replace", recording_replace)

    store.write_operator_status(OperatorStatus(RUN_ID, "STARTING", "", None))

    assert events[-3:] == ["file-fsync", "replace", "directory-fsync"]


def test_read_rejects_corrupt_nonregular_and_hardlinked_protocol_files(tmp_path):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    target = run_directory / ".status/operator-state.json"
    target.write_text("{", encoding="utf-8")
    with pytest.raises(ProtocolFileError, match="JSON"):
        store.read_operator_status(RUN_ID)

    target.unlink()
    target.mkdir()
    with pytest.raises(ProtocolFileError, match="regular"):
        store.read_operator_status(RUN_ID)

    target.rmdir()
    target.write_text("{}", encoding="utf-8")
    os.link(target, run_directory / ".status/alias")
    with pytest.raises(ProtocolFileError, match="hard link"):
        store.read_operator_status(RUN_ID)


def test_runtime_status_read_consumes_cooperative_deadline_callback(tmp_path):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    (run_directory / ".status/artifacts-final.json").write_text(
        json.dumps({"run_id": RUN_ID, "payload": "x" * 200_000})
    )
    checks = 0

    timeout = TimeoutError("finalization_deadline")

    def deadline_check():
        nonlocal checks
        checks += 1
        if checks == 3:
            raise timeout

    with pytest.raises(TimeoutError, match="finalization_deadline") as raised:
        store.read_runtime_status(
            RUN_ID,
            ArtifactsFinalStatus,
            deadline_check=deadline_check,
        )

    assert checks == 3
    assert raised.value is timeout


def test_abort_request_is_atomic_idempotent_and_first_cause_wins(tmp_path):
    store = _store(tmp_path)
    store.allocate(RUN_ID)

    def request():
        return store.request_finalization(RUN_ID, "ABORTED", "operator_abort")

    with ThreadPoolExecutor(max_workers=4) as pool:
        requests = tuple(pool.map(lambda _: request(), range(4)))

    assert len({json.dumps(value, sort_keys=True) for value in requests}) == 1
    assert requests[0] == {
        "run_id": RUN_ID,
        "requested_terminal": "ABORTED",
        "reason": "operator_abort",
    }
    target = tmp_path / RUN_ID / ".control/finalize-request.json"
    assert json.loads(target.read_text(encoding="utf-8")) == requests[0]
    assert list(target.parent.glob(".finalize-request.json.*.tmp")) == []

    later = store.request_finalization(RUN_ID, "FAILED", "later_failure")
    assert later == requests[0]


def test_finalize_request_refuses_unsafe_preexisting_target(tmp_path):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    outside = tmp_path / "outside"
    outside.write_text("keep", encoding="utf-8")
    (run_directory / ".control/finalize-request.json").symlink_to(outside)

    with pytest.raises(ProtocolFileError):
        store.request_finalization(RUN_ID, "ABORTED", "operator_abort")
    assert outside.read_text(encoding="utf-8") == "keep"


@pytest.mark.parametrize("status", RUNTIME_STATUSES, ids=lambda value: value.name)
def test_runtime_status_reads_every_registered_value_as_its_exact_type(tmp_path, status):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    target = run_directory / f".status/{status.name}.json"
    target.write_text(json.dumps(status_document(status)), encoding="utf-8")
    before = target.stat()

    parsed = store.read_runtime_status(RUN_ID, type(status))

    assert type(parsed) is type(status)
    assert parsed == status
    assert target.stat() == before


def test_runtime_status_read_requires_matching_run_id(tmp_path):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    target = run_directory / ".status/artifacts-ready.json"
    target.write_text(
        json.dumps({"run_id": "00000000-0000-4000-8000-000000000999", "ready": True}),
        encoding="utf-8",
    )

    with pytest.raises(ProtocolFileError, match="run_id"):
        store.read_runtime_status(RUN_ID, ArtifactsReadyStatus)


@pytest.mark.parametrize(
    ("status_type", "document"),
    [
        (
            SourceFinishedStatus,
            {"run_id": RUN_ID, "finished": False, "sim_timestamp_ns": 2},
        ),
        (RuntimeFrozenStatus, {"run_id": RUN_ID, "frozen": False}),
        (
            ArtifactsFinalStatus,
            {
                **FINAL_DOCUMENT,
                "records": [
                    {
                        **FINAL_DOCUMENT["records"][0],
                        "detail": "",
                    },
                    *FINAL_DOCUMENT["records"][1:],
                ],
            },
        ),
        (
            ArtifactsFinalStatus,
            {
                **FINAL_DOCUMENT,
                "records": [
                    {
                        **FINAL_DOCUMENT["records"][0],
                        "size_bytes": None,
                        "sha256": None,
                    },
                    *FINAL_DOCUMENT["records"][1:],
                ],
            },
        ),
        (
            ArtifactsFinalStatus,
            {
                **FINAL_DOCUMENT,
                "records": [
                    FINAL_DOCUMENT["records"][0],
                    {
                        **FINAL_DOCUMENT["records"][1],
                        "size_bytes": 20,
                    },
                    FINAL_DOCUMENT["records"][2],
                ],
            },
        ),
        (
            ArtifactsFinalStatus,
            {
                **FINAL_DOCUMENT,
                "records": list(reversed(FINAL_DOCUMENT["records"])),
            },
        ),
    ],
    ids=(
        "source-not-finished",
        "runtime-not-frozen",
        "empty-artifact-detail",
        "valid-artifact-without-size-hash",
        "half-present-size-hash",
        "reordered-artifacts",
    ),
)
def test_runtime_status_read_rejects_malformed_typed_documents(
    tmp_path, status_type, document
):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    (run_directory / f".status/{status_type.name}.json").write_text(
        json.dumps(document), encoding="utf-8"
    )

    with pytest.raises(ProtocolFileError) as raised:
        store.read_runtime_status(RUN_ID, status_type)

    assert isinstance(raised.value.__cause__, RuntimeStatusError)


def test_protocol_json_rejects_nonstandard_nan_numbers(tmp_path):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    target = run_directory / ".status/artifacts-ready.json"
    target.write_text(
        '{"run_id":"00000000-0000-4000-8000-000000000606","ready":NaN}',
        encoding="utf-8",
    )

    with pytest.raises(ProtocolFileError, match="JSON"):
        store.read_runtime_status(RUN_ID, ArtifactsReadyStatus)


def test_terminal_commit_is_exclusive_idempotent_and_never_rewrites(tmp_path):
    store = _store(tmp_path)
    store.allocate(RUN_ID)
    document = {
        "run_id": RUN_ID,
        "terminal_status": "FAILED",
        "reason": "recorder_failed",
        "manifest_path": "manifest.json",
    }

    path = store.write_terminal_committed(RUN_ID, document)
    original = path.read_bytes()
    assert store.write_terminal_committed(RUN_ID, document) == path
    with pytest.raises(ProtocolFileError, match="differently"):
        store.write_terminal_committed(RUN_ID, {**document, "reason": "other"})
    assert path.read_bytes() == original


def test_repeated_cleanup_is_non_destructive(tmp_path):
    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    payload = run_directory / "video/onboard.mp4.partial"
    payload.parent.mkdir()
    payload.write_bytes(b"surviving bytes")

    store.cleanup(RUN_ID)
    store.cleanup(RUN_ID)

    assert run_directory.is_dir()
    assert payload.read_bytes() == b"surviving bytes"


def test_collect_manifest_validation_rejects_missing_required_inventory_record(tmp_path):
    from datetime import datetime, timezone

    from artifacts import ArtifactSession, FinalizationInput
    from artifacts.manifest import ConfigurationRecord, REQUIRED_ARTIFACT_PATHS

    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    for relative_path in REQUIRED_ARTIFACT_PATHS:
        target = run_directory / relative_path
        if relative_path in {"configuration", "gazebo/state", "rosbag"}:
            target.mkdir(parents=True, exist_ok=True)
            (target / "content").write_bytes(b"artifact")
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(b"artifact")
    now = datetime(2026, 8, 24, tzinfo=timezone.utc)
    ArtifactSession(run_directory).finalize(
        FinalizationInput(
            RUN_ID,
            "COMPLETED",
            "mission_complete",
            None,
            None,
            now,
            now,
            (),
            (),
            (ConfigurationRecord("configuration/run.json", "a" * 64),),
            None,
            None,
            None,
            (),
        )
    )
    manifest = run_directory / "manifest.json"
    document = json.loads(manifest.read_text())
    document["artifacts"] = document["artifacts"][1:]
    manifest.chmod(0o644)
    manifest.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ProtocolFileError, match="required inventory"):
        store.validated_manifest_path(RUN_ID)


def test_manifest_cross_reader_accepts_the_same_portable_paths(tmp_path):
    from datetime import datetime, timezone

    from artifacts import ArtifactSession, FinalizationInput
    from artifacts.manifest import ConfigurationRecord, REQUIRED_ARTIFACT_PATHS
    from artifacts.runtime_protocol import RuntimeProtocol

    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    for relative_path in REQUIRED_ARTIFACT_PATHS:
        target = run_directory / relative_path
        if relative_path in {"configuration", "gazebo/state", "rosbag"}:
            target.mkdir(parents=True, exist_ok=True)
            (target / "content").write_bytes(b"artifact")
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(b"artifact")
    now = datetime(2026, 8, 24, tzinfo=timezone.utc)
    ArtifactSession(run_directory).finalize(
        FinalizationInput(
            RUN_ID,
            "COMPLETED",
            "mission_complete",
            None,
            None,
            now,
            now,
            (),
            (),
            (ConfigurationRecord("configuration/run.json", "a" * 64),),
            None,
            None,
            None,
            ("scoring/events.jsonl#event-12",),
        )
    )

    assert store.validated_manifest_path(RUN_ID) == run_directory / "manifest.json"
    with RuntimeProtocol(run_directory, RUN_ID) as protocol:
        assert protocol.read_manifest_status()["complete"] is True


def test_manifest_host_reader_rejects_backslash_artifact_path(tmp_path):
    from datetime import datetime, timezone

    from artifacts import ArtifactSession, FinalizationInput
    from artifacts.manifest import ConfigurationRecord, REQUIRED_ARTIFACT_PATHS

    store = _store(tmp_path)
    run_directory = store.allocate(RUN_ID)
    for relative_path in REQUIRED_ARTIFACT_PATHS:
        target = run_directory / relative_path
        if relative_path in {"configuration", "gazebo/state", "rosbag"}:
            target.mkdir(parents=True, exist_ok=True)
            (target / "content").write_bytes(b"artifact")
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(b"artifact")
    now = datetime(2026, 8, 24, tzinfo=timezone.utc)
    ArtifactSession(run_directory).finalize(
        FinalizationInput(
            RUN_ID,
            "COMPLETED",
            "mission_complete",
            None,
            None,
            now,
            now,
            (),
            (),
            (ConfigurationRecord("configuration/run.json", "a" * 64),),
            None,
            None,
            None,
            (),
        )
    )
    bad_path = r"logs/docker/bad\name.log"
    target = run_directory / bad_path
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(b"bad")
    manifest_path = run_directory / "manifest.json"
    document = json.loads(manifest_path.read_text(encoding="utf-8"))
    document["artifacts"].append(
        {
            "relative_path": bad_path,
            "size_bytes": 3,
            "sha256": hashlib.sha256(b"bad").hexdigest(),
            "validation": "valid",
            "detail": "valid regular file",
        }
    )
    manifest_path.chmod(0o644)
    manifest_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ProtocolFileError, match="artifact path"):
        store.validated_manifest_path(RUN_ID)


def test_read_only_manifest_lookup_does_not_create_missing_output_root(tmp_path):
    output_root = tmp_path / "does-not-exist"
    store = StatusStore(output_root.resolve())

    with pytest.raises(ProtocolFileError, match="output root"):
        store.validated_manifest_path(RUN_ID)

    assert not output_root.exists()
