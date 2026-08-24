import json
import os
from pathlib import Path
import stat

import pytest

from artifacts.runtime_protocol import ProtocolError, RuntimeProtocol


RUN_ID = "11111111-1111-4111-8111-111111111111"


@pytest.fixture
def run_directory(tmp_path: Path) -> Path:
    run = tmp_path / RUN_ID
    (run / ".status").mkdir(parents=True)
    (run / ".status/quiescence").mkdir()
    (run / ".control").mkdir()
    return run


def test_runtime_status_schemas_round_trip_and_conflicting_rewrite_is_rejected(run_directory):
    protocol = RuntimeProtocol(run_directory, RUN_ID)
    documents = {
        "artifacts-ready": {"run_id": RUN_ID, "ready": True},
        "runtime-running": {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": 0},
        "source-finished": {"run_id": RUN_ID, "finished": True, "sim_timestamp_ns": 2_000_000_000},
        "runtime-failure": {
            "run_id": RUN_ID,
            "module": "artifacts",
            "reason": "observer encoder failed",
            "diagnostic_paths": ["logs/docker/ffmpeg-observer.log.partial"],
        },
        "runtime-frozen": {"run_id": RUN_ID, "frozen": True},
        "terminal-notified": {"run_id": RUN_ID, "notified": True},
    }
    for name, document in documents.items():
        path = protocol.write_status(name, document)
        assert json.loads(path.read_text()) == document
        assert protocol.write_status(name, document) == path

    changed = {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": 1}
    with pytest.raises(ProtocolError, match="conflict"):
        protocol.write_status("runtime-running", changed)


@pytest.mark.parametrize(
    ("name", "document"),
    [
        ("artifacts-ready", {"run_id": RUN_ID, "ready": 1}),
        ("runtime-running", {"run_id": RUN_ID, "state": "READY", "sim_timestamp_ns": 0}),
        ("runtime-running", {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": True}),
        ("source-finished", {"run_id": RUN_ID, "finished": True, "sim_timestamp_ns": -1}),
        ("runtime-failure", {"run_id": RUN_ID, "module": "", "reason": "bad", "diagnostic_paths": []}),
        ("runtime-failure", {"run_id": RUN_ID, "module": "x", "reason": "bad", "diagnostic_paths": ["../x"]}),
        ("runtime-failure", {"run_id": RUN_ID, "module": "x", "reason": "bad", "diagnostic_paths": ["a", "a"]}),
        ("runtime-frozen", {"run_id": RUN_ID, "frozen": False}),
        ("terminal-notified", {"run_id": RUN_ID, "notified": True, "extra": 1}),
    ],
)
def test_runtime_status_rejects_wrong_schema(run_directory, name, document):
    with pytest.raises((ProtocolError, ValueError)):
        RuntimeProtocol(run_directory, RUN_ID).write_status(name, document)


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
    RuntimeProtocol(run_directory, RUN_ID).write_status(
        "artifacts-ready", {"run_id": RUN_ID, "ready": True}
    )
    assert len(calls) >= 2


def test_runtime_status_is_readable_by_the_non_root_host_controller(run_directory):
    previous = os.umask(0o077)
    try:
        path = RuntimeProtocol(run_directory, RUN_ID).write_status(
            "artifacts-ready", {"run_id": RUN_ID, "ready": True}
        )
    finally:
        os.umask(previous)

    assert stat.S_IMODE(path.stat().st_mode) == 0o644


@pytest.mark.parametrize("bad_path", [None, 7, True, {"path": "logs/x"}, ["logs/x"]])
def test_runtime_failure_rejects_non_string_diagnostic_paths_as_protocol_error(
    run_directory, bad_path
):
    with pytest.raises(ProtocolError, match="invalid schema"):
        RuntimeProtocol(run_directory, RUN_ID).write_status(
            "runtime-failure",
            {
                "run_id": RUN_ID,
                "module": "artifacts",
                "reason": "bad diagnostic path",
                "diagnostic_paths": [bad_path],
            },
        )


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
