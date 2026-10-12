from __future__ import annotations

import base64
import json
from pathlib import Path
import sys
import textwrap
import time
from uuid import uuid4

import pytest

import orchestration.windows_helper as helper_module
from orchestration.windows_helper import (
    MAX_DATAGRAM_BYTES,
    MAX_FRAME_BYTES,
    WindowsHelper,
    WindowsHelperError,
)


SESSION_ID = "11111111-1111-4111-8111-111111111111"
SOURCE_DIGEST = "f" * 64


def _fixture_command(tmp_path: Path, body: str) -> list[str]:
    script = tmp_path / "helper_fixture.py"
    header = f'''import base64
import json
import sys
import time

session_id = {SESSION_ID!r}

def emit(value):
    sys.stdout.write(json.dumps(value, separators=(",", ":")) + "\\n")
    sys.stdout.flush()

emit({{
    "version": 1,
    "kind": "status",
    "session_id": session_id,
    "state": "ready",
    "helper_version": "1",
    "source_sha256": {SOURCE_DIGEST!r},
}})
time.sleep(0.02)
'''
    script.write_text(header + textwrap.dedent(body), encoding="utf-8")
    return [sys.executable, "-u", str(script)]


def _start(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, body: str) -> WindowsHelper:
    command = _fixture_command(tmp_path, body)
    monkeypatch.setattr(helper_module, "_build_helper_command", lambda: command)
    monkeypatch.setattr(helper_module, "_packaged_source_digest", lambda: SOURCE_DIGEST)
    return WindowsHelper.start(session_id=SESSION_ID)


def test_stderr_cannot_be_parsed_as_control_output(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    helper = _start(
        tmp_path,
        monkeypatch,
        """
        for line in sys.stdin:
            request = json.loads(line)
            print(json.dumps({"version": 1, "kind": "response", "session_id": session_id,
                              "request_id": request["request_id"], "ok": True,
                              "result": {"from": "stderr"}}), file=sys.stderr, flush=True)
            emit({"version": 1, "kind": "response", "session_id": session_id,
                  "request_id": request["request_id"], "ok": True,
                  "result": {"from": "stdout"}})
            if request["operation"] == "shutdown":
                break
        """,
    )
    try:
        assert helper.info.version == "1"
        assert helper.info.source_sha256 == SOURCE_DIGEST
        responses = [helper.request("probe", {})]
        assert responses == [{"from": "stdout"}]
    finally:
        helper.close()


def test_request_rejects_unknown_operation_before_writing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    helper = _start(
        tmp_path,
        monkeypatch,
        """
        for line in sys.stdin:
            request = json.loads(line)
            emit({"version": 1, "kind": "response", "session_id": session_id,
                  "request_id": request["request_id"], "ok": True, "result": {}})
            if request["operation"] == "shutdown": break
        """,
    )
    try:
        with pytest.raises(ValueError, match="not allowed"):
            helper.request("run_command", {"command": "whoami"})
        assert helper.request("probe", {}) == {}
    finally:
        helper.close()


def test_request_caps_serialized_utf8_bytes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    helper = _start(tmp_path, monkeypatch, "time.sleep(2)")
    try:
        with pytest.raises(WindowsHelperError, match="128 KiB"):
            helper.request("probe", {"text": "\N{ROCKET}" * MAX_FRAME_BYTES})
    finally:
        helper.close()


@pytest.mark.parametrize(
    ("frame", "message"),
    [
        (b"not-json\n", "JSON"),
        (
            json.dumps(
                {
                    "version": 2,
                    "kind": "status",
                    "session_id": SESSION_ID,
                    "state": "ready",
                    "helper_version": "1",
                    "source_sha256": SOURCE_DIGEST,
                }
            ).encode() + b"\n",
            "version",
        ),
        (
            json.dumps(
                {
                    "version": 1,
                    "kind": "status",
                    "session_id": SESSION_ID,
                    "state": "ready",
                    "helper_version": "1",
                    "source_sha256": SOURCE_DIGEST,
                    "extra": True,
                }
            ).encode() + b"\n",
            "fields",
        ),
    ],
)
def test_invalid_startup_frame_fails_connection(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    frame: bytes,
    message: str,
) -> None:
    command = [
        sys.executable,
        "-u",
        "-c",
        "import sys,time;sys.stdout.buffer.write(" + repr(frame) + ");sys.stdout.flush();time.sleep(1)",
    ]
    monkeypatch.setattr(helper_module, "_build_helper_command", lambda: command)
    monkeypatch.setattr(helper_module, "_packaged_source_digest", lambda: SOURCE_DIGEST)
    with pytest.raises(WindowsHelperError, match=message):
        WindowsHelper.start(session_id=SESSION_ID)


def test_startup_deadline_terminates_unresponsive_process(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    command = [sys.executable, "-u", "-c", "import time; time.sleep(1)"]
    monkeypatch.setattr(helper_module, "_build_helper_command", lambda: command)
    monkeypatch.setattr(helper_module, "_packaged_source_digest", lambda: SOURCE_DIGEST)
    monkeypatch.setattr(helper_module, "STARTUP_TIMEOUT_S", 0.02)
    with pytest.raises(WindowsHelperError, match="startup deadline"):
        WindowsHelper.start(session_id=SESSION_ID)


def test_execution_policy_failure_has_actionable_code(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    command = [
        sys.executable,
        "-u",
        "-c",
        "import sys; print('running scripts is disabled by execution policy', file=sys.stderr)",
    ]
    monkeypatch.setattr(helper_module, "_build_helper_command", lambda: command)
    monkeypatch.setattr(helper_module, "_packaged_source_digest", lambda: SOURCE_DIGEST)
    with pytest.raises(WindowsHelperError) as raised:
        WindowsHelper.start(session_id=SESSION_ID)
    assert raised.value.code == "helper_policy_blocked"


def test_oversized_and_partial_lines_fail_connection(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    command = [
        sys.executable,
        "-u",
        "-c",
        f"import sys;sys.stdout.buffer.write(b'x'*{MAX_FRAME_BYTES + 1});sys.stdout.flush()",
    ]
    monkeypatch.setattr(helper_module, "_build_helper_command", lambda: command)
    monkeypatch.setattr(helper_module, "_packaged_source_digest", lambda: SOURCE_DIGEST)
    with pytest.raises(WindowsHelperError, match="128 KiB"):
        WindowsHelper.start(session_id=SESSION_ID)

    ready = json.dumps(
        {
            "version": 1,
            "kind": "status",
            "session_id": SESSION_ID,
            "state": "ready",
            "helper_version": "1",
            "source_sha256": SOURCE_DIGEST,
        }
    )
    command = [
        sys.executable,
        "-u",
        "-c",
        f"import sys,time;print({ready!r},flush=True);sys.stdout.write('{{');sys.stdout.flush()",
    ]
    monkeypatch.setattr(helper_module, "_build_helper_command", lambda: command)
    helper = WindowsHelper.start(session_id=SESSION_ID)
    with pytest.raises(WindowsHelperError, match="partial"):
        helper.request("probe", {}, timeout_s=0.5)
    helper.close()


@pytest.mark.parametrize("fault", ["mismatch", "duplicate"])
def test_unknown_mismatched_or_duplicate_response_ids_disconnect(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, fault: str
) -> None:
    helper = _start(
        tmp_path,
        monkeypatch,
        f"""
        line = sys.stdin.readline()
        request = json.loads(line)
        response = {{"version": 1, "kind": "response", "session_id": session_id,
                    "request_id": request["request_id"], "ok": True, "result": {{}}}}
        if {fault!r} == "mismatch": response["request_id"] = "22222222-2222-4222-8222-222222222222"
        emit(response)
        if {fault!r} == "duplicate": emit(response)
        time.sleep(1)
        """,
    )
    try:
        if fault == "mismatch":
            with pytest.raises(WindowsHelperError, match="request ID"):
                helper.request("probe", {}, timeout_s=0.5)
        else:
            assert helper.request("probe", {}) == {}
            time.sleep(0.05)
            with pytest.raises(WindowsHelperError, match="request ID"):
                helper.request("probe", {})
    finally:
        helper.close()


def test_invalid_and_oversized_datagrams_disconnect(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    for encoded in ("%%%", base64.b64encode(b"x" * (MAX_DATAGRAM_BYTES + 1)).decode()):
        helper = _start(
            tmp_path,
            monkeypatch,
            f"""
            emit({{"version": 1, "kind": "datagram", "session_id": session_id,
                  "data_base64": {encoded!r}}})
            time.sleep(1)
            """,
        )
        with pytest.raises(WindowsHelperError, match="datagram"):
            helper.receive(0.5)
        helper.close()


def test_datagram_queue_saturation_disconnects_instead_of_dropping(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    helper = _start(
        tmp_path,
        monkeypatch,
        """
        frame = {"version": 1, "kind": "datagram", "session_id": session_id,
                 "data_base64": base64.b64encode(b'x').decode()}
        for _ in range(257): emit(frame)
        time.sleep(1)
        """,
    )
    time.sleep(0.1)
    with pytest.raises(WindowsHelperError, match="queue"):
        helper.receive(0.5)
    helper.close()


def test_binary_datagram_round_trip_and_size_guard(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    helper = _start(
        tmp_path,
        monkeypatch,
        """
        for line in sys.stdin:
            value = json.loads(line)
            if value["kind"] == "datagram":
                emit(value)
            elif value["operation"] == "shutdown":
                emit({"version": 1, "kind": "response", "session_id": session_id,
                      "request_id": value["request_id"], "ok": True, "result": {}})
                break
        """,
    )
    payload = b"\x00\xfe\xfdMAVLink\n\xff"
    helper.send(payload)
    assert helper.receive(1.0) == payload
    with pytest.raises(WindowsHelperError, match="65,507"):
        helper.send(b"x" * (MAX_DATAGRAM_BYTES + 1))
    helper.close()


def test_request_deadline_and_eof_are_actionable(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    helper = _start(tmp_path, monkeypatch, "sys.stdin.readline(); time.sleep(1)")
    with pytest.raises(WindowsHelperError, match="deadline"):
        helper.request("probe", {}, timeout_s=0.02)
    helper.close()

    helper = _start(tmp_path, monkeypatch, "pass")
    with pytest.raises(WindowsHelperError, match="exited|EOF"):
        helper.request("probe", {}, timeout_s=0.5)
    helper.close()


def test_close_is_idempotent_and_terminates_process(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    helper = _start(
        tmp_path,
        monkeypatch,
        """
        request = json.loads(sys.stdin.readline())
        emit({"version": 1, "kind": "response", "session_id": session_id,
              "request_id": request["request_id"], "ok": True, "result": {}})
        """,
    )
    process = helper._process
    helper.close()
    helper.close()
    assert process.poll() is not None


def test_restart_does_not_replay_buffered_datagrams(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    first = _start(
        tmp_path,
        monkeypatch,
        "emit({\"version\":1,\"kind\":\"datagram\",\"session_id\":session_id,\"data_base64\":\"eA==\"}); time.sleep(1)",
    )
    time.sleep(0.05)
    first.close()
    replacement = _start(tmp_path, monkeypatch, "time.sleep(1)")
    try:
        received = []
        value = replacement.receive(0.02)
        if value is not None:
            received.append(value)
        assert received == []
    finally:
        replacement.close()
