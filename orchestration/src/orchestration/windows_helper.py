"""Bounded JSON-lines client for the packaged Windows host helper."""

from __future__ import annotations

import base64
import binascii
from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import threading
import time
from typing import IO, Mapping
from urllib.parse import urlparse
from uuid import UUID, uuid4

from .host_integration import HostHelperInfo


PROTOCOL_VERSION = 1
HELPER_VERSION = "1"
MAX_FRAME_BYTES = 128 * 1024
MAX_DATAGRAM_BYTES = 65_507
DATAGRAM_QUEUE_SIZE = 256
MAX_OUTSTANDING_REQUESTS = 32
STARTUP_TIMEOUT_S = 10.0
SHUTDOWN_GRACE_S = 5.0

_OPERATIONS = frozenset(
    {
        "probe",
        "inspect_qgc",
        "acquire_lease",
        "register_controller",
        "begin_run",
        "verify_resume",
        "finish_run",
        "publish_actions",
        "open_qgc",
        "open_browser",
        "open_transport",
        "close_transport",
        "release_lease",
        "shutdown",
    }
)


class WindowsHelperError(RuntimeError):
    """The helper process, protocol, or requested native operation failed."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(slots=True)
class _Pending:
    event: threading.Event
    result: Mapping[str, object] | None = None
    error: WindowsHelperError | None = None


def _canonical_uuid(value: str, label: str) -> str:
    try:
        parsed = UUID(value)
    except (ValueError, AttributeError) as error:
        raise ValueError(f"{label} must be a canonical UUID") from error
    if parsed.int == 0 or str(parsed) != value:
        raise ValueError(f"{label} must be a canonical lowercase nonzero UUID")
    return value


def _packaged_script() -> Path:
    return Path(__file__).with_name("windows") / "host.ps1"


def _packaged_source_digest() -> str:
    return hashlib.sha256(_packaged_script().read_bytes()).hexdigest()


def _build_helper_command() -> list[str]:
    script = _packaged_script().resolve()
    try:
        translated = subprocess.run(
            ["wslpath", "-w", str(script)],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10,
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError) as error:
        raise WindowsHelperError(
            "wsl_interop_unavailable",
            f"cannot convert the packaged helper path for Windows: {error}",
        ) from error
    if not translated:
        raise WindowsHelperError(
            "wsl_interop_unavailable",
            "wslpath returned no Windows path for the packaged helper",
        )
    return [
        "powershell.exe",
        "-NoLogo",
        "-NoProfile",
        "-NonInteractive",
        "-File",
        translated,
    ]


class WindowsHelper:
    """One helper subprocess with bounded control and datagram channels."""

    @classmethod
    def start(cls, *, session_id: str) -> WindowsHelper:
        canonical = _canonical_uuid(session_id, "helper session ID")
        command = [*_build_helper_command(), "-SessionId", canonical]
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except OSError as error:
            raise WindowsHelperError(
                "wsl_interop_unavailable",
                f"cannot start Windows PowerShell through WSL interoperability: {error}",
            ) from error
        helper = cls(process, canonical, _packaged_source_digest())
        if not helper._ready.wait(STARTUP_TIMEOUT_S):
            helper._fail(
                WindowsHelperError(
                    "helper_start_timeout",
                    "Windows helper did not become ready within the 10-second startup deadline",
                )
            )
        failure = helper._failure
        if failure is not None:
            helper._stop_process()
            helper._stderr_thread.join(timeout=0.2)
            if failure.code == "helper_eof":
                diagnostics = helper._diagnostic_text().lower()
                if any(
                    marker in diagnostics
                    for marker in (
                        "execution policy",
                        "running scripts is disabled",
                        "not digitally signed",
                        "authorizationmanager check failed",
                    )
                ):
                    raise WindowsHelperError(
                        "helper_policy_blocked",
                        "Windows PowerShell policy blocked the packaged helper; permit the "
                        "repository script without an execution-policy override",
                    ) from failure
            raise failure
        return helper

    def __init__(
        self,
        process: subprocess.Popen[bytes],
        session_id: str,
        expected_source_digest: str,
    ) -> None:
        if process.stdin is None or process.stdout is None or process.stderr is None:
            raise AssertionError("helper subprocess pipes were not created")
        self._process = process
        self._stdin: IO[bytes] = process.stdin
        self._stdout: IO[bytes] = process.stdout
        self._stderr: IO[bytes] = process.stderr
        self._session_id = session_id
        self._expected_source_digest = expected_source_digest
        self._info: HostHelperInfo | None = None
        self._ready = threading.Event()
        self._failure: WindowsHelperError | None = None
        self._failure_lock = threading.Lock()
        self._pending: dict[str, _Pending] = {}
        self._pending_lock = threading.Lock()
        self._control_output: queue.Queue[bytes] = queue.Queue(
            MAX_OUTSTANDING_REQUESTS + 1
        )
        self._datagram_output: queue.Queue[bytes] = queue.Queue(DATAGRAM_QUEUE_SIZE)
        self._datagram_input: queue.Queue[bytes] = queue.Queue(DATAGRAM_QUEUE_SIZE)
        self._writer_stop = threading.Event()
        self._closed = False
        self._close_lock = threading.Lock()
        self._diagnostics: deque[bytes] = deque(maxlen=64)
        self._reader_thread = threading.Thread(
            target=self._read_stdout, name="windows-helper-stdout", daemon=True
        )
        self._stderr_thread = threading.Thread(
            target=self._read_stderr, name="windows-helper-stderr", daemon=True
        )
        self._writer_thread = threading.Thread(
            target=self._write_stdin, name="windows-helper-stdin", daemon=True
        )
        self._reader_thread.start()
        self._stderr_thread.start()
        self._writer_thread.start()

    @property
    def info(self) -> HostHelperInfo:
        if self._info is None:
            raise WindowsHelperError(
                "helper_not_ready", "Windows helper startup identity is not available"
            )
        return self._info

    def request(
        self,
        operation: str,
        payload: Mapping[str, object],
        *,
        timeout_s: float = 10.0,
    ) -> Mapping[str, object]:
        if operation not in _OPERATIONS:
            raise ValueError(f"Windows helper operation {operation!r} is not allowed")
        if (
            not isinstance(timeout_s, (int, float))
            or isinstance(timeout_s, bool)
            or not math.isfinite(timeout_s)
            or timeout_s <= 0
        ):
            raise ValueError("helper request timeout must be a positive number")
        request_id = str(uuid4())
        envelope: dict[str, object] = {
            "version": PROTOCOL_VERSION,
            "kind": "request",
            "session_id": self._session_id,
            "request_id": request_id,
            "operation": operation,
            "payload": dict(payload),
        }
        frame = _encode_frame(envelope)
        _validate_request_payload(operation, payload)
        pending = _Pending(threading.Event())
        with self._pending_lock:
            self._raise_if_failed_or_closed()
            if len(self._pending) >= MAX_OUTSTANDING_REQUESTS:
                raise WindowsHelperError(
                    "helper_request_saturated",
                    "Windows helper already has 32 outstanding requests",
                )
            self._pending[request_id] = pending
        try:
            self._control_output.put_nowait(frame)
        except queue.Full as error:
            with self._pending_lock:
                self._pending.pop(request_id, None)
            failure = WindowsHelperError(
                "helper_control_queue_saturated",
                "Windows helper control output queue is saturated",
            )
            self._fail(failure)
            raise failure from error
        if not pending.event.wait(float(timeout_s)):
            with self._pending_lock:
                self._pending.pop(request_id, None)
            failure = WindowsHelperError(
                "helper_request_timeout",
                f"Windows helper {operation} request exceeded its {timeout_s:g}-second deadline",
            )
            self._fail(failure)
            raise failure
        if pending.error is not None:
            raise pending.error
        if pending.result is None:
            raise WindowsHelperError(
                "helper_protocol_error", "Windows helper response had no result"
            )
        return pending.result

    def send(self, payload: bytes) -> None:
        if not isinstance(payload, bytes):
            raise TypeError("datagram payload must be bytes")
        if len(payload) > MAX_DATAGRAM_BYTES:
            raise WindowsHelperError(
                "datagram_too_large", "Windows datagram payload exceeds 65,507 bytes"
            )
        self._raise_if_failed_or_closed()
        frame = _encode_frame(
            {
                "version": PROTOCOL_VERSION,
                "kind": "datagram",
                "session_id": self._session_id,
                "data_base64": base64.b64encode(payload).decode("ascii"),
            }
        )
        try:
            self._datagram_output.put_nowait(frame)
        except queue.Full as error:
            failure = WindowsHelperError(
                "helper_datagram_queue_saturated",
                "Windows helper outbound datagram queue is saturated; connection closed without dropping a datagram",
            )
            self._fail(failure)
            raise failure from error

    def receive(self, timeout_s: float) -> bytes | None:
        if (
            not isinstance(timeout_s, (int, float))
            or isinstance(timeout_s, bool)
            or not math.isfinite(timeout_s)
            or timeout_s < 0
        ):
            raise ValueError("datagram timeout must be a nonnegative number")
        deadline = time.monotonic() + float(timeout_s)
        while True:
            self._raise_if_failed_or_closed()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                return self._datagram_input.get(timeout=min(remaining, 0.05))
            except queue.Empty:
                continue

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            deadline = time.monotonic() + SHUTDOWN_GRACE_S
            if self._failure is None and self._ready.is_set() and self._process.poll() is None:
                try:
                    self.request("shutdown", {}, timeout_s=SHUTDOWN_GRACE_S)
                except WindowsHelperError:
                    pass
            self._closed = True
            self._writer_stop.set()
            try:
                self._stdin.close()
            except OSError:
                pass
            self._stop_process(max(0.0, deadline - time.monotonic()))

    def _stop_process(self, grace_s: float = SHUTDOWN_GRACE_S) -> None:
        if self._process.poll() is not None:
            return
        try:
            self._process.wait(timeout=grace_s)
            return
        except subprocess.TimeoutExpired:
            self._process.terminate()
        try:
            self._process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait(timeout=1.0)

    def _raise_if_failed_or_closed(self) -> None:
        if self._failure is not None:
            raise self._failure
        if self._closed:
            raise WindowsHelperError("helper_closed", "Windows helper connection is closed")

    def _fail(self, error: WindowsHelperError) -> None:
        with self._failure_lock:
            if self._failure is not None:
                return
            self._failure = error
            self._ready.set()
            self._writer_stop.set()
            with self._pending_lock:
                pending = tuple(self._pending.values())
                self._pending.clear()
            for item in pending:
                item.error = error
                item.event.set()
            try:
                self._stdin.close()
            except OSError:
                pass

    def _read_stdout(self) -> None:
        buffered = bytearray()
        try:
            while True:
                chunk = os.read(self._stdout.fileno(), 4096)
                if not chunk:
                    if buffered:
                        self._fail(
                            WindowsHelperError(
                                "helper_partial_frame",
                                "Windows helper exited with a partial protocol line",
                            )
                        )
                    else:
                        self._fail(
                            WindowsHelperError(
                                "helper_eof", "Windows helper process reached EOF"
                            )
                        )
                    return
                offset = 0
                while offset < len(chunk):
                    newline = chunk.find(b"\n", offset)
                    end = len(chunk) if newline < 0 else newline
                    segment = chunk[offset:end]
                    if len(buffered) + len(segment) > MAX_FRAME_BYTES:
                        raise WindowsHelperError(
                            "helper_frame_too_large",
                            "Windows helper protocol line exceeds 128 KiB",
                        )
                    buffered.extend(segment)
                    if newline < 0:
                        break
                    line = bytes(buffered)
                    buffered.clear()
                    if line.endswith(b"\r"):
                        line = line[:-1]
                    self._handle_frame(_decode_frame(line))
                    offset = newline + 1
        except WindowsHelperError as error:
            self._fail(error)
        except OSError as error:
            self._fail(
                WindowsHelperError(
                    "helper_pipe_failed", f"Windows helper stdout pipe failed: {error}"
                )
            )

    def _read_stderr(self) -> None:
        try:
            while chunk := os.read(self._stderr.fileno(), 4096):
                self._diagnostics.append(chunk)
        except OSError:
            return

    def _diagnostic_text(self) -> str:
        return b"".join(self._diagnostics).decode("utf-8", errors="replace")

    def _write_stdin(self) -> None:
        while not self._writer_stop.is_set():
            frame: bytes | None = None
            try:
                frame = self._control_output.get_nowait()
            except queue.Empty:
                try:
                    frame = self._datagram_output.get(timeout=0.02)
                except queue.Empty:
                    continue
            try:
                self._stdin.write(frame)
                self._stdin.flush()
            except (BrokenPipeError, OSError, ValueError) as error:
                self._fail(
                    WindowsHelperError(
                        "helper_pipe_failed", f"Windows helper stdin pipe failed: {error}"
                    )
                )
                return

    def _handle_frame(self, value: Mapping[str, object]) -> None:
        common = {"version", "kind", "session_id"}
        if value.get("version") != PROTOCOL_VERSION:
            raise WindowsHelperError(
                "helper_protocol_version", "Windows helper protocol version must be 1"
            )
        if value.get("session_id") != self._session_id:
            raise WindowsHelperError(
                "helper_session_mismatch", "Windows helper response session ID mismatched"
            )
        kind = value.get("kind")
        if kind == "status":
            expected = common | {"state", "helper_version", "source_sha256"}
            _require_fields(value, expected)
            if self._ready.is_set() or value["state"] != "ready":
                raise WindowsHelperError(
                    "helper_protocol_status", "Windows helper sent an invalid status frame"
                )
            if value["helper_version"] != HELPER_VERSION:
                raise WindowsHelperError(
                    "helper_version_mismatch",
                    "Windows helper version does not match the Python client",
                )
            if value["source_sha256"] != self._expected_source_digest:
                raise WindowsHelperError(
                    "helper_source_mismatch",
                    "Windows helper source digest does not match the packaged script",
                )
            self._info = HostHelperInfo(
                version=str(value["helper_version"]),
                source_sha256=str(value["source_sha256"]),
            )
            self._ready.set()
            return
        if not self._ready.is_set():
            raise WindowsHelperError(
                "helper_protocol_startup", "Windows helper sent data before its ready status"
            )
        if kind == "response":
            request_id = value.get("request_id")
            if not isinstance(request_id, str):
                raise WindowsHelperError(
                    "helper_protocol_response", "Windows helper response request ID is invalid"
                )
            ok = value.get("ok")
            if ok is True:
                _require_fields(value, common | {"request_id", "ok", "result"})
                result = value["result"]
                if not isinstance(result, dict):
                    raise WindowsHelperError(
                        "helper_protocol_response", "Windows helper result must be an object"
                    )
                response_error = None
            elif ok is False:
                _require_fields(value, common | {"request_id", "ok", "error"})
                error_value = value["error"]
                if not isinstance(error_value, dict) or set(error_value) != {
                    "code",
                    "message",
                }:
                    raise WindowsHelperError(
                        "helper_protocol_response", "Windows helper error result is invalid"
                    )
                code, message = error_value.get("code"), error_value.get("message")
                if (
                    not isinstance(code, str)
                    or not code
                    or not isinstance(message, str)
                    or not message
                ):
                    raise WindowsHelperError(
                        "helper_protocol_response", "Windows helper error result is invalid"
                    )
                result = None
                response_error = WindowsHelperError(code, message)
            else:
                raise WindowsHelperError(
                    "helper_protocol_response", "Windows helper response ok field must be boolean"
                )
            with self._pending_lock:
                pending = self._pending.pop(request_id, None)
            if pending is None:
                raise WindowsHelperError(
                    "helper_request_id_mismatch",
                    "Windows helper sent an unknown, mismatched, or duplicate request ID",
                )
            pending.result = result
            pending.error = response_error
            pending.event.set()
            return
        if kind == "datagram":
            _require_fields(value, common | {"data_base64"})
            encoded = value["data_base64"]
            if not isinstance(encoded, str):
                raise WindowsHelperError(
                    "helper_protocol_datagram", "Windows helper datagram data must be base64 text"
                )
            try:
                payload = base64.b64decode(encoded, validate=True)
            except (ValueError, binascii.Error) as error:
                raise WindowsHelperError(
                    "helper_protocol_datagram", "Windows helper datagram contains invalid base64"
                ) from error
            if len(payload) > MAX_DATAGRAM_BYTES:
                raise WindowsHelperError(
                    "helper_protocol_datagram", "Windows helper datagram exceeds 65,507 bytes"
                )
            try:
                self._datagram_input.put_nowait(payload)
            except queue.Full as error:
                raise WindowsHelperError(
                    "helper_datagram_queue_saturated",
                    "Windows helper inbound datagram queue is saturated; connection closed without dropping a datagram",
                ) from error
            return
        raise WindowsHelperError(
            "helper_protocol_kind", "Windows helper protocol kind is unknown"
        )


def _encode_frame(value: Mapping[str, object]) -> bytes:
    try:
        encoded = json.dumps(
            value, ensure_ascii=False, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise ValueError(f"helper request is not JSON serializable: {error}") from error
    if len(encoded) > MAX_FRAME_BYTES:
        raise WindowsHelperError(
            "helper_frame_too_large", "Windows helper protocol line exceeds 128 KiB"
        )
    return encoded + b"\n"


def _decode_frame(line: bytes) -> Mapping[str, object]:
    def unique_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise WindowsHelperError(
                    "helper_protocol_json", f"Windows helper JSON repeats field {key}"
                )
            result[key] = value
        return result

    try:
        text = line.decode("utf-8", errors="strict")
        value = json.loads(
            text,
            object_pairs_hook=unique_object,
            parse_constant=lambda token: (_ for _ in ()).throw(
                ValueError(f"invalid JSON constant {token}")
            ),
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        raise WindowsHelperError(
            "helper_protocol_json", "Windows helper protocol line is not valid UTF-8 JSON"
        ) from error
    if not isinstance(value, dict):
        raise WindowsHelperError(
            "helper_protocol_json", "Windows helper protocol line must contain an object"
        )
    return value


def _require_fields(value: Mapping[str, object], expected: set[str]) -> None:
    if set(value) != expected:
        raise WindowsHelperError(
            "helper_protocol_fields", "Windows helper protocol frame has unknown or missing fields"
        )


def _validate_request_payload(operation: str, payload: Mapping[str, object]) -> None:
    if not isinstance(payload, Mapping):
        raise ValueError("Windows helper payload must be a mapping")
    required: dict[str, dict[str, type | tuple[type, ...]]] = {
        "probe": {},
        "inspect_qgc": {"executable": str, "settings_file": str},
        "acquire_lease": {},
        "register_controller": {
            "installation_id": str,
            "ledger_id": str,
            "distro_name": str,
            "pristine": bool,
        },
        "begin_run": {"run_id": str},
        "verify_resume": {"run_id": str, "publication": Mapping},
        "finish_run": {"run_id": str, "verified_cleanup": bool},
        "publish_actions": {
            "directory": str,
            "filename": str,
            "data_base64": str,
            "sha256": str,
        },
        "open_qgc": {"installation": Mapping},
        "open_browser": {"url": str},
        "open_transport": {"run_id": str, "publication": Mapping},
        "close_transport": {},
        "release_lease": {},
        "shutdown": {},
    }
    schema = required[operation]
    if set(payload) != set(schema):
        raise ValueError(f"Windows helper {operation} payload has unknown or missing fields")
    for field, expected_type in schema.items():
        value = payload[field]
        if not isinstance(value, expected_type) or (
            expected_type is bool and type(value) is not bool
        ):
            raise ValueError(f"Windows helper {operation} payload field {field} has the wrong type")
    if operation == "register_controller":
        _canonical_uuid(str(payload["installation_id"]), "installation ID")
        _canonical_uuid(str(payload["ledger_id"]), "ledger ID")
        _nonempty_string(payload["distro_name"], "distro name")
    if operation in {"begin_run", "verify_resume", "finish_run", "open_transport"}:
        _canonical_uuid(str(payload["run_id"]), "run ID")
    if operation in {"verify_resume", "open_transport"}:
        _validate_publication(payload["publication"])
    if operation == "publish_actions":
        for field in ("directory", "filename", "data_base64", "sha256"):
            _nonempty_string(payload[field], field)
        _sha256(payload["sha256"], "publication SHA-256")
        try:
            base64.b64decode(str(payload["data_base64"]), validate=True)
        except (ValueError, binascii.Error) as error:
            raise ValueError("publication data_base64 must be valid base64") from error
    if operation == "open_qgc":
        _validate_installation(payload["installation"])
    if operation == "open_browser":
        selected = urlparse(str(payload["url"]))
        if selected.scheme not in {"http", "https"} or not selected.netloc:
            raise ValueError("browser URL must be absolute HTTP or HTTPS")


def _nonempty_string(value: object, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a nonempty string")
    return value


def _exact_mapping(
    value: object, fields: set[str], label: str
) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or set(value) != fields:
        raise ValueError(f"{label} has unknown or missing fields")
    return value


def _sha256(value: object, label: str) -> str:
    selected = _nonempty_string(value, label)
    if len(selected) != 64 or any(
        character not in "0123456789abcdef" for character in selected
    ):
        raise ValueError(f"{label} must be lowercase SHA-256 text")
    return selected


def _validate_publication(value: object) -> None:
    selected = _exact_mapping(
        value, {"native_path", "filename", "sha256"}, "publication"
    )
    _nonempty_string(selected["native_path"], "publication native path")
    _nonempty_string(selected["filename"], "publication filename")
    _sha256(selected["sha256"], "publication SHA-256")


def _validate_installation(value: object) -> None:
    selected = _exact_mapping(
        value,
        {"executable", "settings_file", "save_path", "version", "source"},
        "installation",
    )
    for field in ("executable", "settings_file", "save_path", "version"):
        _nonempty_string(selected[field], f"installation {field}")
    source = _exact_mapping(
        selected["source"], {"system_id", "component_id"}, "installation source"
    )
    if type(source["system_id"]) is not int or type(source["component_id"]) is not int:
        raise ValueError("installation source IDs must be integers")


__all__ = [
    "DATAGRAM_QUEUE_SIZE",
    "HELPER_VERSION",
    "MAX_DATAGRAM_BYTES",
    "MAX_FRAME_BYTES",
    "MAX_OUTSTANDING_REQUESTS",
    "WindowsHelper",
    "WindowsHelperError",
]
