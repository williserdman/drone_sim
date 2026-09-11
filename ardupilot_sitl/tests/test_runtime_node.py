from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import pytest
from artifacts.runtime_protocol import ProtocolError, RuntimeProtocol
from artifacts.runtime_status import ArduPilotReadyStatus, RuntimeFailureStatus
import drone_sim_ardupilot.runtime_node as runtime_node


RUN_ID = "123e4567-e89b-42d3-a456-426614174000"
LAUNCH_ORIGIN_JSON = json.dumps(
    {
        "latitude_deg": 37.4003371,
        "longitude_deg": -122.0800351,
        "amsl_m": 12.5,
        "heading_deg": 270,
    }
)


class RecordingProtocol:
    instances: list["RecordingProtocol"] = []

    def __init__(self, run_directory: Path, run_id: str) -> None:
        self.run_directory = run_directory
        self.run_id = run_id
        self.statuses: list[object] = []
        self.quiescent_modules: list[str] = []
        self.finalize_reads = 0
        self.closed = False
        self.instances.append(self)

    def write_status(self, status: object) -> None:
        self.statuses.append(status)

    def write_quiescence(self, module: str) -> None:
        self.quiescent_modules.append(module)

    def read_finalize_request(self) -> dict[str, object] | None:
        self.finalize_reads += 1
        if self.finalize_reads < 3:
            return None
        return {
            "run_id": RUN_ID,
            "requested_terminal": "COMPLETED",
            "reason": "test shutdown after diagnostic",
        }

    def close(self) -> None:
        self.closed = True


def _write(path: Path, document: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document), encoding="utf-8")


def _install_fake_process(
    monkeypatch: Any,
    run_directory: Path,
) -> list[tuple[tuple[str, ...], Path]]:
    process_constructions: list[tuple[tuple[str, ...], Path]] = []
    class FakeProcess:
        def __init__(self, command: tuple[str, ...], work: Path) -> None:
            process_constructions.append((command, work))
            self._read_count = 0
            self._records = iter(
                (
                    ("stderr", "bind port 5760 for 0"),
                    ("stdout", "JSON received:"),
                    (
                        "stdout",
                        "No JSON sensor message received, resending servos",
                    ),
                )
            )

        def start(self) -> None:
            pass

        def read_line(self, _timeout_seconds: float) -> tuple[str, str] | None:
            self._read_count += 1
            return next(self._records)

        @property
        def return_code(self) -> None:
            return None

        def stop(self, _timeout_seconds: float) -> int:
            return -15

    monkeypatch.setenv("SIM_RUN_ID", RUN_ID)
    monkeypatch.setenv("SIM_RUN_DIRECTORY", str(run_directory))
    monkeypatch.setenv("SIM_LAUNCH_ORIGIN_JSON", LAUNCH_ORIGIN_JSON)
    monkeypatch.setattr(runtime_node, "resolve_gazebo_address", lambda _host: "127.0.0.1")
    monkeypatch.setattr(runtime_node, "SITLProcess", FakeProcess)
    return process_constructions


def test_main_starts_exact_firmware_with_native_tcp_argv_and_resolved_gazebo_address(
    tmp_path: Path,
    monkeypatch: Any,
    capsys: Any,
) -> None:
    process_constructions = _install_fake_process(monkeypatch, tmp_path)
    _install_recording_protocol(monkeypatch)

    assert runtime_node.main() == 0
    command, work = process_constructions[0]
    assert command == (
        "/opt/ardupilot/bin/arducopter",
        "--model",
        "JSON",
        "--speedup",
        "1",
        "--sim-address",
        "127.0.0.1",
        "--sim-port-in",
        "9003",
        "--sim-port-out",
        "9002",
        "--serial0",
        "tcp:5760",
        "--defaults",
        "/opt/drone_sim/ardupilot/params/descent.parm",
        "--home",
        "37.4003371,-122.0800351,12.5,270",
        "--wipe",
    )
    assert work == tmp_path / "ardupilot_sitl"
    events = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    starting = next(event for event in events if event["event"] == "starting")
    assert (
        starting["fields"]["ardupilot_revision"]
        == "2a3dc4b7bf2507120f7378a7b2fde73185e0c325"
    )
    assert starting["fields"]["gazebo_resolved_address"] == "127.0.0.1"


@pytest.mark.parametrize(
    "launch_origin_json",
    [
        None,
        "",
        "{",
        "[]",
        '{"latitude_deg":0,"longitude_deg":0,"amsl_m":0}',
        '{"latitude_deg":0,"longitude_deg":0,"amsl_m":0,"heading_deg":0,"extra":0}',
        '{"latitude_deg":true,"longitude_deg":0,"amsl_m":0,"heading_deg":0}',
        '{"latitude_deg":0,"longitude_deg":0,"amsl_m":"0","heading_deg":0}',
        '{"latitude_deg":NaN,"longitude_deg":0,"amsl_m":0,"heading_deg":0}',
        '{"latitude_deg":0,"longitude_deg":Infinity,"amsl_m":0,"heading_deg":0}',
        '{"latitude_deg":91,"longitude_deg":0,"amsl_m":0,"heading_deg":0}',
        '{"latitude_deg":0,"longitude_deg":-181,"amsl_m":0,"heading_deg":0}',
        '{"latitude_deg":0,"longitude_deg":0,"amsl_m":0,"heading_deg":360}',
        '{"latitude_deg":0,"latitude_deg":1,"longitude_deg":0,"amsl_m":0,"heading_deg":0}',
    ],
)
def test_main_rejects_invalid_launch_origin_before_startup_side_effects(
    launch_origin_json: str | None,
    tmp_path: Path,
    monkeypatch: Any,
    capsys: Any,
) -> None:
    run_directory = tmp_path / "run"
    process_constructions: list[tuple[tuple[str, ...], Path]] = []
    resolver_calls: list[str] = []

    class ForbiddenProcess:
        def __init__(self, command: tuple[str, ...], work: Path) -> None:
            process_constructions.append((command, work))

    monkeypatch.setenv("SIM_RUN_ID", RUN_ID)
    monkeypatch.setenv("SIM_RUN_DIRECTORY", str(run_directory))
    if launch_origin_json is None:
        monkeypatch.delenv("SIM_LAUNCH_ORIGIN_JSON", raising=False)
    else:
        monkeypatch.setenv("SIM_LAUNCH_ORIGIN_JSON", launch_origin_json)
    monkeypatch.setattr(runtime_node, "SITLProcess", ForbiddenProcess)
    monkeypatch.setattr(
        runtime_node,
        "resolve_gazebo_address",
        lambda host: resolver_calls.append(host) or "127.0.0.1",
    )

    with pytest.raises((KeyError, ValueError)):
        runtime_node.main()

    assert process_constructions == []
    assert resolver_calls == []
    assert not run_directory.exists()
    assert capsys.readouterr().out == ""


def _install_recording_protocol(monkeypatch: Any) -> None:
    RecordingProtocol.instances = []
    monkeypatch.setattr(runtime_node, "RuntimeProtocol", RecordingProtocol)


def test_main_publishes_ready_and_quiescence_through_one_closed_protocol(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    _install_fake_process(monkeypatch, tmp_path)
    _install_recording_protocol(monkeypatch)

    assert runtime_node.main() == 0

    assert len(RecordingProtocol.instances) == 1
    protocol = RecordingProtocol.instances[0]
    assert protocol.run_directory == tmp_path
    assert protocol.run_id == RUN_ID
    assert protocol.statuses == [ArduPilotReadyStatus(RUN_ID)]
    assert protocol.finalize_reads == 3
    assert protocol.quiescent_modules == ["ardupilot_sitl"]
    assert protocol.closed


def test_main_preserves_private_failure_and_publishes_shared_failure(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    class FailedProcess:
        def __init__(self, _command: tuple[str, ...], _work: Path) -> None:
            pass

        def start(self) -> None:
            pass

        def read_line(self, _timeout_seconds: float) -> None:
            return None

        @property
        def return_code(self) -> int:
            return 17

        def stop(self, _timeout_seconds: float) -> int:
            return 17

    monkeypatch.setenv("SIM_RUN_ID", RUN_ID)
    monkeypatch.setenv("SIM_RUN_DIRECTORY", str(tmp_path))
    monkeypatch.setenv("SIM_LAUNCH_ORIGIN_JSON", LAUNCH_ORIGIN_JSON)
    monkeypatch.setattr(runtime_node, "resolve_gazebo_address", lambda _host: "127.0.0.1")
    monkeypatch.setattr(runtime_node, "SITLProcess", FailedProcess)
    _install_recording_protocol(monkeypatch)

    assert runtime_node.main() == 1

    private_failure = json.loads((tmp_path / "ardupilot_sitl/failure.json").read_text())
    assert private_failure == {
        "run_id": RUN_ID,
        "reason": "ArduCopter exited unexpectedly with status 17",
        "return_code": 17,
    }
    protocol = RecordingProtocol.instances[0]
    assert protocol.statuses == [
        RuntimeFailureStatus(
            RUN_ID,
            "ardupilot_sitl",
            "ArduCopter exited unexpectedly with status 17",
            ("ardupilot_sitl/failure.json",),
        )
    ]
    assert protocol.quiescent_modules == ["ardupilot_sitl"]
    assert protocol.closed


def test_main_allows_json_resend_diagnostic_during_slow_running_exchange(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    _write(
        tmp_path / ".status/runtime-running.json",
        {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": 0},
    )
    _install_fake_process(monkeypatch, tmp_path)
    _install_recording_protocol(monkeypatch)

    assert runtime_node.main() == 0
    assert not any(
        isinstance(status, RuntimeFailureStatus)
        for status in RecordingProtocol.instances[0].statuses
    )


def test_main_allows_missing_json_during_readiness_pause(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    _install_fake_process(monkeypatch, tmp_path)
    _install_recording_protocol(monkeypatch)

    assert runtime_node.main() == 0
    protocol = RecordingProtocol.instances[0]
    assert not any(
        isinstance(status, RuntimeFailureStatus) for status in protocol.statuses
    )
    assert protocol.quiescent_modules == ["ardupilot_sitl"]


def test_main_uses_strict_finalize_reader_and_cleans_up_before_reraising(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    trace: list[str] = []
    (tmp_path / ".control").mkdir()
    (tmp_path / ".status/quiescence").mkdir(parents=True)
    _write(tmp_path / ".control/finalize-request.json", {"run_id": RUN_ID})

    class TracedProtocol(RuntimeProtocol):
        def read_finalize_request(self) -> dict[str, object] | None:
            trace.append("finalize")
            return super().read_finalize_request()

        def write_status(self, status: object) -> Path:
            trace.append("shared_failure")
            return super().write_status(status)  # type: ignore[arg-type]

        def write_quiescence(self, module: str) -> Path:
            trace.append("quiescence")
            return super().write_quiescence(module)

        def close(self) -> None:
            trace.append("close")
            super().close()

    class Process:
        def __init__(self, _command: tuple[str, ...], _work: Path) -> None:
            pass

        def start(self) -> None:
            pass

        def read_line(self, _timeout_seconds: float) -> None:
            return None

        @property
        def return_code(self) -> None:
            return None

        def stop(self, _timeout_seconds: float) -> int:
            trace.append("stop")
            return -15

    class Writer:
        def __init__(self, _run_id: str, _stream: object) -> None:
            pass

        def emit(self, event: str, **_fields: object) -> None:
            if event == "failed":
                trace.append("terminal")

    real_atomic_document = runtime_node.atomic_document

    def atomic_document(path: Path, document: dict[str, object]) -> None:
        trace.append("private_failure")
        real_atomic_document(path, document)

    def diagnostic_paths(_run_directory: Path, _work: Path) -> list[str]:
        trace.append("inventory")
        return ["ardupilot_sitl/failure.json"]

    monkeypatch.setenv("SIM_RUN_ID", RUN_ID)
    monkeypatch.setenv("SIM_RUN_DIRECTORY", str(tmp_path))
    monkeypatch.setenv("SIM_LAUNCH_ORIGIN_JSON", LAUNCH_ORIGIN_JSON)
    monkeypatch.setattr(runtime_node, "resolve_gazebo_address", lambda _host: "127.0.0.1")
    monkeypatch.setattr(runtime_node, "RuntimeProtocol", TracedProtocol)
    monkeypatch.setattr(runtime_node, "SITLProcess", Process)
    monkeypatch.setattr(runtime_node, "EventWriter", Writer)
    monkeypatch.setattr(runtime_node, "atomic_document", atomic_document)
    monkeypatch.setattr(runtime_node, "_diagnostic_paths", diagnostic_paths)

    with pytest.raises(ProtocolError) as raised:
        runtime_node.main()

    assert type(raised.value) is ProtocolError
    assert trace == [
        "finalize",
        "stop",
        "private_failure",
        "inventory",
        "terminal",
        "shared_failure",
        "quiescence",
        "close",
    ]


@pytest.mark.parametrize(
    ("failure_phase", "exit_before_cleanup", "expected_trace"),
    [
        (
            "read_line",
            False,
            ["stop", "private_failure", "inventory", "terminal", "shared_failure", "quiescence", "close"],
        ),
        (
            "readiness_status",
            False,
            ["stop", "private_failure", "inventory", "terminal", "shared_failure", "quiescence", "close"],
        ),
        (
            "finalize",
            False,
            ["stop", "private_failure", "inventory", "terminal", "shared_failure", "quiescence", "close"],
        ),
        (
            "stop",
            False,
            ["stop", "private_failure", "inventory", "terminal", "shared_failure", "close"],
        ),
        (
            "private_failure",
            True,
            ["stop", "private_failure", "inventory", "terminal", "shared_failure", "quiescence", "close"],
        ),
        (
            "inventory",
            True,
            ["stop", "private_failure", "inventory", "terminal", "shared_failure", "quiescence", "close"],
        ),
        (
            "terminal",
            True,
            ["stop", "private_failure", "inventory", "terminal", "shared_failure", "quiescence", "close"],
        ),
        (
            "shared_failure",
            True,
            ["stop", "private_failure", "inventory", "terminal", "shared_failure", "quiescence", "close"],
        ),
    ],
)
def test_main_attempts_later_cleanup_phases_after_each_failure(
    tmp_path: Path,
    monkeypatch: Any,
    failure_phase: str,
    exit_before_cleanup: bool,
    expected_trace: list[str],
) -> None:
    trace: list[str] = []
    injected = RuntimeError(f"{failure_phase} failed")

    class Process:
        def __init__(self, _command: tuple[str, ...], _work: Path) -> None:
            self.reads = 0

        def start(self) -> None:
            pass

        def read_line(self, _timeout_seconds: float) -> tuple[str, str] | None:
            self.reads += 1
            if failure_phase == "read_line":
                raise injected
            if failure_phase == "readiness_status":
                return (
                    ("stdout", "JSON received:")
                    if self.reads == 1
                    else ("stderr", "bind port 5760 for 0")
                )
            return None

        @property
        def return_code(self) -> int | None:
            return 17 if exit_before_cleanup else None

        def stop(self, _timeout_seconds: float) -> int:
            trace.append("stop")
            if failure_phase == "stop":
                raise injected
            return 17 if exit_before_cleanup else -15

    class Protocol:
        def __init__(self, _run_directory: Path, _run_id: str) -> None:
            self.finalize_reads = 0

        def read_finalize_request(self) -> dict[str, object] | None:
            self.finalize_reads += 1
            if failure_phase == "finalize":
                raise injected
            if failure_phase == "readiness_status" and self.finalize_reads == 1:
                return None
            return {
                "run_id": RUN_ID,
                "requested_terminal": "COMPLETED",
                "reason": "done",
            }

        def write_status(self, status: object) -> None:
            if isinstance(status, ArduPilotReadyStatus):
                if failure_phase == "readiness_status":
                    raise injected
                return
            trace.append("shared_failure")
            if failure_phase == "shared_failure":
                raise injected

        def write_quiescence(self, _module: str) -> None:
            trace.append("quiescence")

        def close(self) -> None:
            trace.append("close")

    class Writer:
        def __init__(self, _run_id: str, _stream: object) -> None:
            pass

        def emit(self, event: str, **_fields: object) -> None:
            if event in {"failed", "stopped"}:
                trace.append("terminal")
                if failure_phase == "terminal":
                    raise injected

    def atomic_document(_path: Path, _document: dict[str, object]) -> None:
        trace.append("private_failure")
        if failure_phase == "private_failure":
            raise injected

    def diagnostic_paths(_run_directory: Path, _work: Path) -> list[str]:
        trace.append("inventory")
        if failure_phase == "inventory":
            raise injected
        return ["ardupilot_sitl/failure.json"]

    monkeypatch.setenv("SIM_RUN_ID", RUN_ID)
    monkeypatch.setenv("SIM_RUN_DIRECTORY", str(tmp_path))
    monkeypatch.setenv("SIM_LAUNCH_ORIGIN_JSON", LAUNCH_ORIGIN_JSON)
    monkeypatch.setattr(runtime_node, "resolve_gazebo_address", lambda _host: "127.0.0.1")
    monkeypatch.setattr(runtime_node, "SITLProcess", Process)
    monkeypatch.setattr(runtime_node, "RuntimeProtocol", Protocol)
    monkeypatch.setattr(runtime_node, "EventWriter", Writer)
    monkeypatch.setattr(runtime_node, "atomic_document", atomic_document)
    monkeypatch.setattr(runtime_node, "_diagnostic_paths", diagnostic_paths)

    with pytest.raises(RuntimeError) as raised:
        runtime_node.main()

    assert raised.value is injected
    assert trace == expected_trace
    assert trace.count("stop") == 1


def test_main_preserves_monitoring_error_and_notes_cleanup_errors(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    monitoring_error = RuntimeError("monitoring failed")
    cleanup_error = RuntimeError("stop failed")
    trace: list[str] = []
    private_failures: list[dict[str, object]] = []
    terminal_reasons: list[object] = []
    shared_reasons: list[str] = []

    class Process:
        def __init__(self, _command: tuple[str, ...], _work: Path) -> None:
            pass

        def start(self) -> None:
            pass

        def read_line(self, _timeout_seconds: float) -> None:
            raise monitoring_error

        @property
        def return_code(self) -> None:
            return None

        def stop(self, _timeout_seconds: float) -> int:
            trace.append("stop")
            raise cleanup_error

    class Protocol:
        def __init__(self, _run_directory: Path, _run_id: str) -> None:
            pass

        def write_status(self, status: object) -> None:
            trace.append("shared_failure")
            assert isinstance(status, RuntimeFailureStatus)
            shared_reasons.append(status.reason)

        def write_quiescence(self, _module: str) -> None:
            trace.append("quiescence")

        def close(self) -> None:
            trace.append("close")

    class Writer:
        def __init__(self, _run_id: str, _stream: object) -> None:
            pass

        def emit(self, event: str, **_fields: object) -> None:
            if event == "failed":
                trace.append("terminal")
                terminal_reasons.append(_fields["reason"])

    def atomic_document(_path: Path, document: dict[str, object]) -> None:
        trace.append("private_failure")
        private_failures.append(document)

    monkeypatch.setenv("SIM_RUN_ID", RUN_ID)
    monkeypatch.setenv("SIM_RUN_DIRECTORY", str(tmp_path))
    monkeypatch.setenv("SIM_LAUNCH_ORIGIN_JSON", LAUNCH_ORIGIN_JSON)
    monkeypatch.setattr(runtime_node, "resolve_gazebo_address", lambda _host: "127.0.0.1")
    monkeypatch.setattr(runtime_node, "SITLProcess", Process)
    monkeypatch.setattr(runtime_node, "RuntimeProtocol", Protocol)
    monkeypatch.setattr(runtime_node, "EventWriter", Writer)
    monkeypatch.setattr(runtime_node, "atomic_document", atomic_document)
    monkeypatch.setattr(
        runtime_node,
        "_diagnostic_paths",
        lambda *_args: trace.append("inventory") or [],
    )

    with pytest.raises(RuntimeError) as raised:
        runtime_node.main()

    assert raised.value is monitoring_error
    assert raised.value.__notes__ == ["cleanup failure during stop/reap: stop failed"]
    assert [document["reason"] for document in private_failures] == ["monitoring failed"]
    assert terminal_reasons == ["monitoring failed"]
    assert shared_reasons == ["monitoring failed"]
    assert trace == [
        "stop",
        "private_failure",
        "inventory",
        "terminal",
        "shared_failure",
        "close",
    ]
