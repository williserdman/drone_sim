from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from artifacts.runtime_status import ArduPilotReadyStatus, RuntimeFailureStatus
import drone_sim_ardupilot.runtime_node as runtime_node


RUN_ID = "123e4567-e89b-42d3-a456-426614174000"


class RecordingProtocol:
    instances: list["RecordingProtocol"] = []

    def __init__(self, run_directory: Path, run_id: str) -> None:
        self.run_directory = run_directory
        self.run_id = run_id
        self.statuses: list[object] = []
        self.quiescent_modules: list[str] = []
        self.closed = False
        self.instances.append(self)

    def write_status(self, status: object) -> None:
        self.statuses.append(status)

    def write_quiescence(self, module: str) -> None:
        self.quiescent_modules.append(module)

    def close(self) -> None:
        self.closed = True


def _write(path: Path, document: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document), encoding="utf-8")


def _install_fake_process(
    monkeypatch: Any,
    run_directory: Path,
    *,
    finalize_after_missing: bool,
) -> None:
    class FakeProcess:
        def __init__(self, _command: tuple[str, ...], _work: Path) -> None:
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
            if finalize_after_missing and self._read_count == 3:
                _write(
                    run_directory / ".control/finalize-request.json",
                    {
                        "run_id": RUN_ID,
                        "requested_terminal": "COMPLETED",
                        "reason": "test shutdown after diagnostic",
                    },
                )
            return None

        def stop(self, _timeout_seconds: float) -> int:
            return -15

    monkeypatch.setenv("SIM_RUN_ID", RUN_ID)
    monkeypatch.setenv("SIM_RUN_DIRECTORY", str(run_directory))
    monkeypatch.setattr(runtime_node, "resolve_gazebo_address", lambda _host: "127.0.0.1")
    monkeypatch.setattr(runtime_node, "SITLProcess", FakeProcess)


def _install_recording_protocol(monkeypatch: Any) -> None:
    RecordingProtocol.instances = []
    monkeypatch.setattr(runtime_node, "RuntimeProtocol", RecordingProtocol)


def test_main_publishes_ready_and_quiescence_through_one_closed_protocol(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    _install_fake_process(monkeypatch, tmp_path, finalize_after_missing=True)
    _install_recording_protocol(monkeypatch)

    assert runtime_node.main() == 0

    assert len(RecordingProtocol.instances) == 1
    protocol = RecordingProtocol.instances[0]
    assert protocol.run_directory == tmp_path
    assert protocol.run_id == RUN_ID
    assert protocol.statuses == [ArduPilotReadyStatus(RUN_ID)]
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
    _install_fake_process(monkeypatch, tmp_path, finalize_after_missing=True)
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
    _install_fake_process(monkeypatch, tmp_path, finalize_after_missing=True)
    _install_recording_protocol(monkeypatch)

    assert runtime_node.main() == 0
    protocol = RecordingProtocol.instances[0]
    assert not any(
        isinstance(status, RuntimeFailureStatus) for status in protocol.statuses
    )
    assert protocol.quiescent_modules == ["ardupilot_sitl"]
