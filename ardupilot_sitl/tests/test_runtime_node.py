from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import drone_sim_ardupilot.runtime_node as runtime_node


RUN_ID = "123e4567-e89b-42d3-a456-426614174000"


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


def test_main_allows_json_resend_diagnostic_during_slow_running_exchange(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    _write(
        tmp_path / ".status/runtime-running.json",
        {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": 0},
    )
    _install_fake_process(monkeypatch, tmp_path, finalize_after_missing=True)

    assert runtime_node.main() == 0
    assert not (tmp_path / ".status/runtime-failure.json").exists()


def test_main_allows_missing_json_during_readiness_pause(
    tmp_path: Path,
    monkeypatch: Any,
) -> None:
    _install_fake_process(monkeypatch, tmp_path, finalize_after_missing=True)

    assert runtime_node.main() == 0
    assert not (tmp_path / ".status/runtime-failure.json").exists()
    quiescence = json.loads(
        (tmp_path / ".status/quiescence/ardupilot_sitl.json").read_text()
    )
    assert quiescence == {
        "run_id": RUN_ID,
        "module": "ardupilot_sitl",
        "quiescent": True,
    }
