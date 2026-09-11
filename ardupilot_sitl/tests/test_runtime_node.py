from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import drone_sim_ardupilot.runtime_node as runtime_node
import pytest


RUN_ID = "123e4567-e89b-42d3-a456-426614174000"
LAUNCH_ORIGIN_JSON = json.dumps(
    {
        "latitude_deg": 37.4003371,
        "longitude_deg": -122.0800351,
        "amsl_m": 12.5,
        "heading_deg": 270,
    }
)


def _write(path: Path, document: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document), encoding="utf-8")


def _install_fake_process(
    monkeypatch: Any,
    run_directory: Path,
    *,
    finalize_after_missing: bool,
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
    monkeypatch.setenv("SIM_LAUNCH_ORIGIN_JSON", LAUNCH_ORIGIN_JSON)
    monkeypatch.setattr(runtime_node, "resolve_gazebo_address", lambda _host: "127.0.0.1")
    monkeypatch.setattr(runtime_node, "SITLProcess", FakeProcess)
    return process_constructions


def test_main_starts_exact_firmware_with_native_tcp_argv_and_resolved_gazebo_address(
    tmp_path: Path,
    monkeypatch: Any,
    capsys: Any,
) -> None:
    process_constructions = _install_fake_process(
        monkeypatch, tmp_path, finalize_after_missing=True
    )

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
