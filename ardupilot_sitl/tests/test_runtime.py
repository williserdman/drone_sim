from __future__ import annotations

import io
import json
from pathlib import Path
import selectors
import subprocess
import sys

import pytest

from drone_sim_ardupilot.runtime import (
    DiagnosticInventory,
    EventWriter,
    OutputFacts,
    SITLProcess,
)


RUN_ID = "123e4567-e89b-42d3-a456-426614174000"


def test_event_writer_wraps_child_output_as_one_structured_module_event() -> None:
    stream = io.StringIO()
    writer = EventWriter(RUN_ID, stream)

    writer.emit("sitl_output", stream_name="stderr", line="JSON peer unavailable")

    event = json.loads(stream.getvalue())
    assert event["run_id"] == RUN_ID
    assert event["module"] == "ardupilot_sitl"
    assert event["event"] == "sitl_output"
    assert event["sim_timestamp"] is None
    assert event["fields"] == {"line": "JSON peer unavailable", "stream_name": "stderr"}
    assert event["wall_timestamp"].endswith("Z")


def test_diagnostic_inventory_preserves_dataflash_storage_and_failure(tmp_path: Path) -> None:
    (tmp_path / "logs").mkdir()
    (tmp_path / "logs/00000001.BIN").write_bytes(b"dataflash")
    (tmp_path / "eeprom.bin").write_bytes(b"parameters")
    (tmp_path / "failure.json").write_text("{}")
    (tmp_path / "ignore.txt").write_text("not owned evidence")

    assert DiagnosticInventory(tmp_path).relative_paths() == (
        "eeprom.bin",
        "failure.json",
        "logs/00000001.BIN",
    )


def test_output_facts_require_json_exchange_and_mavlink_listener() -> None:
    facts = OutputFacts()
    facts.observe("Starting SITL: JSON")
    facts.observe("bind port 5760 for 0")
    assert not facts.ready

    facts.observe("JSON received:")
    assert facts.ready


def test_output_facts_keep_readiness_after_json_resend_diagnostic() -> None:
    facts = OutputFacts()
    facts.observe("bind port 5760 for 0")
    facts.observe("JSON received:")
    facts.observe("No JSON sensor message received, resending servos")

    assert facts.ready


def test_sitl_process_captures_both_streams_and_stops_boundedly(tmp_path: Path) -> None:
    child = tmp_path / "child.py"
    child.write_text(
        "import sys,time\n"
        "print('JSON received:', flush=True)\n"
        "print('bind port 5760 for 0', file=sys.stderr, flush=True)\n"
        "time.sleep(60)\n"
    )
    process = SITLProcess((sys.executable, str(child)), tmp_path)
    process.start()

    observed = {process.read_line(1.0), process.read_line(1.0)}
    return_code = process.stop(2.0)

    assert observed == {
        ("stdout", "JSON received:"),
        ("stderr", "bind port 5760 for 0"),
    }
    assert return_code < 0
    assert process.read_line(0) is None


def test_sitl_process_stop_without_created_child_returns_none(tmp_path: Path) -> None:
    process = SITLProcess((sys.executable, "unused.py"), tmp_path)

    assert process.stop(0.1) is None


def test_sitl_process_reaps_child_and_closes_resources_when_second_selector_registration_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Child:
        stdout = io.StringIO()
        stderr = io.StringIO()
        returncode: int | None = None
        terminated = False
        waits = 0

        def poll(self) -> int | None:
            return self.returncode

        def terminate(self) -> None:
            self.terminated = True

        def wait(self, timeout: float) -> int:
            assert timeout == 10.0
            self.waits += 1
            self.returncode = -15
            return self.returncode

    child = Child()

    class FailedSelector:
        def __init__(self) -> None:
            self.registrations = 0
            self.closed = False

        def register(self, *_args: object) -> None:
            self.registrations += 1
            if self.registrations == 2:
                raise OSError("selector registration failed")

        def close(self) -> None:
            self.closed = True

        def select(self, _timeout: float) -> list[object]:
            if self.closed:
                raise ValueError("selector is closed")
            return []

    selector = FailedSelector()
    monkeypatch.setattr(subprocess, "Popen", lambda *_args, **_kwargs: child)
    monkeypatch.setattr(selectors, "DefaultSelector", lambda: selector)
    process = SITLProcess((sys.executable, "unused.py"), tmp_path)

    with pytest.raises(OSError, match="selector registration failed"):
        process.start()

    assert child.terminated
    assert child.waits == 1
    assert child.stdout.closed
    assert child.stderr.closed
    assert selector.registrations == 2
    assert selector.closed
    assert process.return_code == -15
    assert process.stop(0.1) == -15
    assert child.waits == 1
    assert process.read_line(0) is None
