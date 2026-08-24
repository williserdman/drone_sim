from __future__ import annotations

import io
import json
from pathlib import Path
import sys

from drone_sim_ardupilot.runtime import (
    DiagnosticInventory,
    EventWriter,
    OutputFacts,
    SITLProcess,
    atomic_document,
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


def test_atomic_document_writes_exact_readiness_evidence(tmp_path: Path) -> None:
    target = tmp_path / ".status/ardupilot-ready.json"
    document = {
        "run_id": RUN_ID,
        "ready": True,
        "json_exchange": True,
        "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
    }

    atomic_document(target, document)

    assert json.loads(target.read_text()) == document
    assert not list(target.parent.glob("*.tmp"))


def test_output_facts_require_json_exchange_and_mavlink_listener() -> None:
    facts = OutputFacts()
    facts.observe("Starting SITL: JSON")
    facts.observe("bind port 5760 for 0")
    assert not facts.ready

    facts.observe("JSON received:")
    assert facts.ready


def test_output_facts_latch_peer_loss_only_after_exchange() -> None:
    facts = OutputFacts()
    facts.observe("No JSON sensor message received, resending servos")
    assert not facts.peer_lost

    facts.observe("JSON received:")
    facts.observe("No JSON sensor message received, resending servos")
    assert facts.peer_lost


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
