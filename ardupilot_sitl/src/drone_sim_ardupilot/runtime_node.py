"""Production ArduPilot SITL process wrapper."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import sys
from typing import Any

from .config import RuntimeConfig, resolve_gazebo_address
from .runtime import (
    DiagnosticInventory,
    DurableLifecycle,
    EventWriter,
    OutputFacts,
    SITLProcess,
    atomic_document,
    json_peer_loss_is_fatal,
)


_TERMINAL_STATES = frozenset({"COMPLETED", "FAILED", "ABORTED"})


def _control_matches(path: Path, run_id: str) -> bool:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, UnicodeDecodeError, json.JSONDecodeError):
        return False
    return isinstance(document, dict) and document.get("run_id") == run_id


def _read_document(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None


def _nonnegative_integer(value: Any) -> bool:
    return not isinstance(value, bool) and isinstance(value, int) and value >= 0


def _running_matches(document: Any, run_id: str) -> bool:
    return (
        isinstance(document, dict)
        and set(document) == {"run_id", "state", "sim_timestamp_ns"}
        and document["run_id"] == run_id
        and document["state"] == "RUNNING"
        and _nonnegative_integer(document["sim_timestamp_ns"])
    )


def _source_finished_matches(document: Any, run_id: str) -> bool:
    return (
        isinstance(document, dict)
        and set(document) == {"run_id", "finished", "sim_timestamp_ns"}
        and document["run_id"] == run_id
        and document["finished"] is True
        and _nonnegative_integer(document["sim_timestamp_ns"])
    )


def _finalize_matches(document: Any, run_id: str) -> bool:
    return (
        isinstance(document, dict)
        and set(document) == {"run_id", "requested_terminal", "reason"}
        and document["run_id"] == run_id
        and document["requested_terminal"] in _TERMINAL_STATES
        and isinstance(document["reason"], str)
        and bool(document["reason"])
    )


def read_durable_lifecycle(
    run_directory: Path,
    run_id: str,
    *,
    requested_stop: bool = False,
) -> DurableLifecycle:
    return DurableLifecycle(
        running=_running_matches(
            _read_document(run_directory / ".status/runtime-running.json"), run_id
        ),
        source_finished=_source_finished_matches(
            _read_document(run_directory / ".status/source-finished.json"), run_id
        ),
        finalize_started=requested_stop
        or _finalize_matches(
            _read_document(run_directory / ".control/finalize-request.json"), run_id
        ),
    )


def _diagnostic_paths(run_directory: Path, working_directory: Path) -> list[str]:
    return [
        str((working_directory / item).relative_to(run_directory))
        for item in DiagnosticInventory(working_directory).relative_paths()
    ]


def main() -> int:
    run_id = os.environ["SIM_RUN_ID"]
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"])
    gazebo_service = os.environ.get("SIM_GAZEBO_HOST", "gazebo-runtime")
    config = RuntimeConfig(
        run_id=run_id,
        run_directory=run_directory,
        gazebo_host=resolve_gazebo_address(gazebo_service),
    )
    work = run_directory / "ardupilot_sitl"
    work.mkdir(parents=True, exist_ok=True)
    writer = EventWriter(run_id, sys.stdout)
    process = SITLProcess(config.argv, work)
    facts = OutputFacts()
    requested_stop = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal requested_stop
        requested_stop = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    writer.emit(
        "starting",
        ardupilot_revision="1511f27194f1dcc3728270883047bdf022b3fd53",
        gazebo_endpoint=f"udp://{gazebo_service}:{config.gazebo_port}",
        gazebo_resolved_address=config.gazebo_host,
        mavlink_endpoint=f"tcp://ardupilot-sitl:{config.mavlink_port}",
    )
    process.start()
    failure_reason: str | None = None
    ready_written = False
    finalize_request = run_directory / ".control/finalize-request.json"

    while True:
        record = process.read_line(0.25)
        if record is not None:
            stream_name, line = record
            writer.emit("sitl_output", stream_name=stream_name, line=line)
            missing_json_after_exchange = facts.observe(line)
            if missing_json_after_exchange and json_peer_loss_is_fatal(
                missing_json_after_exchange=missing_json_after_exchange,
                lifecycle=read_durable_lifecycle(
                    run_directory,
                    run_id,
                    requested_stop=requested_stop,
                ),
            ):
                failure_reason = "Gazebo JSON peer stopped advancing after exchange began"
                break
            if facts.ready and not ready_written:
                atomic_document(
                    run_directory / ".status/ardupilot-ready.json",
                    {
                        "run_id": run_id,
                        "ready": True,
                        "json_exchange": True,
                        "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
                    },
                )
                writer.emit("ready", json_exchange=True, mavlink_listening=True)
                ready_written = True
        return_code = process.return_code
        if return_code is not None:
            failure_reason = f"ArduCopter exited unexpectedly with status {return_code}"
            break
        if requested_stop or _control_matches(finalize_request, run_id):
            break

    return_code = process.stop(10.0)
    if failure_reason is not None:
        failure = {"run_id": run_id, "reason": failure_reason, "return_code": return_code}
        atomic_document(work / "failure.json", failure)
        diagnostics = _diagnostic_paths(run_directory, work)
        atomic_document(
            run_directory / ".status/runtime-failure.json",
            {
                "run_id": run_id,
                "module": "ardupilot_sitl",
                "reason": failure_reason,
                "diagnostic_paths": diagnostics,
            },
        )
        writer.emit("failed", severity="ERROR", reason=failure_reason, diagnostic_paths=diagnostics)
    else:
        writer.emit(
            "stopped",
            return_code=return_code,
            diagnostic_paths=_diagnostic_paths(run_directory, work),
        )

    atomic_document(
        run_directory / ".status/quiescence/ardupilot_sitl.json",
        {"run_id": run_id, "module": "ardupilot_sitl", "quiescent": True},
    )
    return 1 if failure_reason is not None else 0


if __name__ == "__main__":
    raise SystemExit(main())
