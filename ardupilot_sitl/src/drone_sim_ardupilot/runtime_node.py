"""Production ArduPilot SITL process wrapper."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import sys
from typing import Any

from .config import LaunchOrigin, RuntimeConfig, resolve_gazebo_address
from .runtime import (
    DiagnosticInventory,
    EventWriter,
    OutputFacts,
    SITLProcess,
    atomic_document,
)


_LAUNCH_ORIGIN_KEYS = {
    "latitude_deg",
    "longitude_deg",
    "amsl_m",
    "heading_deg",
}


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-standard JSON number {value}")


def _unique_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    document: dict[str, object] = {}
    for key, value in pairs:
        if key in document:
            raise ValueError(f"duplicate JSON key {key}")
        document[key] = value
    return document


def _decode_launch_origin(value: str) -> LaunchOrigin:
    try:
        value.encode("utf-8", errors="strict")
        document = json.loads(
            value,
            object_pairs_hook=_unique_json_object,
            parse_constant=_reject_json_constant,
        )
    except (AttributeError, TypeError, UnicodeEncodeError, json.JSONDecodeError) as error:
        raise ValueError("SIM_LAUNCH_ORIGIN_JSON must contain valid UTF-8 JSON") from error
    if not isinstance(document, dict) or set(document) != _LAUNCH_ORIGIN_KEYS:
        raise ValueError(
            "SIM_LAUNCH_ORIGIN_JSON must be an object with exactly "
            "latitude_deg, longitude_deg, amsl_m, and heading_deg"
        )
    try:
        return LaunchOrigin(**document)
    except (TypeError, ValueError) as error:
        raise ValueError("SIM_LAUNCH_ORIGIN_JSON contains an invalid launch origin") from error


def _control_matches(path: Path, run_id: str) -> bool:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, UnicodeDecodeError, json.JSONDecodeError):
        return False
    return isinstance(document, dict) and document.get("run_id") == run_id


def _diagnostic_paths(run_directory: Path, working_directory: Path) -> list[str]:
    return [
        str((working_directory / item).relative_to(run_directory))
        for item in DiagnosticInventory(working_directory).relative_paths()
    ]


def main() -> int:
    launch_origin = _decode_launch_origin(os.environ["SIM_LAUNCH_ORIGIN_JSON"])
    run_id = os.environ["SIM_RUN_ID"]
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"])
    gazebo_service = os.environ.get("SIM_GAZEBO_HOST", "gazebo-runtime")
    config = RuntimeConfig(
        run_id=run_id,
        run_directory=run_directory,
        launch_origin=launch_origin,
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
        ardupilot_revision="2a3dc4b7bf2507120f7378a7b2fde73185e0c325",
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
            facts.observe(line)
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
