#!/usr/bin/env python3
"""Move the stationary landing target after independently verified acquisition."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping
from dataclasses import asdict
from dataclasses import dataclass
import csv
import fcntl
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import time
from typing import Any

from artifacts.operator_protocol import OperatorProtocol, OperatorProtocolError
from artifacts.protocol_files import open_directory
from artifacts.qgc_operator import MAV_RESULT_ACCEPTED, QGC_START_COMMAND
from orchestration.config import load_run_config
from orchestration.status_store import ProtocolFileError, StatusStore


WORLD = "moving_pad_stationary"
ENTITY = "moving_pad"
SERVICE = f"/world/{WORLD}/set_pose"
REQUEST_TYPE = "gz.msgs.Pose"
RESPONSE_TYPE = "gz.msgs.Boolean"
ODOMETRY_TOPIC = "/gazebo/private/moving_pad/odometry"
RANGE_TOPIC = "/gazebo/private/range/downward"
START_X_M = 35.0
TARGET_X_M = 55.0
MINIMUM_CLEARANCE_M = 0.75
PL_FRESHNESS_NS = 500_000_000
LAND_FRESHNESS_NS = 5_000_000_000
SENSOR_FRESHNESS_NS = 500_000_000
PERSISTENCE_NS = 500_000_000
# Covers 0.8 simulated seconds at the world's nominal 20 Hz publication rate.
MAX_PERSISTENCE_SAMPLES = 16
POSITION_TOLERANCE_M = 0.25
REQUEST_TEXT = (
    'name: "moving_pad" position { x: 55 y: 0 z: 0 } '
    "orientation { x: 0 y: 0 z: 0 w: 1 }"
)
RECORD_NAME = "precision-loss-stimulus.jsonl"


class StimulusError(RuntimeError):
    """The requested physical stimulus is unsafe or lacks required evidence."""


@dataclass(frozen=True)
class PoseSample:
    native_timestamp_ns: int
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True)
class RangeSample:
    native_timestamp_ns: int
    distance_m: float


@dataclass(frozen=True)
class PrecisionRow:
    time_us: int
    acquired: bool
    last_measurement_ms: int


@dataclass(frozen=True)
class ModeRow:
    time_us: int
    mode: int


Runner = Callable[..., subprocess.CompletedProcess[str]]


def _block(text: str, name: str, *, start: int = 0) -> str:
    match = re.search(rf"(?:^|\s){re.escape(name)}\s*\{{", text[start:])
    if match is None:
        raise StimulusError(f"Gazebo output is missing {name!r}")
    opening = start + match.end() - 1
    depth = 0
    for index in range(opening, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[opening + 1 : index]
    raise StimulusError(f"Gazebo output has an unterminated {name!r} block")


def _scalar(block: str, name: str, *, default: float | None = None) -> float:
    match = re.search(
        rf"(?:^|\s){re.escape(name)}\s*:\s*([^\s}}]+)", block
    )
    if match is None:
        if default is not None:
            return default
        raise StimulusError(f"Gazebo output is missing scalar {name!r}")
    try:
        value = float(match.group(1))
    except ValueError as error:
        raise StimulusError(f"Gazebo scalar {name!r} is invalid") from error
    if not math.isfinite(value):
        raise StimulusError(f"Gazebo scalar {name!r} is not finite")
    return value


def _timestamp_ns(text: str) -> int:
    stamp = _block(_block(text, "header"), "stamp")
    seconds = _scalar(stamp, "sec", default=0.0)
    nanoseconds = _scalar(stamp, "nsec", default=0.0)
    if not seconds.is_integer() or not nanoseconds.is_integer():
        raise StimulusError("Gazebo timestamp fields must be integers")
    timestamp = int(seconds) * 1_000_000_000 + int(nanoseconds)
    if timestamp < 0 or not 0 <= int(nanoseconds) < 1_000_000_000:
        raise StimulusError("Gazebo timestamp is invalid")
    return timestamp


def parse_gz_odometry(text: str) -> PoseSample:
    """Parse one `gz.msgs.Odometry` protobuf-text sample."""
    try:
        pose = _block(text, "pose")
        position = _block(pose, "position")
        orientation = _block(pose, "orientation")
        coordinates = (
            _scalar(position, "x", default=0.0),
            _scalar(position, "y", default=0.0),
            _scalar(position, "z", default=0.0),
        )
        if not re.search(r"(?:^|\s)x\s*:", position):
            raise StimulusError("Gazebo odometry position has no x coordinate")
        quaternion = (
            _scalar(orientation, "x", default=0.0),
            _scalar(orientation, "y", default=0.0),
            _scalar(orientation, "z", default=0.0),
            _scalar(orientation, "w", default=0.0),
        )
        return PoseSample(_timestamp_ns(text), coordinates, quaternion)
    except StimulusError as error:
        raise StimulusError(f"Gazebo odometry is malformed: {error}") from error


def parse_gz_range(text: str) -> RangeSample:
    """Parse one `gz.msgs.LaserScan` protobuf-text sample."""
    match = re.search(r"(?:^|\s)ranges\s*:\s*([^\s}}]+)", text)
    if match is None:
        raise StimulusError("Gazebo range sample has no distance")
    try:
        distance = float(match.group(1))
    except ValueError as error:
        raise StimulusError("Gazebo range distance is invalid") from error
    if not math.isfinite(distance) or distance <= 0:
        raise StimulusError("Gazebo range distance must be finite and positive")
    return RangeSample(_timestamp_ns(text), distance)


def _csv_rows(text: str, label: str) -> list[dict[str, str]]:
    try:
        rows = list(csv.DictReader(io.StringIO(text)))
    except csv.Error as error:
        raise StimulusError(f"DataFlash {label} CSV is malformed") from error
    if not rows:
        raise StimulusError(f"DataFlash {label} CSV has no rows")
    return rows


def _integer(row: dict[str, str], field: str, label: str) -> int:
    try:
        raw = row[field]
        value = int(raw)
    except (KeyError, TypeError, ValueError) as error:
        raise StimulusError(f"DataFlash {label} row has invalid {field}") from error
    if value < 0:
        raise StimulusError(f"DataFlash {label} row has invalid {field}")
    return value


def parse_dataflash_precision_csv(text: str) -> tuple[PrecisionRow, ...]:
    rows = _csv_rows(text, "PL")
    parsed: list[PrecisionRow] = []
    for row in rows:
        acquired = _integer(row, "TAcq", "PL")
        if acquired not in {0, 1}:
            raise StimulusError("DataFlash PL row has invalid TAcq")
        parsed.append(
            PrecisionRow(
                _integer(row, "TimeUS", "PL"),
                bool(acquired),
                _integer(row, "LastMeasMS", "PL"),
            )
        )
    return tuple(parsed)


def parse_dataflash_mode_csv(text: str) -> tuple[ModeRow, ...]:
    rows = _csv_rows(text, "MODE")
    parsed: list[ModeRow] = []
    for row in rows:
        mode = _integer(row, "Mode", "MODE")
        alias = _integer(row, "ModeNum", "MODE")
        if mode != alias:
            raise StimulusError("DataFlash MODE row aliases disagree")
        parsed.append(ModeRow(_integer(row, "TimeUS", "MODE"), mode))
    return tuple(parsed)


def _run(
    runner: Runner,
    command: list[str],
    *,
    timeout: float,
) -> subprocess.CompletedProcess[str]:
    try:
        result = runner(
            command,
            shell=False,
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise StimulusError(f"command failed or timed out: {command[0]}: {error}") from error
    if not isinstance(result.returncode, int):
        raise StimulusError(f"command returned an invalid status: {command[0]}")
    return result


def _checked_output(
    runner: Runner,
    command: list[str],
    *,
    timeout: float,
    label: str,
) -> str:
    result = _run(runner, command, timeout=timeout)
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or "no diagnostic output"
        raise StimulusError(f"{label} failed: {detail}")
    return result.stdout


def _reserve_record(run_directory: Path) -> tuple[int, int, Path]:
    log_directory = run_directory / "logs/docker"
    if log_directory.is_symlink() or not log_directory.is_dir():
        raise StimulusError("run logs/docker directory is missing or unsafe")
    directory_flags = (
        os.O_RDONLY
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        directory_fd = os.open(log_directory, directory_flags)
    except OSError as error:
        raise StimulusError("run logs/docker directory is unsafe") from error
    file_flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | os.O_APPEND
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    try:
        file_fd = os.open(RECORD_NAME, file_flags, 0o600, dir_fd=directory_fd)
    except FileExistsError:
        os.close(directory_fd)
        raise FileExistsError(
            f"{log_directory / RECORD_NAME} already exists; prepare a fresh run"
        ) from None
    except OSError as error:
        os.close(directory_fd)
        raise StimulusError("could not reserve the no-clobber stimulus record") from error
    return directory_fd, file_fd, log_directory / RECORD_NAME


def _append_record(directory_fd: int, file_fd: int, document: Mapping[str, Any]) -> None:
    payload = (
        json.dumps(
            document,
            allow_nan=False,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    ).encode("utf-8")
    written = 0
    while written < len(payload):
        count = os.write(file_fd, payload[written:])
        if count <= 0:
            raise OSError("stimulus record write made no progress")
        written += count
    os.fsync(file_fd)
    os.fsync(directory_fd)


def _assert_running(run_directory: Path, run_id: str) -> None:
    status = StatusStore(run_directory.parent).read_operator_status(run_id)
    if status.state != "RUNNING":
        raise StimulusError(
            f"stimulus requires the run to remain RUNNING, found {status.state}"
        )
    finalization = run_directory / ".control/finalize-request.json"
    if finalization.exists() or finalization.is_symlink():
        raise StimulusError("stimulus is unavailable after finalization was requested")
    manifest = run_directory / "manifest.json"
    if manifest.exists() or manifest.is_symlink():
        raise StimulusError("stimulus is unavailable for a finalized run")


def _validate_start_admission(run_directory: Path, config: Any) -> tuple[Any, int]:
    descriptor = config.operator_control
    if config.launch_mode != "operator" or descriptor is None:
        raise StimulusError("stimulus requires a native QGC operator run")
    try:
        with OperatorProtocol(
            run_directory, config.run_id, config.config_sha256
        ) as protocol:
            request = protocol.read_start_request()
            admission = protocol.read_start_admission()
            statuses = protocol.read_qgc_command_statuses()
    except OperatorProtocolError as error:
        raise StimulusError(f"native QGC Start binding is invalid: {error}") from error
    if request is None or admission is None:
        raise StimulusError("native QGC Start admission is missing")
    expected_parameters = (descriptor.attempt_token, 0, 0, 0, 0, 0, 0)
    accepted = [
        row
        for row in statuses
        if (
            row.envelope.wire_version == 2
            and row.envelope.source == descriptor.qgc_source
            and row.envelope.target == descriptor.companion
            and row.envelope.command == QGC_START_COMMAND
            and row.envelope.parameters == expected_parameters
            and row.request_id == request.request_id
            and row.request_sha256 == request.sha256
            and row.admission_sha256 == admission.sha256
            and row.response.result == MAV_RESULT_ACCEPTED
        )
    ]
    if not accepted:
        raise StimulusError("native QGC Start admission has no matching accepted command")
    return admission, accepted[-1].envelope.received_wall_time_ns


def _qgc_intervention_evidence(
    run_directory: Path,
    descriptor: Any,
    start_received_wall_time_ns: int,
) -> dict[str, Any]:
    path = run_directory / "logs/docker/qgc-interventions.jsonl"
    if path.is_symlink() or not path.is_file():
        raise StimulusError("native QGC intervention evidence is missing or unsafe")
    rows: list[dict[str, Any]] = []
    try:
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if not line:
                continue
            value = json.loads(line)
            if not isinstance(value, dict):
                raise ValueError(f"row {line_number} is not an object")
            wall_time = value.get("wall_time_ns")
            if type(wall_time) is not int or wall_time < 0:
                raise ValueError(f"row {line_number} has an invalid wall time")
            rows.append(value)
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        raise StimulusError(f"native QGC intervention evidence is unreadable: {error}") from error

    def bound_start(row: Mapping[str, Any]) -> bool:
        fields = row.get("fields")
        return (
            row.get("event") == "qgc_manual_command"
            and row.get("message_type") == "COMMAND_LONG"
            and row.get("command") == QGC_START_COMMAND
            and row.get("source_system") == descriptor.qgc_source.system_id
            and row.get("source_component") == descriptor.qgc_source.component_id
            and isinstance(fields, dict)
            and fields.get("target_system") == descriptor.companion.system_id
            and fields.get("target_component") == descriptor.companion.component_id
            and fields.get("param1") == descriptor.attempt_token
            and all(fields.get(f"param{index}") == 0 for index in range(2, 8))
        )

    starts = [
        row
        for row in rows
        if bound_start(row) and row["wall_time_ns"] <= start_received_wall_time_ns
    ]
    if not starts:
        raise StimulusError("native QGC Start has no matching bridge intervention evidence")
    for row in rows:
        if row["wall_time_ns"] > start_received_wall_time_ns and not bound_start(row):
            raise StimulusError(
                "native QGC takeover evidence follows the accepted mission Start"
            )
    return {
        "path": "logs/docker/qgc-interventions.jsonl",
        "start_wall_time_ns": starts[-1]["wall_time_ns"],
        "accepted_start_received_wall_time_ns": start_received_wall_time_ns,
        "rows_checked": len(rows),
    }


def _read_companion_events(
    runner: Runner, container_id: str, run_id: str
) -> list[dict[str, Any]]:
    output = _checked_output(
        runner,
        ["docker", "logs", "--tail", "4096", container_id],
        timeout=10.0,
        label="live companion log read",
    )
    events: list[dict[str, Any]] = []
    try:
        for line_number, line in enumerate(output.splitlines(), start=1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(value, dict):
                continue
            structured_keys = {
                "run_id",
                "module",
                "severity",
                "event",
                "sim_timestamp",
                "wall_timestamp",
            }
            if not structured_keys.intersection(value):
                continue
            if set(value) != structured_keys | {"fields"}:
                raise StimulusError(
                    f"live companion log row {line_number} is a malformed structured event"
                )
            if value.get("run_id") != run_id or value.get("module") != "companion":
                raise StimulusError(
                    f"live companion log row {line_number} has the wrong run binding"
                )
            events.append(value)
    except UnicodeError as error:
        raise StimulusError(f"live companion log is unreadable: {error}") from error
    return events


def _precision_operation(
    runner: Runner, container_id: str, run_id: str, admission: Any
) -> dict[str, Any]:
    events = _read_companion_events(runner, container_id, run_id)
    starts = [
        event
        for event in events
        if event.get("event") == "operation_started"
        and isinstance(event.get("fields"), dict)
        and event["fields"].get("tool") == "precision_land"
    ]
    if not starts:
        raise StimulusError("current mission has no precision_land operation evidence")
    start = starts[-1]
    fields = start["fields"]
    operation_id = fields.get("operation_id")
    args = fields.get("args")
    timestamp = start.get("sim_timestamp")
    if (
        not isinstance(operation_id, str)
        or not isinstance(args, dict)
        or args.get("marker_id") != 7
        or isinstance(timestamp, bool)
        or not isinstance(timestamp, (int, float))
        or not math.isfinite(timestamp)
        or timestamp < 0
    ):
        raise StimulusError("precision_land operation evidence is malformed")
    if any(
        event.get("event") == "operation_finished"
        and isinstance(event.get("fields"), dict)
        and event["fields"].get("operation_id") == operation_id
        for event in events
    ):
        raise StimulusError("precision_land operation is no longer active")
    requested_modes: list[tuple[float, str]] = []
    for event in events:
        if event.get("event") != "precision_mode_requested":
            continue
        mode_fields = event.get("fields")
        mode_timestamp = event.get("sim_timestamp")
        if not isinstance(mode_fields, dict) or mode_fields.get("operation_id") != operation_id:
            continue
        mode = mode_fields.get("mode")
        if (
            mode not in {"LAND", "GUIDED"}
            or isinstance(mode_timestamp, bool)
            or not isinstance(mode_timestamp, (int, float))
            or not math.isfinite(mode_timestamp)
            or float(mode_timestamp) < float(timestamp)
        ):
            raise StimulusError("precision mode-request evidence is malformed")
        requested_modes.append((float(mode_timestamp), mode))
    land_requests = [item for item in requested_modes if item[1] == "LAND"]
    if not land_requests:
        raise StimulusError("current precision operation has no mission-owned LAND request")
    land_timestamp = land_requests[-1][0]
    if requested_modes[-1][1] != "LAND":
        raise StimulusError("mission-owned LAND is no longer the precision operation's latest request")
    native_start_ns = admission.native_epoch_ns + round(float(timestamp) * 1e9)
    native_land_request_ns = admission.native_epoch_ns + round(land_timestamp * 1e9)
    return {
        "operation_id": operation_id,
        "sim_timestamp": float(timestamp),
        "native_start_ns": native_start_ns,
        "land_request_sim_timestamp": land_timestamp,
        "native_land_request_ns": native_land_request_ns,
        "marker_id": 7,
        "source": "live_companion_container_log",
        "source_container_id": container_id,
        "source_tail_lines": 4096,
    }


def _discover_container(
    runner: Runner,
    run_directory: Path,
    run_id: str,
    service: str,
) -> tuple[str, str]:
    project = "drone-sim-" + run_id.replace("-", "")
    listing = _checked_output(
        runner,
        [
            "docker",
            "ps",
            "--filter",
            f"label=com.docker.compose.project={project}",
            "--filter",
            f"label=com.docker.compose.service={service}",
            "--format",
            "{{json .}}",
        ],
        timeout=10.0,
        label=f"owned {service} discovery",
    )
    rows: list[dict[str, Any]] = []
    try:
        for line in listing.splitlines():
            if line.strip():
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise ValueError("row is not an object")
                rows.append(value)
    except (json.JSONDecodeError, ValueError) as error:
        raise StimulusError(f"owned {service} discovery returned malformed JSON") from error
    if len(rows) != 1 or not isinstance(rows[0].get("ID"), str):
        raise StimulusError(f"exactly one live owned {service} container is required")
    container_id = rows[0]["ID"]
    inspection_text = _checked_output(
        runner,
        ["docker", "inspect", "--type", "container", container_id],
        timeout=10.0,
        label=f"owned {service} inspection",
    )
    try:
        inspection_rows = json.loads(inspection_text)
        if not isinstance(inspection_rows, list) or len(inspection_rows) != 1:
            raise ValueError("inspection count")
        inspection = inspection_rows[0]
        state = inspection["State"]
        container_config = inspection["Config"]
        labels = container_config["Labels"]
        environment = dict(
            entry.split("=", 1)
            for entry in container_config["Env"]
            if isinstance(entry, str) and "=" in entry
        )
        mounts = inspection["Mounts"]
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise StimulusError(f"owned {service} inspection is malformed") from error
    config_path = run_directory / "configuration/run.json"
    expected_labels = {
        "com.docker.compose.project": project,
        "com.docker.compose.service": service,
        "com.docker.compose.oneoff": "False",
    }
    expected_environment = {
        "SIM_RUN_ID": run_id,
        "SIM_RUN_DIRECTORY": str(run_directory),
        "SIM_CONFIG_PATH": str(config_path),
    }
    run_mount = any(
        isinstance(mount, dict)
        and mount.get("Source") == str(run_directory)
        and mount.get("Destination") == str(run_directory)
        and mount.get("RW") is True
        for mount in mounts
    )
    config_mount = any(
        isinstance(mount, dict)
        and mount.get("Source") == str(config_path)
        and mount.get("Destination") == str(config_path)
        and mount.get("RW") is False
        for mount in mounts
    )
    if (
        state.get("Running") is not True
        or any(labels.get(key) != value for key, value in expected_labels.items())
        or any(environment.get(key) != value for key, value in expected_environment.items())
        or not run_mount
        or not config_mount
    ):
        raise StimulusError(f"{service} container is not owned by the bound run")
    return container_id, project


def _docker_gz(
    runner: Runner,
    container_id: str,
    partition: str,
    arguments: list[str],
    *,
    timeout: float,
    label: str,
) -> str:
    return _checked_output(
        runner,
        [
            "docker",
            "exec",
            "--env",
            f"GZ_PARTITION={partition}",
            container_id,
            "gz",
            *arguments,
        ],
        timeout=timeout,
        label=label,
    )


def _topic(
    runner: Runner,
    container_id: str,
    partition: str,
    topic: str,
) -> str:
    return _docker_gz(
        runner,
        container_id,
        partition,
        ["topic", "-e", "-t", topic, "-n", "1"],
        timeout=5.0,
        label=f"Gazebo topic {topic}",
    )


def _dataflash_csv(
    runner: Runner,
    run_directory: Path,
    message_type: str,
) -> str:
    logs = tuple((run_directory / "ardupilot_sitl/logs").glob("*.BIN"))
    if len(logs) != 1 or logs[0].is_symlink() or not logs[0].is_file():
        raise StimulusError("exactly one live DataFlash BIN log is required")
    executable = shutil.which("mavlogdump.py")
    if executable is None:
        raise StimulusError("mavlogdump.py is unavailable in the locked environment")
    return _checked_output(
        runner,
        [
            executable,
            "--robust",
            "--types",
            message_type,
            "--format",
            "csv",
            str(logs[0]),
        ],
        timeout=15.0,
        label=f"live DataFlash {message_type} read",
    )


def _validate_flight_evidence(
    precision: tuple[PrecisionRow, ...],
    modes: tuple[ModeRow, ...],
    operation: dict[str, Any],
    native_now_ns: int,
) -> dict[str, Any]:
    latest_precision = max(precision, key=lambda row: row.time_us)
    latest_mode = max(modes, key=lambda row: row.time_us)
    precision_ns = latest_precision.time_us * 1_000
    last_measurement_ns = latest_precision.last_measurement_ms * 1_000_000
    mode_ns = latest_mode.time_us * 1_000
    operation_start_ns = int(operation["native_start_ns"])
    land_request_ns = int(operation["native_land_request_ns"])
    if not latest_precision.acquired:
        raise StimulusError("latest live DataFlash precision row is not TAcq=1")
    if latest_mode.mode != 9:
        raise StimulusError("latest live DataFlash mode is not mission-owned LAND")
    if precision_ns < operation_start_ns:
        raise StimulusError("live DataFlash TAcq=1 predates precision_land")
    if mode_ns <= land_request_ns:
        raise StimulusError("live DataFlash LAND does not follow the mission-owned LAND request")
    for label, timestamp_ns in (("TAcq=1", precision_ns), ("LAND", mode_ns)):
        if timestamp_ns > native_now_ns:
            raise StimulusError(f"live DataFlash {label} is ahead of native odometry")
    if native_now_ns - precision_ns > PL_FRESHNESS_NS:
        raise StimulusError("live DataFlash TAcq=1 evidence is stale")
    if (
        last_measurement_ns < operation_start_ns
        or last_measurement_ns > native_now_ns
        or native_now_ns - last_measurement_ns > PL_FRESHNESS_NS
    ):
        raise StimulusError("live DataFlash PL.LastMeasMS evidence is stale")
    if native_now_ns - mode_ns > LAND_FRESHNESS_NS:
        raise StimulusError("live DataFlash LAND evidence is stale")
    return {
        "precision": asdict(latest_precision),
        "mode": asdict(latest_mode),
        "precision_max_age_ns": PL_FRESHNESS_NS,
        "land_max_age_ns": LAND_FRESHNESS_NS,
    }


def _pose_document(sample: PoseSample) -> dict[str, Any]:
    return {
        "native_timestamp_ns": sample.native_timestamp_ns,
        "position": list(sample.position),
        "orientation": list(sample.orientation),
    }


def _at_expected_pose(sample: PoseSample, expected_x: float) -> bool:
    x, y, z = sample.position
    qx, qy, qz, qw = sample.orientation
    return (
        abs(x - expected_x) <= POSITION_TOLERANCE_M
        and abs(y) <= POSITION_TOLERANCE_M
        and abs(z) <= POSITION_TOLERANCE_M
        and abs(qx) <= 1e-3
        and abs(qy) <= 1e-3
        and abs(qz) <= 1e-3
        and abs(qw - 1.0) <= 1e-3
    )


def induce_precision_target_loss(
    run_directory: Path | str,
    *,
    runner: Runner = subprocess.run,
    wall_time_ns: Callable[[], int] = time.time_ns,
) -> dict[str, Any]:
    """Apply one no-clobber physical target-loss stimulus to a live bound run."""
    directory = Path(run_directory).resolve()
    try:
        config = load_run_config(directory / "configuration/run.json")
    except (OSError, ValueError) as error:
        raise StimulusError(f"immutable run configuration is invalid: {error}") from error
    if directory.name != config.run_id or directory.parent != config.output_root:
        raise StimulusError("run directory does not match immutable run binding")
    if config.world != WORLD:
        raise StimulusError(f"stimulus requires world {WORLD}")
    if config.mission != "configured" or config.scenario != "moving_pad_v1":
        raise StimulusError("stimulus requires the stationary precision mission")
    _assert_running(directory, config.run_id)
    admission, start_received_wall_time_ns = _validate_start_admission(
        directory, config
    )
    record_path = directory / "logs/docker" / RECORD_NAME
    if record_path.exists() or record_path.is_symlink():
        raise FileExistsError(f"{record_path} already exists; prepare a fresh run")

    partition = "drone_sim_" + config.run_id.replace("-", "_")
    container_id, project = _discover_container(
        runner, directory, config.run_id, "gazebo-runtime"
    )
    companion_container_id, companion_project = _discover_container(
        runner, directory, config.run_id, "companion-runtime"
    )
    if companion_project != project:
        raise StimulusError("owned runtime containers disagree on Compose project")
    _precision_operation(runner, companion_container_id, config.run_id, admission)
    _qgc_intervention_evidence(
        directory, config.operator_control, start_received_wall_time_ns
    )
    services = set(
        _docker_gz(
            runner,
            container_id,
            partition,
            ["service", "-l"],
            timeout=5.0,
            label="Gazebo service discovery",
        ).splitlines()
    )
    if SERVICE not in services:
        raise StimulusError(f"required Gazebo set_pose service is missing: {SERVICE}")
    parse_dataflash_precision_csv(_dataflash_csv(runner, directory, "PL"))
    parse_dataflash_mode_csv(_dataflash_csv(runner, directory, "MODE"))

    record: dict[str, Any] = {
        "schema_version": 1,
        "status": "failed",
        "detail": "stimulus did not complete",
        "run_id": config.run_id,
        "config_sha256": config.config_sha256,
        "world": config.world,
        "entity": ENTITY,
        "partition": partition,
        "container": {
            "id": container_id,
            "project": project,
            "service": "gazebo-runtime",
            "companion_id": companion_container_id,
        },
        "pre_command": None,
        "service": None,
        "post_command": None,
    }
    control_fd: int | None = None
    record_directory_fd: int | None = None
    record_fd: int | None = None
    terminal_written = False
    dispatched = False
    try:
        control_fd = open_directory(directory / ".control")
        fcntl.flock(control_fd, fcntl.LOCK_EX)
        _assert_running(directory, config.run_id)
        current_container_id, current_project = _discover_container(
            runner, directory, config.run_id, "gazebo-runtime"
        )
        current_companion_id, current_companion_project = _discover_container(
            runner, directory, config.run_id, "companion-runtime"
        )
        if (
            (current_container_id, current_project) != (container_id, project)
            or (current_companion_id, current_companion_project)
            != (companion_container_id, project)
        ):
            raise StimulusError("owned runtime containers changed before the stimulus")
        record_directory_fd, record_fd, _record_path = _reserve_record(directory)
        operation = _precision_operation(
            runner, companion_container_id, config.run_id, admission
        )
        qgc_evidence = _qgc_intervention_evidence(
            directory, config.operator_control, start_received_wall_time_ns
        )
        services = set(
            _docker_gz(
                runner,
                container_id,
                partition,
                ["service", "-l"],
                timeout=5.0,
                label="Gazebo service discovery",
            ).splitlines()
        )
        if SERVICE not in services:
            raise StimulusError(f"required Gazebo set_pose service is missing: {SERVICE}")
        precision_rows = parse_dataflash_precision_csv(
            _dataflash_csv(runner, directory, "PL")
        )
        mode_rows = parse_dataflash_mode_csv(
            _dataflash_csv(runner, directory, "MODE")
        )
        pre_pose = parse_gz_odometry(
            _topic(runner, container_id, partition, ODOMETRY_TOPIC)
        )
        if not _at_expected_pose(pre_pose, START_X_M):
            raise StimulusError("pre-command moving_pad odometry is not at x=35 identity pose")
        pre_range = parse_gz_range(
            _topic(runner, container_id, partition, RANGE_TOPIC)
        )
        if pre_range.native_timestamp_ns > pre_pose.native_timestamp_ns:
            native_now_ns = pre_range.native_timestamp_ns
        else:
            native_now_ns = pre_pose.native_timestamp_ns
        if abs(pre_pose.native_timestamp_ns - pre_range.native_timestamp_ns) > SENSOR_FRESHNESS_NS:
            raise StimulusError("pre-command native pose and range samples are not fresh together")
        if pre_range.distance_m <= MINIMUM_CLEARANCE_M:
            raise StimulusError(
                f"downward clearance must be greater than {MINIMUM_CLEARANCE_M} m"
            )
        flight = _validate_flight_evidence(
            precision_rows, mode_rows, operation, native_now_ns
        )
        record["pre_command"] = {
            "pad_odometry": _pose_document(pre_pose),
            "range": asdict(pre_range),
            "native_horizon_ns": native_now_ns,
            "mission": operation,
            "qgc_interventions": qgc_evidence,
            "dataflash": flight,
        }
        _assert_running(directory, config.run_id)
        requested_at_ns = wall_time_ns()
        if type(requested_at_ns) is not int or requested_at_ns < 0:
            raise StimulusError("request wall clock returned an invalid timestamp")
        service_command = [
            "docker",
            "exec",
            "--env",
            f"GZ_PARTITION={partition}",
            container_id,
            "gz",
            "service",
            "-s",
            SERVICE,
            "--reqtype",
            REQUEST_TYPE,
            "--reptype",
            RESPONSE_TYPE,
            "--timeout",
            "5000",
            "--req",
            REQUEST_TEXT,
        ]
        record["service"] = {
            "name": SERVICE,
            "request_type": REQUEST_TYPE,
            "response_type": RESPONSE_TYPE,
            "request_text": REQUEST_TEXT,
            "request_utf8_hex": REQUEST_TEXT.encode("utf-8").hex(),
            "request_sha256": hashlib.sha256(
                REQUEST_TEXT.encode("utf-8")
            ).hexdigest(),
            "requested_wall_time_ns": requested_at_ns,
            "outcome": "pending",
            "returncode": None,
            "response": None,
            "stderr": None,
        }
        record["status"] = "attempted"
        record["detail"] = "durable request recorded before Gazebo dispatch"
        _append_record(record_directory_fd, record_fd, record)
        dispatched = True
        try:
            service_result = runner(
                service_command,
                shell=False,
                capture_output=True,
                text=True,
                check=False,
                timeout=8.0,
            )
            if not isinstance(service_result.returncode, int):
                raise TypeError("Gazebo service returned an invalid status")
        except BaseException as error:
            record["status"] = "unknown"
            record["detail"] = (
                "Gazebo set_pose dispatch did not return; physical outcome is unknown: "
                f"{type(error).__name__}: {error}"
            )
            record["service"]["outcome"] = "unknown"
            _append_record(record_directory_fd, record_fd, record)
            terminal_written = True
            if isinstance(error, Exception):
                raise StimulusError(record["detail"]) from error
            raise
        record["service"].update(
            outcome="returned",
            returncode=service_result.returncode,
            response=service_result.stdout,
            stderr=service_result.stderr,
        )
        record["status"] = "response_received"
        record["detail"] = "Gazebo set_pose response durably recorded"
        _append_record(record_directory_fd, record_fd, record)
        if service_result.returncode != 0:
            raise StimulusError(
                "Gazebo set_pose command failed: "
                + (service_result.stderr.strip() or "no diagnostic output")
            )
        if re.fullmatch(r"\s*data:\s*true\s*", service_result.stdout) is None:
            raise StimulusError("Gazebo set_pose service rejected the request")
        first_post = parse_gz_odometry(
            _topic(runner, container_id, partition, ODOMETRY_TOPIC)
        )
        displacement = first_post.position[0] - pre_pose.position[0]
        if first_post.native_timestamp_ns <= native_now_ns:
            record["post_command"] = {
                "first": _pose_document(first_post),
                "persistence": [],
                "displacement_m": displacement,
            }
            raise StimulusError(
                "first post-command odometry must be strictly after the pre-command horizon"
            )
        if not _at_expected_pose(first_post, TARGET_X_M) or not math.isclose(
            displacement, 20.0, abs_tol=0.5
        ):
            record["post_command"] = {
                "first": _pose_document(first_post),
                "persistence": [],
                "displacement_m": displacement,
            }
            raise StimulusError("moving_pad did not move approximately 20 m to x=55")
        persistence: list[PoseSample] = []
        previous_timestamp_ns = first_post.native_timestamp_ns
        for _attempt in range(MAX_PERSISTENCE_SAMPLES):
            sample = parse_gz_odometry(
                _topic(runner, container_id, partition, ODOMETRY_TOPIC)
            )
            persistence.append(sample)
            if sample.native_timestamp_ns <= previous_timestamp_ns:
                record["post_command"] = {
                    "first": _pose_document(first_post),
                    "persistence": [_pose_document(value) for value in persistence],
                    "displacement_m": displacement,
                }
                raise StimulusError(
                    "post-command odometry timestamps must strictly increase"
                )
            previous_timestamp_ns = sample.native_timestamp_ns
            if not _at_expected_pose(sample, TARGET_X_M):
                record["post_command"] = {
                    "first": _pose_document(first_post),
                    "persistence": [_pose_document(value) for value in persistence],
                    "displacement_m": displacement,
                }
                raise StimulusError("moving_pad displacement did not persist")
            if sample.native_timestamp_ns - first_post.native_timestamp_ns > PERSISTENCE_NS:
                break
        if (
            not persistence
            or persistence[-1].native_timestamp_ns - first_post.native_timestamp_ns
            <= PERSISTENCE_NS
        ):
            raise StimulusError(
                "moving_pad did not persist strictly more than 0.5 simulated seconds"
            )
        record["post_command"] = {
            "first": _pose_document(first_post),
            "persistence": [_pose_document(value) for value in persistence],
            "displacement_m": displacement,
            "minimum_persistence_ns": PERSISTENCE_NS,
            "threshold_comparison": "strictly_greater_than",
        }
        record["post_command"]["qgc_interventions"] = _qgc_intervention_evidence(
            directory, config.operator_control, start_received_wall_time_ns
        )
        record["status"] = "succeeded"
        record["detail"] = (
            "moving_pad remained at x=55 for more than 0.5 simulated seconds "
            "after an approximately 20 m displacement"
        )
        _append_record(record_directory_fd, record_fd, record)
        terminal_written = True
        return record
    except FileExistsError:
        raise
    except (ProtocolFileError, StimulusError) as error:
        record["detail"] = str(error)
        if record_fd is not None and record_directory_fd is not None and not terminal_written:
            record["status"] = "failed"
            _append_record(record_directory_fd, record_fd, record)
            terminal_written = True
        raise StimulusError(str(error)) from error
    except Exception as error:
        record["detail"] = f"unexpected stimulus failure: {type(error).__name__}: {error}"
        if record_fd is not None and record_directory_fd is not None and not terminal_written:
            record["status"] = "failed"
            _append_record(record_directory_fd, record_fd, record)
            terminal_written = True
        raise StimulusError(record["detail"]) from error
    except BaseException as error:
        if record_fd is not None and record_directory_fd is not None and not terminal_written:
            record["status"] = "unknown" if dispatched else "failed"
            record["detail"] = f"stimulus interrupted: {type(error).__name__}: {error}"
            if isinstance(record.get("service"), dict) and dispatched:
                record["service"]["outcome"] = "unknown"
            _append_record(record_directory_fd, record_fd, record)
            terminal_written = True
        raise
    finally:
        if record_fd is not None:
            os.close(record_fd)
        if record_directory_fd is not None:
            os.close(record_directory_fd)
        if control_fd is not None:
            try:
                fcntl.flock(control_fd, fcntl.LOCK_UN)
            finally:
                os.close(control_fd)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path, metavar="RUN_DIR")
    arguments = parser.parse_args()
    try:
        result = induce_precision_target_loss(arguments.run_directory)
    except Exception as error:
        print(
            json.dumps(
                {"status": "failed", "detail": str(error)},
                sort_keys=True,
                separators=(",", ":"),
            )
        )
        return 1
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
