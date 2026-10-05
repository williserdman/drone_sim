"""Validated roll, pitch, and yaw AutoTune parameter artifacts."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any
from uuid import UUID
import xml.etree.ElementTree as ET

from pymavlink import mavutil


def airframe_fingerprint(model_xml: bytes) -> str:
    """Bind physical dynamics, excluding only declared evidence/payload plugins."""
    document = ET.fromstring(model_xml)
    model = document.find("model")
    if model is None:
        raise ValueError("airframe SDF requires an outer model")
    model.set("name", "competition-airframe")
    for plugin in list(model.findall("plugin")):
        if plugin.get("name") == "gz::sim::systems::PosePublisher" or plugin.get("filename") in {
            "libcwru_payload_command_coordinator.so",
            "libdrone_sim_detachable_joint_system.so",
        }:
            model.remove(plugin)
    canonical = ET.canonicalize(ET.tostring(document, encoding="unicode"), strip_text=True)
    return hashlib.sha256(canonical.encode()).hexdigest()


CALIBRATION_PARAMETERS = (
    "ATC_ANG_RLL_P",
    "ATC_RAT_RLL_P",
    "ATC_RAT_RLL_I",
    "ATC_RAT_RLL_D",
    "ATC_ACC_R_MAX",
    "ATC_ANG_PIT_P",
    "ATC_RAT_PIT_P",
    "ATC_RAT_PIT_I",
    "ATC_RAT_PIT_D",
    "ATC_ACC_P_MAX",
    "ATC_ANG_YAW_P",
    "ATC_RAT_YAW_P",
    "ATC_RAT_YAW_I",
    "ATC_RAT_YAW_FLTE",
    "ATC_ACC_Y_MAX",
)

PRESERVED_PARAMETERS = (
    "ATC_RATE_FF_ENAB",
    "ATC_RAT_YAW_D",
    "ATC_RAT_RLL_FF",
    "ATC_RAT_RLL_D_FF",
    "ATC_RAT_RLL_FLTT",
    "ATC_RAT_RLL_FLTD",
    "ATC_RAT_RLL_FLTE",
    "ATC_RAT_RLL_SMAX",
    "ATC_RAT_PIT_FF",
    "ATC_RAT_PIT_D_FF",
    "ATC_RAT_PIT_FLTT",
    "ATC_RAT_PIT_FLTD",
    "ATC_RAT_PIT_FLTE",
    "ATC_RAT_PIT_SMAX",
    "ATC_RAT_YAW_FF",
    "ATC_RAT_YAW_D_FF",
    "ATC_RAT_YAW_FLTT",
    "ATC_RAT_YAW_FLTD",
    "ATC_RAT_YAW_SMAX",
)

_HEADER = "# drone_sim AutoTune calibration v1"
_AXES_HEADER = "# autotune_axes 7"
_SAVED_STATUS = "AutoTune: Saved gains for Roll Pitch Yaw(E)"
_DISARMED_EVENT = 11
_SAVED_GAINS_EVENT = 37
_SAVE_WINDOW_US = 1_000_000
_PARAMETER_REL_TOL = 1e-5
_PARAMETER_ABS_TOL = 1e-7


def _canonical_run_id(run_id: str) -> str:
    try:
        canonical = str(UUID(run_id))
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if run_id != canonical:
        raise ValueError("run_id must be a canonical UUID")
    return canonical


def _finite_parameters(
    values: Mapping[str, float], names: tuple[str, ...], *, positive: bool
) -> dict[str, float]:
    if not isinstance(values, Mapping) or set(values) != set(names):
        raise ValueError("parameter names do not match the required calibration contract")
    normalized: dict[str, float] = {}
    for name in names:
        value = values[name]
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or (positive and value <= 0)
        ):
            requirement = "finite and positive" if positive else "finite"
            raise ValueError(f"parameter {name} must be {requirement}")
        normalized[name] = float(value)
    return normalized


def validate_calibration_parameters(values: Mapping[str, float]) -> dict[str, float]:
    """Return the canonical 15-value axes-7 calibration mapping."""
    normalized = _finite_parameters(values, CALIBRATION_PARAMETERS, positive=True)
    for axis in ("RLL", "PIT"):
        if not math.isclose(
            normalized[f"ATC_RAT_{axis}_I"],
            normalized[f"ATC_RAT_{axis}_P"],
            rel_tol=_PARAMETER_REL_TOL,
            abs_tol=_PARAMETER_ABS_TOL,
        ):
            raise ValueError(f"{axis.lower()} I gain must equal P gain")
    if not math.isclose(
        normalized["ATC_RAT_YAW_I"],
        normalized["ATC_RAT_YAW_P"] * 0.1,
        rel_tol=_PARAMETER_REL_TOL,
        abs_tol=_PARAMETER_ABS_TOL,
    ):
        raise ValueError("yaw I gain must equal 0.1 times P gain")
    return normalized


def read_calibration_parameters(path: Path) -> tuple[str, dict[str, float]]:
    """Read one versioned, deterministic axes-7 parameter artifact."""
    try:
        lines = Path(path).read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise ValueError("calibration parameter artifact is not readable UTF-8") from error
    if len(lines) != 3 + len(CALIBRATION_PARAMETERS) or lines[0] != _HEADER:
        raise ValueError("calibration parameter artifact has an invalid format")
    prefix = "# run_id "
    if not lines[1].startswith(prefix) or lines[2] != _AXES_HEADER:
        raise ValueError("calibration parameter artifact has invalid metadata")
    run_id = _canonical_run_id(lines[1][len(prefix) :])
    values: dict[str, float] = {}
    for expected_name, line in zip(CALIBRATION_PARAMETERS, lines[3:], strict=True):
        fields = line.split()
        if len(fields) != 2 or fields[0] != expected_name:
            raise ValueError("calibration parameters are missing, extra, or out of order")
        try:
            values[expected_name] = float(fields[1])
        except ValueError as error:
            raise ValueError(f"parameter {expected_name} is not numeric") from error
    return run_id, validate_calibration_parameters(values)


def write_calibration_parameters(
    run_directory: Path, run_id: str, values: Mapping[str, float]
) -> Path:
    """Atomically write the canonical run-scoped axes-7 parameter artifact."""
    canonical = _canonical_run_id(run_id)
    normalized = validate_calibration_parameters(values)
    target = Path(run_directory) / "ardupilot_sitl/autotune.parm"
    target.parent.mkdir(parents=True, exist_ok=True)
    lines = (
        _HEADER,
        f"# run_id {canonical}",
        _AXES_HEADER,
        *(f"{name} {normalized[name]:g}" for name in CALIBRATION_PARAMETERS),
    )
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=target.parent,
            prefix=f".{target.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write("\n".join(lines) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(0o644)
        os.replace(temporary, target)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return target


def _text(value: object) -> str | None:
    if isinstance(value, bytes):
        try:
            value = value.decode("ascii")
        except UnicodeDecodeError:
            return None
    return value.rstrip("\0 ") if isinstance(value, str) else None


def _selected_dataflash_log(run_directory: Path) -> Path:
    logs = tuple((Path(run_directory) / "ardupilot_sitl/logs").glob("*.BIN"))
    if not logs:
        raise ValueError("ArduPilot DataFlash log is missing")
    return max(logs, key=lambda path: (path.stat().st_mtime_ns, path.name))


def read_saved_calibration_parameters(
    run_directory: Path,
    *,
    connection_factory: Callable[[str], Any] = mavutil.mavlink_connection,
) -> dict[str, float]:
    """Read the latest complete parameter save associated with native disarm."""
    log = _selected_dataflash_log(Path(run_directory))
    connection = connection_factory(str(log))
    saved_statuses: set[int] = set()
    disarms: set[int] = set()
    saved_events: set[int] = set()
    parameters: list[tuple[int, str, float]] = []
    try:
        while True:
            message = connection.recv_match(blocking=False)
            if message is None:
                break
            row = message.to_dict()
            message_type = row.get("mavpackettype")
            timestamp = row.get("TimeUS")
            if type(timestamp) is not int:
                if message_type in {"MSG", "EV", "PARM"}:
                    raise ValueError("saved DataFlash evidence has an invalid timestamp")
                continue
            if message_type == "MSG" and _text(row.get("Message")) == _SAVED_STATUS:
                saved_statuses.add(timestamp)
            elif message_type == "EV":
                event_id = row.get("Id")
                if event_id == _DISARMED_EVENT:
                    disarms.add(timestamp)
                elif event_id == _SAVED_GAINS_EVENT:
                    saved_events.add(timestamp)
            elif message_type == "PARM":
                name = _text(row.get("Name"))
                if name not in CALIBRATION_PARAMETERS:
                    continue
                value = row.get("Value")
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError("saved DataFlash parameter record is invalid")
                parameters.append((timestamp, name, float(value)))
    finally:
        close = getattr(connection, "close", None)
        if callable(close):
            close()

    markers = saved_statuses & disarms & saved_events
    if not markers:
        raise ValueError("complete post-disarm AutoTune save evidence is missing")
    marker = max(markers)
    epoch: dict[str, float] = {}
    for timestamp, name, value in parameters:
        if not marker < timestamp <= marker + _SAVE_WINDOW_US:
            continue
        if name in epoch:
            raise ValueError(f"saved DataFlash epoch contains duplicate parameter {name}")
        epoch[name] = value
    if set(epoch) != set(CALIBRATION_PARAMETERS):
        raise ValueError("saved DataFlash calibration parameters are incomplete")
    return validate_calibration_parameters(epoch)


def read_calibration_baseline(run_directory: Path) -> dict[str, float]:
    """Read the exact preserved baseline mapping from companion evidence."""
    stages = _verified_parameter_stages(run_directory)
    baseline = stages["baseline"]
    return {name: baseline[name] for name in PRESERVED_PARAMETERS}


def _verified_parameter_stages(run_directory: Path) -> dict[str, dict[str, float]]:
    path = Path(run_directory) / "logs/companion.jsonl"
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise ValueError("companion calibration evidence is not readable") from error
    expected_stages = ("baseline", "activation", "post_disarm")
    stages: dict[str, dict[str, float]] = {}
    last_timestamp = -math.inf
    for line in lines:
        try:
            row = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError("companion calibration evidence is invalid JSON") from error
        if row.get("event") != "calibration_parameters_verified":
            continue
        fields = row.get("fields")
        stage = fields.get("stage") if isinstance(fields, dict) else None
        if stage not in expected_stages:
            raise ValueError("calibration parameter evidence has an invalid stage")
        if stage in stages or stage != expected_stages[len(stages)]:
            raise ValueError("calibration parameter evidence stages are duplicated or out of order")
        timestamp = row.get("sim_timestamp")
        if (
            isinstance(timestamp, bool)
            or not isinstance(timestamp, (int, float))
            or not math.isfinite(timestamp)
            or timestamp <= last_timestamp
        ):
            raise ValueError("calibration parameter evidence timestamps are invalid")
        parameters = fields.get("parameters")
        if not isinstance(parameters, dict):
            raise ValueError("companion calibration parameter evidence is invalid")
        stages[stage] = _finite_parameters(
            parameters, CALIBRATION_PARAMETERS + PRESERVED_PARAMETERS, positive=False
        )
        last_timestamp = float(timestamp)
    if tuple(stages) != expected_stages:
        raise ValueError("calibration parameter evidence stages are incomplete")
    return stages


def _parameters_match(left: Mapping[str, float], right: Mapping[str, float]) -> bool:
    return set(left) == set(right) and all(
        math.isclose(
            left[name], right[name],
            rel_tol=_PARAMETER_REL_TOL, abs_tol=_PARAMETER_ABS_TOL,
        )
        for name in left
    )


def _require_inventoried_file(
    directory: Path, manifest: Mapping[str, object], relative_path: str
) -> None:
    path = directory / relative_path
    try:
        size = path.stat().st_size
        with path.open("rb") as stream:
            digest = hashlib.file_digest(stream, "sha256").hexdigest()
    except OSError as error:
        raise ValueError(f"inventoried file {relative_path} is unreadable") from error
    rows = manifest.get("artifacts")
    records = [
        row for row in rows if isinstance(row, dict)
        and row.get("relative_path") == relative_path
    ] if isinstance(rows, list) else []
    if len(records) != 1 or records[0] != {
        "relative_path": relative_path,
        "size_bytes": size,
        "sha256": digest,
        "validation": "valid",
        "detail": "valid regular file",
    }:
        raise ValueError(f"{relative_path} is not validly inventoried")


def validate_calibration_artifact(
    run_directory: Path,
    *,
    connection_factory: Callable[[str], Any] = mavutil.mavlink_connection,
) -> dict[str, float]:
    """Validate an inventoried calibration against saved and live evidence."""
    directory = Path(run_directory)
    try:
        configuration = json.loads(
            (directory / "configuration/run.json").read_text(encoding="utf-8")
        )
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError("calibration configuration or manifest is unreadable") from error
    if not isinstance(configuration, dict) or not isinstance(manifest, dict):
        raise ValueError("calibration configuration and manifest must be objects")
    run_id = configuration.get("run_id")
    if (
        configuration.get("mission") != "autotune"
        or configuration.get("scenario") != "calibration_v1"
        or manifest.get("terminal_status") != "COMPLETED"
        or manifest.get("run_id") != run_id
    ):
        raise ValueError("run is not a completed calibration mission")
    _canonical_run_id(run_id)

    artifact_path = directory / "ardupilot_sitl/autotune.parm"
    artifact_run_id, artifact_values = read_calibration_parameters(artifact_path)
    if artifact_run_id != run_id:
        raise ValueError("calibration artifact belongs to another run")
    _require_inventoried_file(
        directory, manifest, "ardupilot_sitl/autotune.parm"
    )
    dataflash_relative = _selected_dataflash_log(directory).relative_to(directory).as_posix()
    _require_inventoried_file(directory, manifest, dataflash_relative)

    saved = read_saved_calibration_parameters(
        directory, connection_factory=connection_factory
    )
    stages = _verified_parameter_stages(directory)
    activation = {name: stages["activation"][name] for name in CALIBRATION_PARAMETERS}
    post_disarm = {name: stages["post_disarm"][name] for name in CALIBRATION_PARAMETERS}
    if not (
        _parameters_match(artifact_values, saved)
        and _parameters_match(artifact_values, activation)
        and _parameters_match(artifact_values, post_disarm)
    ):
        raise ValueError("saved, activated, readback, and artifact gains do not match")
    baseline_preserved = {
        name: stages["baseline"][name] for name in PRESERVED_PARAMETERS
    }
    if any(
        not _parameters_match(
            baseline_preserved,
            {name: stages[stage][name] for name in PRESERVED_PARAMETERS},
        )
        for stage in ("activation", "post_disarm")
    ):
        raise ValueError("preserved calibration parameters changed")
    return artifact_values


__all__ = [
    "CALIBRATION_PARAMETERS",
    "PRESERVED_PARAMETERS",
    "read_calibration_baseline",
    "read_calibration_parameters",
    "read_saved_calibration_parameters",
    "validate_calibration_artifact",
    "validate_calibration_parameters",
    "write_calibration_parameters",
]
