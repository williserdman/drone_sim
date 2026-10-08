"""Independent acceptance checks for hover and roll AutoTune diagnostics."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from pathlib import Path
from uuid import UUID

from pymavlink import mavutil

from .acceptance import BundleAcceptanceError


ROLL_GAIN_PARAMETERS = (
    "ATC_ANG_RLL_P",
    "ATC_RAT_RLL_P",
    "ATC_RAT_RLL_I",
    "ATC_RAT_RLL_D",
    "ATC_ACC_R_MAX",
)

_ARTIFACT_HEADER = "# Roll gains saved by ArduPilot AutoTune"
_RUN_ID_PREFIX = "# run_id "
_REL_TOL = 1e-5
_ABS_TOL = 1e-7
_SAVE_WINDOW_US = 1_000_000
_DISARMED_EVENT = 11
_SAVED_GAINS_EVENT = 37
_SAVED_ROLL_STATUS = "AutoTune: Saved gains for Roll"


def _ordered(values: Sequence[str], required: tuple[str, ...]) -> bool:
    index = 0
    for value in values:
        if value == required[index]:
            index += 1
            if index == len(required):
                return True
    return False


def validate_diagnostic_logs(
    mission: str, companion: Sequence[Mapping[str, object]]
) -> None:
    """Require the mission-specific terminal phase and ordered flight evidence."""
    if mission == "hover_roll":
        phases = [
            fields.get("phase")
            for row in companion
            if row.get("event") == "hover_phase"
            and isinstance((fields := row.get("fields")), Mapping)
        ]
        if not _ordered(phases, ("HOVERING", "WAIT_LAND", "COMPLETE")):
            raise BundleAcceptanceError("hover diagnostic flight evidence is incomplete")
        return
    if mission == "autotune_roll":
        phases: list[str] = []
        statuses: list[str] = []
        for row in companion:
            fields = row.get("fields")
            if not isinstance(fields, Mapping):
                continue
            if row.get("event") == "autotune_phase":
                phase = fields.get("phase")
                if isinstance(phase, str):
                    phases.append(phase)
            elif row.get("event") == "ardupilot_status_text":
                status = fields.get("text")
                if isinstance(status, str):
                    statuses.append(status)
        if not _ordered(phases, ("TUNING", "LANDING_TO_SAVE", "COMPLETE")) or not _ordered(
            statuses,
            ("AutoTune: Success", "AutoTune: Saved gains for Roll"),
        ):
            raise BundleAcceptanceError("roll AutoTune diagnostic evidence is incomplete")
        return
    raise BundleAcceptanceError(f"unsupported diagnostic mission: {mission}")


def validate_diagnostic_landing(rule_results: Sequence[Mapping[str, object]]) -> None:
    """Require the three safe-landing predicates used by diagnostic missions."""
    results: dict[str, object] = {}
    for row in rule_results:
        rule_id = row.get("rule_id")
        if isinstance(rule_id, str):
            if rule_id in results:
                raise BundleAcceptanceError(f"duplicate diagnostic landing rule: {rule_id}")
            results[rule_id] = row.get("passed")
    for rule_id in (
        "airborne_then_contact",
        "safe_preimpact_speed",
        "stable_contact",
    ):
        if results.get(rule_id) is not True:
            raise BundleAcceptanceError(f"diagnostic landing rule failed: {rule_id}")


def _read_roll_artifact(path: Path) -> dict[str, float]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise ValueError("roll gain artifact is not readable UTF-8") from error
    if len(lines) != 2 + len(ROLL_GAIN_PARAMETERS) or lines[0] != _ARTIFACT_HEADER:
        raise ValueError("roll gain artifact has an invalid format")
    if not lines[1].startswith(_RUN_ID_PREFIX):
        raise ValueError("roll gain artifact lacks its run ID")
    run_id = lines[1][len(_RUN_ID_PREFIX) :]
    try:
        if str(UUID(run_id)) != run_id:
            raise ValueError
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError("roll gain artifact run ID is not canonical") from error
    values: dict[str, float] = {}
    for name, line in zip(ROLL_GAIN_PARAMETERS, lines[2:], strict=True):
        fields = line.split()
        if len(fields) != 2 or fields[0] != name:
            raise ValueError("roll gains are missing, extra, or out of order")
        try:
            value = float(fields[1])
        except ValueError as error:
            raise ValueError(f"roll gain {name} is not numeric") from error
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"roll gain {name} must be finite and positive")
        values[name] = value
    if not math.isclose(
        values["ATC_RAT_RLL_I"],
        values["ATC_RAT_RLL_P"],
        rel_tol=1e-9,
        abs_tol=1e-12,
    ):
        raise ValueError("saved roll I gain must equal the roll P gain")
    return values


def _read_native_roll_save(run_directory: Path) -> dict[str, float]:
    logs = tuple((run_directory / "ardupilot_sitl/logs").glob("*.BIN"))
    if not logs:
        raise ValueError("ArduPilot DataFlash log is missing")
    log = max(logs, key=lambda path: (path.stat().st_mtime_ns, path.name))
    connection = mavutil.mavlink_connection(str(log))
    disarms: set[int] = set()
    saved_markers: set[int] = set()
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
            if message_type == "EV":
                if row.get("Id") == _DISARMED_EVENT:
                    disarms.add(timestamp)
                elif row.get("Id") == _SAVED_GAINS_EVENT:
                    saved_markers.add(timestamp)
                continue
            if message_type == "MSG":
                status = row.get("Message")
                if isinstance(status, bytes):
                    try:
                        status = status.decode("ascii")
                    except UnicodeDecodeError:
                        continue
                if isinstance(status, str) and status.rstrip("\0 ") == _SAVED_ROLL_STATUS:
                    saved_markers.add(timestamp)
                continue
            if message_type != "PARM":
                continue
            name = row.get("Name")
            if isinstance(name, bytes):
                try:
                    name = name.decode("ascii").rstrip("\0 ")
                except UnicodeDecodeError:
                    continue
            if name not in ROLL_GAIN_PARAMETERS:
                continue
            value = row.get("Value")
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError("saved DataFlash parameter record is invalid")
            parameters.append((timestamp, name, float(value)))
    finally:
        close = getattr(connection, "close", None)
        if callable(close):
            close()
    markers = disarms & saved_markers
    if not markers:
        raise ValueError("complete post-disarm roll save evidence is missing")
    marker = max(markers)
    epoch: dict[str, float] = {}
    for timestamp, name, value in parameters:
        if not marker < timestamp <= marker + _SAVE_WINDOW_US:
            continue
        if name in epoch:
            raise ValueError(f"saved DataFlash epoch contains duplicate parameter {name}")
        epoch[name] = value
    if set(epoch) != set(ROLL_GAIN_PARAMETERS):
        raise ValueError("saved DataFlash roll gains are incomplete")
    return {name: epoch[name] for name in ROLL_GAIN_PARAMETERS}


def validate_roll_gain_artifact(run_directory: Path) -> dict[str, float]:
    """Validate the exact roll artifact against one native DataFlash save."""
    directory = Path(run_directory)
    try:
        artifact = _read_roll_artifact(
            directory / "ardupilot_sitl/autotune-roll.parm"
        )
        saved = _read_native_roll_save(directory)
    except (OSError, ValueError) as error:
        raise BundleAcceptanceError(f"roll gain artifact is invalid: {error}") from error
    if any(
        not math.isclose(
            artifact[name], saved[name], rel_tol=_REL_TOL, abs_tol=_ABS_TOL
        )
        for name in ROLL_GAIN_PARAMETERS
    ):
        raise BundleAcceptanceError("roll gain artifact does not match DataFlash save")
    return artifact


__all__ = [
    "ROLL_GAIN_PARAMETERS",
    "validate_diagnostic_landing",
    "validate_diagnostic_logs",
    "validate_roll_gain_artifact",
]
