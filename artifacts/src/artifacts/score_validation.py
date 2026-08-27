"""Validation of completed ``descent_v1`` scoring evidence."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from ._adapters.rosbag import (
    GroundTruthEvidence,
    PhysicalBagEvidence,
    ScoreEventEvidence,
)
from .validation import ValidationStatus, read_regular_file_bytes, validate_tree


_RULE_IDS = (
    "airborne_then_contact",
    "touchdown_precision",
    "safe_preimpact_speed",
    "stable_contact",
)
_EVENT_TYPES = tuple(f"descent.{rule_id}" for rule_id in _RULE_IDS) + (
    "score.finalized",
)
_AVAILABLE_POINTS = (20.0, 40.0, 20.0, 20.0)
_EVIDENCE_PATHS = tuple(
    f"scoring/events.jsonl#event-{index}" for index in range(5)
) + ("rosbag#/simulation/ground_truth",)


class ScoreValidationError(ValueError):
    """The persisted scoring evidence is not a valid completed descent score."""


@dataclass(frozen=True)
class ValidatedScoreMetadata:
    achieved_score: float
    maximum_available_score: float
    scoring_checksum: str
    evidence_paths: tuple[str, ...]


@dataclass(frozen=True)
class _IndependentDescentScore:
    awarded_points: tuple[float, float, float, float]
    achieved_score: float
    sim_timestamp_ns: int


def _finite_number(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScoreValidationError(f"{field} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ScoreValidationError(f"{field} must be a finite number")
    return number


def _read_json(
    run_directory: Path | str,
    relative_path: str,
    *,
    deadline_check: Callable[[], None] | None,
) -> object:
    validation, payload = read_regular_file_bytes(
        run_directory,
        relative_path,
        deadline_check=deadline_check,
    )
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise ScoreValidationError(f"{relative_path} is not safe scoring evidence")
    try:
        return json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ScoreValidationError(f"{relative_path} is not valid JSON") from error


def _validate_rule_results(document: object, achieved_score: float) -> tuple[float, ...]:
    if not isinstance(document, list) or len(document) != len(_RULE_IDS):
        raise ScoreValidationError("rule_results must contain exactly four rules")
    awarded: list[float] = []
    for index, (row, rule_id, available) in enumerate(
        zip(document, _RULE_IDS, _AVAILABLE_POINTS, strict=True)
    ):
        if not isinstance(row, dict) or row.get("rule_id") != rule_id:
            raise ScoreValidationError("rule_results identities or order are invalid")
        passed = row.get("passed")
        if not isinstance(passed, bool):
            raise ScoreValidationError(f"rule_results[{index}].passed must be boolean")
        actual_available = _finite_number(
            row.get("available_points"), f"rule_results[{index}].available_points"
        )
        actual_awarded = _finite_number(
            row.get("awarded_points"), f"rule_results[{index}].awarded_points"
        )
        if actual_available != available or actual_awarded != (available if passed else 0.0):
            raise ScoreValidationError("rule_results points are inconsistent")
        expected_ref = f"rosbag#/simulation/ground_truth:{rule_id}"
        if row.get("evidence_ref") != expected_ref:
            raise ScoreValidationError("rule_results evidence reference is invalid")
        awarded.append(actual_awarded)
    if sum(awarded) != achieved_score:
        raise ScoreValidationError("achieved_score does not match rule_results")
    return tuple(awarded)


def _validate_events(
    run_directory: Path | str,
    *,
    run_id: str,
    awarded: tuple[float, ...],
    achieved_score: float,
    deadline_check: Callable[[], None] | None,
) -> tuple[ScoreEventEvidence, ...]:
    payload = _read_json_lines(
        run_directory, "scoring/events.jsonl", deadline_check=deadline_check
    )
    if len(payload) != 5:
        raise ScoreValidationError("events.jsonl must contain exactly five events")
    expected_values = (*awarded, achieved_score)
    validated: list[ScoreEventEvidence] = []
    for index, (event, event_type, expected_value) in enumerate(
        zip(payload, _EVENT_TYPES, expected_values, strict=True)
    ):
        if not isinstance(event, dict):
            raise ScoreValidationError("events.jsonl rows must be JSON objects")
        if (
            event.get("run_id") != run_id
            or event.get("event_id") != index
            or event.get("event_type") != event_type
            or event.get("evidence_ref") != f"scoring/events.jsonl#event-{index}"
        ):
            raise ScoreValidationError("score events are not ordered contiguous descent_v1 events")
        timestamp = event.get("sim_timestamp_ns")
        if isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp < 0:
            raise ScoreValidationError("score event timestamp is invalid")
        if _finite_number(event.get("value"), f"events[{index}].value") != expected_value:
            raise ScoreValidationError("score event value does not match result")
        validated.append(
            ScoreEventEvidence(
                timestamp,
                index,
                event_type,
                expected_value,
                f"scoring/events.jsonl#event-{index}",
            )
        )
    return tuple(validated)


def _read_json_lines(
    run_directory: Path | str,
    relative_path: str,
    *,
    deadline_check: Callable[[], None] | None,
) -> list[object]:
    validation, payload = read_regular_file_bytes(
        run_directory,
        relative_path,
        deadline_check=deadline_check,
    )
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise ScoreValidationError(f"{relative_path} is not safe scoring evidence")
    try:
        lines = payload.decode("utf-8").splitlines()
        return [json.loads(line) for line in lines]
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ScoreValidationError(f"{relative_path} is not valid JSONL") from error


def _rules_document(payload: bytes) -> dict[str, Any]:
    try:
        document = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ScoreValidationError("committed descent rules are invalid JSON") from error
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != 1
        or document.get("ruleset_id") != "descent_v1"
        or document.get("sample_interval_ns") != 50_000_000
    ):
        raise ScoreValidationError("committed descent rules are incompatible")
    rule_rows = document.get("rules")
    if not isinstance(rule_rows, list) or [row.get("id") for row in rule_rows] != list(
        _RULE_IDS
    ):
        raise ScoreValidationError("committed descent rule identities are incompatible")
    return document


def _independent_descent_score(
    ground_truth: tuple[GroundTruthEvidence, ...],
    rules: dict[str, Any],
) -> _IndependentDescentScore:
    if not ground_truth:
        raise ScoreValidationError("physical ground truth is empty")
    interval = rules["sample_interval_ns"]
    if any(
        current.sim_timestamp_ns != previous.sim_timestamp_ns + interval
        for previous, current in zip(ground_truth, ground_truth[1:])
    ):
        raise ScoreValidationError("physical ground truth is not contiguous")
    rows = rules["rules"]
    points = tuple(_finite_number(row.get("points"), "rules points") for row in rows)
    if len(points) != 4 or sum(points) != 100.0:
        raise ScoreValidationError("committed descent rule points are incompatible")
    first_airborne = next(
        (
            index
            for index, sample in enumerate(ground_truth)
            if sample.position_xyz[2] > float(rules["rise_height_m"])
        ),
        None,
    )
    first_contact = next(
        (
            index
            for index, sample in enumerate(ground_truth)
            if first_airborne is not None
            and index > first_airborne
            and sample.in_contact
        ),
        None,
    )
    airborne_then_contact = first_airborne is not None and first_contact is not None
    if not airborne_then_contact or first_contact is None:
        outcomes = (False, False, False, False)
    else:
        touchdown = ground_truth[first_contact]
        marker_x, marker_y = rules["marker_center_xy_m"]
        touchdown_precision = math.hypot(
            touchdown.position_xyz[0] - float(marker_x),
            touchdown.position_xyz[1] - float(marker_y),
        ) <= float(rules["touchdown_radius_m"])
        safe_preimpact = first_contact > 0 and max(
            0.0,
            -ground_truth[first_contact - 1].linear_velocity_xyz[2],
        ) <= float(rules["safe_preimpact_downward_speed_mps"])
        required_samples = int(rules["settled_duration_ns"]) // interval + 1
        contact_window = ground_truth[first_contact : first_contact + required_samples]
        stable_contact = len(contact_window) == required_samples and all(
            sample.in_contact
            and math.sqrt(sum(value * value for value in sample.linear_velocity_xyz))
            <= float(rules["settled_linear_speed_mps"])
            and _tilt_degrees(sample.orientation_xyzw)
            <= float(rules["settled_max_tilt_degrees"])
            for sample in contact_window
        )
        outcomes = (
            airborne_then_contact,
            touchdown_precision,
            safe_preimpact,
            stable_contact,
        )
    awarded = tuple(
        available if passed else 0.0
        for available, passed in zip(points, outcomes, strict=True)
    )
    return _IndependentDescentScore(
        (awarded[0], awarded[1], awarded[2], awarded[3]),
        sum(awarded),
        ground_truth[-1].sim_timestamp_ns,
    )


def _tilt_degrees(orientation: tuple[float, float, float, float]) -> float:
    x, y, _z, w = orientation
    norm_squared = sum(value * value for value in orientation)
    world_up_z = 1.0 - 2.0 * (x * x + y * y) / norm_squared
    return math.degrees(math.acos(max(-1.0, min(1.0, world_up_z))))


def validate_descent_score_outputs(
    run_directory: Path | str,
    *,
    run_id: str,
    rules_path: Path | str,
    deadline_check: Callable[[], None] | None = None,
    physical_evidence: PhysicalBagEvidence | None = None,
) -> ValidatedScoreMetadata:
    """Validate a complete score and return manifest-ready score metadata."""
    document = _read_json(
        run_directory, "scoring/result.json", deadline_check=deadline_check
    )
    if not isinstance(document, dict):
        raise ScoreValidationError("scoring/result.json must contain a JSON object")
    if document.get("run_id") != run_id:
        raise ScoreValidationError("score belongs to another run")
    if document.get("ruleset_id") != "descent_v1":
        raise ScoreValidationError("score ruleset is not descent_v1")
    if document.get("complete") is not True or document.get("diagnostic") is not None:
        raise ScoreValidationError("score is not complete")

    achieved = _finite_number(document.get("achieved_score"), "achieved_score")
    maximum = _finite_number(
        document.get("maximum_available_score"), "maximum_available_score"
    )
    if maximum != 100.0 or not 0.0 <= achieved <= maximum:
        raise ScoreValidationError("score must be within the descent_v1 range")

    try:
        rules_payload = Path(rules_path).read_bytes()
    except OSError as error:
        raise ScoreValidationError("committed rules could not be read") from error
    expected_checksum = hashlib.sha256(rules_payload).hexdigest()
    checksum = document.get("scoring_checksum")
    if checksum != expected_checksum:
        raise ScoreValidationError("scoring checksum does not match committed rules")

    evidence = document.get("evidence_paths")
    if evidence != list(_EVIDENCE_PATHS):
        raise ScoreValidationError("score evidence paths are incomplete or invalid")
    rosbag = validate_tree(run_directory, "rosbag", deadline_check=deadline_check)
    if rosbag.status is not ValidationStatus.VALID:
        raise ScoreValidationError("score references an unsafe or missing rosbag")

    awarded = _validate_rule_results(document.get("rule_results"), achieved)
    persisted_events = _validate_events(
        run_directory,
        run_id=run_id,
        awarded=awarded,
        achieved_score=achieved,
        deadline_check=deadline_check,
    )
    if physical_evidence is not None:
        before = validate_tree(run_directory, "rosbag", deadline_check=deadline_check)
        if (
            before.status is not ValidationStatus.VALID
            or before.sha256 != physical_evidence.bag_sha256
        ):
            raise ScoreValidationError("physical ground truth bag digest changed")
        computed = _independent_descent_score(
            physical_evidence.ground_truth,
            _rules_document(rules_payload),
        )
        expected_events = tuple(
            ScoreEventEvidence(
                computed.sim_timestamp_ns,
                index,
                event_type,
                value,
                f"scoring/events.jsonl#event-{index}",
            )
            for index, (event_type, value) in enumerate(
                zip(
                    _EVENT_TYPES,
                    (*computed.awarded_points, computed.achieved_score),
                    strict=True,
                )
            )
        )
        if (
            awarded != computed.awarded_points
            or achieved != computed.achieved_score
            or persisted_events != expected_events
            or physical_evidence.score_events != expected_events
        ):
            raise ScoreValidationError(
                "persisted score does not match independently evaluated ground truth"
            )
        after = validate_tree(run_directory, "rosbag", deadline_check=deadline_check)
        if (
            after.status is not ValidationStatus.VALID
            or after.sha256 != physical_evidence.bag_sha256
        ):
            raise ScoreValidationError("physical ground truth bag digest changed")
    return ValidatedScoreMetadata(achieved, maximum, checksum, tuple(evidence))


def validate_score_outputs(
    run_directory: Path | str,
    *,
    run_id: str,
    rules_path: Path | str,
    deadline_check: Callable[[], None] | None = None,
    physical_evidence: PhysicalBagEvidence | None = None,
):
    """Dispatch score validation using the persisted result ruleset identity."""
    document = _read_json(
        run_directory, "scoring/result.json", deadline_check=deadline_check
    )
    ruleset_id = document.get("ruleset_id") if isinstance(document, dict) else None
    if ruleset_id == "descent_v1":
        return validate_descent_score_outputs(
            run_directory,
            run_id=run_id,
            rules_path=rules_path,
            deadline_check=deadline_check,
            physical_evidence=physical_evidence,
        )
    if ruleset_id == "competition_v1":
        from .competition_score_validation import validate_competition_score_outputs

        return validate_competition_score_outputs(
            run_directory,
            run_id=run_id,
            rules_path=rules_path,
            deadline_check=deadline_check,
            physical_evidence=physical_evidence,
        )
    raise ScoreValidationError("score ruleset is unsupported")


__all__ = [
    "ScoreValidationError",
    "ValidatedScoreMetadata",
    "validate_descent_score_outputs",
    "validate_score_outputs",
]
