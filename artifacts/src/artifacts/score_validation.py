"""Validation of completed ``descent_v1`` scoring evidence."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

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
) -> None:
    payload = _read_json_lines(
        run_directory, "scoring/events.jsonl", deadline_check=deadline_check
    )
    if len(payload) != 5:
        raise ScoreValidationError("events.jsonl must contain exactly five events")
    expected_values = (*awarded, achieved_score)
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


def validate_descent_score_outputs(
    run_directory: Path | str,
    *,
    run_id: str,
    rules_path: Path | str,
    deadline_check: Callable[[], None] | None = None,
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
        expected_checksum = hashlib.sha256(Path(rules_path).read_bytes()).hexdigest()
    except OSError as error:
        raise ScoreValidationError("committed rules could not be read") from error
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
    _validate_events(
        run_directory,
        run_id=run_id,
        awarded=awarded,
        achieved_score=achieved,
        deadline_check=deadline_check,
    )
    return ValidatedScoreMetadata(achieved, maximum, checksum, tuple(evidence))


__all__ = [
    "ScoreValidationError",
    "ValidatedScoreMetadata",
    "validate_descent_score_outputs",
]
