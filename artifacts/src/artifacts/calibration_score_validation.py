"""Independent validation of completed ``calibration_v1`` physical scores."""

from __future__ import annotations

from collections.abc import Callable
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from ._adapters.rosbag import PhysicalBagEvidence, ScoreEventEvidence
from .score_validation import (
    ScoreValidationError,
    ValidatedScoreMetadata,
    _finite_number,
    _read_json,
    _read_json_lines,
)
from .validation import ValidationStatus, validate_tree


_RULE_IDS = ("airborne_then_contact", "safe_preimpact_speed", "stable_contact")
_POINTS = (20.0, 40.0, 40.0)
_EVENT_TYPES = tuple(f"calibration.{rule_id}" for rule_id in _RULE_IDS) + ("score.finalized",)
_EVIDENCE_PATHS = tuple(f"scoring/events.jsonl#event-{index}" for index in range(4)) + ("rosbag#/simulation/ground_truth",)


def _rules(payload: bytes) -> dict[str, Any]:
    try:
        document = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ScoreValidationError("committed calibration rules are invalid JSON") from error
    if not isinstance(document, dict) or document.get("schema_version") != 1 or document.get("ruleset_id") != "calibration_v1" or document.get("sample_interval_ns") != 50_000_000:
        raise ScoreValidationError("committed calibration rules are incompatible")
    rows = document.get("rules")
    if not isinstance(rows, list) or tuple(row.get("id") for row in rows if isinstance(row, dict)) != _RULE_IDS or tuple(_finite_number(row.get("points"), "rules points") for row in rows) != _POINTS:
        raise ScoreValidationError("committed calibration rules are incompatible")
    return document


def _tilt(orientation: tuple[float, float, float, float]) -> float:
    x, y, _z, _w = orientation
    norm = sum(value * value for value in orientation)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y) / norm))))


def _recompute(evidence: PhysicalBagEvidence, rules: dict[str, Any]) -> tuple[tuple[float, float, float], int]:
    samples = evidence.ground_truth
    interval = rules["sample_interval_ns"]
    if not samples or any(current.sim_timestamp_ns != previous.sim_timestamp_ns + interval for previous, current in zip(samples, samples[1:])):
        raise ScoreValidationError("calibration physical ground truth is missing or not contiguous")
    airborne = next((index for index, sample in enumerate(samples) if sample.position_xyz[2] > float(rules["rise_height_m"])), None)
    contact = next((index for index, sample in enumerate(samples) if airborne is not None and index > airborne and sample.in_contact), None)
    if airborne is None or contact is None:
        outcomes = (False, False, False)
    else:
        safe = contact > 0 and max(0.0, -samples[contact - 1].linear_velocity_xyz[2]) <= float(rules["safe_preimpact_downward_speed_mps"])
        required = int(rules["settled_duration_ns"]) // interval + 1
        window = samples[contact:contact + required]
        stable = len(window) == required and all(sample.in_contact and math.sqrt(sum(value * value for value in sample.linear_velocity_xyz)) <= float(rules["settled_linear_speed_mps"]) and _tilt(sample.orientation_xyzw) <= float(rules["settled_max_tilt_degrees"]) for sample in window)
        outcomes = (True, safe, stable)
    return tuple(points if passed else 0.0 for points, passed in zip(_POINTS, outcomes, strict=True)), samples[-1].sim_timestamp_ns


def validate_calibration_score_outputs(run_directory: Path | str, *, run_id: str, rules_path: Path | str, deadline_check: Callable[[], None] | None = None, physical_evidence: PhysicalBagEvidence | None = None) -> ValidatedScoreMetadata:
    document = _read_json(run_directory, "scoring/result.json", deadline_check=deadline_check)
    if not isinstance(document, dict) or document.get("run_id") != run_id or document.get("ruleset_id") != "calibration_v1" or document.get("complete") is not True or document.get("diagnostic") is not None:
        raise ScoreValidationError("calibration score metadata is invalid")
    achieved = _finite_number(document.get("achieved_score"), "achieved_score")
    if _finite_number(document.get("maximum_available_score"), "maximum_available_score") != 100.0:
        raise ScoreValidationError("calibration maximum score is invalid")
    try:
        payload = Path(rules_path).read_bytes()
    except OSError as error:
        raise ScoreValidationError("committed rules could not be read") from error
    checksum = hashlib.sha256(payload).hexdigest()
    rules_document = _rules(payload)
    if document.get("scoring_checksum") != checksum or document.get("evidence_paths") != list(_EVIDENCE_PATHS):
        raise ScoreValidationError("calibration score evidence metadata is invalid")
    rows = document.get("rule_results")
    if not isinstance(rows, list) or len(rows) != 3:
        raise ScoreValidationError("calibration rule results are invalid")
    awarded = []
    for index, (row, rule_id, points) in enumerate(zip(rows, _RULE_IDS, _POINTS, strict=True)):
        if not isinstance(row, dict) or row.get("rule_id") != rule_id or type(row.get("passed")) is not bool or _finite_number(row.get("available_points"), "available_points") != points or row.get("evidence_ref") != f"rosbag#/simulation/ground_truth:{rule_id}":
            raise ScoreValidationError("calibration rule results are invalid")
        value = _finite_number(row.get("awarded_points"), "awarded_points")
        if value != (points if row["passed"] else 0.0):
            raise ScoreValidationError("calibration rule points are inconsistent")
        awarded.append(value)
    if sum(awarded) != achieved:
        raise ScoreValidationError("calibration achieved score is inconsistent")
    raw_events = _read_json_lines(run_directory, "scoring/events.jsonl", deadline_check=deadline_check)
    if len(raw_events) != 4:
        raise ScoreValidationError("calibration score events are invalid")
    expected_values = (*awarded, achieved)
    events = []
    for index, (row, event_type, value) in enumerate(zip(raw_events, _EVENT_TYPES, expected_values, strict=True)):
        if not isinstance(row, dict) or row.get("run_id") != run_id or row.get("event_id") != index or row.get("event_type") != event_type or row.get("evidence_ref") != f"scoring/events.jsonl#event-{index}" or _finite_number(row.get("value"), "event value") != value:
            raise ScoreValidationError("calibration score events are invalid")
        timestamp = row.get("sim_timestamp_ns")
        if type(timestamp) is not int or timestamp < 0:
            raise ScoreValidationError("calibration score event timestamp is invalid")
        events.append(ScoreEventEvidence(timestamp, index, event_type, value, row["evidence_ref"]))
    bag = validate_tree(run_directory, "rosbag", deadline_check=deadline_check)
    if bag.status is not ValidationStatus.VALID:
        raise ScoreValidationError("score references an unsafe or missing rosbag")
    if physical_evidence is not None:
        computed, timestamp = _recompute(physical_evidence, rules_document)
        expected_events = tuple(ScoreEventEvidence(timestamp, index, event_type, value, f"scoring/events.jsonl#event-{index}") for index, (event_type, value) in enumerate(zip(_EVENT_TYPES, (*computed, sum(computed)), strict=True)))
        if tuple(awarded) != computed or achieved != sum(computed) or tuple(events) != expected_events or physical_evidence.score_events != expected_events or bag.sha256 != physical_evidence.bag_sha256:
            raise ScoreValidationError("persisted calibration score does not match independently evaluated ground truth")
    return ValidatedScoreMetadata(achieved, 100.0, checksum, tuple(document["evidence_paths"]))


__all__ = ["validate_calibration_score_outputs"]
