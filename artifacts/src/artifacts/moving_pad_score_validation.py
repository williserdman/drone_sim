"""Independent validation of completed ``moving_pad_v1`` scoring evidence."""

from __future__ import annotations

from collections.abc import Callable
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from ._adapters.rosbag import (
    GroundTruthEvidence,
    LandingPadStateEvidence,
    PhysicalBagEvidence,
    ScoreEventEvidence,
)
from .score_validation import ScoreValidationError, ValidatedScoreMetadata
from .validation import ValidationStatus, read_regular_file_bytes, validate_tree


_SAMPLE_INTERVAL_NS = 50_000_000
_ABOARD_DURATION_NS = 2_000_000_000
_MARKER_ID = 7
_DECK_HALF_EXTENT_M = 1.5
_AVAILABLE_POINTS = 100.0
_ROS_EVIDENCE_PATHS = (
    "rosbag#/simulation/ground_truth",
    "rosbag#/simulation/landing_pad_state",
    "rosbag#/simulation/mission_events",
)


def _read_json(
    run_directory: Path | str,
    relative_path: str,
    *,
    deadline_check: Callable[[], None] | None,
) -> object:
    validation, payload = read_regular_file_bytes(
        run_directory, relative_path, deadline_check=deadline_check
    )
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise ScoreValidationError(f"{relative_path} is not safe scoring evidence")
    try:
        return json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ScoreValidationError(f"{relative_path} is not valid JSON") from error


def _read_events(
    run_directory: Path | str,
    *,
    deadline_check: Callable[[], None] | None,
) -> list[object]:
    validation, payload = read_regular_file_bytes(
        run_directory, "scoring/events.jsonl", deadline_check=deadline_check
    )
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise ScoreValidationError("scoring/events.jsonl is not safe scoring evidence")
    try:
        return [json.loads(line) for line in payload.decode("utf-8").splitlines()]
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ScoreValidationError("scoring/events.jsonl is not valid JSONL") from error


def _finite_number(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScoreValidationError(f"{field} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ScoreValidationError(f"{field} must be a finite number")
    return number


def _load_rules(path: Path | str) -> tuple[str, float]:
    try:
        payload = Path(path).read_bytes()
        document = json.loads(payload.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ScoreValidationError("moving-pad rules are unreadable") from error
    expected = {
        "schema_version": 1,
        "ruleset_id": "moving_pad_v1",
        "sample_interval_ns": _SAMPLE_INTERVAL_NS,
        "marker_id": _MARKER_ID,
        "deck_half_extent_m": _DECK_HALF_EXTENT_M,
        "aboard_duration_ns": _ABOARD_DURATION_NS,
        "rules": [{"id": "physical_landing", "points": _AVAILABLE_POINTS}],
    }
    if document != expected:
        raise ScoreValidationError("moving-pad ruleset is incompatible")
    return hashlib.sha256(payload).hexdigest(), _AVAILABLE_POINTS


def _pad_relative_position(
    vehicle: GroundTruthEvidence, pad: LandingPadStateEvidence
) -> tuple[float, float, float]:
    rx, ry, rz = (
        vehicle.position_xyz[index] - pad.position_xyz[index] for index in range(3)
    )
    qx, qy, qz, qw = pad.orientation_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    tx = -2.0 * (qy * rz - qz * ry)
    ty = -2.0 * (qz * rx - qx * rz)
    tz = -2.0 * (qx * ry - qy * rx)
    return (
        rx + qw * tx - qy * tz + qz * ty,
        ry + qw * ty - qz * tx + qx * tz,
        rz + qw * tz - qx * ty + qy * tx,
    )


def _relative_speed(
    vehicle: GroundTruthEvidence, pad: LandingPadStateEvidence
) -> float:
    rx, ry, rz = (
        vehicle.position_xyz[index] - pad.position_xyz[index] for index in range(3)
    )
    wx, wy, wz = pad.angular_velocity_xyz
    point_velocity = (
        pad.linear_velocity_xyz[0] + wy * rz - wz * ry,
        pad.linear_velocity_xyz[1] + wz * rx - wx * rz,
        pad.linear_velocity_xyz[2] + wx * ry - wy * rx,
    )
    return math.sqrt(
        sum(
            (vehicle.linear_velocity_xyz[index] - point_velocity[index]) ** 2
            for index in range(3)
        )
    )


def _recompute_events(
    physical_evidence: PhysicalBagEvidence,
) -> tuple[bool, tuple[ScoreEventEvidence, ...]]:
    vehicles = physical_evidence.ground_truth
    pads = physical_evidence.landing_pad_states
    if not vehicles or len(vehicles) != len(pads):
        raise ScoreValidationError("moving-pad vehicle and pad truth counts differ")
    timestamps = tuple(sample.sim_timestamp_ns for sample in vehicles)
    if tuple(sample.sim_timestamp_ns for sample in pads) != timestamps or any(
        current - previous != _SAMPLE_INTERVAL_NS
        for previous, current in zip(timestamps, timestamps[1:])
    ):
        raise ScoreValidationError("moving-pad vehicle and pad truth timestamps are not exact")
    if any(pad.marker_id != _MARKER_ID for pad in pads):
        raise ScoreValidationError("moving-pad truth has the wrong marker")

    mission = physical_evidence.mission_events
    if (
        len(mission) != 2
        or tuple(event.event_id for event in mission) != (0, 1)
        or tuple(event.phase for event in mission) != ("MOVING_PAD", "MOVING_PAD")
        or tuple(event.state for event in mission) != ("ARMED", "DISARMED")
        or mission[0].sim_timestamp_ns > mission[1].sim_timestamp_ns
        or mission[1].sim_timestamp_ns not in set(timestamps)
    ):
        raise ScoreValidationError("moving-pad ARMED/DISARMED evidence is invalid")
    disarmed_ns = mission[1].sim_timestamp_ns

    touchdown: tuple[GroundTruthEvidence, LandingPadStateEvidence] | None = None
    airborne_seen = False
    for vehicle, pad in zip(vehicles, pads, strict=True):
        if not vehicle.in_contact:
            airborne_seen = True
        elif airborne_seen and pad.vehicle_in_contact:
            touchdown = (vehicle, pad)
            break

    def aboard(vehicle: GroundTruthEvidence, pad: LandingPadStateEvidence) -> bool:
        x, y, z = _pad_relative_position(vehicle, pad)
        return (
            vehicle.in_contact
            and pad.vehicle_in_contact
            and abs(x) <= _DECK_HALF_EXTENT_M
            and abs(y) <= _DECK_HALF_EXTENT_M
            and z >= 0.0
        )

    passed = False
    if touchdown is not None and touchdown[0].sim_timestamp_ns <= disarmed_ns:
        required = _ABOARD_DURATION_NS // _SAMPLE_INTERVAL_NS + 1
        consecutive = 0
        for vehicle, pad in zip(vehicles, pads, strict=True):
            if vehicle.sim_timestamp_ns < disarmed_ns:
                continue
            consecutive = consecutive + 1 if aboard(vehicle, pad) else 0
            if consecutive >= required:
                passed = True
                break

    awarded = _AVAILABLE_POINTS if passed else 0.0
    last_timestamp = timestamps[-1]
    specs: list[tuple[int, str, float]] = [
        (last_timestamp, "moving_pad.physical_landing", awarded)
    ]
    if touchdown is not None:
        vehicle, pad = touchdown
        x, y, _z = _pad_relative_position(vehicle, pad)
        specs.extend(
            (
                (vehicle.sim_timestamp_ns, "moving_pad.touchdown_offset_m", math.hypot(x, y)),
                (
                    vehicle.sim_timestamp_ns,
                    "moving_pad.touchdown_relative_velocity_mps",
                    _relative_speed(vehicle, pad),
                ),
            )
        )
    specs.append((last_timestamp, "score.finalized", awarded))
    return passed, tuple(
        ScoreEventEvidence(
            timestamp,
            index,
            event_type,
            value,
            f"scoring/events.jsonl#event-{index}",
        )
        for index, (timestamp, event_type, value) in enumerate(specs)
    )


def _events_match(
    actual: tuple[ScoreEventEvidence, ...], expected: tuple[ScoreEventEvidence, ...]
) -> bool:
    return len(actual) == len(expected) and all(
        left.sim_timestamp_ns == right.sim_timestamp_ns
        and left.event_id == right.event_id
        and left.event_type == right.event_type
        and math.isclose(left.value, right.value, rel_tol=0.0, abs_tol=1e-12)
        and left.evidence_ref == right.evidence_ref
        for left, right in zip(actual, expected, strict=True)
    )


def _validate_logical_events(
    events: tuple[ScoreEventEvidence, ...], achieved: float
) -> None:
    event_types = tuple(event.event_type for event in events)
    if event_types not in {
        ("moving_pad.physical_landing", "score.finalized"),
        (
            "moving_pad.physical_landing",
            "moving_pad.touchdown_offset_m",
            "moving_pad.touchdown_relative_velocity_mps",
            "score.finalized",
        ),
    }:
        raise ScoreValidationError("moving-pad score events are not logically ordered")
    if (
        events[0].value != achieved
        or events[-1].value != achieved
        or events[0].sim_timestamp_ns != events[-1].sim_timestamp_ns
    ):
        raise ScoreValidationError("moving-pad score events are logically inconsistent")
    if achieved == _AVAILABLE_POINTS and len(events) != 4:
        raise ScoreValidationError(
            "moving-pad physical pass requires touchdown diagnostics"
        )
    if len(events) == 4 and (
        events[1].value < 0.0
        or events[2].value < 0.0
        or events[1].sim_timestamp_ns != events[2].sim_timestamp_ns
        or events[1].sim_timestamp_ns > events[-1].sim_timestamp_ns
    ):
        raise ScoreValidationError("moving-pad touchdown events are logically inconsistent")


def validate_moving_pad_score_outputs(
    run_directory: Path | str,
    *,
    run_id: str,
    rules_path: Path | str,
    deadline_check: Callable[[], None] | None = None,
    physical_evidence: PhysicalBagEvidence | None = None,
) -> ValidatedScoreMetadata:
    """Recompute the moving-pad result from recorded vehicle, pad, and disarm truth."""
    checksum, maximum = _load_rules(rules_path)
    document = _read_json(
        run_directory, "scoring/result.json", deadline_check=deadline_check
    )
    required_fields = {
        "run_id",
        "ruleset_id",
        "complete",
        "achieved_score",
        "maximum_available_score",
        "scoring_checksum",
        "evidence_paths",
        "rule_results",
        "diagnostic",
    }
    if not isinstance(document, dict) or set(document) != required_fields:
        raise ScoreValidationError("moving-pad score result schema is invalid")
    achieved = _finite_number(document["achieved_score"], "achieved_score")
    if (
        document["run_id"] != run_id
        or document["ruleset_id"] != "moving_pad_v1"
        or document["complete"] is not True
        or document["diagnostic"] is not None
        or _finite_number(document["maximum_available_score"], "maximum_available_score")
        != maximum
        or document["scoring_checksum"] != checksum
    ):
        raise ScoreValidationError("moving-pad completed score metadata is invalid")
    rules = document["rule_results"]
    if not isinstance(rules, list) or len(rules) != 1 or not isinstance(rules[0], dict):
        raise ScoreValidationError("moving-pad rule result is invalid")
    rule = rules[0]
    passed = rule.get("passed")
    if (
        set(rule)
        != {
            "rule_id",
            "passed",
            "awarded_points",
            "available_points",
            "evidence_ref",
        }
        or rule.get("rule_id") != "physical_landing"
        or type(passed) is not bool
        or _finite_number(rule.get("available_points"), "available_points") != maximum
        or _finite_number(rule.get("awarded_points"), "awarded_points")
        != (maximum if passed else 0.0)
        or rule.get("evidence_ref") != "rosbag#moving_pad:physical_landing"
        or achieved != (maximum if passed else 0.0)
    ):
        raise ScoreValidationError("moving-pad rule result is inconsistent")

    raw_events = _read_events(run_directory, deadline_check=deadline_check)
    persisted_events: list[ScoreEventEvidence] = []
    for index, event in enumerate(raw_events):
        if not isinstance(event, dict) or set(event) != {
            "run_id",
            "sim_timestamp_ns",
            "event_id",
            "event_type",
            "value",
            "evidence_ref",
        }:
            raise ScoreValidationError("moving-pad score event schema is invalid")
        timestamp = event["sim_timestamp_ns"]
        if (
            event["run_id"] != run_id
            or event["event_id"] != index
            or type(timestamp) is not int
            or timestamp < 0
            or not isinstance(event["event_type"], str)
            or event["evidence_ref"] != f"scoring/events.jsonl#event-{index}"
        ):
            raise ScoreValidationError("moving-pad score events are not contiguous")
        persisted_events.append(
            ScoreEventEvidence(
                timestamp,
                index,
                event["event_type"],
                _finite_number(event["value"], f"events[{index}].value"),
                event["evidence_ref"],
            )
        )
    evidence_paths = tuple(event.evidence_ref for event in persisted_events) + _ROS_EVIDENCE_PATHS
    if document["evidence_paths"] != list(evidence_paths):
        raise ScoreValidationError("moving-pad score evidence paths are invalid")
    persisted_tuple = tuple(persisted_events)
    _validate_logical_events(persisted_tuple, achieved)

    before = validate_tree(run_directory, "rosbag", deadline_check=deadline_check)
    if before.status is not ValidationStatus.VALID:
        raise ScoreValidationError("score references an unsafe or missing rosbag")
    if physical_evidence is None:
        return ValidatedScoreMetadata(achieved, maximum, checksum, evidence_paths)
    if before.sha256 != physical_evidence.bag_sha256:
        raise ScoreValidationError("score references an unsafe or changed rosbag")
    computed_passed, computed_events = _recompute_events(physical_evidence)
    if (
        passed is not computed_passed
        or achieved != (_AVAILABLE_POINTS if computed_passed else 0.0)
        or not _events_match(persisted_tuple, computed_events)
        or not _events_match(physical_evidence.score_events, computed_events)
    ):
        raise ScoreValidationError(
            "persisted moving-pad score does not match independently evaluated truth"
        )
    after = validate_tree(run_directory, "rosbag", deadline_check=deadline_check)
    if after.status is not ValidationStatus.VALID or after.sha256 != before.sha256:
        raise ScoreValidationError("moving-pad physical evidence changed during validation")
    return ValidatedScoreMetadata(achieved, maximum, checksum, evidence_paths)


__all__ = ["validate_moving_pad_score_outputs"]
