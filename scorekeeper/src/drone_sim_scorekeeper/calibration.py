"""Pure ``calibration_v1`` physical landing policy."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from artifacts.runtime_status import canonical_run_id

from .descent import GroundTruthSample, _positive_number, _tilt_degrees
from .models import RuleResult, ScoreEvent, ScoreResult


_RULE_IDS = ("airborne_then_contact", "safe_preimpact_speed", "stable_contact")
_EVENT_TYPES = tuple(f"calibration.{rule_id}" for rule_id in _RULE_IDS)


@dataclass(frozen=True)
class CalibrationRules:
    ruleset_id: str
    scoring_checksum: str
    sample_interval_ns: int
    rise_height_m: float
    safe_preimpact_downward_speed_mps: float
    settled_duration_ns: int
    settled_linear_speed_mps: float
    settled_max_tilt_degrees: float
    points: tuple[float, float, float]

    @property
    def maximum_available_score(self) -> float:
        return sum(self.points)


def load_calibration_rules(path: Path | str) -> CalibrationRules:
    payload = Path(path).read_bytes()
    try:
        document = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("calibration rules must be UTF-8 JSON") from error
    if not isinstance(document, dict) or document.get("schema_version") != 1 or document.get("ruleset_id") != "calibration_v1":
        raise ValueError("unsupported calibration ruleset")
    interval = document.get("sample_interval_ns")
    duration = document.get("settled_duration_ns")
    if interval != 50_000_000 or type(duration) is not int or duration < interval or duration % interval:
        raise ValueError("calibration timing must use the 50 ms sample grid")
    rows = document.get("rules")
    if not isinstance(rows, list) or len(rows) != 3 or tuple(row.get("id") for row in rows if isinstance(row, dict)) != _RULE_IDS:
        raise ValueError("calibration_v1 requires exactly three ordered rules")
    points = tuple(_positive_number(row, "points") for row in rows)
    if sum(points) != 100.0:
        raise ValueError("calibration_v1 maximum score must be 100")
    return CalibrationRules(
        "calibration_v1", hashlib.sha256(payload).hexdigest(), interval,
        _positive_number(document, "rise_height_m"),
        _positive_number(document, "safe_preimpact_downward_speed_mps"), duration,
        _positive_number(document, "settled_linear_speed_mps"),
        _positive_number(document, "settled_max_tilt_degrees"),
        (points[0], points[1], points[2]),
    )


class CalibrationScorer:
    def __init__(self, run_id: str, rules: CalibrationRules, *, expected_ground_truth_samples: int) -> None:
        self.run_id = canonical_run_id(run_id)
        if not isinstance(rules, CalibrationRules):
            raise TypeError("rules must be CalibrationRules")
        if type(expected_ground_truth_samples) is not int or expected_ground_truth_samples <= 0:
            raise ValueError("expected_ground_truth_samples must be a positive integer")
        self.rules = rules
        self.expected_ground_truth_samples = expected_ground_truth_samples
        self._samples: list[GroundTruthSample] = []
        self._diagnostic: str | None = None
        self._result: ScoreResult | None = None

    @property
    def last_sim_timestamp_ns(self) -> int | None:
        return self._samples[-1].sim_timestamp_ns if self._samples else None

    def fail(self, reason: str) -> None:
        if self._result is not None:
            raise RuntimeError("score has already been finalized")
        if type(reason) is not str or not reason:
            raise ValueError("score failure reason must be nonempty")
        if self._diagnostic is None:
            self._diagnostic = reason

    def accept(self, sample: GroundTruthSample) -> None:
        if self._result is not None:
            raise RuntimeError("score has already been finalized")
        if not isinstance(sample, GroundTruthSample):
            raise TypeError("sample must be GroundTruthSample")
        if sample.run_id != self.run_id:
            raise ValueError("ground truth belongs to another run")
        if self._diagnostic is not None:
            return
        if len(self._samples) >= self.expected_ground_truth_samples:
            self._diagnostic = "ground_truth_sample_overrun"
            return
        if self._samples:
            previous = self._samples[-1].sim_timestamp_ns
            expected = previous + self.rules.sample_interval_ns
            if sample.sim_timestamp_ns == previous:
                self._diagnostic = "ground_truth_timestamp_duplicate"
                return
            if sample.sim_timestamp_ns < expected:
                self._diagnostic = "ground_truth_timestamp_regression"
                return
            if sample.sim_timestamp_ns > expected:
                self._diagnostic = "ground_truth_timestamp_gap"
                return
        self._samples.append(sample)

    def _outcomes(self) -> tuple[bool, bool, bool]:
        airborne = next((i for i, sample in enumerate(self._samples) if sample.position_xyz[2] > self.rules.rise_height_m), None)
        contact = next((i for i, sample in enumerate(self._samples) if airborne is not None and i > airborne and sample.in_contact), None)
        if airborne is None or contact is None:
            return False, False, False
        safe = contact > 0 and max(0.0, -self._samples[contact - 1].linear_velocity_xyz[2]) <= self.rules.safe_preimpact_downward_speed_mps
        required = self.rules.settled_duration_ns // self.rules.sample_interval_ns + 1
        window = self._samples[contact:contact + required]
        stable = len(window) == required and all(
            sample.in_contact
            and math.sqrt(sum(value * value for value in sample.linear_velocity_xyz)) <= self.rules.settled_linear_speed_mps
            and _tilt_degrees(sample.orientation_xyzw) <= self.rules.settled_max_tilt_degrees
            for sample in window
        )
        return True, safe, stable

    def finalize(self) -> ScoreResult:
        if self._result is not None:
            return self._result
        diagnostic = self._diagnostic
        if diagnostic is None and len(self._samples) != self.expected_ground_truth_samples:
            diagnostic = "ground_truth_sample_count_mismatch"
        complete = diagnostic is None
        outcomes = self._outcomes() if complete else (False, False, False)
        results = tuple(RuleResult(rule_id, passed, points if passed else 0.0, points, f"rosbag#/simulation/ground_truth:{rule_id}") for rule_id, points, passed in zip(_RULE_IDS, self.rules.points, outcomes, strict=True))
        achieved = sum(row.awarded_points for row in results)
        timestamp = self._samples[-1].sim_timestamp_ns if self._samples else 0
        events = tuple(ScoreEvent(self.run_id, timestamp, index, event_type, row.awarded_points, f"scoring/events.jsonl#event-{index}") for index, (event_type, row) in enumerate(zip(_EVENT_TYPES, results, strict=True))) + (ScoreEvent(self.run_id, timestamp, 3, "score.finalized", achieved, "scoring/events.jsonl#event-3"),)
        self._result = ScoreResult(self.run_id, self.rules.ruleset_id, complete, achieved, self.rules.maximum_available_score, self.rules.scoring_checksum, tuple(event.evidence_ref for event in events) + ("rosbag#/simulation/ground_truth",), results, events, diagnostic)
        return self._result


__all__ = ["CalibrationRules", "CalibrationScorer", "load_calibration_rules"]
