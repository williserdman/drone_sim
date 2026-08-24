"""Pure ``descent_v1`` policy over contiguous Gazebo ground truth."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any
from uuid import UUID


_RULE_IDS = (
    "airborne_then_contact",
    "touchdown_precision",
    "safe_preimpact_speed",
    "stable_contact",
)
_EVENT_TYPES = (
    "descent.airborne_then_contact",
    "descent.touchdown_precision",
    "descent.safe_preimpact_speed",
    "descent.stable_contact",
)


def _canonical_run_id(value: object) -> str:
    if not isinstance(value, str):
        raise ValueError("run_id must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (TypeError, ValueError, AttributeError) as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise ValueError("run_id must be a canonical UUID")
    return value


def _finite_tuple(value: object, *, name: str, length: int) -> tuple[float, ...]:
    if (
        not isinstance(value, tuple)
        or len(value) != length
        or any(
            isinstance(item, bool)
            or not isinstance(item, (int, float))
            or not math.isfinite(item)
            for item in value
        )
    ):
        raise ValueError(f"{name} must be a finite {length}-tuple")
    return tuple(float(item) for item in value)


@dataclass(frozen=True)
class GroundTruthSample:
    run_id: str
    sim_timestamp_ns: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    in_contact: bool

    def __post_init__(self) -> None:
        object.__setattr__(self, "run_id", _canonical_run_id(self.run_id))
        if (
            not isinstance(self.sim_timestamp_ns, int)
            or isinstance(self.sim_timestamp_ns, bool)
            or self.sim_timestamp_ns < 0
        ):
            raise ValueError("sim_timestamp_ns must be a nonnegative integer")
        object.__setattr__(
            self, "position_xyz", _finite_tuple(self.position_xyz, name="position_xyz", length=3)
        )
        orientation = _finite_tuple(
            self.orientation_xyzw, name="orientation_xyzw", length=4
        )
        if math.sqrt(sum(value * value for value in orientation)) == 0.0:
            raise ValueError("orientation_xyzw must have nonzero norm")
        object.__setattr__(self, "orientation_xyzw", orientation)
        object.__setattr__(
            self,
            "linear_velocity_xyz",
            _finite_tuple(self.linear_velocity_xyz, name="linear_velocity_xyz", length=3),
        )
        object.__setattr__(
            self,
            "angular_velocity_xyz",
            _finite_tuple(self.angular_velocity_xyz, name="angular_velocity_xyz", length=3),
        )
        if not isinstance(self.in_contact, bool):
            raise ValueError("in_contact must be a boolean")


@dataclass(frozen=True)
class DescentRules:
    ruleset_id: str
    scoring_checksum: str
    sample_interval_ns: int
    marker_center_xy_m: tuple[float, float]
    rise_height_m: float
    touchdown_radius_m: float
    safe_preimpact_downward_speed_mps: float
    settled_duration_ns: int
    settled_linear_speed_mps: float
    settled_max_tilt_degrees: float
    points: tuple[float, float, float, float]

    @property
    def maximum_available_score(self) -> float:
        return sum(self.points)


def _positive_number(document: dict[str, Any], name: str) -> float:
    value = document.get(name)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError(f"{name} must be a positive finite number")
    return float(value)


def load_descent_rules(path: Path | str) -> DescentRules:
    """Load and checksum the repository-owned, frozen scoring policy."""
    payload = Path(path).read_bytes()
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("descent rules must be UTF-8 JSON") from error
    if not isinstance(document, dict):
        raise ValueError("descent rules must be an object")
    if document.get("schema_version") != 1 or document.get("ruleset_id") != "descent_v1":
        raise ValueError("unsupported descent ruleset")
    interval = document.get("sample_interval_ns")
    settled_duration = document.get("settled_duration_ns")
    if interval != 50_000_000 or isinstance(interval, bool):
        raise ValueError("descent_v1 sample interval must be 50000000 ns")
    if (
        not isinstance(settled_duration, int)
        or isinstance(settled_duration, bool)
        or settled_duration < interval
        or settled_duration % interval
    ):
        raise ValueError("settled duration must be a positive sample-grid duration")
    marker = document.get("marker_center_xy_m")
    if not isinstance(marker, list):
        raise ValueError("marker_center_xy_m must contain two finite values")
    marker_tuple = _finite_tuple(tuple(marker), name="marker_center_xy_m", length=2)
    rule_rows = document.get("rules")
    if not isinstance(rule_rows, list) or len(rule_rows) != 4:
        raise ValueError("descent_v1 requires exactly four rules")
    points: list[float] = []
    for expected_id, row in zip(_RULE_IDS, rule_rows, strict=True):
        if not isinstance(row, dict) or row.get("id") != expected_id:
            raise ValueError("descent_v1 rule identities or order changed")
        points.append(_positive_number(row, "points"))
    if sum(points) != 100.0:
        raise ValueError("descent_v1 maximum score must be 100")
    return DescentRules(
        ruleset_id="descent_v1",
        scoring_checksum=hashlib.sha256(payload).hexdigest(),
        sample_interval_ns=interval,
        marker_center_xy_m=(marker_tuple[0], marker_tuple[1]),
        rise_height_m=_positive_number(document, "rise_height_m"),
        touchdown_radius_m=_positive_number(document, "touchdown_radius_m"),
        safe_preimpact_downward_speed_mps=_positive_number(
            document, "safe_preimpact_downward_speed_mps"
        ),
        settled_duration_ns=settled_duration,
        settled_linear_speed_mps=_positive_number(document, "settled_linear_speed_mps"),
        settled_max_tilt_degrees=_positive_number(
            document, "settled_max_tilt_degrees"
        ),
        points=(points[0], points[1], points[2], points[3]),
    )


@dataclass(frozen=True)
class RuleResult:
    rule_id: str
    passed: bool
    awarded_points: float
    available_points: float
    evidence_ref: str

    def to_document(self) -> dict[str, object]:
        return {
            "rule_id": self.rule_id,
            "passed": self.passed,
            "awarded_points": self.awarded_points,
            "available_points": self.available_points,
            "evidence_ref": self.evidence_ref,
        }


@dataclass(frozen=True)
class ScoreEvent:
    run_id: str
    sim_timestamp_ns: int
    event_id: int
    event_type: str
    value: float
    evidence_ref: str

    def to_document(self) -> dict[str, object]:
        return {
            "run_id": self.run_id,
            "sim_timestamp_ns": self.sim_timestamp_ns,
            "event_id": self.event_id,
            "event_type": self.event_type,
            "value": self.value,
            "evidence_ref": self.evidence_ref,
        }


@dataclass(frozen=True)
class ScoreResult:
    run_id: str
    ruleset_id: str
    complete: bool
    achieved_score: float
    maximum_available_score: float
    scoring_checksum: str
    evidence_paths: tuple[str, ...]
    rule_results: tuple[RuleResult, ...]
    events: tuple[ScoreEvent, ...]
    diagnostic: str | None

    def result_document(self) -> dict[str, object]:
        return {
            "run_id": self.run_id,
            "ruleset_id": self.ruleset_id,
            "complete": self.complete,
            "achieved_score": self.achieved_score,
            "maximum_available_score": self.maximum_available_score,
            "scoring_checksum": self.scoring_checksum,
            "evidence_paths": list(self.evidence_paths),
            "rule_results": [result.to_document() for result in self.rule_results],
            "diagnostic": self.diagnostic,
        }

    def finished_status(self) -> dict[str, object]:
        return {
            "run_id": self.run_id,
            "complete": self.complete,
            "ruleset_id": self.ruleset_id,
            "achieved_score": self.achieved_score,
            "maximum_available_score": self.maximum_available_score,
            "scoring_checksum": self.scoring_checksum,
            "result_path": "scoring/result.json",
        }


def _tilt_degrees(orientation: tuple[float, float, float, float]) -> float:
    x, y, _z, w = orientation
    norm_squared = sum(value * value for value in orientation)
    world_up_z = 1.0 - 2.0 * (x * x + y * y) / norm_squared
    return math.degrees(math.acos(max(-1.0, min(1.0, world_up_z))))


class DescentScorer:
    """Accept immutable samples once and fail closed on the first discontinuity."""

    def __init__(
        self,
        run_id: str,
        rules: DescentRules,
        *,
        expected_ground_truth_samples: int,
    ) -> None:
        self.run_id = _canonical_run_id(run_id)
        if not isinstance(rules, DescentRules):
            raise TypeError("rules must be DescentRules")
        if (
            not isinstance(expected_ground_truth_samples, int)
            or isinstance(expected_ground_truth_samples, bool)
            or expected_ground_truth_samples <= 0
        ):
            raise ValueError("expected_ground_truth_samples must be a positive integer")
        self.rules = rules
        self.expected_ground_truth_samples = expected_ground_truth_samples
        self._samples: list[GroundTruthSample] = []
        self._diagnostic: str | None = None
        self._result: ScoreResult | None = None

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
        if self._samples and (
            sample.sim_timestamp_ns
            != self._samples[-1].sim_timestamp_ns + self.rules.sample_interval_ns
        ):
            self._diagnostic = "ground_truth_timestamp_gap"
            return
        self._samples.append(sample)

    def _evaluated_rules(self) -> tuple[bool, bool, bool, bool]:
        first_contact = next(
            (index for index, sample in enumerate(self._samples) if sample.in_contact),
            None,
        )
        airborne_then_contact = first_contact is not None and any(
            sample.position_xyz[2] > self.rules.rise_height_m
            for sample in self._samples[:first_contact]
        )
        if not airborne_then_contact or first_contact is None:
            return False, False, False, False

        touchdown = self._samples[first_contact]
        dx = touchdown.position_xyz[0] - self.rules.marker_center_xy_m[0]
        dy = touchdown.position_xyz[1] - self.rules.marker_center_xy_m[1]
        touchdown_precision = math.hypot(dx, dy) <= self.rules.touchdown_radius_m

        safe_preimpact = first_contact > 0 and max(
            0.0, -self._samples[first_contact - 1].linear_velocity_xyz[2]
        ) <= self.rules.safe_preimpact_downward_speed_mps

        required_samples = self.rules.settled_duration_ns // self.rules.sample_interval_ns + 1
        contact_window = self._samples[first_contact : first_contact + required_samples]
        stable_contact = len(contact_window) == required_samples and all(
            sample.in_contact
            and math.sqrt(sum(value * value for value in sample.linear_velocity_xyz))
            <= self.rules.settled_linear_speed_mps
            and _tilt_degrees(sample.orientation_xyzw)
            <= self.rules.settled_max_tilt_degrees
            for sample in contact_window
        )
        return airborne_then_contact, touchdown_precision, safe_preimpact, stable_contact

    def finalize(self) -> ScoreResult:
        if self._result is not None:
            return self._result
        diagnostic = self._diagnostic
        if diagnostic is None and len(self._samples) != self.expected_ground_truth_samples:
            diagnostic = "ground_truth_sample_count_mismatch"
        complete = diagnostic is None
        outcomes = self._evaluated_rules() if complete else (False, False, False, False)
        rule_results = tuple(
            RuleResult(
                rule_id=rule_id,
                passed=passed,
                awarded_points=points if passed else 0.0,
                available_points=points,
                evidence_ref=f"rosbag#/simulation/ground_truth:{rule_id}",
            )
            for rule_id, points, passed in zip(
                _RULE_IDS, self.rules.points, outcomes, strict=True
            )
        )
        achieved = sum(result.awarded_points for result in rule_results)
        timestamp = self._samples[-1].sim_timestamp_ns if self._samples else 0
        events = tuple(
            ScoreEvent(
                run_id=self.run_id,
                sim_timestamp_ns=timestamp,
                event_id=index,
                event_type=event_type,
                value=result.awarded_points,
                evidence_ref=f"scoring/events.jsonl#event-{index}",
            )
            for index, (event_type, result) in enumerate(
                zip(_EVENT_TYPES, rule_results, strict=True)
            )
        ) + (
            ScoreEvent(
                run_id=self.run_id,
                sim_timestamp_ns=timestamp,
                event_id=4,
                event_type="score.finalized",
                value=achieved,
                evidence_ref="scoring/events.jsonl#event-4",
            ),
        )
        evidence_paths = tuple(event.evidence_ref for event in events) + (
            "rosbag#/simulation/ground_truth",
        )
        self._result = ScoreResult(
            run_id=self.run_id,
            ruleset_id=self.rules.ruleset_id,
            complete=complete,
            achieved_score=achieved,
            maximum_available_score=self.rules.maximum_available_score,
            scoring_checksum=self.rules.scoring_checksum,
            evidence_paths=evidence_paths,
            rule_results=rule_results,
            events=events,
            diagnostic=diagnostic,
        )
        return self._result


__all__ = [
    "DescentRules",
    "DescentScorer",
    "GroundTruthSample",
    "RuleResult",
    "ScoreEvent",
    "ScoreResult",
    "load_descent_rules",
]
