"""Pure ``moving_pad_v1`` policy over paired vehicle and landing-pad truth."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from artifacts.runtime_status import canonical_run_id

from .descent import GroundTruthSample
from .models import RuleResult, ScoreEvent, ScoreResult


_RULE_ID = "physical_landing"


def _timestamp(value: object) -> int:
    if type(value) is not int or value < 0:
        raise ValueError("sim_timestamp_ns must be a nonnegative integer")
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
class LandingPadSample:
    run_id: str
    sim_timestamp_ns: int
    marker_id: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    vehicle_in_contact: bool

    def __post_init__(self) -> None:
        object.__setattr__(self, "run_id", canonical_run_id(self.run_id))
        _timestamp(self.sim_timestamp_ns)
        if type(self.marker_id) is not int or not 0 <= self.marker_id <= 4_294_967_295:
            raise ValueError("marker_id must be an unsigned 32-bit integer")
        object.__setattr__(
            self,
            "position_xyz",
            _finite_tuple(self.position_xyz, name="position_xyz", length=3),
        )
        orientation = _finite_tuple(
            self.orientation_xyzw, name="orientation_xyzw", length=4
        )
        if sum(value * value for value in orientation) == 0.0:
            raise ValueError("orientation_xyzw must have nonzero norm")
        object.__setattr__(self, "orientation_xyzw", orientation)
        object.__setattr__(
            self,
            "linear_velocity_xyz",
            _finite_tuple(
                self.linear_velocity_xyz, name="linear_velocity_xyz", length=3
            ),
        )
        object.__setattr__(
            self,
            "angular_velocity_xyz",
            _finite_tuple(
                self.angular_velocity_xyz, name="angular_velocity_xyz", length=3
            ),
        )
        if type(self.vehicle_in_contact) is not bool:
            raise ValueError("vehicle_in_contact must be boolean")


@dataclass(frozen=True)
class MovingPadRules:
    ruleset_id: str
    scoring_checksum: str
    sample_interval_ns: int
    marker_id: int
    deck_half_extent_m: float
    aboard_duration_ns: int
    points: float

    @property
    def maximum_available_score(self) -> float:
        return self.points


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


def load_moving_pad_rules(path: Path | str) -> MovingPadRules:
    """Load and checksum the exact frozen moving-pad policy."""
    payload = Path(path).read_bytes()
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("moving-pad rules must be UTF-8 JSON") from error
    if not isinstance(document, dict):
        raise ValueError("moving-pad rules must be an object")
    if set(document) != {
        "schema_version",
        "ruleset_id",
        "sample_interval_ns",
        "marker_id",
        "deck_half_extent_m",
        "aboard_duration_ns",
        "rules",
    }:
        raise ValueError("moving-pad rules fields changed")
    if document.get("schema_version") != 1 or document.get("ruleset_id") != "moving_pad_v1":
        raise ValueError("unsupported moving-pad ruleset")
    interval = document.get("sample_interval_ns")
    aboard_duration = document.get("aboard_duration_ns")
    marker_id = document.get("marker_id")
    if interval != 50_000_000 or isinstance(interval, bool):
        raise ValueError("moving_pad_v1 sample interval must be 50000000 ns")
    if aboard_duration != 2_000_000_000 or isinstance(aboard_duration, bool):
        raise ValueError("moving_pad_v1 aboard duration must be 2000000000 ns")
    if type(marker_id) is not int or marker_id != 7:
        raise ValueError("moving_pad_v1 marker must be 7")
    deck_half_extent = _positive_number(document, "deck_half_extent_m")
    if deck_half_extent != 1.5:
        raise ValueError("moving_pad_v1 deck half extent must be 1.5 m")
    rows = document.get("rules")
    if (
        not isinstance(rows, list)
        or len(rows) != 1
        or not isinstance(rows[0], dict)
        or set(rows[0]) != {"id", "points"}
        or rows[0].get("id") != _RULE_ID
    ):
        raise ValueError("moving_pad_v1 requires the physical_landing rule")
    points = _positive_number(rows[0], "points")
    if points != 100.0:
        raise ValueError("moving_pad_v1 physical landing must be worth 100 points")
    return MovingPadRules(
        ruleset_id="moving_pad_v1",
        scoring_checksum=hashlib.sha256(payload).hexdigest(),
        sample_interval_ns=interval,
        marker_id=marker_id,
        deck_half_extent_m=deck_half_extent,
        aboard_duration_ns=aboard_duration,
        points=points,
    )


def _pad_relative_position(
    vehicle: GroundTruthSample, pad: LandingPadSample
) -> tuple[float, float, float]:
    """Rotate the vehicle offset into the pad frame with a normalized inverse."""
    rx, ry, rz = (
        vehicle.position_xyz[index] - pad.position_xyz[index] for index in range(3)
    )
    qx, qy, qz, qw = pad.orientation_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    # Inverse quaternion rotation, expanded to avoid a general geometry dependency.
    tx = -2.0 * (qy * rz - qz * ry)
    ty = -2.0 * (qz * rx - qx * rz)
    tz = -2.0 * (qx * ry - qy * rx)
    return (
        rx + qw * tx - qy * tz + qz * ty,
        ry + qw * ty - qz * tx + qx * tz,
        rz + qw * tz - qx * ty + qy * tx,
    )


def _relative_speed(vehicle: GroundTruthSample, pad: LandingPadSample) -> float:
    offset = tuple(
        vehicle.position_xyz[index] - pad.position_xyz[index] for index in range(3)
    )
    wx, wy, wz = pad.angular_velocity_xyz
    rx, ry, rz = offset
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


class MovingPadScorer:
    """Score one exact-grid stream of paired vehicle and pad frames."""

    def __init__(
        self,
        run_id: str,
        rules: MovingPadRules,
        expected_frames: int,
    ) -> None:
        self.run_id = canonical_run_id(run_id)
        if not isinstance(rules, MovingPadRules):
            raise TypeError("rules must be MovingPadRules")
        if type(expected_frames) is not int or expected_frames <= 0:
            raise ValueError("expected_frames must be a positive integer")
        self.rules = rules
        self.expected_frames = expected_frames
        self._frames: list[tuple[GroundTruthSample, LandingPadSample]] = []
        self._disarmed_timestamp_ns: int | None = None
        self._diagnostic: str | None = None
        self._result: ScoreResult | None = None

    @property
    def last_sim_timestamp_ns(self) -> int:
        return self._frames[-1][0].sim_timestamp_ns if self._frames else 0

    def fail(self, reason: str) -> None:
        if self._result is not None:
            raise RuntimeError("score has already been finalized")
        if type(reason) is not str or not reason:
            raise ValueError("score failure reason must be nonempty")
        if self._diagnostic is None:
            self._diagnostic = reason

    def accept_frame(
        self,
        vehicle: GroundTruthSample,
        pad: LandingPadSample,
    ) -> None:
        if self._result is not None:
            raise RuntimeError("score has already been finalized")
        if not isinstance(vehicle, GroundTruthSample):
            raise TypeError("vehicle must be GroundTruthSample")
        if not isinstance(pad, LandingPadSample):
            raise TypeError("pad must be LandingPadSample")
        if vehicle.run_id != self.run_id or pad.run_id != self.run_id:
            raise ValueError("frame belongs to another run")
        if self._diagnostic is not None:
            return
        if len(self._frames) >= self.expected_frames:
            self._diagnostic = "frame_overrun"
            return
        if vehicle.sim_timestamp_ns != pad.sim_timestamp_ns:
            self._diagnostic = "frame_timestamp_mismatch"
            return
        if pad.marker_id != self.rules.marker_id:
            self._diagnostic = "pad_marker_mismatch"
            return
        if self._frames:
            previous = self._frames[-1][0].sim_timestamp_ns
            timestamp = vehicle.sim_timestamp_ns
            expected = previous + self.rules.sample_interval_ns
            if timestamp == previous:
                self._diagnostic = "frame_timestamp_duplicate"
                return
            if timestamp < expected:
                self._diagnostic = "frame_timestamp_regression"
                return
            if timestamp > expected:
                self._diagnostic = "frame_timestamp_gap"
                return
        self._frames.append((vehicle, pad))

    def accept_disarmed(self, timestamp_ns: int) -> None:
        if self._result is not None:
            raise RuntimeError("score has already been finalized")
        timestamp = _timestamp(timestamp_ns)
        if self._disarmed_timestamp_ns is None:
            self._disarmed_timestamp_ns = timestamp
        elif self._disarmed_timestamp_ns != timestamp:
            self.fail("disarm_event_conflict")

    def _touchdown(
        self,
    ) -> tuple[GroundTruthSample, LandingPadSample] | None:
        airborne_seen = False
        for vehicle, pad in self._frames:
            if not vehicle.in_contact:
                airborne_seen = True
            elif airborne_seen and pad.vehicle_in_contact:
                return vehicle, pad
        return None

    def _aboard(self, vehicle: GroundTruthSample, pad: LandingPadSample) -> bool:
        x, y, z = _pad_relative_position(vehicle, pad)
        return (
            vehicle.in_contact
            and pad.vehicle_in_contact
            and abs(x) <= self.rules.deck_half_extent_m
            and abs(y) <= self.rules.deck_half_extent_m
            and z >= 0.0
        )

    def _physical_pass(
        self,
        touchdown: tuple[GroundTruthSample, LandingPadSample] | None,
    ) -> bool:
        if touchdown is None or self._disarmed_timestamp_ns is None:
            return False
        touchdown_vehicle, _touchdown_pad = touchdown
        if touchdown_vehicle.sim_timestamp_ns > self._disarmed_timestamp_ns:
            return False
        required = self.rules.aboard_duration_ns // self.rules.sample_interval_ns + 1
        consecutive = 0
        for vehicle, pad in self._frames:
            if vehicle.sim_timestamp_ns < self._disarmed_timestamp_ns:
                continue
            consecutive = consecutive + 1 if self._aboard(vehicle, pad) else 0
            if consecutive >= required:
                return True
        return False

    def finalize(self) -> ScoreResult:
        if self._result is not None:
            return self._result
        diagnostic = self._diagnostic
        if diagnostic is None and len(self._frames) != self.expected_frames:
            diagnostic = "frame_count_mismatch"
        if diagnostic is None and self._disarmed_timestamp_ns is None:
            diagnostic = "disarm_missing"
        if diagnostic is None and self._frames:
            first_timestamp = self._frames[0][0].sim_timestamp_ns
            last_timestamp = self._frames[-1][0].sim_timestamp_ns
            assert self._disarmed_timestamp_ns is not None
            if not first_timestamp <= self._disarmed_timestamp_ns <= last_timestamp:
                diagnostic = "disarm_outside_evidence"
            elif all(
                vehicle.sim_timestamp_ns != self._disarmed_timestamp_ns
                for vehicle, _pad in self._frames
            ):
                diagnostic = "disarm_timestamp_unaligned"
        complete = diagnostic is None
        touchdown = self._touchdown()
        passed = complete and self._physical_pass(touchdown)
        awarded = self.rules.points if passed else 0.0
        rule_result = RuleResult(
            rule_id=_RULE_ID,
            passed=passed,
            awarded_points=awarded,
            available_points=self.rules.points,
            evidence_ref="rosbag#moving_pad:physical_landing",
        )
        final_timestamp = self.last_sim_timestamp_ns
        event_specs: list[tuple[int, str, float]] = [
            (final_timestamp, "moving_pad.physical_landing", awarded)
        ]
        if touchdown is not None:
            vehicle, pad = touchdown
            x, y, _z = _pad_relative_position(vehicle, pad)
            event_specs.extend(
                (
                    (
                        vehicle.sim_timestamp_ns,
                        "moving_pad.touchdown_offset_m",
                        math.hypot(x, y),
                    ),
                    (
                        vehicle.sim_timestamp_ns,
                        "moving_pad.touchdown_relative_velocity_mps",
                        _relative_speed(vehicle, pad),
                    ),
                )
            )
        event_specs.append((final_timestamp, "score.finalized", awarded))
        events = tuple(
            ScoreEvent(
                run_id=self.run_id,
                sim_timestamp_ns=timestamp,
                event_id=index,
                event_type=event_type,
                value=value,
                evidence_ref=f"scoring/events.jsonl#event-{index}",
            )
            for index, (timestamp, event_type, value) in enumerate(event_specs)
        )
        self._result = ScoreResult(
            run_id=self.run_id,
            ruleset_id=self.rules.ruleset_id,
            complete=complete,
            achieved_score=awarded,
            maximum_available_score=self.rules.maximum_available_score,
            scoring_checksum=self.rules.scoring_checksum,
            evidence_paths=tuple(event.evidence_ref for event in events)
            + (
                "rosbag#/simulation/ground_truth",
                "rosbag#/simulation/landing_pad_state",
                "rosbag#/simulation/mission_events",
            ),
            rule_results=(rule_result,),
            events=events,
            diagnostic=diagnostic,
        )
        return self._result


__all__ = [
    "LandingPadSample",
    "MovingPadRules",
    "MovingPadScorer",
    "load_moving_pad_rules",
]
