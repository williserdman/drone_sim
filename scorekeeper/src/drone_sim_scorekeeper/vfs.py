"""Pure automatic judging and scoring for ``competition_2027_v1``."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from artifacts.runtime_status import canonical_run_id

from .competition import MissionEventSample, PayloadEventSample, PayloadStateSample
from .descent import GroundTruthSample
from .models import RuleResult, ScoreEvent, ScoreResult


_AUTONOMOUS_MODES = frozenset({"GUIDED", "LAND"})
_STATUS_FRESHNESS_NS = 500_000_000


def _timestamp(value: object) -> int:
    if type(value) is not int or value < 0:
        raise ValueError("timestamp must be a nonnegative integer")
    return value


class JudgeOutcome(IntEnum):
    APPROVED = 1
    REJECTED = 2
    ERROR = 3


class AttemptState(IntEnum):
    WAITING = 0
    ACTIVE = 1
    PAUSED = 2
    APPROVED = 3
    FINISHED = 4


@dataclass(frozen=True)
class VehicleStatusSample:
    run_id: str
    sim_timestamp_ns: int
    sequence_id: int
    vehicle_id: str
    armed: bool
    mode: str
    failsafe_active: bool
    failsafe_reason: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "run_id", canonical_run_id(self.run_id))
        _timestamp(self.sim_timestamp_ns)
        if type(self.sequence_id) is not int or self.sequence_id < 0:
            raise ValueError("sequence_id must be a nonnegative integer")
        if not self.vehicle_id or not self.mode:
            raise ValueError("vehicle_id and mode must be nonempty")
        if type(self.armed) is not bool or type(self.failsafe_active) is not bool:
            raise ValueError("vehicle status flags must be booleans")
        if not isinstance(self.failsafe_reason, str) or (
            self.failsafe_active and not self.failsafe_reason
        ) or (not self.failsafe_active and self.failsafe_reason):
            raise ValueError("failsafe reason must match active state")


@dataclass(frozen=True)
class JudgeDecision:
    run_id: str
    sim_timestamp_ns: int
    decision_id: int
    request_id: str
    request_event_id: int
    phase: str
    checkpoint: str
    outcome: JudgeOutcome
    eligible_aruco_ids: tuple[int, ...]
    active_elapsed_ns: int
    reason_code: str
    evidence_refs: tuple[str, ...]


@dataclass(frozen=True)
class AttemptStatus:
    run_id: str
    sim_timestamp_ns: int
    sequence_id: int
    state: AttemptState
    active_elapsed_ns: int
    approval_timestamp_ns: int
    started: bool


@dataclass(frozen=True)
class VfsRules:
    ruleset_id: str
    scoring_checksum: str
    point_a_manual: float
    point_a_autonomous: float
    target_points: dict[str, float]
    recovery_points: float
    autonomy_bonus: float
    time_bonuses: tuple[tuple[int, float], ...]
    weight_multipliers: tuple[tuple[float, float], ...]
    deduction_points: float

    @property
    def maximum_available_score(self) -> float:
        return 380.0

    def multiplier(self, weight_lb: float) -> float:
        for upper, multiplier in self.weight_multipliers:
            if weight_lb < upper:
                return multiplier
        return self.weight_multipliers[-1][1]

    def time_bonus(self, remaining_ns: int) -> float:
        seconds = remaining_ns / 1_000_000_000
        return max((points for threshold, points in self.time_bonuses if seconds >= threshold), default=0.0)


def load_vfs_rules(path: Path | str) -> VfsRules:
    payload = Path(path).read_bytes()
    document = json.loads(payload)
    if document.get("schema_version") != 1 or document.get("ruleset_id") != "competition_2027_v1":
        raise ValueError("unsupported VFS ruleset")
    expected_points = {
        "A_manual": 55,
        "A_autonomous": 65,
        "BP_by_target": {"A": 5, "B": 10, "C": 30},
        "CP_per_payload": 30,
        "autonomy_bonus": 15,
    }
    expected_time = tuple(zip((0, 30, 60, 90, 120, 150, 180, 210, 240), (0, 1, 3, 5, 7, 9, 11, 13, 15), strict=True))
    expected_weight = tuple(zip((10, 12, 14, 16, 18, 20, 1e300), (1.5, 1.4, 1.3, 1.2, 1.1, 1.05, 1.0), strict=True))
    actual_time = tuple(
        (row.get("minimum_remaining_seconds"), row.get("points"))
        for row in document.get("time_bonus", ())
        if isinstance(row, dict)
    )
    actual_weight = tuple(
        (row.get("upper_bound_lb"), row.get("multiplier"))
        for row in document.get("weight_multiplier", ())
        if isinstance(row, dict)
    )
    if (
        set(document) != {"schema_version", "ruleset_id", "points", "time_bonus", "weight_multiplier", "deduction_points"}
        or document.get("points") != expected_points
        or actual_time != expected_time
        or actual_weight != expected_weight
        or document.get("deduction_points") != 25
    ):
        raise ValueError("official VFS scoring policy changed")
    return VfsRules(
        "competition_2027_v1",
        hashlib.sha256(payload).hexdigest(),
        float(document["points"]["A_manual"]),
        float(document["points"]["A_autonomous"]),
        {key: float(value) for key, value in document["points"]["BP_by_target"].items()},
        float(document["points"]["CP_per_payload"]),
        float(document["points"]["autonomy_bonus"]),
        tuple((int(row["minimum_remaining_seconds"]), float(row["points"])) for row in document["time_bonus"]),
        tuple((float(row["upper_bound_lb"]), float(row["multiplier"])) for row in document["weight_multiplier"]),
        float(document["deduction_points"]),
    )


def _waypoint_contains(waypoint: Any, x: float, y: float, *, inward: float = 0.0) -> bool:
    radius = getattr(waypoint, "radius", None)
    if radius is not None:
        return math.hypot(x - waypoint.x, y - waypoint.y) <= float(radius) - inward + 1e-9
    width = getattr(waypoint, "width", None)
    height = getattr(waypoint, "height", None)
    return (
        width is not None
        and height is not None
        and abs(x - waypoint.x) <= float(width) / 2 - inward + 1e-9
        and abs(y - waypoint.y) <= float(height) / 2 - inward + 1e-9
    )


def _contact_points(
    sample: PayloadStateSample,
    size: tuple[float, float, float],
    *,
    ground_z_m: float,
    epsilon_m: float,
) -> tuple[tuple[float, float], ...]:
    if not sample.grounded:
        return ()
    half_x, half_y, half_z = (extent / 2 for extent in size)
    qx, qy, qz, qw = sample.orientation_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return ()
    qx, qy, qz, qw = (component / norm for component in (qx, qy, qz, qw))
    rotation = (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )
    center_x, center_y, center_z = sample.position_xyz
    corners = tuple(
        (
            center_x + rotation[0][0] * dx + rotation[0][1] * dy + rotation[0][2] * dz,
            center_y + rotation[1][0] * dx + rotation[1][1] * dy + rotation[1][2] * dz,
            center_z + rotation[2][0] * dx + rotation[2][1] * dy + rotation[2][2] * dz,
        )
        for dx in (-half_x, half_x)
        for dy in (-half_y, half_y)
        for dz in (-half_z, half_z)
    )
    lowest_z = min(point[2] for point in corners)
    if abs(lowest_z - ground_z_m) > epsilon_m:
        return ()
    return tuple(
        (x, y)
        for x, y, z in corners
        if z - lowest_z <= epsilon_m and abs(z - ground_z_m) <= epsilon_m
    )


def _convex_hull(points: tuple[tuple[float, float], ...]) -> tuple[tuple[float, float], ...]:
    unique = sorted(set(points))
    if len(unique) <= 1:
        return tuple(unique)

    def cross(origin: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return tuple(lower[:-1] + upper[:-1])


def _segments(points: tuple[tuple[float, float], ...]) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...]:
    if len(points) < 2:
        return ()
    if len(points) == 2:
        return ((points[0], points[1]),)
    return tuple((points[index], points[(index + 1) % len(points)]) for index in range(len(points)))


def _point_in_polygon(point: tuple[float, float], polygon: tuple[tuple[float, float], ...]) -> bool:
    if len(polygon) < 3:
        return False
    sign = 0
    for start, end in _segments(polygon):
        cross = (end[0] - start[0]) * (point[1] - start[1]) - (end[1] - start[1]) * (point[0] - start[0])
        if abs(cross) <= 1e-9:
            continue
        current = 1 if cross > 0 else -1
        if sign and current != sign:
            return False
        sign = current
    return True


def _segment_distance(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_squared))
    return math.hypot(point[0] - start[0] - fraction * dx, point[1] - start[1] - fraction * dy)


def _orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(
    first_start: tuple[float, float],
    first_end: tuple[float, float],
    second_start: tuple[float, float],
    second_end: tuple[float, float],
) -> bool:
    first_a = _orientation(first_start, first_end, second_start)
    first_b = _orientation(first_start, first_end, second_end)
    second_a = _orientation(second_start, second_end, first_start)
    second_b = _orientation(second_start, second_end, first_end)
    if first_a * first_b > 1e-18 or second_a * second_b > 1e-18:
        return False

    def overlaps(a: float, b: float, c: float, d: float) -> bool:
        return max(min(a, b), min(c, d)) <= min(max(a, b), max(c, d)) + 1e-9

    return overlaps(first_start[0], first_end[0], second_start[0], second_end[0]) and overlaps(
        first_start[1], first_end[1], second_start[1], second_end[1]
    )


def _payload_contained(
    sample: PayloadStateSample,
    waypoint: Any,
    size: tuple[float, float, float],
    *,
    ground_z_m: float,
    epsilon_m: float,
) -> bool:
    points = _convex_hull(_contact_points(
        sample,
        size,
        ground_z_m=ground_z_m,
        epsilon_m=epsilon_m,
    ))
    if not points:
        return False
    if any(_waypoint_contains(waypoint, x, y) for x, y in points):
        return True
    radius = getattr(waypoint, "radius", None)
    if radius is not None:
        center = (float(waypoint.x), float(waypoint.y))
        return _point_in_polygon(center, points) or any(
            _segment_distance(center, start, end) <= float(radius) + 1e-9
            for start, end in _segments(points)
        )
    width, height = getattr(waypoint, "width", None), getattr(waypoint, "height", None)
    if width is None or height is None:
        return False
    rectangle = (
        (waypoint.x - width / 2, waypoint.y - height / 2),
        (waypoint.x + width / 2, waypoint.y - height / 2),
        (waypoint.x + width / 2, waypoint.y + height / 2),
        (waypoint.x - width / 2, waypoint.y + height / 2),
    )
    return any(_point_in_polygon(corner, points) for corner in rectangle) or any(
        _segments_intersect(contact_start, contact_end, zone_start, zone_end)
        for contact_start, contact_end in _segments(points)
        for zone_start, zone_end in _segments(rectangle)
    )


def _payload_targets(scenario: Any) -> dict[int, str]:
    configured = getattr(scenario, "targets", None)
    if configured is not None:
        return {int(marker): str(target) for marker, target in configured.items()}
    return {item.aruco_id: item.target for item in scenario.payloads}


class VfsJudge:
    """Evaluate ordered public facts and issue correlated check decisions."""

    def __init__(self, run_id: str, course: Any, scenario: Any) -> None:
        self.run_id = canonical_run_id(run_id)
        self.course = course
        self.scenario = scenario
        self._truth: GroundTruthSample | None = None
        self._status: VehicleStatusSample | None = None
        self._payloads: dict[int, PayloadStateSample] = {}
        self._requests: dict[str, tuple[str, str, JudgeDecision]] = {}
        self._decision_id = 0
        self._status_sequence = 0
        self._started_at_ns: int | None = None
        self._active_since_ns: int | None = None
        self._active_accumulated_ns = 0
        self._paused = False
        self._approval_ns = 0
        self._phase = ""
        self._finished = False
        self._terminal_failure_reason: str | None = None
        self._payload_grounded_since: dict[int, int] = {}
        self._takeoff_count = 0
        self._fm1_d_approved = False
        self._fm2_approved = False
        self._fm2_airborne = False
        self._fm2_eligible_ids: tuple[int, ...] = ()
        self._leg_peak_agl_m = 0.0

    @property
    def active_elapsed_ns(self) -> int:
        if self._active_since_ns is None or self._truth is None:
            return self._active_accumulated_ns
        return self._active_accumulated_ns + self._truth.sim_timestamp_ns - self._active_since_ns

    def _active_elapsed_at(self, timestamp_ns: int) -> int:
        if self._active_since_ns is None:
            return self._active_accumulated_ns
        return self._active_accumulated_ns + max(0, timestamp_ns - self._active_since_ns)

    @property
    def attempt_status(self) -> AttemptStatus:
        stamp = self._truth.sim_timestamp_ns if self._truth else 0
        if self._finished:
            state = AttemptState.FINISHED
        elif self._started_at_ns is None:
            state = AttemptState.WAITING
        elif self._paused and self._approval_ns:
            state = AttemptState.APPROVED
        elif self._paused:
            state = AttemptState.PAUSED
        else:
            state = AttemptState.ACTIVE
        return AttemptStatus(
            self.run_id, stamp, self._status_sequence, state,
            self.active_elapsed_ns, self._approval_ns, self._started_at_ns is not None,
        )

    def _same_run(self, sample: Any) -> None:
        if sample.run_id != self.run_id:
            raise ValueError("evidence belongs to another run")

    def accept_ground_truth(self, sample: GroundTruthSample) -> AttemptStatus:
        self._same_run(sample)
        if self._truth is not None and sample.sim_timestamp_ns <= self._truth.sim_timestamp_ns:
            raise ValueError("ground truth timestamps must increase")
        previous = self._truth
        self._truth = sample
        ground_z_m = float(getattr(self.course, "ground_z_m", 0.0))
        if not sample.in_contact:
            self._leg_peak_agl_m = max(
                self._leg_peak_agl_m, sample.position_xyz[2] - ground_z_m
            )
        if (
            self._paused
            and self._approval_ns
            and sample.sim_timestamp_ns - self._approval_ns
            > int(float(self.course.attempt.approval_timeout_s) * 1e9)
        ):
            self._finished = True
            self._terminal_failure_reason = "approval_timeout"
        if previous is not None and previous.in_contact and not sample.in_contact:
            self._takeoff_count += 1
            if self._phase == "FM2":
                self._fm2_airborne = True
            self._leg_peak_agl_m = sample.position_xyz[2] - ground_z_m
            if self._started_at_ns is None:
                self._started_at_ns = sample.sim_timestamp_ns
                self._active_since_ns = sample.sim_timestamp_ns
            elif self._paused and not self._finished:
                self._paused = False
                self._active_since_ns = sample.sim_timestamp_ns
                self._approval_ns = 0
        elif (
            previous is not None
            and not previous.in_contact
            and sample.in_contact
            and self._phase == "FM3"
            and _waypoint_contains(
                self.course.waypoints["H"],
                sample.position_xyz[0],
                sample.position_xyz[1],
                inward=float(self.course.attempt.aircraft_radius_m),
            )
            and self._active_since_ns is not None
        ):
            self._active_accumulated_ns += sample.sim_timestamp_ns - self._active_since_ns
            self._active_since_ns = None
            self._paused = True
        if self.active_elapsed_ns > int(float(self.course.attempt.duration_seconds) * 1e9):
            if self._active_since_ns is not None:
                self._active_accumulated_ns += sample.sim_timestamp_ns - self._active_since_ns
                self._active_since_ns = None
            self._finished = True
            self._terminal_failure_reason = "active_deadline_exceeded"
        self._status_sequence += 1
        return self.attempt_status

    def accept_vehicle_status(self, sample: VehicleStatusSample) -> None:
        self._same_run(sample)
        if self._status is not None and (
            sample.sim_timestamp_ns <= self._status.sim_timestamp_ns
            or sample.sequence_id != self._status.sequence_id + 1
        ):
            raise ValueError("vehicle status must be ordered and contiguous")
        self._status = sample
        if (
            self._phase == "FM2"
            and self._fm2_airborne
            and self._started_at_ns is not None
            and self._active_since_ns is not None
            and not sample.armed
            and self._truth is not None
            and self._truth.in_contact
            and _waypoint_contains(
                self.course.waypoints["H"],
                self._truth.position_xyz[0],
                self._truth.position_xyz[1],
                inward=float(self.course.attempt.aircraft_radius_m),
            )
            and abs(sample.sim_timestamp_ns - self._truth.sim_timestamp_ns) <= _STATUS_FRESHNESS_NS
        ):
            pause_stamp = max(sample.sim_timestamp_ns, self._truth.sim_timestamp_ns)
            self._active_accumulated_ns += pause_stamp - self._active_since_ns
            self._active_since_ns = None
            self._paused = True

    def accept_payload_state(self, sample: PayloadStateSample) -> None:
        self._same_run(sample)
        previous = self._payloads.get(sample.aruco_id)
        if previous is not None and sample.sim_timestamp_ns <= previous.sim_timestamp_ns:
            raise ValueError("payload timestamps must increase")
        self._payloads[sample.aruco_id] = sample
        if sample.grounded and math.sqrt(sum(value * value for value in sample.linear_velocity_xyz)) <= 0.10:
            self._payload_grounded_since.setdefault(sample.aruco_id, sample.sim_timestamp_ns)
        else:
            self._payload_grounded_since.pop(sample.aruco_id, None)

    def accept_payload_event(self, sample: PayloadEventSample) -> None:
        self._same_run(sample)

    def _fresh_status(self, stamp: int) -> bool:
        return self._status is not None and 0 <= stamp - self._status.sim_timestamp_ns <= _STATUS_FRESHNESS_NS

    def _contained(self, sample: PayloadStateSample, waypoint: Any) -> bool:
        raw_size = self.scenario.payload_geometry.size_m
        size = (float(raw_size[0]), float(raw_size[1]), float(raw_size[2]))
        return _payload_contained(
            sample,
            waypoint,
            size,
            ground_z_m=float(getattr(self.course, "ground_z_m", 0.0)),
            epsilon_m=float(self.course.attempt.ground_contact_epsilon_m),
        )

    def _eligible(self, timestamp_ns: int) -> tuple[int, ...]:
        targets = _payload_targets(self.scenario)
        interaction = getattr(self.scenario, "payload_interaction", None)
        settle_ns = int(float(getattr(interaction, "settle_time_s", 1.0)) * 1e9)
        return tuple(
            marker for marker in sorted(targets)
            if marker in self._payloads
            and 0 <= timestamp_ns - self._payloads[marker].sim_timestamp_ns <= 50_000_000
            and self._payloads[marker].sim_timestamp_ns
            - self._payload_grounded_since.get(marker, self._payloads[marker].sim_timestamp_ns)
            >= settle_ns
            and self._contained(self._payloads[marker], self.course.waypoints[targets[marker]])
        )

    def _decision(self, event: MissionEventSample, request_id: str, checkpoint: str) -> JudgeDecision:
        outcome = JudgeOutcome.REJECTED
        reason = "physical_requirements_not_met"
        eligible: tuple[int, ...] = ()
        refs = [f"mission-events#{event.event_id}"]
        if self._truth is not None and self._truth.sim_timestamp_ns <= event.sim_timestamp_ns:
            refs.append(f"groundtruth#{self._truth.sim_timestamp_ns}")
        if self._status is not None and self._status.sim_timestamp_ns <= event.sim_timestamp_ns:
            refs.append(f"vehicle-status#{self._status.sequence_id}")
        if (
            self._truth is None
            or not 0 <= event.sim_timestamp_ns - self._truth.sim_timestamp_ns <= 50_000_000
            or not self._fresh_status(event.sim_timestamp_ns)
        ):
            outcome, reason = JudgeOutcome.ERROR, "evidence_stale_or_missing"
        elif event.phase == "FM1" and checkpoint in {"D", "H"}:
            inside = _waypoint_contains(
                self.course.waypoints[checkpoint],
                self._truth.position_xyz[0], self._truth.position_xyz[1],
                inward=float(self.course.attempt.aircraft_radius_m),
            )
            ordered = (
                checkpoint == "D" and self._phase == "FM1" and self._takeoff_count >= 1
            ) or (
                checkpoint == "H"
                and self._fm1_d_approved
                and self._takeoff_count >= 2
            )
            if (
                self._truth.in_contact
                and inside
                and ordered
                and self._status is not None
                and not self._status.armed
                and self._leg_peak_agl_m >= float(self.course.attempt.minimum_agl_m)
            ):
                outcome, reason = JudgeOutcome.APPROVED, "landing_approved"
                if checkpoint == "D":
                    self._fm1_d_approved = True
        elif event.phase == "FM2" and checkpoint == "":
            targets = _payload_targets(self.scenario)
            refs.extend(
                f"payload-state#{marker}/{self._payloads[marker].sim_timestamp_ns}"
                for marker in sorted(targets)
                if marker in self._payloads
            )
            payload_evidence_current = all(
                marker in self._payloads
                and 0 <= event.sim_timestamp_ns - self._payloads[marker].sim_timestamp_ns <= 50_000_000
                for marker in targets
            )
            if not payload_evidence_current:
                outcome, reason = JudgeOutcome.ERROR, "evidence_stale_or_missing"
            elif (
                self._truth.in_contact
                and _waypoint_contains(
                    self.course.waypoints["H"],
                    self._truth.position_xyz[0],
                    self._truth.position_xyz[1],
                    inward=float(self.course.attempt.aircraft_radius_m),
                )
                and self._status is not None
                and not self._status.armed
            ):
                eligible = self._eligible(event.sim_timestamp_ns)
                outcome, reason = JudgeOutcome.APPROVED, "eligible_payloads_frozen"
                if self._active_since_ns is not None:
                    self._active_accumulated_ns += event.sim_timestamp_ns - self._active_since_ns
                    self._active_since_ns = None
                self._paused = True
                self._approval_ns = event.sim_timestamp_ns
                self._fm2_approved = True
                self._fm2_eligible_ids = eligible
                if not eligible:
                    self._finished = True
        elif event.phase == "FM3" and checkpoint == "":
            refs.extend(
                f"payload-state#{marker}/{self._payloads[marker].sim_timestamp_ns}"
                for marker in self._fm2_eligible_ids
                if marker in self._payloads
            )
            inside = _waypoint_contains(
                self.course.waypoints["H"], self._truth.position_xyz[0], self._truth.position_xyz[1],
                inward=float(self.course.attempt.aircraft_radius_m),
            )
            payload_evidence_current = all(
                marker in self._payloads
                and 0 <= event.sim_timestamp_ns - self._payloads[marker].sim_timestamp_ns <= 50_000_000
                for marker in self._fm2_eligible_ids
            )
            payloads_retrieved = payload_evidence_current and all(
                self._payloads[marker].grounded
                and not self._payloads[marker].attached
                and self._contained(self._payloads[marker], self.course.waypoints["L"])
                for marker in self._fm2_eligible_ids
            )
            if self._terminal_failure_reason is not None:
                reason = self._terminal_failure_reason
            elif not payload_evidence_current:
                outcome, reason = JudgeOutcome.ERROR, "evidence_stale_or_missing"
            elif not payloads_retrieved:
                reason = "payload_retrieval_incomplete"
            elif (
                self._phase == "FM3"
                and self._fm2_approved
                and self._truth.in_contact
                and inside
                and self._status is not None
                and not self._status.armed
            ):
                outcome, reason = JudgeOutcome.APPROVED, "attempt_complete"
                self._finished = True
        else:
            outcome, reason = JudgeOutcome.ERROR, "invalid_check_request"
        decision = JudgeDecision(
            self.run_id, event.sim_timestamp_ns, self._decision_id, request_id,
            event.event_id, event.phase, checkpoint, outcome, eligible,
            self.active_elapsed_ns, reason, tuple(refs),
        )
        self._decision_id += 1
        return decision

    def accept_mission_event(self, event: MissionEventSample) -> tuple[JudgeDecision, ...]:
        self._same_run(event)
        if event.state == "STARTED" and event.phase in {"FM1", "FM2", "FM3"}:
            self._phase = event.phase
            if event.phase == "FM2":
                self._fm2_airborne = self._truth is not None and not self._truth.in_contact
            return ()
        if event.state != "CHECK_REQUESTED":
            return ()
        try:
            detail = json.loads(event.detail)
            if set(detail) != {"checkpoint", "request_id"}:
                raise ValueError
            checkpoint, request_id = detail["checkpoint"], detail["request_id"]
            if not isinstance(checkpoint, str) or not isinstance(request_id, str) or not request_id:
                raise ValueError
        except (ValueError, TypeError, json.JSONDecodeError):
            return (self._error(event, "", "", "invalid_request_detail"),)
        previous = self._requests.get(request_id)
        if previous is not None:
            if previous[:2] == (event.phase, checkpoint):
                return (previous[2],)
            return (self._error(event, request_id, checkpoint, "request_id_conflict"),)
        decision = self._decision(event, request_id, checkpoint)
        self._requests[request_id] = (event.phase, checkpoint, decision)
        return (decision,)

    def reject_mission_event_missing_evidence(
        self, event: MissionEventSample
    ) -> tuple[JudgeDecision, ...]:
        """Freeze a pending check as an evidence error without re-evaluating facts."""
        self._same_run(event)
        if event.state != "CHECK_REQUESTED":
            return ()
        try:
            detail = json.loads(event.detail)
            if set(detail) != {"checkpoint", "request_id"}:
                raise ValueError
            checkpoint, request_id = detail["checkpoint"], detail["request_id"]
            if not isinstance(checkpoint, str) or not isinstance(request_id, str) or not request_id:
                raise ValueError
        except (ValueError, TypeError, json.JSONDecodeError):
            return (self._error(event, "", "", "invalid_request_detail"),)
        previous = self._requests.get(request_id)
        if previous is not None:
            if previous[:2] == (event.phase, checkpoint):
                return (previous[2],)
            return (self._error(event, request_id, checkpoint, "request_id_conflict"),)
        decision = self._error(
            event,
            request_id,
            checkpoint,
            "evidence_stale_or_missing",
            active_elapsed_ns=self._active_elapsed_at(event.sim_timestamp_ns),
        )
        self._requests[request_id] = (event.phase, checkpoint, decision)
        return (decision,)

    def _error(
        self,
        event: MissionEventSample,
        request_id: str,
        checkpoint: str,
        reason: str,
        *,
        active_elapsed_ns: int | None = None,
    ) -> JudgeDecision:
        decision = JudgeDecision(
            self.run_id, event.sim_timestamp_ns, self._decision_id, request_id,
            event.event_id, event.phase, checkpoint, JudgeOutcome.ERROR, (),
            self.active_elapsed_ns if active_elapsed_ns is None else active_elapsed_ns,
            reason, (f"mission-events#{event.event_id}",),
        )
        self._decision_id += 1
        return decision


class VfsScorer:
    """Accumulate independently established score components and adjustments."""

    def __init__(self, run_id: str, rules: VfsRules, course: Any, scenario: Any, *, unloaded_weight_lb: float) -> None:
        self.run_id = canonical_run_id(run_id)
        self.rules, self.course, self.scenario = rules, course, scenario
        if not math.isfinite(unloaded_weight_lb) or unloaded_weight_lb < 0:
            raise ValueError("unloaded weight must be nonnegative and finite")
        self.weight_lb = unloaded_weight_lb
        self.a = self.bp = self.cp = self.ba = self.ca = 0.0
        self.t = 0.0
        self.deductions = 0.0
        self.active_ns: int | None = None
        self.home_before_deadline = False
        self.refs: dict[str, str] = {}
        self._last_timestamp_ns = 0
        self._diagnostic: str | None = None
        self._phase = ""
        self._truth: GroundTruthSample | None = None
        self._payloads: dict[int, PayloadStateSample] = {}
        self._last_vehicle_status: VehicleStatusSample | None = None
        self._autonomous = {"FM1": True, "FM2": True, "FM3": True}
        self._bp_invalid: tuple[str, str] | None = None
        self._cp_invalid: tuple[str, str] | None = None
        self._penalties: list[tuple[str, str]] = []
        self._fm2_eligible_ids: frozenset[int] = frozenset()

    @property
    def last_sim_timestamp_ns(self) -> int:
        return self._last_timestamp_ns

    def fail(self, reason: str) -> None:
        if not reason:
            raise ValueError("failure reason must be nonempty")
        self._diagnostic = self._diagnostic or reason

    def accept_ground_truth(self, sample: GroundTruthSample) -> None:
        if sample.run_id != self.run_id:
            return
        self._last_timestamp_ns = max(self._last_timestamp_ns, sample.sim_timestamp_ns)
        previous = self._truth
        self._truth = sample
        if previous is not None and not previous.in_contact and sample.in_contact:
            allowed = any(
                name in self.course.waypoints
                and _waypoint_contains(
                    self.course.waypoints[name],
                    sample.position_xyz[0],
                    sample.position_xyz[1],
                )
                for name in ("S", "D", "L", "H")
            )
            if not allowed:
                self.add_penalty("off_course_contact", f"ground-truth#{sample.sim_timestamp_ns}")
            status = self._last_vehicle_status
            if (
                status is not None
                and status.failsafe_active
                and 0 <= sample.sim_timestamp_ns - status.sim_timestamp_ns <= _STATUS_FRESHNESS_NS
            ):
                self.add_penalty("failsafe_landing", f"vehicle-status#{status.sequence_id}")

    def accept_vehicle_status(self, sample: VehicleStatusSample) -> None:
        if sample.run_id != self.run_id:
            return
        self._last_timestamp_ns = max(self._last_timestamp_ns, sample.sim_timestamp_ns)
        self._last_vehicle_status = sample
        if self._phase in self._autonomous and (
            (sample.armed and sample.mode not in _AUTONOMOUS_MODES)
            or sample.failsafe_active
        ):
            self._autonomous[self._phase] = False

    def accept_payload_state(self, sample: PayloadStateSample) -> None:
        if sample.run_id == self.run_id:
            self._last_timestamp_ns = max(self._last_timestamp_ns, sample.sim_timestamp_ns)
            self._payloads[sample.aruco_id] = sample

    def accept_payload_event(self, sample: PayloadEventSample) -> None:
        if sample.run_id != self.run_id:
            return
        self._last_timestamp_ns = max(self._last_timestamp_ns, sample.sim_timestamp_ns)
        if sample.action != "release" or sample.state != "detached" or self._truth is None:
            return
        if self._phase == "FM2" and self._truth.position_xyz[2] < float(self.course.attempt.minimum_agl_m):
            self.invalidate_bp("low_release", f"payload-events#{sample.event_id}")
        if self._phase == "FM3":
            evidence_ref = f"payload-events#{sample.event_id}"
            if not self._truth.in_contact:
                self.invalidate_cp("airborne_release", evidence_ref)
            elif not _waypoint_contains(
                self.course.waypoints["L"],
                self._truth.position_xyz[0],
                self._truth.position_xyz[1],
                inward=float(self.course.attempt.aircraft_radius_m),
            ):
                self.invalidate_cp("invalid_landing", evidence_ref)

    def accept_mission_event(self, sample: MissionEventSample) -> None:
        if sample.run_id == self.run_id:
            self._last_timestamp_ns = max(self._last_timestamp_ns, sample.sim_timestamp_ns)
            if sample.state == "STARTED" and sample.phase in self._autonomous:
                self._phase = sample.phase

    def accept_judge_decision(self, decision: JudgeDecision) -> None:
        if decision.run_id != self.run_id or decision.outcome is not JudgeOutcome.APPROVED:
            return
        self._last_timestamp_ns = max(self._last_timestamp_ns, decision.sim_timestamp_ns)
        ref = decision.evidence_refs[0] if decision.evidence_refs else "judge-decisions"
        if decision.phase == "FM1" and decision.checkpoint == "H":
            self.record_fm1(autonomous=self._autonomous["FM1"], complete=True, evidence_ref=ref)
        elif decision.phase == "FM2":
            self._fm2_eligible_ids = frozenset(decision.eligible_aruco_ids)
            self.record_fm2(decision.eligible_aruco_ids, autonomous=self._autonomous["FM2"], evidence_ref=ref)
            self.record_active_time(decision.active_elapsed_ns, home_before_deadline=False)
        elif decision.phase == "FM3":
            raw_size = self.scenario.payload_geometry.size_m
            size = (float(raw_size[0]), float(raw_size[1]), float(raw_size[2]))
            delivered = tuple(
                marker for marker, sample in self._payloads.items()
                if marker in self._fm2_eligible_ids
                and not sample.attached
                and _payload_contained(
                    sample,
                    self.course.waypoints["L"],
                    size,
                    ground_z_m=float(getattr(self.course, "ground_z_m", 0.0)),
                    epsilon_m=float(self.course.attempt.ground_contact_epsilon_m),
                )
            )
            self.record_fm3(delivered, autonomous=self._autonomous["FM3"], evidence_ref=ref)
            self.record_active_time(decision.active_elapsed_ns, home_before_deadline=True)

    def record_fm1(self, *, autonomous: bool, complete: bool, evidence_ref: str) -> None:
        self.a = (self.rules.point_a_autonomous if autonomous else self.rules.point_a_manual) if complete else 0.0
        self.refs["A"] = evidence_ref

    def record_fm2(self, eligible_ids: tuple[int, ...], *, autonomous: bool, evidence_ref: str) -> None:
        targets = _payload_targets(self.scenario)
        self.bp = sum(self.rules.target_points[targets[marker]] for marker in eligible_ids)
        if self._bp_invalid is not None:
            self.bp = 0.0
        self.ba = self.rules.autonomy_bonus if autonomous and eligible_ids else 0.0
        self.refs.update(BP=evidence_ref, BA=evidence_ref)

    def record_fm3(self, delivered_ids: tuple[int, ...], *, autonomous: bool, evidence_ref: str) -> None:
        self.cp = len(delivered_ids) * self.rules.recovery_points
        if self._cp_invalid is not None:
            reason, invalidation_ref = self._cp_invalid
            self.cp = 0.0
            self.refs["CP"] = f"{invalidation_ref}:{reason}"
        else:
            self.refs["CP"] = evidence_ref
        self.ca = self.rules.autonomy_bonus if autonomous and delivered_ids else 0.0
        self.refs["CA"] = evidence_ref

    def invalidate_bp(self, reason: str, evidence_ref: str) -> None:
        self._bp_invalid = (reason, evidence_ref)
        self.bp = 0.0
        self.refs["BP"] = f"{evidence_ref}:{reason}"

    def invalidate_cp(self, reason: str, evidence_ref: str) -> None:
        self._cp_invalid = (reason, evidence_ref)
        self.cp = 0.0
        self.refs["CP"] = f"{evidence_ref}:{reason}"

    def add_penalty(self, reason: str, evidence_ref: str) -> None:
        self.deductions += self.rules.deduction_points
        self._penalties.append((reason, evidence_ref))
        self.refs["deductions"] = f"{evidence_ref}:{reason}"

    def record_active_time(self, active_elapsed_ns: int, *, home_before_deadline: bool) -> None:
        self.active_ns = _timestamp(active_elapsed_ns)
        self.home_before_deadline = home_before_deadline

    def finalize(self, timestamp_ns: int | None = None) -> ScoreResult:
        stamp = _timestamp(self._last_timestamp_ns if timestamp_ns is None else timestamp_ns)
        multiplier = self.rules.multiplier(self.weight_lb)
        duration_ns = int(float(self.course.attempt.duration_seconds) * 1e9)
        if (
            self.active_ns is not None
            and self.active_ns > duration_ns
            and not self.home_before_deadline
            and all(reason != "late_home" for reason, _ref in self._penalties)
        ):
            self.add_penalty("late_home", "attempt-status#active-deadline")
        if self.active_ns is not None and self.cp > 0 and self.home_before_deadline and self.active_ns <= duration_ns:
            self.t = self.rules.time_bonus(duration_ns - self.active_ns)
        achieved = max(0.0, self.a + multiplier * (self.bp + self.cp) + self.ba + self.ca + self.t - self.deductions)
        values = (
            ("A", self.a, 65.0), ("BP", self.bp, 90.0), ("CP", self.cp, 90.0),
            ("R", multiplier, 1.5), ("BA", self.ba, 15.0), ("CA", self.ca, 15.0),
            ("T", self.t, 15.0), ("deductions", self.deductions, 0.0),
        )
        rows = tuple(
            RuleResult(name, value == available, value, available, self.refs.get(name, f"rules#{name}"))
            for name, value, available in values
        )
        component_events = tuple(
            ScoreEvent(self.run_id, stamp, index, f"competition_2027.{name}", value, row.evidence_ref)
            for index, ((name, value, _available), row) in enumerate(zip(values, rows, strict=True))
        )
        penalty_events = tuple(
            ScoreEvent(
                self.run_id,
                stamp,
                len(component_events) + index,
                f"competition_2027.penalty.{reason}",
                -self.rules.deduction_points,
                evidence_ref,
            )
            for index, (reason, evidence_ref) in enumerate(self._penalties)
        )
        next_id = len(component_events) + len(penalty_events)
        events = component_events + penalty_events + (
            ScoreEvent(
                self.run_id, stamp, next_id,
                "competition_2027.unloaded_weight_lb", self.weight_lb,
                "configuration/scenario.yaml#aircraft_empty_mass_kg",
            ),
            ScoreEvent(self.run_id, stamp, next_id + 1, "score.finalized", achieved, "scoring/result.json"),
        )
        return ScoreResult(
            self.run_id, self.rules.ruleset_id, self._diagnostic is None, achieved, 380.0,
            self.rules.scoring_checksum, tuple(row.evidence_ref for row in rows), rows,
            events, self._diagnostic,
        )


__all__ = [
    "AttemptState", "AttemptStatus", "JudgeDecision", "JudgeOutcome",
    "VehicleStatusSample", "VfsJudge", "VfsRules", "VfsScorer", "load_vfs_rules",
]
