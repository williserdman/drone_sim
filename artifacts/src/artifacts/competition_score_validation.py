"""Independent validation of completed ``competition_v1`` scoring evidence.

This module deliberately does not import the production competition scorer.  It
reconstructs the official result from immutable bag facts and then compares that
result with both persisted score representations.
"""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any

from ._adapters.rosbag import (
    GroundTruthEvidence,
    MissionEventEvidence,
    PayloadEventEvidence,
    PayloadStateEvidence,
    PhysicalBagEvidence,
    ScoreEventEvidence,
)
from .score_validation import ScoreValidationError
from .validation import ValidationStatus, read_regular_file_bytes, validate_tree


_RULE_IDS = (
    "fm1_landing",
    "fm1_autonomy",
    "payload_2",
    "fm2_autonomy",
    "payload_3",
    "fm3_autonomy",
    "payload_4",
)
_EVENT_TYPES = tuple(f"competition.{rule_id}" for rule_id in _RULE_IDS) + (
    "score.finalized",
)
_MISSION_SEQUENCE = (
    ("FM1", "STARTED"),
    ("FM1", "COMPLETE"),
    ("FM2", "STARTED"),
    ("FM2", "COMPLETE"),
    ("FM3_3", "STARTED"),
    ("FM3_3", "COMPLETE"),
    ("FM3_4", "STARTED"),
    ("FM3_4", "COMPLETE"),
    ("HOME", "STARTED"),
    ("HOME", "DISARMED"),
    ("HOME", "COMPLETE"),
)
_PAYLOAD_SEQUENCE = (
    (2, "release", "detached"),
    (3, "attach", "attached"),
    (3, "release", "detached"),
    (4, "attach", "attached"),
    (4, "release", "detached"),
)
_PHASE_INDEX = {"FM1": 0, "FM2": 2, "FM3_3": 4, "FM3_4": 6, "HOME": 8}
_PAYLOAD_PREFIX_LENGTH = {2: 1, 3: 3, 4: 5}
_PICKUP_WAYPOINT = {3: "WA", 4: "WM"}
_WAYPOINTS = {
    "H": (0.0, 0.0, 4.572, 4.572),
    "L": (-91.44, 0.0, 4.572, 4.572),
    "F2": (-152.40, 0.0, 0.9144, 0.9144),
    "WA": (-45.72, -9.144, 6.096, 6.096),
    "WM": (-45.72, 9.144, 6.096, 6.096),
}
_PAYLOAD_XY_SIZE_M = (0.1524, 0.1524)
_EVIDENCE_PATHS = tuple(
    f"scoring/events.jsonl#event-{index}" for index in range(8)
) + (
    "rosbag#/simulation/ground_truth",
    "rosbag#/simulation/payload_state",
    "rosbag#/simulation/payload_events",
    "rosbag#/simulation/mission_events",
)


class CompetitionScoreValidationError(ScoreValidationError):
    """Competition score outputs disagree with independently evaluated facts."""


@dataclass(frozen=True)
class ValidatedCompetitionScoreMetadata:
    achieved_score: float
    maximum_available_score: float
    scoring_checksum: str
    evidence_paths: tuple[str, ...]
    cumulative_checkpoints: tuple[float, float, float]
    elapsed_simulated_ns: int


@dataclass(frozen=True)
class _Rules:
    checksum: str
    interval_ns: int
    pickup_tolerance_m: float
    release_tolerance_m: float
    release_speed_mps: float
    release_stability_ns: int
    settle_ns: int
    release_agl_m: float
    deadline_ns: int
    points: tuple[float, ...]


@dataclass(frozen=True)
class _ComputedScore:
    awarded: tuple[float, ...]
    achieved: float
    complete: bool
    diagnostic: str | None
    timestamp_ns: int
    elapsed_ns: int


def _finite(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CompetitionScoreValidationError(f"{field} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise CompetitionScoreValidationError(f"{field} must be a finite number")
    return number


def _safe_json(
    run_directory: Path | str,
    relative_path: str,
    deadline_check: Callable[[], None] | None,
) -> object:
    validation, payload = read_regular_file_bytes(
        run_directory, relative_path, deadline_check=deadline_check
    )
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise CompetitionScoreValidationError(
            f"{relative_path} is not safe competition evidence"
        )
    try:
        return json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CompetitionScoreValidationError(
            f"{relative_path} is not valid JSON"
        ) from error


def _safe_jsonl(
    run_directory: Path | str,
    relative_path: str,
    deadline_check: Callable[[], None] | None,
) -> list[object]:
    validation, payload = read_regular_file_bytes(
        run_directory, relative_path, deadline_check=deadline_check
    )
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise CompetitionScoreValidationError(
            f"{relative_path} is not safe competition evidence"
        )
    try:
        return [json.loads(line) for line in payload.decode("utf-8").splitlines()]
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CompetitionScoreValidationError(
            f"{relative_path} is not valid JSONL"
        ) from error


def _load_rules(path: Path | str) -> _Rules:
    try:
        payload = Path(path).read_bytes()
        document = json.loads(payload)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CompetitionScoreValidationError(
            "committed competition rules could not be decoded"
        ) from error
    expected_keys = {
        "schema_version",
        "ruleset_id",
        "sample_interval_ns",
        "pickup_center_tolerance_m",
        "release_position_tolerance_m",
        "release_horizontal_speed_mps",
        "release_stability_ns",
        "payload_settle_ns",
        "release_agl_m",
        "deadline_ns",
        "points",
    }
    if not isinstance(document, dict) or set(document) != expected_keys:
        raise CompetitionScoreValidationError("competition rules fields changed")
    if document["schema_version"] != 1 or document["ruleset_id"] != "competition_v1":
        raise CompetitionScoreValidationError("competition ruleset is incompatible")
    points_document = document["points"]
    if not isinstance(points_document, dict) or tuple(points_document) != _RULE_IDS:
        raise CompetitionScoreValidationError("competition point identities changed")
    points = tuple(_finite(points_document[key], f"points.{key}") for key in _RULE_IDS)
    exact = (
        document["sample_interval_ns"],
        _finite(document["pickup_center_tolerance_m"], "pickup tolerance"),
        _finite(document["release_position_tolerance_m"], "release tolerance"),
        _finite(document["release_horizontal_speed_mps"], "release speed"),
        document["release_stability_ns"],
        document["payload_settle_ns"],
        _finite(document["release_agl_m"], "release AGL"),
        document["deadline_ns"],
        points,
    )
    if exact != (
        50_000_000,
        0.075,
        0.15,
        0.10,
        2_000_000_000,
        1_000_000_000,
        10.0,
        600_000_000_000,
        (20.0, 30.0, 10.0, 20.0, 15.0, 50.0, 5.0),
    ):
        raise CompetitionScoreValidationError(
            "competition physical thresholds or points changed"
        )
    return _Rules(
        hashlib.sha256(payload).hexdigest(),
        exact[0],
        exact[1],
        exact[2],
        exact[3],
        exact[4],
        exact[5],
        exact[6],
        exact[7],
        exact[8],
    )


def _speed(values: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in values))


class _PhysicalOracle:
    def __init__(self, evidence: PhysicalBagEvidence, rules: _Rules) -> None:
        self.evidence = evidence
        self.rules = rules
        starts = [
            event.sim_timestamp_ns
            for event in evidence.mission_events
            if event.phase == "FM1" and event.state == "STARTED"
        ]
        if len(starts) != 1:
            raise CompetitionScoreValidationError(
                "independent evaluation requires one FM1 start"
            )
        self.start_ns = starts[0]
        self.ground_truth = tuple(
            row for row in evidence.ground_truth if row.sim_timestamp_ns >= self.start_ns
        )
        self.payload_states = {
            marker: tuple(
                row
                for row in evidence.payload_states
                if row.aruco_id == marker and row.sim_timestamp_ns >= self.start_ns
            )
            for marker in (2, 3, 4)
        }
        self.payload_events = tuple(
            row for row in evidence.payload_events if row.sim_timestamp_ns >= self.start_ns
        )
        self.mission_events = tuple(
            row for row in evidence.mission_events if row.sim_timestamp_ns >= self.start_ns
        )
        self.ranges = tuple(
            row for row in evidence.downward_ranges if row.sim_timestamp_ns >= self.start_ns
        )

    def _contiguous(self, rows: Iterable[object]) -> bool:
        timestamps = [row.sim_timestamp_ns for row in rows]  # type: ignore[attr-defined]
        return bool(timestamps) and timestamps[0] - self.start_ns <= self.rules.interval_ns and all(
            current - previous == self.rules.interval_ns
            for previous, current in zip(timestamps, timestamps[1:])
        )

    @staticmethod
    def _inside(position: tuple[float, float, float], waypoint: str) -> bool:
        center_x, center_y, width, height = _WAYPOINTS[waypoint]
        return (
            abs(position[0] - center_x) <= width / 2.0
            and abs(position[1] - center_y) <= height / 2.0
        )

    def _vehicle_at(self, timestamp_ns: int) -> GroundTruthEvidence | None:
        rows = [
            row
            for row in self.ground_truth
            if row.sim_timestamp_ns <= timestamp_ns
            and timestamp_ns - row.sim_timestamp_ns <= self.rules.interval_ns
        ]
        return rows[-1] if rows else None

    def _range_at(self, timestamp_ns: int) -> float | None:
        rows = [
            row
            for row in self.ranges
            if row.sim_timestamp_ns <= timestamp_ns
            and timestamp_ns - row.sim_timestamp_ns <= self.rules.interval_ns
        ]
        return None if not rows else rows[-1].range_m

    def _is_landed(self, row: GroundTruthEvidence, waypoint: str) -> bool:
        return (
            row.in_contact
            and _speed(row.linear_velocity_xyz) <= self.rules.release_speed_mps
            and self._inside(row.position_xyz, waypoint)
        )

    def _phase(self, phase: str) -> tuple[MissionEventEvidence, MissionEventEvidence] | None:
        index = _PHASE_INDEX[phase]
        end_index = index + (2 if phase == "HOME" else 1)
        if len(self.mission_events) <= end_index:
            return None
        if tuple((row.phase, row.state) for row in self.mission_events[: end_index + 1]) != _MISSION_SEQUENCE[: end_index + 1]:
            return None
        return self.mission_events[index], self.mission_events[end_index]

    def _landing(self, phase: str, waypoint: str) -> bool:
        events = self._phase(phase)
        if events is None:
            return False
        row = self._vehicle_at(events[1].sim_timestamp_ns)
        return row is not None and self._is_landed(row, waypoint)

    def _payload_event(self, marker: int, action: str) -> PayloadEventEvidence | None:
        return next(
            (
                row
                for row in self.payload_events
                if row.aruco_id == marker
                and row.action == action
                and row.state == ("attached" if action == "attach" else "detached")
                and row.code == "OK"
            ),
            None,
        )

    def _release_gate(self, event: PayloadEventEvidence) -> bool:
        first = event.sim_timestamp_ns - self.rules.release_stability_ns
        if first < self.start_ns:
            return False
        rows = tuple(
            row
            for row in self.ground_truth
            if first <= row.sim_timestamp_ns <= event.sim_timestamp_ns
        )
        required = self.rules.release_stability_ns // self.rules.interval_ns + 1
        center_x, center_y, _width, _height = _WAYPOINTS["F2"]
        return (
            len(rows) == required
            and rows[0].sim_timestamp_ns == first
            and rows[-1].sim_timestamp_ns == event.sim_timestamp_ns
            and all(
                current.sim_timestamp_ns - previous.sim_timestamp_ns
                == self.rules.interval_ns
                for previous, current in zip(rows, rows[1:])
            )
            and all(
                math.hypot(
                    row.position_xyz[0] - center_x, row.position_xyz[1] - center_y
                )
                <= self.rules.release_tolerance_m
                and math.hypot(*row.linear_velocity_xyz[:2])
                <= self.rules.release_speed_mps
                and row.position_xyz[2] >= self.rules.release_agl_m
                and (self._range_at(row.sim_timestamp_ns) or -1.0)
                >= self.rules.release_agl_m
                for row in rows
            )
        )

    def _physical_release(self, marker: int, event: PayloadEventEvidence) -> bool:
        before = [
            row
            for row in self.payload_states[marker]
            if row.sim_timestamp_ns <= event.sim_timestamp_ns
        ]
        detached = next(
            (
                row
                for row in self.payload_states[marker]
                if event.sim_timestamp_ns <= row.sim_timestamp_ns
                <= event.sim_timestamp_ns + self.rules.interval_ns
                and not row.attached
            ),
            None,
        )
        return bool(
            before
            and before[-1].attached
            and event.sim_timestamp_ns - before[-1].sim_timestamp_ns
            <= self.rules.interval_ns
            and detached is not None
        )

    def _physical_attach(self, marker: int, event: PayloadEventEvidence) -> bool:
        states = self.payload_states[marker]
        index = next(
            (
                index
                for index, row in enumerate(states)
                if event.sim_timestamp_ns <= row.sim_timestamp_ns
                <= event.sim_timestamp_ns + self.rules.interval_ns
                and row.attached
            ),
            None,
        )
        if index is None or index == 0:
            return False
        previous, current = states[index - 1], states[index]
        vehicle = self._vehicle_at(current.sim_timestamp_ns)
        waypoint = _PICKUP_WAYPOINT[marker]
        return bool(
            vehicle is not None
            and current.sim_timestamp_ns - previous.sim_timestamp_ns
            == self.rules.interval_ns
            and previous.grounded
            and not previous.attached
            and current.attached
            and self._inside(current.position_xyz, waypoint)
            and vehicle.in_contact
            and math.hypot(
                current.position_xyz[0] - vehicle.position_xyz[0],
                current.position_xyz[1] - vehicle.position_xyz[1],
            )
            <= self.rules.pickup_tolerance_m
            and math.dist(current.position_xyz, previous.position_xyz)
            <= self.rules.pickup_tolerance_m
        )

    @staticmethod
    def _yaw(orientation: tuple[float, float, float, float]) -> float:
        norm = math.sqrt(sum(value * value for value in orientation))
        x, y, z, w = (value / norm for value in orientation)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _inside_f2(self, row: PayloadStateEvidence) -> bool:
        yaw = self._yaw(row.orientation_xyzw)
        half_x, half_y = (size / 2.0 for size in _PAYLOAD_XY_SIZE_M)
        extent_x = abs(math.cos(yaw)) * half_x + abs(math.sin(yaw)) * half_y
        extent_y = abs(math.sin(yaw)) * half_x + abs(math.cos(yaw)) * half_y
        center_x, center_y, width, height = _WAYPOINTS["F2"]
        return (
            abs(row.position_xyz[0] - center_x) + extent_x <= width / 2.0
            and abs(row.position_xyz[1] - center_y) + extent_y <= height / 2.0
        )

    def _settled(self, marker: int, release: PayloadEventEvidence) -> int | None:
        started: int | None = None
        previous: int | None = None
        for row in self.payload_states[marker]:
            if row.sim_timestamp_ns <= release.sim_timestamp_ns:
                continue
            eligible = (
                row.grounded
                and not row.attached
                and _speed(row.linear_velocity_xyz) <= self.rules.release_speed_mps
                and self._inside_f2(row)
            )
            if not eligible or previous is None or row.sim_timestamp_ns - previous != self.rules.interval_ns:
                started = row.sim_timestamp_ns if eligible else None
            if eligible and started is not None:
                if row.sim_timestamp_ns - started >= self.rules.settle_ns:
                    return row.sim_timestamp_ns
                previous = row.sim_timestamp_ns
            else:
                previous = None
        return None

    def _capacity_valid(self) -> bool:
        grouped: dict[int, list[PayloadStateEvidence]] = {}
        for rows in self.payload_states.values():
            for row in rows:
                grouped.setdefault(row.sim_timestamp_ns, []).append(row)
        latest: dict[int, PayloadStateEvidence] = {}
        for timestamp_ns in sorted(grouped):
            for row in grouped[timestamp_ns]:
                latest[row.aruco_id] = row
            if sum(row.attached for row in latest.values()) > 1:
                return False
        return True

    def _delivery(
        self, marker: int, phase: str, boundary_ns: int | None
    ) -> tuple[bool, int | None]:
        phase_events = self._phase(phase)
        release = self._payload_event(marker, "release")
        if phase_events is None or release is None:
            return False, None
        prefix = self.payload_events[: _PAYLOAD_PREFIX_LENGTH[marker]]
        prefix_valid = tuple((row.aruco_id, row.action, row.state) for row in prefix) == _PAYLOAD_SEQUENCE[: len(prefix)] and len(prefix) == _PAYLOAD_PREFIX_LENGTH[marker] and all(row.code == "OK" for row in prefix)
        valid = (
            phase_events[0].sim_timestamp_ns
            <= release.sim_timestamp_ns
            <= phase_events[1].sim_timestamp_ns
            and prefix_valid
            and self._release_gate(release)
            and self._physical_release(marker, release)
        )
        if marker in (3, 4):
            attach = self._payload_event(marker, "attach")
            valid = bool(
                valid
                and attach is not None
                and phase_events[0].sim_timestamp_ns
                <= attach.sim_timestamp_ns
                < release.sim_timestamp_ns
                and self._physical_attach(marker, attach)
            )
        settled = self._settled(marker, release) if valid else None
        if settled is not None and boundary_ns is not None and settled >= boundary_ns:
            settled = None
        return settled is not None, settled

    def _home(self) -> tuple[bool, int]:
        events = self._phase("HOME")
        if events is None or len(self.mission_events) != 11:
            return False, 0
        started = self.mission_events[8]
        disarmed = self.mission_events[9]
        complete = self.mission_events[10]
        elapsed = complete.sim_timestamp_ns - self.start_ns
        if not started.sim_timestamp_ns < disarmed.sim_timestamp_ns < complete.sim_timestamp_ns:
            return False, elapsed
        landing = next(
            (
                row
                for row in self.ground_truth
                if row.sim_timestamp_ns >= started.sim_timestamp_ns
                and self._is_landed(row, "H")
            ),
            None,
        )
        if landing is None or landing.sim_timestamp_ns >= disarmed.sim_timestamp_ns:
            return False, elapsed
        through_complete = tuple(
            row
            for row in self.ground_truth
            if landing.sim_timestamp_ns <= row.sim_timestamp_ns <= complete.sim_timestamp_ns
        )
        return bool(through_complete and all(self._is_landed(row, "H") for row in through_complete)), elapsed

    def compute(self) -> _ComputedScore:
        streams_valid = self._contiguous(self.ground_truth) and self._contiguous(self.ranges) and all(
            self._contiguous(rows) for rows in self.payload_states.values()
        )
        mission_valid = (
            tuple((row.phase, row.state) for row in self.mission_events)
            == _MISSION_SEQUENCE
            and tuple(row.event_id for row in self.mission_events) == tuple(range(11))
        )
        payload_valid = (
            tuple((row.aruco_id, row.action, row.state) for row in self.payload_events)
            == _PAYLOAD_SEQUENCE
            and tuple(row.event_id for row in self.payload_events) == tuple(range(5))
            and all(row.code == "OK" for row in self.payload_events)
        )
        capacity = self._capacity_valid()
        marker3_attach = self._payload_event(3, "attach")
        marker4_attach = self._payload_event(4, "attach")
        home_started = self.mission_events[8] if len(self.mission_events) > 8 else None
        home_boundary_candidates = []
        if home_started is not None:
            home_boundary_candidates.extend(
                row.sim_timestamp_ns for row in self.mission_events[9:]
            )
            landing = next(
                (
                    row.sim_timestamp_ns
                    for row in self.ground_truth
                    if row.sim_timestamp_ns >= home_started.sim_timestamp_ns
                    and self._is_landed(row, "H")
                ),
                None,
            )
            if landing is not None:
                home_boundary_candidates.append(landing)
        home_boundary = min(home_boundary_candidates) if home_boundary_candidates else None

        fm1_landing = self._landing("FM1", "L")
        fm1_autonomy = fm1_landing and tuple((row.phase, row.state) for row in self.mission_events[:2]) == _MISSION_SEQUENCE[:2]
        payload2, settled2 = self._delivery(2, "FM2", None if marker3_attach is None else marker3_attach.sim_timestamp_ns)
        payload2 = payload2 and fm1_autonomy
        fm2_autonomy = payload2 and tuple((row.phase, row.state) for row in self.mission_events[:4]) == _MISSION_SEQUENCE[:4]
        payload3, settled3 = self._delivery(3, "FM3_3", None if marker4_attach is None else marker4_attach.sim_timestamp_ns)
        payload3 = payload3 and fm2_autonomy and capacity
        fm3_autonomy = payload3 and tuple((row.phase, row.state) for row in self.mission_events[:6]) == _MISSION_SEQUENCE[:6]
        payload4, settled4 = self._delivery(4, "FM3_4", home_boundary)
        payload4 = payload4 and fm3_autonomy and capacity
        outcomes = (fm1_landing, fm1_autonomy, payload2, fm2_autonomy, payload3, fm3_autonomy, payload4)

        ordering = (
            (settled2 is None or marker3_attach is None or settled2 < marker3_attach.sim_timestamp_ns)
            and (settled3 is None or marker4_attach is None or settled3 < marker4_attach.sim_timestamp_ns)
            and (settled4 is None or home_boundary is None or settled4 < home_boundary)
        )
        home, elapsed = self._home()
        complete = streams_valid and mission_valid and payload_valid and capacity and ordering and home and elapsed <= self.rules.deadline_ns
        if not streams_valid:
            diagnostic = "physical_stream_invalid"
        elif not mission_valid:
            diagnostic = "mission_sequence_invalid"
        elif not payload_valid:
            diagnostic = "payload_event_sequence_invalid"
        elif not capacity:
            diagnostic = "payload_capacity_exceeded"
        elif not ordering:
            diagnostic = "physical_sequence_invalid"
        elif not home:
            diagnostic = "home_completion_invalid"
        elif elapsed > self.rules.deadline_ns:
            diagnostic = "home_deadline_exceeded"
        else:
            diagnostic = None
        awarded = tuple(points if passed else 0.0 for points, passed in zip(self.rules.points, outcomes, strict=True))
        timestamp = max(
            (
                row.sim_timestamp_ns
                for rows in (
                    self.ground_truth,
                    *self.payload_states.values(),
                    self.payload_events,
                    self.mission_events,
                    self.ranges,
                )
                for row in rows
            ),
            default=0,
        )
        return _ComputedScore(awarded, sum(awarded), complete, diagnostic, timestamp, elapsed)


def _expected_events(run_id: str, computed: _ComputedScore) -> tuple[ScoreEventEvidence, ...]:
    values = (*computed.awarded, computed.achieved)
    return tuple(
        ScoreEventEvidence(
            computed.timestamp_ns,
            index,
            event_type,
            value,
            f"scoring/events.jsonl#event-{index}",
        )
        for index, (event_type, value) in enumerate(zip(_EVENT_TYPES, values, strict=True))
    )


def validate_competition_score_outputs(
    run_directory: Path | str,
    *,
    run_id: str,
    rules_path: Path | str,
    physical_evidence: PhysicalBagEvidence | None,
    deadline_check: Callable[[], None] | None = None,
) -> ValidatedCompetitionScoreMetadata:
    """Recompute and validate all seven competition score components."""
    if physical_evidence is None:
        raise CompetitionScoreValidationError(
            "independent competition validation requires physical evidence"
        )
    rules = _load_rules(rules_path)
    before = validate_tree(run_directory, "rosbag", deadline_check=deadline_check)
    if before.status is not ValidationStatus.VALID or before.sha256 != physical_evidence.bag_sha256:
        raise CompetitionScoreValidationError("independent physical bag digest changed")
    computed = _PhysicalOracle(physical_evidence, rules).compute()

    result = _safe_json(run_directory, "scoring/result.json", deadline_check)
    if not isinstance(result, dict):
        raise CompetitionScoreValidationError("scoring/result.json must be an object")
    expected_rows = [
        {
            "rule_id": rule_id,
            "passed": awarded == available,
            "awarded_points": awarded,
            "available_points": available,
            "evidence_ref": f"rosbag#competition:{rule_id}",
        }
        for rule_id, available, awarded in zip(
            _RULE_IDS, rules.points, computed.awarded, strict=True
        )
    ]
    expected_result = {
        "run_id": run_id,
        "ruleset_id": "competition_v1",
        "complete": computed.complete,
        "achieved_score": computed.achieved,
        "maximum_available_score": 150.0,
        "scoring_checksum": rules.checksum,
        "evidence_paths": list(_EVIDENCE_PATHS),
        "rule_results": expected_rows,
        "diagnostic": computed.diagnostic,
    }
    if result != expected_result:
        raise CompetitionScoreValidationError(
            "persisted result does not match independent physical evaluation"
        )

    expected_events = _expected_events(run_id, computed)
    event_rows = _safe_jsonl(run_directory, "scoring/events.jsonl", deadline_check)
    expected_event_rows = [
        {
            "run_id": run_id,
            "sim_timestamp_ns": event.sim_timestamp_ns,
            "event_id": event.event_id,
            "event_type": event.event_type,
            "value": event.value,
            "evidence_ref": event.evidence_ref,
        }
        for event in expected_events
    ]
    if event_rows != expected_event_rows or physical_evidence.score_events != expected_events:
        raise CompetitionScoreValidationError(
            "persisted score events do not match independent physical evaluation"
        )
    after = validate_tree(run_directory, "rosbag", deadline_check=deadline_check)
    if after.status is not ValidationStatus.VALID or after.sha256 != physical_evidence.bag_sha256:
        raise CompetitionScoreValidationError("independent physical bag digest changed")
    return ValidatedCompetitionScoreMetadata(
        computed.achieved,
        150.0,
        rules.checksum,
        _EVIDENCE_PATHS,
        (
            sum(computed.awarded[:4]),
            sum(computed.awarded[:6]),
            sum(computed.awarded),
        ),
        computed.elapsed_ns,
    )


__all__ = [
    "CompetitionScoreValidationError",
    "ValidatedCompetitionScoreMetadata",
    "validate_competition_score_outputs",
]
