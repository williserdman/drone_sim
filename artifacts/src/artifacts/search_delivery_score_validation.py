"""Independent validation of completed ``search_delivery_v1`` scoring evidence."""

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
    MissionEventEvidence,
    PayloadEventEvidence,
    PayloadStateEvidence,
    PhysicalBagEvidence,
    ScoreEventEvidence,
)
from .score_validation import ScoreValidationError, ValidatedScoreMetadata
from .validation import ValidationStatus, read_regular_file_bytes, validate_tree


_RULE_IDS = ("search", "pickup", "delivery", "home")
_EVENT_TYPES = tuple(f"search_delivery.{rule_id}" for rule_id in _RULE_IDS) + (
    "score.finalized",
)
_MISSION_SEQUENCE = (
    ("SEARCH", "STARTED"),
    ("SEARCH", "COMPLETE"),
    ("DELIVERY", "STARTED"),
    ("DELIVERY", "COMPLETE"),
    ("HOME", "STARTED"),
    ("HOME", "DISARMED"),
    ("HOME", "COMPLETE"),
)
_PAYLOAD_SEQUENCE = ((3, "attach", "attached"), (3, "release", "detached"))
_STAGING = (16.0, 8.0)
_WAYPOINTS = {
    "H": (0.0, 0.0, 4.572, 4.572),
    "WA": (18.0, 8.0, 6.096, 6.096),
    "F2": (6.0, 20.0, 0.9144, 0.9144),
}
_PAYLOAD_XY_SIZE_M = (0.1524, 0.1524)
_EVIDENCE_PATHS = tuple(
    f"scoring/events.jsonl#event-{index}" for index in range(5)
) + (
    "rosbag#/simulation/ground_truth",
    "rosbag#/simulation/payload_state",
    "rosbag#/simulation/payload_events",
    "rosbag#/simulation/mission_events",
)


class SearchDeliveryScoreValidationError(ScoreValidationError):
    """Search-delivery outputs disagree with independently evaluated facts."""


@dataclass(frozen=True)
class ValidatedSearchDeliveryScoreMetadata(ValidatedScoreMetadata):
    elapsed_simulated_ns: int


@dataclass(frozen=True)
class _Rules:
    checksum: str
    interval_ns: int
    staging_tolerance_m: float
    altitude_min_m: float
    altitude_max_m: float
    east_progress_m: float
    pickup_tolerance_m: float
    pickup_lift_m: float
    release_tolerance_m: float
    release_speed_mps: float
    release_max_tilt_rad: float
    release_stability_ns: int
    post_release_hold_ns: int
    settle_ns: int
    release_agl_m: float
    deadline_ns: int
    points: tuple[float, float, float, float]


@dataclass(frozen=True)
class _ComputedScore:
    awarded: tuple[float, float, float, float]
    achieved: float
    complete: bool
    diagnostic: str | None
    timestamp_ns: int
    elapsed_ns: int


def _finite(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SearchDeliveryScoreValidationError(f"{field} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise SearchDeliveryScoreValidationError(f"{field} must be a finite number")
    return result


def _safe_json(
    run_directory: Path | str,
    relative_path: str,
    deadline_check: Callable[[], None] | None,
) -> object:
    validation, payload = read_regular_file_bytes(
        run_directory, relative_path, deadline_check=deadline_check
    )
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise SearchDeliveryScoreValidationError(
            f"{relative_path} is not safe search delivery evidence"
        )
    try:
        return json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SearchDeliveryScoreValidationError(
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
        raise SearchDeliveryScoreValidationError(
            f"{relative_path} is not safe search delivery evidence"
        )
    try:
        return [json.loads(line) for line in payload.decode("utf-8").splitlines()]
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SearchDeliveryScoreValidationError(
            f"{relative_path} is not valid JSONL"
        ) from error


def _load_rules(path: Path | str) -> _Rules:
    try:
        payload = Path(path).read_bytes()
        document = json.loads(payload)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SearchDeliveryScoreValidationError(
            "committed search delivery rules could not be decoded"
        ) from error
    expected_keys = {
        "schema_version",
        "ruleset_id",
        "sample_interval_ns",
        "search_staging_tolerance_m",
        "search_altitude_min_m",
        "search_altitude_max_m",
        "search_east_progress_m",
        "pickup_center_tolerance_m",
        "pickup_lift_m",
        "release_position_tolerance_m",
        "release_horizontal_speed_mps",
        "release_max_tilt_rad",
        "release_stability_ns",
        "post_release_hold_ns",
        "payload_settle_ns",
        "release_agl_m",
        "deadline_ns",
        "points",
    }
    if not isinstance(document, dict) or set(document) != expected_keys:
        raise SearchDeliveryScoreValidationError("search delivery rules fields changed")
    points_document = document["points"]
    if not isinstance(points_document, dict) or tuple(points_document) != _RULE_IDS:
        raise SearchDeliveryScoreValidationError(
            "search delivery point identities changed"
        )
    points = tuple(_finite(points_document[key], f"points.{key}") for key in _RULE_IDS)
    exact = (
        document["schema_version"],
        document["ruleset_id"],
        document["sample_interval_ns"],
        _finite(document["search_staging_tolerance_m"], "staging tolerance"),
        _finite(document["search_altitude_min_m"], "minimum search altitude"),
        _finite(document["search_altitude_max_m"], "maximum search altitude"),
        _finite(document["search_east_progress_m"], "east progress"),
        _finite(document["pickup_center_tolerance_m"], "pickup tolerance"),
        _finite(document["pickup_lift_m"], "pickup lift"),
        _finite(document["release_position_tolerance_m"], "release tolerance"),
        _finite(document["release_horizontal_speed_mps"], "release speed"),
        _finite(document["release_max_tilt_rad"], "release tilt"),
        document["release_stability_ns"],
        document["post_release_hold_ns"],
        document["payload_settle_ns"],
        _finite(document["release_agl_m"], "release AGL"),
        document["deadline_ns"],
        points,
    )
    if exact != (
        1,
        "search_delivery_v1",
        50_000_000,
        0.30,
        4.25,
        4.90,
        0.75,
        0.075,
        0.25,
        0.15,
        0.10,
        0.10,
        2_000_000_000,
        3_000_000_000,
        1_000_000_000,
        10.0,
        240_000_000_000,
        (25.0, 25.0, 25.0, 25.0),
    ):
        raise SearchDeliveryScoreValidationError(
            "search delivery physical thresholds or points changed"
        )
    return _Rules(
        hashlib.sha256(payload).hexdigest(),
        exact[2],
        exact[3],
        exact[4],
        exact[5],
        exact[6],
        exact[7],
        exact[8],
        exact[9],
        exact[10],
        exact[11],
        exact[12],
        exact[13],
        exact[14],
        exact[15],
        exact[16],
        exact[17],
    )


def _speed(values: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def _tilt(orientation: tuple[float, float, float, float]) -> float:
    norm = math.sqrt(sum(value * value for value in orientation))
    if norm == 0.0:
        return math.inf
    x, y, _z, w = (value / norm for value in orientation)
    world_up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, world_up_z)))


class _PhysicalOracle:
    def __init__(self, evidence: PhysicalBagEvidence, rules: _Rules) -> None:
        self.evidence = evidence
        self.rules = rules
        starts = [
            event.sim_timestamp_ns
            for event in evidence.mission_events
            if event.phase == "SEARCH" and event.state == "STARTED"
        ]
        if len(starts) != 1:
            raise SearchDeliveryScoreValidationError(
                "independent evaluation requires one SEARCH start"
            )
        self.start_ns = starts[0]
        self.ground_truth = tuple(
            row for row in evidence.ground_truth if row.sim_timestamp_ns >= self.start_ns
        )
        self.payload_states = tuple(
            row
            for row in evidence.payload_states
            if row.aruco_id == 3 and row.sim_timestamp_ns >= self.start_ns
        )
        self.payload_events = tuple(
            row for row in evidence.payload_events if row.sim_timestamp_ns >= self.start_ns
        )
        self.mission_events = tuple(
            row for row in evidence.mission_events if row.sim_timestamp_ns >= self.start_ns
        )

    def _contiguous(self, rows: tuple[object, ...]) -> bool:
        timestamps = [row.sim_timestamp_ns for row in rows]  # type: ignore[attr-defined]
        return bool(timestamps) and timestamps[0] == self.start_ns and all(
            current - previous == self.rules.interval_ns
            for previous, current in zip(timestamps, timestamps[1:])
        )

    @staticmethod
    def _inside(position: tuple[float, float, float], waypoint: str) -> bool:
        x, y, width, height = _WAYPOINTS[waypoint]
        return (
            abs(position[0] - x) <= width / 2.0
            and abs(position[1] - y) <= height / 2.0
        )

    def _vehicle_at(self, timestamp_ns: int) -> GroundTruthEvidence | None:
        rows = [
            row
            for row in self.ground_truth
            if row.sim_timestamp_ns <= timestamp_ns
            and timestamp_ns - row.sim_timestamp_ns <= self.rules.interval_ns
        ]
        return rows[-1] if rows else None

    def _payload_at(self, timestamp_ns: int) -> PayloadStateEvidence | None:
        rows = [
            row
            for row in self.payload_states
            if row.sim_timestamp_ns <= timestamp_ns
            and timestamp_ns - row.sim_timestamp_ns <= self.rules.interval_ns
        ]
        return rows[-1] if rows else None

    def _phase(self, phase: str) -> tuple[MissionEventEvidence, MissionEventEvidence]:
        index = {"SEARCH": 0, "DELIVERY": 2, "HOME": 4}[phase]
        end_index = index + (2 if phase == "HOME" else 1)
        return self.mission_events[index], self.mission_events[end_index]

    def _payload_event(self, action: str) -> PayloadEventEvidence | None:
        return next(
            (
                row
                for row in self.payload_events
                if row.aruco_id == 3
                and row.action == action
                and row.state == ("attached" if action == "attach" else "detached")
                and row.code == "OK"
            ),
            None,
        )

    def _search(self) -> bool:
        started, complete = self._phase("SEARCH")
        staging = tuple(
            row
            for row in self.ground_truth
            if started.sim_timestamp_ns <= row.sim_timestamp_ns <= complete.sim_timestamp_ns
            and self.rules.altitude_min_m <= row.position_xyz[2] <= self.rules.altitude_max_m
            and math.hypot(
                row.position_xyz[0] - _STAGING[0], row.position_xyz[1] - _STAGING[1]
            )
            <= self.rules.staging_tolerance_m
        )
        marker_x, marker_y, _width, _height = _WAYPOINTS["WA"]
        return any(
            later.sim_timestamp_ns > first.sim_timestamp_ns
            and later.sim_timestamp_ns <= complete.sim_timestamp_ns
            and later.position_xyz[0] - first.position_xyz[0]
            >= self.rules.east_progress_m
            and self.rules.altitude_min_m
            <= later.position_xyz[2]
            <= self.rules.altitude_max_m
            and math.hypot(
                later.position_xyz[0] - marker_x, later.position_xyz[1] - marker_y
            )
            < math.hypot(
                first.position_xyz[0] - marker_x, first.position_xyz[1] - marker_y
            )
            for first in staging
            for later in self.ground_truth
        )

    def _physical_attach(
        self, event: PayloadEventEvidence
    ) -> tuple[bool, PayloadStateEvidence | None]:
        index = next(
            (
                index
                for index, row in enumerate(self.payload_states)
                if event.sim_timestamp_ns
                <= row.sim_timestamp_ns
                <= event.sim_timestamp_ns + self.rules.interval_ns
                and row.attached
            ),
            None,
        )
        if index is None or index == 0:
            return False, None
        previous, current = self.payload_states[index - 1], self.payload_states[index]
        vehicle = self._vehicle_at(current.sim_timestamp_ns)
        valid = bool(
            current.sim_timestamp_ns - previous.sim_timestamp_ns
            == self.rules.interval_ns
            and previous.grounded
            and not previous.attached
            and current.attached
            and self._inside(current.position_xyz, "WA")
            and vehicle is not None
            and vehicle.in_contact
            and math.hypot(
                current.position_xyz[0] - vehicle.position_xyz[0],
                current.position_xyz[1] - vehicle.position_xyz[1],
            )
            <= self.rules.pickup_tolerance_m
            and math.dist(current.position_xyz, previous.position_xyz)
            <= self.rules.pickup_tolerance_m
        )
        return valid, previous if valid else None

    def _pickup(self) -> bool:
        search_start, search_complete = self._phase("SEARCH")
        delivery_start, _delivery_complete = self._phase("DELIVERY")
        attach = self._payload_event("attach")
        if (
            attach is None
            or not search_start.sim_timestamp_ns
            <= attach.sim_timestamp_ns
            < search_complete.sim_timestamp_ns
        ):
            return False
        attached, grounded = self._physical_attach(attach)
        complete_vehicle = self._vehicle_at(search_complete.sim_timestamp_ns)
        complete_payload = self._payload_at(search_complete.sim_timestamp_ns)
        return bool(
            attached
            and grounded is not None
            and complete_vehicle is not None
            and complete_vehicle.in_contact
            and complete_payload is not None
            and complete_payload.attached
            and any(
                row.sim_timestamp_ns >= delivery_start.sim_timestamp_ns
                and row.attached
                and not row.grounded
                and row.position_xyz[2]
                >= grounded.position_xyz[2] + self.rules.pickup_lift_m
                for row in self.payload_states
            )
        )

    def _release_stable(self, row: GroundTruthEvidence) -> bool:
        x, y, _width, _height = _WAYPOINTS["F2"]
        return (
            math.hypot(row.position_xyz[0] - x, row.position_xyz[1] - y)
            <= self.rules.release_tolerance_m
            and math.hypot(*row.linear_velocity_xyz[:2])
            <= self.rules.release_speed_mps
            and row.position_xyz[2] >= self.rules.release_agl_m
            and _tilt(row.orientation_xyzw) <= self.rules.release_max_tilt_rad
        )

    def _continuous_window(self, start_ns: int, end_ns: int) -> bool:
        rows = tuple(
            row
            for row in self.ground_truth
            if start_ns <= row.sim_timestamp_ns <= end_ns
        )
        required = (end_ns - start_ns) // self.rules.interval_ns + 1
        return (
            len(rows) == required
            and rows[0].sim_timestamp_ns == start_ns
            and rows[-1].sim_timestamp_ns == end_ns
            and all(self._release_stable(row) for row in rows)
        )

    def _physical_release(self, event: PayloadEventEvidence) -> bool:
        before = [
            row for row in self.payload_states if row.sim_timestamp_ns < event.sim_timestamp_ns
        ]
        detached = next(
            (
                row
                for row in self.payload_states
                if event.sim_timestamp_ns
                < row.sim_timestamp_ns
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

    @staticmethod
    def _yaw(orientation: tuple[float, float, float, float]) -> float:
        norm = math.sqrt(sum(value * value for value in orientation))
        if norm == 0.0:
            return math.inf
        x, y, z, w = (value / norm for value in orientation)
        return math.atan2(
            2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
        )

    def _inside_f2(self, row: PayloadStateEvidence) -> bool:
        yaw = self._yaw(row.orientation_xyzw)
        half_x, half_y = (size / 2.0 for size in _PAYLOAD_XY_SIZE_M)
        extent_x = abs(math.cos(yaw)) * half_x + abs(math.sin(yaw)) * half_y
        extent_y = abs(math.sin(yaw)) * half_x + abs(math.cos(yaw)) * half_y
        x, y, width, height = _WAYPOINTS["F2"]
        return (
            abs(row.position_xyz[0] - x) + extent_x <= width / 2.0
            and abs(row.position_xyz[1] - y) + extent_y <= height / 2.0
        )

    def _settled_before(self, release_ns: int, boundary_ns: int) -> bool:
        started_ns: int | None = None
        previous_ns: int | None = None
        for row in self.payload_states:
            if row.sim_timestamp_ns <= release_ns or row.sim_timestamp_ns > boundary_ns:
                continue
            eligible = (
                row.grounded
                and not row.attached
                and _speed(row.linear_velocity_xyz) <= self.rules.release_speed_mps
                and self._inside_f2(row)
            )
            if (
                not eligible
                or previous_ns is None
                or row.sim_timestamp_ns - previous_ns != self.rules.interval_ns
            ):
                started_ns = row.sim_timestamp_ns if eligible else None
            if eligible and started_ns is not None:
                if row.sim_timestamp_ns - started_ns >= self.rules.settle_ns:
                    return True
                previous_ns = row.sim_timestamp_ns
            else:
                previous_ns = None
        return False

    def _delivery(self) -> bool:
        started, complete = self._phase("DELIVERY")
        release = self._payload_event("release")
        if (
            release is None
            or not started.sim_timestamp_ns
            <= release.sim_timestamp_ns
            < complete.sim_timestamp_ns
        ):
            return False
        hold_end = release.sim_timestamp_ns + self.rules.post_release_hold_ns
        return (
            self._pickup()
            and complete.sim_timestamp_ns >= hold_end
            and self._continuous_window(
                release.sim_timestamp_ns - self.rules.release_stability_ns,
                release.sim_timestamp_ns,
            )
            and self._continuous_window(release.sim_timestamp_ns, hold_end)
            and self._physical_release(release)
            and self._settled_before(
                release.sim_timestamp_ns, complete.sim_timestamp_ns
            )
        )

    def _landed_home(self, row: GroundTruthEvidence | None) -> bool:
        return bool(
            row is not None
            and row.in_contact
            and _speed(row.linear_velocity_xyz) <= self.rules.release_speed_mps
            and self._inside(row.position_xyz, "H")
        )

    def _home(self) -> bool:
        delivery_start, delivery_complete = self._phase("DELIVERY")
        home_start, home_complete = self._phase("HOME")
        disarmed = self.mission_events[5]
        if not (
            delivery_start.sim_timestamp_ns
            < delivery_complete.sim_timestamp_ns
            < home_start.sim_timestamp_ns
        ):
            return False
        landing = next(
            (
                row
                for row in self.ground_truth
                if row.sim_timestamp_ns >= home_start.sim_timestamp_ns
                and self._landed_home(row)
            ),
            None,
        )
        return bool(
            landing is not None
            and landing.sim_timestamp_ns
            < disarmed.sim_timestamp_ns
            < home_complete.sim_timestamp_ns
            and all(
                self._landed_home(row)
                for row in self.ground_truth
                if landing.sim_timestamp_ns
                <= row.sim_timestamp_ns
                <= home_complete.sim_timestamp_ns
            )
            and self._landed_home(self._vehicle_at(home_complete.sim_timestamp_ns))
        )

    def compute(self) -> _ComputedScore:
        mission_valid = (
            tuple((row.phase, row.state) for row in self.mission_events)
            == _MISSION_SEQUENCE
            and tuple(row.event_id for row in self.mission_events) == tuple(range(7))
        )
        payload_valid = (
            tuple((row.aruco_id, row.action, row.state) for row in self.payload_events)
            == _PAYLOAD_SEQUENCE
            and tuple(row.event_id for row in self.payload_events) == (0, 1)
            and all(row.code == "OK" for row in self.payload_events)
        )
        streams_valid = self._contiguous(self.ground_truth) and self._contiguous(
            self.payload_states
        )
        if not streams_valid:
            outcomes = (False, False, False, False)
            complete, diagnostic = False, "physical_stream_invalid"
            elapsed = 0
        elif not mission_valid:
            outcomes = (False, False, False, False)
            complete, diagnostic = False, "mission_sequence_invalid"
            elapsed = 0
        elif not payload_valid:
            outcomes = (False, False, False, False)
            complete, diagnostic = False, "payload_event_sequence_invalid"
            elapsed = 0
        else:
            outcomes = (self._search(), self._pickup(), self._delivery(), self._home())
            elapsed = self.mission_events[-1].sim_timestamp_ns - self.start_ns
            complete = outcomes[3] and elapsed <= self.rules.deadline_ns
            diagnostic = (
                None
                if complete
                else "home_completion_invalid"
                if not outcomes[3]
                else "home_deadline_exceeded"
            )
        awarded = tuple(
            points if passed else 0.0
            for points, passed in zip(self.rules.points, outcomes, strict=True)
        )
        timestamp = max(
            (
                row.sim_timestamp_ns
                for rows in (
                    self.ground_truth,
                    self.payload_states,
                    self.payload_events,
                    self.mission_events,
                )
                for row in rows
            ),
            default=0,
        )
        return _ComputedScore(
            (awarded[0], awarded[1], awarded[2], awarded[3]),
            sum(awarded),
            complete,
            diagnostic,
            timestamp,
            elapsed,
        )


def _expected_events(computed: _ComputedScore) -> tuple[ScoreEventEvidence, ...]:
    return tuple(
        ScoreEventEvidence(
            computed.timestamp_ns,
            index,
            event_type,
            value,
            f"scoring/events.jsonl#event-{index}",
        )
        for index, (event_type, value) in enumerate(
            zip(_EVENT_TYPES, (*computed.awarded, computed.achieved), strict=True)
        )
    )


def validate_search_delivery_score_outputs(
    run_directory: Path | str,
    *,
    run_id: str,
    rules_path: Path | str,
    physical_evidence: PhysicalBagEvidence | None,
    deadline_check: Callable[[], None] | None = None,
) -> ValidatedSearchDeliveryScoreMetadata:
    """Recompute and validate all four search-delivery score components."""
    if physical_evidence is None:
        raise SearchDeliveryScoreValidationError(
            "independent search delivery validation requires physical evidence"
        )
    rules = _load_rules(rules_path)
    before = validate_tree(run_directory, "rosbag", deadline_check=deadline_check)
    if (
        before.status is not ValidationStatus.VALID
        or before.sha256 != physical_evidence.bag_sha256
    ):
        raise SearchDeliveryScoreValidationError(
            "independent physical bag digest changed"
        )
    computed = _PhysicalOracle(physical_evidence, rules).compute()
    result = _safe_json(run_directory, "scoring/result.json", deadline_check)
    expected_result = {
        "run_id": run_id,
        "ruleset_id": "search_delivery_v1",
        "complete": computed.complete,
        "achieved_score": computed.achieved,
        "maximum_available_score": 100.0,
        "scoring_checksum": rules.checksum,
        "evidence_paths": list(_EVIDENCE_PATHS),
        "rule_results": [
            {
                "rule_id": rule_id,
                "passed": awarded == available,
                "awarded_points": awarded,
                "available_points": available,
                "evidence_ref": f"rosbag#search_delivery:{rule_id}",
            }
            for rule_id, available, awarded in zip(
                _RULE_IDS, rules.points, computed.awarded, strict=True
            )
        ],
        "diagnostic": computed.diagnostic,
    }
    if result != expected_result:
        raise SearchDeliveryScoreValidationError(
            "persisted result does not match independent physical evaluation"
        )
    expected_events = _expected_events(computed)
    event_rows = _safe_jsonl(run_directory, "scoring/events.jsonl", deadline_check)
    if event_rows != [
        {
            "run_id": run_id,
            "sim_timestamp_ns": event.sim_timestamp_ns,
            "event_id": event.event_id,
            "event_type": event.event_type,
            "value": event.value,
            "evidence_ref": event.evidence_ref,
        }
        for event in expected_events
    ] or physical_evidence.score_events != expected_events:
        raise SearchDeliveryScoreValidationError(
            "persisted score events do not match independent physical evaluation"
        )
    after = validate_tree(run_directory, "rosbag", deadline_check=deadline_check)
    if (
        after.status is not ValidationStatus.VALID
        or after.sha256 != physical_evidence.bag_sha256
    ):
        raise SearchDeliveryScoreValidationError(
            "independent physical bag digest changed"
        )
    return ValidatedSearchDeliveryScoreMetadata(
        computed.achieved,
        100.0,
        rules.checksum,
        _EVIDENCE_PATHS,
        computed.elapsed_ns,
    )


__all__ = [
    "SearchDeliveryScoreValidationError",
    "ValidatedSearchDeliveryScoreMetadata",
    "validate_search_delivery_score_outputs",
]
