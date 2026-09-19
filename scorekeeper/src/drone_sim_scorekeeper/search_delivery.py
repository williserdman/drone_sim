"""Read-only physical scoring for ``search_delivery_v1``."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any
from uuid import UUID

from .competition import MissionEventSample, PayloadEventSample, PayloadStateSample
from .descent import GroundTruthSample
from .models import RuleResult, ScoreEvent, ScoreResult


_RULE_IDS = ("search", "pickup", "delivery", "home")
_EVENT_TYPES = tuple(f"search_delivery.{rule_id}" for rule_id in _RULE_IDS)
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
_PHASE_INDEX = {"SEARCH": 0, "DELIVERY": 2, "HOME": 4}
_STAGING = (16.0, 8.0)
_WAYPOINTS = {
    "H": (0.0, 0.0, 4.572, 4.572),
    "WA": (18.0, 8.0, 6.096, 6.096),
    "F2": (6.0, 20.0, 0.9144, 0.9144),
}
_PAYLOAD_XY_SIZE_M = (0.1524, 0.1524)


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


@dataclass(frozen=True)
class SearchDeliveryRules:
    ruleset_id: str
    scoring_checksum: str
    sample_interval_ns: int
    search_staging_tolerance_m: float
    search_altitude_min_m: float
    search_altitude_max_m: float
    search_east_progress_m: float
    pickup_center_tolerance_m: float
    pickup_lift_m: float
    release_position_tolerance_m: float
    release_horizontal_speed_mps: float
    release_max_tilt_rad: float
    release_stability_ns: int
    post_release_hold_ns: int
    payload_settle_ns: int
    release_agl_m: float
    deadline_ns: int
    points: tuple[float, float, float, float]

    @property
    def maximum_available_score(self) -> float:
        return sum(self.points)


def load_search_delivery_rules(path: Path | str) -> SearchDeliveryRules:
    """Load and checksum the frozen search-and-delivery policy."""
    payload = Path(path).read_bytes()
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("search delivery rules must be UTF-8 JSON") from error
    expected = {
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
    if not isinstance(document, dict) or set(document) != expected:
        raise ValueError("search delivery rules fields changed")
    if (
        document.get("schema_version") != 1
        or document.get("ruleset_id") != "search_delivery_v1"
    ):
        raise ValueError("unsupported search delivery ruleset")
    interval = document.get("sample_interval_ns")
    if interval != 50_000_000 or isinstance(interval, bool):
        raise ValueError("search_delivery_v1 sample interval must be 50000000 ns")
    durations = tuple(
        document.get(name)
        for name in (
            "release_stability_ns",
            "post_release_hold_ns",
            "payload_settle_ns",
            "deadline_ns",
        )
    )
    if any(
        type(value) is not int or value < interval or value % interval
        for value in durations
    ):
        raise ValueError("search delivery durations must lie on the sample grid")
    if durations != (
        2_000_000_000,
        3_000_000_000,
        1_000_000_000,
        240_000_000_000,
    ):
        raise ValueError("search_delivery_v1 timing policy changed")
    numeric_names = (
        "search_staging_tolerance_m",
        "search_altitude_min_m",
        "search_altitude_max_m",
        "search_east_progress_m",
        "pickup_center_tolerance_m",
        "pickup_lift_m",
        "release_position_tolerance_m",
        "release_horizontal_speed_mps",
        "release_max_tilt_rad",
        "release_agl_m",
    )
    values = tuple(_positive_number(document, name) for name in numeric_names)
    if values != (
        0.30,
        4.25,
        4.90,
        0.75,
        0.075,
        0.25,
        0.15,
        0.10,
        0.10,
        10.0,
    ):
        raise ValueError("search_delivery_v1 physical thresholds changed")
    points_document = document.get("points")
    if not isinstance(points_document, dict) or tuple(points_document) != _RULE_IDS:
        raise ValueError("search delivery point identities or order changed")
    points = tuple(_positive_number(points_document, rule_id) for rule_id in _RULE_IDS)
    if points != (25.0, 25.0, 25.0, 25.0):
        raise ValueError("search_delivery_v1 allocation must total 100 points")
    return SearchDeliveryRules(
        ruleset_id="search_delivery_v1",
        scoring_checksum=hashlib.sha256(payload).hexdigest(),
        sample_interval_ns=interval,
        search_staging_tolerance_m=values[0],
        search_altitude_min_m=values[1],
        search_altitude_max_m=values[2],
        search_east_progress_m=values[3],
        pickup_center_tolerance_m=values[4],
        pickup_lift_m=values[5],
        release_position_tolerance_m=values[6],
        release_horizontal_speed_mps=values[7],
        release_max_tilt_rad=values[8],
        release_stability_ns=durations[0],
        post_release_hold_ns=durations[1],
        payload_settle_ns=durations[2],
        release_agl_m=values[9],
        deadline_ns=durations[3],
        points=points,
    )


def _speed(values: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def _horizontal_speed(values: tuple[float, float, float]) -> float:
    return math.hypot(values[0], values[1])


def _tilt_radians(orientation: tuple[float, float, float, float]) -> float:
    norm = math.sqrt(sum(value * value for value in orientation))
    x, y, _z, w = (value / norm for value in orientation)
    world_up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, world_up_z)))


class SearchDeliveryScorer:
    """Evaluate one-payload search-and-delivery evidence without commanding it."""

    payload_ids = (3,)
    terminal_payload_event_id = 1

    def __init__(self, run_id: str, rules: SearchDeliveryRules) -> None:
        self.run_id = _canonical_run_id(run_id)
        if not isinstance(rules, SearchDeliveryRules):
            raise TypeError("rules must be SearchDeliveryRules")
        self.rules = rules
        self.start_sim_time_ns: int | None = None
        self._ground_truth: list[GroundTruthSample] = []
        self._payload_states: dict[int, list[PayloadStateSample]] = {3: []}
        self._payload_events: list[PayloadEventSample] = []
        self._mission_events: list[MissionEventSample] = []
        self._diagnostic: str | None = None
        self._result: ScoreResult | None = None

    @property
    def last_sim_timestamp_ns(self) -> int:
        timestamps = [sample.sim_timestamp_ns for sample in self._ground_truth]
        timestamps.extend(sample.sim_timestamp_ns for sample in self._payload_states[3])
        timestamps.extend(event.sim_timestamp_ns for event in self._payload_events)
        timestamps.extend(event.sim_timestamp_ns for event in self._mission_events)
        return max(timestamps, default=0)

    def fail(self, reason: str) -> None:
        if self._result is not None:
            raise RuntimeError("score has already been finalized")
        if not isinstance(reason, str) or not reason:
            raise ValueError("score failure reason must be nonempty")
        if self._diagnostic is None:
            self._diagnostic = reason

    def _open(self) -> None:
        if self._result is not None:
            raise RuntimeError("score has already been finalized")

    def _latest_gap(self, timestamps: list[int], diagnostic: str) -> None:
        if self.start_sim_time_ns is None or not timestamps:
            return
        current = timestamps[-1]
        if len(timestamps) == 1:
            invalid = current - self.start_sim_time_ns > self.rules.sample_interval_ns
        else:
            invalid = timestamps[-1] - timestamps[-2] != self.rules.sample_interval_ns
        if invalid:
            self.fail(diagnostic)

    def _existing_stream_gap(self, timestamps: list[int], diagnostic: str) -> None:
        if self.start_sim_time_ns is None:
            return
        after_start = [
            timestamp
            for timestamp in timestamps
            if timestamp >= self.start_sim_time_ns
        ]
        if not after_start:
            return
        if after_start[0] - self.start_sim_time_ns > self.rules.sample_interval_ns:
            self.fail(diagnostic)
            return
        if any(
            current - previous != self.rules.sample_interval_ns
            for previous, current in zip(after_start, after_start[1:])
        ):
            self.fail(diagnostic)

    def accept_ground_truth(self, sample: GroundTruthSample) -> None:
        self._open()
        if not isinstance(sample, GroundTruthSample):
            raise TypeError("sample must be GroundTruthSample")
        if sample.run_id != self.run_id:
            return
        if (
            self.start_sim_time_ns is not None
            and sample.sim_timestamp_ns < self.start_sim_time_ns
        ):
            return
        if (
            self._ground_truth
            and sample.sim_timestamp_ns <= self._ground_truth[-1].sim_timestamp_ns
        ):
            self.fail("ground_truth_timestamp_not_increasing")
            return
        self._ground_truth.append(sample)
        self._latest_gap(
            [entry.sim_timestamp_ns for entry in self._ground_truth[-2:]],
            "ground_truth_timestamp_gap",
        )

    def accept_payload_state(self, sample: PayloadStateSample) -> None:
        self._open()
        if not isinstance(sample, PayloadStateSample):
            raise TypeError("sample must be PayloadStateSample")
        if sample.run_id != self.run_id:
            return
        if sample.aruco_id != 3:
            self.fail("unknown_payload_state")
            return
        if (
            self.start_sim_time_ns is not None
            and sample.sim_timestamp_ns < self.start_sim_time_ns
        ):
            return
        states = self._payload_states[3]
        if states and sample.sim_timestamp_ns <= states[-1].sim_timestamp_ns:
            self.fail("payload_3_timestamp_not_increasing")
            return
        states.append(sample)
        self._latest_gap(
            [entry.sim_timestamp_ns for entry in states[-2:]],
            "payload_3_timestamp_gap",
        )

    def accept_payload_event(self, sample: PayloadEventSample) -> None:
        self._open()
        if not isinstance(sample, PayloadEventSample):
            raise TypeError("sample must be PayloadEventSample")
        if sample.run_id != self.run_id:
            return
        if sample.aruco_id != 3:
            self.fail("unknown_payload_event")
            return
        if self._payload_events:
            previous = self._payload_events[-1]
            if sample.event_id != previous.event_id + 1:
                self.fail("payload_event_id_discontinuity")
                return
            if sample.sim_timestamp_ns < previous.sim_timestamp_ns:
                self.fail("payload_event_timestamp_regression")
                return
        elif sample.event_id != 0:
            self.fail("payload_event_id_discontinuity")
            return
        self._payload_events.append(sample)

    def accept_mission_event(self, sample: MissionEventSample) -> None:
        self._open()
        if not isinstance(sample, MissionEventSample):
            raise TypeError("sample must be MissionEventSample")
        if sample.run_id != self.run_id:
            return
        if self._mission_events:
            previous = self._mission_events[-1]
            if sample.event_id != previous.event_id + 1:
                self.fail("mission_event_id_discontinuity")
                return
            if sample.sim_timestamp_ns < previous.sim_timestamp_ns:
                self.fail("mission_event_timestamp_regression")
                return
        elif sample.event_id != 0:
            self.fail("mission_event_id_discontinuity")
            return
        self._mission_events.append(sample)
        if (
            self.start_sim_time_ns is None
            and sample.phase == "SEARCH"
            and sample.state == "STARTED"
        ):
            self.start_sim_time_ns = sample.sim_timestamp_ns
            self._existing_stream_gap(
                [entry.sim_timestamp_ns for entry in self._ground_truth],
                "ground_truth_timestamp_gap",
            )
            self._existing_stream_gap(
                [entry.sim_timestamp_ns for entry in self._payload_states[3]],
                "payload_3_timestamp_gap",
            )

    def _mission(self) -> list[MissionEventSample]:
        if self.start_sim_time_ns is None:
            return []
        return [
            event
            for event in self._mission_events
            if event.sim_timestamp_ns >= self.start_sim_time_ns
        ]

    def _payload(self) -> list[PayloadEventSample]:
        if self.start_sim_time_ns is None:
            return []
        return [
            event
            for event in self._payload_events
            if event.sim_timestamp_ns >= self.start_sim_time_ns
        ]

    def _phase_events(
        self, phase: str
    ) -> tuple[MissionEventSample, MissionEventSample]:
        index = _PHASE_INDEX[phase]
        end_index = index + (2 if phase == "HOME" else 1)
        events = self._mission()
        return events[index], events[end_index]

    def _payload_event(self, action: str) -> PayloadEventSample | None:
        return next(
            (
                event
                for event in self._payload()
                if event.aruco_id == 3
                and event.action == action
                and event.state == ("attached" if action == "attach" else "detached")
                and event.code == "OK"
            ),
            None,
        )

    def _vehicle_at(self, timestamp_ns: int) -> GroundTruthSample | None:
        eligible = [
            sample
            for sample in self._ground_truth
            if sample.sim_timestamp_ns <= timestamp_ns
            and timestamp_ns - sample.sim_timestamp_ns <= self.rules.sample_interval_ns
        ]
        return eligible[-1] if eligible else None

    def _state_at(self, timestamp_ns: int) -> PayloadStateSample | None:
        eligible = [
            sample
            for sample in self._payload_states[3]
            if sample.sim_timestamp_ns <= timestamp_ns
            and timestamp_ns - sample.sim_timestamp_ns <= self.rules.sample_interval_ns
        ]
        return eligible[-1] if eligible else None

    @staticmethod
    def _inside_waypoint(position: tuple[float, float, float], waypoint: str) -> bool:
        x, y, width, height = _WAYPOINTS[waypoint]
        return abs(position[0] - x) <= width / 2 and abs(position[1] - y) <= height / 2

    def _search(self) -> bool:
        started, complete = self._phase_events("SEARCH")
        candidates = [
            sample
            for sample in self._ground_truth
            if started.sim_timestamp_ns
            <= sample.sim_timestamp_ns
            <= complete.sim_timestamp_ns
            and self.rules.search_altitude_min_m
            <= sample.position_xyz[2]
            <= self.rules.search_altitude_max_m
            and math.hypot(
                sample.position_xyz[0] - _STAGING[0],
                sample.position_xyz[1] - _STAGING[1],
            )
            <= self.rules.search_staging_tolerance_m
        ]
        marker_x, marker_y, _width, _height = _WAYPOINTS["WA"]
        return any(
            later.sim_timestamp_ns > staging.sim_timestamp_ns
            and later.position_xyz[0] - staging.position_xyz[0]
            >= self.rules.search_east_progress_m
            and self.rules.search_altitude_min_m
            <= later.position_xyz[2]
            <= self.rules.search_altitude_max_m
            and math.hypot(
                later.position_xyz[0] - marker_x,
                later.position_xyz[1] - marker_y,
            )
            < math.hypot(
                staging.position_xyz[0] - marker_x,
                staging.position_xyz[1] - marker_y,
            )
            for staging in candidates
            for later in self._ground_truth
            if later.sim_timestamp_ns <= complete.sim_timestamp_ns
        )

    def _physical_attach(
        self, event: PayloadEventSample
    ) -> tuple[bool, PayloadStateSample | None]:
        states = self._payload_states[3]
        current_index = next(
            (
                index
                for index, sample in enumerate(states)
                if event.sim_timestamp_ns
                <= sample.sim_timestamp_ns
                <= event.sim_timestamp_ns + self.rules.sample_interval_ns
                and sample.attached
            ),
            None,
        )
        if current_index is None or current_index == 0:
            return False, None
        previous, current = states[current_index - 1], states[current_index]
        vehicle = self._vehicle_at(current.sim_timestamp_ns)
        valid = (
            current.sim_timestamp_ns - previous.sim_timestamp_ns
            == self.rules.sample_interval_ns
            and previous.grounded
            and not previous.attached
            and current.attached
            and self._inside_waypoint(current.position_xyz, "WA")
            and vehicle is not None
            and vehicle.in_contact
            and math.hypot(
                current.position_xyz[0] - vehicle.position_xyz[0],
                current.position_xyz[1] - vehicle.position_xyz[1],
            )
            <= self.rules.pickup_center_tolerance_m
            and math.dist(current.position_xyz, previous.position_xyz)
            <= self.rules.pickup_center_tolerance_m
        )
        return valid, previous if valid else None

    def _initial_payload_valid(self) -> bool:
        if self.start_sim_time_ns is None:
            return False
        initial = next(
            (
                sample
                for sample in self._payload_states[3]
                if sample.sim_timestamp_ns >= self.start_sim_time_ns
            ),
            None,
        )
        return bool(
            initial is not None
            and initial.grounded
            and not initial.attached
            and self._inside_waypoint(initial.position_xyz, "WA")
        )

    def _pickup(self) -> bool:
        search_start, search_complete = self._phase_events("SEARCH")
        delivery_start, _delivery_complete = self._phase_events("DELIVERY")
        attach = self._payload_event("attach")
        if attach is None or not (
            search_start.sim_timestamp_ns
            <= attach.sim_timestamp_ns
            < search_complete.sim_timestamp_ns
        ):
            return False
        attached, grounded = self._physical_attach(attach)
        complete_vehicle = self._vehicle_at(search_complete.sim_timestamp_ns)
        complete_payload = self._state_at(search_complete.sim_timestamp_ns)
        if (
            not attached
            or grounded is None
            or complete_vehicle is None
            or not complete_vehicle.in_contact
            or complete_payload is None
            or not complete_payload.attached
        ):
            return False
        return any(
            sample.sim_timestamp_ns >= delivery_start.sim_timestamp_ns
            and sample.attached
            and not sample.grounded
            and sample.position_xyz[2]
            >= grounded.position_xyz[2] + self.rules.pickup_lift_m
            for sample in self._payload_states[3]
        )

    def _release_stable(self, sample: GroundTruthSample) -> bool:
        x, y, _width, _height = _WAYPOINTS["F2"]
        return (
            math.hypot(sample.position_xyz[0] - x, sample.position_xyz[1] - y)
            <= self.rules.release_position_tolerance_m
            and _horizontal_speed(sample.linear_velocity_xyz)
            <= self.rules.release_horizontal_speed_mps
            and sample.position_xyz[2] >= self.rules.release_agl_m
            and _tilt_radians(sample.orientation_xyzw)
            <= self.rules.release_max_tilt_rad
        )

    def _continuous_vehicle_window(self, start_ns: int, end_ns: int) -> bool:
        window = [
            sample
            for sample in self._ground_truth
            if start_ns <= sample.sim_timestamp_ns <= end_ns
        ]
        required = (end_ns - start_ns) // self.rules.sample_interval_ns + 1
        return (
            len(window) == required
            and window[0].sim_timestamp_ns == start_ns
            and window[-1].sim_timestamp_ns == end_ns
            and all(self._release_stable(sample) for sample in window)
        )

    def _physical_release(self, event: PayloadEventSample) -> bool:
        before = [
            sample
            for sample in self._payload_states[3]
            if sample.sim_timestamp_ns < event.sim_timestamp_ns
        ]
        detached = next(
            (
                sample
                for sample in self._payload_states[3]
                if event.sim_timestamp_ns
                < sample.sim_timestamp_ns
                <= event.sim_timestamp_ns + self.rules.sample_interval_ns
                and not sample.attached
            ),
            None,
        )
        return bool(
            before
            and event.sim_timestamp_ns - before[-1].sim_timestamp_ns
            <= self.rules.sample_interval_ns
            and before[-1].attached
            and detached is not None
        )

    @staticmethod
    def _yaw(orientation: tuple[float, float, float, float]) -> float:
        norm = math.sqrt(sum(value * value for value in orientation))
        x, y, z, w = (value / norm for value in orientation)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _inside_f2(self, sample: PayloadStateSample) -> bool:
        yaw = self._yaw(sample.orientation_xyzw)
        half_x, half_y = (
            _PAYLOAD_XY_SIZE_M[0] / 2,
            _PAYLOAD_XY_SIZE_M[1] / 2,
        )
        extent_x = abs(math.cos(yaw)) * half_x + abs(math.sin(yaw)) * half_y
        extent_y = abs(math.sin(yaw)) * half_x + abs(math.cos(yaw)) * half_y
        x, y, width, height = _WAYPOINTS["F2"]
        return (
            abs(sample.position_xyz[0] - x) + extent_x <= width / 2
            and abs(sample.position_xyz[1] - y) + extent_y <= height / 2
        )

    def _settled_before(self, release_ns: int, boundary_ns: int) -> bool:
        started_ns: int | None = None
        previous_ns: int | None = None
        for sample in self._payload_states[3]:
            if (
                sample.sim_timestamp_ns <= release_ns
                or sample.sim_timestamp_ns > boundary_ns
            ):
                continue
            eligible = (
                sample.grounded
                and not sample.attached
                and _speed(sample.linear_velocity_xyz)
                <= self.rules.release_horizontal_speed_mps
                and self._inside_f2(sample)
            )
            if (
                not eligible
                or previous_ns is None
                or sample.sim_timestamp_ns - previous_ns
                != self.rules.sample_interval_ns
            ):
                started_ns = sample.sim_timestamp_ns if eligible else None
            if eligible and started_ns is not None:
                if sample.sim_timestamp_ns - started_ns >= self.rules.payload_settle_ns:
                    return True
                previous_ns = sample.sim_timestamp_ns
            else:
                previous_ns = None
        return False

    def _delivery(self) -> bool:
        started, complete = self._phase_events("DELIVERY")
        release = self._payload_event("release")
        if release is None or not (
            started.sim_timestamp_ns
            <= release.sim_timestamp_ns
            < complete.sim_timestamp_ns
        ):
            return False
        hold_end = release.sim_timestamp_ns + self.rules.post_release_hold_ns
        return (
            self._pickup()
            and complete.sim_timestamp_ns >= hold_end
            and self._continuous_vehicle_window(
                release.sim_timestamp_ns - self.rules.release_stability_ns,
                release.sim_timestamp_ns,
            )
            and self._continuous_vehicle_window(release.sim_timestamp_ns, hold_end)
            and self._physical_release(release)
            and self._settled_before(
                release.sim_timestamp_ns, complete.sim_timestamp_ns
            )
        )

    def _landed_home(self, sample: GroundTruthSample) -> bool:
        return (
            sample.in_contact
            and _speed(sample.linear_velocity_xyz)
            <= self.rules.release_horizontal_speed_mps
            and self._inside_waypoint(sample.position_xyz, "H")
        )

    def _home(self) -> bool:
        delivery_start, delivery_complete = self._phase_events("DELIVERY")
        home_start, home_complete = self._phase_events("HOME")
        events = self._mission()
        disarmed = events[5]
        if not (
            delivery_start.sim_timestamp_ns
            < delivery_complete.sim_timestamp_ns
            < home_start.sim_timestamp_ns
        ):
            return False
        landing = next(
            (
                sample
                for sample in self._ground_truth
                if sample.sim_timestamp_ns >= home_start.sim_timestamp_ns
                and self._landed_home(sample)
            ),
            None,
        )
        complete_truth = self._vehicle_at(home_complete.sim_timestamp_ns)
        return bool(
            landing is not None
            and landing.sim_timestamp_ns
            < disarmed.sim_timestamp_ns
            < home_complete.sim_timestamp_ns
            and all(
                self._landed_home(sample)
                for sample in self._ground_truth
                if landing.sim_timestamp_ns
                <= sample.sim_timestamp_ns
                <= home_complete.sim_timestamp_ns
            )
            and complete_truth is not None
            and self._landed_home(complete_truth)
        )

    def _evaluate(self) -> tuple[tuple[bool, ...], bool, str | None]:
        if self.start_sim_time_ns is None:
            return (False,) * 4, False, "mission_start_missing"
        mission = self._mission()
        mission_valid = (
            [(event.phase, event.state) for event in mission]
            == list(_MISSION_SEQUENCE)
            and all(
                current.sim_timestamp_ns > previous.sim_timestamp_ns
                for previous, current in zip(mission, mission[1:])
            )
        )
        payload = self._payload()
        payload_valid = (
            [(event.aruco_id, event.action, event.state) for event in payload]
            == list(_PAYLOAD_SEQUENCE)
            and all(event.code == "OK" for event in payload)
            and all(
                current.sim_timestamp_ns > previous.sim_timestamp_ns
                for previous, current in zip(payload, payload[1:])
            )
        )
        if self._diagnostic is not None:
            return (False,) * 4, False, self._diagnostic
        if not self._ground_truth:
            return (False,) * 4, False, "ground_truth_missing"
        if not self._payload_states[3]:
            return (False,) * 4, False, "payload_3_state_missing"
        if not self._initial_payload_valid():
            return (False,) * 4, False, "payload_initial_state_invalid"
        if not mission_valid:
            return (False,) * 4, False, "mission_sequence_invalid"
        if not payload_valid:
            return (False,) * 4, False, "payload_event_sequence_invalid"
        search = self._search()
        pickup = self._pickup()
        delivery = self._delivery()
        home = self._home()
        outcomes = (search, pickup, delivery, home)
        _home_start, home_complete = self._phase_events("HOME")
        deadline_valid = (
            home_complete.sim_timestamp_ns - self.start_sim_time_ns
            <= self.rules.deadline_ns
        )
        complete = home and deadline_valid
        diagnostic = None
        if not home:
            diagnostic = "home_completion_invalid"
        elif not deadline_valid:
            diagnostic = "home_deadline_exceeded"
        return outcomes, complete, diagnostic

    def finalize(self) -> ScoreResult:
        if self._result is not None:
            return self._result
        outcomes, complete, diagnostic = self._evaluate()
        rule_results = tuple(
            RuleResult(
                rule_id=rule_id,
                passed=passed,
                awarded_points=points if passed else 0.0,
                available_points=points,
                evidence_ref=f"rosbag#search_delivery:{rule_id}",
            )
            for rule_id, points, passed in zip(
                _RULE_IDS, self.rules.points, outcomes, strict=True
            )
        )
        achieved = sum(rule.awarded_points for rule in rule_results)
        timestamp = self.last_sim_timestamp_ns
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
        self._result = ScoreResult(
            run_id=self.run_id,
            ruleset_id=self.rules.ruleset_id,
            complete=complete,
            achieved_score=achieved,
            maximum_available_score=self.rules.maximum_available_score,
            scoring_checksum=self.rules.scoring_checksum,
            evidence_paths=tuple(event.evidence_ref for event in events)
            + (
                "rosbag#/simulation/ground_truth",
                "rosbag#/simulation/payload_state",
                "rosbag#/simulation/payload_events",
                "rosbag#/simulation/mission_events",
            ),
            rule_results=rule_results,
            events=events,
            diagnostic=diagnostic,
        )
        return self._result


__all__ = [
    "SearchDeliveryRules",
    "SearchDeliveryScorer",
    "load_search_delivery_rules",
]
