"""Independent read-only physical scoring for ``competition_v1``."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any
from uuid import UUID

from .descent import GroundTruthSample
from .models import RuleResult, ScoreEvent, ScoreResult


_RULE_IDS = (
    "fm1_landing",
    "fm1_autonomy",
    "payload_2",
    "fm2_autonomy",
    "payload_3",
    "fm3_autonomy",
    "payload_4",
)
_EVENT_TYPES = tuple(f"competition.{rule_id}" for rule_id in _RULE_IDS)
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
_PICKUP_WAYPOINT = {3: "WA", 4: "WM"}
_PAYLOAD_PREFIX_LENGTH = {2: 1, 3: 3, 4: 5}
_WAYPOINTS = {
    "H": (0.0, 0.0, 4.572, 4.572),
    "L": (-91.44, 0.0, 4.572, 4.572),
    "F2": (-152.40, 0.0, 0.9144, 0.9144),
    "WA": (-45.72, -9.144, 6.096, 6.096),
    "WM": (-45.72, 9.144, 6.096, 6.096),
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


def _timestamp(value: object) -> int:
    if type(value) is not int or value < 0:
        raise ValueError("sim_timestamp_ns must be a nonnegative integer")
    return value


def _event_id(value: object) -> int:
    if type(value) is not int or value < 0:
        raise ValueError("event_id must be a nonnegative integer")
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
class PayloadStateSample:
    run_id: str
    sim_timestamp_ns: int
    aruco_id: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    grounded: bool
    attached: bool

    def __post_init__(self) -> None:
        object.__setattr__(self, "run_id", _canonical_run_id(self.run_id))
        _timestamp(self.sim_timestamp_ns)
        if type(self.aruco_id) is not int or not 0 <= self.aruco_id <= 65_535:
            raise ValueError("aruco_id must be an unsigned 16-bit integer")
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
        if type(self.grounded) is not bool or type(self.attached) is not bool:
            raise ValueError("payload physical states must be boolean")


@dataclass(frozen=True)
class PayloadEventSample:
    run_id: str
    sim_timestamp_ns: int
    event_id: int
    aruco_id: int
    command_id: str
    action: str
    state: str
    code: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "run_id", _canonical_run_id(self.run_id))
        _timestamp(self.sim_timestamp_ns)
        _event_id(self.event_id)
        if type(self.aruco_id) is not int or not 0 <= self.aruco_id <= 65_535:
            raise ValueError("aruco_id must be an unsigned 16-bit integer")
        if any(
            not isinstance(value, str) or not value
            for value in (self.command_id, self.action, self.state, self.code)
        ):
            raise ValueError("payload event strings must be nonempty")


@dataclass(frozen=True)
class MissionEventSample:
    run_id: str
    sim_timestamp_ns: int
    event_id: int
    phase: str
    state: str
    detail: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "run_id", _canonical_run_id(self.run_id))
        _timestamp(self.sim_timestamp_ns)
        _event_id(self.event_id)
        if any(
            not isinstance(value, str) or not value
            for value in (self.phase, self.state)
        ) or not isinstance(self.detail, str):
            raise ValueError("mission event fields are invalid")


@dataclass(frozen=True)
class CompetitionRules:
    ruleset_id: str
    scoring_checksum: str
    sample_interval_ns: int
    pickup_center_tolerance_m: float
    release_position_tolerance_m: float
    release_horizontal_speed_mps: float
    release_stability_ns: int
    payload_settle_ns: int
    release_agl_m: float
    deadline_ns: int
    points: tuple[float, float, float, float, float, float, float]

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


def load_competition_rules(path: Path | str) -> CompetitionRules:
    """Load and checksum the exact frozen physical competition policy."""
    payload = Path(path).read_bytes()
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("competition rules must be UTF-8 JSON") from error
    if not isinstance(document, dict):
        raise ValueError("competition rules must be an object")
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
    if set(document) != expected_keys:
        raise ValueError("competition rules fields changed")
    if document.get("schema_version") != 1 or document.get("ruleset_id") != "competition_v1":
        raise ValueError("unsupported competition ruleset")
    interval = document.get("sample_interval_ns")
    stability = document.get("release_stability_ns")
    settle = document.get("payload_settle_ns")
    deadline = document.get("deadline_ns")
    if interval != 50_000_000 or isinstance(interval, bool):
        raise ValueError("competition_v1 sample interval must be 50000000 ns")
    for name, value in (
        ("release_stability_ns", stability),
        ("payload_settle_ns", settle),
        ("deadline_ns", deadline),
    ):
        if type(value) is not int or value < interval or value % interval:
            raise ValueError(f"{name} must be a positive sample-grid duration")
    if (stability, settle, deadline) != (
        2_000_000_000,
        1_000_000_000,
        600_000_000_000,
    ):
        raise ValueError("competition timing policy changed")
    points_document = document.get("points")
    if not isinstance(points_document, dict) or tuple(points_document) != _RULE_IDS:
        raise ValueError("competition point identities or order changed")
    points = tuple(_positive_number(points_document, rule_id) for rule_id in _RULE_IDS)
    if points != (20.0, 30.0, 10.0, 20.0, 15.0, 50.0, 5.0):
        raise ValueError("competition_v1 allocation must total the official 150 points")
    pickup_tolerance = _positive_number(document, "pickup_center_tolerance_m")
    release_tolerance = _positive_number(document, "release_position_tolerance_m")
    release_speed = _positive_number(document, "release_horizontal_speed_mps")
    release_agl = _positive_number(document, "release_agl_m")
    if (pickup_tolerance, release_tolerance, release_speed, release_agl) != (
        0.075,
        0.15,
        0.10,
        10.0,
    ):
        raise ValueError("competition_v1 physical thresholds changed")
    return CompetitionRules(
        ruleset_id="competition_v1",
        scoring_checksum=hashlib.sha256(payload).hexdigest(),
        sample_interval_ns=interval,
        pickup_center_tolerance_m=pickup_tolerance,
        release_position_tolerance_m=release_tolerance,
        release_horizontal_speed_mps=release_speed,
        release_stability_ns=stability,
        payload_settle_ns=settle,
        release_agl_m=release_agl,
        deadline_ns=deadline,
        points=points,
    )


def _speed(values: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def _horizontal_speed(values: tuple[float, float, float]) -> float:
    return math.hypot(values[0], values[1])


class CompetitionScorer:
    """Evaluate immutable physical evidence without commanding any subsystem."""

    def __init__(self, run_id: str, rules: CompetitionRules) -> None:
        self.run_id = _canonical_run_id(run_id)
        if not isinstance(rules, CompetitionRules):
            raise TypeError("rules must be CompetitionRules")
        self.rules = rules
        self.start_sim_time_ns: int | None = None
        self._ground_truth: list[GroundTruthSample] = []
        self._payload_states: dict[int, list[PayloadStateSample]] = {
            2: [],
            3: [],
            4: [],
        }
        self._payload_events: list[PayloadEventSample] = []
        self._mission_events: list[MissionEventSample] = []
        self._diagnostic: str | None = None
        self._result: ScoreResult | None = None

    @property
    def last_sim_timestamp_ns(self) -> int:
        timestamps = [sample.sim_timestamp_ns for sample in self._ground_truth]
        timestamps.extend(
            sample.sim_timestamp_ns
            for states in self._payload_states.values()
            for sample in states
        )
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

    def _stream_gap(
        self,
        timestamps_ns: list[int],
        *,
        diagnostic: str,
    ) -> None:
        if self.start_sim_time_ns is None:
            return
        after_start = [
            timestamp_ns
            for timestamp_ns in timestamps_ns
            if timestamp_ns >= self.start_sim_time_ns
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

    def _ground_truth_gap(self) -> None:
        self._stream_gap(
            [sample.sim_timestamp_ns for sample in self._ground_truth],
            diagnostic="ground_truth_timestamp_gap",
        )

    def _payload_gap(self, marker: int) -> None:
        self._stream_gap(
            [sample.sim_timestamp_ns for sample in self._payload_states[marker]],
            diagnostic=f"payload_{marker}_timestamp_gap",
        )

    def _latest_timestamp_gap(
        self,
        timestamps_ns: list[int],
        *,
        diagnostic: str,
    ) -> None:
        if self.start_sim_time_ns is None or not timestamps_ns:
            return
        current = timestamps_ns[-1]
        previous = timestamps_ns[-2] if len(timestamps_ns) > 1 else None
        if previous is None or previous < self.start_sim_time_ns:
            gap = current - self.start_sim_time_ns
            if gap > self.rules.sample_interval_ns:
                self.fail(diagnostic)
        elif current - previous != self.rules.sample_interval_ns:
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
        if self._ground_truth and sample.sim_timestamp_ns <= self._ground_truth[-1].sim_timestamp_ns:
            self.fail("ground_truth_timestamp_not_increasing")
            return
        self._ground_truth.append(sample)
        self._latest_timestamp_gap(
            [entry.sim_timestamp_ns for entry in self._ground_truth[-2:]],
            diagnostic="ground_truth_timestamp_gap",
        )

    def accept_payload_state(self, sample: PayloadStateSample) -> None:
        self._open()
        if not isinstance(sample, PayloadStateSample):
            raise TypeError("sample must be PayloadStateSample")
        if sample.run_id != self.run_id:
            return
        if (
            self.start_sim_time_ns is not None
            and sample.sim_timestamp_ns < self.start_sim_time_ns
        ):
            return
        states = self._payload_states.get(sample.aruco_id)
        if states is None:
            self.fail("unknown_payload_state")
            return
        if states and sample.sim_timestamp_ns <= states[-1].sim_timestamp_ns:
            self.fail(f"payload_{sample.aruco_id}_timestamp_not_increasing")
            return
        states.append(sample)
        self._latest_timestamp_gap(
            [entry.sim_timestamp_ns for entry in states[-2:]],
            diagnostic=f"payload_{sample.aruco_id}_timestamp_gap",
        )

    def accept_payload_event(self, sample: PayloadEventSample) -> None:
        self._open()
        if not isinstance(sample, PayloadEventSample):
            raise TypeError("sample must be PayloadEventSample")
        if sample.run_id != self.run_id:
            return
        if sample.aruco_id not in self._payload_states:
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
            and sample.phase == "FM1"
            and sample.state == "STARTED"
        ):
            self.start_sim_time_ns = sample.sim_timestamp_ns
            self._ground_truth_gap()
            for marker in self._payload_states:
                self._payload_gap(marker)

    def _mission_after_start(self) -> list[MissionEventSample]:
        if self.start_sim_time_ns is None:
            return []
        return [
            event
            for event in self._mission_events
            if event.sim_timestamp_ns >= self.start_sim_time_ns
        ]

    def _payload_after_start(self) -> list[PayloadEventSample]:
        if self.start_sim_time_ns is None:
            return []
        return [
            event
            for event in self._payload_events
            if event.sim_timestamp_ns >= self.start_sim_time_ns
        ]

    def _mission_prefix(self, length: int) -> bool:
        actual = [(event.phase, event.state) for event in self._mission_after_start()]
        return actual[:length] == list(_MISSION_SEQUENCE[:length])

    def _payload_prefix(self, length: int) -> bool:
        events = self._payload_after_start()
        actual = [
            (event.aruco_id, event.action, event.state)
            for event in events
        ]
        return actual[:length] == list(_PAYLOAD_SEQUENCE[:length]) and all(
            event.code == "OK" for event in events[:length]
        )

    def _phase_events(
        self, phase: str
    ) -> tuple[MissionEventSample, MissionEventSample] | None:
        index = _PHASE_INDEX[phase]
        events = self._mission_after_start()
        end_index = index + (2 if phase == "HOME" else 1)
        if len(events) <= end_index or not self._mission_prefix(end_index + 1):
            return None
        return events[index], events[end_index]

    def _home_events(
        self,
    ) -> tuple[MissionEventSample, MissionEventSample, MissionEventSample] | None:
        events = self._mission_after_start()
        if len(events) < len(_MISSION_SEQUENCE) or not self._mission_prefix(
            len(_MISSION_SEQUENCE)
        ):
            return None
        return events[8], events[9], events[10]

    def _vehicle_at(self, timestamp_ns: int) -> GroundTruthSample | None:
        if self.start_sim_time_ns is None:
            return None
        eligible = [
            sample
            for sample in self._ground_truth
            if self.start_sim_time_ns <= sample.sim_timestamp_ns <= timestamp_ns
            and timestamp_ns - sample.sim_timestamp_ns <= self.rules.sample_interval_ns
        ]
        return eligible[-1] if eligible else None

    @staticmethod
    def _inside_waypoint(
        position_xyz: tuple[float, float, float], waypoint: str
    ) -> bool:
        center_x, center_y, width, height = _WAYPOINTS[waypoint]
        return (
            abs(position_xyz[0] - center_x) <= width / 2.0
            and abs(position_xyz[1] - center_y) <= height / 2.0
        )

    def _is_landed(self, sample: GroundTruthSample, waypoint: str) -> bool:
        return (
            sample.in_contact
            and _speed(sample.linear_velocity_xyz)
            <= self.rules.release_horizontal_speed_mps
            and self._inside_waypoint(sample.position_xyz, waypoint)
        )

    def _landing(self, phase: str, waypoint: str) -> bool:
        phase_events = self._phase_events(phase)
        if phase_events is None:
            return False
        sample = self._vehicle_at(phase_events[1].sim_timestamp_ns)
        if sample is None:
            return False
        return self._is_landed(sample, waypoint)

    def _home_landing_timestamp(self) -> int | None:
        home_events = self._home_events()
        if home_events is None or self.start_sim_time_ns is None:
            return None
        started, disarmed, complete = home_events
        if not (
            started.sim_timestamp_ns
            < disarmed.sim_timestamp_ns
            < complete.sim_timestamp_ns
        ):
            return None
        landing_timestamp = self._physical_home_landing_timestamp(started)
        if landing_timestamp is None or landing_timestamp >= disarmed.sim_timestamp_ns:
            return None
        return landing_timestamp

    def _physical_home_landing_timestamp(
        self, started: MissionEventSample
    ) -> int | None:
        landing = next(
            (
                sample
                for sample in self._ground_truth
                if sample.sim_timestamp_ns >= started.sim_timestamp_ns
                and self._is_landed(sample, "H")
            ),
            None,
        )
        return None if landing is None else landing.sim_timestamp_ns

    def _home_checkpoint_boundary_timestamp(self) -> int | None:
        events = self._mission_after_start()
        if len(events) <= _PHASE_INDEX["HOME"] or not self._mission_prefix(9):
            return None
        started = events[_PHASE_INDEX["HOME"]]
        candidates = [
            event.sim_timestamp_ns
            for event in events[9:]
            if event.phase == "HOME" and event.state in {"DISARMED", "COMPLETE"}
        ]
        physical_landing = self._physical_home_landing_timestamp(started)
        if physical_landing is not None:
            candidates.append(physical_landing)
        return min(candidates) if candidates else None

    def _payload_event(self, marker: int, action: str) -> PayloadEventSample | None:
        return next(
            (
                event
                for event in self._payload_after_start()
                if event.aruco_id == marker
                and event.action == action
                and event.state == ("attached" if action == "attach" else "detached")
                and event.code == "OK"
            ),
            None,
        )

    def _release_gate(self, event: PayloadEventSample) -> bool:
        start_ns = event.sim_timestamp_ns - self.rules.release_stability_ns
        if self.start_sim_time_ns is None or start_ns < self.start_sim_time_ns:
            return False
        window = [
            sample
            for sample in self._ground_truth
            if start_ns <= sample.sim_timestamp_ns <= event.sim_timestamp_ns
        ]
        required = self.rules.release_stability_ns // self.rules.sample_interval_ns + 1
        if (
            len(window) != required
            or window[0].sim_timestamp_ns != start_ns
            or window[-1].sim_timestamp_ns != event.sim_timestamp_ns
            or any(
                current.sim_timestamp_ns - previous.sim_timestamp_ns
                != self.rules.sample_interval_ns
                for previous, current in zip(window, window[1:])
            )
        ):
            return False
        target_x, target_y, _width, _height = _WAYPOINTS["F2"]
        return all(
            math.hypot(
                sample.position_xyz[0] - target_x,
                sample.position_xyz[1] - target_y,
            )
            <= self.rules.release_position_tolerance_m
            and _horizontal_speed(sample.linear_velocity_xyz)
            <= self.rules.release_horizontal_speed_mps
            and sample.position_xyz[2] >= self.rules.release_agl_m
            for sample in window
        )

    def _state_at_or_before(
        self, marker: int, timestamp_ns: int
    ) -> PayloadStateSample | None:
        if self.start_sim_time_ns is None:
            return None
        eligible = [
            sample
            for sample in self._payload_states[marker]
            if self.start_sim_time_ns <= sample.sim_timestamp_ns <= timestamp_ns
        ]
        return eligible[-1] if eligible else None

    def _physical_release(self, marker: int, event: PayloadEventSample) -> bool:
        attached = self._state_at_or_before(marker, event.sim_timestamp_ns)
        detached = next(
            (
                sample
                for sample in self._payload_states[marker]
                if event.sim_timestamp_ns <= sample.sim_timestamp_ns
                <= event.sim_timestamp_ns + self.rules.sample_interval_ns
                and not sample.attached
            ),
            None,
        )
        return (
            attached is not None
            and event.sim_timestamp_ns - attached.sim_timestamp_ns
            <= self.rules.sample_interval_ns
            and attached.attached
            and detached is not None
        )

    def _physical_attach(self, marker: int, event: PayloadEventSample) -> bool:
        states = self._payload_states[marker]
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
            return False
        previous = states[current_index - 1]
        current = states[current_index]
        vehicle = self._vehicle_at(current.sim_timestamp_ns)
        waypoint = _PICKUP_WAYPOINT.get(marker)
        if vehicle is None or waypoint is None or self.start_sim_time_ns is None:
            return False
        return (
            previous.sim_timestamp_ns >= self.start_sim_time_ns
            and current.sim_timestamp_ns - previous.sim_timestamp_ns
            == self.rules.sample_interval_ns
            and previous.grounded
            and not previous.attached
            and current.attached
            and self._inside_waypoint(current.position_xyz, waypoint)
            and vehicle.in_contact
            and math.hypot(
                current.position_xyz[0] - vehicle.position_xyz[0],
                current.position_xyz[1] - vehicle.position_xyz[1],
            )
            <= self.rules.pickup_center_tolerance_m
            and math.dist(current.position_xyz, previous.position_xyz)
            <= self.rules.pickup_center_tolerance_m
        )

    def _capacity_valid(self) -> bool:
        latest: dict[int, PayloadStateSample] = {}
        grouped: dict[int, list[PayloadStateSample]] = {}
        if self.start_sim_time_ns is None:
            return False
        for states in self._payload_states.values():
            for sample in states:
                if sample.sim_timestamp_ns >= self.start_sim_time_ns:
                    grouped.setdefault(sample.sim_timestamp_ns, []).append(sample)
        for timestamp_ns in sorted(grouped):
            for sample in grouped[timestamp_ns]:
                latest[sample.aruco_id] = sample
            if sum(sample.attached for sample in latest.values()) > 1:
                return False
        return True

    @staticmethod
    def _yaw(orientation: tuple[float, float, float, float]) -> float:
        norm = math.sqrt(sum(value * value for value in orientation))
        x, y, z, w = (value / norm for value in orientation)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _inside_f2(self, sample: PayloadStateSample) -> bool:
        yaw = self._yaw(sample.orientation_xyzw)
        half_x = _PAYLOAD_XY_SIZE_M[0] / 2.0
        half_y = _PAYLOAD_XY_SIZE_M[1] / 2.0
        extent_x = abs(math.cos(yaw)) * half_x + abs(math.sin(yaw)) * half_y
        extent_y = abs(math.sin(yaw)) * half_x + abs(math.cos(yaw)) * half_y
        center_x, center_y, width, height = _WAYPOINTS["F2"]
        return (
            abs(sample.position_xyz[0] - center_x) + extent_x <= width / 2.0
            and abs(sample.position_xyz[1] - center_y) + extent_y <= height / 2.0
        )

    def _settled_timestamp(
        self, marker: int, release: PayloadEventSample
    ) -> int | None:
        started_ns: int | None = None
        previous_ns: int | None = None
        for sample in self._payload_states[marker]:
            if sample.sim_timestamp_ns <= release.sim_timestamp_ns:
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
                or sample.sim_timestamp_ns - previous_ns != self.rules.sample_interval_ns
            ):
                started_ns = sample.sim_timestamp_ns if eligible else None
            if eligible and started_ns is not None:
                if sample.sim_timestamp_ns - started_ns >= self.rules.payload_settle_ns:
                    return sample.sim_timestamp_ns
                previous_ns = sample.sim_timestamp_ns
            else:
                previous_ns = None
        return None

    def _delivery(
        self,
        marker: int,
        phase: str,
        *,
        settle_before_ns: int | None,
    ) -> tuple[bool, int | None]:
        phase_events = self._phase_events(phase)
        release = self._payload_event(marker, "release")
        if phase_events is None or release is None:
            return False, None
        if not (
            phase_events[0].sim_timestamp_ns
            <= release.sim_timestamp_ns
            <= phase_events[1].sim_timestamp_ns
            and self._payload_prefix(_PAYLOAD_PREFIX_LENGTH[marker])
            and self._release_gate(release)
            and self._physical_release(marker, release)
        ):
            return False, None
        if marker in (3, 4):
            attach = self._payload_event(marker, "attach")
            if (
                attach is None
                or not phase_events[0].sim_timestamp_ns
                <= attach.sim_timestamp_ns
                < release.sim_timestamp_ns
                or not self._physical_attach(marker, attach)
            ):
                return False, None
        settled = self._settled_timestamp(marker, release)
        if settled is not None and settle_before_ns is not None:
            if settled >= settle_before_ns:
                settled = None
        return settled is not None, settled

    def _evaluate(self) -> tuple[tuple[bool, ...], bool, str | None]:
        if self.start_sim_time_ns is None:
            return (False,) * 7, False, "mission_start_missing"
        mission_events = self._mission_after_start()
        mission_valid = [
            (event.phase, event.state) for event in mission_events
        ] == list(_MISSION_SEQUENCE)
        payload_events = self._payload_after_start()
        payload_valid = [
            (event.aruco_id, event.action, event.state)
            for event in payload_events
        ] == list(_PAYLOAD_SEQUENCE) and all(
            event.code == "OK" for event in payload_events
        )
        capacity_valid = self._capacity_valid()
        marker_3_attach = self._payload_event(3, "attach")
        marker_4_attach = self._payload_event(4, "attach")
        home_checkpoint_boundary = self._home_checkpoint_boundary_timestamp()
        home_landing_timestamp = self._home_landing_timestamp()

        fm1_landing = self._landing("FM1", "L")
        fm1_autonomy = fm1_landing and self._mission_prefix(2)
        payload_2, _settled_2 = self._delivery(
            2,
            "FM2",
            settle_before_ns=(
                None if marker_3_attach is None else marker_3_attach.sim_timestamp_ns
            ),
        )
        payload_2 = payload_2 and fm1_autonomy
        fm2_autonomy = payload_2 and self._mission_prefix(4)
        payload_3, _settled_3 = self._delivery(
            3,
            "FM3_3",
            settle_before_ns=(
                None if marker_4_attach is None else marker_4_attach.sim_timestamp_ns
            ),
        )
        payload_3 = payload_3 and fm2_autonomy and capacity_valid
        fm3_autonomy = payload_3 and self._mission_prefix(6)
        payload_4, _settled_4 = self._delivery(
            4, "FM3_4", settle_before_ns=home_checkpoint_boundary
        )
        payload_4 = payload_4 and fm3_autonomy and capacity_valid
        outcomes = (
            fm1_landing,
            fm1_autonomy,
            payload_2,
            fm2_autonomy,
            payload_3,
            fm3_autonomy,
            payload_4,
        )

        home_events = self._phase_events("HOME")
        home_landed = home_landing_timestamp is not None
        elapsed_ns = (
            None
            if home_events is None
            else home_events[1].sim_timestamp_ns - self.start_sim_time_ns
        )
        deadline_valid = elapsed_ns is not None and elapsed_ns <= self.rules.deadline_ns
        complete = (
            self._diagnostic is None
            and mission_valid
            and payload_valid
            and capacity_valid
            and payload_4
            and home_landed
            and deadline_valid
        )
        if self._diagnostic is not None:
            return (False,) * 7, False, self._diagnostic
        home_complete_claimed = any(
            event.phase == "HOME" and event.state == "COMPLETE"
            for event in mission_events
        )
        if home_complete_claimed and self._home_events() is None:
            return outcomes, False, "home_disarm_missing"
        if not mission_valid:
            diagnostic = "mission_sequence_invalid"
        elif not payload_valid:
            diagnostic = "payload_event_sequence_invalid"
        elif not capacity_valid:
            diagnostic = "payload_capacity_exceeded"
        elif not payload_4:
            diagnostic = "physical_sequence_invalid"
        elif not home_landed:
            diagnostic = "home_completion_invalid"
        elif not deadline_valid:
            diagnostic = "home_deadline_exceeded"
        else:
            diagnostic = None
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
                evidence_ref=f"rosbag#competition:{rule_id}",
            )
            for rule_id, points, passed in zip(
                _RULE_IDS, self.rules.points, outcomes, strict=True
            )
        )
        achieved = sum(result.awarded_points for result in rule_results)
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
                event_id=7,
                event_type="score.finalized",
                value=achieved,
                evidence_ref="scoring/events.jsonl#event-7",
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
    "CompetitionRules",
    "CompetitionScorer",
    "MissionEventSample",
    "PayloadEventSample",
    "PayloadStateSample",
    "load_competition_rules",
]
