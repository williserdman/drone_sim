"""Independent validation for persisted ``competition_2027_v1`` scores."""

from __future__ import annotations

from collections.abc import Callable
import hashlib
import json
import math
from pathlib import Path
from typing import cast

from ._adapters.rosbag import (
    GroundTruthEvidence,
    MissionEventEvidence,
    PayloadStateEvidence,
    PhysicalBagEvidence,
    ScoreEventEvidence,
    VehicleStatusEvidence,
)
from .score_validation import ScoreValidationError, ValidatedScoreMetadata
from .validation import ValidationStatus, read_regular_file_bytes, validate_tree
from .vfs_contract import (
    VfsCourse,
    VfsScenario,
    VfsWaypoint,
    load_vfs_course,
    load_vfs_scenario,
)


_RULE_IDS = ("A", "BP", "CP", "R", "BA", "CA", "T", "deductions")
_AVAILABLE = (65.0, 90.0, 90.0, 1.5, 15.0, 15.0, 15.0, 0.0)


def _read_json(run_directory: Path, relative_path: str, deadline_check: Callable[[], None] | None) -> object:
    result, payload = read_regular_file_bytes(run_directory, relative_path, deadline_check=deadline_check)
    if result.status is not ValidationStatus.VALID or payload is None:
        raise ScoreValidationError(f"{relative_path} is not safe scoring evidence")
    try:
        return json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ScoreValidationError(f"{relative_path} is not valid JSON") from error


def _finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ScoreValidationError(f"{label} must be finite")
    return float(value)


def _rules(path: Path | str) -> tuple[bytes, dict]:
    try:
        payload = Path(path).read_bytes()
        document = json.loads(payload)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ScoreValidationError("committed VFS rules could not be read") from error
    expected_points = {
        "A_manual": 55,
        "A_autonomous": 65,
        "BP_by_target": {"A": 5, "B": 10, "C": 30},
        "CP_per_payload": 30,
        "autonomy_bonus": 15,
    }
    expected_time = list(
        zip((0, 30, 60, 90, 120, 150, 180, 210, 240), (0, 1, 3, 5, 7, 9, 11, 13, 15), strict=True)
    )
    expected_weight = list(
        zip((10, 12, 14, 16, 18, 20, 1e300), (1.5, 1.4, 1.3, 1.2, 1.1, 1.05, 1.0), strict=True)
    )
    if (
        not isinstance(document, dict)
        or set(document) != {"schema_version", "ruleset_id", "points", "time_bonus", "weight_multiplier", "deduction_points"}
        or document.get("schema_version") != 1
        or document.get("ruleset_id") != "competition_2027_v1"
        or document.get("points") != expected_points
        or [(row.get("minimum_remaining_seconds"), row.get("points")) for row in document.get("time_bonus", ())] != expected_time
        or [(row.get("upper_bound_lb"), row.get("multiplier")) for row in document.get("weight_multiplier", ())] != expected_weight
        or document.get("deduction_points") != 25
    ):
        raise ScoreValidationError("committed VFS rules are invalid")
    return payload, document


def _persisted_events(run_directory: Path, run_id: str, deadline_check: Callable[[], None] | None) -> tuple[ScoreEventEvidence, ...]:
    result, payload = read_regular_file_bytes(run_directory, "scoring/events.jsonl", deadline_check=deadline_check)
    if result.status is not ValidationStatus.VALID or payload is None:
        raise ScoreValidationError("scoring/events.jsonl is not safe scoring evidence")
    try:
        rows = [json.loads(line) for line in payload.decode().splitlines()]
        if any(row.get("run_id") != run_id for row in rows):
            raise ScoreValidationError("scoring/events.jsonl belongs to another run")
        events = tuple(
            ScoreEventEvidence(
                int(row["sim_timestamp_ns"]), int(row["event_id"]), row["event_type"],
                _finite(row["value"], "event value"), row["evidence_ref"],
            )
            for row in rows
        )
        if any(event.event_id != index for index, event in enumerate(events)):
            raise ScoreValidationError("scoring/events.jsonl IDs are not contiguous")
        return events
    except (KeyError, TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ScoreValidationError("scoring/events.jsonl has invalid VFS events") from error


def _inside_l(
    sample: PayloadStateEvidence,
    course: VfsCourse,
    scenario: VfsScenario,
) -> bool:
    return _payload_contacts_waypoint(
        sample, course.waypoints["L"], course, scenario
    )


def _autonomous(phase: str, evidence: PhysicalBagEvidence, end_ns: int) -> bool:
    starts = [event.sim_timestamp_ns for event in evidence.mission_events if event.phase == phase and event.state == "STARTED"]
    if not starts:
        return False
    start = starts[-1]
    statuses = [row for row in evidence.vehicle_statuses if start <= row.sim_timestamp_ns <= end_ns]
    return bool(statuses) and all(
        not row.failsafe_active
        and (not row.armed or row.mode in {"GUIDED", "LAND"})
        for row in statuses
    )


def _payload_contacts_target(
    sample: PayloadStateEvidence,
    course: VfsCourse,
    scenario: VfsScenario,
) -> bool:
    waypoint = course.waypoints[scenario.targets[sample.aruco_id]]
    return _payload_contacts_waypoint(sample, waypoint, course, scenario)


def _payload_contacts_waypoint(
    sample: PayloadStateEvidence,
    waypoint: VfsWaypoint,
    course: VfsCourse,
    scenario: VfsScenario,
) -> bool:
    if not sample.grounded:
        return False
    half_x, half_y, half_z = (
        value / 2 for value in scenario.payload_geometry.size_m
    )
    qx, qy, qz, qw = sample.orientation_xyzw
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    rotation = (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )
    center = sample.position_xyz
    corners = tuple(
        tuple(
            center[axis] + sum(rotation[axis][index] * local[index] for index in range(3))
            for axis in range(3)
        )
        for local in (
            (dx, dy, dz)
            for dx in (-half_x, half_x)
            for dy in (-half_y, half_y)
            for dz in (-half_z, half_z)
        )
    )
    minimum_z = min(point[2] for point in corners)
    epsilon = course.attempt.ground_contact_epsilon_m
    if abs(minimum_z - course.ground_z_m) > epsilon:
        return False
    contact_xy = tuple(
        (point[0], point[1])
        for point in corners
        if point[2] <= minimum_z + epsilon
        and abs(point[2] - course.ground_z_m) <= epsilon
    )
    if waypoint.radius is None:
        return any(waypoint.contains(point) for point in contact_xy)
    center_xy = (waypoint.x, waypoint.y)
    radius = waypoint.radius
    if any(math.hypot(x - center_xy[0], y - center_xy[1]) <= radius + 1e-9 for x, y in contact_xy):
        return True
    if len(contact_xy) == 1:
        return False
    ordered = sorted(
        contact_xy,
        key=lambda point: math.atan2(
            point[1] - sum(item[1] for item in contact_xy) / len(contact_xy),
            point[0] - sum(item[0] for item in contact_xy) / len(contact_xy),
        ),
    )
    for first, second in zip(ordered, ordered[1:] + ordered[:1]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length_squared = dx * dx + dy * dy
        fraction = 0.0 if length_squared == 0 else max(
            0.0,
            min(1.0, ((center_xy[0] - first[0]) * dx + (center_xy[1] - first[1]) * dy) / length_squared),
        )
        closest = (first[0] + fraction * dx, first[1] + fraction * dy)
        if math.hypot(closest[0] - center_xy[0], closest[1] - center_xy[1]) <= radius + 1e-9:
            return True
    return False


def _eligible_from_physics(
    decision_ns: int,
    evidence: PhysicalBagEvidence,
    course: VfsCourse,
    scenario: VfsScenario,
) -> tuple[int, ...]:
    eligible = []
    settle_ns = int(scenario.payload_interaction.settle_time_s * 1e9)
    for marker in sorted(scenario.targets):
        stable_since = None
        latest = None
        for sample in evidence.payload_states:
            if sample.aruco_id != marker or sample.sim_timestamp_ns > decision_ns:
                continue
            latest = sample
            stable = (
                math.sqrt(sum(value * value for value in sample.linear_velocity_xyz)) <= 0.10
                and _payload_contacts_target(sample, course, scenario)
            )
            if stable:
                if stable_since is None:
                    stable_since = sample.sim_timestamp_ns
            else:
                stable_since = None
        if (
            latest is not None
            and stable_since is not None
            and 0 <= decision_ns - latest.sim_timestamp_ns <= 50_000_000
            and latest.sim_timestamp_ns - stable_since >= settle_ns
        ):
            eligible.append(marker)
    return tuple(eligible)


def _active_elapsed_from_physics(
    timestamp_ns: int,
    evidence: PhysicalBagEvidence,
    course: VfsCourse,
) -> int:
    events = sorted(
        (
            *((row.sim_timestamp_ns, 0, "truth", row) for row in evidence.ground_truth),
            *((row.sim_timestamp_ns, 1, "status", row) for row in evidence.vehicle_statuses),
            *(
                (row.sim_timestamp_ns, 2, "phase", row)
                for row in evidence.mission_events
                if row.state == "STARTED"
            ),
        ),
        key=lambda item: (item[0], item[1]),
    )
    previous_truth = None
    latest_truth = None
    active_since = None
    accumulated = 0
    paused = False
    status: VehicleStatusEvidence | None = None
    phase = ""
    fm2_airborne = False
    for stamp, _priority, kind, row in events:
        if stamp > timestamp_ns:
            break
        if kind == "truth":
            row = cast(GroundTruthEvidence, row)
            latest_truth = row
            if previous_truth is not None and previous_truth.in_contact and not row.in_contact:
                if phase == "FM2":
                    fm2_airborne = True
                if active_since is None:
                    active_since = stamp
                    paused = False
            elif (
                previous_truth is not None
                and not previous_truth.in_contact
                and row.in_contact
                and phase == "FM3"
                and course.waypoints["H"].contains(
                    row.position_xyz[:2], course.attempt.aircraft_radius_m
                )
                and active_since is not None
            ):
                accumulated += stamp - active_since
                active_since = None
                paused = True
            previous_truth = row
        elif kind == "status":
            status = cast(VehicleStatusEvidence, row)
        else:
            row = cast(MissionEventEvidence, row)
            phase = row.phase
            if phase == "FM2":
                fm2_airborne = latest_truth is not None and not latest_truth.in_contact
        if kind == "status" and (
            status is not None
            and phase == "FM2"
            and fm2_airborne
            and not status.armed
            and latest_truth is not None
            and latest_truth.in_contact
            and course.waypoints["H"].contains(
                latest_truth.position_xyz[:2], course.attempt.aircraft_radius_m
            )
            and stamp - latest_truth.sim_timestamp_ns <= 500_000_000
            and active_since is not None
        ):
            accumulated += stamp - active_since
            active_since = None
            paused = True
    if latest_truth is None:
        raise ScoreValidationError("VFS active clock lacks physical truth")
    if active_since is not None and not paused:
        accumulated += latest_truth.sim_timestamp_ns - active_since
    return accumulated


def _independent_values(run_directory: Path, evidence: PhysicalBagEvidence, rules: dict) -> tuple[float, ...]:
    configuration = _read_json(run_directory, "configuration/run.json", None)
    if not isinstance(configuration, dict) or not isinstance(configuration.get("competition"), dict):
        raise ScoreValidationError("VFS copied configuration provenance is missing")
    provenance = configuration["competition"]
    try:
        course_payload = (run_directory / "configuration/course.yaml").read_bytes()
        scenario_payload = (run_directory / "configuration/scenario.yaml").read_bytes()
    except OSError as error:
        raise ScoreValidationError("VFS copied configuration could not be read") from error
    if (
        hashlib.sha256(course_payload).hexdigest() != provenance.get("course_sha256")
        or hashlib.sha256(scenario_payload).hexdigest() != provenance.get("scenario_sha256")
    ):
        raise ScoreValidationError("VFS copied configuration checksum is invalid")
    course = load_vfs_course(run_directory / "configuration/course.yaml")
    scenario = load_vfs_scenario(run_directory / "configuration/scenario.yaml")
    unique = {row.decision_id: row for row in evidence.judge_decisions}
    approved = [row for row in unique.values() if row.outcome == 1]
    fm1 = next((row for row in approved if row.phase == "FM1" and row.checkpoint == "H"), None)
    fm2 = next((row for row in approved if row.phase == "FM2"), None)
    fm3 = next((row for row in approved if row.phase == "FM3"), None)
    for decision in (fm1, fm2, fm3):
        if decision is not None and decision.active_elapsed_ns != _active_elapsed_from_physics(
            decision.sim_timestamp_ns, evidence, course
        ):
            raise ScoreValidationError(
                "VFS judge active clock does not match physical takeoff/home evidence"
            )
    points = rules.get("points", {})
    a = float(points.get("A_autonomous", 0)) if fm1 and _autonomous("FM1", evidence, fm1.sim_timestamp_ns) else (float(points.get("A_manual", 0)) if fm1 else 0.0)
    eligible = fm2.eligible_aruco_ids if fm2 else ()
    if fm2 is not None and eligible != _eligible_from_physics(
        fm2.sim_timestamp_ns, evidence, course, scenario
    ):
        raise ScoreValidationError(
            "VFS FM2 judge eligibility does not match physical payload evidence"
        )
    target_points = points.get("BP_by_target", {})
    bp = sum(float(target_points[scenario.targets[marker]]) for marker in eligible)
    final_payloads = {}
    for sample in evidence.payload_states:
        if fm3 is not None and sample.sim_timestamp_ns <= fm3.sim_timestamp_ns:
            final_payloads[sample.aruco_id] = sample
    delivered = tuple(
        marker
        for marker in eligible
        if marker in final_payloads
        and final_payloads[marker].grounded
        and not final_payloads[marker].attached
        and _inside_l(final_payloads[marker], course, scenario)
    )
    cp = len(delivered) * float(points.get("CP_per_payload", 0)) if fm3 else 0.0
    ba = float(points.get("autonomy_bonus", 0)) if fm2 and eligible and _autonomous("FM2", evidence, fm2.sim_timestamp_ns) else 0.0
    ca = float(points.get("autonomy_bonus", 0)) if fm3 and delivered and _autonomous("FM3", evidence, fm3.sim_timestamp_ns) else 0.0
    weight_lb = (scenario.aircraft_empty_mass_kg + scenario.payload_geometry.tare_mass_kg) * 2.2046226218487757
    multiplier = next(float(row["multiplier"]) for row in rules["weight_multiplier"] if weight_lb < float(row["upper_bound_lb"]))
    time_points = 0.0
    if fm3 and fm3.active_elapsed_ns <= int(course.attempt.duration_seconds * 1e9):
        remaining = (int(course.attempt.duration_seconds * 1e9) - fm3.active_elapsed_ns) / 1e9
        time_points = max(float(row["points"]) for row in rules["time_bonus"] if remaining >= float(row["minimum_remaining_seconds"]))
    starts = sorted(
        (event.sim_timestamp_ns, event.phase)
        for event in evidence.mission_events
        if event.state == "STARTED"
    )
    for event in evidence.payload_events:
        if event.action != "release" or event.state != "detached":
            continue
        phase = next((phase for stamp, phase in reversed(starts) if stamp <= event.sim_timestamp_ns), "")
        truth = next((row for row in reversed(evidence.ground_truth) if row.sim_timestamp_ns <= event.sim_timestamp_ns), None)
        if phase == "FM2" and truth is not None and truth.position_xyz[2] < course.attempt.minimum_agl_m:
            bp = 0.0
        if phase == "FM3" and truth is not None and not truth.in_contact:
            cp = 0.0
        if (
            phase == "FM3"
            and truth is not None
            and not course.waypoints["L"].contains(
                truth.position_xyz[:2], course.attempt.aircraft_radius_m
            )
        ):
            cp = 0.0
    penalty_count = 0
    for previous, current in zip(evidence.ground_truth, evidence.ground_truth[1:]):
        if previous.in_contact or not current.in_contact:
            continue
        x, y, _z = current.position_xyz
        if not any(course.waypoints[name].contains((x, y)) for name in ("S", "D", "L", "H")):
            penalty_count += 1
        status = next((row for row in reversed(evidence.vehicle_statuses) if row.sim_timestamp_ns <= current.sim_timestamp_ns), None)
        if status is not None and status.failsafe_active and current.sim_timestamp_ns - status.sim_timestamp_ns <= 500_000_000:
            penalty_count += 1
    if fm3 is None and evidence.attempt_statuses and evidence.attempt_statuses[-1].active_elapsed_ns > int(course.attempt.duration_seconds * 1e9):
        penalty_count += 1
    deductions = penalty_count * float(rules.get("deduction_points", 0))
    return a, bp, cp, multiplier, ba, ca, time_points, deductions


def validate_vfs_score_outputs(
    run_directory: Path | str, *, run_id: str, rules_path: Path | str,
    deadline_check: Callable[[], None] | None = None,
    physical_evidence: PhysicalBagEvidence | None = None,
) -> ValidatedScoreMetadata:
    directory = Path(run_directory)
    document = _read_json(directory, "scoring/result.json", deadline_check)
    if not isinstance(document, dict):
        raise ScoreValidationError("VFS result must be an object")
    if physical_evidence is None:
        raise ScoreValidationError("score requires decoded VFS physical evidence")
    rules_payload, rules = _rules(rules_path)
    checksum = hashlib.sha256(rules_payload).hexdigest()
    if document.get("scoring_checksum") != checksum:
        raise ScoreValidationError("VFS scoring checksum does not match committed rules")
    if document.get("run_id") != run_id or document.get("ruleset_id") != "competition_2027_v1" or document.get("complete") is not True or document.get("diagnostic") is not None:
        raise ScoreValidationError("VFS score identity or completion is invalid")
    achieved = _finite(document.get("achieved_score"), "achieved_score")
    maximum = _finite(document.get("maximum_available_score"), "maximum_available_score")
    if maximum != 380.0 or not 0 <= achieved <= maximum:
        raise ScoreValidationError("VFS score is outside the flight range")
    rows = document.get("rule_results")
    if not isinstance(rows, list) or len(rows) != 8:
        raise ScoreValidationError("VFS result must contain eight ordered components")
    values = []
    refs = []
    for index, (row, rule_id, available) in enumerate(zip(rows, _RULE_IDS, _AVAILABLE, strict=True)):
        if not isinstance(row, dict) or row.get("rule_id") != rule_id or _finite(row.get("available_points"), "available_points") != available:
            raise ScoreValidationError("VFS score components are invalid or unordered")
        value = _finite(row.get("awarded_points"), "awarded_points")
        if row.get("passed") is not (value == available) or not isinstance(row.get("evidence_ref"), str) or not row["evidence_ref"]:
            raise ScoreValidationError(f"VFS component {index} is inconsistent")
        values.append(value)
        refs.append(row["evidence_ref"])
    independently_computed = _independent_values(directory, physical_evidence, rules)
    if tuple(values) != independently_computed:
        raise ScoreValidationError("persisted VFS components do not match independently replayed evidence")
    expected_total = max(0.0, values[0] + values[3] * (values[1] + values[2]) + sum(values[4:7]) - values[7])
    if achieved != expected_total or document.get("evidence_paths") != refs:
        raise ScoreValidationError("VFS total or evidence paths are inconsistent")
    events = _persisted_events(directory, run_id, deadline_check)
    if events != physical_evidence.score_events or not events or events[-1].event_type != "score.finalized" or events[-1].value != achieved:
        raise ScoreValidationError("persisted VFS events do not match the authoritative bag")
    penalties = events[8:-2]
    if (
        tuple(event.value for event in events[:8]) != tuple(values)
        or any(
            not event.event_type.startswith("competition_2027.penalty.")
            or event.value != -float(rules["deduction_points"])
            for event in penalties
        )
        or len(penalties) * float(rules["deduction_points"]) != values[7]
        or any(event.sim_timestamp_ns != events[0].sim_timestamp_ns for event in events)
    ):
        raise ScoreValidationError("VFS score events do not encode the replayed components")
    bag = validate_tree(directory, "rosbag", deadline_check=deadline_check)
    if bag.status is not ValidationStatus.VALID or bag.sha256 != physical_evidence.bag_sha256:
        raise ScoreValidationError("VFS physical evidence bag digest changed")
    return ValidatedScoreMetadata(achieved, maximum, checksum, tuple(refs))


__all__ = ["validate_vfs_score_outputs"]
