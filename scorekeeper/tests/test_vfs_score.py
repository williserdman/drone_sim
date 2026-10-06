from __future__ import annotations

from dataclasses import dataclass
import json
import math

from drone_sim_scorekeeper.competition import MissionEventSample, PayloadEventSample, PayloadStateSample
from drone_sim_scorekeeper.descent import GroundTruthSample
from drone_sim_scorekeeper.vfs import (
    AttemptState,
    JudgeDecision,
    JudgeOutcome,
    VehicleStatusSample,
    VfsJudge,
    VfsScorer,
    load_vfs_rules,
)


RUN_ID = "00000000-0000-4000-8000-000000002027"


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    role: str
    width: float | None = None
    height: float | None = None
    radius: float | None = None


@dataclass(frozen=True)
class Attempt:
    duration_seconds: float = 600.0
    release_agl_m: float = 9.144
    minimum_agl_m: float = 9.144
    approval_timeout_s: float = 60.0
    ground_contact_epsilon_m: float = 0.01
    aircraft_radius_m: float = 0.35


@dataclass(frozen=True)
class Course:
    waypoints: dict[str, Waypoint]
    attempt: Attempt = Attempt()


@dataclass(frozen=True)
class Payload:
    aruco_id: int
    target: str


@dataclass(frozen=True)
class Geometry:
    size_m: tuple[float, float, float] = (0.1524, 0.1524, 0.0762)


@dataclass(frozen=True)
class Scenario:
    payloads: tuple[Payload, ...] = (
        Payload(2, "C"), Payload(3, "C"), Payload(4, "C")
    )
    payload_geometry: Geometry = Geometry()


COURSE = Course(
    {
        "H": Waypoint("H", 0.0, 0.0, "home", radius=2.286),
        "D": Waypoint("D", -91.44, 0.0, "zone", width=15.24, height=19.812),
        "L": Waypoint("L", -30.48, 0.0, "delivery", width=15.24, height=19.812),
        "A": Waypoint("A", -86.106, 3.048, "target", radius=1.0668),
        "B": Waypoint("B", -86.106, 0.0, "target", radius=0.762),
        "C": Waypoint("C", -86.106, -3.048, "target", radius=0.4572),
    }
)


def truth(seconds: float, x: float, y: float, z: float, contact: bool) -> GroundTruthSample:
    return GroundTruthSample(
        RUN_ID, int(seconds * 1e9), (x, y, z), (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), contact,
    )


def payload(seconds: float, marker: int, x: float, y: float, *, grounded: bool) -> PayloadStateSample:
    return PayloadStateSample(
        RUN_ID, int(seconds * 1e9), marker, (x, y, 0.0381),
        (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), grounded, False,
    )


def status(seconds: float, sequence: int, *, armed: bool, mode: str = "GUIDED") -> VehicleStatusSample:
    return VehicleStatusSample(RUN_ID, int(seconds * 1e9), sequence, "iris", armed, mode, False, "")


def request(seconds: float, event_id: int, phase: str, checkpoint: str = "") -> MissionEventSample:
    return MissionEventSample(
        RUN_ID, int(seconds * 1e9), event_id, phase, "CHECK_REQUESTED",
        json.dumps({"checkpoint": checkpoint, "request_id": f"request-{event_id}"}, separators=(",", ":")),
    )


def test_circle_boundary_ground_contact_is_eligible_but_suspended_overhang_is_not():
    judge = VfsJudge(RUN_ID, COURSE, Scenario())
    judge.accept_ground_truth(truth(1, 0, 0, 0, True))
    judge.accept_vehicle_status(status(1, 0, armed=False))
    for seconds in (0, 1):
        judge.accept_payload_state(payload(seconds, 2, -86.106 + 0.4572 + 0.0762, -3.048, grounded=True))
        judge.accept_payload_state(payload(seconds, 3, -86.106, -3.048, grounded=False))
        judge.accept_payload_state(payload(seconds, 4, -80, -3.048, grounded=True))

    decision = judge.accept_mission_event(request(1, 0, "FM2"))[0]

    assert decision.outcome is JudgeOutcome.APPROVED
    assert decision.eligible_aruco_ids == (2,)


def test_tilted_aerial_overhang_without_circle_ground_contact_is_ineligible():
    judge = VfsJudge(RUN_ID, COURSE, Scenario())
    judge.accept_ground_truth(truth(1, 0, 0, 0, True))
    judge.accept_vehicle_status(status(1, 0, armed=False))
    half_angle = math.radians(22.5)
    orientation = (0.0, math.sin(half_angle), 0.0, math.cos(half_angle))
    center_x = -86.106 + 0.4572 + 0.04
    center_z = 0.080822304
    for seconds in (0, 1):
        judge.accept_payload_state(PayloadStateSample(
            RUN_ID, int(seconds * 1e9), 2, (center_x, -3.048, center_z),
            orientation, (0.0, 0.0, 0.0), True, False,
        ))
        for marker in (3, 4):
            judge.accept_payload_state(payload(seconds, marker, -80, 0, grounded=True))
    decision = judge.accept_mission_event(request(1, 8, "FM2"))[0]
    assert decision.outcome is JudgeOutcome.APPROVED
    assert decision.eligible_aruco_ids == ()


def test_attempt_clock_starts_pauses_and_resumes_from_physical_truth():
    judge = VfsJudge(RUN_ID, COURSE, Scenario())
    judge.accept_ground_truth(truth(0, 0, 0, 0, True))
    assert judge.attempt_status.state is AttemptState.WAITING
    judge.accept_ground_truth(truth(1, 0, 0, 1, False))
    judge.accept_mission_event(MissionEventSample(RUN_ID, 2_000_000_000, 9, "FM2", "STARTED", ""))
    judge.accept_ground_truth(truth(11, 0, 0, 0, True))
    judge.accept_payload_state(payload(10, 2, -86.106, -3.048, grounded=True))
    judge.accept_payload_state(payload(11, 2, -86.106, -3.048, grounded=True))
    for marker in (3, 4):
        judge.accept_payload_state(payload(10, marker, -80, 0, grounded=True))
        judge.accept_payload_state(payload(11, marker, -80, 0, grounded=True))
    judge.accept_vehicle_status(status(11, 0, armed=False))
    judge.accept_ground_truth(truth(20, 0, 0, 0, True))
    judge.accept_payload_state(payload(20, 2, -86.106, -3.048, grounded=True))
    for marker in (3, 4):
        judge.accept_payload_state(payload(20, marker, -80, 0, grounded=True))
    judge.accept_vehicle_status(status(20, 1, armed=False))
    judge.accept_mission_event(request(20, 0, "FM2"))
    assert judge.attempt_status.state is AttemptState.APPROVED
    assert judge.attempt_status.active_elapsed_ns == 10_000_000_000
    judge.accept_vehicle_status(status(21, 2, armed=True))
    judge.accept_ground_truth(truth(21, 0, 0, 1, False))
    assert judge.attempt_status.state is AttemptState.ACTIVE
    assert judge.attempt_status.active_elapsed_ns == 10_000_000_000
    judge.accept_ground_truth(truth(26, 0, 0, 1, False))
    assert judge.attempt_status.active_elapsed_ns == 15_000_000_000


def test_approval_to_takeoff_over_sixty_seconds_finishes_attempt():
    judge = VfsJudge(RUN_ID, COURSE, Scenario())
    judge.accept_ground_truth(truth(0, 0, 0, 0, True))
    judge.accept_ground_truth(truth(1, 0, 0, 1, False))
    judge.accept_mission_event(MissionEventSample(RUN_ID, 2_000_000_000, 1, "FM2", "STARTED", ""))
    judge.accept_ground_truth(truth(10, 0, 0, 0, True))
    judge.accept_payload_state(payload(9, 2, -86.106, -3.048, grounded=True))
    judge.accept_payload_state(payload(10, 2, -86.106, -3.048, grounded=True))
    for marker in (3, 4):
        judge.accept_payload_state(payload(9, marker, -80, 0, grounded=True))
        judge.accept_payload_state(payload(10, marker, -80, 0, grounded=True))
    judge.accept_vehicle_status(status(10, 0, armed=False))
    judge.accept_mission_event(request(10, 2, "FM2"))
    judge.accept_ground_truth(truth(71, 0, 0, 0, True))
    assert judge.attempt_status.state is AttemptState.FINISHED


def test_fm3_home_touchdown_stops_active_clock_before_check_request():
    judge = VfsJudge(RUN_ID, COURSE, Scenario())
    judge.accept_ground_truth(truth(0, 0, 0, 0, True))
    judge.accept_ground_truth(truth(1, 0, 0, 1, False))
    judge.accept_mission_event(MissionEventSample(RUN_ID, 2_000_000_000, 1, "FM3", "STARTED", ""))
    judge.accept_ground_truth(truth(11, 0, 0, 0, True))
    judge.accept_ground_truth(truth(20, 0, 0, 0, True))
    assert judge.attempt_status.active_elapsed_ns == 10_000_000_000


def test_duplicate_check_replays_decision_and_conflicting_request_id_errors():
    judge = VfsJudge(RUN_ID, COURSE, Scenario())
    judge.accept_ground_truth(truth(1, 0, 0, 0, True))
    judge.accept_vehicle_status(status(1, 0, armed=False))
    first = judge.accept_mission_event(request(1, 4, "FM2"))[0]
    replay = judge.accept_mission_event(request(2, 4, "FM2"))[0]
    conflict = MissionEventSample(
        RUN_ID, 3_000_000_000, 5, "FM3", "CHECK_REQUESTED",
        '{"checkpoint":"","request_id":"request-4"}',
    )
    rejected = judge.accept_mission_event(conflict)[0]
    assert replay == first
    assert rejected.outcome is JudgeOutcome.ERROR
    assert rejected.reason_code == "request_id_conflict"


def test_stale_payload_truth_cannot_enter_frozen_eligibility():
    judge = VfsJudge(RUN_ID, COURSE, Scenario())
    judge.accept_ground_truth(truth(2, 0, 0, 0, True))
    judge.accept_vehicle_status(status(2, 0, armed=False))
    judge.accept_payload_state(payload(0, 2, -86.106, -3.048, grounded=True))
    judge.accept_payload_state(payload(1, 2, -86.106, -3.048, grounded=True))
    decision = judge.accept_mission_event(request(2, 7, "FM2"))[0]
    assert decision.outcome is JudgeOutcome.ERROR
    assert decision.reason_code == "evidence_stale_or_missing"


def test_fm1_home_approval_requires_d_checkpoint_and_return_takeoff():
    judge = VfsJudge(RUN_ID, COURSE, Scenario())
    judge.accept_ground_truth(truth(0, 0, 0, 0, True))
    judge.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM1", "STARTED", ""))
    judge.accept_ground_truth(truth(1, 0, 0, 10, False))
    judge.accept_ground_truth(truth(2, -91.44, 0, 0, True))
    judge.accept_vehicle_status(status(2, 0, armed=False))
    assert judge.accept_mission_event(request(2, 1, "FM1", "D"))[0].outcome is JudgeOutcome.APPROVED
    judge.accept_ground_truth(truth(3, 0, 0, 0, True))
    judge.accept_vehicle_status(status(3, 1, armed=False))
    early = judge.accept_mission_event(request(3, 2, "FM1", "H"))[0]
    assert early.outcome is JudgeOutcome.REJECTED
    judge.accept_ground_truth(truth(4, 0, 0, 10, False))
    judge.accept_ground_truth(truth(5, 0, 0, 0, True))
    judge.accept_vehicle_status(status(5, 2, armed=False))
    approved = judge.accept_mission_event(request(5, 3, "FM1", "H"))[0]
    assert approved.outcome is JudgeOutcome.APPROVED


def test_full_autonomous_all_c_score_is_365_plus_time_bonus():
    rules = load_vfs_rules("scorekeeper/rules/competition_2027_v1.json")
    scorer = VfsScorer(RUN_ID, rules, COURSE, Scenario(), unloaded_weight_lb=9.0)
    scorer.record_fm1(autonomous=True, complete=True, evidence_ref="judge#fm1")
    scorer.record_fm2((2, 3, 4), autonomous=True, evidence_ref="judge#fm2")
    scorer.record_fm3((2, 3, 4), autonomous=True, evidence_ref="judge#fm3")
    scorer.record_active_time(360_000_000_000, home_before_deadline=True)

    result = scorer.finalize(400_000_000_000)

    assert result.achieved_score == 380.0
    assert result.maximum_available_score == 380.0
    values = {row.rule_id: row.awarded_points for row in result.rule_results}
    assert values == {
        "A": 65.0, "BP": 90.0, "CP": 90.0, "R": 1.5,
        "BA": 15.0, "CA": 15.0, "T": 15.0, "deductions": 0.0,
    }


def test_adjustments_and_deductions_clamp_flight_total_to_zero():
    rules = load_vfs_rules("scorekeeper/rules/competition_2027_v1.json")
    scorer = VfsScorer(RUN_ID, rules, COURSE, Scenario(), unloaded_weight_lb=21.0)
    scorer.record_fm1(autonomous=False, complete=True, evidence_ref="judge#fm1")
    scorer.record_fm2((2,), autonomous=False, evidence_ref="judge#fm2")
    scorer.invalidate_bp("low_release", "payload-events#4")
    scorer.add_penalty("off_course_contact", "ground-truth#5")
    scorer.add_penalty("failsafe_landing", "vehicle-status#6")
    scorer.add_penalty("late_home", "ground-truth#7")
    result = scorer.finalize(700_000_000_000)
    assert result.achieved_score == 0.0


def test_rules_loader_rejects_changed_official_point_table(tmp_path):
    source = json.loads(open("scorekeeper/rules/competition_2027_v1.json").read())
    source["points"]["BP_by_target"]["C"] = 31
    path = tmp_path / "rules.json"
    path.write_text(json.dumps(source))
    try:
        load_vfs_rules(path)
    except ValueError as error:
        assert "official" in str(error)
    else:
        raise AssertionError("changed point table was accepted")


def test_raw_failsafe_landing_outside_allowed_zones_applies_two_deductions():
    rules = load_vfs_rules("scorekeeper/rules/competition_2027_v1.json")
    scorer = VfsScorer(RUN_ID, rules, COURSE, Scenario(), unloaded_weight_lb=9.0)
    scorer.accept_ground_truth(truth(1, 50, 50, 1, False))
    scorer.accept_vehicle_status(VehicleStatusSample(
        RUN_ID, 2_000_000_000, 0, "iris", True, "LAND", True, "radio failsafe"
    ))
    scorer.accept_ground_truth(truth(2, 50, 50, 0, True))
    scorer.record_fm1(autonomous=False, complete=True, evidence_ref="judge#fm1")
    result = scorer.finalize(2_000_000_000)
    assert result.achieved_score == 5.0


def test_active_deadline_without_home_adds_late_home_deduction():
    rules = load_vfs_rules("scorekeeper/rules/competition_2027_v1.json")
    scorer = VfsScorer(RUN_ID, rules, COURSE, Scenario(), unloaded_weight_lb=9.0)
    scorer.record_fm1(autonomous=False, complete=True, evidence_ref="judge#fm1")
    scorer.record_active_time(600_000_000_001, home_before_deadline=False)
    assert scorer.finalize(601_000_000_000).achieved_score == 30.0


def test_real_course_uses_thirty_foot_minimum_for_fm2_release_height():
    from artifacts.vfs_contract import load_vfs_course, load_vfs_scenario

    course = load_vfs_course("config/course-2027.yaml")
    scenario = load_vfs_scenario("config/scenario-2027.yaml")
    rules = load_vfs_rules("scorekeeper/rules/competition_2027_v1.json")

    def scored_bp(height_m: float) -> float:
        scorer = VfsScorer(RUN_ID, rules, course, scenario, unloaded_weight_lb=9.0)
        scorer.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM2", "STARTED", ""))
        scorer.accept_ground_truth(truth(1, -86.106, -3.048, height_m, False))
        scorer.accept_payload_event(PayloadEventSample(
            RUN_ID, 1_000_000_000, 0, 2, "release-2", "release", "detached", "OK"
        ))
        scorer.record_fm2((2,), autonomous=True, evidence_ref="judge#fm2")
        return {row.rule_id: row.awarded_points for row in scorer.finalize(2_000_000_000).rule_results}["BP"]

    assert scored_bp(9.5) == 30.0
    assert scored_bp(9.0) == 0.0


def test_fm3_grounded_release_without_full_aircraft_footprint_in_l_zeros_cp():
    rules = load_vfs_rules("scorekeeper/rules/competition_2027_v1.json")
    scorer = VfsScorer(RUN_ID, rules, COURSE, Scenario(), unloaded_weight_lb=9.0)
    scorer.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM3", "STARTED", ""))
    l = COURSE.waypoints["L"]
    scorer.accept_ground_truth(truth(1, l.x + l.width / 2, l.y, 0, True))
    scorer.accept_payload_event(PayloadEventSample(
        RUN_ID, 1_000_000_000, 0, 2, "release-2", "release", "detached", "OK"
    ))
    scorer.record_fm3((2,), autonomous=True, evidence_ref="judge#fm3")

    result = scorer.finalize(2_000_000_000)

    cp = next(row for row in result.rule_results if row.rule_id == "CP")
    assert cp.awarded_points == 0.0
    assert cp.evidence_ref == "payload-events#0:invalid_landing"


def test_real_fm2_disarmed_landing_outside_home_does_not_pause_or_approve():
    from artifacts.vfs_contract import load_vfs_course, load_vfs_scenario

    course = load_vfs_course("config/course-2027.yaml")
    scenario = load_vfs_scenario("config/scenario-2027.yaml")
    judge = VfsJudge(RUN_ID, course, scenario)
    judge.accept_ground_truth(truth(0, 0, 0, 0, True))
    judge.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM2", "STARTED", ""))
    judge.accept_ground_truth(truth(1, 0, 0, 10, False))
    judge.accept_ground_truth(truth(10, -91.44, 0, 0, True))
    judge.accept_vehicle_status(status(10, 0, armed=False))
    for marker in (2, 3, 4):
        judge.accept_payload_state(payload(9, marker, -80, 0, grounded=True))
        judge.accept_payload_state(payload(10, marker, -80, 0, grounded=True))
    decision = judge.accept_mission_event(request(10, 1, "FM2"))[0]
    assert judge.attempt_status.state is AttemptState.ACTIVE
    assert decision.outcome is JudgeOutcome.REJECTED


def test_real_fm3_takeoff_clears_pending_approval_before_home_touchdown():
    from artifacts.vfs_contract import load_vfs_course, load_vfs_scenario

    course = load_vfs_course("config/course-2027.yaml")
    scenario = load_vfs_scenario("config/scenario-2027.yaml")
    judge = VfsJudge(RUN_ID, course, scenario)
    judge.accept_ground_truth(truth(0, 0, 0, 0, True))
    judge.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM2", "STARTED", ""))
    judge.accept_ground_truth(truth(1, 0, 0, 10, False))
    judge.accept_ground_truth(truth(10, 0, 0, 0, True))
    judge.accept_vehicle_status(status(10, 0, armed=False))
    for marker, x, y in ((2, -86.106, -3.048), (3, -80, 0), (4, -80, 0)):
        judge.accept_payload_state(payload(9, marker, x, y, grounded=True))
        judge.accept_payload_state(payload(10, marker, x, y, grounded=True))
    assert judge.accept_mission_event(request(10, 1, "FM2"))[0].outcome is JudgeOutcome.APPROVED
    judge.accept_mission_event(MissionEventSample(RUN_ID, 11_000_000_000, 2, "FM3", "STARTED", ""))
    judge.accept_ground_truth(truth(20, 0, 0, 10, False))
    assert judge.attempt_status.approval_timestamp_ns == 0
    judge.accept_ground_truth(truth(30, 0, 0, 0, True))
    assert judge.attempt_status.state is AttemptState.PAUSED


def test_real_fm1_checkpoint_requires_disarm_and_thirty_foot_leg_peak():
    from artifacts.vfs_contract import load_vfs_course, load_vfs_scenario

    course = load_vfs_course("config/course-2027.yaml")
    scenario = load_vfs_scenario("config/scenario-2027.yaml")
    judge = VfsJudge(RUN_ID, course, scenario)
    judge.accept_ground_truth(truth(0, 0, 0, 0, True))
    judge.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM1", "STARTED", ""))
    judge.accept_ground_truth(truth(1, 0, 0, 9.0, False))
    judge.accept_ground_truth(truth(2, -91.44, 0, 0, True))
    judge.accept_vehicle_status(status(2, 0, armed=False))
    assert judge.accept_mission_event(request(2, 1, "FM1", "D"))[0].outcome is JudgeOutcome.REJECTED

    judge = VfsJudge(RUN_ID, course, scenario)
    judge.accept_ground_truth(truth(0, 0, 0, 0, True))
    judge.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM1", "STARTED", ""))
    judge.accept_ground_truth(truth(1, 0, 0, 9.5, False))
    judge.accept_ground_truth(truth(2, -91.44, 0, 0, True))
    judge.accept_vehicle_status(status(2, 0, armed=True))
    assert judge.accept_mission_event(request(2, 1, "FM1", "D"))[0].outcome is JudgeOutcome.REJECTED


def test_disarmed_preflight_mode_does_not_forfeit_fm1_autonomy():
    rules = load_vfs_rules("scorekeeper/rules/competition_2027_v1.json")
    scorer = VfsScorer(RUN_ID, rules, COURSE, Scenario(), unloaded_weight_lb=9.0)
    scorer.accept_mission_event(MissionEventSample(RUN_ID, 0, 0, "FM1", "STARTED", ""))
    scorer.accept_vehicle_status(status(0, 0, armed=False, mode="STABILIZE"))
    scorer.accept_vehicle_status(status(1, 1, armed=True, mode="GUIDED"))
    scorer.accept_judge_decision(JudgeDecision(
        RUN_ID, 2_000_000_000, 0, "fm1-home", 1, "FM1", "H",
        JudgeOutcome.APPROVED, (), 2_000_000_000, "landing_approved", ("judge#fm1",),
    ))
    result = scorer.finalize(2_000_000_000)
    assert {row.rule_id: row.awarded_points for row in result.rule_results}["A"] == 65.0


def _approved_decision(phase: str, eligible: tuple[int, ...] = ()) -> JudgeDecision:
    return JudgeDecision(
        RUN_ID, 1_000_000_000, 0, f"{phase}-request", 1, phase, "",
        JudgeOutcome.APPROVED, eligible, 1_000_000_000, "approved", (f"judge#{phase}",),
    )


def test_ineligible_grounded_payload_in_l_cannot_earn_cp():
    rules = load_vfs_rules("scorekeeper/rules/competition_2027_v1.json")
    scorer = VfsScorer(RUN_ID, rules, COURSE, Scenario(), unloaded_weight_lb=9.0)
    scorer.accept_judge_decision(_approved_decision("FM2", (2,)))
    scorer.accept_payload_state(payload(2, 3, -30.48, 0, grounded=True))
    scorer.accept_judge_decision(_approved_decision("FM3"))
    result = scorer.finalize(2_000_000_000)
    assert {row.rule_id: row.awarded_points for row in result.rule_results}["CP"] == 0.0


def test_attached_payload_in_l_cannot_earn_cp():
    rules = load_vfs_rules("scorekeeper/rules/competition_2027_v1.json")
    scorer = VfsScorer(RUN_ID, rules, COURSE, Scenario(), unloaded_weight_lb=9.0)
    scorer.accept_judge_decision(_approved_decision("FM2", (2,)))
    scorer.accept_payload_state(PayloadStateSample(
        RUN_ID, 2_000_000_000, 2, (-30.48, 0, 0.0381),
        (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.0), True, True,
    ))
    scorer.accept_judge_decision(_approved_decision("FM3"))
    result = scorer.finalize(2_000_000_000)
    assert {row.rule_id: row.awarded_points for row in result.rule_results}["CP"] == 0.0
