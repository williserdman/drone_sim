from __future__ import annotations

import json
import math
from pathlib import Path

from drone_sim_scorekeeper.competition import (
    CompetitionScorer,
    MissionEventSample,
    PayloadEventSample,
    PayloadStateSample,
    load_competition_rules,
)
from drone_sim_scorekeeper.descent import GroundTruthSample


RUN_ID = "11111111-1111-4111-8111-111111111111"
RULES = Path(__file__).parents[1] / "rules/competition_v1.json"
DT = 50_000_000
START = 900_000_000_000
F2 = (-152.40, 0.0)
SOURCES = {3: (-45.72, -9.144), 4: (-45.72, 9.144)}


def ground_truth(
    timestamp_ns: int,
    *,
    xy: tuple[float, float],
    z: float,
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
    contact: bool = False,
) -> GroundTruthSample:
    return GroundTruthSample(
        RUN_ID,
        timestamp_ns,
        (xy[0], xy[1], z),
        (0.0, 0.0, 0.0, 1.0),
        velocity,
        (0.0, 0.0, 0.0),
        contact,
    )


def payload_state(
    timestamp_ns: int,
    marker: int,
    *,
    xy: tuple[float, float],
    attached: bool,
    grounded: bool,
    speed: float = 0.0,
    yaw: float = 0.0,
    z: float = 0.0254,
) -> PayloadStateSample:
    return PayloadStateSample(
        RUN_ID,
        timestamp_ns,
        marker,
        (xy[0], xy[1], z),
        (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
        (speed, 0.0, 0.0),
        grounded,
        attached,
    )


class AttemptTrace:
    def __init__(self, scorer: CompetitionScorer) -> None:
        self.scorer = scorer
        self.accepted_inputs: list[tuple[str, object]] = []
        self.start_ns = START
        self.cursor_ns = START
        self.last_tick_ns: int | None = None
        self.mission_event_id = 0
        self.payload_event_id = 0
        self.pending_settlements: dict[
            int, tuple[tuple[float, float], float, float]
        ] = {}
        self.ground_truth_fact: dict[str, object] = {
            "xy": (0.0, 0.0),
            "z": 0.0,
            "velocity": (0.0, 0.0, 0.0),
            "contact": True,
        }
        self.payload_facts: dict[int, dict[str, object]] = {
            2: {
                "xy": (0.0, 0.0),
                "attached": True,
                "grounded": False,
                "speed": 0.0,
                "yaw": 0.0,
                "z": 0.0254,
            },
            3: {
                "xy": SOURCES[3],
                "attached": False,
                "grounded": True,
                "speed": 0.0,
                "yaw": 0.0,
                "z": 0.0254,
            },
            4: {
                "xy": SOURCES[4],
                "attached": False,
                "grounded": True,
                "speed": 0.0,
                "yaw": 0.0,
                "z": 0.0254,
            },
        }

    def mission(self, phase: str, state: str, timestamp_ns: int) -> None:
        sample = MissionEventSample(
            RUN_ID,
            timestamp_ns,
            self.mission_event_id,
            phase,
            state,
            "automatic attempt",
        )
        self.accepted_inputs.append(("mission_event", sample))
        self.scorer.accept_mission_event(sample)
        self.mission_event_id += 1

    def payload_event(self, marker: int, action: str, timestamp_ns: int) -> None:
        sample = PayloadEventSample(
            RUN_ID,
            timestamp_ns,
            self.payload_event_id,
            marker,
            f"run:{marker}:{action}:{self.payload_event_id}",
            action,
            "attached" if action == "attach" else "detached",
            "OK",
        )
        self.accepted_inputs.append(("payload_event", sample))
        self.scorer.accept_payload_event(sample)
        self.payload_event_id += 1

    def accept_ground_truth(self, sample: GroundTruthSample) -> None:
        self.accepted_inputs.append(("ground_truth", sample))
        self.scorer.accept_ground_truth(sample)

    def accept_payload_state(self, sample: PayloadStateSample) -> None:
        self.accepted_inputs.append(("payload_state", sample))
        self.scorer.accept_payload_state(sample)

    def emit_tick(
        self,
        timestamp_ns: int,
        *,
        ground_truth_update: dict[str, object] | None = None,
        payload_updates: dict[int, dict[str, object]] | None = None,
        omit_payloads: frozenset[int] = frozenset(),
    ) -> None:
        if self.last_tick_ns is not None:
            assert timestamp_ns - self.last_tick_ns == DT
        if ground_truth_update is not None:
            self.ground_truth_fact.update(ground_truth_update)
        for marker, update in (payload_updates or {}).items():
            self.payload_facts[marker].update(update)
        self.accept_ground_truth(
            ground_truth(
                timestamp_ns,
                xy=self.ground_truth_fact["xy"],
                z=self.ground_truth_fact["z"],
                velocity=self.ground_truth_fact["velocity"],
                contact=self.ground_truth_fact["contact"],
            )
        )
        for marker in (2, 3, 4):
            if marker in omit_payloads:
                continue
            fact = self.payload_facts[marker]
            self.accept_payload_state(
                payload_state(
                    timestamp_ns,
                    marker,
                    xy=fact["xy"],
                    attached=fact["attached"],
                    grounded=fact["grounded"],
                    speed=fact["speed"],
                    yaw=fact["yaw"],
                    z=fact["z"],
                )
            )
        self.last_tick_ns = timestamp_ns
        self.cursor_ns = timestamp_ns

    def advance_to(
        self,
        timestamp_ns: int,
        *,
        final_ground_truth_update: dict[str, object] | None = None,
        final_payload_updates: dict[int, dict[str, object]] | None = None,
    ) -> None:
        assert self.last_tick_ns is not None
        assert timestamp_ns >= self.last_tick_ns
        while self.last_tick_ns < timestamp_ns:
            next_timestamp_ns = self.last_tick_ns + DT
            self.emit_tick(
                next_timestamp_ns,
                ground_truth_update=(
                    final_ground_truth_update
                    if next_timestamp_ns == timestamp_ns
                    else None
                ),
                payload_updates=(
                    final_payload_updates
                    if next_timestamp_ns == timestamp_ns
                    else None
                ),
            )

    def fm1(self, *, complete: bool = True) -> None:
        self.mission("FM1", "STARTED", self.start_ns)
        self.emit_tick(self.start_ns)
        landing_time = self.start_ns + 10_000_000_000
        self.advance_to(
            landing_time,
            final_ground_truth_update={
                "xy": (-91.44, 0.0),
                "z": 0.0,
                "velocity": (0.0, 0.0, 0.0),
                "contact": True,
            },
        )
        if complete:
            self.mission("FM1", "COMPLETE", landing_time)

    def drop(
        self,
        marker: int,
        *,
        phase: str,
        release_xy: tuple[float, float] = F2,
        release_z: float = 10.0,
        release_speed: float = 0.10,
        stability_samples: int = 41,
        release_attached_age_ns: int = 0,
        attach_event: bool = True,
        attach_truth_delay_ns: int = 0,
        pickup_jump_m: float = 0.0,
        pickup_jump_z_m: float = 0.0,
        capacity_overflow: bool = False,
        settle_xy: tuple[float, float] = F2,
        settle_yaw: float = 0.0,
        settle_speed: float = 0.0,
        pickup_xy: tuple[float, float] | None = None,
        defer_settle: bool = False,
        detach_truth_at_event: bool = False,
    ) -> None:
        phase_start = self.cursor_ns + 1_000_000_000
        self.advance_to(
            phase_start,
            final_payload_updates=(
                {marker: {"xy": pickup_xy}}
                if marker in SOURCES and pickup_xy is not None
                else None
            ),
        )
        self.mission(phase, "STARTED", phase_start)
        if marker in SOURCES:
            attach_time = phase_start + 1_000_000_000
            source = SOURCES[marker] if pickup_xy is None else pickup_xy
            self.advance_to(attach_time - DT)
            attached_xy = (source[0] + pickup_jump_m, source[1])
            attach_updates = {
                marker: {
                    "xy": attached_xy,
                    "attached": True,
                    "grounded": True,
                    "z": 0.0254 + pickup_jump_z_m,
                }
            }
            if capacity_overflow:
                attach_updates[4 if marker == 3 else 3] = {"attached": True}
            if attach_truth_delay_ns == 0:
                self.emit_tick(
                    attach_time,
                    ground_truth_update={
                        "xy": attached_xy,
                        "z": 0.0,
                        "velocity": (0.0, 0.0, 0.0),
                        "contact": True,
                    },
                    payload_updates=attach_updates,
                )
            else:
                self.emit_tick(
                    attach_time,
                    ground_truth_update={
                        "xy": attached_xy,
                        "z": 0.0,
                        "velocity": (0.0, 0.0, 0.0),
                        "contact": True,
                    },
                )
            if attach_event:
                self.payload_event(marker, "attach", attach_time)
            if attach_truth_delay_ns:
                assert attach_truth_delay_ns == DT
                self.emit_tick(
                    attach_time + DT,
                    payload_updates=attach_updates,
                )
            stability_start = attach_time + attach_truth_delay_ns + 1_000_000_000
        else:
            stability_start = phase_start + 1_000_000_000

        self.advance_to(stability_start - DT)
        release_time = stability_start + (stability_samples - 1) * DT
        for index in range(stability_samples):
            timestamp_ns = stability_start + index * DT
            omit = (
                frozenset({marker})
                if release_attached_age_ns
                and timestamp_ns > release_time - release_attached_age_ns
                else frozenset()
            )
            self.emit_tick(
                timestamp_ns,
                ground_truth_update={
                    "xy": release_xy,
                    "z": release_z,
                    "velocity": (release_speed, 0.0, 0.0),
                    "contact": False,
                },
                payload_updates={
                    marker: {
                        "xy": release_xy,
                        "attached": not (
                            detach_truth_at_event and timestamp_ns == release_time
                        ),
                        "grounded": False,
                    }
                },
                omit_payloads=omit,
            )
        self.payload_event(marker, "release", release_time)
        self.emit_tick(
            release_time + DT,
            payload_updates={
                marker: {
                    "xy": release_xy,
                    "attached": False,
                    "grounded": False,
                }
            },
        )
        self.mission(phase, "COMPLETE", release_time + DT)

        if defer_settle:
            self.pending_settlements[marker] = (
                settle_xy,
                settle_yaw,
                settle_speed,
            )
        else:
            self.settle_payload(
                marker,
                xy=settle_xy,
                yaw=settle_yaw,
                speed=settle_speed,
                start_ns=release_time + 1_000_000_000,
            )

    def settle_payload(
        self,
        marker: int,
        *,
        xy: tuple[float, float] | None = None,
        yaw: float | None = None,
        speed: float | None = None,
        start_ns: int | None = None,
    ) -> None:
        pending = self.pending_settlements.pop(marker, None)
        if pending is not None:
            pending_xy, pending_yaw, pending_speed = pending
        else:
            pending_xy, pending_yaw, pending_speed = F2, 0.0, 0.0
        settled_xy = pending_xy if xy is None else xy
        settled_yaw = pending_yaw if yaw is None else yaw
        settled_speed = pending_speed if speed is None else speed
        settle_start = self.cursor_ns + 1_000_000_000 if start_ns is None else start_ns
        self.advance_to(settle_start - DT)
        for index in range(21):
            self.emit_tick(
                settle_start + index * DT,
                payload_updates={
                    marker: {
                        "xy": settled_xy,
                        "attached": False,
                        "grounded": True,
                        "speed": settled_speed,
                        "yaw": settled_yaw,
                    }
                },
            )

    def home(
        self,
        *,
        elapsed_ns: int | None = None,
        disarmed: bool = True,
        landing_after_disarmed: bool = False,
        leave_before_disarmed: bool = False,
    ) -> None:
        complete_time = (
            self.cursor_ns + 2_000_000_000
            if elapsed_ns is None
            else self.start_ns + elapsed_ns
        )
        home_start = complete_time - 1_000_000_000
        self.advance_to(home_start)
        self.mission("HOME", "STARTED", home_start)
        landing_time = complete_time - 2 * DT
        disarmed_time = complete_time - DT
        if landing_after_disarmed:
            self.advance_to(disarmed_time)
        else:
            self.advance_to(
                landing_time,
                final_ground_truth_update={
                    "xy": (0.0, 0.0),
                    "z": 0.0,
                    "velocity": (0.0, 0.0, 0.0),
                    "contact": True,
                },
            )
            self.advance_to(
                disarmed_time,
                final_ground_truth_update=(
                    {
                        "xy": (5.0, 0.0),
                        "z": 0.0,
                        "velocity": (0.2, 0.0, 0.0),
                        "contact": False,
                    }
                    if leave_before_disarmed
                    else None
                ),
            )
        if disarmed:
            self.mission("HOME", "DISARMED", disarmed_time)
        self.advance_to(
            complete_time,
            final_ground_truth_update=(
                {
                    "xy": (0.0, 0.0),
                    "z": 0.0,
                    "velocity": (0.0, 0.0, 0.0),
                    "contact": True,
                }
                if landing_after_disarmed
                else None
            ),
        )
        self.mission("HOME", "COMPLETE", complete_time)


def new_trace() -> AttemptTrace:
    return AttemptTrace(CompetitionScorer(RUN_ID, load_competition_rules(RULES)))


def perfect_trace() -> AttemptTrace:
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home(elapsed_ns=420_000_000_000)
    return trace


def rule_passed(result, rule_id: str) -> bool:
    return next(rule.passed for rule in result.rule_results if rule.rule_id == rule_id)


def cumulative_values(events) -> tuple[float, ...]:
    total = 0.0
    values = []
    for event in events:
        total += event.value
        values.append(total)
    return tuple(values)


def test_perfect_physical_attempt_scores_150_at_required_checkpoints():
    """Changing any official point component or physical prerequisite breaks 150/150."""
    result = perfect_trace().scorer.finalize()

    assert result.complete is True
    assert result.achieved_score == result.maximum_available_score == 150.0
    assert cumulative_values(result.events[:-1]) == (
        20.0,
        50.0,
        60.0,
        80.0,
        95.0,
        145.0,
        150.0,
    )
    assert [event.event_id for event in result.events] == list(range(8))
    assert result.events[-1].event_type == "score.finalized"


def test_fm1_fm2_only_evidence_can_score_80_but_is_not_a_complete_competition():
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")

    result = trace.scorer.finalize()

    assert result.achieved_score == 80.0
    assert result.maximum_available_score == 150.0
    assert result.complete is False
    assert result.diagnostic == "mission_sequence_invalid"


def test_competition_ruleset_rejects_changed_binding_threshold(tmp_path):
    """The same ruleset ID must not silently authorize a larger release error."""
    document = json.loads(RULES.read_text())
    document["release_position_tolerance_m"] = 0.151
    changed = tmp_path / "competition_v1.json"
    changed.write_text(json.dumps(document))

    try:
        load_competition_rules(changed)
    except ValueError:
        pass
    else:
        raise AssertionError("changed competition_v1 threshold was accepted")


def test_missing_fm1_complete_mission_evidence_cannot_award_autonomy():
    """Physical contact at L must not substitute for the ordered FM1 completion."""
    trace = new_trace()
    trace.fm1(complete=False)
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()

    result = trace.scorer.finalize()

    assert rule_passed(result, "fm1_autonomy") is False
    assert result.complete is False


def test_prestart_vehicle_contact_cannot_award_fm1_points():
    """A fresh L contact from before FM1 STARTED must not enter landing evidence."""
    scorer = CompetitionScorer(RUN_ID, load_competition_rules(RULES))
    scorer.accept_ground_truth(
        ground_truth(START - DT, xy=(-91.44, 0.0), z=0.0, contact=True)
    )
    scorer.accept_mission_event(
        MissionEventSample(RUN_ID, START, 0, "FM1", "STARTED", "automatic attempt")
    )
    scorer.accept_mission_event(
        MissionEventSample(RUN_ID, START, 1, "FM1", "COMPLETE", "automatic attempt")
    )

    result = scorer.finalize()

    assert rule_passed(result, "fm1_landing") is False
    assert rule_passed(result, "fm1_autonomy") is False


def test_ground_truth_gap_after_mission_start_fails_closed():
    """A missing vehicle tick can hide a release excursion or physical transition."""
    scorer = CompetitionScorer(RUN_ID, load_competition_rules(RULES))
    scorer.accept_mission_event(
        MissionEventSample(RUN_ID, START, 0, "FM1", "STARTED", "automatic attempt")
    )
    scorer.accept_ground_truth(
        ground_truth(START, xy=(0.0, 0.0), z=0.0, contact=True)
    )
    scorer.accept_ground_truth(
        ground_truth(START + 2 * DT, xy=(0.0, 0.0), z=0.0, contact=True)
    )

    result = scorer.finalize()

    assert result.complete is False
    assert result.diagnostic == "ground_truth_timestamp_gap"


def test_payload_gap_cannot_hide_teleport_or_capacity_transfer():
    """Missing payload ticks cannot bridge two attachments without overlap evidence."""
    scorer = CompetitionScorer(RUN_ID, load_competition_rules(RULES))
    scorer.accept_mission_event(
        MissionEventSample(RUN_ID, START, 0, "FM1", "STARTED", "automatic attempt")
    )
    scorer.accept_payload_state(
        payload_state(
            START,
            2,
            xy=(0.0, 0.0),
            attached=True,
            grounded=False,
        )
    )
    scorer.accept_payload_state(
        payload_state(
            START + 2 * DT,
            2,
            xy=F2,
            attached=False,
            grounded=True,
        )
    )

    result = scorer.finalize()

    assert result.complete is False
    assert result.diagnostic == "payload_2_timestamp_gap"


def test_release_before_two_full_seconds_cannot_award_payload():
    """Forty exact samples span 1.95 seconds and must reset the release gate."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2", stability_samples=40)
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()

    result = trace.scorer.finalize()

    assert rule_passed(result, "payload_2") is False


def test_detached_truth_at_release_timestamp_uses_prior_attached_sample():
    """A transition published with the event must retain the prior physical state."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2", detach_truth_at_event=True)
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_2") is True


def test_release_speed_above_exact_threshold_cannot_award_payload():
    """Rounding 0.100001 m/s down would incorrectly pass the physical gate."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2", release_speed=0.100001)
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_2") is False


def test_release_position_above_exact_threshold_cannot_award_payload():
    """Rounding a 0.150001 m horizontal error would incorrectly pass."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2", release_xy=(F2[0] + 0.150001, F2[1]))
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_2") is False


def test_release_below_ten_metres_agl_cannot_award_payload():
    """A confirmed release below the required 10 m AGL is not score evidence."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2", release_z=9.999999)
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_2") is False


def test_stale_attachment_truth_at_release_cannot_award_payload():
    """A confirmed release cannot reuse physical attachment older than one state tick."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2", release_attached_age_ns=2 * DT)
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_2") is False


def test_rotated_payload_bounds_outside_f2_cannot_award_payload():
    """Checking only the center or unrotated footprint would accept this 45-degree box."""
    trace = new_trace()
    trace.fm1()
    trace.drop(
        2,
        phase="FM2",
        settle_xy=(F2[0] + 0.36, F2[1]),
        settle_yaw=math.pi / 4.0,
    )
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_2") is False


def test_settling_speed_above_threshold_cannot_award_payload():
    """Ground contact alone must not count as one second of settled payload truth."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2", settle_speed=0.100001)
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_2") is False


def test_missing_confirmed_attach_before_fm3_release_cannot_award_payload():
    """An attached state without its matching confirmed pickup event is insufficient."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3", attach_event=False)
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_3") is False


def test_fm3_pickup_outside_configured_source_zone_cannot_score():
    """Matching vehicle/payload centers away from WA must not satisfy marker 3 pickup."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3", pickup_xy=(0.0, 0.0))
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_3") is False


def test_marker_four_pickup_at_wa_cannot_substitute_for_configured_wm_source():
    """The two FM3 markers have distinct configured physical pickup sources."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4", pickup_xy=SOURCES[3])
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_4") is False


def test_next_tick_physical_attach_truth_confirms_fresh_event():
    """Gazebo confirmation may precede recurrent attached truth by one 50 ms tick."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3", attach_truth_delay_ns=DT)
    trace.drop(4, phase="FM3_4")
    trace.home()

    result = trace.scorer.finalize()

    assert rule_passed(result, "payload_3") is True
    assert result.achieved_score == 150.0


def test_payload_pose_jump_at_pickup_cannot_award_payload():
    """An attach transition must not teleport the payload farther than pickup tolerance."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3", pickup_jump_m=0.075001)
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_3") is False


def test_payload_vertical_pose_jump_at_pickup_cannot_award_payload():
    """Checking only XY continuity would permit a payload to teleport vertically."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3", pickup_jump_z_m=0.075001)
    trace.drop(4, phase="FM3_4")
    trace.home()

    assert rule_passed(trace.scorer.finalize(), "payload_3") is False


def test_capacity_greater_than_one_invalidates_fm3_payload():
    """Two concurrent physical attachments must never be treated as free capacity."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3", capacity_overflow=True)
    trace.drop(4, phase="FM3_4")
    trace.home()

    result = trace.scorer.finalize()

    assert rule_passed(result, "payload_3") is False
    assert result.complete is False


def test_home_after_mission_relative_deadline_is_incomplete():
    """The 600-second limit is measured from authoritative FM1 STARTED."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home(elapsed_ns=600_050_000_000)

    result = trace.scorer.finalize()

    assert result.complete is False
    assert result.diagnostic == "home_deadline_exceeded"


def test_large_absolute_clock_does_not_consume_mission_deadline():
    """A 900-second source epoch cannot make a 420-second mission late."""
    result = perfect_trace().scorer.finalize()

    assert result.events[-1].sim_timestamp_ns == START + 420_000_000_000
    assert result.complete is True


def test_out_of_order_marker_four_cannot_earn_its_payload_points():
    """Delivering marker 4 before marker 3 must not satisfy the ordered allocation."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(4, phase="FM3_4")
    trace.drop(3, phase="FM3_3")
    trace.home()

    result = trace.scorer.finalize()

    assert rule_passed(result, "payload_4") is False
    assert result.complete is False


def test_payload_two_settling_after_marker_three_cannot_backfill_checkpoint():
    """Marker 3 physical work cannot begin before payload 2 has settled for scoring."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2", defer_settle=True)
    trace.drop(3, phase="FM3_3")
    trace.settle_payload(2)
    trace.drop(4, phase="FM3_4")
    trace.home()

    result = trace.scorer.finalize()

    assert rule_passed(result, "payload_2") is False
    assert result.achieved_score < 150.0


def test_payload_three_settling_after_marker_four_cannot_backfill_checkpoint():
    """Marker 4 physical work cannot begin before payload 3 has settled for scoring."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3", defer_settle=True)
    trace.drop(4, phase="FM3_4")
    trace.settle_payload(3)
    trace.home()

    result = trace.scorer.finalize()

    assert rule_passed(result, "payload_3") is False
    assert result.achieved_score < 150.0


def test_payload_four_settling_after_home_cannot_backfill_final_score():
    """Home landing/completion must occur only after payload 4 physically settles."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4", defer_settle=True)
    trace.home()
    trace.settle_payload(4)

    result = trace.scorer.finalize()

    assert rule_passed(result, "payload_4") is False
    assert result.complete is False


def test_home_complete_without_distinct_disarmed_event_is_incomplete():
    """A companion HOME/COMPLETE claim alone cannot prove ArduPilot disarm."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home(disarmed=False)

    result = trace.scorer.finalize()

    assert result.complete is False
    assert result.diagnostic == "home_disarm_missing"


def test_home_disarmed_before_physical_landing_is_incomplete():
    """ArduPilot disarm evidence must follow, not precede, the Gazebo landing."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home(landing_after_disarmed=True)

    result = trace.scorer.finalize()

    assert result.complete is False
    assert result.diagnostic == "home_completion_invalid"


def test_home_contact_then_departure_before_disarmed_is_incomplete():
    """A stale first Home contact cannot hide contradictory terminal truth."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home(leave_before_disarmed=True)

    result = trace.scorer.finalize()

    assert result.complete is False
    assert result.diagnostic == "home_completion_invalid"


def test_valid_home_finalizes_honest_partial_score():
    """A missed payload component changes points, not terminal validity."""
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4", release_speed=0.100001)
    trace.home()

    result = trace.scorer.finalize()

    assert result.complete is True
    assert result.diagnostic is None
    assert result.achieved_score == 145.0
    assert rule_passed(result, "payload_4") is False
    assert [event.event_id for event in result.events] == list(range(8))


def test_confirmed_release_before_mission_start_earns_no_score():
    """Pre-start physical/event history cannot be rebased into the attempt."""
    trace = new_trace()
    prestart = START - 10_000_000_000
    for index in range(41):
        trace.scorer.accept_ground_truth(
            ground_truth(prestart + index * DT, xy=F2, z=10.0)
        )
    trace.scorer.accept_payload_state(
        payload_state(
            prestart + 40 * DT,
            2,
            xy=F2,
            attached=True,
            grounded=False,
        )
    )
    trace.payload_event(2, "release", prestart + 40 * DT)
    for index in range(21):
        trace.scorer.accept_payload_state(
            payload_state(
                prestart + 3_000_000_000 + index * DT,
                2,
                xy=F2,
                attached=False,
                grounded=True,
            )
        )
    trace.fm1()
    trace.home()

    result = trace.scorer.finalize()

    assert rule_passed(result, "payload_2") is False


def test_non_ok_payload_event_cannot_disappear_from_ordered_evidence():
    """Filtering an invalid event out could let a later valid-looking suffix score."""
    trace = new_trace()
    trace.fm1()
    trace.scorer.accept_payload_event(
        PayloadEventSample(
            RUN_ID,
            trace.cursor_ns + DT,
            0,
            2,
            "run:2:release:rejected",
            "release",
            "unknown",
            "PHYSICAL_CONFIRMATION_MISMATCH",
        )
    )
    trace.payload_event_id = 1
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()

    result = trace.scorer.finalize()

    assert result.complete is False
    assert rule_passed(result, "payload_2") is False
