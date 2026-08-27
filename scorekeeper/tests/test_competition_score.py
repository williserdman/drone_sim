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
        self.mission_event_id = 0
        self.payload_event_id = 0

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

    def fm1(self, *, complete: bool = True) -> None:
        self.mission("FM1", "STARTED", self.start_ns)
        self.accept_ground_truth(
            ground_truth(self.start_ns, xy=(0.0, 0.0), z=0.0, contact=True)
        )
        self.accept_payload_state(
            payload_state(
                self.start_ns,
                2,
                xy=(0.0, 0.0),
                attached=True,
                grounded=False,
            )
        )
        for marker, xy in SOURCES.items():
            self.accept_payload_state(
                payload_state(
                    self.start_ns,
                    marker,
                    xy=xy,
                    attached=False,
                    grounded=True,
                )
            )
        self.cursor_ns = self.start_ns + 10_000_000_000
        self.accept_ground_truth(
            ground_truth(self.cursor_ns, xy=(-91.44, 0.0), z=0.0, contact=True)
        )
        if complete:
            self.mission("FM1", "COMPLETE", self.cursor_ns)

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
    ) -> None:
        phase_start = self.cursor_ns + 1_000_000_000
        self.mission(phase, "STARTED", phase_start)
        if marker in SOURCES:
            attach_time = phase_start + 1_000_000_000
            source = SOURCES[marker]
            detached_time = attach_time - DT if attach_truth_delay_ns == 0 else attach_time
            self.accept_payload_state(
                payload_state(
                    detached_time,
                    marker,
                    xy=source,
                    attached=False,
                    grounded=True,
                )
            )
            attached_xy = (source[0] + pickup_jump_m, source[1])
            self.accept_ground_truth(
                ground_truth(attach_time, xy=attached_xy, z=0.0, contact=True)
            )
            self.accept_payload_state(
                payload_state(
                    attach_time + attach_truth_delay_ns,
                    marker,
                    xy=attached_xy,
                    attached=True,
                    grounded=True,
                    z=0.0254 + pickup_jump_z_m,
                )
            )
            if capacity_overflow:
                other = 4 if marker == 3 else 3
                self.accept_payload_state(
                    payload_state(
                        attach_time,
                        other,
                        xy=SOURCES[other],
                        attached=True,
                        grounded=True,
                    )
                )
            if attach_event:
                self.payload_event(marker, "attach", attach_time)
            stability_start = attach_time + attach_truth_delay_ns + 1_000_000_000
        else:
            stability_start = phase_start + 1_000_000_000

        for index in range(stability_samples):
            self.accept_ground_truth(
                ground_truth(
                    stability_start + index * DT,
                    xy=release_xy,
                    z=release_z,
                    velocity=(release_speed, 0.0, 0.0),
                )
            )
        release_time = stability_start + (stability_samples - 1) * DT
        self.accept_payload_state(
            payload_state(
                release_time - release_attached_age_ns,
                marker,
                xy=release_xy,
                attached=True,
                grounded=False,
            )
        )
        self.payload_event(marker, "release", release_time)
        self.accept_payload_state(
            payload_state(
                release_time + DT,
                marker,
                xy=release_xy,
                attached=False,
                grounded=False,
            )
        )
        self.mission(phase, "COMPLETE", release_time + DT)

        settle_start = release_time + 1_000_000_000
        for index in range(21):
            self.accept_payload_state(
                payload_state(
                    settle_start + index * DT,
                    marker,
                    xy=settle_xy,
                    attached=False,
                    grounded=True,
                    speed=settle_speed,
                    yaw=settle_yaw,
                )
            )
        self.cursor_ns = settle_start + 20 * DT

    def home(self, *, elapsed_ns: int = 420_000_000_000) -> None:
        complete_time = self.start_ns + elapsed_ns
        self.mission("HOME", "STARTED", complete_time - 1_000_000_000)
        self.accept_ground_truth(
            ground_truth(complete_time, xy=(0.0, 0.0), z=0.0, contact=True)
        )
        for marker in (2, 3, 4):
            self.accept_payload_state(
                payload_state(
                    complete_time,
                    marker,
                    xy=F2,
                    attached=False,
                    grounded=True,
                )
            )
        self.mission("HOME", "COMPLETE", complete_time)
        self.cursor_ns = complete_time


def new_trace() -> AttemptTrace:
    return AttemptTrace(CompetitionScorer(RUN_ID, load_competition_rules(RULES)))


def perfect_trace() -> AttemptTrace:
    trace = new_trace()
    trace.fm1()
    trace.drop(2, phase="FM2")
    trace.drop(3, phase="FM3_3")
    trace.drop(4, phase="FM3_4")
    trace.home()
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
