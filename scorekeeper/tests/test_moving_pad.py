from __future__ import annotations

from pathlib import Path

from drone_sim_scorekeeper.descent import GroundTruthSample
from drone_sim_scorekeeper.moving_pad import (
    LandingPadSample,
    MovingPadScorer,
    load_moving_pad_rules,
)


RUN_ID = "11111111-1111-4111-8111-111111111111"
RULES_PATH = Path(__file__).parents[1] / "rules/moving_pad_v1.json"
DT = 50_000_000


def vehicle(
    index: int,
    *,
    pad_x: float | None = None,
    y: float = 0.0,
    z: float = 0.35,
    velocity_x: float = 0.5,
    contact: bool = False,
) -> GroundTruthSample:
    x = 10.0 + 0.5 * index * DT / 1_000_000_000 if pad_x is None else pad_x
    return GroundTruthSample(
        RUN_ID,
        index * DT,
        (x, y, z),
        (0.0, 0.0, 0.0, 1.0),
        (velocity_x, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        contact,
    )


def pad(
    index: int,
    *,
    timestamp_ns: int | None = None,
    contact: bool = False,
) -> LandingPadSample:
    return LandingPadSample(
        RUN_ID,
        index * DT if timestamp_ns is None else timestamp_ns,
        7,
        (10.0 + 0.5 * index * DT / 1_000_000_000, 0.0, 0.1),
        (0.0, 0.0, 0.0, 1.0),
        (0.5, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        contact,
    )


def scorer(frame_count: int) -> MovingPadScorer:
    return MovingPadScorer(
        RUN_ID,
        load_moving_pad_rules(RULES_PATH),
        frame_count,
    )


def start_airborne(subject: MovingPadScorer) -> None:
    subject.accept_frame(vehicle(0, z=0.0, velocity_x=0.0, contact=True), pad(0))
    subject.accept_frame(vehicle(1, z=5.0, contact=False), pad(1))


def test_disarmed_vehicle_aboard_for_two_endpoint_seconds_scores_100():
    subject = scorer(43)
    start_airborne(subject)
    subject.accept_disarmed(2 * DT)

    for index in range(2, 43):
        subject.accept_frame(vehicle(index, contact=True), pad(index, contact=True))

    result = subject.finalize()

    assert result.complete is True
    assert result.achieved_score == 100.0
    assert result.maximum_available_score == 100.0
    assert result.diagnostic is None
    assert [(row.rule_id, row.passed) for row in result.rule_results] == [
        ("physical_landing", True)
    ]
    assert [event.event_type for event in result.events] == [
        "moving_pad.physical_landing",
        "moving_pad.touchdown_offset_m",
        "moving_pad.touchdown_relative_velocity_mps",
        "score.finalized",
    ]
    assert [event.value for event in result.events] == [100.0, 0.0, 0.0, 100.0]


def test_forty_aboard_samples_are_short_of_two_endpoint_seconds():
    subject = scorer(42)
    start_airborne(subject)
    subject.accept_disarmed(2 * DT)
    for index in range(2, 42):
        subject.accept_frame(vehicle(index, contact=True), pad(index, contact=True))

    result = subject.finalize()

    assert result.complete is True
    assert result.achieved_score == 0.0


def test_stationary_control_uses_the_same_physical_rules():
    subject = scorer(43)

    def stationary_pad(index: int, *, contact: bool = False) -> LandingPadSample:
        return LandingPadSample(
            RUN_ID,
            index * DT,
            7,
            (35.0, 0.0, 0.1),
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            contact,
        )

    subject.accept_frame(
        vehicle(0, pad_x=35.0, z=0.0, velocity_x=0.0, contact=True),
        stationary_pad(0),
    )
    subject.accept_frame(
        vehicle(1, pad_x=35.0, z=5.0, velocity_x=0.0),
        stationary_pad(1),
    )
    subject.accept_disarmed(2 * DT)
    for index in range(2, 43):
        subject.accept_frame(
            vehicle(index, pad_x=35.0, velocity_x=0.0, contact=True),
            stationary_pad(index, contact=True),
        )

    result = subject.finalize()

    assert result.complete is True
    assert result.achieved_score == 100.0
    assert result.events[2].value == 0.0


def test_floor_landing_near_marker_cannot_pass_without_pad_contact():
    subject = scorer(43)
    start_airborne(subject)
    subject.accept_disarmed(2 * DT)
    for index in range(2, 43):
        subject.accept_frame(vehicle(index, z=0.0, contact=True), pad(index))

    result = subject.finalize()

    assert result.complete is True
    assert result.achieved_score == 0.0


def test_marker_passing_below_airborne_vehicle_cannot_pass():
    subject = scorer(43)
    start_airborne(subject)
    subject.accept_disarmed(2 * DT)
    for index in range(2, 43):
        subject.accept_frame(vehicle(index, z=5.0), pad(index))

    result = subject.finalize()

    assert result.complete is True
    assert result.achieved_score == 0.0


def test_sliding_off_resets_continuous_aboard_window():
    subject = scorer(43)
    start_airborne(subject)
    subject.accept_disarmed(2 * DT)
    for index in range(2, 43):
        y = 1.6 if index == 22 else 0.0
        subject.accept_frame(
            vehicle(index, y=y, contact=True),
            pad(index, contact=True),
        )

    result = subject.finalize()

    assert result.complete is True
    assert result.achieved_score == 0.0


def test_missing_joined_frame_is_incomplete_instead_of_zero_point_failure():
    subject = scorer(43)
    start_airborne(subject)
    subject.accept_disarmed(2 * DT)
    for index in range(2, 42):
        subject.accept_frame(vehicle(index, contact=True), pad(index, contact=True))

    result = subject.finalize()

    assert result.complete is False
    assert result.achieved_score == 0.0
    assert result.diagnostic == "frame_count_mismatch"


def test_misaligned_vehicle_and_pad_timestamps_are_incomplete():
    subject = scorer(1)

    subject.accept_frame(vehicle(0), pad(0, timestamp_ns=DT))
    result = subject.finalize()

    assert result.complete is False
    assert result.achieved_score == 0.0
    assert result.diagnostic == "frame_timestamp_mismatch"


def test_missing_disarm_is_incomplete():
    subject = scorer(2)
    start_airborne(subject)

    result = subject.finalize()

    assert result.complete is False
    assert result.diagnostic == "disarm_missing"
