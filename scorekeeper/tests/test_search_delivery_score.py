from __future__ import annotations

from pathlib import Path

from drone_sim_scorekeeper.competition import (
    MissionEventSample,
    PayloadEventSample,
    PayloadStateSample,
)
from drone_sim_scorekeeper.descent import GroundTruthSample
from drone_sim_scorekeeper.search_delivery import (
    SearchDeliveryScorer,
    load_search_delivery_rules,
)
from drone_sim_scorekeeper.competition_runtime import CompetitionScorekeeperRuntime
from drone_sim_scorekeeper.runtime_node import (
    load_runtime_settings,
    rules_path_for_scenario,
)


RUN_ID = "11111111-1111-4111-8111-111111111111"
RULES = Path(__file__).parents[1] / "rules/search_delivery_v1.json"
DT = 50_000_000
WA = (18.0, 8.0)
F2 = (6.0, 20.0)


def _ground_truth(
    stamp: int,
    xy: tuple[float, float],
    z: float,
    *,
    contact: bool = False,
) -> GroundTruthSample:
    return GroundTruthSample(
        RUN_ID,
        stamp,
        (xy[0], xy[1], z),
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        contact,
    )


def _payload(
    stamp: int,
    xy: tuple[float, float],
    z: float,
    *,
    grounded: bool,
    attached: bool,
) -> PayloadStateSample:
    return PayloadStateSample(
        RUN_ID,
        stamp,
        3,
        (xy[0], xy[1], z),
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0),
        grounded,
        attached,
    )


def _mission(stamp: int, event_id: int, phase: str, state: str) -> MissionEventSample:
    return MissionEventSample(
        RUN_ID, stamp, event_id, phase, state, "automatic attempt"
    )


def _payload_event(stamp: int, event_id: int, action: str) -> PayloadEventSample:
    return PayloadEventSample(
        RUN_ID,
        stamp,
        event_id,
        3,
        f"run:3:{action}:{event_id}",
        action,
        "attached" if action == "attach" else "detached",
        "OK",
    )


def _trace(
    *,
    searched: bool = True,
    picked_up: bool = True,
    release_z: float = 10.0,
) -> SearchDeliveryScorer:
    scorer = SearchDeliveryScorer(RUN_ID, load_search_delivery_rules(RULES))
    scorer.accept_mission_event(_mission(0, 0, "SEARCH", "STARTED"))

    mission_events = {
        1_500_000_000: _mission(1_500_000_000, 1, "SEARCH", "COMPLETE"),
        1_550_000_000: _mission(1_550_000_000, 2, "DELIVERY", "STARTED"),
        7_000_000_000: _mission(7_000_000_000, 3, "DELIVERY", "COMPLETE"),
        7_050_000_000: _mission(7_050_000_000, 4, "HOME", "STARTED"),
        7_150_000_000: _mission(7_150_000_000, 5, "HOME", "DISARMED"),
        7_200_000_000: _mission(7_200_000_000, 6, "HOME", "COMPLETE"),
    }
    payload_events = {
        1_450_000_000: _payload_event(1_450_000_000, 0, "attach"),
        4_000_000_000: _payload_event(4_000_000_000, 1, "release"),
    }

    for stamp in range(0, 7_200_000_000 + DT, DT):
        if stamp < 500_000_000:
            vehicle_xy, vehicle_z, contact = (0.0, 0.0), 0.0, True
        elif stamp < 600_000_000:
            vehicle_xy = (16.0, 8.0) if searched else (15.6, 8.0)
            vehicle_z, contact = 4.572, False
        elif stamp < 1_050_000_000:
            vehicle_xy, vehicle_z, contact = (16.8, 8.0), 4.572, False
        elif stamp <= 1_500_000_000:
            vehicle_xy, vehicle_z, contact = WA, 0.0, True
        elif stamp < 2_000_000_000:
            vehicle_xy, vehicle_z, contact = WA, 1.0, False
        elif stamp <= 7_050_000_000:
            vehicle_xy, vehicle_z, contact = F2, release_z, False
        else:
            vehicle_xy, vehicle_z, contact = (0.0, 0.0), 0.0, True

        attached = picked_up and 1_450_000_000 <= stamp <= 4_000_000_000
        if stamp < 1_550_000_000:
            payload_xy, payload_z, grounded = WA, 0.0254, not attached
        elif stamp <= 4_000_000_000:
            payload_xy, payload_z, grounded = vehicle_xy, vehicle_z, False
        elif stamp < 4_500_000_000:
            payload_xy, payload_z, grounded = F2, 1.0, False
        else:
            payload_xy, payload_z, grounded = F2, 0.0254, True

        scorer.accept_ground_truth(
            _ground_truth(stamp, vehicle_xy, vehicle_z, contact=contact)
        )
        scorer.accept_payload_state(
            _payload(
                stamp,
                payload_xy,
                payload_z,
                grounded=grounded,
                attached=attached,
            )
        )
        if stamp in payload_events:
            scorer.accept_payload_event(payload_events[stamp])
        if stamp in mission_events:
            scorer.accept_mission_event(mission_events[stamp])
    return scorer


def _passed(result, rule_id: str) -> bool:
    return next(rule.passed for rule in result.rule_results if rule.rule_id == rule_id)


def test_search_delivery_physical_attempt_scores_100_with_stable_contract():
    result = _trace().finalize()

    assert result.complete is True
    assert result.achieved_score == result.maximum_available_score == 100.0
    assert [rule.rule_id for rule in result.rule_results] == [
        "search",
        "pickup",
        "delivery",
        "home",
    ]
    assert [event.event_id for event in result.events] == list(range(5))
    assert [event.event_type for event in result.events] == [
        "search_delivery.search",
        "search_delivery.pickup",
        "search_delivery.delivery",
        "search_delivery.home",
        "score.finalized",
    ]
    assert result.evidence_paths[-4:] == (
        "rosbag#/simulation/ground_truth",
        "rosbag#/simulation/payload_state",
        "rosbag#/simulation/payload_events",
        "rosbag#/simulation/mission_events",
    )


def test_search_rule_needs_staging_then_eastward_physical_motion():
    result = _trace(searched=False).finalize()

    assert result.complete is True
    assert _passed(result, "search") is False
    assert result.achieved_score == 75.0


def test_attach_event_without_physical_pickup_cannot_award_pickup_or_delivery():
    result = _trace(picked_up=False).finalize()

    assert result.complete is True
    assert _passed(result, "pickup") is False
    assert _passed(result, "delivery") is False
    assert result.achieved_score == 50.0


def test_release_below_ten_metres_cannot_award_delivery():
    result = _trace(release_z=9.999).finalize()

    assert result.complete is True
    assert _passed(result, "search") is True
    assert _passed(result, "pickup") is True
    assert _passed(result, "delivery") is False
    assert _passed(result, "home") is True
    assert result.achieved_score == 75.0


def test_missing_20_hz_vehicle_sample_fails_closed():
    scorer = SearchDeliveryScorer(RUN_ID, load_search_delivery_rules(RULES))
    scorer.accept_mission_event(_mission(0, 0, "SEARCH", "STARTED"))
    scorer.accept_ground_truth(_ground_truth(0, (0.0, 0.0), 0.0, contact=True))
    scorer.accept_ground_truth(
        _ground_truth(2 * DT, (0.0, 0.0), 0.0, contact=True)
    )
    scorer.accept_payload_state(
        _payload(0, WA, 0.0254, grounded=True, attached=False)
    )

    result = scorer.finalize()

    assert result.complete is False
    assert result.achieved_score == 0.0
    assert result.diagnostic == "ground_truth_timestamp_gap"


def test_runtime_selects_240_second_search_delivery_rules(tmp_path):
    config = tmp_path / "run.json"
    config.write_text(
        __import__("json").dumps(
            {
                "run_id": RUN_ID,
                "scenario": "search_delivery_v1",
                "recording": {"fps": 20},
                "simulation": {"duration_sim_seconds": 240.0},
            }
        )
    )

    settings = load_runtime_settings(config, RUN_ID)

    assert settings.scenario == "search_delivery_v1"
    assert settings.expected_ground_truth_samples == 4_800
    assert rules_path_for_scenario(tmp_path / "rules", settings.scenario) == (
        tmp_path / "rules/search_delivery_v1.json"
    )


def test_runtime_source_drain_requires_only_payload_3_and_two_payload_events(tmp_path):
    class Protocol:
        def write_status(self, _name, _document):
            return None

        def write_quiescence(self, _module):
            return None

    runtime = CompetitionScorekeeperRuntime(
        RUN_ID,
        SearchDeliveryScorer(RUN_ID, load_search_delivery_rules(RULES)),
        run_directory=tmp_path,
        protocol=Protocol(),
        publish=lambda _event: None,
        flush=lambda: None,
    )
    runtime.accept_ground_truth(_ground_truth(100, (0.0, 0.0), 0.0, contact=True))
    runtime.accept_payload_state(
        _payload(100, WA, 0.0254, grounded=True, attached=False)
    )
    runtime.accept_payload_event(_payload_event(10, 0, "attach"))
    runtime.accept_payload_event(_payload_event(20, 1, "release"))
    for event_id, (phase, state) in enumerate(
        (
            ("SEARCH", "STARTED"),
            ("SEARCH", "COMPLETE"),
            ("DELIVERY", "STARTED"),
            ("DELIVERY", "COMPLETE"),
            ("HOME", "STARTED"),
            ("HOME", "DISARMED"),
            ("HOME", "COMPLETE"),
        )
    ):
        runtime.accept_mission_event(
            _mission(30 + event_id * 10, event_id, phase, state)
        )

    assert runtime.source_inputs_observed_through(100) is True
