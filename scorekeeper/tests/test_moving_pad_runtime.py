from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from artifacts.runtime_status import RuntimeFailureStatus, ScoreFinishedStatus
from drone_sim_scorekeeper.competition import MissionEventSample
from drone_sim_scorekeeper.moving_pad import MovingPadScorer, load_moving_pad_rules
from drone_sim_scorekeeper.moving_pad_runtime import MovingPadScorekeeperRuntime
from drone_sim_scorekeeper.runtime_node import (
    landing_pad_from_message,
    load_runtime_settings,
    rules_path_for_scenario,
)

from .test_moving_pad import DT, RULES_PATH, RUN_ID, pad, vehicle


class ProtocolRecorder:
    def __init__(self) -> None:
        self.statuses = []
        self.quiescence: list[str] = []

    def write_status(self, status) -> None:
        self.statuses.append(status)

    def write_quiescence(self, module: str) -> None:
        self.quiescence.append(module)


def runtime_for(tmp_path: Path, *, expected_frames: int = 43):
    protocol = ProtocolRecorder()
    runtime = MovingPadScorekeeperRuntime(
        RUN_ID,
        MovingPadScorer(
            RUN_ID,
            load_moving_pad_rules(RULES_PATH),
            expected_frames,
        ),
        run_directory=tmp_path,
        protocol=protocol,
        publish=lambda _event: None,
        flush=lambda: None,
    )
    return runtime, protocol


def feed_complete_landing(runtime: MovingPadScorekeeperRuntime, *, omit_pad=None):
    runtime.accept_mission_event(
        MissionEventSample(RUN_ID, DT, 0, "MOVING_PAD", "ARMED", "observed")
    )
    runtime.accept_mission_event(
        MissionEventSample(RUN_ID, 2 * DT, 1, "MOVING_PAD", "DISARMED", "observed")
    )
    for index in range(43):
        if index == 0:
            ground = vehicle(index, z=0.0, velocity_x=0.0, contact=True)
            deck = pad(index)
        elif index == 1:
            ground = vehicle(index, z=5.0)
            deck = pad(index)
        else:
            ground = vehicle(index, contact=True)
            deck = pad(index, contact=True)
        runtime.accept_ground_truth(ground)
        if index != omit_pad:
            runtime.accept_landing_pad(deck)


def test_runtime_pairs_exact_timestamps_and_finishes_durable_score(tmp_path):
    runtime, protocol = runtime_for(tmp_path)
    feed_complete_landing(runtime)

    assert runtime.source_inputs_observed_through(42 * DT) is True
    result = runtime.accept_source_finished(42 * DT)

    assert result.complete is True
    assert result.achieved_score == 100.0
    assert protocol.statuses == [ScoreFinishedStatus(RUN_ID, 42 * DT)]
    persisted = json.loads((tmp_path / "scoring/result.json").read_text())
    assert persisted["ruleset_id"] == "moving_pad_v1"
    assert persisted["achieved_score"] == 100.0


def test_missing_pad_timestamp_cannot_be_repaired_by_later_pairs(tmp_path):
    runtime, protocol = runtime_for(tmp_path)
    feed_complete_landing(runtime, omit_pad=20)

    assert runtime.source_inputs_observed_through(42 * DT) is True
    result = runtime.accept_source_finished(42 * DT)

    assert result.complete is False
    assert result.achieved_score == 0.0
    assert result.diagnostic == "frame_timestamp_gap"
    assert isinstance(protocol.statuses[0], RuntimeFailureStatus)


def test_disarm_before_armed_is_incomplete_even_with_physical_landing(tmp_path):
    runtime, protocol = runtime_for(tmp_path)
    runtime.accept_mission_event(
        MissionEventSample(RUN_ID, DT, 0, "MOVING_PAD", "DISARMED", "observed")
    )
    runtime.accept_mission_event(
        MissionEventSample(RUN_ID, 2 * DT, 1, "MOVING_PAD", "ARMED", "observed")
    )
    for index in range(43):
        ground = vehicle(
            index,
            z=5.0 if index == 1 else (0.0 if index == 0 else 0.35),
            velocity_x=0.0 if index == 0 else 0.5,
            contact=index != 1,
        )
        runtime.accept_ground_truth(ground)
        runtime.accept_landing_pad(pad(index, contact=index >= 2))

    runtime.begin_finalization()

    assert runtime.result is not None
    assert runtime.result.complete is False
    assert runtime.result.diagnostic == "mission_event_sequence_invalid"
    assert runtime.quiescent is True
    assert protocol.quiescence == ["scorekeeper"]


def test_runtime_node_selects_and_translates_moving_pad_evidence(tmp_path):
    config = tmp_path / "run.json"
    config.write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "scenario": "moving_pad_v1",
                "recording": {"fps": 20},
                "simulation": {"duration_sim_seconds": 2.15},
            }
        )
    )
    pose = SimpleNamespace(
        position=SimpleNamespace(x=11.0, y=2.0, z=0.1),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    twist = SimpleNamespace(
        linear=SimpleNamespace(x=0.5, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=0.1),
    )

    settings = load_runtime_settings(config, RUN_ID)
    sample = landing_pad_from_message(
        SimpleNamespace(
            run_id=RUN_ID,
            sim_timestamp=SimpleNamespace(sec=1, nanosec=250_000_000),
            marker_id=7,
            pose=pose,
            twist=twist,
            vehicle_in_contact=True,
        )
    )

    assert settings.scenario == "moving_pad_v1"
    assert settings.expected_ground_truth_samples == 43
    assert rules_path_for_scenario(tmp_path / "rules", settings.scenario) == (
        tmp_path / "rules/moving_pad_v1.json"
    )
    assert sample.sim_timestamp_ns == 1_250_000_000
    assert sample.marker_id == 7
    assert sample.position_xyz == (11.0, 2.0, 0.1)
    assert sample.linear_velocity_xyz == (0.5, 0.0, 0.0)
    assert sample.angular_velocity_xyz == (0.0, 0.0, 0.1)
    assert sample.vehicle_in_contact is True


def test_ros_boundary_subscribes_to_paired_truth_and_mission_events(monkeypatch):
    from drone_sim_scorekeeper.runtime_node import _create_ros_boundary
    from .test_runtime_node import _install_fake_ros

    Node = _install_fake_ros(monkeypatch)

    class Runtime:
        def accept_ground_truth(self, _sample):
            return None

        def accept_landing_pad(self, _sample):
            return None

        def accept_mission_event(self, _sample):
            return None

        def fail(self, _reason):
            return None

    boundary = _create_ros_boundary(
        RUN_ID,
        [Runtime()],
        SimpleNamespace(emit=lambda *_args, **_kwargs: None),
        scenario="moving_pad_v1",
    )

    assert boundary.errors == []
    assert {topic for _type, topic, _callback, _qos in Node.last.subscriptions} == {
        "/simulation/ground_truth",
        "/simulation/landing_pad_state",
        "/simulation/mission_events",
        "/simulation/run_state",
        "/clock",
    }
