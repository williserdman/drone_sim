from __future__ import annotations

import json
import sys
from types import ModuleType, SimpleNamespace

import pytest

from artifacts.runtime_status import ScoreFinishedStatus, SourceFinishedStatus
from drone_sim_scorekeeper.runtime_node import (
    _RosBoundary,
    ScorekeeperDriver,
    ground_truth_from_message,
    load_runtime_settings,
    scenario_from_message,
    score_event_message,
)
from drone_sim_scorekeeper.descent import DescentScorer, load_descent_rules
from drone_sim_scorekeeper.runtime import ScenarioSample, ScorekeeperRuntime


RUN_ID = "11111111-1111-4111-8111-111111111111"
RULES = __import__("pathlib").Path(__file__).parents[1] / "rules/descent_v1.json"


def _stamp(ns: int):
    return SimpleNamespace(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def test_runtime_settings_derive_six_hundred_samples_from_resolved_config(tmp_path):
    """A production node must not retain the Phase 2 hard-coded 40 samples."""
    config = tmp_path / "run.json"
    config.write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "scenario": "descent_v1",
                "recording": {"fps": 20},
                "simulation": {"duration_sim_seconds": 30.0},
            }
        ),
        encoding="utf-8",
    )

    settings = load_runtime_settings(config, RUN_ID)

    assert settings.expected_ground_truth_samples == 600


@pytest.mark.parametrize("duration", [0, 1.001, True, "30"])
def test_runtime_settings_reject_non_grid_duration(tmp_path, duration):
    """Rounding duration would permit a truncated trace to look complete."""
    config = tmp_path / "run.json"
    config.write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "scenario": "descent_v1",
                "recording": {"fps": 20},
                "simulation": {"duration_sim_seconds": duration},
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError):
        load_runtime_settings(config, RUN_ID)


def test_ros_ground_truth_conversion_preserves_exact_simulation_values():
    """Using receipt or wall time would change score inputs under host slowdown."""
    message = SimpleNamespace(
        run_id=RUN_ID,
        sim_timestamp=_stamp(1_250_000_000),
        vehicle_id="iris",
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.1, y=0.2, z=-0.8),
            angular=SimpleNamespace(x=0.01, y=0.02, z=0.03),
        ),
        in_contact=True,
    )

    sample = ground_truth_from_message(message)

    assert sample.run_id == RUN_ID
    assert sample.sim_timestamp_ns == 1_250_000_000
    assert sample.position_xyz == (1.0, 2.0, 3.0)
    assert sample.linear_velocity_xyz == (0.1, 0.2, -0.8)
    assert sample.angular_velocity_xyz == (0.01, 0.02, 0.03)
    assert sample.in_contact is True


def test_ros_scenario_conversion_and_score_event_output_are_exact():
    """Changing event identity or timestamp would break bag/evidence correlation."""
    scenario = scenario_from_message(
        SimpleNamespace(
            run_id=RUN_ID,
            sim_timestamp=_stamp(50_000_000),
            event_id=7,
            magnet_id="landing_pad",
            state="INACTIVE",
        )
    )
    event = SimpleNamespace(
        run_id=RUN_ID,
        sim_timestamp_ns=1_500_000_000,
        event_id=4,
        event_type="score.finalized",
        value=100.0,
        evidence_ref="scoring/events.jsonl#event-4",
    )

    message = score_event_message(event, SimpleNamespace)

    assert scenario.event_id == 7
    assert scenario.state == "INACTIVE"
    assert message.run_id == RUN_ID
    assert (message.sim_timestamp.sec, message.sim_timestamp.nanosec) == (1, 500_000_000)
    assert message.event_id == 4
    assert message.event_type == "score.finalized"
    assert message.value == 100.0
    assert message.evidence_ref == "scoring/events.jsonl#event-4"


def test_ros_boundary_flush_uses_jazzy_duration_timeout(monkeypatch):
    """Jazzy accepts ``timeout=Duration(...)``, not ``timeout_sec=...``."""
    class Duration:
        def __init__(self, *, seconds=0, nanoseconds=0):
            self.seconds = seconds
            self.nanoseconds = nanoseconds

    duration_module = ModuleType("rclpy.duration")
    duration_module.Duration = Duration
    monkeypatch.setitem(sys.modules, "rclpy.duration", duration_module)

    class Publisher:
        def __init__(self):
            self.timeout = None

        def wait_for_all_acked(self, timeout):
            self.timeout = timeout
            return True

    publisher = Publisher()

    _RosBoundary(node=None, publisher=publisher, errors=[]).flush()

    assert publisher.timeout.seconds == 5.0
    assert publisher.timeout.nanoseconds == 0


def test_driver_observes_source_and_finalize_once_then_waits_for_terminal(tmp_path):
    """Polling duplicates must not republish score or let the service exit before commit."""
    class Protocol:
        def __init__(self):
            self.finalize = None
            self.terminal = None
            self.quiescence = []
            self.statuses = []

        def read_status(self, status_type):
            assert status_type is SourceFinishedStatus
            return SourceFinishedStatus(RUN_ID, 0)

        def read_finalize_request(self):
            return self.finalize

        def read_terminal_committed(self):
            return self.terminal

        def write_status(self, status):
            self.statuses.append(status)

        def write_quiescence(self, module):
            self.quiescence.append(module)

    protocol = Protocol()
    scorer = DescentScorer(
        RUN_ID, load_descent_rules(RULES), expected_ground_truth_samples=1
    )
    ground_truth = ground_truth_from_message(SimpleNamespace(
        run_id=RUN_ID,
        sim_timestamp=_stamp(0),
        vehicle_id="iris",
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        in_contact=False,
    ))
    published = []
    runtime = ScorekeeperRuntime(
        RUN_ID,
        scorer,
        run_directory=tmp_path,
        protocol=protocol,
        publish=published.append,
        flush=lambda: None,
    )
    runtime.accept_scenario(ScenarioSample(RUN_ID, 0, 0, "landing_pad", "INACTIVE"))
    runtime.accept_ground_truth(ground_truth)
    driver = ScorekeeperDriver(RUN_ID, runtime, protocol)

    assert driver.poll() is False
    assert driver.poll() is False
    assert len(published) == 5
    assert protocol.statuses == [ScoreFinishedStatus(RUN_ID, 0)]
    protocol.finalize = {
        "run_id": RUN_ID,
        "requested_terminal": "COMPLETED",
        "reason": "mission_complete",
    }
    assert driver.poll() is False
    assert protocol.quiescence == ["scorekeeper"]
    protocol.terminal = {
        "run_id": RUN_ID,
        "terminal_status": "COMPLETED",
        "reason": "mission_complete",
        "manifest_path": "manifest.json",
    }
    assert driver.poll() is True


def test_driver_drains_ros_ground_truth_through_source_timestamp_before_scoring(tmp_path):
    """The durable source marker can race ahead of still-queued ROS samples."""
    class Protocol:
        def __init__(self):
            self.statuses = []

        def read_status(self, status_type):
            assert status_type is SourceFinishedStatus
            return SourceFinishedStatus(RUN_ID, 0)

        def read_finalize_request(self):
            return None

        def read_terminal_committed(self):
            return None

        def write_status(self, status):
            self.statuses.append(status)

        def write_quiescence(self, _module):
            raise AssertionError("finalization was not requested")

    scorer = DescentScorer(
        RUN_ID, load_descent_rules(RULES), expected_ground_truth_samples=1
    )
    protocol = Protocol()
    runtime = ScorekeeperRuntime(
        RUN_ID,
        scorer,
        run_directory=tmp_path,
        protocol=protocol,
        publish=lambda _event: None,
        flush=lambda: None,
    )
    driver = ScorekeeperDriver(RUN_ID, runtime, protocol)

    assert driver.poll() is False
    assert runtime.result is None
    runtime.accept_ground_truth(ground_truth_from_message(SimpleNamespace(
        run_id=RUN_ID,
        sim_timestamp=_stamp(0),
        vehicle_id="iris",
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        ),
        in_contact=False,
    )))
    assert driver.poll() is False
    assert runtime.result is None
    runtime.accept_scenario(ScenarioSample(RUN_ID, 0, 0, "landing_pad", "INACTIVE"))
    assert driver.poll() is False
    assert runtime.result is not None
    assert runtime.result.complete is True
    assert protocol.statuses == [ScoreFinishedStatus(RUN_ID, 0)]
