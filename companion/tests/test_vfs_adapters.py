from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / "companion/comp2026/src"))

from artifacts.vfs_contract import load_vfs_scenario
from drone.sensors.camera._camera_manager import CameraManager
from drone.sensors.camera.camera import Camera
from drone_sim_companion.mission_plan import Condition
from drone_sim_companion.mission import Telemetry
from drone_sim_companion.configured_vision import ConfiguredVision
from drone_sim_companion.vfs_operations import (
    ConfiguredRangefinder,
    PayloadActions,
    PhaseExchange,
    VehicleStatusStream,
)

def test_phase_check_detail_is_canonical_and_eligibility_is_frozen() -> None:
    events = []
    def publish(*fields):
        events.append(fields)
        return len(events) - 1

    exchange = PhaseExchange("run", publish)
    exchange.check("FM1", "D", "request-1", 50)
    assert events == [(
        "FM1",
        "CHECK_REQUESTED",
        '{"checkpoint":"D","request_id":"request-1"}',
        50,
    )]

    exchange.check("FM2", "", "fm2", 60)
    exchange.accept(SimpleNamespace(
        run_id="run", outcome=1, APPROVED=1, REJECTED=2, ERROR=3,
        request_id="fm2", phase="FM2", checkpoint="",
        request_event_id=1, eligible_aruco_ids=[2, 4], reason_code="APPROVED",
    ))
    assert exchange.evaluate(Condition("any_payload_eligible")).matched
    assert exchange.evaluate(Condition("payload_eligible", 2)).matched
    assert not exchange.evaluate(Condition("payload_eligible", 3)).matched


def test_phase_exchange_rejects_condition_before_current_run_fm2_decision() -> None:
    exchange = PhaseExchange("run", lambda *_fields: 0)
    exchange.check("FM2", "", "fm2", 0)
    exchange.accept(SimpleNamespace(
        run_id="foreign", outcome=1, APPROVED=1, REJECTED=2, ERROR=3,
        request_id="fm2", phase="FM2", checkpoint="",
        request_event_id=0, eligible_aruco_ids=[2], reason_code="APPROVED",
    ))
    with pytest.raises(RuntimeError, match="eligibility.*unavailable"):
        exchange.evaluate(Condition("any_payload_eligible"))


def test_rangefinder_rejects_stale_and_malformed_physical_samples() -> None:
    rangefinder = ConfiguredRangefinder()
    with pytest.raises(ValueError, match="exactly one"):
        rangefinder.accept(SimpleNamespace(ranges=[], range_min=0.1, range_max=20), 0)
    rangefinder.accept(
        SimpleNamespace(ranges=[4.572], range_min=0.1, range_max=20), 100
    )
    assert rangefinder.latest(500_000_100).distance_m == pytest.approx(4.572)
    with pytest.raises(RuntimeError, match="fresh"):
        rangefinder.latest(500_000_101)


def test_payload_action_requires_command_and_response_sequence_correlation() -> None:
    class Request:
        ATTACH = 1
        RELEASE = 2

    class Future:
        def __init__(self) -> None:
            self.response = None

        def done(self):
            return self.response is not None

        def result(self):
            return self.response

        def cancel(self):
            pass

    future = Future()
    client = SimpleNamespace(call_async=lambda _request: future)
    actions = PayloadActions("run", client, SimpleNamespace(Request=Request))
    actions.request("attach", 2, "attach:1", None)
    future.response = SimpleNamespace(
        command_id="attach:1", accepted=True, response_sequence=0,
        code="OK", detail="",
    )
    assert not actions.confirmation("attach:1").confirmed
    future.response.response_sequence = 1
    assert actions.confirmation("attach:1").confirmed


def test_vehicle_status_stream_samples_complete_observed_state_once_per_grid() -> None:
    stream = VehicleStatusStream(interval_ns=100_000_000)
    stream.observe(Telemetry(0, mode="GUIDED", armed=True))
    assert stream.sample(100_000_000) is None

    stream.observe(Telemetry(100_000_000, failsafe=False))
    first = stream.sample(100_000_000)
    assert first is not None
    assert (first.timestamp_ns, first.sequence_id, first.mode, first.armed) == (
        100_000_000, 0, "GUIDED", True,
    )
    assert not first.failsafe_active and first.failsafe_reason == ""
    assert stream.sample(100_000_000) is None
    assert stream.sample(199_999_999) is None

    # Publishing continues from the latest observed state without MAVLink input.
    second = stream.sample(200_000_000)
    assert second is not None
    assert (second.timestamp_ns, second.sequence_id, second.mode) == (
        200_000_000, 1, "GUIDED",
    )


def test_phase_check_waits_for_one_full_published_disarmed_interval() -> None:
    stream = VehicleStatusStream()
    events = []

    def publish(*fields):
        events.append(fields)
        return len(events) - 1

    exchange = PhaseExchange("run", publish, check_ready=stream.disarmed_ready)
    stream.observe(Telemetry(
        47_900_000_000, mode="LAND", armed=True, failsafe=False
    ))
    assert stream.sample(47_900_000_000).armed
    exchange.check("FM1", "D", "request-1", 47_950_000_000)
    assert events == []

    stream.observe(Telemetry(47_950_000_000, armed=False))
    exchange.advance(47_950_000_000)
    assert events == []
    assert stream.sample(48_000_000_000).armed is False
    exchange.advance(48_000_000_000)
    assert events == []
    assert stream.sample(48_100_000_000).armed is False
    exchange.advance(48_100_000_000)

    assert events == [(
        "FM1",
        "CHECK_REQUESTED",
        '{"checkpoint":"D","request_id":"request-1"}',
        48_100_000_000,
    )]
    exchange.advance(48_200_000_000)
    assert len(events) == 1
    exchange.accept(SimpleNamespace(
        run_id="run", outcome=1, APPROVED=1, REJECTED=2, ERROR=3,
        request_id="request-1", phase="FM1", checkpoint="D",
        request_event_id=0, eligible_aruco_ids=[], reason_code="APPROVED",
    ))
    assert exchange.decision("request-1").request_event_id == 0


def test_disarmed_readiness_requires_fresh_published_state_and_resets_on_armed() -> None:
    stream = VehicleStatusStream()
    assert not stream.disarmed_ready(100_000_000)
    stream.observe(Telemetry(100_000_000, mode="LAND", armed=False, failsafe=False))
    assert not stream.disarmed_ready(200_000_000)
    stream.sample(200_000_000)
    assert not stream.disarmed_ready(200_000_000)
    assert stream.disarmed_ready(300_000_000)
    assert not stream.disarmed_ready(700_000_001)
    stream.observe(Telemetry(700_000_001, armed=True))
    assert not stream.disarmed_ready(700_000_001)


def test_gazebo_calibration_is_precision_ready_at_configured_image_dimensions() -> None:
    calibration_path = (
        ROOT / "companion/src/drone_sim_companion/gazebo_camera_calibration.json"
    )
    mounting_path = ROOT / "companion/src/drone_sim_companion/gazebo_camera_mounting.json"
    scenario = load_vfs_scenario(ROOT / "config/scenario-2027.yaml")
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    assert calibration["dimensions_verified"] is True
    assert (
        calibration["image_width_px"], calibration["image_height_px"]
    ) == (scenario.camera.width_px, scenario.camera.height_px)
    expected_focal_px = scenario.camera.width_px / (
        2.0 * math.tan(scenario.camera.horizontal_fov_rad / 2.0)
    )
    assert calibration["camera_matrix"][0][0] == pytest.approx(expected_focal_px)
    assert calibration["camera_matrix"][0][2] == scenario.camera.width_px / 2
    assert calibration["camera_matrix"][1][2] == scenario.camera.height_px / 2

    class FrameSource:
        last_timestamp_ns = None

        def capture_frame(self, quality=4):
            assert quality == 4
            self.last_timestamp_ns = 100
            return np.zeros(
                (scenario.camera.height_px, scenario.camera.width_px, 3),
                dtype=np.uint8,
            )

    manager = CameraManager(
        frame_source=FrameSource(),
        calibration_path=calibration_path,
        clock=lambda: 100,
        max_exposure_age_ns=500_000_000,
    )
    camera = Camera(100, manager=manager, mounting_path=mounting_path)
    observation = manager.capture_observation()

    assert camera.precision_readiness(observation).ready
    assert observation.metadata.raw_image_size_px == (
        scenario.camera.width_px, scenario.camera.height_px
    )


def test_configured_vision_survives_slow_live_frame_after_request_handover() -> None:
    calibration_path = (
        ROOT / "companion/src/drone_sim_companion/gazebo_camera_calibration.json"
    )
    mounting_path = ROOT / "companion/src/drone_sim_companion/gazebo_camera_mounting.json"

    class Source:
        def __init__(self):
            self.delay_s = 0.02
            self.last_timestamp_ns = None
            self.calls = 0
            self.lock = threading.Lock()

        def capture_frame(self, quality=4):
            assert quality == 4
            with self.lock:
                delay = self.delay_s
            time.sleep(delay)
            with self.lock:
                self.calls += 1
                self.last_timestamp_ns = self.calls * 50_000_000
            return np.zeros((480, 640, 3), dtype=np.uint8)

    class MarkerManager(CameraManager):
        def get_coords(self, _frame):
            return [object()], np.array([[2]]), []

        def estimate_pose_3d(self, *_args, **_kwargs):
            return np.array([0.0, 0.0, 4_572.0])

    source = Source()
    manager = MarkerManager(
        frame_source=source,
        calibration_path=calibration_path,
        clock=lambda: source.last_timestamp_ns or 0,
        max_exposure_age_ns=500_000_000,
    )
    vision = ConfiguredVision(
        Camera(100, manager=manager, mounting_path=mounting_path)
    )
    vision.request(2, 0)
    detections = []
    deadline = time.monotonic() + 2.0
    while len(detections) < 5 and time.monotonic() < deadline:
        observation = vision.poll()
        if observation is not None:
            detections.append(observation)
        time.sleep(0.005)
    assert len(detections) == 5

    with source.lock:
        source.delay_s = 0.31
    time.sleep(0.05)
    after_sequence = manager._frame_sequence
    while vision.poll() is not None:
        pass
    vision.request(2, after_sequence)
    resumed = None
    deadline = time.monotonic() + 1.2
    while resumed is None and time.monotonic() < deadline:
        resumed = vision.poll()
        time.sleep(0.005)
    vision.close()

    assert resumed is not None
    assert resumed.sequence > after_sequence


def test_configured_vision_skips_cached_stale_frame_at_new_marker_request() -> None:
    calibration_path = (
        ROOT / "companion/src/drone_sim_companion/gazebo_camera_calibration.json"
    )
    mounting_path = ROOT / "companion/src/drone_sim_companion/gazebo_camera_mounting.json"
    clock = [0]

    class Source:
        def __init__(self):
            self.timestamps = deque([0, 1_000_000_000])
            self.last_timestamp_ns = None

        def capture_frame(self, quality=4):
            assert quality == 4
            timestamp = self.timestamps.popleft()
            if timestamp:
                time.sleep(0.05)
            self.last_timestamp_ns = timestamp
            return np.zeros((480, 640, 3), dtype=np.uint8)

    class MarkerManager(CameraManager):
        def get_coords(self, _frame):
            return [object()], np.array([[4]]), []

        def estimate_pose_3d(self, *_args, **_kwargs):
            return np.array([0.0, 0.0, 4_572.0])

    source = Source()
    manager = MarkerManager(
        frame_source=source,
        calibration_path=calibration_path,
        clock=lambda: clock[0],
        max_exposure_age_ns=500_000_000,
    )
    manager.capture_observation()
    assert manager.last_frame_metadata.sequence == 1
    clock[0] = 1_000_000_000
    vision = ConfiguredVision(
        Camera(100, manager=manager, mounting_path=mounting_path)
    )

    vision.request(4, 0)
    result = None
    deadline = time.monotonic() + 1.0
    while result is None and time.monotonic() < deadline:
        result = vision.poll()
        time.sleep(0.005)
    vision.close()

    assert result is not None
    assert result.aruco_id == 4
    assert result.sequence == 2
    assert result.exposure_timestamp_ns == 1_000_000_000
