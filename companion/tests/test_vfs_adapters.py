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
from drone_sim_companion.comp2026_host import RosFrameSource
from drone_sim_companion.comp2026_host import StaleSensorError
from drone_sim_companion.configured_runtime import _configure_vfs_resources
from drone_sim_companion.mission_plan import Condition
from drone_sim_companion.mission import Telemetry
from drone_sim_companion.configured_vision import ConfiguredVision
from drone_sim_companion.vfs_operations import (
    ConfiguredRangefinder,
    PayloadActions,
    PhaseExchange,
    VehicleStatusStream,
)


def rgb_image(timestamp_ns: int, *, red: int = 10) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=timestamp_ns // 1_000_000_000,
                nanosec=timestamp_ns % 1_000_000_000,
            ),
            frame_id="camera/onboard",
        ),
        width=640,
        height=480,
        encoding="rgb8",
        is_bigendian=False,
        step=640 * 3,
        data=bytes((red, 20, 30)) * (640 * 480),
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


def test_configured_camera_consumes_freshest_queued_ros_image() -> None:
    clock_ns = 344_150_000_000
    source = RosFrameSource(
        width_px=640,
        height_px=480,
        clock_ns=lambda: clock_ns,
        max_exposure_age_ns=500_000_000,
    )
    for timestamp_ns, red in (
        (343_550_000_000, 10),
        (343_600_000_000, 20),
        (343_650_000_000, 30),
        (343_700_000_000, 40),
    ):
        source.accept_image(rgb_image(timestamp_ns, red=red))
    manager = CameraManager(
        frame_source=source,
        calibration_path=(
            ROOT
            / "companion/src/drone_sim_companion/gazebo_camera_calibration.json"
        ),
        clock=lambda: clock_ns,
        max_exposure_age_ns=500_000_000,
    )

    observation = manager.capture_observation()

    assert observation.metadata.exposure_timestamp_ns == 343_700_000_000
    assert observation.metadata.exposure_age_ns == 450_000_000
    assert observation.frame[0, 0].tolist() == [30, 20, 40]


def test_configured_camera_waits_for_fresh_image_after_stale_queue() -> None:
    clock_ns = 344_150_000_000
    source = RosFrameSource(
        width_px=640,
        height_px=480,
        clock_ns=lambda: clock_ns,
        max_exposure_age_ns=500_000_000,
    )
    for timestamp_ns in (
        343_450_000_000,
        343_500_000_000,
        343_550_000_000,
        343_600_000_000,
    ):
        source.accept_image(rgb_image(timestamp_ns))
    manager = CameraManager(
        frame_source=source,
        calibration_path=(
            ROOT
            / "companion/src/drone_sim_companion/gazebo_camera_calibration.json"
        ),
        clock=lambda: clock_ns,
        max_exposure_age_ns=500_000_000,
    )
    results: list[object] = []

    def capture() -> None:
        try:
            results.append(manager.capture_observation())
        except BaseException as error:
            results.append(error)

    worker = threading.Thread(target=capture)
    worker.start()
    deadline = time.monotonic() + 1.0
    while source.ready and worker.is_alive() and time.monotonic() < deadline:
        time.sleep(0.005)

    assert worker.is_alive()
    assert results == []

    source.accept_image(rgb_image(clock_ns, red=40))
    worker.join(1.0)

    assert len(results) == 1
    assert not isinstance(results[0], BaseException)
    assert results[0].metadata.exposure_timestamp_ns == clock_ns


def test_configured_camera_rechecks_freshness_after_pixel_copy() -> None:
    clock_ns = [1_050]
    copied = threading.Event()

    class AgingPixels:
        def __bytes__(self) -> bytes:
            clock_ns[0] = 1_101
            copied.set()
            return bytes((10, 20, 30)) * (640 * 480)

    source = RosFrameSource(
        width_px=640,
        height_px=480,
        clock_ns=lambda: clock_ns[0],
        max_exposure_age_ns=100,
    )
    aging_image = rgb_image(1_000)
    aging_image.data = AgingPixels()
    source.accept_image(aging_image)
    manager = CameraManager(
        frame_source=source,
        calibration_path=(
            ROOT
            / "companion/src/drone_sim_companion/gazebo_camera_calibration.json"
        ),
        clock=lambda: clock_ns[0],
        max_exposure_age_ns=100,
    )
    results: list[object] = []

    def capture() -> None:
        try:
            results.append(manager.capture_observation())
        except BaseException as error:
            results.append(error)

    worker = threading.Thread(target=capture)
    worker.start()
    assert copied.wait(1.0)
    time.sleep(0.01)

    assert worker.is_alive()
    assert source.last_timestamp_ns is None

    source.accept_image(rgb_image(1_101, red=40))
    worker.join(1.0)

    assert len(results) == 1
    assert not isinstance(results[0], BaseException)
    assert results[0].metadata.exposure_timestamp_ns == 1_101


def test_vfs_composition_enables_freshest_frame_policy(monkeypatch) -> None:
    monkeypatch.setitem(
        sys.modules,
        "sensor_msgs.msg",
        SimpleNamespace(Image=object, LaserScan=object),
    )
    monkeypatch.setitem(
        sys.modules,
        "simulation_interfaces.msg",
        SimpleNamespace(
            AttemptStatus=object,
            JudgeDecision=object,
            MissionEvent=object,
            VehicleStatus=object,
        ),
    )
    monkeypatch.setitem(
        sys.modules,
        "simulation_interfaces.srv",
        SimpleNamespace(PayloadCommand=object),
    )

    class Node:
        def create_client(self, *_args):
            return object()

        def create_publisher(self, *_args):
            return SimpleNamespace(publish=lambda _message: None)

        def create_subscription(self, *_args):
            return object()

    clock_ns = 344_150_000_000
    resources = _configure_vfs_resources(
        SimpleNamespace(
            course_path=ROOT / "config/course-2027.yaml",
            scenario_path=ROOT / "config/scenario-2027.yaml",
            run_id="run",
        ),
        Node(),
        qos_profile=lambda **values: SimpleNamespace(**values),
        reliability=SimpleNamespace(RELIABLE=1),
        durability=SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2),
        clock_ns=lambda: clock_ns,
    )
    for timestamp_ns in (
        343_550_000_000,
        343_600_000_000,
        343_650_000_000,
        343_700_000_000,
    ):
        resources.frame_source.accept_image(rgb_image(timestamp_ns))

    resources.frame_source.capture_frame()

    assert resources.frame_source.last_timestamp_ns == 343_700_000_000


def test_frame_source_default_keeps_legacy_fifo_consumption() -> None:
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(rgb_image(100, red=10))
    source.accept_image(rgb_image(200, red=40))

    frame = source.capture_frame()

    assert source.last_timestamp_ns == 100
    assert frame[0, 0].tolist() == [30, 20, 10]


def test_configured_camera_accepts_frame_at_exact_age_limit() -> None:
    source = RosFrameSource(
        width_px=640,
        height_px=480,
        clock_ns=lambda: 1_100,
        max_exposure_age_ns=100,
    )
    source.accept_image(rgb_image(1_000))
    manager = CameraManager(
        frame_source=source,
        calibration_path=(
            ROOT
            / "companion/src/drone_sim_companion/gazebo_camera_calibration.json"
        ),
        clock=lambda: 1_100,
        max_exposure_age_ns=100,
    )

    observation = manager.capture_observation()

    assert observation.metadata.exposure_age_ns == 100
    assert observation.metadata.exposure_age_bounded is True


def test_configured_camera_still_rejects_future_exposure() -> None:
    source = RosFrameSource(
        width_px=640,
        height_px=480,
        clock_ns=lambda: 1_050,
        max_exposure_age_ns=100,
    )
    source.accept_image(rgb_image(1_100))
    manager = CameraManager(
        frame_source=source,
        calibration_path=(
            ROOT
            / "companion/src/drone_sim_companion/gazebo_camera_calibration.json"
        ),
        clock=lambda: 1_050,
        max_exposure_age_ns=100,
    )

    with pytest.raises(RuntimeError, match="future"):
        manager.capture_observation()


def test_configured_frame_source_still_rejects_malformed_pixels() -> None:
    source = RosFrameSource(
        width_px=640,
        height_px=480,
        clock_ns=lambda: 1_050,
        max_exposure_age_ns=100,
    )
    malformed = rgb_image(1_000)
    malformed.data = b"short"
    source.accept_image(malformed)

    with pytest.raises(ValueError, match="data length"):
        source.capture_frame()


def test_configured_frame_source_stop_wakes_fresh_frame_waiter() -> None:
    source = RosFrameSource(
        width_px=640,
        height_px=480,
        clock_ns=lambda: 1_050,
        max_exposure_age_ns=100,
    )
    results: list[BaseException] = []

    def capture() -> None:
        try:
            source.capture_frame()
        except BaseException as error:
            results.append(error)

    worker = threading.Thread(target=capture)
    worker.start()
    time.sleep(0.01)

    source.stop("test shutdown")
    worker.join(1.0)

    assert len(results) == 1
    assert isinstance(results[0], StaleSensorError)
    assert "test shutdown" in str(results[0])


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


def test_configured_vision_reports_camera_worker_root_cause() -> None:
    class FailedCamera:
        cm = SimpleNamespace(last_frame_metadata=None)

        def vec_to_marker_3d_bounded(self, *_args, **_kwargs):
            try:
                raise ValueError("Frame exposure timestamp is in the future")
            except ValueError as cause:
                raise RuntimeError("Camera acquisition worker failed") from cause

    vision = ConfiguredVision(FailedCamera())
    vision.request(4, 0)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        try:
            vision.poll()
        except RuntimeError as error:
            assert str(error) == (
                "configured vision failed: Camera acquisition worker failed; "
                "caused by ValueError: Frame exposure timestamp is in the future"
            )
            assert isinstance(error.__cause__, RuntimeError)
            return
        time.sleep(0.005)
    pytest.fail("configured vision did not report the camera worker failure")


def test_new_vision_request_discards_cancelled_worker_failure() -> None:
    entered = threading.Event()
    release = threading.Event()
    old_done = threading.Event()

    class Camera:
        cm = SimpleNamespace(last_frame_metadata=None)

        def vec_to_marker_3d_bounded(self, aruco_id, **_kwargs):
            if aruco_id == 2:
                entered.set()
                assert release.wait(1.0)
                old_done.set()
                raise RuntimeError("cancelled request failed late")
            assert old_done.wait(1.0)
            return (
                SimpleNamespace(x=0.0, y=0.0, z=4.0),
                SimpleNamespace(
                    sequence=2, exposure_timestamp_ns=100,
                    receipt_timestamp_ns=100,
                ),
            )

    vision = ConfiguredVision(Camera())
    vision.request(2, 0)
    assert entered.wait(1.0)
    vision.request(4, 0)
    release.set()
    result = None
    deadline = time.monotonic() + 1.0
    while result is None and time.monotonic() < deadline:
        result = vision.poll()
        time.sleep(0.005)
    vision.close()

    assert result is not None
    assert result.aruco_id == 4
