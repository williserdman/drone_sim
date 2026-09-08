from __future__ import annotations

import json
import threading
import time

import numpy as np
import pytest

from drone.sensors.camera import _camera_manager as camera_manager_module
from drone.sensors.camera._camera_manager import CameraManager
from drone.sensors.camera.camera import Camera


class FakeClock:
    def __init__(self, now_ns):
        self.now_ns = now_ns

    def __call__(self):
        return self.now_ns


class ScriptedClock:
    def __init__(self, values):
        self.values = iter(values)

    def __call__(self):
        return next(self.values)


class TimestampedSource:
    def __init__(self, timestamps, shape=(480, 640, 3)):
        self._timestamps = iter(timestamps)
        self.last_timestamp_ns = None
        self.frame = np.zeros(shape, dtype=np.uint8)

    def capture_frame(self, quality=4):
        assert quality == 4
        self.last_timestamp_ns = next(self._timestamps)
        return self.frame


def write_calibration(path, *, width=None, height=None, verified=False):
    path.write_text(
        json.dumps(
            {
                "camera_matrix": [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
                "dist_coeff": [[0.0, 0.0, 0.0, 0.0, 0.0]],
                "image_width_px": width,
                "image_height_px": height,
                "dimensions_verified": verified,
            }
        )
    )


def write_mounting(path, *, verified):
    path.write_text(
        json.dumps(
            {
                "camera_to_body_frd": [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
                "down_offset_m": 0.1,
                "verified": verified,
            }
        )
    )


def test_capture_exposes_sequence_receipt_and_real_exposure_metadata(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    clock = FakeClock(1_050)
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=clock,
        max_exposure_age_ns=100,
    )

    frame = manager.capture_frame()

    assert frame.shape == (480, 640, 3)
    assert manager.last_frame_timestamp == 1_000
    assert manager.last_frame_metadata is not None
    assert manager.last_frame_metadata.sequence == 1
    assert manager.last_frame_metadata.exposure_timestamp_ns == 1_000
    assert manager.last_frame_metadata.receipt_timestamp_ns == 1_050
    assert manager.last_frame_metadata.exposure_age_ns == 50
    assert manager.last_frame_metadata.exposure_age_bounded is True


def test_direct_observation_rechecks_age_at_final_handover(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=ScriptedClock([1_050, 1_101]),
        max_exposure_age_ns=100,
    )

    with pytest.raises(RuntimeError, match="older than"):
        manager.capture_observation()

    assert manager.last_frame_metadata is None


def test_direct_handover_resamples_age_after_publication_lock_contention(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)

    class ContendedClock:
        def __init__(self):
            self.now_ns = 1_050
            self.calls = 0
            self.final_sampled = threading.Event()

        def __call__(self):
            self.calls += 1
            if self.calls == 2:
                self.final_sampled.set()
            return self.now_ns

    clock = ContendedClock()
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=clock,
        max_exposure_age_ns=100,
    )
    results = []

    def capture_observation():
        try:
            results.append(manager.capture_observation())
        except Exception as error:
            results.append(error)

    manager._state_condition.acquire()
    try:
        capture = threading.Thread(target=capture_observation)
        capture.start()
        assert clock.final_sampled.wait(0.2)
        clock.now_ns = 1_101
    finally:
        manager._state_condition.release()
    capture.join(1.0)

    assert len(results) == 1
    assert isinstance(results[0], RuntimeError)
    assert "older than" in str(results[0])
    assert manager.last_frame_metadata is None


@pytest.mark.parametrize("timestamps", [[1_000, 1_000], [1_000, 999]])
def test_replayed_or_out_of_order_exposure_is_rejected(tmp_path, timestamps):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    manager = CameraManager(
        frame_source=TimestampedSource(timestamps),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    manager.capture_frame()

    with pytest.raises(RuntimeError, match="newer"):
        manager.capture_frame()

    assert manager.last_frame_metadata.sequence == 1


def test_stale_exposure_is_rejected_instead_of_restamped(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    manager = CameraManager(
        frame_source=TimestampedSource([800]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )

    with pytest.raises(RuntimeError, match="older than"):
        manager.capture_frame()

    assert manager.last_frame_timestamp is None
    assert manager.last_frame_metadata is None


def test_future_exposure_timestamp_is_rejected(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    manager = CameraManager(
        frame_source=TimestampedSource([1_100]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )

    with pytest.raises(RuntimeError, match="future"):
        manager.capture_frame()


def test_unknown_calibration_dimensions_keep_precision_readiness_closed(tmp_path):
    calibration = tmp_path / "calibration.json"
    mounting = tmp_path / "mounting.json"
    write_calibration(calibration)
    write_mounting(mounting, verified=True)
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    camera = Camera(100, manager=manager, mounting_path=mounting)
    manager.capture_frame()

    readiness = camera.precision_readiness()

    assert readiness.ready is False
    assert "calibrated image dimensions are unknown or unverified" in readiness.reasons
    assert manager._latest_observation.camera_matrix is None


def test_unverified_mounting_keeps_precision_readiness_closed(tmp_path):
    calibration = tmp_path / "calibration.json"
    mounting = tmp_path / "mounting.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    write_mounting(mounting, verified=False)
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    camera = Camera(100, manager=manager, mounting_path=mounting)
    manager.capture_frame()

    readiness = camera.precision_readiness()

    assert readiness.ready is False
    assert "camera mounting is unverified" in readiness.reasons


def test_matching_dimensions_verified_mounting_and_bounded_age_are_ready(tmp_path):
    calibration = tmp_path / "calibration.json"
    mounting = tmp_path / "mounting.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    write_mounting(mounting, verified=True)
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    camera = Camera(100, manager=manager, mounting_path=mounting)
    manager.capture_frame()

    readiness = camera.precision_readiness()

    assert readiness.ready is True
    assert readiness.reasons == ()


def test_readiness_rechecks_exposure_age_at_use_time(tmp_path):
    calibration = tmp_path / "calibration.json"
    mounting = tmp_path / "mounting.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    write_mounting(mounting, verified=True)
    clock = FakeClock(1_050)
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=clock,
        max_exposure_age_ns=100,
    )
    camera = Camera(100, manager=manager, mounting_path=mounting)
    manager.capture_frame()
    clock.now_ns = 1_101

    readiness = camera.precision_readiness()

    assert readiness.ready is False
    assert "latest frame exposure is older than the acquisition limit" in readiness.reasons


def test_mismatched_capture_dimensions_keep_readiness_closed(tmp_path):
    calibration = tmp_path / "calibration.json"
    mounting = tmp_path / "mounting.json"
    write_calibration(calibration, width=1280, height=720, verified=True)
    write_mounting(mounting, verified=True)
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    camera = Camera(100, manager=manager, mounting_path=mounting)
    manager.capture_frame()

    readiness = camera.precision_readiness()

    assert readiness.ready is False
    assert "captured image dimensions do not match calibration" in readiness.reasons

    with pytest.raises(RuntimeError, match="dimensions do not match"):
        camera.vector_from_observation_3d(manager._latest_observation, 3)


def test_invalid_calibration_is_rejected_before_hardware_open(monkeypatch, tmp_path):
    calibration = tmp_path / "calibration.json"
    calibration.write_text(json.dumps({"camera_matrix": [[float("nan")]], "dist_coeff": []}))
    monkeypatch.setattr(
        "drone.sensors.camera._camera_manager.cv2.VideoCapture",
        lambda index: pytest.fail(f"opened hardware camera index {index}"),
    )

    with pytest.raises(ValueError, match="camera_matrix"):
        CameraManager(calibration_path=calibration)


def test_hardware_read_completion_is_receipt_not_exposure(monkeypatch, tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)

    class FakeCapture:
        def read(self):
            return True, np.zeros((480, 640, 3), dtype=np.uint8)

    monkeypatch.setattr(
        "drone.sensors.camera._camera_manager.cv2.VideoCapture", lambda index: FakeCapture()
    )
    manager = CameraManager(
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )

    manager.capture_frame()

    assert manager.last_frame_timestamp is None
    assert manager.last_frame_metadata.exposure_timestamp_ns is None
    assert manager.last_frame_metadata.receipt_timestamp_ns == 1_050
    assert manager.last_frame_metadata.exposure_age_bounded is False


def test_bounded_capture_times_out_without_later_publishing_delayed_frame(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)

    class BlockingSource:
        last_timestamp_ns = 1_000

        def __init__(self):
            self.entered = threading.Event()
            self.release = threading.Event()

        def capture_frame(self, quality=4):
            self.entered.set()
            assert self.release.wait(1.0)
            return np.zeros((480, 640, 3), dtype=np.uint8)

    source = BlockingSource()
    manager = CameraManager(
        frame_source=source,
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )

    with pytest.raises(TimeoutError, match="timed out"):
        manager.capture_frame_bounded(timeout_s=0.01)

    assert source.entered.is_set()
    source.release.set()
    time.sleep(0.02)
    assert manager.last_frame_metadata is None


def test_repeated_timeouts_leave_only_one_owned_capture_running(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)

    class BlockingSource:
        last_timestamp_ns = 1_000

        def __init__(self):
            self.calls = 0
            self.entered = threading.Event()
            self.release_read = threading.Event()

        def capture_frame(self, quality=4):
            self.calls += 1
            self.entered.set()
            assert self.release_read.wait(1.0)
            return np.zeros((480, 640, 3), dtype=np.uint8)

    source = BlockingSource()
    manager = CameraManager(
        frame_source=source,
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )

    with pytest.raises(TimeoutError):
        manager.capture_observation_bounded(timeout_s=0.01)
    with pytest.raises(TimeoutError):
        manager.capture_observation_bounded(timeout_s=0.01)

    assert source.calls == 1
    assert manager.stop_acquisition(timeout_s=0.01) is False
    source.release_read.set()
    assert manager.stop_acquisition(timeout_s=1.0) is True
    assert source.calls == 1


def test_concurrent_capture_returns_atomic_frame_metadata_and_intrinsics(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)

    class Source:
        def __init__(self):
            self.value = 0
            self.last_timestamp_ns = None

        def capture_frame(self, quality=4):
            self.value += 1
            self.last_timestamp_ns = 1_000 + self.value
            return np.full((480, 640, 3), self.value, dtype=np.uint8)

    manager = CameraManager(
        frame_source=Source(),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    observations = []
    threads = [threading.Thread(target=lambda: observations.append(manager.capture_observation())) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(1.0)

    assert sorted(
        (observation.metadata.sequence, observation.metadata.exposure_timestamp_ns, int(observation.frame[0, 0, 0]))
        for observation in observations
    ) == [(1, 1_001, 1), (2, 1_002, 2)]
    assert all(observation.camera_matrix[2].tolist() == [0.0, 0.0, 1.0] for observation in observations)


def test_observation_owns_frame_when_source_reuses_its_buffer(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)

    class ReusingSource:
        def __init__(self):
            self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
            self.last_timestamp_ns = 999

        def capture_frame(self, quality=4):
            self.last_timestamp_ns += 1
            self.frame.fill(self.last_timestamp_ns - 999)
            return self.frame

    manager = CameraManager(
        frame_source=ReusingSource(),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )

    first = manager.capture_observation()
    manager.capture_observation()

    assert int(first.frame[0, 0, 0]) == 1


def test_supported_resize_attaches_scaled_intrinsics(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)

    class ResizeSource(TimestampedSource):
        def capture_frame(self, quality=4):
            assert quality == 3
            self.last_timestamp_ns = next(self._timestamps)
            return self.frame

    manager = CameraManager(
        frame_source=ResizeSource([1_000]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )

    observation = manager.capture_observation(quality=3)

    assert observation.frame.shape == (120, 159, 3)
    assert observation.metadata.raw_image_size_px == (640, 480)
    assert observation.metadata.image_size_px == (159, 120)
    np.testing.assert_allclose(
        observation.camera_matrix,
        [[24.84375, 0.0, 12.421875], [0.0, 25.0, 10.0], [0.0, 0.0, 1.0]],
    )


def test_unsupported_quality_transform_is_rejected_before_capture(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    source = TimestampedSource([1_000])
    manager = CameraManager(frame_source=source, calibration_path=calibration)

    with pytest.raises(ValueError, match="quality"):
        manager.capture_observation(quality=5)

    assert source.last_timestamp_ns is None


@pytest.mark.parametrize(
    "overrides",
    [
        {"dist_coeff": [[0.0, 0.0, 0.0]]},
        {"dist_coeff": [[0.0, 0.0], [0.0, 0.0]]},
        {"camera_matrix": [[100.0, 1.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]]},
        {"camera_matrix": [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 1.0, 1.0]]},
    ],
)
def test_unsupported_opencv_calibration_is_rejected_before_hardware(
    monkeypatch, tmp_path, overrides
):
    calibration = tmp_path / "calibration.json"
    values = {
        "camera_matrix": [[100.0, 0.0, 50.0], [0.0, 100.0, 40.0], [0.0, 0.0, 1.0]],
        "dist_coeff": [[0.0, 0.0, 0.0, 0.0, 0.0]],
        "dimensions_verified": False,
    }
    values.update(overrides)
    calibration.write_text(json.dumps(values))
    monkeypatch.setattr(
        "drone.sensors.camera._camera_manager.cv2.VideoCapture",
        lambda index: pytest.fail(f"opened hardware camera index {index}"),
    )

    with pytest.raises(ValueError, match="camera_matrix|dist_coeff"):
        CameraManager(calibration_path=calibration)


def test_latest_observation_rechecks_age_at_handover(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    clock = FakeClock(1_050)
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=clock,
        max_exposure_age_ns=100,
    )
    observation = manager.capture_observation()
    manager._latest_observation = observation
    clock.now_ns = 1_101

    with pytest.raises(RuntimeError, match="older than"):
        manager.latest_observation(after_sequence=0, timeout_s=0.01)


def test_latest_observation_rejects_candidate_available_after_deadline(
    monkeypatch, tmp_path
):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    observation = manager.capture_observation()
    manager._latest_observation = None
    wall_clock = FakeClock(1_000_000_000)

    class PublishAfterDeadline:
        def __enter__(self):
            wall_clock.now_ns += 20_000_000
            manager._latest_observation = observation
            return self

        def __exit__(self, _type, _value, _traceback):
            return False

    manager._state_condition = PublishAfterDeadline()
    monkeypatch.setattr(
        camera_manager_module.time,
        "monotonic",
        lambda: wall_clock() / 1_000_000_000,
    )

    with pytest.raises(TimeoutError, match="timed out"):
        manager.latest_observation(after_sequence=0, timeout_s=0.01)


def test_latest_observation_rejects_candidate_when_freshness_exceeds_deadline(
    monkeypatch, tmp_path
):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    manager = CameraManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    manager.capture_observation()
    wall_clock = FakeClock(1_000_000_000)

    def freshness_clock():
        wall_clock.now_ns += 20_000_000
        return 1_050

    manager._clock = freshness_clock
    monkeypatch.setattr(
        camera_manager_module.time,
        "monotonic",
        lambda: wall_clock() / 1_000_000_000,
    )

    with pytest.raises(TimeoutError, match="timed out"):
        manager.latest_observation(after_sequence=0, timeout_s=0.01)


def test_camera_processes_existing_observation_without_recapture(tmp_path):
    calibration = tmp_path / "calibration.json"
    mounting = tmp_path / "mounting.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    write_mounting(mounting, verified=True)

    class PoseManager(CameraManager):
        def get_coords(self, frame):
            return [object()], np.array([[3]]), []

        def estimate_pose_3d(self, target_id, corners, ids, marker_size_mm, *, camera_matrix=None, distortion=None):
            assert camera_matrix is not None
            assert distortion is not None
            return np.array([200.0, -300.0, 4_000.0])

    source = TimestampedSource([1_000])
    manager = PoseManager(
        frame_source=source,
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    camera = Camera(100, manager=manager, mounting_path=mounting)
    observation = manager.capture_observation()

    vector = camera.vector_from_observation_3d(observation, 3)

    assert (vector.x, vector.y, vector.z) == pytest.approx((0.3, 0.2, 4.1))
    assert source.last_timestamp_ns == 1_000


def test_vector_rechecks_age_after_pose_processing(tmp_path):
    calibration = tmp_path / "calibration.json"
    mounting = tmp_path / "mounting.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    write_mounting(mounting, verified=True)
    clock = FakeClock(1_050)

    class AgingPoseManager(CameraManager):
        def get_coords(self, frame):
            return [object()], np.array([[3]]), []

        def estimate_pose_3d(
            self,
            target_id,
            corners,
            ids,
            marker_size_mm,
            *,
            camera_matrix=None,
            distortion=None,
        ):
            clock.now_ns = 1_101
            return np.array([200.0, -300.0, 4_000.0])

    manager = AgingPoseManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=clock,
        max_exposure_age_ns=100,
    )
    camera = Camera(100, manager=manager, mounting_path=mounting)
    observation = manager.capture_observation()

    with pytest.raises(RuntimeError, match="older than"):
        camera.vector_from_observation_3d(observation, 3)


def test_timeout_cancellation_wins_final_observation_publication(tmp_path):
    calibration = tmp_path / "calibration.json"
    write_calibration(calibration, width=640, height=480, verified=True)

    class PausedManager(CameraManager):
        def __init__(self, *args, **kwargs):
            self.publication_reached = threading.Event()
            self.release_publication = threading.Event()
            super().__init__(*args, **kwargs)

        def _accept_candidate(self, candidate, *args, **kwargs):
            self.publication_reached.set()
            assert self.release_publication.wait(1.0)
            return super()._accept_candidate(candidate, *args, **kwargs)

    manager = PausedManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    results = []

    def consume():
        try:
            manager.capture_observation_bounded(timeout_s=0.02)
        except Exception as error:
            results.append(error)

    consumer = threading.Thread(target=consume)
    consumer.start()
    assert manager.publication_reached.wait(0.2)
    consumer.join(0.2)
    assert isinstance(results[0], TimeoutError)

    manager.release_publication.set()
    assert manager.stop_acquisition(timeout_s=1.0) is True
    assert manager.last_frame_metadata is None


def test_camera_bounded_vector_returns_the_consumed_sequence(tmp_path):
    calibration = tmp_path / "calibration.json"
    mounting = tmp_path / "mounting.json"
    write_calibration(calibration, width=640, height=480, verified=True)
    write_mounting(mounting, verified=True)

    class PoseManager(CameraManager):
        def get_coords(self, frame):
            return [object()], np.array([[3]]), []

        def estimate_pose_3d(
            self,
            target_id,
            corners,
            ids,
            marker_size_mm,
            *,
            camera_matrix=None,
            distortion=None,
        ):
            return np.array([200.0, -300.0, 4_000.0])

    manager = PoseManager(
        frame_source=TimestampedSource([1_000]),
        calibration_path=calibration,
        clock=FakeClock(1_050),
        max_exposure_age_ns=100,
    )
    camera = Camera(100, manager=manager, mounting_path=mounting)

    vector, metadata = camera.vec_to_marker_3d_bounded(
        3, timeout_s=1.0, after_sequence=0
    )

    assert (vector.x, vector.y, vector.z) == pytest.approx((0.3, 0.2, 4.1))
    assert metadata.sequence == 1
    assert metadata.exposure_timestamp_ns == 1_000
    assert manager.stop_acquisition(timeout_s=1.0) is True


def test_missing_manager_readiness_contract_fails_closed(tmp_path):
    mounting = tmp_path / "mounting.json"
    write_mounting(mounting, verified=True)

    readiness = Camera(100, manager=object(), mounting_path=mounting).precision_readiness()

    assert readiness.ready is False
    assert "manager does not expose precision readiness" in readiness.reasons
