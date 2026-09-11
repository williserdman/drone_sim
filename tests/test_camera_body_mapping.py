from __future__ import annotations

import numpy as np
import pytest

from drone.sensors.camera.camera import Camera


class FixedMarkerPose:
    """Return one literal OpenCV camera-frame marker translation."""

    last_frame_timestamp = 170_000_000_000

    def capture_frame(self, quality=4, deadline_sim_ns=None):
        assert quality == 4
        assert deadline_sim_ns is None
        return np.zeros((2, 2, 3), dtype=np.uint8)

    def get_coords(self, frame):
        assert frame.shape == (2, 2, 3)
        return [object()], np.array([[3]]), []

    def estimate_pose_3d(self, target_id, corners, ids, marker_size_mm):
        assert (target_id, marker_size_mm) == (3, 100)
        return np.array([200.0, -300.0, 4_000.0])


def test_downward_camera_pose_maps_image_top_to_forward_and_right_to_right():
    """Regression: swapping image axes drives marker correction away from truth."""
    update = Camera(100, manager=FixedMarkerPose()).vec_to_marker_3d(3)

    assert update is not None
    assert (update.x, update.y, update.z) == pytest.approx((0.3, 0.2, 4.1))


def test_marker_observation_keeps_vector_and_source_timestamp_together():
    observation = Camera(100, manager=FixedMarkerPose()).observe_marker_3d(3)

    assert observation is not None
    assert observation.frame_timestamp_ns == 170_000_000_000
    assert observation.vector is not None
    assert (observation.vector.x, observation.vector.y, observation.vector.z) == pytest.approx(
        (0.3, 0.2, 4.1)
    )


class NoMarkerFrame(FixedMarkerPose):
    last_frame_timestamp = 171_000_000_000

    def get_coords(self, frame):
        return [], None, []


def test_marker_observation_keeps_timestamp_when_target_is_absent():
    observation = Camera(100, manager=NoMarkerFrame()).observe_marker_3d(3)

    assert observation is not None
    assert observation.frame_timestamp_ns == 171_000_000_000
    assert observation.vector is None


class DeadlineRecordingMarkerPose(FixedMarkerPose):
    def __init__(self):
        self.deadlines = []

    def capture_frame(self, quality=4, deadline_sim_ns=None):
        self.deadlines.append(deadline_sim_ns)
        return np.zeros((2, 2, 3), dtype=np.uint8)


def test_marker_observation_propagates_capture_deadline():
    manager = DeadlineRecordingMarkerPose()

    Camera(100, manager=manager).observe_marker_3d(3, deadline_sim_ns=125)

    assert manager.deadlines == [125]
