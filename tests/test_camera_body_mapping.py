from __future__ import annotations

import json

import numpy as np
import pytest

from drone.sensors.camera.camera import Camera


class FixedMarkerPose:
    """Return one literal OpenCV camera-frame marker translation."""

    last_frame_timestamp = 170_000_000_000

    def capture_frame(self, quality=4):
        assert quality == 4
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


def test_custom_mounting_config_controls_body_mapping_and_down_offset(tmp_path):
    mounting_path = tmp_path / "mounting.json"
    mounting_path.write_text(
        json.dumps(
            {
                "camera_to_body_frd": [
                    [1, 0, 0],
                    [0, 0, -1],
                    [0, 1, 0],
                ],
                "down_offset_m": 0.25,
                "verified": True,
            }
        )
    )

    update = Camera(
        100,
        manager=FixedMarkerPose(),
        mounting_path=mounting_path,
    ).vec_to_marker_3d(3, lidar_alt=5.0)

    assert update is not None
    assert (update.x, update.y, update.z) == pytest.approx((0.2, -4.0, 5.25))


def test_any_marker_pose_uses_same_mounting_mapping():
    update = Camera(100, manager=FixedMarkerPose()).vec_to_any_marker_3d()

    assert update is not None
    direction, marker_id = update
    assert (direction.x, direction.y, direction.z) == pytest.approx(
        (0.3, 0.2, 4.1)
    )
    assert marker_id == 3
    assert isinstance(marker_id, int)


def test_mounting_config_rejects_non_three_by_three_transform(tmp_path):
    mounting_path = tmp_path / "mounting.json"
    mounting_path.write_text(
        json.dumps(
            {
                "camera_to_body_frd": [[1, 0], [0, 1]],
                "down_offset_m": 0.1,
            }
        )
    )

    with pytest.raises(ValueError, match="3x3"):
        Camera(100, manager=FixedMarkerPose(), mounting_path=mounting_path)


@pytest.mark.parametrize(
    ("rotation", "message"),
    [
        ([[1, 0, 0], [0, -1, 0], [0, 0, 1]], "right-handed"),
        ([[1, 0, 0], [0, 2, 0], [0, 0, 1]], "orthonormal"),
        ([[1, 0, 0], [0, 1, 0], [0, 0, float("nan")]], "finite"),
    ],
)
def test_mounting_config_rejects_unsafe_rotations(tmp_path, rotation, message):
    mounting_path = tmp_path / "mounting.json"
    mounting_path.write_text(
        json.dumps(
            {
                "camera_to_body_frd": rotation,
                "down_offset_m": 0.1,
                "verified": True,
            }
        )
    )

    with pytest.raises(ValueError, match=message):
        Camera(100, manager=FixedMarkerPose(), mounting_path=mounting_path)


@pytest.mark.parametrize("marker_size_mm", [0, -1, float("inf"), float("nan")])
def test_marker_size_must_be_finite_and_positive(marker_size_mm):
    with pytest.raises(ValueError, match="marker_size_mm"):
        Camera(marker_size_mm, manager=FixedMarkerPose())


def test_invalid_marker_size_is_rejected_before_default_manager_opens_hardware(
    monkeypatch,
):
    monkeypatch.setattr(
        "drone.sensors.camera.camera.CameraManager",
        lambda: pytest.fail("constructed the hardware manager"),
    )

    with pytest.raises(ValueError, match="marker_size_mm"):
        Camera(0)


def test_mounting_translation_must_be_finite(tmp_path):
    mounting_path = tmp_path / "mounting.json"
    mounting_path.write_text(
        json.dumps(
            {
                "camera_to_body_frd": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "down_offset_m": float("inf"),
                "verified": True,
            }
        )
    )

    with pytest.raises(ValueError, match="finite"):
        Camera(100, manager=FixedMarkerPose(), mounting_path=mounting_path)


def test_invalid_mounting_is_rejected_before_default_manager_opens_hardware(
    monkeypatch, tmp_path
):
    mounting_path = tmp_path / "mounting.json"
    mounting_path.write_text(
        json.dumps(
            {
                "camera_to_body_frd": [[1, 0, 0], [0, -1, 0], [0, 0, 1]],
                "down_offset_m": 0.1,
                "verified": True,
            }
        )
    )
    monkeypatch.setattr(
        "drone.sensors.camera.camera.CameraManager",
        lambda: pytest.fail("constructed the hardware manager"),
    )

    with pytest.raises(ValueError, match="right-handed"):
        Camera(100, mounting_path=mounting_path)
