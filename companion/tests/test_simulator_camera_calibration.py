from __future__ import annotations

import math
import json
from pathlib import Path

import numpy as np
import pytest

from drone_sim_companion.comp2026_host import SimulationClock
import drone_sim_companion.runtime_node as runtime_node


def test_competition_runtime_uses_gazebo_camera_intrinsics(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    nested_source = Path(__file__).parents[1] / "comp2026/src"
    monkeypatch.syspath_prepend(str(nested_source))

    from drone.sensors.camera._camera_manager import CameraManager
    from drone.sensors.camera.camera import Camera

    camera = runtime_node._create_simulator_camera(
        CameraManager,
        Camera,
        frame_source=object(),
        clock=SimulationClock(),
    )
    rendered_corners = [
        np.array(
            [
                [
                    [388.27520751953125, 198.62107849121094],
                    [408.9646911621094, 198.62107849121094],
                    [408.9646911621094, 219.310546875],
                    [388.27520751953125, 219.310546875],
                ]
            ],
            dtype=np.float32,
        )
    ]

    estimated_mm = camera.cm.estimate_pose_3d(
        3,
        rendered_corners,
        np.array([[3]], dtype=np.int32),
        100,
    )

    assert estimated_mm is not None
    horizontal_error_m = math.hypot(
        estimated_mm[0] / 1000.0 - 0.38,
        estimated_mm[1] / 1000.0 - (-0.15),
    )
    assert horizontal_error_m < 0.07


def test_moving_pad_metadata_maps_verified_sdf_camera_to_body_frd(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    nested_source = Path(__file__).parents[1] / "comp2026/src"
    monkeypatch.syspath_prepend(str(nested_source))
    from drone.sensors.camera._camera_manager import CameraManager
    from drone.sensors.camera.camera import Camera

    module = Path(__file__).parents[1] / "src/drone_sim_companion"
    calibration = module / "moving_pad_camera_calibration.json"
    mounting = module / "moving_pad_camera_mounting.json"
    manager = CameraManager(frame_source=object(), calibration_path=calibration)
    camera = Camera(100, manager=manager, mounting_path=mounting)
    half = 10.3447300601
    center = (402.7578404804, 198.6210797598)
    corners = [np.array([[
        [center[0] - half, center[1] - half],
        [center[0] + half, center[1] - half],
        [center[0] + half, center[1] + half],
        [center[0] - half, center[1] + half],
    ]], dtype=np.float32)]
    tvec_mm = manager.estimate_pose_3d(
        7, corners, np.array([[7]], dtype=np.int32), 100,
    )

    assert tvec_mm is not None
    body = camera._body_marker_vector(tvec_mm)
    assert (body.x, body.y, body.z) == pytest.approx((0.2, 0.4, 5.1), abs=0.02)
    assert json.loads(calibration.read_text())["dimensions_verified"] is True
    assert json.loads(mounting.read_text())["verified"] is True
    assert json.loads(mounting.read_text())["sdf_pose_body_flu"] == [
        0.0, 0.0, -0.1, 0.0, 1.570796327, 0.0,
    ]
