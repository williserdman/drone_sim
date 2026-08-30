from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

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
