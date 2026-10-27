from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from drone_sim_companion.comp2026_host import RosFrameSource, SimulationClock
import drone_sim_companion.configured_runtime as configured_runtime
from drone_sim_companion.precision_camera._camera_manager import CameraManager
from drone_sim_companion.precision_camera.camera import Camera


def _image(timestamp_ns: int, *, width: int = 640, height: int = 480) -> object:
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=timestamp_ns // 1_000_000_000,
                nanosec=timestamp_ns % 1_000_000_000,
            ),
            frame_id="camera/onboard",
        ),
        width=width,
        height=height,
        encoding="rgb8",
        is_bigendian=False,
        step=width * 3,
        data=bytes((10, 20, 30)) * (width * height),
    )


def _camera(source: RosFrameSource, clock: SimulationClock) -> Camera:
    module = Path(configured_runtime.__file__).parent
    manager = CameraManager(
        frame_source=source,
        calibration_path=module / "gazebo_camera_calibration.json",
        clock=lambda: clock.timestamp_ns,
        max_exposure_age_ns=250_000_000,
    )
    return Camera(
        100,
        manager=manager,
        mounting_path=module / "gazebo_camera_mounting.json",
    )


def test_real_gazebo_frame_is_precision_ready() -> None:
    clock = SimulationClock()
    clock.accept(1_000_000_000)
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(_image(900_000_000))
    camera = _camera(source, clock)

    observation = camera.cm.capture_observation()

    assert camera.precision_readiness(observation).ready is True
    assert observation.metadata.exposure_timestamp_ns == 900_000_000
    assert observation.metadata.exposure_age_ns == 100_000_000
    assert observation.metadata.raw_image_size_px == (640, 480)
    body = camera._body_marker_vector((400.0, -200.0, 5_000.0))
    assert (body.x, body.y, body.z) == pytest.approx((0.2, 0.4, 5.1))


def test_precision_camera_rejects_frame_dimensions_that_do_not_match_calibration() -> None:
    clock = SimulationClock()
    clock.accept(1_000_000_000)
    source = RosFrameSource(width_px=320, height_px=240)
    source.accept_image(_image(900_000_000, width=320, height=240))
    camera = _camera(source, clock)

    observation = camera.cm.capture_observation()
    readiness = camera.precision_readiness(observation)

    assert readiness.ready is False
    assert readiness.reasons == ("captured image dimensions do not match calibration",)


def test_precision_camera_rejects_exposure_older_than_quarter_simulated_second() -> None:
    clock = SimulationClock()
    clock.accept(1_000_000_000)
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(_image(749_999_999))
    camera = _camera(source, clock)

    with pytest.raises(RuntimeError, match="older than the configured acquisition limit"):
        camera.cm.capture_observation()


def test_camera_metadata_json_files_are_packaged() -> None:
    module = Path(configured_runtime.__file__).parent
    calibration = json.loads((module / "gazebo_camera_calibration.json").read_text())
    mounting = json.loads((module / "gazebo_camera_mounting.json").read_text())
    package_data = (
        Path(__file__).parents[1] / "pyproject.toml"
    ).read_text()

    assert calibration["image_width_px"] == 640
    assert calibration["image_height_px"] == 480
    assert calibration["dimensions_verified"] is True
    assert mounting["sdf_pose_body_flu"] == [
        0.0, 0.0, -0.1, 0.0, 1.570796327, 0.0,
    ]
    assert '"gazebo_camera_mounting.json"' in package_data
