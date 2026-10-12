from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from drone_sim_companion.comp2026_host import RosFrameSource, SimulationClock
import drone_sim_companion.runtime_node as runtime_node


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


@pytest.fixture
def camera_types(monkeypatch: pytest.MonkeyPatch) -> tuple[type, type]:
    monkeypatch.syspath_prepend(
        str(Path(__file__).parents[1] / "comp2026" / "src")
    )
    from drone.sensors.camera._camera_manager import CameraManager
    from drone.sensors.camera.camera import Camera

    return CameraManager, Camera


def _camera(camera_types: tuple[type, type], source: RosFrameSource, clock: SimulationClock):
    manager_type, camera_type = camera_types
    return runtime_node._create_simulator_camera(
        manager_type,
        camera_type,
        source,
        clock=clock,
    )


def test_factory_makes_real_gazebo_frame_precision_ready(
    camera_types: tuple[type, type],
) -> None:
    clock = SimulationClock()
    clock.accept(1_000_000_000)
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(_image(900_000_000))
    camera = _camera(camera_types, source, clock)

    observation = camera.cm.capture_observation()

    assert camera.precision_readiness(observation).ready is True
    assert observation.metadata.exposure_timestamp_ns == 900_000_000
    assert observation.metadata.exposure_age_ns == 100_000_000
    assert observation.metadata.raw_image_size_px == (640, 480)
    body = camera._body_marker_vector((400.0, -200.0, 5_000.0))
    assert (body.x, body.y, body.z) == pytest.approx((0.2, 0.4, 5.1))


def test_factory_rejects_frame_dimensions_that_do_not_match_calibration(
    camera_types: tuple[type, type],
) -> None:
    clock = SimulationClock()
    clock.accept(1_000_000_000)
    source = RosFrameSource(width_px=320, height_px=240)
    source.accept_image(_image(900_000_000, width=320, height=240))
    camera = _camera(camera_types, source, clock)

    observation = camera.cm.capture_observation()
    readiness = camera.precision_readiness(observation)

    assert readiness.ready is False
    assert readiness.reasons == ("captured image dimensions do not match calibration",)


def test_factory_rejects_exposure_older_than_quarter_simulated_second(
    camera_types: tuple[type, type],
) -> None:
    clock = SimulationClock()
    clock.accept(1_000_000_000)
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(_image(749_999_999))
    camera = _camera(camera_types, source, clock)

    with pytest.raises(RuntimeError, match="older than the configured acquisition limit"):
        camera.cm.capture_observation()


def test_camera_metadata_json_files_are_packaged() -> None:
    module = Path(runtime_node.__file__).parent
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
