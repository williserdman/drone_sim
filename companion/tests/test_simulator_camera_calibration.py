from __future__ import annotations

import math
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

import drone_sim_companion.runtime_node as runtime_node
from drone_sim_companion.comp2026_host import RosFrameSource, SimulationClock


ROOT = Path(__file__).parents[2]
SCENARIO_PATH = ROOT / "config/scenario.yaml"


def _stamp(timestamp_ns: int) -> SimpleNamespace:
    return SimpleNamespace(
        sec=timestamp_ns // 1_000_000_000,
        nanosec=timestamp_ns % 1_000_000_000,
    )


def _image(timestamp_ns: int) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(timestamp_ns), frame_id="camera/onboard"),
        width=640,
        height=480,
        encoding="rgb8",
        is_bigendian=False,
        step=640 * 3,
        data=bytes((10, 20, 30)) * (640 * 480),
    )


def _nested_camera_types(monkeypatch: pytest.MonkeyPatch):
    nested_source = ROOT / "companion/comp2026/src"
    monkeypatch.syspath_prepend(str(nested_source))
    from drone.sensors.camera._camera_manager import CameraManager
    from drone.sensors.camera.camera import Camera

    return CameraManager, Camera


def test_competition_runtime_uses_gazebo_camera_intrinsics(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    CameraManager, Camera = _nested_camera_types(monkeypatch)
    clock = SimulationClock()
    clock.accept(200_000_000)

    camera = runtime_node._create_simulator_camera(
        CameraManager,
        Camera,
        frame_source=object(),
        clock=clock,
        scenario_path=SCENARIO_PATH,
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


def test_competition_runtime_uses_verified_simulator_geometry(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    CameraManager, Camera = _nested_camera_types(monkeypatch)
    clock = SimulationClock()
    clock.accept(200_000_000)

    camera = runtime_node._create_simulator_camera(
        CameraManager,
        Camera,
        frame_source=object(),
        clock=clock,
        scenario_path=SCENARIO_PATH,
    )

    assert camera.marker_size_mm == 100
    assert camera.cm.calibrated_image_size_px == (640, 480)
    assert camera._camera_to_body_frd == (
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    assert camera._down_offset_m == pytest.approx(0.1)
    assert camera._mounting_verified is True
    marker = camera._body_marker_vector(np.array([200.0, -300.0, 4_000.0]))
    assert (marker.x, marker.y, marker.z) == pytest.approx((0.3, 0.2, 4.1))


def test_simulator_camera_records_bounded_source_exposure_age(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    CameraManager, Camera = _nested_camera_types(monkeypatch)
    clock = SimulationClock()
    clock.accept(1_500_000_000)
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(_image(1_000_000_000))
    camera = runtime_node._create_simulator_camera(
        CameraManager,
        Camera,
        frame_source=source,
        clock=clock,
        scenario_path=SCENARIO_PATH,
    )

    observation = camera.cm.capture_observation()

    assert observation.metadata.exposure_timestamp_ns == 1_000_000_000
    assert observation.metadata.receipt_timestamp_ns == 1_500_000_000
    assert observation.metadata.exposure_age_ns == 500_000_000
    assert observation.metadata.exposure_age_bounded is True


def test_bounded_simulator_camera_shutdown_cannot_return_the_last_observation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    CameraManager, Camera = _nested_camera_types(monkeypatch)
    clock = SimulationClock()
    clock.accept(1_000_000_000)
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(_image(1_000_000_000))
    camera = runtime_node._create_simulator_camera(
        CameraManager,
        Camera,
        frame_source=source,
        clock=clock,
        scenario_path=SCENARIO_PATH,
    )
    first = camera.cm.capture_observation_bounded(timeout_s=1.0)
    completed = threading.Event()
    results: list[object] = []

    def request_next() -> None:
        try:
            results.append(
                camera.cm.capture_observation_bounded(
                    timeout_s=1.0,
                    after_sequence=first.metadata.sequence,
                )
            )
        except BaseException as error:
            results.append(error)
        finally:
            completed.set()

    consumer = threading.Thread(target=request_next)
    consumer.start()
    assert not completed.wait(0.05)

    source.stop("attempt finished")
    assert camera.cm.stop_acquisition(timeout_s=1.0) is True
    consumer.join(timeout=1.0)

    assert consumer.is_alive() is False
    assert camera.cm._acquisition_thread.is_alive() is False
    assert len(results) == 1
    assert isinstance(results[0], RuntimeError)
    assert "worker failed" in str(results[0])


def test_ros_frame_source_stop_is_idempotent() -> None:
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(_image(1_000_000_000))

    source.stop("first shutdown")
    source.stop("later shutdown")

    with pytest.raises(RuntimeError, match="first shutdown"):
        source.capture_frame()


@pytest.mark.parametrize(
    ("section", "field", "replacement"),
    [
        ("camera", "width_px", 320),
        ("camera", "height_px", 240),
        ("camera", "horizontal_fov_rad", 0.7),
        ("camera", "body_position_m", [0.0, 0.0, -0.2]),
        ("payload_geometry", "marker_size_mm", 80),
    ],
)
def test_competition_runtime_rejects_scenario_camera_mismatch(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    section: str,
    field: str,
    replacement: object,
) -> None:
    CameraManager, Camera = _nested_camera_types(monkeypatch)
    scenario = yaml.safe_load(SCENARIO_PATH.read_text(encoding="utf-8"))
    scenario[section][field] = replacement
    mismatch_path = tmp_path / "scenario.yaml"
    mismatch_path.write_text(yaml.safe_dump(scenario), encoding="utf-8")

    with pytest.raises(ValueError, match="simulator camera"):
        runtime_node._create_simulator_camera(
            CameraManager,
            Camera,
            frame_source=object(),
            clock=SimulationClock(),
            scenario_path=mismatch_path,
        )


@pytest.mark.parametrize(
    ("clock_timestamp_ns", "image_timestamp_ns", "message"),
    [
        (1_500_000_001, 1_000_000_000, "older"),
        (999_999_999, 1_000_000_000, "future"),
    ],
)
def test_simulator_camera_bounds_source_exposure_against_simulation_time(
    monkeypatch: pytest.MonkeyPatch,
    clock_timestamp_ns: int,
    image_timestamp_ns: int,
    message: str,
) -> None:
    CameraManager, Camera = _nested_camera_types(monkeypatch)
    clock = SimulationClock()
    clock.accept(clock_timestamp_ns)
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(_image(image_timestamp_ns))
    camera = runtime_node._create_simulator_camera(
        CameraManager,
        Camera,
        frame_source=source,
        clock=clock,
        scenario_path=SCENARIO_PATH,
    )

    with pytest.raises(RuntimeError, match=message):
        camera.cm.capture_observation()


@pytest.mark.parametrize("clock_state", ["unavailable", "stopped"])
def test_simulator_camera_rejects_unavailable_clock_evidence(
    monkeypatch: pytest.MonkeyPatch,
    clock_state: str,
) -> None:
    CameraManager, Camera = _nested_camera_types(monkeypatch)
    clock = SimulationClock()
    if clock_state == "stopped":
        clock.accept(1_000_000_000)
        clock.stop("run ended")
    source = RosFrameSource(width_px=640, height_px=480)
    source.accept_image(_image(1_000_000_000))
    camera = runtime_node._create_simulator_camera(
        CameraManager,
        Camera,
        frame_source=source,
        clock=clock,
        scenario_path=SCENARIO_PATH,
    )

    with pytest.raises(RuntimeError, match="clock"):
        camera.cm.capture_observation()
