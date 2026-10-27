"""Integration seams around the unmodified competition mission decisions."""

import importlib
import json
from pathlib import Path
import subprocess
import sys
import threading
import time as wall_time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest


SOURCE = Path(__file__).resolve().parents[1] / "src"


@pytest.fixture(autouse=True)
def upstream_path(monkeypatch):
    monkeypatch.syspath_prepend(str(SOURCE))


class Vehicle:
    def __init__(self):
        self.listeners = {}
        self.acks = []
        self._master = SimpleNamespace(
            mav=SimpleNamespace(command_ack_send=lambda *args: self.acks.append(args))
        )

    def on_message(self, names):
        def register(callback):
            for name in [names] if isinstance(names, str) else names:
                self.listeners[name] = callback
            return callback

        return register


class Clock:
    def __init__(self, now=10.0):
        self.current = now

    def now(self):
        return self.current

    def sleep(self, seconds):
        self.current += seconds


def test_injected_vehicle_skips_connect_and_keeps_landed_listener(monkeypatch):
    control = importlib.import_module("drone.control.drone_control")
    monkeypatch.setattr(control, "connect", lambda *a, **kw: pytest.fail("opened connection"))
    vehicle = Vehicle()

    controller = control.DroneControl(vehicle=vehicle, command_listener=False)

    assert controller.vehicle is vehicle
    assert set(vehicle.listeners) == {"EXTENDED_SYS_STATE"}
    vehicle.listeners["EXTENDED_SYS_STATE"](
        vehicle, "EXTENDED_SYS_STATE", SimpleNamespace(landed_state=1)
    )
    assert controller.is_landed()
    assert controller.command_queue.empty()
    assert vehicle.acks == []


def test_default_vehicle_connection_and_command_ack_are_preserved(monkeypatch):
    control = importlib.import_module("drone.control.drone_control")
    vehicle = Vehicle()
    connections = []

    def connect(*args, **kwargs):
        connections.append((args, kwargs))
        return vehicle

    monkeypatch.setattr(control, "connect", connect)
    controller = control.DroneControl()
    assert connections == [
        (("/dev/cu.usbmodem1103",), dict(
            wait_ready=True, heartbeat_timeout=60, timeout=120,
            source_system=1, source_component=191,
        ))
    ]
    vehicle.listeners["COMMAND_LONG"](
        vehicle, "COMMAND_LONG", SimpleNamespace(target_component=191, command=31020)
    )
    assert controller.command_queue.get_nowait() == 31020
    assert vehicle.acks == [(31020, 0)]


def test_timebase_restores_nested_context_and_wall_clock_after_error():
    timebase = importlib.import_module("drone.timebase")
    outer, inner = Clock(10), Clock(20)
    with timebase.configured(outer):
        assert timebase.time() == timebase.monotonic() == 10
        timebase.sleep(2)
        with pytest.raises(RuntimeError, match="cancelled"):
            with timebase.configured(inner):
                assert timebase.time() == 20
                raise RuntimeError("cancelled")
        assert timebase.time() == 12
    before = wall_time.time()
    assert before <= timebase.time() <= wall_time.time()
    before = wall_time.monotonic()
    assert before <= timebase.monotonic() <= wall_time.monotonic()
    timebase.sleep(0)


def test_control_wait_uses_simulation_time_and_preserves_timeout_result():
    timebase = importlib.import_module("drone.timebase")
    control = importlib.import_module("drone.control.drone_control")
    clock = Clock()
    vehicle = SimpleNamespace(location=SimpleNamespace(global_relative_frame=SimpleNamespace(alt=0)))
    with timebase.configured(clock):
        assert control.wait_alt(vehicle, 10, timeout=1) is False
        assert 11 <= clock.now() < 11.3
        vehicle.location.global_relative_frame.alt = 10
        assert control.wait_alt(vehicle, 10, timeout=1) is True


def test_paused_control_wait_can_be_cancelled_without_changing_another_thread():
    timebase = importlib.import_module("drone.timebase")
    control = importlib.import_module("drone.control.drone_control")
    sleeping, cancel = threading.Event(), threading.Event()
    failures = []

    class Cancelled(Exception):
        pass

    class PausedClock(Clock):
        def sleep(self, seconds):
            sleeping.set()
            if cancel.wait(2):
                raise Cancelled()
            raise AssertionError("cancellation did not arrive")

    def wait():
        vehicle = SimpleNamespace(location=SimpleNamespace(global_relative_frame=SimpleNamespace(alt=0)))
        try:
            with timebase.configured(PausedClock()):
                control.wait_alt(vehicle, 10, timeout=1)
        except BaseException as error:
            failures.append(error)

    worker = threading.Thread(target=wait)
    worker.start()
    try:
        assert sleeping.wait(1)
        assert worker.is_alive()
        assert timebase.time() > 1_000_000
    finally:
        cancel.set()
        worker.join(1)
    assert not worker.is_alive()
    assert len(failures) == 1 and isinstance(failures[0], Cancelled)


def test_mission_timers_use_injected_time_but_waypoint_age_stays_wall_time(tmp_path, monkeypatch):
    timebase = importlib.import_module("drone.timebase")
    tracker_module = importlib.import_module("drone.control.mission_info")
    common = importlib.import_module("drone.common_types")
    monkeypatch.chdir(tmp_path)
    (tmp_path / "mission_data").mkdir()
    waypoint_file = tmp_path / "mission_data/waypoints.json"
    waypoint_file.write_text("{}")
    monkeypatch.setattr(wall_time, "time", lambda: 1_000)
    clock = Clock(10)
    tracker = tracker_module.MissonTracker(600)
    with timebase.configured(clock):
        tracker.begin_mission()
        tracker.begin_aux_timer()
        tracker.set_waypoint("L", common.GPSCoord(1, 2, 3))
        clock.sleep(25)
        assert tracker.time_left() == 575
        assert tracker.end_aux_timer() == 25
        assert json.loads(waypoint_file.read_text())["L"]["loaded_at"] == 1_000
        assert tracker.get_waypoint("L", max_age_seconds=10).alt == 3
        monkeypatch.setattr(wall_time, "time", lambda: 1_011)
        assert tracker.get_waypoint("L", max_age_seconds=10) is None


def test_mission_and_aux_timers_accept_simulation_epoch_zero():
    timebase = importlib.import_module("drone.timebase")
    tracker_module = importlib.import_module("drone.control.mission_info")
    clock = Clock(0)
    tracker = tracker_module.MissonTracker(600)
    with timebase.configured(clock):
        assert tracker.time_left() == -1
        assert tracker.end_aux_timer() == -1
        tracker.begin_mission()
        tracker.begin_aux_timer()
        assert tracker.time_left() == 600
        assert tracker.end_aux_timer() == 0
        clock.sleep(5)
        assert tracker.time_left() == 595
        assert tracker.end_aux_timer() == 5


def test_actual_missions_import_without_i2c_or_gpio_dependencies():
    script = """
import importlib.abc
import sys

class NoHardware(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname.split('.')[0] in {'board', 'busio', 'adafruit_lidarlite', 'gpiozero'}:
            raise AssertionError('hardware import: ' + fullname)

sys.meta_path.insert(0, NoHardware())
sys.path.insert(0, sys.argv[1])
from drone.missions.fm1 import fm1
from drone.missions.fm2 import fm2
from drone.mock_mission import fm3
assert all(callable(mission) for mission in (fm1, fm2, fm3))
"""
    result = subprocess.run([sys.executable, "-c", script, str(SOURCE)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_precision_landing_uses_cancellable_injected_sleep():
    timebase = importlib.import_module("drone.timebase")
    mission = importlib.import_module("drone.mock_mission")

    class Cancelled(Exception):
        pass

    class CancelledClock(Clock):
        def sleep(self, seconds):
            raise Cancelled()

    commands = []
    controller = SimpleNamespace(
        set_land_mode=lambda: commands.append("LAND"),
        vehicle=SimpleNamespace(armed=True),
    )
    camera = SimpleNamespace(vec_to_marker_3d=lambda *a, **kw: None)
    lidar = SimpleNamespace(get_distance=lambda: 1)
    with timebase.configured(CancelledClock()):
        with pytest.raises(Cancelled):
            mission.aruco_land_precision(controller, camera, lidar, 7)
    assert commands == ["LAND"]


def test_default_camera_capture_and_downsampling_are_preserved(monkeypatch):
    manager_module = importlib.import_module("drone.sensors.camera._camera_manager")
    camera_module = importlib.import_module("drone.sensors.camera.camera")
    monkeypatch.chdir(SOURCE.parent)
    frame = np.zeros((60, 90, 3), dtype=np.uint8)
    devices, released = [], []

    def open_camera(index):
        devices.append(index)
        return SimpleNamespace(read=lambda: (True, frame), release=lambda: released.append(index))

    monkeypatch.setattr(cv2, "VideoCapture", open_camera)
    camera = camera_module.Camera(100)
    assert isinstance(camera.cm, manager_module.CameraManager)
    assert camera.cm.capture_frame() is frame
    assert camera.cm.capture_frame(quality=2).shape == (5, 7, 3)
    assert camera.cm.CAMERA_CENTER == [3.5, 2.5]
    camera.cm.end_all()
    assert devices == released == [0]


def test_injected_frames_and_calibration_reach_actual_pose_math(tmp_path, monkeypatch):
    manager_module = importlib.import_module("drone.sensors.camera._camera_manager")
    camera_module = importlib.import_module("drone.sensors.camera.camera")
    monkeypatch.setattr(cv2, "VideoCapture", lambda *args: pytest.fail("opened webcam"))
    calibration = tmp_path / "calibration.json"
    calibration.write_text(json.dumps({
        "camera_matrix": [[800, 0, 320], [0, 800, 240], [0, 0, 1]],
        "dist_coeff": [[0, 0, 0, 0, 0]],
    }))
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250), 7, 81
    )
    frame[240:321, 360:441] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    blank = np.full_like(frame, 255)
    source = SimpleNamespace(capture_frame=lambda quality=4: frame if quality == 4 else blank)
    manager = manager_module.CameraManager(frame_source=source, calibration_path=calibration)
    monkeypatch.setattr(camera_module, "CameraManager", lambda: pytest.fail("constructed another camera"))
    camera = camera_module.Camera(100, manager=manager)

    position = camera.vec_to_marker_3d(7)

    assert position.x == pytest.approx(0.1, abs=0.001)
    assert position.y == pytest.approx(0.05, abs=0.001)
    assert position.z == pytest.approx(1.1, abs=0.001)
    assert camera.frame_buffer[0] is frame
    assert camera.vec_to_marker_3d(99) is None
    assert camera.vec_to_marker_3d(7, quality=1) is None
    assert camera.vec_to_marker_3d(7, lidar_alt=2).z == 2.1
    manager.end_all()
