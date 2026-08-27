import importlib
import sys
from types import SimpleNamespace
from types import ModuleType

import pytest

from drone import timebase
from drone.common_types import GPSCoord, RelPosComplete


# DroneKit 2.9.2 imports this Python 2 compatibility alias but does not declare
# the ``future`` distribution that provides it. Keep the package workaround in
# the test process; production dependency installation belongs to the host.
try:
    from past.builtins import basestring as _basestring  # type: ignore
except ModuleNotFoundError:
    past_module = ModuleType("past")
    past_module.__path__ = []  # type: ignore[attr-defined]
    builtins_module = ModuleType("past.builtins")
    builtins_module.basestring = str  # type: ignore[attr-defined]
    sys.modules["past"] = past_module
    sys.modules["past.builtins"] = builtins_module


def test_original_mission_imports_do_not_require_hardware_drivers():
    """Regression: dependency injection must not import GPIO/I2C modules."""
    fm2 = importlib.import_module("drone.missions.fm2")
    active_fm3 = importlib.import_module("drone.mock_mission")

    assert callable(fm2.fm2)
    assert callable(active_fm3.fm3)


class FakeClock:
    def __init__(self, now_value=0.0):
        self.now_value = now_value
        self.sleeps = []

    def now(self):
        return self.now_value

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now_value += seconds


class DelayedDisarmVehicle:
    def __init__(self, clock, confirmation_delay=None):
        self.clock = clock
        self.confirmation_delay = confirmation_delay
        self.requested_at = None

    @property
    def armed(self):
        if self.requested_at is None or self.confirmation_delay is None:
            return True
        return self.clock.now() - self.requested_at < self.confirmation_delay

    @armed.setter
    def armed(self, value):
        if value is False:
            self.requested_at = self.clock.now()


def controller_without_connect(vehicle):
    from drone.control.drone_control import DroneControl

    controller = object.__new__(DroneControl)
    controller.vehicle = vehicle
    controller.cruise_alt = 10.0
    return controller


def test_mission_tracker_uses_elapsed_simulation_time():
    """Regression: a nonzero simulation epoch must not shorten the mission."""
    from drone.control.mission_info import MissonTracker

    clock = FakeClock(now_value=900.0)
    with timebase.configured(clock):
        tracker = MissonTracker(600)
        tracker.begin_mission()
        clock.sleep(50.0)
        assert tracker.time_left() == 550.0


def test_disarm_waits_for_dronekit_to_observe_false():
    """Regression: assigning armed=False is not confirmation of disarm."""
    clock = FakeClock()
    vehicle = DelayedDisarmVehicle(clock, confirmation_delay=3.0)
    controller = controller_without_connect(vehicle)

    with timebase.configured(clock):
        assert controller.disarm() == 0

    assert vehicle.armed is False
    assert clock.now_value == pytest.approx(3.0, abs=0.11)
    assert clock.sleeps


def test_disarm_returns_failure_after_fifteen_simulated_seconds():
    """Regression: an unconfirmed disarm must not block forever or report success."""
    clock = FakeClock()
    controller = controller_without_connect(
        DelayedDisarmVehicle(clock, confirmation_delay=None)
    )

    with timebase.configured(clock):
        assert controller.disarm() == -1

    assert clock.now_value == pytest.approx(15.0, abs=0.11)


class StableVehicle:
    def __init__(self, clock):
        self.clock = clock
        self.location = SimpleNamespace(
            global_relative_frame=SimpleNamespace(lat=41.0, lon=-81.0, alt=10.0)
        )
        self.attitude = SimpleNamespace(pitch=0.0)
        self.targets = []

    @property
    def velocity(self):
        speed = 0.100001 if 0.8 <= self.clock.now() <= 1.0 else 0.10
        return (speed, 0.0, 0.0)

    def simple_goto(self, target):
        self.targets.append(target)


def test_release_stability_uses_horizontal_speed_and_resets_continuous_window():
    """Regression: pitch or an earlier quiet sample must not authorize release."""
    clock = FakeClock()
    controller = controller_without_connect(StableVehicle(clock))

    with timebase.configured(clock):
        stable = controller.hold_waypoint_until_stable(
            GPSCoord(41.0, -81.0, 10.0), timeout=4.0
        )

    assert stable is True
    assert clock.now_value == pytest.approx(3.2, abs=0.21)


class RisingVehicle(StableVehicle):
    @property
    def velocity(self):
        return (0.10, 0.0, 0.0)

    def simple_goto(self, target):
        super().simple_goto(target)
        self.location.global_relative_frame.alt = (
            9.999999 if self.clock.now() <= 1.0 else 10.0
        )


def test_release_stability_resets_below_ten_metres_agl():
    """Regression: quiet flight below the release height must not count."""
    clock = FakeClock()
    controller = controller_without_connect(RisingVehicle(clock))

    with timebase.configured(clock):
        stable = controller.hold_waypoint_until_stable(
            GPSCoord(41.0, -81.0, 10.0), timeout=4.0
        )

    assert stable is True
    assert clock.now_value == pytest.approx(3.2, abs=0.21)


class NullMav:
    def statustext_send(self, severity, text):
        pass


class MissionVehicle:
    def __init__(self, armed=True):
        self._master = SimpleNamespace(mav=NullMav())
        self.armed = armed


class ReleaseController:
    def __init__(self, stable):
        self.vehicle = MissionVehicle()
        self.stable = stable
        self.climbs = []

    def goto_waypoint(self, waypoint, position_tol=None):
        return 0

    def hold_waypoint_until_stable(self, waypoint):
        return self.stable

    def set_guided_mode(self):
        return 0

    def is_landed(self):
        return False

    def climb(self, altitude):
        self.climbs.append(altitude)

    def simple_takeoff(self, altitude):
        self.climbs.append(altitude)

    def force_arm_takeoff(self, altitude):
        self.climbs.append(altitude)

    def rtl(self):
        raise AssertionError("fm3 should not enter its exception fallback")


class FakeLidar:
    def get_distance(self):
        return 10.0


class RecordingPayload:
    def __init__(self, attach_result=None, events=None):
        self.attach_result = attach_result
        self.events = events if events is not None else []
        self.drop_calls = 0

    def attach(self, target_id):
        self.events.append(("attach", target_id))
        return self.attach_result

    def drop(self):
        self.drop_calls += 1


def test_fm2_refuses_release_when_stability_gate_times_out():
    """Regression: reaching F2 alone must not release payload 2."""
    from drone.missions.fm2 import fm2

    payload = RecordingPayload()
    result = fm2(
        mt=object(),
        controller=ReleaseController(stable=False),
        cruise_alt=10,
        drop_target=GPSCoord(41.0, -81.0, 10.0),
        dropper=payload,
        lidar=FakeLidar(),
    )

    assert result is False
    assert payload.drop_calls == 0


class RecordingCamera:
    def save_frame_buffer_async(self):
        pass


class TrackerWithTime:
    def time_left(self):
        return 500.0


def test_fm3_refuses_release_when_stability_gate_times_out(monkeypatch):
    """Regression: a failed two-second gate must suppress FM3 release."""
    active = importlib.import_module("drone.mock_mission")
    monkeypatch.setattr(active, "pickup_sequence", lambda *args: True)
    payload = RecordingPayload()

    result = active.fm3(
        TrackerWithTime(),
        ReleaseController(stable=False),
        RecordingCamera(),
        FakeLidar(),
        payload,
        {3},
        GPSCoord(41.0, -81.0, 10.0),
        GPSCoord(41.1, -81.1, 10.0),
    )

    assert result is False
    assert payload.drop_calls == 0


class AcquisitionCamera:
    def __init__(self, timestamped_results):
        self.results = list(timestamped_results)
        self.last_frame_timestamp = None
        self.consumed_timestamps = []

    def vec_to_marker_3d(self, target_id, quality=4, lidar_alt=None):
        if not self.results:
            return None
        timestamp, result = self.results.pop(0)
        self.last_frame_timestamp = timestamp
        self.consumed_timestamps.append(timestamp)
        return result

    def save_frame_buffer_async(self):
        pass


class PickupController:
    def __init__(self, events):
        self.events = events
        self.vehicle = SimpleNamespace(
            armed=True,
            attitude=SimpleNamespace(yaw=0.0),
            flush=lambda: None,
        )

    def set_guided_mode(self):
        self.events.append(("mode", "GUIDED"))
        return 0

    def climb(self, altitude):
        self.events.append(("climb", altitude))

    def get_current_gps(self):
        return GPSCoord(41.0, -81.0, 10.0)

    def get_location_metres(self, original, north, east):
        return GPSCoord(original.lat, original.long, original.alt)

    def goto_waypoint(self, waypoint, position_tol=None):
        self.events.append(("goto", waypoint.alt))
        return 0

    def disarm(self):
        self.events.append(("disarm", 3))
        self.vehicle.armed = False
        return 0

    def is_landed(self):
        return True

    def simple_takeoff(self, altitude):
        self.events.append(("takeoff", altitude))

    def force_arm_takeoff(self, altitude):
        self.events.append(("takeoff", altitude))
        self.vehicle.armed = True

    def hold_waypoint_until_stable(self, waypoint):
        return True

    def rtl(self):
        self.events.append(("rtl", None))


def centered_updates(timestamps):
    return [(timestamp, RelPosComplete(0.5, 0.0, 4.572)) for timestamp in timestamps]


def test_pickup_requires_five_distinct_centered_results_after_correction(monkeypatch):
    """Regression: the first marker sighting must not immediately trigger LAND."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 2, 3, 4, 5, 6]))
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: events.append(("landed", 3)) or True,
    )
    payload = RecordingPayload(events=events)

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events), camera, FakeLidar(), 3, payload
        )

    assert result is True
    assert camera.consumed_timestamps == [1, 2, 3, 4, 5, 6]
    assert events[-3:] == [("landed", 3), ("disarm", 3), ("attach", 3)]


def test_four_results_with_one_repeated_timestamp_cannot_acquire(monkeypatch):
    """Regression: redelivering one frame must never advance marker acquisition."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 2, 2, 2, 2]))
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: events.append(("landed", 3)) or True,
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            FakeLidar(),
            3,
            RecordingPayload(events=events),
        )

    assert result is False
    assert camera.consumed_timestamps == [1, 2, 2, 2, 2]
    assert not any(event[0] == "landed" for event in events)


def test_rejected_attachment_prevents_fm3_takeoff(monkeypatch):
    """Regression: an unconfirmed joint must not be carried away from pickup."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    controller = PickupController(events)
    camera = AcquisitionCamera(
        [
            (timestamp, RelPosComplete(0.1, 0.0, 4.572))
            for timestamp in [1, 2, 3, 4, 5, 6]
        ]
    )
    monkeypatch.setattr(active, "aruco_land_precision", lambda *args: True)

    with timebase.configured(FakeClock()):
        result = active.fm3(
            TrackerWithTime(),
            controller,
            camera,
            FakeLidar(),
            RecordingPayload(attach_result=False, events=events),
            {3},
            GPSCoord(41.0, -81.0, 10.0),
            GPSCoord(41.1, -81.1, 10.0),
        )

    assert result is False
    assert ("attach", 3) in events
    attach_index = events.index(("attach", 3))
    assert not any(
        event[0] == "climb" and event[1] == 10 for event in events[attach_index + 1 :]
    )


class InjectedFrameSource:
    def __init__(self, frame):
        self.frame = frame
        self.last_timestamp_ns = 123_000_000
        self.calls = []

    def capture_frame(self, quality=4):
        self.calls.append(quality)
        return self.frame


def test_injected_camera_frames_do_not_open_hardware_index_zero(monkeypatch, tmp_path):
    """Regression: the simulation frame source must not probe host camera hardware."""
    import cv2
    import numpy as np

    from drone.sensors.camera._camera_manager import CameraManager

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(
        cv2,
        "VideoCapture",
        lambda index: pytest.fail(f"opened hardware camera index {index}"),
    )
    expected = np.zeros((480, 640, 3), dtype=np.uint8)
    source = InjectedFrameSource(expected)

    manager = CameraManager(frame_source=source)
    actual = manager.capture_frame()

    assert actual is expected
    assert source.calls == [4]
    assert manager.last_frame_timestamp == 123_000_000


def test_camera_accepts_an_injected_manager():
    """Regression: constructing Camera for simulation must not create hardware."""
    from drone.sensors.camera.camera import Camera

    manager = object()
    camera = Camera(50, manager=manager)

    assert camera.cm is manager
