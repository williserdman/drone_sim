import importlib
import math
import runpy
import struct
import sys
import threading
from types import SimpleNamespace
from types import ModuleType

import pytest

from drone import timebase
from drone.common_types import GPSCoord, MissionHome, RelPosComplete
from drone.control.stability import ReleaseStabilityConfig
from drone.sensors.lidar.clearance import ClearanceCalibration
from drone.sensors.lidar.lidar import LidarSample


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
        self.callbacks = []

    def now(self):
        return self.now_value

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now_value += seconds
        for callback in tuple(self.callbacks):
            callback()




class DelayedDisarmVehicle:
    def __init__(self, clock, confirmation_delay=None):
        self.clock = clock
        self.confirmation_delay = confirmation_delay
        self.requested_at = None
        self.command_callback = lambda: None

    @property
    def armed(self):
        if self.requested_at is None or self.confirmation_delay is None:
            return True
        return self.clock.now() - self.requested_at < self.confirmation_delay

    @armed.setter
    def armed(self, value):
        if value is False:
            self.requested_at = self.clock.now()
            self.command_callback()


class DeniedDisarmThenAutoDisarmVehicle:
    """Model the preserved L-pad timing around ArduPilot auto-disarm."""

    def __init__(self, clock):
        self.clock = clock
        self.touchdown_at = 14.632541
        self.auto_disarm_delay = 0.506915
        self.disarm_requests = 0

    @property
    def armed(self):
        return self.clock.now() < self.touchdown_at + self.auto_disarm_delay

    @armed.setter
    def armed(self, value):
        if value is not False:
            raise AssertionError("the focused fixture only accepts a disarm request")
        self.disarm_requests += 1


def controller_without_connect(vehicle, clock, *, airborne=False):
    from drone.control.drone_control import (
        CommandAckTracker,
        DroneControl,
        MissionAckTracker,
    )
    from drone.control.flight_state import FlightState, RCModeBand

    controller = object.__new__(DroneControl)
    controller.vehicle = vehicle
    controller.cruise_alt = 10.0
    bounds = {name: 100.0 for name in (
        "heartbeat", "mode", "location", "velocity", "attitude",
        "landed_state", "armed", "home", "rc_input", "range", "failsafe",
    )}
    state = FlightState(
        source_system=1,
        source_component=1,
        freshness_bounds=bounds,
        rc_channel=7,
        rc_mode_mapping=(
            RCModeBand("companion", 1400, 1600, "GUIDED"),
            RCModeBand("pilot", 1800, 2100, "LOITER"),
        ),
        clock=clock.now,
    )
    sequence = [1]
    assert state.update_many(
        {
            "heartbeat": "simulation",
            "mode": "GUIDED",
            "location": (410000000, -810000000, 0, 10000),
            "velocity": (10, 0, 0),
            "attitude": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "landed_state": 1,
            "armed": False,
            "failsafe": (False, "simulation clear"),
        },
        received_at=clock.now(), sequence=sequence[0],
        source_system=1, source_component=1,
    )
    assert state.observe_rc_input(
        channel=7, pwm=1500, signal_healthy=True,
        received_at=clock.now(), sequence=sequence[0],
        source_system=1, source_component=1,
    )
    assert state.acquire_initial_companion_authority(now=clock.now())
    controller.flight_controller_target = state.source
    controller.flight_state = state
    controller.permission_guard = lambda: None
    controller.mission_home_check = lambda home: None
    controller.fc_home_position_tolerance_m = 1.0
    controller.fc_home_altitude_tolerance_m = 1.0
    controller.clearance_calibration = ClearanceCalibration(
        beam_direction_body_frd=(0.0, 0.0, 1.0),
        measured_reference_offset_body_frd_m=(0.0, 0.0, 0.0),
        lidar_mounting_offset_already_applied=True,
        max_tilt_rad=0.3,
        max_age_seconds=0.3,
        max_skew_seconds=0.2,
        locally_horizontal_planar_surface=True,
    )
    controller.release_stability_config = ReleaseStabilityConfig(
        hold_seconds=2.0,
        timeout_seconds=4.0,
        max_horizontal_speed_m_s=0.1,
        max_vertical_speed_m_s=0.1,
        max_roll_rad=0.2,
        max_pitch_rad=0.2,
        horizontal_position_tolerance_m=0.15,
        vertical_position_tolerance_m=0.2,
        max_observation_skew_seconds=0.2,
        max_observation_gap_seconds=0.3,
        poll_interval_seconds=0.05,
        waypoint_reissue_interval_seconds=0.2,
    )
    controller._release_hold_confirmation = None
    controller._mission_home = None
    controller._last_arm_boundary_sequence = None
    if not hasattr(vehicle, "message_factory"):
        vehicle.message_factory = SimpleNamespace(
            command_long_encode=lambda *fields: ("command-long", fields)
        )
    if not hasattr(vehicle, "send_mavlink"):
        def send_mavlink(message):
            kind, fields = message
            assert kind == "command-long"
            if fields[2] == 400:
                vehicle.armed = fields[4] == 1.0

        vehicle.send_mavlink = send_mavlink
    controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=lambda operation, enqueue_check: (
            enqueue_check(), operation()
        )[1],
    )
    controller.set_mission_home(MissionHome(41.0, -81.0, 0.0))
    sequence[0] += 1
    assert state.update_many(
        {"landed_state": 2 if airborne else 1, "armed": True},
        received_at=clock.now(), sequence=sequence[0],
        source_system=1, source_component=1,
    )

    def publish_vehicle_state():
        sequence[0] += 1
        values = {"armed": bool(getattr(vehicle, "armed", True))}
        frame = getattr(getattr(vehicle, "location", None), "global_relative_frame", None)
        if frame is not None:
            values["location"] = (
                round(frame.lat * 1e7),
                round(frame.lon * 1e7),
                round(frame.alt * 1000),
                round(frame.alt * 1000),
            )
        velocity = getattr(vehicle, "velocity", None)
        if velocity is not None:
            values["velocity"] = tuple(round(component * 100) for component in velocity)
        attitude = getattr(vehicle, "attitude", None)
        if attitude is not None:
            values["attitude"] = (
                attitude.roll,
                attitude.pitch,
                attitude.yaw,
                0.0,
                0.0,
                0.0,
            )
        state.update_many(
            values, received_at=clock.now(),
            sequence=sequence[0], source_system=1, source_component=1,
        )

    clock.callbacks.append(publish_vehicle_state)
    controller._command_ack_tracker = CommandAckTracker(
        wire_protocol="2.0", source_system=1, source_component=1,
        target_system=1, target_component=191,
    )
    controller._mission_ack_tracker = MissionAckTracker(
        source_system=1, source_component=1,
        target_system=1, target_component=191,
        mission_type=0,
    )
    controller._mission_ack_transaction_lock = threading.Lock()
    mission_mav = getattr(vehicle, "mission_mav", None)
    if mission_mav is not None:
        mission_mav.mission_callback = lambda: controller._mission_ack_tracker.observe_message(
            SimpleNamespace(
                type=0,
                mission_type=0,
                target_system=1,
                target_component=191,
                get_srcSystem=lambda: 1,
                get_srcComponent=lambda: 1,
            )
        )
    if hasattr(vehicle, "command_callback"):
        vehicle.command_callback = lambda: controller._command_ack_tracker.observe(
            command=400,
            result=0,
            source_system=1,
            source_component=1,
            target_system=1,
            target_component=191,
        )
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


def test_mission_tracker_accepts_zero_as_a_valid_start_time():
    """Regression: simulation epoch zero must retain the mission allowance."""
    from drone.control.mission_info import MissonTracker

    clock = FakeClock(now_value=0.0)
    with timebase.configured(clock):
        tracker = MissonTracker(600)
        tracker.begin_mission()
        clock.sleep(50.0)
        assert tracker.time_left() == 550.0


def test_disarm_waits_for_dronekit_to_observe_false():
    """Regression: assigning armed=False is not confirmation of disarm."""
    clock = FakeClock()
    vehicle = DelayedDisarmVehicle(clock, confirmation_delay=3.0)
    controller = controller_without_connect(vehicle, clock)

    with timebase.configured(clock):
        assert controller.disarm() == 0

    assert vehicle.armed is False
    assert clock.now_value == pytest.approx(3.0, abs=0.11)
    assert clock.sleeps


def test_disarm_returns_failure_after_sixteen_simulated_seconds():
    """Regression: an unconfirmed disarm must not block forever or report success."""
    clock = FakeClock()
    controller = controller_without_connect(
        DelayedDisarmVehicle(clock, confirmation_delay=None), clock
    )

    with timebase.configured(clock):
        with pytest.raises(RuntimeError, match="disarm was not confirmed"):
            controller.disarm()

    assert clock.now_value == pytest.approx(16.0, abs=1e-12)


def test_disarm_does_not_issue_an_airborne_request():
    """Airborne auto-disarm timing is not permission to request disarm."""
    clock = FakeClock()
    vehicle = DeniedDisarmThenAutoDisarmVehicle(clock)
    controller = controller_without_connect(vehicle, clock, airborne=True)

    with timebase.configured(clock):
        with pytest.raises(RuntimeError, match="touchdown"):
            controller.disarm()

    assert vehicle.disarm_requests == 0
    assert clock.now_value == 0.0


class RecordingMissionMav:
    def __init__(self):
        self.items = []
        self.mission_callback = lambda: None

    def mission_item_int_send(self, *args):
        self.items.append(args)
        self.mission_callback()


class StableVehicle:
    def __init__(self, clock):
        self.clock = clock
        self.location = SimpleNamespace(
            global_relative_frame=SimpleNamespace(lat=41.0, lon=-81.0, alt=10.0)
        )
        self.attitude = SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0)
        self.targets = []
        self.groundspeed = None
        self.mission_mav = RecordingMissionMav()
        self._master = SimpleNamespace(mav=self.mission_mav)

    @property
    def velocity(self):
        speed = 0.11 if 0.8 <= self.clock.now() <= 1.0 else 0.10
        return (speed, 0.0, 0.0)

    def simple_goto(self, target):
        self.targets.append(target)


def test_legacy_free_goto_is_disabled_without_losing_precision_regression():
    """Only the pinned, confirmed controller operation may send global transit."""
    from drone.control.drone_control import goto

    target_lat = 37.4003371
    target_lon = -122.08175843013008
    vehicle = StableVehicle(FakeClock())

    with pytest.raises(NotImplementedError, match="pinned mission home"):
        goto(vehicle, target_lat, target_lon, 10.0, lambda: None)

    assert vehicle.groundspeed is None
    assert vehicle.targets == []
    assert vehicle.mission_mav.items == []

    legacy_lon = struct.unpack("f", struct.pack("f", target_lon))[0]
    legacy = GPSCoord(target_lat, legacy_lon, 0)
    exact = GPSCoord(target_lat, -1220817584 / 1e7, 0)
    target = GPSCoord(target_lat, target_lon, 0)
    assert horiz_distance_m_for_test(legacy, target) > 0.15
    assert horiz_distance_m_for_test(exact, target) < 0.01


def horiz_distance_m_for_test(a, b):
    from drone.control.drone_control import horiz_distance_m

    return horiz_distance_m(a, b)


def test_release_hold_reissues_exact_integer_waypoint_every_two_tenths():
    from pymavlink import mavutil

    clock = FakeClock()
    vehicle = StableVehicle(clock)
    controller = controller_without_connect(vehicle, clock, airborne=True)
    controller.release_stability_config = ReleaseStabilityConfig(
        **{
            **controller.release_stability_config.__dict__,
            "hold_seconds": 0.4,
            "timeout_seconds": 1.0,
        }
    )

    with timebase.configured(clock):
        stable = controller.hold_waypoint_until_stable(
            GPSCoord(37.4003371, -122.08175843013008, 10.0),
            FakeLidar(clock),
            required_agl_m=10.0,
        )

    assert stable is False
    assert clock.sleeps == [0.05] * 20
    assert vehicle.targets == []
    assert len(vehicle.mission_mav.items) == 5
    assert all(
        item[3] == mavutil.mavlink.MAV_FRAME_GLOBAL_INT
        and item[4] == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        and item[5] == 2
        and item[11:14] == (374003371, -1220817584, 10.0)
        for item in vehicle.mission_mav.items
    )


def test_release_stability_uses_horizontal_speed_and_resets_continuous_window():
    """Regression: pitch or an earlier quiet sample must not authorize release."""
    clock = FakeClock()
    controller = controller_without_connect(StableVehicle(clock), clock, airborne=True)

    with timebase.configured(clock):
        stable = controller.hold_waypoint_until_stable(
            GPSCoord(41.0, -81.0, 10.0),
            FakeLidar(clock),
            required_agl_m=10.0,
        )

    assert stable is True
    assert clock.now_value == pytest.approx(3.2, abs=0.21)


def test_release_stability_uses_lidar_agl_not_adjusted_navigation_altitude():
    """Regression: terrain-corrected navigation altitude is not lidar AGL."""
    clock = FakeClock()
    vehicle = StableVehicle(clock)
    vehicle.location.global_relative_frame.alt = 15.0
    controller = controller_without_connect(vehicle, clock, airborne=True)

    with timebase.configured(clock):
        stable = controller.hold_waypoint_until_stable(
            GPSCoord(41.0, -81.0, 15.0),
            FakeLidar(clock),
            required_agl_m=10.0,
        )

    assert stable is True
    assert vehicle.mission_mav.items
    assert all(item[13] == 15.0 for item in vehicle.mission_mav.items)


def test_release_stability_resets_below_ten_metres_agl_after_terrain_adjustment():
    """Regression: quiet flight below 10 m AGL must reset the hold window."""
    clock = FakeClock()
    vehicle = StableVehicle(clock)
    vehicle.location.global_relative_frame.alt = 15.0
    controller = controller_without_connect(vehicle, clock, airborne=True)

    class RisingLidar(FakeLidar):
        def get_sample(self):
            self.distance = 9.999999 if clock.now() <= 1.0 else 10.0
            return super().get_sample()

    with timebase.configured(clock):
        stable = controller.hold_waypoint_until_stable(
            GPSCoord(41.0, -81.0, 15.0),
            RisingLidar(clock),
            required_agl_m=10.0,
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

    def check_permission(self):
        return None

    def require_release_configuration(self):
        return None

    def release_waypoint_for_clearance(self, waypoint, lidar, *, desired_agl_m):
        return waypoint

    def release_payload_if_stable(
        self, dropper, waypoint, lidar, *, required_agl_m
    ):
        dropper.drop()

    def goto_waypoint(self, waypoint, position_tol=None):
        return 0

    def hold_waypoint_until_stable(self, waypoint, lidar, *, required_agl_m):
        self.hold_lidar = lidar
        self.hold_waypoint = waypoint
        self.required_agl_m = required_agl_m
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
    def __init__(self, clock=None):
        self.clock = clock
        self.sequence = 0
        self.distance = 10.0

    def get_sample(self):
        self.sequence += 1
        sampled_at = self.clock.now() if self.clock is not None else timebase.monotonic()
        return LidarSample(self.distance, sampled_at, self.sequence, 0)

    def get_distance(self):
        return self.distance


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
    controller = ReleaseController(stable=False)
    lidar = FakeLidar()
    result = fm2(
        mt=object(),
        controller=controller,
        cruise_alt=10,
        drop_target=GPSCoord(41.0, -81.0, 10.0),
        dropper=payload,
        lidar=lidar,
    )

    assert result is False
    assert payload.drop_calls == 0
    assert controller.hold_lidar is lidar
    assert controller.required_agl_m == 10.0




class InjectedFrameSource:
    def __init__(self, frame):
        self.frame = frame
        self.last_timestamp_ns = 123_000_000
        self.calls = []

    def capture_frame(self, quality=4):
        self.calls.append(quality)
        return self.frame


class UntimestampedFrameSource:
    def __init__(self, frame):
        self.frame = frame

    def capture_frame(self, quality=4):
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

    assert np.array_equal(actual, expected)
    assert not np.shares_memory(actual, expected)
    expected.fill(255)
    assert np.count_nonzero(actual) == 0
    assert source.calls == [4]
    assert manager.last_frame_timestamp == 123_000_000


def test_injected_camera_source_requires_a_real_timestamp(monkeypatch):
    """Regression: repeated injected frames must not receive synthetic freshness."""
    import cv2
    import numpy as np

    from drone.sensors.camera._camera_manager import CameraManager

    monkeypatch.setattr(
        cv2,
        "VideoCapture",
        lambda index: pytest.fail(f"opened hardware camera index {index}"),
    )
    source = UntimestampedFrameSource(
        np.zeros((480, 640, 3), dtype=np.uint8)
    )
    manager = CameraManager(frame_source=source)

    with pytest.raises(RuntimeError, match="timestamp"):
        manager.capture_frame()


def test_camera_accepts_an_injected_manager():
    """Regression: constructing Camera for simulation must not create hardware."""
    from drone.sensors.camera.camera import Camera

    manager = object()
    camera = Camera(50, manager=manager)

    assert camera.cm is manager




def test_active_fm3_requires_explicit_precision_policy_before_outputs():
    """A listener wiring omission cannot fall back to legacy flight constants."""
    active = importlib.import_module("drone.mock_mission")

    class OutputTrap:
        def __getattr__(self, name):
            raise AssertionError(f"FM3 produced output through {name}")

    with pytest.raises(ValueError, match="precision policy"):
        active.fm3(
            OutputTrap(),
            OutputTrap(),
            OutputTrap(),
            OutputTrap(),
            OutputTrap(),
            {3},
            GPSCoord(41.0, -81.0, 10.0),
            GPSCoord(41.1, -81.1, 10.0),
        )


def test_mock_mission_main_is_an_inert_route(capsys):
    """Direct execution must explain the guarded route without opening hardware."""
    with pytest.warns(RuntimeWarning, match="found in sys.modules"):
        with pytest.raises(SystemExit, match="guarded mission listener"):
            runpy.run_module("drone.mock_mission", run_name="__main__")

    assert capsys.readouterr().out == ""
