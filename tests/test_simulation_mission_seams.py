import importlib
import math
import runpy
import struct
import sys
from types import SimpleNamespace
from types import ModuleType

import pytest

from drone import timebase
from drone.common_types import GPSCoord, RelPosComplete
from drone.sensors.camera.camera import MarkerObservation


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


class TimedTouchdownLidar:
    def __init__(self, clock, touchdown_at=None):
        self.clock = clock
        self.touchdown_at = touchdown_at

    def get_distance(self):
        if self.touchdown_at is not None and self.clock.now() >= self.touchdown_at:
            return 0.0
        return 1.0


class LandingCamera:
    def vec_to_marker_3d(self, target_id, lidar_alt=None, quality=4):
        assert (target_id, quality) == (3, 4)
        return RelPosComplete(0.0, 0.0, lidar_alt)


class LandingController:
    def __init__(self):
        self.vehicle = SimpleNamespace(armed=True)
        self.targets = []
        self.land_mode_calls = 0

    def set_land_mode(self):
        self.land_mode_calls += 1

    def land_send_landing_target(self, update):
        self.targets.append(update)

    def is_landed(self):
        return False


class RecoveryLandingCamera:
    def __init__(self, clock, vectors):
        self.clock = clock
        self.vectors = list(vectors)
        self.calls = 0

    def observe_marker_3d(
        self, target_id, lidar_alt=None, quality=4, deadline_sim_ns=None
    ):
        assert (target_id, quality) == (3, 4)
        assert deadline_sim_ns is not None
        vector = self.vectors.pop(0) if self.vectors else None
        timestamp = int(self.clock.now() * 1_000_000_000)
        self.calls += 1
        return MarkerObservation(vector=vector, frame_timestamp_ns=timestamp)


class RecoveryLandingController:
    def __init__(self):
        self.vehicle = SimpleNamespace(
            armed=True,
            attitude=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0),
        )
        self.mode = "GUIDED"
        self.events = []
        self.targets = []

    def set_land_mode(self):
        self.mode = "LAND"
        self.events.append(("mode", "LAND"))
        return 0

    def set_guided_mode(self):
        self.mode = "GUIDED"
        self.events.append(("mode", "GUIDED"))
        return 0

    def get_current_gps(self):
        return GPSCoord(41.0, -81.0, 3.0)

    def get_location_metres(self, original, north, east):
        return GPSCoord(
            original.lat + north / 111_111.0,
            original.long + east / 84_000.0,
            original.alt,
        )

    def send_guided_waypoint(self, waypoint):
        self.events.append(("hold", self.mode, waypoint))
        return 0

    def land_send_landing_target(self, update):
        self.events.append(("target", self.mode, update))
        self.targets.append(update)
        return 0

    def is_landed(self):
        land_modes = [event for event in self.events if event == ("mode", "LAND")]
        return len(land_modes) >= 2 and bool(self.targets)


def test_half_second_without_healthy_target_holds_then_requests_retry():
    active = importlib.import_module("drone.mock_mission")
    clock = FakeClock()
    controller = RecoveryLandingController()

    with timebase.configured(clock):
        result = active.aruco_land_precision(
            controller,
            RecoveryLandingCamera(clock, [None] * 200),
            TimedTouchdownLidar(clock),
            3,
            GPSCoord(41.0, -81.0, 0.0),
            60.0,
            True,
        )

    assert result is active.LandingResult.RETRY
    assert controller.events[0] == ("mode", "LAND")
    guided_index = controller.events.index(("mode", "GUIDED"))
    assert all(event[0] != "target" for event in controller.events[guided_index:])
    holds = [event for event in controller.events if event[0] == "hold"]
    assert len(holds) >= 20
    assert all(event[1] == "GUIDED" for event in holds)
    assert len({(event[2].lat, event[2].long, event[2].alt) for event in holds}) == 1


def test_five_consecutive_healthy_hold_frames_resume_land_once():
    active = importlib.import_module("drone.mock_mission")
    clock = FakeClock()
    controller = RecoveryLandingController()
    centered = RelPosComplete(0.0, 0.0, 3.0)
    vectors = [None] * 12 + [centered] * 6

    with timebase.configured(clock):
        result = active.aruco_land_precision(
            controller,
            RecoveryLandingCamera(clock, vectors),
            TimedTouchdownLidar(clock),
            3,
            GPSCoord(41.0, -81.0, 0.0),
            60.0,
            True,
        )

    assert result is active.LandingResult.TOUCHDOWN
    assert [event for event in controller.events if event[0] == "mode"] == [
        ("mode", "LAND"),
        ("mode", "GUIDED"),
        ("mode", "LAND"),
    ]
    assert all(
        event[1] == "LAND"
        for event in controller.events
        if event[0] == "target"
    )


def test_reacquisition_count_resets_after_one_bad_frame():
    active = importlib.import_module("drone.mock_mission")
    clock = FakeClock()
    controller = RecoveryLandingController()
    centered = RelPosComplete(0.0, 0.0, 3.0)
    vectors = [None] * 12 + [centered] * 4 + [None] + [centered] * 6

    with timebase.configured(clock):
        result = active.aruco_land_precision(
            controller,
            RecoveryLandingCamera(clock, vectors),
            TimedTouchdownLidar(clock),
            3,
            GPSCoord(41.0, -81.0, 0.0),
            60.0,
            True,
        )

    assert result is active.LandingResult.TOUCHDOWN
    assert len([event for event in controller.events if event == ("mode", "LAND")]) == 2


def test_marker_loss_below_precision_minimum_keeps_land_until_touchdown():
    active = importlib.import_module("drone.mock_mission")
    clock = FakeClock()
    controller = RecoveryLandingController()

    class NearGroundLidar:
        def get_distance(self):
            return 0.0 if clock.now() >= 1.0 else 0.10

    with timebase.configured(clock):
        result = active.aruco_land_precision(
            controller,
            RecoveryLandingCamera(clock, [None] * 30),
            NearGroundLidar(),
            3,
            GPSCoord(41.0, -81.0, 0.0),
            5.0,
            True,
        )

    assert result is active.LandingResult.TOUCHDOWN
    assert [event for event in controller.events if event[0] == "mode"] == [
        ("mode", "LAND")
    ]


def test_rejected_observation_is_compact_sorted_json(capsys):
    active = importlib.import_module("drone.mock_mission")
    clock = FakeClock()
    controller = RecoveryLandingController()

    with timebase.configured(clock):
        active.aruco_land_precision(
            controller,
            RecoveryLandingCamera(clock, [RelPosComplete(float("nan"), 0.0, 3.0)]),
            TimedTouchdownLidar(clock),
            3,
            GPSCoord(41.0, -81.0, 0.0),
            0.1,
            True,
        )

    records = [
        line.removeprefix("PRECISION_LANDING ")
        for line in capsys.readouterr().out.splitlines()
        if line.startswith("PRECISION_LANDING ")
    ]
    assert records
    assert any('"reason":"non_finite"' in record for record in records)
    assert all(" " not in record for record in records)


class AutoDisarmLandingController(LandingController):
    def get_current_gps(self):
        return GPSCoord(0.0, 0.0, 0.20)


class AutoDisarmingLandingCamera(LandingCamera):
    def __init__(self, controller):
        self.controller = controller

    def vec_to_marker_3d(self, target_id, lidar_alt=None, quality=4):
        self.controller.vehicle.armed = False
        return super().vec_to_marker_3d(target_id, lidar_alt, quality)


class StaleAfterInitialLandingRange:
    def __init__(self):
        self.calls = 0

    def get_distance(self):
        self.calls += 1
        if self.calls == 1:
            return 1.45
        raise RuntimeError("downward range is older than 0.5 simulated seconds")


def test_precision_land_accepts_auto_disarm_when_range_and_landed_state_lag():
    active = importlib.import_module("drone.mock_mission")
    clock = FakeClock()
    controller = AutoDisarmLandingController()

    with timebase.configured(clock):
        result = active.aruco_land_precision(
            controller,
            AutoDisarmingLandingCamera(controller),
            StaleAfterInitialLandingRange(),
            3,
        )

    assert result is True


def test_precision_land_confirms_touchdown_after_old_iteration_cap():
    """A physical touchdown at 55s is inside the declared 60s allowance."""
    active = importlib.import_module("drone.mock_mission")
    clock = FakeClock()
    controller = LandingController()

    with timebase.configured(clock):
        result = active.aruco_land_precision(
            controller, LandingCamera(), TimedTouchdownLidar(clock, 55.0), 3
        )

    assert result is True
    assert 55.0 <= clock.now_value <= 55.30


def test_precision_land_without_touchdown_uses_full_timeout_and_fails_closed():
    active = importlib.import_module("drone.mock_mission")
    clock = FakeClock()
    controller = LandingController()

    with timebase.configured(clock):
        result = active.aruco_land_precision(
            controller, LandingCamera(), TimedTouchdownLidar(clock), 3
        )

    assert result is False
    assert clock.now_value == pytest.approx(60.05, abs=0.06)


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


class DeniedDisarmThenAutoDisarmVehicle:
    """Model the preserved L-pad timing around ArduPilot auto-disarm."""

    def __init__(self, clock):
        self.clock = clock
        self.touchdown_at = 14.632541
        self.auto_disarm_delay = 0.506915

    @property
    def armed(self):
        return self.clock.now() < self.touchdown_at + self.auto_disarm_delay

    @armed.setter
    def armed(self, value):
        if value is not False:
            raise AssertionError("the focused fixture only accepts a disarm request")
        # The first request is denied while airborne; ArduPilot auto-disarms
        # after touchdown from its own physical state.


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
    controller = controller_without_connect(vehicle)

    with timebase.configured(clock):
        assert controller.disarm() == 0

    assert vehicle.armed is False
    assert clock.now_value == pytest.approx(3.0, abs=0.11)
    assert clock.sleeps


def test_disarm_returns_failure_after_sixteen_simulated_seconds():
    """Regression: an unconfirmed disarm must not block forever or report success."""
    clock = FakeClock()
    controller = controller_without_connect(
        DelayedDisarmVehicle(clock, confirmation_delay=None)
    )

    with timebase.configured(clock):
        assert controller.disarm() == -1

    assert clock.now_value == pytest.approx(16.0, abs=1e-12)


def test_disarm_observes_auto_disarm_just_after_fifteen_simulated_seconds():
    """A denied airborne request must still observe the physical L auto-disarm."""
    clock = FakeClock()
    controller = controller_without_connect(
        DeniedDisarmThenAutoDisarmVehicle(clock)
    )

    with timebase.configured(clock):
        assert controller.disarm() == 0

    assert controller.vehicle.armed is False
    assert clock.now_value == pytest.approx(15.2, abs=1e-12)


class RecordingMissionMav:
    def __init__(self):
        self.items = []

    def mission_item_int_send(self, *args):
        self.items.append(args)


class StableVehicle:
    def __init__(self, clock):
        self.clock = clock
        self.location = SimpleNamespace(
            global_relative_frame=SimpleNamespace(lat=41.0, lon=-81.0, alt=10.0)
        )
        self.attitude = SimpleNamespace(pitch=0.0)
        self.targets = []
        self.groundspeed = None
        self.mission_mav = RecordingMissionMav()
        self._master = SimpleNamespace(mav=self.mission_mav)

    @property
    def velocity(self):
        speed = 0.100001 if 0.8 <= self.clock.now() <= 1.0 else 0.10
        return (speed, 0.0, 0.0)

    def simple_goto(self, target):
        self.targets.append(target)


def test_guided_waypoint_uses_stable_speed_and_exact_mission_item_int_coordinates():
    """Guided transit must avoid the observed unstable 20 m/s approach."""
    from drone.control.drone_control import goto
    from pymavlink import mavutil

    target_lat = 37.4003371
    target_lon = -122.08175843013008
    vehicle = StableVehicle(FakeClock())

    goto(vehicle, target_lat, target_lon, 10.0)

    assert vehicle.groundspeed == 10.0
    assert vehicle.targets == []
    assert vehicle.mission_mav.items == [
        (
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,
            0,
            0,
            0,
            0,
            0,
            374003371,
            -1220817584,
            10.0,
        )
    ]

    legacy_lon = struct.unpack("f", struct.pack("f", target_lon))[0]
    legacy = GPSCoord(target_lat, legacy_lon, 0)
    exact = GPSCoord(target_lat, -1220817584 / 1e7, 0)
    target = GPSCoord(target_lat, target_lon, 0)
    assert horiz_distance_m_for_test(legacy, target) > 0.15
    assert horiz_distance_m_for_test(exact, target) < 0.01


def test_nonblocking_guided_waypoint_preserves_integer_coordinates():
    vehicle = StableVehicle(FakeClock())
    controller = controller_without_connect(vehicle)

    assert (
        controller.send_guided_waypoint(
            GPSCoord(41.12345678, -81.87654321, 4.572)
        )
        == 0
    )

    assert vehicle.mission_mav.items[-1][11:14] == (
        411234568,
        -818765432,
        4.572,
    )


def valid_precision_landing_profile():
    return {
        "LAND_SPD_MS": 0.50,
        "PLND_ENABLED": 1,
        "PLND_TYPE": 1,
        "PLND_EST_TYPE": 0,
        "PLND_LAG": 0.08,
        "PLND_XY_DIST_MAX": 0.50,
        "PLND_STRICT": 2,
        "PLND_RET_MAX": 1,
        "PLND_TIMEOUT": 0.50,
        "PLND_ALT_MIN": 0.75,
        "PLND_ALT_MAX": 8.0,
        "PLND_OPTIONS": 4,
    }


class ParameterVehicle:
    def __init__(self, parameters):
        self.parameters = parameters


class ConfirmingModeVehicle:
    def __init__(self):
        self._mode = SimpleNamespace(name="GUIDED")
        self.mode_assignments = 0

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, requested):
        self.mode_assignments += 1
        if self.mode_assignments >= 2:
            self._mode = SimpleNamespace(name=requested.name)


def test_land_mode_waits_for_vehicle_confirmation():
    clock = FakeClock()
    vehicle = ConfirmingModeVehicle()
    controller = controller_without_connect(vehicle)

    with timebase.configured(clock):
        assert controller.set_land_mode() == 0

    assert vehicle.mode.name == "LAND"
    assert vehicle.mode_assignments == 2
    assert clock.now_value == pytest.approx(2.0)


def test_precision_landing_profile_accepts_exact_runtime_values():
    controller = controller_without_connect(
        ParameterVehicle(valid_precision_landing_profile())
    )

    assert controller.require_precision_landing_profile() is True


@pytest.mark.parametrize(
    ("name", "value"),
    (("PLND_OPTIONS", 0), ("LAND_SPD_MS", 0.10), ("PLND_ENABLED", 1.5)),
)
def test_precision_landing_profile_rejects_runtime_mismatch(name, value):
    parameters = valid_precision_landing_profile()
    parameters[name] = value
    controller = controller_without_connect(ParameterVehicle(parameters))

    assert controller.require_precision_landing_profile() is False


def test_precision_landing_profile_rejects_missing_runtime_parameter():
    parameters = valid_precision_landing_profile()
    del parameters["PLND_OPTIONS"]
    controller = controller_without_connect(ParameterVehicle(parameters))

    assert controller.require_precision_landing_profile() is False


def horiz_distance_m_for_test(a, b):
    from drone.control.drone_control import horiz_distance_m

    return horiz_distance_m(a, b)


def test_release_hold_reissues_exact_integer_waypoint_every_two_tenths():
    from pymavlink import mavutil

    clock = FakeClock()
    vehicle = StableVehicle(clock)
    controller = controller_without_connect(vehicle)

    with timebase.configured(clock):
        stable = controller.hold_waypoint_until_stable(
            GPSCoord(37.4003371, -122.08175843013008, 10.0),
            FakeLidar(),
            required_agl_m=10.0,
            hold_seconds=0.4,
            timeout=1.0,
        )

    assert stable is False
    assert clock.sleeps == [0.2] * 5
    assert vehicle.targets == []
    assert len(vehicle.mission_mav.items) == 5
    assert all(
        item[3] == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        and item[4] == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        and item[5] == 2
        and item[11:14] == (374003371, -1220817584, 10.0)
        for item in vehicle.mission_mav.items
    )


def test_release_stability_uses_horizontal_speed_and_resets_continuous_window():
    """Regression: pitch or an earlier quiet sample must not authorize release."""
    clock = FakeClock()
    controller = controller_without_connect(StableVehicle(clock))

    with timebase.configured(clock):
        stable = controller.hold_waypoint_until_stable(
            GPSCoord(41.0, -81.0, 10.0),
            FakeLidar(),
            required_agl_m=10.0,
            timeout=4.0,
        )

    assert stable is True
    assert clock.now_value == pytest.approx(3.2, abs=0.21)


def test_release_stability_uses_lidar_agl_not_adjusted_navigation_altitude():
    """Regression: terrain-corrected navigation altitude is not lidar AGL."""
    clock = FakeClock()
    vehicle = StableVehicle(clock)
    controller = controller_without_connect(vehicle)

    with timebase.configured(clock):
        stable = controller.hold_waypoint_until_stable(
            GPSCoord(41.0, -81.0, 15.0),
            FakeLidar(),
            required_agl_m=10.0,
            timeout=4.0,
        )

    assert stable is True
    assert vehicle.mission_mav.items
    assert all(item[13] == 15.0 for item in vehicle.mission_mav.items)


def test_release_stability_resets_below_ten_metres_agl_after_terrain_adjustment():
    """Regression: quiet flight below 10 m AGL must reset the hold window."""
    clock = FakeClock()
    controller = controller_without_connect(StableVehicle(clock))

    class RisingLidar:
        def get_distance(self):
            return 9.999999 if clock.now() <= 1.0 else 10.0

    with timebase.configured(clock):
        stable = controller.hold_waypoint_until_stable(
            GPSCoord(41.0, -81.0, 15.0),
            RisingLidar(),
            required_agl_m=10.0,
            timeout=4.0,
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


class RecordingCamera:
    def save_frame_buffer_async(self):
        pass


class TrackerWithTime:
    def __init__(self, seconds=500.0):
        self.seconds = seconds

    def time_left(self):
        return self.seconds


def test_fm3_refuses_release_when_stability_gate_times_out(monkeypatch):
    """Regression: a failed two-second gate must suppress FM3 release."""
    active = importlib.import_module("drone.mock_mission")
    monkeypatch.setattr(active, "pickup_sequence", lambda *args: True)
    payload = RecordingPayload()

    controller = ReleaseController(stable=False)
    result = active.fm3(
        TrackerWithTime(),
        controller,
        RecordingCamera(),
        FakeLidar(),
        payload,
        {3},
        GPSCoord(41.0, -81.0, 10.0),
        GPSCoord(41.1, -81.1, 10.0),
    )

    assert result is False
    assert payload.drop_calls == 0
    assert controller.required_agl_m == 10.0


def test_fm3_low_time_exit_is_explicit_failure():
    """Regression: a skipped FM3 phase must not look successful to its caller."""
    active = importlib.import_module("drone.mock_mission")

    result = active.fm3(
        TrackerWithTime(59.0),
        ReleaseController(stable=True),
        RecordingCamera(),
        FakeLidar(),
        RecordingPayload(),
        {3},
        GPSCoord(41.0, -81.0, 10.0),
        GPSCoord(41.1, -81.1, 10.0),
    )

    assert result is False


class FailingMissionController:
    def __init__(self):
        self.rtl_calls = 0

    def goto_waypoint(self, waypoint):
        raise RuntimeError("navigation failed")

    def rtl(self):
        self.rtl_calls += 1


class SavingCamera:
    def __init__(self):
        self.save_calls = 0

    def save_frame_buffer_async(self):
        self.save_calls += 1


def test_fm3_exception_path_is_explicit_failure():
    """Regression: caught mission exceptions must not become FM3 completion."""
    active = importlib.import_module("drone.mock_mission")
    controller = FailingMissionController()
    camera = SavingCamera()

    result = active.fm3(
        TrackerWithTime(),
        controller,
        camera,
        FakeLidar(),
        RecordingPayload(),
        {3},
        GPSCoord(41.0, -81.0, 10.0),
        GPSCoord(41.1, -81.1, 10.0),
    )

    assert result is False
    assert controller.rtl_calls == 1
    assert camera.save_calls == 1


class FailedGotoController(ReleaseController):
    def goto_waypoint(self, waypoint, position_tol=None):
        return -1


def test_fm3_navigation_failure_is_explicit_failure(monkeypatch):
    """Regression: a waypoint timeout must not become FM3 success."""
    active = importlib.import_module("drone.mock_mission")
    monkeypatch.setattr(active, "pickup_sequence", lambda *args: True)

    result = active.fm3(
        TrackerWithTime(),
        FailedGotoController(stable=True),
        RecordingCamera(),
        FakeLidar(),
        RecordingPayload(),
        {3},
        GPSCoord(41.0, -81.0, 10.0),
        GPSCoord(41.1, -81.1, 10.0),
    )

    assert result is False


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
            attitude=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0),
            flush=lambda: None,
        )

    def set_guided_mode(self):
        self.events.append(("mode", "GUIDED"))
        return 0

    def require_precision_landing_profile(self):
        self.events.append(("profile", 3))
        return True

    def climb(self, altitude):
        self.events.append(("climb", altitude))

    def guide_move_relative_frame(self, direction):
        self.events.append(("relative_down", direction.z))
        return 0

    def get_current_gps(self):
        return GPSCoord(41.0, -81.0, 10.0)

    def get_location_metres(self, original, north, east):
        return GPSCoord(original.lat, original.long, original.alt)

    def goto_waypoint(self, waypoint, position_tol=None):
        self.events.append(("goto", waypoint.alt, position_tol))
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

    def hold_waypoint_until_stable(self, waypoint, lidar):
        return True

    def rtl(self):
        self.events.append(("rtl", None))


def centered_updates(timestamps):
    return [(timestamp, RelPosComplete(0.5, 0.0, 4.572)) for timestamp in timestamps]


class TiltedPickupController(PickupController):
    def __init__(self, events):
        super().__init__(events)
        self.vehicle.attitude = SimpleNamespace(roll=0.2, pitch=0.0, yaw=0.0)
        self.location_offsets = []

    def get_location_metres(self, original, north, east):
        self.location_offsets.append((north, east))
        return super().get_location_metres(original, north, east)


class RecenterPickupController(PickupController):
    def __init__(self, events, recenter_result=0):
        super().__init__(events)
        self.recenter_result = recenter_result
        self.location_offsets = []
        self.goto_tolerances = []

    def get_location_metres(self, original, north, east):
        self.location_offsets.append((north, east))
        return super().get_location_metres(original, north, east)

    def goto_waypoint(self, waypoint, position_tol=None):
        self.goto_tolerances.append(position_tol)
        self.events.append(("goto", waypoint.alt, position_tol))
        if len(self.goto_tolerances) == 3:
            return self.recenter_result
        return 0


def test_marker_offset_level_identity():
    active = importlib.import_module("drone.mock_mission")

    north, east = active._marker_offset_ne(
        RelPosComplete(1.2, -0.4, 5.0),
        SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0),
    )

    assert (north, east) == pytest.approx((1.2, -0.4))


def test_marker_offset_uses_roll_pitch_and_yaw():
    active = importlib.import_module("drone.mock_mission")

    north, east = active._marker_offset_ne(
        RelPosComplete(1.2, -0.4, 5.0),
        SimpleNamespace(roll=0.2, pitch=-0.1, yaw=0.3),
    )

    assert (north, east) == pytest.approx(
        (1.0902947109674521, -1.1128740280589786)
    )


class PickupLidar:
    def __init__(self, acquisition_values=None):
        self.calls = 0
        self.acquisition_values = list(
            acquisition_values
            if acquisition_values is not None
            else [4.572] * 20
        )

    def get_distance(self):
        self.calls += 1
        if self.calls == 1:
            return 10.0
        if self.acquisition_values:
            return self.acquisition_values.pop(0)
        return 4.572


class ScriptedPickupLidar:
    def __init__(self, results):
        self.results = list(results)

    def get_distance(self):
        result = self.results.pop(0)
        if isinstance(result, Exception):
            raise result
        return result


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
    payload = RecordingPayload(attach_result=True, events=events)

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events), camera, PickupLidar(), 3, payload
        )

    assert result is True
    assert camera.consumed_timestamps == [1, 2, 3, 4, 5, 6]
    assert events[-3:] == [("landed", 3), ("disarm", 3), ("attach", 3)]


def test_pickup_refuses_land_when_runtime_profile_is_wrong(monkeypatch):
    active = importlib.import_module("drone.mock_mission")
    events = []
    controller = PickupController(events)
    controller.require_precision_landing_profile = lambda: False
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: pytest.fail("LAND must not start with a mismatched profile"),
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            controller,
            AcquisitionCamera(centered_updates([1, 2, 3, 4, 5, 6])),
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False
    assert ("disarm", 3) not in events
    assert ("attach", 3) not in events


def test_pickup_reacquires_once_after_hold_timeout(monkeypatch):
    active = importlib.import_module("drone.mock_mission")
    events = []
    results = iter((active.LandingResult.RETRY, active.LandingResult.TOUCHDOWN))
    anchors = []

    def land(*args):
        anchors.append(args[4])
        return next(results)

    monkeypatch.setattr(active, "aruco_land_precision", land)
    camera = AcquisitionCamera(centered_updates(range(1, 13)))

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is True
    assert len(anchors) == 2
    assert any(
        event[0] == "goto" and event[1] == pytest.approx(4.572)
        for event in events
    )
    assert events[-2:] == [("disarm", 3), ("attach", 3)]


def test_second_hold_timeout_fails_without_disarm_or_attach(monkeypatch):
    active = importlib.import_module("drone.mock_mission")
    events = []
    landing_calls = []

    def retry(*args):
        landing_calls.append(args[3])
        return active.LandingResult.RETRY

    monkeypatch.setattr(active, "aruco_land_precision", retry)

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            AcquisitionCamera(centered_updates(range(1, 13))),
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False
    assert landing_calls == [3, 3]
    assert ("disarm", 3) not in events
    assert ("attach", 3) not in events


def test_acquisition_anchor_uses_coordinate_median_and_rejects_wide_set():
    active = importlib.import_module("drone.mock_mission")
    clustered = [
        GPSCoord(41.0 + north / 111_111.0, -81.0, 0.0)
        for north in (0.09, -0.04, 0.0, 0.04, -0.09)
    ]
    wide = clustered[:-1] + [GPSCoord(41.0 + 0.31 / 111_111.0, -81.0, 0.0)]

    anchor = active._median_anchor(clustered)

    assert anchor is not None
    assert anchor.lat == pytest.approx(41.0)
    assert anchor.long == pytest.approx(-81.0)
    assert active._median_anchor(wide) is None


def test_pickup_damps_one_recenter_before_five_later_centered_results(monkeypatch):
    """A noisy fresh offset must drive one damped second correction."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(
        [
            (1, RelPosComplete(0.1, 0.0, 4.572)),
            (2, RelPosComplete(0.6, 0.0, 4.572)),
            (3, RelPosComplete(0.4, 0.0, 4.572)),
            (4, RelPosComplete(0.4, 0.0, 4.572)),
            (5, RelPosComplete(0.4, 0.0, 4.572)),
            (6, RelPosComplete(0.4, 0.0, 4.572)),
            (7, RelPosComplete(0.4, 0.0, 4.572)),
        ]
    )
    controller = RecenterPickupController(events)
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: events.append(("landed", 3)) or True,
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            controller,
            camera,
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is True
    assert controller.goto_tolerances == [0.15, 0.15, 0.15]
    assert controller.location_offsets == [(0, 0), (0.1, 0.0), (0.18, 0.0)]
    assert camera.consumed_timestamps == [1, 2, 3, 4, 5, 6, 7]
    assert events[-3:] == [("landed", 3), ("disarm", 3), ("attach", 3)]


def test_pickup_failed_recenter_cannot_land_or_attach(monkeypatch):
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(
        [
            (1, RelPosComplete(0.1, 0.0, 4.572)),
            (2, RelPosComplete(0.6, 0.0, 4.572)),
            (3, RelPosComplete(0.4, 0.0, 4.572)),
            (4, RelPosComplete(0.4, 0.0, 4.572)),
            (5, RelPosComplete(0.4, 0.0, 4.572)),
            (6, RelPosComplete(0.4, 0.0, 4.572)),
            (7, RelPosComplete(0.4, 0.0, 4.572)),
        ]
    )
    controller = RecenterPickupController(events, recenter_result=-1)
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: events.append(("landed", 3)) or True,
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            controller,
            camera,
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False
    assert controller.goto_tolerances == [0.15, 0.15, 0.15]
    assert camera.consumed_timestamps == [1, 2]
    assert ("landed", 3) not in events
    assert ("attach", 3) not in events


def test_pickup_recenter_still_requires_five_later_centered_results(monkeypatch):
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(
        [
            (1, RelPosComplete(0.1, 0.0, 4.572)),
            (2, RelPosComplete(0.6, 0.0, 4.572)),
            (3, RelPosComplete(0.4, 0.0, 4.572)),
            (4, RelPosComplete(0.4, 0.0, 4.572)),
            (5, RelPosComplete(0.4, 0.0, 4.572)),
            (6, RelPosComplete(0.4, 0.0, 4.572)),
        ]
    )
    controller = RecenterPickupController(events)
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: events.append(("landed", 3)) or True,
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            controller,
            camera,
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False
    assert controller.goto_tolerances == [0.15, 0.15, 0.15]
    assert camera.consumed_timestamps == [1, 2, 3, 4, 5, 6]
    assert ("landed", 3) not in events
    assert ("attach", 3) not in events


def test_pickup_compensates_tilt_for_correction_and_centered_gate(monkeypatch):
    """A tilted view of an earth-centered marker must remain earth-centered."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    body_right = math.sin(0.2) * 5.0
    body_down = math.cos(0.2) * 5.0
    camera = AcquisitionCamera(
        [
            (timestamp, RelPosComplete(0.0, body_right, body_down))
            for timestamp in [1, 2, 3, 4, 5, 6]
        ]
    )
    controller = TiltedPickupController(events)
    monkeypatch.setattr(active, "aruco_land_precision", lambda *args: True)

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            controller,
            camera,
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is True
    assert controller.location_offsets[1] == pytest.approx((0.0, 0.0), abs=1e-9)


def test_pickup_uses_precision_tolerance_for_search_and_marker_correction(monkeypatch):
    """Regression: search arrival must not leave the drone outside acquisition range."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 2, 3, 4, 5, 6]))
    monkeypatch.setattr(active, "aruco_land_precision", lambda *args: True)

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is True
    assert [event[2] for event in events if event[0] == "goto"] == [0.15, 0.15]


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
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False
    assert camera.consumed_timestamps == [1, 2, 2, 2, 2]
    assert not any(event[0] == "landed" for event in events)


def test_pre_correction_frame_cannot_count_again_for_acquisition(monkeypatch):
    """Regression: the correction frame must not be one of five fresh results."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 1, 2, 3, 4, 5]))
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: events.append(("landed", 3)) or True,
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False
    assert camera.consumed_timestamps == [1, 1, 2, 3, 4, 5]
    assert not any(event[0] == "landed" for event in events)


def test_alternating_replayed_frames_cannot_acquire(monkeypatch):
    """Regression: non-adjacent timestamp replay must not advance acquisition."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 2, 3, 2, 3, 4, 5]))
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: events.append(("landed", 3)) or True,
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            PickupLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False
    assert camera.consumed_timestamps == [1, 2, 3, 2, 3, 4, 5]
    assert not any(event[0] == "landed" for event in events)


@pytest.mark.parametrize("attach_result", [None, 0, False])
def test_rejected_attachment_prevents_fm3_takeoff(monkeypatch, attach_result):
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
            PickupLidar(),
            RecordingPayload(attach_result=attach_result, events=events),
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


def test_pickup_fails_when_lidar_never_reaches_acquisition_agl(monkeypatch):
    """Regression: commanding altitude is not proof of 4.572 m AGL."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 2, 3, 4, 5, 6]))
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: pytest.fail("LAND must not start above acquisition AGL"),
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            FakeLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False
    assert ("relative_down", 10.0 - 4.572) in events


def test_transient_stale_hover_range_waits_for_fresh_acquisition_agl(monkeypatch):
    """A callback-lagged hover sample must not abort the bounded AGL wait."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 2, 3, 4, 5, 6]))
    stale = RuntimeError("downward range is older than 0.5 simulated seconds")
    lidar = ScriptedPickupLidar(
        [10.0, 6.0, stale, 5.5, 4.572, 4.572, 4.572, 4.572, 4.572]
    )
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: events.append(("landed", 3)) or True,
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            lidar,
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is True
    assert events[-3:] == [("landed", 3), ("disarm", 3), ("attach", 3)]


def test_persistently_stale_hover_range_uses_timeout_and_fails_closed(monkeypatch):
    """No fresh hover AGL may advance to marker search or LAND."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    clock = FakeClock()
    stale = RuntimeError("downward range is older than 0.5 simulated seconds")

    class PersistentlyStaleHoverLidar:
        def __init__(self):
            self.calls = 0

        def get_distance(self):
            self.calls += 1
            if self.calls == 1:
                return 10.0
            raise stale

    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: pytest.fail("LAND requires a fresh acquisition AGL"),
    )

    with timebase.configured(clock):
        result = active.pickup_sequence(
            PickupController(events),
            AcquisitionCamera(centered_updates([1, 2, 3, 4, 5, 6])),
            PersistentlyStaleHoverLidar(),
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False
    assert clock.now_value == pytest.approx(60.0, abs=0.11)
    assert not any(event[0] == "landed" for event in events)


def test_pickup_agl_must_remain_valid_for_five_results(monkeypatch):
    """Regression: an AGL excursion must reset the acquisition window."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 2, 3, 4, 5, 6]))
    lidar = PickupLidar(
        acquisition_values=[4.572, 4.572, 4.572, 6.0, 4.572, 4.572]
    )
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: pytest.fail("LAND requires five continuous AGL samples"),
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            lidar,
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False


def test_pickup_corrects_acquisition_agl_drift_before_counting_results(monkeypatch):
    """A post-centering AGL drift must command a return to 4.572 m."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 2, 3, 4, 5, 6, 7]))
    lidar = PickupLidar(
        acquisition_values=[4.572, 5.705, 4.572, 4.572, 4.572, 4.572, 4.572]
    )
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: events.append(("landed", 3)) or True,
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            lidar,
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is True
    assert ("relative_down", pytest.approx(1.133)) in events
    assert events[-3:] == [("landed", 3), ("disarm", 3), ("attach", 3)]


def test_transient_stale_acquisition_range_resets_window_without_aborting(monkeypatch):
    """Regression: one callback-lagged range must not abort the whole pickup."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 2, 3, 4, 5, 6, 7, 8]))
    lidar = ScriptedPickupLidar(
        [
            10.0,
            4.572,
            4.572,
            RuntimeError("downward range is older than 0.5 simulated seconds"),
            4.572,
            4.572,
            4.572,
            4.572,
            4.572,
        ]
    )
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: events.append(("landed", 3)) or True,
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            lidar,
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is True
    assert camera.consumed_timestamps == [1, 2, 3, 4, 5, 6, 7, 8]
    assert events[-3:] == [("landed", 3), ("disarm", 3), ("attach", 3)]


def test_persistently_stale_acquisition_range_still_fails_closed(monkeypatch):
    """Regression: no physically current range must never trigger LAND."""
    active = importlib.import_module("drone.mock_mission")
    events = []
    camera = AcquisitionCamera(centered_updates([1, 2, 3, 4, 5, 6]))
    stale = RuntimeError("downward range is older than 0.5 simulated seconds")
    lidar = ScriptedPickupLidar([10.0, 4.572, stale, stale, stale, stale, stale])
    monkeypatch.setattr(
        active,
        "aruco_land_precision",
        lambda *args: pytest.fail("LAND requires physically current range"),
    )

    with timebase.configured(FakeClock()):
        result = active.pickup_sequence(
            PickupController(events),
            camera,
            lidar,
            3,
            RecordingPayload(attach_result=True, events=events),
        )

    assert result is False
    assert not any(event[0] == "landed" for event in events)


class InjectedFrameSource:
    def __init__(self, frame):
        self.frame = frame
        self.last_timestamp_ns = 123_000_000
        self.calls = []

    def capture_frame(self, quality=4, deadline_sim_ns=None):
        self.calls.append((quality, deadline_sim_ns))
        return self.frame


class UntimestampedFrameSource:
    def __init__(self, frame):
        self.frame = frame

    def capture_frame(self, quality=4, deadline_sim_ns=None):
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
    assert source.calls == [(4, None)]
    assert manager.last_frame_timestamp == 123_000_000


def test_injected_camera_capture_propagates_simulation_deadline(monkeypatch, tmp_path):
    """A silent ROS camera must be bounded by the mission's simulated deadline."""
    import cv2
    import numpy as np

    from drone.sensors.camera._camera_manager import CameraManager

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(
        cv2,
        "VideoCapture",
        lambda index: pytest.fail(f"opened hardware camera index {index}"),
    )
    source = InjectedFrameSource(np.zeros((480, 640, 3), dtype=np.uint8))

    CameraManager(frame_source=source).capture_frame(deadline_sim_ns=125)

    assert source.calls == [(4, 125)]


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


def test_mock_mission_main_imports_hardware_classes_locally(monkeypatch):
    """Regression: type-only imports must not break the direct mission entry."""
    import drone.control.drone_control as control_module
    import drone.sensors.camera.camera as camera_module

    reached_lidar_constructor = []

    class MainController:
        def __init__(self, connection_port):
            pass

    class MainCamera:
        def __init__(self, marker_size_mm):
            pass

    class MainLidar:
        def __init__(self):
            reached_lidar_constructor.append(True)
            raise RuntimeError("stop main after local import")

    lidar_module = ModuleType("drone.sensors.lidar.lidar")
    lidar_module.Lidar = MainLidar  # type: ignore[attr-defined]
    servo_module = ModuleType("drone.sensors.servo.servo")
    servo_module.Dropper = object  # type: ignore[attr-defined]

    monkeypatch.setattr(control_module, "DroneControl", MainController)
    monkeypatch.setattr(camera_module, "Camera", MainCamera)
    monkeypatch.setitem(sys.modules, "drone.sensors.lidar.lidar", lidar_module)
    monkeypatch.setitem(sys.modules, "drone.sensors.servo.servo", servo_module)
    monkeypatch.delitem(sys.modules, "drone.mock_mission", raising=False)

    with pytest.raises(SystemExit):
        runpy.run_module("drone.mock_mission", run_name="__main__")

    assert reached_lidar_constructor == [True]
