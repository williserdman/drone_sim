import math
import queue
import threading

import pytest

from drone.common_types import GPSCoord
from drone.control import drone_control
from drone.control.drone_control import DroneControl, MissionAckTracker
from dronekit.mavlink import MAVWriter
from drone.control.flight_state import FlightState, RCModeBand, SourceIdentity
from drone.control.mission_supervisor import AuthorityLost, FlightOperationError
from drone.control.stability import ReleaseStabilityConfig
from drone.sensors.lidar.clearance import ClearanceCalibration
from drone.sensors.lidar.lidar import LidarSample, StaleSensorError
from drone.sensors.servo.servo import Dropper as HardwareDropper


VALID_CONFIG = {
    "hold_seconds": 0.4,
    "timeout_seconds": 2.0,
    "max_horizontal_speed_m_s": 0.2,
    "max_vertical_speed_m_s": 0.1,
    "max_roll_rad": 0.2,
    "max_pitch_rad": 0.2,
    "horizontal_position_tolerance_m": 0.5,
    "vertical_position_tolerance_m": 0.4,
    "max_observation_skew_seconds": 0.1,
    "max_observation_gap_seconds": 0.3,
    "poll_interval_seconds": 0.05,
    "waypoint_reissue_interval_seconds": 0.2,
}


def test_release_stability_config_requires_every_finite_positive_threshold():
    for field, bad_value in (
        ("hold_seconds", None),
        ("timeout_seconds", False),
        ("max_horizontal_speed_m_s", -0.1),
        ("max_vertical_speed_m_s", float("nan")),
        ("max_roll_rad", 0.0),
        ("max_pitch_rad", math.inf),
        ("horizontal_position_tolerance_m", 0.0),
        ("vertical_position_tolerance_m", -1.0),
        ("max_observation_skew_seconds", 0.0),
        ("max_observation_gap_seconds", 0.0),
        ("poll_interval_seconds", 0.0),
        ("waypoint_reissue_interval_seconds", 0.0),
    ):
        values = dict(VALID_CONFIG)
        values[field] = bad_value
        with pytest.raises(ValueError, match=field):
            ReleaseStabilityConfig(**values)


def test_release_stability_config_rejects_impossible_timing_policy():
    values = dict(VALID_CONFIG, timeout_seconds=VALID_CONFIG["hold_seconds"])
    with pytest.raises(ValueError, match="timeout_seconds"):
        ReleaseStabilityConfig(**values)

    values = dict(VALID_CONFIG, max_observation_gap_seconds=0.04)
    with pytest.raises(ValueError, match="poll_interval_seconds"):
        ReleaseStabilityConfig(**values)


class Clock:
    def __init__(self):
        self.value = 10.0
        self.callbacks = []

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds
        for callback in tuple(self.callbacks):
            callback()


class Mav:
    def __init__(self):
        self.waypoints = []
        self.mission_callback = lambda: None

    def mission_item_int_send(self, *fields):
        self.waypoints.append(fields)
        self.mission_callback()


class Vehicle:
    def __init__(self):
        from types import SimpleNamespace

        self._master = SimpleNamespace(mav=Mav())


class SampleLidar:
    def __init__(self, clock, distance=10.0):
        self.clock = clock
        self.distance = distance
        self.sequence = 0
        self.generation = 0
        self.available = True
        self.sample = None
        self.publish()

    def publish(self):
        self.sequence += 1
        self.sample = LidarSample(
            self.distance, self.clock.now(), self.sequence, self.generation
        )

    def invalidate_between_reads(self):
        self.generation += 1
        self.publish()

    def get_sample(self):
        if not self.available:
            raise StaleSensorError("range unavailable")
        return self.sample


class Dropper:
    def __init__(self):
        self.calls = 0

    def drop(self):
        self.calls += 1


class RecordingServo:
    def __init__(self, events):
        self.events = events

    def max(self):
        self.events.append("max")

    def mid(self):
        self.events.append("mid")


def calibration(**changes):
    values = {
        "beam_direction_body_frd": (0.0, 0.0, 1.0),
        "measured_reference_offset_body_frd_m": (0.0, 0.0, 0.0),
        "lidar_mounting_offset_already_applied": True,
        "max_tilt_rad": 0.3,
        "max_age_seconds": 0.15,
        "max_skew_seconds": 0.1,
        "locally_horizontal_planar_surface": True,
    }
    values.update(changes)
    return ClearanceCalibration(**values)


def controller_and_stream(monkeypatch, *, config=None, clearance=None):
    clock = Clock()
    monkeypatch.setattr(drone_control.time, "monotonic", clock.now)
    monkeypatch.setattr(drone_control.time, "sleep", clock.sleep)
    bounds = {
        name: 0.15
        for name in (
            "heartbeat",
            "mode",
            "location",
            "velocity",
            "attitude",
            "landed_state",
            "armed",
            "home",
            "rc_input",
            "range",
            "failsafe",
        )
    }
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
    sequence = [0]

    def publish(*, omit=(), **overrides):
        sequence[0] += 1
        values = {
            "heartbeat": "alive",
            "mode": "GUIDED",
            "location": (410000000, -810000000, 260000, 10000),
            "velocity": (0, 0, 0),
            "attitude": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "landed_state": 2,
            "armed": True,
            "failsafe": (False, "test clear"),
        }
        values.update(overrides)
        for name in omit:
            values.pop(name)
        assert state.update_many(
            values,
            received_at=clock.now(),
            sequence=sequence[0],
            source_system=1,
            source_component=1,
        )
        sequence[0] += 1
        assert state.observe_rc_input(
            channel=7,
            pwm=1500,
            signal_healthy=True,
            received_at=clock.now(),
            sequence=sequence[0],
            source_system=1,
            source_component=1,
        )

    publish(landed_state=1, armed=False)
    assert state.acquire_initial_companion_authority(now=clock.now())
    publish()
    lidar = SampleLidar(clock)
    controller = object.__new__(DroneControl)
    controller.vehicle = Vehicle()
    controller.flight_controller_target = SourceIdentity(1, 1)
    controller.flight_state = state
    controller.permission_guard = lambda: None
    controller._mission_ack_tracker = MissionAckTracker(
        source_system=1,
        source_component=1,
        target_system=1,
        target_component=191,
        mission_type=0,
    )
    controller._mission_ack_transaction_lock = threading.Lock()
    controller.vehicle._master.mav.mission_callback = lambda: (
        controller._mission_ack_tracker.observe_message(
            type(
                "MissionAck",
                (),
                {
                    "type": 0,
                    "mission_type": 0,
                    "target_system": 1,
                    "target_component": 191,
                    "get_srcSystem": lambda self: 1,
                    "get_srcComponent": lambda self: 1,
                },
            )()
        )
    )
    controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: (
            controller.permission_guard(), operation()
        )[1],
        transport_transaction=lambda operation, enqueue_check: (
            enqueue_check(), operation()
        )[1],
    )
    controller._mission_home = type(
        "Home", (), {"lat": 41.0, "long": -81.0, "amsl_m": 250.0}
    )()
    controller.cruise_alt = 10.0
    controller.clearance_calibration = clearance
    controller.release_stability_config = config
    controller._release_hold_confirmation = None
    return controller, clock, lidar, publish


def stable_config(**changes):
    values = dict(VALID_CONFIG)
    values.update(changes)
    return ReleaseStabilityConfig(**values)


def add_stable_stream(clock, lidar, publish, *, changes=None, omit=()):
    changes = changes or {}

    def update():
        publish(omit=omit, **changes)
        lidar.publish()

    clock.callbacks.append(update)


def test_real_writer_handover_and_hardware_drop_complete_without_fake_receipts(
    monkeypatch,
):
    """Break caught: evidence and GPIO boundaries are not MAVLink enqueues."""
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch,
        config=stable_config(),
        clearance=calibration(max_age_seconds=0.5),
    )
    add_stable_stream(clock, lidar, publish)
    outbound = queue.Queue()
    guarded_writer = drone_control._OutputGuardedWriter(MAVWriter(outbound))

    class WriterMav:
        file = guarded_writer

        @staticmethod
        def mission_item_int_send(*_fields):
            guarded_writer.write(b"waypoint")
            controller._mission_ack_tracker.observe_message(
                type(
                    "MissionAck",
                    (),
                    {
                        "type": 0,
                        "mission_type": 0,
                        "target_system": 1,
                        "target_component": 191,
                        "get_srcSystem": lambda self: 1,
                        "get_srcComponent": lambda self: 1,
                    },
                )()
            )

    controller.vehicle._master.mav = WriterMav()
    controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=lambda operation: operation(),
        transport_transaction=guarded_writer.transaction,
    )
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True

    servo_events = []
    dropper = HardwareDropper(
        pins=(17,),
        permission=lambda: True,
        release_hold_seconds=0.5,
        permission_check_interval_seconds=0.1,
        servo_factory=lambda _pin, *, initial_value: RecordingServo(servo_events),
        sleeper=clock.sleep,
    )
    controller.release_payload_if_stable(
        dropper, waypoint, lidar, required_agl_m=10.0
    )

    assert servo_events == ["max", "mid"]
    assert outbound.qsize() >= 1


def test_hardware_drop_omits_mid_after_authority_revocation(monkeypatch):
    """Break caught: continuation GPIO must stop after release authority is lost."""
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch,
        config=stable_config(),
        clearance=calibration(max_age_seconds=0.5),
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    servo_events = []
    dropper = HardwareDropper(
        pins=(17,),
        permission=lambda: True,
        release_hold_seconds=0.5,
        permission_check_interval_seconds=0.1,
        servo_factory=lambda _pin, *, initial_value: RecordingServo(servo_events),
        sleeper=clock.sleep,
    )
    revoked = [False]

    def revoke_once():
        if not revoked[0]:
            revoked[0] = True
            controller.flight_state.invalidate_observation(
                "heartbeat", source_system=1, source_component=1
            )

    clock.callbacks.append(revoke_once)

    with pytest.raises(AuthorityLost):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )

    assert servo_events == ["max"]


def test_second_servo_release_rechecks_range_and_neutralizes_after_partial_failure(
    monkeypatch,
):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch,
        config=stable_config(),
        clearance=calibration(max_age_seconds=0.5),
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    events = []

    class Servo:
        def __init__(self, pin):
            self.pin = pin

        def max(self):
            events.append(("max", self.pin))
            if self.pin == 17:
                lidar.available = False

        def mid(self):
            events.append(("mid", self.pin))
            if self.pin == 17:
                raise OSError("servo 17 neutral failed")

    dropper = HardwareDropper(
        pins=(17, 18),
        permission=lambda: True,
        release_hold_seconds=0.5,
        permission_check_interval_seconds=0.1,
        servo_factory=lambda pin, *, initial_value: Servo(pin),
        sleeper=clock.sleep,
    )

    with pytest.raises(FlightOperationError, match="clearance") as captured:
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )

    assert not isinstance(captured.value, OSError)
    assert events == [("max", 17), ("mid", 17), ("mid", 18)]
    with pytest.raises(FlightOperationError, match="confirmation"):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )
    assert events == [("max", 17), ("mid", 17), ("mid", 18)]


def test_second_servo_release_rechecks_velocity_and_neutralizes_after_partial_failure(
    monkeypatch,
):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch,
        config=stable_config(),
        clearance=calibration(max_age_seconds=0.5),
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    events = []

    class Servo:
        def __init__(self, pin):
            self.pin = pin

        def max(self):
            events.append(("max", self.pin))
            if self.pin == 17:
                publish(velocity=(30, 0, 0))
                lidar.publish()

        def mid(self):
            events.append(("mid", self.pin))

    dropper = HardwareDropper(
        pins=(17, 18),
        permission=lambda: True,
        release_hold_seconds=0.5,
        permission_check_interval_seconds=0.1,
        servo_factory=lambda pin, *, initial_value: Servo(pin),
        sleeper=clock.sleep,
    )

    with pytest.raises(FlightOperationError, match="stability limits"):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )

    assert events == [("max", 17), ("mid", 17), ("mid", 18)]
    with pytest.raises(FlightOperationError, match="confirmation"):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )
    assert events == [("max", 17), ("mid", 17), ("mid", 18)]


def test_hold_fails_before_output_without_explicit_calibration_or_policy(monkeypatch):
    for config, clearance in ((stable_config(), None), (None, calibration())):
        controller, _, lidar, _ = controller_and_stream(
            monkeypatch, config=config, clearance=clearance
        )
        with pytest.raises(FlightOperationError, match="release stability"):
            controller.hold_waypoint_until_stable(
                GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
            )
        assert controller.vehicle._master.mav.waypoints == []


def test_distinct_coherent_samples_are_required_for_continuous_hold(monkeypatch):
    controller, _, lidar, _ = controller_and_stream(
        monkeypatch,
        config=stable_config(timeout_seconds=0.45),
        clearance=calibration(max_age_seconds=0.5),
    )

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is False


@pytest.mark.parametrize("missing_field", ["location", "velocity", "attitude"])
def test_each_stale_flight_observation_restarts_the_hold(monkeypatch, missing_field):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch,
        config=stable_config(timeout_seconds=0.45),
        clearance=calibration(),
    )
    add_stable_stream(clock, lidar, publish, omit=(missing_field,))

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is False


@pytest.mark.parametrize(
    ("field", "invalid_value"),
    [
        ("location", (410000000, -810000000, 260000)),
        ("velocity", (0, 0)),
        ("attitude", (0.0, 0.0)),
    ],
)
def test_each_malformed_flight_observation_restarts_the_hold(
    monkeypatch, field, invalid_value
):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch,
        config=stable_config(timeout_seconds=0.45),
        clearance=calibration(),
    )
    publish(**{field: invalid_value})
    add_stable_stream(clock, lidar, publish, omit=(field,))

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is False


def test_unavailable_range_restarts_the_hold(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch,
        config=stable_config(timeout_seconds=0.45),
        clearance=calibration(),
    )
    add_stable_stream(clock, lidar, publish)
    lidar.available = False

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is False


@pytest.mark.parametrize(
    ("changes", "distance"),
    [
        ({"location": (410001000, -810000000, 260000, 10000)}, 10.0),
        ({"location": (410000000, -810000000, 261000, 10000)}, 10.0),
        ({"velocity": (30, 0, 0)}, 10.0),
        ({"velocity": (0, 0, 20)}, 10.0),
        ({"attitude": (0.25, 0.0, 0.0, 0.0, 0.0, 0.0)}, 10.0),
        ({"attitude": (0.0, 0.25, 0.0, 0.0, 0.0, 0.0)}, 10.0),
        ({}, 9.99),
    ],
)
def test_each_out_of_tolerance_observation_prevents_hold(
    monkeypatch, changes, distance
):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch,
        config=stable_config(timeout_seconds=0.45),
        clearance=calibration(),
    )
    lidar.distance = distance
    add_stable_stream(clock, lidar, publish, changes=changes)

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is False


def test_missed_range_invalidation_restarts_the_hold(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    callbacks = [0]

    def invalidate_once():
        callbacks[0] += 1
        if callbacks[0] == 3:
            lidar.invalidate_between_reads()

    clock.callbacks.append(invalidate_once)

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is True
    assert clock.now() >= 10.5


def test_range_invalidation_during_success_handover_restarts_the_hold(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    original_get_sample = lidar.get_sample
    invalidated = [False]
    calls_at_boundary = [0]

    def invalidate_at_handover():
        if clock.now() >= 10.4 and not invalidated[0]:
            calls_at_boundary[0] += 1
            if calls_at_boundary[0] == 3:
                invalidated[0] = True
                lidar.invalidate_between_reads()
        return original_get_sample()

    lidar.get_sample = invalidate_at_handover

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is True
    assert clock.now() >= 10.8


def test_unstable_first_handover_read_cannot_be_overwritten_by_final_read(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch,
        config=stable_config(hold_seconds=0.35),
        clearance=calibration(),
    )
    add_stable_stream(clock, lidar, publish)
    original_get_sample = lidar.get_sample
    calls_at_handover = [0]
    injected = [False]

    def low_then_stable_handover():
        if clock.now() >= 10.35 and not injected[0]:
            calls_at_handover[0] += 1
            if calls_at_handover[0] == 2:
                lidar.sequence += 1
                return LidarSample(9.0, clock.now(), lidar.sequence, 0)
            if calls_at_handover[0] == 3:
                injected[0] = True
                lidar.publish()
        return original_get_sample()

    lidar.get_sample = low_then_stable_handover

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is True
    assert clock.now() >= 10.7


def test_unstable_range_observed_only_during_reissue_restarts_hold(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    original_get_sample = lidar.get_sample
    calls_at_reissue = [0]
    injected = [False]

    def low_during_reissue():
        if clock.now() >= 10.2 and not injected[0]:
            calls_at_reissue[0] += 1
            if calls_at_reissue[0] == 2:
                injected[0] = True
                lidar.sequence += 1
                return LidarSample(9.0, clock.now(), lidar.sequence, 0)
        return original_get_sample()

    lidar.get_sample = low_during_reissue

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is True
    assert clock.now() >= 10.6


def test_range_sequence_regression_restarts_the_hold(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    calls = [0]

    def regress_once():
        calls[0] += 1
        if calls[0] == 3:
            lidar.sample = LidarSample(lidar.distance, clock.now(), 1, 0)

    clock.callbacks.append(regress_once)

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is True
    assert clock.now() >= 10.5


def test_outer_sequence_regression_cannot_be_overwritten_by_reissue(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    original_get_sample = lidar.get_sample
    calls_at_reissue = [0]
    injected = [False]

    def regressed_then_current_reissue():
        if clock.now() >= 10.2 and not injected[0]:
            calls_at_reissue[0] += 1
            if calls_at_reissue[0] == 1:
                return LidarSample(10.0, clock.now(), 1, 0)
            injected[0] = True
        return original_get_sample()

    lidar.get_sample = regressed_then_current_reissue

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is True
    assert clock.now() >= 10.6


def test_fresh_evidence_after_a_gap_starts_a_new_hold_window(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch,
        config=stable_config(timeout_seconds=1.2),
        clearance=calibration(),
    )
    calls = [0]

    def update():
        calls[0] += 1
        publish(omit=("attitude",) if calls[0] <= 4 else ())
        lidar.publish()

    clock.callbacks.append(update)

    assert controller.hold_waypoint_until_stable(
        GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
    ) is True
    assert clock.now() >= 10.6


def test_release_boundary_rechecks_authority_and_does_not_drop(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    dropper = Dropper()
    def revoke():
        controller.flight_state.invalidate_observation(
            "heartbeat", source_system=1, source_component=1
        )

    controller.permission_guard = revoke

    with pytest.raises(AuthorityLost):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )
    assert dropper.calls == 0


def test_release_boundary_rechecks_range_after_last_supervisor_callback(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    def invalidate_range_at_output(operation):
        lidar.available = False
        return operation()

    controller._supervisor_output_transaction = invalidate_range_at_output
    dropper = Dropper()

    with pytest.raises(FlightOperationError, match="clearance"):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )
    assert dropper.calls == 0


def test_invalid_then_valid_attitude_breaks_completed_release_hold(monkeypatch):
    """Break caught: an unseen invalid sample must reset release continuity."""
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    source = controller.flight_state.source
    snapshot = controller.flight_state.snapshot()
    next_sequence = snapshot.attitude.observation.sequence + 1
    assert controller.flight_state.invalidate_evidence(
        ("attitude",),
        source_system=source.system_id,
        source_component=source.component_id,
        received_at=clock.now(),
        sequence=next_sequence,
    )
    assert controller.flight_state.update(
        "attitude",
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        source_system=source.system_id,
        source_component=source.component_id,
        received_at=clock.now(),
        sequence=next_sequence + 1,
    )
    lidar.publish()
    dropper = Dropper()

    with pytest.raises(FlightOperationError, match="no longer current"):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )
    assert dropper.calls == 0


def test_release_boundary_rejects_regression_from_hold_confirmation(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    confirmation_sequence = (
        controller._release_hold_confirmation.evidence.range_sequence
    )
    def regressing_sample():
        return LidarSample(10.0, clock.now(), confirmation_sequence - 1, 0)

    lidar.get_sample = regressing_sample
    dropper = Dropper()

    with pytest.raises(FlightOperationError, match="release confirmation"):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )
    assert dropper.calls == 0


def test_hold_cannot_succeed_after_authority_loss_during_final_observation(
    monkeypatch,
):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(hold_seconds=0.35), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    original_get_sample = lidar.get_sample

    def revoke_at_handover():
        sample = original_get_sample()
        if clock.now() >= 10.35:
            controller.flight_state.invalidate_observation(
                "heartbeat", source_system=1, source_component=1
            )
        return sample

    lidar.get_sample = revoke_at_handover

    with pytest.raises(AuthorityLost):
        controller.hold_waypoint_until_stable(
            GPSCoord(41.0, -81.0, 10.0), lidar, required_agl_m=10.0
        )


def test_release_boundary_rejects_new_unstable_motion(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    publish(velocity=(0, 0, 20))
    lidar.publish()
    dropper = Dropper()

    with pytest.raises(FlightOperationError, match="stability limits"):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )
    assert dropper.calls == 0


def test_successful_release_consumes_confirmation_before_payload_call(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    dropper = Dropper()

    controller.release_payload_if_stable(
        dropper, waypoint, lidar, required_agl_m=10.0
    )
    with pytest.raises(FlightOperationError, match="release confirmation"):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )
    assert dropper.calls == 1


def test_release_boundary_rejects_changed_range_generation(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    lidar.invalidate_between_reads()
    dropper = Dropper()

    with pytest.raises(FlightOperationError, match="release confirmation"):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )
    assert dropper.calls == 0


def test_release_confirmation_expires_before_payload_output(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    add_stable_stream(clock, lidar, publish)
    waypoint = GPSCoord(41.0, -81.0, 10.0)
    assert controller.hold_waypoint_until_stable(
        waypoint, lidar, required_agl_m=10.0
    ) is True
    clock.value += 0.31
    dropper = Dropper()

    with pytest.raises(FlightOperationError, match="expired"):
        controller.release_payload_if_stable(
            dropper, waypoint, lidar, required_agl_m=10.0
        )
    assert dropper.calls == 0


def test_clearance_correction_uses_current_original_home_altitude(monkeypatch):
    controller, _, lidar, _ = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    lidar.distance = 8.0
    lidar.publish()

    assert controller.release_waypoint_for_clearance(
        GPSCoord(41.1, -81.1, 30.0), lidar, desired_agl_m=10.0
    ) == GPSCoord(41.1, -81.1, 12.0)


def test_clearance_correction_rejects_incoherent_location_before_move(monkeypatch):
    controller, clock, lidar, publish = controller_and_stream(
        monkeypatch, config=stable_config(), clearance=calibration()
    )
    clock.value += 0.11
    publish(omit=("location", "velocity"))
    lidar.publish()

    with pytest.raises(FlightOperationError, match="coherent"):
        controller.release_waypoint_for_clearance(
            GPSCoord(41.1, -81.1, 30.0), lidar, desired_agl_m=10.0
        )
