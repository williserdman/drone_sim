import math
from types import SimpleNamespace

import pytest

from drone import timebase
from drone.common_types import GPSCoord, MissionHome, RelPosComplete
from drone.control.mission_supervisor import AuthorityLost, MissionAbort
from drone.sensors.camera._camera_manager import FrameMetadata
from drone.sensors.lidar.clearance import ClearanceCalibration
from drone.sensors.lidar.lidar import LidarSample
from drone.sensors.lidar.lidar import StaleSensorError


class FakeClock:
    def __init__(self, now=10.0):
        self.value = now
        self.sleeps = []

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.value += seconds


def policy(clock, **changes):
    from drone.mock_mission import PrecisionMissionPolicy

    values = dict(
        clearance_calibration=ClearanceCalibration(
            beam_direction_body_frd=(0.0, 0.0, 1.0),
            measured_reference_offset_body_frd_m=(0.0, 0.0, 0.0),
            lidar_mounting_offset_already_applied=True,
            max_tilt_rad=0.5,
            max_age_seconds=0.5,
            max_skew_seconds=0.2,
            locally_horizontal_planar_surface=True,
        ),
        clock=timebase.monotonic,
        max_exposure_age_s=0.5,
        max_image_attitude_skew_s=0.2,
        max_image_location_skew_s=0.2,
        max_attitude_transport_latency_s=0.05,
        max_location_transport_latency_s=0.05,
        acquisition_timeout_s=2.0,
        frame_timeout_s=0.1,
        observation_period_s=0.05,
        target_hover_height_m=4.572,
        hover_tolerance_m=1.0,
        centered_tolerance_m=0.5,
        correction_gain=0.3,
        cruise_altitude_m=10.0,
        desired_drop_height_m=10.0,
        landing_timeout_s=60.0,
        target_loss_timeout_s=0.5,
        reacquisition_count=5,
    )
    values.update(changes)
    return PrecisionMissionPolicy(**values)


def field(
    value,
    sampled_at=10.0,
    sequence=1,
    fresh=True,
    invalidation_generation=0,
):
    return SimpleNamespace(
        observation=SimpleNamespace(
            value=value,
            received_at=sampled_at,
            sequence=sequence,
        ),
        fresh=fresh,
        invalidation_generation=invalidation_generation,
    )


def snapshot(
    *,
    sampled_at=10.0,
    sequence=1,
    location=None,
    attitude=None,
    location_generation=0,
    attitude_generation=0,
):
    return SimpleNamespace(
        location=field(
            location or (410000000, -810000000, 104_572, 4_572),
            sampled_at,
            sequence,
            invalidation_generation=location_generation,
        ),
        attitude=field(
            attitude or (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            sampled_at,
            sequence,
            invalidation_generation=attitude_generation,
        ),
        landed_state=field(2, sampled_at, sequence),
        armed=field(True, sampled_at, sequence),
    )


class ReadyCamera:
    def __init__(self, clock, results):
        self.clock = clock
        self.results = list(results)
        self.calls = []

    def precision_readiness(self):
        return SimpleNamespace(ready=True, reasons=())

    def vec_to_marker_3d_bounded(
        self, target_id, *, timeout_s, after_sequence=0, quality=4
    ):
        self.calls.append((target_id, timeout_s, after_sequence, quality))
        if not self.results:
            raise TimeoutError("no fresh frame")
        sequence, vector = self.results.pop(0)
        now_ns = int(self.clock.now() * 1_000_000_000)
        return vector, FrameMetadata(
            sequence=sequence,
            exposure_timestamp_ns=now_ns,
            receipt_timestamp_ns=now_ns,
            exposure_age_ns=0,
            exposure_age_bounded=True,
            raw_image_size_px=(640, 480),
            image_size_px=(640, 480),
        )

    def save_frame_buffer_async(self):
        return None


class SampleLidar:
    def __init__(self, clock, distances=None, generations=None):
        self.clock = clock
        self.distances = list(distances or [4.572] * 30)
        self.generations = list(generations or [0] * len(self.distances))
        self.sequence = 0

    def get_sample(self):
        self.sequence += 1
        distance = self.distances.pop(0)
        generation = self.generations.pop(0)
        if isinstance(distance, BaseException):
            raise distance
        return LidarSample(distance, self.clock.now(), self.sequence, generation)


class PrecisionController:
    def __init__(self, clock, snapshots=None, current_original_home_alt=10.0):
        self.clock = clock
        self.events = []
        self.mission_home = MissionHome(41.0, -81.0, 100.0)
        self.snapshots = list(snapshots or [])
        self.hold_arguments = None
        self.release_arguments = None
        self.current_original_home_alt = current_original_home_alt

    def check_permission(self):
        self.events.append(("permission",))

    def flight_snapshot(self):
        if self.snapshots:
            return self.snapshots.pop(0)
        return snapshot(sampled_at=self.clock.now())

    def set_guided_mode(self):
        self.events.append(("guided",))
        return 0

    def guide_move_relative_frame(self, offset, timeout=None):
        self.events.append(("relative", offset, timeout))
        return 0

    def goto_waypoint(self, waypoint, position_tol=None, timeout=None):
        self.events.append(("goto", waypoint, position_tol, timeout))
        return 0

    def get_location_metres(self, origin, north, east):
        self.events.append(("offset", north, east))
        return GPSCoord(origin.lat, origin.long, origin.alt)

    def set_land_mode(self):
        self.events.append(("land",))
        return 0

    def send_guided_waypoint(self, waypoint):
        self.events.append(("guided_waypoint", waypoint))
        return 0

    def require_precision_landing_profile(self):
        self.events.append(("precision_profile",))
        return True

    def land_send_landing_target(self, vector):
        self.events.append(("landing_target", vector))
        return 0

    def confirm_landing(self, *, timeout):
        self.events.append(("confirm_landing", timeout))
        return 0

    def disarm(self):
        self.events.append(("disarm",))
        return 0

    def force_arm_takeoff(self, altitude):
        self.events.append(("takeoff", altitude))
        return None

    def hold_waypoint_until_stable(self, waypoint, lidar, required_agl_m):
        self.hold_arguments = (waypoint, lidar, required_agl_m)
        sample = lidar.get_sample()
        self.events.append(("hold", waypoint, sample.distance_m, required_agl_m))
        return True

    def release_waypoint_for_clearance(self, waypoint, lidar, *, desired_agl_m):
        sample = lidar.get_sample()
        corrected = GPSCoord(
            waypoint.lat,
            waypoint.long,
            self.current_original_home_alt + desired_agl_m - sample.distance_m,
        )
        self.events.append(("release_waypoint", waypoint, lidar, desired_agl_m))
        return corrected

    def release_payload_if_stable(
        self, dropper, waypoint, lidar, *, required_agl_m
    ):
        self.release_arguments = (dropper, waypoint, lidar, required_agl_m)
        self.events.append(("release", waypoint, required_agl_m))
        lidar.get_sample()
        return dropper.drop()


class Payload:
    supports_attachment = True

    def __init__(self, attach_result=True, drop_result=None):
        self.attach_result = attach_result
        self.drop_result = drop_result
        self.events = []

    def attach(self, target_id):
        self.events.append(("attach", target_id))
        return self.attach_result

    def drop(self):
        self.events.append(("drop",))
        return self.drop_result


class Tracker:
    def time_left(self):
        return 300.0


def centered(sequence):
    return sequence, RelPosComplete(0.1, 0.0, 4.572)


def test_missing_precision_policy_fails_before_any_output():
    from drone.mock_mission import fm3

    clock = FakeClock()
    controller = PrecisionController(clock)

    with pytest.raises(ValueError, match="precision policy"):
        fm3(
            Tracker(),
            controller,
            ReadyCamera(clock, []),
            SampleLidar(clock),
            Payload(),
            {3},
            GPSCoord(41.0, -81.0, 10.0),
            GPSCoord(41.1, -81.1, 10.0),
        )

    assert controller.events == []


def test_unsupported_attachment_fails_before_any_output():
    from drone.mock_mission import fm3

    clock = FakeClock()
    controller = PrecisionController(clock)
    payload = Payload()
    payload.supports_attachment = False

    with pytest.raises(RuntimeError, match="attachment capability"):
        fm3(
            Tracker(),
            controller,
            ReadyCamera(clock, []),
            SampleLidar(clock),
            payload,
            {3},
            GPSCoord(41.0, -81.0, 10.0),
            GPSCoord(41.1, -81.1, 10.0),
            precision_policy=policy(clock),
        )

    assert controller.events == []


def test_pickup_uses_five_fresh_correlated_samples_then_disarms_and_attaches(
    monkeypatch,
):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    camera = ReadyCamera(clock, [centered(i) for i in range(1, 7)])
    payload = Payload()
    monkeypatch.setattr(mission, "aruco_land_precision", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        result = mission.pickup_sequence(
            controller,
            camera,
            SampleLidar(clock),
            3,
            payload,
            policy=policy(clock),
        )

    assert result is True
    assert payload.events == [("attach", 3)]
    terminal = [
        event[0]
        for event in controller.events
        if event[0] in {"land", "confirm_landing", "disarm"}
    ]
    assert terminal == ["disarm"]


def test_pickup_passes_five_sample_median_anchor_to_precision_land(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    landing_calls = []

    def land(*args, **kwargs):
        landing_calls.append((args, kwargs))
        return mission.LandingResult.TOUCHDOWN

    monkeypatch.setattr(mission, "aruco_land_precision", land)

    with timebase.configured(clock):
        result = mission.pickup_sequence(
            controller,
            ReadyCamera(clock, [centered(i) for i in range(1, 7)]),
            SampleLidar(clock),
            3,
            Payload(),
            policy=policy(clock),
        )

    assert result is True
    assert len(landing_calls) == 1
    anchor = landing_calls[0][1]["anchor"]
    assert anchor.lat == pytest.approx(41.00000089831528)
    assert anchor.long == pytest.approx(-81.0)
    assert [event for event in controller.events if event[0] == "precision_profile"] == [
        ("precision_profile",)
    ]


def test_pickup_reacquires_once_after_hold_timeout(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    outcomes = iter((mission.LandingResult.RETRY, mission.LandingResult.TOUCHDOWN))
    landing_calls = []

    def land(*args, **kwargs):
        landing_calls.append((args, kwargs))
        return next(outcomes)

    monkeypatch.setattr(mission, "aruco_land_precision", land)

    with timebase.configured(clock):
        result = mission.pickup_sequence(
            controller,
            ReadyCamera(clock, [centered(i) for i in range(1, 13)]),
            SampleLidar(clock, distances=[4.572] * 30),
            3,
            Payload(),
            policy=policy(clock),
        )

    assert result is True
    assert len(landing_calls) == 2
    assert [call[1]["retry_number"] for call in landing_calls] == [0, 1]
    retry_moves = [
        event
        for event in controller.events
        if event[0] == "goto" and event[1].alt == pytest.approx(4.572)
    ]
    assert retry_moves
    assert [event for event in controller.events if event[0] == "precision_profile"] == [
        ("precision_profile",)
    ]


def test_lost_target_holds_then_reacquires_before_resuming_land():
    from drone.mock_mission import aruco_land_precision

    class TouchdownAfterReacquire(PrecisionController):
        def flight_snapshot(self):
            current = super().flight_snapshot()
            if sum(event == ("land",) for event in self.events) >= 2:
                current.landed_state = field(1, self.clock.now(), 100)
            return current

    clock = FakeClock()
    controller = TouchdownAfterReacquire(clock)
    observations = [(index, None) for index in range(1, 4)]
    observations.extend(centered(index) for index in range(4, 7))

    with timebase.configured(clock):
        result = aruco_land_precision(
            controller,
            ReadyCamera(clock, observations),
            SampleLidar(clock, distances=[1.0] * 40),
            3,
            policy=policy(
                clock,
                target_loss_timeout_s=0.1,
                reacquisition_count=3,
            ),
        )

    assert result is True
    terminal = [event[0] for event in controller.events if event[0] != "permission"]
    assert terminal.count("land") == 2
    guided_index = terminal.index("guided")
    resumed_land_index = terminal.index("land", guided_index)
    assert "guided_waypoint" in terminal[guided_index:resumed_land_index]
    assert "landing_target" not in terminal[guided_index:resumed_land_index]
    assert terminal[-1] == "confirm_landing"


def test_marker_loss_below_precision_floor_stays_in_land_until_touchdown():
    from drone.mock_mission import aruco_land_precision

    class TouchdownNearGround(PrecisionController):
        def flight_snapshot(self):
            current = super().flight_snapshot()
            if self.clock.now() >= 10.2:
                current.landed_state = field(1, self.clock.now(), 100)
            return current

    class CameraMustNotBeRead(ReadyCamera):
        def vec_to_marker_3d_bounded(self, *args, **kwargs):
            raise AssertionError("camera must not gate descent below PLND_ALT_MIN")

    clock = FakeClock()
    controller = TouchdownNearGround(clock)

    with timebase.configured(clock):
        result = aruco_land_precision(
            controller,
            CameraMustNotBeRead(clock, []),
            SampleLidar(clock, distances=[0.5] * 20),
            3,
            policy=policy(clock),
        )

    assert result is True
    terminal = [event[0] for event in controller.events if event[0] != "permission"]
    assert terminal == ["land", "confirm_landing"]


def test_rejected_landing_observation_emits_parseable_diagnostic(capsys):
    import json
    from drone.mock_mission import LandingResult, aruco_land_precision

    clock = FakeClock()
    with timebase.configured(clock):
        result = aruco_land_precision(
            PrecisionController(clock),
            ReadyCamera(clock, [(index, None) for index in range(1, 200)]),
            SampleLidar(clock, distances=[1.0] * 400),
            3,
            policy=policy(clock),
            deadline=16.0,
            return_result=True,
        )

    records = [
        json.loads(line.removeprefix("PRECISION_LANDING "))
        for line in capsys.readouterr().out.splitlines()
        if line.startswith("PRECISION_LANDING ")
    ]
    assert result is LandingResult.RETRY
    rejected = next(record for record in records if record["reason"] == "target_absent")
    assert rejected["state"] == "TRACKING"
    assert rejected["target_id"] == 3
    assert rejected["frame_sequence"] == 1
    assert rejected["agl_m"] == pytest.approx(1.0)


def test_replayed_frame_resets_the_five_sample_window():
    from drone.mock_mission import pickup_sequence

    clock = FakeClock()
    controller = PrecisionController(clock)
    camera = ReadyCamera(
        clock,
        [centered(i) for i in (1, 2, 2, 3, 4, 5, 6)],
    )

    with timebase.configured(clock):
        result = pickup_sequence(
            controller,
            camera,
            SampleLidar(clock),
            3,
            Payload(),
            policy=policy(clock, acquisition_timeout_s=0.31),
        )

    assert result is False
    assert not any(event[0] == "land" for event in controller.events)


def test_image_attitude_skew_resets_centered_samples():
    from drone.mock_mission import pickup_sequence

    clock = FakeClock()
    skewed = snapshot(sampled_at=clock.now() - 0.3)
    fresh = snapshot(sampled_at=clock.now())
    controller = PrecisionController(clock, [fresh] * 3 + [skewed] + [fresh] * 8)
    camera = ReadyCamera(clock, [centered(i) for i in range(1, 6)])

    with timebase.configured(clock):
        result = pickup_sequence(
            controller,
            camera,
            SampleLidar(clock),
            3,
            Payload(),
            policy=policy(clock, acquisition_timeout_s=0.31),
        )

    assert result is False
    assert not any(event[0] == "land" for event in controller.events)


def test_range_invalidation_generation_change_resets_centered_samples():
    from drone.mock_mission import pickup_sequence

    clock = FakeClock()
    controller = PrecisionController(clock)
    camera = ReadyCamera(clock, [centered(i) for i in range(1, 7)])
    lidar = SampleLidar(clock, generations=[0, 0, 0, 1, 1, 1, 1, 1, 1, 1])

    with timebase.configured(clock):
        result = pickup_sequence(
            controller,
            camera,
            lidar,
            3,
            Payload(),
            policy=policy(clock, acquisition_timeout_s=0.31),
        )

    assert result is False
    assert not any(event[0] == "land" for event in controller.events)


@pytest.mark.parametrize("field_name", ["attitude", "location"])
def test_telemetry_invalidation_generation_change_resets_centered_samples(
    monkeypatch, field_name
):
    import drone.mock_mission as mission

    clock = FakeClock()
    class GenerationController(PrecisionController):
        def __init__(self, clock):
            super().__init__(clock)
            self.snapshot_calls = 0

        def flight_snapshot(self):
            self.snapshot_calls += 1
            # Three calls establish and reconfirm clearance plus the grid
            # origin. The transition is observed on centered read three.
            generation = 1 if self.snapshot_calls >= 6 else 0
            return snapshot(
                sampled_at=self.clock.now(),
                attitude_generation=generation if field_name == "attitude" else 0,
                location_generation=generation if field_name == "location" else 0,
            )

    controller = GenerationController(clock)
    camera = ReadyCamera(clock, [centered(i) for i in range(1, 8)])
    precision_land_calls = []
    monkeypatch.setattr(
        mission,
        "aruco_land_precision",
        lambda *args, **kwargs: precision_land_calls.append((args, kwargs)) or True,
    )

    with timebase.configured(clock):
        result = mission.pickup_sequence(
            controller,
            camera,
            SampleLidar(clock),
            3,
            Payload(),
            policy=policy(clock, acquisition_timeout_s=0.5),
        )

    assert result is False
    assert precision_land_calls == []
    assert not any(event[0] == "land" for event in controller.events)


def test_precision_land_uses_configured_deadline_and_positive_confirmation():
    from drone.mock_mission import aruco_land_precision

    clock = FakeClock()
    controller = PrecisionController(clock)
    camera = ReadyCamera(clock, [centered(i) for i in range(1, 2_000)])

    with timebase.configured(clock):
        result = aruco_land_precision(
            controller,
            camera,
            SampleLidar(clock, distances=[1.0] * 2_000),
            3,
            policy=policy(
                clock,
                observation_period_s=0.1,
                landing_timeout_s=0.6,
            ),
        )

    assert result is True
    assert clock.now() == pytest.approx(10.6, abs=0.11)
    assert controller.events[-1][0] == "confirm_landing"
    assert not any(event[0] == "disarm" for event in controller.events)


def test_fm3_rearms_only_after_checked_disarm_and_confirmed_attachment(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    payload = Payload()

    def pickup(controller, camera, lidar, target_id, dropper, *, policy):
        controller.events.extend([("land",), ("confirm_landing", 1.0), ("disarm",)])
        assert dropper.attach(target_id) is True
        controller.events.append(("attachment_confirmed",))
        return True

    monkeypatch.setattr(mission, "pickup_sequence", pickup)

    with timebase.configured(clock):
        result = mission.fm3(
            Tracker(),
            controller,
            ReadyCamera(clock, []),
            SampleLidar(clock),
            payload,
            {3},
            GPSCoord(41.0, -81.0, 10.0),
            GPSCoord(41.1, -81.1, 10.0),
            precision_policy=policy(clock),
        )

    assert result is True
    names = [event[0] for event in controller.events]
    assert names.index("disarm") < names.index("attachment_confirmed") < names.index("takeoff")
    assert payload.events == [("attach", 3), ("drop",)]
    assert "rtl" not in names


@pytest.mark.parametrize(
    ("captured_pickup_alt", "captured_delivery_alt"),
    [(0.0, -25.0), (-3.5, 4_000.0), (9_000.0, 0.0)],
)
def test_fm3_transit_uses_cruise_altitude_without_mutating_captured_points(
    monkeypatch, captured_pickup_alt, captured_delivery_alt
):
    """Break caught: captured ground altitudes must not drive FM3 transit."""
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    pickup = GPSCoord(41.25, -81.25, captured_pickup_alt)
    delivery = GPSCoord(41.75, -81.75, captured_delivery_alt)
    original_pickup = GPSCoord(pickup.lat, pickup.long, pickup.alt)
    original_delivery = GPSCoord(delivery.lat, delivery.long, delivery.alt)
    monkeypatch.setattr(mission, "pickup_sequence", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        result = mission.fm3(
            Tracker(),
            controller,
            ReadyCamera(clock, []),
            SampleLidar(clock),
            Payload(),
            {3},
            pickup,
            delivery,
            precision_policy=policy(clock, cruise_altitude_m=12.5),
        )

    assert result is True
    transit_moves = [event[1] for event in controller.events if event[0] == "goto"][:2]
    assert transit_moves == [
        GPSCoord(41.25, -81.25, 12.5),
        GPSCoord(41.75, -81.75, 12.5),
    ]
    assert transit_moves[0] is not pickup
    assert transit_moves[1] is not delivery
    assert pickup == original_pickup
    assert delivery == original_delivery


def test_keyboard_interrupt_propagates_without_local_rtl(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)

    def interrupted(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(mission, "pickup_sequence", interrupted)

    with pytest.raises(KeyboardInterrupt):
        mission.fm3(
            Tracker(),
            controller,
            ReadyCamera(clock, []),
            SampleLidar(clock),
            Payload(),
            {3},
            GPSCoord(41.0, -81.0, 10.0),
            GPSCoord(41.1, -81.1, 10.0),
            precision_policy=policy(clock),
        )

    assert not any(event[0] == "rtl" for event in controller.events)


def test_policy_rejects_an_unrelated_clock_domain():
    from drone.mock_mission import PrecisionMissionPolicy

    clock = FakeClock()
    with pytest.raises(ValueError, match="shared timebase"):
        PrecisionMissionPolicy(
            **{
                **policy(clock).__dict__,
                "clock": clock.now,
            }
        )


def test_receipt_correlation_accounts_for_measured_transport_latency():
    from drone.mock_mission import _source_interval_within_skew

    assert not _source_interval_within_skew(
        received_at=9.82,
        exposure_at=10.0,
        max_transport_latency_s=0.05,
        max_skew_s=0.2,
    )
    assert _source_interval_within_skew(
        received_at=9.82,
        exposure_at=10.0,
        max_transport_latency_s=0.01,
        max_skew_s=0.2,
    )


def test_missing_marker_breaks_consecutive_centered_run():
    from drone.mock_mission import pickup_sequence

    clock = FakeClock()
    camera = ReadyCamera(
        clock,
        [centered(1), centered(2), (3, None), centered(4), centered(5), centered(6)],
    )
    controller = PrecisionController(clock)

    with timebase.configured(clock):
        result = pickup_sequence(
            controller,
            camera,
            SampleLidar(clock),
            3,
            Payload(),
            policy=policy(clock, acquisition_timeout_s=0.31),
        )

    assert result is False
    assert not any(event[0] == "land" for event in controller.events)


@pytest.mark.parametrize("invalid", [float("nan"), float("inf")])
def test_nonfinite_marker_vectors_break_centered_run_before_land(monkeypatch, invalid):
    """Break caught: malformed pose vectors must not count toward precision LAND."""
    import drone.mock_mission as mission

    clock = FakeClock()
    invalid_frames = [
        (index, RelPosComplete(invalid, 0.0, 4.572)) for index in range(1, 6)
    ]
    camera = ReadyCamera(
        clock, invalid_frames + [centered(index) for index in range(6, 11)]
    )
    controller = PrecisionController(clock)
    monkeypatch.setattr(mission, "aruco_land_precision", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        assert mission.pickup_sequence(
            controller,
            camera,
            SampleLidar(clock),
            3,
            Payload(),
            policy=policy(clock),
        ) is True

    assert len(camera.calls) >= 10


def test_grid_search_advances_after_bounded_absent_first_cell(monkeypatch):
    import drone.mock_mission as mission

    class GridAwareCamera(ReadyCamera):
        def __init__(self, clock, controller):
            super().__init__(clock, [centered(index) for index in range(1, 100)])
            self.controller = controller

        def vec_to_marker_3d_bounded(self, *args, **kwargs):
            vector, metadata = super().vec_to_marker_3d_bounded(*args, **kwargs)
            grid_visits = len(
                [event for event in self.controller.events if event[0] == "goto"]
            )
            return (None if grid_visits == 1 else vector), metadata

    clock = FakeClock()
    controller = PrecisionController(clock)
    monkeypatch.setattr(mission, "aruco_land_precision", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        result = mission.pickup_sequence(
            controller,
            GridAwareCamera(clock, controller),
            SampleLidar(clock, distances=[4.572] * 100),
            3,
            Payload(),
            policy=policy(clock, acquisition_timeout_s=2.0),
        )

    assert result is True
    grid_gotos = [event for event in controller.events if event[0] == "goto"]
    assert len(grid_gotos) == 2


def test_slow_fifth_frame_cannot_start_land_after_acquisition_deadline(monkeypatch):
    import drone.mock_mission as mission

    class SlowFifthCamera(ReadyCamera):
        def vec_to_marker_3d_bounded(self, *args, **kwargs):
            result = super().vec_to_marker_3d_bounded(*args, **kwargs)
            if len(self.calls) == 5:
                self.clock.sleep(0.06)
            return result

    clock = FakeClock()
    controller = PrecisionController(clock)
    land_calls = []
    monkeypatch.setattr(
        mission,
        "aruco_land_precision",
        lambda *args, **kwargs: land_calls.append(clock.now()) or True,
    )

    with timebase.configured(clock):
        result = mission.pickup_sequence(
            controller,
            SlowFifthCamera(clock, [centered(index) for index in range(1, 6)]),
            SampleLidar(clock),
            3,
            Payload(),
            policy=policy(
                clock,
                acquisition_timeout_s=0.21,
                observation_period_s=0.04,
            ),
        )

    assert result is False
    assert land_calls == []


def test_grid_navigation_uses_only_remaining_acquisition_budget():
    from drone.mock_mission import pickup_sequence

    class SlowNavigationController(PrecisionController):
        def goto_waypoint(self, waypoint, position_tol=None, timeout=None):
            self.events.append(("goto", waypoint, position_tol, timeout))
            if timeout is None:
                self.clock.sleep(0.25)
            else:
                self.clock.sleep(timeout + 0.01)
            return 0

    clock = FakeClock()
    controller = SlowNavigationController(clock)

    with timebase.configured(clock):
        result = pickup_sequence(
            controller,
            ReadyCamera(clock, [centered(index) for index in range(1, 6)]),
            SampleLidar(clock),
            3,
            Payload(),
            policy=policy(clock, acquisition_timeout_s=0.2),
        )

    assert result is False
    goto = next(event for event in controller.events if event[0] == "goto")
    assert 0 < goto[3] <= 0.2
    assert not any(event[0] == "land" for event in controller.events)


def test_slow_land_frame_cannot_emit_target_after_touchdown_deadline(monkeypatch):
    import drone.mock_mission as mission

    real_read = mission._read_precision_evidence

    def slow_processed_read(*args, **kwargs):
        evidence = real_read(*args, **kwargs)
        clock.sleep(60.01)
        return evidence

    monkeypatch.setattr(mission, "_read_precision_evidence", slow_processed_read)

    clock = FakeClock()
    controller = PrecisionController(clock)

    with timebase.configured(clock):
        with pytest.raises(TimeoutError, match="touchdown deadline"):
            mission.aruco_land_precision(
                controller,
                ReadyCamera(clock, [centered(1)]),
                SampleLidar(clock, distances=[1.0, 1.0]),
                3,
                policy=policy(clock),
            )

    assert not any(event[0] == "landing_target" for event in controller.events)


def test_tilted_body_vector_projects_to_center_with_correlated_attitude():
    from drone.mock_mission import _marker_offset_ne
    from drone.sensors.lidar.clearance import AttitudeSample

    roll = 0.2
    north, east = _marker_offset_ne(
        RelPosComplete(0.0, math.sin(roll) * 5.0, math.cos(roll) * 5.0),
        AttitudeSample(roll, 0.0, 0.0, 10.0, 1),
    )

    assert north == pytest.approx(0.0, abs=1e-12)
    assert east == pytest.approx(0.0, abs=1e-12)


def test_yaw_rotates_body_forward_to_local_east():
    from drone.mock_mission import _marker_offset_ne
    from drone.sensors.lidar.clearance import AttitudeSample

    north, east = _marker_offset_ne(
        RelPosComplete(1.0, 0.0, 0.0),
        AttitudeSample(0.0, 0.0, math.pi / 2, 10.0, 1),
    )

    assert north == pytest.approx(0.0, abs=1e-12)
    assert east == pytest.approx(1.0, abs=1e-12)


def test_marker_offset_level_identity_is_preserved():
    from drone.mock_mission import _marker_offset_ne
    from drone.sensors.lidar.clearance import AttitudeSample

    assert _marker_offset_ne(
        RelPosComplete(1.2, -0.4, 5.0),
        AttitudeSample(0.0, 0.0, 0.0, 10.0, 1),
    ) == pytest.approx((1.2, -0.4))


def test_marker_offset_combines_roll_pitch_and_yaw():
    from drone.mock_mission import _marker_offset_ne
    from drone.sensors.lidar.clearance import AttitudeSample

    assert _marker_offset_ne(
        RelPosComplete(1.2, -0.4, 5.0),
        AttitudeSample(0.2, -0.1, 0.3, 10.0, 1),
    ) == pytest.approx((1.0902947109674521, -1.1128740280589786))


def test_tilted_pickup_uses_correlated_attitude_without_recenter(monkeypatch):
    import drone.mock_mission as mission

    class TiltedController(PrecisionController):
        def flight_snapshot(self):
            return snapshot(
                sampled_at=self.clock.now(),
                attitude=(0.2, 0.0, 0.0, 0.0, 0.0, 0.0),
            )

    clock = FakeClock()
    controller = TiltedController(clock)
    slant_range = 4.572 / math.cos(0.2)
    vector = RelPosComplete(0.0, math.sin(0.2) * 5.0, math.cos(0.2) * 5.0)
    monkeypatch.setattr(mission, "aruco_land_precision", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        result = mission.pickup_sequence(
            controller,
            ReadyCamera(clock, [(index, vector) for index in range(1, 6)]),
            SampleLidar(clock, distances=[slant_range] * 8),
            3,
            Payload(),
            policy=policy(clock),
        )

    assert result is True
    assert [event[1:] for event in controller.events if event[0] == "offset"] == [
        (0.0, 0.0)
    ]


def test_zero_range_never_substitutes_for_touchdown():
    from drone.mock_mission import aruco_land_precision

    class RefusingConfirmationController(PrecisionController):
        def confirm_landing(self, *, timeout):
            self.events.append(("confirm_landing", timeout))
            raise RuntimeError("touchdown was not confirmed")

    clock = FakeClock()
    controller = RefusingConfirmationController(clock)
    camera = ReadyCamera(clock, [centered(i) for i in range(1, 2_000)])

    with timebase.configured(clock):
        with pytest.raises(RuntimeError, match="touchdown was not confirmed"):
            aruco_land_precision(
                controller,
                camera,
                SampleLidar(clock, distances=[0.0] * 2_000),
                3,
                policy=policy(clock, observation_period_s=0.1),
            )

    assert not any(event[0] == "disarm" for event in controller.events)


@pytest.mark.parametrize("bad_result", [False, True, None, object()])
def test_fm3_rejects_unconfirmed_pickup_navigation_before_pickup(
    monkeypatch, bad_result
):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    controller.goto_waypoint = lambda waypoint: bad_result
    reached_pickup = []
    monkeypatch.setattr(
        mission,
        "pickup_sequence",
        lambda *args, **kwargs: reached_pickup.append(True) or True,
    )

    with pytest.raises(RuntimeError, match="pickup waypoint"):
        mission.fm3(
            Tracker(),
            controller,
            ReadyCamera(clock, []),
            SampleLidar(clock),
            Payload(),
            {3},
            GPSCoord(41.0, -81.0, 10.0),
            GPSCoord(41.1, -81.1, 10.0),
            precision_policy=policy(clock),
        )

    assert reached_pickup == []


def test_unconfirmed_attachment_stops_before_rearm(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    payload = Payload(attach_result=False)
    camera = ReadyCamera(clock, [centered(i) for i in range(1, 7)])
    monkeypatch.setattr(mission, "aruco_land_precision", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        with pytest.raises(RuntimeError, match="attachment"):
            mission.pickup_sequence(
                controller,
                camera,
                SampleLidar(clock),
                3,
                payload,
                policy=policy(clock),
            )

    assert payload.events == [("attach", 3)]
    assert not any(event[0] == "takeoff" for event in controller.events)


@pytest.mark.parametrize("attach_result", [None, 0, False])
def test_fm3_rejects_every_non_true_attachment_before_takeoff(
    monkeypatch, attach_result
):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    payload = Payload(attach_result=attach_result)
    monkeypatch.setattr(mission, "aruco_land_precision", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        with pytest.raises(RuntimeError, match="attachment"):
            mission.fm3(
                Tracker(),
                controller,
                ReadyCamera(clock, [centered(index) for index in range(1, 6)]),
                SampleLidar(clock),
                payload,
                {3},
                GPSCoord(41.0, -81.0, 10.0),
                GPSCoord(41.1, -81.1, 10.0),
                precision_policy=policy(clock),
            )

    assert payload.events == [("attach", 3)]
    assert not any(event[0] == "takeoff" for event in controller.events)


def test_late_touchdown_between_fifty_and_sixty_seconds_is_confirmed():
    from drone.mock_mission import aruco_land_precision

    class LateTouchdownController(PrecisionController):
        def flight_snapshot(self):
            current = snapshot(sampled_at=self.clock.now())
            if self.clock.now() >= 65.0:
                current.landed_state = field(1, self.clock.now(), 2)
            return current

    clock = FakeClock()
    controller = LateTouchdownController(clock)
    camera = ReadyCamera(clock, [centered(i) for i in range(1, 2_000)])

    with timebase.configured(clock):
        result = aruco_land_precision(
            controller,
            camera,
            SampleLidar(clock, distances=[1.0] * 2_000),
            3,
            policy=policy(clock, observation_period_s=0.1),
        )

    assert result is True
    assert 65.0 <= clock.now() < 70.0
    assert [event[0] for event in controller.events].count("confirm_landing") == 1


def test_marker_correction_resets_count_and_requires_five_later_samples(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    camera = ReadyCamera(
        clock,
        [(1, RelPosComplete(0.6, 0.0, 4.572))]
        + [centered(i) for i in range(2, 7)],
    )
    monkeypatch.setattr(mission, "aruco_land_precision", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        result = mission.pickup_sequence(
            controller,
            camera,
            SampleLidar(clock),
            3,
            Payload(),
            policy=policy(clock),
        )

    assert result is True
    assert ("offset", pytest.approx(0.18), pytest.approx(0.0)) in controller.events
    assert len(camera.calls) == 6
    assert all(
        event[2] == pytest.approx(0.15)
        for event in controller.events
        if event[0] == "goto"
    )


def test_failed_marker_correction_prevents_land_and_attachment():
    from drone.mock_mission import pickup_sequence

    class FailedCorrectionController(PrecisionController):
        def goto_waypoint(self, waypoint, position_tol=None, timeout=None):
            result = super().goto_waypoint(waypoint, position_tol, timeout)
            if len([event for event in self.events if event[0] == "goto"]) == 2:
                return -1
            return result

    clock = FakeClock()
    controller = FailedCorrectionController(clock)
    payload = Payload()

    with timebase.configured(clock):
        with pytest.raises(RuntimeError, match="recenter waypoint"):
            pickup_sequence(
                controller,
                ReadyCamera(clock, [(1, RelPosComplete(0.6, 0.0, 4.572))]),
                SampleLidar(clock),
                3,
                payload,
                policy=policy(clock),
            )

    assert payload.events == []
    assert not any(event[0] == "land" for event in controller.events)


def test_stale_hover_clearance_resets_until_fresh_calibrated_sample(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    lidar = SampleLidar(
        clock,
        distances=[10.0, StaleSensorError("stale"), 4.572] + [4.572] * 8,
    )
    monkeypatch.setattr(mission, "aruco_land_precision", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        assert mission.pickup_sequence(
            PrecisionController(clock),
            ReadyCamera(clock, [centered(i) for i in range(1, 6)]),
            lidar,
            3,
            Payload(),
            policy=policy(clock),
        ) is True

    assert clock.sleeps[0] == pytest.approx(0.05)


def test_hover_never_reaches_calibrated_agl_and_cannot_start_land():
    from drone.mock_mission import pickup_sequence

    clock = FakeClock()
    controller = PrecisionController(clock)

    with timebase.configured(clock):
        result = pickup_sequence(
            controller,
            ReadyCamera(clock, []),
            SampleLidar(clock, distances=[10.0] * 100),
            3,
            Payload(),
            policy=policy(clock, acquisition_timeout_s=0.2),
        )

    assert result is False
    assert not any(event[0] == "land" for event in controller.events)


def test_agl_drift_commands_calibrated_vertical_correction_before_counting(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    lidar = SampleLidar(
        clock,
        distances=[10.0, 4.572, 5.7] + [4.572] * 8,
    )
    monkeypatch.setattr(mission, "aruco_land_precision", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        result = mission.pickup_sequence(
            controller,
            ReadyCamera(clock, [centered(i) for i in range(1, 7)]),
            lidar,
            3,
            Payload(),
            policy=policy(clock),
        )

    assert result is True
    vertical = [event[1].z for event in controller.events if event[0] == "relative"]
    assert vertical == [pytest.approx(5.428), pytest.approx(1.128)]


def test_fm3_low_time_stops_before_navigation():
    from drone.mock_mission import fm3

    clock = FakeClock()
    controller = PrecisionController(clock)

    result = fm3(
        SimpleNamespace(time_left=lambda: 59.0),
        controller,
        ReadyCamera(clock, []),
        SampleLidar(clock),
        Payload(),
        {3},
        GPSCoord(41.0, -81.0, 10.0),
        GPSCoord(41.1, -81.1, 10.0),
        precision_policy=policy(clock),
    )

    assert result is False
    assert not any(event[0] == "goto" for event in controller.events)


def test_configured_drop_height_drives_adjustment_and_stability_gate(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock)
    monkeypatch.setattr(mission, "pickup_sequence", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        result = mission.fm3(
            Tracker(),
            controller,
            ReadyCamera(clock, []),
            SampleLidar(clock, distances=[6.0, 7.0, 7.0]),
            Payload(),
            {3},
            GPSCoord(41.0, -81.0, 10.0),
            GPSCoord(41.1, -81.1, 10.0),
            precision_policy=policy(clock, desired_drop_height_m=7.0),
        )

    assert result is True
    hold = next(event for event in controller.events if event[0] == "hold")
    assert hold[2:] == (7.0, 7.0)
    assert controller.release_arguments is not None
    release_dropper, release_target, release_lidar, release_height = (
        controller.release_arguments
    )
    hold_target, hold_lidar, hold_height = controller.hold_arguments
    assert release_dropper.events == [("drop",)]
    assert release_target is hold_target
    assert release_lidar is hold_lidar
    assert release_height == hold_height == 7.0


def test_drop_correction_uses_observed_altitude_and_raw_lidar(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    controller = PrecisionController(clock, current_original_home_alt=12.0)
    lidar = SampleLidar(clock, distances=[8.0, 10.0, 10.0])
    requested_target = GPSCoord(41.1, -81.1, 30.0)
    monkeypatch.setattr(mission, "pickup_sequence", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        result = mission.fm3(
            Tracker(),
            controller,
            ReadyCamera(clock, []),
            lidar,
            Payload(),
            {3},
            GPSCoord(41.0, -81.0, 10.0),
            requested_target,
            precision_policy=policy(clock, desired_drop_height_m=10.0),
        )

    assert result is True
    corrected_moves = [
        event for event in controller.events if event[0] == "goto" and event[1].alt == 14.0
    ]
    assert len(corrected_moves) == 1
    corrected_target = corrected_moves[0][1]
    assert controller.hold_arguments == (corrected_target, lidar, 10.0)
    assert controller.release_arguments[1:] == (corrected_target, lidar, 10.0)


@pytest.mark.parametrize("release_result", [False, True, 0, object()])
def test_fm3_rejects_non_none_final_release_result(monkeypatch, release_result):
    import drone.mock_mission as mission

    class UnconfirmedReleaseController(PrecisionController):
        def release_payload_if_stable(
            self, dropper, waypoint, lidar, *, required_agl_m
        ):
            self.release_arguments = (dropper, waypoint, lidar, required_agl_m)
            return release_result

    clock = FakeClock()
    controller = UnconfirmedReleaseController(clock)
    payload = Payload()
    monkeypatch.setattr(mission, "pickup_sequence", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        with pytest.raises(RuntimeError, match="payload release"):
            mission.fm3(
                Tracker(),
                controller,
                ReadyCamera(clock, []),
                SampleLidar(clock, distances=[10.0] * 10),
                payload,
                {3},
                GPSCoord(41.0, -81.0, 10.0),
                GPSCoord(41.1, -81.1, 10.0),
                precision_policy=policy(clock),
            )

    assert controller.release_arguments is not None
    assert payload.events == []


def test_raised_final_release_guard_prevents_next_target(monkeypatch):
    import drone.mock_mission as mission

    class GuardedReleaseController(PrecisionController):
        def release_payload_if_stable(
            self, dropper, waypoint, lidar, *, required_agl_m
        ):
            self.events.append(("release_guard", required_agl_m))
            raise AuthorityLost("release authority changed")

    clock = FakeClock()
    controller = GuardedReleaseController(clock)
    payload = Payload()
    pickups = []
    monkeypatch.setattr(
        mission,
        "pickup_sequence",
        lambda *args, **kwargs: pickups.append(args[3]) or True,
    )

    with timebase.configured(clock):
        with pytest.raises(AuthorityLost, match="release authority changed"):
            mission.fm3(
                Tracker(),
                controller,
                ReadyCamera(clock, []),
                SampleLidar(clock, distances=[10.0] * 10),
                payload,
                {3, 4},
                GPSCoord(41.0, -81.0, 10.0),
                GPSCoord(41.1, -81.1, 10.0),
                precision_policy=policy(clock),
            )

    assert pickups == [3]
    assert payload.events == []
    assert [event[0] for event in controller.events].count("takeoff") == 1


def test_failed_calibrated_stability_gate_suppresses_release(monkeypatch):
    import drone.mock_mission as mission

    class UnstableController(PrecisionController):
        def hold_waypoint_until_stable(self, waypoint, lidar, required_agl_m):
            self.events.append(("hold", required_agl_m))
            return False

    clock = FakeClock()
    controller = UnstableController(clock)
    payload = Payload()
    monkeypatch.setattr(mission, "pickup_sequence", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        result = mission.fm3(
            Tracker(),
            controller,
            ReadyCamera(clock, []),
            SampleLidar(clock, distances=[10.0]),
            payload,
            {3},
            GPSCoord(41.0, -81.0, 10.0),
            GPSCoord(41.1, -81.1, 10.0),
            precision_policy=policy(clock),
        )

    assert result is False
    assert payload.events == []


def test_transient_stale_acquisition_sample_resets_then_recovers(monkeypatch):
    import drone.mock_mission as mission

    clock = FakeClock()
    stale = StaleSensorError("range revoked")
    lidar = SampleLidar(
        clock,
        distances=[4.572, 4.572, 4.572, stale] + [4.572] * 5,
    )
    camera = ReadyCamera(clock, [centered(i) for i in range(1, 8)])
    monkeypatch.setattr(mission, "aruco_land_precision", lambda *args, **kwargs: True)

    with timebase.configured(clock):
        result = mission.pickup_sequence(
            PrecisionController(clock),
            camera,
            lidar,
            3,
            Payload(),
            policy=policy(clock),
        )

    assert result is True
    assert len(camera.calls) == 7


def test_persistently_stale_acquisition_never_starts_land():
    from drone.mock_mission import pickup_sequence

    clock = FakeClock()
    controller = PrecisionController(clock)
    stale = StaleSensorError("range revoked")
    lidar = SampleLidar(
        clock,
        distances=[4.572, 4.572] + [stale] * 20,
    )

    with timebase.configured(clock):
        result = pickup_sequence(
            controller,
            ReadyCamera(clock, [centered(i) for i in range(1, 20)]),
            lidar,
            3,
            Payload(),
            policy=policy(clock, acquisition_timeout_s=0.2),
        )

    assert result is False
    assert not any(event[0] == "land" for event in controller.events)


def test_clock_failure_propagates_before_fm3_output():
    from drone.mock_mission import fm3

    class BrokenClock:
        def now(self):
            raise ValueError("clock backend failed")

        def sleep(self, seconds):
            raise AssertionError("sleep must not be reached")

    controller = PrecisionController(FakeClock())
    with timebase.configured(BrokenClock()):
        with pytest.raises(timebase.ClockError) as failure:
            fm3(
                Tracker(),
                controller,
                ReadyCamera(FakeClock(), []),
                SampleLidar(FakeClock()),
                Payload(),
                {3},
                GPSCoord(41.0, -81.0, 10.0),
                GPSCoord(41.1, -81.1, 10.0),
                precision_policy=policy(FakeClock()),
            )

    assert isinstance(failure.value.__cause__, ValueError)
    assert controller.events == []


@pytest.mark.parametrize("abort_type", [AuthorityLost, MissionAbort])
def test_camera_timeout_recheck_cannot_swallow_permission_abort(abort_type):
    from drone.mock_mission import aruco_land_precision

    class RevokedController(PrecisionController):
        def __init__(self, clock):
            super().__init__(clock)
            self.permission_checks = 0

        def check_permission(self):
            self.permission_checks += 1
            if self.permission_checks >= 4:
                raise abort_type("revoked during camera timeout")
            super().check_permission()

    clock = FakeClock()
    controller = RevokedController(clock)
    with timebase.configured(clock):
        with pytest.raises(abort_type, match="camera timeout"):
            aruco_land_precision(
                controller,
                ReadyCamera(clock, []),
                SampleLidar(clock),
                3,
                policy=policy(clock),
            )

    assert not any(event[0] == "confirm_landing" for event in controller.events)


def test_non_camera_timeout_during_evidence_propagates_without_confirmation():
    from drone.mock_mission import aruco_land_precision

    class ExpiredController(PrecisionController):
        def __init__(self, clock):
            super().__init__(clock)
            self.snapshot_calls = 0

        def flight_snapshot(self):
            self.snapshot_calls += 1
            if self.snapshot_calls == 2:
                raise TimeoutError("outer operation deadline expired")
            return super().flight_snapshot()

    clock = FakeClock()
    controller = ExpiredController(clock)
    with timebase.configured(clock):
        with pytest.raises(TimeoutError, match="outer operation deadline"):
            aruco_land_precision(
                controller,
                ReadyCamera(clock, [centered(1)]),
                SampleLidar(clock),
                3,
                policy=policy(clock),
            )

    assert not any(event[0] == "confirm_landing" for event in controller.events)
