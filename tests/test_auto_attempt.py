import pytest

from drone import timebase
from drone.auto_attempt import run_auto_attempt
from drone.common_types import GPSCoord


class FakeClock:
    def __init__(self, now_value: float = 0.0):
        self.now_value = now_value

    def now(self) -> float:
        return self.now_value

    def sleep(self, seconds: float) -> None:
        self.now_value += seconds


class FakeTracker:
    def __init__(self, calls):
        self.calls = calls

    def begin_mission(self):
        self.calls.append(("tracker", "begin"))


class FakePayload:
    def __init__(self, marker_id, calls):
        self.marker_id = marker_id
        self.calls = calls


class FakeController:
    def __init__(self, calls):
        self.calls = calls

    def goto_waypoint(self, waypoint):
        self.calls.append(("goto", waypoint))
        return 0

    def simple_land(self):
        self.calls.append(("land", "H"))
        return 0

    def disarm(self):
        self.calls.append(("disarm", "H"))
        return 0


class FakeMissionFunctions:
    def __init__(self, calls, clock=None):
        self.calls = calls
        self.clock = clock

    def fm1(self, tracker, controller, cruise_alt, waypoint_l):
        self.calls.append(("mission", "FM1", cruise_alt, waypoint_l))
        if self.clock is not None:
            self.clock.sleep(601.0)

    def fm2(
        self,
        tracker,
        controller,
        cruise_alt,
        drop_target,
        payload,
        lidar,
        desired_drop_height_m=10,
    ):
        self.calls.append(
            ("mission", "FM2", payload.marker_id, desired_drop_height_m)
        )

    def fm3(
        self,
        tracker,
        controller,
        camera,
        lidar,
        payload,
        possible_ids,
        pickup_point,
        target_point,
    ):
        marker_id = next(iter(possible_ids))
        self.calls.append(
            (
                "mission",
                f"FM3_{marker_id}",
                payload.marker_id,
                pickup_point,
                target_point,
            )
        )


class BlockingMissionFunctions(FakeMissionFunctions):
    def fm1(self, tracker, controller, cruise_alt, waypoint_l):
        for _ in range(7):
            timebase.sleep(100.0)
            self.calls.append(("blocking_tick", timebase.time()))


class FailedFm2MissionFunctions(FakeMissionFunctions):
    def fm2(
        self,
        tracker,
        controller,
        cruise_alt,
        drop_target,
        payload,
        lidar,
        desired_drop_height_m=10,
    ):
        self.calls.append(("mission", "FM2_FAILED"))
        return False


def fake_waypoints():
    return {
        "H": GPSCoord(41.0, -81.0, 0.0),
        "L": GPSCoord(41.1, -81.1, 0.0),
        "F2": GPSCoord(41.2, -81.2, 10.0),
        "WA": GPSCoord(41.3, -81.3, 0.0),
        "WM": GPSCoord(41.4, -81.4, 0.0),
    }


def event_phases(calls):
    return [
        phase
        for kind, phase, *rest in calls
        if kind == "event" and (not rest or rest[0] == "STARTED")
    ]


def test_auto_attempt_invokes_original_phases_and_home_in_order():
    """Regression: an outer rewrite must not replace or reorder original phases."""
    calls = []
    clock = FakeClock(now_value=900.0)

    with timebase.configured(clock):
        run_auto_attempt(
            tracker=FakeTracker(calls),
            controller=FakeController(calls),
            camera=object(),
            lidar=object(),
            payloads={
                2: FakePayload(2, calls),
                3: FakePayload(3, calls),
                4: FakePayload(4, calls),
            },
            waypoints=fake_waypoints(),
            emit=lambda phase, state: calls.append(("event", phase, state)),
            mission_functions=FakeMissionFunctions(calls),
        )

    assert event_phases(calls) == ["FM1", "FM2", "FM3_3", "FM3_4", "HOME"]
    assert [call[1] for call in calls if call[0] == "mission"] == [
        "FM1",
        "FM2",
        "FM3_3",
        "FM3_4",
    ]
    assert [call[2] for call in calls if call[0] == "mission" and "FM" in call[1]][
        1:
    ] == [2, 3, 4]
    assert [call for call in calls if call[0] in {"land", "disarm"}] == [
        ("land", "H"),
        ("disarm", "H"),
    ]


def test_home_emits_disarmed_only_after_controller_confirms_disarm():
    """Regression: HOME/COMPLETE alone must not stand in for observed disarm."""
    calls = []

    with timebase.configured(FakeClock()):
        run_auto_attempt(
            tracker=FakeTracker(calls),
            controller=FakeController(calls),
            camera=object(),
            lidar=object(),
            payloads={marker: FakePayload(marker, calls) for marker in (2, 3, 4)},
            waypoints=fake_waypoints(),
            emit=lambda phase, state: calls.append(("event", phase, state)),
            mission_functions=FakeMissionFunctions(calls),
        )

    disarm_index = calls.index(("disarm", "H"))
    disarmed_event_index = calls.index(("event", "HOME", "DISARMED"))
    complete_index = calls.index(("event", "HOME", "COMPLETE"))
    assert disarm_index < disarmed_event_index < complete_index


def test_home_complete_has_a_later_simulation_timestamp_than_disarmed():
    """Regression: equal-time Home tail events are invalid scoring evidence."""
    calls = []
    events = []
    clock = FakeClock()

    with timebase.configured(clock):
        run_auto_attempt(
            tracker=FakeTracker(calls),
            controller=FakeController(calls),
            camera=object(),
            lidar=object(),
            payloads={marker: FakePayload(marker, calls) for marker in (2, 3, 4)},
            waypoints=fake_waypoints(),
            emit=lambda phase, state: events.append((phase, state, clock.now())),
            mission_functions=FakeMissionFunctions(calls),
        )

    disarmed_time = next(
        timestamp
        for phase, state, timestamp in events
        if (phase, state) == ("HOME", "DISARMED")
    )
    complete_time = next(
        timestamp
        for phase, state, timestamp in events
        if (phase, state) == ("HOME", "COMPLETE")
    )
    assert disarmed_time < complete_time


def test_auto_attempt_deadline_is_relative_to_nonzero_mission_start():
    """Regression: a 600-second deadline must not be compared to clock epoch zero."""
    calls = []
    clock = FakeClock(now_value=900.0)

    with timebase.configured(clock):
        with pytest.raises(TimeoutError, match="600"):
            run_auto_attempt(
                tracker=FakeTracker(calls),
                controller=FakeController(calls),
                camera=object(),
                lidar=object(),
                payloads={
                    marker: FakePayload(marker, calls) for marker in (2, 3, 4)
                },
                waypoints=fake_waypoints(),
                emit=lambda phase, state: calls.append(("event", phase, state)),
                mission_functions=FakeMissionFunctions(calls, clock=clock),
            )

    assert clock.now_value == 1501.0
    assert ("event", "FM1", "COMPLETE") not in calls


def test_deadline_interrupts_an_original_blocking_phase():
    """Regression: a blocking original loop must not run beyond 600 seconds."""
    calls = []
    clock = FakeClock(now_value=900.0)

    with timebase.configured(clock):
        with pytest.raises(TimeoutError, match="600"):
            run_auto_attempt(
                tracker=FakeTracker(calls),
                controller=FakeController(calls),
                camera=object(),
                lidar=object(),
                payloads={
                    marker: FakePayload(marker, calls) for marker in (2, 3, 4)
                },
                waypoints=fake_waypoints(),
                emit=lambda phase, state: calls.append(("event", phase, state)),
                mission_functions=BlockingMissionFunctions(calls),
            )

    assert [call[1] for call in calls if call[0] == "blocking_tick"] == [
        1000.0,
        1100.0,
        1200.0,
        1300.0,
        1400.0,
        1500.0,
    ]


def test_explicit_phase_failure_aborts_the_single_attempt():
    """Regression: a rejected release gate must not emit phase completion."""
    calls = []

    with timebase.configured(FakeClock()):
        with pytest.raises(RuntimeError, match="FM2 failed"):
            run_auto_attempt(
                tracker=FakeTracker(calls),
                controller=FakeController(calls),
                camera=object(),
                lidar=object(),
                payloads={
                    marker: FakePayload(marker, calls) for marker in (2, 3, 4)
                },
                waypoints=fake_waypoints(),
                emit=lambda phase, state: calls.append(("event", phase, state)),
                mission_functions=FailedFm2MissionFunctions(calls),
            )

    assert ("event", "FM2", "COMPLETE") not in calls
    assert not any(
        call[:2] in {("event", "FM3_3"), ("event", "FM3_4")}
        for call in calls
    )
