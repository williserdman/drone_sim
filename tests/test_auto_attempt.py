import pytest

from drone import timebase
from drone.auto_attempt import run_auto_attempt
from drone.common_types import GPSCoord, MissionHome


class FakeClock:
    def __init__(self, now_value: float = 0.0, calls=None):
        self.now_value = now_value
        self.calls = calls

    def now(self) -> float:
        return self.now_value

    def sleep(self, seconds: float) -> None:
        if self.calls is not None:
            self.calls.append(("sleep", seconds))
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
    def __init__(
        self,
        calls,
        *,
        goto_result=0,
        land_result=0,
        disarm_result=0,
        mission_home=MissionHome(41.0, -81.0, 250.0),
    ):
        self.calls = calls
        self.goto_result = goto_result
        self.land_result = land_result
        self.disarm_result = disarm_result
        self.mission_home = mission_home

    def check_permission(self):
        self.calls.append(("permission",))
        return None

    def goto_waypoint(self, waypoint):
        self.calls.append(("goto", waypoint))
        return self.goto_result

    def simple_land(self):
        self.calls.append(("land", "H"))
        return self.land_result

    def disarm(self):
        self.calls.append(("disarm", "H"))
        return self.disarm_result


class FakeMissionFunctions:
    def __init__(self, calls, clock=None):
        self.calls = calls
        self.clock = clock

    def fm1(self, tracker, controller, cruise_alt, waypoint_l):
        self.calls.append(("mission", "FM1", cruise_alt, waypoint_l))
        if self.clock is not None:
            timebase.sleep(601.0)
        return True

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
        return True

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
        return True


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


class OriginalFm1ThenFailedFm2Functions(FakeMissionFunctions):
    def fm1(self, tracker, controller, cruise_alt, waypoint_l):
        from drone.missions.fm1 import fm1

        return fm1(tracker, controller, cruise_alt, waypoint_l)

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
        self.calls.append(("mission", "FM2_WHILE_ARMED"))
        return False


class UnconfirmedLDisarmController:
    def __init__(self, calls):
        self.calls = calls
        mav = type("Mav", (), {"statustext_send": lambda *_args: None})()
        master = type("Master", (), {"mav": mav})()
        self.vehicle = type("Vehicle", (), {"armed": True, "_master": master})()
        self.mission_home = MissionHome(41.0, -81.0, 250.0)

    def check_permission(self):
        return None

    def force_arm_takeoff(self, altitude):
        self.calls.append(("takeoff", altitude))

    def goto_waypoint(self, waypoint):
        self.calls.append(("goto", waypoint))
        return 0

    def simple_land(self):
        self.calls.append(("land", "L"))
        return 0

    def disarm(self):
        self.calls.append(("disarm", "L"))
        return -1


class NoneFm3MissionFunctions(FakeMissionFunctions):
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
        self.calls.append(("mission", "FM3_NONE"))
        return None


class PhaseResultMissionFunctions(FakeMissionFunctions):
    def __init__(self, calls, phase, result):
        super().__init__(calls)
        self.phase = phase
        self.result = result

    def fm1(self, *args):
        result = super().fm1(*args)
        return self.result if self.phase == "FM1" else result

    def fm2(self, *args, **kwargs):
        result = super().fm2(*args, **kwargs)
        return self.result if self.phase == "FM2" else result

    def fm3(self, *args):
        result = super().fm3(*args)
        marker_id = next(iter(args[5]))
        return self.result if self.phase == f"FM3_{marker_id}" else result


class PermissionController(FakeController):
    def __init__(self, calls, *, fail_at=None, permission_result=None, **kwargs):
        super().__init__(calls, **kwargs)
        self.fail_at = fail_at
        self.permission_result = permission_result
        self.permission_calls = 0

    def check_permission(self):
        self.permission_calls += 1
        self.calls.append(("permission", self.permission_calls))
        if self.permission_calls == self.fail_at:
            raise KeyboardInterrupt("operator abort")
        return self.permission_result


class DisarmRevokesPermissionController(PermissionController):
    def disarm(self):
        self.calls.append(("disarm", "H"))
        self.fail_at = self.permission_calls + 1
        return 0


def run_basic_attempt(calls, controller=None, functions=None, waypoints=None):
    controller = controller or FakeController(calls)
    run_auto_attempt(
        tracker=FakeTracker(calls),
        controller=controller,
        camera=object(),
        lidar=object(),
        payloads={marker: FakePayload(marker, calls) for marker in (2, 3, 4)},
        waypoints=waypoints or fake_waypoints(),
        emit=lambda phase, state: calls.append(("event", phase, state)),
        mission_functions=functions or FakeMissionFunctions(calls),
    )


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
        call[1]
        for call in calls
        if len(call) >= 3 and call[0] == "event" and call[2] == "STARTED"
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


def test_payload_phases_complete_after_one_physical_evidence_interval():
    """A phase cannot close before its 20 Hz attach or release fact is observable."""
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

    completion_times = {
        phase: timestamp
        for phase, state, timestamp in events
        if state == "COMPLETE" and phase in {"FM2", "FM3_3", "FM3_4"}
    }
    assert completion_times == pytest.approx(
        {"FM2": 0.05, "FM3_3": 0.1, "FM3_4": 0.15}
    )


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

    assert clock.now_value == 1500.0
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
    assert clock.now_value == 1500.0


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


def test_unconfirmed_fm1_disarm_prevents_fm2_from_starting():
    """FM1 must not return while the vehicle remains physically armed at L."""
    calls = []
    controller = UnconfirmedLDisarmController(calls)

    with timebase.configured(FakeClock()):
        with pytest.raises(RuntimeError, match="FM1 disarm was not confirmed"):
            run_auto_attempt(
                tracker=FakeTracker(calls),
                controller=controller,
                camera=object(),
                lidar=object(),
                payloads={
                    marker: FakePayload(marker, calls) for marker in (2, 3, 4)
                },
                waypoints=fake_waypoints(),
                emit=lambda phase, state: calls.append(("event", phase, state)),
                mission_functions=OriginalFm1ThenFailedFm2Functions(calls),
            )

    assert controller.vehicle.armed is True
    assert ("mission", "FM2_WHILE_ARMED") not in calls


def test_fm3_requires_explicit_true_before_emitting_complete():
    """Regression: implicit None from active FM3 must abort the attempt."""
    calls = []

    with timebase.configured(FakeClock()):
        with pytest.raises(RuntimeError, match="FM3_3 failed"):
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
                mission_functions=NoneFm3MissionFunctions(calls),
            )

    assert ("event", "FM3_3", "COMPLETE") not in calls


@pytest.mark.parametrize(
    ("goto_result", "land_result", "expected_physical_calls"),
    [
        (-1, 0, ["goto"]),
        (0, -1, ["goto", "land"]),
    ],
)
def test_home_failure_prevents_disarm_and_terminal_events(
    goto_result, land_result, expected_physical_calls
):
    """Regression: failed Home movement must not produce completion evidence."""
    calls = []
    controller = FakeController(
        calls, goto_result=goto_result, land_result=land_result
    )

    with timebase.configured(FakeClock()):
        with pytest.raises(RuntimeError, match="Home"):
            run_auto_attempt(
                tracker=FakeTracker(calls),
                controller=controller,
                camera=object(),
                lidar=object(),
                payloads={
                    marker: FakePayload(marker, calls) for marker in (2, 3, 4)
                },
                waypoints=fake_waypoints(),
                emit=lambda phase, state: calls.append(("event", phase, state)),
                mission_functions=FakeMissionFunctions(calls),
            )

    assert [call[0] for call in calls if call[0] in {"goto", "land", "disarm"}] == (
        expected_physical_calls
    )
    assert ("event", "HOME", "DISARMED") not in calls
    assert ("event", "HOME", "COMPLETE") not in calls


@pytest.mark.parametrize("phase", ["FM1", "FM2", "FM3_3", "FM3_4"])
@pytest.mark.parametrize(
    "result",
    [None, False, 0, 1, object()],
    ids=["none", "false", "zero", "one", "object"],
)
def test_every_phase_requires_exact_true_and_stops_later_work(phase, result):
    """A false-success phase result must not reach completion or the next phase."""
    calls = []

    with timebase.configured(FakeClock(calls=calls)):
        with pytest.raises(RuntimeError, match=rf"{phase} failed"):
            run_basic_attempt(
                calls,
                functions=PhaseResultMissionFunctions(calls, phase, result),
            )

    started = event_phases(calls)
    assert started[-1] == phase
    assert ("event", phase, "COMPLETE") not in calls
    assert not any(call[0] in {"goto", "land", "disarm"} for call in calls)
    expected_prior_waits = {"FM1": 0, "FM2": 0, "FM3_3": 1, "FM3_4": 2}
    assert sum(call[0] == "sleep" for call in calls) == expected_prior_waits[phase]


@pytest.mark.parametrize(
    ("operation", "result", "expected_operations"),
    [
        ("goto", None, ["goto"]),
        ("goto", False, ["goto"]),
        ("goto", True, ["goto"]),
        ("goto", 1, ["goto"]),
        ("goto", 0.0, ["goto"]),
        ("land", None, ["goto", "land"]),
        ("land", False, ["goto", "land"]),
        ("land", True, ["goto", "land"]),
        ("land", 1, ["goto", "land"]),
        ("land", 0.0, ["goto", "land"]),
        ("disarm", None, ["goto", "land", "disarm"]),
        ("disarm", False, ["goto", "land", "disarm"]),
        ("disarm", True, ["goto", "land", "disarm"]),
        ("disarm", 1, ["goto", "land", "disarm"]),
        ("disarm", 0.0, ["goto", "land", "disarm"]),
    ],
)
def test_home_operations_require_exact_nonbool_integer_zero(
    operation, result, expected_operations
):
    """HOME must reject values that compare equal to zero but are not exact int zero."""
    calls = []
    results = {"goto_result": 0, "land_result": 0, "disarm_result": 0}
    results[f"{operation}_result"] = result
    controller = FakeController(calls, **results)

    with timebase.configured(FakeClock(calls=calls)):
        with pytest.raises(RuntimeError, match="Home"):
            run_basic_attempt(calls, controller=controller)

    assert [
        call[0] for call in calls if call[0] in {"goto", "land", "disarm"}
    ] == expected_operations
    assert ("event", "HOME", "DISARMED") not in calls
    assert ("event", "HOME", "COMPLETE") not in calls


@pytest.mark.parametrize(
    "mission_home",
    [
        None,
        GPSCoord(41.0, -81.0, 250.0),
        MissionHome(float("nan"), -81.0, 250.0),
        MissionHome(91.0, -81.0, 250.0),
        MissionHome(41.0, -181.0, 250.0),
        MissionHome(True, -81.0, 250.0),
    ],
    ids=["missing", "wrong-type", "nonfinite", "latitude", "longitude", "bool"],
)
def test_invalid_pinned_home_fails_before_attempt_work(mission_home):
    """An invalid external pin must stop before mission state or phase work starts."""
    calls = []
    controller = FakeController(calls, mission_home=mission_home)

    with timebase.configured(FakeClock(calls=calls)):
        with pytest.raises(RuntimeError, match="mission home"):
            run_basic_attempt(calls, controller=controller)

    assert calls == []


@pytest.mark.parametrize("permission_result", [False, True, 0, object()])
def test_permission_check_must_return_exact_none_before_attempt(permission_result):
    """A guard's false-success value must not authorize mission state changes."""
    calls = []
    controller = PermissionController(calls, permission_result=permission_result)

    with timebase.configured(FakeClock(calls=calls)):
        with pytest.raises(RuntimeError, match="permission"):
            run_basic_attempt(calls, controller=controller)

    assert calls == [("permission", 1)]


@pytest.mark.parametrize("permission", [None, 1])
def test_permission_check_must_be_callable_before_attempt(permission):
    calls = []
    controller = FakeController(calls)
    controller.check_permission = permission

    with timebase.configured(FakeClock(calls=calls)):
        with pytest.raises(RuntimeError, match="permission"):
            run_basic_attempt(calls, controller=controller)

    assert calls == []


def test_permission_is_checked_at_all_phase_and_home_boundaries_in_order():
    """Removing any boundary guard would permit a later event or physical operation."""
    calls = []
    controller = PermissionController(calls)

    with timebase.configured(FakeClock(calls=calls)):
        run_basic_attempt(calls, controller=controller)

    assert calls == [
        ("permission", 1),
        ("tracker", "begin"),
        ("permission", 2),
        ("event", "FM1", "STARTED"),
        ("mission", "FM1", 10.0, fake_waypoints()["L"]),
        ("permission", 3),
        ("event", "FM1", "COMPLETE"),
        ("permission", 4),
        ("event", "FM2", "STARTED"),
        ("mission", "FM2", 2, 10),
        ("permission", 5),
        ("sleep", 0.05),
        ("permission", 6),
        ("event", "FM2", "COMPLETE"),
        ("permission", 7),
        ("event", "FM3_3", "STARTED"),
        (
            "mission",
            "FM3_3",
            3,
            fake_waypoints()["WA"],
            fake_waypoints()["F2"],
        ),
        ("permission", 8),
        ("sleep", 0.05),
        ("permission", 9),
        ("event", "FM3_3", "COMPLETE"),
        ("permission", 10),
        ("event", "FM3_4", "STARTED"),
        (
            "mission",
            "FM3_4",
            4,
            fake_waypoints()["WM"],
            fake_waypoints()["F2"],
        ),
        ("permission", 11),
        ("sleep", 0.05),
        ("permission", 12),
        ("event", "FM3_4", "COMPLETE"),
        ("permission", 13),
        ("event", "HOME", "STARTED"),
        ("permission", 14),
        ("goto", GPSCoord(41.0, -81.0, 10.0)),
        ("permission", 15),
        ("land", "H"),
        ("permission", 16),
        ("disarm", "H"),
        ("permission", 17),
        ("event", "HOME", "DISARMED"),
        ("permission", 18),
        ("sleep", 0.05),
        ("permission", 19),
        ("event", "HOME", "COMPLETE"),
    ]


@pytest.mark.parametrize("fail_at", range(1, 20))
def test_ctrl_c_at_any_permission_boundary_propagates_without_local_recovery(fail_at):
    """Permission cancellation must stop on the exact boundary that observes it."""
    calls = []
    controller = PermissionController(calls, fail_at=fail_at)

    with timebase.configured(FakeClock(calls=calls)):
        with pytest.raises(KeyboardInterrupt, match="operator abort"):
            run_basic_attempt(calls, controller=controller)

    assert calls[-1] == ("permission", fail_at)
    assert not any(call[0] in {"rtl", "reconnect", "rearm"} for call in calls)


def test_disarm_permission_revocation_prevents_normal_disarmed_event():
    """A successful disarm result cannot authorize an event after permission loss."""
    calls = []
    controller = DisarmRevokesPermissionController(calls)

    with timebase.configured(FakeClock(calls=calls)):
        with pytest.raises(KeyboardInterrupt, match="operator abort"):
            run_basic_attempt(calls, controller=controller)

    assert calls[-2:] == [("disarm", "H"), ("permission", 17)]
    assert ("event", "HOME", "DISARMED") not in calls
    assert ("event", "HOME", "COMPLETE") not in calls
    assert not any(call[0] in {"rtl", "reconnect", "rearm"} for call in calls)


def test_home_uses_once_captured_controller_pin_not_mutable_saved_h():
    """Changing stored H after takeoff must not change the return authority."""
    calls = []
    waypoints = fake_waypoints()
    controller = FakeController(
        calls, mission_home=MissionHome(40.5, -80.5, 300.0)
    )

    class MutatingFunctions(FakeMissionFunctions):
        def fm3(self, *args):
            result = super().fm3(*args)
            waypoints["H"] = GPSCoord(1.0, 2.0, 3.0)
            return result

    with timebase.configured(FakeClock(calls=calls)):
        run_basic_attempt(
            calls,
            controller=controller,
            functions=MutatingFunctions(calls),
            waypoints=waypoints,
        )

    assert [call for call in calls if call[0] == "goto"] == [
        ("goto", GPSCoord(40.5, -80.5, 10.0))
    ]
