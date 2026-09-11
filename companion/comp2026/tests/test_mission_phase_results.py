import importlib
import math
from types import SimpleNamespace

import pytest

from drone.common_types import GPSCoord
from drone.control.mission_supervisor import (
    AuthorityLost,
    FlightOperationError,
    MissionAbort,
)
from drone.missions.fm1 import fm1
from drone.missions.fm2 import fm2


class RecordingController:
    def __init__(
        self,
        *,
        takeoff_result=None,
        goto_result=0,
        land_result=0,
        disarm_result=0,
        stable_result=True,
        failures=None,
        permission_failure=None,
        release_waypoint=None,
        configuration_error=None,
    ):
        mav = SimpleNamespace(statustext_send=lambda severity, message: None)
        self.vehicle = SimpleNamespace(_master=SimpleNamespace(mav=mav))
        self.events = []
        self.takeoff_result = takeoff_result
        self.goto_result = goto_result
        self.land_result = land_result
        self.disarm_result = disarm_result
        self.stable_result = stable_result
        self.failures = failures or {}
        self.permission_failure = permission_failure
        self.release_waypoint = release_waypoint
        self.configuration_error = configuration_error
        self.permission_checks = 0

    def _result(self, name, result):
        error = self.failures.get(name)
        if error is not None:
            raise error
        return result

    def check_permission(self):
        self.permission_checks += 1
        self.events.append(("permission", self.permission_checks))
        if self.permission_failure is not None:
            check_number, error = self.permission_failure
            if self.permission_checks == check_number:
                raise error

    def require_release_configuration(self):
        self.events.append(("release_configuration",))
        if self.configuration_error is not None:
            raise self.configuration_error

    def force_arm_takeoff(self, altitude):
        self.events.append(("takeoff", altitude))
        return self._result("takeoff", self.takeoff_result)

    def goto_waypoint(self, waypoint):
        self.events.append(("goto", waypoint))
        return self._result("goto", self.goto_result)

    def simple_land(self):
        self.events.append(("land",))
        return self._result("land", self.land_result)

    def disarm(self):
        self.events.append(("disarm",))
        return self._result("disarm", self.disarm_result)

    def hold_waypoint_until_stable(self, waypoint, lidar, *, required_agl_m):
        self.events.append(("stable", waypoint, lidar, required_agl_m))
        return self._result("stable", self.stable_result)

    def release_waypoint_for_clearance(self, waypoint, lidar, *, desired_agl_m):
        self.events.append(("release_correction", waypoint, lidar, desired_agl_m))
        if self.release_waypoint is not None:
            return self.release_waypoint
        return GPSCoord(waypoint.lat, waypoint.long, 35.0)

    def release_payload_if_stable(
        self, dropper, waypoint, lidar, *, required_agl_m
    ):
        self.events.append(("release", waypoint, lidar, required_agl_m))
        result = dropper.drop()
        if result is not None:
            raise RuntimeError("payload release command returned an invalid result")


class RecordingLidar:
    def __init__(self, distance=10.0):
        self.distance = distance
        self.events = []

    def get_distance(self):
        self.events.append(("distance",))
        return self.distance


class RecordingDropper:
    def __init__(self, result=None, error=None):
        self.result = result
        self.error = error
        self.calls = 0

    def drop(self):
        self.calls += 1
        if self.error is not None:
            raise self.error
        return self.result


def call_fm1(controller):
    return fm1(object(), controller, 10, GPSCoord(41.0, -81.0, 3.0))


def call_fm2(controller, dropper=None, lidar=None, desired_height=15):
    dropper = dropper or RecordingDropper()
    lidar = lidar or RecordingLidar()
    result = fm2(
        object(),
        controller,
        30,
        GPSCoord(41.0, -81.0, 4.0),
        dropper,
        lidar,
        desired_drop_height_m=desired_height,
    )
    return result, dropper, lidar


@pytest.mark.parametrize("takeoff_result", [None, 0])
def test_fm1_returns_true_only_after_all_confirmed_steps(takeoff_result):
    controller = RecordingController(takeoff_result=takeoff_result)

    assert call_fm1(controller) is True
    assert controller.events == [
        ("permission", 1),
        ("takeoff", 10),
        ("permission", 2),
        ("goto", GPSCoord(41.0, -81.0, 10)),
        ("permission", 3),
        ("land",),
        ("permission", 4),
        ("disarm",),
    ]


@pytest.mark.parametrize(
    ("result_name", "bad_result", "forbidden_event"),
    [
        ("takeoff_result", False, "goto"),
        ("takeoff_result", object(), "goto"),
        ("goto_result", False, "land"),
        ("goto_result", -1, "land"),
        ("land_result", False, "disarm"),
        ("land_result", -1, "disarm"),
        ("disarm_result", False, None),
        ("disarm_result", None, None),
    ],
)
def test_fm1_rejects_unconfirmed_legacy_results(
    result_name, bad_result, forbidden_event
):
    controller = RecordingController(**{result_name: bad_result})

    with pytest.raises(RuntimeError):
        call_fm1(controller)

    if forbidden_event is not None:
        assert all(event[0] != forbidden_event for event in controller.events)


@pytest.mark.parametrize("operation", ["takeoff", "goto", "land", "disarm"])
def test_fm1_propagates_operation_exceptions_without_later_steps(operation):
    error = FlightOperationError(f"{operation} failed")
    controller = RecordingController(failures={operation: error})

    with pytest.raises(FlightOperationError) as raised:
        call_fm1(controller)

    assert raised.value is error
    operation_names = ["takeoff", "goto", "land", "disarm"]
    failed_index = operation_names.index(operation)
    later_names = set(operation_names[failed_index + 1 :])
    assert later_names.isdisjoint(event[0] for event in controller.events)


@pytest.mark.parametrize("error_type", [AuthorityLost, MissionAbort])
@pytest.mark.parametrize("permission_check", [1, 2, 3, 4])
def test_fm1_propagates_permission_interruptions_at_every_boundary(
    error_type, permission_check
):
    error = error_type("stop")
    controller = RecordingController(
        permission_failure=(permission_check, error)
    )

    with pytest.raises(error_type) as raised:
        call_fm1(controller)

    assert raised.value is error
    assert controller.permission_checks == permission_check


@pytest.mark.parametrize(
    "desired_height",
    [False, 0, -1, math.inf, -math.inf, math.nan, "15"],
)
def test_fm2_rejects_invalid_drop_height_before_flight_output(desired_height):
    controller = RecordingController()
    dropper = RecordingDropper()
    lidar = RecordingLidar()

    with pytest.raises(ValueError):
        call_fm2(controller, dropper, lidar, desired_height)

    assert controller.events == []
    assert lidar.events == []
    assert dropper.calls == 0


def test_fm2_rejects_missing_release_configuration_before_flight_output():
    error = FlightOperationError("release configuration missing")
    controller = RecordingController(configuration_error=error)
    dropper = RecordingDropper()

    with pytest.raises(FlightOperationError) as raised:
        call_fm2(controller, dropper)

    assert raised.value is error
    assert controller.events == [("release_configuration",)]
    assert dropper.calls == 0


@pytest.mark.parametrize("takeoff_result", [None, 0])
def test_fm2_uses_desired_height_and_reports_command_attempt(
    monkeypatch, takeoff_result
):
    messages = []
    fm2_module = importlib.import_module("drone.missions.fm2")
    monkeypatch.setattr(
        fm2_module, "log", lambda message, master: messages.append(message)
    )
    controller = RecordingController(takeoff_result=takeoff_result)

    result, dropper, lidar = call_fm2(controller)

    assert result is True
    assert dropper.calls == 1
    assert lidar.events == []
    assert controller.events == [
        ("release_configuration",),
        ("permission", 1),
        ("takeoff", 30),
        ("permission", 2),
        ("goto", GPSCoord(41.0, -81.0, 30)),
        ("permission", 3),
        ("release_correction", GPSCoord(41.0, -81.0, 30), lidar, 15),
        ("permission", 4),
        ("goto", GPSCoord(41.0, -81.0, 35.0)),
        ("stable", GPSCoord(41.0, -81.0, 35.0), lidar, 15),
        ("release", GPSCoord(41.0, -81.0, 35.0), lidar, 15),
    ]
    assert any("release command attempted" in message for message in messages)
    assert all("payload dropped" not in message for message in messages)


@pytest.mark.parametrize(
    ("result_name", "bad_result", "forbidden_event"),
    [
        ("takeoff_result", False, "goto"),
        ("takeoff_result", object(), "goto"),
        ("goto_result", False, "stable"),
        ("goto_result", -1, "stable"),
    ],
)
def test_fm2_rejects_unconfirmed_flight_results(
    result_name, bad_result, forbidden_event
):
    controller = RecordingController(**{result_name: bad_result})
    dropper = RecordingDropper()

    with pytest.raises(RuntimeError):
        call_fm2(controller, dropper)

    assert all(event[0] != forbidden_event for event in controller.events)
    assert dropper.calls == 0


@pytest.mark.parametrize("stable_result", [False, None, 0, 1, object()])
def test_fm2_requires_exact_true_stability_before_payload(stable_result):
    controller = RecordingController(stable_result=stable_result)
    dropper = RecordingDropper()

    result, _, _ = call_fm2(controller, dropper)

    assert result is False
    assert dropper.calls == 0
    assert controller.permission_checks == 4


@pytest.mark.parametrize("drop_result", [False, True, 0, "released"])
def test_fm2_rejects_non_none_drop_results(drop_result):
    controller = RecordingController()
    dropper = RecordingDropper(result=drop_result)

    with pytest.raises(RuntimeError):
        call_fm2(controller, dropper)

    assert dropper.calls == 1


@pytest.mark.parametrize("operation", ["takeoff", "goto", "stable"])
def test_fm2_propagates_operation_exceptions_without_payload(operation):
    error = FlightOperationError(f"{operation} failed")
    controller = RecordingController(failures={operation: error})
    dropper = RecordingDropper()

    with pytest.raises(FlightOperationError) as raised:
        call_fm2(controller, dropper)

    assert raised.value is error
    assert dropper.calls == 0


def test_fm2_propagates_drop_exception():
    error = FlightOperationError("drop failed")
    controller = RecordingController()
    dropper = RecordingDropper(error=error)

    with pytest.raises(FlightOperationError) as raised:
        call_fm2(controller, dropper)

    assert raised.value is error


@pytest.mark.parametrize("error_type", [AuthorityLost, MissionAbort])
@pytest.mark.parametrize("permission_check", [1, 2, 3, 4])
def test_fm2_propagates_permission_interruptions_at_every_boundary(
    error_type, permission_check
):
    error = error_type("stop")
    controller = RecordingController(
        permission_failure=(permission_check, error)
    )
    dropper = RecordingDropper()

    with pytest.raises(error_type) as raised:
        call_fm2(controller, dropper)

    assert raised.value is error
    assert controller.permission_checks == permission_check
    assert dropper.calls == 0
