import pytest
import sys
from types import SimpleNamespace

from drone.control.mission_supervisor import AuthorityLost
from drone.sensors.servo.servo import Dropper, PayloadPermissionError


class RecordingServo:
    def __init__(self):
        self.commands = []

    def max(self):
        self.commands.append("max")

    def mid(self):
        self.commands.append("mid")


def make_dropper(*, permission, sleeper=lambda _seconds: None, count=2, atomic=True):
    servos = [RecordingServo() for _ in range(count)]
    by_pin = dict(zip(range(1, count + 1), servos))

    def inert_factory(pin, *, initial_value):
        assert initial_value is None
        return by_pin[pin]

    if atomic and not callable(getattr(permission, "actuate", None)):
        check = permission

        class InertAtomicPermission:
            def __call__(self):
                return check()

            def actuate(self, output):
                if self() is not True:
                    raise PayloadPermissionError("payload permission is not current")
                output()

        permission = InertAtomicPermission()

    dropper = Dropper(
        pins=tuple(by_pin),
        permission=permission,
        release_hold_seconds=1.0,
        permission_check_interval_seconds=0.25,
        servo_factory=inert_factory,
        sleeper=sleeper,
    )
    return dropper, servos


def test_hardware_dropper_does_not_claim_attachment():
    dropper = object.__new__(Dropper)
    assert dropper.supports_attachment is False
    with pytest.raises(NotImplementedError):
        dropper.attach(3)


def test_drop_refuses_all_servo_commands_without_permission():
    allowed = True
    dropper, servos = make_dropper(permission=lambda: allowed)
    allowed = False

    with pytest.raises(PayloadPermissionError):
        dropper.drop()

    assert [servo.commands for servo in servos] == [[], []]


def test_permission_revocation_during_hold_issues_no_later_command():
    allowed = True

    def revoke_after_first_wait(_seconds):
        nonlocal allowed
        allowed = False

    dropper, servos = make_dropper(
        permission=lambda: allowed,
        sleeper=revoke_after_first_wait,
    )

    with pytest.raises(PayloadPermissionError):
        dropper.drop()

    assert [servo.commands for servo in servos] == [["max"], ["max"]]


def test_permission_is_checked_before_each_servo_command():
    checks = iter([True, True, True, False, False])
    dropper, servos = make_dropper(permission=lambda: next(checks))

    with pytest.raises(PayloadPermissionError):
        dropper.drop()

    assert [servo.commands for servo in servos] == [["max"], []]


def test_each_servo_command_uses_atomic_actuation_guard():
    """Break caught: a GPIO call cannot sit after an unprotected permission check."""
    class Permission:
        def __call__(self):
            return True

        def actuate(self, _output):
            raise PayloadPermissionError("revoked at GPIO boundary")

    dropper, servos = make_dropper(permission=Permission())

    with pytest.raises(PayloadPermissionError, match="GPIO boundary"):
        dropper.drop()

    assert [servo.commands for servo in servos] == [[], []]


def test_drop_fails_closed_when_atomic_actuation_guard_is_missing():
    dropper, servos = make_dropper(permission=lambda: True, atomic=False)

    with pytest.raises(PayloadPermissionError, match="atomic actuation"):
        dropper.drop()

    assert [servo.commands for servo in servos] == [[], []]


def test_drop_reports_only_a_completed_command_sequence():
    dropper, servos = make_dropper(permission=lambda: True)

    result = dropper.drop()

    assert result is None
    assert [servo.commands for servo in servos] == [
        ["max", "mid"],
        ["max", "mid"],
    ]


def test_hold_failure_preserves_original_error_and_attempts_every_neutral_position():
    primary = RuntimeError("hold failed")
    events = []

    class Servo:
        def __init__(self, pin):
            self.pin = pin

        def max(self):
            events.append(("max", self.pin))

        def mid(self):
            events.append(("mid", self.pin))
            if self.pin == 1:
                raise OSError("first neutral failed")

    dropper = Dropper(
        pins=(1, 2),
        permission=lambda: True,
        release_hold_seconds=1.0,
        permission_check_interval_seconds=0.25,
        servo_factory=lambda pin, *, initial_value: Servo(pin),
        sleeper=lambda _seconds: (_ for _ in ()).throw(primary),
    )

    with pytest.raises(RuntimeError) as captured:
        dropper.drop_with_guard(lambda output: output(), lambda output: output())

    assert captured.value is primary
    assert events == [("max", 1), ("max", 2), ("mid", 1), ("mid", 2)]


def test_neutral_failure_tries_every_channel_then_raises_first_error():
    first_error = OSError("first neutral failed")
    second_error = OSError("second neutral failed")
    events = []

    class Servo:
        def __init__(self, pin):
            self.pin = pin

        def max(self):
            events.append(("max", self.pin))

        def mid(self):
            events.append(("mid", self.pin))
            raise first_error if self.pin == 1 else second_error

    dropper = Dropper(
        pins=(1, 2),
        permission=lambda: True,
        release_hold_seconds=1.0,
        permission_check_interval_seconds=0.25,
        servo_factory=lambda pin, *, initial_value: Servo(pin),
        sleeper=lambda _seconds: None,
    )

    with pytest.raises(OSError) as captured:
        dropper.drop_with_guard(lambda output: output(), lambda output: output())

    assert captured.value is first_error
    assert events == [("max", 1), ("max", 2), ("mid", 1), ("mid", 2)]


def test_authority_loss_stops_later_release_and_neutralization_attempts():
    primary = AuthorityLost("pilot takeover")
    cleanup_denial = AuthorityLost("pilot takeover remains active")
    events = []
    continuation_attempts = []
    authority = ["COMPANION"]

    class Servo:
        def __init__(self, pin):
            self.pin = pin

        def max(self):
            events.append(("max", self.pin))
            authority[0] = "PILOT"

        def mid(self):
            events.append(("mid", self.pin))

    def release_actuate(output):
        if authority[0] != "COMPANION":
            raise primary
        output()

    def continuation_actuate(output):
        continuation_attempts.append(output.__self__.pin)
        if authority[0] != "COMPANION":
            raise cleanup_denial
        output()

    dropper = Dropper(
        pins=(1, 2),
        permission=lambda: True,
        release_hold_seconds=1.0,
        permission_check_interval_seconds=0.25,
        servo_factory=lambda pin, *, initial_value: Servo(pin),
        sleeper=lambda _seconds: None,
    )

    with pytest.raises(AuthorityLost) as captured:
        dropper.drop_with_guard(release_actuate, continuation_actuate)

    assert captured.value is primary
    assert continuation_attempts == [1]
    assert events == [("max", 1)]


@pytest.mark.parametrize(
    "override",
    [
        {"pins": ()},
        {"pins": (1, 1)},
        {"pins": (-1,)},
        {"release_hold_seconds": 0.0},
        {"permission_check_interval_seconds": 0.0},
        {"permission_check_interval_seconds": 2.0},
    ],
)
def test_dropper_rejects_invalid_hardware_and_hold_configuration(override):
    values = {
        "pins": (1,),
        "permission": lambda: True,
        "release_hold_seconds": 1.0,
        "permission_check_interval_seconds": 0.25,
        "servo_factory": lambda _pin, *, initial_value: RecordingServo(),
    }
    values.update(override)
    with pytest.raises(ValueError):
        Dropper(**values)


def test_default_gpio_factory_rejects_non_numeric_pulse_configuration_before_import():
    with pytest.raises(ValueError, match="pulse widths"):
        Dropper(
            pins=(1,),
            permission=lambda: True,
            release_hold_seconds=1.0,
            permission_check_interval_seconds=0.25,
            min_pulse_width="short",
            max_pulse_width="long",
        )


def test_gpiozero_construction_explicitly_requests_no_initial_output(monkeypatch):
    constructions = []

    class FakeGPIOZeroServo(RecordingServo):
        def __init__(self, pin, **kwargs):
            super().__init__()
            constructions.append((pin, kwargs))

    monkeypatch.setitem(
        sys.modules, "gpiozero", SimpleNamespace(Servo=FakeGPIOZeroServo)
    )

    Dropper(
        pins=(1, 2),
        permission=lambda: True,
        release_hold_seconds=1.0,
        permission_check_interval_seconds=0.25,
        min_pulse_width=0.0005,
        max_pulse_width=0.0025,
    )

    assert constructions == [
        (
            1,
            {
                "initial_value": None,
                "min_pulse_width": 0.0005,
                "max_pulse_width": 0.0025,
            },
        ),
        (
            2,
            {
                "initial_value": None,
                "min_pulse_width": 0.0005,
                "max_pulse_width": 0.0025,
            },
        ),
    ]


def test_denied_startup_constructs_no_servo_even_with_inert_factory():
    constructions = []

    def inert_factory(pin, *, initial_value):
        constructions.append((pin, initial_value))
        return RecordingServo()

    with pytest.raises(PayloadPermissionError):
        Dropper(
            pins=(1, 2),
            permission=lambda: False,
            release_hold_seconds=1.0,
            permission_check_interval_seconds=0.25,
            servo_factory=inert_factory,
        )

    assert constructions == []


def test_constructor_revocation_stops_before_the_next_inert_actuator():
    constructions = []
    checks = iter([True, False])

    def inert_factory(pin, *, initial_value):
        constructions.append((pin, initial_value))
        return RecordingServo()

    with pytest.raises(PayloadPermissionError):
        Dropper(
            pins=(1, 2),
            permission=lambda: next(checks),
            release_hold_seconds=1.0,
            permission_check_interval_seconds=0.25,
            servo_factory=inert_factory,
        )

    assert constructions == [(1, None)]
