from types import SimpleNamespace

import pytest

from drone import timebase
from drone.control import listener
from drone.control.listener import TelemetryStartupCollector
from drone.control.mission_supervisor import FlightOperationError, MissionAbort
from drone.control.listener_runtime import (
    AutopilotVersionContract,
    TelemetryStartupPolicy,
)
from test_listener import profile


class UnprintableCleanupFailure(KeyboardInterrupt):
    def __str__(self):
        raise SystemExit("cleanup formatting exited")


class Clock:
    def __init__(self):
        self.value = 0.0
        self.on_sleep = lambda: None

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds
        self.on_sleep()


class WallClock:
    def __init__(self):
        self.value = 100.0
        self.on_sleep = lambda: None

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds
        self.on_sleep()


class Vehicle:
    def __init__(self):
        self.listeners = {}

    def add_message_listener(self, name, callback):
        self.listeners[name] = callback

    def remove_message_listener(self, name, callback):
        if self.listeners.get(name) == callback:
            del self.listeners[name]

    def emit(self, message):
        self.listeners["*"](self, message.get_type(), message)


class Controller:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.calls = []
        self.closed = False

    def set_telemetry_interval(self, message_id, interval_us, **limits):
        assert "*" in self.vehicle.listeners
        self.calls.append(("interval", message_id, interval_us, limits))

    def request_autopilot_version(self, **limits):
        assert "*" in self.vehicle.listeners
        self.calls.append(("version", limits))
        timebase.sleep(0.01)
        self.vehicle.emit(version_message())

    def close_startup_telemetry(self):
        self.closed = True

    def check_permission(self):
        return None


def message(message_id, *, system=1, component=1, **fields):
    return SimpleNamespace(
        **fields,
        get_msgId=lambda: message_id,
        get_type=lambda: "TEST",
        get_srcSystem=lambda: system,
        get_srcComponent=lambda: component,
    )


def version_message(**changes):
    fields = {
        "flight_sw_version": 0x040507FF,
        "flight_custom_version": b"abcdef0\0",
    }
    fields.update(changes)
    return message(148, **fields)


def policy():
    return TelemetryStartupPolicy(
        command_ack_timeout_s=1.0,
        collection_timeout_s=3.0,
        home_request_timeout_s=1.0,
        poll_interval_s=0.5,
        minimum_distinct_samples=3,
        maximum_interval_error_fraction=0.25,
    )


def version_contract():
    return AutopilotVersionContract(
        firmware_label="ArduCopter 4.5.7",
        flight_sw_version=0x040507FF,
        flight_custom_version=b"abcdef0\0",
        evidence_reference="test fixture mapping",
    )


def test_callbacks_precede_requests_and_distinct_achieved_cadence_is_required():
    clock = Clock()
    vehicle = Vehicle()
    controller = Controller(vehicle)
    collector = TelemetryStartupCollector(
        controller=controller,
        flight_profile=profile(),
        policy=policy(),
        autopilot_version=version_contract(),
        clock=timebase.monotonic,
    )
    emitted = [0]

    def emit_heartbeat():
        emitted[0] += 1
        vehicle.emit(message(0))

    clock.on_sleep = emit_heartbeat

    with timebase.configured(clock):
        evidence = collector.configure_and_collect()

    assert len(evidence.received_at_by_message_id[0]) == 3
    assert evidence.autopilot_version_received_at == 0.01
    assert controller.calls[0][0] == "interval"
    assert controller.calls[-1][0] == "version"
    assert controller.closed is True
    assert "*" not in vehicle.listeners
    collector._observe(vehicle, "TEST", message(0))
    assert len(evidence.received_at_by_message_id[0]) == 3


def test_wrong_source_version_is_ignored_and_startup_times_out_closed():
    clock = Clock()
    vehicle = Vehicle()
    controller = Controller(vehicle)

    def wrong_source_version(**limits):
        controller.calls.append(("version", limits))
        vehicle.emit(version_message(system=44))

    controller.request_autopilot_version = wrong_source_version
    collector = TelemetryStartupCollector(
        controller=controller,
        flight_profile=profile(),
        policy=policy(),
        autopilot_version=version_contract(),
        clock=timebase.monotonic,
    )
    clock.on_sleep = lambda: vehicle.emit(message(0))

    with timebase.configured(clock), pytest.raises(
        TimeoutError, match="AUTOPILOT_VERSION"
    ):
        collector.configure_and_collect()

    assert controller.closed is True


def test_mismatched_actual_firmware_fails_closed():
    clock = Clock()
    vehicle = Vehicle()
    controller = Controller(vehicle)

    def mismatched_version(**limits):
        controller.calls.append(("version", limits))
        vehicle.emit(version_message(flight_custom_version=b"different"))

    controller.request_autopilot_version = mismatched_version
    collector = TelemetryStartupCollector(
        controller=controller,
        flight_profile=profile(),
        policy=policy(),
        autopilot_version=version_contract(),
        clock=timebase.monotonic,
    )

    with timebase.configured(clock), pytest.raises(
        RuntimeError, match="firmware metadata"
    ):
        collector.configure_and_collect()

    assert controller.closed is True


def test_same_time_burst_and_old_window_do_not_prove_cadence():
    clock = Clock()
    vehicle = Vehicle()
    controller = Controller(vehicle)
    collector = TelemetryStartupCollector(
        controller=controller,
        flight_profile=profile(),
        policy=policy(),
        autopilot_version=version_contract(),
        clock=timebase.monotonic,
    )

    emitted = [False]

    def burst_then_age():
        if not emitted[0]:
            emitted[0] = True
            for _ in range(3):
                vehicle.emit(message(0))
            clock.value = 20.0

    clock.on_sleep = burst_then_age
    with timebase.configured(clock), pytest.raises(
        TimeoutError, match="cadence"
    ):
        collector.configure_and_collect()

    assert "*" not in vehicle.listeners


def staged_collector(monkeypatch):
    simulation_clock = Clock()
    wall_clock = WallClock()
    vehicle = Vehicle()
    controller = Controller(vehicle)

    def request_version(**limits):
        controller.calls.append(("version", limits))
        vehicle.emit(version_message())

    controller.request_autopilot_version = request_version
    monkeypatch.setattr(listener.time, "monotonic", wall_clock.now)
    monkeypatch.setattr(listener.time, "sleep", wall_clock.sleep)
    collector = TelemetryStartupCollector(
        controller=controller,
        flight_profile=profile(),
        policy=policy(),
        autopilot_version=version_contract(),
        clock=timebase.monotonic,
    )
    return simulation_clock, wall_clock, vehicle, controller, collector


def test_staged_prepare_becomes_ready_without_telemetry_cadence(monkeypatch):
    simulation_clock, _wall_clock, vehicle, controller, collector = staged_collector(
        monkeypatch
    )

    with timebase.configured(simulation_clock):
        assert collector.prepare() is None

    assert controller.calls[0][0] == "interval"
    assert controller.calls[-1][0] == "version"
    assert controller.closed is True
    assert "*" in vehicle.listeners
    collector.close()
    collector.close()
    assert "*" not in vehicle.listeners


def test_staged_verification_discards_pre_gate_samples_and_uses_advancing_sim_time(
    monkeypatch,
):
    simulation_clock, wall_clock, vehicle, _controller, collector = staged_collector(
        monkeypatch
    )
    with timebase.configured(simulation_clock):
        collector.prepare()
        for observed_at in (0.5, 1.0, 1.5):
            simulation_clock.value = observed_at
            vehicle.emit(message(0))

        def advance_and_emit():
            simulation_clock.value += 0.5
            vehicle.emit(message(0))

        wall_clock.on_sleep = advance_and_emit
        evidence = collector.verify_after_guided()

    assert evidence.received_at_by_message_id[0] == (2.0, 2.5, 3.0)
    assert "*" not in vehicle.listeners


def test_staged_verification_rejects_zero_time_and_wrong_source_samples(monkeypatch):
    simulation_clock, wall_clock, vehicle, controller, collector = staged_collector(
        monkeypatch
    )
    with timebase.configured(simulation_clock):
        collector.prepare()

        def emit_invalid_samples():
            vehicle.emit(message(0))
            vehicle.emit(message(0, system=44))

        wall_clock.on_sleep = emit_invalid_samples
        with pytest.raises(FlightOperationError, match="cadence"):
            collector.verify_after_guided()

    assert controller.closed is True
    assert "*" not in vehicle.listeners


def test_staged_verification_closes_when_shared_clock_stops(monkeypatch):
    simulation_clock, _wall_clock, vehicle, controller, collector = staged_collector(
        monkeypatch
    )
    with timebase.configured(simulation_clock):
        collector.prepare()
        simulation_clock.now = lambda: (_ for _ in ()).throw(
            timebase.ClockError("simulation stopped")
        )
        with pytest.raises(timebase.ClockError, match="simulation stopped"):
            collector.verify_after_guided()

    assert controller.closed is True
    assert "*" not in vehicle.listeners


def test_staged_verification_detects_clock_stop_after_first_wall_sleep(monkeypatch):
    simulation_clock, wall_clock, vehicle, controller, collector = staged_collector(
        monkeypatch
    )
    original = timebase.ClockError("simulation stopped during cadence wait")
    with timebase.configured(simulation_clock):
        collector.prepare()
        wall_clock.on_sleep = lambda: setattr(
            simulation_clock,
            "now",
            lambda: (_ for _ in ()).throw(original),
        )
        with pytest.raises(timebase.ClockError) as caught:
            collector.verify_after_guided()

    assert caught.value is original
    assert controller.closed is True
    assert "*" not in vehicle.listeners


def test_staged_verification_checks_abort_cooperatively_while_waiting(monkeypatch):
    simulation_clock, _wall_clock, vehicle, controller, collector = staged_collector(
        monkeypatch
    )
    original = MissionAbort("QGC abort during cadence wait")
    checks = [0]

    def check_permission():
        checks[0] += 1
        if checks[0] == 2:
            raise original

    controller.check_permission = check_permission
    with timebase.configured(simulation_clock):
        collector.prepare()
        with pytest.raises(MissionAbort) as caught:
            collector.verify_after_guided()

    assert caught.value is original
    assert controller.closed is True
    assert "*" not in vehicle.listeners


@pytest.mark.parametrize(
    "primary",
    [RuntimeError("interval request failed"), KeyboardInterrupt("startup cancelled")],
)
def test_complete_collection_preserves_primary_when_listener_cleanup_is_interrupted(
    primary,
):
    clock = Clock()
    vehicle = Vehicle()
    controller = Controller(vehicle)
    calls = []

    def fail_request(*_args, **_kwargs):
        raise primary

    def fail_remove(name, callback):
        calls.append(("remove", name, callback))
        raise KeyboardInterrupt("listener removal interrupted")

    controller.set_telemetry_interval = fail_request
    controller.close_startup_telemetry = lambda: calls.append("close requests")
    vehicle.remove_message_listener = fail_remove
    collector = TelemetryStartupCollector(
        controller=controller,
        flight_profile=profile(),
        policy=policy(),
        autopilot_version=version_contract(),
        clock=timebase.monotonic,
    )

    with timebase.configured(clock), pytest.raises(BaseException) as raised:
        collector.configure_and_collect()

    assert raised.value is primary
    traceback_tail = raised.value.__traceback__
    while traceback_tail.tb_next is not None:
        traceback_tail = traceback_tail.tb_next
    assert traceback_tail.tb_frame.f_code.co_name == "fail_request"
    assert isinstance(raised.value.__cause__, RuntimeError)
    assert "listener removal interrupted" in str(raised.value.__cause__)
    assert calls[0][0] == "remove"
    assert calls[1] == "close requests"

    collector.close()
    assert len(calls) == 2


def test_close_reports_both_cleanup_failures_after_attempting_each_once():
    vehicle = Vehicle()
    controller = Controller(vehicle)
    calls = []

    def fail_remove(_name, _callback):
        calls.append("remove")
        raise KeyboardInterrupt("listener removal interrupted")

    def fail_close_requests():
        calls.append("close requests")
        raise SystemExit("request cleanup exited")

    vehicle.remove_message_listener = fail_remove
    controller.close_startup_telemetry = fail_close_requests
    collector = TelemetryStartupCollector(
        controller=controller,
        flight_profile=profile(),
        policy=policy(),
        autopilot_version=version_contract(),
        clock=timebase.monotonic,
    )
    collector._begin_collection()

    with pytest.raises(RuntimeError) as raised:
        collector.close()

    assert calls == ["remove", "close requests"]
    assert "listener removal interrupted" in str(raised.value)
    assert "request cleanup exited" in str(raised.value)
    assert tuple(label for label, _error in raised.value.errors) == (
        "listener removal",
        "startup request cleanup",
    )
    assert isinstance(raised.value.errors[0][1], KeyboardInterrupt)
    assert isinstance(raised.value.errors[1][1], SystemExit)

    collector.close()
    assert calls == ["remove", "close requests"]


def test_prepare_preserves_primary_and_runs_each_cleanup_once():
    simulation_clock = Clock()
    vehicle = Vehicle()
    controller = Controller(vehicle)
    primary = MissionAbort("abort during startup preparation")
    removal_error = KeyboardInterrupt("listener removal interrupted")
    request_error = SystemExit("request cleanup exited")
    calls = []

    def fail_version(**_limits):
        raise primary

    def fail_remove(_name, _callback):
        calls.append("remove")
        raise removal_error

    def fail_close_requests():
        calls.append("close requests")
        raise request_error

    vehicle.remove_message_listener = fail_remove
    controller.close_startup_telemetry = fail_close_requests
    controller.request_autopilot_version = fail_version
    collector = TelemetryStartupCollector(
        controller=controller,
        flight_profile=profile(),
        policy=policy(),
        autopilot_version=version_contract(),
        clock=timebase.monotonic,
    )

    with timebase.configured(simulation_clock), pytest.raises(MissionAbort) as raised:
        collector.prepare()

    assert raised.value is primary
    traceback_tail = raised.value.__traceback__
    while traceback_tail.tb_next is not None:
        traceback_tail = traceback_tail.tb_next
    assert traceback_tail.tb_frame.f_code.co_name == "fail_version"
    assert calls == ["remove", "close requests"]
    assert tuple(error for _label, error in raised.value.__cause__.errors) == (
        removal_error,
        request_error,
    )
    assert "*" in vehicle.listeners
    collector._observe(vehicle, "TEST", message(0))
    assert list(collector._received[0]) == []

    collector.close()
    assert calls == ["remove", "close requests"]


def test_successful_prepare_cleanup_failure_closes_listener_without_retry():
    simulation_clock = Clock()
    vehicle = Vehicle()
    controller = Controller(vehicle)
    cleanup_error = UnprintableCleanupFailure()
    calls = []

    def request_version(**limits):
        controller.calls.append(("version", limits))
        vehicle.emit(version_message())

    def remove(name, callback):
        calls.append("remove")
        Vehicle.remove_message_listener(vehicle, name, callback)

    def fail_close_requests():
        calls.append("close requests")
        raise cleanup_error

    controller.request_autopilot_version = request_version
    vehicle.remove_message_listener = remove
    controller.close_startup_telemetry = fail_close_requests
    collector = TelemetryStartupCollector(
        controller=controller,
        flight_profile=profile(),
        policy=policy(),
        autopilot_version=version_contract(),
        clock=timebase.monotonic,
    )

    with timebase.configured(simulation_clock), pytest.raises(RuntimeError) as raised:
        collector.prepare()

    assert calls == ["close requests", "remove"]
    assert "UnprintableCleanupFailure" in str(raised.value)
    assert "*" not in vehicle.listeners
    collector._observe(vehicle, "TEST", message(0))
    assert list(collector._received[0]) == []

    collector.close()
    assert calls == ["close requests", "remove"]


def test_verification_preserves_primary_when_listener_removal_exits(monkeypatch):
    simulation_clock, _wall_clock, vehicle, controller, collector = staged_collector(
        monkeypatch
    )
    primary = MissionAbort("abort during staged verification")
    cleanup_error = UnprintableCleanupFailure()
    calls = []

    def fail_remove(_name, _callback):
        calls.append("remove")
        raise cleanup_error

    def abort_verification():
        raise primary

    controller.check_permission = abort_verification
    vehicle.remove_message_listener = fail_remove

    with timebase.configured(simulation_clock):
        collector.prepare()
        with pytest.raises(MissionAbort) as raised:
            collector.verify_after_guided()

    assert raised.value is primary
    traceback_tail = raised.value.__traceback__
    while traceback_tail.tb_next is not None:
        traceback_tail = traceback_tail.tb_next
    assert traceback_tail.tb_frame.f_code.co_name == "abort_verification"
    assert "UnprintableCleanupFailure" in str(raised.value.__cause__)
    assert calls == ["remove"]
    collector.close()
    assert calls == ["remove"]


@pytest.mark.parametrize("cleanup_error", [KeyboardInterrupt(), SystemExit()])
def test_successful_verification_listener_cleanup_failure_fails_closed_once(
    monkeypatch, cleanup_error
):
    simulation_clock, wall_clock, vehicle, _controller, collector = staged_collector(
        monkeypatch
    )
    calls = []

    def fail_remove(_name, _callback):
        calls.append("remove")
        raise cleanup_error

    vehicle.remove_message_listener = fail_remove

    with timebase.configured(simulation_clock):
        collector.prepare()

        def advance_and_emit():
            simulation_clock.value += 0.5
            vehicle.emit(message(0))

        wall_clock.on_sleep = advance_and_emit
        with pytest.raises(RuntimeError) as raised:
            collector.verify_after_guided()

    assert type(cleanup_error).__name__ in str(raised.value)
    assert calls == ["remove"]
    collector.close()
    assert calls == ["remove"]
