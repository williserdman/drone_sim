from __future__ import annotations

from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from drone_sim_companion.comp2026_host import (
    PayloadDropper,
    PayloadResponse,
    SimulationClock,
)
from drone_sim_companion.runtime_node import _RosPayloadClient


NESTED_SOURCE = Path(__file__).parents[1] / "comp2026/src"
sys.path.insert(0, str(NESTED_SOURCE))
from drone.control.mission_supervisor import MissionSupervisor  # noqa: E402
from drone.control.drone_control import DroneControl  # noqa: E402
from drone.control.flight_state import (  # noqa: E402
    FailsafeEvidence,
    FlightState,
    RCModeBand,
)
from drone.control.mission_supervisor import (  # noqa: E402
    CommandEnvelope,
    FM2,
)


RUN_ID = "00000000-0000-4000-8000-000000000001"


class InertPayloadServiceType:
    class Request:
        ATTACH = 1
        RELEASE = 2


class InertFuture:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._callbacks = []
        self._response: object | None = None
        self._done = False
        self.cancelled = False

    def add_done_callback(self, callback) -> None:
        with self._condition:
            if self._done:
                invoke_now = True
            else:
                self._callbacks.append(callback)
                invoke_now = False
        if invoke_now:
            callback(self)

    def complete(self, response: object) -> None:
        with self._condition:
            self._response = response
            self._done = True
            callbacks = tuple(self._callbacks)
            self._callbacks.clear()
        for callback in callbacks:
            callback(self)

    def done(self) -> bool:
        with self._condition:
            return self._done

    def result(self) -> object:
        return self._response

    def cancel(self) -> None:
        self.cancelled = True


class InertPayloadService:
    def __init__(self) -> None:
        self.requests: list[object] = []
        self.futures: list[InertFuture] = []
        self._condition = threading.Condition()

    def service_is_ready(self) -> bool:
        return True

    def call_async(self, request: object) -> InertFuture:
        future = InertFuture()
        with self._condition:
            self.requests.append(request)
            self.futures.append(future)
            self._condition.notify_all()
        return future

    def wait_for_request(self, count: int = 1) -> None:
        with self._condition:
            assert self._condition.wait_for(
                lambda: len(self.requests) >= count, timeout=1.0
            )


def test_inert_future_notifies_callback_added_after_none_completion() -> None:
    future = InertFuture()
    callbacks: list[InertFuture] = []

    future.complete(None)
    future.add_done_callback(callbacks.append)

    assert future.done() is True
    assert future.result() is None
    assert callbacks == [future]


def make_supervisor() -> MissionSupervisor:
    return MissionSupervisor(
        1,
        admission_check=lambda _envelope: None,
        attempt_consumer=lambda _attempt_id: None,
        permission_check=lambda: None,
    )


def make_permission(supervisor: MissionSupervisor):
    def permission() -> bool:
        supervisor.check_permission()
        return True

    permission.actuate = supervisor.output_transaction
    return permission


def make_authorized_flight_state() -> FlightState:
    state = FlightState(
        source_system=1,
        source_component=1,
        freshness_bounds={
            name: 1.0
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
        },
        rc_channel=7,
        rc_mode_mapping=(
            RCModeBand("manual", 900, 1200, "STABILIZE"),
            RCModeBand("companion", 1400, 1600, "GUIDED"),
            RCModeBand("pilot", 1800, 2100, "LOITER"),
        ),
        clock=lambda: 10.0,
    )
    assert state.update_many(
        {
            "heartbeat": "alive",
            "mode": "GUIDED",
            "landed_state": 1,
            "armed": False,
            "failsafe": FailsafeEvidence(active=False, reason="test clear"),
        },
        received_at=10.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )
    assert state.observe_rc_input(
        channel=7,
        pwm=1500,
        signal_healthy=True,
        received_at=10.0,
        sequence=2,
        source_system=1,
        source_component=1,
    )
    assert state.acquire_initial_companion_authority(now=10.0)
    return state


def make_dropper(
    service: InertPayloadService,
    *,
    marker: int = 3,
    permission=None,
    response_timeout_seconds: float = 0.25,
) -> PayloadDropper:
    supervisor = make_supervisor()
    selected_permission = permission or make_permission(supervisor)
    client = _RosPayloadClient(
        service,
        InertPayloadServiceType,
        response_timeout_seconds=response_timeout_seconds,
    )
    return PayloadDropper(
        RUN_ID,
        marker,
        client,
        SimulationClock(),
        permission=selected_permission,
        delay_wall_timeout_seconds=0.25,
    )


def test_final_boundary_revocation_emits_no_ros_request() -> None:
    service = InertPayloadService()
    supervisor = make_supervisor()

    def permission() -> bool:
        return True

    def revoke_at_boundary(operation):
        supervisor.request_abort("test revocation")
        return supervisor.output_transaction(operation)

    permission.actuate = revoke_at_boundary
    dropper = make_dropper(service, permission=permission)

    with pytest.raises(Exception, match="test revocation"):
        dropper.drop()

    assert service.requests == []


def test_confirmation_wait_does_not_hold_output_transaction() -> None:
    service = InertPayloadService()
    supervisor = make_supervisor()
    dropper = make_dropper(service, permission=make_permission(supervisor))
    returned: list[object] = []
    worker = threading.Thread(target=lambda: returned.append(dropper.drop()))
    worker.start()
    service.wait_for_request()

    probe_returned = threading.Event()
    probe = threading.Thread(
        target=lambda: (supervisor.output_transaction(lambda: None), probe_returned.set())
    )
    probe.start()
    assert probe_returned.wait(0.2)

    service.futures[0].complete(
        PayloadResponse(True, "OK", "detached", "run:3:release:1", 1)
    )
    worker.join(timeout=1.0)
    probe.join(timeout=1.0)
    assert returned == [None]


def test_real_controller_payload_boundary_does_not_deadlock_with_admit_and_abort() -> None:
    service = InertPayloadService()
    state = make_authorized_flight_state()
    admission_entered = threading.Event()

    def admission_check(_envelope: CommandEnvelope) -> None:
        admission_entered.set()
        state.snapshot()

    supervisor = MissionSupervisor(
        1,
        admission_check=admission_check,
        attempt_consumer=lambda _attempt_id: None,
        permission_check=lambda: None,
    )
    controller = object.__new__(DroneControl)
    controller.flight_state = state
    controller.permission_guard = supervisor.check_permission
    controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=supervisor.output_transaction,
        transport_transaction=lambda operation, enqueue_check: (
            enqueue_check(),
            operation(),
        )[1],
    )
    permission = make_permission(supervisor)
    permission.actuate = controller._actuate_guarded
    dropper = make_dropper(service, permission=permission)
    payload_at_boundary = threading.Event()
    payload_errors: list[BaseException] = []
    admission_errors: list[BaseException] = []
    abort_errors: list[BaseException] = []

    def release_actuate(output) -> None:
        def ordered_output() -> None:
            payload_at_boundary.set()
            assert admission_entered.wait(0.5)
            output()

        controller._actuate_guarded(ordered_output)

    payload_thread = threading.Thread(
        target=lambda: _capture_error(
            payload_errors,
            lambda: dropper.drop_with_guard(
                release_actuate, controller._actuate_guarded
            ),
        ),
        daemon=True,
    )
    payload_thread.start()
    assert payload_at_boundary.wait(0.2)

    admission = CommandEnvelope(
        200,
        190,
        1,
        191,
        FM2,
        1,
        (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        10.0,
    )
    admission_thread = threading.Thread(
        target=lambda: _capture_error(
            admission_errors, lambda: supervisor.admit(admission)
        ),
        daemon=True,
    )
    admission_thread.start()
    assert admission_entered.wait(0.2)
    abort_thread = threading.Thread(
        target=lambda: _capture_error(
            abort_errors, lambda: supervisor.request_abort("concurrent abort")
        ),
        daemon=True,
    )
    abort_thread.start()

    service.wait_for_request()
    service.futures[0].complete(
        PayloadResponse(True, "OK", "detached", "run:3:release:1", 1)
    )
    for thread in (payload_thread, admission_thread, abort_thread):
        thread.join(timeout=0.5)

    assert not payload_thread.is_alive()
    assert not admission_thread.is_alive()
    assert not abort_thread.is_alive()
    assert payload_errors == []
    assert abort_errors == []
    assert len(admission_errors) == 1
    assert len(service.requests) == 1


@pytest.mark.parametrize("permission_result", [None, False])
def test_non_literal_permission_denies_output(permission_result: object) -> None:
    service = InertPayloadService()

    def permission():
        return permission_result

    permission.actuate = lambda operation: operation()
    dropper = make_dropper(service, permission=permission)

    with pytest.raises(PermissionError, match="not current"):
        dropper.drop()
    assert service.requests == []


def test_raising_permission_denies_output() -> None:
    service = InertPayloadService()

    def permission() -> bool:
        raise RuntimeError("authority source failed")

    permission.actuate = lambda operation: operation()
    dropper = make_dropper(service, permission=permission)

    with pytest.raises(PermissionError, match="check failed"):
        dropper.drop()
    assert service.requests == []


def test_permission_without_atomic_actuation_hook_is_rejected() -> None:
    with pytest.raises(ValueError, match="atomic actuation"):
        PayloadDropper(
            RUN_ID,
            3,
            _RosPayloadClient(
                InertPayloadService(),
                InertPayloadServiceType,
                response_timeout_seconds=0.25,
            ),
            SimulationClock(),
            permission=lambda: True,
            delay_wall_timeout_seconds=0.25,
        )


def test_correlated_attach_returns_literal_true() -> None:
    service = InertPayloadService()
    dropper = make_dropper(service)
    returned: list[object] = []
    worker = threading.Thread(target=lambda: returned.append(dropper.attach(3)))
    worker.start()
    service.wait_for_request()
    service.futures[0].complete(
        PayloadResponse(True, "OK", "attached", "run:3:attach:1", 7)
    )
    worker.join(timeout=1.0)

    request = service.requests[0]
    assert returned == [True]
    assert dropper.supports_attachment is True
    assert (request.run_id, request.aruco_id, request.action, request.command_id) == (
        RUN_ID,
        3,
        request.ATTACH,
        "run:3:attach:1",
    )


@pytest.mark.parametrize(
    ("response", "message"),
    [
        (None, "no confirmation"),
        (PayloadResponse(True, "OK", "", "wrong", 1), "command_id"),
        (PayloadResponse(True, "OK", "", "run:3:release:1", 0), "sequence"),
        (PayloadResponse(False, "DENIED", "unsafe", "run:3:release:1", 2), "DENIED"),
    ],
)
def test_missing_rejected_or_mismatched_confirmation_fails(
    response: object, message: str
) -> None:
    service = InertPayloadService()
    dropper = make_dropper(service)
    errors: list[BaseException] = []
    worker = threading.Thread(
        target=lambda: _capture_error(errors, dropper.drop), daemon=True
    )
    worker.start()
    service.wait_for_request()
    service.futures[0].complete(response)
    worker.join(timeout=1.0)

    assert len(errors) == 1
    assert message in str(errors[0])
    assert len(service.requests) == 1


def _capture_error(errors: list[BaseException], operation) -> None:
    try:
        operation()
    except BaseException as error:
        errors.append(error)


def test_release_guard_is_used_once_and_stale_rejection_is_not_retried() -> None:
    service = InertPayloadService()
    dropper = make_dropper(service)
    release_calls = 0
    continuation_calls = 0
    errors: list[BaseException] = []

    def release_actuate(operation):
        nonlocal release_calls
        release_calls += 1
        return operation()

    def continuation_actuate(operation):
        nonlocal continuation_calls
        continuation_calls += 1
        return operation()

    worker = threading.Thread(
        target=lambda: _capture_error(
            errors,
            lambda: dropper.drop_with_guard(release_actuate, continuation_actuate),
        ),
        daemon=True,
    )
    worker.start()
    service.wait_for_request()
    service.futures[0].complete(
        PayloadResponse(
            False,
            "STALE_PHYSICAL_STATE",
            "truth changed",
            "run:3:release:1",
            3,
        )
    )
    worker.join(timeout=1.0)
    time.sleep(0.05)

    assert release_calls == 1
    assert continuation_calls == 0
    assert len(service.requests) == 1
    assert "STALE_PHYSICAL_STATE" in str(errors[0])


def test_unknown_completion_times_out_without_a_second_request() -> None:
    service = InertPayloadService()
    dropper = make_dropper(service, response_timeout_seconds=0.05)

    with pytest.raises(TimeoutError, match="confirmation timed out"):
        dropper.drop()

    assert len(service.requests) == 1
    assert service.futures[0].cancelled is True


def test_revocation_after_dispatch_keeps_correlated_outcome_without_more_output() -> None:
    service = InertPayloadService()
    supervisor = make_supervisor()
    dropper = make_dropper(service, permission=make_permission(supervisor))
    returned: list[object] = []
    worker = threading.Thread(target=lambda: returned.append(dropper.drop()))
    worker.start()
    service.wait_for_request()

    supervisor.request_abort("late revocation")
    service.futures[0].complete(
        PayloadResponse(True, "OK", "detached", "run:3:release:1", 4)
    )
    worker.join(timeout=1.0)

    assert returned == [None]
    assert len(service.requests) == 1


def test_passive_cleanup_stops_local_wait_without_sending_output() -> None:
    service = InertPayloadService()
    dropper = make_dropper(service)
    errors: list[BaseException] = []
    worker = threading.Thread(
        target=lambda: _capture_error(errors, dropper.drop), daemon=True
    )
    worker.start()
    service.wait_for_request()

    assert dropper.cleanup_passive() is True
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert len(service.requests) == 1
    assert "closed during confirmation" in str(errors[0])


def test_passive_cleanup_before_use_sends_no_output() -> None:
    service = InertPayloadService()
    dropper = make_dropper(service)

    assert dropper.cleanup_passive() is True
    with pytest.raises(RuntimeError, match="closed"):
        dropper.attach(3)
    assert service.requests == []


def test_passive_cleanup_cancels_a_delayed_release_without_output() -> None:
    service = InertPayloadService()
    dropper = make_dropper(service)
    errors: list[BaseException] = []
    worker = threading.Thread(
        target=lambda: _capture_error(errors, lambda: dropper.drop(delay_hold=1.0)),
        daemon=True,
    )
    worker.start()
    time.sleep(0.05)

    assert dropper.cleanup_passive() is True
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert service.requests == []
    assert "closed" in str(errors[0])


def test_clock_stop_cancels_a_delayed_release_without_output() -> None:
    service = InertPayloadService()
    clock = SimulationClock()
    clock.accept(2_000_000_000)
    supervisor = make_supervisor()
    dropper = PayloadDropper(
        RUN_ID,
        3,
        _RosPayloadClient(
            service,
            InertPayloadServiceType,
            response_timeout_seconds=0.25,
        ),
        clock,
        permission=make_permission(supervisor),
        delay_wall_timeout_seconds=0.25,
    )
    errors: list[BaseException] = []
    worker = threading.Thread(
        target=lambda: _capture_error(errors, lambda: dropper.drop(delay_hold=1.0)),
        daemon=True,
    )
    worker.start()
    time.sleep(0.05)

    clock.stop("test stop")
    worker.join(timeout=0.2)

    assert not worker.is_alive()
    assert service.requests == []
    assert "simulation clock stopped" in str(errors[0])


@pytest.mark.parametrize("initial_clock_ns", [None, 2_000_000_000])
def test_release_delay_has_a_wall_deadline_when_clock_is_missing_or_frozen(
    initial_clock_ns: int | None,
) -> None:
    service = InertPayloadService()
    clock = SimulationClock()
    if initial_clock_ns is not None:
        clock.accept(initial_clock_ns)
    supervisor = make_supervisor()
    dropper = PayloadDropper(
        RUN_ID,
        3,
        _RosPayloadClient(
            service,
            InertPayloadServiceType,
            response_timeout_seconds=0.25,
        ),
        clock,
        permission=make_permission(supervisor),
        delay_wall_timeout_seconds=0.05,
    )

    errors: list[BaseException] = []
    worker = threading.Thread(
        target=lambda: _capture_error(errors, lambda: dropper.drop(delay_hold=1.0)),
        daemon=True,
    )
    worker.start()
    worker.join(timeout=0.2)

    assert not worker.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], TimeoutError)
    assert "release delay timed out" in str(errors[0])
    assert service.requests == []


@pytest.mark.parametrize("wall_timeout", [False, 0.0, float("inf")])
def test_release_delay_wall_budget_must_be_finite_and_positive(
    wall_timeout: object,
) -> None:
    supervisor = make_supervisor()

    with pytest.raises(ValueError, match="wall timeout"):
        PayloadDropper(
            RUN_ID,
            3,
            _RosPayloadClient(
                InertPayloadService(),
                InertPayloadServiceType,
                response_timeout_seconds=0.25,
            ),
            SimulationClock(),
            permission=make_permission(supervisor),
            delay_wall_timeout_seconds=wall_timeout,
        )
