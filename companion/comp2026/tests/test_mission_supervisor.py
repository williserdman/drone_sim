import threading
import time
from types import SimpleNamespace

import pytest

from drone import timebase
from drone.control.drone_control import MissionAckError
from drone.control.mission_supervisor import (
    ABORT_AND_RECOVER,
    FM1,
    FM2,
    FM3,
    AuthorityLost,
    FlightOperationError,
    CommandEnvelope,
    CommandRejected,
    MissionAbort,
    MissionSupervisor,
    RecoveryPolicy,
    return_altitude_amsl,
)
from drone.common_types import GPSCoord, MissionHome
from drone.control.flight_state import Authority


def envelope(command=FM1, *, attempt_id=7, params=None):
    if params is None:
        params = (float(attempt_id), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    return CommandEnvelope(
        200,
        190,
        1,
        191,
        command,
        attempt_id,
        params,
        1.0,
    )


def supervisor(*, consume=lambda attempt_id: None, admit=lambda request: None,
               permit=lambda: None, enabled_phases=(FM1, FM2, FM3)):
    return MissionSupervisor(
        attempt_id=7,
        admission_check=admit,
        attempt_consumer=consume,
        permission_check=permit,
        enabled_phases=enabled_phases,
    )


def test_concurrent_abort_completes_after_active_output_and_denies_later_output():
    """Break caught: abort completion cannot race an unqueued active output."""
    mission = supervisor()
    entered = threading.Event()
    release = threading.Event()
    output_done = threading.Event()
    abort_done = threading.Event()

    def output():
        entered.set()
        assert release.wait(1.0)
        output_done.set()

    output_thread = threading.Thread(target=lambda: mission.output_transaction(output))
    output_thread.start()
    assert entered.wait(0.2)
    abort_thread = threading.Thread(
        target=lambda: (mission.request_abort("SIGINT"), abort_done.set())
    )
    abort_thread.start()
    assert not abort_done.wait(0.05)
    release.set()
    output_thread.join(0.5)
    abort_thread.join(0.5)

    assert output_done.is_set()
    assert abort_done.is_set()
    with pytest.raises(MissionAbort, match="SIGINT"):
        mission.output_transaction(lambda: pytest.fail("late output ran"))


def test_qgc_abort_is_pending_while_gpio_runs_then_denies_later_output():
    """QGC ACK completion follows the output gate, with one abort reservation."""
    mission = supervisor()
    assert mission.admit(envelope(FM1))
    mission.begin(FM1)
    entered = threading.Event()
    release = threading.Event()
    gpio_done = threading.Event()
    abort_done = threading.Event()
    receiver_errors = []

    def gpio_output():
        entered.set()
        assert release.wait(1.0)
        gpio_done.set()

    def receive_abort():
        try:
            assert mission.admit(envelope(ABORT_AND_RECOVER)) is False
        except BaseException as error:
            receiver_errors.append(error)
        finally:
            abort_done.set()

    output_thread = threading.Thread(
        target=lambda: mission.output_transaction(gpio_output)
    )
    output_thread.start()
    assert entered.wait(0.2)
    abort_thread = threading.Thread(target=receive_abort)
    abort_thread.start()

    deadline = time.monotonic() + 0.2
    while (
        mission.admission_result(ABORT_AND_RECOVER) != "IN_PROGRESS"
        and time.monotonic() < deadline
    ):
        time.sleep(0.001)
    try:
        assert mission.admission_result(ABORT_AND_RECOVER) == "IN_PROGRESS"
        assert not abort_done.is_set()
        assert mission.admit(envelope(ABORT_AND_RECOVER)) is False
        assert mission.admission_result(ABORT_AND_RECOVER) == "IN_PROGRESS"
    finally:
        release.set()
        output_thread.join(0.5)
        abort_thread.join(0.5)

    assert gpio_done.is_set()
    assert abort_done.is_set()
    assert receiver_errors == []
    assert mission.admission_result(ABORT_AND_RECOVER) == "ACCEPTED"
    with pytest.raises(MissionAbort, match="QGC abort command"):
        mission.output_transaction(lambda: pytest.fail("late output ran"))


def test_abort_latch_does_not_wait_for_supervisor_state_lock():
    mission = supervisor()
    held = threading.Event()
    release = threading.Event()

    def hold_state_lock():
        with mission._lock:
            held.set()
            assert release.wait(1.0)

    holder = threading.Thread(target=hold_state_lock)
    holder.start()
    assert held.wait(0.2)

    mission.request_abort("SIGINT")

    assert mission._output_abort_reason == "SIGINT"
    release.set()
    holder.join(0.5)
    assert mission.terminal_result == "ABORTED"


def test_dependencies_are_required_and_attempt_id_is_exact_integer():
    with pytest.raises(TypeError):
        MissionSupervisor(attempt_id=7)
    with pytest.raises(ValueError):
        MissionSupervisor(
            attempt_id=True,
            admission_check=lambda request: None,
            attempt_consumer=lambda attempt_id: None,
            permission_check=lambda: None,
        )


def test_duplicate_start_reserves_only_one_slot_and_consumes_first():
    events = []
    mission = supervisor(
        consume=lambda attempt_id: events.append(("consumed", attempt_id)),
        admit=lambda request: events.append(("checked", request.command)),
    )
    request = envelope()

    assert mission.admit(request) is True
    assert events == [("checked", FM1), ("consumed", 7)]
    assert mission.status(FM1) == "QUEUED"
    assert mission.reserved_envelope(FM1) is request
    assert mission.admit(request) is False
    assert mission.admission_result(FM1) == "IN_PROGRESS"

    mission.begin(FM1)
    assert mission.status(FM1) == "RUNNING"
    assert mission.admit(request) is False
    mission.finish(FM1, "SUCCEEDED")
    assert mission.status(FM1) == "TERMINAL"
    assert mission.admit(request) is False
    assert mission.admission_result(FM1) == "SUCCEEDED"
    assert events.count(("consumed", 7)) == 1


def test_concurrent_duplicate_start_has_one_winner_and_one_consumption():
    barrier = threading.Barrier(8)
    consumed = []
    mission = supervisor(consume=consumed.append)
    results = []

    def submit():
        barrier.wait()
        results.append(mission.admit(envelope()))

    threads = [threading.Thread(target=submit) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert results.count(True) == 1
    assert results.count(False) == 7
    assert consumed == [7]


def test_consumer_failure_or_non_none_result_never_reserves_fm1():
    def failed(_attempt_id):
        raise OSError("ledger unavailable")

    mission = supervisor(consume=failed)
    with pytest.raises(CommandRejected, match="ledger unavailable") as failure:
        mission.admit(envelope())
    assert failure.value.result == "DENIED"
    assert mission.status(FM1) is None

    mission = supervisor(consume=lambda _attempt_id: False)
    with pytest.raises(CommandRejected, match="must return None"):
        mission.admit(envelope())
    assert mission.status(FM1) is None


@pytest.mark.parametrize(
    "command_envelope",
    [
        envelope(attempt_id=8),
        envelope(params=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        envelope(params=(7.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        envelope(params=(7.0, float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0)),
    ],
)
def test_wrong_or_nonfinite_attempt_parameters_are_denied(command_envelope):
    with pytest.raises(CommandRejected) as failure:
        supervisor().admit(command_envelope)
    assert failure.value.result == "DENIED"


def test_external_admission_check_rejects_identity_before_consumption():
    consumed = []

    def reject(_request):
        raise AuthorityLost("wrong packet source")

    with pytest.raises(AuthorityLost, match="wrong packet source"):
        supervisor(consume=consumed.append, admit=reject).admit(envelope())
    assert consumed == []


def test_duplicate_from_wrong_identity_is_still_rejected():
    reject = False

    def check(_request):
        if reject:
            raise AuthorityLost("wrong packet source")

    mission = supervisor(admit=check)
    assert mission.admit(envelope())
    reject = True
    with pytest.raises(AuthorityLost, match="wrong packet source"):
        mission.admit(envelope())


def test_phases_require_order_and_success():
    mission = supervisor()
    with pytest.raises(CommandRejected, match="expected command 31000"):
        mission.admit(envelope(FM2))

    assert mission.admit(envelope(FM1))
    mission.begin(FM1)
    mission.finish(FM1, "SUCCEEDED")
    assert mission.admit(envelope(FM2))
    mission.begin(FM2)
    mission.finish(FM2, "SUCCEEDED")
    assert mission.admit(envelope(FM3))
    mission.begin(FM3)
    mission.finish(FM3, "FAILED")

    assert mission.admit(envelope(FM1)) is False
    with pytest.raises(CommandRejected, match="attempt is terminal"):
        mission.admit(envelope(31003))


def test_two_phase_supervisor_succeeds_at_fm2_and_denies_disabled_fm3():
    mission = supervisor(enabled_phases=(FM1, FM2))

    assert mission.admit(envelope(FM1))
    mission.begin(FM1)
    mission.finish(FM1, "SUCCEEDED")
    assert mission.terminal_result is None
    with pytest.raises(CommandRejected, match="disabled"):
        mission.admit(envelope(FM3))
    assert mission.admit(envelope(FM2))
    mission.begin(FM2)
    mission.finish(FM2, "SUCCEEDED")

    assert mission.terminal_result == "SUCCEEDED"
    with pytest.raises(CommandRejected, match="attempt is terminal"):
        mission.admit(envelope(FM3))


def test_three_phase_supervisor_still_waits_for_fm3():
    mission = supervisor(enabled_phases=(FM1, FM2, FM3))

    for command in (FM1, FM2):
        assert mission.admit(envelope(command))
        mission.begin(command)
        mission.finish(command, "SUCCEEDED")

    assert mission.terminal_result is None
    assert mission.admit(envelope(FM3))


@pytest.mark.parametrize(
    "enabled_phases",
    [(), (FM1,), (FM1, FM3), (FM2, FM1), (FM1, FM2, FM3, FM3), [FM1, FM2]],
)
def test_supervisor_rejects_non_prefix_or_mutable_phase_sequences(enabled_phases):
    with pytest.raises((TypeError, ValueError), match="enabled phases"):
        supervisor(enabled_phases=enabled_phases)


def test_coordinate_mutations_are_rejected_after_fm1_admission():
    mission = supervisor()
    assert mission.admit(envelope(31003))
    mission.begin(31003)
    mission.finish(31003, "SUCCEEDED")
    assert mission.admit(envelope(FM1))

    with pytest.raises(CommandRejected, match="active attempt"):
        mission.admit(envelope(31012))


def test_retired_command_is_unsupported():
    mission = supervisor()
    with pytest.raises(CommandRejected) as failure:
        mission.admit(envelope(31014))
    assert failure.value.result == "UNSUPPORTED"


def test_abort_latches_immediately_without_queue_or_second_consumption():
    consumed = []
    mission = supervisor(consume=consumed.append)
    assert mission.admit(envelope(FM1))
    mission.begin(FM1)

    assert mission.admit(envelope(ABORT_AND_RECOVER)) is False
    assert mission.abort_reason == "QGC abort command"
    assert mission.status(ABORT_AND_RECOVER) is None
    assert mission.admission_result(ABORT_AND_RECOVER) == "ACCEPTED"
    with pytest.raises(MissionAbort, match="QGC abort command"):
        mission.check_permission()
    assert consumed == [7]
    assert mission.admit(envelope(ABORT_AND_RECOVER)) is False
    assert mission.abort_reason == "QGC abort command"


def test_abort_prevents_a_queued_phase_from_starting():
    mission = supervisor()
    assert mission.admit(envelope(FM1))
    assert mission.admit(envelope(ABORT_AND_RECOVER)) is False
    assert mission.status(FM1) == "TERMINAL"
    assert mission.admission_result(FM1) == "ABORTED"
    with pytest.raises(CommandRejected):
        mission.begin(FM1)


@pytest.mark.parametrize("late_result", ["SUCCEEDED", "FAILED", "ABORTED"])
def test_abort_during_running_phase_forces_terminal_abort(late_result):
    mission = supervisor()
    assert mission.admit(envelope(FM1))
    mission.begin(FM1)
    mission.request_abort("operator abort")

    mission.finish(FM1, late_result)

    assert mission.terminal_result == "ABORTED"
    assert mission.status(FM1) == "TERMINAL"
    assert mission.admission_result(FM1) == "ABORTED"


@pytest.mark.parametrize("terminal_result", ["SUCCEEDED", "FAILED"])
def test_abort_after_terminal_phase_cannot_replace_outcome(terminal_result):
    mission = supervisor()
    assert mission.admit(envelope(FM1))
    mission.begin(FM1)
    if terminal_result == "SUCCEEDED":
        mission.finish(FM1, "SUCCEEDED")
        assert mission.admit(envelope(FM2))
        mission.begin(FM2)
        mission.finish(FM2, "SUCCEEDED")
        assert mission.admit(envelope(FM3))
        mission.begin(FM3)
        mission.finish(FM3, "SUCCEEDED")
    else:
        mission.finish(FM1, "FAILED")

    mission.request_abort("late abort")

    assert mission.terminal_result == terminal_result
    assert mission.abort_reason is None


def test_authority_has_precedence_and_is_separate_for_future_recovery():
    authority_lost = False

    def permit():
        if authority_lost:
            raise AuthorityLost("pilot takeover")

    mission = supervisor(permit=permit)
    mission.request_abort("operator abort")
    mission.check_authority()
    with pytest.raises(MissionAbort, match="operator abort"):
        mission.check_permission()
    authority_lost = True
    with pytest.raises(AuthorityLost, match="pilot takeover"):
        mission.check_permission()


def test_non_none_permission_result_fails_closed():
    mission = supervisor(permit=lambda: False)
    with pytest.raises(AuthorityLost, match="must return None"):
        mission.check_permission()


def test_mission_result_and_recovery_outcome_remain_separate():
    mission = supervisor()
    assert mission.admit(envelope(FM1))
    mission.begin(FM1)
    mission.finish(FM1, "FAILED")
    mission.record_recovery_outcome("LOCAL_LANDED")

    assert mission.terminal_result == "FAILED"
    assert mission.recovery_outcome == "LOCAL_LANDED"


def test_illegal_transitions_and_result_values_fail_closed():
    mission = supervisor()
    with pytest.raises(CommandRejected):
        mission.begin(FM1)
    assert mission.admit(envelope(FM1))
    with pytest.raises(ValueError):
        mission.finish(FM1, "SUCCESS")
    mission.begin(FM1)
    with pytest.raises(ValueError):
        mission.finish(FM1, "SUCCESS")
    with pytest.raises(ValueError):
        mission.record_recovery_outcome("LANDED")


def test_recovery_keeps_the_higher_transit_altitude_and_rejects_invalid_values():
    home = MissionHome(41.0, -81.0, 100.0)

    assert return_altitude_amsl(home, 10.0, 103.0) == 110.0
    assert return_altitude_amsl(home, 10.0, 125.0) == 125.0
    with pytest.raises(ValueError, match="finite"):
        return_altitude_amsl(home, float("nan"), 103.0)


class RecoveryController:
    def __init__(self):
        self.events = []
        self.authority = Authority.COMPANION
        self.armed = True
        self.landed_state = 2
        self.mode = "GUIDED"
        self.amsl_m = 103.0
        self.mission_home = MissionHome(41.0, -81.0, 100.0)
        self.permission_guard = lambda: None
        self.hooks = {}
        self.failures = {}
        self.results = {}

    @staticmethod
    def _field(value):
        return SimpleNamespace(
            fresh=True,
            observation=SimpleNamespace(value=value),
        )

    def flight_snapshot(self):
        return SimpleNamespace(
            authority=self.authority,
            commands_suspended=self.authority is not Authority.COMPANION,
            armed=self._field(self.armed),
            landed_state=self._field(self.landed_state),
            mode=self._field(self.mode),
            location=self._field((410000000, -810000000, self.amsl_m * 1000.0, 0.0)),
        )

    def _permit(self):
        self.permission_guard()

    def _after(self, operation):
        hook = self.hooks.get(operation)
        if hook is not None:
            hook()
        failure = self.failures.get(operation)
        if failure is not None:
            raise failure

    def climb(self, target_alt, *, timeout):
        self._permit()
        self.events.append(("climb", target_alt, timeout))
        self.amsl_m = 100.0 + target_alt
        self._after("climb")
        return self.results.get("climb")

    def goto_waypoint(self, coord, *, timeout):
        self._permit()
        self.events.append(("return", coord, timeout))
        self._after("return")
        return self.results.get("return", 0)

    def goto_recovery_waypoint(self, coord, *, approve_target_amsl, timeout):
        self._permit()
        self.events.append(("return", coord, timeout))
        target_amsl = self.mission_home.amsl_m + coord.alt
        if self.amsl_m > target_amsl:
            if approve_target_amsl(self.amsl_m) is not None:
                raise FlightOperationError("return approval must return None")
        self._after("return")
        return self.results.get("return", 0)

    def simple_land(self, *, timeout, mode_timeout):
        self._permit()
        self.events.append(("land", timeout, mode_timeout))
        self.mode = "LAND"
        self.landed_state = 1
        self._after("land")
        return self.results.get("land", 0)

    def confirm_landing(self, *, timeout):
        self._permit()
        self.events.append(("confirm_land", timeout))
        self.landed_state = 1
        self._after("confirm_land")
        return self.results.get("confirm_land", 0)

    def disarm(self, *, timeout):
        self._permit()
        self.events.append(("disarm", timeout))
        self.armed = False
        self._after("disarm")
        return self.results.get("disarm", 0)


def recovery_supervisor(controller, *, check=lambda operation, home, altitude: None):
    mission = MissionSupervisor(
        attempt_id=7,
        admission_check=lambda request: None,
        attempt_consumer=lambda attempt_id: None,
        permission_check=lambda: None,
        recovery_policy=RecoveryPolicy(
            check=check,
            timeout_s=60.0,
            local_land_reserve_s=20.0,
        ),
    )
    controller.permission_guard = mission.check_permission
    mission.request_abort("operator abort")
    return mission


def test_recovery_uses_abort_local_scope_and_orders_climb_return_land_confirm():
    controller = RecoveryController()
    mission = recovery_supervisor(controller)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "HOME_LANDED"
    assert [event[0] for event in controller.events] == [
        "climb",
        "return",
        "land",
        "disarm",
    ]
    assert controller.events[0][1] == 10.0
    assert controller.events[1][1] == GPSCoord(41.0, -81.0, 10.0)
    assert mission.terminal_result == "ABORTED"
    assert mission.recovery_outcome == "HOME_LANDED"
    with pytest.raises(MissionAbort):
        mission.check_permission()


def test_recovery_keeps_higher_altitude_and_is_idempotent():
    controller = RecoveryController()
    controller.amsl_m = 125.0
    mission = recovery_supervisor(controller)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "HOME_LANDED"
    assert [event[0] for event in controller.events] == ["return", "land", "disarm"]
    assert controller.events[0][1].alt == 25.0

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "HOME_LANDED"
    assert [event[0] for event in controller.events] == ["return", "land", "disarm"]


def test_missing_return_approval_falls_back_to_one_local_land():
    controller = RecoveryController()

    def check(operation, _home, _altitude):
        if operation == "RETURN":
            raise FlightOperationError("corridor unavailable")

    mission = recovery_supervisor(controller, check=check)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == ["land", "disarm"]


@pytest.mark.parametrize(
    ("authority", "outcome"),
    [
        (Authority.PILOT, "PILOT"),
        (Authority.FC_FAILSAFE, "FC_FAILSAFE"),
        (Authority.UNKNOWN, "UNCONFIRMED"),
    ],
)
def test_recovery_authority_loss_at_entry_sends_nothing(authority, outcome):
    controller = RecoveryController()
    controller.authority = authority
    mission = recovery_supervisor(controller)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == outcome
    assert controller.events == []


def test_recovery_without_explicit_policy_sends_nothing():
    controller = RecoveryController()
    mission = supervisor()

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "UNCONFIRMED"
    assert controller.events == []


@pytest.mark.parametrize(
    "failure",
    [
        FlightOperationError("no progress"),
        RuntimeError("controller transport failure"),
    ],
)
def test_return_operation_failure_attempts_local_land_once(failure):
    controller = RecoveryController()
    controller.failures["return"] = failure
    mission = recovery_supervisor(controller)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == [
        "climb",
        "return",
        "land",
        "disarm",
    ]


def test_ambiguous_guided_mission_failure_switches_to_local_land_without_return():
    controller = RecoveryController()
    controller.failures["climb"] = MissionAckError("prior MISSION_ACK timed out")
    mission = recovery_supervisor(controller)

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == ["climb", "land", "disarm"]


def test_nonzero_return_result_cannot_report_home_landing():
    controller = RecoveryController()
    controller.results["return"] = -1
    mission = recovery_supervisor(controller)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == [
        "climb",
        "return",
        "land",
        "disarm",
    ]


def test_failed_land_is_unconfirmed_and_never_disarms():
    controller = RecoveryController()
    controller.failures["land"] = FlightOperationError("LAND rejected")
    mission = recovery_supervisor(controller)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "UNCONFIRMED"
    assert [event[0] for event in controller.events] == ["climb", "return", "land"]


@pytest.mark.parametrize("operation", ["land", "confirm_land", "disarm"])
def test_nonzero_landing_or_confirmation_result_remains_unconfirmed(operation):
    controller = RecoveryController()
    controller.results[operation] = False
    if operation == "confirm_land":
        controller.mode = "LAND"
        controller.landed_state = 4
    mission = recovery_supervisor(controller)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "UNCONFIRMED"


def test_recovery_preserves_landing_already_underway():
    controller = RecoveryController()
    controller.mode = "LAND"
    controller.landed_state = 4
    mission = recovery_supervisor(controller)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == ["confirm_land", "disarm"]


def test_recovery_on_confirmed_ground_never_rearms_or_selects_land():
    controller = RecoveryController()
    controller.landed_state = 1
    mission = recovery_supervisor(controller)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == ["disarm"]


def test_takeoff_state_aborts_directly_to_one_local_land():
    controller = RecoveryController()
    controller.landed_state = 3
    mission = recovery_supervisor(controller)

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == ["land", "disarm"]


def test_takeoff_state_rejected_land_is_unconfirmed_without_disarm():
    controller = RecoveryController()
    controller.landed_state = 3
    controller.failures["land"] = FlightOperationError("LAND rejected")
    mission = recovery_supervisor(controller)

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "UNCONFIRMED"
    assert [event[0] for event in controller.events] == ["land"]


def test_transition_to_takeoff_at_local_land_boundary_still_sends_one_land():
    controller = RecoveryController()

    def check(operation, _home, _altitude):
        if operation == "RETURN":
            raise FlightOperationError("return unavailable")
        if operation == "LOCAL_LAND":
            controller.landed_state = 3

    mission = recovery_supervisor(controller, check=check)

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == ["land", "disarm"]


def test_raised_recovery_altitude_rejection_falls_back_to_one_local_land():
    controller = RecoveryController()
    controller.hooks["climb"] = lambda: setattr(controller, "amsl_m", 135.0)
    approvals = []

    def check(operation, _home, altitude):
        approvals.append((operation, altitude))
        if operation == "RETURN" and altitude == 135.0:
            raise FlightOperationError("raised altitude outside envelope")

    mission = recovery_supervisor(controller, check=check)

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "LOCAL_LANDED"
    assert approvals == [
        ("RETURN", 110.0),
        ("RETURN", 135.0),
        ("LOCAL_LAND", None),
    ]
    assert [event[0] for event in controller.events] == [
        "climb",
        "return",
        "land",
        "disarm",
    ]


def test_writer_boundary_altitude_race_falls_back_to_exactly_one_local_land():
    controller = RecoveryController()
    controller.failures["return"] = FlightOperationError(
        "recovery altitude changed after packet preparation"
    )
    mission = recovery_supervisor(controller)

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == [
        "climb",
        "return",
        "land",
        "disarm",
    ]
    assert len([event for event in controller.events if event[0] == "land"]) == 1


def test_reentrant_abort_during_recovery_output_latches_without_interrupting_recovery():
    controller = RecoveryController()
    mission = recovery_supervisor(controller)

    def abort_during_return() -> None:
        def interrupt_twice() -> None:
            mission.request_abort("signal during recovery")
            mission.request_abort("repeated signal during recovery")

        mission.output_transaction(interrupt_twice)

    controller.hooks["return"] = abort_during_return

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "HOME_LANDED"
    assert mission.terminal_result == "ABORTED"
    assert [event[0] for event in controller.events] == [
        "climb",
        "return",
        "land",
        "disarm",
    ]


@pytest.mark.parametrize("takeover_after", ["climb", "return", "land", "disarm"])
@pytest.mark.parametrize(
    ("authority", "outcome"),
    [
        (Authority.PILOT, "PILOT"),
        (Authority.FC_FAILSAFE, "FC_FAILSAFE"),
        (Authority.UNKNOWN, "UNCONFIRMED"),
    ],
)
def test_authority_change_during_each_recovery_output_stops_following_commands(
    takeover_after, authority, outcome
):
    controller = RecoveryController()
    controller.hooks[takeover_after] = lambda: setattr(
        controller, "authority", authority
    )
    mission = recovery_supervisor(controller)

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == outcome
    names = [event[0] for event in controller.events]
    assert names[-1] == takeover_after


class RecoveryClock:
    def __init__(self, now=600.0):
        self.now_value = now

    def __call__(self):
        return self.now_value

    def now(self):
        return self.now_value

    def sleep(self, seconds):
        self.now_value += seconds


def test_recovery_after_mission_timeout_has_its_own_bound_and_land_reserve():
    controller = RecoveryController()
    clock = RecoveryClock()
    mission = MissionSupervisor(
        attempt_id=7,
        admission_check=lambda request: None,
        attempt_consumer=lambda attempt_id: None,
        permission_check=lambda: None,
        recovery_policy=RecoveryPolicy(
            check=lambda operation, home, altitude: None,
            timeout_s=60.0,
            local_land_reserve_s=20.0,
            clock=clock,
        ),
    )
    controller.permission_guard = mission.check_permission
    mission.request_abort("600-second mission timeout")

    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "HOME_LANDED"
    assert controller.events[0][2] == 40.0
    assert controller.events[1][2] == 40.0
    assert controller.events[2][1] == 60.0
    assert controller.events[2][2] == 20.0


def test_recovery_policy_rejects_open_bounds_and_non_none_approval():
    with pytest.raises(ValueError, match="reserve"):
        RecoveryPolicy(
            check=lambda operation, home, altitude: None,
            timeout_s=10.0,
            local_land_reserve_s=10.0,
        )

    controller = RecoveryController()
    mission = recovery_supervisor(
        controller,
        check=lambda operation, home, altitude: False,
    )
    assert mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0) == "UNCONFIRMED"
    assert controller.events == []


def test_recovery_rejects_mismatched_controller_datum_before_return_conversion():
    controller = RecoveryController()
    controller.mission_home = MissionHome(41.0, -81.0, 250.0)
    controller.amsl_m = 325.0
    checks = []
    mission = recovery_supervisor(
        controller,
        check=lambda operation, home, altitude: checks.append(
            (operation, home, altitude)
        ),
    )

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 300.0), 10.0
    ) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == ["land", "disarm"]
    assert [check[0] for check in checks] == ["LOCAL_LAND"]


@pytest.mark.parametrize(
    ("landed_state", "mode", "expected_events"),
    [
        (1, "GUIDED", ["disarm"]),
        (4, "LAND", ["confirm_land", "disarm"]),
    ],
)
def test_return_policy_state_change_is_reassessed_before_navigation(
    landed_state, mode, expected_events
):
    controller = RecoveryController()

    def change_state(operation, _home, _altitude):
        if operation == "RETURN":
            controller.landed_state = landed_state
            controller.mode = mode

    mission = recovery_supervisor(controller, check=change_state)

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == expected_events


def test_ground_after_climb_stops_return_and_land_mode_output():
    controller = RecoveryController()

    def touch_down():
        controller.landed_state = 1

    controller.hooks["climb"] = touch_down
    mission = recovery_supervisor(controller)

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "LOCAL_LANDED"
    assert [event[0] for event in controller.events] == ["climb", "disarm"]


def test_ground_during_land_policy_check_prevents_land_mode_reassertion():
    controller = RecoveryController()

    def change_state(operation, _home, _altitude):
        if operation == "LOCAL_LAND":
            controller.landed_state = 1

    mission = recovery_supervisor(controller, check=change_state)

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "HOME_LANDED"
    assert [event[0] for event in controller.events] == [
        "climb",
        "return",
        "disarm",
    ]


@pytest.mark.parametrize("malformed_ground", [True, 1.0])
def test_malformed_ground_enum_never_confirms_landing(malformed_ground):
    controller = RecoveryController()
    controller.armed = False
    controller.landed_state = malformed_ground
    mission = recovery_supervisor(controller)

    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "UNCONFIRMED"
    assert controller.events == []


def test_outer_deadline_propagates_and_latches_recovery_against_restart():
    controller = RecoveryController()
    controller.failures["return"] = TimeoutError("host deadline")
    mission = recovery_supervisor(controller)

    with pytest.raises(TimeoutError, match="host deadline"):
        mission.recover(controller, MissionHome(41.0, -81.0, 100.0), 10.0)
    first_events = list(controller.events)

    assert mission.recovery_outcome == "UNCONFIRMED"
    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "UNCONFIRMED"
    assert controller.events == first_events


def test_timebase_outer_deadline_is_not_converted_to_recovery_fallback():
    controller = RecoveryController()
    clock = RecoveryClock(now=0.0)
    controller.hooks["return"] = lambda: setattr(clock, "now_value", 6.0)
    mission = MissionSupervisor(
        attempt_id=7,
        admission_check=lambda request: None,
        attempt_consumer=lambda attempt_id: None,
        permission_check=lambda: None,
        recovery_policy=RecoveryPolicy(
            check=lambda operation, home, altitude: None,
            timeout_s=60.0,
            local_land_reserve_s=20.0,
            clock=timebase.monotonic,
        ),
    )
    controller.permission_guard = mission.check_permission
    mission.request_abort("mission failure")

    with timebase.configured(clock), timebase._deadline(0.0, 5.0):
        with pytest.raises(TimeoutError, match="mission exceeded"):
            mission.recover(
                controller, MissionHome(41.0, -81.0, 100.0), 10.0
            )

    assert mission.recovery_outcome == "UNCONFIRMED"
    assert [event[0] for event in controller.events] == [
        "climb",
        "return",
    ]


def test_stopped_parent_simulation_clock_propagates_and_latches_recovery():
    from drone_sim_companion.comp2026_host import SimulationClock

    controller = RecoveryController()
    clock = SimulationClock()
    clock.accept(0)

    def stop_host_clock():
        clock.stop("host cancellation")
        timebase.sleep(0.1)

    controller.hooks["return"] = stop_host_clock
    mission = MissionSupervisor(
        attempt_id=7,
        admission_check=lambda request: None,
        attempt_consumer=lambda attempt_id: None,
        permission_check=lambda: None,
        recovery_policy=RecoveryPolicy(
            check=lambda operation, home, altitude: None,
            timeout_s=60.0,
            local_land_reserve_s=20.0,
            clock=timebase.monotonic,
        ),
    )
    controller.permission_guard = mission.check_permission
    mission.request_abort("mission failure")

    with timebase.configured(clock):
        with pytest.raises(timebase.ClockError) as failure:
            mission.recover(
                controller, MissionHome(41.0, -81.0, 100.0), 10.0
            )

    first_events = list(controller.events)
    assert isinstance(failure.value.__cause__, RuntimeError)
    assert mission.recovery_outcome == "UNCONFIRMED"
    assert [event[0] for event in first_events] == ["climb", "return"]
    assert mission.recover(
        controller, MissionHome(41.0, -81.0, 100.0), 10.0
    ) == "UNCONFIRMED"
    assert controller.events == first_events
