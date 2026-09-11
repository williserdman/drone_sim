import queue
import threading
import time
from dataclasses import replace
from types import SimpleNamespace
from types import MappingProxyType

import pytest

from drone.control.flight_profile import FlightProfile, TelemetryRequest
from drone.control.flight_state import RCModeBand, SourceIdentity
from drone.control.listener import (
    ACK_ACCEPTED,
    ACK_CANCELLED,
    ACK_DENIED,
    ACK_IN_PROGRESS,
    ACK_UNSUPPORTED,
    CommandAck,
    CommandExecutionOwner,
    ListenerStartupFiles,
    QGCCommandListener,
    construct_after_listener_validation,
    load_listener_artifacts,
)
from drone.control.attempt_setup import prepare_attempt
from drone.control.mission_supervisor import (
    ABORT_AND_RECOVER,
    AuthorityLost,
    CommandEnvelope,
    FM1,
    FM2,
    MissionAbort,
    MissionSupervisor,
    RETIRED_COMMAND,
    UPDATE_L,
    AttemptLedger,
    initialize_ledger,
)
from drone import timebase


class UnprintableObserverFailure(Exception):
    def __str__(self):
        raise RuntimeError("observer diagnostic formatting failed")


class EmptyDetailFailure(Exception):
    def __str__(self):
        return ""


class NonStringDetailFailure(Exception):
    def __str__(self):
        return 17


class Packet:
    def __init__(
        self,
        *,
        message_type="COMMAND_LONG",
        source=(200, 190),
        target=(1, 191),
        command=FM1,
        params=(7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ):
        self.message_type = message_type
        self._source = source
        self.target_system, self.target_component = target
        self.command = command
        for index, value in enumerate(params, 1):
            setattr(self, f"param{index}", value)

    def get_type(self):
        return self.message_type

    def get_srcSystem(self):
        return self._source[0]

    def get_srcComponent(self):
        return self._source[1]


class InertCommandTransport:
    def __init__(self, *, source=(1, 191), wire_protocol="2.0"):
        self.source_identity = SourceIdentity(*source)
        self.wire_protocol = wire_protocol
        self.callbacks = {}
        self.acks = []
        self.raise_on_send = False

    def install_message_callback(self, message_names, callback):
        for name in message_names:
            self.callbacks[name] = callback

    def send_command_ack(self, ack):
        if self.raise_on_send:
            raise RuntimeError("inert ACK failure")
        self.acks.append(ack)

    def deliver(self, packet):
        self.callbacks[packet.get_type()](packet)


def profile(*, wire_protocol="2.0"):
    return FlightProfile(
        profile_id="listener-test",
        raw_sha256="a" * 64,
        firmware="ArduCopter 4.5.7",
        qgc_source=SourceIdentity(200, 190),
        companion_target=SourceIdentity(1, 191),
        flight_controller=SourceIdentity(1, 1),
        freshness_bounds=MappingProxyType({"heartbeat": 1.0}),
        rc_health_max_age=0.5,
        rc_channel=7,
        rc_mode_bands=(RCModeBand("GUIDED", 1400, 1600, "GUIDED"),),
        heartbeat_type=2,
        heartbeat_autopilot=3,
        copter_modes=MappingProxyType(
            {0: "STABILIZE", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}
        ),
        failsafe_active_statuses=frozenset((5,)),
        failsafe_clear_statuses=frozenset((3, 4)),
        decoder_contract_version=1,
        decoder_contract_evidence="sha256:" + "b" * 64,
        decoder_contract_reference="test reference",
        telemetry_requests=(TelemetryRequest(0, 500_000),),
    )


def supervisor(
    *,
    admission_check=lambda _envelope: None,
    consume=lambda _token: None,
    permission_check=lambda: None,
):
    return MissionSupervisor(
        7,
        admission_check=admission_check,
        attempt_consumer=consume,
        permission_check=permission_check,
    )


def installed_listener(
    *, admission_check=lambda _envelope: None, consume=lambda _token: None
):
    transport = InertCommandTransport()
    commands = queue.Queue()
    listener = QGCCommandListener(
        supervisor=supervisor(admission_check=admission_check, consume=consume),
        profile=profile(),
        command_queue=commands,
        clock=lambda: 12.5,
        ack_transport=transport,
        wire_protocol="2.0",
    )
    listener.install()
    return listener, transport, commands


def test_callback_queues_the_exact_complete_command_envelope_after_durable_admission():
    consumed = []
    listener, transport, commands = installed_listener(
        consume=lambda token: consumed.append(token)
    )

    transport.deliver(Packet())

    assert consumed == [7]
    assert commands.get_nowait() == CommandEnvelope(
        source_system=200,
        source_component=190,
        target_system=1,
        target_component=191,
        command=FM1,
        attempt_id=7,
        params=(7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        received_at=12.5,
    )
    assert transport.acks == [
        CommandAck(
            command=FM1,
            result=ACK_IN_PROGRESS,
            progress=0,
            target_system=200,
            target_component=190,
        )
    ]
    assert listener.supervisor.status(FM1) == "QUEUED"


def test_trusted_authority_rejection_receives_denied_ack_without_enqueue():
    listener, transport, commands = installed_listener(
        admission_check=lambda _envelope: (_ for _ in ()).throw(
            AuthorityLost("stale authority")
        )
    )

    transport.deliver(Packet())

    assert commands.empty()
    assert transport.acks[-1].result == ACK_DENIED


def test_valid_duplicate_reports_status_without_rechecking_new_work_or_enqueuing():
    admission_calls = []
    listener, transport, commands = installed_listener(
        admission_check=lambda envelope: admission_calls.append(envelope)
    )
    packet = Packet()
    transport.deliver(packet)
    queued = commands.get_nowait()
    listener.supervisor.begin(FM1)
    admission_calls.clear()

    transport.deliver(packet)

    assert admission_calls == []
    assert commands.empty()
    assert listener.supervisor.reserved_envelope(FM1) == queued
    assert [ack.result for ack in transport.acks] == [
        ACK_IN_PROGRESS,
        ACK_IN_PROGRESS,
    ]


def test_concurrent_duplicate_runs_new_work_admission_once():
    entered = threading.Event()
    release = threading.Event()
    admission_calls = []

    def admission_check(envelope):
        admission_calls.append(envelope)
        entered.set()
        assert release.wait(1.0)

    listener, transport, commands = installed_listener(admission_check=admission_check)
    packet = Packet()
    first = threading.Thread(target=transport.deliver, args=(packet,))
    second = threading.Thread(target=transport.deliver, args=(packet,))

    first.start()
    assert entered.wait(1.0)
    second.start()
    release.set()
    first.join(1.0)
    second.join(1.0)

    assert len(admission_calls) == 1
    assert commands.qsize() == 1
    assert [ack.result for ack in transport.acks] == [
        ACK_IN_PROGRESS,
        ACK_IN_PROGRESS,
    ]


@pytest.mark.parametrize(
    ("packet", "expected_result", "expect_ack"),
    [
        (Packet(source=(201, 190)), None, False),
        (Packet(source=(200, 191)), None, False),
        (Packet(target=(0, 191)), None, False),
        (Packet(target=(1, 0)), None, False),
        (Packet(target=(2, 191)), None, False),
        (Packet(target=(1, 192)), None, False),
        (Packet(message_type="COMMAND_INT"), ACK_UNSUPPORTED, True),
        (Packet(command=RETIRED_COMMAND), ACK_UNSUPPORTED, True),
        (Packet(params=(6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)), ACK_DENIED, True),
        (Packet(params=(7.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)), ACK_DENIED, True),
        (Packet(params=(7.0, 0.0, 0.0, 0.0, float("nan"), 0.0, 0.0)), ACK_DENIED, True),
        (Packet(params=(7.0, 0.0, 0.0, 0.0, True, 0.0, 0.0)), ACK_DENIED, True),
    ],
)
def test_foreign_or_malformed_packets_never_enter_the_command_queue(
    packet, expected_result, expect_ack
):
    _listener, transport, commands = installed_listener()

    transport.deliver(packet)

    assert commands.empty()
    assert bool(transport.acks) is expect_ack
    if expect_ack:
        assert transport.acks[-1].result == expected_result


def test_missing_parameter_is_denied_without_manufacturing_a_zero():
    _listener, transport, commands = installed_listener()
    packet = Packet()
    del packet.param7

    transport.deliver(packet)

    assert commands.empty()
    assert transport.acks[-1].result == ACK_DENIED


def test_invalid_receipt_clock_is_denied_without_escaping_callback():
    transport = InertCommandTransport()
    commands = queue.Queue()

    def broken_clock():
        raise RuntimeError("clock unavailable")

    listener = QGCCommandListener(
        supervisor=supervisor(),
        profile=profile(),
        command_queue=commands,
        clock=broken_clock,
        ack_transport=transport,
        wire_protocol="2.0",
    )
    listener.install()

    transport.deliver(Packet())

    assert commands.empty()
    assert transport.acks[-1].result == ACK_DENIED


@pytest.mark.parametrize("changed_index", range(1, 7))
def test_command_must_match_every_fixed_prepared_action_parameter(changed_index):
    _listener, transport, commands = installed_listener()
    params = [7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    params[changed_index] = 1.0

    transport.deliver(Packet(params=tuple(params)))

    assert commands.empty()
    assert transport.acks[-1].result == ACK_DENIED


def test_mismatched_duplicate_parameters_are_denied():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    commands.get_nowait()

    transport.deliver(
        Packet(params=(7.0, 99.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    )

    assert commands.empty()
    assert listener.supervisor.status(FM1) == "QUEUED"
    assert transport.acks[-1].result == ACK_DENIED


def test_abort_sets_the_shared_latch_immediately_and_bypasses_queue_and_ledger():
    consumed = []
    listener, transport, commands = installed_listener(
        consume=lambda token: consumed.append(token)
    )
    transport.deliver(Packet())
    commands.get_nowait()

    transport.deliver(Packet(command=ABORT_AND_RECOVER))
    transport.deliver(Packet(command=ABORT_AND_RECOVER))

    assert consumed == [7]
    assert commands.empty()
    assert listener.supervisor.abort_reason == "QGC abort command"
    assert listener.supervisor.admission_result(ABORT_AND_RECOVER) == "ACCEPTED"
    assert [ack.result for ack in transport.acks[-2:]] == [
        ACK_ACCEPTED,
        ACK_ACCEPTED,
    ]


def test_qgc_abort_ack_waits_for_active_gpio_and_pending_duplicate_is_in_progress():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    commands.get_nowait()
    listener.supervisor.begin(FM1)
    entered = threading.Event()
    release = threading.Event()
    gpio_done = threading.Event()

    def gpio_output():
        entered.set()
        assert release.wait(1.0)
        gpio_done.set()

    output = threading.Thread(
        target=lambda: listener.supervisor.output_transaction(gpio_output)
    )
    output.start()
    assert entered.wait(0.2)
    first_abort = threading.Thread(
        target=transport.deliver,
        args=(Packet(command=ABORT_AND_RECOVER),),
    )
    first_abort.start()
    deadline = time.monotonic() + 0.2
    while (
        listener.supervisor.admission_result(ABORT_AND_RECOVER) != "IN_PROGRESS"
        and time.monotonic() < deadline
    ):
        time.sleep(0.001)
    try:
        assert listener.supervisor.admission_result(ABORT_AND_RECOVER) == "IN_PROGRESS"
        assert first_abort.is_alive()
        assert transport.acks[-1].command == FM1
        transport.deliver(Packet(command=ABORT_AND_RECOVER))
        assert transport.acks[-1].result == ACK_IN_PROGRESS
    finally:
        release.set()
        output.join(0.5)
        first_abort.join(0.5)

    assert gpio_done.is_set()
    assert not first_abort.is_alive()
    assert transport.acks[-1].result == ACK_ACCEPTED
    with pytest.raises(MissionAbort, match="QGC abort command"):
        listener.supervisor.output_transaction(
            lambda: pytest.fail("output ran after accepted QGC abort")
        )


def test_ack_failure_does_not_undo_the_durable_reservation_or_enable_replay():
    listener, transport, commands = installed_listener()
    transport.raise_on_send = True

    transport.deliver(Packet())
    transport.deliver(Packet())

    assert commands.qsize() == 1
    assert listener.supervisor.status(FM1) == "QUEUED"


@pytest.mark.parametrize(
    ("failure", "expected_detail"),
    [
        (EmptyDetailFailure(), "EmptyDetailFailure"),
        (UnprintableObserverFailure(), "UnprintableObserverFailure"),
        (NonStringDetailFailure(), "NonStringDetailFailure"),
    ],
)
@pytest.mark.parametrize("diagnostic_failure", [KeyboardInterrupt(), SystemExit()])
def test_callback_safely_reports_hostile_errors_and_contains_diagnostic_control_flow(
    failure, expected_detail, diagnostic_failure
):
    diagnostics = []

    def reject(_envelope):
        raise failure

    def diagnose(message):
        diagnostics.append(message)
        raise diagnostic_failure

    transport = InertCommandTransport()
    commands = queue.Queue()
    listener = QGCCommandListener(
        supervisor=supervisor(admission_check=reject),
        profile=profile(),
        command_queue=commands,
        clock=lambda: 12.5,
        ack_transport=transport,
        wire_protocol="2.0",
        diagnostics=diagnose,
    )
    listener.install()

    transport.deliver(Packet())

    assert diagnostics == [
        f"QGC command callback rejected packet: {expected_detail}"
    ]
    assert commands.empty()
    assert transport.acks == []


@pytest.mark.parametrize(
    ("failure", "expected_detail"),
    [
        (EmptyDetailFailure(), "EmptyDetailFailure"),
        (UnprintableObserverFailure(), "UnprintableObserverFailure"),
        (NonStringDetailFailure(), "NonStringDetailFailure"),
    ],
)
@pytest.mark.parametrize("diagnostic_failure", [KeyboardInterrupt(), SystemExit()])
def test_ack_failure_is_reported_once_without_retry_or_losing_admission(
    failure, expected_detail, diagnostic_failure
):
    diagnostics = []

    class FailingAckTransport(InertCommandTransport):
        def __init__(self):
            super().__init__()
            self.send_attempts = 0
            self.attempted_acks = []

        def send_command_ack(self, ack):
            self.send_attempts += 1
            self.attempted_acks.append(ack)
            raise failure

    def diagnose(message):
        diagnostics.append(message)
        raise diagnostic_failure

    transport = FailingAckTransport()
    commands = queue.Queue()
    listener = QGCCommandListener(
        supervisor=supervisor(),
        profile=profile(),
        command_queue=commands,
        clock=lambda: 12.5,
        ack_transport=transport,
        wire_protocol="2.0",
        diagnostics=diagnose,
    )
    listener.install()

    transport.deliver(Packet())

    assert transport.send_attempts == 1
    assert transport.attempted_acks == [
        CommandAck(
            command=FM1,
            result=ACK_IN_PROGRESS,
            progress=0,
            target_system=200,
            target_component=190,
        )
    ]
    assert diagnostics == [
        f"QGC ACK failed for command {FM1}: {expected_detail}"
    ]
    assert commands.qsize() == 1
    assert listener.supervisor.status(FM1) == "QUEUED"


def test_listener_rejects_transport_with_wrong_actual_outbound_source():
    with pytest.raises(ValueError, match="outbound source"):
        QGCCommandListener(
            supervisor=supervisor(),
            profile=profile(),
            command_queue=queue.Queue(),
            clock=lambda: 1.0,
            ack_transport=InertCommandTransport(source=(1, 192)),
            wire_protocol="2.0",
        )


def test_listener_rejects_a_bounded_command_queue_that_can_block_receive():
    with pytest.raises(ValueError, match="unbounded"):
        QGCCommandListener(
            supervisor=supervisor(),
            profile=profile(),
            command_queue=queue.Queue(maxsize=1),
            clock=lambda: 1.0,
            ack_transport=InertCommandTransport(),
            wire_protocol="2.0",
        )


def test_listener_rejects_a_profile_shaped_placeholder():
    fake_profile = SimpleNamespace(
        qgc_source=SourceIdentity(200, 190),
        companion_target=SourceIdentity(1, 191),
    )
    with pytest.raises(TypeError, match="FlightProfile"):
        QGCCommandListener(
            supervisor=supervisor(),
            profile=fake_profile,
            command_queue=queue.Queue(),
            clock=lambda: 1.0,
            ack_transport=InertCommandTransport(),
            wire_protocol="2.0",
        )


@pytest.mark.parametrize(
    "modes",
    [
        {0: "STABILIZE", 4: "GUIDED", 5: "LOITER", 9: "LAND"},
        {0: "STABILIZE", 4: "GUIDED", 5: "LOITER", 6: "RTL"},
        {0: "STABILIZE", 4: "GUIDED", 5: "LOITER", 6: "LAND", 9: "RTL"},
    ],
)
def test_listener_requires_selected_copter_rtl_and_land_mode_numbers(modes):
    invalid_profile = replace(profile(), copter_modes=MappingProxyType(modes))
    with pytest.raises(ValueError, match="RTL=6 and LAND=9"):
        QGCCommandListener(
            supervisor=supervisor(),
            profile=invalid_profile,
            command_queue=queue.Queue(),
            clock=lambda: 1.0,
            ack_transport=InertCommandTransport(),
            wire_protocol="2.0",
        )


def test_mavlink_1_ack_omits_extension_targets():
    transport = InertCommandTransport(wire_protocol="1.0")
    listener = QGCCommandListener(
        supervisor=supervisor(),
        profile=profile(wire_protocol="1.0"),
        command_queue=queue.Queue(),
        clock=lambda: 1.0,
        ack_transport=transport,
        wire_protocol="1.0",
    )
    listener.install()

    transport.deliver(Packet())

    assert transport.acks[-1].target_system is None
    assert transport.acks[-1].target_component is None


def test_execution_owner_runs_one_phase_and_sends_terminal_status():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    envelope = listener.supervisor.reserved_envelope(FM1)
    calls = []
    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda actual: calls.append(actual) or True},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: calls.append("recovery"),
    )

    assert owner.process_next(timeout_s=0.0) == "SUCCEEDED"

    assert calls == [envelope]
    assert commands.unfinished_tasks == 0
    assert listener.supervisor.admission_result(FM1) == "SUCCEEDED"
    assert transport.acks[-1].result == ACK_ACCEPTED


def test_phase_observer_follows_admission_begin_deadline_handler_finish_order():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    events = [("receiver", transport.acks[-1].result)]

    def observe(phase, state):
        events.append(("event", phase, state, listener.supervisor.status(FM1)))

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: events.append(("handler", FM1)) or True},
        clock=lambda: events.append(("deadline", FM1)) or 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=lambda envelope, result: events.append(("ack", envelope.command, result)),
        recovery=lambda: events.append(("recovery",)),
        phase_observer=observe,
    )

    assert owner.process_next(timeout_s=0.0) == "SUCCEEDED"
    assert events == [
        ("receiver", ACK_IN_PROGRESS),
        ("deadline", FM1),
        ("deadline", FM1),
        ("event", "FM1", "STARTED", "RUNNING"),
        ("handler", FM1),
        ("deadline", FM1),
        ("event", "FM1", "COMPLETE", "TERMINAL"),
        ("ack", FM1, "SUCCEEDED"),
    ]


def test_late_abort_emits_started_without_complete_and_recovers_once():
    listener, _transport, commands = installed_listener()
    listener._transport.deliver(Packet())
    events = []

    def handler(_envelope):
        listener.request_abort("late abort")
        return True

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: handler},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=lambda _envelope, result: events.append(("ack", result)),
        recovery=lambda: events.append(("recovery",)),
        phase_observer=lambda phase, state: events.append((phase, state)),
    )

    assert owner.process_next(timeout_s=0.0) == "ABORTED"
    assert events == [("FM1", "STARTED"), ("ack", "ABORTED"), ("recovery",)]
    assert commands.unfinished_tasks == 0


@pytest.mark.parametrize(
    "failure",
    [
        RuntimeError("publisher failed"),
        KeyboardInterrupt(),
        SystemExit(),
        UnprintableObserverFailure(),
    ],
)
def test_started_observer_failure_preserves_ack_task_done_and_one_recovery(failure):
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    calls = []

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: pytest.fail("handler ran after event failure")},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: calls.append("recovery"),
        phase_observer=lambda _phase, _state: (_ for _ in ()).throw(failure),
    )

    with pytest.raises(type(failure)) as raised:
        owner.process_next(timeout_s=0.0)

    assert raised.value is failure
    assert listener.supervisor.admission_result(FM1) == "FAILED"
    assert transport.acks[-1].result == 4
    assert commands.unfinished_tasks == 0
    assert calls == ["recovery"]


@pytest.mark.parametrize("failure", [RuntimeError("publisher failed"), KeyboardInterrupt()])
def test_complete_observer_failure_keeps_success_ack_and_cleanup_then_propagates(failure):
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    calls = []

    def observe(_phase, state):
        calls.append(state)
        if state == "COMPLETE":
            raise failure

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: True},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: calls.append("recovery"),
        phase_observer=observe,
    )

    with pytest.raises(type(failure)) as raised:
        owner.process_next(timeout_s=0.0)

    assert raised.value is failure
    assert listener.supervisor.admission_result(FM1) == "SUCCEEDED"
    assert transport.acks[-1].result == ACK_ACCEPTED
    assert commands.unfinished_tasks == 0
    assert calls == ["STARTED", "COMPLETE", "recovery"]


def test_observer_control_flow_failure_survives_recovery_control_flow_failure():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    primary = KeyboardInterrupt("publisher interrupted")
    secondary = SystemExit("recovery interrupted")
    recoveries = []

    def recover():
        recoveries.append("recovery")
        raise secondary

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: pytest.fail("handler must not run")},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=recover,
        phase_observer=lambda _phase, _state: (_ for _ in ()).throw(primary),
    )

    with pytest.raises(KeyboardInterrupt) as raised:
        owner.process_next(timeout_s=0.0)

    assert raised.value is primary
    assert raised.value.__cause__ is secondary
    assert transport.acks[-1].result == 4
    assert commands.unfinished_tasks == 0
    assert recoveries == ["recovery"]


@pytest.mark.parametrize("fail_state", ["STARTED", "COMPLETE"])
def test_observer_control_flow_failure_survives_terminal_ack_control_flow_failure(
    fail_state,
):
    listener, _transport, commands = installed_listener()
    listener._transport.deliver(Packet())
    primary = SystemExit(f"{fail_state} publisher interrupted")
    ack_error = KeyboardInterrupt("ACK interrupted")
    calls = []

    def observe(_phase, state):
        calls.append(state)
        if state == fail_state:
            raise primary

    def acknowledge(_envelope, _result):
        calls.append("ack")
        raise ack_error

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: calls.append("handler") or True},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=acknowledge,
        recovery=lambda: calls.append("recovery"),
        phase_observer=observe,
    )

    with pytest.raises(SystemExit) as raised:
        owner.process_next(timeout_s=0.0)

    assert raised.value is primary
    assert commands.unfinished_tasks == 0
    assert calls.count("ack") == 1
    assert calls.count("recovery") == 1


def test_final_fm2_complete_is_after_confirmed_home_recovery():
    active = MissionSupervisor(
        7,
        admission_check=lambda _envelope: None,
        attempt_consumer=lambda _token: None,
        permission_check=lambda: None,
        enabled_phases=(FM1, FM2),
    )
    first = CommandEnvelope(200, 190, 1, 191, FM1, 7, (7.0,) + (0.0,) * 6, 1.0)
    assert active.admit(first)
    active.begin(FM1)
    active.finish(FM1, "SUCCEEDED")
    second = replace(first, command=FM2)
    assert active.admit(second)
    commands = queue.Queue()
    commands.put(second)
    events = []

    def recover():
        events.append("recover")
        active.record_recovery_outcome("HOME_LANDED")

    owner = CommandExecutionOwner(
        supervisor=active,
        command_queue=commands,
        handlers={FM2: lambda _envelope: events.append("handler") or True},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=lambda _envelope, result: events.append(("ack", result)),
        recovery=recover,
        phase_observer=lambda phase, state: events.append((phase, state)),
    )

    assert owner.process_next(timeout_s=0.0) == "SUCCEEDED"
    assert events == [
        ("FM2", "STARTED"),
        "handler",
        "recover",
        ("FM2", "COMPLETE"),
        ("ack", "SUCCEEDED"),
    ]


def test_waypoint_command_emits_no_phase_event():
    active = supervisor()
    request = CommandEnvelope(
        200,
        190,
        1,
        191,
        UPDATE_L,
        7,
        (7.0, 41.0, -81.0, 10.0, 0.0, 0.0, 0.0),
        1.0,
    )
    assert active.admit(request)
    commands = queue.Queue()
    commands.put(request)
    observed = []
    owner = CommandExecutionOwner(
        supervisor=active,
        command_queue=commands,
        handlers={UPDATE_L: lambda _envelope: True},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=lambda _envelope, _result: None,
        recovery=lambda: None,
        phase_observer=lambda phase, state: observed.append((phase, state)),
    )

    assert owner.process_next(timeout_s=0.0) == "SUCCEEDED"
    assert observed == []


def test_execution_owner_caps_queue_wait_at_remaining_attempt_time():
    class RecordingQueue(queue.Queue):
        def __init__(self):
            super().__init__()
            self.waits = []

        def get(self, block=True, timeout=None):
            self.waits.append(timeout)
            raise queue.Empty

    commands = RecordingQueue()
    owner = CommandExecutionOwner(
        supervisor=supervisor(),
        command_queue=commands,
        handlers={},
        clock=lambda: 10.0,
        attempt_timeout_s=600.0,
        idle_poll_s=5.0,
        terminal_ack=lambda *_args: None,
        recovery=lambda: None,
    )
    owner._deadline = 10.05

    assert owner.process_next() is None
    assert commands.waits == [pytest.approx(0.05)]


def test_running_abort_uses_finalized_status_for_ack_return_and_recovery():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    calls = []

    def abort_during_phase(_envelope):
        listener.request_abort("abort during phase")
        return True

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: abort_during_phase},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: calls.append("recovery"),
    )

    assert owner.process_next(timeout_s=0.0) == "ABORTED"

    assert listener.supervisor.admission_result(FM1) == "ABORTED"
    assert transport.acks[-1].result == ACK_CANCELLED
    assert calls == ["recovery"]


def test_idle_clock_failure_terminalizes_and_recovers_before_propagating():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    failed = [False]
    clock_error = timebase.ClockError("host clock stopped")

    def clock():
        if failed[0]:
            raise clock_error
        return 20.0

    recoveries = []
    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: True},
        clock=clock,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: recoveries.append("recovery"),
    )
    assert owner.process_next(timeout_s=0.0) == "SUCCEEDED"
    failed[0] = True

    with pytest.raises(timebase.ClockError) as caught:
        owner.process_next(timeout_s=0.0)

    assert caught.value is clock_error
    assert listener.supervisor.terminal_result == "ABORTED"
    assert listener.supervisor.abort_reason == "listener clock unavailable"
    assert recoveries == ["recovery"]


def test_active_idle_poll_revokes_attempt_on_authority_loss():
    permission_calls = []
    authority_available = [True]

    def permission_check():
        permission_calls.append("check")
        if not authority_available[0]:
            raise AuthorityLost("pilot takeover")

    transport = InertCommandTransport()
    commands = queue.Queue()
    active_supervisor = supervisor(permission_check=permission_check)
    listener = QGCCommandListener(
        supervisor=active_supervisor,
        profile=profile(),
        command_queue=commands,
        clock=lambda: 12.5,
        ack_transport=transport,
        wire_protocol="2.0",
    )
    listener.install()
    transport.deliver(Packet())
    recoveries = []
    owner = CommandExecutionOwner(
        supervisor=active_supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: True},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: recoveries.append("recovery"),
    )
    assert owner.process_next(timeout_s=0.0) == "SUCCEEDED"
    authority_available[0] = False

    assert owner.process_next(timeout_s=0.0) == "ABORTED"

    assert permission_calls == ["check"]
    assert active_supervisor.abort_reason == "authority lost: pilot takeover"
    assert active_supervisor.terminal_result == "ABORTED"
    assert recoveries == ["recovery"]


def test_terminal_duplicate_reports_the_exact_completed_result():
    listener, transport, commands = installed_listener()
    packet = Packet()
    transport.deliver(packet)
    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: False},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: None,
    )
    assert owner.process_next(timeout_s=0.0) == "FAILED"

    transport.deliver(packet)

    assert commands.empty()
    assert transport.acks[-1].result == 4


def test_false_phase_result_stops_normal_work_and_recovers_once():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    calls = []
    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: False},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: calls.append("recovery"),
    )

    assert owner.process_next(timeout_s=0.0) == "FAILED"
    assert owner.run() == "FAILED"

    assert calls == ["recovery"]
    assert commands.unfinished_tasks == 0
    assert transport.acks[-1].result == 4


def test_terminal_ack_error_cannot_skip_task_done_or_recovery():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    recoveries = []

    def broken_ack(_envelope, _result):
        raise RuntimeError("ACK unavailable")

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: False},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=broken_ack,
        recovery=lambda: recoveries.append("recovery"),
    )

    assert owner.process_next(timeout_s=0.0) == "FAILED"

    assert commands.unfinished_tasks == 0
    assert recoveries == ["recovery"]


def test_abort_of_queued_phase_skips_handler_reports_terminal_and_recovers_once():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    transport.deliver(Packet(command=ABORT_AND_RECOVER))
    calls = []
    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: calls.append("handler") or True},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: calls.append("recovery"),
    )

    assert owner.process_next(timeout_s=0.0) == "ABORTED"

    assert calls == ["recovery"]
    assert commands.unfinished_tasks == 0
    assert transport.acks[-1].result == 6


def test_queued_abort_publishes_terminal_phase_status_before_recovery_output():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    listener.request_abort("SIGINT")
    order = []
    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: order.append("handler") or True},
        clock=lambda: 20.0,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=lambda _envelope, result: order.append(f"terminal:{result}"),
        recovery=lambda: order.append("recovery"),
    )

    assert owner.process_next(timeout_s=0.0) == "ABORTED"

    assert order == ["terminal:ABORTED", "recovery"]


def test_attempt_deadline_continues_while_idle_after_fm2():
    now = [10.0]
    shared_clock = SimpleNamespace(
        now=lambda: now[0],
        sleep=lambda seconds: now.__setitem__(0, now[0] + seconds),
    )
    listener, transport, commands = installed_listener()
    calls = []
    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={
            31000: lambda _envelope: True,
            31001: lambda _envelope: True,
        },
        clock=lambda: now[0],
        attempt_timeout_s=600.0,
        idle_poll_s=0.001,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: calls.append("recovery"),
    )
    with timebase.configured(shared_clock):
        transport.deliver(Packet(command=31000))
        assert owner.process_next(timeout_s=0.0) == "SUCCEEDED"
        transport.deliver(Packet(command=31001))
        assert owner.process_next(timeout_s=0.0) == "SUCCEEDED"

    now[0] = 610.0
    assert owner.process_next(timeout_s=0.0) == "ABORTED"

    assert listener.supervisor.abort_reason == "attempt deadline expired"
    assert calls == ["recovery"]


def test_clock_cancellation_is_propagated_after_one_recovery_claim():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    recoveries = []

    def cancelled():
        raise timebase.ClockError("host clock stopped")

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: True},
        clock=cancelled,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=lambda: recoveries.append("recovery"),
    )

    with pytest.raises(timebase.ClockError, match="host clock stopped"):
        owner.process_next(timeout_s=0.0)

    assert recoveries == ["recovery"]
    assert commands.unfinished_tasks == 0


def test_phase_clock_cancellation_is_not_replaced_by_recovery_failure():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    original = timebase.ClockError("phase clock stopped")
    recoveries = []

    def cancelled():
        raise original

    def failed_recovery():
        recoveries.append("recovery")
        raise timebase.ClockError("recovery clock stopped")

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: lambda _envelope: True},
        clock=cancelled,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=failed_recovery,
    )

    with pytest.raises(timebase.ClockError) as caught:
        owner.process_next(timeout_s=0.0)

    assert caught.value is original
    assert recoveries == ["recovery"]
    assert commands.unfinished_tasks == 0


def test_repeated_sigint_requests_latch_one_abort_without_running_recovery_in_callback():
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())

    listener.request_abort("SIGINT")
    listener.request_abort("SIGINT repeated")

    assert listener.supervisor.abort_reason == "SIGINT"
    assert commands.qsize() == 1
    assert [ack.result for ack in transport.acks] == [ACK_IN_PROGRESS]


def test_blocked_ack_does_not_hold_admission_lock_against_abort():
    class BlockingAckTransport(InertCommandTransport):
        def __init__(self):
            super().__init__()
            self.send_started = threading.Event()
            self.release_send = threading.Event()

        def send_command_ack(self, ack):
            self.send_started.set()
            assert self.release_send.wait(1.0)
            super().send_command_ack(ack)

    transport = BlockingAckTransport()
    commands = queue.Queue()
    listener = QGCCommandListener(
        supervisor=supervisor(),
        profile=profile(),
        command_queue=commands,
        clock=lambda: 12.5,
        ack_transport=transport,
        wire_protocol="2.0",
    )
    listener.install()
    delivery = threading.Thread(target=transport.deliver, args=(Packet(),))
    delivery.start()
    assert transport.send_started.wait(1.0)

    abort_delivery = threading.Thread(
        target=transport.deliver,
        args=(Packet(command=ABORT_AND_RECOVER),),
    )
    abort_delivery.start()
    abort_delivery.join(0.1)

    abort_latched_while_ack_blocked = (
        listener.supervisor.abort_reason == "QGC abort command"
    )
    transport.release_send.set()
    delivery.join(1.0)
    abort_delivery.join(1.0)
    assert abort_latched_while_ack_blocked is True


def test_blocking_phase_uses_shared_deadline_then_recovers_after_context_unwinds():
    class Clock:
        def __init__(self):
            self.value = 0.0

        def now(self):
            return self.value

        def sleep(self, seconds):
            self.value += seconds

    clock = Clock()
    listener, transport, commands = installed_listener()
    transport.deliver(Packet())
    calls = []

    def blocking_phase(_envelope):
        timebase.sleep(601.0)
        calls.append("after deadline")
        return True

    def recover():
        calls.append("recovery")
        assert timebase.monotonic() == 600.0

    owner = CommandExecutionOwner(
        supervisor=listener.supervisor,
        command_queue=commands,
        handlers={FM1: blocking_phase},
        clock=timebase.monotonic,
        attempt_timeout_s=600.0,
        idle_poll_s=0.1,
        terminal_ack=listener.acknowledge_terminal,
        recovery=recover,
    )

    with timebase.configured(clock):
        assert owner.process_next(timeout_s=0.0) == "ABORTED"

    assert calls == ["recovery"]
    assert listener.supervisor.abort_reason == "attempt deadline expired"


def prepared_startup_files(tmp_path, *, include_runtime_modes=True):
    from test_prepare_attempt import profile_data

    data = profile_data()
    if include_runtime_modes:
        data["observation"]["mode_mapping"].update({"6": "RTL", "9": "LAND"})
    profile_path = tmp_path / "profile.json"
    profile_path.write_text(__import__("json").dumps(data))
    ledger_path = tmp_path / "attempt-ledger.json"
    session_path = tmp_path / "listener-session.json"
    actions_path = tmp_path / "qgc-actions.json"
    initialize_ledger(ledger_path)
    prepared = prepare_attempt(
        profile_path,
        ledger_path,
        session_path,
        actions_path,
        acknowledge_on_ground=True,
    )
    return (
        ListenerStartupFiles(
            profile_path=profile_path,
            session_path=session_path,
            actions_path=actions_path,
            ledger_path=ledger_path,
        ),
        prepared,
    )


def test_startup_fixes_exact_profile_session_actions_and_unconsumed_ledger(tmp_path):
    files, prepared = prepared_startup_files(tmp_path)

    artifacts = load_listener_artifacts(files)

    assert artifacts.prepared_attempt == prepared
    assert artifacts.flight_profile.profile_id == prepared.profile_id
    assert artifacts.flight_profile.raw_sha256 == artifacts.deployment_profile.raw_sha256
    assert artifacts.wire_protocol == "2.0"
    assert artifacts.ledger.path == files.ledger_path.resolve()


def test_validated_artifact_snapshot_cannot_be_dataclass_replaced(tmp_path):
    files, _prepared = prepared_startup_files(tmp_path)
    artifacts = load_listener_artifacts(files)

    with pytest.raises(TypeError):
        replace(artifacts, profile_sha256="0" * 64)


def test_altered_snapshot_relationships_are_revalidated_before_factory(tmp_path):
    files, _prepared = prepared_startup_files(tmp_path)
    artifacts = load_listener_artifacts(files)
    object.__setattr__(
        artifacts,
        "_prepared_attempt",
        replace(artifacts.prepared_attempt, profile_id="altered-profile"),
    )
    calls = []

    with pytest.raises(ValueError, match="relationships"):
        construct_after_listener_validation(
            artifacts, lambda validated: calls.append(validated)
        )
    assert calls == []


def test_runtime_modes_are_validated_before_any_component_factory_runs(tmp_path):
    files, _prepared = prepared_startup_files(
        tmp_path, include_runtime_modes=False
    )
    factory_calls = []

    with pytest.raises(ValueError, match="RTL=6 and LAND=9"):
        construct_after_listener_validation(
            files,
            lambda artifacts: factory_calls.append(artifacts),
        )

    assert factory_calls == []


def test_consumed_attempt_is_rejected_before_any_component_factory_runs(tmp_path):
    files, prepared = prepared_startup_files(tmp_path)
    AttemptLedger(files.ledger_path).consume(prepared.attempt_id)
    factory_calls = []

    with pytest.raises(Exception, match="consumed"):
        construct_after_listener_validation(
            files,
            lambda artifacts: factory_calls.append(artifacts),
        )

    assert factory_calls == []
