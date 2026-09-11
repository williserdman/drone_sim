from __future__ import annotations

from io import StringIO

from artifacts.runtime_status import (
    CompanionReadyStatus,
    MissionCommandDeliveredStatus,
    MissionFinishedStatus,
    MissionReadyStatus,
    RuntimeFailureStatus,
)
from drone_sim_companion.lifecycle import CompanionLifecycle
from drone_sim_companion.mission import CommandKind, MissionPhase, MissionState


class Protocol:
    def __init__(self) -> None:
        self.statuses = []
        self.quiescence: list[str] = []

    def write_status(self, status) -> None:
        self.statuses.append(status)

    def write_quiescence(self, module: str) -> None:
        self.quiescence.append(module)


def test_command_delivery_persists_a_skipped_zero_timestamp() -> None:
    protocol = Protocol()
    lifecycle = CompanionLifecycle(
        run_id="00000000-0000-4000-8000-000000000001",
        protocol=protocol,
        stream=StringIO(),
    )

    lifecycle.observe_command_delivery(CommandKind.SET_GUIDED, 50_000_000)

    assert protocol.statuses == [
        MissionCommandDeliveredStatus(
            "00000000-0000-4000-8000-000000000001", 50_000_000
        )
    ]


def test_command_delivery_rejects_a_timestamp_after_the_paused_window() -> None:
    protocol = Protocol()
    lifecycle = CompanionLifecycle(
        run_id="00000000-0000-4000-8000-000000000001",
        protocol=protocol,
        stream=StringIO(),
    )

    lifecycle.observe_command_delivery(CommandKind.SET_GUIDED, 50_000_001)

    assert protocol.statuses == []


def test_lifecycle_persists_transport_readiness_landed_completion_and_silence_boundary() -> None:
    protocol = Protocol()
    stream = StringIO()
    lifecycle = CompanionLifecycle(
        run_id="00000000-0000-4000-8000-000000000001",
        protocol=protocol,
        stream=stream,
    )
    lifecycle.mark_transport_ready()
    lifecycle.mark_transport_ready()
    lifecycle.observe_terminal(MissionState(MissionPhase.LANDED, last_timestamp_ns=500))
    lifecycle.finalize(550)
    before = stream.getvalue()
    lifecycle.emit("too_late", 600, {})

    assert protocol.statuses == [
        CompanionReadyStatus("00000000-0000-4000-8000-000000000001"),
        MissionFinishedStatus("00000000-0000-4000-8000-000000000001", 500),
    ]
    assert protocol.quiescence == ["companion"]
    assert stream.getvalue() == before
    assert '"event":"ready"' in before
    assert '"event":"mission_finished"' in before
    assert '"event":"finalizing"' in before


def test_failure_is_terminal_and_logged_once_without_false_finished_status() -> None:
    operations: list[object] = []

    class RecordingProtocol(Protocol):
        def write_status(self, status) -> None:
            super().write_status(status)
            operations.append(status)

    class RecordingStream(StringIO):
        def write(self, value: str) -> int:
            if '"event":"mission_failed"' in value:
                operations.append("mission_failed")
            return super().write(value)

    protocol = RecordingProtocol()
    stream = RecordingStream()
    lifecycle = CompanionLifecycle(
        run_id="00000000-0000-4000-8000-000000000001",
        protocol=protocol,
        stream=stream,
    )
    failed = MissionState(
        MissionPhase.FAILED,
        last_timestamp_ns=70,
        failure_reason="negative acknowledgement",
    )
    lifecycle.observe_terminal(failed)
    lifecycle.observe_terminal(failed)

    failure_status = RuntimeFailureStatus(
        "00000000-0000-4000-8000-000000000001",
        "companion",
        "negative acknowledgement",
        ("logs/docker/companion.log.partial",),
    )
    assert protocol.statuses == [failure_status]
    assert not any(
        isinstance(status, MissionFinishedStatus) for status in protocol.statuses
    )
    assert operations == [failure_status, "mission_failed"]
    assert stream.getvalue().count('"event":"mission_failed"') == 1


def test_mission_readiness_requires_both_passive_facts_and_is_persisted_once() -> None:
    protocol = Protocol()
    stream = StringIO()
    lifecycle = CompanionLifecycle(
        run_id="00000000-0000-4000-8000-000000000001",
        protocol=protocol,
        stream=stream,
    )

    lifecycle.observe_mission_readiness(
        heartbeat_observed=True,
        prearm_checks_healthy=False,
    )
    lifecycle.observe_mission_readiness(
        heartbeat_observed=False,
        prearm_checks_healthy=True,
    )
    assert protocol.statuses == []

    for _ in range(2):
        lifecycle.observe_mission_readiness(
            heartbeat_observed=True,
            prearm_checks_healthy=True,
        )

    assert protocol.statuses == [
        MissionReadyStatus("00000000-0000-4000-8000-000000000001")
    ]
    assert stream.getvalue().count('"event":"mission_ready"') == 1
