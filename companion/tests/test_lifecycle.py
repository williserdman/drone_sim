from __future__ import annotations

from io import StringIO

from drone_sim_companion.lifecycle import CompanionLifecycle
from drone_sim_companion.mission import CommandKind, MissionPhase, MissionState


class Protocol:
    def __init__(self) -> None:
        self.statuses: list[tuple[str, dict[str, object]]] = []
        self.quiescence: list[str] = []

    def write_status(self, name: str, document: dict[str, object]) -> None:
        self.statuses.append((name, document))

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
        (
            "mission-command-delivered",
            {
                "run_id": "00000000-0000-4000-8000-000000000001",
                "command": "SET_GUIDED",
                "sim_timestamp_ns": 50_000_000,
                "delivered": True,
            },
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
        (
            "companion-ready",
            {
                "run_id": "00000000-0000-4000-8000-000000000001",
                "ready": True,
                "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
                "mavlink_transport_connected": True,
            },
        ),
        (
            "mission-finished",
            {
                "run_id": "00000000-0000-4000-8000-000000000001",
                "finished": True,
                "sim_timestamp_ns": 500,
                "outcome": "LANDED",
            },
        ),
    ]
    assert protocol.quiescence == ["companion"]
    assert stream.getvalue() == before
    assert '"event":"ready"' in before
    assert '"event":"mission_finished"' in before
    assert '"event":"finalizing"' in before


def test_failure_is_terminal_and_logged_once_without_false_finished_status() -> None:
    protocol = Protocol()
    stream = StringIO()
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
    assert protocol.statuses == []
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
        (
            "mission-ready",
            {
                "run_id": "00000000-0000-4000-8000-000000000001",
                "ready": True,
                "heartbeat_observed": True,
                "prearm_checks_healthy": True,
            },
        )
    ]
    assert stream.getvalue().count('"event":"mission_ready"') == 1
