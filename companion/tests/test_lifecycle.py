from __future__ import annotations

from io import StringIO

from drone_sim_companion.lifecycle import CompanionLifecycle
from drone_sim_companion.mission import MissionPhase, MissionState


class Protocol:
    def __init__(self) -> None:
        self.statuses: list[tuple[str, dict[str, object]]] = []
        self.quiescence: list[str] = []

    def write_status(self, name: str, document: dict[str, object]) -> None:
        self.statuses.append((name, document))

    def write_quiescence(self, module: str) -> None:
        self.quiescence.append(module)


def test_lifecycle_persists_readiness_landed_completion_and_silence_boundary() -> None:
    protocol = Protocol()
    stream = StringIO()
    lifecycle = CompanionLifecycle(
        run_id="00000000-0000-4000-8000-000000000001",
        protocol=protocol,
        stream=stream,
    )
    lifecycle.mark_ready(50)
    lifecycle.mark_ready(100)
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
                "heartbeat_sim_timestamp_ns": 50,
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
