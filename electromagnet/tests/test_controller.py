from __future__ import annotations

from io import StringIO

from drone_sim_electromagnet.controller import ScenarioController
from drone_sim_electromagnet.scenario import InactiveScenarioEvent, ScenarioPolicy


class Protocol:
    def __init__(self) -> None:
        self.quiescence: list[str] = []

    def write_quiescence(self, module: str) -> None:
        self.quiescence.append(module)


def test_controller_publishes_once_logs_readiness_and_crosses_silence_boundary() -> None:
    published: list[InactiveScenarioEvent] = []
    protocol = Protocol()
    stream = StringIO()
    controller = ScenarioController(
        run_id="00000000-0000-4000-8000-000000000001",
        policy=ScenarioPolicy(run_id="00000000-0000-4000-8000-000000000001"),
        publish=published.append,
        protocol=protocol,
        stream=stream,
    )
    controller.mark_ready()
    controller.observe_clock(50_000_000)
    controller.observe_clock(100_000_000)
    controller.finalize(150_000_000)
    before = stream.getvalue()
    controller.observe_clock(200_000_000)

    assert len(published) == 1
    assert published[0].state == "INACTIVE"
    assert published[0].timestamp_ns == 50_000_000
    assert protocol.quiescence == ["electromagnet"]
    assert stream.getvalue() == before
    assert '"event":"ready"' in before
    assert '"event":"scenario_event_published"' in before
    assert '"event":"finalizing"' in before


def test_failure_is_structured_before_quiescence() -> None:
    protocol = Protocol()
    stream = StringIO()
    controller = ScenarioController(
        run_id="00000000-0000-4000-8000-000000000001",
        policy=ScenarioPolicy(run_id="00000000-0000-4000-8000-000000000001"),
        publish=lambda _event: None,
        protocol=protocol,
        stream=stream,
    )
    controller.fail(10, "clock regressed")
    controller.finalize(10)
    assert '"event":"scenario_failed"' in stream.getvalue()
    assert '"severity":"ERROR"' in stream.getvalue()
    assert protocol.quiescence == ["electromagnet"]


def test_search_delivery_readiness_reports_physical_payload_authority() -> None:
    stream = StringIO()
    controller = ScenarioController(
        run_id="00000000-0000-4000-8000-000000000001",
        policy=None,
        publish=lambda _event: None,
        protocol=Protocol(),
        stream=stream,
        scenario="search_delivery_v1",
    )

    controller.mark_ready()

    readiness = stream.getvalue()
    assert '"scenario":"search_delivery_v1"' in readiness
    assert '"physical_force":true' in readiness
