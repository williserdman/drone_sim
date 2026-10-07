from types import SimpleNamespace

import pytest

from drone_sim_companion.comp2026_host import SimulationClock
from drone_sim_companion import runtime_node


@pytest.mark.parametrize("timestamp", [0, 50_000_000])
def test_initial_guarded_enqueue_and_delivery_share_clock_boundary(timestamp):
    clock = SimulationClock()
    clock.accept(timestamp)
    control = SimpleNamespace(_guided_output_delivery_reported=False)
    events = []

    def native(output, before_final):
        before_final()
        output()
        return "enqueued"

    transaction = runtime_node._comp2026_initial_transport(clock, control, native)
    result = transaction(lambda: events.append("output"), lambda: events.append("boundary"))

    assert result == "enqueued"
    assert events == ["boundary", "output"]


def test_late_initial_guarded_enqueue_is_rejected_before_any_transport_output():
    clock = SimulationClock()
    clock.accept(50_000_001)
    calls = []
    transaction = runtime_node._comp2026_initial_transport(
        clock, SimpleNamespace(_guided_output_delivery_reported=False),
        lambda *args: calls.append(args),
    )
    with pytest.raises(RuntimeError, match="50 ms"):
        transaction(lambda: None, lambda: None)
    assert calls == []


def test_later_flight_outputs_keep_existing_transport_guard_without_startup_window():
    clock = SimulationClock()
    clock.accept(4_000_000_000)
    calls = []
    transaction = runtime_node._comp2026_initial_transport(
        clock, SimpleNamespace(_guided_output_delivery_reported=True),
        lambda *args: calls.append(args),
    )
    transaction("output", "boundary")
    assert calls == [("output", "boundary")]
