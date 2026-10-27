"""The public clock and finalization latch bound the actual enqueue transaction."""
import threading

import pytest
from drone_sim_companion.comp2026_host import SimulationClock
from drone_sim_companion.runtime_node import _Comp2026ShutdownAdmission, _comp2026_shutdown_dependency


def test_signal_before_final_enqueue_closes_output():
    admission = _Comp2026ShutdownAdmission()
    output = []
    transaction = _comp2026_shutdown_dependency(admission, lambda action: action())
    admission.request_stop_from_signal()
    with pytest.raises(RuntimeError, match='finalizing'):
        transaction(lambda: output.append('packet'))
    assert output == []


def test_clock_cannot_cross_initial_delivery_boundary_during_enqueue():
    clock = SimulationClock()
    clock.accept(50_000_000)
    entered = threading.Event()
    release = threading.Event()
    advanced = threading.Event()
    timestamps = []
    def enqueue(timestamp):
        timestamps.append(timestamp)
        entered.set()
        assert release.wait(1)
        timestamps.append(clock.timestamp_ns)
    writer = threading.Thread(target=lambda: clock.run_at_current_timestamp(enqueue))
    writer.start()
    assert entered.wait(1)
    advancing = threading.Thread(target=lambda: (clock.accept(100_000_000), advanced.set()))
    advancing.start()
    try:
        assert not advanced.wait(0.02)
        release.set()
        writer.join(1)
        advancing.join(1)
    finally:
        release.set()
    assert timestamps == [50_000_000, 50_000_000]
    assert advanced.is_set()
