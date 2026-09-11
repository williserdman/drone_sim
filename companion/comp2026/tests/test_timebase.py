import threading

import pytest

from drone import timebase
from drone_sim_companion.comp2026_host import SimulationClock


class FakeClock:
    def __init__(self, now_value: float):
        self.now_value = now_value
        self.sleeps = []

    def now(self) -> float:
        return self.now_value

    def sleep(self, seconds: float) -> None:
        self.sleeps.append(seconds)
        self.now_value += seconds


def test_configured_timebase_sleeps_on_injected_clock():
    """Regression: mission waits must not consume wall time in simulation."""
    fake = FakeClock(now_value=12.0)

    with timebase.configured(fake):
        assert timebase.time() == 12.0
        timebase.sleep(2.5)

    assert fake.sleeps == [2.5]


def test_configured_timebase_restores_the_previous_clock():
    """Regression: leaving a nested clock context must restore its caller."""
    outer = FakeClock(now_value=3.0)
    inner = FakeClock(now_value=9.0)

    with timebase.configured(outer):
        with timebase.configured(inner):
            assert timebase.time() == 9.0
        assert timebase.time() == 3.0


def test_epoch_jump_does_not_change_elapsed_time(monkeypatch):
    monkeypatch.setattr(timebase._wall_time, "monotonic", lambda: 12.0)
    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_800_000_000.0)

    assert timebase.monotonic() == 12.0
    assert timebase.time() == 12.0
    assert timebase.epoch() == 1_800_000_000.0

    monkeypatch.setattr(timebase._wall_time, "time", lambda: 1_700_000_000.0)
    assert timebase.monotonic() == 12.0


def test_expired_deadline_does_not_leak_to_another_thread():
    clock = FakeClock(now_value=0.0)
    deadline_expired = threading.Event()
    release_mission = threading.Event()
    mission_errors = []

    def expire_mission_deadline():
        try:
            with timebase._deadline(0.0, 1.0):
                clock.now_value = 2.0
                with pytest.raises(TimeoutError, match="1"):
                    timebase.monotonic()
                deadline_expired.set()
                release_mission.wait(timeout=1.0)
        except BaseException as error:
            mission_errors.append(error)

    with timebase.configured(clock):
        mission_thread = threading.Thread(target=expire_mission_deadline)
        mission_thread.start()
        assert deadline_expired.wait(timeout=1.0)
        assert timebase.monotonic() == 2.0
        release_mission.set()
        mission_thread.join(timeout=1.0)

    assert not mission_thread.is_alive()
    assert mission_errors == []


def test_expired_deadline_unwinds_before_fresh_bounded_recovery():
    clock = FakeClock(now_value=0.0)

    with timebase.configured(clock):
        with pytest.raises(TimeoutError, match="1"):
            with timebase._deadline(0.0, 1.0):
                timebase.sleep(2.0)

        assert timebase.monotonic() == 1.0

        with pytest.raises(TimeoutError, match="2"):
            with timebase._deadline(timebase.monotonic(), 2.0):
                timebase.sleep(3.0)

    assert clock.now_value == 3.0


def test_nested_deadline_cannot_relax_outer_cancellation():
    clock = FakeClock(now_value=0.0)

    with timebase.configured(clock):
        with pytest.raises(TimeoutError, match="5"):
            with timebase._deadline(0.0, 5.0):
                with timebase._deadline(0.0, 10.0):
                    timebase.sleep(6.0)

    assert clock.now_value == 5.0


def test_stopped_parent_simulation_clock_is_a_typed_infrastructure_failure():
    clock = SimulationClock()
    clock.accept(0)
    clock.stop("host cancellation")

    with timebase.configured(clock):
        with pytest.raises(timebase.ClockError, match="sleep") as failure:
            timebase.sleep(0.1)

    assert isinstance(failure.value, RuntimeError)
    assert isinstance(failure.value.__cause__, RuntimeError)
    assert "host cancellation" in str(failure.value.__cause__)
