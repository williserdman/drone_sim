from drone import timebase


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
