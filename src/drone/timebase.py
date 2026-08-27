"""Wall-default clock seam for original blocking mission code."""

from contextlib import contextmanager
import time as _wall_time
from typing import Iterator, Protocol


class Clock(Protocol):
    def now(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


class _WallClock:
    def now(self) -> float:
        return _wall_time.time()

    def sleep(self, seconds: float) -> None:
        _wall_time.sleep(seconds)


_active: Clock = _WallClock()
_deadline_limit: tuple[float, float] | None = None


def _check_deadline(now: float) -> None:
    if _deadline_limit is None:
        return
    start, duration = _deadline_limit
    if now - start > duration:
        raise TimeoutError(f"mission exceeded {duration:g} simulated seconds")


def time() -> float:
    now = _active.now()
    _check_deadline(now)
    return now


def monotonic() -> float:
    return time()


def sleep(seconds: float) -> None:
    _active.sleep(seconds)
    _check_deadline(_active.now())


@contextmanager
def configured(clock: Clock) -> Iterator[None]:
    global _active
    previous, _active = _active, clock
    try:
        yield
    finally:
        _active = previous


@contextmanager
def _deadline(start: float, duration: float) -> Iterator[None]:
    global _deadline_limit
    previous, _deadline_limit = _deadline_limit, (start, duration)
    try:
        yield
    finally:
        _deadline_limit = previous
