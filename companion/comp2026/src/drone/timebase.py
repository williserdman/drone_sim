"""Scoped mission clock; aircraft execution defaults to the standard clock."""

from contextlib import contextmanager
from contextvars import ContextVar
import time as _time
from typing import Iterator, Protocol


class Clock(Protocol):
    def now(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


_clock: ContextVar[Clock | None] = ContextVar("mission_clock", default=None)


@contextmanager
def configured(clock: Clock) -> Iterator[None]:
    """Use a clock in this execution context, restoring it even on cancellation."""
    token = _clock.set(clock)
    try:
        yield
    finally:
        _clock.reset(token)


def time() -> float:
    clock = _clock.get()
    return _time.time() if clock is None else clock.now()


def monotonic() -> float:
    clock = _clock.get()
    return _time.monotonic() if clock is None else clock.now()


def sleep(seconds: float) -> None:
    clock = _clock.get()
    if clock is None:
        _time.sleep(seconds)
    else:
        clock.sleep(seconds)
