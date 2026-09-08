"""Wall-default clock seam for original blocking mission code."""

from contextlib import contextmanager
from contextvars import ContextVar
import time as _wall_time
from typing import Iterator, Protocol


class Clock(Protocol):
    def now(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...


class ClockError(RuntimeError):
    """The configured clock adapter failed independently of mission logic."""


class _WallClock:
    def now(self) -> float:
        return _wall_time.monotonic()

    def sleep(self, seconds: float) -> None:
        _wall_time.sleep(seconds)


_active: Clock = _WallClock()
_deadline_limit: ContextVar[tuple[float, float] | None] = ContextVar(
    "mission_deadline_limit", default=None
)


def _clock_now() -> float:
    try:
        return _active.now()
    except ClockError:
        raise
    except Exception as error:
        raise ClockError("configured clock now() failed") from error


def _clock_sleep(seconds: float) -> None:
    try:
        _active.sleep(seconds)
    except ClockError:
        raise
    except Exception as error:
        raise ClockError("configured clock sleep() failed") from error


def _check_deadline(now: float) -> None:
    deadline_limit = _deadline_limit.get()
    if deadline_limit is None:
        return
    start, duration = deadline_limit
    if now - start > duration:
        raise TimeoutError(f"mission exceeded {duration:g} simulated seconds")


def monotonic() -> float:
    """Return elapsed mission time, using the injected clock when configured."""
    now = _clock_now()
    _check_deadline(now)
    return now


def time() -> float:
    """Compatibility alias for :func:`monotonic`; do not use for persisted time."""
    return monotonic()


def epoch() -> float:
    """Return the Unix timestamp for data persisted across process restarts."""
    return _wall_time.time()


def sleep(seconds: float) -> None:
    deadline_limit = _deadline_limit.get()
    if deadline_limit is not None:
        start, duration = deadline_limit
        now = _clock_now()
        _check_deadline(now)
        remaining = duration - (now - start)
        if seconds > remaining:
            if remaining > 0:
                _clock_sleep(remaining)
            raise TimeoutError(
                f"mission exceeded {duration:g} simulated seconds"
            )
    _clock_sleep(seconds)
    _check_deadline(_clock_now())


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
    deadline_limit = (start, duration)
    previous = _deadline_limit.get()
    if previous is not None and previous[0] + previous[1] <= start + duration:
        deadline_limit = previous
    token = _deadline_limit.set(deadline_limit)
    try:
        yield
    finally:
        _deadline_limit.reset(token)
