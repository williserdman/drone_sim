"""One reentrant workstation lease shared by CLI, suites and console workers."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import fcntl
from functools import wraps
import os
from pathlib import Path
import stat
import threading
from typing import ParamSpec, TypeVar


_process_lock = threading.RLock()
_held = threading.local()
_P = ParamSpec("_P")
_T = TypeVar("_T")


@contextmanager
def workstation_lock(path: Path | None = None, *, allow_nested: bool = False) -> Iterator[None]:
    """Nested suite/controller calls share the lease; unrelated workers fail fast."""
    path = path or Path(f"/tmp/drone-sim-suite-{os.getuid()}.lock")
    if not _process_lock.acquire(blocking=False):
        raise RuntimeError("another drone-sim session already owns this workstation (already running)")
    descriptor: int | None = None
    try:
        current = getattr(_held, "path", None)
        if current is not None:
            if not allow_nested:
                raise RuntimeError("another drone-sim session already owns this workstation (already running)")
            if current != path:
                raise RuntimeError("cannot acquire a different workstation lease recursively")
            yield
            return
        descriptor = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o600)
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_uid != os.getuid() or metadata.st_nlink != 1:
            raise RuntimeError("unsafe workstation lease file")
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("another drone-sim session already owns this workstation (already running)") from error
        _held.path = path
        try:
            yield
        finally:
            del _held.path
            fcntl.flock(descriptor, fcntl.LOCK_UN)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        _process_lock.release()


def exclusive_run(operation: Callable[_P, _T]) -> Callable[_P, _T]:
    @wraps(operation)
    def run(*args: _P.args, **kwargs: _P.kwargs) -> _T:
        with workstation_lock(allow_nested=True):
            return operation(*args, **kwargs)
    return run
