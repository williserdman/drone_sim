"""One reentrant workstation lease shared by CLI, suites and console workers."""

from __future__ import annotations

from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import stat
import threading
from typing import Any, Iterator

from .controller_identity import load_or_enroll_identity
from .host_integration import HostIntegration, HostLease


_process_lock = threading.RLock()
_held = threading.local()


def select_workstation(
    workstation: Workstation | None, *dependencies: Any
) -> Workstation | None:
    """Adopt one injected workstation and reject split ownership graphs."""
    candidates = [
        candidate
        for dependency in dependencies
        if (candidate := getattr(dependency, "workstation", None)) is not None
    ]
    selected = workstation or (candidates[0] if candidates else None)
    if selected is not None and any(candidate is not selected for candidate in candidates):
        raise ValueError("injected dependencies must use the same workstation")
    return selected


class Workstation:
    def __init__(
        self, host: HostIntegration, *, lock_path: Path | None = None
    ) -> None:
        self.host = host
        self.lock_path = lock_path or Path(
            f"/tmp/drone-sim-suite-{os.getuid()}.lock"
        )

    def owned(self) -> HostLease:
        if getattr(_held, "workstation", None) is not self:
            raise RuntimeError("workstation lease is not owned by this thread")
        lease = getattr(_held, "lease", None)
        if lease is None:
            raise RuntimeError("workstation lease is not owned by this thread")
        return lease

    @contextmanager
    def hold(
        self,
        *,
        allow_nested: bool = False,
        recovery_run_id: str | None = None,
    ) -> Iterator[HostLease]:
        if not _process_lock.acquire(blocking=False):
            raise RuntimeError(
                "another drone-sim session already owns this workstation (already running)"
            )
        descriptor: int | None = None
        try:
            current = getattr(_held, "workstation", None)
            if current is not None:
                if not allow_nested:
                    raise RuntimeError(
                        "another drone-sim session already owns this workstation (already running)"
                    )
                if current is not self:
                    raise RuntimeError(
                        "cannot acquire a different workstation lease recursively"
                    )
                if recovery_run_id is not None:
                    raise RuntimeError(
                        "nested workstation lease cannot change recovery ownership"
                    )
                yield self.owned()
                return

            descriptor = os.open(
                self.lock_path,
                os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC,
                0o600,
            )
            metadata = os.fstat(descriptor)
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_uid != os.getuid()
                or metadata.st_nlink != 1
            ):
                raise RuntimeError("unsafe workstation lease file")
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError(
                    "another drone-sim session already owns this workstation (already running)"
                ) from error

            with self.host.acquire_native_lease() as lease:
                if self.host.facts.kind == "wsl2":
                    registration = load_or_enroll_identity(
                        self.host.state_directory, lease
                    ).registration
                    active_run_id = registration.active_run_id
                    if registration.recovery_required and active_run_id is None:
                        raise RuntimeError(
                            "native controller ownership requires explicit recovery inspection"
                        )
                    if recovery_run_id is None and active_run_id is not None:
                        raise RuntimeError(
                            f"native controller retains active run {active_run_id}; exact-run recovery is required"
                        )
                    if recovery_run_id is not None and active_run_id != recovery_run_id:
                        raise RuntimeError(
                            "requested recovery run does not match retained native ownership"
                        )
                elif recovery_run_id is not None:
                    # POSIX hosts have no second durable registry. The caller still
                    # names the run so one cleanup path works on every host.
                    pass
                _held.workstation = self
                _held.lease = lease
                try:
                    yield lease
                finally:
                    del _held.lease
                    del _held.workstation
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            if descriptor is not None:
                os.close(descriptor)
            _process_lock.release()


__all__ = ["Workstation", "select_workstation"]
