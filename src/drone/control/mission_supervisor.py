"""Hardware-independent QGC command admission and mission state."""

from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
import fcntl
import json
import math
import os
from pathlib import Path
import stat
import threading
import tempfile
from typing import Callable, Iterator

from .. import timebase
from ..common_types import GPSCoord, MissionHome


class FlightOperationError(RuntimeError):
    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)


class MissionAbort(RuntimeError):
    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)


class AuthorityLost(RuntimeError):
    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)


FM1 = 31000
FM2 = 31001
FM3 = 31002
UPDATE_WA = 31003
UPDATE_WM1 = 31004
UPDATE_WM2 = 31005
UPDATE_WM3 = 31006
UPDATE_WM4 = 31007
UPDATE_WM5 = 31008
UPDATE_WM6 = 31009
UPDATE_L = 31010
UPDATE_TARGET = 31011
CLEAR_PICKUPS = 31012
CLEAR_ALL = 31013
RETIRED_COMMAND = 31014
ABORT_AND_RECOVER = 31015

MIN_ATTEMPT_ID = 1
MAX_ATTEMPT_ID = 16_777_215
LEDGER_SCHEMA_VERSION = 1

PHASE_COMMANDS = (FM1, FM2, FM3)
MUTATION_COMMANDS = frozenset(range(UPDATE_WA, CLEAR_ALL + 1))
SUPPORTED_COMMANDS = frozenset((*PHASE_COMMANDS, *MUTATION_COMMANDS, ABORT_AND_RECOVER))
TERMINAL_RESULTS = frozenset(("SUCCEEDED", "FAILED", "ABORTED"))
RECOVERY_OUTCOMES = frozenset(
    ("HOME_LANDED", "LOCAL_LANDED", "PILOT", "FC_FAILSAFE", "UNCONFIRMED")
)


@dataclass(frozen=True)
class RecoveryPolicy:
    check: Callable[[str, MissionHome, float | None], None]
    timeout_s: float
    local_land_reserve_s: float
    clock: Callable[[], float] = timebase.monotonic

    def __post_init__(self) -> None:
        if not callable(self.check):
            raise TypeError("recovery policy check must be callable")
        if not callable(self.clock):
            raise TypeError("recovery policy clock must be callable")
        if not _finite_number(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("recovery timeout must be finite and positive")
        if (
            not _finite_number(self.local_land_reserve_s)
            or self.local_land_reserve_s <= 0
            or self.local_land_reserve_s >= self.timeout_s
        ):
            raise ValueError(
                "local LAND reserve must be finite, positive, and below the timeout"
            )


def return_altitude_amsl(
    home: MissionHome, cruise_m: float, current_amsl_m: float
) -> float:
    if not isinstance(home, MissionHome) or not all(
        _finite_number(value)
        for value in (home.lat, home.lon, home.amsl_m, cruise_m, current_amsl_m)
    ):
        raise ValueError("home and recovery altitudes must be finite")
    if cruise_m < 0:
        raise ValueError("recovery cruise altitude cannot be negative")
    return max(float(home.amsl_m) + float(cruise_m), float(current_amsl_m))


def _finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


class _RecoveryStop(Exception):
    def __init__(self, outcome: str) -> None:
        self.outcome = outcome


@dataclass(frozen=True)
class CommandEnvelope:
    source_system: int
    source_component: int
    target_system: int
    target_component: int
    command: int
    attempt_id: int
    params: tuple[float, ...]
    received_at: float


class CommandRejected(RuntimeError):
    """An admission failure with the transport result it maps to."""

    def __init__(self, reason: str, result: str = "DENIED") -> None:
        self.reason = reason
        self.result = result
        super().__init__(reason)


class LedgerError(RuntimeError):
    pass


def _json_bytes(value: object) -> bytes:
    return (json.dumps(value, indent=4, sort_keys=True) + "\n").encode("utf-8")


def _fsync_directory(directory: Path) -> None:
    descriptor = os.open(directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        _fsync_directory(path.parent)
    finally:
        try:
            temporary_path.unlink()
        except FileNotFoundError:
            pass


@contextmanager
def _exclusive_ledger_lock(ledger_path: Path) -> Iterator[None]:
    lock_path = ledger_path.with_name(f"{ledger_path.name}.lock")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX)
        yield
    finally:
        fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)


def _validate_counter(value: object, field: str) -> int:
    if (
        not isinstance(value, int)
        or isinstance(value, bool)
        or not 0 <= value <= MAX_ATTEMPT_ID
    ):
        raise LedgerError(f"ledger field {field} is invalid")
    return value


def _read_ledger(path: Path) -> dict[str, int]:
    if not path.is_file():
        raise LedgerError(f"attempt ledger does not exist: {path}")
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise LedgerError(f"attempt ledger is corrupt or unreadable: {error}") from error
    if not isinstance(data, dict) or set(data) != {
        "schema_version",
        "last_prepared_attempt_id",
        "consumed_through_attempt_id",
    }:
        raise LedgerError("attempt ledger schema is corrupt")
    if data["schema_version"] != LEDGER_SCHEMA_VERSION:
        raise LedgerError("attempt ledger schema version is unsupported")
    prepared = _validate_counter(
        data["last_prepared_attempt_id"], "last_prepared_attempt_id"
    )
    consumed = _validate_counter(
        data["consumed_through_attempt_id"], "consumed_through_attempt_id"
    )
    if consumed > prepared:
        raise LedgerError("attempt ledger counters are inconsistent")
    return {
        "schema_version": LEDGER_SCHEMA_VERSION,
        "last_prepared_attempt_id": prepared,
        "consumed_through_attempt_id": consumed,
    }


def _read_ledger_descriptor(descriptor: int) -> dict[str, int]:
    try:
        metadata = os.fstat(descriptor)
        if metadata.st_size > 1_048_576:
            raise LedgerError("attempt ledger is too large")
        content = os.pread(descriptor, metadata.st_size + 1, 0)
        if len(content) != metadata.st_size:
            raise LedgerError("attempt ledger changed while being read")
        data = json.loads(content.decode("utf-8"))
    except LedgerError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise LedgerError(f"attempt ledger is corrupt or unreadable: {error}") from error
    if not isinstance(data, dict) or set(data) != {
        "schema_version",
        "last_prepared_attempt_id",
        "consumed_through_attempt_id",
    }:
        raise LedgerError("attempt ledger schema is corrupt")
    if type(data["schema_version"]) is not int or data["schema_version"] != LEDGER_SCHEMA_VERSION:
        raise LedgerError("attempt ledger schema version is unsupported")
    prepared = _validate_counter(data["last_prepared_attempt_id"], "last_prepared_attempt_id")
    consumed = _validate_counter(
        data["consumed_through_attempt_id"], "consumed_through_attempt_id"
    )
    if consumed > prepared:
        raise LedgerError("attempt ledger counters are inconsistent")
    return {
        "schema_version": LEDGER_SCHEMA_VERSION,
        "last_prepared_attempt_id": prepared,
        "consumed_through_attempt_id": consumed,
    }


class PinnedAttemptLedger:
    """Ledger capability pinned to exact existing directory, lock, and data inodes."""

    __slots__ = (
        "_path",
        "_directory_fd",
        "_ledger_fd",
        "_lock_fd",
        "_directory_identity",
        "_ledger_identity",
        "_lock_identity",
        "_mutex",
    )

    def __init__(self, path: os.PathLike[str] | str) -> None:
        raw = os.fspath(path)
        candidate = Path(raw)
        canonical = Path(os.path.abspath(raw))
        if raw.startswith("//") or not candidate.is_absolute() or candidate != canonical:
            raise LedgerError("pinned attempt ledger path must be absolute and canonical")
        directory_flags = (
            os.O_RDONLY
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_DIRECTORY", 0)
            | getattr(os, "O_NOFOLLOW", 0)
        )
        file_flags = (
            os.O_RDWR
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0)
            | getattr(os, "O_NONBLOCK", 0)
        )
        directory_fd = os.open("/", directory_flags)
        ledger_fd = -1
        lock_fd = -1
        try:
            for component in candidate.parent.parts[1:]:
                next_fd = os.open(component, directory_flags, dir_fd=directory_fd)
                predecessor_fd = directory_fd
                directory_fd = -1
                try:
                    os.close(predecessor_fd)
                except BaseException:
                    successor_fd = next_fd
                    next_fd = -1
                    os.close(successor_fd)
                    raise
                directory_fd = next_fd
            ledger_fd = os.open(candidate.name, file_flags, dir_fd=directory_fd)
            lock_name = f"{candidate.name}.lock"
            lock_fd = os.open(lock_name, file_flags, dir_fd=directory_fd)
            ledger_metadata = os.fstat(ledger_fd)
            lock_metadata = os.fstat(lock_fd)
            if not stat.S_ISREG(ledger_metadata.st_mode) or not stat.S_ISREG(
                lock_metadata.st_mode
            ):
                raise LedgerError("pinned ledger and lock must be regular files")
            directory_metadata = os.fstat(directory_fd)
            if not stat.S_ISDIR(directory_metadata.st_mode):
                raise LedgerError("pinned ledger parent must be a directory")
        except BaseException:
            descriptors = (lock_fd, ledger_fd, directory_fd)
            lock_fd = ledger_fd = directory_fd = -1
            for descriptor in descriptors:
                if descriptor >= 0:
                    try:
                        os.close(descriptor)
                    except OSError:
                        pass
            raise
        self._path = candidate
        self._directory_fd = directory_fd
        self._ledger_fd = ledger_fd
        self._lock_fd = lock_fd
        self._directory_identity = (
            directory_metadata.st_dev,
            directory_metadata.st_ino,
        )
        self._ledger_identity = (ledger_metadata.st_dev, ledger_metadata.st_ino)
        self._lock_identity = (lock_metadata.st_dev, lock_metadata.st_ino)
        self._mutex = threading.Lock()

    @property
    def path(self) -> Path:
        return self._path

    @property
    def ledger_identity(self) -> tuple[int, int]:
        return self._ledger_identity

    @property
    def lock_identity(self) -> tuple[int, int]:
        return self._lock_identity

    def close(self) -> None:
        with self._mutex:
            error: BaseException | None = None
            for name in ("_ledger_fd", "_lock_fd", "_directory_fd"):
                descriptor = getattr(self, name)
                if descriptor >= 0:
                    setattr(self, name, -1)
                    try:
                        os.close(descriptor)
                    except BaseException as caught:
                        if error is None:
                            error = caught
        if error is not None:
            raise error

    def _verify_named_identity(
        self, name: str, expected: tuple[int, int], *, label: str
    ) -> None:
        flags = (
            os.O_RDWR
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_NOFOLLOW", 0)
            | getattr(os, "O_NONBLOCK", 0)
        )
        try:
            descriptor = os.open(name, flags, dir_fd=self._directory_fd)
            try:
                metadata = os.fstat(descriptor)
                if not stat.S_ISREG(metadata.st_mode) or (
                    metadata.st_dev,
                    metadata.st_ino,
                ) != expected:
                    raise LedgerError(f"pinned attempt {label} identity changed")
            finally:
                os.close(descriptor)
        except LedgerError:
            raise
        except OSError as error:
            raise LedgerError(f"pinned attempt {label} is unavailable or unsafe") from error

    def _verify_canonical_parent(self) -> None:
        flags = (
            os.O_RDONLY
            | getattr(os, "O_CLOEXEC", 0)
            | getattr(os, "O_DIRECTORY", 0)
            | getattr(os, "O_NOFOLLOW", 0)
        )
        descriptor = os.open("/", flags)
        try:
            for component in self._path.parent.parts[1:]:
                next_fd = os.open(component, flags, dir_fd=descriptor)
                predecessor_fd = descriptor
                descriptor = -1
                try:
                    os.close(predecessor_fd)
                except BaseException:
                    successor_fd = next_fd
                    next_fd = -1
                    os.close(successor_fd)
                    raise
                descriptor = next_fd
            metadata = os.fstat(descriptor)
            if (metadata.st_dev, metadata.st_ino) != self._directory_identity:
                raise LedgerError("pinned attempt ledger directory identity changed")
        except LedgerError:
            raise
        except OSError as error:
            raise LedgerError("pinned attempt ledger directory is unavailable or unsafe") from error
        finally:
            if descriptor >= 0:
                closing_fd = descriptor
                descriptor = -1
                os.close(closing_fd)

    def _verify_retained_descriptors(self) -> None:
        for descriptor, expected, label, expected_kind in (
            (
                self._directory_fd,
                self._directory_identity,
                "ledger directory",
                stat.S_ISDIR,
            ),
            (self._ledger_fd, self._ledger_identity, "ledger", stat.S_ISREG),
            (self._lock_fd, self._lock_identity, "ledger lock", stat.S_ISREG),
        ):
            try:
                metadata = os.fstat(descriptor)
            except OSError as error:
                raise LedgerError(f"pinned attempt {label} is unavailable") from error
            if not expected_kind(metadata.st_mode) or (
                metadata.st_dev,
                metadata.st_ino,
            ) != expected:
                raise LedgerError(f"pinned attempt {label} descriptor identity changed")

    @contextmanager
    def _locked(self) -> Iterator[None]:
        with self._mutex:
            if min(self._directory_fd, self._ledger_fd, self._lock_fd) < 0:
                raise LedgerError("pinned attempt ledger is closed")
            self._verify_retained_descriptors()
            lock_fd = self._lock_fd
            fcntl.flock(lock_fd, fcntl.LOCK_EX)
            try:
                self._verify_retained_descriptors()
                self._verify_canonical_parent()
                self._verify_named_identity(
                    f"{self._path.name}.lock", self._lock_identity, label="ledger lock"
                )
                self._verify_named_identity(
                    self._path.name, self._ledger_identity, label="ledger"
                )
                yield
            finally:
                fcntl.flock(lock_fd, fcntl.LOCK_UN)

    def require_current_unconsumed(self, attempt_id: int) -> None:
        with self._locked():
            ledger = _read_ledger_descriptor(self._ledger_fd)
            if attempt_id != ledger["last_prepared_attempt_id"]:
                raise LedgerError("prepared session contains a stale attempt ID")
            if attempt_id <= ledger["consumed_through_attempt_id"]:
                raise LedgerError("prepared session contains a consumed attempt ID")

    def consume(self, attempt_id: int) -> None:
        if not _valid_attempt_id(attempt_id):
            raise LedgerError("attempt ID is invalid")
        with self._locked():
            ledger = _read_ledger_descriptor(self._ledger_fd)
            prepared = ledger["last_prepared_attempt_id"]
            consumed = ledger["consumed_through_attempt_id"]
            if attempt_id <= consumed:
                raise LedgerError(f"attempt {attempt_id} is already consumed")
            if attempt_id != prepared:
                raise LedgerError(
                    f"attempt {attempt_id} is not the current prepared attempt {prepared}"
                )
            ledger["consumed_through_attempt_id"] = attempt_id
            payload = _json_bytes(ledger)
            offset = 0
            while offset < len(payload):
                written = os.pwrite(self._ledger_fd, payload[offset:], offset)
                if written <= 0:
                    raise LedgerError("could not persist attempt consumption")
                offset += written
            os.ftruncate(self._ledger_fd, len(payload))
            os.fsync(self._ledger_fd)


def initialize_ledger(path: os.PathLike[str] | str) -> None:
    """Create the first ledger. Existing paths are never overwritten."""
    ledger_path = Path(path).resolve(strict=False)
    ledger_path.parent.mkdir(parents=True, exist_ok=True)
    payload = _json_bytes(
        {
            "schema_version": LEDGER_SCHEMA_VERSION,
            "last_prepared_attempt_id": 0,
            "consumed_through_attempt_id": 0,
        }
    )
    with _exclusive_ledger_lock(ledger_path):
        descriptor = os.open(ledger_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write(payload)
                stream.flush()
                os.fsync(stream.fileno())
            _fsync_directory(ledger_path.parent)
        except BaseException:
            try:
                ledger_path.unlink()
            except FileNotFoundError:
                pass
            raise


class AttemptLedger:
    """Monotonic preparation and one-time consumption under one file lock."""

    def __init__(self, path: os.PathLike[str] | str) -> None:
        self.path = Path(path).resolve(strict=False)

    @contextmanager
    def prepare_next(self) -> Iterator[int]:
        """Persist a new ID, then hold the lock while its files are written."""
        with _exclusive_ledger_lock(self.path):
            ledger = _read_ledger(self.path)
            previous = ledger["last_prepared_attempt_id"]
            if previous >= MAX_ATTEMPT_ID:
                raise LedgerError("attempt ledger is exhausted")
            attempt_id = previous + 1
            ledger["last_prepared_attempt_id"] = attempt_id
            try:
                _atomic_write(self.path, _json_bytes(ledger))
            except OSError as error:
                raise LedgerError(f"could not persist prepared attempt: {error}") from error
            yield attempt_id

    def consume(self, attempt_id: int) -> None:
        """Durably consume the current prepared ID, exactly once."""
        if not _valid_attempt_id(attempt_id):
            raise LedgerError("attempt ID is invalid")
        with _exclusive_ledger_lock(self.path):
            ledger = _read_ledger(self.path)
            prepared = ledger["last_prepared_attempt_id"]
            consumed = ledger["consumed_through_attempt_id"]
            if attempt_id <= consumed:
                raise LedgerError(f"attempt {attempt_id} is already consumed")
            if attempt_id != prepared:
                raise LedgerError(
                    f"attempt {attempt_id} is not the current prepared attempt {prepared}"
                )
            ledger["consumed_through_attempt_id"] = attempt_id
            try:
                _atomic_write(self.path, _json_bytes(ledger))
            except OSError as error:
                raise LedgerError(f"could not persist attempt consumption: {error}") from error

    def require_current_unconsumed(self, attempt_id: int) -> None:
        with _exclusive_ledger_lock(self.path):
            ledger = _read_ledger(self.path)
            if attempt_id != ledger["last_prepared_attempt_id"]:
                raise LedgerError("prepared session contains a stale attempt ID")
            if attempt_id <= ledger["consumed_through_attempt_id"]:
                raise LedgerError("prepared session contains a consumed attempt ID")


def _valid_attempt_id(value: object) -> bool:
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and MIN_ATTEMPT_ID <= value <= MAX_ATTEMPT_ID
    )


class MissionSupervisor:
    """Reserve commands and enforce one-way progress through one attempt.

    The injected checks must be short and synchronous. ``attempt_consumer`` is
    the durability boundary: it must return ``None`` only after the attempt ID
    has been durably marked consumed.
    """

    def __init__(
        self,
        attempt_id: int,
        *,
        admission_check: Callable[[CommandEnvelope], None],
        attempt_consumer: Callable[[int], None],
        permission_check: Callable[[], None],
        recovery_policy: RecoveryPolicy | None = None,
        enabled_phases: tuple[int, ...] = PHASE_COMMANDS,
    ) -> None:
        if not _valid_attempt_id(attempt_id):
            raise ValueError(
                f"attempt_id must be an integer from {MIN_ATTEMPT_ID} "
                f"through {MAX_ATTEMPT_ID}"
            )
        for name, dependency in (
            ("admission_check", admission_check),
            ("attempt_consumer", attempt_consumer),
            ("permission_check", permission_check),
        ):
            if not callable(dependency):
                raise TypeError(f"{name} must be callable")
        if not isinstance(enabled_phases, tuple):
            raise TypeError("enabled phases must be an immutable tuple")
        if enabled_phases not in (PHASE_COMMANDS[:2], PHASE_COMMANDS):
            raise ValueError("enabled phases must be the supported FM1/FM2 prefix")

        self.attempt_id = attempt_id
        self._enabled_phases = enabled_phases
        self._admission_check = admission_check
        self._attempt_consumer = attempt_consumer
        self._permission_check = permission_check
        self._recovery_policy = recovery_policy
        if recovery_policy is not None and not isinstance(
            recovery_policy, RecoveryPolicy
        ):
            raise TypeError("recovery_policy must be a RecoveryPolicy or None")
        self._lock = threading.RLock()
        self._output_gate = threading.RLock()
        self._output_thread_id: int | None = None
        self._output_abort_reason: str | None = None
        self._recovery_lock = threading.Lock()
        self._recovery_deadline: ContextVar[float | None] = ContextVar(
            f"mission_recovery_deadline_{id(self)}", default=None
        )
        self._states: dict[int, str] = {}
        self._results: dict[int, str] = {}
        self._envelopes: dict[int, CommandEnvelope] = {}
        self._immediate_results: dict[int, str] = {}
        self._expected_phase = enabled_phases[0]
        self._attempt_active = False
        self._attempt_terminal = False
        self._abort_reason: str | None = None
        self._terminal_result: str | None = None
        self._recovery_outcome: str | None = None

    def recover(self, controller, home: MissionHome, cruise_m: float) -> str:
        """Run at most one synchronous recovery path for this attempt."""
        with self._recovery_lock:
            with self._lock:
                if self._recovery_outcome is not None:
                    return self._recovery_outcome
            policy = self._recovery_policy
            if policy is None:
                return self._complete_recovery("UNCONFIRMED")

            try:
                started_at = policy.clock()
            except (TimeoutError, timebase.ClockError):
                self._complete_recovery("UNCONFIRMED")
                raise
            except Exception:
                return self._complete_recovery("UNCONFIRMED")
            except BaseException:
                self._complete_recovery("UNCONFIRMED")
                raise
            if not _finite_number(started_at):
                return self._complete_recovery("UNCONFIRMED")
            deadline = float(started_at) + policy.timeout_s
            if not _finite_number(deadline):
                return self._complete_recovery("UNCONFIRMED")
            token = self._recovery_deadline.set(deadline)
            try:
                try:
                    outcome = self._recover(controller, home, cruise_m, policy, deadline)
                except _RecoveryStop as stopped:
                    outcome = stopped.outcome
                except AuthorityLost:
                    outcome = self._authority_outcome(controller)
                except (TimeoutError, timebase.ClockError):
                    self._complete_recovery("UNCONFIRMED")
                    raise
                except Exception:
                    outcome = "UNCONFIRMED"
                return self._complete_recovery(outcome)
            except BaseException:
                self._complete_recovery("UNCONFIRMED")
                raise
            finally:
                self._recovery_deadline.reset(token)

    def _recover(
        self,
        controller,
        home: MissionHome,
        cruise_m: float,
        policy: RecoveryPolicy,
        deadline: float,
    ) -> str:
        snapshot = self._recovery_snapshot(controller)
        armed = self._fresh_value(snapshot, "armed")
        landed = self._fresh_value(snapshot, "landed_state")
        mode = self._fresh_value(snapshot, "mode")

        if self._is_ground(landed):
            return self._confirm_recovery_ground(
                controller, home, returned_home=False, deadline=deadline
            )
        if armed is not True:
            raise _RecoveryStop("UNCONFIRMED")
        if self._is_landing(landed) or str(mode).upper() == "LAND":
            return self._preserve_landing(
                controller, home, deadline, returned_home=False
            )
        if self._is_takeoff(landed):
            return self._local_land(controller, home, policy, deadline)
        if not self._is_in_air(landed):
            raise _RecoveryStop("UNCONFIRMED")

        pinned_home = getattr(controller, "mission_home", None)
        if not isinstance(pinned_home, MissionHome) or pinned_home != home:
            policy_home = pinned_home if isinstance(pinned_home, MissionHome) else home
            return self._local_land(controller, policy_home, policy, deadline)

        try:
            current_amsl_m = self._current_amsl(snapshot)
            transit_amsl_m = return_altitude_amsl(home, cruise_m, current_amsl_m)
            self._approve(policy, "RETURN", home, transit_amsl_m)
            interrupted = self._stage_interruption(
                controller, home, deadline, returned_home=False
            )
            if interrupted is not None:
                return interrupted
            if current_amsl_m < transit_amsl_m:
                climb_result = controller.climb(
                    transit_amsl_m - home.amsl_m,
                    timeout=self._remaining(policy, deadline, reserve_land=True),
                )
                if climb_result is not None:
                    raise FlightOperationError("climb must return None on success")
            interrupted = self._stage_interruption(
                controller, home, deadline, returned_home=False
            )
            if interrupted is not None:
                return interrupted
            self._require_zero(
                controller.goto_recovery_waypoint(
                    GPSCoord(home.lat, home.lon, transit_amsl_m - home.amsl_m),
                    approve_target_amsl=lambda raised_amsl: self._approve(
                        policy,
                        "RETURN",
                        home,
                        raised_amsl,
                    ),
                    timeout=self._remaining(policy, deadline, reserve_land=True),
                ),
                "return navigation",
            )
        except _RecoveryStop:
            raise
        except AuthorityLost:
            raise
        except (TimeoutError, timebase.ClockError):
            raise
        except Exception:
            return self._local_land(controller, home, policy, deadline)

        return self._land(controller, home, policy, deadline, returned_home=True)

    def _land(
        self,
        controller,
        home: MissionHome,
        policy: RecoveryPolicy,
        deadline: float,
        *,
        returned_home: bool,
    ) -> str:
        try:
            interrupted = self._stage_interruption(
                controller,
                home,
                deadline,
                returned_home=returned_home,
                landing_boundary=True,
            )
            if interrupted is not None:
                return interrupted
            self._approve(policy, "LOCAL_LAND", home, None)
            interrupted = self._stage_interruption(
                controller,
                home,
                deadline,
                returned_home=returned_home,
                landing_boundary=True,
            )
            if interrupted is not None:
                return interrupted
            remaining = self._remaining(policy, deadline)
            self._require_zero(
                controller.simple_land(
                    timeout=remaining,
                    mode_timeout=min(remaining, policy.local_land_reserve_s),
                ),
                "LAND",
            )
            return self._confirm_recovery_ground(
                controller, home, returned_home=returned_home, deadline=deadline
            )
        except _RecoveryStop:
            raise
        except AuthorityLost:
            raise
        except (TimeoutError, timebase.ClockError):
            raise
        except Exception:
            return "UNCONFIRMED"

    def _local_land(
        self,
        controller,
        home: MissionHome,
        policy: RecoveryPolicy,
        deadline: float,
    ) -> str:
        return self._land(
            controller, home, policy, deadline, returned_home=False
        )

    def _preserve_landing(
        self,
        controller,
        home: MissionHome,
        deadline: float,
        *,
        returned_home: bool,
    ) -> str:
        policy = self._recovery_policy
        if policy is None:
            return "UNCONFIRMED"
        try:
            self._recovery_snapshot(controller)
            self._require_zero(
                controller.confirm_landing(
                    timeout=self._remaining(policy, deadline)
                ),
                "landing confirmation",
            )
            return self._confirm_recovery_ground(
                controller, home, returned_home=returned_home, deadline=deadline
            )
        except _RecoveryStop:
            raise
        except AuthorityLost:
            raise
        except (TimeoutError, timebase.ClockError):
            raise
        except Exception:
            return "UNCONFIRMED"

    def _stage_interruption(
        self,
        controller,
        home: MissionHome,
        deadline: float,
        *,
        returned_home: bool,
        landing_boundary: bool = False,
    ) -> str | None:
        snapshot = self._recovery_snapshot(controller)
        armed = self._fresh_value(snapshot, "armed")
        landed = self._fresh_value(snapshot, "landed_state")
        mode = self._fresh_value(snapshot, "mode")
        if self._is_ground(landed):
            return self._confirm_recovery_ground(
                controller,
                home,
                returned_home=returned_home,
                deadline=deadline,
            )
        if self._is_landing(landed) or str(mode).upper() == "LAND":
            return self._preserve_landing(
                controller,
                home,
                deadline,
                returned_home=returned_home,
            )
        if self._is_takeoff(landed):
            if landing_boundary:
                return None
            policy = self._recovery_policy
            if policy is None:
                raise _RecoveryStop("UNCONFIRMED")
            return self._local_land(controller, home, policy, deadline)
        if armed is not True or not self._is_in_air(landed):
            raise _RecoveryStop("UNCONFIRMED")
        return None

    def _confirm_recovery_ground(
        self,
        controller,
        home: MissionHome,
        *,
        returned_home: bool,
        deadline: float,
    ) -> str:
        policy = self._recovery_policy
        if policy is None:
            return "UNCONFIRMED"
        snapshot = self._recovery_snapshot(controller)
        if not self._is_ground(self._fresh_value(snapshot, "landed_state")):
            return "UNCONFIRMED"
        armed = self._fresh_value(snapshot, "armed")
        if armed is True:
            self._recovery_snapshot(controller)
            self._require_zero(
                controller.disarm(timeout=self._remaining(policy, deadline)),
                "disarm",
            )
            snapshot = self._recovery_snapshot(controller)
            if (
                not self._is_ground(
                    self._fresh_value(snapshot, "landed_state")
                )
                or self._fresh_value(snapshot, "armed") is not False
            ):
                return "UNCONFIRMED"
        elif armed is not False:
            return "UNCONFIRMED"
        return "HOME_LANDED" if returned_home else "LOCAL_LANDED"

    @staticmethod
    def _fresh_value(snapshot, name: str):
        field = getattr(snapshot, name, None)
        if field is None or field.fresh is not True:
            raise FlightOperationError(f"recovery requires fresh {name} evidence")
        return field.observation.value

    @staticmethod
    def _is_ground(value: object) -> bool:
        return type(value) is int and value == 1

    @staticmethod
    def _is_in_air(value: object) -> bool:
        return type(value) is int and value == 2

    @staticmethod
    def _is_takeoff(value: object) -> bool:
        return type(value) is int and value == 3

    @staticmethod
    def _is_landing(value: object) -> bool:
        return type(value) is int and value == 4

    def _recovery_snapshot(self, controller):
        self.check_authority()
        snapshot = controller.flight_snapshot()
        authority = getattr(snapshot, "authority", None)
        authority_name = getattr(authority, "value", authority)
        if authority_name != "COMPANION" or getattr(
            snapshot, "commands_suspended", True
        ) is not False:
            raise _RecoveryStop(self._authority_outcome(controller, snapshot))
        return snapshot

    @staticmethod
    def _authority_outcome(controller, snapshot=None) -> str:
        try:
            current = controller.flight_snapshot() if snapshot is None else snapshot
            authority = getattr(current, "authority", None)
            authority_name = getattr(authority, "value", authority)
        except (TimeoutError, timebase.ClockError):
            raise
        except Exception:
            authority_name = None
        if authority_name == "PILOT":
            return "PILOT"
        if authority_name == "FC_FAILSAFE":
            return "FC_FAILSAFE"
        return "UNCONFIRMED"

    @classmethod
    def _current_amsl(cls, snapshot) -> float:
        location = cls._fresh_value(snapshot, "location")
        if (
            not isinstance(location, tuple)
            or len(location) != 4
            or not _finite_number(location[2])
        ):
            raise FlightOperationError("recovery location is invalid")
        return float(location[2]) / 1000.0

    @staticmethod
    def _approve(
        policy: RecoveryPolicy,
        operation: str,
        home: MissionHome,
        altitude: float | None,
    ) -> None:
        result = policy.check(operation, home, altitude)
        if result is not None:
            raise FlightOperationError("recovery policy check must return None")

    @staticmethod
    def _require_zero(result: object, operation: str) -> None:
        if type(result) is not int or result != 0:
            raise FlightOperationError(f"{operation} did not confirm success")

    @staticmethod
    def _remaining(
        policy: RecoveryPolicy, deadline: float, *, reserve_land: bool = False
    ) -> float:
        remaining = deadline - policy.clock()
        if reserve_land:
            remaining -= policy.local_land_reserve_s
        if not _finite_number(remaining) or remaining <= 0:
            raise FlightOperationError("recovery deadline expired")
        return remaining

    def _complete_recovery(self, outcome: str) -> str:
        with self._lock:
            if self._recovery_outcome is None:
                self._recovery_outcome = outcome
            return self._recovery_outcome

    @property
    def enabled_phases(self) -> tuple[int, ...]:
        return self._enabled_phases

    @property
    def abort_reason(self) -> str | None:
        with self._lock:
            self._apply_output_abort_locked()
            return self._abort_reason

    @property
    def terminal_result(self) -> str | None:
        with self._lock:
            self._apply_output_abort_locked()
            return self._terminal_result

    @property
    def recovery_outcome(self) -> str | None:
        with self._lock:
            return self._recovery_outcome

    def status(self, command: int) -> str | None:
        with self._lock:
            return self._states.get(command)

    def admission_result(self, command: int) -> str | None:
        with self._lock:
            immediate = self._immediate_results.get(command)
            if immediate is not None:
                return immediate
            state = self._states.get(command)
            if state in ("QUEUED", "RUNNING"):
                return "IN_PROGRESS"
            if state == "TERMINAL":
                return self._results[command]
            return None

    def reserved_envelope(self, command: int) -> CommandEnvelope | None:
        with self._lock:
            return self._envelopes.get(command)

    def admit(self, envelope: CommandEnvelope) -> bool:
        """Atomically validate and reserve one command for enqueueing."""
        if envelope.command == ABORT_AND_RECOVER:
            self._validate_envelope(envelope)
            admission_result = self._admission_check(envelope)
            if admission_result is not None:
                raise CommandRejected("admission_check must return None")
            with self._lock:
                self._apply_output_abort_locked()
                existing = self._immediate_results.get(envelope.command)
                if existing is not None:
                    return False
                if not self._attempt_active or self._attempt_terminal:
                    raise CommandRejected("no active attempt to abort")
                self._immediate_results[envelope.command] = "IN_PROGRESS"
                self._envelopes[envelope.command] = envelope
            try:
                with self._output_gate:
                    if self._output_abort_reason is None:
                        self._output_abort_reason = "QGC abort command"
            except BaseException:
                with self._lock:
                    if self._immediate_results.get(envelope.command) == "IN_PROGRESS":
                        self._immediate_results.pop(envelope.command)
                        self._envelopes.pop(envelope.command, None)
                raise
            with self._lock:
                self._apply_output_abort_locked()
                self._immediate_results[envelope.command] = "ACCEPTED"
            return False
        with self._lock:
            self._validate_envelope(envelope)

            admission_result = self._admission_check(envelope)
            if admission_result is not None:
                raise CommandRejected("admission_check must return None")

            existing = self._states.get(envelope.command)
            if existing is not None:
                return False

            if self._attempt_terminal:
                raise CommandRejected("attempt is terminal")
            if envelope.command in MUTATION_COMMANDS and self._attempt_active:
                raise CommandRejected("coordinate mutation denied during active attempt")
            if envelope.command in PHASE_COMMANDS:
                if envelope.command not in self._enabled_phases:
                    raise CommandRejected(f"phase command {envelope.command} is disabled")
                if envelope.command != self._expected_phase:
                    raise CommandRejected(
                        f"expected command {self._expected_phase}, got {envelope.command}",
                        "TEMPORARILY_REJECTED",
                    )
                if any(
                    self._states.get(command) in ("QUEUED", "RUNNING")
                    for command in PHASE_COMMANDS
                ):
                    raise CommandRejected("another phase is busy", "TEMPORARILY_REJECTED")
                if envelope.command == FM1 and any(
                    state in ("QUEUED", "RUNNING") for state in self._states.values()
                ):
                    raise CommandRejected(
                        "another command is busy", "TEMPORARILY_REJECTED"
                    )

            if envelope.command == FM1:
                try:
                    consumed = self._attempt_consumer(self.attempt_id)
                except Exception as error:
                    raise CommandRejected(
                        f"attempt ledger consumption failed: {error}"
                    ) from error
                if consumed is not None:
                    raise CommandRejected("attempt_consumer must return None")
                self._attempt_active = True

            self._states[envelope.command] = "QUEUED"
            self._envelopes[envelope.command] = envelope
            return True

    def begin(self, command: int) -> None:
        with self._lock:
            if self._states.get(command) != "QUEUED":
                raise CommandRejected(f"command {command} is not queued")
            self._states[command] = "RUNNING"

    def finish(self, command: int, result: str) -> None:
        if result not in TERMINAL_RESULTS:
            raise ValueError(f"invalid terminal result: {result}")
        with self._lock:
            self._apply_output_abort_locked()
            if self._states.get(command) != "RUNNING":
                raise CommandRejected(f"command {command} is not running")
            if command in PHASE_COMMANDS and self._abort_reason is not None:
                result = "ABORTED"
            self._states[command] = "TERMINAL"
            self._results[command] = result

            if command not in PHASE_COMMANDS:
                return
            if self._terminal_result is not None:
                return
            if result != "SUCCEEDED":
                self._attempt_terminal = True
                self._terminal_result = result
                return
            phase_index = self._enabled_phases.index(command)
            if phase_index == len(self._enabled_phases) - 1:
                self._attempt_terminal = True
                self._terminal_result = result
            else:
                self._expected_phase = self._enabled_phases[phase_index + 1]

    def request_abort(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason:
            raise ValueError("abort reason must be a non-empty string")
        interrupt_active_output = False
        with self._output_gate:
            if self._output_abort_reason is None:
                self._output_abort_reason = reason
            interrupt_active_output = self._output_thread_id == threading.get_ident()
        if interrupt_active_output and self._recovery_deadline.get() is None:
            raise MissionAbort(self._output_abort_reason)
        if self._lock.acquire(blocking=False):
            try:
                self._apply_output_abort_locked()
            finally:
                self._lock.release()

    def _latch_abort_locked(self, reason: str) -> None:
        if self._output_abort_reason is None:
            self._output_abort_reason = reason
        self._apply_output_abort_locked()

    def _apply_output_abort_locked(self) -> None:
        reason = self._output_abort_reason
        if reason is None:
            return
        if self._attempt_terminal:
            return
        if self._abort_reason is None:
            self._abort_reason = reason
            self._attempt_terminal = True
            self._terminal_result = "ABORTED"
            for command in PHASE_COMMANDS:
                if self._states.get(command) == "QUEUED":
                    self._states[command] = "TERMINAL"
                    self._results[command] = "ABORTED"

    def check_authority(self) -> None:
        """Run the authority/freshness guard without mission abort handling.

        Task 6 recovery can use this narrower check while installing its own
        recovery deadline. Normal controller callers use ``check_permission``.
        """
        permission_result = self._permission_check()
        if permission_result is not None:
            raise AuthorityLost("permission_check must return None")

    def check_permission(self) -> None:
        self.check_authority()
        recovery_deadline = self._recovery_deadline.get()
        if recovery_deadline is not None:
            policy = self._recovery_policy
            now = None if policy is None else policy.clock()
            if (
                policy is None
                or not _finite_number(now)
                or float(now) >= recovery_deadline
            ):
                raise FlightOperationError("recovery deadline expired")
            return
        with self._lock:
            self._apply_output_abort_locked()
            abort_reason = self._abort_reason
        if abort_reason is not None:
            raise MissionAbort(abort_reason)

    def output_transaction(self, operation: Callable[[], object]) -> object:
        """Serialize one actual output with abort and recovery deadline state."""
        if not callable(operation):
            raise TypeError("operation must be callable")
        with self._output_gate:
            previous_output_thread_id = self._output_thread_id
            self._output_thread_id = threading.get_ident()
            try:
                recovery_deadline = self._recovery_deadline.get()
                if self._output_abort_reason is not None and recovery_deadline is None:
                    raise MissionAbort(self._output_abort_reason)
                if recovery_deadline is not None:
                    policy = self._recovery_policy
                    now = None if policy is None else policy.clock()
                    if (
                        policy is None
                        or not _finite_number(now)
                        or float(now) >= recovery_deadline
                    ):
                        raise FlightOperationError("recovery deadline expired")
                return operation()
            finally:
                self._output_thread_id = previous_output_thread_id

    def record_recovery_outcome(self, outcome: str) -> None:
        if outcome not in RECOVERY_OUTCOMES:
            raise ValueError(f"invalid recovery outcome: {outcome}")
        with self._lock:
            if self._recovery_outcome is not None:
                raise ValueError("recovery outcome is already recorded")
            self._recovery_outcome = outcome

    def _validate_envelope(self, envelope: CommandEnvelope) -> None:
        if not isinstance(envelope, CommandEnvelope):
            raise TypeError("envelope must be a CommandEnvelope")
        if envelope.command == RETIRED_COMMAND or envelope.command not in SUPPORTED_COMMANDS:
            raise CommandRejected(
                f"unsupported command {envelope.command}", "UNSUPPORTED"
            )
        if envelope.attempt_id != self.attempt_id or not _valid_attempt_id(
            envelope.attempt_id
        ):
            raise CommandRejected("attempt ID does not match the active session")
        if not isinstance(envelope.params, tuple) or len(envelope.params) != 7 or not all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            for value in envelope.params
        ):
            raise CommandRejected("command parameters must be seven finite numbers")
        token = envelope.params[0]
        if not float(token).is_integer() or int(token) != self.attempt_id:
            raise CommandRejected("param1 must contain the exact attempt ID")
        if not isinstance(envelope.received_at, (int, float)) or isinstance(
            envelope.received_at, bool
        ) or not math.isfinite(envelope.received_at):
            raise CommandRejected("received_at must be finite")
