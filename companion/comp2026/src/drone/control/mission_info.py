import json
import math
import os
import tempfile
import threading
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from pathlib import Path

from .. import timebase as time
from ..common_types import GPSCoord

"""
mission_info.py - This file allows tracking of time within a mission, storing waypoints (id, coord), and storing payload id's
"""

__author__ = "Vivian Chuang"

WAYPOINTS_PATH = Path(__file__).resolve().parent / "mission_data" / "waypoints.json"
MAX_WAYPOINT_AGE_SECONDS = 4 * 60 * 60


class WaypointStoreError(ValueError):
    """The waypoint store cannot safely provide or accept waypoint data."""


@dataclass(frozen=True)
class WaypointRecord:
    lat: float
    long: float
    alt: float
    loaded_at: float

    @property
    def coords(self) -> GPSCoord:
        return GPSCoord(self.lat, self.long, self.alt)


@dataclass(frozen=True)
class WaypointSnapshot:
    """Validated attempt data stored without mutable GPSCoord references."""

    _records: tuple[tuple[str, WaypointRecord], ...]

    @property
    def names(self) -> frozenset[str]:
        return frozenset(name for name, _record in self._records)

    def get_record(self, name: str) -> WaypointRecord | None:
        for record_name, record in self._records:
            if record_name == name:
                return record
        return None

    def get_waypoint(self, name: str) -> GPSCoord | None:
        record = self.get_record(name)
        return None if record is None else record.coords

    def as_coords(self) -> dict[str, GPSCoord]:
        return {name: record.coords for name, record in self._records}


def _finite_number(value, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise WaypointStoreError(f"{field} must be a non-boolean number")
    try:
        number = float(value)
    except OverflowError as error:
        raise WaypointStoreError(
            f"{field} is outside the supported numeric range"
        ) from error
    if not math.isfinite(number):
        raise WaypointStoreError(f"{field} must be finite")
    return number


def _validate_record(name: str, entry, now: float) -> WaypointRecord:
    if not isinstance(name, str) or not name:
        raise WaypointStoreError("waypoint names must be non-empty strings")
    if not isinstance(entry, dict) or set(entry) != {"coords", "loaded_at"}:
        raise WaypointStoreError(f"waypoint {name!r} has invalid record schema")
    coords = entry["coords"]
    if not isinstance(coords, dict) or set(coords) != {"lat", "long", "alt"}:
        raise WaypointStoreError(f"waypoint {name!r} has invalid coordinate schema")

    lat = _finite_number(coords["lat"], f"waypoint {name!r} latitude")
    longitude = _finite_number(coords["long"], f"waypoint {name!r} longitude")
    altitude = _finite_number(coords["alt"], f"waypoint {name!r} altitude")
    if not -90.0 <= lat <= 90.0:
        raise WaypointStoreError(f"waypoint {name!r} latitude is outside [-90, 90]")
    if not -180.0 <= longitude <= 180.0:
        raise WaypointStoreError(
            f"waypoint {name!r} longitude is outside [-180, 180]"
        )

    loaded_at = _finite_number(entry["loaded_at"], f"waypoint {name!r} loaded_at")
    if loaded_at <= 0.0:
        raise WaypointStoreError(f"waypoint {name!r} loaded_at must be positive")
    if loaded_at > now:
        raise WaypointStoreError(
            f"waypoint {name!r} loaded_at is in the future ({loaded_at} > {now})"
        )
    return WaypointRecord(lat, longitude, altitude, loaded_at)


def _load_valid_store(path: Path, now: float) -> dict[str, WaypointRecord]:
    try:
        with path.open("r") as waypoint_file:
            raw = json.load(waypoint_file)
    except FileNotFoundError as error:
        raise WaypointStoreError(f"waypoint store does not exist: {path}") from error
    except (json.JSONDecodeError, UnicodeError) as error:
        raise WaypointStoreError(
            f"waypoint store contains malformed JSON or text: {error}"
        ) from error

    if not isinstance(raw, dict):
        raise WaypointStoreError("waypoint store root schema must be an object")
    return {name: _validate_record(name, entry, now) for name, entry in raw.items()}


def _serialize_records(records: dict[str, WaypointRecord]) -> dict:
    return {
        name: {
            "coords": {"lat": record.lat, "long": record.long, "alt": record.alt},
            "loaded_at": record.loaded_at,
        }
        for name, record in records.items()
    }


def _atomic_write(
    path: Path,
    records: dict[str, WaypointRecord],
    *,
    refuse_existing: bool = False,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
            json.dump(_serialize_records(records), temporary, allow_nan=False)
            temporary.flush()
            os.fsync(temporary.fileno())
        if refuse_existing:
            try:
                os.link(temporary_path, path)
            except FileExistsError as error:
                raise WaypointStoreError(
                    f"waypoint store already exists: {path}"
                ) from error
            temporary_path.unlink()
        else:
            os.replace(temporary_path, path)
        temporary_path = None

        directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        directory_fd = os.open(path.parent, directory_flags)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass


class MissonTracker:
    def __init__(self, mission_time_seconds=600, *, waypoint_path=None):
        self.mission_begin = None
        self.timer_start = None
        self.mission_time_seconds = mission_time_seconds
        selected_path = WAYPOINTS_PATH if waypoint_path is None else waypoint_path
        self._waypoint_path = Path(os.path.abspath(os.fspath(selected_path)))
        self.waypoint_unavailable_reason: str | None = None
        self._waypoint_lock = threading.RLock()
        # record of past passes using keyword: list of timed passes
        self.timed_passes = {}

    @property
    def waypoint_path(self) -> Path:
        return self._waypoint_path

    # set a waypoint by giving the waypoint id and coordinate
    def initialize_waypoint_store(self) -> None:
        """Explicitly create a missing store for operator waypoint recording."""
        with self._waypoint_lock:
            _atomic_write(self._waypoint_path, {}, refuse_existing=True)

    def set_waypoint(self, waypoint_ID: str, coords: GPSCoord):
        with self._waypoint_lock:
            now = time.epoch()
            candidate = {
                "coords": {
                    "lat": getattr(coords, "lat", None),
                    "long": getattr(coords, "long", None),
                    "alt": getattr(coords, "alt", None),
                },
                "loaded_at": now,
            }
            record = _validate_record(waypoint_ID, candidate, now)
            records = _load_valid_store(self._waypoint_path, now)
            records[waypoint_ID] = record
            _atomic_write(self._waypoint_path, records)

    # clear a waypoint by its id
    def clear_waypoint(self, waypoint_ID: str) -> bool:
        with self._waypoint_lock:
            records = _load_valid_store(self._waypoint_path, time.epoch())
            if waypoint_ID not in records:
                return False
            del records[waypoint_ID]
            _atomic_write(self._waypoint_path, records)
            return True

    # get a waypoint's coordinates from its' id
    def get_waypoint(
        self, waypoint_ID: str, max_age_seconds: float | None = None
    ) -> GPSCoord | None:
        record = self.get_waypoint_record(waypoint_ID, max_age_seconds)
        return None if record is None else record.coords

    def get_waypoint_record(
        self, waypoint_ID: str, max_age_seconds: float | None = None
    ) -> WaypointRecord | None:
        with self._waypoint_lock:
            now = time.epoch()
            try:
                records = _load_valid_store(self._waypoint_path, now)
                if waypoint_ID not in records:
                    raise WaypointStoreError(f"waypoint {waypoint_ID!r} is missing")
                record = records[waypoint_ID]
                if max_age_seconds is not None:
                    maximum = _finite_number(max_age_seconds, "max_age_seconds")
                    if maximum < 0.0:
                        raise WaypointStoreError("max_age_seconds cannot be negative")
                    age = now - record.loaded_at
                    if age > maximum:
                        raise WaypointStoreError(
                            f"waypoint {waypoint_ID!r} age {age:.1f}s exceeds {maximum:.1f}s"
                        )
            except (OSError, WaypointStoreError) as error:
                self.waypoint_unavailable_reason = str(error)
                return None
            self.waypoint_unavailable_reason = None
            return record

    def snapshot_for_attempt(
        self,
        required_names: Iterable[str],
        operating_area: Callable[[str, GPSCoord], bool],
        *,
        optional_names: Iterable[str] = (),
    ) -> WaypointSnapshot:
        """Freeze fresh, site-approved records read from one coherent store."""
        with self._waypoint_lock:
            now = time.epoch()
            records = _load_valid_store(self._waypoint_path, now)
            required = frozenset(required_names)
            optional = frozenset(optional_names)
            missing = sorted(required - records.keys())
            if missing:
                raise WaypointStoreError(f"required waypoint {missing[0]!r} is missing")

            included_names = required | (optional & records.keys())
            included = []
            for name in sorted(included_names):
                record = records[name]
                age = now - record.loaded_at
                if age > MAX_WAYPOINT_AGE_SECONDS:
                    raise WaypointStoreError(
                        f"waypoint {name!r} is older than {MAX_WAYPOINT_AGE_SECONDS}s"
                    )
                if not operating_area(name, record.coords):
                    raise WaypointStoreError(
                        f"waypoint {name!r} is outside the approved operating area"
                    )
                included.append((name, record))
            return WaypointSnapshot(tuple(included))

    """ # add a payload id
    def add_payload(self, id: int):
        with open("mission_data/payloads.json", "r+") as payloads_write:
            payloads_data = json.load(payloads_write)
            if not payloads_data["id"]:
                payloads_data["id"] = []

            payloads_data["id"].append(id)
            json.dump(payloads_data, payloads_write)

    # remove a payload given its' id
    def remove_payload(self, id: int) -> bool:
        with open("mission_data/payloads.json", "r+") as payloads_write:
            payloads_data = json.load(payloads_write)
            if not payloads_data["id"]:
                return False

            payloads_data["id"].remove(id)
            json.dump(payloads_data, payloads_write)

            return True

    # returns all payload ids
    def get_payload_id(self) -> list[int]:
        with open("mission_data/payloads.json", "r") as payloads_read:
            payloads_data = json.load(payloads_read)
            return payloads_data["id"] """

    def begin_mission(self):
        self.mission_begin = time.monotonic()
        return

    def begin_aux_timer(self):
        self.timer_start = time.monotonic()
        return

    def end_aux_timer(self) -> float:
        if self.timer_start is not None:
            return time.monotonic() - self.timer_start
        return -1.0

    def time_left(self) -> float:
        if self.mission_begin is not None:
            elapsed = time.monotonic() - self.mission_begin
            return self.mission_time_seconds - elapsed
        return -1.0

    def end_mission(self) -> float:
        return 0.0

    # records and ends the timer
    def record_aux_timer(self, record) -> list:
        timer_length = self.end_aux_timer()

        if timer_length != -1:

            # record the pass in self.timed_passes if record
            # if keyword already recorded then adds to the keyword's list
            if record and record in self.timed_passes:
                self.timed_passes[record].append(timer_length)

            # otherwise creates a new key word with the list[0] being the recorded time
            elif record:
                self.timed_passes[record] = [timer_length]

            return self.timed_passes[record]

        return []

    # returns the length of the last pass for the given record
    def get_last(self, record: str) -> float:
        if record in self.timed_passes:
            return self.timed_passes[record][-1]

        return -1.0

    # returns the length of the average pass for the given record
    def get_avg(self, record: str) -> float:
        if record in self.timed_passes:
            return sum(self.timed_passes[record]) / len(self.timed_passes[record])

        return -1.0

    # returns the length of the worst pass for the given record
    def get_worst(self, record: str) -> float:
        if record in self.timed_passes:
            return max(self.timed_passes[record])

        return -1.0

    # returns the length of the best pass for the given record
    def get_best(self, record: str) -> float:
        if record in self.timed_passes:
            return min(self.timed_passes[record])

        return -1.0


if __name__ == "__main__":
    mt = MissonTracker()
    mt.begin_mission()
    mt.begin_aux_timer()
    time.sleep(5.0)
    print("timer went for:", mt.record_aux_timer("LF1"))
    print("we have", mt.time_left(), "seconds left in mission")
    mt.begin_mission()
    mt.begin_aux_timer()
    time.sleep(5.5)
    print("timer went for:", mt.record_aux_timer("LF1"))
    print("we have", mt.time_left(), "seconds left in mission")
    print("the worst case time:", mt.get_worst("LF1"))
    print("the best case time:", mt.get_best("LF1"))
    print("the average case time:", mt.get_avg("LF1"))
    print("the last case time:", mt.get_last("LF1"))
