"""Thin simulation/ROS adapters for the original Comp2026 mission."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import ClassVar, Protocol

import numpy as np


EARTH_RADIUS_M = 6_378_137.0
MAX_RANGE_AGE_NS = 500_000_000
DRONEKIT_HEARTBEAT_TIMEOUT_SECONDS = 60.0


class StaleSensorError(RuntimeError):
    """A blocking mission sensor cannot provide a current simulation sample."""


@dataclass
class GPSCoord:
    """Shape-compatible coordinate used by the original nested mission."""

    lat: float
    long: float
    alt: float


def world_xy_to_gps(home: GPSCoord, *, x_m: float, y_m: float) -> GPSCoord:
    """Map world ENU x=east/y=north to WGS84 using the approved sphere."""

    latitude_radians = math.radians(home.lat)
    latitude = home.lat + math.degrees(y_m / EARTH_RADIUS_M)
    longitude = home.long + math.degrees(
        x_m / (EARTH_RADIUS_M * math.cos(latitude_radians))
    )
    return GPSCoord(latitude, longitude, home.alt)


def horizontal_distance_m(first: GPSCoord, second: GPSCoord) -> float:
    """Use the same small-distance WGS84 approximation as the original code."""

    first_lat = math.radians(first.lat)
    second_lat = math.radians(second.lat)
    north = second_lat - first_lat
    east = math.radians(second.long - first.long) * math.cos(
        (first_lat + second_lat) / 2.0
    )
    return EARTH_RADIUS_M * math.hypot(east, north)


def _stamp_ns(stamp: object) -> int:
    try:
        timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)  # type: ignore[attr-defined]
    except (AttributeError, TypeError, ValueError) as error:
        raise ValueError("message timestamp is malformed") from error
    if timestamp_ns < 0:
        raise ValueError("message timestamp must be nonnegative")
    return timestamp_ns


class SimulationClock:
    """Condition-backed view of the existing authoritative simulation clock."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._timestamp_ns: int | None = None
        self._stop_reason: str | None = None

    @property
    def timestamp_ns(self) -> int | None:
        with self._condition:
            return self._timestamp_ns

    @property
    def ready(self) -> bool:
        return self.timestamp_ns is not None

    def accept(self, timestamp_ns: int) -> None:
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int):
            raise TypeError("simulation timestamp must be an integer")
        if timestamp_ns < 0:
            raise ValueError("simulation timestamp must be nonnegative")
        with self._condition:
            if self._timestamp_ns is not None and timestamp_ns < self._timestamp_ns:
                raise ValueError("authoritative simulation clock regressed")
            self._timestamp_ns = timestamp_ns
            self._condition.notify_all()

    def now(self) -> float:
        timestamp_ns = self.timestamp_ns
        return 0.0 if timestamp_ns is None else timestamp_ns / 1_000_000_000

    def sleep(self, seconds: float) -> None:
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
            raise TypeError("simulation sleep duration must be numeric")
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("simulation sleep duration must be finite and nonnegative")
        with self._condition:
            while self._timestamp_ns is None and self._stop_reason is None:
                self._condition.wait()
            self._raise_if_stopped()
            assert self._timestamp_ns is not None
            saved_start_ns = self._timestamp_ns
            while (
                (self._timestamp_ns - saved_start_ns) / 1_000_000_000 < seconds
                and self._stop_reason is None
            ):
                self._condition.wait()
            self._raise_if_stopped()

    def stop(self, reason: str) -> None:
        with self._condition:
            if self._stop_reason is None:
                self._stop_reason = reason or "shutdown"
            self._condition.notify_all()

    def _raise_if_stopped(self) -> None:
        if self._stop_reason is not None:
            raise RuntimeError(f"simulation clock stopped: {self._stop_reason}")


@dataclass(frozen=True)
class _FrameMetadata:
    frame_id: int
    timestamp_ns: int


class RosFrameSource:
    """Join exact current-run image/metadata pairs for blocking BGR capture."""

    def __init__(self, *, width_px: int, height_px: int, run_id: str | None = None) -> None:
        self._width_px = width_px
        self._height_px = height_px
        self._run_id = run_id
        self._condition = threading.Condition()
        self._images: dict[int, object] = {}
        self._metadata: dict[int, _FrameMetadata] = {}
        self._latest_metadata_id: int | None = None
        self._latest_metadata_timestamp_ns: int | None = None
        self._last_frame_id: int | None = None
        self.last_timestamp_ns: int | None = None
        self._stop_reason: str | None = None

    @property
    def ready(self) -> bool:
        with self._condition:
            return bool(self._available_pairs())

    def accept(self, metadata: object, image: object) -> None:
        self.accept_metadata(metadata)
        self.accept_image(image)

    def accept_image(self, message: object) -> None:
        timestamp_ns = _stamp_ns(message.header.stamp)  # type: ignore[attr-defined]
        if getattr(message.header, "frame_id", None) != "camera/onboard":  # type: ignore[attr-defined]
            raise ValueError("image must belong to the onboard camera stream")
        if (
            getattr(message, "width", None) != self._width_px
            or getattr(message, "height", None) != self._height_px
            or getattr(message, "encoding", None) != "rgb8"
            or getattr(message, "is_bigendian", None) not in (False, 0)
            or getattr(message, "step", None) != self._width_px * 3
        ):
            raise ValueError("image does not match the resolved 640x480 RGB8 contract")
        try:
            data = bytes(message.data)  # type: ignore[attr-defined]
        except (AttributeError, TypeError, ValueError) as error:
            raise ValueError("image data is malformed") from error
        if len(data) != self._width_px * self._height_px * 3:
            raise ValueError("image data length does not match its geometry")
        with self._condition:
            if self.last_timestamp_ns is None or timestamp_ns > self.last_timestamp_ns:
                self._images.setdefault(timestamp_ns, (message, data))
                self._prune_buffers()
            self._condition.notify_all()

    def accept_metadata(self, message: object) -> None:
        if self._run_id is not None and getattr(message, "run_id", None) != self._run_id:
            return
        if getattr(message, "stream", None) != "onboard":
            return
        timestamp_ns = _stamp_ns(message.sim_timestamp)  # type: ignore[attr-defined]
        frame_id = getattr(message, "frame_id", None)
        if isinstance(frame_id, bool) or not isinstance(frame_id, int) or frame_id < 0:
            raise ValueError("frame metadata ID must be a nonnegative integer")
        with self._condition:
            if self._latest_metadata_id is not None:
                if frame_id < self._latest_metadata_id:
                    raise ValueError("frame metadata ID regressed")
                if (
                    frame_id == self._latest_metadata_id
                    and timestamp_ns != self._latest_metadata_timestamp_ns
                ):
                    raise ValueError("frame ID was reused with a different timestamp")
                if (
                    frame_id > self._latest_metadata_id
                    and self._latest_metadata_timestamp_ns is not None
                    and timestamp_ns <= self._latest_metadata_timestamp_ns
                ):
                    raise ValueError("frame metadata timestamp did not advance")
            self._latest_metadata_id = frame_id
            self._latest_metadata_timestamp_ns = timestamp_ns
            if self.last_timestamp_ns is None or timestamp_ns > self.last_timestamp_ns:
                self._metadata.setdefault(
                    timestamp_ns, _FrameMetadata(frame_id, timestamp_ns)
                )
                self._prune_buffers()
            self._condition.notify_all()

    def capture_frame(
        self, quality: int = 4, deadline_sim_ns: int | None = None
    ) -> np.ndarray:
        del quality  # The original API's quality hint must not rescale public evidence.
        with self._condition:
            while True:
                pairs = self._available_pairs()
                if pairs:
                    timestamp_ns = pairs[0]
                    image, data = self._images.pop(timestamp_ns)
                    metadata = self._metadata.pop(timestamp_ns)
                    self._discard_through(timestamp_ns)
                    self.last_timestamp_ns = timestamp_ns
                    self._last_frame_id = metadata.frame_id
                    return (
                        np.frombuffer(data, dtype=np.uint8)
                        .reshape(self._height_px, self._width_px, 3)[..., ::-1]
                        .copy()
                    )
                if deadline_sim_ns is not None:
                    raise StaleSensorError(
                        f"no newer frame after {self.last_timestamp_ns} before "
                        f"simulation deadline {deadline_sim_ns}"
                    )
                if self._stop_reason is not None:
                    raise StaleSensorError(
                        f"frame source stopped before a newer frame: {self._stop_reason}"
                    )
                self._condition.wait()

    def stop(self, reason: str) -> None:
        with self._condition:
            self._stop_reason = reason or "shutdown"
            self._condition.notify_all()

    def _available_pairs(self) -> list[int]:
        timestamps = self._images.keys() & self._metadata.keys()
        return sorted(
            timestamp
            for timestamp in timestamps
            if (self.last_timestamp_ns is None or timestamp > self.last_timestamp_ns)
            and (
                self._last_frame_id is None
                or self._metadata[timestamp].frame_id > self._last_frame_id
            )
        )

    def _discard_through(self, timestamp_ns: int) -> None:
        for values in (self._images, self._metadata):
            for stale in tuple(key for key in values if key <= timestamp_ns):
                values.pop(stale, None)

    def _prune_buffers(self) -> None:
        """Keep only a small newest pairing window, never a recording queue."""

        timestamps = sorted(self._images.keys() | self._metadata.keys())
        for stale in timestamps[:-4]:
            self._images.pop(stale, None)
            self._metadata.pop(stale, None)


class RosLidar:
    """Latest downward range whose age is measured only in simulation time."""

    def __init__(self, clock: SimulationClock) -> None:
        self._clock = clock
        self._lock = threading.Lock()
        self._distance_m: float | None = None
        self._timestamp_ns: int | None = None

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._timestamp_ns is not None

    def accept(self, message: object, sim_timestamp_ns: int) -> None:
        try:
            ranges = list(message.ranges)  # type: ignore[attr-defined]
            range_min = float(message.range_min)  # type: ignore[attr-defined]
            range_max = float(message.range_max)  # type: ignore[attr-defined]
        except (AttributeError, TypeError, ValueError) as error:
            raise ValueError("downward range message is malformed") from error
        if len(ranges) != 1:
            raise ValueError("downward range must contain exactly one beam")
        distance_m = float(ranges[0])
        if (
            not math.isfinite(distance_m)
            or not math.isfinite(range_min)
            or not math.isfinite(range_max)
            or range_min < 0
            or range_max <= range_min
            or not range_min <= distance_m <= range_max
        ):
            raise ValueError("downward range is invalid")
        if isinstance(sim_timestamp_ns, bool) or not isinstance(sim_timestamp_ns, int):
            raise TypeError("range simulation timestamp must be an integer")
        with self._lock:
            if self._timestamp_ns is not None and sim_timestamp_ns < self._timestamp_ns:
                raise ValueError("downward range timestamp regressed")
            self._timestamp_ns = sim_timestamp_ns
            self._distance_m = distance_m

    def get_distance(self) -> float:
        with self._lock:
            timestamp_ns = self._timestamp_ns
            distance_m = self._distance_m
        now_ns = self._clock.timestamp_ns
        if timestamp_ns is None or distance_m is None or now_ns is None:
            raise StaleSensorError("downward range is not ready")
        age_ns = now_ns - timestamp_ns
        if age_ns < 0:
            raise StaleSensorError("downward range is newer than the current clock")
        if age_ns > MAX_RANGE_AGE_NS:
            raise StaleSensorError("downward range is older than 0.5 simulated seconds")
        return distance_m


@dataclass(frozen=True)
class PayloadRequest:
    ATTACH: ClassVar[int] = 1
    RELEASE: ClassVar[int] = 2

    run_id: str
    aruco_id: int
    action: int
    command_id: str


@dataclass(frozen=True)
class PayloadResponse:
    accepted: bool
    code: str
    detail: str
    command_id: str
    response_sequence: int


class BlockingPayloadClient(Protocol):
    def call(self, request: PayloadRequest) -> object: ...


class PayloadDropper:
    """Original dropper shape backed by confirmed run-scoped ROS commands."""

    def __init__(
        self,
        run_id: str,
        aruco_id: int,
        client: BlockingPayloadClient,
        clock: SimulationClock,
    ) -> None:
        self._run_id = run_id
        self._aruco_id = aruco_id
        self._client = client
        self._clock = clock
        self._sequence = {PayloadRequest.ATTACH: 0, PayloadRequest.RELEASE: 0}
        self._lock = threading.Lock()

    def attach(self, aruco_id: int) -> bool:
        if aruco_id != self._aruco_id:
            raise ValueError("attachment marker does not match this payload adapter")
        return self._command(PayloadRequest.ATTACH, "attach")

    def drop(self, delay_hold: float = 0.0) -> bool:
        if delay_hold:
            self._clock.sleep(delay_hold)
        return self._command(PayloadRequest.RELEASE, "release")

    def _command(self, action: int, action_name: str) -> bool:
        with self._lock:
            self._sequence[action] += 1
            command_id = (
                f"run:{self._aruco_id}:{action_name}:{self._sequence[action]}"
            )
        request = PayloadRequest(
            self._run_id,
            self._aruco_id,
            action,
            command_id,
        )
        response = self._client.call(request)
        if response is None:
            raise RuntimeError(f"payload {command_id} returned no confirmation")
        if getattr(response, "command_id", None) != command_id:
            raise RuntimeError(f"payload response command_id did not match {command_id}")
        response_sequence = getattr(response, "response_sequence", None)
        if (
            isinstance(response_sequence, bool)
            or not isinstance(response_sequence, int)
            or response_sequence <= 0
        ):
            raise RuntimeError(f"payload {command_id} returned an invalid response sequence")
        if getattr(response, "accepted", None) is not True:
            code = getattr(response, "code", "REJECTED")
            detail = getattr(response, "detail", "")
            raise RuntimeError(f"payload {command_id} failed: {code}: {detail}")
        return True


@dataclass(frozen=True)
class MissionEventRecord:
    run_id: str
    sim_timestamp_ns: int
    event_id: int
    phase: str
    state: str
    detail: str


class MissionEventEmitter:
    """Convert the nested phase callback into ordered current-time records."""

    def __init__(self, run_id: str, clock: SimulationClock, publish) -> None:
        self._run_id = run_id
        self._clock = clock
        self._publish = publish
        self._lock = threading.Lock()
        self._next_event_id = 0
        self.last_phase: str | None = None
        self.last_state: str | None = None
        self._stop_reason: str | None = None

    def __call__(self, phase: str, state: str) -> None:
        timestamp_ns = self._clock.timestamp_ns
        if timestamp_ns is None:
            raise StaleSensorError("mission event cannot precede the public clock")
        with self._lock:
            if self._stop_reason is not None:
                raise RuntimeError(f"mission event emitter stopped: {self._stop_reason}")
            record = MissionEventRecord(
                self._run_id,
                timestamp_ns,
                self._next_event_id,
                phase,
                state,
                "automatic attempt",
            )
            self._next_event_id += 1
            self.last_phase = phase
            self.last_state = state
            self._publish(record)

    def stop(self, reason: str) -> None:
        with self._lock:
            if self._stop_reason is None:
                self._stop_reason = reason or "shutdown"

    @property
    def home_complete(self) -> bool:
        with self._lock:
            return self.last_phase == "HOME" and self.last_state == "COMPLETE"


class AttemptFailureCoordinator:
    """Own the first fatal attempt reason, cancellation, and recovery claim."""

    def __init__(self, *, stop_attempt, write_failure, recover) -> None:
        self._stop_attempt = stop_attempt
        self._write_failure = write_failure
        self._recover = recover
        self._lock = threading.Lock()
        self._failure_reason: str | None = None
        self._recovery_claimed = False

    @property
    def failed(self) -> bool:
        with self._lock:
            return self._failure_reason is not None

    @property
    def failure_reason(self) -> str | None:
        with self._lock:
            return self._failure_reason

    def guard_input(self, name: str, operation) -> bool:
        if self.failed:
            return False
        try:
            operation()
        except Exception as error:
            self.fail(f"competition {name} input failed: {error}")
            return False
        return not self.failed

    def fail(self, reason: str) -> bool:
        with self._lock:
            if self._failure_reason is not None:
                return False
            self._failure_reason = reason
            self._stop_attempt(reason)
        self._write_failure(reason)
        return True

    def finish_success(self, operation) -> bool:
        with self._lock:
            if self._failure_reason is not None:
                return False
            operation()
            return True

    def recover_once(self) -> bool:
        with self._lock:
            if self._failure_reason is None or self._recovery_claimed:
                return False
            self._recovery_claimed = True
        self._recover()
        return True


class Comp2026StartGate:
    """Separate durable process readiness from RUNNING-era mission readiness."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._process_ready = False
        self._running = False
        self._clock = False
        self._frame_ready = False
        self._range_ready = False
        self._payload_service_ready = False
        self._heartbeat_live = False
        self._armable = False
        self._stop_reason: str | None = None

    @property
    def mission_ready(self) -> bool:
        with self._condition:
            return self._process_ready

    @property
    def mission_start_ready(self) -> bool:
        with self._condition:
            return self._mission_start_ready()

    def mark_process_ready(self) -> None:
        self._mark("_process_ready")

    def accept_running(self) -> None:
        self._mark("_running")

    def accept_clock(self) -> None:
        self._mark("_clock")

    def refresh_live_readiness(
        self,
        *,
        frame_ready: bool,
        range_ready: bool,
        payload_service_ready: bool,
        heartbeat_live: bool,
        armable: bool,
    ) -> None:
        with self._condition:
            self._frame_ready = frame_ready is True
            self._range_ready = range_ready is True
            self._payload_service_ready = payload_service_ready is True
            self._heartbeat_live = heartbeat_live is True
            self._armable = armable is True
            self._condition.notify_all()

    def wait_until_ready(self) -> None:
        with self._condition:
            self._condition.wait_for(
                lambda: self._mission_start_ready() or self._stop_reason is not None
            )
            if self._stop_reason is not None:
                raise RuntimeError(f"competition mission stopped: {self._stop_reason}")

    def stop(self, reason: str) -> None:
        with self._condition:
            self._stop_reason = reason or "shutdown"
            self._condition.notify_all()

    def _mark(self, field: str) -> None:
        with self._condition:
            setattr(self, field, True)
            self._condition.notify_all()

    def _mission_start_ready(self) -> bool:
        return all(
            (
                self._process_ready,
                self._running,
                self._clock,
                self._frame_ready,
                self._range_ready,
                self._payload_service_ready,
                self._heartbeat_live,
                self._armable,
            )
        )


def _heartbeat_is_live(last_heartbeat: object) -> bool:
    return (
        isinstance(last_heartbeat, (int, float))
        and not isinstance(last_heartbeat, bool)
        and math.isfinite(last_heartbeat)
        and 0.0 <= last_heartbeat <= DRONEKIT_HEARTBEAT_TIMEOUT_SECONDS
    )


def refresh_comp2026_start_gate(
    gate: Comp2026StartGate,
    *,
    frame_source: RosFrameSource,
    lidar: RosLidar,
    payload_client: object,
    vehicle: object,
) -> None:
    """Publish one atomic snapshot of every dynamic start predicate."""

    try:
        lidar.get_distance()
    except StaleSensorError:
        range_ready = False
    else:
        range_ready = True
    gate.refresh_live_readiness(
        frame_ready=frame_source.ready,
        range_ready=range_ready,
        payload_service_ready=payload_client.service_is_ready() is True,  # type: ignore[attr-defined]
        heartbeat_live=_heartbeat_is_live(
            getattr(vehicle, "last_heartbeat", None)
        ),
        armable=getattr(vehicle, "is_armable", False) is True,
    )


def load_course_waypoints(course_path, home: GPSCoord) -> dict[str, GPSCoord]:
    """Load the one resolved ENU course source and map it around vehicle Home."""

    import yaml

    with open(course_path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if (
        not isinstance(document, dict)
        or document.get("units") != "meters"
        or document.get("origin") != "H"
        or not isinstance(document.get("waypoints"), dict)
    ):
        raise ValueError("resolved competition course is malformed")
    attempt = document.get("attempt")
    if not isinstance(attempt, dict):
        raise ValueError("resolved competition attempt configuration is malformed")
    transit_altitude = attempt.get("transit_agl_m")
    if (
        isinstance(transit_altitude, bool)
        or not isinstance(transit_altitude, (int, float))
        or not math.isfinite(transit_altitude)
        or transit_altitude <= 0
    ):
        raise ValueError("competition transit altitude is invalid")
    result: dict[str, GPSCoord] = {}
    for name in ("H", "L", "F2", "WA", "WM"):
        value = document["waypoints"].get(name)
        if not isinstance(value, dict):
            raise ValueError(f"competition waypoint {name} is missing")
        x_m, y_m = value.get("x"), value.get("y")
        if any(
            isinstance(axis, bool)
            or not isinstance(axis, (int, float))
            or not math.isfinite(axis)
            for axis in (x_m, y_m)
        ):
            raise ValueError(f"competition waypoint {name} is invalid")
        converted = world_xy_to_gps(home, x_m=float(x_m), y_m=float(y_m))
        result[name] = GPSCoord(converted.lat, converted.long, float(transit_altitude))
    return result


__all__ = [
    "AttemptFailureCoordinator",
    "Comp2026StartGate",
    "GPSCoord",
    "MissionEventEmitter",
    "MissionEventRecord",
    "PayloadDropper",
    "PayloadRequest",
    "PayloadResponse",
    "RosFrameSource",
    "RosLidar",
    "SimulationClock",
    "StaleSensorError",
    "horizontal_distance_m",
    "load_course_waypoints",
    "refresh_comp2026_start_gate",
    "world_xy_to_gps",
]
