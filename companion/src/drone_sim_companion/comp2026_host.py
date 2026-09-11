"""Thin simulation/ROS adapters for the original Comp2026 mission."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, ClassVar, Protocol

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

    @property
    def stopped(self) -> bool:
        with self._condition:
            return self._stop_reason is not None

    def accept(self, timestamp_ns: int) -> None:
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int):
            raise TypeError("simulation timestamp must be an integer")
        if timestamp_ns < 0:
            raise ValueError("simulation timestamp must be nonnegative")
        with self._condition:
            self._raise_if_stopped()
            if self._timestamp_ns is not None and timestamp_ns < self._timestamp_ns:
                raise ValueError("authoritative simulation clock regressed")
            self._timestamp_ns = timestamp_ns
            self._condition.notify_all()

    def now(self) -> float:
        with self._condition:
            self._raise_if_stopped()
            return (
                0.0
                if self._timestamp_ns is None
                else self._timestamp_ns / 1_000_000_000
            )

    def read_timestamp_ns(self) -> int:
        """Return current authoritative time, failing closed when unavailable."""

        with self._condition:
            self._raise_if_stopped()
            if self._timestamp_ns is None:
                raise StaleSensorError("authoritative simulation clock is unavailable")
            return self._timestamp_ns

    def run_at_current_timestamp(self, operation: Callable[[int], None]) -> bool:
        with self._condition:
            if self._timestamp_ns is None:
                return False
            operation(self._timestamp_ns)
            return True

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


class RosFrameSource:
    """Retain only the latest onboard image for blocking BGR capture."""

    def __init__(self, *, width_px: int, height_px: int) -> None:
        self._width_px = width_px
        self._height_px = height_px
        self._condition = threading.Condition()
        self._latest_image: object | None = None
        self._latest_timestamp_ns: int | None = None
        self.last_timestamp_ns: int | None = None
        self._stop_reason: str | None = None

    @property
    def ready(self) -> bool:
        with self._condition:
            return self._stop_reason is None and self._latest_image is not None

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
        with self._condition:
            if self._stop_reason is not None:
                raise StaleSensorError(
                    f"frame source stopped before a newer frame: {self._stop_reason}"
                )
            if self.last_timestamp_ns is None or timestamp_ns > self.last_timestamp_ns:
                if (
                    self._latest_timestamp_ns is None
                    or timestamp_ns > self._latest_timestamp_ns
                ):
                    self._latest_timestamp_ns = timestamp_ns
                    self._latest_image = message
            self._condition.notify_all()

    def capture_frame(
        self, quality: int = 4, deadline_sim_ns: int | None = None
    ) -> np.ndarray:
        del quality  # The original API's quality hint must not rescale public evidence.
        with self._condition:
            while True:
                if self._stop_reason is not None:
                    raise StaleSensorError(
                        f"frame source stopped before a newer frame: {self._stop_reason}"
                    )
                if self._latest_image is not None:
                    timestamp_ns = self._latest_timestamp_ns
                    image = self._latest_image
                    assert timestamp_ns is not None
                    self._latest_image = None
                    self._latest_timestamp_ns = None
                    self.last_timestamp_ns = timestamp_ns
                    try:
                        data = bytes(image.data)  # type: ignore[attr-defined]
                    except (AttributeError, TypeError, ValueError) as error:
                        raise ValueError("image data is malformed") from error
                    if len(data) != self._width_px * self._height_px * 3:
                        raise ValueError(
                            "image data length does not match its geometry"
                        )
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
                self._condition.wait()

    def stop(self, reason: str) -> None:
        with self._condition:
            if self._stop_reason is None:
                self._stop_reason = reason or "shutdown"
            self._latest_image = None
            self._latest_timestamp_ns = None
            self._condition.notify_all()


class RosLidar:
    """Latest downward range whose age is measured only in simulation time."""

    def __init__(
        self,
        clock: SimulationClock,
        *,
        sample_factory: Callable[[float, float, int, int], object],
    ) -> None:
        if not callable(sample_factory):
            raise TypeError("range sample_factory must be callable")
        self._clock = clock
        self._sample_factory = sample_factory
        self._lock = threading.RLock()
        self._sample: object | None = None
        self._sample_timestamp_ns: int | None = None
        self._pending_sample: object | None = None
        self._pending_timestamp_ns: int | None = None
        self._latest_source_timestamp_ns: int | None = None
        self._sequence = 0
        self._invalidation_generation = 0
        self._stop_reason: str | None = None

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._sample is not None and self._stop_reason is None

    def accept(self, message: object, sim_timestamp_ns: int) -> None:
        if isinstance(sim_timestamp_ns, bool) or not isinstance(sim_timestamp_ns, int):
            self.invalidate()
            raise TypeError("range simulation timestamp must be an integer")
        if sim_timestamp_ns < 0:
            self.invalidate()
            raise ValueError("range simulation timestamp must be nonnegative")

        with self._lock:
            if self._stop_reason is not None:
                raise StaleSensorError(f"downward range stopped: {self._stop_reason}")
            current_clock_ns = self._clock.timestamp_ns
            self._promote_pending_through(current_clock_ns)
            if (
                self._latest_source_timestamp_ns is not None
                and sim_timestamp_ns <= self._latest_source_timestamp_ns
            ):
                self._invalidate_locked()
                raise ValueError("downward range timestamp must advance")
            self._latest_source_timestamp_ns = sim_timestamp_ns
            try:
                ranges = list(message.ranges)  # type: ignore[attr-defined]
                range_min = float(message.range_min)  # type: ignore[attr-defined]
                range_max = float(message.range_max)  # type: ignore[attr-defined]
                if len(ranges) != 1:
                    raise ValueError("downward range must contain exactly one beam")
                distance_m = float(ranges[0])
            except Exception as error:
                self._invalidate_locked()
                raise ValueError("downward range message is malformed") from error
            if (
                not math.isfinite(distance_m)
                or not math.isfinite(range_min)
                or not math.isfinite(range_max)
                or range_min < 0
                or range_max <= range_min
                or not range_min <= distance_m <= range_max
            ):
                self._invalidate_locked()
                raise ValueError("downward range is invalid")
            sequence = self._sequence + 1
            try:
                sample = self._sample_factory(
                    distance_m,
                    sim_timestamp_ns / 1_000_000_000,
                    sequence,
                    self._invalidation_generation,
                )
            except Exception:
                self._invalidate_locked()
                raise
            self._sequence = sequence
            if current_clock_ns is not None and sim_timestamp_ns <= current_clock_ns:
                self._sample_timestamp_ns = sim_timestamp_ns
                self._sample = sample
                self._pending_timestamp_ns = None
                self._pending_sample = None
            else:
                self._pending_timestamp_ns = sim_timestamp_ns
                self._pending_sample = sample

    def invalidate(self) -> None:
        with self._lock:
            self._invalidate_locked()

    def stop(self, reason: str) -> None:
        with self._lock:
            if self._stop_reason is None:
                self._stop_reason = reason or "shutdown"
            self._invalidate_locked()

    def get_sample(self) -> object:
        with self._lock:
            if self._stop_reason is not None:
                raise StaleSensorError(f"downward range stopped: {self._stop_reason}")
            if self._clock.stopped:
                raise StaleSensorError("downward range clock stopped")
            now_ns = self._clock.timestamp_ns
            self._promote_pending_through(now_ns)
            timestamp_ns = self._sample_timestamp_ns
            sample = self._sample
            if timestamp_ns is None or sample is None or now_ns is None:
                raise StaleSensorError("downward range is not ready")
            age_ns = now_ns - timestamp_ns
            if age_ns < 0:
                raise StaleSensorError("downward range is newer than the current clock")
            if age_ns > MAX_RANGE_AGE_NS:
                raise StaleSensorError(
                    "downward range is older than 0.5 simulated seconds"
                )
            return sample

    def get_distance(self) -> float:
        sample = self.get_sample()
        return sample.distance_m  # type: ignore[attr-defined, no-any-return]

    def _invalidate_locked(self) -> None:
        self._sample = None
        self._sample_timestamp_ns = None
        self._pending_sample = None
        self._pending_timestamp_ns = None
        self._invalidation_generation += 1

    def _promote_pending_through(self, clock_timestamp_ns: int | None) -> None:
        if (
            clock_timestamp_ns is not None
            and self._pending_timestamp_ns is not None
            and self._pending_timestamp_ns <= clock_timestamp_ns
        ):
            self._sample_timestamp_ns = self._pending_timestamp_ns
            self._sample = self._pending_sample
            self._pending_timestamp_ns = None
            self._pending_sample = None


@dataclass(frozen=True)
class QgcLidarStopStatus:
    """Compatibility result for a synchronous ROS lidar shutdown."""

    worker_stopped: bool
    cleanup_completed: bool


class QgcRangeIngress:
    """Serialize ROS range ingress with nested lidar shutdown."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._open = True

    def accept(self, operation: Callable[[], bool]) -> bool:
        with self._lock:
            if not self._open:
                return False
            return operation()

    def close_and_stop(self, operation: Callable[[], None]) -> None:
        with self._lock:
            self._open = False
            operation()


class QgcRosLidarAdapter:
    """Expose a synchronous ``RosLidar`` through the QGC lidar contract."""

    def __init__(
        self, lidar: RosLidar, *, range_ingress: QgcRangeIngress | None = None
    ) -> None:
        self._lidar = lidar
        self._range_ingress = range_ingress

    def get_sample(self) -> object:
        return self._lidar.get_sample()

    def get_distance(self) -> float:
        return self._lidar.get_distance()

    def stop(
        self, *, timeout_seconds: float | None = None
    ) -> QgcLidarStopStatus:
        if timeout_seconds is not None:
            if isinstance(timeout_seconds, bool) or not isinstance(
                timeout_seconds, (int, float)
            ):
                raise TypeError("timeout_seconds must be numeric")
            try:
                normalized_timeout = float(timeout_seconds)
            except (OverflowError, ValueError) as error:
                raise ValueError(
                    "timeout_seconds must be finite and nonnegative"
                ) from error
            if not math.isfinite(normalized_timeout) or normalized_timeout < 0:
                raise ValueError("timeout_seconds must be finite and nonnegative")
        stop = lambda: self._lidar.stop("QGC listener cleanup")
        if self._range_ingress is None:
            stop()
        else:
            self._range_ingress.close_and_stop(stop)
        return QgcLidarStopStatus(
            worker_stopped=True,
            cleanup_completed=True,
        )


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
    def prepare(self, request: PayloadRequest) -> object: ...

    def dispatch(self, prepared: object) -> object: ...

    def await_response(
        self, pending: object, *, cancelled: threading.Event
    ) -> object: ...


class PayloadPermissionError(PermissionError):
    """A payload request lacks current permission at its dispatch boundary."""


class PayloadCompletionIndeterminateError(RuntimeError):
    """A dispatched payload request has no trustworthy physical outcome."""


class PayloadDropper:
    """Original dropper shape backed by confirmed run-scoped ROS commands."""

    supports_attachment = True

    def __init__(
        self,
        run_id: str,
        aruco_id: int,
        client: BlockingPayloadClient,
        clock: SimulationClock,
        *,
        permission: Callable[[], bool],
        delay_wall_timeout_seconds: float,
    ) -> None:
        if isinstance(aruco_id, bool) or not isinstance(aruco_id, int):
            raise TypeError("payload ArUco ID must be an integer")
        if not callable(permission):
            raise ValueError("payload permission must be callable")
        if not callable(getattr(permission, "actuate", None)):
            raise ValueError("payload permission needs an atomic actuation hook")
        if (
            isinstance(delay_wall_timeout_seconds, bool)
            or not isinstance(delay_wall_timeout_seconds, (int, float))
            or not math.isfinite(delay_wall_timeout_seconds)
            or delay_wall_timeout_seconds <= 0
        ):
            raise ValueError("payload delay wall timeout must be finite and positive")
        self._run_id = run_id
        self._aruco_id = aruco_id
        self._client = client
        self._clock = clock
        self._permission = permission
        self._delay_wall_timeout_seconds = float(delay_wall_timeout_seconds)
        self._sequence: dict[tuple[int, int], int] = {}
        self._lock = threading.Lock()
        self._production_lock = threading.RLock()
        self._closed = threading.Event()

    @property
    def aruco_id(self) -> int:
        return self._aruco_id

    def attach(self, aruco_id: int) -> bool:
        if aruco_id != self._aruco_id:
            raise ValueError("attachment marker does not match this payload adapter")
        self._command(PayloadRequest.ATTACH, "attach", self._actuate)
        return True

    def drop(self, delay_hold: float = 0.0) -> None:
        self.drop_with_guard(self._actuate, self._actuate, delay_hold=delay_hold)

    def drop_with_guard(
        self,
        release_actuate: Callable[[Callable[[], None]], None],
        continuation_actuate: Callable[[Callable[[], None]], None],
        *,
        delay_hold: float = 0.0,
    ) -> None:
        if not callable(release_actuate) or not callable(continuation_actuate):
            raise TypeError("release and continuation actuators must be callable")
        if delay_hold:
            self._wait_before_release(delay_hold)
        self._command(PayloadRequest.RELEASE, "release", release_actuate)

    def cleanup_passive(self) -> bool:
        """Stop this adapter from producing or waiting for local ROS requests."""

        with self._production_lock:
            self._closed.set()
        return True

    def _actuate(self, output: Callable[[], None]) -> None:
        transaction = getattr(self._permission, "actuate", None)
        if not callable(transaction):
            raise PayloadPermissionError(
                "payload atomic actuation transaction is not installed"
            )
        transaction(output)

    def _require_permission(self) -> None:
        try:
            permitted = self._permission() is True
        except Exception as error:
            raise PayloadPermissionError("payload permission check failed") from error
        if not permitted:
            raise PayloadPermissionError("payload actuation permission is not current")

    def _ensure_open(self) -> None:
        if self._closed.is_set():
            raise RuntimeError("payload adapter is closed")

    def _wait_before_release(self, seconds: float) -> None:
        if isinstance(seconds, bool) or not isinstance(seconds, (int, float)):
            raise TypeError("payload delay must be numeric")
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("payload delay must be finite and nonnegative")
        wall_deadline = time.monotonic() + self._delay_wall_timeout_seconds
        start_ns = self._clock.timestamp_ns
        while start_ns is None:
            self._ensure_open()
            if self._clock.stopped:
                self._clock.read_timestamp_ns()
            self._require_permission()
            self._wait_for_delay_poll(wall_deadline)
            start_ns = self._clock.timestamp_ns
        delay_ns = int(float(seconds) * 1_000_000_000)
        while True:
            self._ensure_open()
            if self._clock.stopped:
                self._clock.read_timestamp_ns()
            self._require_permission()
            now_ns = self._clock.timestamp_ns
            if time.monotonic() >= wall_deadline:
                raise TimeoutError("payload release delay timed out")
            if now_ns is not None and now_ns - start_ns >= delay_ns:
                return
            self._wait_for_delay_poll(wall_deadline)

    def _wait_for_delay_poll(self, wall_deadline: float) -> None:
        remaining = wall_deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("payload release delay timed out")
        self._closed.wait(min(0.05, remaining))

    def _command(
        self,
        action: int,
        action_name: str,
        actuate: Callable[[Callable[[], None]], None],
        *,
        aruco_id: int | None = None,
    ) -> object:
        self._ensure_open()
        selected_aruco_id = self._aruco_id if aruco_id is None else aruco_id
        with self._lock:
            sequence_key = (selected_aruco_id, action)
            sequence = self._sequence.get(sequence_key, 0) + 1
            self._sequence[sequence_key] = sequence
            command_id = f"run:{selected_aruco_id}:{action_name}:{sequence}"
        request = PayloadRequest(
            self._run_id,
            selected_aruco_id,
            action,
            command_id,
        )
        prepared = self._client.prepare(request)
        self._require_permission()
        pending_calls: list[object] = []

        def dispatch() -> None:
            with self._production_lock:
                self._ensure_open()
                pending_calls.append(self._client.dispatch(prepared))

        try:
            actuate(dispatch)
        except BaseException as error:
            if pending_calls:
                raise PayloadCompletionIndeterminateError(
                    f"payload {command_id} completion is indeterminate"
                ) from error
            raise
        if len(pending_calls) != 1:
            error_type = (
                RuntimeError
                if not pending_calls
                else PayloadCompletionIndeterminateError
            )
            raise error_type("payload actuation did not dispatch exactly one request")
        try:
            response = self._client.await_response(
                pending_calls[0], cancelled=self._closed
            )
        except BaseException as error:
            if isinstance(error, (KeyboardInterrupt, SystemExit)):
                raise
            if isinstance(error, TimeoutError):
                raise
            raise PayloadCompletionIndeterminateError(str(error)) from error
        if response is None:
            raise PayloadCompletionIndeterminateError(
                f"payload {command_id} returned no confirmation"
            )
        if getattr(response, "command_id", None) != command_id:
            raise PayloadCompletionIndeterminateError(
                f"payload response command_id did not match {command_id}"
            )
        response_sequence = getattr(response, "response_sequence", None)
        if (
            isinstance(response_sequence, bool)
            or not isinstance(response_sequence, int)
            or response_sequence <= 0
        ):
            raise PayloadCompletionIndeterminateError(
                f"payload {command_id} returned an invalid response sequence"
            )
        if getattr(response, "accepted", None) is True:
            return response
        code = getattr(response, "code", "REJECTED")
        detail = getattr(response, "detail", "")
        raise PayloadCompletionIndeterminateError(
            f"payload {command_id} failed: {code}: {detail}"
        )


class QgcCompetitionPayloadAdapter:
    """Enforce the confirmed three-payload competition sequence."""

    supports_attachment = True
    _PAYLOAD_IDS = frozenset({2, 3, 4})

    def __init__(
        self,
        run_id: str,
        client: BlockingPayloadClient,
        clock: SimulationClock,
        *,
        permission: Callable[[], bool],
        delay_wall_timeout_seconds: float,
    ) -> None:
        self._dropper = PayloadDropper(
            run_id,
            2,
            client,
            clock,
            permission=permission,
            delay_wall_timeout_seconds=delay_wall_timeout_seconds,
        )
        self._operation_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._attached_id: int | None = 2
        self._next_pickup_id: int | None = 3
        self._last_response_sequence = 0
        self._indeterminate = False

    def attach(self, aruco_id: int) -> bool:
        if isinstance(aruco_id, bool) or not isinstance(aruco_id, int):
            raise ValueError("payload marker must be integer 3 or 4")
        if aruco_id not in self._PAYLOAD_IDS:
            raise ValueError("unknown competition payload marker")
        if aruco_id == 2:
            raise ValueError("payload 2 starts attached and cannot be attached")
        with self._operation_lock:
            with self._state_lock:
                self._ensure_determinate_locked()
                if self._attached_id is not None:
                    raise RuntimeError("payload capacity one is already occupied")
                if aruco_id != self._next_pickup_id:
                    raise RuntimeError("payload attachment is outside the competition sequence")
            self._confirmed_command(
                aruco_id,
                PayloadRequest.ATTACH,
                "attach",
                self._dropper._actuate,
            )
            with self._state_lock:
                self._attached_id = aruco_id
            return True

    def drop(self, delay_hold: float = 0.0) -> None:
        self.drop_with_guard(
            self._dropper._actuate,
            self._dropper._actuate,
            delay_hold=delay_hold,
        )

    def drop_with_guard(
        self,
        release_actuate: Callable[[Callable[[], None]], None],
        continuation_actuate: Callable[[Callable[[], None]], None],
        *,
        delay_hold: float = 0.0,
    ) -> None:
        if not callable(release_actuate) or not callable(continuation_actuate):
            raise TypeError("release and continuation actuators must be callable")
        del continuation_actuate
        with self._operation_lock:
            with self._state_lock:
                self._ensure_determinate_locked()
                aruco_id = self._attached_id
                if aruco_id is None:
                    raise RuntimeError("no payload is attached for release")
            if delay_hold:
                self._dropper._wait_before_release(delay_hold)
            self._confirmed_command(
                aruco_id,
                PayloadRequest.RELEASE,
                "release",
                release_actuate,
            )
            with self._state_lock:
                self._attached_id = None
                self._next_pickup_id = {2: 3, 3: 4, 4: None}[aruco_id]

    def cleanup_passive(self) -> bool:
        return self._dropper.cleanup_passive() is True

    def _ensure_determinate_locked(self) -> None:
        if self._indeterminate:
            raise RuntimeError("payload state is indeterminate")

    def _confirmed_command(
        self,
        aruco_id: int,
        action: int,
        action_name: str,
        actuate: Callable[[Callable[[], None]], None],
    ) -> None:
        try:
            response = self._dropper._command(
                action,
                action_name,
                actuate,
                aruco_id=aruco_id,
            )
            response_sequence = getattr(response, "response_sequence", None)
            if (
                getattr(response, "accepted", None) is not True
                or getattr(response, "code", None) != "OK"
                or isinstance(response_sequence, bool)
                or not isinstance(response_sequence, int)
                or response_sequence <= self._last_response_sequence
            ):
                raise PayloadCompletionIndeterminateError(
                    "payload response did not exactly confirm the requested transition"
                )
        except (PayloadCompletionIndeterminateError, TimeoutError):
            with self._state_lock:
                self._indeterminate = True
            raise
        self._last_response_sequence = response_sequence


class QgcFm2PayloadAdapter:
    """Expose only scenario payload 2 for the FM2 release mission."""

    supports_attachment = False

    def __init__(self, dropper: PayloadDropper, *, aruco_id: int) -> None:
        if isinstance(aruco_id, bool) or not isinstance(aruco_id, int):
            raise TypeError("FM2 payload ArUco ID must be integer 2")
        if aruco_id != 2:
            raise ValueError("FM2 payload ArUco ID must be integer 2")
        delegate_aruco_id = getattr(dropper, "aruco_id", None)
        if isinstance(delegate_aruco_id, bool) or not isinstance(
            delegate_aruco_id, int
        ):
            raise TypeError("FM2 payload delegate must expose integer ArUco ID 2")
        if delegate_aruco_id != aruco_id:
            raise ValueError("FM2 payload delegate ArUco ID must match integer 2")
        self._dropper = dropper

    def drop(self, delay_hold: float = 0.0) -> None:
        self._dropper.drop(delay_hold=delay_hold)

    def drop_with_guard(
        self,
        release_actuate: Callable[[Callable[[], None]], None],
        continuation_actuate: Callable[[Callable[[], None]], None],
        *,
        delay_hold: float = 0.0,
    ) -> None:
        self._dropper.drop_with_guard(
            release_actuate,
            continuation_actuate,
            delay_hold=delay_hold,
        )

    def cleanup_passive(self) -> bool:
        return self._dropper.cleanup_passive() is True

    def attach(self, aruco_id: int) -> bool:
        del aruco_id
        raise NotImplementedError("FM2 payload adapter does not support attachment")


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

    _SEQUENCE = (
        ("FM1", "STARTED"),
        ("FM1", "COMPLETE"),
        ("FM2", "STARTED"),
        ("FM2", "COMPLETE"),
    )

    def __init__(self, run_id: str, clock: SimulationClock, publish) -> None:
        self._run_id = run_id
        self._clock = clock
        self._publish = publish
        self._lock = threading.RLock()
        self._next_event_id = 0
        self.last_phase: str | None = None
        self.last_state: str | None = None
        self._stop_reason: str | None = None
        self._publishing = False

    def __call__(self, phase: str, state: str) -> None:
        with self._lock:
            if self._stop_reason is not None:
                raise RuntimeError(f"mission event emitter stopped: {self._stop_reason}")
            if self._publishing:
                raise RuntimeError("mission event publication is already in progress")
            if self._next_event_id == len(self._SEQUENCE):
                raise RuntimeError("mission event sequence is complete")
            expected = self._SEQUENCE[self._next_event_id]
            if (phase, state) != expected:
                raise ValueError(
                    "next mission event must be "
                    f"{expected[0]} {expected[1]}, got {phase} {state}"
                )
            timestamp_ns = self._clock.read_timestamp_ns()
            record = MissionEventRecord(
                self._run_id,
                timestamp_ns,
                self._next_event_id,
                phase,
                state,
                "automatic attempt",
            )
            self._publishing = True
            try:
                self._publish(record)
            except BaseException as error:
                self._stop_reason = f"publication failed: {type(error).__name__}"
                raise
            else:
                self._next_event_id += 1
                self.last_phase = phase
                self.last_state = state
            finally:
                self._publishing = False

    def stop(self, reason: str) -> None:
        with self._lock:
            if self._stop_reason is None:
                self._stop_reason = reason or "shutdown"

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
        failure_reason: str | None = None
        with self._lock:
            if self._failure_reason is not None:
                return False
            try:
                operation()
            except Exception as error:
                failure_reason = f"competition {name} input failed: {error}"
                self._failure_reason = failure_reason
        if failure_reason is not None:
            self._complete_failure(failure_reason)
            return False
        return True

    def fail(self, reason: str) -> bool:
        with self._lock:
            if self._failure_reason is not None:
                return False
            self._failure_reason = reason
        self._complete_failure(reason)
        return True

    def _complete_failure(self, reason: str) -> None:
        self._stop_attempt(reason)
        self._write_failure(reason)

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
        self._command_delivered = False
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

    @property
    def readiness(self) -> dict[str, bool]:
        with self._condition:
            return {
                "process_ready": self._process_ready,
                "running": self._running,
                "clock": self._clock,
                "command_delivered": self._command_delivered,
                "frame_ready": self._frame_ready,
                "range_ready": self._range_ready,
                "payload_service_ready": self._payload_service_ready,
                "heartbeat_live": self._heartbeat_live,
                "armable": self._armable,
            }

    def mark_process_ready(self) -> None:
        self._mark("_process_ready")

    def accept_running(self) -> None:
        self._mark("_running")

    def accept_clock(self) -> None:
        self._mark("_clock")

    def mark_command_delivered(self) -> None:
        self._mark("_command_delivered")

    def refresh_live_readiness(
        self,
        *,
        frame_ready: bool,
        payload_service_ready: bool,
        heartbeat_live: bool,
        armable: bool,
        range_is_current: Callable[[], bool],
    ) -> None:
        with self._condition:
            self._frame_ready = frame_ready is True
            self._payload_service_ready = payload_service_ready is True
            self._heartbeat_live = heartbeat_live is True
            self._armable = armable is True
            self._range_ready = range_is_current() is True
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
                self._command_delivered,
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

    frame_ready = frame_source.ready
    payload_service_ready = payload_client.service_is_ready() is True  # type: ignore[attr-defined]
    heartbeat_live = _heartbeat_is_live(getattr(vehicle, "last_heartbeat", None))
    armable = getattr(vehicle, "is_armable", False) is True

    def range_is_current() -> bool:
        try:
            lidar.get_distance()
        except StaleSensorError:
            return False
        return True

    gate.refresh_live_readiness(
        frame_ready=frame_ready,
        payload_service_ready=payload_service_ready,
        heartbeat_live=heartbeat_live,
        armable=armable,
        range_is_current=range_is_current,
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
    "QgcCompetitionPayloadAdapter",
    "PayloadRequest",
    "PayloadResponse",
    "QgcFm2PayloadAdapter",
    "QgcLidarStopStatus",
    "QgcRangeIngress",
    "QgcRosLidarAdapter",
    "RosFrameSource",
    "RosLidar",
    "SimulationClock",
    "StaleSensorError",
    "horizontal_distance_m",
    "load_course_waypoints",
    "refresh_comp2026_start_gate",
    "world_xy_to_gps",
]
