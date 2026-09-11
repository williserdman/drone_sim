from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time as _wall_time
from typing import Callable, Protocol

from ... import timebase


class DistanceSensor(Protocol):
    @property
    def distance(self) -> float: ...


@dataclass(frozen=True)
class LidarResource:
    """A sensor and the cleanup operation for resources owned by its factory.

    A factory must return this value only after construction succeeds. If the
    factory raises, it remains responsible for closing anything allocated
    before the exception.
    """

    sensor: DistanceSensor
    cleanup: Callable[[], None]

    def __post_init__(self) -> None:
        if not callable(self.cleanup):
            raise TypeError("LidarResource cleanup must be callable")


@dataclass(frozen=True)
class LidarStopStatus:
    """Observed shutdown state after a bounded stop attempt."""

    worker_stopped: bool
    cleanup_completed: bool
    cleanup_error: str | None = None


class _ResourceConstructionCleanupError(RuntimeError):
    def __init__(self, cleanup_error: Exception):
        super().__init__(str(cleanup_error))
        self.cleanup_error = cleanup_error


class LidarConfigurationError(ValueError):
    """Raised when the hardware profile is incomplete or invalid."""


class LidarStartupError(RuntimeError):
    """Raised when no valid hardware sample arrives before startup times out."""

    def __init__(self, message: str, shutdown_status: LidarStopStatus):
        super().__init__(message)
        self.shutdown_status = shutdown_status


class StaleSensorError(RuntimeError):
    """Raised when the latest sensor sample is not current."""


@dataclass(frozen=True)
class LidarSample:
    """One corrected range observation and its invalidation epoch.

    ``distance_m`` is the raw beam range with ``mounting_offset_cm`` subtracted
    once. This is equivalent to moving the range origin from the sensor along
    the configured beam by that offset. ``invalidation_generation`` stays the
    same across uninterrupted valid reads and increases whenever valid evidence
    is revoked, including cancellation.
    """

    distance_m: float
    sampled_at: float
    sequence: int
    invalidation_generation: int = 0


def _finite_number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise LidarConfigurationError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise LidarConfigurationError(f"{name} must be a finite number") from None
    if not math.isfinite(result):
        raise LidarConfigurationError(f"{name} must be a finite number")
    return result


def _make_hardware_sensor() -> LidarResource:
    # Keep optional device libraries out of module import and offline tests.
    import board  # type: ignore
    import busio  # type: ignore
    import adafruit_lidarlite  # type: ignore

    i2c = busio.I2C(board.SCL, board.SDA)
    try:
        sensor = adafruit_lidarlite.LIDARLite(i2c)
    except Exception as construction_error:
        try:
            i2c.deinit()
        except Exception as cleanup_error:
            raise _ResourceConstructionCleanupError(
                cleanup_error
            ) from construction_error
        raise

    def cleanup() -> None:
        try:
            driver_cleanup = getattr(sensor, "deinit", None)
            if not callable(driver_cleanup):
                driver_cleanup = getattr(sensor, "close", None)
            if callable(driver_cleanup):
                driver_cleanup()
        finally:
            i2c.deinit()

    return LidarResource(sensor=sensor, cleanup=cleanup)


class Lidar:
    """Poll one LiDAR resource and publish current, validated observations."""

    def __init__(
        self,
        *,
        raw_min_cm: float,
        raw_max_cm: float,
        mounting_offset_cm: float,
        stale_after_seconds: float,
        startup_timeout_seconds: float,
        poll_interval_seconds: float,
        sensor_factory: Callable[[], LidarResource] | None = None,
        clock: Callable[[], float] = timebase.monotonic,
    ):
        self._raw_min_cm = _finite_number("raw_min_cm", raw_min_cm)
        self._raw_max_cm = _finite_number("raw_max_cm", raw_max_cm)
        self._mounting_offset_cm = _finite_number(
            "mounting_offset_cm", mounting_offset_cm
        )
        self._stale_after_seconds = _finite_number(
            "stale_after_seconds", stale_after_seconds
        )
        self._startup_timeout_seconds = _finite_number(
            "startup_timeout_seconds", startup_timeout_seconds
        )
        self._poll_interval_seconds = _finite_number(
            "poll_interval_seconds", poll_interval_seconds
        )
        if self._raw_min_cm < 0 or self._raw_max_cm < self._raw_min_cm:
            raise LidarConfigurationError("raw range must be ordered and non-negative")
        if self._mounting_offset_cm < 0:
            raise LidarConfigurationError("mounting_offset_cm must be non-negative")
        if self._mounting_offset_cm > self._raw_max_cm:
            raise LidarConfigurationError(
                "mounting_offset_cm cannot exceed the configured raw maximum"
            )
        if self._stale_after_seconds <= 0:
            raise LidarConfigurationError("stale_after_seconds must be positive")
        if self._startup_timeout_seconds <= 0:
            raise LidarConfigurationError("startup_timeout_seconds must be positive")
        if self._poll_interval_seconds <= 0:
            raise LidarConfigurationError("poll_interval_seconds must be positive")

        self._clock = clock
        self._sensor_factory = sensor_factory or _make_hardware_sensor
        self._sensor: DistanceSensor | None = None
        self._sample: LidarSample | None = None
        self._invalidation_generation = 0
        self._sample_lock = threading.Lock()
        self._first_sample = threading.Event()
        self._stop_requested = threading.Event()
        self._cleanup_completed = False
        self._cleanup_error: str | None = None
        self._thread = threading.Thread(target=self._run_sensor, daemon=True)
        startup_deadline = _wall_time.monotonic() + self._startup_timeout_seconds
        self._thread.start()

        remaining = max(0.0, startup_deadline - _wall_time.monotonic())
        if not self._first_sample.wait(remaining):
            # Do not wait on a driver call that has already exceeded its budget.
            # Cancellation and publication use the same lock, so a worker that
            # resumes later cannot publish or signal a first sample.
            self._cancel()
            raise LidarStartupError(
                "no valid LiDAR sample arrived before the startup timeout",
                self._current_stop_status(),
            )

    def _read_valid_distance(self, sensor: DistanceSensor) -> float | None:
        raw = sensor.distance
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            return None
        raw_cm = float(raw)
        if not math.isfinite(raw_cm):
            return None
        if not self._raw_min_cm <= raw_cm <= self._raw_max_cm:
            return None

        corrected_m = (raw_cm - self._mounting_offset_cm) / 100.0
        if not math.isfinite(corrected_m) or corrected_m < 0:
            return None
        return corrected_m

    def _run_sensor(self) -> None:
        resource: LidarResource | None = None
        try:
            resource = self._sensor_factory()
            if not isinstance(resource, LidarResource):
                raise TypeError("sensor_factory must return LidarResource")
            sensor = resource.sensor
            if self._stop_requested.is_set():
                return
            self._sensor = sensor
            self._poll_sensor(sensor)
        except _ResourceConstructionCleanupError as exc:
            self._cleanup_error = (
                f"{type(exc.cleanup_error).__name__}: {exc.cleanup_error}"
            )
            return
        except Exception:
            return
        finally:
            if resource is None:
                if self._cleanup_error is None:
                    self._cleanup_completed = True
            else:
                try:
                    resource.cleanup()
                except Exception as exc:
                    self._cleanup_error = f"{type(exc).__name__}: {exc}"
                else:
                    self._cleanup_completed = True

    def _poll_sensor(self, sensor: DistanceSensor) -> None:
        sequence = 0
        last_sampled_at: float | None = None
        while not self._stop_requested.is_set():
            try:
                distance_m = self._read_valid_distance(sensor)
                if self._stop_requested.is_set():
                    return
                if distance_m is not None:
                    sampled_at = self._clock()
                    if self._stop_requested.is_set():
                        return
                    if (
                        not isinstance(sampled_at, bool)
                        and isinstance(sampled_at, (int, float))
                        and math.isfinite(float(sampled_at))
                        and (
                            last_sampled_at is None
                            or float(sampled_at) >= last_sampled_at
                        )
                    ):
                        sampled_at = float(sampled_at)
                        sample = LidarSample(
                            distance_m,
                            sampled_at,
                            sequence + 1,
                            self._invalidation_generation,
                        )
                        if not self._publish_sample(sample):
                            return
                        sequence += 1
                        last_sampled_at = sampled_at
                    else:
                        self._invalidate_sample()
                else:
                    self._invalidate_sample()
            except Exception:
                self._invalidate_sample()
            self._stop_requested.wait(self._poll_interval_seconds)

    def _invalidate_sample(self) -> None:
        with self._sample_lock:
            if self._sample is not None:
                self._sample = None
                self._invalidation_generation += 1

    def _publish_sample(self, sample: LidarSample) -> bool:
        with self._sample_lock:
            if self._stop_requested.is_set():
                return False
            self._sample = sample
            self._first_sample.set()
            return True

    def _cancel(self) -> None:
        with self._sample_lock:
            self._stop_requested.set()
            if self._sample is not None:
                self._sample = None
                self._invalidation_generation += 1

    def get_sample(self) -> LidarSample:
        with self._sample_lock:
            sample = self._sample
            invalidation_generation = self._invalidation_generation
        if sample is None:
            raise StaleSensorError("LiDAR has no valid sample")

        while True:
            now_value = self._clock()
            if isinstance(now_value, bool) or not isinstance(
                now_value, (int, float)
            ):
                raise StaleSensorError("LiDAR clock is not finite")
            try:
                now = float(now_value)
            except (OverflowError, ValueError):
                raise StaleSensorError("LiDAR clock is not finite") from None
            if not math.isfinite(now):
                raise StaleSensorError("LiDAR clock is not finite")
            age = now - sample.sampled_at
            if age < 0:
                raise StaleSensorError("LiDAR sample timestamp is in the future")
            if age > self._stale_after_seconds:
                raise StaleSensorError(f"LiDAR sample is stale by {age:.2f} seconds")

            if not self._sample_lock.acquire(blocking=False):
                with self._sample_lock:
                    pass
                continue
            try:
                if (
                    self._stop_requested.is_set()
                    or self._sample is None
                    or self._invalidation_generation != invalidation_generation
                ):
                    raise StaleSensorError("LiDAR sample changed while being checked")
                return sample
            finally:
                self._sample_lock.release()

    def get_distance(self) -> float:
        """Return the latest current corrected range in metres."""
        return self.get_sample().distance_m

    def stop(self, *, timeout_seconds: float | None = None) -> LidarStopStatus:
        """Cancel polling and report whether worker-owned cleanup finished."""
        self._cancel()
        thread = getattr(self, "_thread", None)
        if thread is not None and thread is not threading.current_thread():
            timeout = self._poll_interval_seconds * 2
            if timeout_seconds is not None:
                timeout = _finite_number("timeout_seconds", timeout_seconds)
                if timeout < 0:
                    raise LidarConfigurationError(
                        "timeout_seconds must be non-negative"
                    )
            thread.join(timeout=timeout)
        return self._current_stop_status()

    def _current_stop_status(self) -> LidarStopStatus:
        thread = getattr(self, "_thread", None)
        worker_stopped = thread is None or not thread.is_alive()
        return LidarStopStatus(
            worker_stopped=worker_stopped,
            cleanup_completed=worker_stopped and self._cleanup_completed,
            cleanup_error=self._cleanup_error if worker_stopped else None,
        )
