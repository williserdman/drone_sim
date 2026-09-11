"""Explicit LiDAR diagnostic with metre output and checked shutdown."""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable
from typing import Any


def _load_lidar_adapter():
    from drone.sensors.lidar.lidar import Lidar

    return Lidar


_STATUS_UNAVAILABLE = object()


def _shutdown_failure(status: object) -> str | None:
    if status is _STATUS_UNAVAILABLE or status is None:
        return "LiDAR shutdown status unavailable"

    try:
        worker_stopped = status.worker_stopped
        cleanup_completed = status.cleanup_completed
        cleanup_error = status.cleanup_error
    except (AttributeError, TypeError):
        return "LiDAR shutdown status malformed"

    if (
        not isinstance(worker_stopped, bool)
        or not isinstance(cleanup_completed, bool)
        or (cleanup_error is not None and not isinstance(cleanup_error, str))
    ):
        return "LiDAR shutdown status malformed"
    problems = []
    if not worker_stopped:
        problems.append("worker is still running")
    if not cleanup_completed:
        problems.append("cleanup did not complete")
    if cleanup_error is not None:
        problems.append(f"cleanup error: {cleanup_error}")
    if problems:
        return f"LiDAR shutdown incomplete: {'; '.join(problems)}"
    return None


def _validate_sampling_options(
    sample_count: int, sample_interval_seconds: float
) -> None:
    if isinstance(sample_count, bool) or not isinstance(sample_count, int):
        raise ValueError("sample_count must be a positive integer")
    if sample_count <= 0:
        raise ValueError("sample_count must be a positive integer")
    if (
        isinstance(sample_interval_seconds, bool)
        or not isinstance(sample_interval_seconds, (int, float))
        or not math.isfinite(sample_interval_seconds)
        or sample_interval_seconds < 0
    ):
        raise ValueError("sample_interval_seconds must be finite and non-negative")


def _report_diagnostics(
    diagnostics: list[str],
    cancellation: KeyboardInterrupt | SystemExit | None,
) -> None:
    try:
        for message in diagnostics:
            print(message, file=sys.stderr)
    finally:
        if cancellation is not None:
            raise cancellation


def run_diagnostic(
    *,
    raw_min_cm: float,
    raw_max_cm: float,
    mounting_offset_cm: float,
    stale_after_seconds: float,
    startup_timeout_seconds: float,
    poll_interval_seconds: float,
    sample_count: int = 100,
    sample_interval_seconds: float = 0.05,
    adapter_factory: Callable[..., Any] | None = None,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """Return zero only after sampling and shutdown both succeed."""
    _validate_sampling_options(sample_count, sample_interval_seconds)
    adapter = None
    acquisition_error: Exception | None = None
    cancellation: KeyboardInterrupt | SystemExit | None = None
    shutdown_status: object = _STATUS_UNAVAILABLE
    stop_error: Exception | None = None
    stop_cancellation: KeyboardInterrupt | SystemExit | None = None
    diagnostics = []
    distances = []

    try:
        factory = adapter_factory or _load_lidar_adapter()
        adapter = factory(
            raw_min_cm=raw_min_cm,
            raw_max_cm=raw_max_cm,
            mounting_offset_cm=mounting_offset_cm,
            stale_after_seconds=stale_after_seconds,
            startup_timeout_seconds=startup_timeout_seconds,
            poll_interval_seconds=poll_interval_seconds,
        )
        for _ in range(sample_count):
            distance_m = adapter.get_distance()
            distances.append(distance_m)
            print(f"Distance: {distance_m:.3f} m")
            sleep(sample_interval_seconds)
    except (KeyboardInterrupt, SystemExit) as error:
        cancellation = error
    except Exception as error:
        acquisition_error = error
        diagnostics.append(f"LiDAR diagnostic failed: {error}")

    if adapter is None:
        if acquisition_error is not None:
            shutdown_status = getattr(
                acquisition_error, "shutdown_status", _STATUS_UNAVAILABLE
            )
        shutdown_failure = _shutdown_failure(shutdown_status)
        if shutdown_failure is not None:
            diagnostics.append(shutdown_failure)
        _report_diagnostics(diagnostics, cancellation)
        return 1

    try:
        shutdown_status = adapter.stop()
    except (KeyboardInterrupt, SystemExit) as error:
        stop_cancellation = error
        diagnostics.append(
            f"LiDAR shutdown interrupted: {type(error).__name__}"
        )
    except Exception as error:
        stop_error = error
        diagnostics.append(f"LiDAR shutdown failed: {error}")

    shutdown_failure = _shutdown_failure(shutdown_status)
    if shutdown_failure is not None:
        diagnostics.append(shutdown_failure)

    _report_diagnostics(diagnostics, cancellation or stop_cancellation)
    if acquisition_error is not None or stop_error is not None:
        return 1
    if shutdown_failure is not None:
        return 1

    print(f"Average: {sum(distances) / len(distances):.3f} m")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-min-cm", required=True, type=float)
    parser.add_argument("--raw-max-cm", required=True, type=float)
    parser.add_argument("--mounting-offset-cm", required=True, type=float)
    parser.add_argument("--stale-after-seconds", required=True, type=float)
    parser.add_argument("--startup-timeout-seconds", required=True, type=float)
    parser.add_argument("--poll-interval-seconds", required=True, type=float)
    parser.add_argument("--sample-count", type=int, default=100)
    parser.add_argument("--sample-interval-seconds", type=float, default=0.05)
    args = parser.parse_args(argv)
    return run_diagnostic(
        raw_min_cm=args.raw_min_cm,
        raw_max_cm=args.raw_max_cm,
        mounting_offset_cm=args.mounting_offset_cm,
        stale_after_seconds=args.stale_after_seconds,
        startup_timeout_seconds=args.startup_timeout_seconds,
        poll_interval_seconds=args.poll_interval_seconds,
        sample_count=args.sample_count,
        sample_interval_seconds=args.sample_interval_seconds,
    )


if __name__ == "__main__":
    raise SystemExit(main())
