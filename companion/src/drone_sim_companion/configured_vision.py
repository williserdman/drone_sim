"""Bounded camera observations for configured marker operations.

The worker performs image processing only.  Flight output remains owned by
``DroneOperations`` on the configured runtime thread.
"""

from __future__ import annotations

from dataclasses import dataclass
from queue import Empty, Queue
import threading


@dataclass(frozen=True)
class MarkerObservation:
    aruco_id: int
    sequence: int
    exposure_timestamp_ns: int
    receipt_timestamp_ns: int
    forward_m: float
    right_m: float
    down_m: float


@dataclass(frozen=True)
class RangeObservation:
    sequence: int
    timestamp_ns: int
    distance_m: float


class ConfiguredVision:
    """Adapt the imported Comp2026 camera to nonblocking requested-ID reads."""

    def __init__(self, camera, *, frame_timeout_s: float = 1.0) -> None:
        self._camera = camera
        self._frame_timeout_s = frame_timeout_s
        self._results: Queue[MarkerObservation | BaseException] = Queue()
        self._stop: threading.Event | None = None
        self._thread: threading.Thread | None = None
        self._threads: list[threading.Thread] = []

    def request(self, aruco_id: int, after_sequence: int) -> None:
        self.cancel()
        manager = getattr(self._camera, "cm", None)
        metadata = getattr(manager, "last_frame_metadata", None)
        cached_sequence = getattr(metadata, "sequence", None)
        if type(cached_sequence) is int and cached_sequence >= 0:
            after_sequence = max(after_sequence, cached_sequence)
        stop = threading.Event()
        self._stop = stop
        self._thread = threading.Thread(
            target=self._run,
            args=(stop, aruco_id, after_sequence),
            name=f"configured-aruco-{aruco_id}",
            daemon=True,
        )
        self._threads.append(self._thread)
        self._thread.start()

    def _run(self, stop: threading.Event, aruco_id: int, sequence: int) -> None:
        while not stop.is_set():
            try:
                vector, metadata = self._camera.vec_to_marker_3d_bounded(
                    aruco_id,
                    timeout_s=self._frame_timeout_s,
                    after_sequence=sequence,
                    quality=4,
                )
            except TimeoutError:
                continue
            except BaseException as error:
                self._results.put(error)
                return
            sequence = metadata.sequence
            if vector is None or metadata.exposure_timestamp_ns is None:
                continue
            self._results.put(MarkerObservation(
                aruco_id=aruco_id,
                sequence=metadata.sequence,
                exposure_timestamp_ns=metadata.exposure_timestamp_ns,
                receipt_timestamp_ns=metadata.receipt_timestamp_ns,
                forward_m=float(vector.x),
                right_m=float(vector.y),
                down_m=float(vector.z),
            ))

    def poll(self) -> MarkerObservation | None:
        try:
            result = self._results.get_nowait()
        except Empty:
            return None
        if isinstance(result, BaseException):
            raise RuntimeError(f"configured vision failed: {result}") from result
        return result

    def cancel(self) -> None:
        if self._stop is not None:
            self._stop.set()

    def close(self) -> None:
        self.cancel()
        for thread in self._threads:
            thread.join(timeout=1.0)
        stop_acquisition = getattr(self._camera.cm, "stop_acquisition", None)
        if stop_acquisition is not None:
            stop_acquisition(timeout_s=1.0)


__all__ = ["ConfiguredVision", "MarkerObservation", "RangeObservation"]
