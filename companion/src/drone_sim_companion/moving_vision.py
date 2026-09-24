"""Bounded latest-frame observer for the configured moving-pad mission."""

from __future__ import annotations

import threading
from typing import Any, Callable

from .moving_precision import AttitudeEvidence, MovingObservation, RangeEvidence


class MovingVision:
    def __init__(
        self,
        camera: Any,
        *,
        latest_range: Callable[[], RangeEvidence | None],
        latest_attitude: Callable[[], AttitudeEvidence | None],
    ) -> None:
        self._camera = camera
        self._latest_range = latest_range
        self._latest_attitude = latest_attitude
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest: MovingObservation | None = None
        self._error: BaseException | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._camera.cm.start_acquisition(quality=4)
        self._thread = threading.Thread(
            target=self._run,
            name="moving-pad-vision",
            daemon=True,
        )
        self._thread.start()

    def _detections(self, frame: Any, range_m: float):
        detector = getattr(self._camera, "detect_markers_body_frd", None)
        if detector is not None:
            return detector(frame, range_m)
        corners, ids, _rejected = self._camera.cm.get_coords(frame.frame)
        if ids is None:
            return []
        detections = []
        for raw_id in ids:
            marker_id = int(raw_id[0])
            vector = self._camera.vector_from_observation_3d(
                frame, marker_id, range_m
            )
            if vector is not None:
                detections.append((marker_id, (float(vector.x), float(vector.y), float(vector.z))))
        return detections

    def _run(self) -> None:
        sequence = 0
        try:
            while not self._stop.is_set():
                try:
                    frame = self._camera.cm.latest_observation(
                        after_sequence=sequence,
                        timeout_s=0.05,
                    )
                except TimeoutError:
                    continue
                sequence = frame.metadata.sequence
                range_evidence = self._latest_range()
                attitude = self._latest_attitude()
                if range_evidence is None or attitude is None:
                    continue
                detections = self._detections(frame, range_evidence.distance_m)
                if not detections:
                    continue
                marker_id, vector = detections[0]
                current = MovingObservation(
                    camera_timestamp_ns=frame.metadata.exposure_timestamp_ns,
                    camera_sequence=sequence,
                    marker_id=marker_id,
                    target_body_frd=vector,
                    range_m=range_evidence.distance_m,
                    range_timestamp_ns=range_evidence.timestamp_ns,
                    attitude_rpy_rad=attitude.rpy_rad,
                    attitude_timestamp_ns=attitude.timestamp_ns,
                )
                with self._lock:
                    self._latest = current
        except BaseException as error:
            with self._lock:
                self._error = error

    def latest(self) -> MovingObservation | None:
        with self._lock:
            if self._error is not None:
                raise RuntimeError("moving-pad vision worker failed") from self._error
            return self._latest

    def close(self, timeout_s: float) -> bool:
        self._stop.set()
        camera_closed = self._camera.cm.stop_acquisition(timeout_s=timeout_s)
        thread = self._thread
        if thread is not None:
            thread.join(timeout_s)
        return camera_closed and (thread is None or not thread.is_alive())


__all__ = ["MovingVision"]
