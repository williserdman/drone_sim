from ._camera_manager import CameraManager
from ...common_types import RelativePosition, RelPosComplete
from typing import Optional
from collections import deque
import json
from pathlib import Path
import threading
from dataclasses import dataclass
import math
import cv2  # pyright: ignore[reportMissingImports]
import time
import numpy as np


_RELEASE_FAILURE_NOTE_PREFIX = "Video writer release also failed: "


@dataclass(frozen=True)
class CameraReadiness:
    ready: bool
    reasons: tuple[str, ...]


def _error_detail(error: BaseException) -> str:
    try:
        return str(error) or type(error).__name__
    except BaseException:
        return type(error).__name__


def _recording_error_detail(error: BaseException) -> str:
    primary_detail = _error_detail(error)
    release_notes = []
    try:
        for note in error.__notes__:
            if (
                type(note) is str
                and note.startswith(_RELEASE_FAILURE_NOTE_PREFIX)
                and note not in release_notes
            ):
                release_notes.append(note)
    except BaseException:
        return primary_detail
    return "; ".join((primary_detail, *release_notes))


class Camera:
    def __init__(
        self,
        marker_size_mm: int,
        manager=None,
        mounting_path=None,
        *,
        writer_factory=None,
    ):
        self.marker_size_mm = float(marker_size_mm)
        if not math.isfinite(self.marker_size_mm) or self.marker_size_mm <= 0:
            raise ValueError("marker_size_mm must be finite and positive")
        self.frame_buffer = deque(maxlen=100)  # 10_000
        self._frame_buffer_lock = threading.Lock()
        self._writer_factory = writer_factory or cv2.VideoWriter
        self._recording_lock = threading.Lock()
        self._recording_threads: dict[str, threading.Thread] = {}
        self._recording_errors: dict[str, str] = {}

        config_path = (
            Path(mounting_path)
            if mounting_path is not None
            else Path(__file__).with_name("mounting.json")
        )
        with config_path.open("r") as config_file:
            mounting = json.load(config_file)

        transform = mounting["camera_to_body_frd"]
        if len(transform) != 3 or any(len(row) != 3 for row in transform):
            raise ValueError("camera_to_body_frd must be a 3x3 matrix")
        try:
            self._camera_to_body_frd = tuple(
                tuple(float(value) for value in row) for row in transform
            )
            self._down_offset_m = float(mounting["down_offset_m"])
        except (TypeError, ValueError) as error:
            raise ValueError("camera mounting values must be numeric") from error
        rotation = np.asarray(self._camera_to_body_frd, dtype=float)
        if not np.all(np.isfinite(rotation)) or not math.isfinite(self._down_offset_m):
            raise ValueError("camera mounting values must be finite")
        if not np.allclose(rotation.T @ rotation, np.identity(3), atol=1e-6):
            raise ValueError("camera_to_body_frd must be orthonormal")
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-6):
            raise ValueError("camera_to_body_frd must be right-handed")
        self._mounting_verified = mounting.get("verified") is True
        self.cm = manager if manager is not None else CameraManager()

    @property
    def last_frame_timestamp(self):
        return self.cm.last_frame_timestamp

    def _buffer_frame(self, frame) -> None:
        with self._frame_buffer_lock:
            self.frame_buffer.append(frame)

    def _body_marker_vector(self, tvec_mm, lidar_alt=None) -> RelPosComplete:
        camera_metres = tuple(float(value) / 1000.0 for value in tvec_mm)
        forward, right, visual_down = (
            sum(row[index] * camera_metres[index] for index in range(3))
            for row in self._camera_to_body_frd
        )
        down = float(lidar_alt) if lidar_alt is not None else visual_down
        down += self._down_offset_m

        print(f"forward: {forward:.2f}m, right: {right:.2f}m, down: {down:.2f}m")
        return RelPosComplete(forward, right, down)

    def _save_frame_buffer_to_disk(self, file_path: str, fps: float = 30.0) -> None:
        with self._frame_buffer_lock:
            buffered_frames = list(self.frame_buffer)

        if not buffered_frames:
            return

        first_frame = buffered_frames[0]
        height, width = first_frame.shape[:2]
        is_color = len(first_frame.shape) == 3 and first_frame.shape[2] == 3

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = self._writer_factory(file_path, fourcc, fps, (width, height), is_color)

        if not writer.isOpened():
            writer.release()
            raise RuntimeError(f"Video writer for {file_path} could not be opened")

        try:
            for frame in buffered_frames:
                if len(frame.shape) == 2 and is_color:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                writer.write(frame)
        except BaseException as error:
            try:
                writer.release()
            except BaseException as release_error:
                try:
                    error.add_note(
                        _RELEASE_FAILURE_NOTE_PREFIX + _error_detail(release_error)
                    )
                except BaseException:
                    pass
            raise
        else:
            writer.release()

    def _record_frame_buffer(self, file_path: str, fps: float) -> None:
        try:
            self._save_frame_buffer_to_disk(file_path, fps)
        except BaseException as error:
            with self._recording_lock:
                self._recording_errors[file_path] = _recording_error_detail(error)
        finally:
            with self._recording_lock:
                self._recording_threads.pop(file_path, None)

    def save_frame_buffer_async(
        self, file_path: str = "frame_buffer.mp4", fps: float = 30.0
    ) -> threading.Thread:
        file_path = str(Path(file_path).expanduser().resolve(strict=False))
        with self._recording_lock:
            active = self._recording_threads.get(file_path)
            if active is not None:
                raise RuntimeError(f"Recording already in progress for {file_path}")
            self._recording_errors.pop(file_path, None)
            save_thread = threading.Thread(
                target=self._record_frame_buffer,
                args=(file_path, fps),
                daemon=True,
            )
            self._recording_threads[file_path] = save_thread
            save_thread.start()
        return save_thread

    @property
    def recording_errors(self) -> dict[str, str]:
        with self._recording_lock:
            return dict(self._recording_errors)

    def wait_for_recordings(self, timeout_s: float) -> bool:
        if not math.isfinite(timeout_s) or timeout_s < 0:
            raise ValueError("timeout_s must be finite and non-negative")
        deadline = time.monotonic() + timeout_s
        while True:
            with self._recording_lock:
                threads = tuple(self._recording_threads.values())
            if not threads:
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            threads[0].join(remaining)

    def precision_readiness(self, observation=None) -> CameraReadiness:
        if observation is not None:
            readiness = getattr(self.cm, "observation_readiness_reasons", None)
        else:
            readiness = getattr(self.cm, "precision_readiness_reasons", None)
        reasons = (
            list(readiness(observation) if observation is not None else readiness())
            if readiness is not None
            else ["manager does not expose precision readiness"]
        )
        if not self._mounting_verified:
            reasons.append("camera mounting is unverified")
        return CameraReadiness(ready=not reasons, reasons=tuple(reasons))

    def prepare_precision_readiness(self, *, timeout_s: float) -> CameraReadiness:
        """Acquire one bounded observation and leave the latest-frame worker running."""
        observation = self.cm.capture_observation_bounded(
            timeout_s=timeout_s,
            quality=4,
        )
        return self.precision_readiness(observation)

    def vector_from_observation_3d(
        self, observation, id: int, lidar_alt: Optional[float] = None
    ) -> RelPosComplete | None:
        readiness = self.precision_readiness(observation)
        if not readiness.ready:
            raise RuntimeError(
                "Camera observation is not precision-ready: "
                + "; ".join(readiness.reasons)
            )
        corners, ids, rejected = self.cm.get_coords(observation.frame)
        if ids is None or len(ids) == 0:
            self.cm.ensure_observation_fresh(observation)
            return None
        tvec_mm = self.cm.estimate_pose_3d(
            id,
            corners,
            ids,
            self.marker_size_mm,
            camera_matrix=observation.camera_matrix,
            distortion=observation.distortion,
        )
        if tvec_mm is None:
            self.cm.ensure_observation_fresh(observation)
            return None
        vector = self._body_marker_vector(tvec_mm, lidar_alt)
        self.cm.ensure_observation_fresh(observation)
        return vector

    def vec_to_marker_3d_bounded(
        self,
        id: int,
        *,
        timeout_s: float,
        after_sequence: int = 0,
        lidar_alt: Optional[float] = None,
        quality: int = 4,
    ):
        observation = self.cm.capture_observation_bounded(
            timeout_s=timeout_s,
            quality=quality,
            after_sequence=after_sequence,
        )
        self._buffer_frame(observation.frame)
        return (
            self.vector_from_observation_3d(observation, id, lidar_alt),
            observation.metadata,
        )

    def vec_to_marker(
        self, id: int, height_meters: Optional[float] = None
    ) -> RelativePosition | None:
        f = self.cm.capture_frame()
        self._buffer_frame(f)
        centers, ids, corners = self.cm.find_centers(f)  # type: ignore
        if centers:
            vc_meters = self.cm.find_target_center(
                id, centers, corners, ids, self.marker_size_mm
            )
            if vc_meters is not None:
                return RelativePosition(
                    -vc_meters[0], -vc_meters[1]
                )  # from testing x dir needs to be flipped so that from drone POV right is positive

        return

    def vec_to_marker_3d(
        self, id: int, lidar_alt: Optional[float] = None, quality: int = 4
    ) -> RelPosComplete | None:
        f = self.cm.capture_frame(quality=quality)
        self._buffer_frame(f)
        corners, ids, rejected = self.cm.get_coords(f)

        if ids is not None and len(ids) > 0:
            tvec_mm = self.cm.estimate_pose_3d(id, corners, ids, self.marker_size_mm)

            if tvec_mm is not None:
                return self._body_marker_vector(tvec_mm, lidar_alt)

        return None

    def vec_to_any_marker_3d(
        self, lidar_alt: Optional[float] = None, quality: int = 4
    ) -> tuple[RelPosComplete, int] | None:
        f = self.cm.capture_frame(quality=quality)
        self._buffer_frame(f)
        corners, ids, rejected = self.cm.get_coords(f)

        if ids is not None and len(ids) > 0:
            id = int(ids[0][0])
            tvec_mm = self.cm.estimate_pose_3d(id, corners, ids, self.marker_size_mm)

            if tvec_mm is not None:
                return self._body_marker_vector(tvec_mm, lidar_alt), id

        return None


if __name__ == "__main__":
    c = Camera(100)
    ARUCO_ID = 1
    while True:
        rp = c.vec_to_marker(1)
        if isinstance(rp, RelativePosition):
            print(f"x: {rp.x} meters, y: {rp.y} meters")
        else:
            print(f"no markers of id:{ARUCO_ID} detected")
        time.sleep(1)
