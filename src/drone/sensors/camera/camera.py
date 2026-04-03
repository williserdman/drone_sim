from ._camera_manager import CameraManager
from ...common_types import RelativePosition, RelPosComplete
from typing import Optional
from collections import deque
import threading
import cv2  # pyright: ignore[reportMissingImports]
import time


class Camera:
    def __init__(self, marker_size_mm: int):
        self.cm = CameraManager()
        self.marker_size_mm = marker_size_mm
        self.frame_buffer = deque(maxlen=10_000)
        self._frame_buffer_lock = threading.Lock()

    def _buffer_frame(self, frame) -> None:
        with self._frame_buffer_lock:
            self.frame_buffer.append(frame)

    def _save_frame_buffer_to_disk(self, file_path: str, fps: float = 30.0) -> None:
        with self._frame_buffer_lock:
            buffered_frames = list(self.frame_buffer)

            if not buffered_frames:
                return

            first_frame = buffered_frames[0]
            height, width = first_frame.shape[:2]
            is_color = len(first_frame.shape) == 3 and first_frame.shape[2] == 3

            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(file_path, fourcc, fps, (width, height), is_color)

            if not writer.isOpened():
                return

            try:
                for frame in buffered_frames:
                    if len(frame.shape) == 2 and is_color:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    writer.write(frame)
            finally:
                writer.release()

    def save_frame_buffer_async(
        self, file_path: str = "frame_buffer.mp4", fps: float = 30.0
    ) -> threading.Thread:
        save_thread = threading.Thread(
            target=self._save_frame_buffer_to_disk,
            args=(file_path, fps),
            daemon=True,
        )
        save_thread.start()
        return save_thread

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
        self, id: int, lidar_alt: Optional[float] = None, quality: Optional[int] = 4
    ) -> RelPosComplete | None:
        f = self.cm.capture_frame(quality)
        self._buffer_frame(f)
        corners, ids, rejected = self.cm.get_coords(f)

        if ids is not None and len(ids) > 0:
            tvec_mm = self.cm.estimate_pose_3d(id, corners, ids, self.marker_size_mm)

            if tvec_mm is not None:
                # 1. Convert camera translation from millimeters to meters
                cam_x_m = tvec_mm[0] / 1000.0
                cam_y_m = tvec_mm[1] / 1000.0
                cam_z_m = tvec_mm[2] / 1000.0

                # 2. Map Camera Frame -> Drone Body FRD Frame
                # MOUNTING ASSUMPTIONS:
                # - Top of image (-cam_y) is the drone's front -> Bottom (+cam_y) is back
                # - Right of image (+cam_x) is the drone's right -> Left (-cam_x) is left
                drone_forward = cam_x_m
                drone_right = cam_y_m

                # 3. Integrate LiDAR
                # Use highly accurate LiDAR for Z (Down) if available,
                # otherwise fallback to OpenCV's visual depth estimation.
                drone_down = lidar_alt if lidar_alt is not None else cam_z_m

                print(
                    f"forward: {drone_forward:.2f}m, right: {drone_right:.2f}m, down: {drone_down:.2f}m"
                )

                camera_offset = 0.1  # meters
                return RelPosComplete(drone_forward, drone_right, drone_down + 0.1)

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
