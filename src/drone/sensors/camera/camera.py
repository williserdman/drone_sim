from _camera_manager import CameraManager
from common_types import RelativePosition
from typing import Optional
import time


class Camera:
    def __init__(self, marker_size_mm: int):
        self.cm = CameraManager()
        self.marker_size_mm = marker_size_mm

    def vec_to_marker(
        self, id: int, height_meters: Optional[float] = None
    ) -> RelativePosition | None:
        f = self.cm.capture_frame()
        centers, ids, corners = self.cm.find_centers(f)  # type: ignore
        if centers:
            vc_meters = self.cm.find_target_center(
                id, centers, corners, ids, self.marker_size_mm
            )
            if vc_meters:
                return RelativePosition(vc_meters[0], vc_meters[1])

        return


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
