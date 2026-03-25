from ._camera_manager import CameraManager
from ...common_types import RelativePosition, RelPosComplete
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
            if vc_meters is not None:
                return RelativePosition(
                    -vc_meters[0], -vc_meters[1]
                )  # from testing x dir needs to be flipped so that from drone POV right is positive

        return

    def vec_to_marker_3d(self, id: int) -> RelPosComplete | None:
        f = self.cm.capture_frame()
        corners, ids, rejected = self.cm.get_coords(f)

        if ids is not None and len(ids) > 0:
            tvec_mm = self.cm.estimate_pose_3d(id, corners, ids, self.marker_size_mm)

            if tvec_mm is not None:
                # 1. Convert millimeters to meters
                cam_x_m = tvec_mm[0] / 1000.0
                cam_y_m = tvec_mm[1] / 1000.0
                cam_z_m = tvec_mm[2] / 1000.0

                # 2. Map Camera Frame -> Drone Body FRD Frame
                # ASSUMPTION: Camera is mounted flat on the drone belly, pointing straight down.
                # - Top of image (negative cam_y) is the drone's nose (Forward)
                # - Right of image (positive cam_x) is the drone's right wing (Right)
                # - Distance from lens to marker (positive cam_z) is straight down (Down)
                drone_forward = -cam_y_m
                drone_right = cam_x_m
                drone_down = cam_z_m

                return RelPosComplete(drone_forward, drone_right, drone_down)

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
