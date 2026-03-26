from src.drone.common_types import RelativePosition, RelPosComplete
from src.drone.sensors.camera.camera import Camera
import time

""" if __name__ == "__main__":
    c = Camera(100)
    ARUCO_ID = 1
    while True:
        rp = c.vec_to_marker(1)
        if isinstance(rp, RelativePosition):
            print(f"x: {rp.x} meters, y: {rp.y} meters")
        else:
            print(f"no markers of id:{ARUCO_ID} detected")
        time.sleep(1) """

if __name__ == "__main__":
    c = Camera(100)  # marker size is 100mm
    ARUCO_ID = 2
    counter = 1
    begin = time.time()
    while True:
        rp = c.vec_to_marker_3d(ARUCO_ID)
        if isinstance(rp, RelPosComplete):
            # Print to 2 decimal places (centimeter resolution)
            print(f"Target: Fwd: {rp.x:.2f}m, Right: {rp.y:.2f}m, Down: {rp.z:.2f}m")
        else:
            pass
            # print(f"no markers of id:{ARUCO_ID} detected")

        if counter % 100 == 0:
            counter = 0
            now = time.time()
            print(f"100 frames in {now-begin}s, {100/(now-begin)} fps")
            begin = now
        counter += 1
        # time.sleep(0.05)
