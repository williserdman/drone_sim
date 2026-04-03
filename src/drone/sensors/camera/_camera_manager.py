import numpy as np
import cv2  # type: ignore
import cv2.aruco as aruco  # type: ignore
import math

# from picamera2 import Picamera2  # type: ignore
import time
from datetime import datetime

one_over_root_2 = 1 / np.sqrt(2)


class CameraManager:
    def __init__(self) -> None:
        self.marker_size = 100  # millimeters (adjust as needed)

        # --- Default Camera Calibration for Raspberry Pi Camera v2 (480p) ---
        # Source: typical calibration for 640x480 with 62.2° x 48.8° FOV
        self.camera_matrix = np.array(
            [[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]]
        )
        self.camera_distortion = np.array([[-0.32, 0.1, 0.0, 0.0, 0.0]])

        # --------------------------------------------------------------
        # Initialize PiCamera2
        # --------------------------------------------------------------
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

        # self.picam2 = Picamera2()
        # this only gives partial sensor area
        # camera_width, camera_height, camera_frame_rate = 640, 480, 40

        # for full sensor area
        self.camera_width, self.camera_height, self.camera_frame_rate = 1640, 1232, 40

        # https://picamera.readthedocs.io/en/release-1.13/fov.html#sensor-modes
        # if we set 480p as the target resolution for the camera then we get a high framerate (way more that we can process)
        # instead we can try to set camera resolution to capture the entire sensor area then downsample it
        """ video_config = self.picam2.create_video_configuration(
            main={"size": (self.camera_width, self.camera_height), "format": "BGR888"},
            controls={
                "FrameDurationLimits": (
                    int(1e6 / self.camera_frame_rate),
                    int(1e6 / self.camera_frame_rate),
                )
            },
        )
        self.picam2.configure(video_config)
        self.picam2.set_controls({"ExposureValue": -1.5})
        self.picam2.start() """

        self.DISTANCE_THRESHOLD = 50  # pixels
        # dropper = ElectromagneticDropper()
        self.sample_ratio = 3

        # 1. FIX TYPO & CONVERT TO INTEGERS:
        # OpenCV needs integers for frame dimensions.
        # You previously had self.frame_height = camera_width / sample_ratio!
        self.frame_width = int(self.camera_width / self.sample_ratio)
        self.frame_height = int(self.camera_height / self.sample_ratio)

        # 2. FIX VIDEO WRITER:
        # Use the downsampled size, otherwise your MP4 will be corrupted.
        self.frame_size = (self.frame_width, self.frame_height)

        # 3. FIX OPTICAL CENTER:
        self.CAMERA_CENTER = [self.frame_width / 2.0, self.frame_height / 2.0]

        # 4. FIX CAMERA MATRIX:
        # Scale the 640x480 calibration matrix to match your new downsampled resolution
        scale_x = self.frame_width / 640.0
        scale_y = self.frame_height / 480.0

        self.camera_matrix[0, 0] *= scale_x  # Scale Focal Length X (fx)
        self.camera_matrix[1, 1] *= scale_y  # Scale Focal Length Y (fy)
        self.camera_matrix[0, 2] = self.CAMERA_CENTER[0]  # Set Optical Center X (cx)
        self.camera_matrix[1, 2] = self.CAMERA_CENTER[1]  # Set Optical Center Y (cy)

        self.one_over_root_2 = 1 / np.sqrt(2)

        output_filename = f'output_{datetime.now().strftime("%Y%m%d_%H%M%S")}.mp4'
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Codec for MP4
        fps = 40  # Frames per second

        # self.out = cv2.VideoWriter(output_filename, fourcc, fps, self.frame_size)
        # cap = cv2.VideoCapture("aruco/flight_videos/short_good_arucos.mp4")

        self.target_id = None

    def vector_to_center(self, center_of_aruco):
        v = np.subtract(self.CAMERA_CENTER, center_of_aruco)

        # to adjust for the front being in the corner of the camera
        """
        neg_45_deg_rotation = np.array(
            [[one_over_root_2, one_over_root_2], [-one_over_root_2, one_over_root_2]]
        ) """

        # rotate by -90 degrees (or 270 degrees) to align with drone frame if needed
        v = np.array([v[1], v[0]])
        return v

    def get_coords(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejected = aruco.detectMarkers(gray_frame, self.aruco_dict)

        return corners, ids, rejected

    def get_centers(self, corners):
        if len(corners) < 1:
            return
        out = []
        for corner_group in corners:
            # print("corner group", corner_group[0])
            corner_group = corner_group[0]
            xs = 0
            ys = 0
            for corner in corner_group:
                xs += corner[0]
                ys += corner[1]
            center = [xs / 4, ys / 4]
            # center = {"x": xs / 4, "y": ys / 4}
            out.append(center)
        return out

    def euclidian_distance(self, a, b) -> int:
        dif = np.subtract(a, b)
        # [0] - b[0]
        # ydif = a[1] - b[1]
        # print("x:", xdif, "y", ydif)
        return np.linalg.norm(dif)  # type: ignore abs(dif).sum()

    def draw_markers(self, frame, corners, ids, rejected):
        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(frame, corners)

            # Estimate pose of detected markers
            rvec_list_all, tvec_list_all, _objPoints = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.camera_distortion
            )

            # Use first marker
            rvec = rvec_list_all[0][0]
            tvec = tvec_list_all[0][0]

            """ # Draw axis
            cv2.drawFrameAxes(
                frame, self.camera_matrix, self.camera_distortion, rvec, tvec, 100
            ) """

            # Compute Euler angles
            rvec_flipped = rvec * -1
            tvec_flipped = tvec * -1
            rotation_matrix, _ = cv2.Rodrigues(rvec_flipped)
            realworld_tvec = np.dot(rotation_matrix, tvec_flipped)
            pitch, roll, yaw = self.rotationMatrixToEulerAngles(rotation_matrix)

            # Display coordinates + yaw angle
            tvec_str = "x=%4.0f  y=%4.0f  dir=%4.0f°" % (
                realworld_tvec[0],
                realworld_tvec[1],
                math.degrees(yaw),
            )
            cv2.putText(
                frame,
                tvec_str,
                (20, 460),
                cv2.FONT_HERSHEY_PLAIN,
                2,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

    def pixels_to_metric(self, corners_of_one_aruco, marker_size) -> float:
        corners_of_one_aruco = corners_of_one_aruco[0]
        dist_p = self.euclidian_distance(
            corners_of_one_aruco[0], corners_of_one_aruco[1]
        )
        return dist_p / marker_size

    def isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self, R):
        assert self.isRotationMatrix(R)
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

    def capture_frame(self, quality=4) -> np.ndarray:
        """Capture a BGR frame from the camera and downsample by sample_ratio."""

        scale = 1
        if quality == 3:
            scale = 0.75
        elif quality == 2:
            scale = 0.25
        elif quality == 1:
            scale = 0.10

        if not hasattr(self, "webcam"):
            self.webcam = cv2.VideoCapture(0)
            # Request full sensor/frame size to maximize captured scene.
            self.webcam.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            self.webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            # Try to force widest view by disabling digital zoom (if supported).
            if hasattr(cv2, "CAP_PROP_ZOOM"):
                self.webcam.set(cv2.CAP_PROP_ZOOM, 0)
        ok, frame = self.webcam.read()
        if not ok:
            raise RuntimeError("Failed to capture frame from webcam index 0")

        # Downsample using cv2.resize for better quality (less aliasing) than slicing
        width = frame.shape[1] // self.sample_ratio
        height = frame.shape[0] // self.sample_ratio

        small_w, small_h = int(width * scale), int(height * scale)
        frame = cv2.resize(frame, (small_w, small_h), interpolation=cv2.INTER_AREA)

        self.CAMERA_CENTER = [small_w / 2, small_h / 2]

        return frame

    def find_centers(self, frame) -> tuple[list | None, any, any] | tuple[list, list]:  # type: ignore
        corners, ids, rejected = self.get_coords(frame)
        # ids: [[1], [2], [3], ...]
        # corners: [[[1, 2, 3, 4]], [[5, 6, 7, 8]], [[9, 10, 11, 12]]] im assuming so that this is now flexible enough to have duplicate IDd markers
        if len(corners) >= 1:
            # print("corner count:", len(corners))
            # update target_id
            return self.get_centers(corners), ids, corners

        return [], [], []

    def find_target_center(
        self, target_id, centers: list, corners: list, ids: list, marker_size_mm: int
    ) -> list[float] | None:
        if target_id not in ids:
            return
        target_idx = list(ids).index(target_id)

        target_center = centers[target_idx]

        vc_p = self.vector_to_center(target_center)
        # Using corners[target_idx] ensures we use the scale of the target marker
        # Divide by pixels/mm to get mm.
        vc_meters = (
            vc_p / self.pixels_to_metric(corners[target_idx], marker_size_mm) / 1000
        )

        return vc_meters  # type: ignore

    """ def find_target_center(self, centers: list, ids: list, corners: list):
        if self.target_id is not None:
            self.target_idx = (
                list(ids).index(self.target_id) if self.target_id in ids else -1
            )
            target_center = centers[self.target_idx]
        else:
            if len(centers) > 0:
                target_center = centers[0]
                self.target_id = ids[0][0]
            else:
                self.target_id = None

        if target_center:
            corrector = self.pixels_to_metric(corners[0])
            vc_p = self.vector_to_center(target_center)  # in pixels!!
            vc_cm = vc_p * corrector / 100
            return vc_cm

        return None

        # self.send_relative_position_update(vc_cm)

        # print("target center", target_center)

        distance = self.euclidian_distance(target_center, self.CAMERA_CENTER)
        # print("distance:", distance)

        # TODO: write corners to file, write frame to file
        print(time.time() - start_time, ":", ids, corners)
        self.out.write(frame) """

    def end_all(self):
        if hasattr(self, "webcam"):
            self.webcam.release()
        # self.out.release()

    def step(self):
        s = time.time()
        frame = self.capture_frame(4)
        centers, ids, corners = self.find_centers(frame)  # type: ignore
        vc_cm = self.find_target_center(centers, ids, corners)  # type: ignore
        delta = time.time() - s
        return vc_cm, delta

    # https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    def estimate_pose_3d(
        self, target_id: int, corners: list, ids: list, marker_size_mm: int
    ) -> list[float] | None:
        """
        Uses OpenCV's pose estimation to calculate the 3D translation vector (tvec)
        from the camera lens to the ArUco marker.
        """
        if ids is None or target_id not in ids:
            return None

        target_idx = list(ids).index(target_id)
        target_corners = corners[target_idx]

        # estimatePoseSingleMarkers returns rotation (rvecs) and translation (tvecs)
        # Because we pass marker_size_mm, the resulting tvec will be in millimeters.
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            target_corners, marker_size_mm, self.camera_matrix, self.camera_distortion
        )

        # tvecs is returned as an array of shape (1, 1, 3) for a single marker
        tvec = tvecs[0][0]

        return tvec  # Returns [cam_x, cam_y, cam_z] in millimeters
