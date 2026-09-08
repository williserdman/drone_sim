import numpy as np
import cv2  # type: ignore
import cv2.aruco as aruco  # type: ignore
import math

# from picamera2 import Picamera2  # type: ignore
import time
from dataclasses import dataclass
from contextlib import contextmanager
import json
from pathlib import Path
import threading

one_over_root_2 = 1 / np.sqrt(2)


@dataclass(frozen=True)
class FrameMetadata:
    sequence: int
    exposure_timestamp_ns: int | None
    receipt_timestamp_ns: int
    exposure_age_ns: int | None
    exposure_age_bounded: bool
    raw_image_size_px: tuple[int, int]
    image_size_px: tuple[int, int]


@dataclass(frozen=True)
class FrameObservation:
    frame: np.ndarray
    metadata: FrameMetadata
    camera_matrix: np.ndarray | None
    distortion: np.ndarray


class CameraManager:
    def __init__(
        self,
        frame_source=None,
        calibration_path=None,
        *,
        clock=time.monotonic_ns,
        max_exposure_age_ns=None,
    ) -> None:
        self.marker_size = 100  # millimeters (adjust as needed)
        self.frame_source = frame_source
        self._clock = clock
        if max_exposure_age_ns is not None:
            if not isinstance(max_exposure_age_ns, int) or max_exposure_age_ns <= 0:
                raise ValueError("max_exposure_age_ns must be a positive integer")
        self.max_exposure_age_ns = max_exposure_age_ns
        self.last_frame_timestamp = None
        self.last_frame_metadata: FrameMetadata | None = None
        self._frame_sequence = 0
        self._capture_lock = threading.Lock()
        self._state_condition = threading.Condition()
        self._latest_observation: FrameObservation | None = None
        self._acquisition_thread: threading.Thread | None = None
        self._acquisition_stop = threading.Event()
        self._acquisition_error: Exception | None = None
        self._acquisition_quality = 4

        # --- Default Camera Calibration for Raspberry Pi Camera v2 (480p) ---
        # Source: typical calibration for 640x480 with 62.2° x 48.8° FOV
        """ self.camera_matrix = np.array(
            [[620.0, 0.0, 320.0], [0.0, 620.0, 240.0], [0.0, 0.0, 1.0]]
        )
        self.camera_distortion = np.array([[-0.32, 0.1, 0.0, 0.0, 0.0]]) """

        # --------------------------------------------------------------
        # Initialize PiCamera2
        # --------------------------------------------------------------
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

        # Load calibration data from JSON
        json_file_path = (
            Path(calibration_path)
            if calibration_path is not None
            else Path(__file__).with_name("calibration.json")
        )
        with open(json_file_path, "r") as file:
            json_data = json.load(file)

        try:
            self.cam_mat = np.asarray(json_data["camera_matrix"], dtype=float)
            self.cam_dist = np.asarray(json_data["dist_coeff"], dtype=float)
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("camera calibration values must be numeric") from error
        if self.cam_mat.shape != (3, 3) or not np.all(np.isfinite(self.cam_mat)):
            raise ValueError("camera_matrix must be a finite 3x3 matrix")
        if not np.allclose(self.cam_mat[2], [0.0, 0.0, 1.0], atol=1e-12):
            raise ValueError("camera_matrix must have OpenCV homogeneous bottom row")
        if not math.isclose(float(self.cam_mat[0, 1]), 0.0, abs_tol=1e-12) or not math.isclose(
            float(self.cam_mat[1, 0]), 0.0, abs_tol=1e-12
        ):
            raise ValueError("camera_matrix skew terms are unsupported")
        if self.cam_dist.size == 0 or not np.all(np.isfinite(self.cam_dist)):
            raise ValueError("dist_coeff must contain finite values")
        if self.cam_dist.ndim != 2 or 1 not in self.cam_dist.shape:
            raise ValueError("dist_coeff must be a row or column vector")
        if self.cam_dist.size not in (4, 5, 8, 12, 14):
            raise ValueError("dist_coeff has an unsupported OpenCV coefficient count")
        if self.cam_mat[0, 0] <= 0 or self.cam_mat[1, 1] <= 0:
            raise ValueError("camera_matrix focal lengths must be positive")

        self._calibration_dimensions_verified = (
            json_data.get("dimensions_verified") is True
        )
        width = json_data.get("image_width_px")
        height = json_data.get("image_height_px")
        if self._calibration_dimensions_verified:
            if not isinstance(width, int) or width <= 0:
                raise ValueError("verified image_width_px must be a positive integer")
            if not isinstance(height, int) or height <= 0:
                raise ValueError("verified image_height_px must be a positive integer")
            self.calibrated_image_size_px = (width, height)
        else:
            self.calibrated_image_size_px = None

        # self.picam2 = Picamera2()
        # this only gives partial sensor area
        # camera_width, camera_height, camera_frame_rate = 640, 480, 40

        # for full sensor area
        # self.camera_width, self.camera_height, self.camera_frame_rate = 1640, 1232, 40

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

        # for webcam
        self.camera_width, self.camera_height, self.camera_frame_rate = 1920, 1080, 30
        self.camera_width, self.camera_height, self.camera_frame_rate = 1280, 720, 30

        """
        if not hasattr(self, "webcam"):
            self.webcam = cv2.VideoCapture(0)
            # Request full sensor/frame size to maximize captured scene.
            self.webcam.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
            self.webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
            # Try to force widest view by disabling digital zoom (if supported).
            if hasattr(cv2, "CAP_PROP_ZOOM"):
                self.webcam.set(cv2.CAP_PROP_ZOOM, 0)
        """
        if self.frame_source is None:
            self.webcam = cv2.VideoCapture(0)
        ### ENABLE FOR LOW FOV CAMERA
        # self.webcam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        # self.webcam.set(cv2.CAP_PROP_EXPOSURE, 40)

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

        self.one_over_root_2 = 1 / np.sqrt(2)

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

    def _capture_candidate(self, quality):
        if quality not in (1, 2, 3, 4):
            raise ValueError("quality must be one of 1, 2, 3, or 4")
        scale = {1: 0.10, 2: 0.25, 3: 0.75, 4: 1.0}[quality]

        if self.frame_source is None:
            ok, raw_frame = self.webcam.read()
            if not ok:
                raise RuntimeError("Failed to capture frame from webcam index 0")
            exposure_timestamp_ns = None
        else:
            raw_frame = self.frame_source.capture_frame(quality=quality)
            exposure_timestamp_ns = getattr(
                self.frame_source, "last_timestamp_ns", None
            )
            if exposure_timestamp_ns is None:
                raise RuntimeError("Injected frame source must expose last_timestamp_ns")

        receipt_timestamp_ns = self._clock()
        raw_size = (raw_frame.shape[1], raw_frame.shape[0])
        output_frame = raw_frame.copy()
        if scale != 1:
            base_width = raw_frame.shape[1] // self.sample_ratio
            base_height = raw_frame.shape[0] // self.sample_ratio
            output_size = (int(base_width * scale), int(base_height * scale))
            if output_size[0] <= 0 or output_size[1] <= 0:
                raise RuntimeError("Requested resize produces an empty image")
            output_frame = cv2.resize(
                raw_frame, output_size, interpolation=cv2.INTER_AREA
            )

        output_size = (output_frame.shape[1], output_frame.shape[0])
        scaled_matrix = None
        if self.calibrated_image_size_px == raw_size:
            scaled_matrix = self.cam_mat.copy()
            scaled_matrix[0, :] *= output_size[0] / raw_size[0]
            scaled_matrix[1, :] *= output_size[1] / raw_size[1]
            scaled_matrix[2, :] = self.cam_mat[2, :]
            scaled_matrix.setflags(write=False)
        distortion = self.cam_dist.copy()
        distortion.setflags(write=False)
        return (
            output_frame,
            exposure_timestamp_ns,
            receipt_timestamp_ns,
            raw_size,
            output_size,
            scaled_matrix,
            distortion,
        )

    @contextmanager
    def _state_lock_after_clock_sample(self):
        while True:
            handover_timestamp_ns = self._clock()
            if self._state_condition.acquire(blocking=False):
                break
            with self._state_condition:
                pass
        try:
            yield handover_timestamp_ns
        finally:
            self._state_condition.release()

    def _accept_candidate(self, candidate, *, cancel_if_stopped=False):
        (
            frame,
            exposure_timestamp_ns,
            receipt_timestamp_ns,
            raw_size,
            output_size,
            camera_matrix,
            distortion,
        ) = candidate
        exposure_age_ns = None
        exposure_age_bounded = False
        with self._state_lock_after_clock_sample() as handover_timestamp_ns:
            if cancel_if_stopped and self._acquisition_stop.is_set():
                return None
            if exposure_timestamp_ns is not None:
                if not isinstance(exposure_timestamp_ns, int):
                    raise RuntimeError("Frame exposure timestamp must be an integer")
                if (
                    self.last_frame_metadata is not None
                    and self.last_frame_metadata.exposure_timestamp_ns is not None
                    and exposure_timestamp_ns
                    <= self.last_frame_metadata.exposure_timestamp_ns
                ):
                    raise RuntimeError(
                        "Frame exposure timestamp must be newer than the last frame"
                    )
                exposure_age_ns = receipt_timestamp_ns - exposure_timestamp_ns
                if exposure_age_ns < 0:
                    raise RuntimeError("Frame exposure timestamp is in the future")
                if self.max_exposure_age_ns is not None:
                    if exposure_age_ns > self.max_exposure_age_ns:
                        raise RuntimeError(
                            "Frame exposure is older than the configured acquisition limit"
                        )
                    handover_age_ns = handover_timestamp_ns - exposure_timestamp_ns
                    if handover_age_ns < 0:
                        raise RuntimeError("Frame exposure timestamp is in the future")
                    if handover_age_ns > self.max_exposure_age_ns:
                        raise RuntimeError(
                            "Frame exposure is older than the configured acquisition limit"
                        )
                    exposure_age_bounded = True

            sequence = self._frame_sequence + 1
            metadata = FrameMetadata(
                sequence=sequence,
                exposure_timestamp_ns=exposure_timestamp_ns,
                receipt_timestamp_ns=receipt_timestamp_ns,
                exposure_age_ns=exposure_age_ns,
                exposure_age_bounded=exposure_age_bounded,
                raw_image_size_px=raw_size,
                image_size_px=output_size,
            )
            observation = FrameObservation(
                frame=frame,
                metadata=metadata,
                camera_matrix=camera_matrix,
                distortion=distortion,
            )
            self._frame_sequence = sequence
            self.last_frame_timestamp = exposure_timestamp_ns
            self.last_frame_metadata = metadata
            self._latest_observation = observation
            self.CAMERA_CENTER = [output_size[0] / 2, output_size[1] / 2]
            self._state_condition.notify_all()
            return observation

    def capture_observation(self, quality=4) -> FrameObservation:
        with self._capture_lock:
            return self._accept_candidate(self._capture_candidate(quality))

    def capture_frame(self, quality=4) -> np.ndarray:
        """Capture a frame and retain source exposure and local receipt metadata."""
        return self.capture_observation(quality).frame

    def _acquisition_loop(self) -> None:
        try:
            while not self._acquisition_stop.is_set():
                with self._capture_lock:
                    candidate = self._capture_candidate(self._acquisition_quality)
                    if self._acquisition_stop.is_set():
                        break
                    self._accept_candidate(candidate, cancel_if_stopped=True)
        except Exception as error:
            with self._state_condition:
                self._acquisition_error = error
                self._state_condition.notify_all()
        finally:
            with self._state_condition:
                self._state_condition.notify_all()

    def start_acquisition(self, *, quality=4) -> None:
        if quality not in (1, 2, 3, 4):
            raise ValueError("quality must be one of 1, 2, 3, or 4")
        with self._state_condition:
            if self._acquisition_thread is not None and self._acquisition_thread.is_alive():
                if quality != self._acquisition_quality:
                    raise RuntimeError("Acquisition worker is already using another quality")
                return
            self._acquisition_quality = quality
            self._acquisition_error = None
            self._acquisition_stop.clear()
            self._acquisition_thread = threading.Thread(
                target=self._acquisition_loop,
                name="camera-latest-frame",
                daemon=True,
            )
            self._acquisition_thread.start()

    def _ensure_handover_fresh(self, observation: FrameObservation) -> None:
        timestamp = observation.metadata.exposure_timestamp_ns
        if timestamp is None or self.max_exposure_age_ns is None:
            raise RuntimeError("Frame exposure age is not bounded")
        age = self._clock() - timestamp
        if age < 0:
            raise RuntimeError("Frame exposure timestamp is in the future")
        if age > self.max_exposure_age_ns:
            raise RuntimeError("Frame exposure is older than the acquisition limit")

    def ensure_observation_fresh(self, observation: FrameObservation) -> None:
        self._ensure_handover_fresh(observation)

    def latest_observation(
        self, *, after_sequence: int, timeout_s: float
    ) -> FrameObservation:
        if not isinstance(after_sequence, int) or after_sequence < 0:
            raise ValueError("after_sequence must be a non-negative integer")
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("timeout_s must be finite and positive")
        deadline = time.monotonic() + timeout_s
        while True:
            with self._state_condition:
                observation = self._latest_observation
                if (
                    observation is not None
                    and observation.metadata.sequence > after_sequence
                ):
                    pass
                elif self._acquisition_error is not None:
                    raise RuntimeError("Camera acquisition worker failed") from self._acquisition_error
                else:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("Camera frame acquisition timed out")
                    self._state_condition.wait(remaining)
                    continue
            self._ensure_handover_fresh(observation)
            return observation

    def capture_observation_bounded(
        self, *, timeout_s: float, quality=4, after_sequence=None
    ) -> FrameObservation:
        if after_sequence is None:
            with self._state_condition:
                after_sequence = self._frame_sequence
        self.start_acquisition(quality=quality)
        try:
            return self.latest_observation(
                after_sequence=after_sequence, timeout_s=timeout_s
            )
        except TimeoutError:
            self._cancel_acquisition()
            raise

    def capture_frame_bounded(self, *, timeout_s: float, quality=4) -> np.ndarray:
        return self.capture_observation_bounded(
            timeout_s=timeout_s, quality=quality
        ).frame

    def stop_acquisition(self, *, timeout_s: float) -> bool:
        if not math.isfinite(timeout_s) or timeout_s < 0:
            raise ValueError("timeout_s must be finite and non-negative")
        self._cancel_acquisition()
        with self._state_condition:
            thread = self._acquisition_thread
        if thread is None:
            return True
        thread.join(timeout_s)
        return not thread.is_alive()

    def _cancel_acquisition(self) -> None:
        with self._state_condition:
            self._acquisition_stop.set()
            self._state_condition.notify_all()

    def observation_readiness_reasons(
        self, observation: FrameObservation
    ) -> tuple[str, ...]:
        reasons = []
        if not self._calibration_dimensions_verified:
            reasons.append("calibrated image dimensions are unknown or unverified")
        if (
            self.calibrated_image_size_px is not None
            and observation.metadata.raw_image_size_px
            != self.calibrated_image_size_px
        ):
            reasons.append("captured image dimensions do not match calibration")
        if not observation.metadata.exposure_age_bounded:
            reasons.append("frame exposure age is not bounded")
        else:
            try:
                self._ensure_handover_fresh(observation)
            except RuntimeError as error:
                if "older than" in str(error):
                    reasons.append(
                        "latest frame exposure is older than the acquisition limit"
                    )
                else:
                    reasons.append(str(error))
        return tuple(reasons)

    def precision_readiness_reasons(self) -> tuple[str, ...]:
        with self._state_condition:
            observation = self._latest_observation
        if observation is None:
            reasons = []
            if not self._calibration_dimensions_verified:
                reasons.append("calibrated image dimensions are unknown or unverified")
            reasons.append("frame exposure age is not bounded")
            return tuple(reasons)
        return self.observation_readiness_reasons(observation)

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
        self,
        target_id: int,
        corners: list,
        ids: list,
        marker_size_mm: int,
        *,
        camera_matrix=None,
        distortion=None,
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
            target_corners,
            marker_size_mm,
            self.cam_mat if camera_matrix is None else camera_matrix,
            self.cam_dist if distortion is None else distortion,
        )

        # tvecs is returned as an array of shape (1, 1, 3) for a single marker
        tvec = tvecs[0][0]

        return tvec  # Returns [cam_x, cam_y, cam_z] in millimeters
