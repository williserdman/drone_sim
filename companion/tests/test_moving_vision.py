from __future__ import annotations

from dataclasses import dataclass
import threading
import time

import numpy as np

from drone_sim_companion.moving_precision import AttitudeEvidence, RangeEvidence
from drone_sim_companion.moving_vision import MovingVision


@dataclass(frozen=True)
class Metadata:
    sequence: int
    exposure_timestamp_ns: int


@dataclass(frozen=True)
class Frame:
    metadata: Metadata


class CameraManager:
    def __init__(self) -> None:
        self.started = False
        self.closed = False
        self.frames = [Frame(Metadata(1, 1_000_000_000)), Frame(Metadata(2, 1_050_000_000))]

    def start_acquisition(self, *, quality: int) -> None:
        assert quality == 4
        self.started = True

    def latest_observation(self, *, after_sequence: int, timeout_s: float):
        assert timeout_s <= 0.1
        for frame in self.frames:
            if frame.metadata.sequence > after_sequence:
                return frame
        raise TimeoutError

    def stop_acquisition(self, *, timeout_s: float) -> bool:
        self.closed = True
        return True


class Camera:
    def __init__(self) -> None:
        self.cm = CameraManager()

    def detect_markers_body_frd(self, frame, range_m):
        assert frame.metadata.sequence in {1, 2}
        return [(7, (0.1, -0.2, range_m))]


def test_worker_keeps_only_latest_coherent_observation_and_closes() -> None:
    camera = Camera()
    evidence = {
        "range": RangeEvidence(4.8, 1_050_000_000),
        "attitude": AttitudeEvidence((0.01, -0.02, 0.0), 1_050_000_000),
    }
    vision = MovingVision(
        camera,
        latest_range=lambda: evidence["range"],
        latest_attitude=lambda: evidence["attitude"],
    )

    vision.start()
    deadline = time.monotonic() + 1
    while (latest := vision.latest()) is None and time.monotonic() < deadline:
        time.sleep(0.001)

    assert latest is not None
    assert latest.camera_sequence == 2
    assert latest.target_body_frd == (0.1, -0.2, 4.8)
    assert vision.close(0.5)
    assert camera.cm.started and camera.cm.closed


def test_worker_does_not_issue_vehicle_commands() -> None:
    camera = Camera()
    vision = MovingVision(
        camera,
        latest_range=lambda: RangeEvidence(4.8, 1_000_000_000),
        latest_attitude=lambda: AttitudeEvidence((0.0, 0.0, 0.0), 1_000_000_000),
    )
    assert not hasattr(vision, "vehicle")
