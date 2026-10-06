"""Resolved recording facts consumed by the artifact runtime."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from typing import Any, Mapping, cast


_FRAME_INTERVAL_NS = 50_000_000


@dataclass(frozen=True)
class RecordingRuntimeConfig:
    expected_camera_frames: int
    synthetic_camera_ack: bool
    physical_run: bool = False
    width_px: int = 320
    height_px: int = 240
    fps: int = 20
    encoding: str = "rgb8"
    step_bytes: int = 960
    image_payload_bytes: int = 320 * 240 * 3
    ruleset_id: str = "descent_v1"

    @property
    def topics(self) -> tuple[str, ...]:
        from ._adapters.rosbag import BASE_TOPICS, COMPETITION_TOPICS, VFS_2027_TOPICS

        if self.ruleset_id == "competition_2027_v1":
            return VFS_2027_TOPICS
        return COMPETITION_TOPICS if self.ruleset_id == "competition_v1" else BASE_TOPICS


def resolve_recording_runtime_config(document: Mapping[str, Any]) -> RecordingRuntimeConfig:
    """Derive recording count and transport policy from one resolved run document."""
    recording = document.get("recording")
    if not isinstance(recording, Mapping):
        raise ValueError("recording configuration must be an object")
    geometry = (
        recording.get("width_px"),
        recording.get("height_px"),
        recording.get("fps"),
        recording.get("encoding"),
    )
    requested_scenario = document.get("scenario", "descent_v1")
    if requested_scenario == "competition_2027_v1" and geometry != (
        640,
        480,
        20,
        "rgb8",
    ):
        raise ValueError("competition_2027_v1 recording must equal 640x480 rgb8 at 20 FPS")
    if geometry not in ((320, 240, 20, "rgb8"), (640, 480, 20, "rgb8")):
        raise ValueError(
            "recording configuration must equal 320x240 or 640x480 rgb8 at 20 FPS"
        )
    width_px, height_px, fps, encoding = cast(tuple[int, int, int, str], geometry)
    ruleset_id = requested_scenario
    if ruleset_id == "competition_2027_v1":
        if (
            document.get("schema_version") != 2
            or document.get("mission") != "configured"
            or (width_px, height_px) != (640, 480)
            or document.get("runtime_profile") != "phase3"
        ):
            raise ValueError(
                "competition_2027_v1 requires version 2 configured mission and "
                "640x480 rgb8 at 20 FPS in phase3"
            )
    if ruleset_id == "competition_v1" and (width_px, height_px) != (640, 480):
        raise ValueError("competition_v1 recording must equal 640x480 rgb8 at 20 FPS")
    if ruleset_id not in {"competition_v1", "competition_2027_v1"}:
        ruleset_id = "descent_v1"

    profile = document.get("runtime_profile", "phase2")
    if profile == "phase2":
        return RecordingRuntimeConfig(
            expected_camera_frames=40,
            synthetic_camera_ack=True,
            physical_run=False,
            width_px=width_px,
            height_px=height_px,
            fps=fps,
            encoding=encoding,
            step_bytes=width_px * 3,
            image_payload_bytes=width_px * height_px * 3,
            ruleset_id=ruleset_id,
        )

    simulation = document.get("simulation")
    if not isinstance(simulation, Mapping):
        raise ValueError("physical runtime requires simulation duration")
    duration = simulation.get("duration_sim_seconds")
    if isinstance(duration, bool) or not isinstance(duration, (int, float)):
        raise ValueError("simulation duration must be a positive exact-grid number")
    try:
        decimal_duration = Decimal(str(duration))
    except InvalidOperation as error:
        raise ValueError("simulation duration must be a positive exact-grid number") from error
    if not decimal_duration.is_finite() or decimal_duration <= 0:
        raise ValueError("simulation duration must be a positive exact-grid number")
    numerator, denominator = decimal_duration.as_integer_ratio()
    nanoseconds_numerator = numerator * 1_000_000_000
    if nanoseconds_numerator % denominator:
        raise ValueError("simulation duration must resolve to integer nanoseconds")
    duration_ns = nanoseconds_numerator // denominator
    if duration_ns % _FRAME_INTERVAL_NS:
        raise ValueError("simulation duration must contain an integral camera frame count")
    if ruleset_id == "competition_2027_v1" and duration_ns < 780_000_000_000:
        raise ValueError("competition_2027_v1 requires at least 780 public seconds")
    return RecordingRuntimeConfig(
        expected_camera_frames=duration_ns // _FRAME_INTERVAL_NS,
        synthetic_camera_ack=False,
        physical_run=True,
        width_px=width_px,
        height_px=height_px,
        fps=fps,
        encoding=encoding,
        step_bytes=width_px * 3,
        image_payload_bytes=width_px * height_px * 3,
        ruleset_id=ruleset_id,
    )


__all__ = ["RecordingRuntimeConfig", "resolve_recording_runtime_config"]
