"""Resolved recording facts consumed by the artifact runtime."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from typing import Any, Mapping


_FRAME_INTERVAL_NS = 50_000_000


@dataclass(frozen=True)
class RecordingRuntimeConfig:
    expected_camera_frames: int
    synthetic_camera_ack: bool


def resolve_recording_runtime_config(document: Mapping[str, Any]) -> RecordingRuntimeConfig:
    """Derive recording count and transport policy from one resolved run document."""
    recording = document.get("recording")
    if not isinstance(recording, Mapping) or (
        recording.get("width_px"),
        recording.get("height_px"),
        recording.get("fps"),
        recording.get("encoding"),
    ) != (320, 240, 20, "rgb8"):
        raise ValueError("recording configuration must equal 320x240 rgb8 at 20 FPS")

    profile = document.get("runtime_profile", "phase2")
    if profile == "phase2":
        return RecordingRuntimeConfig(expected_camera_frames=40, synthetic_camera_ack=True)

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
    return RecordingRuntimeConfig(
        expected_camera_frames=duration_ns // _FRAME_INTERVAL_NS,
        synthetic_camera_ack=False,
    )


__all__ = ["RecordingRuntimeConfig", "resolve_recording_runtime_config"]
