from __future__ import annotations

import pytest

from artifacts.runtime_configuration import resolve_recording_runtime_config


def test_omitted_profile_preserves_phase2_forty_frame_ack_contract():
    """Treating an omitted profile as production would break Phase 2 replay."""
    contract = resolve_recording_runtime_config(
        {"recording": {"width_px": 320, "height_px": 240, "fps": 20, "encoding": "rgb8"}}
    )

    assert contract.expected_camera_frames == 40
    assert contract.synthetic_camera_ack is True


def test_physical_profile_derives_frame_count_and_never_requires_camera_ack():
    """A production duration must not inherit the synthetic 40-frame backpressure seam."""
    contract = resolve_recording_runtime_config(
        {
            "runtime_profile": "phase3",
            "recording": {"width_px": 320, "height_px": 240, "fps": 20, "encoding": "rgb8"},
            "simulation": {"duration_sim_seconds": 0.15},
        }
    )

    assert contract.expected_camera_frames == 3
    assert contract.synthetic_camera_ack is False


@pytest.mark.parametrize("duration", [0, 0.075, True, float("nan")])
def test_physical_profile_rejects_nonpositive_or_off_grid_duration(duration):
    """Rounding an invalid duration could certify the wrong camera inventory."""
    with pytest.raises(ValueError, match="duration"):
        resolve_recording_runtime_config(
            {
                "runtime_profile": "phase3",
                "recording": {"width_px": 320, "height_px": 240, "fps": 20, "encoding": "rgb8"},
                "simulation": {"duration_sim_seconds": duration},
            }
        )
