from __future__ import annotations

import pytest

from artifacts.runtime_configuration import (
    RecordingRuntimeConfig,
    resolve_recording_runtime_config,
)


def test_legacy_phase2_contract_constructor_defaults_to_nonphysical_validation():
    assert RecordingRuntimeConfig(40, True).physical_run is False


def test_omitted_profile_preserves_phase2_forty_frame_ack_contract():
    """Treating an omitted profile as production would break Phase 2 replay."""
    contract = resolve_recording_runtime_config(
        {"recording": {"width_px": 320, "height_px": 240, "fps": 20, "encoding": "rgb8"}}
    )

    assert contract.expected_camera_frames == 40
    assert contract.synthetic_camera_ack is True
    assert contract.physical_run is False


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
    assert contract.physical_run is True


def test_competition_profile_preserves_resolved_640x480_geometry_and_topics():
    """Hard-coding descent geometry would reject or mis-size competition evidence."""
    from artifacts._adapters.rosbag import COMPETITION_TOPICS

    contract = resolve_recording_runtime_config(
        {
            "runtime_profile": "phase3",
            "mission": "comp2026_auto",
            "scenario": "competition_v1",
            "recording": {
                "width_px": 640,
                "height_px": 480,
                "fps": 20,
                "encoding": "rgb8",
            },
            "simulation": {"duration_sim_seconds": 600},
        }
    )

    assert contract.expected_camera_frames == 12_000
    assert (contract.width_px, contract.height_px, contract.step_bytes) == (
        640,
        480,
        1_920,
    )
    assert contract.image_payload_bytes == 640 * 480 * 3
    assert contract.ruleset_id == "competition_v1"
    assert contract.topics == COMPETITION_TOPICS


@pytest.mark.parametrize("duration", [780, 900])
def test_vfs_2027_profile_records_full_public_window_and_judge_streams(duration):
    from artifacts._adapters.rosbag import VFS_2027_TOPICS

    contract = resolve_recording_runtime_config(
        {
            "runtime_profile": "phase3",
            "mission": "configured",
            "mission_plan": {
                "schema_version": 2,
                "steps": [{"tool": "begin_phase", "args": {"phase": "FM1"}}],
            },
            "scenario": "competition_2027_v1",
            "recording": {
                "width_px": 640,
                "height_px": 480,
                "fps": 20,
                "encoding": "rgb8",
            },
            "simulation": {"duration_sim_seconds": duration},
        }
    )

    assert contract.expected_camera_frames == duration * 20
    assert contract.ruleset_id == "competition_2027_v1"
    assert contract.topics == VFS_2027_TOPICS


def test_vfs_2027_profile_rejects_wrong_geometry_or_public_window():
    base = {
        "runtime_profile": "phase3",
        "mission": "configured",
        "mission_plan": {
            "schema_version": 2,
            "steps": [{"tool": "begin_phase", "args": {"phase": "FM1"}}],
        },
        "scenario": "competition_2027_v1",
        "recording": {
            "width_px": 640,
            "height_px": 480,
            "fps": 20,
            "encoding": "rgb8",
        },
        "simulation": {"duration_sim_seconds": 780},
    }
    for mutation in (
        {"recording": {**base["recording"], "width_px": 320}},
        {"simulation": {"duration_sim_seconds": 600}},
        {"mission_plan": {"schema_version": 1, "steps": []}},
        {"mission_plan": None},
        {"mission": "comp2026_auto"},
    ):
        with pytest.raises(ValueError, match="competition_2027_v1"):
            resolve_recording_runtime_config({**base, **mutation})

    missing_plan = dict(base)
    missing_plan.pop("mission_plan")
    with pytest.raises(ValueError, match="competition_2027_v1"):
        resolve_recording_runtime_config(missing_plan)


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
