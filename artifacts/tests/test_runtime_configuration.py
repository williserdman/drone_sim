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
    assert (contract.observer_width_px, contract.observer_height_px) == (320, 240)


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
    assert (contract.observer_width_px, contract.observer_height_px) == (640, 480)
    assert contract.ruleset_id == "competition_v1"
    assert contract.topics == COMPETITION_TOPICS


def test_search_delivery_profile_selects_physical_topics_and_240_second_grid():
    """Falling back to descent would omit payload and mission evidence."""
    from artifacts._adapters.rosbag import COMPETITION_TOPICS

    contract = resolve_recording_runtime_config(
        {
            "runtime_profile": "phase3",
            "world": "search_delivery",
            "vehicle": "iris_search_delivery",
            "mission": "configured",
            "scenario": "search_delivery_v1",
            "recording": {
                "width_px": 640,
                "height_px": 480,
                "observer_width_px": 1280,
                "observer_height_px": 960,
                "fps": 20,
                "encoding": "rgb8",
            },
            "simulation": {
                "duration_sim_seconds": 240,
                "public_epoch_native_sim_seconds": 90,
            },
        }
    )

    assert contract.expected_camera_frames == 4_800
    assert contract.ruleset_id == "search_delivery_v1"
    assert (contract.width_px, contract.height_px) == (640, 480)
    assert (contract.observer_width_px, contract.observer_height_px) == (1280, 960)
    assert contract.topics == COMPETITION_TOPICS


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda value: value.update(runtime_profile="phase2"), "runtime_profile"),
        (lambda value: value.update(world="competition_mission"), "world"),
        (lambda value: value.update(vehicle="iris_competition"), "vehicle"),
        (lambda value: value.update(mission="comp2026_auto"), "mission"),
        (lambda value: value["recording"].update(width_px=320), "640x480"),
        (lambda value: value["recording"].update(observer_width_px=640), "1280x960"),
        (lambda value: value["recording"].pop("observer_width_px"), "1280x960"),
        (lambda value: value["simulation"].update(duration_sim_seconds=239.95), "240"),
        (
            lambda value: value["simulation"].update(
                public_epoch_native_sim_seconds=89.95
            ),
            "90",
        ),
    ],
)
def test_search_delivery_profile_rejects_noncanonical_selection(mutation, message):
    """Artifacts must not apply the search contract to a lookalike configuration."""
    document = {
        "runtime_profile": "phase3",
        "world": "search_delivery",
        "vehicle": "iris_search_delivery",
        "mission": "configured",
        "scenario": "search_delivery_v1",
        "recording": {
            "width_px": 640,
            "height_px": 480,
            "observer_width_px": 1280,
            "observer_height_px": 960,
            "fps": 20,
            "encoding": "rgb8",
        },
        "simulation": {
            "duration_sim_seconds": 240,
            "public_epoch_native_sim_seconds": 90,
        },
    }
    mutation(document)

    with pytest.raises(ValueError, match=message):
        resolve_recording_runtime_config(document)


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
