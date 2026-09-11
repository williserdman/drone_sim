import math

import pytest

from drone.sensors.lidar.clearance import (
    AttitudeSample,
    ClearanceCalibration,
    ClearanceConfigurationError,
    ClearanceUnavailableError,
    project_vertical_clearance,
)
from drone.sensors.lidar.lidar import LidarSample


def calibration(**overrides):
    settings = {
        "beam_direction_body_frd": (0.0, 0.0, 1.0),
        "measured_reference_offset_body_frd_m": (0.0, 0.0, 0.2),
        "lidar_mounting_offset_already_applied": True,
        "max_tilt_rad": math.radians(30.0),
        "max_age_seconds": 0.5,
        "max_skew_seconds": 0.1,
        "locally_horizontal_planar_surface": True,
    }
    settings.update(overrides)
    return ClearanceCalibration(**settings)


def test_level_projection_preserves_range_and_attitude_metadata():
    range_sample = LidarSample(
        distance_m=2.0,
        sampled_at=10.0,
        sequence=4,
        invalidation_generation=2,
    )
    attitude_sample = AttitudeSample(
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=1.2,
        sampled_at=10.05,
        sequence=8,
    )

    projected = project_vertical_clearance(
        range_sample,
        attitude_sample,
        calibration(),
        now=10.1,
    )

    assert projected.projected_clearance_m == pytest.approx(2.2)
    assert projected.range_sample is range_sample
    assert projected.attitude_sample is attitude_sample


@pytest.mark.parametrize(
    "override",
    [
        {"beam_direction_body_frd": (0.0, 0.0, 2.0)},
        {"beam_direction_body_frd": (0.0, 0.0, -1.0)},
        {"beam_direction_body_frd": (0.0, True, 1.0)},
        {"measured_reference_offset_body_frd_m": (0.0, 0.0, math.nan)},
        {"lidar_mounting_offset_already_applied": False},
        {"lidar_mounting_offset_already_applied": "yes"},
        {"max_tilt_rad": 0.0},
        {"max_tilt_rad": math.pi / 2},
        {"max_age_seconds": 0.0},
        {"max_skew_seconds": 0.0},
        {"locally_horizontal_planar_surface": False},
        {"locally_horizontal_planar_surface": 1},
    ],
)
def test_calibration_rejects_implicit_or_unsupported_geometry(override):
    with pytest.raises(ClearanceConfigurationError):
        calibration(**override)


def test_stale_range_is_unavailable():
    with pytest.raises(ClearanceUnavailableError, match="range sample is stale"):
        project_vertical_clearance(
            LidarSample(2.0, 9.49, 4, 0),
            AttitudeSample(0.0, 0.0, 0.0, 9.9, 8),
            calibration(max_skew_seconds=1.0),
            now=10.0,
        )


def test_attitude_age_and_range_attitude_skew_are_bounded_inclusively():
    range_sample = LidarSample(2.0, 9.5, 4, 0)
    attitude_sample = AttitudeSample(0.0, 0.0, 0.0, 9.6, 8)

    projected = project_vertical_clearance(
        range_sample,
        attitude_sample,
        calibration(),
        now=10.0,
    )
    assert projected.projected_clearance_m == pytest.approx(2.2)

    with pytest.raises(ClearanceUnavailableError, match="attitude sample is stale"):
        project_vertical_clearance(
            range_sample,
            AttitudeSample(0.0, 0.0, 0.0, 9.49, 8),
            calibration(max_skew_seconds=1.0),
            now=10.0,
        )

    with pytest.raises(ClearanceUnavailableError, match="sample skew"):
        project_vertical_clearance(
            range_sample,
            AttitudeSample(0.0, 0.0, 0.0, 9.61, 8),
            calibration(),
            now=10.0,
        )


def test_projection_applies_tilt_and_reference_offset_once():
    tilt = math.radians(30.0)
    range_sample = LidarSample(2.0, 10.0, 4, 0)
    attitude_sample = AttitudeSample(0.0, tilt, 0.7, 10.0, 8)

    projected = project_vertical_clearance(
        range_sample,
        attitude_sample,
        calibration(max_tilt_rad=tilt),
        now=10.0,
    )

    assert projected.projected_clearance_m == pytest.approx(2.2 * math.cos(tilt))

    with pytest.raises(ClearanceUnavailableError, match="tilt"):
        project_vertical_clearance(
            range_sample,
            AttitudeSample(0.0, tilt + 0.001, 0.7, 10.0, 9),
            calibration(max_tilt_rad=tilt),
            now=10.0,
        )


def test_upward_reference_offset_uses_frd_sign_and_negative_result_is_unavailable():
    range_sample = LidarSample(2.0, 10.0, 4, 0)
    attitude_sample = AttitudeSample(0.0, 0.0, 0.0, 10.0, 8)

    projected = project_vertical_clearance(
        range_sample,
        attitude_sample,
        calibration(measured_reference_offset_body_frd_m=(0.0, 0.0, -0.25)),
        now=10.0,
    )
    assert projected.projected_clearance_m == pytest.approx(1.75)

    with pytest.raises(ClearanceUnavailableError, match="negative"):
        project_vertical_clearance(
            LidarSample(0.1, 10.0, 5, 0),
            attitude_sample,
            calibration(
                measured_reference_offset_body_frd_m=(0.0, 0.0, -0.25)
            ),
            now=10.0,
        )


def test_frd_forward_offset_uses_pitch_sign_in_ned_down_projection():
    pitch = math.radians(30.0)
    projected = project_vertical_clearance(
        LidarSample(2.0, 10.0, 4, 0),
        AttitudeSample(0.0, pitch, 1.1, 10.0, 8),
        calibration(
            measured_reference_offset_body_frd_m=(1.0, 0.0, 0.0),
            max_tilt_rad=math.radians(31.0),
        ),
        now=10.0,
    )

    assert projected.projected_clearance_m == pytest.approx(
        2.0 * math.cos(pitch) - math.sin(pitch)
    )


@pytest.mark.parametrize(
    ("range_time", "attitude_time", "message"),
    [
        (10.01, 10.0, "range sample is future-dated"),
        (10.0, 10.01, "attitude sample is future-dated"),
    ],
)
def test_future_dated_observation_is_unavailable(range_time, attitude_time, message):
    with pytest.raises(ClearanceUnavailableError, match=message):
        project_vertical_clearance(
            LidarSample(2.0, range_time, 4, 0),
            AttitudeSample(0.0, 0.0, 0.0, attitude_time, 8),
            calibration(),
            now=10.0,
        )


@pytest.mark.parametrize(
    ("range_sample", "attitude_sample", "now"),
    [
        (LidarSample(True, 10.0, 1, 0), AttitudeSample(0.0, 0.0, 0.0, 10.0, 1), 10.0),
        (LidarSample(-0.1, 10.0, 1, 0), AttitudeSample(0.0, 0.0, 0.0, 10.0, 1), 10.0),
        (LidarSample(1.0, 10.0, 0, 0), AttitudeSample(0.0, 0.0, 0.0, 10.0, 1), 10.0),
        (LidarSample(1.0, 10.0, 1, -1), AttitudeSample(0.0, 0.0, 0.0, 10.0, 1), 10.0),
        (
            LidarSample(1.0, 10.0, 1, 0),
            AttitudeSample(math.nan, 0.0, 0.0, 10.0, 1),
            10.0,
        ),
        (
            LidarSample(1.0, 10.0, 1, 0),
            AttitudeSample(0.0, 0.0, 0.0, 10.0, False),
            10.0,
        ),
        (LidarSample(1.0, 10.0, 1, 0), AttitudeSample(0.0, 0.0, 0.0, 10.0, 1), True),
    ],
)
def test_malformed_observations_are_unavailable(range_sample, attitude_sample, now):
    with pytest.raises(ClearanceUnavailableError):
        project_vertical_clearance(
            range_sample,
            attitude_sample,
            calibration(),
            now=now,
        )
