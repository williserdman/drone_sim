"""Active FM3 mission flow with fail-closed precision evidence."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Callable, TYPE_CHECKING

from . import timebase as time
from .common_types import GPSCoord, RelPosComplete
from .control.drone_control import DroneControl
from .control.mission_info import MissonTracker
from .sensors.camera.camera import Camera
from .sensors.lidar.clearance import (
    AttitudeSample,
    ClearanceCalibration,
    ClearanceUnavailableError,
    ProjectedVerticalClearance,
    project_vertical_clearance,
)
from .sensors.lidar.lidar import StaleSensorError

if TYPE_CHECKING:
    from .sensors.lidar.lidar import Lidar
    from .sensors.servo.servo import Dropper


PRECISION_LAND_DEADLINE_S = 60.0
REQUIRED_CENTERED_OBSERVATIONS = 5
MAX_OBSERVATIONS_PER_GRID_CELL = REQUIRED_CENTERED_OBSERVATIONS * 2
GRID_POSITION_TOLERANCE_M = 0.15


class PrecisionEvidenceUnavailable(RuntimeError):
    """Current observations cannot support a precision decision."""


class _CameraAcquisitionTimeout(TimeoutError):
    """A bounded camera read produced no fresh frame."""


def _positive_number(name: str, value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(float(value))
        or float(value) <= 0
    ):
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


@dataclass(frozen=True)
class PrecisionMissionPolicy:
    """Explicit measured and operational limits for active precision FM3."""

    clearance_calibration: ClearanceCalibration
    clock: Callable[[], float]
    max_exposure_age_s: float
    max_image_attitude_skew_s: float
    max_image_location_skew_s: float
    max_attitude_transport_latency_s: float
    max_location_transport_latency_s: float
    acquisition_timeout_s: float
    frame_timeout_s: float
    observation_period_s: float
    target_hover_height_m: float
    hover_tolerance_m: float
    centered_tolerance_m: float
    correction_gain: float
    cruise_altitude_m: float
    desired_drop_height_m: float

    def __post_init__(self) -> None:
        if not isinstance(self.clearance_calibration, ClearanceCalibration):
            raise ValueError("clearance_calibration must be explicit")
        if not callable(self.clock):
            raise ValueError("clock must be callable")
        if self.clock is not time.monotonic:
            raise ValueError("clock must be the shared timebase.monotonic callable")
        for name in (
            "max_exposure_age_s",
            "max_image_attitude_skew_s",
            "max_image_location_skew_s",
            "max_attitude_transport_latency_s",
            "max_location_transport_latency_s",
            "acquisition_timeout_s",
            "frame_timeout_s",
            "observation_period_s",
            "target_hover_height_m",
            "hover_tolerance_m",
            "centered_tolerance_m",
            "cruise_altitude_m",
            "desired_drop_height_m",
        ):
            object.__setattr__(self, name, _positive_number(name, getattr(self, name)))
        gain = _positive_number("correction_gain", self.correction_gain)
        if gain > 1:
            raise ValueError("correction_gain must not exceed one")
        object.__setattr__(self, "correction_gain", gain)
        if self.frame_timeout_s > self.acquisition_timeout_s:
            raise ValueError("frame_timeout_s must not exceed acquisition_timeout_s")


@dataclass(frozen=True)
class _PrecisionEvidence:
    vector: RelPosComplete | None
    frame_sequence: int
    location: GPSCoord
    attitude: AttitudeSample
    clearance: ProjectedVerticalClearance
    location_invalidation_generation: int
    attitude_invalidation_generation: int


def _require_policy(policy: PrecisionMissionPolicy | None) -> PrecisionMissionPolicy:
    if not isinstance(policy, PrecisionMissionPolicy):
        raise ValueError("an explicit validated precision policy is required")
    return policy


def _require_zero(operation: str, result: object) -> None:
    if isinstance(result, bool) or not isinstance(result, int) or result != 0:
        raise RuntimeError(f"{operation} was not confirmed")


def _require_none(operation: str, result: object) -> None:
    if result is not None:
        raise RuntimeError(f"{operation} was not confirmed")


def _now(policy: PrecisionMissionPolicy) -> float:
    value = policy.clock()
    if (
        isinstance(value, bool)
        or not isinstance(value, Real)
        or not math.isfinite(float(value))
    ):
        raise RuntimeError("precision clock is unavailable")
    return float(value)


def _camera_ready(camera: Camera) -> None:
    readiness = camera.precision_readiness()
    if readiness.ready is not True:
        reasons = "; ".join(readiness.reasons)
        raise RuntimeError(f"camera is not precision-ready: {reasons}")


def _field_observation(snapshot, name: str):
    field = getattr(snapshot, name, None)
    if field is None or field.fresh is not True:
        raise PrecisionEvidenceUnavailable(
            f"fresh {name} observation is unavailable"
        )
    return field.observation


def _attitude_sample(snapshot) -> AttitudeSample:
    observation = _field_observation(snapshot, "attitude")
    value = observation.value
    if not isinstance(value, tuple) or len(value) < 3:
        raise PrecisionEvidenceUnavailable("attitude observation is malformed")
    return AttitudeSample(
        roll_rad=value[0],
        pitch_rad=value[1],
        yaw_rad=value[2],
        sampled_at=observation.received_at,
        sequence=observation.sequence,
    )


def _location(snapshot, controller: DroneControl) -> tuple[GPSCoord, object]:
    observation = _field_observation(snapshot, "location")
    value = observation.value
    if not isinstance(value, tuple) or len(value) != 4:
        raise PrecisionEvidenceUnavailable("location observation is malformed")
    lat, lon, amsl_mm, _relative_mm = value
    if any(
        isinstance(item, bool)
        or not isinstance(item, Real)
        or not math.isfinite(float(item))
        for item in (lat, lon, amsl_mm)
    ):
        raise PrecisionEvidenceUnavailable("location observation is malformed")
    home = controller.mission_home
    if home is None:
        raise PrecisionEvidenceUnavailable(
            "precision mission requires pinned mission home"
        )
    return (
        GPSCoord(
            float(lat) / 1e7,
            float(lon) / 1e7,
            float(amsl_mm) / 1000.0 - home.amsl_m,
        ),
        observation,
    )


def _read_clearance(
    controller: DroneControl,
    lidar: Lidar,
    policy: PrecisionMissionPolicy,
) -> ProjectedVerticalClearance:
    controller.check_permission()
    snapshot = controller.flight_snapshot()
    attitude = _attitude_sample(snapshot)
    range_sample = lidar.get_sample()
    now = _now(policy)
    return project_vertical_clearance(
        range_sample,
        attitude,
        policy.clearance_calibration,
        now=float(now),
    )


def _read_precision_evidence(
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    target_id: int,
    policy: PrecisionMissionPolicy,
    *,
    after_sequence: int,
    timeout_s: float,
) -> _PrecisionEvidence:
    controller.check_permission()
    _camera_ready(camera)
    try:
        vector, metadata = camera.vec_to_marker_3d_bounded(
            target_id,
            timeout_s=timeout_s,
            after_sequence=after_sequence,
            quality=4,
        )
    except TimeoutError as error:
        raise _CameraAcquisitionTimeout(str(error)) from error
    if (
        isinstance(metadata.sequence, bool)
        or not isinstance(metadata.sequence, int)
        or metadata.sequence <= 0
        or metadata.exposure_age_bounded is not True
        or isinstance(metadata.exposure_timestamp_ns, bool)
        or not isinstance(metadata.exposure_timestamp_ns, int)
    ):
        raise PrecisionEvidenceUnavailable(
            "camera metadata cannot support precision flight"
        )
    try:
        exposure_s = metadata.exposure_timestamp_ns / 1_000_000_000
    except OverflowError:
        raise PrecisionEvidenceUnavailable(
            "camera exposure timestamp is invalid"
        ) from None
    if not math.isfinite(exposure_s):
        raise PrecisionEvidenceUnavailable("camera exposure timestamp is invalid")
    snapshot = controller.flight_snapshot()
    attitude = _attitude_sample(snapshot)
    location, location_observation = _location(snapshot, controller)
    if not _source_interval_within_skew(
        received_at=attitude.sampled_at,
        exposure_at=exposure_s,
        max_transport_latency_s=policy.max_attitude_transport_latency_s,
        max_skew_s=policy.max_image_attitude_skew_s,
    ):
        raise PrecisionEvidenceUnavailable(
            "camera and attitude observations exceed approved skew"
        )
    if not _source_interval_within_skew(
        received_at=location_observation.received_at,
        exposure_at=exposure_s,
        max_transport_latency_s=policy.max_location_transport_latency_s,
        max_skew_s=policy.max_image_location_skew_s,
    ):
        raise PrecisionEvidenceUnavailable(
            "camera and location observations exceed approved skew"
        )
    range_sample = lidar.get_sample()
    now = _now(policy)
    exposure_age = float(now) - exposure_s
    if not 0 <= exposure_age <= policy.max_exposure_age_s:
        raise PrecisionEvidenceUnavailable(
            "camera exposure is outside the approved age"
        )
    clearance = project_vertical_clearance(
        range_sample,
        attitude,
        policy.clearance_calibration,
        now=float(now),
    )
    if vector is not None and (
        not isinstance(vector, RelPosComplete)
        or any(
            isinstance(value, bool)
            or not isinstance(value, Real)
            or not math.isfinite(float(value))
            for value in (vector.x, vector.y, vector.z)
        )
    ):
        raise PrecisionEvidenceUnavailable("camera marker vector is malformed")
    return _PrecisionEvidence(
        vector=vector,
        frame_sequence=metadata.sequence,
        location=location,
        attitude=attitude,
        clearance=clearance,
        location_invalidation_generation=getattr(
            snapshot.location, "invalidation_generation", 0
        ),
        attitude_invalidation_generation=getattr(
            snapshot.attitude, "invalidation_generation", 0
        ),
    )


def _source_interval_within_skew(
    *,
    received_at: float,
    exposure_at: float,
    max_transport_latency_s: float,
    max_skew_s: float,
) -> bool:
    earliest_source_time = received_at - max_transport_latency_s
    return max(
        abs(received_at - exposure_at),
        abs(earliest_source_time - exposure_at),
    ) <= max_skew_s


def _marker_offset_ne(update: RelPosComplete, attitude) -> tuple[float, float]:
    """Project one body-FRD marker vector into local north/east."""
    roll = attitude.roll_rad if isinstance(attitude, AttitudeSample) else attitude.roll
    pitch = attitude.pitch_rad if isinstance(attitude, AttitudeSample) else attitude.pitch
    yaw = attitude.yaw_rad if isinstance(attitude, AttitudeSample) else attitude.yaw
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    north = (
        cy * cp * update.x
        + (cy * sp * sr - sy * cr) * update.y
        + (cy * sp * cr + sy * sr) * update.z
    )
    east = (
        sy * cp * update.x
        + (sy * sp * sr + cy * cr) * update.y
        + (sy * sp * cr - cy * sr) * update.z
    )
    return north, east


def _touchdown_candidate(controller: DroneControl) -> bool:
    snapshot = controller.flight_snapshot()
    landed = getattr(snapshot, "landed_state", None)
    armed = getattr(snapshot, "armed", None)
    explicit_ground = (
        landed is not None
        and landed.fresh is True
        and isinstance(landed.observation.value, int)
        and not isinstance(landed.observation.value, bool)
        and landed.observation.value == 1
    )
    fresh_auto_disarm = (
        armed is not None
        and armed.fresh is True
        and armed.observation.value is False
    )
    return explicit_ground or fresh_auto_disarm


def _confirm_current_land(
    controller: DroneControl,
    *,
    deadline: float,
    policy: PrecisionMissionPolicy,
) -> bool:
    controller.check_permission()
    remaining = deadline - _now(policy)
    if remaining <= 0:
        raise TimeoutError("precision landing touchdown deadline expired")
    _require_zero(
        "landing confirmation",
        controller.confirm_landing(timeout=remaining),
    )
    return True


def _sleep_bounded(deadline: float, policy: PrecisionMissionPolicy) -> bool:
    remaining = deadline - _now(policy)
    if remaining <= 0:
        return False
    time.sleep(min(policy.observation_period_s, remaining))
    return _now(policy) < deadline


def _finalize_failed_precision_land(
    controller: DroneControl,
    *,
    deadline: float,
    policy: PrecisionMissionPolicy,
) -> bool:
    _confirm_current_land(controller, deadline=deadline, policy=policy)
    return False


def aruco_land_precision(
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    target_id: int,
    *,
    policy: PrecisionMissionPolicy | None = None,
) -> bool:
    policy = _require_policy(policy)
    _camera_ready(camera)
    started_at = _now(policy)
    controller.check_permission()
    _require_zero("LAND mode", controller.set_land_mode())
    deadline = started_at + PRECISION_LAND_DEADLINE_S
    last_sequence = 0

    while True:
        controller.check_permission()
        remaining = deadline - _now(policy)
        if remaining <= 0:
            raise TimeoutError("precision landing touchdown deadline expired")
        if _touchdown_candidate(controller):
            return _confirm_current_land(controller, deadline=deadline, policy=policy)
        if remaining <= policy.observation_period_s:
            return _confirm_current_land(controller, deadline=deadline, policy=policy)
        try:
            evidence = _read_precision_evidence(
                controller,
                camera,
                lidar,
                target_id,
                policy,
                after_sequence=last_sequence,
                timeout_s=min(policy.frame_timeout_s, remaining),
            )
        except _CameraAcquisitionTimeout:
            controller.check_permission()
            if _now(policy) >= deadline:
                raise TimeoutError("precision landing touchdown deadline expired")
            return _finalize_failed_precision_land(
                controller,
                deadline=deadline,
                policy=policy,
            )
        except (
            ClearanceUnavailableError,
            PrecisionEvidenceUnavailable,
            StaleSensorError,
        ):
            controller.check_permission()
            if _now(policy) >= deadline:
                raise TimeoutError("precision landing touchdown deadline expired")
            return _finalize_failed_precision_land(
                controller,
                deadline=deadline,
                policy=policy,
            )
        controller.check_permission()
        if _now(policy) >= deadline:
            raise TimeoutError("precision landing touchdown deadline expired")
        if evidence.frame_sequence <= last_sequence or evidence.vector is None:
            return _finalize_failed_precision_land(
                controller,
                deadline=deadline,
                policy=policy,
            )
        last_sequence = evidence.frame_sequence
        controller.check_permission()
        if _now(policy) >= deadline:
            raise TimeoutError("precision landing touchdown deadline expired")
        _require_zero(
            "landing target",
            controller.land_send_landing_target(evidence.vector),
        )
        _sleep_bounded(deadline, policy)


def pickup_sequence(
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    target_id: int,
    dropper: Dropper,
    *,
    policy: PrecisionMissionPolicy | None = None,
) -> bool:
    policy = _require_policy(policy)
    if getattr(dropper, "supports_attachment", None) is not True:
        raise RuntimeError("FM3 requires explicit attachment capability")
    _camera_ready(camera)
    started_at = _now(policy)
    controller.check_permission()
    _require_zero("GUIDED mode", controller.set_guided_mode())
    acquisition_deadline = started_at + policy.acquisition_timeout_s

    clearance = _read_clearance(controller, lidar, policy)
    down = clearance.projected_clearance_m - policy.target_hover_height_m
    controller.check_permission()
    remaining = acquisition_deadline - _now(policy)
    if remaining <= 0:
        return False
    _require_zero(
        "acquisition altitude correction",
        controller.guide_move_relative_frame(
            RelPosComplete(0.0, 0.0, down),
            timeout=remaining,
        ),
    )
    if _now(policy) >= acquisition_deadline:
        return False

    while _now(policy) < acquisition_deadline:
        try:
            clearance = _read_clearance(controller, lidar, policy)
        except (
            ClearanceUnavailableError,
            PrecisionEvidenceUnavailable,
            StaleSensorError,
        ):
            if not _sleep_bounded(acquisition_deadline, policy):
                return False
            continue
        if (
            abs(clearance.projected_clearance_m - policy.target_hover_height_m)
            <= policy.hover_tolerance_m
        ):
            break
        if not _sleep_bounded(acquisition_deadline, policy):
            return False
    else:
        return False

    center_snapshot = controller.flight_snapshot()
    center, _ = _location(center_snapshot, controller)
    grid_size = 1.5
    grid_offsets_ne = (
        (0.0, 0.0),
        (0.0, grid_size),
        (grid_size, grid_size),
        (grid_size, 0.0),
        (grid_size, -grid_size),
        (0.0, -grid_size),
        (-grid_size, -grid_size),
        (-grid_size, 0.0),
        (-grid_size, grid_size),
    )
    last_sequence = 0
    last_invalidation_generations = (
        clearance.range_sample.invalidation_generation,
        getattr(center_snapshot.location, "invalidation_generation", 0),
        getattr(center_snapshot.attitude, "invalidation_generation", 0),
    )
    centered_count = 0
    target_found = False

    for north, east in grid_offsets_ne:
        if _now(policy) >= acquisition_deadline:
            break
        controller.check_permission()
        waypoint = controller.get_location_metres(center, north, east)
        remaining = acquisition_deadline - _now(policy)
        if remaining <= 0:
            break
        _require_zero(
            "grid waypoint",
            controller.goto_waypoint(
                waypoint,
                position_tol=GRID_POSITION_TOLERANCE_M,
                timeout=remaining,
            ),
        )
        if _now(policy) >= acquisition_deadline:
            return False
        centered_count = 0
        observations_at_cell = 0
        while (
            _now(policy) < acquisition_deadline
            and observations_at_cell < MAX_OBSERVATIONS_PER_GRID_CELL
        ):
            observations_at_cell += 1
            remaining = acquisition_deadline - _now(policy)
            try:
                evidence = _read_precision_evidence(
                    controller,
                    camera,
                    lidar,
                    target_id,
                    policy,
                    after_sequence=last_sequence,
                    timeout_s=min(policy.frame_timeout_s, remaining),
                )
            except _CameraAcquisitionTimeout:
                controller.check_permission()
                if _now(policy) >= acquisition_deadline:
                    return False
                centered_count = 0
                if not _sleep_bounded(acquisition_deadline, policy):
                    return False
                continue
            except (
                ClearanceUnavailableError,
                PrecisionEvidenceUnavailable,
                StaleSensorError,
            ):
                centered_count = 0
                if not _sleep_bounded(acquisition_deadline, policy):
                    return False
                continue
            controller.check_permission()
            if _now(policy) >= acquisition_deadline:
                return False
            if evidence.frame_sequence <= last_sequence:
                centered_count = 0
                if not _sleep_bounded(acquisition_deadline, policy):
                    return False
                continue
            last_sequence = evidence.frame_sequence
            invalidation_generations = (
                evidence.clearance.range_sample.invalidation_generation,
                evidence.location_invalidation_generation,
                evidence.attitude_invalidation_generation,
            )
            if invalidation_generations != last_invalidation_generations:
                last_invalidation_generations = invalidation_generations
                centered_count = 0
                if not _sleep_bounded(acquisition_deadline, policy):
                    return False
                continue
            if evidence.vector is None:
                centered_count = 0
                if not _sleep_bounded(acquisition_deadline, policy):
                    return False
                continue
            clearance_error = (
                evidence.clearance.projected_clearance_m
                - policy.target_hover_height_m
            )
            if abs(clearance_error) > policy.hover_tolerance_m:
                controller.check_permission()
                remaining = acquisition_deadline - _now(policy)
                if remaining <= 0:
                    return False
                _require_zero(
                    "acquisition altitude correction",
                    controller.guide_move_relative_frame(
                        RelPosComplete(0.0, 0.0, clearance_error),
                        timeout=remaining,
                    ),
                )
                if _now(policy) >= acquisition_deadline:
                    return False
                centered_count = 0
                if not _sleep_bounded(acquisition_deadline, policy):
                    return False
                continue
            marker_north, marker_east = _marker_offset_ne(
                evidence.vector,
                evidence.attitude,
            )
            horizontal_error = math.hypot(marker_north, marker_east)
            if horizontal_error > policy.centered_tolerance_m:
                corrected = controller.get_location_metres(
                    evidence.location,
                    marker_north * policy.correction_gain,
                    marker_east * policy.correction_gain,
                )
                controller.check_permission()
                remaining = acquisition_deadline - _now(policy)
                if remaining <= 0:
                    return False
                _require_zero(
                    "marker recenter waypoint",
                    controller.goto_waypoint(
                        corrected,
                        position_tol=GRID_POSITION_TOLERANCE_M,
                        timeout=remaining,
                    ),
                )
                if _now(policy) >= acquisition_deadline:
                    return False
                centered_count = 0
                if not _sleep_bounded(acquisition_deadline, policy):
                    return False
                continue
            centered_count += 1
            if centered_count == REQUIRED_CENTERED_OBSERVATIONS:
                target_found = True
                break
            if not _sleep_bounded(acquisition_deadline, policy):
                return False
        if target_found:
            break

    if not target_found:
        return False
    controller.check_permission()
    if _now(policy) >= acquisition_deadline:
        return False
    if aruco_land_precision(
        controller,
        camera,
        lidar,
        target_id,
        policy=policy,
    ) is not True:
        return False
    controller.check_permission()
    _require_zero("disarm", controller.disarm())
    controller.check_permission()
    if dropper.attach(target_id) is not True:
        raise RuntimeError("attachment was not independently confirmed")
    return True


def fm3(
    mt: MissonTracker,
    controller: DroneControl,
    camera: Camera,
    lidar: Lidar,
    dropper: Dropper,
    possible_ids: set,
    pickup_point: GPSCoord,
    target_point: GPSCoord,
    *,
    precision_policy: PrecisionMissionPolicy | None = None,
) -> bool:
    policy = _require_policy(precision_policy)
    if getattr(dropper, "supports_attachment", None) is not True:
        raise RuntimeError("FM3 requires explicit attachment capability")
    _camera_ready(camera)
    _now(policy)
    pickup_transit = GPSCoord(
        pickup_point.lat,
        pickup_point.long,
        policy.cruise_altitude_m,
    )
    delivery_transit = GPSCoord(
        target_point.lat,
        target_point.long,
        policy.cruise_altitude_m,
    )

    for target_id in sorted(possible_ids):
        controller.check_permission()
        if mt.time_left() < PRECISION_LAND_DEADLINE_S:
            return False
        _require_zero("pickup waypoint", controller.goto_waypoint(pickup_transit))
        if pickup_sequence(
            controller,
            camera,
            lidar,
            target_id,
            dropper,
            policy=policy,
        ) is not True:
            return False

        controller.check_permission()
        _require_none(
            "post-attachment takeoff",
            controller.force_arm_takeoff(policy.cruise_altitude_m),
        )
        controller.check_permission()
        _require_zero("drop waypoint", controller.goto_waypoint(delivery_transit))

        controller.check_permission()
        drop_target = controller.release_waypoint_for_clearance(
            delivery_transit,
            lidar,
            desired_agl_m=policy.desired_drop_height_m,
        )
        controller.check_permission()
        _require_zero(
            "drop-height waypoint",
            controller.goto_waypoint(drop_target),
        )
        controller.check_permission()
        if controller.hold_waypoint_until_stable(
            drop_target,
            lidar,
            required_agl_m=policy.desired_drop_height_m,
        ) is not True:
            return False
        controller.check_permission()
        _require_none(
            "payload release",
            controller.release_payload_if_stable(
                dropper,
                drop_target,
                lidar,
                required_agl_m=policy.desired_drop_height_m,
            ),
        )
    return True


if __name__ == "__main__":
    raise SystemExit(
        "Direct mock_mission flight is disabled. Start the guarded mission listener."
    )
