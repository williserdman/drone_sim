"""Bounded alignment of private Gazebo odometry and contact samples."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

from .model import NativeGroundTruth


TRUTH_PERIOD_NS = 50_000_000
_COMPLETED_CAPACITY = 20


class AggregationFault(RuntimeError):
    """Private physical-truth samples cannot form one aligned value."""


@dataclass(frozen=True)
class NativeOdometry:
    sim_timestamp_ns: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]


def _stamp(value: object) -> int:
    if type(value) is not int or value <= 0:
        raise AggregationFault("native timestamp must be a positive integer")
    return value


def _contact_epoch(stamp_ns: int) -> int:
    """Map a positive contact event into its 20 Hz truth interval."""
    return ((stamp_ns + TRUTH_PERIOD_NS - 1) // TRUTH_PERIOD_NS) * TRUTH_PERIOD_NS


def _vector(value: object, length: int, name: str) -> tuple[float, ...]:
    if type(value) is not tuple or len(value) != length:
        raise AggregationFault(f"{name} must be an immutable {length}-tuple")
    if any(
        isinstance(item, bool)
        or not isinstance(item, (int, float))
        or not math.isfinite(float(item))
        for item in value
    ):
        raise AggregationFault(f"{name} must contain finite numbers")
    return tuple(float(item) for item in value)


def _odometry(value: object) -> NativeOdometry:
    try:
        return NativeOdometry(
            _stamp(value.sim_timestamp_ns),
            _vector(value.position_xyz, 3, "position_xyz"),
            _vector(value.orientation_xyzw, 4, "orientation_xyzw"),
            _vector(value.linear_velocity_xyz, 3, "linear_velocity_xyz"),
            _vector(value.angular_velocity_xyz, 3, "angular_velocity_xyz"),
        )
    except AttributeError as error:
        raise AggregationFault("odometry sample is incomplete") from error


class PrivateTruthAggregator:
    """Hold one native input and bounded completed-truth lookahead."""

    def __init__(self) -> None:
        self._odometry: NativeOdometry | None = None
        self._last_odometry_stamp: int | None = None
        self._contact_epochs: dict[int, bool] = {}
        self._last_contact_stamp: int | None = None
        self._completed: deque[NativeGroundTruth] = deque()

    def _complete(self, odometry: NativeOdometry, in_contact: bool) -> NativeGroundTruth:
        self._require_completion_capacity()
        self._contact_epochs = {
            epoch: state
            for epoch, state in self._contact_epochs.items()
            if epoch > odometry.sim_timestamp_ns
        }
        completed = NativeGroundTruth(
            sim_timestamp_ns=odometry.sim_timestamp_ns,
            position_xyz=odometry.position_xyz,
            orientation_xyzw=odometry.orientation_xyzw,
            linear_velocity_xyz=odometry.linear_velocity_xyz,
            angular_velocity_xyz=odometry.angular_velocity_xyz,
            in_contact=in_contact,
        )
        self._completed.append(completed)
        return completed

    def _combine(self) -> NativeGroundTruth | None:
        if self._odometry is None:
            return None
        odometry = self._odometry
        stamp = odometry.sim_timestamp_ns
        if stamp in self._contact_epochs:
            self._odometry = None
            return self._complete(odometry, self._contact_epochs[stamp])
        if (
            self._last_contact_stamp is not None
            and _contact_epoch(self._last_contact_stamp) > stamp
        ):
            self._odometry = None
            return self._complete(odometry, False)
        return None

    def _require_completion_capacity(self) -> None:
        if len(self._completed) == _COMPLETED_CAPACITY:
            raise AggregationFault("completed ground truth lookahead is full")

    def accept_odometry(self, sample: object) -> NativeGroundTruth | None:
        self._require_completion_capacity()
        odometry = _odometry(sample)
        if (
            self._last_odometry_stamp is not None
            and odometry.sim_timestamp_ns <= self._last_odometry_stamp
        ):
            raise AggregationFault("odometry timestamps must advance")
        self._last_odometry_stamp = odometry.sim_timestamp_ns
        if self._odometry is not None:
            previous = self._odometry
            self._odometry = odometry
            return self._complete(previous, False)
        self._odometry = odometry
        return self._combine()

    def accept_contact(
        self, sim_timestamp_ns: int, in_contact: bool
    ) -> NativeGroundTruth | None:
        if type(in_contact) is not bool:
            raise AggregationFault("contact state must be a boolean")
        stamp = _stamp(sim_timestamp_ns)
        if self._last_contact_stamp is not None and stamp <= self._last_contact_stamp:
            raise AggregationFault("contact timestamps must advance")
        self._last_contact_stamp = stamp
        epoch = _contact_epoch(stamp)
        self._contact_epochs[epoch] = self._contact_epochs.get(epoch, False) or in_contact
        return self._combine()

    def take(self, sim_timestamp_ns: int) -> NativeGroundTruth | None:
        stamp = _stamp(sim_timestamp_ns)
        if not self._completed:
            return None
        if self._completed[0].sim_timestamp_ns != stamp:
            raise AggregationFault("ground truth does not align with the camera pair")
        return self._completed.popleft()
