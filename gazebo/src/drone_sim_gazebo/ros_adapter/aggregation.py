"""Bounded alignment of private Gazebo odometry and contact samples."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .model import NativeGroundTruth


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
    """Hold at most one unmatched native input and one completed truth sample."""

    def __init__(self) -> None:
        self._odometry: NativeOdometry | None = None
        self._contact: tuple[int, bool] | None = None
        self._completed: NativeGroundTruth | None = None

    def _complete(self, odometry: NativeOdometry, in_contact: bool) -> NativeGroundTruth:
        self._completed = NativeGroundTruth(
            sim_timestamp_ns=odometry.sim_timestamp_ns,
            position_xyz=odometry.position_xyz,
            orientation_xyzw=odometry.orientation_xyzw,
            linear_velocity_xyz=odometry.linear_velocity_xyz,
            angular_velocity_xyz=odometry.angular_velocity_xyz,
            in_contact=in_contact,
        )
        return self._completed

    def _combine(self) -> NativeGroundTruth | None:
        if self._odometry is None or self._contact is None:
            return None
        if self._odometry.sim_timestamp_ns != self._contact[0]:
            raise AggregationFault("odometry and contact timestamps do not align")
        odometry = self._odometry
        contact = self._contact
        self._odometry = None
        self._contact = None
        return self._complete(odometry, contact[1])

    def _require_completion_slot(self) -> None:
        if self._completed is not None:
            raise AggregationFault("completed ground truth awaits camera pair")

    def accept_odometry(self, sample: object) -> NativeGroundTruth | None:
        self._require_completion_slot()
        odometry = _odometry(sample)
        if self._odometry is not None:
            previous = self._odometry
            if odometry.sim_timestamp_ns <= previous.sim_timestamp_ns:
                raise AggregationFault("odometry timestamps must advance")
            self._odometry = odometry
            return self._complete(previous, False)
        self._odometry = odometry
        return self._combine()

    def accept_contact(
        self, sim_timestamp_ns: int, in_contact: bool
    ) -> NativeGroundTruth | None:
        self._require_completion_slot()
        if self._contact is not None:
            raise AggregationFault("one unmatched contact sample is already buffered")
        if type(in_contact) is not bool:
            raise AggregationFault("contact state must be a boolean")
        self._contact = (_stamp(sim_timestamp_ns), in_contact)
        return self._combine()

    def take(self, sim_timestamp_ns: int) -> NativeGroundTruth | None:
        stamp = _stamp(sim_timestamp_ns)
        if self._completed is None:
            return None
        if self._completed.sim_timestamp_ns != stamp:
            raise AggregationFault("ground truth does not align with the camera pair")
        completed = self._completed
        self._completed = None
        return completed
