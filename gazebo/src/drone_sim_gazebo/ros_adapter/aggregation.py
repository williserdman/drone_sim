"""Bounded alignment of private Gazebo odometry and contact samples."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Protocol, cast

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


class _OdometrySample(Protocol):
    sim_timestamp_ns: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]


class _PayloadSupportSample(Protocol):
    sim_timestamp_ns: int
    aruco_id: int
    grounded: bool
    attached: bool


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
        sample = cast(_OdometrySample, value)
        return NativeOdometry(
            _stamp(sample.sim_timestamp_ns),
            cast(tuple[float, float, float], _vector(sample.position_xyz, 3, "position_xyz")),
            cast(
                tuple[float, float, float, float],
                _vector(sample.orientation_xyzw, 4, "orientation_xyzw"),
            ),
            cast(
                tuple[float, float, float],
                _vector(sample.linear_velocity_xyz, 3, "linear_velocity_xyz"),
            ),
            cast(
                tuple[float, float, float],
                _vector(sample.angular_velocity_xyz, 3, "angular_velocity_xyz"),
            ),
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


class VehicleSupportTracker:
    """Join leg and attached-payload support on exact public truth epochs."""

    def __init__(self, *, payload_ids: tuple[int, ...], interval_ns: int) -> None:
        if not payload_ids or len(set(payload_ids)) != len(payload_ids):
            raise AggregationFault("payload support IDs must be unique and nonempty")
        if type(interval_ns) is not int or interval_ns <= 0:
            raise AggregationFault("support interval must be a positive integer")
        self._payload_ids = frozenset(payload_ids)
        self._interval_ns = interval_ns
        self._leg_epochs: dict[int, bool] = {}
        self._last_leg_stamp: int | None = None
        self._last_odometry_stamp: int | None = None
        self._odometry_epochs: dict[int, NativeOdometry] = {}
        self._payload_epochs: dict[int, dict[int, bool]] = {}
        self._last_payload_stamps: dict[int, int] = {}

    def _epoch(self, stamp_ns: int) -> int:
        return ((stamp_ns + self._interval_ns - 1) // self._interval_ns) * self._interval_ns

    def accept_leg_contact(
        self, sim_timestamp_ns: int, in_contact: bool
    ) -> tuple[tuple[NativeOdometry, bool], ...]:
        stamp = _stamp(sim_timestamp_ns)
        if type(in_contact) is not bool:
            raise AggregationFault("leg contact state must be a boolean")
        if self._last_leg_stamp is not None and stamp <= self._last_leg_stamp:
            raise AggregationFault("leg contact timestamps must advance")
        self._last_leg_stamp = stamp
        epoch = self._epoch(stamp)
        self._leg_epochs[epoch] = self._leg_epochs.get(epoch, False) or in_contact
        return self._drain()

    def accept_odometry(
        self, sample: object
    ) -> tuple[tuple[NativeOdometry, bool], ...]:
        odometry = _odometry(sample)
        stamp = odometry.sim_timestamp_ns
        if stamp % self._interval_ns != 0:
            raise AggregationFault("support odometry must align with a public epoch")
        if self._last_odometry_stamp is not None and stamp <= self._last_odometry_stamp:
            raise AggregationFault("support odometry timestamps must advance")
        self._last_odometry_stamp = stamp
        self._odometry_epochs[stamp] = odometry
        return self._drain()

    def accept_payload(
        self, state: object
    ) -> tuple[tuple[NativeOdometry, bool], ...]:
        try:
            sample = cast(_PayloadSupportSample, state)
            marker = sample.aruco_id
            stamp = sample.sim_timestamp_ns
            grounded = sample.grounded
            attached = sample.attached
        except AttributeError as error:
            raise AggregationFault("payload support state is incomplete") from error
        if type(marker) is not int or marker not in self._payload_ids:
            raise AggregationFault("payload support ID is unknown")
        stamp = _stamp(stamp)
        if type(grounded) is not bool or type(attached) is not bool:
            raise AggregationFault("payload support flags must be boolean")
        previous = self._last_payload_stamps.get(marker)
        if previous is None:
            if stamp != self._interval_ns:
                raise AggregationFault(
                    "first payload support must match the first public epoch"
                )
        elif stamp - previous != self._interval_ns:
            raise AggregationFault(
                "payload support timestamps must advance by one public epoch"
            )
        self._last_payload_stamps[marker] = stamp
        self._payload_epochs.setdefault(stamp, {})[marker] = grounded and attached
        return self._drain()

    def _drain(self) -> tuple[tuple[NativeOdometry, bool], ...]:
        output: list[tuple[NativeOdometry, bool]] = []
        for epoch in sorted(self._payload_epochs):
            payloads = self._payload_epochs[epoch]
            if set(payloads) != self._payload_ids or epoch not in self._odometry_epochs:
                break
            leg_known = epoch in self._leg_epochs
            if not leg_known:
                contact_closes_epoch = (
                    self._last_leg_stamp is not None
                    and self._epoch(self._last_leg_stamp) > epoch
                )
                odometry_closes_epoch = (
                    self._last_odometry_stamp is not None
                    and self._last_odometry_stamp > epoch
                )
                if not contact_closes_epoch and not odometry_closes_epoch:
                    break
                leg_contact = False
            else:
                leg_contact = self._leg_epochs[epoch]
            output.append(
                (self._odometry_epochs.pop(epoch), leg_contact or any(payloads.values()))
            )
            del self._payload_epochs[epoch]
            self._leg_epochs = {
                stamp: value for stamp, value in self._leg_epochs.items() if stamp > epoch
            }
        if (
            len(self._payload_epochs) > _COMPLETED_CAPACITY
            or len(self._odometry_epochs) > _COMPLETED_CAPACITY
        ):
            raise AggregationFault("payload support lookahead is full")
        return tuple(output)
