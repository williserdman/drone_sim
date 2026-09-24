"""Align measured moving-pad odometry and collision-owned contact truth."""

from __future__ import annotations

from dataclasses import dataclass
from uuid import UUID

from .aggregation import NativeOdometry, TRUTH_PERIOD_NS, _contact_epoch, _odometry
from .model import AdapterFault


_DECK_COLLISION = "moving_pad::deck::deck_collision"
_IRIS_LEG_COLLISIONS = frozenset(
    f"iris::airframe::base_link::{name}_leg_collision"
    for name in ("front_left", "front_right", "rear_left", "rear_right")
)


@dataclass(frozen=True)
class PublicLandingPadState:
    run_id: str
    sim_timestamp_ns: int
    marker_id: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    vehicle_in_contact: bool


def _matches_scoped_name(value: object, expected: str) -> bool:
    name = value if isinstance(value, str) else getattr(value, "name", None)
    return isinstance(name, str) and (
        name == expected or name.endswith(f"::{expected}")
    )


def _is_deck(value: object) -> bool:
    return _matches_scoped_name(value, _DECK_COLLISION)


def _is_iris_leg(value: object) -> bool:
    return any(_matches_scoped_name(value, expected) for expected in _IRIS_LEG_COLLISIONS)


def vehicle_on_deck(contacts: object) -> bool:
    """Return true only for a physical Iris-leg / moving-deck pair."""
    try:
        pairs = tuple((contact.collision1, contact.collision2) for contact in contacts)
    except (AttributeError, TypeError) as error:
        raise AdapterFault("landing-pad contacts are malformed") from error
    return any(
        (_is_deck(first) and _is_iris_leg(second))
        or (_is_iris_leg(first) and _is_deck(second))
        for first, second in pairs
    )


class LandingPadTracker:
    """Join one exact 20 Hz pad odometry/contact pair without inventing absence."""

    def __init__(self, *, run_id: str, marker_id: int) -> None:
        try:
            parsed = UUID(run_id)
        except (AttributeError, TypeError, ValueError) as error:
            raise AdapterFault("run_id must be a canonical UUID") from error
        if str(parsed) != run_id:
            raise AdapterFault("run_id must be a canonical UUID")
        if type(marker_id) is not int or not 0 <= marker_id <= 4_294_967_295:
            raise AdapterFault("marker_id must be an unsigned 32-bit integer")
        self._run_id = run_id
        self._marker_id = marker_id
        self._odometry: NativeOdometry | None = None
        self._contact_epochs: dict[int, bool] = {}
        self._last_odometry_stamp: int | None = None
        self._last_contact_stamp: int | None = None
        self._accepted_samples = 0

    @property
    def accepted_samples(self) -> int:
        return self._accepted_samples

    def _combine(self) -> PublicLandingPadState | None:
        if self._odometry is None:
            return None
        contact_stamp = self._odometry.sim_timestamp_ns
        if contact_stamp not in self._contact_epochs:
            if any(epoch > contact_stamp for epoch in self._contact_epochs):
                raise AdapterFault("landing-pad contact sample is missing")
            return None
        vehicle_in_contact = self._contact_epochs[contact_stamp]
        expected = (self._accepted_samples + 1) * TRUTH_PERIOD_NS
        if contact_stamp != expected:
            raise AdapterFault(
                "landing-pad timestamps must start at 50000000 ns and advance by exactly 50000000 ns"
            )
        state = PublicLandingPadState(
            run_id=self._run_id,
            sim_timestamp_ns=contact_stamp,
            marker_id=self._marker_id,
            position_xyz=self._odometry.position_xyz,
            orientation_xyzw=self._odometry.orientation_xyzw,
            linear_velocity_xyz=self._odometry.linear_velocity_xyz,
            angular_velocity_xyz=self._odometry.angular_velocity_xyz,
            vehicle_in_contact=vehicle_in_contact,
        )
        self._accepted_samples += 1
        self._odometry = None
        self._contact_epochs = {
            epoch: state
            for epoch, state in self._contact_epochs.items()
            if epoch > contact_stamp
        }
        return state

    def accept_odometry(self, sample: object) -> PublicLandingPadState | None:
        odometry = _odometry(sample)
        if self._last_odometry_stamp is not None and odometry.sim_timestamp_ns <= self._last_odometry_stamp:
            raise AdapterFault("landing-pad odometry timestamps must advance")
        if self._odometry is not None:
            raise AdapterFault("landing-pad contact sample is missing")
        self._last_odometry_stamp = odometry.sim_timestamp_ns
        self._odometry = odometry
        return self._combine()

    def accept_contact(
        self, sim_timestamp_ns: int, vehicle_in_contact: bool
    ) -> PublicLandingPadState | None:
        if type(sim_timestamp_ns) is not int or sim_timestamp_ns <= 0:
            raise AdapterFault("landing-pad contact timestamp must be positive")
        if type(vehicle_in_contact) is not bool:
            raise AdapterFault("vehicle_in_contact must be a boolean")
        if self._last_contact_stamp is not None and sim_timestamp_ns <= self._last_contact_stamp:
            raise AdapterFault("landing-pad contact timestamps must advance")
        self._last_contact_stamp = sim_timestamp_ns
        epoch = _contact_epoch(sim_timestamp_ns)
        if epoch <= self._accepted_samples * TRUTH_PERIOD_NS:
            raise AdapterFault("landing-pad contact arrived after its public tick")
        if epoch not in self._contact_epochs and len(self._contact_epochs) == 20:
            raise AdapterFault("landing-pad contact lookahead is full")
        self._contact_epochs[epoch] = (
            self._contact_epochs.get(epoch, False) or vehicle_in_contact
        )
        return self._combine()


def landing_pad_state_message(value: PublicLandingPadState, message_type):
    message = message_type()
    message.run_id = value.run_id
    message.sim_timestamp.sec, message.sim_timestamp.nanosec = divmod(
        value.sim_timestamp_ns, 1_000_000_000
    )
    message.marker_id = value.marker_id
    (
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z,
    ) = value.position_xyz
    (
        message.pose.orientation.x,
        message.pose.orientation.y,
        message.pose.orientation.z,
        message.pose.orientation.w,
    ) = value.orientation_xyzw
    (
        message.twist.linear.x,
        message.twist.linear.y,
        message.twist.linear.z,
    ) = value.linear_velocity_xyz
    (
        message.twist.angular.x,
        message.twist.angular.y,
        message.twist.angular.z,
    ) = value.angular_velocity_xyz
    message.vehicle_in_contact = value.vehicle_in_contact
    return message
