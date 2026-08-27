"""Join Gazebo-owned payload pose, contact, and joint-state facts."""

from __future__ import annotations

from dataclasses import dataclass

from .model import (
    AdapterFault,
    _canonical_run_id,
    _positive_integer,
    _validate_vector,
)


@dataclass(frozen=True)
class PublicPayloadState:
    run_id: str
    sim_timestamp_ns: int
    aruco_id: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    grounded: bool
    attached: bool


def payload_state_message(value: PublicPayloadState, message_type):
    """Convert immutable physical truth to the shared ROS message contract."""
    message = message_type()
    message.run_id = value.run_id
    message.sim_timestamp.sec, message.sim_timestamp.nanosec = divmod(
        value.sim_timestamp_ns,
        1_000_000_000,
    )
    message.aruco_id = value.aruco_id
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
    message.twist.angular.x = 0.0
    message.twist.angular.y = 0.0
    message.twist.angular.z = 0.0
    message.grounded = value.grounded
    message.attached = value.attached
    return message


class PayloadTracker:
    """Publish one fail-closed physical payload sample per exact pose tick."""

    def __init__(self, *, run_id: str, aruco_id: int, interval_ns: int) -> None:
        self._run_id = _canonical_run_id(run_id)
        if type(aruco_id) is not int or not 0 <= aruco_id <= 65_535:
            raise AdapterFault("aruco_id must be an unsigned 16-bit integer")
        self._aruco_id = aruco_id
        self._interval_ns = _positive_integer(interval_ns, field="interval_ns")
        self._last_pose_timestamp_ns: int | None = None
        self._last_position_xyz: tuple[float, float, float] | None = None
        self._contact_timestamp_ns: int | None = None
        self._grounded: bool | None = None
        self._attachment_timestamp_ns: int | None = None
        self._attached: bool | None = None
        self._accepted_poses = 0
        self._fault_reason: str | None = None

    @property
    def accepted_poses(self) -> int:
        return self._accepted_poses

    def _raise_if_faulted(self) -> None:
        if self._fault_reason is not None:
            raise AdapterFault(self._fault_reason)

    def _latch(self, error: AdapterFault) -> None:
        if self._fault_reason is None:
            self._fault_reason = (
                f"payload {self._aruco_id} tracker faulted: {error}"
            )
        raise AdapterFault(self._fault_reason) from error

    def _timestamp(self, value: object) -> int:
        return _positive_integer(value, field="sim_timestamp_ns")

    def accept_contact(self, sim_timestamp_ns: int, grounded: bool) -> None:
        self._raise_if_faulted()
        try:
            timestamp_ns = self._timestamp(sim_timestamp_ns)
            if type(grounded) is not bool:
                raise AdapterFault("grounded contact state must be a boolean")
            if (
                self._contact_timestamp_ns is not None
                and timestamp_ns <= self._contact_timestamp_ns
            ):
                raise AdapterFault("contact timestamps must increase")
            self._contact_timestamp_ns = timestamp_ns
            self._grounded = grounded
        except AdapterFault as error:
            self._latch(error)

    def accept_attachment(
        self,
        sim_timestamp_ns: int,
        physical_state: bool | str,
    ) -> None:
        self._raise_if_faulted()
        try:
            timestamp_ns = self._timestamp(sim_timestamp_ns)
            if type(physical_state) is bool:
                attached = physical_state
            elif physical_state == "attached":
                attached = True
            elif physical_state == "detached":
                attached = False
            else:
                raise AdapterFault(
                    "physical joint state must be exactly attached or detached"
                )
            if (
                self._attachment_timestamp_ns is not None
                and timestamp_ns < self._attachment_timestamp_ns
            ):
                raise AdapterFault("attachment timestamps must not regress")
            self._attachment_timestamp_ns = timestamp_ns
            self._attached = attached
        except AdapterFault as error:
            self._latch(error)

    def accept_pose(
        self,
        sim_timestamp_ns: int,
        position_xyz: tuple[float, float, float],
        orientation_xyzw: tuple[float, float, float, float],
    ) -> PublicPayloadState:
        self._raise_if_faulted()
        try:
            timestamp_ns = self._timestamp(sim_timestamp_ns)
            position = _validate_vector(
                position_xyz,
                field="position_xyz",
                length=3,
            )
            orientation = _validate_vector(
                orientation_xyzw,
                field="orientation_xyzw",
                length=4,
            )
            if self._last_pose_timestamp_ns is not None:
                delta_ns = timestamp_ns - self._last_pose_timestamp_ns
                if delta_ns != self._interval_ns:
                    raise AdapterFault(
                        "payload pose timestamps must advance by exactly "
                        f"{self._interval_ns} ns"
                    )
            if self._contact_timestamp_ns is None or self._grounded is None:
                raise AdapterFault("payload contact state is unknown")
            contact_age_ns = timestamp_ns - self._contact_timestamp_ns
            if contact_age_ns < 0:
                raise AdapterFault("payload contact timestamp is ahead of pose")
            if contact_age_ns > self._interval_ns:
                raise AdapterFault("payload contact is older than one sample")
            if self._attachment_timestamp_ns is None or self._attached is None:
                raise AdapterFault("payload attachment state is unknown")
            if self._attachment_timestamp_ns > timestamp_ns:
                raise AdapterFault("payload attachment timestamp is ahead of pose")

            if self._last_position_xyz is None:
                velocity = (0.0, 0.0, 0.0)
            else:
                seconds = self._interval_ns / 1_000_000_000
                velocity = tuple(
                    (current - previous) / seconds
                    for current, previous in zip(position, self._last_position_xyz)
                )
            public = PublicPayloadState(
                run_id=self._run_id,
                sim_timestamp_ns=timestamp_ns,
                aruco_id=self._aruco_id,
                position_xyz=position,
                orientation_xyzw=orientation,
                linear_velocity_xyz=velocity,
                grounded=self._grounded,
                attached=self._attached,
            )
            self._last_pose_timestamp_ns = timestamp_ns
            self._last_position_xyz = position
            self._accepted_poses += 1
            return public
        except AdapterFault as error:
            self._latch(error)
