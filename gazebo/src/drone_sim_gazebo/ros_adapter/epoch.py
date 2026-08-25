"""Pure mapping from warm Gazebo time to the zero-based public run epoch."""

from __future__ import annotations

from dataclasses import dataclass

from .model import AdapterFault


FRAME_INTERVAL_NS = 50_000_000


def _native_timestamp(value: object) -> int:
    if type(value) is not int or value < 0:
        raise AdapterFault("native timestamp must be a nonnegative integer")
    return value


@dataclass(frozen=True)
class PublicEpoch:
    """Rebase native samples strictly after one floored camera-grid epoch."""

    native_epoch_ns: int

    def __post_init__(self) -> None:
        timestamp = _native_timestamp(self.native_epoch_ns)
        if timestamp % FRAME_INTERVAL_NS != 0:
            raise AdapterFault("native epoch must lie on the 50 ms camera grid")

    @classmethod
    def from_latest_native_clock(cls, latest_native_clock_ns: object) -> PublicEpoch:
        latest = _native_timestamp(latest_native_clock_ns)
        return cls(latest - latest % FRAME_INTERVAL_NS)

    def rebase(self, native_timestamp_ns: object) -> int | None:
        native = _native_timestamp(native_timestamp_ns)
        if native <= self.native_epoch_ns:
            return None
        return native - self.native_epoch_ns


class OutputEpochGate:
    """Cache warmup clock only, then expose one rebased public epoch."""

    def __init__(self, *, expected_frames: int) -> None:
        if type(expected_frames) is not int or expected_frames <= 0:
            raise AdapterFault("expected_frames must be a positive integer")
        self._maximum_public_timestamp_ns = expected_frames * FRAME_INTERVAL_NS
        self._latest_native_clock_ns: int | None = None
        self._epoch: PublicEpoch | None = None
        self._activation_requested = False
        self._barrier_camera_stamps: dict[str, int] = {}

    @property
    def native_epoch_ns(self) -> int | None:
        return None if self._epoch is None else self._epoch.native_epoch_ns

    @property
    def activation_pending(self) -> bool:
        return self._activation_requested and self._epoch is None

    def request_activation(self) -> None:
        self._activation_requested = True

    def accept_clock(self, native_timestamp_ns: object) -> int | None:
        native = _native_timestamp(native_timestamp_ns)
        if self._epoch is not None:
            public = self._epoch.rebase(native)
            if (
                public is not None
                and public > self._maximum_public_timestamp_ns
            ):
                return None
            return public
        if (
            self._latest_native_clock_ns is None
            or native > self._latest_native_clock_ns
        ):
            self._latest_native_clock_ns = native
        if (
            self._activation_requested
            and set(self._barrier_camera_stamps) == {"onboard", "observer"}
            and self._latest_native_clock_ns
            >= max(self._barrier_camera_stamps.values())
        ):
            self._epoch = PublicEpoch.from_latest_native_clock(
                self._latest_native_clock_ns
            )
            return 0
        return None

    def accept_camera(self, stream: object, native_timestamp_ns: object) -> int | None:
        if stream not in {"onboard", "observer"} or not isinstance(stream, str):
            raise AdapterFault("camera stream must be onboard or observer")
        native = _native_timestamp(native_timestamp_ns)
        if self._epoch is not None:
            return self._epoch.rebase(native)
        if self._activation_requested:
            previous = self._barrier_camera_stamps.get(stream)
            if previous is None or native > previous:
                self._barrier_camera_stamps[stream] = native
        return None

    def rebase_sample(self, native_timestamp_ns: object) -> int | None:
        if self._epoch is None:
            return None
        return self._epoch.rebase(native_timestamp_ns)
