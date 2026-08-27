"""Pure validation and sequencing for Gazebo public samples."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from uuid import UUID


_STREAMS = ("onboard", "observer")
_FRAME_INTERVAL_NS = 50_000_000
_ENCODING = "rgb8"
_UNMATCHED_FRAME_CAPACITY = 2
_APPROVED_IMAGE_GEOMETRIES = {(320, 240), (640, 480)}


class AdapterFault(RuntimeError):
    """A native sample violates the fixed public adapter contract."""


class RangeSequence:
    """Validate one exact zero-based public range timestamp sequence."""

    def __init__(self, *, expected_samples: int, interval_ns: int) -> None:
        self._expected_samples = _positive_integer(
            expected_samples,
            field="expected_samples",
        )
        self._interval_ns = _positive_integer(interval_ns, field="interval_ns")
        self._accepted_samples = 0
        self._fault_reason: str | None = None

    @property
    def accepted_samples(self) -> int:
        return self._accepted_samples

    def accept(self, sim_timestamp_ns: int) -> None:
        if self._fault_reason is not None:
            raise AdapterFault(self._fault_reason)
        try:
            timestamp_ns = _positive_integer(
                sim_timestamp_ns,
                field="sim_timestamp_ns",
            )
            if self._accepted_samples == self._expected_samples:
                raise AdapterFault("downward range sample overrun")
            expected_timestamp_ns = (
                self._accepted_samples + 1
            ) * self._interval_ns
            if timestamp_ns != expected_timestamp_ns:
                if self._accepted_samples == 0:
                    raise AdapterFault(
                        "first range sample must be exactly "
                        f"{self._interval_ns} ns"
                    )
                raise AdapterFault(
                    "downward range timestamps must advance by exactly "
                    f"{self._interval_ns} ns"
                )
            self._accepted_samples += 1
        except AdapterFault as error:
            if self._fault_reason is None:
                self._fault_reason = f"range sequence faulted: {error}"
            raise AdapterFault(self._fault_reason) from error


@dataclass(frozen=True)
class NativeImage:
    sim_timestamp_ns: int
    width: int
    height: int
    encoding: str
    step: int
    data: bytes


@dataclass(frozen=True)
class PublicFrame:
    run_id: str
    stream: str
    frame_id: int
    sim_timestamp_ns: int
    header_timestamp_ns: int
    width: int
    height: int
    encoding: str
    step: int
    data: bytes


@dataclass(frozen=True)
class NativeGroundTruth:
    sim_timestamp_ns: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    in_contact: bool


@dataclass(frozen=True)
class PublicGroundTruth:
    run_id: str
    vehicle_id: str
    sim_timestamp_ns: int
    position_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    linear_velocity_xyz: tuple[float, float, float]
    angular_velocity_xyz: tuple[float, float, float]
    in_contact: bool


@dataclass(frozen=True)
class AdapterSummary:
    onboard_frames: int
    observer_frames: int
    paired_frames: int
    ground_truth_samples: int
    first_sim_timestamp_ns: int | None
    last_sim_timestamp_ns: int | None


def _canonical_run_id(value: object) -> str:
    if not isinstance(value, str):
        raise AdapterFault("run_id must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (ValueError, TypeError, AttributeError) as error:
        raise AdapterFault("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise AdapterFault("run_id must be a canonical UUID")
    return value


def _positive_integer(value: object, *, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise AdapterFault(f"{field} must be a positive integer")
    return value


def _nonnegative_integer(value: object, *, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise AdapterFault(f"{field} must be a nonnegative integer")
    return value


def _validate_stream(stream: object) -> str:
    if stream not in _STREAMS or not isinstance(stream, str):
        raise AdapterFault("stream must be exactly 'onboard' or 'observer'")
    return stream


def _image_geometry(width_px: object, height_px: object) -> tuple[int, int]:
    if (
        type(width_px) is not int
        or type(height_px) is not int
        or (width_px, height_px) not in _APPROVED_IMAGE_GEOMETRIES
    ):
        raise AdapterFault("image geometry must be exactly 320x240 or 640x480")
    return width_px, height_px


def _validate_native_image(
    sample: object,
    *,
    width_px: int,
    height_px: int,
) -> NativeImage:
    if not isinstance(sample, NativeImage):
        raise AdapterFault("camera input must be a NativeImage")
    _positive_integer(sample.sim_timestamp_ns, field="sim_timestamp_ns")
    if (
        not isinstance(sample.width, int)
        or isinstance(sample.width, bool)
        or sample.width != width_px
        or not isinstance(sample.height, int)
        or isinstance(sample.height, bool)
        or sample.height != height_px
        or not isinstance(sample.encoding, str)
        or sample.encoding != _ENCODING
        or not isinstance(sample.step, int)
        or isinstance(sample.step, bool)
        or sample.step != width_px * 3
        or type(sample.data) is not bytes
        or len(sample.data) != width_px * height_px * 3
    ):
        raise AdapterFault(
            f"camera sample must be {width_px}x{height_px} rgb8 with step "
            f"{width_px * 3} and {width_px * height_px * 3} immutable bytes"
        )
    return sample


def _validate_vector(value: object, *, field: str, length: int) -> tuple:
    if type(value) is not tuple or len(value) != length:
        raise AdapterFault(f"{field} must be an immutable tuple of length {length}")
    for component in value:
        if (
            not isinstance(component, (int, float))
            or isinstance(component, bool)
            or (isinstance(component, float) and not math.isfinite(component))
        ):
            raise AdapterFault(f"{field} values must be finite numbers")
    return value


def _validate_native_ground_truth(sample: object) -> NativeGroundTruth:
    if not isinstance(sample, NativeGroundTruth):
        raise AdapterFault("ground-truth input must be a NativeGroundTruth")
    _positive_integer(sample.sim_timestamp_ns, field="sim_timestamp_ns")
    _validate_vector(sample.position_xyz, field="position_xyz", length=3)
    _validate_vector(sample.orientation_xyzw, field="orientation_xyzw", length=4)
    _validate_vector(
        sample.linear_velocity_xyz,
        field="linear_velocity_xyz",
        length=3,
    )
    _validate_vector(
        sample.angular_velocity_xyz,
        field="angular_velocity_xyz",
        length=3,
    )
    if not isinstance(sample.in_contact, bool):
        raise AdapterFault("in_contact must be a boolean")
    return sample


class CameraSequence:
    """Validate one camera stream and assign its contiguous public IDs."""

    def __init__(
        self,
        *,
        run_id: str,
        stream: str,
        expected_frames: int,
        width_px: int = 320,
        height_px: int = 240,
    ) -> None:
        self._run_id = _canonical_run_id(run_id)
        self._stream = _validate_stream(stream)
        self._expected_frames = _positive_integer(
            expected_frames,
            field="expected_frames",
        )
        self._width_px, self._height_px = _image_geometry(width_px, height_px)
        self._accepted_frames = 0
        self._first_sim_timestamp_ns: int | None = None
        self._last_sim_timestamp_ns: int | None = None
        self._fault_reason: str | None = None

    @property
    def accepted_frames(self) -> int:
        return self._accepted_frames

    @property
    def complete(self) -> bool:
        return (
            self._fault_reason is None
            and self._accepted_frames == self._expected_frames
        )

    def _raise_if_faulted(self) -> None:
        if self._fault_reason is not None:
            raise AdapterFault(self._fault_reason)

    def _latch(self, error: AdapterFault) -> None:
        if self._fault_reason is None:
            self._fault_reason = (
                f"{self._stream} camera sequence faulted: {error}"
            )
        raise AdapterFault(self._fault_reason) from error

    def _preflight(self, sample: NativeImage) -> NativeImage:
        self._raise_if_faulted()
        try:
            sample = _validate_native_image(
                sample,
                width_px=self._width_px,
                height_px=self._height_px,
            )
            if self._accepted_frames == self._expected_frames:
                raise AdapterFault(f"{self._stream} camera frame overrun")
            if self._last_sim_timestamp_ns is not None:
                delta_ns = sample.sim_timestamp_ns - self._last_sim_timestamp_ns
                if delta_ns != _FRAME_INTERVAL_NS:
                    raise AdapterFault(
                        f"{self._stream} camera timestamps must advance by exactly "
                        f"{_FRAME_INTERVAL_NS} ns"
                    )
        except AdapterFault as error:
            self._latch(error)
        return sample

    def accept(self, sample: NativeImage) -> PublicFrame:
        sample = self._preflight(sample)

        frame_id = self._accepted_frames
        if self._first_sim_timestamp_ns is None:
            self._first_sim_timestamp_ns = sample.sim_timestamp_ns
        self._last_sim_timestamp_ns = sample.sim_timestamp_ns
        self._accepted_frames += 1
        return PublicFrame(
            run_id=self._run_id,
            stream=self._stream,
            frame_id=frame_id,
            sim_timestamp_ns=sample.sim_timestamp_ns,
            header_timestamp_ns=sample.sim_timestamp_ns,
            width=sample.width,
            height=sample.height,
            encoding=sample.encoding,
            step=sample.step,
            data=sample.data,
        )


class AdapterModel:
    """Align two fixed-rate camera streams with one physical truth sample."""

    def __init__(
        self,
        *,
        run_id: str,
        expected_frames: int,
        width_px: int = 320,
        height_px: int = 240,
    ) -> None:
        self._run_id = _canonical_run_id(run_id)
        self._expected_frames = _positive_integer(
            expected_frames,
            field="expected_frames",
        )
        width_px, height_px = _image_geometry(width_px, height_px)
        self._sequences = {
            stream: CameraSequence(
                run_id=self._run_id,
                stream=stream,
                expected_frames=self._expected_frames,
                width_px=width_px,
                height_px=height_px,
            )
            for stream in _STREAMS
        }
        self._unmatched_frames: dict[str, deque[PublicFrame]] = {
            stream: deque() for stream in _STREAMS
        }
        self._pending_pairs: deque[tuple[int, int]] = deque()
        self._paired_frames = 0
        self._ground_truth_samples = 0
        self._first_sim_timestamp_ns: int | None = None
        self._last_sim_timestamp_ns: int | None = None
        self._last_ground_truth_timestamp_ns: int | None = None
        self._summary: AdapterSummary | None = None
        self._fault_reason: str | None = None

    @property
    def complete(self) -> bool:
        return (
            self._fault_reason is None
            and all(sequence.complete for sequence in self._sequences.values())
            and self._paired_frames == self._expected_frames
            and self._ground_truth_samples == self._expected_frames
            and not self._pending_pairs
            and all(not frames for frames in self._unmatched_frames.values())
        )

    def _raise_if_faulted(self) -> None:
        if self._fault_reason is not None:
            raise AdapterFault(self._fault_reason)

    def _latch(self, error: AdapterFault) -> None:
        if self._fault_reason is None:
            self._fault_reason = f"adapter processing fault: {error}"
        raise AdapterFault(self._fault_reason) from error

    def _require_open(self) -> None:
        self._raise_if_faulted()
        if self._summary is not None:
            raise AdapterFault("adapter is frozen")

    def accept_frame(self, stream: str, sample: NativeImage) -> PublicFrame:
        self._require_open()
        try:
            return self._accept_frame(stream, sample)
        except AdapterFault as error:
            self._latch(error)

    def _accept_frame(self, stream: str, sample: NativeImage) -> PublicFrame:
        stream = _validate_stream(stream)
        if len(self._pending_pairs) == 2:
            raise AdapterFault("camera-pair lookahead buffer is full")
        frames = self._unmatched_frames[stream]
        if len(frames) == _UNMATCHED_FRAME_CAPACITY:
            raise AdapterFault(f"{stream} camera lookahead buffer is full")

        sequence = self._sequences[stream]
        sample = sequence._preflight(sample)
        other_stream = "observer" if stream == "onboard" else "onboard"
        other_frames = self._unmatched_frames[other_stream]
        other = other_frames[0] if other_frames else None
        if other is not None:
            if other.frame_id != sequence.accepted_frames:
                raise AdapterFault("camera pair IDs do not match")
            if other.sim_timestamp_ns != sample.sim_timestamp_ns:
                raise AdapterFault("camera pair timestamps do not match")

        frame = sequence.accept(sample)
        frames.append(frame)
        if other is not None:
            self._pending_pairs.append((frame.frame_id, frame.sim_timestamp_ns))
            frames.popleft()
            other_frames.popleft()
        return frame

    def camera_pair_complete(self, frame_id: int, stamp_ns: int) -> bool:
        frame_id = _nonnegative_integer(frame_id, field="frame_id")
        stamp_ns = _positive_integer(stamp_ns, field="stamp_ns")
        return (frame_id, stamp_ns) in self._pending_pairs

    def accept_ground_truth(
        self,
        sample: NativeGroundTruth,
    ) -> PublicGroundTruth:
        self._require_open()
        try:
            return self._accept_ground_truth(sample)
        except AdapterFault as error:
            self._latch(error)

    def _accept_ground_truth(
        self,
        sample: NativeGroundTruth,
    ) -> PublicGroundTruth:
        sample = _validate_native_ground_truth(sample)
        if (
            self._last_ground_truth_timestamp_ns is not None
            and sample.sim_timestamp_ns <= self._last_ground_truth_timestamp_ns
        ):
            raise AdapterFault("ground-truth timestamps must increase")
        if not self._pending_pairs:
            raise AdapterFault("ground truth requires one aligned camera pair")
        _, pair_stamp_ns = self._pending_pairs[0]
        if sample.sim_timestamp_ns != pair_stamp_ns:
            raise AdapterFault("ground truth timestamp does not match camera pair")

        public = PublicGroundTruth(
            run_id=self._run_id,
            vehicle_id="iris",
            sim_timestamp_ns=sample.sim_timestamp_ns,
            position_xyz=sample.position_xyz,
            orientation_xyzw=sample.orientation_xyzw,
            linear_velocity_xyz=sample.linear_velocity_xyz,
            angular_velocity_xyz=sample.angular_velocity_xyz,
            in_contact=sample.in_contact,
        )
        self._pending_pairs.popleft()
        self._paired_frames += 1
        self._ground_truth_samples += 1
        self._last_ground_truth_timestamp_ns = sample.sim_timestamp_ns
        if self._first_sim_timestamp_ns is None:
            self._first_sim_timestamp_ns = sample.sim_timestamp_ns
        self._last_sim_timestamp_ns = sample.sim_timestamp_ns
        return public

    def freeze(self) -> AdapterSummary:
        if self._summary is not None:
            return self._summary
        self._raise_if_faulted()
        if not self.complete:
            raise AdapterFault(
                "adapter completion requires exactly the expected aligned pairs"
            )
        self._summary = AdapterSummary(
            onboard_frames=self._sequences["onboard"].accepted_frames,
            observer_frames=self._sequences["observer"].accepted_frames,
            paired_frames=self._paired_frames,
            ground_truth_samples=self._ground_truth_samples,
            first_sim_timestamp_ns=self._first_sim_timestamp_ns,
            last_sim_timestamp_ns=self._last_sim_timestamp_ns,
        )
        return self._summary
