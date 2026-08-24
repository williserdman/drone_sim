"""Production-shaped sequencing around the pure adapter and truth aggregator."""

from __future__ import annotations

from .aggregation import PrivateTruthAggregator
from .model import (
    AdapterModel,
    AdapterSummary,
    NativeImage,
    PublicFrame,
    PublicGroundTruth,
)


class LiveAdapter:
    """Join callback-order-independent private samples without queue growth."""

    def __init__(self, *, run_id: str, expected_frames: int) -> None:
        self._adapter = AdapterModel(run_id=run_id, expected_frames=expected_frames)
        self._truth = PrivateTruthAggregator()
        self._pending_pair_stamp: int | None = None

    @property
    def complete(self) -> bool:
        return self._adapter.complete

    def _release_truth(self) -> tuple[PublicGroundTruth, ...]:
        if self._pending_pair_stamp is None:
            return ()
        native = self._truth.take(self._pending_pair_stamp)
        if native is None:
            return ()
        public = self._adapter.accept_ground_truth(native)
        self._pending_pair_stamp = None
        return (public,)

    def accept_image(
        self, stream: str, sample: NativeImage
    ) -> tuple[PublicFrame | PublicGroundTruth, ...]:
        frame = self._adapter.accept_frame(stream, sample)
        if self._adapter.camera_pair_complete(frame.frame_id, frame.sim_timestamp_ns):
            self._pending_pair_stamp = frame.sim_timestamp_ns
        return (frame, *self._release_truth())

    def accept_odometry(self, sample: object) -> tuple[PublicGroundTruth, ...]:
        self._truth.accept_odometry(sample)
        return self._release_truth()

    def accept_contact(
        self, sim_timestamp_ns: int, in_contact: bool
    ) -> tuple[PublicGroundTruth, ...]:
        self._truth.accept_contact(sim_timestamp_ns, in_contact)
        return self._release_truth()

    def freeze(self) -> AdapterSummary:
        return self._adapter.freeze()
