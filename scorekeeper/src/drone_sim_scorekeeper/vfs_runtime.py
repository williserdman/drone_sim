"""Runtime routing for the read-only VFS 2027 judge and scorer."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any, cast

from .competition import MissionEventSample, PayloadEventSample, PayloadStateSample
from .descent import GroundTruthSample
from ._finalization import _ScoreFinalizer
from .models import ScoreEvent, ScoreResult
from .vfs import (
    AttemptStatus,
    JudgeDecision,
    VehicleStatusSample,
    VfsJudge,
    VfsScorer,
)


class VfsScorekeeperRuntime:
    """Route ordered evidence without exposing physical positions to the companion."""

    def __init__(
        self,
        run_id: str,
        judge: VfsJudge,
        scorer: VfsScorer | None,
        *,
        publish_decision: Callable[[JudgeDecision], None],
        publish_attempt: Callable[[AttemptStatus], None],
        run_directory: Path | str | None = None,
        protocol: Any | None = None,
        publish_score: Callable[[ScoreEvent], None] | None = None,
        flush: Callable[[], None] | None = None,
    ) -> None:
        if judge.run_id != run_id or (scorer is not None and scorer.run_id != run_id):
            raise ValueError("judge and scorer must belong to the runtime run")
        self.run_id = run_id
        self.judge = judge
        self.scorer = scorer
        self._publish_decision = publish_decision
        self._publish_attempt = publish_attempt
        self._ground_truth_timestamp_ns: int | None = None
        self._vehicle_status_timestamp_ns: int | None = None
        self._payload_timestamps_ns: dict[int, int | None] = {2: None, 3: None, 4: None}
        self._finalizer = (
            _ScoreFinalizer(
                cast(Any, scorer),
                run_directory,
                protocol,
                publish_score,
                flush,
            )
            if scorer is not None
            and run_directory is not None
            and protocol is not None
            and publish_score is not None
            and flush is not None
            else None
        )

    @property
    def result(self) -> ScoreResult | None:
        return None if self._finalizer is None else self._finalizer.result

    @property
    def quiescent(self) -> bool:
        return False if self._finalizer is None else self._finalizer.quiescent

    def source_inputs_observed_through(self, timestamp_ns: int) -> bool:
        return (
            self._ground_truth_timestamp_ns is not None
            and self._ground_truth_timestamp_ns >= timestamp_ns
            and self._vehicle_status_timestamp_ns is not None
            and 0 <= timestamp_ns - self._vehicle_status_timestamp_ns <= 500_000_000
            and all(value is not None and value >= timestamp_ns for value in self._payload_timestamps_ns.values())
        )

    def accept_ground_truth(self, sample: GroundTruthSample) -> None:
        attempt = self.judge.accept_ground_truth(sample)
        self._ground_truth_timestamp_ns = sample.sim_timestamp_ns
        if self.scorer is not None:
            self.scorer.accept_ground_truth(sample)
        self._publish_attempt(attempt)

    def accept_vehicle_status(self, sample: VehicleStatusSample) -> None:
        self.judge.accept_vehicle_status(sample)
        self._vehicle_status_timestamp_ns = sample.sim_timestamp_ns
        if self.scorer is not None:
            self.scorer.accept_vehicle_status(sample)

    def accept_payload_state(self, sample: PayloadStateSample) -> None:
        self.judge.accept_payload_state(sample)
        if sample.aruco_id in self._payload_timestamps_ns:
            self._payload_timestamps_ns[sample.aruco_id] = sample.sim_timestamp_ns
        if self.scorer is not None:
            self.scorer.accept_payload_state(sample)

    def accept_payload_event(self, sample: PayloadEventSample) -> None:
        self.judge.accept_payload_event(sample)
        if self.scorer is not None:
            self.scorer.accept_payload_event(sample)

    def accept_mission_event(self, sample: MissionEventSample) -> None:
        decisions = self.judge.accept_mission_event(sample)
        if self.scorer is not None:
            self.scorer.accept_mission_event(sample)
        for decision in decisions:
            if self.scorer is not None:
                self.scorer.accept_judge_decision(decision)
            self._publish_decision(decision)

    def fail(self, reason: str) -> None:
        if self._finalizer is None:
            if self.scorer is not None:
                self.scorer.fail(reason)
            return
        self._finalizer.fail(reason)

    def accept_source_finished(self, sim_timestamp_ns: int) -> ScoreResult:
        if self._finalizer is None:
            raise RuntimeError("runtime finalization is not configured")
        return self._finalizer.finalize(source_timestamp_ns=sim_timestamp_ns)

    def begin_finalization(self) -> None:
        if self._finalizer is None:
            raise RuntimeError("runtime finalization is not configured")
        self._finalizer.quiesce()


__all__ = ["VfsScorekeeperRuntime"]
