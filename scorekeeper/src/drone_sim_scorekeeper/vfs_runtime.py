"""Runtime routing for the read-only VFS 2027 judge and scorer."""

from __future__ import annotations

from collections.abc import Callable
import json
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
        self._ground_truth_buffer: dict[int, GroundTruthSample] = {}
        self._vehicle_status_buffer: dict[int, VehicleStatusSample] = {}
        self._payload_buffer: dict[int, dict[int, PayloadStateSample]] = {}
        self._status_grid_offset_ns: int | None = None
        self._last_epoch_timestamp_ns: int | None = None
        self._latest_observed_timestamp_ns = 0
        self._latest_ground_truth_received_ns: int | None = None
        self._latest_vehicle_status_received_ns: int | None = None
        self._latest_payload_received_ns: dict[int, int] = {}
        self._pending_checks: list[MissionEventSample] = []
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
        if (
            self._latest_ground_truth_received_ns is not None
            and sample.sim_timestamp_ns <= self._latest_ground_truth_received_ns
        ):
            raise ValueError("ground truth receive timestamps must increase")
        self._latest_ground_truth_received_ns = sample.sim_timestamp_ns
        self._ground_truth_buffer[sample.sim_timestamp_ns] = sample
        self._observe_timestamp(sample.sim_timestamp_ns)
        self._drain_physical_epochs()

    def accept_vehicle_status(self, sample: VehicleStatusSample) -> None:
        if (
            self._latest_vehicle_status_received_ns is not None
            and sample.sim_timestamp_ns <= self._latest_vehicle_status_received_ns
        ):
            raise ValueError("vehicle status receive timestamps must increase")
        self._latest_vehicle_status_received_ns = sample.sim_timestamp_ns
        self._vehicle_status_buffer[sample.sim_timestamp_ns] = sample
        if self._status_grid_offset_ns is None:
            self._status_grid_offset_ns = sample.sim_timestamp_ns % 100_000_000
        self._observe_timestamp(sample.sim_timestamp_ns)
        self._drain_physical_epochs()

    def accept_payload_state(self, sample: PayloadStateSample) -> None:
        previous = self._latest_payload_received_ns.get(sample.aruco_id)
        if previous is not None and sample.sim_timestamp_ns <= previous:
            raise ValueError("payload receive timestamps must increase per marker")
        self._latest_payload_received_ns[sample.aruco_id] = sample.sim_timestamp_ns
        self._payload_buffer.setdefault(sample.sim_timestamp_ns, {})[sample.aruco_id] = sample
        self._observe_timestamp(sample.sim_timestamp_ns)
        self._drain_physical_epochs()

    def accept_payload_event(self, sample: PayloadEventSample) -> None:
        self.judge.accept_payload_event(sample)
        if self.scorer is not None:
            self.scorer.accept_payload_event(sample)

    def accept_mission_event(self, sample: MissionEventSample) -> None:
        if sample.state != "CHECK_REQUESTED":
            self._accept_mission_event_now(sample)
            return
        try:
            detail = json.loads(sample.detail)
            valid_request = (
                isinstance(detail, dict)
                and set(detail) == {"checkpoint", "request_id"}
                and isinstance(detail["checkpoint"], str)
                and isinstance(detail["request_id"], str)
                and bool(detail["request_id"])
            )
        except (TypeError, ValueError):
            valid_request = False
        if not valid_request:
            self._accept_mission_event_now(sample)
            return
        if sample in self._pending_checks:
            return
        if (
            self._last_epoch_timestamp_ns is not None
            and self._last_epoch_timestamp_ns >= sample.sim_timestamp_ns
        ):
            self._accept_mission_event_now(sample)
            return
        self._pending_checks.append(sample)
        self._resolve_pending_checks()

    def _accept_mission_event_now(
        self, sample: MissionEventSample, *, missing_evidence: bool = False
    ) -> None:
        decisions = (
            self.judge.reject_mission_event_missing_evidence(sample)
            if missing_evidence
            else self.judge.accept_mission_event(sample)
        )
        if self.scorer is not None:
            self.scorer.accept_mission_event(sample)
        for decision in decisions:
            if self.scorer is not None:
                self.scorer.accept_judge_decision(decision)
            self._publish_decision(decision)

    def _observe_timestamp(self, timestamp_ns: int) -> None:
        self._latest_observed_timestamp_ns = max(
            self._latest_observed_timestamp_ns, timestamp_ns
        )

    def _epoch_complete(self, timestamp_ns: int) -> bool:
        payloads = self._payload_buffer.get(timestamp_ns, {})
        if set(payloads) != set(self._payload_timestamps_ns):
            return False
        if self._status_grid_offset_ns is None:
            return False
        requires_status = timestamp_ns % 100_000_000 == self._status_grid_offset_ns
        return not requires_status or timestamp_ns in self._vehicle_status_buffer

    def _drain_physical_epochs(self) -> None:
        while self._ground_truth_buffer:
            timestamp_ns = min(self._ground_truth_buffer)
            if any(
                sample.sim_timestamp_ns < timestamp_ns
                for sample in self._pending_checks
            ):
                break
            if not self._epoch_complete(timestamp_ns):
                break
            truth = self._ground_truth_buffer.pop(timestamp_ns)
            payloads = self._payload_buffer.pop(timestamp_ns)
            self.judge.accept_ground_truth(truth)
            self._ground_truth_timestamp_ns = timestamp_ns
            if self.scorer is not None:
                self.scorer.accept_ground_truth(truth)
            for marker in sorted(payloads):
                sample = payloads[marker]
                self.judge.accept_payload_state(sample)
                self._payload_timestamps_ns[marker] = timestamp_ns
                if self.scorer is not None:
                    self.scorer.accept_payload_state(sample)
            status = self._vehicle_status_buffer.pop(timestamp_ns, None)
            if status is not None:
                self.judge.accept_vehicle_status(status)
                self._vehicle_status_timestamp_ns = timestamp_ns
                if self.scorer is not None:
                    self.scorer.accept_vehicle_status(status)
            self._last_epoch_timestamp_ns = timestamp_ns
            self._publish_attempt(self.judge.attempt_status)
            self._resolve_pending_checks()
        self._resolve_pending_checks()
        buffered_timestamps = (
            set(self._ground_truth_buffer)
            | set(self._vehicle_status_buffer)
            | set(self._payload_buffer)
        )
        if (
            buffered_timestamps
            and self._latest_observed_timestamp_ns - min(buffered_timestamps)
            > 500_000_000
        ):
            raise ValueError("VFS physical epoch remained incomplete for over 500 ms")

    def _resolve_pending_checks(self, *, force: bool = False) -> None:
        remaining: list[MissionEventSample] = []
        for sample in self._pending_checks:
            epoch_ready = (
                self._last_epoch_timestamp_ns is not None
                and self._last_epoch_timestamp_ns >= sample.sim_timestamp_ns
            )
            expired = (
                self._latest_observed_timestamp_ns
                > sample.sim_timestamp_ns + 500_000_000
            )
            if epoch_ready:
                self._accept_mission_event_now(sample)
            elif expired or force:
                self._accept_mission_event_now(sample, missing_evidence=True)
            else:
                remaining.append(sample)
        self._pending_checks = remaining

    def fail(self, reason: str) -> None:
        if self._finalizer is None:
            if self.scorer is not None:
                self.scorer.fail(reason)
            return
        self._finalizer.fail(reason)

    def accept_source_finished(self, sim_timestamp_ns: int) -> ScoreResult:
        if self._finalizer is None:
            raise RuntimeError("runtime finalization is not configured")
        self._resolve_pending_checks(force=True)
        return self._finalizer.finalize(source_timestamp_ns=sim_timestamp_ns)

    def begin_finalization(self) -> None:
        if self._finalizer is None:
            raise RuntimeError("runtime finalization is not configured")
        self._finalizer.quiesce()


__all__ = ["VfsScorekeeperRuntime"]
