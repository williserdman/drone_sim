"""Production-independent lifecycle around the physical competition scorer."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from artifacts.runtime_status import canonical_run_id

from ._finalization import _RuntimeProtocol, _ScoreFinalizer
from .competition import (
    CompetitionScorer,
    MissionEventSample,
    PayloadEventSample,
    PayloadStateSample,
)
from .descent import GroundTruthSample
from .models import ScoreEvent, ScoreResult


class CompetitionScorekeeperRuntime:
    """Persist and publish one read-only competition result for one run."""

    def __init__(
        self,
        run_id: str,
        scorer: CompetitionScorer,
        *,
        run_directory: Path | str,
        protocol: _RuntimeProtocol,
        publish: Callable[[ScoreEvent], None],
        flush: Callable[[], None],
    ) -> None:
        self.run_id = canonical_run_id(run_id)
        if not isinstance(scorer, CompetitionScorer) or scorer.run_id != self.run_id:
            raise ValueError("scorer must belong to the current run")
        self.scorer = scorer
        self._finalizer = _ScoreFinalizer(
            scorer, run_directory, protocol, publish, flush
        )
        self._ground_truth_timestamp_ns: int | None = None
        self._payload_timestamps_ns: dict[int, int | None] = {
            2: None,
            3: None,
            4: None,
        }
        self._last_payload_event_id: int | None = None
        self._last_payload_event_timestamp_ns: int | None = None
        self._home_disarmed_timestamp_ns: int | None = None
        self._home_complete_timestamp_ns: int | None = None

    @property
    def result(self) -> ScoreResult | None:
        return self._finalizer.result

    @property
    def quiescent(self) -> bool:
        return self._finalizer.quiescent

    @property
    def last_observed_ground_truth_timestamp_ns(self) -> int | None:
        return self._ground_truth_timestamp_ns

    def source_inputs_observed_through(self, timestamp_ns: int) -> bool:
        return (
            self._ground_truth_timestamp_ns is not None
            and self._ground_truth_timestamp_ns >= timestamp_ns
            and all(
                observed is not None and observed >= timestamp_ns
                for observed in self._payload_timestamps_ns.values()
            )
            and self._last_payload_event_id is not None
            and self._last_payload_event_id >= 4
            and self._last_payload_event_timestamp_ns is not None
            and self._last_payload_event_timestamp_ns <= timestamp_ns
            and self._home_disarmed_timestamp_ns is not None
            and self._home_disarmed_timestamp_ns <= timestamp_ns
            and self._home_complete_timestamp_ns is not None
            and self._home_disarmed_timestamp_ns < self._home_complete_timestamp_ns
            and self._home_complete_timestamp_ns <= timestamp_ns
        )

    def accept_ground_truth(self, sample: GroundTruthSample) -> None:
        if self.quiescent or self.result is not None:
            return
        if not isinstance(sample, GroundTruthSample):
            raise TypeError("sample must be GroundTruthSample")
        if sample.run_id != self.run_id:
            return
        if (
            self._ground_truth_timestamp_ns is None
            or sample.sim_timestamp_ns > self._ground_truth_timestamp_ns
        ):
            self._ground_truth_timestamp_ns = sample.sim_timestamp_ns
        self.scorer.accept_ground_truth(sample)

    def accept_payload_state(self, sample: PayloadStateSample) -> None:
        if self.quiescent or self.result is not None:
            return
        if not isinstance(sample, PayloadStateSample):
            raise TypeError("sample must be PayloadStateSample")
        if sample.run_id != self.run_id:
            return
        if sample.aruco_id in self._payload_timestamps_ns:
            observed = self._payload_timestamps_ns[sample.aruco_id]
            if observed is None or sample.sim_timestamp_ns > observed:
                self._payload_timestamps_ns[sample.aruco_id] = sample.sim_timestamp_ns
        self.scorer.accept_payload_state(sample)

    def accept_payload_event(self, sample: PayloadEventSample) -> None:
        if self.quiescent or self.result is not None:
            return
        if not isinstance(sample, PayloadEventSample):
            raise TypeError("sample must be PayloadEventSample")
        if sample.run_id == self.run_id:
            self.scorer.accept_payload_event(sample)
            if (
                self._last_payload_event_id is None
                or sample.event_id > self._last_payload_event_id
            ):
                self._last_payload_event_id = sample.event_id
                self._last_payload_event_timestamp_ns = sample.sim_timestamp_ns

    def accept_mission_event(self, sample: MissionEventSample) -> None:
        if self.quiescent or self.result is not None:
            return
        if not isinstance(sample, MissionEventSample):
            raise TypeError("sample must be MissionEventSample")
        if sample.run_id == self.run_id:
            self.scorer.accept_mission_event(sample)
            if sample.phase == "HOME":
                if sample.state == "DISARMED":
                    self._home_disarmed_timestamp_ns = sample.sim_timestamp_ns
                elif sample.state == "COMPLETE":
                    self._home_complete_timestamp_ns = sample.sim_timestamp_ns

    def fail(self, reason: str) -> None:
        self._finalizer.fail(reason)

    def _finalize(self, *, source_timestamp_ns: int | None = None) -> ScoreResult:
        return self._finalizer.finalize(source_timestamp_ns=source_timestamp_ns)

    def accept_source_finished(self, sim_timestamp_ns: int) -> ScoreResult:
        if type(sim_timestamp_ns) is not int or sim_timestamp_ns < 0:
            raise ValueError("source-finished timestamp must be nonnegative")
        return self._finalize(source_timestamp_ns=sim_timestamp_ns)

    def begin_finalization(self) -> None:
        self._finalizer.quiesce()


__all__ = ["CompetitionScorekeeperRuntime"]
