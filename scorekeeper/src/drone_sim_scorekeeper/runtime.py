"""Production-independent lifecycle around the pure descent scorer."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from artifacts.runtime_status import canonical_run_id

from ._finalization import _RuntimeProtocol, _ScoreFinalizer
from .descent import DescentScorer, GroundTruthSample
from .models import ScoreEvent, ScoreResult


@dataclass(frozen=True)
class ScenarioSample:
    run_id: str
    sim_timestamp_ns: int
    event_id: int
    magnet_id: str
    state: str

    def __post_init__(self) -> None:
        canonical_run_id(self.run_id)
        if (
            not isinstance(self.sim_timestamp_ns, int)
            or isinstance(self.sim_timestamp_ns, bool)
            or self.sim_timestamp_ns < 0
            or not isinstance(self.event_id, int)
            or isinstance(self.event_id, bool)
            or self.event_id < 0
            or not isinstance(self.magnet_id, str)
            or not self.magnet_id
            or not isinstance(self.state, str)
            or not self.state
        ):
            raise ValueError("scenario sample has invalid fields")


class ScorekeeperRuntime:
    """Persist and publish exactly one fail-closed score for one run."""

    def __init__(
        self,
        run_id: str,
        scorer: DescentScorer,
        *,
        run_directory: Path | str,
        protocol: _RuntimeProtocol,
        publish: Callable[[ScoreEvent], None],
        flush: Callable[[], None],
    ) -> None:
        self.run_id = canonical_run_id(run_id)
        if not isinstance(scorer, DescentScorer) or scorer.run_id != self.run_id:
            raise ValueError("scorer must belong to the current run")
        self.scorer = scorer
        self._finalizer = _ScoreFinalizer(
            scorer, run_directory, protocol, publish, flush
        )
        self._scenario_events: dict[int, ScenarioSample] = {}
        self._scenario_failure: str | None = None
        self._last_observed_ground_truth_timestamp_ns: int | None = None

    @property
    def result(self) -> ScoreResult | None:
        return self._finalizer.result

    @property
    def quiescent(self) -> bool:
        return self._finalizer.quiescent

    @property
    def last_observed_ground_truth_timestamp_ns(self) -> int | None:
        return self._last_observed_ground_truth_timestamp_ns

    def source_inputs_observed_through(self, timestamp_ns: int) -> bool:
        observed = self._last_observed_ground_truth_timestamp_ns
        return (
            bool(self._scenario_events)
            and observed is not None
            and observed >= timestamp_ns
        )

    def accept_scenario(self, sample: ScenarioSample) -> None:
        if self.quiescent or self.result is not None:
            return
        if not isinstance(sample, ScenarioSample):
            raise TypeError("sample must be ScenarioSample")
        if sample.run_id != self.run_id:
            return
        previous = self._scenario_events.get(sample.event_id)
        if previous is not None:
            if previous != sample and self._scenario_failure is None:
                self._scenario_failure = "scenario_event_conflict"
            return
        self._scenario_events[sample.event_id] = sample
        if sample.state != "INACTIVE" and self._scenario_failure is None:
            self._scenario_failure = "scenario_not_inactive"

    def accept_ground_truth(self, sample: GroundTruthSample) -> None:
        if self.quiescent or self.result is not None:
            return
        if not isinstance(sample, GroundTruthSample):
            raise TypeError("sample must be GroundTruthSample")
        if sample.run_id != self.run_id:
            return
        observed = self._last_observed_ground_truth_timestamp_ns
        if observed is None or sample.sim_timestamp_ns > observed:
            self._last_observed_ground_truth_timestamp_ns = sample.sim_timestamp_ns
        self.scorer.accept(sample)

    def fail(self, reason: str) -> None:
        self._finalizer.fail(reason)

    def _finalize(self, *, source_timestamp_ns: int | None = None) -> ScoreResult:
        if self.result is not None:
            return self.result
        if not self._scenario_events:
            self.scorer.fail("scenario_initialization_missing")
        elif self._scenario_failure is not None:
            self.scorer.fail(self._scenario_failure)
        return self._finalizer.finalize(source_timestamp_ns=source_timestamp_ns)

    def accept_source_finished(self, sim_timestamp_ns: int) -> ScoreResult:
        if (
            not isinstance(sim_timestamp_ns, int)
            or isinstance(sim_timestamp_ns, bool)
            or sim_timestamp_ns < 0
        ):
            raise ValueError("source-finished timestamp must be nonnegative")
        return self._finalize(source_timestamp_ns=sim_timestamp_ns)

    def begin_finalization(self) -> None:
        if self.quiescent:
            return
        self._finalize()
        self._finalizer.quiesce()


__all__ = ["ScenarioSample", "ScorekeeperRuntime"]
