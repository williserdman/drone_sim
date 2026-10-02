"""Production-independent lifecycle around the calibration scorer."""

from collections.abc import Callable
from pathlib import Path

from artifacts.runtime_status import canonical_run_id

from ._finalization import _RuntimeProtocol, _ScoreFinalizer
from .calibration import CalibrationScorer
from .descent import GroundTruthSample
from .models import ScoreEvent, ScoreResult


class CalibrationScorekeeperRuntime:
    def __init__(self, run_id: str, scorer: CalibrationScorer, *, run_directory: Path | str, protocol: _RuntimeProtocol, publish: Callable[[ScoreEvent], None], flush: Callable[[], None]) -> None:
        self.run_id = canonical_run_id(run_id)
        if not isinstance(scorer, CalibrationScorer) or scorer.run_id != self.run_id:
            raise ValueError("scorer must belong to the current run")
        self.scorer = scorer
        self._finalizer = _ScoreFinalizer(scorer, run_directory, protocol, publish, flush)
        self._last_timestamp_ns: int | None = None

    @property
    def result(self) -> ScoreResult | None: return self._finalizer.result
    @property
    def quiescent(self) -> bool: return self._finalizer.quiescent
    @property
    def last_observed_ground_truth_timestamp_ns(self) -> int | None: return self._last_timestamp_ns

    def source_inputs_observed_through(self, timestamp_ns: int) -> bool:
        return self._last_timestamp_ns is not None and self._last_timestamp_ns >= timestamp_ns

    def accept_ground_truth(self, sample: GroundTruthSample) -> None:
        if self.quiescent or self.result is not None:
            return
        if not isinstance(sample, GroundTruthSample):
            raise TypeError("sample must be GroundTruthSample")
        if sample.run_id != self.run_id:
            return
        if self._last_timestamp_ns is None or sample.sim_timestamp_ns > self._last_timestamp_ns:
            self._last_timestamp_ns = sample.sim_timestamp_ns
        self.scorer.accept(sample)

    def fail(self, reason: str) -> None: self._finalizer.fail(reason)
    def accept_source_finished(self, sim_timestamp_ns: int) -> ScoreResult:
        if type(sim_timestamp_ns) is not int or sim_timestamp_ns < 0:
            raise ValueError("source-finished timestamp must be nonnegative")
        return self._finalizer.finalize(source_timestamp_ns=sim_timestamp_ns)
    def begin_finalization(self) -> None:
        if not self.quiescent:
            self._finalizer.finalize()
            self._finalizer.quiesce()


__all__ = ["CalibrationScorekeeperRuntime"]
