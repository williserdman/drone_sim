"""Production-independent lifecycle around the moving-pad scorer."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

from artifacts.runtime_status import canonical_run_id

from ._finalization import _RuntimeProtocol, _ScoreFinalizer
from .competition import MissionEventSample
from .descent import GroundTruthSample
from .models import ScoreEvent, ScoreResult
from .moving_pad import LandingPadSample, MovingPadScorer


_MISSION_SEQUENCE = (("MOVING_PAD", "ARMED"), ("MOVING_PAD", "DISARMED"))


class MovingPadScorekeeperRuntime:
    """Join exact-time physical truth and finalize one moving-pad result."""

    def __init__(
        self,
        run_id: str,
        scorer: MovingPadScorer,
        *,
        run_directory: Path | str,
        protocol: _RuntimeProtocol,
        publish: Callable[[ScoreEvent], None],
        flush: Callable[[], None],
    ) -> None:
        self.run_id = canonical_run_id(run_id)
        if not isinstance(scorer, MovingPadScorer) or scorer.run_id != self.run_id:
            raise ValueError("scorer must belong to the current run")
        self.scorer = scorer
        self._finalizer = _ScoreFinalizer(
            scorer, run_directory, protocol, publish, flush
        )
        self._ground_truth: dict[int, GroundTruthSample] = {}
        self._landing_pad: dict[int, LandingPadSample] = {}
        self._paired_timestamps: set[int] = set()
        self._ground_truth_timestamp_ns: int | None = None
        self._landing_pad_timestamp_ns: int | None = None
        self._mission_events: dict[int, MissionEventSample] = {}
        self._mission_sequence: list[MissionEventSample] = []
        self._mission_failure: str | None = None
        self._disarm_observed_timestamp_ns: int | None = None

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
            and self._landing_pad_timestamp_ns is not None
            and self._landing_pad_timestamp_ns >= timestamp_ns
            and self._disarm_observed_timestamp_ns is not None
            and self._disarm_observed_timestamp_ns <= timestamp_ns
        )

    def _try_pair(self, timestamp_ns: int) -> None:
        if timestamp_ns in self._paired_timestamps:
            return
        vehicle = self._ground_truth.get(timestamp_ns)
        pad = self._landing_pad.get(timestamp_ns)
        if vehicle is None or pad is None:
            return
        self._paired_timestamps.add(timestamp_ns)
        self.scorer.accept_frame(vehicle, pad)

    def accept_ground_truth(self, sample: GroundTruthSample) -> None:
        if self.quiescent or self.result is not None:
            return
        if not isinstance(sample, GroundTruthSample):
            raise TypeError("sample must be GroundTruthSample")
        if sample.run_id != self.run_id:
            return
        timestamp = sample.sim_timestamp_ns
        previous = self._ground_truth.get(timestamp)
        if previous is not None:
            if previous != sample:
                self.scorer.fail("ground_truth_sample_conflict")
            return
        if (
            self._ground_truth_timestamp_ns is not None
            and timestamp < self._ground_truth_timestamp_ns
        ):
            self.scorer.fail("ground_truth_timestamp_regression")
        self._ground_truth[timestamp] = sample
        if self._ground_truth_timestamp_ns is None or timestamp > self._ground_truth_timestamp_ns:
            self._ground_truth_timestamp_ns = timestamp
        self._try_pair(timestamp)

    def accept_landing_pad(self, sample: LandingPadSample) -> None:
        if self.quiescent or self.result is not None:
            return
        if not isinstance(sample, LandingPadSample):
            raise TypeError("sample must be LandingPadSample")
        if sample.run_id != self.run_id:
            return
        timestamp = sample.sim_timestamp_ns
        previous = self._landing_pad.get(timestamp)
        if previous is not None:
            if previous != sample:
                self.scorer.fail("landing_pad_sample_conflict")
            return
        if (
            self._landing_pad_timestamp_ns is not None
            and timestamp < self._landing_pad_timestamp_ns
        ):
            self.scorer.fail("landing_pad_timestamp_regression")
        self._landing_pad[timestamp] = sample
        if self._landing_pad_timestamp_ns is None or timestamp > self._landing_pad_timestamp_ns:
            self._landing_pad_timestamp_ns = timestamp
        self._try_pair(timestamp)

    def accept_mission_event(self, sample: MissionEventSample) -> None:
        if self.quiescent or self.result is not None:
            return
        if not isinstance(sample, MissionEventSample):
            raise TypeError("sample must be MissionEventSample")
        if sample.run_id != self.run_id or sample.phase != "MOVING_PAD":
            return
        previous = self._mission_events.get(sample.event_id)
        if previous is not None:
            if previous != sample:
                self._mission_failure = "mission_event_sequence_invalid"
            return
        self._mission_events[sample.event_id] = sample
        if sample.state == "DISARMED" and self._disarm_observed_timestamp_ns is None:
            self._disarm_observed_timestamp_ns = sample.sim_timestamp_ns
        expected_index = len(self._mission_sequence)
        ordered = (
            expected_index < len(_MISSION_SEQUENCE)
            and (sample.phase, sample.state) == _MISSION_SEQUENCE[expected_index]
            and (
                not self._mission_sequence
                or (
                    sample.event_id > self._mission_sequence[-1].event_id
                    and sample.sim_timestamp_ns >= self._mission_sequence[-1].sim_timestamp_ns
                )
            )
        )
        if not ordered:
            self._mission_failure = "mission_event_sequence_invalid"
            return
        self._mission_sequence.append(sample)
        if sample.state == "DISARMED":
            self.scorer.accept_disarmed(sample.sim_timestamp_ns)

    def fail(self, reason: str) -> None:
        self._finalizer.fail(reason)

    def _finalize(self, *, source_timestamp_ns: int | None = None) -> ScoreResult:
        if self.result is not None:
            return self.result
        if self._mission_failure is not None:
            self.scorer.fail(self._mission_failure)
        elif self._mission_sequence and tuple(
            (event.phase, event.state) for event in self._mission_sequence
        ) != _MISSION_SEQUENCE:
            self.scorer.fail("mission_event_sequence_invalid")
        return self._finalizer.finalize(source_timestamp_ns=source_timestamp_ns)

    def accept_source_finished(self, sim_timestamp_ns: int) -> ScoreResult:
        if type(sim_timestamp_ns) is not int or sim_timestamp_ns < 0:
            raise ValueError("source-finished timestamp must be nonnegative")
        return self._finalize(source_timestamp_ns=sim_timestamp_ns)

    def begin_finalization(self) -> None:
        if self.quiescent:
            return
        self._finalize()
        self._finalizer.quiesce()


__all__ = ["MovingPadScorekeeperRuntime"]
