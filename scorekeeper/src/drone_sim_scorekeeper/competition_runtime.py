"""Production-independent lifecycle around the physical competition scorer."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Protocol
from uuid import UUID

from .competition import (
    CompetitionScorer,
    MissionEventSample,
    PayloadEventSample,
    PayloadStateSample,
)
from .descent import GroundTruthSample
from .models import ScoreEvent, ScoreResult
from .output import persist_score_outputs
from .status import write_score_finished


class RuntimeProtocol(Protocol):
    def write_status(self, name: str, document: dict[str, object]) -> object: ...
    def write_quiescence(self, module: str) -> object: ...


def _canonical_run_id(value: object) -> str:
    if not isinstance(value, str):
        raise ValueError("run_id must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (TypeError, ValueError, AttributeError) as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise ValueError("run_id must be a canonical UUID")
    return value


class CompetitionScorekeeperRuntime:
    """Persist and publish one read-only competition result for one run."""

    def __init__(
        self,
        run_id: str,
        scorer: CompetitionScorer,
        *,
        run_directory: Path | str,
        protocol: RuntimeProtocol,
        publish: Callable[[ScoreEvent], None],
        flush: Callable[[], None],
        write_finished: Callable[[dict[str, object]], object] | None = None,
    ) -> None:
        self.run_id = _canonical_run_id(run_id)
        if not isinstance(scorer, CompetitionScorer) or scorer.run_id != self.run_id:
            raise ValueError("scorer must belong to the current run")
        self.scorer = scorer
        self.run_directory = Path(run_directory)
        self.protocol = protocol
        self._publish = publish
        self._flush = flush
        self._write_finished = write_finished or (
            lambda document: write_score_finished(
                self.run_directory, self.run_id, document
            )
        )
        self._result: ScoreResult | None = None
        self._failure_written = False
        self._ground_truth_timestamp_ns: int | None = None
        self._payload_timestamps_ns: dict[int, int | None] = {
            2: None,
            3: None,
            4: None,
        }
        self._last_payload_event_id: int | None = None
        self._last_payload_event_timestamp_ns: int | None = None
        self._home_complete_timestamp_ns: int | None = None
        self.quiescent = False

    @property
    def result(self) -> ScoreResult | None:
        return self._result

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
            and self._home_complete_timestamp_ns is not None
            and self._home_complete_timestamp_ns <= timestamp_ns
        )

    def accept_ground_truth(self, sample: GroundTruthSample) -> None:
        if self.quiescent or self._result is not None:
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
        if self.quiescent or self._result is not None:
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
        if self.quiescent or self._result is not None:
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
        if self.quiescent or self._result is not None:
            return
        if not isinstance(sample, MissionEventSample):
            raise TypeError("sample must be MissionEventSample")
        if sample.run_id == self.run_id:
            self.scorer.accept_mission_event(sample)
            if sample.phase == "HOME" and sample.state == "COMPLETE":
                self._home_complete_timestamp_ns = sample.sim_timestamp_ns

    def _write_failure(self, reason: str) -> None:
        if self._failure_written:
            return
        try:
            self.protocol.write_status(
                "runtime-failure",
                {
                    "run_id": self.run_id,
                    "module": "scorekeeper",
                    "reason": reason,
                    "diagnostic_paths": [
                        "scoring/events.jsonl",
                        "scoring/result.json",
                    ],
                },
            )
        except Exception:
            reader = getattr(self.protocol, "read_status", None)
            existing = reader("runtime-failure") if callable(reader) else None
            if not isinstance(existing, dict) or existing.get("run_id") != self.run_id:
                raise
        self._failure_written = True

    def _finalize(self, *, source_timestamp_ns: int | None = None) -> ScoreResult:
        if self._result is not None:
            return self._result
        if (
            source_timestamp_ns is not None
            and self.scorer.last_sim_timestamp_ns != source_timestamp_ns
        ):
            self.scorer.fail("source_finished_timestamp_mismatch")
        result = self.scorer.finalize()
        persist_score_outputs(self.run_directory, result)
        for event in result.events:
            self._publish(event)
        self._flush()
        if result.complete:
            self._write_finished(result.finished_status())
        else:
            self._write_failure(result.diagnostic or "score_incomplete")
        self._result = result
        return result

    def accept_source_finished(self, sim_timestamp_ns: int) -> ScoreResult:
        if type(sim_timestamp_ns) is not int or sim_timestamp_ns < 0:
            raise ValueError("source-finished timestamp must be nonnegative")
        return self._finalize(source_timestamp_ns=sim_timestamp_ns)

    def begin_finalization(self) -> None:
        if self.quiescent:
            return
        self._finalize()
        self.quiescent = True
        self.protocol.write_quiescence("scorekeeper")


__all__ = ["CompetitionScorekeeperRuntime"]
