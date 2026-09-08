"""Shared private score-result finalization lifecycle."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Protocol

from artifacts.runtime_status import (
    RuntimeFailureStatus,
    RuntimeStatus,
    ScoreFinishedStatus,
)

from .models import ScoreEvent, ScoreResult
from .output import persist_score_outputs


class _Scorer(Protocol):
    run_id: str
    last_sim_timestamp_ns: int

    def fail(self, reason: str) -> None: ...
    def finalize(self) -> ScoreResult: ...


class _RuntimeProtocol(Protocol):
    def write_status(self, status: RuntimeStatus) -> object: ...
    def write_quiescence(self, module: str) -> object: ...


class _ScoreFinalizer:
    def __init__(
        self,
        scorer: _Scorer,
        run_directory: Path | str,
        protocol: _RuntimeProtocol,
        publish: Callable[[ScoreEvent], None],
        flush: Callable[[], None],
    ) -> None:
        self._scorer = scorer
        self._run_directory = Path(run_directory)
        self._protocol = protocol
        self._publish = publish
        self._flush = flush
        self._result: ScoreResult | None = None
        self._failure_written = False
        self._quiescent = False

    @property
    def result(self) -> ScoreResult | None:
        return self._result

    @property
    def quiescent(self) -> bool:
        return self._quiescent

    def fail(self, reason: str) -> None:
        if type(reason) is not str or not reason:
            raise ValueError("failure reason must be nonempty")
        if self._quiescent or self._result is not None:
            return
        self._scorer.fail(reason)

    def finalize(self, *, source_timestamp_ns: int | None = None) -> ScoreResult:
        if self._result is not None:
            return self._result
        if (
            source_timestamp_ns is not None
            and self._scorer.last_sim_timestamp_ns != source_timestamp_ns
        ):
            self._scorer.fail("source_finished_timestamp_mismatch")
        result = self._scorer.finalize()
        persist_score_outputs(self._run_directory, result)
        for event in result.events:
            self._publish(event)
        self._flush()
        if result.complete:
            self._protocol.write_status(
                ScoreFinishedStatus(
                    self._scorer.run_id,
                    result.finished_status()["sim_timestamp_ns"],
                )
            )
        elif not self._failure_written:
            self._protocol.write_status(
                RuntimeFailureStatus(
                    self._scorer.run_id,
                    "scorekeeper",
                    result.diagnostic or "score_incomplete",
                    ("scoring/events.jsonl", "scoring/result.json"),
                )
            )
            self._failure_written = True
        self._result = result
        return result

    def quiesce(self) -> None:
        if self._quiescent:
            return
        self.finalize()
        self._quiescent = True
        self._protocol.write_quiescence("scorekeeper")


__all__ = ["_ScoreFinalizer"]
