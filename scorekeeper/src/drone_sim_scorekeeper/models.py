"""Scorer-neutral immutable result and event contracts."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RuleResult:
    rule_id: str
    passed: bool
    awarded_points: float
    available_points: float
    evidence_ref: str

    def to_document(self) -> dict[str, object]:
        return {
            "rule_id": self.rule_id,
            "passed": self.passed,
            "awarded_points": self.awarded_points,
            "available_points": self.available_points,
            "evidence_ref": self.evidence_ref,
        }


@dataclass(frozen=True)
class ScoreEvent:
    run_id: str
    sim_timestamp_ns: int
    event_id: int
    event_type: str
    value: float
    evidence_ref: str

    def to_document(self) -> dict[str, object]:
        return {
            "run_id": self.run_id,
            "sim_timestamp_ns": self.sim_timestamp_ns,
            "event_id": self.event_id,
            "event_type": self.event_type,
            "value": self.value,
            "evidence_ref": self.evidence_ref,
        }


@dataclass(frozen=True)
class ScoreResult:
    run_id: str
    ruleset_id: str
    complete: bool
    achieved_score: float
    maximum_available_score: float
    scoring_checksum: str
    evidence_paths: tuple[str, ...]
    rule_results: tuple[RuleResult, ...]
    events: tuple[ScoreEvent, ...]
    diagnostic: str | None

    def result_document(self) -> dict[str, object]:
        return {
            "run_id": self.run_id,
            "ruleset_id": self.ruleset_id,
            "complete": self.complete,
            "achieved_score": self.achieved_score,
            "maximum_available_score": self.maximum_available_score,
            "scoring_checksum": self.scoring_checksum,
            "evidence_paths": list(self.evidence_paths),
            "rule_results": [result.to_document() for result in self.rule_results],
            "diagnostic": self.diagnostic,
        }

    def finished_status(self) -> dict[str, object]:
        if not self.events:
            raise ValueError("score result has no final event timestamp")
        return {
            "run_id": self.run_id,
            "finished": True,
            "sim_timestamp_ns": self.events[-1].sim_timestamp_ns,
        }


__all__ = ["RuleResult", "ScoreEvent", "ScoreResult"]
