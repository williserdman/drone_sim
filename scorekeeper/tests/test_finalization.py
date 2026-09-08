from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import pytest

from artifacts.runtime_status import RuntimeFailureStatus, ScoreFinishedStatus
from drone_sim_scorekeeper._finalization import _ScoreFinalizer
from drone_sim_scorekeeper.models import ScoreEvent, ScoreResult


RUN_ID = "11111111-1111-4111-8111-111111111111"
EVENTS = tuple(
    ScoreEvent(
        RUN_ID,
        timestamp,
        event_id,
        event_type,
        1.0,
        f"scoring/events.jsonl#event-{event_id}",
    )
    for event_id, timestamp, event_type in (
        (0, 50_000_000, "test.point"),
        (1, 100_000_000, "score.finalized"),
    )
)


def _result(*, complete: bool = True, diagnostic: str | None = None) -> ScoreResult:
    return ScoreResult(
        RUN_ID, "test_v1", complete, 1.0, 1.0, "sha256:" + "1" * 64,
        ("scoring/events.jsonl#event-0",), (), EVENTS, diagnostic,
    )


class Scorer:
    def __init__(self, result: ScoreResult, operations: list[object]) -> None:
        self.run_id = result.run_id
        self.last_sim_timestamp_ns = result.events[-1].sim_timestamp_ns
        self.result = result
        self.diagnostic = result.diagnostic
        self.operations = operations

    def fail(self, reason: str) -> None:
        self.operations.append(("fail", reason))
        self.diagnostic = self.diagnostic or reason

    def finalize(self) -> ScoreResult:
        self.operations.append(("finalize",))
        if self.diagnostic == self.result.diagnostic:
            return self.result
        return replace(self.result, complete=False, diagnostic=self.diagnostic)


def _finalizer(tmp_path, result: ScoreResult, *, failure: str | None = None):
    operations: list[object] = []
    finalizer: _ScoreFinalizer

    def write_status(status: object) -> None:
        assert finalizer.result is None and finalizer.quiescent is False
        operations.append(("status", status))

    def write_quiescence(module: str) -> None:
        assert finalizer.result is not None and finalizer.quiescent is True
        operations.append(("quiescence", module))

    def publish(event: ScoreEvent) -> None:
        assert finalizer.result is None and finalizer.quiescent is False
        if not any(operation[0] == "persist" for operation in operations):
            assert (tmp_path / "scoring/events.jsonl").is_file()
            assert (tmp_path / "scoring/result.json").is_file()
            operations.append(("persist",))
        operations.append(("publish", event.event_id))
        if failure == "publish":
            raise RuntimeError("publish failed")

    def flush() -> None:
        operations.append(("flush",))
        if failure == "flush":
            raise RuntimeError("flush failed")

    protocol = SimpleNamespace(
        write_status=write_status, write_quiescence=write_quiescence
    )
    finalizer = _ScoreFinalizer(
        Scorer(result, operations), tmp_path, protocol, publish, flush
    )
    return finalizer, operations


def test_complete_result_orders_all_output_then_result_and_quiescence(tmp_path):
    """Moving result visibility or quiescence ahead of durable output breaks shutdown."""
    expected = _result()
    finalizer, operations = _finalizer(tmp_path, expected)

    assert finalizer.finalize(source_timestamp_ns=100_000_000) is expected
    finalizer.quiesce()

    assert finalizer.result is expected and finalizer.quiescent is True
    assert operations == [
        ("finalize",), ("persist",), ("publish", 0), ("publish", 1),
        ("flush",), ("status", ScoreFinishedStatus(RUN_ID, 100_000_000)),
        ("quiescence", "scorekeeper"),
    ]


def test_incomplete_result_writes_typed_failure_before_becoming_visible(tmp_path):
    expected = _result(complete=False, diagnostic="input_invalid")
    finalizer, operations = _finalizer(tmp_path, expected)

    assert finalizer.finalize() is expected
    assert operations[-1] == (
        "status",
        RuntimeFailureStatus(
            RUN_ID, "scorekeeper", "input_invalid",
            ("scoring/events.jsonl", "scoring/result.json"),
        ),
    )


def test_duplicate_finalization_returns_cached_result_without_more_output(tmp_path):
    finalizer, operations = _finalizer(tmp_path, _result())
    first = finalizer.finalize()
    before_duplicate = list(operations)

    assert finalizer.finalize(source_timestamp_ns=0) is first
    assert operations == before_duplicate


@pytest.mark.parametrize("failure", ["publish", "flush"])
def test_delivery_failure_leaves_result_hidden_and_skips_status(tmp_path, failure):
    finalizer, operations = _finalizer(tmp_path, _result(), failure=failure)

    with pytest.raises(RuntimeError, match=f"{failure} failed"):
        finalizer.finalize()

    assert finalizer.result is None and finalizer.quiescent is False
    assert all(operation[0] != "status" for operation in operations)
    assert (("flush",) in operations) is (failure == "flush")


def test_source_timestamp_mismatch_and_failure_are_first_wins(tmp_path):
    finalizer, operations = _finalizer(tmp_path, _result())
    finalizer.fail("input_invalid")

    result = finalizer.finalize(source_timestamp_ns=100_000_001)

    assert result.diagnostic == "input_invalid"
    assert operations[:3] == [
        ("fail", "input_invalid"),
        ("fail", "source_finished_timestamp_mismatch"),
        ("finalize",),
    ]


@pytest.mark.parametrize("reason", [None, "", True])
def test_failure_requires_a_nonempty_exact_string(tmp_path, reason):
    finalizer, _operations = _finalizer(tmp_path, _result())

    with pytest.raises(ValueError, match="failure reason must be nonempty"):
        finalizer.fail(reason)
