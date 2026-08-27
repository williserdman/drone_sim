"""No-clobber persistence for finalized scoring evidence."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path

from .models import ScoreResult


@dataclass(frozen=True)
class ScoreOutputPaths:
    events: Path
    result: Path


def _json_line(document: dict[str, object]) -> bytes:
    return (
        json.dumps(
            document,
            allow_nan=False,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    ).encode("utf-8")


def _write_new(path: Path, payload: bytes) -> None:
    with path.open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def persist_score_outputs(run_directory: Path | str, result: ScoreResult) -> ScoreOutputPaths:
    """Persist ordered events and a manifest-compatible result without overwrite."""
    if not isinstance(result, ScoreResult):
        raise TypeError("result must be ScoreResult")
    scoring = Path(run_directory) / "scoring"
    scoring.mkdir(parents=True, exist_ok=True)
    events_path = scoring / "events.jsonl"
    result_path = scoring / "result.json"
    if events_path.exists():
        raise FileExistsError(events_path)
    if result_path.exists():
        raise FileExistsError(result_path)
    events_payload = b"".join(_json_line(event.to_document()) for event in result.events)
    _write_new(events_path, events_payload)
    try:
        _write_new(result_path, _json_line(result.result_document()))
    finally:
        descriptor = os.open(scoring, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    return ScoreOutputPaths(events_path, result_path)


__all__ = ["ScoreOutputPaths", "persist_score_outputs"]
