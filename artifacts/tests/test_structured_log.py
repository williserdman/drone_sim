import json
from datetime import UTC, datetime
from decimal import Decimal
from io import StringIO

import pytest

from artifacts.structured_log import StructuredEvent, write_event


def test_json_line_contains_common_and_event_fields_in_compact_utf8_json():
    """Removing a common field or nesting event data incorrectly must fail."""
    event = StructuredEvent(
        run_id="run-7",
        module="artifacts",
        severity="INFO",
        event="recorder_ready",
        sim_timestamp=Decimal("12.50"),
        wall_timestamp=datetime(2026, 8, 22, 10, 30, tzinfo=UTC),
        fields={"stream": "onboard", "frames": 1},
    )

    line = event.to_json_line()

    assert line.endswith("\n")
    assert line.count("\n") == 1
    assert ", " not in line
    assert json.loads(line) == {
        "run_id": "run-7",
        "module": "artifacts",
        "severity": "INFO",
        "event": "recorder_ready",
        "sim_timestamp": 12.5,
        "wall_timestamp": "2026-08-22T10:30:00Z",
        "fields": {"stream": "onboard", "frames": 1},
    }


def test_json_line_uses_null_when_simulation_timestamp_is_unavailable():
    """Replacing an unavailable simulation timestamp with wall time must fail."""
    event = StructuredEvent(
        run_id="run-7",
        module="artifacts",
        severity="INFO",
        event="starting",
        wall_timestamp=datetime(2026, 8, 22, tzinfo=UTC),
    )

    assert json.loads(event.to_json_line())["sim_timestamp"] is None


def test_structured_event_rejects_naive_wall_timestamp():
    """Accepting a naive timestamp would make host diagnostics ambiguous."""
    with pytest.raises(ValueError, match="timezone-aware"):
        StructuredEvent(
            run_id="run-7",
            module="artifacts",
            severity="INFO",
            event="starting",
            wall_timestamp=datetime(2026, 8, 22),
        )


def test_structured_event_rejects_event_field_collisions():
    """Allowing event data to redefine common context would corrupt logs."""
    with pytest.raises(ValueError, match="common field"):
        StructuredEvent(
            run_id="run-7",
            module="artifacts",
            severity="INFO",
            event="starting",
            wall_timestamp=datetime(2026, 8, 22, tzinfo=UTC),
            fields={"run_id": "different-run"},
        )


def test_write_event_writes_and_flushes_one_complete_line():
    """Omitting the flush could lose a final lifecycle event on shutdown."""
    class FlushingStream(StringIO):
        def __init__(self) -> None:
            super().__init__()
            self.flushed = False

        def flush(self) -> None:
            self.flushed = True
            super().flush()

    stream = FlushingStream()
    event = StructuredEvent(
        run_id="run-7",
        module="artifacts",
        severity="INFO",
        event="finalizing",
        wall_timestamp=datetime(2026, 8, 22, tzinfo=UTC),
    )

    write_event(stream, event)

    assert stream.flushed is True
    assert json.loads(stream.getvalue())["event"] == "finalizing"
