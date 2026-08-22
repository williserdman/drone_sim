"""JSON Lines events shared by all simulation modules."""

from dataclasses import dataclass, field
from datetime import datetime, timezone
from decimal import Decimal
import json
import math
from typing import Any, Mapping, TextIO


COMMON_FIELDS = frozenset(
    {"run_id", "module", "severity", "event", "sim_timestamp", "wall_timestamp"}
)


def _reject_non_finite_numbers(value: Any) -> None:
    if isinstance(value, Decimal) and not value.is_finite():
        raise ValueError("structured event values must be finite")
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("structured event values must be finite")
    if isinstance(value, Mapping):
        for key, item in value.items():
            _reject_non_finite_numbers(key)
            _reject_non_finite_numbers(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _reject_non_finite_numbers(item)


@dataclass(frozen=True)
class StructuredEvent:
    run_id: str
    module: str
    severity: str
    event: str
    wall_timestamp: datetime
    sim_timestamp: Decimal | float | None = None
    fields: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.wall_timestamp.tzinfo is None or self.wall_timestamp.utcoffset() is None:
            raise ValueError("wall_timestamp must be timezone-aware")
        collisions = COMMON_FIELDS.intersection(self.fields)
        if collisions:
            raise ValueError(f"event fields cannot collide with common field(s): {', '.join(sorted(collisions))}")
        _reject_non_finite_numbers(self.sim_timestamp)
        _reject_non_finite_numbers(self.fields)

    def to_json_line(self) -> str:
        wall_timestamp = self.wall_timestamp.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")
        payload = {
            "run_id": self.run_id,
            "module": self.module,
            "severity": self.severity,
            "event": self.event,
            "sim_timestamp": float(self.sim_timestamp) if self.sim_timestamp is not None else None,
            "wall_timestamp": wall_timestamp,
            "fields": dict(self.fields),
        }
        return json.dumps(
            payload,
            allow_nan=False,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ) + "\n"


def write_event(stream: TextIO, event: StructuredEvent) -> None:
    """Write one complete event line and make it visible to the log collector."""
    stream.write(event.to_json_line())
    stream.flush()
