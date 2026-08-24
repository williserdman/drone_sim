"""Pure policy for the inactive descent_v1 electromagnet scenario."""

from __future__ import annotations

from dataclasses import dataclass
from uuid import UUID


@dataclass(frozen=True)
class InactiveScenarioEvent:
    run_id: str
    timestamp_ns: int
    event_id: int = 0
    magnet_id: str = "descent-v1-magnet"
    state: str = "INACTIVE"


class ScenarioPolicy:
    def __init__(self, *, run_id: str, scenario: str = "descent_v1") -> None:
        try:
            parsed = UUID(run_id)
        except (ValueError, TypeError, AttributeError) as error:
            raise ValueError("run_id must be a canonical UUID") from error
        if str(parsed) != run_id:
            raise ValueError("run_id must be a canonical UUID")
        if scenario != "descent_v1":
            raise ValueError("only descent_v1 is supported by the inactive scenario runtime")
        self._run_id = run_id
        self._last_timestamp_ns: int | None = None
        self._published = False
        self._failed = False

    def observe_clock(self, timestamp_ns: int) -> InactiveScenarioEvent | None:
        if self._failed:
            raise RuntimeError("scenario policy already failed")
        if (
            not isinstance(timestamp_ns, int)
            or isinstance(timestamp_ns, bool)
            or timestamp_ns < 0
        ):
            self._failed = True
            raise ValueError("invalid simulation timestamp")
        if self._last_timestamp_ns is not None and timestamp_ns < self._last_timestamp_ns:
            self._failed = True
            raise ValueError("simulation timestamp regressed")
        self._last_timestamp_ns = timestamp_ns
        if self._published:
            return None
        self._published = True
        return InactiveScenarioEvent(self._run_id, timestamp_ns)


__all__ = ["InactiveScenarioEvent", "ScenarioPolicy"]
