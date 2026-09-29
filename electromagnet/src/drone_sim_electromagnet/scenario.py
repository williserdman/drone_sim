"""Pure policy for scenarios with an inactive electromagnet."""

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
        if scenario not in {"descent_v1", "moving_pad_v1", "calibration_v1"}:
            raise ValueError(
                "inactive scenario runtime requires descent_v1, moving_pad_v1, "
                "or calibration_v1"
            )
        self._run_id = run_id
        self._magnet_id = f"{scenario.replace('_', '-')}-magnet"
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
        return InactiveScenarioEvent(
            self._run_id,
            timestamp_ns,
            magnet_id=self._magnet_id,
        )


__all__ = ["InactiveScenarioEvent", "ScenarioPolicy"]
