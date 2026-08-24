from __future__ import annotations

import pytest

from drone_sim_electromagnet.scenario import ScenarioPolicy


def test_descent_v1_emits_one_truthful_inactive_event_at_first_clock() -> None:
    policy = ScenarioPolicy(run_id="00000000-0000-4000-8000-000000000001")
    event = policy.observe_clock(0)
    assert event is not None
    assert event.run_id == "00000000-0000-4000-8000-000000000001"
    assert event.timestamp_ns == 0
    assert event.event_id == 0
    assert event.magnet_id == "descent-v1-magnet"
    assert event.state == "INACTIVE"
    assert policy.observe_clock(50_000_000) is None


def test_clock_regression_fails_and_cannot_repair_or_republish() -> None:
    policy = ScenarioPolicy(run_id="00000000-0000-4000-8000-000000000001")
    policy.observe_clock(50_000_000)
    with pytest.raises(ValueError, match="regressed"):
        policy.observe_clock(0)
    with pytest.raises(RuntimeError, match="failed"):
        policy.observe_clock(100_000_000)


@pytest.mark.parametrize("timestamp_ns", [-1, True, 1.5])
def test_invalid_clock_fails_closed(timestamp_ns: object) -> None:
    policy = ScenarioPolicy(run_id="00000000-0000-4000-8000-000000000001")
    with pytest.raises(ValueError, match="timestamp"):
        policy.observe_clock(timestamp_ns)  # type: ignore[arg-type]


def test_only_descent_v1_is_supported() -> None:
    with pytest.raises(ValueError, match="descent_v1"):
        ScenarioPolicy(
            run_id="00000000-0000-4000-8000-000000000001",
            scenario="active_magnet",
        )
