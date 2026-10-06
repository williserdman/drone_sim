from __future__ import annotations

from types import SimpleNamespace

import pytest

from drone_sim_companion.mission_plan import Condition
from drone_sim_companion.mission import Telemetry
from drone_sim_companion.vfs_operations import (
    ConfiguredRangefinder,
    PayloadActions,
    PhaseExchange,
    VehicleStatusStream,
)


def test_phase_check_detail_is_canonical_and_eligibility_is_frozen() -> None:
    events = []
    def publish(*fields):
        events.append(fields)
        return len(events) - 1

    exchange = PhaseExchange("run", publish)
    exchange.check("FM1", "D", "request-1", 50)
    assert events == [(
        "FM1",
        "CHECK_REQUESTED",
        '{"checkpoint":"D","request_id":"request-1"}',
        50,
    )]

    exchange.check("FM2", "", "fm2", 60)
    exchange.accept(SimpleNamespace(
        run_id="run", outcome=1, APPROVED=1, REJECTED=2, ERROR=3,
        request_id="fm2", phase="FM2", checkpoint="",
        request_event_id=1, eligible_aruco_ids=[2, 4], reason_code="APPROVED",
    ))
    assert exchange.evaluate(Condition("any_payload_eligible")).matched
    assert exchange.evaluate(Condition("payload_eligible", 2)).matched
    assert not exchange.evaluate(Condition("payload_eligible", 3)).matched


def test_phase_exchange_rejects_condition_before_current_run_fm2_decision() -> None:
    exchange = PhaseExchange("run", lambda *_fields: 0)
    exchange.check("FM2", "", "fm2", 0)
    exchange.accept(SimpleNamespace(
        run_id="foreign", outcome=1, APPROVED=1, REJECTED=2, ERROR=3,
        request_id="fm2", phase="FM2", checkpoint="",
        request_event_id=0, eligible_aruco_ids=[2], reason_code="APPROVED",
    ))
    with pytest.raises(RuntimeError, match="eligibility.*unavailable"):
        exchange.evaluate(Condition("any_payload_eligible"))


def test_rangefinder_rejects_stale_and_malformed_physical_samples() -> None:
    rangefinder = ConfiguredRangefinder()
    with pytest.raises(ValueError, match="exactly one"):
        rangefinder.accept(SimpleNamespace(ranges=[], range_min=0.1, range_max=20), 0)
    rangefinder.accept(
        SimpleNamespace(ranges=[4.572], range_min=0.1, range_max=20), 100
    )
    assert rangefinder.latest(500_000_100).distance_m == pytest.approx(4.572)
    with pytest.raises(RuntimeError, match="fresh"):
        rangefinder.latest(500_000_101)


def test_payload_action_requires_command_and_response_sequence_correlation() -> None:
    class Request:
        ATTACH = 1
        RELEASE = 2

    class Future:
        def __init__(self) -> None:
            self.response = None

        def done(self):
            return self.response is not None

        def result(self):
            return self.response

        def cancel(self):
            pass

    future = Future()
    client = SimpleNamespace(call_async=lambda _request: future)
    actions = PayloadActions("run", client, SimpleNamespace(Request=Request))
    actions.request("attach", 2, "attach:1", None)
    future.response = SimpleNamespace(
        command_id="attach:1", accepted=True, response_sequence=0,
        code="OK", detail="",
    )
    assert not actions.confirmation("attach:1").confirmed
    future.response.response_sequence = 1
    assert actions.confirmation("attach:1").confirmed


def test_vehicle_status_stream_samples_complete_observed_state_once_per_grid() -> None:
    stream = VehicleStatusStream(interval_ns=100_000_000)
    stream.observe(Telemetry(0, mode="GUIDED", armed=True))
    assert stream.sample(100_000_000) is None

    stream.observe(Telemetry(100_000_000, failsafe=False))
    first = stream.sample(100_000_000)
    assert first is not None
    assert (first.timestamp_ns, first.sequence_id, first.mode, first.armed) == (
        100_000_000, 0, "GUIDED", True,
    )
    assert not first.failsafe_active and first.failsafe_reason == ""
    assert stream.sample(100_000_000) is None
    assert stream.sample(199_999_999) is None

    # Publishing continues from the latest observed state without MAVLink input.
    second = stream.sample(200_000_000)
    assert second is not None
    assert (second.timestamp_ns, second.sequence_id, second.mode) == (
        200_000_000, 1, "GUIDED",
    )


def test_phase_check_waits_for_one_full_published_disarmed_interval() -> None:
    stream = VehicleStatusStream()
    events = []

    def publish(*fields):
        events.append(fields)
        return len(events) - 1

    exchange = PhaseExchange("run", publish, check_ready=stream.disarmed_ready)
    stream.observe(Telemetry(
        47_900_000_000, mode="LAND", armed=True, failsafe=False
    ))
    assert stream.sample(47_900_000_000).armed
    exchange.check("FM1", "D", "request-1", 47_950_000_000)
    assert events == []

    stream.observe(Telemetry(47_950_000_000, armed=False))
    exchange.advance(47_950_000_000)
    assert events == []
    assert stream.sample(48_000_000_000).armed is False
    exchange.advance(48_000_000_000)
    assert events == []
    assert stream.sample(48_100_000_000).armed is False
    exchange.advance(48_100_000_000)

    assert events == [(
        "FM1",
        "CHECK_REQUESTED",
        '{"checkpoint":"D","request_id":"request-1"}',
        48_100_000_000,
    )]
    exchange.advance(48_200_000_000)
    assert len(events) == 1
    exchange.accept(SimpleNamespace(
        run_id="run", outcome=1, APPROVED=1, REJECTED=2, ERROR=3,
        request_id="request-1", phase="FM1", checkpoint="D",
        request_event_id=0, eligible_aruco_ids=[], reason_code="APPROVED",
    ))
    assert exchange.decision("request-1").request_event_id == 0


def test_disarmed_readiness_requires_fresh_published_state_and_resets_on_armed() -> None:
    stream = VehicleStatusStream()
    assert not stream.disarmed_ready(100_000_000)
    stream.observe(Telemetry(100_000_000, mode="LAND", armed=False, failsafe=False))
    assert not stream.disarmed_ready(200_000_000)
    stream.sample(200_000_000)
    assert not stream.disarmed_ready(200_000_000)
    assert stream.disarmed_ready(300_000_000)
    assert not stream.disarmed_ready(700_000_001)
    stream.observe(Telemetry(700_000_001, armed=True))
    assert not stream.disarmed_ready(700_000_001)
