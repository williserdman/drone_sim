from __future__ import annotations

import pytest

from drone_sim_ardupilot.state import Event, RuntimeState, transition


def test_runtime_becomes_ready_only_after_json_exchange_and_mavlink_heartbeat() -> None:
    state = RuntimeState.STARTING
    state = transition(state, Event.PROCESS_STARTED)
    state = transition(state, Event.JSON_FRAME)
    assert state is RuntimeState.WAITING_HEARTBEAT
    state = transition(state, Event.MAVLINK_HEARTBEAT)
    assert state is RuntimeState.READY


def test_runtime_peer_loss_fails_closed_after_exchange_started() -> None:
    state = transition(RuntimeState.STARTING, Event.PROCESS_STARTED)
    state = transition(state, Event.JSON_FRAME)

    assert transition(state, Event.PEER_LOST) is RuntimeState.FAILED


def test_finalize_is_quiescent_only_after_process_exit() -> None:
    state = RuntimeState.READY
    state = transition(state, Event.FINALIZE)
    assert state is RuntimeState.STOPPING
    assert transition(state, Event.PROCESS_EXITED) is RuntimeState.QUIESCENT


def test_invalid_transition_is_rejected_instead_of_repaired() -> None:
    with pytest.raises(ValueError, match="invalid runtime transition"):
        transition(RuntimeState.STARTING, Event.MAVLINK_HEARTBEAT)
