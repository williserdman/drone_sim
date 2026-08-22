import pytest

from orchestration.lifecycle import (
    InvalidTransition,
    LifecycleEvent,
    LifecycleState,
    RunLifecycle,
)


def test_success_path():
    lifecycle = RunLifecycle.created(run_id="00000000-0000-4000-8000-000000000001")
    lifecycle = lifecycle.apply(LifecycleEvent.START)
    lifecycle = lifecycle.apply(LifecycleEvent.MODULES_READY)
    lifecycle = lifecycle.apply(LifecycleEvent.CLOCK_STARTED)
    lifecycle = lifecycle.apply(LifecycleEvent.COMPLETE, reason="mission_complete")
    assert lifecycle.state is LifecycleState.FINALIZING
    assert lifecycle.pending_terminal is LifecycleState.COMPLETED
    lifecycle = lifecycle.apply(LifecycleEvent.ARTIFACTS_FINALIZED)
    assert lifecycle.state is LifecycleState.COMPLETED


@pytest.mark.parametrize("state", [LifecycleState.STARTING, LifecycleState.READY, LifecycleState.RUNNING])
@pytest.mark.parametrize("event, terminal", [(LifecycleEvent.FAIL, LifecycleState.FAILED), (LifecycleEvent.ABORT, LifecycleState.ABORTED)])
def test_failure_and_abort_route_through_finalizing(state, event, terminal):
    lifecycle = RunLifecycle("run", state)
    result = lifecycle.apply(event, reason="cause")
    assert result.state is LifecycleState.FINALIZING
    assert result.pending_terminal is terminal
    assert result.reason == "cause"


def test_finalization_failure_yields_failed():
    lifecycle = RunLifecycle("run", LifecycleState.FINALIZING, LifecycleState.COMPLETED)
    result = lifecycle.apply(LifecycleEvent.FINALIZATION_FAILED, reason="missing artifact")
    assert result.state is LifecycleState.FAILED
    assert result.pending_terminal is None
    assert result.reason == "missing artifact"


def test_invalid_transition_mentions_current_state_and_event():
    lifecycle = RunLifecycle.created("run")
    with pytest.raises(InvalidTransition, match="CREATED.*MODULES_READY"):
        lifecycle.apply(LifecycleEvent.MODULES_READY)


def test_terminal_states_reject_every_event():
    lifecycle = RunLifecycle("run", LifecycleState.COMPLETED)
    with pytest.raises(InvalidTransition):
        lifecycle.apply(LifecycleEvent.START)
