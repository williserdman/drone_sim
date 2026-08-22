"""Immutable run lifecycle state machine."""

from dataclasses import dataclass
from enum import Enum


class LifecycleState(str, Enum):
    CREATED = "CREATED"
    STARTING = "STARTING"
    READY = "READY"
    RUNNING = "RUNNING"
    FINALIZING = "FINALIZING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


class LifecycleEvent(str, Enum):
    START = "START"
    MODULES_READY = "MODULES_READY"
    CLOCK_STARTED = "CLOCK_STARTED"
    COMPLETE = "COMPLETE"
    FAIL = "FAIL"
    ABORT = "ABORT"
    ARTIFACTS_FINALIZED = "ARTIFACTS_FINALIZED"
    FINALIZATION_FAILED = "FINALIZATION_FAILED"


class InvalidTransition(ValueError):
    """Raised when an event is not valid for the current state."""


@dataclass(frozen=True)
class RunLifecycle:
    run_id: str
    state: LifecycleState = LifecycleState.CREATED
    pending_terminal: LifecycleState | None = None
    reason: str = ""

    @classmethod
    def created(cls, run_id: str) -> "RunLifecycle":
        return cls(run_id=run_id)

    def apply(self, event: LifecycleEvent, reason: str = "") -> "RunLifecycle":
        transition = {
            (LifecycleState.CREATED, LifecycleEvent.START): LifecycleState.STARTING,
            (LifecycleState.STARTING, LifecycleEvent.MODULES_READY): LifecycleState.READY,
            (LifecycleState.READY, LifecycleEvent.CLOCK_STARTED): LifecycleState.RUNNING,
        }.get((self.state, event))
        if transition is not None:
            return RunLifecycle(self.run_id, transition, reason=reason)
        if self.state in (LifecycleState.STARTING, LifecycleState.READY, LifecycleState.RUNNING):
            if event in (LifecycleEvent.FAIL, LifecycleEvent.ABORT):
                terminal = LifecycleState.FAILED if event is LifecycleEvent.FAIL else LifecycleState.ABORTED
                return RunLifecycle(self.run_id, LifecycleState.FINALIZING, terminal, reason)
        if self.state is LifecycleState.RUNNING and event is LifecycleEvent.COMPLETE:
            return RunLifecycle(self.run_id, LifecycleState.FINALIZING, LifecycleState.COMPLETED, reason)
        if self.state is LifecycleState.FINALIZING:
            if event is LifecycleEvent.ARTIFACTS_FINALIZED and self.pending_terminal is not None:
                return RunLifecycle(self.run_id, self.pending_terminal, reason=self.reason)
            if event is LifecycleEvent.FINALIZATION_FAILED:
                return RunLifecycle(self.run_id, LifecycleState.FAILED, reason=reason)
        raise InvalidTransition(f"cannot apply state {self.state.name} event {event.name}")
