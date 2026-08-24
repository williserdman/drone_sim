"""Pure fail-closed lifecycle for the SITL process boundary."""

from __future__ import annotations

from enum import Enum, auto


class RuntimeState(Enum):
    STARTING = auto()
    WAITING_JSON = auto()
    WAITING_HEARTBEAT = auto()
    READY = auto()
    STOPPING = auto()
    QUIESCENT = auto()
    FAILED = auto()


class Event(Enum):
    PROCESS_STARTED = auto()
    JSON_FRAME = auto()
    MAVLINK_HEARTBEAT = auto()
    PEER_LOST = auto()
    PROCESS_EXITED = auto()
    FINALIZE = auto()


_TRANSITIONS = {
    (RuntimeState.STARTING, Event.PROCESS_STARTED): RuntimeState.WAITING_JSON,
    (RuntimeState.WAITING_JSON, Event.JSON_FRAME): RuntimeState.WAITING_HEARTBEAT,
    (RuntimeState.WAITING_HEARTBEAT, Event.MAVLINK_HEARTBEAT): RuntimeState.READY,
    (RuntimeState.WAITING_HEARTBEAT, Event.PEER_LOST): RuntimeState.FAILED,
    (RuntimeState.READY, Event.PEER_LOST): RuntimeState.FAILED,
    (RuntimeState.READY, Event.FINALIZE): RuntimeState.STOPPING,
    (RuntimeState.STOPPING, Event.PROCESS_EXITED): RuntimeState.QUIESCENT,
    (RuntimeState.STARTING, Event.PROCESS_EXITED): RuntimeState.FAILED,
    (RuntimeState.WAITING_JSON, Event.PROCESS_EXITED): RuntimeState.FAILED,
    (RuntimeState.WAITING_HEARTBEAT, Event.PROCESS_EXITED): RuntimeState.FAILED,
    (RuntimeState.READY, Event.PROCESS_EXITED): RuntimeState.FAILED,
}


def transition(state: RuntimeState, event: Event) -> RuntimeState:
    try:
        return _TRANSITIONS[state, event]
    except KeyError as error:
        raise ValueError(f"invalid runtime transition: {state.name} + {event.name}") from error
