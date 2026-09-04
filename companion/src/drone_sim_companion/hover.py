"""Short roll-gain hover verification mission."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Any, Callable

from .autotune import Action, execute_actions


HOVER_DURATION_NS = 10_000_000_000


class Phase(str, Enum):
    WAIT_READY = "WAIT_READY"
    WAIT_GUIDED = "WAIT_GUIDED"
    WAIT_ARMED = "WAIT_ARMED"
    WAIT_ALTITUDE = "WAIT_ALTITUDE"
    WAIT_ALT_HOLD = "WAIT_ALT_HOLD"
    HOVERING = "HOVERING"
    WAIT_LAND = "WAIT_LAND"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


@dataclass(frozen=True)
class Observation:
    timestamp_ns: int
    heartbeat: bool = False
    prearm_checks_healthy: bool | None = None
    mode: str | None = None
    armed: bool | None = None
    landed: bool | None = None
    relative_altitude_m: float | None = None
    status_text: str | None = None


@dataclass(frozen=True)
class State:
    phase: Phase
    last_timestamp_ns: int | None = None
    hover_started_timestamp_ns: int | None = None
    failure_reason: str = ""

    @classmethod
    def initial(cls) -> "State":
        return cls(Phase.WAIT_READY)


@dataclass(frozen=True)
class Transition:
    state: State
    actions: tuple[Action, ...] = ()


def _failed(state: State, stamp: int, reason: str) -> Transition:
    return Transition(
        replace(
            state,
            phase=Phase.FAILED,
            last_timestamp_ns=stamp,
            failure_reason=reason,
        )
    )


def advance(state: State, observation: Observation) -> Transition:
    """Advance a ten-second ALT_HOLD stability check using simulation time."""
    if state.phase in {Phase.COMPLETE, Phase.FAILED}:
        return Transition(state)
    stamp = observation.timestamp_ns
    if state.last_timestamp_ns is not None and stamp < state.last_timestamp_ns:
        return _failed(state, stamp, "simulation timestamp regressed")
    current = replace(state, last_timestamp_ns=stamp)

    if state.phase is Phase.WAIT_READY:
        if not observation.heartbeat or observation.prearm_checks_healthy is not True:
            return Transition(current)
        return Transition(replace(current, phase=Phase.WAIT_GUIDED), (Action.mode("GUIDED"),))
    if state.phase is Phase.WAIT_GUIDED:
        if observation.mode != "GUIDED":
            return Transition(current)
        return Transition(replace(current, phase=Phase.WAIT_ARMED), (Action.arm(),))
    if state.phase is Phase.WAIT_ARMED:
        if observation.armed is not True:
            return Transition(current)
        return Transition(
            replace(current, phase=Phase.WAIT_ALTITUDE),
            (Action.takeoff(5.0),),
        )
    if state.phase is Phase.WAIT_ALTITUDE:
        if observation.relative_altitude_m is None or observation.relative_altitude_m < 4.5:
            return Transition(current)
        return Transition(
            replace(current, phase=Phase.WAIT_ALT_HOLD),
            (Action.override(1500), Action.mode("ALT_HOLD")),
        )
    if state.phase is Phase.WAIT_ALT_HOLD:
        if observation.mode != "ALT_HOLD":
            return Transition(current)
        return Transition(
            replace(
                current,
                phase=Phase.HOVERING,
                hover_started_timestamp_ns=stamp,
            )
        )
    if state.phase is Phase.HOVERING:
        if observation.mode != "ALT_HOLD":
            return _failed(current, stamp, f"hover mode changed to {observation.mode}")
        assert state.hover_started_timestamp_ns is not None
        if stamp - state.hover_started_timestamp_ns < HOVER_DURATION_NS:
            return Transition(current)
        return Transition(replace(current, phase=Phase.WAIT_LAND), (Action.mode("LAND"),))
    if state.phase is Phase.WAIT_LAND:
        if observation.armed is False and observation.landed is True:
            return Transition(replace(current, phase=Phase.COMPLETE))
        return Transition(current)
    return Transition(current)


class RollHoverDriver:
    """Apply the short hover policy to a DroneKit-compatible vehicle."""

    def __init__(
        self,
        vehicle: Any,
        *,
        run_directory: Path,
        run_id: str,
        mode_factory: Callable[[str], Any],
    ) -> None:
        self.vehicle = vehicle
        self.run_directory = run_directory
        self.run_id = run_id
        self.mode_factory = mode_factory
        self.state = State.initial()

    def observe(self, observation: Observation) -> Transition:
        transition = advance(self.state, observation)
        execute_actions(
            self.vehicle,
            transition.actions,
            run_directory=self.run_directory,
            run_id=self.run_id,
            mode_factory=self.mode_factory,
        )
        self.state = transition.state
        return transition

    def refresh_override(self) -> None:
        if self.state.phase is Phase.WAIT_ARMED:
            self.vehicle.armed = True
        elif self.state.phase in {Phase.WAIT_ALT_HOLD, Phase.HOVERING}:
            execute_actions(
                self.vehicle,
                (Action.override(1500),),
                run_directory=self.run_directory,
                run_id=self.run_id,
                mode_factory=self.mode_factory,
            )
        elif self.state.phase is Phase.WAIT_LAND:
            self.vehicle.channels.overrides = {}
