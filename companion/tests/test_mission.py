from __future__ import annotations

from dataclasses import replace

import pytest

from drone_sim_companion.mission import (
    Ack,
    CommandKind,
    MissionPhase,
    MissionState,
    Telemetry,
    advance,
)


def observation(
    timestamp_ns: int,
    *,
    heartbeat: bool = False,
    mode: str | None = None,
    armed: bool | None = None,
    altitude_m: float | None = None,
    vertical_speed_m_s: float | None = None,
    contact: bool | None = None,
    landed: bool | None = None,
    ack: Ack | None = None,
) -> Telemetry:
    return Telemetry(
        timestamp_ns=timestamp_ns,
        heartbeat=heartbeat,
        mode=mode,
        armed=armed,
        relative_altitude_m=altitude_m,
        vertical_speed_m_s=vertical_speed_m_s,
        in_contact=contact,
        landed=landed,
        ack=ack,
    )


def accepted(timestamp_ns: int, command: CommandKind) -> Telemetry:
    return observation(timestamp_ns, ack=Ack(command=command, accepted=True, result=0))


def drive_to_descent() -> tuple[MissionState, list[CommandKind]]:
    state = MissionState.initial()
    commands: list[CommandKind] = []
    events = (
        observation(0, heartbeat=True, mode="STABILIZE", armed=False),
        accepted(50_000_000, CommandKind.SET_GUIDED),
        observation(100_000_000, heartbeat=True, mode="GUIDED", armed=False),
        accepted(150_000_000, CommandKind.ARM),
        observation(200_000_000, heartbeat=True, mode="GUIDED", armed=True),
        accepted(250_000_000, CommandKind.TAKEOFF),
        observation(300_000_000, mode="GUIDED", armed=True, altitude_m=1.40),
        accepted(350_000_000, CommandKind.LAND),
    )
    for event in events:
        transition = advance(state, event)
        state = transition.state
        commands.extend(command.kind for command in transition.commands)
    return state, commands


def test_nominal_telemetry_drives_guided_arm_takeoff_land_and_landed() -> None:
    state, commands = drive_to_descent()
    assert state.phase is MissionPhase.DESCENDING
    assert commands == [
        CommandKind.SET_GUIDED,
        CommandKind.ARM,
        CommandKind.TAKEOFF,
        CommandKind.LAND,
    ]

    descending = advance(
        state,
        observation(
            400_000_000,
            mode="LAND",
            armed=True,
            altitude_m=1.10,
            vertical_speed_m_s=-0.30,
            contact=False,
            landed=False,
        ),
    )
    assert descending.state.descent_observed
    touchdown = advance(
        descending.state,
        observation(
            450_000_000,
            mode="LAND",
            armed=True,
            altitude_m=0.02,
            vertical_speed_m_s=-0.05,
            contact=True,
            landed=True,
        ),
    )
    assert touchdown.state.phase is MissionPhase.WAIT_DISARM
    finished = advance(
        touchdown.state,
        observation(
            500_000_000,
            mode="LAND",
            armed=False,
            altitude_m=0.0,
            vertical_speed_m_s=0.0,
            contact=True,
            landed=True,
        ),
    )
    assert finished.state.phase is MissionPhase.LANDED
    assert [event.name for event in finished.events] == ["vehicle_disarmed", "mission_landed"]


@pytest.mark.parametrize(
    ("event", "reason"),
    [
        (accepted(50_000_000, CommandKind.ARM), "unexpected acknowledgement"),
        (
            observation(
                50_000_000,
                ack=Ack(command=CommandKind.SET_GUIDED, accepted=False, result=4),
            ),
            "negative acknowledgement",
        ),
    ],
)
def test_acknowledgements_must_match_and_succeed(event: Telemetry, reason: str) -> None:
    waiting = advance(
        MissionState.initial(),
        observation(0, heartbeat=True, mode="STABILIZE", armed=False),
    ).state
    result = advance(waiting, event)
    assert result.state.phase is MissionPhase.FAILED
    assert reason in result.state.failure_reason


def test_regressing_timestamp_fails_and_cannot_be_repaired() -> None:
    waiting = advance(
        MissionState.initial(),
        observation(100_000_000, heartbeat=True, mode="STABILIZE", armed=False),
    ).state
    failed = advance(waiting, accepted(50_000_000, CommandKind.SET_GUIDED)).state
    assert failed.phase is MissionPhase.FAILED
    assert "regressed" in failed.failure_reason

    repaired = advance(
        failed,
        accepted(150_000_000, CommandKind.SET_GUIDED),
    )
    assert repaired.state == failed
    assert repaired.commands == ()


def test_guided_mode_cannot_change_after_it_is_confirmed() -> None:
    state = MissionState.initial()
    state = advance(
        state, observation(0, heartbeat=True, mode="STABILIZE", armed=False)
    ).state
    state = advance(state, accepted(50_000_000, CommandKind.SET_GUIDED)).state
    state = advance(
        state, observation(100_000_000, heartbeat=True, mode="GUIDED", armed=False)
    ).state

    result = advance(
        state, observation(150_000_000, heartbeat=True, mode="LOITER", armed=False)
    )
    assert result.state.phase is MissionPhase.FAILED
    assert "mode changed" in result.state.failure_reason


@pytest.mark.parametrize(
    "bad_event",
    [
        observation(-1),
        observation(True),
        observation(0, altitude_m=float("nan")),
        observation(0, vertical_speed_m_s=float("inf")),
    ],
)
def test_invalid_telemetry_fails_closed(bad_event: Telemetry) -> None:
    result = advance(MissionState.initial(), bad_event)
    assert result.state.phase is MissionPhase.FAILED


def test_equal_simulation_timestamps_are_ordered_without_wall_time() -> None:
    waiting = advance(
        MissionState.initial(),
        observation(1_000_000_000, heartbeat=True, mode="STABILIZE", armed=False),
    ).state
    acknowledged = advance(
        waiting, accepted(1_000_000_000, CommandKind.SET_GUIDED)
    )
    assert acknowledged.state.phase is MissionPhase.WAIT_GUIDED_MODE
    assert acknowledged.state.last_timestamp_ns == 1_000_000_000


def test_contact_before_observed_descent_fails_truthfully() -> None:
    state, _ = drive_to_descent()
    result = advance(
        state,
        observation(
            400_000_000,
            mode="LAND",
            armed=True,
            altitude_m=1.39,
            vertical_speed_m_s=0.0,
            contact=True,
            landed=True,
        ),
    )
    assert result.state.phase is MissionPhase.FAILED
    assert "contact preceded descent" in result.state.failure_reason


def test_state_is_immutable() -> None:
    state = MissionState.initial()
    with pytest.raises(Exception):
        state.phase = MissionPhase.LANDED  # type: ignore[misc]
    assert replace(state).phase is MissionPhase.WAIT_HEARTBEAT
