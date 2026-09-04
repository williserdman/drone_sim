from pathlib import Path
from types import SimpleNamespace

from drone_sim_companion.autotune import Action
from drone_sim_companion.hover import Observation, Phase, RollHoverDriver, State, advance


RUN_ID = "00000000-0000-4000-8000-000000000001"


def test_roll_hover_holds_for_ten_simulated_seconds_then_lands() -> None:
    transition = advance(
        State.initial(),
        Observation(0, heartbeat=True, prearm_checks_healthy=True),
    )
    assert transition.actions == (Action.mode("GUIDED"),)
    transition = advance(transition.state, Observation(1, mode="GUIDED"))
    assert transition.actions == (Action.arm(),)
    transition = advance(transition.state, Observation(2, armed=True))
    assert transition.actions == (Action.takeoff(5.0),)
    transition = advance(
        transition.state,
        Observation(3, relative_altitude_m=4.5, armed=True),
    )
    assert transition.actions == (Action.override(1500), Action.mode("ALT_HOLD"))
    transition = advance(transition.state, Observation(4, mode="ALT_HOLD", armed=True))
    assert transition.state.phase is Phase.HOVERING
    assert transition.actions == ()

    transition = advance(
        transition.state,
        Observation(10_000_000_003, mode="ALT_HOLD", armed=True),
    )
    assert transition.state.phase is Phase.HOVERING
    transition = advance(
        transition.state,
        Observation(10_000_000_004, mode="ALT_HOLD", armed=True),
    )
    assert transition.state.phase is Phase.WAIT_LAND
    assert transition.actions == (Action.mode("LAND"),)

    transition = advance(
        transition.state,
        Observation(11_000_000_000, mode="LAND", armed=False, landed=True),
    )
    assert transition.state.phase is Phase.COMPLETE


def test_roll_hover_driver_keeps_mid_throttle_only_during_hover(tmp_path: Path) -> None:
    vehicle = SimpleNamespace(
        mode="ALT_HOLD",
        armed=True,
        channels=SimpleNamespace(overrides={}),
        simple_takeoff=lambda _altitude: None,
    )
    driver = RollHoverDriver(
        vehicle,
        run_directory=tmp_path,
        run_id=RUN_ID,
        mode_factory=lambda name: name,
    )
    driver.state = State(Phase.HOVERING, hover_started_timestamp_ns=1)
    driver.refresh_override()
    assert vehicle.channels.overrides["3"] == 1500

    driver.state = State(Phase.WAIT_LAND)
    driver.refresh_override()
    assert vehicle.channels.overrides == {}


def test_roll_hover_driver_retries_arm_during_transient_prearm_gate(
    tmp_path: Path,
) -> None:
    vehicle = SimpleNamespace(
        mode="GUIDED",
        armed=False,
        channels=SimpleNamespace(overrides={}),
        simple_takeoff=lambda _altitude: None,
    )
    driver = RollHoverDriver(
        vehicle,
        run_directory=tmp_path,
        run_id=RUN_ID,
        mode_factory=lambda name: name,
    )
    driver.state = State(Phase.WAIT_ARMED)

    driver.refresh_override()

    assert vehicle.armed is True
