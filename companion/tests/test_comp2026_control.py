from pathlib import Path
import sys
from types import MappingProxyType, SimpleNamespace

import pytest

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / "companion/comp2026/src"))

from drone import timebase
from drone.common_types import MissionHome
from drone.control.flight_profile import FlightProfile, TelemetryRequest
from drone.control.flight_state import RCModeBand, SourceIdentity
from drone.control.listener_runtime import AutopilotVersionContract, TelemetryStartupPolicy
from drone.control.mission_supervisor import CommandRejected, FM1, FM2, FM3, RecoveryPolicy
from drone.control.stability import ReleaseStabilityConfig
from drone.sensors.lidar.clearance import ClearanceCalibration
from drone_sim_companion.comp2026_control import SimulationCompetitionControl


class FakeController:
    def __init__(self, _endpoint, **kwargs):
        self.kwargs = kwargs
        self.flight_state = kwargs["flight_state"]
        self.mission_home = None
        self.output_transactions = None
        self.telemetry_verifier = None

    def install_output_transactions(self, **kwargs):
        self.output_transactions = kwargs

    def install_startup_telemetry_verifier(self, verifier, *, verified=False):
        self.telemetry_verifier = (verifier, verified)

    def set_mission_home(self, home):
        self.mission_home = home


class FakeCollector:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.prepared = False
        self.verified = False
        self.closed = False

    def prepare(self):
        self.prepared = True

    def verify_after_guided(self):
        self.verified = True
        return object()

    def close(self):
        self.closed = True


def policy():
    companion = SourceIdentity(1, 191)
    fc = SourceIdentity(1, 1)
    freshness = MappingProxyType({
        name: 1.0 for name in (
            "heartbeat", "mode", "location", "velocity", "attitude",
            "landed_state", "armed", "home", "rc_input", "range", "failsafe",
        )
    })
    profile = FlightProfile(
        "test", "a" * 64, "ArduCopter", companion, companion, fc, freshness,
        1.0, 7,
        (RCModeBand("companion", 1400, 1600, "GUIDED"), RCModeBand("pilot", 1800, 2100, "LOITER")),
        2, 3, MappingProxyType({4: "GUIDED", 5: "LOITER"}),
        frozenset((5,)), frozenset((3, 4)), 1, "sha256:" + "a" * 64,
        "test", (TelemetryRequest(0, 50_000),),
    )
    return SimpleNamespace(
        flight_profile=profile,
        telemetry_policy=TelemetryStartupPolicy(2.0, 5.0, 2.0, 0.05, 2, 0.25),
        autopilot_version_contract=AutopilotVersionContract("ArduCopter 4.5.7", 0x040507FF, b"abcdef0\0", "test"),
        clearance_calibration=ClearanceCalibration((0.0, 0.0, 1.0), (0.3, 0.0, -0.1), True, 0.1, 0.5, 0.05, True),
        release_stability=ReleaseStabilityConfig(2.0, 10.0, 0.1, 0.1, 0.1, 0.1, 0.15, 0.15, 0.05, 0.1, 0.05, 0.2),
        recovery_policy=RecoveryPolicy(lambda *_args: None, 60.0, 10.0, timebase.monotonic),
        mission_home_check=lambda _home: None,
        fc_home_position_tolerance_m=2.0,
        fc_home_altitude_tolerance_m=0.1,
        startup_timeout_s=60.0,
        telemetry_poll_interval_s=0.05,
    )


def make_control(tmp_path, selected_policy=None):
    selected_policy = selected_policy or policy()
    control = SimulationCompetitionControl(
        selected_policy, run_id="run-1", run_directory=tmp_path,
        endpoint="udp:127.0.0.1:14550", heartbeat_timeout=3.0,
        guided_output_delivery_callback=lambda: None,
        controller_factory=FakeController, telemetry_factory=FakeCollector,
    )
    return control, selected_policy


def seed_ground(control, *, rc_pwm=1500, include_home=True):
    state = control.flight_state
    source = state.source
    metadata = dict(received_at=timebase.monotonic(), source_system=source.system_id, source_component=source.component_id)
    state.observe_heartbeat((2, 3, 0, 4, 3), armed=False, mode="GUIDED", sequence=1, **metadata)
    values = {
        "location": (374003371, -1220800351, 0, 0), "velocity": (0, 0, 0),
        "attitude": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), "landed_state": 1,
    }
    if include_home:
        values["home"] = (374003371, -1220800351, 0)
    state.update_many(values, sequence=2, **metadata)
    control.decoders._rc_health = True
    control.decoders._rc_health_received_at = timebase.monotonic()
    state.observe_rc_input(channel=7, pwm=rc_pwm, signal_healthy=True, sequence=3, **metadata)
    state.observe_failsafe("clear", active=False, sequence=4, **metadata)


def test_composes_real_guards_and_staged_telemetry_from_same_policy(tmp_path):
    control, selected = make_control(tmp_path)

    assert control.flight_state.__class__.__name__ == "FlightState"
    assert control.decoders.__class__.__name__ == "ObservationDecoders"
    assert control.supervisor.__class__.__name__ == "MissionSupervisor"
    assert control.controller.kwargs["clearance_calibration"] is selected.clearance_calibration
    assert control.controller.kwargs["release_stability_config"] is selected.release_stability
    assert control.controller.kwargs["wait_ready"] is False
    assert control.controller.output_transactions.keys() == {"dependency_transaction", "supervisor_transaction"}

    control.prepare()
    verifier, verified = control.controller.telemetry_verifier
    assert control.telemetry_collector.prepared is True
    assert verified is False
    assert verifier() is None
    assert control.telemetry_collector.verified is True


def test_ground_readiness_attempt_and_original_phase_order(tmp_path):
    control, _ = make_control(tmp_path)
    seed_ground(control)

    assert control.ready_for_initial_command() is True
    control.begin_attempt()
    assert control.controller.mission_home == MissionHome(37.4003371, -122.0800351, 0.0)
    assert control.supervisor.status(FM1) == "RUNNING"

    for event in (("FM1", "STARTED"), ("FM1", "COMPLETE"), ("FM2", "STARTED"),
                  ("FM2", "COMPLETE"), ("FM3_3", "STARTED"), ("FM3_3", "COMPLETE"),
                  ("FM3_4", "STARTED"), ("FM3_4", "COMPLETE"), ("HOME", "STARTED"),
                  ("HOME", "DISARMED"), ("HOME", "COMPLETE")):
        control.phase_event(*event)

    assert [control.supervisor.status(command) for command in (FM1, FM2, FM3)] == ["TERMINAL"] * 3
    assert control.supervisor.terminal_result == "SUCCEEDED"
    with pytest.raises(RuntimeError, match="unexpected automatic mission event"):
        control.phase_event("HOME", "COMPLETE")


@pytest.mark.parametrize("change", ["stale", "false_rc", "no_home"])
def test_readiness_rejects_missing_or_unsafe_ground_proof(tmp_path, change):
    control, _ = make_control(tmp_path)
    seed_ground(control, rc_pwm=1900 if change == "false_rc" else 1500, include_home=change != "no_home")
    if change == "stale":
        control.flight_state.invalidate_observations(("heartbeat",), source_system=1, source_component=1)
    with pytest.raises((CommandRejected, RuntimeError)):
        control.ready_for_initial_command()


def test_attempt_token_is_durable_and_single_use(tmp_path):
    first, selected = make_control(tmp_path)
    seed_ground(first)
    first.begin_attempt()
    second, _ = make_control(tmp_path, selected)
    seed_ground(second)
    with pytest.raises(CommandRejected, match="consumption failed"):
        second.begin_attempt()


def test_mode_takeover_revokes_output_permission(tmp_path):
    control, _ = make_control(tmp_path)
    seed_ground(control)
    control.begin_attempt()
    state = control.flight_state
    source = state.source
    state.observe_rc_input(channel=7, pwm=1900, signal_healthy=True, sequence=9, received_at=timebase.monotonic(), source_system=source.system_id, source_component=source.component_id)
    with pytest.raises(RuntimeError):
        control.supervisor.check_permission()
