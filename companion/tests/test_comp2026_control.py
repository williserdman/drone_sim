from pathlib import Path
import sys
from types import MappingProxyType, SimpleNamespace

import pytest
from pymavlink import mavutil

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / "companion/comp2026/src"))

from drone import timebase
from drone.common_types import MissionHome
from drone.control import drone_control
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


class CallbackVehicle:
    def __init__(self):
        self.listeners = {}
        self._handler = SimpleNamespace(target_system=1, target_component=1)
        self._master = SimpleNamespace(WIRE_PROTOCOL_VERSION="2.0")

    def on_message(self, message_names):
        def register(callback):
            for name in message_names if isinstance(message_names, list) else [message_names]:
                self.listeners[name] = callback
            return callback

        return register

    def emit(self, name, **fields):
        message = SimpleNamespace(
            get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 1,
            **fields,
        )
        self.listeners[name](self, name, message)


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


def make_callback_control(tmp_path, monkeypatch):
    vehicle = CallbackVehicle()
    monkeypatch.setattr(drone_control, "connect", lambda *_args, **_kwargs: vehicle)
    monkeypatch.setattr(
        drone_control.DroneControl,
        "install_output_transactions",
        lambda self, **transactions: setattr(self, "output_transactions", transactions),
    )
    control = SimulationCompetitionControl(
        policy(), run_id="callback-run", run_directory=tmp_path,
        endpoint="unused", heartbeat_timeout=3.0,
        guided_output_delivery_callback=lambda: None,
        telemetry_factory=FakeCollector,
    )
    return control, vehicle


def emit_safe_ground(vehicle, *, armed=False, rc_pwm=1500, include_home=True):
    vehicle.emit(
        "HEARTBEAT",
        type=2,
        autopilot=3,
        base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0,
        custom_mode=5,
        system_status=4,
    )
    vehicle.emit(
        "SYS_STATUS",
        onboard_control_sensors_present=65_536,
        onboard_control_sensors_enabled=65_536,
        onboard_control_sensors_health=65_536,
    )
    vehicle.emit("RC_CHANNELS", chan7_raw=rc_pwm)
    vehicle.emit(
        "GLOBAL_POSITION_INT",
        lat=374003371,
        lon=-1220800351,
        alt=0,
        relative_alt=0,
        vx=0,
        vy=0,
        vz=0,
    )
    vehicle.emit(
        "ATTITUDE",
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        rollspeed=0.0,
        pitchspeed=0.0,
        yawspeed=0.0,
    )
    vehicle.emit(
        "EXTENDED_SYS_STATE",
        landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
    )
    if include_home:
        vehicle.emit(
            "HOME_POSITION",
            latitude=374003371,
            longitude=-1220800351,
            altitude=0,
        )


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
    assert control.controller.kwargs["vehicle_class"].__name__ == "SimulationVehicle"
    assert control.controller.output_transactions.keys() == {"dependency_transaction", "supervisor_transaction"}

    control.prepare()
    verifier, verified = control.controller.telemetry_verifier
    assert control.telemetry_collector.prepared is True
    assert verified is False
    assert verifier() is None
    assert control.telemetry_collector.verified is True


def test_first_mavlink2_packet_preserves_guarded_dronekit_queue(monkeypatch, tmp_path):
    from drone.control import drone_control
    from dronekit import Vehicle
    from dronekit.mavlink import MAVConnection
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as mavlink2

    # Reproduce a fresh process whose default encoder is MAVLink 1.
    monkeypatch.delenv("MAVLINK20", raising=False)
    monkeypatch.setattr(mavutil, "mavlink", mavutil.mavlink)
    monkeypatch.setattr(mavutil, "current_dialect", mavutil.current_dialect)
    mavutil.set_dialect("ardupilotmega")
    assert mavutil.mavlink.WIRE_PROTOCOL_VERSION == "1.0"
    connections = []

    def connect_without_flight(_endpoint, **options):
        connection = MAVConnection(
            "udpin:127.0.0.1:0",
            source_system=options["source_system"],
            source_component=options["source_component"],
        )
        connections.append(connection)
        connection.target_system = 1
        encoder = mavlink2.MAVLink(None, srcSystem=1, srcComponent=1)
        heartbeat = encoder.heartbeat_encode(2, 3, 0, 0, 3)
        connection.master.auto_mavlink_version(heartbeat.pack(encoder))
        return Vehicle(connection)

    monkeypatch.setattr(drone_control, "connect", connect_without_flight)
    try:
        control = SimulationCompetitionControl(
            policy(), run_id="wire-test", run_directory=tmp_path,
            endpoint="unused", heartbeat_timeout=3.0,
            guided_output_delivery_callback=lambda: None,
            telemetry_factory=FakeCollector,
        )
        checks = []

        def check_enqueue(enqueue):
            checks.append(True)
            return enqueue()

        vehicle = control.controller.vehicle
        message = vehicle.message_factory.heartbeat_encode(6, 8, 0, 0, 3)
        control.controller._transport_output_transaction(
            lambda: vehicle.send_mavlink(message), check_enqueue
        )
        assert checks == [True]
        assert connections[0].out_queue.get_nowait()[0] == 0xFD
    finally:
        for connection in connections:
            # No network worker threads were started by this connection probe.
            connection.mavlink_thread_in = None
            connection.mavlink_thread_out = None
            connection.master.close()


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


def test_real_callbacks_keep_startup_pending_until_ground_snapshot_is_complete(
    tmp_path, monkeypatch
):
    control, vehicle = make_callback_control(tmp_path, monkeypatch)

    assert control.ready_for_initial_command() is False
    assert control.ground_telemetry_pending_reasons == (
        "heartbeat absent",
        "mode absent",
        "location absent",
        "velocity absent",
        "attitude absent",
        "landed_state absent",
        "armed absent",
        "home absent",
        "rc_input absent",
        "failsafe absent",
    )

    emit_safe_ground(vehicle, include_home=False)
    assert control.ready_for_initial_command() is False
    assert control.ground_telemetry_pending_reasons == ("home absent",)

    vehicle.emit(
        "HOME_POSITION",
        latitude=374003371,
        longitude=-1220800351,
        altitude=0,
    )
    assert control.ready_for_initial_command() is True
    assert control.ground_telemetry_pending_reasons == ()


def test_stale_ground_snapshot_remains_pending_with_ordered_reasons(
    tmp_path, monkeypatch
):
    class Clock:
        now = 10.0

        def __call__(self):
            return self.now

    clock = Clock()
    monkeypatch.setattr(timebase, "monotonic", clock)
    control, _ = make_control(tmp_path)
    seed_ground(control)
    clock.now = 12.0

    assert control.ready_for_initial_command() is False
    assert control.ground_telemetry_pending_reasons == (
        "heartbeat stale",
        "mode stale",
        "location stale",
        "velocity stale",
        "attitude stale",
        "landed_state stale",
        "armed stale",
        "home stale",
        "rc_input stale",
        "failsafe stale",
    )


@pytest.mark.parametrize(
    ("armed", "rc_pwm", "reason"),
    [
        (True, 1500, "requires disarmed state"),
        (False, 1900, "initial RC selection must be the companion slot"),
    ],
)
def test_real_callbacks_reject_observed_unsafe_ground_state(
    tmp_path, monkeypatch, armed, rc_pwm, reason
):
    control, vehicle = make_callback_control(tmp_path, monkeypatch)
    emit_safe_ground(vehicle, armed=armed, rc_pwm=rc_pwm)

    with pytest.raises(CommandRejected, match=reason):
        control.ready_for_initial_command()


def test_begin_attempt_rechecks_complete_ground_guard(tmp_path):
    control, _ = make_control(tmp_path)

    assert control.ready_for_initial_command() is False
    with pytest.raises(CommandRejected, match="safe-ground telemetry is absent or stale"):
        control.begin_attempt()
    assert not (tmp_path / ".comp2026-attempt-consumed.json").exists()


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
