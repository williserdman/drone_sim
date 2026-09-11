from dataclasses import replace
import importlib
import queue
import threading
import json
import time
from types import SimpleNamespace
from types import MappingProxyType

import numpy as np
import pytest

from drone.control.listener import (
    ACK_DENIED,
    ACK_IN_PROGRESS,
    CommandExecutionOwner,
    LiveCleanupReport,
    LiveComponentFactories,
    LiveListenerRuntime,
    LiveRunResult,
    TelemetryStartupEvidence,
    _DeferredDropper,
    _cleanup_failed_startup,
    build_live_listener,
    load_listener_artifacts,
    _require_cached_waypoint_ages,
    start_repl,
)
from drone.control.mission_info import MAX_WAYPOINT_AGE_SECONDS
from drone.control.mission_supervisor import (
    AuthorityLost,
    CommandEnvelope,
    CommandRejected,
    MissionSupervisor,
    RecoveryPolicy,
)
from drone.common_types import MissionHome
from drone.control.flight_state import Authority
from drone.control.mission_supervisor import FM3
from drone import timebase
from drone.control.flight_profile import RC_RECEIVER_SENSOR_BIT
from drone.control.mission_supervisor import FM1, FM2
from drone.control.listener_runtime import (
    HardwareComponentConfig,
    InjectedComponentConfig,
    PayloadConfig,
    RangeConfig,
)
from drone.sensors.camera.camera import Camera
from drone.sensors.camera._camera_manager import CameraManager
from test_listener import Packet, prepared_startup_files
from test_listener_runtime import runtime_configuration


class FakeController:
    def __init__(self, flight_state):
        self.flight_state = flight_state
        self.vehicle = SimpleNamespace()
        self.mission_home = None

    def set_mission_home(self, home):
        self.mission_home = home

    def install_output_transactions(self, **transactions):
        self.output_transactions = transactions

    def install_startup_telemetry_verifier(
        self, verifier=None, *, verified=False
    ):
        self.telemetry_verifier = verifier
        self.telemetry_verified = verified


class FakeTelemetryCollector:
    def __init__(self, events):
        self.events = events

    def configure_and_collect(self):
        self.events.append("telemetry")
        return TelemetryStartupEvidence(MappingProxyType({}), 1.0)

    def prepare(self):
        self.events.append("telemetry-prepare")

    def verify_after_guided(self):
        self.events.append("telemetry-verify")
        return TelemetryStartupEvidence(MappingProxyType({}), 1.0)

    def close(self):
        self.events.append("telemetry-close")


class FakeListenerTransport:
    def __init__(self, source_identity, wire_protocol):
        self.source_identity = source_identity
        self.wire_protocol = wire_protocol
        self.callback = None
        self.acks = []

    def install_message_callback(self, _names, callback):
        self.callback = callback

    def send_command_ack(self, ack):
        self.acks.append(ack)


class FakeLidar:
    def get_sample(self):
        return SimpleNamespace(distance_cm=100.0)

    def stop(self, *, timeout_seconds):
        return SimpleNamespace(
            worker_stopped=True,
            cleanup_completed=True,
            cleanup_error=None,
        )


class FakeDropper:
    supports_attachment = True
    servos = ()


def fake_factories(
    events,
    *,
    supports_attachment=True,
    captured=None,
    telemetry_startup_mode="complete",
    backend="test-injected-components-v1",
):
    def controller_factory(**values):
        events.append("controller")
        if captured is not None:
            captured.update(values)
        controller = FakeController(values["flight_state"])
        if captured is not None:
            captured["controller"] = controller
        return controller

    def ack_transport_factory(**values):
        events.append("ack")
        config = values["config"]
        transport = FakeListenerTransport(
            config.connection.source_identity,
            config.connection.wire_protocol,
        )
        if captured is not None:
            captured["transport"] = transport
        return transport

    def telemetry_collector_factory(**_values):
        return FakeTelemetryCollector(events)

    def lidar_factory(**_values):
        events.append("lidar")
        return FakeLidar()

    def dropper_factory(**_values):
        events.append("dropper")
        return FakeDropper()

    def camera_factory(**_values):
        events.append("camera")
        return SimpleNamespace(
            prepare_precision_readiness=lambda **_kwargs: SimpleNamespace(
                ready=True, reasons=()
            ),
            precision_readiness=lambda: SimpleNamespace(ready=True, reasons=()),
        )

    return LiveComponentFactories(
        backend=backend,
        controller_factory=controller_factory,
        ack_transport_factory=ack_transport_factory,
        telemetry_collector_factory=telemetry_collector_factory,
        lidar_factory=lidar_factory,
        dropper_factory=dropper_factory,
        camera_factory=camera_factory,
        supports_attachment=supports_attachment,
        telemetry_startup_mode=telemetry_startup_mode,
    )


def physical_components():
    return HardwareComponentConfig(
        range_sensor=RangeConfig(1.0, 4000.0, 1.0, 0.25, 2.0, 0.02),
        payload=PayloadConfig((17, 27), 0.5, 0.05, 0.001, 0.002),
    )


@pytest.mark.parametrize(
    ("components", "factories_backend", "explicit"),
    [
        (InjectedComponentConfig("injected-a", "test binding evidence"), None, False),
        (physical_components(), "test-injected-components-v1", True),
        (InjectedComponentConfig("injected-a", "test binding evidence"), "injected-b", True),
    ],
)
def test_component_backend_mismatch_fails_before_controller_construction(
    tmp_path, components, factories_backend, explicit
):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    base = runtime_configuration(runtime_dir)
    config = replace(
        base,
        connection=replace(base.connection, wire_protocol=artifacts.wire_protocol),
        components=components,
    )
    events = []
    factories = fake_factories(events)
    if factories_backend is not None:
        factories = replace(factories, backend=factories_backend)

    with pytest.raises((TypeError, ValueError), match="component|backend|factories"):
        build_live_listener(
            artifacts,
            config,
            factories=factories if explicit else None,
        )

    assert events == []


def test_live_component_factories_reject_placeholder_backend_label():
    with pytest.raises(ValueError, match="unverified"):
        replace(fake_factories([]), backend="pending")


def test_staged_telemetry_is_limited_to_the_explicit_fm1_fm2_simulator_backend(
    tmp_path,
):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    base = runtime_configuration(runtime_dir)
    config = replace(
        base,
        connection=replace(base.connection, wire_protocol=artifacts.wire_protocol),
        components=InjectedComponentConfig(
            "drone-sim-ros-confirmed-v1", "simulator binding evidence"
        ),
        enabled_phases=frozenset((FM1, FM2)),
        vision=None,
        precision_policy=None,
    )
    events = []
    captured = {}
    factories = fake_factories(
        events,
        supports_attachment=False,
        captured=captured,
        telemetry_startup_mode="staged_simulation",
        backend="drone-sim-ros-confirmed-v1",
    )

    runtime = build_live_listener(artifacts, config, factories=factories)

    assert events[:4] == ["controller", "ack", "telemetry-prepare", "lidar"]
    assert captured["controller"].telemetry_verified is False
    assert callable(captured["controller"].telemetry_verifier)
    captured["controller"].telemetry_verifier()
    assert "telemetry-verify" in events
    assert isinstance(runtime._telemetry_verification.evidence, TelemetryStartupEvidence)
    runtime.close()
    assert events.count("telemetry-close") == 1


@pytest.mark.parametrize(
    ("backend", "enabled_phases"),
    [
        ("test-injected-components-v1", frozenset((FM1, FM2))),
        ("drone-sim-ros-confirmed-v1", frozenset((FM1, FM2, FM3))),
    ],
)
def test_staged_telemetry_rejects_every_other_composition_before_controller(
    tmp_path, backend, enabled_phases
):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    base = runtime_configuration(runtime_dir)
    config = replace(
        base,
        connection=replace(base.connection, wire_protocol=artifacts.wire_protocol),
        components=InjectedComponentConfig(backend, "binding evidence"),
        enabled_phases=enabled_phases,
    )
    events = []

    with pytest.raises(ValueError, match="staged telemetry"):
        build_live_listener(
            artifacts,
            config,
            factories=fake_factories(
                events,
                telemetry_startup_mode="staged_simulation",
                backend=backend,
            ),
        )

    assert events == []


def test_valid_hardware_configuration_reaches_default_physical_selection(
    tmp_path, monkeypatch
):
    import drone.control.listener as listener_module

    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    base = runtime_configuration(runtime_dir)
    config = replace(
        base,
        connection=replace(base.connection, wire_protocol=artifacts.wire_protocol),
        components=physical_components(),
        enabled_phases=frozenset((FM1, FM2)),
        vision=None,
        precision_policy=None,
    )
    events = []
    physical = replace(
        fake_factories(events, supports_attachment=False),
        backend="physical-hardware-v1",
    )
    monkeypatch.setattr(listener_module, "_default_live_factories", lambda: physical)

    runtime = build_live_listener(artifacts, config)

    assert isinstance(runtime, LiveListenerRuntime)
    assert events == ["controller", "ack", "telemetry", "lidar"]


def test_live_factory_builds_one_validated_composition_and_installs_listener(tmp_path):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol=artifacts.wire_protocol),
    )
    events = []

    runtime = build_live_listener(
        artifacts,
        config,
        factories=fake_factories(events),
    )

    assert isinstance(runtime, LiveListenerRuntime)
    assert runtime.tracker.waypoint_path == config.waypoint_path
    assert runtime.controller.flight_state is runtime.flight_state
    assert runtime.listener._installed is True
    assert events == ["controller", "ack", "telemetry", "lidar", "camera"]


def _real_camera_factory(config, source):
    manager = CameraManager(
        frame_source=source,
        calibration_path=config.vision.calibration_path,
        clock=config.vision.receipt_clock_ns,
        max_exposure_age_ns=config.vision.max_exposure_age_ns,
    )
    return Camera(
        config.vision.marker_size_mm,
        manager=manager,
        mounting_path=config.vision.mounting_path,
    )


def test_live_factory_prepares_real_camera_before_install_and_stops_worker(tmp_path):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol=artifacts.wire_protocol),
    )
    calibration = json.loads(config.vision.calibration_path.read_text())
    shape = (calibration["image_height_px"], calibration["image_width_px"], 3)
    events = []

    class TimestampedSource:
        last_timestamp_ns = None

        def capture_frame(self, quality=4):
            time.sleep(0.001)
            self.last_timestamp_ns = config.vision.receipt_clock_ns()
            events.append("frame-captured")
            return np.zeros(shape, dtype=np.uint8)

    base = fake_factories(events)

    def ack_factory(**values):
        transport = FakeListenerTransport(
            values["config"].connection.source_identity,
            values["config"].connection.wire_protocol,
        )
        original_install = transport.install_message_callback

        def install(names, callback):
            events.append("listener-installed")
            return original_install(names, callback)

        transport.install_message_callback = install
        return transport

    factories = replace(
        base,
        ack_transport_factory=ack_factory,
        camera_factory=lambda **_values: _real_camera_factory(
            config, TimestampedSource()
        ),
    )

    runtime = build_live_listener(artifacts, config, factories=factories)

    assert runtime.camera.precision_readiness().ready is True
    assert events.index("frame-captured") < events.index("listener-installed")
    report = runtime.close()
    assert report.camera_stopped is True


@pytest.mark.parametrize("source_kind", ["blocked", "stale"])
def test_live_factory_rejects_unready_real_camera_before_listener_install(
    tmp_path, source_kind
):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol=artifacts.wire_protocol),
        cleanup_timeout_s=0.05,
    )
    calibration = json.loads(config.vision.calibration_path.read_text())
    shape = (calibration["image_height_px"], calibration["image_width_px"], 3)
    events = []
    release = threading.Event()
    release_timers = []
    cameras = []

    class UnreadySource:
        last_timestamp_ns = None

        def capture_frame(self, quality=4):
            if source_kind == "blocked":
                if not release_timers:
                    timer = threading.Timer(
                        config.precision_policy.frame_timeout_s + 1.0,
                        release.set,
                    )
                    release_timers.append(timer)
                    timer.start()
                release.wait()
            now = config.vision.receipt_clock_ns()
            self.last_timestamp_ns = (
                now
                if source_kind == "blocked"
                else now - config.vision.max_exposure_age_ns - 1
            )
            return np.zeros(shape, dtype=np.uint8)

    base = fake_factories(events)

    def ack_factory(**values):
        transport = FakeListenerTransport(
            values["config"].connection.source_identity,
            values["config"].connection.wire_protocol,
        )
        original_install = transport.install_message_callback

        def install(names, callback):
            events.append("listener-installed")
            return original_install(names, callback)

        transport.install_message_callback = install
        return transport

    def camera_factory(**_values):
        camera = _real_camera_factory(config, UnreadySource())
        cameras.append(camera)
        return camera

    factories = replace(
        base,
        ack_transport_factory=ack_factory,
        camera_factory=camera_factory,
    )

    try:
        with pytest.raises((RuntimeError, TimeoutError)):
            build_live_listener(artifacts, config, factories=factories)
    finally:
        release.set()
        for timer in release_timers:
            timer.cancel()
        for camera in cameras:
            camera.cm.stop_acquisition(timeout_s=1.0)

    assert "listener-installed" not in events


def test_live_factory_rejects_fm3_capability_before_any_component(tmp_path):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol=artifacts.wire_protocol),
    )
    assert FM3 in config.enabled_phases
    events = []

    with pytest.raises(ValueError, match="attachment capability"):
        build_live_listener(
            artifacts,
            config,
            factories=fake_factories(events, supports_attachment=False),
        )

    assert events == []


def admissible_fm1_runtime(
    tmp_path, *, startup_admission_check=None, enabled_phases=None
):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol=artifacts.wire_protocol),
    )
    if enabled_phases is not None:
        config = replace(
            config,
            enabled_phases=frozenset(enabled_phases),
            vision=None if FM3 not in enabled_phases else config.vision,
            precision_policy=(
                None if FM3 not in enabled_phases else config.precision_policy
            ),
        )
    loaded_at = timebase.epoch()
    records = {
        name: {
            "coords": {"lat": 41.0, "long": -81.0, "alt": 10.0},
            "loaded_at": loaded_at,
        }
        for name in ("L", "TARGET", "WA", "WM1")
    }
    config.waypoint_path.write_text(json.dumps(records))
    events = []
    captured = {}
    runtime = build_live_listener(
        artifacts,
        config,
        factories=fake_factories(events, captured=captured),
        startup_admission_check=startup_admission_check,
    )
    state = runtime.flight_state
    source = artifacts.flight_profile.flight_controller
    now = timebase.monotonic()
    metadata = dict(
        received_at=now,
        source_system=source.system_id,
        source_component=source.component_id,
    )
    state.observe_heartbeat(
        (2, 3, 0, 4, 3), armed=False, mode="GUIDED", sequence=1, **metadata
    )
    state.update_many(
        {
            "location": (410000000, -810000000, 250000, 0),
            "velocity": (0, 0, 0),
            "attitude": (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            "landed_state": 1,
            "home": (410000000, -810000000, 250000),
        },
        sequence=2,
        **metadata,
    )
    captured["decoders"].observe_sys_status(
        SimpleNamespace(
            onboard_control_sensors_present=RC_RECEIVER_SENSOR_BIT,
            onboard_control_sensors_enabled=RC_RECEIVER_SENSOR_BIT,
            onboard_control_sensors_health=RC_RECEIVER_SENSOR_BIT,
            get_srcSystem=lambda: source.system_id,
            get_srcComponent=lambda: source.component_id,
        )
    )
    state.observe_rc_input(
        channel=artifacts.flight_profile.rc_channel,
        pwm=1500,
        signal_healthy=True,
        sequence=3,
        **metadata,
    )
    state.observe_failsafe(
        "clear", active=False, sequence=4, **metadata
    )

    packet = Packet(
        source=(200, 190),
        target=(1, 191),
        command=FM1,
        params=(float(prepared.attempt_id), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    )
    return runtime, artifacts, prepared, events, captured, packet


def _queue_phase(runtime, artifacts, prepared, command):
    profile = artifacts.flight_profile
    request = CommandEnvelope(
        source_system=profile.qgc_source.system_id,
        source_component=profile.qgc_source.component_id,
        target_system=1,
        target_component=191,
        command=command,
        attempt_id=prepared.attempt_id,
        params=(float(prepared.attempt_id), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        received_at=timebase.monotonic(),
    )
    assert runtime.supervisor.admit(request)
    runtime.owner.command_queue.put(request)
    return request


def _advance_past_fm1(runtime, artifacts, prepared):
    request = _queue_phase(runtime, artifacts, prepared, FM1)
    runtime.owner.command_queue.get_nowait()
    runtime.owner.command_queue.task_done()
    runtime.supervisor.begin(FM1)
    runtime.supervisor.finish(FM1, "SUCCEEDED")
    return request


def test_fm1_admission_consumes_then_acquires_pins_home_and_constructs_payload(tmp_path):
    runtime, _artifacts, _prepared, events, captured, packet = admissible_fm1_runtime(
        tmp_path
    )

    runtime.listener.handle_message(packet)

    assert runtime.supervisor.status(FM1) == "QUEUED"
    assert runtime.controller.mission_home is not None
    assert runtime.flight_state.ordinary_commands_permitted() is True
    assert events[-1] == "dropper"

    original_snapshot = runtime.flight_state.snapshot
    stale_heartbeat = replace(original_snapshot().heartbeat, fresh=False)
    runtime.flight_state.snapshot = lambda: replace(
        original_snapshot(), heartbeat=stale_heartbeat
    )
    with pytest.raises(AuthorityLost, match="vital flight evidence"):
        captured["permission_guard"]()
    assert runtime.flight_state.authority.value == "UNKNOWN"


def test_final_fm2_recovers_once_after_core_success_and_then_succeeds(
    tmp_path, monkeypatch
):
    runtime, artifacts, prepared, _events, _captured, _packet = (
        admissible_fm1_runtime(tmp_path, enabled_phases=(FM1, FM2))
    )
    _advance_past_fm1(runtime, artifacts, prepared)
    runtime.owner._mission_start = timebase.monotonic()
    runtime.owner._deadline = runtime.owner._mission_start + 600.0
    order = []

    def core_fm2(*_args, **_kwargs):
        order.append("fm2")
        return True

    def recover(controller, home, cruise_m):
        assert timebase._deadline_limit.get() is None
        order.append(("recover", controller, home, cruise_m))
        runtime.supervisor.record_recovery_outcome("HOME_LANDED")
        return "HOME_LANDED"

    monkeypatch.setattr(
        importlib.import_module("drone.missions.fm2"), "fm2", core_fm2
    )
    monkeypatch.setattr(runtime.supervisor, "recover", recover)
    _queue_phase(runtime, artifacts, prepared, FM2)

    assert runtime.owner.process_next(timeout_s=0.0) == "SUCCEEDED"
    assert runtime.supervisor.terminal_result == "SUCCEEDED"
    assert runtime.supervisor.recovery_outcome == "HOME_LANDED"
    assert order[0] == "fm2"
    assert order[1] == (
        "recover",
        runtime.controller,
        runtime.controller.mission_home,
        10.0,
    )
    assert len(order) == 2


@pytest.mark.parametrize("outcome", ["LOCAL_LANDED", "UNCONFIRMED", "PILOT"])
def test_final_fm2_non_home_recovery_is_terminal_failure_without_second_recovery(
    tmp_path, monkeypatch, outcome
):
    runtime, artifacts, prepared, _events, _captured, _packet = (
        admissible_fm1_runtime(tmp_path, enabled_phases=(FM1, FM2))
    )
    _advance_past_fm1(runtime, artifacts, prepared)
    calls = []

    monkeypatch.setattr(
        importlib.import_module("drone.missions.fm2"),
        "fm2",
        lambda *_args, **_kwargs: True,
    )

    def recover(_controller, _home, _cruise_m):
        calls.append(outcome)
        runtime.supervisor.record_recovery_outcome(outcome)
        return outcome

    monkeypatch.setattr(runtime.supervisor, "recover", recover)
    _queue_phase(runtime, artifacts, prepared, FM2)

    assert runtime.owner.process_next(timeout_s=0.0) == "FAILED"
    assert runtime.supervisor.terminal_result == "FAILED"
    assert runtime.supervisor.recovery_outcome == outcome
    assert calls == [outcome]
    assert runtime.supervisor.admit(
        CommandEnvelope(
            200,
            190,
            1,
            191,
            FM2,
            prepared.attempt_id,
            (float(prepared.attempt_id), 0, 0, 0, 0, 0, 0),
            timebase.monotonic(),
        )
    ) is False
    assert calls == [outcome]


def test_failed_core_fm2_uses_only_ordinary_recovery(tmp_path, monkeypatch):
    runtime, artifacts, prepared, _events, _captured, _packet = (
        admissible_fm1_runtime(tmp_path, enabled_phases=(FM1, FM2))
    )
    _advance_past_fm1(runtime, artifacts, prepared)
    order = []

    monkeypatch.setattr(
        importlib.import_module("drone.missions.fm2"),
        "fm2",
        lambda *_args, **_kwargs: order.append("fm2-failed") or False,
    )

    def recover(_controller, _home, _cruise_m):
        order.append("ordinary-recovery")
        runtime.supervisor.record_recovery_outcome("LOCAL_LANDED")
        return "LOCAL_LANDED"

    monkeypatch.setattr(runtime.supervisor, "recover", recover)
    _queue_phase(runtime, artifacts, prepared, FM2)

    assert runtime.owner.process_next(timeout_s=0.0) == "FAILED"
    assert order == ["fm2-failed", "ordinary-recovery"]
    assert runtime.supervisor.recovery_outcome == "LOCAL_LANDED"


def test_final_fm2_preserves_real_recovery_timeout_after_failed_ack():
    cancellation = TimeoutError("return transport cancelled")
    home = MissionHome(41.0, -81.0, 100.0)
    recovery_calls = []
    output_events = []
    acknowledgements = []

    class RecoveryController:
        mission_home = home

        @staticmethod
        def _field(value):
            return SimpleNamespace(
                fresh=True,
                observation=SimpleNamespace(value=value),
            )

        def flight_snapshot(self):
            return SimpleNamespace(
                authority=Authority.COMPANION,
                commands_suspended=False,
                armed=self._field(True),
                landed_state=self._field(2),
                mode=self._field("GUIDED"),
                location=self._field((410000000, -810000000, 103000.0, 0.0)),
            )

        def climb(self, altitude_m, *, timeout):
            output_events.append(("climb", altitude_m, timeout))

        def goto_recovery_waypoint(
            self, waypoint, *, approve_target_amsl, timeout
        ):
            assert callable(approve_target_amsl)
            output_events.append(("return", waypoint, timeout))
            raise cancellation

        def simple_land(self, **_kwargs):
            output_events.append("land")
            pytest.fail("recovery attempted LAND after cancellation")

    supervisor = MissionSupervisor(
        7,
        admission_check=lambda _request: None,
        attempt_consumer=lambda _attempt: None,
        permission_check=lambda: None,
        recovery_policy=RecoveryPolicy(
            check=lambda _operation, _home, _altitude: None,
            timeout_s=30.0,
            local_land_reserve_s=5.0,
            clock=lambda: 100.0,
        ),
        enabled_phases=(FM1, FM2),
    )

    def request(command):
        return CommandEnvelope(
            200,
            190,
            1,
            191,
            command,
            7,
            (7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            1.0,
        )

    fm1_request = request(FM1)
    assert supervisor.admit(fm1_request)
    supervisor.begin(FM1)
    supervisor.finish(FM1, "SUCCEEDED")
    fm2_request = request(FM2)
    assert supervisor.admit(fm2_request)
    commands = queue.Queue()
    commands.put(fm2_request)
    controller = RecoveryController()

    def recover():
        recovery_calls.append("recover")
        return supervisor.recover(controller, home, 10.0)

    owner = CommandExecutionOwner(
        supervisor=supervisor,
        command_queue=commands,
        handlers={FM2: lambda _request: output_events.append("fm2") or True},
        terminal_ack=lambda envelope, result: acknowledgements.append(
            (envelope.command, result)
        ),
        recovery=recover,
        attempt_timeout_s=600.0,
        idle_poll_s=0.01,
        clock=lambda: 20.0,
    )

    with pytest.raises(TimeoutError) as raised:
        owner.process_next(timeout_s=0.0)

    assert raised.value is cancellation
    assert supervisor.terminal_result == "FAILED"
    assert supervisor.abort_reason is None
    assert supervisor.recovery_outcome == "UNCONFIRMED"
    assert supervisor.admission_result(FM2) == "FAILED"
    assert recovery_calls == ["recover"]
    assert [event[0] for event in output_events if isinstance(event, tuple)] == [
        "climb",
        "return",
    ]
    assert "land" not in output_events
    assert acknowledgements == [(FM2, "FAILED")]
    assert commands.unfinished_tasks == 0

    assert owner.run() == "FAILED"
    assert recovery_calls == ["recover"]
    assert "land" not in output_events


def test_closed_startup_barrier_denies_then_open_barrier_admits_same_fm1(tmp_path):
    ready = threading.Event()

    def require_ready():
        if not ready.is_set():
            raise CommandRejected("host startup is not ready")

    runtime, artifacts, prepared, events, captured, packet = admissible_fm1_runtime(
        tmp_path,
        startup_admission_check=require_ready,
    )

    runtime.listener.handle_message(packet)

    assert captured["transport"].acks[-1].result == ACK_DENIED
    assert runtime.supervisor.status(FM1) is None
    assert runtime.controller.mission_home is None
    assert runtime.flight_state.ordinary_commands_permitted() is False
    assert "dropper" not in events
    artifacts.ledger.require_current_unconsumed(prepared.attempt_id)

    ready.set()
    runtime.listener.handle_message(packet)

    assert captured["transport"].acks[-1].result == ACK_IN_PROGRESS
    assert runtime.supervisor.status(FM1) == "QUEUED"
    assert runtime.controller.mission_home is not None
    assert events.count("dropper") == 1
    with pytest.raises(Exception, match="consumed"):
        artifacts.ledger.require_current_unconsumed(prepared.attempt_id)


@pytest.mark.parametrize(
    "startup_admission_check",
    [
        lambda: (_ for _ in ()).throw(CommandRejected("host startup is closed")),
        lambda: True,
    ],
)
def test_startup_barrier_error_or_non_none_result_rejects_without_consumption(
    tmp_path, startup_admission_check
):
    runtime, artifacts, prepared, events, captured, packet = admissible_fm1_runtime(
        tmp_path,
        startup_admission_check=startup_admission_check,
    )

    runtime.listener.handle_message(packet)

    assert captured["transport"].acks[-1].result == ACK_DENIED
    assert runtime.supervisor.status(FM1) is None
    assert runtime.controller.mission_home is None
    assert "dropper" not in events
    artifacts.ledger.require_current_unconsumed(prepared.attempt_id)


def test_cached_waypoint_age_is_rechecked_without_store_io(monkeypatch):
    record = SimpleNamespace(loaded_at=100.0)
    snapshot = SimpleNamespace(
        names=frozenset(("L",)),
        get_record=lambda name: record if name == "L" else None,
    )
    monkeypatch.setattr(
        timebase,
        "epoch",
        lambda: 100.0 + MAX_WAYPOINT_AGE_SECONDS + 0.1,
    )

    with pytest.raises(CommandRejected, match="allowed age"):
        _require_cached_waypoint_ages(snapshot)


def test_fm3_disabled_composition_does_not_require_pickup_waypoints(tmp_path):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol=artifacts.wire_protocol),
        enabled_phases=frozenset((FM1, FM2)),
        vision=None,
        precision_policy=None,
    )
    loaded_at = timebase.epoch()
    config.waypoint_path.write_text(json.dumps({
        name: {
            "coords": {"lat": 41.0, "long": -81.0, "alt": 10.0},
            "loaded_at": loaded_at,
        }
        for name in ("L", "TARGET")
    }))

    runtime = build_live_listener(
        artifacts,
        config,
        factories=fake_factories([], supports_attachment=False),
    )

    assert runtime.camera is None
    assert runtime.supervisor.enabled_phases == (FM1, FM2)


def test_payload_cleanup_never_calls_unverified_raw_servo_close():
    calls = []
    delegate = SimpleNamespace(
        supports_attachment=False,
        servos=(SimpleNamespace(close=lambda: calls.append("raw close")),),
    )
    dropper = _DeferredDropper(
        lambda **_values: delegate,
        object(),
        lambda: True,
        supports_attachment=False,
    )
    dropper.initialize()

    assert dropper.cleanup_passive(0.01) is False
    assert calls == []


@pytest.mark.parametrize("failure", [KeyboardInterrupt(), SystemExit()])
def test_payload_cleanup_contains_control_flow_failure_in_worker(
    monkeypatch, failure
):
    uncaught = []
    monkeypatch.setattr(
        threading,
        "excepthook",
        lambda args: uncaught.append(args.exc_value),
    )
    delegate = SimpleNamespace(
        supports_attachment=False,
        cleanup_passive=lambda: (_ for _ in ()).throw(failure),
    )
    dropper = _DeferredDropper(
        lambda **_values: delegate,
        object(),
        lambda: True,
        supports_attachment=False,
    )
    dropper.initialize()

    assert dropper.cleanup_passive(0.1) is False
    assert uncaught == []


def test_runtime_close_reports_camera_mapping_errors_and_incomplete_payload_cleanup():
    """Break caught: real recording errors and attempted cleanup failure stay visible."""
    manager = SimpleNamespace(stop_acquisition=lambda **_kwargs: True)
    camera = Camera(100, manager=manager)
    camera._recording_errors["capture.mp4"] = "writer failed"
    runtime = LiveListenerRuntime(
        listener=None,
        owner=None,
        supervisor=None,
        controller=SimpleNamespace(vehicle=SimpleNamespace()),
        flight_state=None,
        tracker=None,
        lidar=FakeLidar(),
        camera=camera,
        dropper=SimpleNamespace(cleanup_passive=lambda _timeout: False),
        cleanup_timeout_s=0.01,
        diagnostics=lambda _message: None,
        monitoring_stop=threading.Event(),
    )

    report = runtime.close()

    assert "camera recording capture.mp4: writer failed" in report.diagnostics
    assert "payload cleanup incomplete or unconfirmed" in report.diagnostics


@pytest.mark.parametrize(
    "failure",
    [RuntimeError("payload failed"), KeyboardInterrupt("payload interrupted"), SystemExit("payload exited")],
)
def test_runtime_close_continues_safe_cleanup_after_payload_base_exception(failure):
    calls = []

    def payload_cleanup(_timeout):
        calls.append("payload")
        raise failure

    runtime = LiveListenerRuntime(
        listener=None,
        owner=None,
        supervisor=None,
        controller=SimpleNamespace(
            vehicle=SimpleNamespace(close=lambda: calls.append("vehicle"))
        ),
        flight_state=None,
        tracker=None,
        lidar=FakeLidar(),
        camera=None,
        dropper=SimpleNamespace(cleanup_passive=payload_cleanup),
        cleanup_timeout_s=0.1,
        diagnostics=lambda message: calls.append(("diagnostic", message)),
        monitoring_stop=threading.Event(),
        _artifacts=SimpleNamespace(close=lambda: calls.append("artifacts")),
    )

    report = runtime.close()

    assert report.payload_closed is False
    assert report.vehicle_closed is True
    assert calls[:3] == ["payload", "vehicle", "artifacts"]
    assert "payload cleanup failed" in report.diagnostics[0]
    assert calls[3] == ("diagnostic", report.diagnostics[0])


def test_runtime_close_diagnostic_base_exception_does_not_escape_or_hide_report():
    runtime = LiveListenerRuntime(
        listener=None,
        owner=None,
        supervisor=None,
        controller=SimpleNamespace(vehicle=SimpleNamespace(close=lambda: None)),
        flight_state=None,
        tracker=None,
        lidar=FakeLidar(),
        camera=None,
        dropper=SimpleNamespace(cleanup_passive=lambda _timeout: False),
        cleanup_timeout_s=0.1,
        diagnostics=lambda _message: (_ for _ in ()).throw(SystemExit("diagnostic exited")),
        monitoring_stop=threading.Event(),
    )

    report = runtime.close()

    assert report.payload_closed is False
    assert report.diagnostics == ("payload cleanup incomplete or unconfirmed",)


@pytest.mark.parametrize("failure", [KeyboardInterrupt(), SystemExit()])
def test_failed_startup_cleanup_continues_to_vehicle_after_base_exception(failure):
    calls = []
    telemetry = SimpleNamespace(
        close=lambda: (calls.append("telemetry"), (_ for _ in ()).throw(failure))[1]
    )
    lidar = SimpleNamespace(stop=lambda **_kwargs: calls.append("lidar"))
    camera = SimpleNamespace(
        cm=SimpleNamespace(stop_acquisition=lambda **_kwargs: calls.append("camera"))
    )
    controller = SimpleNamespace(
        vehicle=SimpleNamespace(close=lambda: calls.append("vehicle"))
    )

    _cleanup_failed_startup(
        controller=controller,
        lidar=lidar,
        camera=camera,
        timeout_s=0.1,
        telemetry_collector=telemetry,
    )

    assert calls == ["telemetry", "camera", "lidar", "vehicle"]


@pytest.mark.parametrize("broken_operation", ["start", "join"])
def test_runtime_close_continues_to_artifacts_after_vehicle_thread_failure(
    monkeypatch, broken_operation
):
    import drone.control.listener as listener_module

    calls = []

    class BrokenThread:
        def __init__(self, **_kwargs):
            calls.append("thread-constructed")

        def start(self):
            calls.append("thread-start")
            if broken_operation == "start":
                raise RuntimeError("thread start failed")

        def join(self, _timeout):
            calls.append("thread-join")
            if broken_operation == "join":
                raise KeyboardInterrupt("thread join interrupted")

        @staticmethod
        def is_alive():
            return False

    monkeypatch.setattr(listener_module.threading, "Thread", BrokenThread)
    runtime = LiveListenerRuntime(
        listener=None,
        owner=None,
        supervisor=None,
        controller=SimpleNamespace(vehicle=SimpleNamespace(close=lambda: None)),
        flight_state=None,
        tracker=None,
        lidar=FakeLidar(),
        camera=None,
        dropper=SimpleNamespace(cleanup_passive=lambda _timeout: True),
        cleanup_timeout_s=0.1,
        diagnostics=lambda message: calls.append(("diagnostic", message)),
        monitoring_stop=threading.Event(),
        _artifacts=SimpleNamespace(close=lambda: calls.append("artifacts")),
    )

    report = runtime.close()

    assert report.vehicle_closed is False
    assert "artifacts" in calls
    assert any("vehicle close worker" in note for note in report.diagnostics)


@pytest.mark.parametrize(
    ("stage", "failure"),
    [
        ("telemetry", RuntimeError()),
        ("camera_manager", KeyboardInterrupt()),
        ("recording_errors", SystemExit()),
        ("vehicle", RuntimeError()),
        ("artifacts", KeyboardInterrupt()),
    ],
)
def test_runtime_close_guards_cleanup_capability_property_reads(stage, failure):
    calls = []

    class HostileProperty:
        def __init__(self, attribute):
            self.attribute = attribute

        def __getattribute__(self, attribute):
            if attribute not in {"attribute", "__class__"} and attribute == object.__getattribute__(self, "attribute"):
                raise failure
            return object.__getattribute__(self, attribute)

    camera = SimpleNamespace(
        cm=SimpleNamespace(stop_acquisition=lambda **_kwargs: calls.append("camera-stop") or True),
        wait_for_recordings=lambda _timeout: calls.append("camera-wait") or True,
        recording_errors={},
    )
    if stage == "camera_manager":
        camera = HostileProperty("cm")
    elif stage == "recording_errors":
        camera = HostileProperty("recording_errors")
    telemetry = (
        HostileProperty("close")
        if stage == "telemetry"
        else SimpleNamespace(close=lambda: calls.append("telemetry"))
    )
    controller = (
        HostileProperty("vehicle")
        if stage == "vehicle"
        else SimpleNamespace(vehicle=SimpleNamespace(close=lambda: calls.append("vehicle")))
    )
    artifacts = (
        HostileProperty("close")
        if stage == "artifacts"
        else SimpleNamespace(close=lambda: calls.append("artifacts"))
    )
    runtime = LiveListenerRuntime(
        listener=None,
        owner=None,
        supervisor=None,
        controller=controller,
        flight_state=None,
        tracker=None,
        lidar=FakeLidar(),
        camera=camera,
        dropper=SimpleNamespace(
            cleanup_passive=lambda _timeout: calls.append("payload") or True
        ),
        cleanup_timeout_s=0.1,
        diagnostics=lambda message: calls.append(("diagnostic", message)),
        monitoring_stop=threading.Event(),
        _artifacts=artifacts,
        _telemetry_collector=telemetry,
    )

    report = runtime.close()

    assert "payload" in calls
    assert "artifacts" in calls or stage == "artifacts"
    assert "vehicle" in calls or stage == "vehicle"
    assert any(type(failure).__name__ in note for note in report.diagnostics)


@pytest.mark.parametrize(
    ("stage", "failure"),
    [
        ("telemetry", RuntimeError()),
        ("camera", KeyboardInterrupt()),
        ("lidar", SystemExit()),
        ("vehicle", RuntimeError()),
    ],
)
def test_failed_startup_cleanup_guards_capability_property_reads(stage, failure):
    calls = []

    class HostileProperty:
        def __init__(self, attribute):
            self.attribute = attribute

        def __getattribute__(self, attribute):
            if attribute not in {"attribute", "__class__"} and attribute == object.__getattribute__(self, "attribute"):
                raise failure
            return object.__getattribute__(self, attribute)

    telemetry = HostileProperty("close") if stage == "telemetry" else SimpleNamespace(close=lambda: calls.append("telemetry"))
    camera = HostileProperty("cm") if stage == "camera" else SimpleNamespace(cm=SimpleNamespace(stop_acquisition=lambda **_kwargs: calls.append("camera")))
    lidar = HostileProperty("stop") if stage == "lidar" else SimpleNamespace(stop=lambda **_kwargs: calls.append("lidar"))
    controller = HostileProperty("vehicle") if stage == "vehicle" else SimpleNamespace(vehicle=SimpleNamespace(close=lambda: calls.append("vehicle")))

    _cleanup_failed_startup(
        controller=controller,
        lidar=lidar,
        camera=camera,
        timeout_s=0.1,
        telemetry_collector=telemetry,
    )

    if stage != "vehicle":
        assert "vehicle" in calls
    if stage in {"telemetry", "camera"}:
        assert "lidar" in calls


def test_runtime_keeps_command_silent_monitoring_until_new_ground_disarm():
    """Break caught: PILOT terminalization must not close observation ownership."""
    def field(value, sequence):
        return SimpleNamespace(
            fresh=True,
            observation=SimpleNamespace(value=value, sequence=sequence),
        )

    snapshots = iter(
        (
            SimpleNamespace(landed_state=field(2, 10), armed=field(True, 10)),
            SimpleNamespace(landed_state=field(1, 11), armed=field(False, 11)),
        )
    )
    supervisor = SimpleNamespace(terminal_result="ABORTED", recovery_outcome="PILOT")
    stop = threading.Event()
    runtime = LiveListenerRuntime(
        listener=SimpleNamespace(request_abort=lambda _reason: pytest.fail("abort issued")),
        owner=SimpleNamespace(run=lambda: "ABORTED", _idle_poll_s=0.001),
        supervisor=supervisor,
        controller=None,
        flight_state=SimpleNamespace(snapshot=lambda: next(snapshots)),
        tracker=None,
        lidar=None,
        camera=None,
        dropper=None,
        cleanup_timeout_s=0.01,
        diagnostics=lambda _message: None,
        monitoring_stop=stop,
    )

    result = runtime.run()

    assert result.mission_result == "ABORTED"
    assert result.recovery_outcome == "PILOT"
    assert result.monitoring_exit_reason == "OBSERVED_GROUND_DISARMED"


def test_terminal_stop_exits_monitoring_without_recovery_or_commands():
    """Break caught: explicit terminal stop must only end passive observation."""
    stop = threading.Event()
    stop.set()
    aborts = []
    runtime = LiveListenerRuntime(
        listener=SimpleNamespace(request_abort=aborts.append),
        owner=SimpleNamespace(run=lambda: "FAILED", _idle_poll_s=0.001),
        supervisor=SimpleNamespace(terminal_result="FAILED", recovery_outcome="UNCONFIRMED"),
        controller=None,
        flight_state=SimpleNamespace(
            snapshot=lambda: SimpleNamespace(
                landed_state=None,
                armed=None,
            )
        ),
        tracker=None,
        lidar=None,
        camera=None,
        dropper=None,
        cleanup_timeout_s=0.01,
        diagnostics=lambda _message: None,
        monitoring_stop=stop,
    )

    runtime.terminal_monitoring.set()
    runtime.request_abort("SIGINT")
    result = runtime.run()

    assert aborts == []
    assert result.monitoring_exit_reason == "EXPLICIT_STOP_UNCONFIRMED"


def test_repeated_abort_during_recovery_is_delivered_once_then_stops_monitoring():
    aborts = []
    stop = threading.Event()
    runtime = LiveListenerRuntime(
        listener=SimpleNamespace(request_abort=aborts.append),
        owner=None,
        supervisor=None,
        controller=None,
        flight_state=None,
        tracker=None,
        lidar=None,
        camera=None,
        dropper=None,
        cleanup_timeout_s=0.01,
        diagnostics=lambda _message: None,
        monitoring_stop=stop,
    )

    runtime.request_abort("SIGINT")
    runtime.request_abort("SIGINT repeated")
    runtime.terminal_monitoring.set()
    runtime.request_abort("SIGINT terminal")

    assert aborts == ["SIGINT"]
    assert stop.is_set()


def test_start_repl_returns_separate_mission_recovery_monitor_and_cleanup(monkeypatch):
    import drone.control.listener as listener_module

    cleanup = LiveCleanupReport(True, True, True, True, False, True, ("payload",))
    runtime = SimpleNamespace(
        run=lambda: LiveRunResult("FAILED", "PILOT", "OBSERVED_GROUND_DISARMED"),
        close=lambda: cleanup,
    )
    monkeypatch.setattr(
        listener_module,
        "construct_after_full_validation",
        lambda *_args, **_kwargs: runtime,
    )

    result = start_repl(object(), object())

    assert result == LiveRunResult(
        mission_result="FAILED",
        recovery_outcome="PILOT",
        monitoring_exit_reason="OBSERVED_GROUND_DISARMED",
        cleanup_report=cleanup,
    )


def test_start_repl_preserves_run_failure_when_cleanup_also_fails(monkeypatch):
    import drone.control.listener as listener_module

    primary = RuntimeError("mission execution failed")
    secondary = RuntimeError("cleanup failed")
    runtime = SimpleNamespace(
        run=lambda: (_ for _ in ()).throw(primary),
        close=lambda: (_ for _ in ()).throw(secondary),
    )
    monkeypatch.setattr(
        listener_module,
        "construct_after_full_validation",
        lambda *_args, **_kwargs: runtime,
    )

    with pytest.raises(RuntimeError, match="mission execution failed") as raised:
        start_repl(object(), object())

    assert raised.value is primary
    assert raised.value.__cause__ is secondary


def test_caller_owned_signal_mode_does_not_read_or_replace_signal_handlers(
    monkeypatch,
):
    import drone.control.listener as listener_module

    monkeypatch.setattr(
        LiveListenerRuntime,
        "_monitor_if_required",
        lambda _runtime, _outcome: "NOT_REQUIRED",
    )
    runtime = LiveListenerRuntime(
        listener=None,
        owner=SimpleNamespace(run=lambda: "SUCCEEDED"),
        supervisor=SimpleNamespace(recovery_outcome="HOME_LANDED"),
        controller=None,
        flight_state=SimpleNamespace(
            snapshot=lambda: SimpleNamespace(landed_state=None, armed=None)
        ),
        tracker=None,
        lidar=None,
        camera=None,
        dropper=None,
        cleanup_timeout_s=0.01,
        diagnostics=lambda _message: None,
        monitoring_stop=threading.Event(),
        manage_signals=False,
    )
    monkeypatch.setattr(
        listener_module.signal,
        "getsignal",
        lambda *_args: pytest.fail("caller-owned signals must not be read"),
    )
    monkeypatch.setattr(
        listener_module.signal,
        "signal",
        lambda *_args: pytest.fail("caller-owned signals must not be replaced"),
    )

    assert runtime.run().mission_result == "SUCCEEDED"


def test_start_repl_calls_ready_once_after_install_before_run_in_clock_scope(
    tmp_path, monkeypatch
):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol=artifacts.wire_protocol),
    )
    events = []
    captured = {}
    cleanup = LiveCleanupReport(True, True, True, True, True, True, ())
    clock = SimpleNamespace(now=lambda: 321.0, sleep=lambda _seconds: None)
    phase_observer = lambda _phase, _state: None

    def ready(runtime):
        assert isinstance(runtime, LiveListenerRuntime)
        assert callable(captured["transport"].callback)
        assert runtime.owner._phase_observer is phase_observer
        assert timebase.monotonic() == 321.0
        events.append("ready")

    def run(runtime):
        assert timebase.monotonic() == 321.0
        events.append("run")
        return LiveRunResult("FAILED", "PILOT", "OBSERVED_GROUND_DISARMED")

    def close(runtime):
        assert timebase.monotonic() == 321.0
        events.append("close")
        return cleanup

    monkeypatch.setattr(LiveListenerRuntime, "run", run)
    monkeypatch.setattr(LiveListenerRuntime, "close", close)

    with timebase.configured(clock):
        result = start_repl(
            files,
            config,
            factories=fake_factories(events, captured=captured),
            on_listener_ready=ready,
            phase_observer=phase_observer,
        )

    assert events[-3:] == ["ready", "run", "close"]
    assert events.count("ready") == 1
    assert result.cleanup_report is cleanup


@pytest.mark.parametrize("hook_result", ["raise", "non-none"])
@pytest.mark.parametrize("cleanup_fails", [False, True])
def test_start_repl_ready_failure_closes_without_running(
    tmp_path, monkeypatch, hook_result, cleanup_fails
):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol=artifacts.wire_protocol),
    )
    events = []
    original_error = RuntimeError("ready publication failed")
    cleanup_error = RuntimeError("cleanup failed")

    def ready(_runtime):
        events.append("ready")
        if hook_result == "raise":
            raise original_error
        return True

    monkeypatch.setattr(
        LiveListenerRuntime,
        "run",
        lambda _runtime: events.append("run") or pytest.fail("owner ran"),
    )
    def close(_runtime):
        events.append("close")
        if cleanup_fails:
            raise cleanup_error

    monkeypatch.setattr(LiveListenerRuntime, "close", close)

    expected = RuntimeError if hook_result == "raise" else TypeError
    with pytest.raises(expected) as raised:
        start_repl(
            files,
            config,
            factories=fake_factories(events),
            on_listener_ready=ready,
        )

    if hook_result == "raise":
        assert raised.value is original_error
    if cleanup_fails:
        assert raised.value.__cause__ is cleanup_error
    assert events[-2:] == ["ready", "close"]
    assert "run" not in events


@pytest.mark.parametrize(
    ("keyword", "value"),
    [
        ("startup_admission_check", object()),
        ("on_listener_ready", object()),
    ],
)
def test_malformed_startup_hooks_fail_before_component_construction(
    tmp_path, keyword, value
):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol=artifacts.wire_protocol),
    )
    events = []

    with pytest.raises(TypeError, match=keyword):
        start_repl(files, config, factories=fake_factories(events), **{keyword: value})

    assert events == []


@pytest.mark.parametrize(
    "bad_hook",
    [
        {"startup_admission_check": object()},
        {"on_listener_ready": object()},
    ],
)
def test_bad_prepared_artifacts_win_before_malformed_hooks(tmp_path, bad_hook):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    artifacts.ledger.consume(prepared.attempt_id)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol=artifacts.wire_protocol),
    )
    events = []

    with pytest.raises(Exception, match="consumed"):
        start_repl(
            files,
            config,
            factories=fake_factories(events),
            **bad_hook,
        )

    assert events == []


@pytest.mark.parametrize(
    "bad_hook",
    [
        {"startup_admission_check": object()},
        {"on_listener_ready": object()},
    ],
)
def test_bad_runtime_binding_wins_before_malformed_hooks(tmp_path, bad_hook):
    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, _prepared = prepared_startup_files(attempt_dir)
    config = runtime_configuration(runtime_dir)
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol="1.0"),
    )
    events = []

    with pytest.raises(ValueError, match="wire protocol"):
        start_repl(
            files,
            config,
            factories=fake_factories(events),
            **bad_hook,
        )

    assert events == []


def test_runtime_sigint_latch_breaks_receiver_decoder_lock_cycle():
    from drone.control.mission_supervisor import MissionSupervisor

    mission = MissionSupervisor(
        7,
        admission_check=lambda _request: None,
        attempt_consumer=lambda _attempt: None,
        permission_check=lambda: None,
    )
    runtime = LiveListenerRuntime(
        listener=SimpleNamespace(request_abort=mission.request_abort),
        owner=None,
        supervisor=mission,
        controller=None,
        flight_state=None,
        tracker=None,
        lidar=None,
        camera=None,
        dropper=None,
        cleanup_timeout_s=0.01,
        diagnostics=lambda _message: None,
        monitoring_stop=threading.Event(),
    )
    decoder_lock = threading.Lock()
    state_lock = threading.RLock()
    receiver_holds_state = threading.Event()
    receiver_done = threading.Event()

    def receiver():
        with mission._lock:
            receiver_holds_state.set()
            with decoder_lock:
                receiver_done.set()

    decoder_lock.acquire()
    receiver_thread = threading.Thread(target=receiver)
    receiver_thread.start()
    assert receiver_holds_state.wait(0.2)
    with state_lock, mission._output_gate:
        runtime.request_abort("SIGINT")
    assert mission._output_abort_reason == "SIGINT"
    decoder_lock.release()
    receiver_thread.join(0.5)

    assert receiver_done.is_set()
    assert mission.terminal_result == "ABORTED"


def test_public_sigint_does_not_wait_behind_qgc_abort_holding_output_gate():
    """QGC abort admission cannot hold output-gate while waiting for state."""
    from drone.control.mission_supervisor import (
        ABORT_AND_RECOVER,
        CommandEnvelope,
        MissionSupervisor,
    )

    admission_entered = threading.Event()

    def admission_check(_request):
        admission_entered.set()

    mission = MissionSupervisor(
        7,
        admission_check=admission_check,
        attempt_consumer=lambda _attempt: None,
        permission_check=lambda: None,
    )
    assert mission.admit(
        CommandEnvelope(200, 190, 1, 191, FM1, 7, (7.0, 0, 0, 0, 0, 0, 0), 1.0)
    )
    mission.begin(FM1)
    admission_entered.clear()
    runtime = LiveListenerRuntime(
        listener=SimpleNamespace(request_abort=mission.request_abort),
        owner=None,
        supervisor=mission,
        controller=None,
        flight_state=None,
        tracker=None,
        lidar=None,
        camera=None,
        dropper=None,
        cleanup_timeout_s=0.01,
        diagnostics=lambda _message: None,
        monitoring_stop=threading.Event(),
    )
    real_gate = mission._output_gate
    main_thread = threading.get_ident()
    receiver_acquired_gate = threading.Event()

    class BoundedGate:
        def __enter__(self):
            if threading.get_ident() == main_thread:
                if not real_gate.acquire(timeout=0.1):
                    raise TimeoutError("SIGINT waited behind QGC abort output gate")
            else:
                real_gate.acquire()
                receiver_acquired_gate.set()
            return self

        def __exit__(self, _type, _value, _traceback):
            real_gate.release()

    mission._output_gate = BoundedGate()
    receiver_errors = []
    abort = CommandEnvelope(
        200,
        190,
        1,
        191,
        ABORT_AND_RECOVER,
        7,
        (7.0, 0, 0, 0, 0, 0, 0),
        2.0,
    )

    def receive_abort():
        try:
            mission.admit(abort)
        except BaseException as error:
            receiver_errors.append(error)

    with mission._lock:
        receiver = threading.Thread(target=receive_abort)
        receiver.start()
        assert admission_entered.wait(0.2)
        receiver_acquired_gate.wait(0.05)
        runtime.request_abort("SIGINT")

    receiver.join(0.5)
    assert not receiver.is_alive()
    assert mission.abort_reason == "SIGINT"
    assert len(receiver_errors) == 1
    assert isinstance(receiver_errors[0], CommandRejected)
def test_direct_build_live_listener_rechecks_consumed_attempt_before_config(
    tmp_path,
):
    files, prepared = prepared_startup_files(tmp_path)
    artifacts = load_listener_artifacts(files)
    artifacts.ledger.consume(prepared.attempt_id)

    with pytest.raises(Exception, match="consumed"):
        build_live_listener(artifacts, object())
