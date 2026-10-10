from __future__ import annotations

import collections
import collections.abc
from dataclasses import dataclass, field
import importlib
from io import StringIO
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import threading
import time
from types import SimpleNamespace

import pytest

from artifacts.operator_protocol import OperatorProtocol
from artifacts.operator_protocol import PreflightStatus
from artifacts.qgc_operator import (
    MavlinkIdentity,
    QGC_ABORT_COMMAND,
    QGC_ACTIONS_KIND,
    QGC_START_COMMAND,
    QgcCommandEnvelope,
    QgcCommandResponse,
    QgcOperatorDescriptor,
)
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from drone_sim_companion.operator_control import CompanionOperatorControl, RunningStateBinding
from drone_sim_companion.mission import CommandKind, MissionPhase, Telemetry


RUN_ID = "00000000-0000-4000-8000-000000000001"


def _transport_module():
    if not hasattr(collections, "MutableMapping"):
        collections.MutableMapping = collections.abc.MutableMapping
    return importlib.import_module("drone_sim_companion.qgc_transport")


def _frame(message, *, source: MavlinkIdentity) -> bytes:
    encoder = mavlink2.MAVLink(
        None, srcSystem=source.system_id, srcComponent=source.component_id
    )
    return message.pack(encoder)


def _parse(frame: bytes):
    parser = mavlink2.MAVLink(None)
    decoded = None
    for byte in frame:
        value = parser.parse_char(bytes((byte,)))
        if value is not None:
            decoded = value
    assert decoded is not None
    return decoded


@dataclass
class _RawConnection:
    messages: list[object] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.frames: list[bytes] = []
        self.incoming: collections.deque[object] = collections.deque(self.messages)
        self.mav = mavlink2.MAVLink(
            self, srcSystem=1, srcComponent=191
        )

    def write(self, value: bytes) -> None:
        self.frames.append(bytes(value))

    def recv_match(self, *, blocking: bool):
        assert blocking is False
        return self.incoming.popleft() if self.incoming else None

    def close(self) -> None:
        pass


def test_pymavlink_transport_decodes_command_and_targets_ack() -> None:
    module = _transport_module()
    connection = _RawConnection()
    received = []
    wall = iter((123, 124))
    transport = module.PymavlinkQgcCommandIO(
        connection, mavlink2, wall_time_ns=lambda: next(wall)
    )
    transport.install(received.append)
    message = _parse(
        _frame(
            mavlink2.MAVLink_command_long_message(
                1, 191, 31020, 0, 987654, 0, 0, 0, 0, 0, 0
            ),
            source=MavlinkIdentity(255, 190),
        )
    )

    assert transport.observe_message(message) is True
    assert len(received) == 1
    envelope = received[0]
    assert envelope.wire_version == 2
    assert envelope.source == MavlinkIdentity(255, 190)
    assert envelope.target == MavlinkIdentity(1, 191)
    assert envelope.parameters == (987654.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    connection.frames.clear()  # Ignore the route-advertising heartbeat.
    transport.send_ack(
        QgcCommandResponse(31020, MavlinkIdentity(255, 190), 5, 25, "Start pending")
    )
    ack, status = map(_parse, connection.frames)
    assert (ack.get_srcSystem(), ack.get_srcComponent()) == (1, 191)
    assert (ack.command, ack.result, ack.progress) == (31020, 5, 25)
    assert (ack.target_system, ack.target_component) == (255, 190)
    assert status.text == "Start pending"


def test_native_fc_command_is_left_for_flight_controller() -> None:
    module = _transport_module()
    connection = _RawConnection()
    received = []
    transport = module.PymavlinkQgcCommandIO(connection, mavlink2)
    transport.install(received.append)
    message = _parse(
        _frame(
            mavlink2.MAVLink_command_long_message(
                1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0
            ),
            source=MavlinkIdentity(255, 190),
        )
    )

    assert transport.observe_message(message) is False
    assert received == []


def test_dronekit_transport_preserves_qgc_ack_target_through_real_wrapper() -> None:
    module = _transport_module()
    from dronekit.mavlink import MAVConnection
    from drone_sim_companion.dronekit_sim import SimulationVehicle

    module.configure_dronekit_mavlink2()
    connection = MAVConnection(
        "udpin:127.0.0.1:0", source_system=1, source_component=191
    )
    try:
        vehicle = SimulationVehicle(connection)
        transport = module.DroneKitQgcCommandIO(vehicle, mavlink2)
        heartbeat = _parse(connection.out_queue.get_nowait())
        assert heartbeat.get_type() == "HEARTBEAT"
        assert (heartbeat.get_srcSystem(), heartbeat.get_srcComponent()) == (1, 191)
        received = []
        transport.install(received.append)
        command = _parse(
            _frame(
                mavlink2.MAVLink_command_long_message(
                    1, 191, 31021, 0, 987654, 0, 0, 0, 0, 0, 0
                ),
                source=MavlinkIdentity(255, 190),
            )
        )
        vehicle.notify_message_listeners("COMMAND_LONG", command)
        vehicle.notify_message_listeners("COMMAND_LONG", command)
        assert [item.parameters[0] for item in received] == [987654.0, 987654.0]
        transport.send_ack(
            QgcCommandResponse(
                31021, MavlinkIdentity(255, 190), 0, 100, "Abort accepted"
            )
        )
        ack = _parse(connection.out_queue.get_nowait())
        status = _parse(connection.out_queue.get_nowait())
        assert (ack.get_srcSystem(), ack.get_srcComponent()) == (1, 191)
        assert (ack.target_system, ack.target_component) == (255, 190)
        assert status.text == "Abort accepted"
    finally:
        connection.mavlink_thread_in = None
        connection.mavlink_thread_out = None


def test_qgc_start_preserves_request_scoped_50ms_epoch(tmp_path) -> None:
    module = _transport_module()
    run_id = "00000000-0000-4000-8000-000000000001"
    config_sha256 = "a" * 64
    run_directory = tmp_path / run_id
    for relative in (".control", ".status", "configuration"):
        (run_directory / relative).mkdir(parents=True, exist_ok=True)
    protocol = OperatorProtocol(run_directory, run_id, config_sha256)
    descriptor = QgcOperatorDescriptor(
        QGC_ACTIONS_KIND,
        QGC_START_COMMAND,
        QGC_ABORT_COMMAND,
        17,
        "00000000-0000-4000-8000-000000000002",
        "00000000-0000-4000-8000-000000000003",
        MavlinkIdentity(255, 190),
        MavlinkIdentity(1, 191),
        MavlinkIdentity(1, 1),
        "configuration/qgc-actions.json",
        "b" * 64,
    )
    wall = [2_000_000_000]
    base = CompanionOperatorControl(
        protocol=protocol,
        run_id=run_id,
        config_sha256=config_sha256,
        launch_mode="operator",
        preparation_wall_seconds=1800,
        mission_wall_seconds=120,
        recovery_wall_seconds=10,
        wall_time_ns=lambda: wall[0],
        monotonic_ns=lambda: wall[0],
    )

    class IO:
        receive = None

        def __init__(self) -> None:
            self.responses = []

        def install(self, receive) -> None:
            assert self.receive is None
            self.receive = receive

        def send_ack(self, response) -> None:
            self.responses.append(response)

    io = IO()
    host = module.QgcHostControl(
        protocol=protocol,
        operator=base,
        descriptor=descriptor,
        io=io,
        readiness_reasons=lambda: (),
        is_finalizing=lambda: False,
    )
    host.observe_vehicle(
        native_timestamp_ns=0,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=False,
        landed=True,
        mode="STABILIZE",
    )
    start = QgcCommandEnvelope(
        2,
        descriptor.qgc_source,
        descriptor.companion,
        descriptor.start_command,
        (17, 0, 0, 0, 0, 0, 0),
        1_000_000_000,
    )
    assert io.receive is not None
    with ThreadPoolExecutor(max_workers=8) as pool:
        tuple(pool.map(io.receive, (start,) * 80))

    assert host.poll_start(run_state=None).command_allowed is False
    assert len(protocol.read_qgc_command_statuses()) == 65
    request = protocol.read_start_request()
    assert request is not None
    admission = protocol.write_start_admission(request, native_epoch_ns=90_050_000_000)
    assert admission.request_id == request.request_id
    assert admission.request_sha256 == request.sha256
    assert admission.native_epoch_ns % 50_000_000 == 0
    running = RunningStateBinding(run_id, config_sha256, admission.sha256)
    first = host.poll_start(run_state=running)
    retry = host.poll_start(run_state=running)
    assert first.command_allowed is True and first.admitted_now is True
    assert retry.command_allowed is True and retry.admitted_now is False
    assert protocol.read_start_request() == request

    wall[0] = 2_100_000_000
    host.observe_vehicle(
        native_timestamp_ns=90_100_000_000,
        heartbeat_live=True,
        prearm_checks_healthy=True,
        armed=True,
        landed=False,
        mode="GUIDED",
    )
    abort = QgcCommandEnvelope(
        2,
        descriptor.qgc_source,
        descriptor.companion,
        descriptor.abort_command,
        (17, 0, 0, 0, 0, 0, 0),
        1_000_000_001,
    )
    recovery_calls = []
    io.receive(abort)
    io.receive(abort)
    host.poll_abort(
        finalizing=False,
        begin_recovery=lambda reason: recovery_calls.append(reason) is None,
    )
    host.poll_abort(
        finalizing=False,
        begin_recovery=lambda reason: recovery_calls.append(reason) is None,
    )
    assert recovery_calls == ["QGC requested guarded recovery"]
    protocol.close()


class _HostProtocol:
    def __init__(self, operator: OperatorProtocol) -> None:
        self.operator = operator
        self.statuses: list[object] = []

    def write_status(self, status: object) -> None:
        self.statuses.append(status)

    def read_finalize_request(self) -> None:
        return None

    def write_quiescence(self, _module: str) -> None:
        pass

    def close(self) -> None:
        pass


class _MissionVehicle:
    def __init__(self) -> None:
        self.commands: list[tuple[CommandKind, float | None]] = []
        self.actions: list[tuple[str, object]] = []
        self.parameters: dict[str, float] = {}
        self.channels = SimpleNamespace(overrides={})
        self.armed = False
        self._mode = SimpleNamespace(name="STABILIZE")
        self._master = SimpleNamespace(
            target_system=1,
            target_component=1,
            mav=SimpleNamespace(
                param_request_list_send=lambda *_args: None,
                param_set_send=self._parameter,
                command_long_send=lambda *_args: None,
            ),
        )

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value) -> None:
        self._mode = value
        self.actions.append(("mode", value.name))

    def _parameter(self, _system, _component, name, value, _kind) -> None:
        key = bytes(name).decode("ascii")
        self.parameters[key] = float(value)
        self.actions.append(("parameter", key))

    def send(self, command: CommandKind, altitude_m: float | None) -> None:
        self.commands.append((command, altitude_m))

    def send_waypoint(self, *_args, **_kwargs) -> None:
        pass

    def send_landing_target(self, *_args, **_kwargs) -> None:
        pass

    def simple_takeoff(self, altitude_m: float) -> None:
        self.actions.append(("takeoff", altitude_m))


def _host_release(
    mission: str,
    *,
    run_directory: Path,
    protocol: _HostProtocol,
):
    """Return the actual selected host's one-shot release and recovery probes."""
    from artifacts.runtime_status import OperatorWaitStartedStatus
    from drone_sim_companion import calibration_autotune
    from drone_sim_companion.autotune import Observation as AutoTuneObservation
    from drone_sim_companion.autotune import RollAutoTuneDriver
    from drone_sim_companion.comp2026_host import Comp2026StartGate
    from drone_sim_companion.configured_runtime import ConfiguredHost
    from drone_sim_companion.controller import MissionController
    from drone_sim_companion.hover import Observation as HoverObservation
    from drone_sim_companion.hover import RollHoverDriver
    from drone_sim_companion.lifecycle import CompanionLifecycle
    from drone_sim_companion.mission_plan import parse_mission_plan

    vehicle = _MissionVehicle()
    recovery_calls: list[str] = []

    if mission == "controlled_descent":
        controller = MissionController(vehicle, lambda *_args: None)
        controller.observe_readiness(
            Telemetry(0, heartbeat=True, prearm_checks_healthy=True)
        )

        def release(allowed: bool) -> None:
            if allowed and controller.state.phase is MissionPhase.WAIT_HEARTBEAT:
                controller.begin_mission(0)

        def recover(reason: str) -> bool:
            recovery_calls.append(reason)
            vehicle.send(CommandKind.LAND, None)
            return True

        def verify() -> None:
            assert vehicle.commands.count((CommandKind.SET_GUIDED, None)) == 1
            assert vehicle.commands.count((CommandKind.LAND, None)) == 1

        return release, recover, verify, None

    if mission == "configured":
        lifecycle = CompanionLifecycle(
            run_id=RUN_ID,
            protocol=protocol,
            stream=StringIO(),
        )
        host = ConfiguredHost(
            parse_mission_plan(
                {
                    "schema_version": 1,
                    "steps": [
                        {
                            "tool": "wait_for_state",
                            "args": {"armed": True, "mode": "GUIDED"},
                        }
                    ],
                }
            ),
            vehicle,
            lifecycle,
            protocol,
            RUN_ID,
        )
        host.observe(
            Telemetry(
                0,
                heartbeat=True,
                prearm_checks_healthy=True,
                armed=False,
                landed=True,
                mode="STABILIZE",
            )
        )

        def release(allowed: bool) -> None:
            host.tick(0, mission_running=allowed)

        def recover(reason: str) -> bool:
            recovery_calls.append(reason)
            host.observe(
                Telemetry(
                    1,
                    heartbeat=True,
                    prearm_checks_healthy=True,
                    armed=True,
                    landed=False,
                    mode="GUIDED",
                )
            )
            return host.abort_flight(reason)

        def verify() -> None:
            assert len(
                [value for value in protocol.statuses if isinstance(value, OperatorWaitStartedStatus)]
            ) == 1
            assert host._recovery_id == "2"

        return release, recover, verify, None

    class Mode:
        def __init__(self, name: str) -> None:
            self.name = name

    if mission in {"autotune_roll", "hover_roll"}:
        driver = (
            RollAutoTuneDriver(
                vehicle,
                run_directory=run_directory,
                run_id=RUN_ID,
                mode_factory=Mode,
            )
            if mission == "autotune_roll"
            else RollHoverDriver(
                vehicle,
                run_directory=run_directory,
                run_id=RUN_ID,
                mode_factory=Mode,
            )
        )
        observation_type = (
            AutoTuneObservation if mission == "autotune_roll" else HoverObservation
        )

        def release(allowed: bool) -> None:
            if allowed:
                driver.observe(
                    observation_type(
                        0,
                        heartbeat=True,
                        prearm_checks_healthy=True,
                        mode="STABILIZE",
                        armed=False,
                        landed=True,
                        relative_altitude_m=0.0,
                    )
                )

        def recover(reason: str) -> bool:
            recovery_calls.append(reason)
            vehicle.channels.overrides = {}
            vehicle.mode = Mode("LAND")
            return True

        def verify() -> None:
            assert vehicle.actions.count(("mode", "GUIDED")) == 1
            assert vehicle.actions.count(("mode", "LAND")) == 1

        return release, recover, verify, None

    if mission == "autotune":
        state = [calibration_autotune.AllAxisState.initial(public_deadline_ns=120_000_000_000)]
        actions: list[object] = []

        def release(allowed: bool) -> None:
            if not allowed:
                return
            transition = calibration_autotune.advance(
                state[0],
                calibration_autotune.Observation(
                    0,
                    heartbeat=True,
                    prearm_checks_healthy=True,
                    mode="STABILIZE",
                    armed=False,
                    landed=True,
                    relative_altitude_m=0.0,
                ),
            )
            state[0] = transition.state
            actions.extend(transition.actions)

        def recover(reason: str) -> bool:
            recovery_calls.append(reason)
            actions.append(calibration_autotune.Action.mode("LAND"))
            return True

        def verify() -> None:
            assert [a.name for a in actions if a.kind is calibration_autotune.ActionKind.SET_MODE].count("GUIDED") == 1
            assert [a.name for a in actions if a.kind is calibration_autotune.ActionKind.SET_MODE].count("LAND") == 1

        return release, recover, verify, None

    assert mission == "comp2026_auto"
    gate = Comp2026StartGate()
    worker_entries: list[str] = []
    initial_command_allowed = threading.Event()
    worker = threading.Thread(
        target=lambda: (
            initial_command_allowed.wait(),
            worker_entries.append("begin_attempt"),
            gate.wait_until_ready(),
            worker_entries.append("entered"),
        )
    )
    worker.start()
    delivered = [False]

    def release(allowed: bool) -> None:
        if not allowed or delivered[0]:
            return
        vehicle.mode = Mode("GUIDED")
        delivered[0] = True
        initial_command_allowed.set()
        gate.mark_process_ready()
        gate.accept_running()
        gate.accept_clock()
        gate.mark_command_delivered()
        gate.refresh_live_readiness(
            frame_ready=True,
            payload_service_ready=True,
            heartbeat_live=True,
            armable=True,
            range_is_current=lambda: True,
        )

    def recover(reason: str) -> bool:
        recovery_calls.append(reason)
        vehicle.mode = Mode("LAND")
        return True

    def verify() -> None:
        worker.join(timeout=1.0)
        assert worker_entries == ["begin_attempt", "entered"]
        assert vehicle.actions.count(("mode", "GUIDED")) == 1
        assert vehicle.actions.count(("mode", "LAND")) == 1

    return release, recover, verify, worker


@pytest.mark.parametrize(
    ("mission", "transport_kind"),
    (
        ("controlled_descent", "raw"),
        ("configured", "raw"),
        ("comp2026_auto", "dronekit"),
        ("autotune_roll", "dronekit"),
        ("autotune", "dronekit"),
        ("hover_roll", "dronekit"),
    ),
)
def test_actual_descriptor_host_releases_once_after_duplicate_adapter_ingress(
    mission: str,
    transport_kind: str,
    tmp_path: Path,
) -> None:
    from dronekit.mavlink import MAVConnection
    from drone_sim_companion.dronekit_sim import SimulationVehicle
    from drone_sim_companion.mavlink_adapter import MavlinkAdapter
    from drone_sim_companion import runtime_node
    from pymavlink import mavutil

    module = _transport_module()
    run_directory = tmp_path / mission / RUN_ID
    for relative in (".control", ".status", "configuration"):
        (run_directory / relative).mkdir(parents=True, exist_ok=True)
    descriptor = QgcOperatorDescriptor(
        QGC_ACTIONS_KIND,
        QGC_START_COMMAND,
        QGC_ABORT_COMMAND,
        17,
        "00000000-0000-4000-8000-000000000002",
        "00000000-0000-4000-8000-000000000003",
        MavlinkIdentity(255, 190),
        MavlinkIdentity(1, 191),
        MavlinkIdentity(1, 1),
        "configuration/qgc-actions.json",
        "b" * 64,
    )
    operator_protocol = OperatorProtocol(run_directory, RUN_ID, "a" * 64)
    operator_protocol.write_preflight(
        PreflightStatus(
            RUN_ID,
            "a" * 64,
            "gazebo",
            0,
            time.time_ns(),
            native_timestamp_ns=90_000_000_000,
            physics_advancing=True,
            camera_native_timestamp_ns=90_000_000_000,
            ready=True,
            detail="private native producers ready",
        )
    )
    protocol = _HostProtocol(operator_protocol)
    config = runtime_node.RuntimeConfig(
        run_id=RUN_ID,
        run_directory=run_directory,
        config_sha256="a" * 64,
        launch_mode="operator",
        mission=mission,
        operator_control=descriptor,
    )
    operator = runtime_node._operator_control(config, protocol)
    close_transport = lambda: None
    if transport_kind == "raw":
        connection = _RawConnection()
        io = module.PymavlinkQgcCommandIO(connection, mavlink2)
        fact_source = MavlinkAdapter(connection, mavutil, qgc_io=io)

        def ingress(message) -> None:
            io.observe_message(message)

        def observe(message) -> None:
            connection.incoming.append(message)
            fact_source.poll(90_000_000_000)

    else:
        module.configure_dronekit_mavlink2()
        connection = MAVConnection(
            "udpin:127.0.0.1:0",
            source_system=1,
            source_component=191,
        )
        vehicle = SimulationVehicle(connection)
        io = module.DroneKitQgcCommandIO(vehicle, mavlink2)
        fact_source = vehicle
        ingress = lambda message: vehicle.notify_message_listeners(
            "COMMAND_LONG", message
        )
        observe = lambda message: vehicle.notify_message_listeners(
            message.get_type(), message
        )
        def close_transport() -> None:
            connection.mavlink_thread_in = None
            connection.mavlink_thread_out = None

    control = runtime_node._compose_qgc_control(
        config,
        protocol,
        operator,
        io,
        readiness_reasons=lambda: (),
        is_finalizing=lambda: False,
    )
    release, recover, verify, worker = _host_release(
        mission,
        run_directory=run_directory,
        protocol=protocol,
    )
    fc = MavlinkIdentity(1, 1)
    ready_messages = (
        _parse(
            _frame(
                mavlink2.MAVLink_heartbeat_message(2, 3, 0, 0, 3, 3),
                source=fc,
            )
        ),
        _parse(
            _frame(
                mavlink2.MAVLink_sys_status_message(
                    0,
                    mavlink2.MAV_SYS_STATUS_PREARM_CHECK,
                    mavlink2.MAV_SYS_STATUS_PREARM_CHECK,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                ),
                source=fc,
            )
        ),
        _parse(
            _frame(
                mavlink2.MAVLink_extended_sys_state_message(
                    0, mavlink2.MAV_LANDED_STATE_ON_GROUND
                ),
                source=fc,
            )
        ),
    )
    for message in ready_messages:
        observe(message)
    assert runtime_node._observe_qgc_vehicle_facts(config, control, fact_source)
    start = _parse(
        _frame(
            mavlink2.MAVLink_command_long_message(
                1, 191, QGC_START_COMMAND, 0, 17, 0, 0, 0, 0, 0, 0
            ),
            source=MavlinkIdentity(255, 190),
        )
    )
    with ThreadPoolExecutor(max_workers=4) as pool:
        tuple(pool.map(ingress, (start,) * 8))
    assert control.poll_start(run_state=None).command_allowed is False
    request = operator_protocol.read_start_request()
    assert request is not None
    admission = operator_protocol.write_start_admission(
        request,
        native_epoch_ns=90_050_000_000,
    )
    running = RunningStateBinding(RUN_ID, "a" * 64, admission.sha256)
    first = control.poll_start(run_state=running)
    retry = control.poll_start(run_state=running)
    release(first.command_allowed)
    release(retry.command_allowed)

    armed = _parse(
        _frame(
            mavlink2.MAVLink_heartbeat_message(
                2,
                3,
                mavlink2.MAV_MODE_FLAG_SAFETY_ARMED,
                4,
                3,
                3,
            ),
            source=fc,
        )
    )
    airborne = _parse(
        _frame(
            mavlink2.MAVLink_extended_sys_state_message(
                0, mavlink2.MAV_LANDED_STATE_IN_AIR
            ),
            source=fc,
        )
    )
    observe(armed)
    observe(airborne)
    runtime_node._observe_qgc_vehicle_facts(config, control, fact_source)
    abort = _parse(
        _frame(
            mavlink2.MAVLink_command_long_message(
                1, 191, QGC_ABORT_COMMAND, 0, 17, 0, 0, 0, 0, 0, 0
            ),
            source=MavlinkIdentity(255, 190),
        )
    )
    with ThreadPoolExecutor(max_workers=2) as pool:
        tuple(pool.map(ingress, (abort, abort)))
    control.poll_abort(finalizing=False, begin_recovery=recover)
    control.poll_abort(finalizing=False, begin_recovery=recover)
    verify()
    assert operator_protocol.read_start_request() == request
    assert first.admitted_now is True
    assert retry.admitted_now is False
    assert worker is None or not worker.is_alive()
    operator_protocol.close()
    close_transport()
