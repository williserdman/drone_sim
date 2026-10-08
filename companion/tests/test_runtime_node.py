from __future__ import annotations

import ast
from contextlib import nullcontext
import hashlib
import json
from io import StringIO
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace

import pytest

from artifacts.runtime_status import (
    MissionCommandDeliveredStatus,
    MissionExecutionReadyStatus,
    MissionReadyStatus,
    RuntimeFailureStatus,
)
import drone_sim_companion.runtime_node as runtime_node
from drone_sim_companion.runtime_node import (
    RuntimeConfig,
    autotune_control_timestamp_ns,
    connect_autotune_vehicle,
    connect_mavlink,
    quiesce_comp2026_runtime,
)
from drone_sim_companion.controller import MissionController
from drone_sim_companion.lifecycle import CompanionLifecycle
from drone_sim_companion.mission import CommandKind, Telemetry


RUN_ID = "00000000-0000-4000-8000-000000000001"


def test_all_axis_global_position_decoder_normalizes_return_telemetry() -> None:
    sample = runtime_node._decode_global_position(
        SimpleNamespace(
            vx=30,
            vy=40,
            vz=-12,
            relative_alt=5123,
            lat=374003371,
            lon=-1220800351,
        ),
        7_000_000_000,
    )

    assert sample == (
        7_000_000_000,
        0.5,
        0.12,
        5.123,
        37.4003371,
        -122.0800351,
    )


@pytest.mark.parametrize(
    ("field", "value"),
    [("lat", True), ("lat", 910_000_000), ("lon", float("nan")), ("lon", -1_810_000_000)],
)
def test_all_axis_global_position_decoder_rejects_invalid_coordinates(
    field: str, value: object
) -> None:
    message = SimpleNamespace(
        vx=0,
        vy=0,
        vz=0,
        relative_alt=0,
        lat=374003371,
        lon=-1220800351,
    )
    setattr(message, field, value)

    assert runtime_node._decode_global_position(message, 0) is None


def test_imported_calibration_readiness_blocks_until_complete_and_reports_once() -> None:
    trace: list[tuple[str, object]] = []
    protocol = SimpleNamespace(
        write_status=lambda status: trace.append(("status", status)),
    )
    lifecycle = SimpleNamespace(
        emit=lambda event, stamp, fields: trace.append(
            ("event", (event, stamp, fields))
        ),
    )
    readiness = runtime_node._CalibrationReadiness({
        "ATC_RAT_RLL_P": 0.041,
        "INS_GYRO_FILTER": 20.0,
    })

    readiness.observe_cached({"ATC_RAT_RLL_P": 0.041})
    assert not readiness.release(protocol, lifecycle, RUN_ID, 0)
    assert trace == []

    readiness.observe_cached({
        "ATC_RAT_RLL_P": 0.041,
        "INS_GYRO_FILTER": 20.0,
    })
    assert readiness.release(protocol, lifecycle, RUN_ID, 0)
    readiness.observe_cached({
        "ATC_RAT_RLL_P": 0.0675,
        "INS_GYRO_FILTER": 20.0,
    })
    assert readiness.release(protocol, lifecycle, RUN_ID, 1)
    assert readiness.failure is None
    assert trace == [
        (
            "event",
            (
                "calibration_parameters_verified",
                0,
                {
                    "stage": "pre_arm",
                    "parameters": {
                        "ATC_RAT_RLL_P": 0.041,
                        "INS_GYRO_FILTER": 20.0,
                    },
                },
            ),
        ),
        ("status", MissionExecutionReadyStatus(RUN_ID, 0)),
    ]


def test_imported_calibration_readiness_uses_nonblocking_dronekit_cache() -> None:
    class DroneKitParameters:
        def __init__(self) -> None:
            self.values = {"ATC_RAT_RLL_P": 0.041}

        def __getitem__(self, _name: str) -> float:
            raise AssertionError("parameter subscription must not block the runtime loop")

        def get(self, name: str, *, wait_ready: bool = True) -> float | None:
            assert wait_ready is False
            return self.values.get(name)

    parameters = DroneKitParameters()
    readiness = runtime_node._CalibrationReadiness({
        "ATC_RAT_RLL_P": 0.041,
        "INS_GYRO_FILTER": 20.0,
    })

    readiness.observe_cached(parameters)
    assert not readiness.gate.ready

    parameters.values["INS_GYRO_FILTER"] = 20.0
    readiness.observe_cached(parameters)
    assert readiness.gate.ready


def test_imported_calibration_readiness_exposes_terminal_mismatch() -> None:
    readiness = runtime_node._CalibrationReadiness({"ATC_RAT_RLL_P": 0.041})

    readiness.observe("ATC_RAT_RLL_P", 0.05)
    readiness.observe("ATC_RAT_RLL_P", 0.041)

    assert readiness.failure == (
        "effective required parameter ATC_RAT_RLL_P is 0.05, expected 0.041"
    )
    assert not readiness.release(object(), object(), RUN_ID, 0)


def test_roll_requests_partial_import_during_warmup_before_initial_command(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    statuses: list[object] = []
    subscriptions: dict[str, object] = {}
    message_listeners: dict[str, object] = {}
    added_listeners: list[str] = []
    removed_listeners: list[str] = []
    parameter_writes: list[tuple[str, float]] = []
    parameter_requests: list[tuple[int, int]] = []
    public_running = False

    class DroneKitParameters:
        def __init__(self) -> None:
            self.values = {"ATC_RAT_RLL_P": 0.041}

        def get(self, name: str, *, wait_ready: bool = True) -> float | None:
            assert wait_ready is False
            return self.values.get(name)

    parameters = DroneKitParameters()

    class Protocol:
        def __init__(self, _config: object) -> None:
            pass

        def write_status(self, status: object) -> None:
            statuses.append(status)

        def read_finalize_request(self) -> None:
            return None

        def write_quiescence(self, _module: str) -> None:
            pass

        def close(self) -> None:
            pass

    class Node:
        def __init__(self, _name: str) -> None:
            pass

        def create_subscription(
            self, _type: object, topic: str, callback: object, *_args: object, **_kwargs: object
        ) -> object:
            subscriptions[topic] = callback
            return object()

        def destroy_node(self) -> None:
            pass

    class Mode:
        def __init__(self, name: str) -> None:
            self.name = name

    class Mav:
        def param_request_list_send(self, system: int, component: int) -> None:
            assert public_running is False
            parameter_requests.append((system, component))
            parameters.values.update({
                "ATC_RAT_RLL_I": 0.041,
                "INS_GYRO_FILTER": 20.0,
            })

        def param_set_send(
            self,
            _system: int,
            _component: int,
            name: bytes,
            value: float,
            _parameter_type: int,
        ) -> None:
            parameter_writes.append((name.decode("ascii"), value))

    def add_message_listener(name: str, callback: object) -> None:
        added_listeners.append(name)
        message_listeners[name] = callback

    def remove_message_listener(name: str, callback: object) -> None:
        assert message_listeners[name] is callback
        removed_listeners.append(name)
        del message_listeners[name]

    vehicle = SimpleNamespace(
        last_heartbeat=0.0,
        is_armable=True,
        mode=Mode("STABILIZE"),
        armed=False,
        location=SimpleNamespace(global_relative_frame=SimpleNamespace(alt=0.0)),
        parameters=parameters,
        channels=SimpleNamespace(overrides={}),
        _master=SimpleNamespace(target_system=1, target_component=1, mav=Mav()),
        add_message_listener=add_message_listener,
        remove_message_listener=remove_message_listener,
        close=lambda: None,
        simple_takeoff=lambda _altitude: None,
    )
    spins = 0

    def spin_once(_node: object, *, timeout_sec: float) -> None:
        nonlocal public_running, spins
        assert timeout_sec == 0.02
        spins += 1
        if spins == 2:
            public_running = True
            subscriptions["/simulation/run_state"](
                SimpleNamespace(run_id=RUN_ID, state=1)
            )
            subscriptions["/clock"](
                SimpleNamespace(clock=SimpleNamespace(sec=0, nanosec=0))
            )
            message_listeners["GLOBAL_POSITION_INT"](
                vehicle,
                "GLOBAL_POSITION_INT",
                SimpleNamespace(
                    vx=0,
                    vy=0,
                    vz=0,
                    relative_alt=0,
                    lat=374_000_000,
                    lon=-1_220_800_000,
                ),
            )
            message_listeners["ATTITUDE"](
                vehicle, "ATTITUDE", SimpleNamespace(roll=0.01, pitch=-0.01)
            )
            for index, name in enumerate(
                (
                    "ATC_ANG_RLL_P",
                    "ATC_RAT_RLL_P",
                    "ATC_RAT_RLL_I",
                    "ATC_RAT_RLL_D",
                    "ATC_ACC_R_MAX",
                )
            ):
                message_listeners["PARAM_VALUE"](
                    vehicle,
                    "PARAM_VALUE",
                    SimpleNamespace(param_id=name, param_value=float(index + 1)),
                )
            message_listeners["COMMAND_ACK"](
                vehicle,
                "COMMAND_ACK",
                SimpleNamespace(
                    command=runtime_node.calibration_autotune.mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION,
                    result=runtime_node.calibration_autotune.mavutil.mavlink.MAV_RESULT_ACCEPTED,
                ),
            )

    def module(name: str, **members: object) -> None:
        value = ModuleType(name)
        for member_name, member in members.items():
            setattr(value, member_name, member)
        monkeypatch.setitem(sys.modules, name, value)

    module("dronekit", VehicleMode=Mode, connect=object())
    module("rclpy", init=lambda: None, ok=lambda: spins < 3, spin_once=spin_once, shutdown=lambda: None)
    module("rclpy.node", Node=Node)
    module(
        "rclpy.qos",
        DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1),
        QoSProfile=lambda **kwargs: kwargs,
        ReliabilityPolicy=SimpleNamespace(RELIABLE=1),
    )
    module("rosgraph_msgs.msg", Clock=object)
    module("simulation_interfaces.msg", RunState=SimpleNamespace(RUNNING=1, FINALIZING=2))
    for package in ("rosgraph_msgs", "simulation_interfaces"):
        module(package)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", Protocol)
    monkeypatch.setattr(runtime_node, "connect_autotune_vehicle", lambda *_args, **_kwargs: vehicle)
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="autotune_roll",
        calibration_json=json.dumps({
            "gains": {
                "ATC_RAT_RLL_P": 0.041,
                "ATC_RAT_RLL_I": 0.041,
            },
            "profile": {"baseline_parameters": {"INS_GYRO_FILTER": 20.0}},
        }),
    )

    assert runtime_node._run_autotune_roll(config) == 0

    events = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    named = [event for event in events if event["event"] in {
        "calibration_parameters_verified", "calibration_parameters_overridden"
    }]
    assert [event["event"] for event in named] == [
        "calibration_parameters_verified",
        "calibration_parameters_overridden",
    ]
    assert named[1]["fields"] == {
        "parameters": dict(parameter_writes),
        "reason": "roll diagnostic seed",
    }
    assert parameter_requests == [(1, 1)]
    assert statuses.index(MissionExecutionReadyStatus(RUN_ID, 0)) < statuses.index(
        MissionCommandDeliveredStatus(RUN_ID, 0)
    )
    assert added_listeners == [
        "STATUSTEXT",
        "PARAM_VALUE",
        "COMMAND_ACK",
        "GLOBAL_POSITION_INT",
        "ATTITUDE",
    ]
    assert removed_listeners == added_listeners
    assert message_listeners == {}


def test_autotune_can_deliver_first_command_at_public_zero_before_clock_ticks() -> None:
    assert autotune_control_timestamp_ns(
        mission_running=True, latest_clock_ns=None, first_command_pending=True
    ) == 0
    assert autotune_control_timestamp_ns(
        mission_running=True, latest_clock_ns=None, first_command_pending=False
    ) is None
    assert autotune_control_timestamp_ns(
        mission_running=True, latest_clock_ns=12, first_command_pending=False
    ) == 12
    assert autotune_control_timestamp_ns(
        mission_running=False, latest_clock_ns=12, first_command_pending=True
    ) is None


class InitialCommandVehicle:
    def __init__(self, *, mode_error: Exception | None = None) -> None:
        self.assigned_modes: list[object] = []
        self._mode_error = mode_error

    @property
    def mode(self) -> object | None:
        return self.assigned_modes[-1] if self.assigned_modes else None

    @mode.setter
    def mode(self, value: object) -> None:
        if self._mode_error is not None:
            raise self._mode_error
        self.assigned_modes.append(value)


class InitialCommandMode:
    def __init__(self, name: str) -> None:
        self.name = name


def ready_initial_command_gate() -> runtime_node.Comp2026StartGate:
    gate = runtime_node.Comp2026StartGate()
    gate.mark_process_ready()
    gate.accept_running()
    gate.accept_clock()
    gate.refresh_live_readiness(
        frame_ready=True,
        payload_service_ready=True,
        heartbeat_live=True,
        armable=True,
        range_is_current=lambda: True,
    )
    return gate


def initial_command_coordinator(failures: list[str]):
    return runtime_node.AttemptFailureCoordinator(
        stop_attempt=lambda _reason: None,
        write_failure=failures.append,
        recover=lambda: None,
    )


def test_late_comp2026_initial_command_fails_without_assigning_guided() -> None:
    vehicle = InitialCommandVehicle()
    failures: list[str] = []
    gate = ready_initial_command_gate()
    clock = runtime_node.SimulationClock()
    clock.accept(50_000_001)

    delivered = runtime_node._deliver_comp2026_initial_command(
        vehicle=vehicle,
        vehicle_mode_type=InitialCommandMode,
        lifecycle=object(),
        gate=gate,
        attempt_failure=initial_command_coordinator(failures),
        clock=clock,
        mark_delivered=lambda: None,
    )

    assert delivered is False
    assert vehicle.assigned_modes == []
    assert failures == ["initial GUIDED command missed the 50 ms delivery window"]
    assert gate.readiness["command_delivered"] is False


def test_comp2026_guided_mode_failure_keeps_delivery_gate_closed() -> None:
    vehicle = InitialCommandVehicle(mode_error=RuntimeError("mode rejected"))
    failures: list[str] = []
    gate = ready_initial_command_gate()
    clock = runtime_node.SimulationClock()
    clock.accept(50_000_000)

    delivered = runtime_node._deliver_comp2026_initial_command(
        vehicle=vehicle,
        vehicle_mode_type=InitialCommandMode,
        lifecycle=object(),
        gate=gate,
        attempt_failure=initial_command_coordinator(failures),
        clock=clock,
        mark_delivered=lambda: None,
    )

    assert delivered is False
    assert failures == ["initial GUIDED command failed: mode rejected"]
    assert gate.readiness["command_delivered"] is False


def test_comp2026_command_delivery_status_failure_keeps_gate_closed() -> None:
    class FailingProtocol:
        def write_status(self, _status: object) -> None:
            raise OSError("status disk full")

        def write_quiescence(self, _module: str) -> None:
            pass

    vehicle = InitialCommandVehicle()
    failures: list[str] = []
    gate = ready_initial_command_gate()
    lifecycle = CompanionLifecycle(
        run_id=RUN_ID,
        protocol=FailingProtocol(),
        stream=StringIO(),
    )
    clock = runtime_node.SimulationClock()
    clock.accept(50_000_000)

    delivered = runtime_node._deliver_comp2026_initial_command(
        vehicle=vehicle,
        vehicle_mode_type=InitialCommandMode,
        lifecycle=lifecycle,
        gate=gate,
        attempt_failure=initial_command_coordinator(failures),
        clock=clock,
        mark_delivered=lambda: None,
    )

    assert delivered is False
    assert [mode.name for mode in vehicle.assigned_modes] == ["GUIDED"]
    assert failures == ["initial GUIDED command failed: status disk full"]
    assert gate.readiness["command_delivered"] is False


def test_comp2026_worker_waits_for_durable_command_delivery_status() -> None:
    write_started = threading.Event()
    allow_write = threading.Event()
    worker_released = threading.Event()

    class BlockingProtocol:
        def write_status(self, status: object) -> None:
            assert status == MissionCommandDeliveredStatus(RUN_ID, 50_000_000)
            write_started.set()
            assert allow_write.wait(1.0)

        def write_quiescence(self, _module: str) -> None:
            pass

    gate = ready_initial_command_gate()
    lifecycle = CompanionLifecycle(
        run_id=RUN_ID,
        protocol=BlockingProtocol(),
        stream=StringIO(),
    )
    failures: list[str] = []
    vehicle = InitialCommandVehicle()
    clock = runtime_node.SimulationClock()
    clock.accept(50_000_000)
    local_latch: list[bool] = []
    worker = threading.Thread(
        target=lambda: (gate.wait_until_ready(), worker_released.set())
    )
    delivery = threading.Thread(
        target=lambda: runtime_node._deliver_comp2026_initial_command(
            vehicle=vehicle,
            vehicle_mode_type=InitialCommandMode,
            lifecycle=lifecycle,
            gate=gate,
            attempt_failure=initial_command_coordinator(failures),
            clock=clock,
            mark_delivered=lambda: local_latch.append(True),
        )
    )
    worker.start()
    delivery.start()
    try:
        assert write_started.wait(1.0)
        assert gate.readiness["command_delivered"] is False
        assert local_latch == []
        assert not worker_released.wait(0.05)
        allow_write.set()
        delivery.join(timeout=1.0)
        worker.join(timeout=1.0)
        assert worker_released.is_set()
    finally:
        allow_write.set()
        gate.stop("test cleanup")
        delivery.join(timeout=1.0)
        worker.join(timeout=1.0)

    assert failures == []
    assert gate.readiness["command_delivered"] is True
    assert local_latch == [True]


def test_comp2026_queued_clock_advance_wins_before_guided_transaction() -> None:
    clock = runtime_node.SimulationClock()
    clock.accept(50_000_000)
    failures: list[str] = []
    coordinator = initial_command_coordinator(failures)
    gate = ready_initial_command_gate()
    vehicle = InitialCommandVehicle()
    statuses: list[object] = []
    lifecycle = CompanionLifecycle(
        run_id=RUN_ID,
        protocol=SimpleNamespace(
            write_status=statuses.append,
            write_quiescence=lambda _module: None,
        ),
        stream=StringIO(),
    )
    local_latch: list[bool] = []
    update_started = threading.Event()
    update_finished = threading.Event()
    delivery_finished = threading.Event()

    def advance_clock() -> None:
        update_started.set()
        clock.accept(50_000_001)

    with clock._condition:
        update = threading.Thread(
            target=lambda: (
                coordinator.guard_input("clock", advance_clock),
                update_finished.set(),
            )
        )
        update.start()
        assert update_started.wait(1.0)
        delivery = threading.Thread(
            target=lambda: (
                runtime_node._deliver_comp2026_initial_command(
                    vehicle=vehicle,
                    vehicle_mode_type=InitialCommandMode,
                    lifecycle=lifecycle,
                    gate=gate,
                    attempt_failure=coordinator,
                    clock=clock,
                    mark_delivered=lambda: local_latch.append(True),
                ),
                delivery_finished.set(),
            )
        )
        delivery.start()
        assert not update_finished.is_set()
        assert not delivery_finished.is_set()

    update.join(timeout=1.0)
    delivery.join(timeout=1.0)

    assert update_finished.is_set()
    assert delivery_finished.is_set()
    assert vehicle.assigned_modes == []
    assert statuses == []
    assert gate.readiness["command_delivered"] is False
    assert local_latch == []
    assert failures == ["initial GUIDED command missed the 50 ms delivery window"]


def test_comp2026_concurrent_failure_prevents_guided_transaction() -> None:
    failure_rendering = threading.Event()
    allow_failure = threading.Event()
    failures: list[str] = []

    class PausingError(ValueError):
        def __str__(self) -> str:
            failure_rendering.set()
            assert allow_failure.wait(1.0)
            return "bad frame"

    coordinator = initial_command_coordinator(failures)
    failure_thread = threading.Thread(
        target=lambda: coordinator.guard_input(
            "image", lambda: (_ for _ in ()).throw(PausingError())
        )
    )
    failure_thread.start()
    assert failure_rendering.wait(1.0)

    clock = runtime_node.SimulationClock()
    clock.accept(50_000_000)
    vehicle = InitialCommandVehicle()
    statuses: list[object] = []
    gate = ready_initial_command_gate()
    local_latch: list[bool] = []
    delivery_result: list[bool] = []
    delivery_thread = threading.Thread(
        target=lambda: delivery_result.append(
            runtime_node._deliver_comp2026_initial_command(
                vehicle=vehicle,
                vehicle_mode_type=InitialCommandMode,
                lifecycle=CompanionLifecycle(
                    run_id=RUN_ID,
                    protocol=SimpleNamespace(
                        write_status=statuses.append,
                        write_quiescence=lambda _module: None,
                    ),
                    stream=StringIO(),
                ),
                gate=gate,
                attempt_failure=coordinator,
                clock=clock,
                mark_delivered=lambda: local_latch.append(True),
            )
        )
    )
    delivery_thread.start()
    try:
        assert delivery_thread.is_alive()
        allow_failure.set()
        failure_thread.join(timeout=1.0)
        delivery_thread.join(timeout=1.0)
    finally:
        allow_failure.set()
        failure_thread.join(timeout=1.0)
        delivery_thread.join(timeout=1.0)

    assert delivery_result == [False]
    assert vehicle.assigned_modes == []
    assert statuses == []
    assert gate.readiness["command_delivered"] is False
    assert local_latch == []
    assert failures == ["competition image input failed: bad frame"]


def install_comp2026_runtime_fakes(
    monkeypatch: pytest.MonkeyPatch,
    *,
    timestamp_ns: int,
    command_write_started: threading.Event,
    allow_command_write: threading.Event,
    mission_entered: threading.Event,
    input_failure_started: threading.Event | None = None,
    allow_input_failure: threading.Event | None = None,
    runtime_callbacks: dict[str, object] | None = None,
    installed_signal_handlers: dict[int, object] | None = None,
    ground_ready: threading.Event | None = None,
    readiness_checked: threading.Event | None = None,
    clock_allowed: threading.Event | None = None,
    running_delivered: threading.Event | None = None,
    prepare_delay_seconds: float = 0.0,
) -> tuple[list[object], InitialCommandVehicle]:
    statuses: list[object] = []
    terminal = threading.Event()
    executor_stopped = threading.Event()
    subscriptions: dict[str, object] = {}
    vehicle = InitialCommandVehicle()
    vehicle.last_heartbeat = 0.0  # type: ignore[attr-defined]
    vehicle.is_armable = True  # type: ignore[attr-defined]
    vehicle.close = lambda: None  # type: ignore[attr-defined]
    vehicle.guarded_bootstrap = []  # type: ignore[attr-defined]
    mission_ready_written = threading.Event()
    if ground_ready is None:
        ground_ready = threading.Event()
        ground_ready.set()

    def ns_stamp(value: int) -> object:
        return SimpleNamespace(
            sec=value // 1_000_000_000,
            nanosec=value % 1_000_000_000,
        )

    class Protocol:
        def __init__(self, _config: RuntimeConfig) -> None:
            pass

        def write_status(self, status: object) -> None:
            statuses.append(status)
            if isinstance(status, MissionReadyStatus):
                mission_ready_written.set()
            if isinstance(status, MissionCommandDeliveredStatus):
                command_write_started.set()
                assert allow_command_write.wait(1.0)
            if isinstance(status, (RuntimeFailureStatus, runtime_node.MissionFinishedStatus)):
                terminal.set()

        def read_finalize_request(self) -> object | None:
            return object() if terminal.is_set() else None

        def write_quiescence(self, _module: str) -> None:
            pass

        def close(self) -> None:
            pass

    class Node:
        def __init__(self, _name: str) -> None:
            pass

        def create_publisher(self, *_args: object, **_kwargs: object):
            return SimpleNamespace(publish=lambda _message: None)

        def create_client(self, *_args: object, **_kwargs: object):
            return SimpleNamespace(service_is_ready=lambda: True)

        def create_subscription(
            self, _type: object, topic: str, callback: object, *_args: object, **_kwargs: object
        ) -> object:
            subscriptions[topic] = callback
            if runtime_callbacks is not None:
                runtime_callbacks[topic] = callback
            return object()

        def destroy_subscription(self, _subscription: object) -> None:
            pass

        def destroy_node(self) -> None:
            pass

    class Executor:
        def __init__(self, *, num_threads: int) -> None:
            assert num_threads == 4

        def add_node(self, _node: object) -> None:
            pass

        def spin(self) -> None:
            while not mission_ready_written.wait(0.01):
                if executor_stopped.is_set():
                    return
            subscriptions["/simulation/run_state"](
                SimpleNamespace(run_id=RUN_ID, state=RunState.RUNNING)
            )
            if running_delivered is not None:
                running_delivered.set()
            if clock_allowed is not None:
                while not clock_allowed.wait(0.01):
                    if executor_stopped.is_set():
                        return
            subscriptions["/clock"](SimpleNamespace(clock=ns_stamp(timestamp_ns)))
            if input_failure_started is not None:
                assert allow_input_failure is not None

                class PausingInputError(ValueError):
                    def __str__(self) -> str:
                        input_failure_started.set()
                        assert allow_input_failure.wait(1.0)
                        return "bad image"

                class BadImage:
                    @property
                    def header(self) -> object:
                        raise PausingInputError

                subscriptions["/camera/onboard/image_raw"](BadImage())
                executor_stopped.wait()
                return
            subscriptions["/camera/onboard/image_raw"](
                SimpleNamespace(
                    header=SimpleNamespace(
                        stamp=ns_stamp(timestamp_ns), frame_id="camera/onboard"
                    ),
                    width=640,
                    height=480,
                    encoding="rgb8",
                    is_bigendian=False,
                    step=640 * 3,
                    data=b"",
                )
            )
            subscriptions["/competition/range/downward"](
                SimpleNamespace(
                    header=SimpleNamespace(stamp=ns_stamp(timestamp_ns)),
                    ranges=[4.572],
                    range_min=0.1,
                    range_max=30.0,
                )
            )
            executor_stopped.wait()

        def shutdown(self, *, timeout_sec: float) -> bool:
            assert timeout_sec == 1.0
            executor_stopped.set()
            return True

    class DroneControl:
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            self.vehicle = vehicle

        def get_current_gps(self) -> object:
            return SimpleNamespace(lat=1.0, long=2.0)

        def rtl(self) -> None:
            pass

        def simple_land(self) -> None:
            pass

        def disarm(self) -> None:
            pass

    class SimulationCompetitionControl:
        def __init__(self, **options):
            self.controller = DroneControl()
            self.controller.mission_home = SimpleNamespace(lat=1.0, lon=2.0, amsl_m=0.19)
            self.controller._guided_output_delivery_reported = False
            self.controller._transport_output_transaction = lambda output, before: (before(), output())[-1]
            self.controller.install_output_transactions = lambda **values: setattr(
                self.controller, "_transport_output_transaction", values["transport_transaction"]
            )
            self.decoders = SimpleNamespace(output_transaction=lambda operation: operation())
            self.supervisor = SimpleNamespace(output_transaction=lambda operation: operation())
            self.controller.set_guided_mode = self.set_guided_mode
            self.delivery = options["guided_output_delivery_callback"]
            vehicle.guarded_bootstrap.append("constructed")

        def prepare(self):
            if prepare_delay_seconds:
                threading.Event().wait(prepare_delay_seconds)
            vehicle.guarded_bootstrap.append("prepared")

        def ready_for_initial_command(self):
            if readiness_checked is not None:
                readiness_checked.set()
            return ground_ready.is_set()

        @property
        def ground_telemetry_pending_reasons(self):
            return () if ground_ready.is_set() else ("home absent",)

        def begin_attempt(self):
            vehicle.guarded_bootstrap.append("begin")

        def set_guided_mode(self):
            def output():
                vehicle.mode = InitialCommandMode("GUIDED")
                self.controller._guided_output_delivery_reported = True
                self.delivery()
            self.controller._transport_output_transaction(output, lambda: None)

        def phase_event(self, _phase, _state):
            pass

        def abort(self, _reason):
            pass

        def recover(self):
            pass

        def close(self):
            pass

    class RunState:
        RUNNING = 1
        FINALIZING = 2

    class MissionEvent:
        def __init__(self) -> None:
            self.sim_timestamp = SimpleNamespace(sec=0, nanosec=0)

    class PayloadCommand:
        class Request:
            pass

    def module(name: str, **members: object) -> ModuleType:
        result = ModuleType(name)
        for member_name, member in members.items():
            setattr(result, member_name, member)
        monkeypatch.setitem(sys.modules, name, result)
        return result

    timebase = module("drone.timebase", configured=lambda _clock: nullcontext(), monotonic=lambda: 0.0)
    drone = module("drone", timebase=timebase)
    del drone
    module(
        "drone.auto_attempt",
        run_auto_attempt=lambda **kwargs: (
            mission_entered.set(),
            kwargs["emit"]("HOME", "COMPLETE"),
        ),
    )
    module("drone.control.drone_control", DroneControl=DroneControl)
    module("drone.control.mission_info", MissonTracker=lambda _seconds: object())
    module("drone.sensors.camera._camera_manager", CameraManager=lambda **_kwargs: object())
    module("drone.sensors.camera.camera", Camera=lambda *_args, **_kwargs: object())
    module("dronekit", VehicleMode=InitialCommandMode)
    module("drone_sim_companion.comp2026_control", SimulationCompetitionControl=SimulationCompetitionControl)
    module("drone_sim_companion.comp2026_policy", build_competition_policy=lambda *_args, **_kwargs: SimpleNamespace(parameter_expectations={}, precision_policy=object()))
    rclpy = module(
        "rclpy",
        init=lambda: None,
        ok=lambda: True,
        shutdown=lambda: None,
    )
    del rclpy
    module("rclpy.callback_groups", MutuallyExclusiveCallbackGroup=lambda: object())
    module("rclpy.executors", MultiThreadedExecutor=Executor)
    module("rclpy.node", Node=Node)
    module(
        "rclpy.qos",
        DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2),
        QoSProfile=lambda **kwargs: kwargs,
        ReliabilityPolicy=SimpleNamespace(RELIABLE=1),
    )
    module("rosgraph_msgs.msg", Clock=object)
    module("sensor_msgs.msg", Image=object, LaserScan=object)
    module("simulation_interfaces.msg", MissionEvent=MissionEvent, RunState=RunState)
    module("simulation_interfaces.srv", PayloadCommand=PayloadCommand)
    for package in (
        "drone.control",
        "drone.sensors",
        "drone.sensors.camera",
        "rclpy",
        "rosgraph_msgs",
        "sensor_msgs",
        "simulation_interfaces",
    ):
        if package not in sys.modules:
            module(package)

    monkeypatch.setattr(runtime_node, "_ProductionProtocol", Protocol)
    monkeypatch.setattr(runtime_node, "load_course_waypoints", lambda *_args: {})
    def install_signal_handler(signum: int, handler: object) -> None:
        if installed_signal_handlers is not None:
            installed_signal_handlers[signum] = handler

    monkeypatch.setattr(runtime_node.signal, "signal", install_signal_handler)
    return statuses, vehicle


def test_run_comp2026_keeps_worker_blocked_until_command_status_is_durable(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    write_started = threading.Event()
    allow_write = threading.Event()
    mission_entered = threading.Event()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_000,
        command_write_started=write_started,
        allow_command_write=allow_write,
        mission_entered=mission_entered,
    )
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        max_wall_seconds=10,
        finalization_wall_seconds=1,
    )
    result: list[int] = []
    runtime = threading.Thread(target=lambda: result.append(runtime_node._run_comp2026(config)))
    runtime.start()
    try:
        assert write_started.wait(1.0)
        assert not mission_entered.wait(0.05)
        allow_write.set()
        runtime.join(timeout=2.0)
    finally:
        allow_write.set()
        runtime.join(timeout=2.0)

    assert result == [0]
    assert mission_entered.is_set()
    assert [mode.name for mode in vehicle.assigned_modes] == ["GUIDED"]
    assert MissionCommandDeliveredStatus(RUN_ID, 50_000_000) in statuses
    assert vehicle.guarded_bootstrap == ["constructed", "prepared", "begin"]


def test_run_comp2026_publishes_mission_ready_only_after_ground_and_calibration(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    ground_ready = threading.Event()
    readiness_checked = threading.Event()
    allow_write = threading.Event()
    allow_write.set()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_000,
        command_write_started=threading.Event(),
        allow_command_write=allow_write,
        mission_entered=threading.Event(),
        ground_ready=ground_ready,
        readiness_checked=readiness_checked,
    )
    vehicle.parameters = {"ATC_RAT_RLL_P": 0.041}
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        startup_timeout_seconds=1.0,
        max_wall_seconds=10,
        finalization_wall_seconds=1,
        calibration_json=json.dumps({
            "gains": {"ATC_RAT_RLL_P": 0.041},
            "profile": {"baseline_parameters": {}},
        }),
    )
    result: list[int] = []
    runtime = threading.Thread(
        target=lambda: result.append(runtime_node._run_comp2026(config))
    )
    runtime.start()
    try:
        assert readiness_checked.wait(1.0)
        assert not any(isinstance(status, MissionReadyStatus) for status in statuses)
        assert vehicle.assigned_modes == []
        ground_ready.set()
        runtime.join(timeout=2.0)
    finally:
        ground_ready.set()
        runtime.join(timeout=2.0)

    assert result == [0]
    ready_indexes = [
        index for index, status in enumerate(statuses)
        if isinstance(status, MissionReadyStatus)
    ]
    assert len(ready_indexes) == 1
    assert ready_indexes[0] < statuses.index(
        MissionExecutionReadyStatus(RUN_ID, 50_000_000)
    )
    assert [mode.name for mode in vehicle.assigned_modes] == ["GUIDED"]


@pytest.mark.parametrize(
    ("vehicle_field", "pending_value", "ready_value"),
    [
        ("is_armable", False, True),
        ("last_heartbeat", None, 0.0),
    ],
)
def test_run_comp2026_waits_for_observed_vehicle_readiness_before_mission_ready(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    vehicle_field: str,
    pending_value: object,
    ready_value: object,
) -> None:
    readiness_checked = threading.Event()
    allow_write = threading.Event()
    allow_write.set()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_000,
        command_write_started=threading.Event(),
        allow_command_write=allow_write,
        mission_entered=threading.Event(),
        readiness_checked=readiness_checked,
    )
    setattr(vehicle, vehicle_field, pending_value)
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        startup_timeout_seconds=1.0,
        max_wall_seconds=10,
        finalization_wall_seconds=1,
    )
    result: list[int] = []
    runtime = threading.Thread(
        target=lambda: result.append(runtime_node._run_comp2026(config))
    )
    runtime.start()
    try:
        assert readiness_checked.wait(1.0)
        assert not any(isinstance(status, MissionReadyStatus) for status in statuses)
        assert vehicle.assigned_modes == []
        setattr(vehicle, vehicle_field, ready_value)
        runtime.join(timeout=2.0)
    finally:
        setattr(vehicle, vehicle_field, ready_value)
        runtime.join(timeout=2.0)

    assert result == [0]
    assert [status for status in statuses if isinstance(status, MissionReadyStatus)] == [
        MissionReadyStatus(RUN_ID)
    ]


def test_run_comp2026_startup_deadline_includes_prepare(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    allow_write = threading.Event()
    allow_write.set()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_000,
        command_write_started=threading.Event(),
        allow_command_write=allow_write,
        mission_entered=threading.Event(),
        prepare_delay_seconds=0.06,
    )
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        startup_timeout_seconds=0.05,
        max_wall_seconds=10,
        finalization_wall_seconds=1,
    )

    assert runtime_node._run_comp2026(config) == 1

    assert vehicle.assigned_modes == []
    assert not any(isinstance(status, MissionReadyStatus) for status in statuses)
    failures = [status for status in statuses if isinstance(status, RuntimeFailureStatus)]
    assert [status.reason for status in failures] == [
        "competition startup readiness timed out: startup deadline expired"
    ]


def test_run_comp2026_bounds_missing_ground_telemetry_before_running(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    ground_ready = threading.Event()
    readiness_checked = threading.Event()
    allow_write = threading.Event()
    allow_write.set()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_000,
        command_write_started=threading.Event(),
        allow_command_write=allow_write,
        mission_entered=threading.Event(),
        ground_ready=ground_ready,
        readiness_checked=readiness_checked,
    )
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        startup_timeout_seconds=0.05,
        max_wall_seconds=10,
        finalization_wall_seconds=1,
    )

    assert runtime_node._run_comp2026(config) == 1

    assert readiness_checked.is_set()
    assert vehicle.assigned_modes == []
    assert not any(isinstance(status, MissionReadyStatus) for status in statuses)
    failures = [status for status in statuses if isinstance(status, RuntimeFailureStatus)]
    assert [status.reason for status in failures] == [
        "competition startup readiness timed out: "
        "safe-ground telemetry is absent or stale: home absent"
    ]
    events = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    pending = [event for event in events if event["event"] == "ground_telemetry_pending"]
    assert [event["fields"] for event in pending] == [{"reasons": ["home absent"]}]


def test_run_comp2026_waits_for_public_clock_before_initial_command(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock_allowed = threading.Event()
    running_delivered = threading.Event()
    allow_write = threading.Event()
    allow_write.set()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_000,
        command_write_started=threading.Event(),
        allow_command_write=allow_write,
        mission_entered=threading.Event(),
        clock_allowed=clock_allowed,
        running_delivered=running_delivered,
    )
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        startup_timeout_seconds=1.0,
        max_wall_seconds=10,
        finalization_wall_seconds=1,
    )
    result: list[int] = []
    runtime = threading.Thread(
        target=lambda: result.append(runtime_node._run_comp2026(config))
    )
    runtime.start()
    try:
        assert running_delivered.wait(1.0)
        assert MissionReadyStatus(RUN_ID) in statuses
        assert vehicle.assigned_modes == []
        clock_allowed.set()
        runtime.join(timeout=2.0)
    finally:
        clock_allowed.set()
        runtime.join(timeout=2.0)

    assert result == [0]
    assert [mode.name for mode in vehicle.assigned_modes] == ["GUIDED"]


def test_run_comp2026_verifies_imported_parameters_before_guided(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    allow_write = threading.Event()
    allow_write.set()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_000,
        command_write_started=threading.Event(),
        allow_command_write=allow_write,
        mission_entered=threading.Event(),
    )
    vehicle.parameters = {
        "ATC_RAT_RLL_P": 0.041,
        "INS_GYRO_FILTER": 20.0,
    }
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        max_wall_seconds=10,
        finalization_wall_seconds=1,
        calibration_json=json.dumps({
            "gains": {"ATC_RAT_RLL_P": 0.041},
            "profile": {"baseline_parameters": {"INS_GYRO_FILTER": 20.0}},
        }),
    )

    assert runtime_node._run_comp2026(config) == 0

    execution_ready = MissionExecutionReadyStatus(RUN_ID, 50_000_000)
    command_delivered = MissionCommandDeliveredStatus(RUN_ID, 50_000_000)
    assert statuses.index(execution_ready) < statuses.index(command_delivered)
    assert [mode.name for mode in vehicle.assigned_modes] == ["GUIDED"]
    events = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    verified = [
        event for event in events
        if event["event"] == "calibration_parameters_verified"
    ]
    assert [event["fields"] for event in verified] == [{
        "stage": "pre_arm",
        "parameters": {
            "ATC_RAT_RLL_P": 0.041,
            "INS_GYRO_FILTER": 20.0,
        },
    }]


def test_run_comp2026_calibration_mismatch_emits_zero_flight_commands(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    allow_write = threading.Event()
    allow_write.set()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_000,
        command_write_started=threading.Event(),
        allow_command_write=allow_write,
        mission_entered=threading.Event(),
    )
    vehicle.parameters = {"ATC_RAT_RLL_P": 0.05}
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        max_wall_seconds=10,
        finalization_wall_seconds=1,
        calibration_json=json.dumps({
            "gains": {"ATC_RAT_RLL_P": 0.041},
            "profile": {"baseline_parameters": {}},
        }),
    )

    assert runtime_node._run_comp2026(config) == 1
    assert vehicle.assigned_modes == []
    failures = [status for status in statuses if isinstance(status, RuntimeFailureStatus)]
    assert [status.reason for status in failures] == [
        "effective required parameter ATC_RAT_RLL_P is 0.05, expected 0.041"
    ]
    assert not any(isinstance(status, MissionReadyStatus) for status in statuses)
    assert not any(isinstance(status, MissionExecutionReadyStatus) for status in statuses)


def test_run_comp2026_rejects_late_clock_before_guided_or_status(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    allow_write = threading.Event()
    allow_write.set()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_001,
        command_write_started=threading.Event(),
        allow_command_write=allow_write,
        mission_entered=threading.Event(),
    )
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        max_wall_seconds=10,
        finalization_wall_seconds=1,
    )

    assert runtime_node._run_comp2026(config) == 1
    assert vehicle.assigned_modes == []
    assert not any(isinstance(status, MissionCommandDeliveredStatus) for status in statuses)
    failures = [status for status in statuses if isinstance(status, RuntimeFailureStatus)]
    assert [status.reason for status in failures] == [
        "initial GUIDED command missed the 50 ms delivery window"
    ]


def test_run_comp2026_concurrent_input_failure_prevents_guided_and_status(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    input_failure_started = threading.Event()
    allow_input_failure = threading.Event()
    allow_write = threading.Event()
    allow_write.set()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_000,
        command_write_started=threading.Event(),
        allow_command_write=allow_write,
        mission_entered=threading.Event(),
        input_failure_started=input_failure_started,
        allow_input_failure=allow_input_failure,
    )
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        max_wall_seconds=10,
        finalization_wall_seconds=1,
    )
    result: list[int] = []
    runtime = threading.Thread(target=lambda: result.append(runtime_node._run_comp2026(config)))
    runtime.start()
    try:
        assert input_failure_started.wait(1.0)
        assert vehicle.assigned_modes == []
        assert not any(
            isinstance(status, MissionCommandDeliveredStatus) for status in statuses
        )
        allow_input_failure.set()
        runtime.join(timeout=2.0)
    finally:
        allow_input_failure.set()
        runtime.join(timeout=2.0)

    assert result == [1]
    assert vehicle.assigned_modes == []
    assert not any(isinstance(status, MissionCommandDeliveredStatus) for status in statuses)
    failures = [status for status in statuses if isinstance(status, RuntimeFailureStatus)]
    assert [status.reason for status in failures] == [
        "competition image input failed: bad image"
    ]


@pytest.mark.parametrize("shutdown_source", ["finalizing", "signal"])
def test_run_comp2026_rechecks_shutdown_after_outer_loop_admission(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    shutdown_source: str,
) -> None:
    admitted = threading.Event()
    allow_delivery = threading.Event()
    callbacks: dict[str, object] = {}
    handlers: dict[int, object] = {}
    mission_entered = threading.Event()
    local_latch_marked = threading.Event()
    created_gates: list[runtime_node.Comp2026StartGate] = []
    allow_write = threading.Event()
    allow_write.set()
    statuses, vehicle = install_comp2026_runtime_fakes(
        monkeypatch,
        timestamp_ns=50_000_000,
        command_write_started=threading.Event(),
        allow_command_write=allow_write,
        mission_entered=mission_entered,
        runtime_callbacks=callbacks,
        installed_signal_handlers=handlers,
    )
    original_poll = runtime_node.comp2026_start_gate_poll_required
    pause_once = True

    def pause_after_admission(**options: object) -> bool:
        nonlocal pause_once
        result = original_poll(**options)  # type: ignore[arg-type]
        if result and pause_once:
            pause_once = False
            admitted.set()
            assert allow_delivery.wait(1.0)
        return result

    monkeypatch.setattr(
        runtime_node,
        "comp2026_start_gate_poll_required",
        pause_after_admission,
    )
    original_gate_type = runtime_node.Comp2026StartGate
    original_deliver = runtime_node._deliver_comp2026_initial_command

    def capture_gate() -> runtime_node.Comp2026StartGate:
        gate = original_gate_type()
        created_gates.append(gate)
        return gate

    def capture_local_latch(**options: object) -> bool:
        mark_delivered = options["mark_delivered"]

        def mark_and_record() -> None:
            local_latch_marked.set()
            mark_delivered()  # type: ignore[operator]

        options["mark_delivered"] = mark_and_record
        return original_deliver(**options)  # type: ignore[arg-type]

    monkeypatch.setattr(runtime_node, "Comp2026StartGate", capture_gate)
    monkeypatch.setattr(
        runtime_node,
        "_deliver_comp2026_initial_command",
        capture_local_latch,
    )
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        mission="comp2026_auto",
        course_path=tmp_path / "course.yaml",
        scenario_path=tmp_path / "scenario.yaml",
        max_wall_seconds=10,
        finalization_wall_seconds=1,
    )
    result: list[int] = []
    runtime = threading.Thread(
        target=lambda: result.append(runtime_node._run_comp2026(config))
    )
    runtime.start()
    try:
        assert admitted.wait(1.0)
        if shutdown_source == "finalizing":
            callbacks["/simulation/run_state"](
                SimpleNamespace(run_id=RUN_ID, state=2)
            )
        else:
            handlers[runtime_node.signal.SIGTERM](runtime_node.signal.SIGTERM, None)
        allow_delivery.set()
        runtime.join(timeout=2.0)
    finally:
        allow_delivery.set()
        runtime.join(timeout=2.0)

    assert result == [0]
    assert vehicle.assigned_modes == []
    assert not any(isinstance(status, MissionCommandDeliveredStatus) for status in statuses)
    assert not any(isinstance(status, RuntimeFailureStatus) for status in statuses)
    assert not mission_entered.is_set()
    assert created_gates[0].readiness["command_delivered"] is False
    assert not local_latch_marked.is_set()


def test_comp2026_polls_start_inputs_until_complete_gate_is_ready() -> None:
    assert runtime_node.comp2026_start_gate_poll_required(
        mission_running=True,
        mission_start_ready=False,
        failed=False,
    )
    assert not runtime_node.comp2026_start_gate_poll_required(
        mission_running=True,
        mission_start_ready=True,
        failed=False,
    )
    assert not runtime_node.comp2026_start_gate_poll_required(
        mission_running=True,
        mission_start_ready=False,
        failed=True,
    )
    assert not runtime_node.comp2026_start_gate_poll_required(
        mission_running=False,
        mission_start_ready=False,
        failed=False,
    )


def test_comp2026_stops_sensor_inputs_after_attempt_finishes() -> None:
    assert runtime_node.comp2026_sensor_inputs_required(
        mission_running=True,
        mission_worker_alive=True,
    )
    assert not runtime_node.comp2026_sensor_inputs_required(
        mission_running=True,
        mission_worker_alive=False,
    )
    assert runtime_node.comp2026_sensor_inputs_required(
        mission_running=False,
        mission_worker_alive=True,
    )


def write_resolved_config(
    run_directory: Path,
    *,
    run_id: str = RUN_ID,
    startup_wall_seconds: object = 120,
    max_wall_seconds: object = 3600,
    mission: str = "controlled_descent",
) -> Path:
    config_path = run_directory / "configuration/run.json"
    config_path.parent.mkdir(parents=True)
    document = {
        "run_id": run_id,
        "startup_wall_seconds": startup_wall_seconds,
        "max_wall_seconds": max_wall_seconds,
        "finalization_wall_seconds": 120,
        "mission": mission,
    }
    if mission == "comp2026_auto":
        course_payload = b"waypoints: {}\n"
        scenario_payload = b"payloads: []\n"
        (config_path.parent / "course.yaml").write_bytes(course_payload)
        (config_path.parent / "scenario.yaml").write_bytes(scenario_payload)
        document["competition"] = {
            "course": "course.yaml",
            "scenario": "scenario.yaml",
            "course_sha256": hashlib.sha256(course_payload).hexdigest(),
            "scenario_sha256": hashlib.sha256(scenario_payload).hexdigest(),
        }
    config_path.write_text(
        json.dumps(document),
        encoding="utf-8",
    )
    return config_path


def test_runtime_config_uses_resolved_run_startup_deadline(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory)

    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )

    assert config.mavlink_endpoint == "tcp:ardupilot-sitl:5760"
    assert config.startup_timeout_seconds == 120.0
    assert config.max_wall_seconds == 3600.0
    assert config.finalization_wall_seconds == 120.0
    assert config.mission == "controlled_descent"


def test_runtime_config_accepts_roll_autotune_without_competition_sources(
    tmp_path: Path,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="autotune_roll")

    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )

    assert config.mission == "autotune_roll"
    assert config.course_path is None
    assert config.scenario_path is None


def test_runtime_config_accepts_all_axis_autotune_and_public_deadline(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="autotune")
    document = json.loads(config_path.read_text())
    document["simulation"] = {
        "seed": 2026,
        "duration_sim_seconds": 600.0,
        "public_epoch_native_sim_seconds": 90.0,
        "target_real_time_factor": 0.25,
    }
    document["calibration_json"] = '{"schema_version":1}'
    config_path.write_text(json.dumps(document))

    config = RuntimeConfig.from_environment({
        "SIM_RUN_ID": RUN_ID,
        "SIM_RUN_DIRECTORY": str(run_directory),
        "SIM_CONFIG_PATH": str(config_path),
    })

    assert config.mission == "autotune"
    assert config.public_duration_ns == 600_000_000_000
    assert config.calibration_json == '{"schema_version":1}'


def test_runtime_config_accepts_roll_hover_without_competition_sources(
    tmp_path: Path,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="hover_roll")

    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )

    assert config.mission == "hover_roll"


def test_runtime_config_preserves_explicit_startup_timeout_override(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory)
    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
            "SIM_COMPANION_STARTUP_TIMEOUT_SECONDS": "17.5",
        }
    )

    assert config.startup_timeout_seconds == 17.5
    assert config.max_wall_seconds == 3600.0


def test_runtime_selects_original_competition_host_from_resolved_mission(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="comp2026_auto")
    selected: list[str] = []
    monkeypatch.setattr(runtime_node, "_run_controlled_descent", lambda _config: selected.append("controlled") or 0)
    monkeypatch.setattr(runtime_node, "_run_comp2026", lambda _config: selected.append("comp2026") or 0)
    monkeypatch.setattr(
        runtime_node.os,
        "environ",
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        },
    )

    assert runtime_node.main() == 0
    assert selected == ["comp2026"]


@pytest.mark.parametrize(
    ("field", "replacement"),
    [
        pytest.param("course_sha256", None, id="missing-course-digest"),
        pytest.param(
            "scenario_sha256",
            lambda value: value.upper(),
            id="uppercase-scenario-digest",
        ),
        pytest.param(
            "course_sha256",
            lambda value: value[:-1],
            id="wrong-length-course-digest",
        ),
        pytest.param(
            "scenario_sha256",
            lambda value: "g" + value[1:],
            id="nonhex-scenario-digest",
        ),
    ],
)
def test_runtime_config_rejects_malformed_competition_source_digest(
    tmp_path: Path,
    field: str,
    replacement: object,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="comp2026_auto")
    document = json.loads(config_path.read_text(encoding="utf-8"))
    if replacement is None:
        del document["competition"][field]
    else:
        document["competition"][field] = replacement(
            document["competition"][field]
        )
    config_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match=field):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


@pytest.mark.parametrize("source", ["course", "scenario"])
def test_runtime_config_rejects_competition_source_hash_mismatch(
    tmp_path: Path,
    source: str,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="comp2026_auto")
    (config_path.parent / f"{source}.yaml").write_bytes(b"changed after resolution\n")

    with pytest.raises(ValueError, match=source):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


def test_runtime_selects_roll_autotune_host(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="autotune_roll")
    selected: list[str] = []
    monkeypatch.setattr(
        runtime_node,
        "_run_controlled_descent",
        lambda _config: selected.append("controlled") or 0,
    )
    monkeypatch.setattr(
        runtime_node,
        "_run_autotune_roll",
        lambda _config: selected.append("autotune") or 0,
    )
    monkeypatch.setattr(
        runtime_node,
        "_run_comp2026",
        lambda _config: selected.append("comp2026") or 0,
    )
    monkeypatch.setattr(
        runtime_node.os,
        "environ",
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        },
    )

    assert runtime_node.main() == 0
    assert selected == ["autotune"]


def test_runtime_selects_all_axis_autotune_host(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory, mission="autotune")
    document = json.loads(config_path.read_text())
    document["simulation"] = {"duration_sim_seconds": 600.0}
    config_path.write_text(json.dumps(document))
    selected: list[str] = []
    monkeypatch.setattr(runtime_node, "_run_autotune", lambda _config: selected.append("all-axis") or 0)
    monkeypatch.setattr(runtime_node.os, "environ", {
        "SIM_RUN_ID": RUN_ID,
        "SIM_RUN_DIRECTORY": str(run_directory),
        "SIM_CONFIG_PATH": str(config_path),
    })

    assert runtime_node.main() == 0
    assert selected == ["all-axis"]


def test_first_heartbeat_ignores_startup_wall_deadline_after_transport_connects() -> None:
    assert (
        runtime_node.first_heartbeat_wall_failure(
            heartbeat_observed=False,
            wall_now=120.0,
            overall_wall_deadline=3600.0,
        )
        is None
    )
    assert runtime_node.first_heartbeat_wall_failure(
        heartbeat_observed=False,
        wall_now=3600.0,
        overall_wall_deadline=3600.0,
    ) == "MAVLink heartbeat was unavailable before the overall run wall failsafe"


def test_autotune_vehicle_connection_requires_run_state_subscription() -> None:
    calls: list[tuple[str, bool, float, type]] = []

    def connect(
        endpoint: str,
        *,
        wait_ready: bool,
        heartbeat_timeout: float,
        vehicle_class: type,
    ) -> object:
        calls.append((endpoint, wait_ready, heartbeat_timeout, vehicle_class))
        return object()

    with pytest.raises(RuntimeError, match="run-state subscription"):
        connect_autotune_vehicle(
            connect,
            "tcp:ardupilot-sitl:5760",
            heartbeat_timeout=120.0,
            run_state_subscription=None,
        )

    subscription = object()
    vehicle = connect_autotune_vehicle(
        connect,
        "tcp:ardupilot-sitl:5760",
        heartbeat_timeout=120.0,
        run_state_subscription=subscription,
    )

    assert vehicle is not None
    assert len(calls) == 1
    endpoint, wait_ready, timeout, vehicle_class = calls[0]
    assert (endpoint, wait_ready, timeout) == (
        "tcp:ardupilot-sitl:5760",
        False,
        120.0,
    )
    assert vehicle_class.__name__ == "SimulationVehicle"


def test_runtime_config_rejects_config_outside_current_run(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(tmp_path / "another-run")

    with pytest.raises(ValueError, match="run's resolved configuration"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


def test_runtime_config_rejects_stale_run_configuration(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(
        run_directory,
        run_id="00000000-0000-4000-8000-000000000002",
    )

    with pytest.raises(ValueError, match="run_id must match SIM_RUN_ID"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


@pytest.mark.parametrize("startup_wall_seconds", [True, 0, -1, 1.5, "120"])
def test_runtime_config_rejects_invalid_resolved_startup_deadline(
    tmp_path: Path,
    startup_wall_seconds: object,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(
        run_directory,
        startup_wall_seconds=startup_wall_seconds,
    )

    with pytest.raises(ValueError, match="startup_wall_seconds must be a positive integer"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


@pytest.mark.parametrize("max_wall_seconds", [True, 0, -1, 1.5, "3600"])
def test_runtime_config_rejects_invalid_overall_wall_failsafe(
    tmp_path: Path,
    max_wall_seconds: object,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(
        run_directory,
        max_wall_seconds=max_wall_seconds,
    )

    with pytest.raises(ValueError, match="max_wall_seconds must be a positive integer"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


@pytest.mark.parametrize(
    "environment",
    [
        {"SIM_RUN_ID": "bad", "SIM_RUN_DIRECTORY": "/runs/bad"},
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": "/runs/x",
            "SIM_COMPANION_STARTUP_TIMEOUT_SECONDS": "0",
        },
    ],
)
def test_runtime_config_rejects_invalid_infrastructure_configuration(
    environment: dict[str, str],
) -> None:
    with pytest.raises(ValueError):
        RuntimeConfig.from_environment(environment)


def test_runtime_has_no_gazebo_ground_truth_dependency() -> None:
    source = (
        Path(__file__).parents[1]
        / "src/drone_sim_companion/runtime_node.py"
    ).read_text(encoding="utf-8")

    assert "GroundTruth" not in source
    assert '"/simulation/ground_truth"' not in source
    assert "vertical_truth" not in source


def test_competition_runtime_defers_dronekit_readiness_to_its_live_gate() -> None:
    source = (
        Path(__file__).parents[1]
        / "src/drone_sim_companion/comp2026_control.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    constructor = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        and node.name == "SimulationCompetitionControl"
    )
    constructors = [
        node
        for node in ast.walk(constructor)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "controller_factory"
    ]

    assert len(constructors) == 1
    keywords = {
        keyword.arg: keyword.value
        for keyword in constructors[0].keywords
    }
    wait_ready = keywords["wait_ready"]
    assert isinstance(wait_ready, ast.Constant)
    assert wait_ready.value is False
    heartbeat_timeout = keywords["heartbeat_timeout"]
    assert isinstance(heartbeat_timeout, ast.Name)
    assert heartbeat_timeout.id == "heartbeat_timeout"


def test_competition_ros_callbacks_separate_ordered_control_from_camera_work() -> None:
    source = (
        Path(__file__).parents[1]
        / "src/drone_sim_companion/runtime_node.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    run_comp2026 = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_run_comp2026"
    )
    subscriptions = [
        node
        for node in ast.walk(run_comp2026)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_subscription"
    ]

    assert len(subscriptions) == 4
    expected_groups = {
        "state_callback": "clock_callback_group",
        "clock_callback": "clock_callback_group",
        "range_callback": "range_callback_group",
        "image_callback": "image_callback_group",
    }
    for subscription in subscriptions:
        callback = subscription.args[2]
        assert isinstance(callback, ast.Name)
        callback_group = next(
            (keyword.value for keyword in subscription.keywords if keyword.arg == "callback_group"),
            None,
        )
        assert isinstance(callback_group, ast.Name)
        assert callback_group.id == expected_groups[callback.id]

        if callback.id in {"clock_callback", "range_callback"}:
            qos_call = subscription.args[3]
            assert isinstance(qos_call, ast.Call)
            assert isinstance(qos_call.func, ast.Name)
            assert qos_call.func.id == "qos"
            assert isinstance(qos_call.args[0], ast.Constant)
            assert qos_call.args[0].value == 1

    executors = [
        node
        for node in ast.walk(run_comp2026)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "MultiThreadedExecutor"
    ]
    assert len(executors) == 1
    thread_count = next(
        keyword.value
        for keyword in executors[0].keywords
        if keyword.arg == "num_threads"
    )
    assert isinstance(thread_count, ast.Constant)
    assert thread_count.value == 4


def test_competition_runtime_emits_start_readiness_changes() -> None:
    source = (
        Path(__file__).parents[1]
        / "src/drone_sim_companion/runtime_node.py"
    ).read_text(encoding="utf-8")

    assert '"mission_start_readiness"' in source
    assert "gate.readiness" in source


def test_mavlink_connect_retries_only_within_wall_infrastructure_deadline() -> None:
    attempts = 0
    clock = iter((0.0, 0.2, 0.4))

    def factory(endpoint: str, **options: object) -> object:
        nonlocal attempts
        attempts += 1
        assert endpoint == "tcp:ardupilot-sitl:5760"
        assert options == {"autoreconnect": False, "source_system": 255}
        if attempts < 3:
            raise ConnectionRefusedError("not listening")
        return "connected"

    pauses: list[float] = []
    assert (
        connect_mavlink(
            factory,
            "tcp:ardupilot-sitl:5760",
            deadline=1.0,
            now=lambda: next(clock),
            pause=pauses.append,
        )
        == "connected"
    )
    assert pauses == [0.1, 0.1]


def test_runtime_wires_passive_facts_to_durable_mission_readiness() -> None:
    class Protocol:
        def __init__(self) -> None:
            self.statuses = []

        def write_status(self, status) -> None:
            self.statuses.append(status)

        def write_quiescence(self, _module: str) -> None:
            pass

    class Vehicle:
        def __init__(self) -> None:
            self.sent: list[tuple[CommandKind, float | None]] = []

        def send(self, command: CommandKind, altitude_m: float | None) -> None:
            self.sent.append((command, altitude_m))

    protocol = Protocol()
    lifecycle = CompanionLifecycle(run_id=RUN_ID, protocol=protocol, stream=StringIO())
    vehicle = Vehicle()
    controller = MissionController(vehicle, lifecycle.emit)

    runtime_node.process_runtime_telemetry(
        controller,
        lifecycle,
        Telemetry(10, prearm_checks_healthy=True),
        mission_running=False,
        public_clock_observed=False,
    )
    runtime_node.process_runtime_telemetry(
        controller,
        lifecycle,
        Telemetry(20, heartbeat=True, mode="STABILIZE", armed=False),
        mission_running=False,
        public_clock_observed=False,
    )

    assert protocol.statuses == [MissionReadyStatus(RUN_ID)]
    assert vehicle.sent == []


def test_controlled_descent_startup_failure_reaches_lifecycle(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    statuses = []
    quiescence: list[str] = []
    mavutil = ModuleType("mavutil")
    mavutil.mavlink_connection = object()  # type: ignore[attr-defined]

    class Protocol:
        def __init__(self, _config: RuntimeConfig) -> None:
            pass

        def write_status(self, status) -> None:
            statuses.append(status)

        def write_quiescence(self, module: str) -> None:
            quiescence.append(module)

        def close(self) -> None:
            pass

    dependency_members = {
        "pymavlink": {"mavutil": mavutil},
        "rclpy.node": {"Node": object},
        "rclpy.qos": {
            "DurabilityPolicy": object,
            "QoSProfile": object,
            "ReliabilityPolicy": object,
        },
        "rosgraph_msgs.msg": {"Clock": object},
        "simulation_interfaces.msg": {"RunState": object},
    }
    for name in ("rclpy", "rosgraph_msgs", "simulation_interfaces"):
        monkeypatch.setitem(sys.modules, name, ModuleType(name))
    for name, members in dependency_members.items():
        module = ModuleType(name)
        for member_name, member in members.items():
            setattr(module, member_name, member)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(runtime_node, "_ProductionProtocol", Protocol)
    monkeypatch.setattr(
        runtime_node,
        "connect_mavlink",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            TimeoutError("MAVLink connection deadline expired")
        ),
    )
    config = RuntimeConfig(run_id=RUN_ID, run_directory=tmp_path / RUN_ID)

    assert runtime_node._run_controlled_descent(config) == 1
    assert statuses == [
        RuntimeFailureStatus(
            RUN_ID,
            "companion",
            "MAVLink connection deadline expired",
            ("logs/docker/companion.log.partial",),
        )
    ]
    assert quiescence == ["companion"]


def test_initial_command_delivery_is_durable_and_exactly_at_public_zero() -> None:
    class Protocol:
        def __init__(self) -> None:
            self.statuses = []

        def write_status(self, status) -> None:
            self.statuses.append(status)

        def write_quiescence(self, _module: str) -> None:
            pass

    protocol = Protocol()
    lifecycle = CompanionLifecycle(run_id=RUN_ID, protocol=protocol, stream=StringIO())

    lifecycle.observe_command_delivery(CommandKind.SET_GUIDED, 0)

    assert protocol.statuses == [MissionCommandDeliveredStatus(RUN_ID, 0)]


def test_comp2026_quiescence_follows_terminated_worker_and_executor() -> None:
    events: list[str] = []
    stop_worker = threading.Event()
    stop_executor = threading.Event()

    def worker_target() -> None:
        stop_worker.wait()
        events.append("worker_stopped")

    def executor_target() -> None:
        stop_executor.wait()
        events.append("executor_stopped")

    worker = threading.Thread(target=worker_target)
    executor_thread = threading.Thread(target=executor_target)
    worker.start()
    executor_thread.start()

    class Executor:
        def shutdown(self, *, timeout_sec: float) -> bool:
            assert timeout_sec == 1.0
            events.append("stop_executor")
            stop_executor.set()
            return True

    def stop_attempt(reason: str) -> None:
        assert reason == "finalization"
        events.append("stop_attempt")
        stop_worker.set()

    def finalize() -> None:
        assert not worker.is_alive()
        assert not executor_thread.is_alive()
        events.append("quiescence")

    failures: list[str] = []
    assert quiesce_comp2026_runtime(
        stop_attempt=stop_attempt,
        mission_worker=worker,
        executor=Executor(),
        executor_thread=executor_thread,
        close_output_producers=lambda: events.append("producers_closed"),
        finalize=finalize,
        write_failure=failures.append,
        timeout_seconds=1.0,
    ) is True

    assert failures == []
    assert events.index("worker_stopped") < events.index("stop_executor")
    assert events.index("executor_stopped") < events.index("producers_closed")
    assert events[-1] == "quiescence"


def test_comp2026_never_writes_quiescence_with_a_surviving_worker() -> None:
    events: list[str] = []

    class StuckWorker:
        def join(self, timeout: float) -> None:
            assert timeout == 1.0
            events.append("worker_join_timed_out")

        def is_alive(self) -> bool:
            return True

    class StoppedExecutorThread:
        def join(self, timeout: float) -> None:
            assert timeout == 1.0
            events.append("executor_joined")

        def is_alive(self) -> bool:
            return False

    class Executor:
        def shutdown(self, *, timeout_sec: float) -> bool:
            assert timeout_sec == 1.0
            events.append("executor_shutdown")
            return True

    failures: list[str] = []
    assert quiesce_comp2026_runtime(
        stop_attempt=lambda _reason: events.append("stop_attempt"),
        mission_worker=StuckWorker(),
        executor=Executor(),
        executor_thread=StoppedExecutorThread(),
        close_output_producers=lambda: events.append("producers_closed"),
        finalize=lambda: events.append("quiescence"),
        write_failure=failures.append,
        timeout_seconds=1.0,
    ) is False

    assert failures == ["original mission worker did not stop for finalization"]
    assert "quiescence" not in events
