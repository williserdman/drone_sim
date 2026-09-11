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
from drone_sim_companion.comp2026_host import (
    AttemptFailureCoordinator,
    RosLidar,
    SimulationClock,
    StaleSensorError,
)
from drone_sim_companion.lifecycle import CompanionLifecycle
from drone_sim_companion.mission import CommandKind, Telemetry


RUN_ID = "00000000-0000-4000-8000-000000000001"


def runtime_lidar(clock: SimulationClock) -> RosLidar:
    return RosLidar(
        clock,
        sample_factory=lambda distance, sampled_at, sequence, generation: SimpleNamespace(
            distance_m=distance,
            sampled_at=sampled_at,
            sequence=sequence,
            invalidation_generation=generation,
        ),
    )


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
) -> tuple[list[object], InitialCommandVehicle]:
    statuses: list[object] = []
    terminal = threading.Event()
    executor_stopped = threading.Event()
    subscriptions: dict[str, object] = {}
    vehicle = InitialCommandVehicle()
    vehicle.last_heartbeat = 0.0  # type: ignore[attr-defined]
    vehicle.is_armable = True  # type: ignore[attr-defined]
    vehicle.close = lambda: None  # type: ignore[attr-defined]

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
            subscriptions["/simulation/run_state"](
                SimpleNamespace(run_id=RUN_ID, state=RunState.RUNNING)
            )
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

    timebase = module("drone.timebase", configured=lambda _clock: nullcontext())
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


@pytest.mark.parametrize(
    "invalid_message",
    [
        SimpleNamespace(ranges=[4.25], range_min=0.1, range_max=30.0),
        SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=0)),
            range_min=0.1,
            range_max=30.0,
        ),
    ],
)
def test_comp2026_range_callback_invalidates_before_fatal_reporting(
    invalid_message: object,
) -> None:
    clock = SimulationClock()
    clock.accept(2_000_000_000)
    lidar = runtime_lidar(clock)
    valid_scan = SimpleNamespace(ranges=[4.25], range_min=0.1, range_max=30.0)
    lidar.accept(valid_scan, 1_750_000_000)
    assert lidar.get_sample().distance_m == pytest.approx(4.25)
    events: list[str] = []

    def write_failure(_reason: str) -> None:
        with pytest.raises(StaleSensorError, match="not ready"):
            lidar.get_sample()
        events.append("failure_recorded_after_invalidation")

    coordinator = AttemptFailureCoordinator(
        stop_attempt=lambda _reason: events.append("attempt_stopped"),
        write_failure=write_failure,
        recover=lambda: None,
    )

    assert not runtime_node.accept_comp2026_range_input(
        lidar=lidar,
        message=invalid_message,
        guard_input=coordinator.guard_input,
    )
    assert events == ["attempt_stopped", "failure_recorded_after_invalidation"]


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


def test_non_comp2026_runtime_rejects_qgc_configuration(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_resolved_config(run_directory)
    document = json.loads(config_path.read_text())
    document["qgc"] = {}
    config_path.write_text(json.dumps(document))

    with pytest.raises(ValueError, match="QGC.*comp2026_auto"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


def test_comp2026_runtime_config_requires_resolved_qgc_object(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    configuration = run_directory / "configuration"
    config_path = write_resolved_config(run_directory, mission="comp2026_auto")
    repository = Path(__file__).parents[2]
    (configuration / "course.yaml").write_bytes(
        (repository / "config/course.yaml").read_bytes()
    )
    (configuration / "scenario.yaml").write_bytes(
        (repository / "config/scenario.yaml").read_bytes()
    )
    document = json.loads(config_path.read_text(encoding="utf-8"))
    document["competition"] = {
        "course": "course.yaml",
        "scenario": "scenario.yaml",
        "course_sha256": hashlib.sha256(
            (configuration / "course.yaml").read_bytes()
        ).hexdigest(),
        "scenario_sha256": hashlib.sha256(
            (configuration / "scenario.yaml").read_bytes()
        ).hexdigest(),
    }
    config_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="QGC"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )


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
    configuration = run_directory / "configuration"
    config_path = write_resolved_config(run_directory, mission="comp2026_auto")
    repository = Path(__file__).parents[2]
    (configuration / "course.yaml").write_bytes(
        (repository / "config/course.yaml").read_bytes()
    )
    (configuration / "scenario.yaml").write_bytes(
        (repository / "config/scenario.yaml").read_bytes()
    )
    document = json.loads(config_path.read_text(encoding="utf-8"))
    document["competition"] = {
        "course": "course.yaml",
        "scenario": "scenario.yaml",
        "course_sha256": hashlib.sha256(
            (configuration / "course.yaml").read_bytes()
        ).hexdigest(),
        "scenario_sha256": hashlib.sha256(
            (configuration / "scenario.yaml").read_bytes()
        ).hexdigest(),
    }
    document["qgc"] = {
        "deployment_profile": "deployment-profile.json",
        "deployment_profile_sha256": "2" * 64,
        "listener_session": "listener-session.json",
        "listener_session_sha256": "3" * 64,
        "qgc_actions": "qgc-actions.json",
        "qgc_actions_sha256": "4" * 64,
        "runtime_policy": "qgc-runtime.json",
        "runtime_policy_sha256": "5" * 64,
        "attempt_state_id": "sha256-" + "2" * 64,
    }
    document["runtime_profile"] = "phase3"
    document["simulation"] = {
        "seed": 2026,
        "duration_sim_seconds": 600.0,
        "public_epoch_native_sim_seconds": 90.0,
        "target_real_time_factor": 0.25,
    }
    document["output_root"] = str(tmp_path / "outputs")
    config_path.write_text(json.dumps(document), encoding="utf-8")
    selected: list[str] = []
    monkeypatch.setattr(runtime_node, "resolved_qgc_inputs", lambda *_args, **_kwargs: None)
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
    calls: list[tuple[str, bool, float]] = []

    def connect(endpoint: str, *, wait_ready: bool, heartbeat_timeout: float) -> object:
        calls.append((endpoint, wait_ready, heartbeat_timeout))
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
    assert calls == [("tcp:ardupilot-sitl:5760", False, 120.0)]


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
