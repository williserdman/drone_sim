"""Select and host the resolved controlled-descent or Comp2026 mission."""

from __future__ import annotations

from dataclasses import dataclass
import collections
import collections.abc
import inspect
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
from collections.abc import Callable
from typing import Any, Mapping
from uuid import UUID

from artifacts.runtime_protocol import RuntimeProtocol

from .controller import MissionController, mission_policy_active, process_telemetry
from .lifecycle import CompanionLifecycle
from .mavlink_adapter import MavlinkAdapter
from .mission import CommandKind, MissionPhase, MissionState, Telemetry
from .comp2026_host import (
    AttemptFailureCoordinator,
    Comp2026StartGate,
    GPSCoord,
    MissionEventEmitter,
    MissionEventRecord,
    PayloadDropper,
    PayloadRequest,
    RosFrameSource,
    RosLidar,
    SimulationClock,
    load_course_waypoints,
    refresh_comp2026_start_gate,
)


@dataclass(frozen=True)
class RuntimeConfig:
    run_id: str
    run_directory: Path
    mission: str = "controlled_descent"
    mavlink_endpoint: str = "tcp:ardupilot-sitl:5760"
    startup_timeout_seconds: float = 60.0
    max_wall_seconds: float = 3600.0
    finalization_wall_seconds: float = 120.0
    course_path: Path | None = None
    scenario_path: Path | None = None

    @classmethod
    def from_environment(cls, environment: Mapping[str, str]) -> "RuntimeConfig":
        run_id = environment["SIM_RUN_ID"]
        try:
            parsed = UUID(run_id)
        except (ValueError, TypeError, AttributeError) as error:
            raise ValueError("SIM_RUN_ID must be a canonical UUID") from error
        if str(parsed) != run_id:
            raise ValueError("SIM_RUN_ID must be a canonical UUID")
        run_directory = Path(environment["SIM_RUN_DIRECTORY"])
        timeout_override = environment.get("SIM_COMPANION_STARTUP_TIMEOUT_SECONDS")
        if timeout_override is not None:
            override = float(timeout_override)
            if not math.isfinite(override) or override <= 0:
                raise ValueError("SIM_COMPANION_STARTUP_TIMEOUT_SECONDS must be positive")
        config_path = Path(environment["SIM_CONFIG_PATH"])
        if not run_directory.is_absolute() or not config_path.is_absolute():
            raise ValueError("run and config paths must be absolute")
        if config_path != run_directory / "configuration/run.json":
            raise ValueError("SIM_CONFIG_PATH must be the run's resolved configuration")
        try:
            document = json.loads(config_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ValueError("SIM_CONFIG_PATH must contain readable JSON") from error
        if not isinstance(document, dict):
            raise ValueError("SIM_CONFIG_PATH must contain a JSON object")
        if document.get("run_id") != run_id:
            raise ValueError("resolved configuration run_id must match SIM_RUN_ID")
        startup_wall_seconds = document.get("startup_wall_seconds")
        if (
            isinstance(startup_wall_seconds, bool)
            or not isinstance(startup_wall_seconds, int)
            or startup_wall_seconds <= 0
        ):
            raise ValueError("startup_wall_seconds must be a positive integer")
        max_wall_seconds = document.get("max_wall_seconds")
        if (
            isinstance(max_wall_seconds, bool)
            or not isinstance(max_wall_seconds, int)
            or max_wall_seconds <= 0
        ):
            raise ValueError("max_wall_seconds must be a positive integer")
        finalization_wall_seconds = document.get("finalization_wall_seconds")
        if (
            isinstance(finalization_wall_seconds, bool)
            or not isinstance(finalization_wall_seconds, int)
            or finalization_wall_seconds <= 0
        ):
            raise ValueError("finalization_wall_seconds must be a positive integer")
        mission = document.get("mission")
        if mission not in {"controlled_descent", "comp2026_auto"}:
            raise ValueError("resolved mission must select an approved companion host")
        course_path: Path | None = None
        scenario_path: Path | None = None
        if mission == "comp2026_auto":
            competition = document.get("competition")
            if (
                not isinstance(competition, dict)
                or competition.get("course") != "course.yaml"
                or competition.get("scenario") != "scenario.yaml"
            ):
                raise ValueError("comp2026_auto requires resolved competition sources")
            course_path = config_path.parent / "course.yaml"
            scenario_path = config_path.parent / "scenario.yaml"
            if not course_path.is_file() or not scenario_path.is_file():
                raise ValueError("resolved competition sources are unreadable")
        timeout = override if timeout_override is not None else float(startup_wall_seconds)
        endpoint = environment.get("SIM_MAVLINK_ENDPOINT", "tcp:ardupilot-sitl:5760")
        if endpoint != "tcp:ardupilot-sitl:5760":
            raise ValueError("production MAVLink endpoint must be tcp:ardupilot-sitl:5760")
        return cls(
            run_id=run_id,
            run_directory=run_directory,
            mission=mission,
            mavlink_endpoint=endpoint,
            startup_timeout_seconds=timeout,
            max_wall_seconds=float(max_wall_seconds),
            finalization_wall_seconds=float(finalization_wall_seconds),
            course_path=course_path,
            scenario_path=scenario_path,
        )


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def connect_mavlink(
    factory: Callable[..., Any],
    endpoint: str,
    *,
    deadline: float,
    now: Callable[[], float] = time.monotonic,
    pause: Callable[[float], None] = time.sleep,
) -> Any:
    """Bound retries for unavailable TCP infrastructure using wall time only."""
    last_error: OSError | None = None
    while True:
        try:
            return factory(endpoint, autoreconnect=False, source_system=255)
        except OSError as error:
            last_error = error
        if now() >= deadline:
            raise TimeoutError(f"MAVLink endpoint was unavailable: {last_error}") from last_error
        pause(0.1)


def first_heartbeat_wall_failure(
    *,
    heartbeat_observed: bool,
    wall_now: float,
    overall_wall_deadline: float,
) -> str | None:
    """Keep simulated boot independent of wall speed, with one run-level failsafe."""
    if heartbeat_observed or wall_now < overall_wall_deadline:
        return None
    return "MAVLink heartbeat was unavailable before the overall run wall failsafe"


class _ProductionProtocol:
    """Restrict companion-owned writes through the shared runtime protocol."""

    def __init__(self, config: RuntimeConfig) -> None:
        self._runtime = RuntimeProtocol(config.run_directory, config.run_id)

    def write_status(self, name: str, document: dict[str, object]) -> None:
        if name not in {
            "companion-ready",
            "mission-ready",
            "mission-command-delivered",
            "mission-finished",
            "runtime-failure",
        }:
            raise ValueError("companion does not own that status")
        self._runtime.write_status(name, document)

    def write_quiescence(self, module: str) -> Any:
        return self._runtime.write_quiescence(module)

    def read_finalize_request(self) -> dict[str, Any] | None:
        return self._runtime.read_finalize_request()

    def close(self) -> None:
        self._runtime.close()


def process_runtime_telemetry(
    controller: MissionController,
    lifecycle: CompanionLifecycle,
    telemetry: Telemetry,
    *,
    mission_running: bool,
    public_clock_observed: bool,
) -> None:
    process_telemetry(
        controller,
        telemetry,
        mission_running=mission_running,
        public_clock_observed=public_clock_observed,
    )
    lifecycle.observe_mission_readiness(
        heartbeat_observed=controller.heartbeat_observed,
        prearm_checks_healthy=controller.prearm_checks_healthy,
    )


def quiesce_comp2026_runtime(
    *,
    stop_attempt,
    mission_worker,
    executor,
    executor_thread,
    close_output_producers,
    finalize,
    write_failure,
    timeout_seconds: float,
) -> bool:
    """Stop every competition output producer before durable quiescence."""

    failure: str | None = None
    stop_attempt("finalization")
    if mission_worker is not None:
        mission_worker.join(timeout=timeout_seconds)
        if mission_worker.is_alive():
            failure = "original mission worker did not stop for finalization"
    try:
        executor_stopped = executor.shutdown(timeout_sec=timeout_seconds)
    except Exception as error:
        executor_stopped = False
        if failure is None:
            failure = f"ROS executor shutdown failed: {error}"
    executor_thread.join(timeout=timeout_seconds)
    if executor_stopped is False or executor_thread.is_alive():
        if failure is None:
            failure = "ROS executor did not stop for finalization"
    close_output_producers()
    if failure is not None:
        write_failure(failure)
        return False
    finalize()
    return True


def _enable_dronekit_python312_compatibility() -> None:
    """Install only the aliases DroneKit 2.9.2 still imports on Python 3.12."""

    if not hasattr(collections, "MutableMapping"):
        collections.MutableMapping = collections.abc.MutableMapping  # type: ignore[attr-defined]
    if not hasattr(inspect, "getargspec"):
        inspect.getargspec = inspect.getfullargspec  # type: ignore[attr-defined]


class _RosPayloadClient:
    """Block a mission worker while the one responsive executor resolves ROS."""

    def __init__(self, client: Any, service_type: Any) -> None:
        self._client = client
        self._service_type = service_type
        self._stopped = threading.Event()

    def service_is_ready(self) -> bool:
        return bool(self._client.service_is_ready())

    def call(self, request: PayloadRequest) -> Any:
        if self._stopped.is_set():
            raise RuntimeError("payload client stopped")
        ros_request = self._service_type.Request()
        ros_request.run_id = request.run_id
        ros_request.aruco_id = request.aruco_id
        ros_request.action = request.action
        ros_request.command_id = request.command_id
        future = self._client.call_async(ros_request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        while not completed.wait(0.05):
            if self._stopped.is_set():
                future.cancel()
                raise RuntimeError("payload client stopped during confirmation")
        return future.result()

    def stop(self) -> None:
        self._stopped.set()


def _run_controlled_descent(config: RuntimeConfig) -> int:
    protocol = _ProductionProtocol(config)
    lifecycle = CompanionLifecycle(run_id=config.run_id, protocol=protocol, stream=sys.stdout)
    lifecycle.emit("starting", None, {"mavlink_endpoint": config.mavlink_endpoint})

    import rclpy
    from pymavlink import mavutil
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import RunState

    started = time.monotonic()
    overall_wall_deadline = started + config.max_wall_seconds
    try:
        connection = connect_mavlink(
            mavutil.mavlink_connection,
            config.mavlink_endpoint,
            deadline=min(
                started + config.startup_timeout_seconds,
                overall_wall_deadline,
            ),
        )
    except TimeoutError as error:
        lifecycle.observe_terminal(
            MissionState(MissionPhase.FAILED, last_timestamp_ns=0, failure_reason=str(error))
        )
        lifecycle.finalize(None)
        protocol.close()
        return 1
    lifecycle.mark_transport_ready()
    vehicle = MavlinkAdapter(connection, mavutil)
    controller = MissionController(
        vehicle,
        lifecycle.emit,
        command_delivered=lifecycle.observe_command_delivery,
    )
    rclpy.init()
    node = Node("drone_sim_companion", parameter_overrides=[])
    latest_clock_ns: int | None = None
    mission_running = False
    finalizing = False
    requested_stop = False
    failure: str | None = None

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal requested_stop
        requested_stop = True

    def clock_callback(message: Any) -> None:
        nonlocal latest_clock_ns, failure
        value = stamp_ns(message.clock)
        if latest_clock_ns is not None and value < latest_clock_ns:
            failure = "authoritative simulation clock regressed"
            return
        latest_clock_ns = value

    def state_callback(message: Any) -> None:
        nonlocal finalizing, mission_running
        if message.run_id != config.run_id:
            return
        if message.state == RunState.RUNNING:
            mission_running = True
        elif message.state == RunState.FINALIZING:
            finalizing = True

    node.create_subscription(
        Clock,
        "/clock",
        clock_callback,
        QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE),
    )
    node.create_subscription(
        RunState,
        "/simulation/run_state",
        state_callback,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    telemetry_requested = False
    exit_code = 0
    try:
        while rclpy.ok() and not requested_stop and not finalizing:
            rclpy.spin_once(node, timeout_sec=0.02)
            policy_active = mission_policy_active(
                mission_running=mission_running,
                public_clock_observed=latest_clock_ns is not None,
            )
            if (
                policy_active
                and latest_clock_ns == 0
                and controller.mission_ready
                and controller.state.phase is MissionPhase.WAIT_HEARTBEAT
                and failure is None
            ):
                try:
                    controller.begin_mission(0)
                except Exception as error:
                    failure = f"initial MAVLink command delivery failed: {error}"
            if failure is None:
                try:
                    for _ in range(100):
                        telemetry = vehicle.poll(
                            latest_clock_ns if latest_clock_ns is not None else 0
                        )
                        if telemetry is None:
                            break
                        process_runtime_telemetry(
                            controller,
                            lifecycle,
                            telemetry,
                            mission_running=mission_running,
                            public_clock_observed=latest_clock_ns is not None,
                        )
                except Exception as error:
                    failure = f"MAVLink processing failed: {error}"
            if (
                policy_active
                and controller.heartbeat_observed
                and not telemetry_requested
                and failure is None
            ):
                vehicle.request_telemetry(rate_hz=10)
                lifecycle.emit("telemetry_requested", latest_clock_ns, {"rate_hz": 10})
                telemetry_requested = True
            lifecycle.observe_terminal(controller.state)
            if controller.state.phase is MissionPhase.FAILED:
                failure = controller.state.failure_reason
            if failure is not None:
                lifecycle.observe_terminal(
                    MissionState(
                        MissionPhase.FAILED,
                        last_timestamp_ns=latest_clock_ns or 0,
                        failure_reason=failure,
                    )
                )
                exit_code = 1
                break
            heartbeat_failure = first_heartbeat_wall_failure(
                heartbeat_observed=controller.heartbeat_observed,
                wall_now=time.monotonic(),
                overall_wall_deadline=overall_wall_deadline,
            )
            if heartbeat_failure is not None:
                failure = heartbeat_failure
                lifecycle.observe_terminal(
                    MissionState(
                        MissionPhase.FAILED,
                        last_timestamp_ns=latest_clock_ns or 0,
                        failure_reason=failure,
                    )
                )
                exit_code = 1
                break
            if protocol.read_finalize_request() is not None:
                finalizing = True
    finally:
        lifecycle.finalize(latest_clock_ns)
        node.destroy_node()
        connection.close()
        protocol.close()
        rclpy.shutdown()
    return exit_code


def _create_simulator_camera(
    camera_manager_type: Any,
    camera_type: Any,
    frame_source: Any,
) -> Any:
    calibration_path = Path(__file__).with_name("gazebo_camera_calibration.json")
    manager = camera_manager_type(
        frame_source=frame_source,
        calibration_path=calibration_path,
    )
    return camera_type(100, manager=manager)


def _run_comp2026(config: RuntimeConfig) -> int:
    """Host one original nested attempt behind current ROS/lifecycle seams."""

    if config.course_path is None or config.scenario_path is None:
        raise ValueError("competition runtime requires resolved course and scenario")

    protocol = _ProductionProtocol(config)
    lifecycle = CompanionLifecycle(run_id=config.run_id, protocol=protocol, stream=sys.stdout)
    lifecycle.emit("starting", None, {"mavlink_endpoint": config.mavlink_endpoint})
    _enable_dronekit_python312_compatibility()

    import rclpy
    from drone.auto_attempt import run_auto_attempt
    from drone import timebase
    from drone.control.drone_control import DroneControl
    from drone.control.mission_info import MissonTracker
    from drone.sensors.camera._camera_manager import CameraManager
    from drone.sensors.camera.camera import Camera
    from dronekit import VehicleMode
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, LaserScan
    from simulation_interfaces.msg import FrameMetadata, MissionEvent, RunState
    from simulation_interfaces.srv import PayloadCommand

    def qos(depth: int, *, transient: bool = False) -> Any:
        return QoSProfile(
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if transient
                else DurabilityPolicy.VOLATILE
            ),
        )

    rclpy.init()
    node = Node("drone_sim_companion")
    clock_callback_group = MutuallyExclusiveCallbackGroup()
    range_callback_group = MutuallyExclusiveCallbackGroup()
    image_callback_group = MutuallyExclusiveCallbackGroup()
    metadata_callback_group = MutuallyExclusiveCallbackGroup()
    service_callback_group = MutuallyExclusiveCallbackGroup()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="companion-ros-executor",
        daemon=True,
    )
    clock = SimulationClock()
    frame_source = RosFrameSource(width_px=640, height_px=480, run_id=config.run_id)
    lidar = RosLidar(clock)
    gate = Comp2026StartGate()
    mission_publisher = node.create_publisher(
        MissionEvent,
        "/simulation/mission_events",
        qos(100, transient=True),
    )
    ros_payload_client = node.create_client(
        PayloadCommand,
        "/simulation/payload_command",
        callback_group=service_callback_group,
    )
    payload_client = _RosPayloadClient(ros_payload_client, PayloadCommand)
    finalizing = False
    mission_running = False
    requested_stop = False
    initial_command_delivered = False
    runtime_failure_written = False
    runtime_failure_lock = threading.Lock()
    exit_code = 0
    controller: Any | None = None
    mission_worker: threading.Thread | None = None
    last_start_readiness: dict[str, bool] | None = None

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal requested_stop
        requested_stop = True

    def state_callback(message: Any) -> None:
        nonlocal finalizing, mission_running
        if message.run_id != config.run_id:
            return
        if message.state == RunState.RUNNING:
            mission_running = True
            gate.accept_running()
        elif message.state == RunState.FINALIZING:
            finalizing = True

    def clock_callback(message: Any) -> None:
        if not mission_running:
            return

        def accept() -> None:
            clock.accept(stamp_ns(message.clock))
            gate.accept_clock()

        attempt_failure.guard_input("clock", accept)

    def image_callback(message: Any) -> None:
        if not mission_running:
            return
        attempt_failure.guard_input(
            "image", lambda: frame_source.accept_image(message)
        )

    def metadata_callback(message: Any) -> None:
        if not mission_running:
            return
        attempt_failure.guard_input(
            "metadata", lambda: frame_source.accept_metadata(message)
        )

    def range_callback(message: Any) -> None:
        if not mission_running:
            return

        def accept() -> None:
            timestamp_ns = stamp_ns(message.header.stamp)
            lidar.accept(message, timestamp_ns)

        attempt_failure.guard_input("range", accept)

    def publish_mission_event(record: MissionEventRecord) -> None:
        message = MissionEvent()
        message.run_id = record.run_id
        message.sim_timestamp.sec = record.sim_timestamp_ns // 1_000_000_000
        message.sim_timestamp.nanosec = record.sim_timestamp_ns % 1_000_000_000
        message.event_id = record.event_id
        message.phase = record.phase
        message.state = record.state
        message.detail = record.detail
        mission_publisher.publish(message)

    emitter = MissionEventEmitter(config.run_id, clock, publish_mission_event)

    def write_runtime_failure(reason: str) -> None:
        nonlocal runtime_failure_written, exit_code
        with runtime_failure_lock:
            exit_code = 1
            if runtime_failure_written:
                return
            protocol.write_status(
                "runtime-failure",
                {
                    "run_id": config.run_id,
                    "module": "companion",
                    "reason": reason,
                    "diagnostic_paths": ["logs/docker/companion.log.partial"],
                },
            )
            runtime_failure_written = True

    def best_effort_recovery() -> None:
        if controller is None:
            return
        with timebase.configured(clock):
            for name, action in (
                ("RTL", controller.rtl),
                ("LAND", controller.simple_land),
                ("DISARM", controller.disarm),
            ):
                try:
                    action()
                except Exception as error:
                    lifecycle.emit(
                        "recovery_failed",
                        clock.timestamp_ns,
                        {"action": name, "reason": str(error)},
                    )

    def stop_attempt(reason: str) -> None:
        gate.stop(reason)
        frame_source.stop(reason)
        payload_client.stop()
        clock.stop(reason)
        emitter.stop(reason)

    def record_attempt_failure(reason: str) -> None:
        phase = emitter.last_phase or "WAIT_READY"
        lifecycle.emit(
            "mission_failed",
            clock.timestamp_ns,
            {"phase": phase, "reason": reason},
        )
        write_runtime_failure(reason)

    attempt_failure = AttemptFailureCoordinator(
        stop_attempt=stop_attempt,
        write_failure=record_attempt_failure,
        recover=best_effort_recovery,
    )

    def run_original_attempt() -> None:
        try:
            gate.wait_until_ready()
            assert controller is not None
            current_home = controller.get_current_gps()
            home = GPSCoord(current_home.lat, current_home.long, 0.0)
            waypoints = load_course_waypoints(config.course_path, home)
            camera = _create_simulator_camera(
                CameraManager,
                Camera,
                frame_source,
            )
            tracker = MissonTracker(600)
            payloads = {
                marker: PayloadDropper(config.run_id, marker, payload_client, clock)
                for marker in (2, 3, 4)
            }
            with timebase.configured(clock):
                run_auto_attempt(
                    tracker=tracker,
                    controller=controller,
                    camera=camera,
                    lidar=lidar,
                    payloads=payloads,
                    waypoints=waypoints,
                    emit=emitter,
                )
            if not emitter.home_complete:
                raise RuntimeError("original attempt returned without HOME/COMPLETE")
            attempt_failure.finish_success(
                lambda: lifecycle.observe_terminal(
                    MissionState(
                        MissionPhase.LANDED,
                        last_timestamp_ns=clock.timestamp_ns or 0,
                    )
                )
            )
        except Exception as error:
            phase = emitter.last_phase or "WAIT_READY"
            reason = f"original comp2026 mission failed in {phase}: {error}"
            attempt_failure.fail(reason)

    node.create_subscription(
        RunState,
        "/simulation/run_state",
        state_callback,
        qos(1, transient=True),
        callback_group=clock_callback_group,
    )
    node.create_subscription(
        Clock,
        "/clock",
        clock_callback,
        qos(1000),
        callback_group=clock_callback_group,
    )
    node.create_subscription(
        Image,
        "/camera/onboard/image_raw",
        image_callback,
        qos(100),
        callback_group=image_callback_group,
    )
    node.create_subscription(
        FrameMetadata,
        "/camera/onboard/frame_metadata",
        metadata_callback,
        qos(100),
        callback_group=metadata_callback_group,
    )
    node.create_subscription(
        LaserScan,
        "/competition/range/downward",
        range_callback,
        qos(1),
        callback_group=range_callback_group,
    )
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    executor_thread.start()
    overall_wall_deadline = time.monotonic() + config.max_wall_seconds
    try:
        try:
            controller = DroneControl(
                config.mavlink_endpoint,
                wait_ready=False,
                heartbeat_timeout=config.startup_timeout_seconds,
            )
        except Exception as error:
            attempt_failure.fail(f"DroneKit connection failed: {error}")
        else:
            lifecycle.mark_transport_ready()
            mission_worker = threading.Thread(
                target=run_original_attempt,
                name="comp2026-original-attempt",
                daemon=True,
            )
            mission_worker.start()
            gate.mark_process_ready()
            # Preserve the established status schema. For this mission these
            # booleans attest to the initialized DroneKit/gated worker seam;
            # RUNNING-era armability is independently required by the gate.
            lifecycle.observe_mission_readiness(
                heartbeat_observed=True,
                prearm_checks_healthy=True,
            )

        while rclpy.ok() and not requested_stop and not finalizing:
            if mission_running and controller is not None:
                try:
                    refresh_comp2026_start_gate(
                        gate,
                        frame_source=frame_source,
                        lidar=lidar,
                        payload_client=payload_client,
                        vehicle=controller.vehicle,
                    )
                except Exception as error:
                    attempt_failure.fail(
                        f"competition start readiness failed: {error}"
                    )
                readiness = gate.readiness
                if readiness != last_start_readiness:
                    lifecycle.emit(
                        "mission_start_readiness",
                        clock.timestamp_ns,
                        readiness,
                    )
                    last_start_readiness = readiness
                if (
                    clock.timestamp_ns == 0
                    and not initial_command_delivered
                    and gate.mission_ready
                    and not attempt_failure.failed
                ):
                    controller.vehicle.mode = VehicleMode("GUIDED")
                    lifecycle.observe_command_delivery(CommandKind.SET_GUIDED, 0)
                    initial_command_delivered = True
            if attempt_failure.failed and (
                mission_worker is None or not mission_worker.is_alive()
            ):
                attempt_failure.recover_once()
            if protocol.read_finalize_request() is not None:
                finalizing = True
            if time.monotonic() >= overall_wall_deadline and not runtime_failure_written:
                attempt_failure.fail("companion exceeded the overall run wall failsafe")
            time.sleep(0.02)
    finally:
        def close_output_producers() -> None:
            if attempt_failure.failed and (
                mission_worker is None or not mission_worker.is_alive()
            ):
                attempt_failure.recover_once()
            node.destroy_node()
            if controller is not None:
                try:
                    controller.vehicle.close()
                except Exception:
                    pass

        quiesce_comp2026_runtime(
            stop_attempt=stop_attempt,
            mission_worker=mission_worker,
            executor=executor,
            executor_thread=executor_thread,
            close_output_producers=close_output_producers,
            finalize=lambda: lifecycle.finalize(clock.timestamp_ns),
            write_failure=attempt_failure.fail,
            timeout_seconds=config.finalization_wall_seconds,
        )
        protocol.close()
        rclpy.shutdown()
    return exit_code


def main() -> int:
    config = RuntimeConfig.from_environment(os.environ)
    if config.mission == "controlled_descent":
        return _run_controlled_descent(config)
    return _run_comp2026(config)


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "RuntimeConfig",
    "connect_mavlink",
    "first_heartbeat_wall_failure",
    "main",
    "process_runtime_telemetry",
    "quiesce_comp2026_runtime",
    "stamp_ns",
]
