"""Select and host the resolved diagnostic, configured, or Comp2026 mission."""

from __future__ import annotations

from dataclasses import dataclass
import collections
import collections.abc
import hashlib
import hmac
import inspect
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import threading
import time
from collections.abc import Callable
from typing import Any, Mapping
from uuid import UUID

from artifacts.runtime_protocol import RuntimeProtocol
from artifacts.runtime_status import (
    CompanionReadyStatus,
    MissionCommandDeliveredStatus,
    MissionExecutionReadyStatus,
    MissionFinishedStatus,
    MissionReadyStatus,
    RuntimeFailureStatus,
    RuntimeStatus,
)

from .autotune import ActionKind as AutoTuneActionKind
from .autotune import Observation as AutoTuneObservation
from .autotune import Phase as AutoTunePhase
from .autotune import RollAutoTuneDriver
from . import calibration_autotune
from .calibration_gate import CalibrationGate, calibration_parameters
from .hover import Observation as HoverObservation
from .hover import Phase as HoverPhase
from .hover import RollHoverDriver
from .controller import MissionController, mission_policy_active, process_telemetry
from .lifecycle import CompanionLifecycle, INITIAL_COMMAND_WINDOW_NS
from .mavlink_adapter import MavlinkAdapter
from .mission import CommandKind, MissionPhase, MissionState, Telemetry
from .mission_plan import MissionPlan, parse_mission_plan
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


def _validate_sha256_digest(value: object, field: str) -> str:
    if type(value) is not str or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{field} must be a lowercase SHA-256 digest")
    return value


def _verify_competition_source(path: Path, expected_digest: str, source: str) -> None:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise ValueError(f"resolved {source} competition source is unreadable") from error
    if not hmac.compare_digest(digest.hexdigest(), expected_digest):
        raise ValueError(
            f"resolved {source} competition source hash does not match run.json"
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
    mission_plan: MissionPlan | None = None
    scenario: str = ""
    public_duration_ns: int = 0
    calibration_json: str | None = None

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
        if mission not in {
            "controlled_descent",
            "comp2026_auto",
            "autotune_roll",
            "autotune",
            "hover_roll",
            "configured",
        }:
            raise ValueError("resolved mission must select an approved companion host")
        mission_plan = None
        if mission == "configured":
            if document.get("runtime_profile") != "phase3" or not isinstance(
                document.get("simulation"), dict
            ):
                raise ValueError("configured mission requires phase3 simulation")
            mission_plan = parse_mission_plan(document.get("mission_plan"))
        elif "mission_plan" in document:
            raise ValueError("mission_plan is only valid for configured missions")
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
            course_digest = _validate_sha256_digest(
                competition.get("course_sha256"), "course_sha256"
            )
            scenario_digest = _validate_sha256_digest(
                competition.get("scenario_sha256"), "scenario_sha256"
            )
            course_path = config_path.parent / "course.yaml"
            scenario_path = config_path.parent / "scenario.yaml"
            _verify_competition_source(course_path, course_digest, "course")
            _verify_competition_source(scenario_path, scenario_digest, "scenario")
        timeout = override if timeout_override is not None else float(startup_wall_seconds)
        simulation = document.get("simulation")
        duration_seconds = simulation.get("duration_sim_seconds", 0) if isinstance(simulation, dict) else 0
        if mission == "autotune" and (
            isinstance(duration_seconds, bool)
            or not isinstance(duration_seconds, (int, float))
            or not math.isfinite(duration_seconds)
            or duration_seconds <= 60
        ):
            raise ValueError("autotune requires a simulation duration above 60 seconds")
        calibration_json = document.get("calibration_json")
        if calibration_json is not None and not isinstance(calibration_json, str):
            # Source calibration runs carry a structured profile, while dependent
            # runs carry the immutable compact accepted-calibration document.
            calibration_json = json.dumps(calibration_json, sort_keys=True, separators=(",", ":"))
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
            mission_plan=mission_plan,
            scenario=str(document.get("scenario", "")),
            public_duration_ns=int(float(duration_seconds) * 1_000_000_000),
            calibration_json=calibration_json,
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


def autotune_control_timestamp_ns(
    *,
    mission_running: bool,
    latest_clock_ns: int | None,
    first_command_pending: bool,
) -> int | None:
    """Permit the first control write at public zero while physics is paused."""
    if not mission_running:
        return None
    if latest_clock_ns is not None:
        return latest_clock_ns
    return 0 if first_command_pending else None


def autotune_failure_recovery_complete(
    *, recovery_started_ns: int, timestamp_ns: int, armed: bool | None
) -> bool:
    """Bound failed-flight native-LAND recovery without changing its outcome."""
    return armed is False or timestamp_ns - recovery_started_ns >= 45_000_000_000


def autotune_neutral_refresh_due(
    *, phase: calibration_autotune.Phase, last_refresh_wall: float, wall_now: float
) -> bool:
    return (
        calibration_autotune.neutral_override_required(phase)
        and wall_now - last_refresh_wall >= 0.5
    )


class _CalibrationReadiness:
    def __init__(self, expected: Mapping[str, float]) -> None:
        self.expected = dict(expected)
        self.gate = CalibrationGate(self.expected)
        self.reported = False

    @property
    def required(self) -> bool:
        return bool(self.expected)

    @property
    def failure(self) -> str | None:
        return self.gate.failure

    def observe(self, name: str, value: float) -> None:
        if self.reported:
            return
        self.gate.observe(name, value)

    def observe_cached(self, parameters: Mapping[str, float]) -> None:
        if self.reported:
            return
        for name in self.expected:
            try:
                value = parameters[name]
            except (KeyError, TypeError):
                continue
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                self.gate.observe(name, float(value))

    def release(
        self,
        protocol: object,
        lifecycle: CompanionLifecycle,
        run_id: str,
        timestamp_ns: int,
    ) -> bool:
        if not self.gate.ready:
            return False
        if self.required and not self.reported:
            lifecycle.emit(
                "calibration_parameters_verified",
                timestamp_ns,
                {"stage": "pre_arm", "parameters": self.gate.snapshot()},
            )
            protocol.write_status(MissionExecutionReadyStatus(run_id, timestamp_ns))
            self.reported = True
        return True


class _Comp2026ShutdownAdmission:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._finalizing = False
        self._stop_requested = False

    @property
    def finalizing(self) -> bool:
        with self._lock:
            return self._finalizing

    @property
    def stop_requested(self) -> bool:
        return self._stop_requested

    @property
    def shutdown_requested(self) -> bool:
        with self._lock:
            return self._finalizing or self._stop_requested

    def begin_finalizing(self) -> None:
        with self._lock:
            self._finalizing = True

    def request_stop_from_signal(self) -> None:
        self._stop_requested = True

    def run_if_active(self, operation: Callable[[], None]) -> bool:
        with self._lock:
            if self._finalizing or self._stop_requested:
                return False
            operation()
            return True


def _deliver_comp2026_initial_command(
    *,
    vehicle: object,
    vehicle_mode_type: Callable[[str], object],
    lifecycle: CompanionLifecycle,
    gate: Comp2026StartGate,
    attempt_failure: AttemptFailureCoordinator,
    clock: SimulationClock,
    mark_delivered: Callable[[], None],
    shutdown_admission: _Comp2026ShutdownAdmission | None = None,
) -> bool:
    delivered = False
    admission = shutdown_admission or _Comp2026ShutdownAdmission()

    def deliver_at(timestamp_ns: int) -> None:
        nonlocal delivered
        if timestamp_ns > INITIAL_COMMAND_WINDOW_NS:
            raise _InitialCommandWindowMissed
        vehicle.mode = vehicle_mode_type("GUIDED")  # type: ignore[attr-defined]
        lifecycle.observe_command_delivery(CommandKind.SET_GUIDED, timestamp_ns)
        gate.mark_command_delivered()
        mark_delivered()
        delivered = True

    try:
        claimed = attempt_failure.finish_success(
            lambda: admission.run_if_active(
                lambda: clock.run_at_current_timestamp(deliver_at)
            )
        )
    except _InitialCommandWindowMissed:
        attempt_failure.fail("initial GUIDED command missed the 50 ms delivery window")
        return False
    except Exception as error:
        attempt_failure.fail(f"initial GUIDED command failed: {error}")
        return False
    return claimed and delivered


class _InitialCommandWindowMissed(Exception):
    pass


def comp2026_start_gate_poll_required(
    *,
    mission_running: bool,
    mission_start_ready: bool,
    failed: bool,
) -> bool:
    """Keep dynamic start checks out of the active mission data path."""
    return mission_running and not mission_start_ready and not failed


def comp2026_sensor_inputs_required(
    *,
    mission_running: bool,
    mission_worker_alive: bool,
) -> bool:
    """Keep sensors through startup and while the running attempt can use them."""
    return not mission_running or mission_worker_alive


def connect_autotune_vehicle(
    factory: Callable[..., Any],
    endpoint: str,
    *,
    heartbeat_timeout: float,
    run_state_subscription: Any,
) -> Any:
    """Connect only after ROS can receive the READY state that starts simulation."""
    if run_state_subscription is None:
        raise RuntimeError("AutoTune requires a run-state subscription before connecting")
    return factory(
        endpoint,
        wait_ready=False,
        heartbeat_timeout=heartbeat_timeout,
    )


class _ProductionProtocol:
    """Restrict companion-owned writes through the shared runtime protocol."""

    def __init__(self, config: RuntimeConfig) -> None:
        self._runtime = RuntimeProtocol(config.run_directory, config.run_id)

    def write_status(self, status: RuntimeStatus) -> None:
        if type(status) not in {
            CompanionReadyStatus,
            MissionReadyStatus,
            MissionCommandDeliveredStatus,
            MissionExecutionReadyStatus,
            MissionFinishedStatus,
            RuntimeFailureStatus,
        }:
            raise ValueError("companion does not own that status")
        self._runtime.write_status(status)

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
    calibration = _CalibrationReadiness(calibration_parameters(config))
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
    parameters_requested = False
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
                and calibration.release(
                    protocol, lifecycle, config.run_id, latest_clock_ns
                )
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
                        if (
                            telemetry.parameter_name is not None
                            and telemetry.parameter_value is not None
                        ):
                            calibration.observe(
                                telemetry.parameter_name, telemetry.parameter_value
                            )
                            if calibration.failure is not None:
                                failure = calibration.failure
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
            if (
                calibration.required
                and controller.heartbeat_observed
                and not parameters_requested
                and failure is None
            ):
                vehicle.request_parameters(tuple(calibration.expected))
                parameters_requested = True
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
            if (
                calibration.required
                and not calibration.gate.ready
                and time.monotonic() >= overall_wall_deadline
            ):
                failure = "calibration parameter readback was unavailable before the overall run wall failsafe"
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


def _run_autotune_roll(config: RuntimeConfig) -> int:
    """Run roll-only AutoTune or its short post-promotion hover check."""

    protocol = _ProductionProtocol(config)
    lifecycle = CompanionLifecycle(run_id=config.run_id, protocol=protocol, stream=sys.stdout)
    lifecycle.emit("starting", None, {"mavlink_endpoint": config.mavlink_endpoint})
    _enable_dronekit_python312_compatibility()

    import rclpy
    from dronekit import VehicleMode, connect
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import RunState

    started = time.monotonic()
    overall_wall_deadline = started + config.max_wall_seconds
    rclpy.init()
    node = Node("drone_sim_companion")
    latest_clock_ns: int | None = None
    mission_running = False
    finalizing = False
    requested_stop = False
    command_delivered = False
    failure: str | None = None
    last_override_refresh = 0.0

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal requested_stop
        requested_stop = True

    def clock_callback(message: Any) -> None:
        nonlocal latest_clock_ns, failure
        if not mission_running:
            return
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
    run_state_subscription = node.create_subscription(
        RunState,
        "/simulation/run_state",
        state_callback,
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    try:
        vehicle = connect_autotune_vehicle(
            connect,
            config.mavlink_endpoint,
            heartbeat_timeout=config.startup_timeout_seconds,
            run_state_subscription=run_state_subscription,
        )
    except Exception as error:
        reason = f"DroneKit connection failed: {error}"
        lifecycle.observe_terminal(
            MissionState(MissionPhase.FAILED, last_timestamp_ns=0, failure_reason=reason)
        )
        lifecycle.finalize(None)
        node.destroy_node()
        rclpy.shutdown()
        protocol.close()
        return 1

    lifecycle.mark_transport_ready()
    calibration = _CalibrationReadiness(calibration_parameters(config))
    hover_only = config.mission == "hover_roll"
    driver = (RollHoverDriver if hover_only else RollAutoTuneDriver)(
        vehicle,
        run_directory=config.run_directory,
        run_id=config.run_id,
        mode_factory=VehicleMode,
    )
    status_texts: collections.deque[str] = collections.deque()
    status_lock = threading.Lock()

    def status_callback(_vehicle: Any, _name: str, message: Any) -> None:
        text = getattr(message, "text", "")
        if isinstance(text, bytes):
            text = text.decode("utf-8", errors="replace")
        normalized = str(text).rstrip("\x00").strip()
        if normalized:
            with status_lock:
                status_texts.append(normalized)

    vehicle.add_message_listener("STATUSTEXT", status_callback)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    exit_code = 0
    try:
        while rclpy.ok() and not requested_stop and not finalizing:
            rclpy.spin_once(node, timeout_sec=0.02)
            heartbeat = (
                isinstance(getattr(vehicle, "last_heartbeat", None), (int, float))
                and not isinstance(vehicle.last_heartbeat, bool)
                and math.isfinite(vehicle.last_heartbeat)
                and 0.0 <= vehicle.last_heartbeat <= 60.0
            )
            armable = getattr(vehicle, "is_armable", False) is True
            lifecycle.observe_mission_readiness(
                heartbeat_observed=heartbeat,
                prearm_checks_healthy=armable,
            )
            calibration.observe_cached(getattr(vehicle, "parameters", {}))
            if calibration.failure is not None:
                failure = calibration.failure

            control_timestamp_ns = autotune_control_timestamp_ns(
                mission_running=mission_running,
                latest_clock_ns=latest_clock_ns,
                first_command_pending=driver.state.phase
                is (HoverPhase.WAIT_READY if hover_only else AutoTunePhase.WAIT_READY),
            )
            if (
                control_timestamp_ns is not None
                and failure is None
                and calibration.release(
                    protocol,
                    lifecycle,
                    config.run_id,
                    control_timestamp_ns,
                )
            ):
                with status_lock:
                    status_text = status_texts.popleft() if status_texts else None
                mode_value = getattr(vehicle, "mode", None)
                mode = getattr(mode_value, "name", str(mode_value))
                armed = getattr(vehicle, "armed", None)
                altitude = getattr(
                    getattr(getattr(vehicle, "location", None), "global_relative_frame", None),
                    "alt",
                    None,
                )
                landed = (
                    armed is False
                    and isinstance(altitude, (int, float))
                    and math.isfinite(altitude)
                    and altitude <= 0.3
                )
                try:
                    previous_phase = driver.state.phase
                    observation_type = HoverObservation if hover_only else AutoTuneObservation
                    transition = driver.observe(
                        observation_type(
                            control_timestamp_ns,
                            heartbeat=heartbeat,
                            prearm_checks_healthy=armable,
                            mode=mode,
                            armed=armed,
                            landed=landed,
                            relative_altitude_m=altitude,
                            status_text=status_text,
                        )
                    )
                    if not hover_only:
                        written_parameters = {
                            action.name: action.value
                            for action in transition.actions
                            if action.kind is AutoTuneActionKind.SET_PARAMETER
                        }
                        if written_parameters:
                            lifecycle.emit(
                                "calibration_parameters_overridden",
                                control_timestamp_ns,
                                {
                                    "parameters": written_parameters,
                                    "reason": "roll diagnostic seed",
                                },
                            )
                    if transition.state.phase is not previous_phase:
                        lifecycle.emit(
                            "hover_phase" if hover_only else "autotune_phase",
                            control_timestamp_ns,
                            {"phase": transition.state.phase.value},
                        )
                    if status_text is not None:
                        lifecycle.emit(
                            "ardupilot_status_text",
                            control_timestamp_ns,
                            {"text": status_text},
                        )
                    if previous_phase is (
                        HoverPhase.WAIT_READY if hover_only else AutoTunePhase.WAIT_READY
                    ) and not command_delivered:
                        lifecycle.observe_command_delivery(CommandKind.SET_GUIDED, 0)
                        command_delivered = True
                except Exception as error:
                    mission_name = "roll hover" if hover_only else "roll AutoTune"
                    failure = f"{mission_name} control failed: {error}"

                wall_now = time.monotonic()
                if failure is None and wall_now - last_override_refresh >= 0.5:
                    try:
                        driver.refresh_override()
                    except Exception as error:
                        mission_name = "roll hover" if hover_only else "roll AutoTune"
                        failure = f"{mission_name} RC override failed: {error}"
                    last_override_refresh = wall_now

                complete_phase = HoverPhase.COMPLETE if hover_only else AutoTunePhase.COMPLETE
                failed_phase = HoverPhase.FAILED if hover_only else AutoTunePhase.FAILED
                if driver.state.phase is complete_phase:
                    lifecycle.observe_terminal(
                        MissionState(MissionPhase.LANDED, last_timestamp_ns=latest_clock_ns)
                    )
                elif driver.state.phase is failed_phase:
                    failure = driver.state.failure_reason

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
                heartbeat_observed=heartbeat,
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
            if (
                calibration.required
                and not calibration.gate.ready
                and time.monotonic() >= overall_wall_deadline
            ):
                failure = "calibration parameter readback was unavailable before the overall run wall failsafe"
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
        try:
            vehicle.channels.overrides = {}
            vehicle.remove_message_listener("STATUSTEXT", status_callback)
            vehicle.close()
        finally:
            lifecycle.finalize(latest_clock_ns)
            node.destroy_node()
            protocol.close()
            rclpy.shutdown()
    return exit_code


def _run_autotune(config: RuntimeConfig) -> int:
    """Run all-axis AutoTune, activate its gains, settle, and use native LAND."""
    protocol = _ProductionProtocol(config)
    lifecycle = CompanionLifecycle(run_id=config.run_id, protocol=protocol, stream=sys.stdout)
    lifecycle.emit("starting", None, {"mavlink_endpoint": config.mavlink_endpoint})
    _enable_dronekit_python312_compatibility()
    import rclpy
    from dronekit import VehicleMode, connect
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import RunState

    rclpy.init()
    node = Node("drone_sim_companion")
    latest_clock_ns: int | None = None
    mission_running = False
    finalizing = False
    stopped = False
    failure: str | None = None

    def stop(_signum: int, _frame: Any) -> None:
        nonlocal stopped
        stopped = True
    def clock_callback(message: Any) -> None:
        nonlocal latest_clock_ns, failure
        if not mission_running: return
        value = stamp_ns(message.clock)
        if latest_clock_ns is not None and value < latest_clock_ns:
            failure = "authoritative simulation clock regressed"
        else: latest_clock_ns = value
    def state_callback(message: Any) -> None:
        nonlocal mission_running, finalizing
        if message.run_id != config.run_id: return
        if message.state == RunState.RUNNING: mission_running = True
        elif message.state == RunState.FINALIZING: finalizing = True

    node.create_subscription(Clock, "/clock", clock_callback, QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
    run_state_subscription = node.create_subscription(RunState, "/simulation/run_state", state_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        vehicle = connect_autotune_vehicle(connect, config.mavlink_endpoint, heartbeat_timeout=config.startup_timeout_seconds, run_state_subscription=run_state_subscription)
    except Exception as error:
        lifecycle.observe_terminal(MissionState(MissionPhase.FAILED, last_timestamp_ns=0, failure_reason=f"DroneKit connection failed: {error}"))
        lifecycle.finalize(None); node.destroy_node(); rclpy.shutdown(); protocol.close()
        return 1

    lifecycle.mark_transport_ready()
    state = calibration_autotune.AllAxisState.initial(public_deadline_ns=config.public_duration_ns)
    status_texts: collections.deque[str] = collections.deque()
    status_assembler = calibration_autotune.StatusTextAssembler()
    parameter_values: dict[str, float] = {}
    parameter_generations: dict[str, int] = {}
    parameter_generation_counter = 0
    aux_ack = False
    position_sample: tuple[int, float, float, float] | None = None
    attitude_sample: tuple[int, float, float] | None = None
    locked = threading.Lock()

    def status_callback(_vehicle: Any, _name: str, message: Any) -> None:
        raw = getattr(message, "text", "")
        if isinstance(raw, bytes): raw = raw.decode("utf-8", errors="replace")
        complete = status_assembler.push(int(getattr(message, "id", 0)), int(getattr(message, "chunk_seq", 0)), str(raw))
        if complete:
            with locked: status_texts.append(complete)
    def parameter_callback(_vehicle: Any, _name: str, message: Any) -> None:
        nonlocal parameter_generation_counter
        raw_name = getattr(message, "param_id", "")
        if isinstance(raw_name, bytes): raw_name = raw_name.decode("ascii", errors="ignore")
        name = str(raw_name).rstrip("\x00")
        value = getattr(message, "param_value", None)
        if name and isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
            with locked:
                parameter_generation_counter += 1
                parameter_values[name] = float(value)
                parameter_generations[name] = parameter_generation_counter
    def ack_callback(_vehicle: Any, _name: str, message: Any) -> None:
        nonlocal aux_ack
        if getattr(message, "command", None) == calibration_autotune.mavutil.mavlink.MAV_CMD_DO_AUX_FUNCTION and getattr(message, "result", None) == calibration_autotune.mavutil.mavlink.MAV_RESULT_ACCEPTED:
            with locked: aux_ack = True
    def position_callback(_vehicle: Any, _name: str, message: Any) -> None:
        nonlocal position_sample
        stamp = latest_clock_ns
        if stamp is None: return
        values = (getattr(message, "vx", None), getattr(message, "vy", None), getattr(message, "vz", None), getattr(message, "relative_alt", None))
        if all(isinstance(value, (int, float)) for value in values):
            with locked: position_sample = (stamp, math.hypot(float(values[0]), float(values[1])) / 100.0, abs(float(values[2])) / 100.0, float(values[3]) / 1000.0)
    def attitude_callback(_vehicle: Any, _name: str, message: Any) -> None:
        nonlocal attitude_sample
        stamp = latest_clock_ns
        if stamp is None: return
        roll, pitch = getattr(message, "roll", None), getattr(message, "pitch", None)
        if isinstance(roll, (int, float)) and isinstance(pitch, (int, float)):
            with locked: attitude_sample = (stamp, float(roll), float(pitch))

    listeners = (("STATUSTEXT", status_callback), ("PARAM_VALUE", parameter_callback), ("COMMAND_ACK", ack_callback), ("GLOBAL_POSITION_INT", position_callback), ("ATTITUDE", attitude_callback))
    for name, callback in listeners: vehicle.add_message_listener(name, callback)
    signal.signal(signal.SIGTERM, stop); signal.signal(signal.SIGINT, stop)
    command_delivered = False
    armed_seen = False
    disarm_event_emitted = False
    aux_ack_event_emitted = False
    exit_code = 0
    overall_wall_deadline = time.monotonic() + config.max_wall_seconds
    recovery_started_ns: int | None = None
    last_neutral_refresh_wall = 0.0

    def export_readback() -> None:
        from artifacts.calibration import (
            read_saved_calibration_parameters,
            write_calibration_parameters,
        )
        saved = read_saved_calibration_parameters(config.run_directory)
        live = dict(state.activated_parameters)
        if set(saved) != set(live) or any(
            not math.isclose(saved[name], live[name], rel_tol=1e-5, abs_tol=1e-7)
            for name in saved
        ):
            raise ValueError("saved DataFlash gains do not match post-disarm live readback")
        write_calibration_parameters(config.run_directory, config.run_id, saved)

    def request_parameter_session() -> None:
        """Clear stale replies and start one atomic full-parameter session."""
        with locked:
            parameter_values.clear()
            parameter_generations.clear()
            vehicle._master.mav.param_request_list_send(
                vehicle._master.target_system,
                vehicle._master.target_component,
            )

    try:
        while rclpy.ok() and not stopped and not finalizing:
            rclpy.spin_once(node, timeout_sec=.02)
            heartbeat = isinstance(getattr(vehicle, "last_heartbeat", None), (int, float)) and not isinstance(vehicle.last_heartbeat, bool) and math.isfinite(vehicle.last_heartbeat) and 0 <= vehicle.last_heartbeat <= 60
            armable = getattr(vehicle, "is_armable", False) is True
            lifecycle.observe_mission_readiness(heartbeat_observed=heartbeat, prearm_checks_healthy=armable)
            stamp = autotune_control_timestamp_ns(mission_running=mission_running, latest_clock_ns=latest_clock_ns, first_command_pending=state.phase is calibration_autotune.Phase.WAIT_READY)
            armed = getattr(vehicle, "armed", None)
            if stamp is not None and failure is None:
                with locked:
                    status = status_texts.popleft() if status_texts else None
                    parameters = dict(parameter_values)
                    generation_names = (*calibration_autotune.GAIN_PARAMETERS, *calibration_autotune.PRESERVED_PARAMETERS)
                    generation = calibration_autotune.parameter_readback_generation(
                        parameters, parameter_generations, generation_names
                    )
                    ack = aux_ack
                    pos, att = position_sample, attitude_sample
                mode_value = getattr(vehicle, "mode", None); mode = getattr(mode_value, "name", str(mode_value))
                armed_seen = armed_seen or armed is True
                telemetry_stamp = min(pos[0], att[0]) if pos and att else None
                observation = calibration_autotune.Observation(timestamp_ns=stamp, heartbeat=heartbeat, prearm_checks_healthy=armable, mode=mode, armed=armed, landed=armed is False and pos is not None and pos[3] <= .3, relative_altitude_m=pos[3] if pos else None, status_text=status, aux_ack=ack, parameters=parameters, parameter_generation=generation, telemetry_timestamp_ns=telemetry_stamp, horizontal_speed_m_s=pos[1] if pos else None, vertical_speed_m_s=pos[2] if pos else None, roll_rad=att[1] if att else None, pitch_rad=att[2] if att else None)
                previous = state.phase
                try:
                    transition = calibration_autotune.advance(state, observation)
                    calibration_autotune.execute_actions(
                        vehicle,
                        transition.actions,
                        mode_factory=VehicleMode,
                        export=export_readback,
                        request_parameters=request_parameter_session,
                    )
                    state = transition.state
                except Exception as error: failure = f"all-axis AutoTune control failed: {error}"
                if state.phase is not previous: lifecycle.emit("autotune_phase", stamp, {"phase": state.phase.value})
                evidence_stage = None
                if previous is calibration_autotune.Phase.WAIT_GUIDED and state.phase is calibration_autotune.Phase.WAIT_ARMED:
                    evidence_stage = "baseline"
                elif previous is calibration_autotune.Phase.WAIT_GAIN_ACTIVATION and state.phase is calibration_autotune.Phase.SETTLING:
                    evidence_stage = "activation"
                elif previous is calibration_autotune.Phase.POST_DISARM_READBACK and state.phase is calibration_autotune.Phase.COMPLETE:
                    evidence_stage = "post_disarm"
                if evidence_stage is not None:
                    names = (*calibration_autotune.GAIN_PARAMETERS, *calibration_autotune.PRESERVED_PARAMETERS)
                    lifecycle.emit("calibration_parameters_verified", stamp, {"stage": evidence_stage, "parameters": {name: parameters[name] for name in names if name in parameters}})
                if previous is calibration_autotune.Phase.WAIT_READY and not command_delivered:
                    lifecycle.observe_command_delivery(CommandKind.SET_GUIDED, stamp)
                    command_delivered = True
                if status: lifecycle.emit("ardupilot_status_text", stamp, {"text": status})
                if ack and previous is calibration_autotune.Phase.WAIT_GAIN_ACTIVATION and not aux_ack_event_emitted:
                    lifecycle.emit("autotune_aux_ack", stamp, {"function": 180, "position": 2})
                    aux_ack_event_emitted = True
                if armed_seen and armed is False and not disarm_event_emitted:
                    lifecycle.emit("autotune_disarmed", stamp, {})
                    disarm_event_emitted = True
                if state.phase is calibration_autotune.Phase.COMPLETE: lifecycle.observe_terminal(MissionState(MissionPhase.LANDED, last_timestamp_ns=stamp))
                elif state.phase is calibration_autotune.Phase.FAILED: failure = state.failure_reason
            wall_now = time.monotonic()
            if failure is None and autotune_neutral_refresh_due(
                phase=state.phase,
                last_refresh_wall=last_neutral_refresh_wall,
                wall_now=wall_now,
            ):
                try:
                    calibration_autotune.execute_actions(
                        vehicle,
                        (calibration_autotune.Action.neutral_override(),),
                        mode_factory=VehicleMode,
                        export=lambda: None,
                    )
                    last_neutral_refresh_wall = wall_now
                except Exception as error:
                    failure = f"all-axis AutoTune RC override failed: {error}"
            if failure:
                recovery_stamp = latest_clock_ns or 0
                if armed is True and recovery_started_ns is None:
                    try:
                        calibration_autotune.execute_actions(
                            vehicle,
                            (calibration_autotune.Action.clear_overrides(), calibration_autotune.Action.mode("LAND")),
                            mode_factory=VehicleMode,
                            export=lambda: None,
                        )
                        recovery_started_ns = recovery_stamp
                        lifecycle.emit("autotune_failure_recovery", recovery_stamp, {"mode": "LAND"})
                    except Exception:
                        recovery_started_ns = recovery_stamp - 45_000_000_000
                if recovery_started_ns is None or autotune_failure_recovery_complete(
                    recovery_started_ns=recovery_started_ns,
                    timestamp_ns=recovery_stamp,
                    armed=armed,
                ) or time.monotonic() >= overall_wall_deadline:
                    lifecycle.observe_terminal(MissionState(MissionPhase.FAILED, last_timestamp_ns=recovery_stamp, failure_reason=failure)); exit_code = 1; break
            heartbeat_failure = first_heartbeat_wall_failure(heartbeat_observed=heartbeat, wall_now=time.monotonic(), overall_wall_deadline=overall_wall_deadline)
            if heartbeat_failure: failure = heartbeat_failure
            if protocol.read_finalize_request() is not None: finalizing = True
    finally:
        try:
            vehicle.channels.overrides = {}
            for name, callback in listeners: vehicle.remove_message_listener(name, callback)
            vehicle.close()
        finally:
            lifecycle.finalize(latest_clock_ns); node.destroy_node(); protocol.close(); rclpy.shutdown()
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
    from simulation_interfaces.msg import MissionEvent, RunState
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
    service_callback_group = MutuallyExclusiveCallbackGroup()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="companion-ros-executor",
        daemon=True,
    )
    clock = SimulationClock()
    frame_source = RosFrameSource(width_px=640, height_px=480)
    lidar = RosLidar(clock)
    gate = Comp2026StartGate()
    calibration = _CalibrationReadiness(calibration_parameters(config))
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
    shutdown_admission = _Comp2026ShutdownAdmission()
    mission_running = False
    initial_command_delivered = False
    runtime_failure_written = False
    runtime_failure_lock = threading.Lock()
    exit_code = 0
    controller: Any | None = None
    mission_worker: threading.Thread | None = None
    last_start_readiness: dict[str, bool] | None = None

    def stop(_signum: int, _frame: Any) -> None:
        shutdown_admission.request_stop_from_signal()

    def state_callback(message: Any) -> None:
        nonlocal mission_running
        if message.run_id != config.run_id:
            return
        if message.state == RunState.RUNNING:
            mission_running = True
            gate.accept_running()
        elif message.state == RunState.FINALIZING:
            shutdown_admission.begin_finalizing()

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
                RuntimeFailureStatus(
                    config.run_id,
                    "companion",
                    reason,
                    ("logs/docker/companion.log.partial",),
                )
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
            if shutdown_admission.shutdown_requested:
                return
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
        qos(1),
        callback_group=clock_callback_group,
    )
    image_subscription = node.create_subscription(
        Image,
        "/camera/onboard/image_raw",
        image_callback,
        qos(100),
        callback_group=image_callback_group,
    )
    range_subscription = node.create_subscription(
        LaserScan,
        "/competition/range/downward",
        range_callback,
        qos(1),
        callback_group=range_callback_group,
    )
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    executor_thread.start()
    sensor_subscriptions = (
        image_subscription,
        range_subscription,
    )
    sensor_subscriptions_active = True
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

        while (
            rclpy.ok()
            and not shutdown_admission.stop_requested
            and not shutdown_admission.finalizing
        ):
            if controller is not None and not attempt_failure.failed:
                calibration.observe_cached(
                    getattr(controller.vehicle, "parameters", {})
                )
                if calibration.failure is not None:
                    attempt_failure.fail(calibration.failure)
            if controller is not None and comp2026_start_gate_poll_required(
                mission_running=mission_running,
                mission_start_ready=gate.mission_start_ready,
                failed=attempt_failure.failed,
            ):
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
                    mission_running
                    and gate.mission_ready
                    and not initial_command_delivered
                    and calibration.release(
                        protocol,
                        lifecycle,
                        config.run_id,
                        clock.timestamp_ns or 0,
                    )
                ):
                    def mark_initial_command_delivered() -> None:
                        nonlocal initial_command_delivered
                        initial_command_delivered = True

                    _deliver_comp2026_initial_command(
                        vehicle=controller.vehicle,
                        vehicle_mode_type=VehicleMode,
                        lifecycle=lifecycle,
                        gate=gate,
                        attempt_failure=attempt_failure,
                        clock=clock,
                        mark_delivered=mark_initial_command_delivered,
                        shutdown_admission=shutdown_admission,
                    )
            if (
                sensor_subscriptions_active
                and mission_worker is not None
                and not comp2026_sensor_inputs_required(
                    mission_running=mission_running,
                    mission_worker_alive=mission_worker.is_alive(),
                )
            ):
                for subscription in sensor_subscriptions:
                    node.destroy_subscription(subscription)
                sensor_subscriptions_active = False
            if attempt_failure.failed and (
                mission_worker is None or not mission_worker.is_alive()
            ):
                attempt_failure.recover_once()
            if protocol.read_finalize_request() is not None:
                shutdown_admission.begin_finalizing()
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
    if config.mission == "configured":
        from .configured_runtime import run_configured

        return run_configured(config)
    if config.mission == "controlled_descent":
        return _run_controlled_descent(config)
    if config.mission == "autotune":
        return _run_autotune(config)
    if config.mission in {"autotune_roll", "hover_roll"}:
        return _run_autotune_roll(config)
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
