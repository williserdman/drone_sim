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
from types import SimpleNamespace
from typing import Any, Mapping
from uuid import UUID

import yaml

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

from .autotune import Observation as AutoTuneObservation
from .autotune import Phase as AutoTunePhase
from .autotune import RollAutoTuneDriver
from .hover import Observation as HoverObservation
from .hover import Phase as HoverPhase
from .hover import RollHoverDriver
from .controller import MissionController, mission_policy_active, process_telemetry
from .lifecycle import CompanionLifecycle, INITIAL_COMMAND_WINDOW_NS
from .mavlink_adapter import MavlinkAdapter
from .mission import CommandKind, MissionPhase, MissionState, Telemetry
from .mission_plan import COMPETITION_TOOLS, MissionPlan, parse_mission_plan
from .qgc_runtime_config import (
    ResolvedQGCInputs,
    project_qgc_runtime,
    read_resolved_run_document,
    resolved_qgc_inputs,
    validate_resolved_competition,
)
from .comp2026_host import (
    AttemptFailureCoordinator,
    Comp2026StartGate,
    GPSCoord,
    MissionEventEmitter,
    MissionEventRecord,
    PayloadDropper,
    PayloadRequest,
    QgcFm2PayloadAdapter,
    QgcRangeIngress,
    QgcRosLidarAdapter,
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
    scenario: str = "descent_v1"
    mavlink_endpoint: str = "tcp:ardupilot-sitl:5760"
    startup_timeout_seconds: float = 60.0
    max_wall_seconds: float = 3600.0
    finalization_wall_seconds: float = 120.0
    course_path: Path | None = None
    scenario_path: Path | None = None
    qgc: ResolvedQGCInputs | None = None
    output_root: Path | None = None
    mission_plan: MissionPlan | None = None

    @staticmethod
    def _validate_qgc_structure(raw: object) -> None:
        artifact_names = {
            "deployment_profile": "deployment-profile.json",
            "listener_session": "listener-session.json",
            "qgc_actions": "qgc-actions.json",
            "runtime_policy": "qgc-runtime.json",
        }
        expected_fields = {
            *artifact_names,
            *(f"{field}_sha256" for field in artifact_names),
            "attempt_state_id",
        }
        if not isinstance(raw, dict) or set(raw) != expected_fields:
            raise ValueError("resolved QGC configuration has missing or unknown fields")
        for field, artifact_name in artifact_names.items():
            if raw[field] != artifact_name:
                raise ValueError("resolved QGC sources must use canonical artifact names")
            digest = raw[f"{field}_sha256"]
            if (
                not isinstance(digest, str)
                or len(digest) != 64
                or any(character not in "0123456789abcdef" for character in digest)
            ):
                raise ValueError(f"{field}_sha256 must be a lowercase SHA-256")
        if raw["attempt_state_id"] != (
            "sha256-" + raw["deployment_profile_sha256"]
        ):
            raise ValueError("attempt_state_id does not match the deployment profile digest")

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
        document = read_resolved_run_document(config_path)
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
            "hover_roll",
            "configured",
        }:
            raise ValueError("resolved mission must select an approved companion host")
        scenario = document.get(
            "scenario", "competition_v1" if mission == "comp2026_auto" else "descent_v1"
        )
        if not isinstance(scenario, str) or not scenario:
            raise ValueError("resolved scenario must be a nonempty string")
        mission_plan = None
        if mission == "configured":
            if document.get("runtime_profile") != "phase3" or not isinstance(document.get("simulation"), dict):
                raise ValueError("configured mission requires phase3 simulation")
            mission_plan = parse_mission_plan(document.get("mission_plan"))
            if (
                scenario != "competition_v1"
                and any(step.tool in COMPETITION_TOOLS for step in mission_plan.steps)
            ):
                raise ValueError(
                    "competition mission tools require scenario competition_v1"
                )
        elif "mission_plan" in document:
            raise ValueError("mission_plan is only valid for configured missions")
        course_path: Path | None = None
        scenario_path: Path | None = None
        qgc: ResolvedQGCInputs | None = None
        output_root: Path | None = None
        if mission != "comp2026_auto" and "qgc" in document:
            raise ValueError("QGC configuration is only valid for comp2026_auto")
        if "qgc" in document and (
            document.get("runtime_profile") != "phase3"
            or not isinstance(document.get("simulation"), dict)
        ):
            raise ValueError("QGC configuration requires an explicit phase3 simulation")
        if mission == "comp2026_auto":
            cls._validate_qgc_structure(document.get("qgc"))
        competition: Mapping[str, object] | None = None
        if scenario == "competition_v1":
            competition = document.get("competition")
            if (
                not isinstance(competition, dict)
                or competition.get("course") != "course.yaml"
                or competition.get("scenario") != "scenario.yaml"
            ):
                raise ValueError("competition_v1 requires resolved competition sources")
            _validate_sha256_digest(
                competition.get("course_sha256"), "course_sha256"
            )
            _validate_sha256_digest(
                competition.get("scenario_sha256"), "scenario_sha256"
            )
            if set(competition) != {
                "course",
                "scenario",
                "course_sha256",
                "scenario_sha256",
            }:
                raise ValueError("competition_v1 requires resolved competition sources")
            course_path, scenario_path = validate_resolved_competition(
                competition, configuration_directory=config_path.parent
            )
        if mission == "comp2026_auto":
            if competition is None:
                raise ValueError("comp2026_auto requires scenario competition_v1")
            qgc = resolved_qgc_inputs(
                document.get("qgc"),
                configuration_directory=config_path.parent,
                competition=competition,
            )
            output_root_value = document.get("output_root")
            if not isinstance(output_root_value, str) or output_root_value.startswith("//"):
                raise ValueError("comp2026_auto requires a canonical output_root")
            output_root = Path(output_root_value)
            if (
                not output_root.is_absolute()
                or output_root != Path(os.path.abspath(output_root))
            ):
                raise ValueError("comp2026_auto requires a canonical output_root")
        timeout = override if timeout_override is not None else float(startup_wall_seconds)
        endpoint = environment.get("SIM_MAVLINK_ENDPOINT", "tcp:ardupilot-sitl:5760")
        if endpoint != "tcp:ardupilot-sitl:5760":
            raise ValueError("production MAVLink endpoint must be tcp:ardupilot-sitl:5760")
        return cls(
            run_id=run_id,
            run_directory=run_directory,
            mission=mission,
            scenario=scenario,
            mavlink_endpoint=endpoint,
            startup_timeout_seconds=timeout,
            max_wall_seconds=float(max_wall_seconds),
            finalization_wall_seconds=float(finalization_wall_seconds),
            course_path=course_path,
            scenario_path=scenario_path,
            qgc=qgc,
            output_root=output_root,
            mission_plan=mission_plan,
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


def create_comp2026_lidar(clock: SimulationClock) -> RosLidar:
    """Bind the ROS range adapter to the nested mission's exact sample type."""

    from drone.sensors.lidar.lidar import LidarSample

    return RosLidar(clock, sample_factory=LidarSample)


def accept_comp2026_range_input(*, lidar: RosLidar, message: Any, guard_input) -> bool:
    """Invalidate malformed range evidence before fatal input coordination."""

    def accept() -> None:
        try:
            timestamp_ns = stamp_ns(message.header.stamp)
        except Exception:
            lidar.invalidate()
            raise
        lidar.accept(message, timestamp_ns)

    return guard_input("range", accept)


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

    def __init__(
        self,
        client: Any,
        service_type: Any,
        *,
        response_timeout_seconds: float,
    ) -> None:
        if (
            isinstance(response_timeout_seconds, bool)
            or not isinstance(response_timeout_seconds, (int, float))
            or not math.isfinite(response_timeout_seconds)
            or response_timeout_seconds <= 0
        ):
            raise ValueError("payload response timeout must be finite and positive")
        self._client = client
        self._service_type = service_type
        self._response_timeout_seconds = float(response_timeout_seconds)
        self._stopped = threading.Event()
        self._production_lock = threading.Lock()

    def service_is_ready(self) -> bool:
        return bool(self._client.service_is_ready())

    def prepare(self, request: PayloadRequest) -> Any:
        if self._stopped.is_set():
            raise RuntimeError("payload client stopped")
        ros_request = self._service_type.Request()
        ros_request.run_id = request.run_id
        ros_request.aruco_id = request.aruco_id
        ros_request.action = request.action
        ros_request.command_id = request.command_id
        return ros_request

    def dispatch(self, prepared: object) -> Any:
        with self._production_lock:
            if self._stopped.is_set():
                raise RuntimeError("payload client stopped")
            future = self._client.call_async(prepared)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        return future, completed

    def await_response(
        self, pending: object, *, cancelled: threading.Event
    ) -> Any:
        future, completed = pending
        deadline = time.monotonic() + self._response_timeout_seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                future.cancel()
                raise TimeoutError("payload confirmation timed out")
            if completed.wait(min(0.05, remaining)):
                return future.result()
            if cancelled.is_set():
                future.cancel()
                raise RuntimeError("payload adapter closed during confirmation")
            if self._stopped.is_set():
                future.cancel()
                raise RuntimeError("payload client stopped during confirmation")

    def stop(self) -> None:
        with self._production_lock:
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

            control_timestamp_ns = autotune_control_timestamp_ns(
                mission_running=mission_running,
                latest_clock_ns=latest_clock_ns,
                first_command_pending=driver.state.phase
                is (HoverPhase.WAIT_READY if hover_only else AutoTunePhase.WAIT_READY),
            )
            if control_timestamp_ns is not None and failure is None:
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


def _create_simulator_camera(
    camera_manager_type: Any,
    camera_type: Any,
    frame_source: Any,
    *,
    clock: SimulationClock,
    scenario_path: Path,
) -> Any:
    """Build the legacy camera against the fixed simulation camera contract."""

    try:
        scenario = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
        camera_config = scenario["camera"]
        marker_size_mm = scenario["payload_geometry"]["marker_size_mm"]
    except (OSError, yaml.YAMLError, TypeError, KeyError) as error:
        raise ValueError("simulator camera scenario is unreadable") from error
    expected_camera = {
        "width_px": 640,
        "height_px": 480,
        "update_rate_hz": 20,
        "horizontal_fov_rad": 0.6,
        "body_position_m": [0.0, 0.0, -0.1],
    }
    if camera_config != expected_camera or marker_size_mm != 100:
        raise ValueError(
            "simulator camera scenario does not match its calibration and mounting"
        )

    calibration_path = Path(__file__).with_name("gazebo_camera_calibration.json")
    mounting_path = Path(__file__).with_name("gazebo_camera_mounting.json")
    manager = camera_manager_type(
        frame_source=frame_source,
        calibration_path=calibration_path,
        clock=clock.read_timestamp_ns,
        max_exposure_age_ns=500_000_000,
    )
    return camera_type(
        marker_size_mm,
        manager=manager,
        mounting_path=mounting_path,
    )


def _load_qgc_timebase() -> Any:
    from drone import timebase

    return timebase


def _load_qgc_live_dependencies() -> Any:
    """Import live-only ROS, DroneKit, and nested listener dependencies lazily."""

    _enable_dronekit_python312_compatibility()
    import rclpy
    from drone.control.drone_control import DroneControl
    from drone.control.listener import (
        DroneKitQGCAckTransport,
        LiveComponentFactories,
        TelemetryStartupCollector,
        start_repl,
    )
    from drone.control.mission_supervisor import CommandRejected
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.duration import Duration
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from simulation_interfaces.msg import MissionEvent, RunState
    from simulation_interfaces.srv import PayloadCommand

    return SimpleNamespace(
        rclpy=rclpy,
        DroneControl=DroneControl,
        DroneKitQGCAckTransport=DroneKitQGCAckTransport,
        LiveComponentFactories=LiveComponentFactories,
        TelemetryStartupCollector=TelemetryStartupCollector,
        start_repl=start_repl,
        CommandRejected=CommandRejected,
        MutuallyExclusiveCallbackGroup=MutuallyExclusiveCallbackGroup,
        MultiThreadedExecutor=MultiThreadedExecutor,
        Node=Node,
        DurabilityPolicy=DurabilityPolicy,
        Duration=Duration,
        QoSProfile=QoSProfile,
        ReliabilityPolicy=ReliabilityPolicy,
        Clock=Clock,
        LaserScan=LaserScan,
        MissionEvent=MissionEvent,
        RunState=RunState,
        PayloadCommand=PayloadCommand,
        SignalHandlerOptions=SignalHandlerOptions,
        QgcRosLidarAdapter=QgcRosLidarAdapter,
        QgcFm2PayloadAdapter=QgcFm2PayloadAdapter,
        thread_factory=threading.Thread,
        wall_now=time.monotonic,
    )


class _QgcSignalLatch:
    """Record the first host signal without entering any synchronized API."""

    def __init__(self) -> None:
        self.reason: str | None = None

    def latch(self, reason: str) -> None:
        if self.reason is None:
            self.reason = reason


class _QgcStopCoordinator:
    """Route one latched host interruption to the current listener phase."""

    def __init__(self, clock: SimulationClock, monitoring_stop: threading.Event) -> None:
        self._clock = clock
        self._monitoring_stop = monitoring_stop
        self._lock = threading.RLock()
        self._reason: str | None = None
        self._runtime: object | None = None
        self._clock_stop_dispatched = False
        self._abort_dispatched = False
        self._monitoring_stop_dispatched = False
        self._unbound_stop_dispatched = False

    @staticmethod
    def _raise_failures(errors: list[BaseException]) -> None:
        if not errors:
            return
        if len(errors) == 1:
            raise errors[0]
        raise BaseExceptionGroup("QGC stop coordination failed", errors)

    @property
    def reason(self) -> str | None:
        with self._lock:
            return self._reason

    def bind_runtime(self, runtime: object) -> None:
        with self._lock:
            self._runtime = runtime
            reason = self._reason
            dispatch_abort = reason is not None and not self._abort_dispatched
            dispatch_monitoring_stop = (
                reason is not None and not self._monitoring_stop_dispatched
            )
            self._abort_dispatched |= dispatch_abort
            self._monitoring_stop_dispatched |= dispatch_monitoring_stop
        errors: list[BaseException] = []
        if dispatch_abort:
            try:
                runtime.request_abort(reason)  # type: ignore[attr-defined]
            except BaseException as error:
                errors.append(error)
        if dispatch_monitoring_stop:
            try:
                runtime.stop_monitoring()  # type: ignore[attr-defined]
            except BaseException as error:
                errors.append(error)
        self._raise_failures(errors)

    def request(self, reason: str, *, stop_clock: bool = False) -> None:
        with self._lock:
            if self._reason is None:
                self._reason = reason
            selected = self._reason
            runtime = self._runtime
            dispatch_clock_stop = stop_clock and not self._clock_stop_dispatched
            dispatch_abort = runtime is not None and not self._abort_dispatched
            dispatch_monitoring_stop = (
                runtime is not None and not self._monitoring_stop_dispatched
            )
            dispatch_unbound_stop = (
                runtime is None and not self._unbound_stop_dispatched
            )
            self._clock_stop_dispatched |= dispatch_clock_stop
            self._abort_dispatched |= dispatch_abort
            self._monitoring_stop_dispatched |= dispatch_monitoring_stop
            self._unbound_stop_dispatched |= dispatch_unbound_stop
        errors: list[BaseException] = []
        if dispatch_clock_stop:
            try:
                self._clock.stop(selected)
            except BaseException as error:
                errors.append(error)
        if runtime is not None:
            if dispatch_abort:
                try:
                    runtime.request_abort(selected)  # type: ignore[attr-defined]
                except BaseException as error:
                    errors.append(error)
            if dispatch_monitoring_stop:
                try:
                    runtime.stop_monitoring()  # type: ignore[attr-defined]
                except BaseException as error:
                    errors.append(error)
        elif dispatch_unbound_stop:
            try:
                self._monitoring_stop.set()
            except BaseException as error:
                errors.append(error)
        self._raise_failures(errors)


@dataclass(frozen=True)
class _QgcHostCleanup:
    first_error: BaseException | None
    diagnostics: tuple[str, ...]
    confirmed: bool
    protocol_safe_to_close: bool


def _exception_detail(error: BaseException) -> str:
    try:
        return str(error) or type(error).__name__
    except BaseException:
        return type(error).__name__


class _Comp2026QgcRosHost:
    """Lazy ROS producers and accepted nested factories for one QGC attempt."""

    _MISSION_EVENT_TOPIC = "/simulation/mission_events"
    _MISSION_EVENT_TYPE = "simulation_interfaces/msg/MissionEvent"
    _SCOREKEEPER_NODE = "drone_sim_scorekeeper"
    _ROSBAG_NODE_PREFIX = "rosbag2_recorder_"

    def __init__(
        self,
        config: RuntimeConfig,
        projection: object,
        clock: SimulationClock,
        lifecycle: CompanionLifecycle,
        dependencies: object,
        stop: _QgcStopCoordinator,
        monitoring_stop: threading.Event,
        protocol: _ProductionProtocol,
    ) -> None:
        self.config = config
        self.projection = projection
        self.runtime_config = projection.runtime_configuration  # type: ignore[attr-defined]
        self.clock = clock
        self.lifecycle = lifecycle
        self.dependencies = dependencies
        self.stop = stop
        self.monitoring_stop = monitoring_stop
        self.protocol = protocol
        self.admission = threading.Event()
        self._callbacks_active = threading.Event()
        self._callbacks_active.set()
        self._lock = threading.RLock()
        self._fatal_policy_lock = threading.Lock()
        self._first_error: BaseException | None = None
        self._fatal_error: BaseException | None = None
        self._fatal_context: str | None = None
        self._fatal_reason: str | None = None
        self._fatal_stop_clock = False
        self._fatal_diagnostics: list[str] = []
        self._ros_started = False
        self._rclpy_initialized = False
        self._mission_running = False
        self.node: object | None = None
        self.executor: object | None = None
        self.executor_thread: threading.Thread | None = None
        self._executor_thread_started = False
        self.control_thread: threading.Thread | None = None
        self._control_thread_started = False
        self._control_stop = threading.Event()
        self.subscriptions: list[object] = []
        self.mission_publisher: object | None = None
        self.mission_event_emitter: MissionEventEmitter | None = None
        self.payload_client: _RosPayloadClient | None = None
        self.lidar: RosLidar | None = None
        self.range_ingress = QgcRangeIngress()
        self._controller: object | None = None
        self._controller_handed_off = False
        heartbeat_freshness = projection.flight_profile.freshness_bounds["heartbeat"]  # type: ignore[attr-defined]
        self.heartbeat_freshness_s = min(
            float(self.runtime_config.connection.heartbeat_timeout_s),
            float(heartbeat_freshness),
        )
        if not math.isfinite(self.heartbeat_freshness_s) or self.heartbeat_freshness_s <= 0:
            raise ValueError("validated heartbeat freshness must be finite and positive")
        self._overall_wall_deadline = dependencies.wall_now() + config.max_wall_seconds

    @property
    def first_error(self) -> BaseException | None:
        with self._lock:
            return self._first_error

    def _record_fatal_error(
        self, error: BaseException, context: str, *, stop_clock: bool = True
    ) -> None:
        with self._fatal_policy_lock:
            self._fatal_stop_clock |= stop_clock
        with self._lock:
            if self._fatal_error is None:
                self._fatal_error = error
                self._fatal_context = context
                if self._first_error is None:
                    self._first_error = error
                self._fatal_reason = (
                    f"{self._fatal_context}: {_exception_detail(self._fatal_error)}"
                )
            elif self._fatal_reason is None:
                return
            reason = self._fatal_reason
        with self._fatal_policy_lock:
            selected_stop_clock = self._fatal_stop_clock
        try:
            self.stop.request(reason, stop_clock=selected_stop_clock)
        except BaseException as stop_error:
            self._record_fatal_diagnostics(stop_error)

    def _record_fatal_diagnostics(self, error: BaseException) -> None:
        if isinstance(error, BaseExceptionGroup):
            for nested in error.exceptions:
                self._record_fatal_diagnostics(nested)
            return
        with self._lock:
            self._fatal_diagnostics.append(_exception_detail(error))

    def _guard_input(self, name: str, operation: Callable[[], None]) -> bool:
        if not self._callbacks_active.is_set():
            return False
        try:
            operation()
        except BaseException as error:
            self._record_fatal_error(error, f"{name} input failed")
            return False
        return True

    def _qos(self, depth: int, *, transient: bool = False) -> object:
        deps = self.dependencies
        return deps.QoSProfile(
            depth=depth,
            reliability=deps.ReliabilityPolicy.RELIABLE,
            durability=(
                deps.DurabilityPolicy.TRANSIENT_LOCAL
                if transient
                else deps.DurabilityPolicy.VOLATILE
            ),
        )

    def _state_callback(self, message: object) -> None:
        if not self._callbacks_active.is_set() or getattr(message, "run_id", None) != self.config.run_id:
            return
        state = getattr(message, "state", None)
        if state == self.dependencies.RunState.RUNNING:
            self._mission_running = True
        elif state == self.dependencies.RunState.FINALIZING:
            try:
                self.stop.request("orchestration finalization")
            except BaseException as error:
                self._record_fatal_error(
                    error,
                    "orchestration finalization stop coordination failed",
                    stop_clock=True,
                )

    def _clock_callback(self, message: object) -> None:
        if not self._mission_running:
            return
        self._guard_input(
            "clock", lambda: self.clock.accept(stamp_ns(message.clock))  # type: ignore[attr-defined]
        )

    def _range_callback(self, message: object) -> None:
        if not self._mission_running or self.lidar is None:
            return
        self.range_ingress.accept(
            lambda: accept_comp2026_range_input(
                lidar=self.lidar,  # type: ignore[arg-type]
                message=message,
                guard_input=self._guard_input,
            )
        )

    def _publish_mission_event(self, record: MissionEventRecord) -> None:
        if self.mission_publisher is None:
            raise RuntimeError("mission event publisher is unavailable")
        if not self._mission_event_consumers_ready():
            raise RuntimeError(
                "mission event publisher requires both scorekeeper and rosbag subscribers"
            )
        message = self.dependencies.MissionEvent()
        message.run_id = record.run_id
        seconds, nanoseconds = divmod(record.sim_timestamp_ns, 1_000_000_000)
        message.sim_timestamp.sec = seconds
        message.sim_timestamp.nanosec = nanoseconds
        message.event_id = record.event_id
        message.phase = record.phase
        message.state = record.state
        message.detail = record.detail
        self.mission_publisher.publish(message)  # type: ignore[attr-defined]

    def _mission_event_consumers_ready(self) -> bool:
        if self.node is None:
            return False
        deps = self.dependencies
        valid_names = {
            endpoint.node_name
            for endpoint in self.node.get_subscriptions_info_by_topic(  # type: ignore[attr-defined]
                self._MISSION_EVENT_TOPIC
            )
            if endpoint.node_namespace == "/"
            and endpoint.topic_type == self._MISSION_EVENT_TYPE
            and endpoint.qos_profile.reliability == deps.ReliabilityPolicy.RELIABLE
            and endpoint.qos_profile.durability
            == deps.DurabilityPolicy.TRANSIENT_LOCAL
        }
        return self._SCOREKEEPER_NODE in valid_names and any(
            name.startswith(self._ROSBAG_NODE_PREFIX) for name in valid_names
        )

    def observe_phase(self, phase: str, state: str) -> None:
        emitter = self.mission_event_emitter
        if emitter is None:
            raise RuntimeError("mission event emitter is unavailable")
        try:
            emitter(phase, state)
        except BaseException as error:
            self._record_fatal_error(
                error,
                "mission event publication failed",
                stop_clock=False,
            )
            raise

    def _start_ros(self) -> None:
        if self._ros_started:
            return
        deps = self.dependencies
        deps.rclpy.init(signal_handler_options=deps.SignalHandlerOptions.NO)
        self._rclpy_initialized = True
        self.node = deps.Node("drone_sim_companion", parameter_overrides=[])
        state_group = deps.MutuallyExclusiveCallbackGroup()
        range_group = deps.MutuallyExclusiveCallbackGroup()
        service_group = deps.MutuallyExclusiveCallbackGroup()
        self.mission_publisher = self.node.create_publisher(
            deps.MissionEvent,
            self._MISSION_EVENT_TOPIC,
            self._qos(100, transient=True),
        )
        self.mission_event_emitter = MissionEventEmitter(
            self.config.run_id,
            self.clock,
            self._publish_mission_event,
        )
        raw_payload_client = self.node.create_client(
            deps.PayloadCommand,
            "/simulation/payload_command",
            callback_group=service_group,
        )
        self.payload_client = _RosPayloadClient(
            raw_payload_client,
            deps.PayloadCommand,
            response_timeout_seconds=self.projection.payload_delay_wall_timeout_s,  # type: ignore[attr-defined]
        )
        self.lidar = create_comp2026_lidar(self.clock)
        self.subscriptions = [
            self.node.create_subscription(
                deps.RunState,
                "/simulation/run_state",
                self._state_callback,
                self._qos(1, transient=True),
                callback_group=state_group,
            ),
            self.node.create_subscription(
                deps.Clock,
                "/clock",
                self._clock_callback,
                self._qos(1),
                callback_group=state_group,
            ),
            self.node.create_subscription(
                deps.LaserScan,
                "/competition/range/downward",
                self._range_callback,
                self._qos(1),
                callback_group=range_group,
            ),
        ]
        self.executor = deps.MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.node)
        executor_entered = threading.Event()

        def spin() -> None:
            executor_entered.set()
            try:
                self.executor.spin()  # type: ignore[attr-defined]
            except BaseException as error:
                self._record_fatal_error(error, "ROS executor failed")
            else:
                if self._callbacks_active.is_set():
                    error = RuntimeError("ROS executor stopped before host cleanup")
                    self._record_fatal_error(error, "ROS executor failed")

        self.executor_thread = deps.thread_factory(
            target=spin,
            name="companion-qgc-ros-executor",
            daemon=True,
        )
        self.executor_thread.start()
        self._executor_thread_started = True
        if not executor_entered.wait(min(self.config.startup_timeout_seconds, 5.0)):
            raise TimeoutError("ROS executor did not start before vehicle connection")
        self._ros_started = True

    def start_control_monitor(self, signal_reason: Callable[[], str | None]) -> None:
        if self._control_thread_started:
            return
        control_checked = threading.Event()

        def monitor_host_control() -> None:
            signal_dispatched = False
            finalize_dispatched = False

            def request_stop(
                reason: str, *, stop_clock: bool, failure_context: str
            ) -> bool:
                try:
                    self.stop.request(reason, stop_clock=stop_clock)
                except BaseException as error:
                    self._record_fatal_error(
                        error,
                        failure_context,
                        stop_clock=True,
                    )
                    return False
                return True

            while not self._control_stop.is_set():
                try:
                    pending_signal = signal_reason()
                    finalize_requested = self.protocol.read_finalize_request() is not None
                    wall_expired = (
                        self.dependencies.wall_now() >= self._overall_wall_deadline
                    )
                except BaseException as error:
                    self._record_fatal_error(error, "host control monitor failed")
                    control_checked.set()
                    return
                control_checked.set()
                if pending_signal is not None and not signal_dispatched:
                    if not request_stop(
                        pending_signal,
                        stop_clock=False,
                        failure_context="host signal stop coordination failed",
                    ):
                        return
                    signal_dispatched = True
                if finalize_requested and not finalize_dispatched:
                    if not request_stop(
                        "orchestration finalize request",
                        stop_clock=False,
                        failure_context="host finalize stop coordination failed",
                    ):
                        return
                    finalize_dispatched = True
                if wall_expired:
                    request_stop(
                        "companion exceeded the overall run wall failsafe",
                        stop_clock=True,
                        failure_context="host wall failsafe stop coordination failed",
                    )
                    return
                self._control_stop.wait(0.02)

        self.control_thread = threading.Thread(
            target=monitor_host_control,
            name="companion-qgc-host-control",
            daemon=True,
        )
        self.control_thread.start()
        self._control_thread_started = True
        if not control_checked.wait(min(self.config.startup_timeout_seconds, 5.0)):
            raise TimeoutError("host control monitor did not start")

    def controller_factory(
        self, *, config: object, flight_state: object, permission_guard: object, decoders: object
    ) -> object:
        self._start_ros()
        if self.stop.reason is not None or self.first_error is not None:
            raise RuntimeError(f"QGC startup stopped before vehicle connection: {self.stop.reason}")
        connection = config.connection  # type: ignore[attr-defined]
        controller = self.dependencies.DroneControl(
            connection.endpoint,
            source_identity=connection.source_identity,
            flight_controller_target=connection.target_identity,
            wait_ready=connection.wait_ready,
            heartbeat_timeout=connection.heartbeat_timeout_s,
            flight_state=flight_state,
            permission_guard=permission_guard,
            heartbeat_mode_decoder=decoders.heartbeat_mode_decoder,
            rc_health_decoder=decoders.rc_health_decoder,
            sys_status_observer=decoders.observe_sys_status,
            failsafe_decoders={"HEARTBEAT": decoders.heartbeat_failsafe_decoder},
            mission_home_check=config.operating_site.mission_home_check,  # type: ignore[attr-defined]
            fc_home_position_tolerance_m=config.fc_home_position_tolerance_m,  # type: ignore[attr-defined]
            fc_home_altitude_tolerance_m=config.fc_home_altitude_tolerance_m,  # type: ignore[attr-defined]
            clearance_calibration=config.clearance_calibration,  # type: ignore[attr-defined]
            release_stability_config=config.release_stability,  # type: ignore[attr-defined]
            home_request_timeout_s=config.telemetry.home_request_timeout_s,  # type: ignore[attr-defined]
            telemetry_poll_interval_s=config.telemetry.poll_interval_s,  # type: ignore[attr-defined]
            guided_output_delivery_callback=self._guided_output_delivered,
        )
        self._controller = controller
        self.lifecycle.mark_transport_ready()
        self._controller_handed_off = True
        return controller

    def _guided_output_delivered(self) -> None:
        self.lifecycle.observe_command_delivery(
            CommandKind.SET_GUIDED,
            self.clock.read_timestamp_ns(),
        )

    def ack_transport_factory(self, *, controller: object, config: object) -> object:
        return self.dependencies.DroneKitQGCAckTransport(
            controller.vehicle,  # type: ignore[attr-defined]
            expected_source=config.connection.source_identity,  # type: ignore[attr-defined]
            wire_protocol=config.connection.wire_protocol,  # type: ignore[attr-defined]
        )

    def telemetry_collector_factory(
        self, *, controller: object, profile: object, config: object
    ) -> object:
        return self.dependencies.TelemetryStartupCollector(
            controller=controller,
            flight_profile=profile,
            policy=config.telemetry,  # type: ignore[attr-defined]
            autopilot_version=config.autopilot_version,  # type: ignore[attr-defined]
        )

    def lidar_factory(self, *, config: object) -> object:
        del config
        if self.lidar is None:
            raise RuntimeError("ROS LiDAR was not established before listener construction")
        return self.dependencies.QgcRosLidarAdapter(
            self.lidar,
            range_ingress=self.range_ingress,
        )

    def dropper_factory(self, *, config: object, permission: object) -> object:
        del config
        if self.payload_client is None:
            raise RuntimeError("ROS payload client was not established before FM1 admission")
        dropper = PayloadDropper(
            self.config.run_id,
            2,
            self.payload_client,
            self.clock,
            permission=permission,
            delay_wall_timeout_seconds=self.projection.payload_delay_wall_timeout_s,  # type: ignore[attr-defined]
        )
        return self.dependencies.QgcFm2PayloadAdapter(dropper, aruco_id=2)

    @staticmethod
    def camera_factory(**_kwargs: object) -> object:
        raise RuntimeError("FM3 camera construction is disabled for this composition")

    def factories(self) -> object:
        return self.dependencies.LiveComponentFactories(
            backend=self.runtime_config.components.backend,
            controller_factory=self.controller_factory,
            ack_transport_factory=self.ack_transport_factory,
            telemetry_collector_factory=self.telemetry_collector_factory,
            lidar_factory=self.lidar_factory,
            dropper_factory=self.dropper_factory,
            camera_factory=self.camera_factory,
            supports_attachment=False,
            telemetry_startup_mode="staged_simulation",
        )

    def require_admission_open(self) -> None:
        if not self.admission.is_set():
            raise self.dependencies.CommandRejected(
                "host mission-ready admission is closed"
            )

    def listener_ready(self, runtime: object) -> None:
        self.stop.bind_runtime(runtime)
        if self.stop.reason is not None or self.first_error is not None:
            raise RuntimeError(f"QGC startup stopped before mission-ready: {self.stop.reason}")
        if not self._mission_running or self.clock.ready is not True:
            raise RuntimeError(
                "matching RUNNING state and public clock are required before mission-ready"
            )
        if (
            self.mission_publisher is None
            or not self._mission_event_consumers_ready()
        ):
            raise RuntimeError(
                "mission event publisher requires both scorekeeper and rosbag subscribers "
                "before mission-ready"
            )
        vehicle = runtime.controller.vehicle  # type: ignore[attr-defined]
        heartbeat = getattr(vehicle, "last_heartbeat", None)
        heartbeat_ready = (
            isinstance(heartbeat, (int, float))
            and not isinstance(heartbeat, bool)
            and math.isfinite(heartbeat)
            and 0.0 <= heartbeat <= self.heartbeat_freshness_s
        )
        armable = getattr(vehicle, "is_armable", None) is True
        if not heartbeat_ready or not armable:
            raise RuntimeError(
                "connected vehicle lacks a fresh heartbeat or literal armable state"
            )
        self.lifecycle.observe_mission_readiness(
            heartbeat_observed=heartbeat_ready,
            prearm_checks_healthy=armable,
        )
        self.admission.set()

    def close(self) -> _QgcHostCleanup:
        """Bound ROS callbacks and producers after nested cleanup has completed."""

        self._callbacks_active.clear()
        errors: list[BaseException] = []

        def record(error: BaseException) -> None:
            errors.append(error)

        if self.mission_event_emitter is not None:
            try:
                self.mission_event_emitter.stop("QGC host cleanup")
            except BaseException as error:
                record(error)

        if self.mission_publisher is not None:
            try:
                if not self._mission_event_consumers_ready():
                    raise RuntimeError(
                        "mission event delivery cannot be confirmed without both scorekeeper "
                        "and rosbag subscribers"
                    )
                acknowledged = self.mission_publisher.wait_for_all_acked(  # type: ignore[attr-defined]
                    timeout=self.dependencies.Duration(
                        seconds=self.runtime_config.cleanup_timeout_s
                    )
                )
                if acknowledged is not True:
                    raise RuntimeError(
                        "mission event delivery was not acknowledged before cleanup"
                    )
                if not self._mission_event_consumers_ready():
                    raise RuntimeError(
                        "mission event delivery cannot be confirmed after acknowledgement "
                        "without both scorekeeper and rosbag subscribers"
                    )
            except BaseException as error:
                record(error)

        self._control_stop.set()
        control_stopped = True
        if self.control_thread is not None and self._control_thread_started:
            try:
                self.control_thread.join(self.runtime_config.cleanup_timeout_s)
                control_stopped = not self.control_thread.is_alive()
                if not control_stopped:
                    record(RuntimeError("host control monitor did not stop"))
            except BaseException as error:
                control_stopped = False
                record(error)
        if self.executor is not None:
            try:
                stopped = self.executor.shutdown(
                    timeout_sec=self.runtime_config.cleanup_timeout_s
                )
                if stopped is False:
                    raise RuntimeError("ROS executor shutdown was unconfirmed")
            except BaseException as error:
                record(error)
        executor_stopped = True
        if self.executor_thread is not None and self._executor_thread_started:
            try:
                self.executor_thread.join(self.runtime_config.cleanup_timeout_s)
                executor_stopped = not self.executor_thread.is_alive()
                if not executor_stopped:
                    record(RuntimeError("ROS executor thread did not stop"))
            except BaseException as error:
                executor_stopped = False
                record(error)
        if self.node is not None:
            for subscription in self.subscriptions:
                try:
                    self.node.destroy_subscription(subscription)
                except BaseException as error:
                    record(error)
            if self.mission_publisher is not None:
                try:
                    self.node.destroy_publisher(self.mission_publisher)
                except BaseException as error:
                    record(error)
            try:
                self.node.destroy_node()
            except BaseException as error:
                record(error)
        if self.payload_client is not None:
            try:
                self.payload_client.stop()
            except BaseException as error:
                record(error)
        if self._controller is not None and not self._controller_handed_off:
            vehicle = getattr(self._controller, "vehicle", None)
            close = getattr(vehicle, "close", None)
            if callable(close):
                close_errors: list[BaseException] = []

                def close_vehicle() -> None:
                    try:
                        close()
                    except BaseException as error:
                        close_errors.append(error)

                closer = threading.Thread(
                    target=close_vehicle,
                    name="companion-qgc-unhanded-controller-close",
                    daemon=True,
                )
                try:
                    closer.start()
                    closer.join(self.runtime_config.cleanup_timeout_s)
                    if closer.is_alive():
                        record(RuntimeError("unhanded controller close did not stop"))
                    elif close_errors:
                        record(close_errors[0])
                except BaseException as error:
                    record(error)
        if self._rclpy_initialized:
            try:
                self.dependencies.rclpy.shutdown()
            except BaseException as error:
                record(error)
        self.clock.stop("QGC host cleanup")
        with self._lock:
            fatal_diagnostics = tuple(self._fatal_diagnostics)
        return _QgcHostCleanup(
            first_error=errors[0] if errors else None,
            diagnostics=fatal_diagnostics
            + tuple(_exception_detail(error) for error in errors[1:]),
            confirmed=not errors and control_stopped and executor_stopped,
            protocol_safe_to_close=control_stopped,
        )


def _qgc_cleanup_failure(result: object) -> str | None:
    cleanup = getattr(result, "cleanup_report", None)
    if cleanup is None:
        return "nested listener returned without a cleanup report"
    required = (
        "lidar_stopped",
        "lidar_cleanup_completed",
        "camera_stopped",
        "recordings_completed",
        "payload_closed",
        "vehicle_closed",
    )
    if any(getattr(cleanup, name, None) is not True for name in required):
        return "nested listener cleanup was incomplete or unconfirmed"
    diagnostics = getattr(cleanup, "diagnostics", ())
    if diagnostics:
        return f"nested listener cleanup reported diagnostics: {diagnostics}"
    return None


def _run_comp2026(config: RuntimeConfig) -> int:
    """Host the guarded QGC listener without issuing an automatic command."""

    protocol = _ProductionProtocol(config)
    lifecycle = CompanionLifecycle(run_id=config.run_id, protocol=protocol, stream=sys.stdout)
    clock = SimulationClock()
    monitoring_stop = threading.Event()
    stop = _QgcStopCoordinator(clock, monitoring_stop)
    signal_latch = _QgcSignalLatch()
    host: _Comp2026QgcRosHost | None = None
    result: object | None = None
    primary_error: BaseException | None = None
    cleanup_diagnostics: list[str] = []
    terminal_timestamp_ns: int | None = None
    cleanup_confirmed = True
    protocol_safe_to_close = True
    timebase = _load_qgc_timebase()
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    previous_sigint = signal.getsignal(signal.SIGINT)

    def retain(error: BaseException) -> None:
        nonlocal primary_error
        if primary_error is None:
            primary_error = error
        else:
            cleanup_diagnostics.append(_exception_detail(error))

    def report_secondary(message: str) -> None:
        try:
            lifecycle.emit(
                "cleanup_diagnostic",
                terminal_timestamp_ns,
                {"message": message},
            )
        except BaseException:
            print(f"companion cleanup diagnostic: {message}", file=sys.stderr)

    def interrupt(_signum: int, _frame: object) -> None:
        signal_latch.latch("host termination signal")

    try:
        try:
            lifecycle.emit("starting", None, {"mavlink_endpoint": config.mavlink_endpoint})
            signal.signal(signal.SIGTERM, interrupt)
            signal.signal(signal.SIGINT, interrupt)
            with timebase.configured(clock):
                try:
                    projected = project_qgc_runtime(
                        config,
                        epoch_seconds=timebase.epoch(),
                    )
                    if signal_latch.reason is not None:
                        stop.request(signal_latch.reason)
                        raise RuntimeError(signal_latch.reason)
                    dependencies = _load_qgc_live_dependencies()
                    host = _Comp2026QgcRosHost(
                        config,
                        projected,
                        clock,
                        lifecycle,
                        dependencies,
                        stop,
                        monitoring_stop,
                        protocol,
                    )
                    host.start_control_monitor(lambda: signal_latch.reason)
                    if signal_latch.reason is not None:
                        stop.request(signal_latch.reason)
                        raise RuntimeError(signal_latch.reason)
                    result = dependencies.start_repl(
                        projected.validated_listener_artifacts,
                        projected.runtime_configuration,
                        factories=host.factories(),
                        diagnostics=lambda message: lifecycle.emit(
                            "listener_diagnostic",
                            clock.timestamp_ns,
                            {"message": message},
                        ),
                        monitoring_stop=monitoring_stop,
                        startup_admission_check=host.require_admission_open,
                        on_listener_ready=host.listener_ready,
                        manage_signals=False,
                        phase_observer=host.observe_phase,
                    )
                    if signal_latch.reason is not None:
                        stop.request(signal_latch.reason)
                except BaseException as error:
                    if stop.reason is not None:
                        retain(RuntimeError(stop.reason))
                    retain(error)

                terminal_timestamp_ns = clock.timestamp_ns
                if host is not None:
                    host_cleanup = host.close()
                    cleanup_confirmed = host_cleanup.confirmed
                    protocol_safe_to_close = host_cleanup.protocol_safe_to_close

                    # Callback ingress is closed and producers are joined before
                    # this final outcome snapshot.
                    if host.first_error is not None:
                        retain(host.first_error)
                    if stop.reason is not None:
                        retain(RuntimeError(stop.reason))
                    if host_cleanup.first_error is not None:
                        retain(host_cleanup.first_error)
                    cleanup_diagnostics.extend(host_cleanup.diagnostics)

                if result is None:
                    if host is not None:
                        cleanup_confirmed = False
                    if primary_error is None:
                        retain(RuntimeError("QGC listener returned without a terminal result"))
                else:
                    cleanup_failure = _qgc_cleanup_failure(result)
                    if cleanup_failure is not None:
                        cleanup_confirmed = False
                        retain(RuntimeError(cleanup_failure))

                if primary_error is None and result is not None:
                    terminal_tuple = (
                        getattr(result, "mission_result", None),
                        getattr(result, "recovery_outcome", None),
                        getattr(result, "monitoring_exit_reason", None),
                    )
                    if terminal_tuple != ("SUCCEEDED", "HOME_LANDED", "NOT_REQUIRED"):
                        retain(
                            RuntimeError(
                                "QGC listener ended without mission success: "
                                f"mission={getattr(result, 'mission_result', None)}, "
                                f"recovery={getattr(result, 'recovery_outcome', None)}, "
                                "monitoring="
                                f"{getattr(result, 'monitoring_exit_reason', None)}"
                            )
                        )

                for diagnostic in tuple(cleanup_diagnostics):
                    report_secondary(diagnostic)

                try:
                    if primary_error is None:
                        lifecycle.observe_terminal(
                            MissionState(
                                MissionPhase.LANDED,
                                last_timestamp_ns=terminal_timestamp_ns or 0,
                            )
                        )
                    else:
                        lifecycle.observe_terminal(
                            MissionState(
                                MissionPhase.FAILED,
                                last_timestamp_ns=terminal_timestamp_ns or 0,
                                failure_reason=_exception_detail(primary_error),
                            )
                        )
                except BaseException as error:
                    retain(error)

                if primary_error is not None:
                    reason = _exception_detail(primary_error)
                    try:
                        protocol.write_status(
                            RuntimeFailureStatus(
                                config.run_id,
                                "companion",
                                reason,
                                ("logs/docker/companion.log.partial",),
                            )
                        )
                    except BaseException as error:
                        retain(error)

                if cleanup_confirmed:
                    try:
                        lifecycle.finalize(terminal_timestamp_ns)
                    except BaseException as error:
                        retain(error)
        except BaseException as error:
            retain(error)
    finally:
        try:
            signal.signal(signal.SIGTERM, previous_sigterm)
            signal.signal(signal.SIGINT, previous_sigint)
        except BaseException as error:
            retain(error)
        if protocol_safe_to_close:
            try:
                protocol.close()
            except BaseException as error:
                retain(error)
    return 0 if primary_error is None else 1
def main() -> int:
    config = RuntimeConfig.from_environment(os.environ)
    if config.mission == "configured":
        from .configured_runtime import run_configured

        return run_configured(config)
    if config.mission == "controlled_descent":
        return _run_controlled_descent(config)
    if config.mission in {"autotune_roll", "hover_roll"}:
        return _run_autotune_roll(config)
    return _run_comp2026(config)


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "RuntimeConfig",
    "accept_comp2026_range_input",
    "connect_mavlink",
    "create_comp2026_lidar",
    "first_heartbeat_wall_failure",
    "main",
    "process_runtime_telemetry",
    "quiesce_comp2026_runtime",
    "stamp_ns",
]
