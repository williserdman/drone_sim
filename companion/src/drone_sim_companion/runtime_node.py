"""ROS 2 and PyMAVLink process adapter for the controlled descent mission."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from collections.abc import Callable
from typing import Any, Mapping
from uuid import UUID, uuid4

from artifacts.runtime_protocol import RuntimeProtocol

from .controller import MissionController, process_telemetry
from .lifecycle import CompanionLifecycle
from .mavlink_adapter import MavlinkAdapter
from .mission import MissionPhase, MissionState


@dataclass(frozen=True)
class RuntimeConfig:
    run_id: str
    run_directory: Path
    mavlink_endpoint: str = "tcp:ardupilot-sitl:5760"
    startup_timeout_seconds: float = 60.0
    max_wall_seconds: float = 3600.0

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
        timeout = override if timeout_override is not None else float(startup_wall_seconds)
        endpoint = environment.get("SIM_MAVLINK_ENDPOINT", "tcp:ardupilot-sitl:5760")
        if endpoint != "tcp:ardupilot-sitl:5760":
            raise ValueError("production MAVLink endpoint must be tcp:ardupilot-sitl:5760")
        return cls(run_id, run_directory, endpoint, timeout, float(max_wall_seconds))


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


def _atomic_status(path: Path, document: dict[str, object]) -> None:
    payload = (
        json.dumps(document, allow_nan=False, sort_keys=True, separators=(",", ":")) + "\n"
    ).encode()
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if path.read_bytes() == payload:
            return
        raise RuntimeError(f"status {path.name} conflicts with its durable value")
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    descriptor: int | None = None
    try:
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0),
            0o644,
        )
        os.write(descriptor, payload)
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.replace(temporary, path)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


class _ProductionProtocol:
    """Use the shared control/quiescence protocol plus the new vertical statuses."""

    def __init__(self, config: RuntimeConfig) -> None:
        self._runtime = RuntimeProtocol(config.run_directory, config.run_id)
        self._status = config.run_directory / ".status"

    def write_status(self, name: str, document: dict[str, object]) -> None:
        if name not in {"companion-ready", "mission-finished"}:
            raise ValueError("companion does not own that status")
        _atomic_status(self._status / f"{name}.json", document)

    def write_quiescence(self, module: str) -> Any:
        return self._runtime.write_quiescence(module)

    def read_finalize_request(self) -> dict[str, Any] | None:
        return self._runtime.read_finalize_request()

    def close(self) -> None:
        self._runtime.close()


def main() -> int:
    config = RuntimeConfig.from_environment(os.environ)
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
    controller = MissionController(vehicle, lifecycle.emit)
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
            if failure is None:
                try:
                    for _ in range(100):
                        telemetry = vehicle.poll(
                            latest_clock_ns if latest_clock_ns is not None else 0
                        )
                        if telemetry is None:
                            break
                        process_telemetry(
                            controller,
                            telemetry,
                            mission_running=mission_running,
                        )
                except Exception as error:
                    failure = f"MAVLink processing failed: {error}"
            if controller.ready and not telemetry_requested and failure is None:
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
                heartbeat_observed=controller.ready,
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


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "RuntimeConfig",
    "connect_mavlink",
    "first_heartbeat_wall_failure",
    "main",
    "stamp_ns",
]
