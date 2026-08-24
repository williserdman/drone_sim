"""ROS 2 and PyMAVLink process adapter for the controlled descent mission."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import sys
import time
from collections.abc import Callable
from typing import Any, Mapping
from uuid import UUID, uuid4

from artifacts.runtime_protocol import RuntimeProtocol

from .controller import MissionController
from .lifecycle import CompanionLifecycle
from .mavlink_adapter import MavlinkAdapter
from .mission import MissionPhase, MissionState


@dataclass(frozen=True)
class RuntimeConfig:
    run_id: str
    run_directory: Path
    mavlink_endpoint: str = "tcp:ardupilot-sitl:5760"
    startup_timeout_seconds: float = 60.0

    @classmethod
    def from_environment(cls, environment: Mapping[str, str]) -> "RuntimeConfig":
        run_id = environment["SIM_RUN_ID"]
        try:
            parsed = UUID(run_id)
        except (ValueError, TypeError, AttributeError) as error:
            raise ValueError("SIM_RUN_ID must be a canonical UUID") from error
        if str(parsed) != run_id:
            raise ValueError("SIM_RUN_ID must be a canonical UUID")
        timeout = float(environment.get("SIM_COMPANION_STARTUP_TIMEOUT_SECONDS", "60"))
        if timeout <= 0:
            raise ValueError("SIM_COMPANION_STARTUP_TIMEOUT_SECONDS must be positive")
        endpoint = environment.get("SIM_MAVLINK_ENDPOINT", "tcp:ardupilot-sitl:5760")
        if endpoint != "tcp:ardupilot-sitl:5760":
            raise ValueError("production MAVLink endpoint must be tcp:ardupilot-sitl:5760")
        return cls(run_id, Path(environment["SIM_RUN_DIRECTORY"]), endpoint, timeout)


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
    try:
        connection = connect_mavlink(
            mavutil.mavlink_connection,
            config.mavlink_endpoint,
            deadline=started + config.startup_timeout_seconds,
        )
    except TimeoutError as error:
        lifecycle.observe_terminal(
            MissionState(MissionPhase.FAILED, last_timestamp_ns=0, failure_reason=str(error))
        )
        lifecycle.finalize(None)
        protocol.close()
        return 1
    vehicle = MavlinkAdapter(connection, mavutil)
    controller = MissionController(vehicle, lifecycle.emit)
    rclpy.init()
    node = Node("drone_sim_companion", parameter_overrides=[])
    latest_clock_ns: int | None = None
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
        nonlocal finalizing
        if message.run_id == config.run_id and message.state == RunState.FINALIZING:
            finalizing = True

    node.create_subscription(
        Clock,
        "/clock",
        clock_callback,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
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
            if latest_clock_ns is not None and failure is None:
                try:
                    for _ in range(100):
                        telemetry = vehicle.poll(latest_clock_ns)
                        if telemetry is None:
                            break
                        controller.consume(telemetry)
                except Exception as error:
                    failure = f"MAVLink processing failed: {error}"
            if controller.ready and not telemetry_requested and failure is None:
                vehicle.request_telemetry(rate_hz=10)
                lifecycle.mark_ready(latest_clock_ns or 0)
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
            if not controller.ready and time.monotonic() - started >= config.startup_timeout_seconds:
                failure = "MAVLink heartbeat was unavailable before the infrastructure deadline"
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


__all__ = ["RuntimeConfig", "connect_mavlink", "main", "stamp_ns"]
