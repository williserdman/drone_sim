"""Bounded external operator for the configured operator-wait mission."""

from __future__ import annotations

from enum import Enum
import json
import os
from pathlib import Path
import signal
import time
from typing import Any, Callable

from .mavlink_adapter import MavlinkAdapter
from .mission import CommandKind, Telemetry


def connect_operator_mavlink(
    factory: Callable[..., Any],
    *,
    deadline: float,
    now: Callable[[], float] = time.monotonic,
    pause: Callable[[float], None] = time.sleep,
) -> Any:
    """Retry the private endpoint while retaining the operator's distinct sysid."""
    from .runtime_node import connect_mavlink

    def operator_factory(endpoint: str, **keywords: object) -> Any:
        keywords["source_system"] = 253
        return factory(endpoint, **keywords)

    return connect_mavlink(
        operator_factory,
        "tcp:ardupilot-sitl:5762",
        deadline=deadline,
        now=now,
        pause=pause,
    )


class OperatorPhase(str, Enum):
    WAIT_READY = "WAIT_READY"
    WAIT_GUIDED_ACK = "WAIT_GUIDED_ACK"
    WAIT_GUIDED = "WAIT_GUIDED"
    WAIT_ARM_ACK = "WAIT_ARM_ACK"
    WAIT_ARMED = "WAIT_ARMED"
    PASSIVE = "PASSIVE"
    FAILED = "FAILED"
    FINALIZED = "FINALIZED"


class OperatorWaitActor:
    """Order two operator commands from run-scoped, simulation-time evidence."""

    HEARTBEAT_MAX_AGE_NS = 500_000_000

    def __init__(
        self,
        vehicle: MavlinkAdapter,
        emit: Callable[[str, int, dict[str, object]], None],
        *,
        run_id: str,
    ) -> None:
        self._vehicle = vehicle
        self._emit = emit
        self._run_id = run_id
        self._running = False
        self._execution_ready_ns: int | None = None
        self._wait_started_ns: int | None = None
        self._heartbeat_ns: int | None = None
        self.phase = OperatorPhase.WAIT_READY

    def observe_running(self, run_id: str) -> None:
        if run_id == self._run_id:
            self._running = True

    def observe_execution_ready(self, status: Any) -> None:
        if getattr(status, "run_id", None) == self._run_id:
            stamp = getattr(status, "sim_timestamp_ns", None)
            if type(stamp) is int and stamp >= 0:
                self._execution_ready_ns = stamp

    def observe_wait_started(self, status: Any) -> None:
        if getattr(status, "run_id", None) != self._run_id:
            return
        stamp = getattr(status, "sim_timestamp_ns", None)
        operation_id = getattr(status, "operation_id", None)
        if type(stamp) is int and stamp >= 0 and isinstance(operation_id, str) and operation_id:
            self._wait_started_ns = stamp

    def _event(self, name: str, stamp: int, **fields: object) -> None:
        self._emit(name, stamp, fields)

    def _fail(self, stamp: int, reason: str) -> None:
        self.phase = OperatorPhase.FAILED
        self._event("operator_failed", stamp, reason=reason)

    def observe(self, telemetry: Telemetry) -> None:
        if self.phase in {OperatorPhase.FAILED, OperatorPhase.FINALIZED}:
            return
        stamp = telemetry.timestamp_ns
        if telemetry.heartbeat:
            self._heartbeat_ns = stamp
        if telemetry.ack is not None:
            expected = {
                OperatorPhase.WAIT_GUIDED_ACK: CommandKind.SET_GUIDED,
                OperatorPhase.WAIT_ARM_ACK: CommandKind.ARM,
            }.get(self.phase)
            if telemetry.ack.command is not expected:
                self._fail(stamp, "unexpected command acknowledgement")
                return
            if not telemetry.ack.accepted:
                self._fail(stamp, f"command rejected with MAV_RESULT {telemetry.ack.result}")
                return
            self._event("operator_acknowledgement", stamp, command=expected.value)
            self.phase = (
                OperatorPhase.WAIT_GUIDED
                if expected is CommandKind.SET_GUIDED
                else OperatorPhase.WAIT_ARMED
            )
            return
        if self.phase is OperatorPhase.WAIT_GUIDED and telemetry.mode is not None:
            if telemetry.mode != "GUIDED":
                self._fail(stamp, f"observed mode {telemetry.mode} after GUIDED acknowledgement")
                return
            self._event("operator_observed_guided", stamp)
            self._vehicle.send(CommandKind.ARM, None)
            self._event("operator_command", stamp, command=CommandKind.ARM.value)
            self.phase = OperatorPhase.WAIT_ARM_ACK
        elif self.phase is OperatorPhase.WAIT_ARMED:
            if telemetry.mode is not None and telemetry.mode != "GUIDED":
                self._fail(stamp, f"mode changed to {telemetry.mode} before arming")
            elif telemetry.armed is True:
                self._event("operator_observed_armed", stamp)
                self.phase = OperatorPhase.PASSIVE

    def tick(self, timestamp_ns: int) -> None:
        if self.phase is not OperatorPhase.WAIT_READY:
            return
        heartbeat_fresh = (
            self._heartbeat_ns is not None
            and 0 <= timestamp_ns - self._heartbeat_ns <= self.HEARTBEAT_MAX_AGE_NS
        )
        prerequisites_precede_tick = all(
            stamp is not None and stamp <= timestamp_ns
            for stamp in (self._execution_ready_ns, self._wait_started_ns)
        )
        if not (self._running and prerequisites_precede_tick and heartbeat_fresh):
            return
        self._vehicle.send(CommandKind.SET_GUIDED, None)
        self._event("operator_command", timestamp_ns, command=CommandKind.SET_GUIDED.value)
        self.phase = OperatorPhase.WAIT_GUIDED_ACK

    def stop_for_finalization(self) -> None:
        if self.phase is not OperatorPhase.FAILED:
            self.phase = OperatorPhase.FINALIZED


def main() -> int:
    """Run the private SERIAL1 actor until global finalization."""
    from artifacts.runtime_protocol import RuntimeProtocol
    from artifacts.runtime_status import MissionExecutionReadyStatus, OperatorWaitStartedStatus
    from pymavlink import mavutil
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import RunState

    run_id = os.environ["SIM_RUN_ID"]
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"])
    config = json.loads(
        (run_directory / "configuration/run.json").read_text(encoding="utf-8")
    )
    startup_wall_seconds = float(config["startup_wall_seconds"])
    log_path = run_directory / "logs/docker/operator.jsonl"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stopping = False
    latest_clock_ns: int | None = None

    with log_path.open("a", encoding="utf-8", buffering=1) as log:
        def emit(name: str, stamp: int, fields: dict[str, object]) -> None:
            log.write(json.dumps({"run_id": run_id, "event": name, "sim_timestamp_ns": stamp, **fields}, sort_keys=True) + "\n")

        connection = connect_operator_mavlink(
            mavutil.mavlink_connection,
            deadline=time.monotonic() + startup_wall_seconds,
        )
        vehicle = MavlinkAdapter(connection, mavutil)
        vehicle.request_telemetry()
        actor = OperatorWaitActor(vehicle, emit, run_id=run_id)
        protocol = RuntimeProtocol(run_directory, run_id)
        rclpy.init()
        node = Node("drone_sim_operator_wait", parameter_overrides=[])

        def clock_callback(message: Any) -> None:
            nonlocal latest_clock_ns
            latest_clock_ns = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)

        def state_callback(message: Any) -> None:
            nonlocal stopping
            if message.run_id != run_id:
                return
            if message.state == RunState.RUNNING:
                actor.observe_running(run_id)
            elif message.state == RunState.FINALIZING:
                stopping = True
                actor.stop_for_finalization()

        def request_stop(*_args: object) -> None:
            nonlocal stopping
            stopping = True
            actor.stop_for_finalization()

        node.create_subscription(Clock, "/clock", clock_callback, QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        node.create_subscription(RunState, "/simulation/run_state", state_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        try:
            while not stopping:
                rclpy.spin_once(node, timeout_sec=0.01)
                ready = protocol.read_status(MissionExecutionReadyStatus)
                started = protocol.read_status(OperatorWaitStartedStatus)
                if ready is not None:
                    actor.observe_execution_ready(ready)
                if started is not None:
                    actor.observe_wait_started(started)
                if latest_clock_ns is not None:
                    telemetry = vehicle.poll(latest_clock_ns)
                    if telemetry is not None:
                        actor.observe(telemetry)
                    actor.tick(latest_clock_ns)
                if actor.phase is OperatorPhase.FAILED:
                    return 1
                time.sleep(0.001)
            return 0
        finally:
            protocol.close()
            node.destroy_node()
            rclpy.shutdown()
            connection.close()


__all__ = [
    "OperatorPhase",
    "OperatorWaitActor",
    "connect_operator_mavlink",
    "main",
]
