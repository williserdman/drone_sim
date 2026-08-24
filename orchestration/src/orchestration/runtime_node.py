"""ROS orchestration runtime for the Phase 2 lifecycle handshake."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
from typing import Any, Callable, Protocol

from artifacts.runtime_protocol import RuntimeProtocol, canonical_run_id
from artifacts.structured_log import StructuredEvent, write_event


_PRETERMINAL = frozenset({"STARTING", "READY", "RUNNING"})
_TERMINAL = frozenset({"COMPLETED", "FAILED", "ABORTED"})


class _Protocol(Protocol):
    def read_finalize_request(self) -> dict[str, Any] | None: ...
    def read_terminal_committed(self) -> dict[str, Any] | None: ...
    def write_status(self, name: str, document: dict[str, Any]) -> Any: ...


@dataclass(frozen=True)
class RuntimeStateEvent:
    run_id: str
    state: str
    sim_timestamp_ns: int
    reason: str
    config_sha256: str


class OrchestrationRuntime:
    """Small deterministic lifecycle core; ROS and polling remain adapters."""

    def __init__(
        self,
        run_id: str,
        config_sha256: str,
        *,
        protocol: _Protocol,
        publish: Callable[[RuntimeStateEvent], None],
        diagnostic: Callable[[str], None] = lambda _message: None,
    ) -> None:
        self.run_id = canonical_run_id(run_id)
        if (
            not isinstance(config_sha256, str)
            or len(config_sha256) != 64
            or any(value not in "0123456789abcdef" for value in config_sha256)
        ):
            raise ValueError("config_sha256 must be a lower-case SHA-256")
        self.config_sha256 = config_sha256
        self._protocol = protocol
        self._publish = publish
        self._diagnostic = diagnostic
        self.state = "CREATED"
        self.last_sim_timestamp_ns = 0
        self._terminal = False

    def _emit(self, state: str, reason: str = "") -> None:
        self.state = state
        self._publish(
            RuntimeStateEvent(
                self.run_id,
                state,
                self.last_sim_timestamp_ns,
                reason,
                self.config_sha256,
            )
        )

    def start(self) -> None:
        if self.state != "CREATED":
            raise RuntimeError("runtime has already started")
        self._emit("STARTING")

    def accept_artifact_status(self, run_id: str, ready: bool) -> None:
        if self.state != "STARTING":
            return
        if run_id != self.run_id:
            self._diagnostic("ignored stale artifact status")
            return
        if ready is True:
            self._emit("READY")

    def accept_clock(self, sim_timestamp_ns: int) -> None:
        if isinstance(sim_timestamp_ns, bool) or not isinstance(sim_timestamp_ns, int):
            raise TypeError("clock timestamp must be an integer")
        if sim_timestamp_ns < 0:
            raise ValueError("clock timestamp must be nonnegative")
        if self.state not in {"READY", "RUNNING"}:
            return
        self.last_sim_timestamp_ns = sim_timestamp_ns
        if self.state == "READY":
            self._emit("RUNNING")
            self._protocol.write_status(
                "runtime-running",
                {
                    "run_id": self.run_id,
                    "state": "RUNNING",
                    "sim_timestamp_ns": sim_timestamp_ns,
                },
            )

    def poll(self) -> bool:
        """Observe durable controls once; return true after terminal acknowledgement."""
        if self._terminal:
            return True
        if self.state in _PRETERMINAL:
            request = self._protocol.read_finalize_request()
            if request is not None:
                self._emit("FINALIZING", request["reason"])
        if self.state == "FINALIZING":
            committed = self._protocol.read_terminal_committed()
            if committed is not None:
                terminal = committed["terminal_status"]
                if terminal not in _TERMINAL:
                    raise RuntimeError("protocol returned an invalid terminal state")
                self._emit(terminal, committed["reason"])
                self._protocol.write_status(
                    "terminal-notified", {"run_id": self.run_id, "notified": True}
                )
                self._terminal = True
        return self._terminal


def _timestamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _assign_stamp(stamp: Any, timestamp_ns: int) -> None:
    stamp.sec = timestamp_ns // 1_000_000_000
    stamp.nanosec = timestamp_ns % 1_000_000_000


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import ArtifactStatus, RunState

    run_id = os.environ["SIM_RUN_ID"]
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"])
    config_path = Path(os.environ["SIM_CONFIG_PATH"])
    config = json.loads(config_path.read_text(encoding="utf-8"))
    config_sha256 = config["config_sha256"]
    protocol = RuntimeProtocol(run_directory, run_id)

    rclpy.init()
    node = Node("orchestration_runtime")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(RunState, "/simulation/run_state", qos)

    quiescent = False

    def emit_log(event: str, sim_timestamp_ns: int | None, **fields: Any) -> None:
        nonlocal quiescent
        if quiescent:
            return
        write_event(
            sys.stdout,
            StructuredEvent(
                run_id=run_id,
                module="orchestration",
                severity="INFO",
                event=event,
                sim_timestamp=(None if sim_timestamp_ns is None else sim_timestamp_ns / 1e9),
                wall_timestamp=datetime.now(timezone.utc),
                fields=fields,
            ),
        )

    def publish(event: RuntimeStateEvent) -> None:
        nonlocal quiescent
        message = RunState()
        message.run_id = event.run_id
        _assign_stamp(message.sim_timestamp, event.sim_timestamp_ns)
        message.state = getattr(RunState, event.state)
        message.reason = event.reason
        message.config_sha256 = event.config_sha256
        publisher.publish(message)
        if event.state == "FINALIZING":
            emit_log("finalizing", event.sim_timestamp_ns, reason=event.reason)
            quiescent = True
        elif event.state not in _TERMINAL:
            emit_log(event.state.lower(), event.sim_timestamp_ns)

    runtime = OrchestrationRuntime(
        run_id,
        config_sha256,
        protocol=protocol,
        publish=publish,
        diagnostic=lambda detail: emit_log(
            "stale_input", runtime.last_sim_timestamp_ns, detail=detail
        ),
    )

    def artifact_callback(message: Any) -> None:
        runtime.accept_artifact_status(message.run_id, message.ready)

    def clock_callback(message: Any) -> None:
        runtime.accept_clock(_timestamp_ns(message.clock))

    node.create_subscription(ArtifactStatus, "/simulation/artifact_status", artifact_callback, qos)
    node.create_subscription(
        Clock,
        "/clock",
        clock_callback,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    runtime.start()
    try:
        while rclpy.ok() and not runtime.poll():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        protocol.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


__all__ = ["OrchestrationRuntime", "RuntimeStateEvent", "main"]
