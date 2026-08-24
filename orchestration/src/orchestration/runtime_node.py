"""ROS orchestration runtime for the Phase 2 lifecycle handshake."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time
from typing import Any, Callable, Iterable, Protocol

from artifacts.runtime_protocol import RuntimeProtocol, canonical_run_id
from artifacts.structured_log import StructuredEvent, write_event


_PRETERMINAL = frozenset({"STARTING", "READY", "RUNNING"})
_TERMINAL = frozenset({"COMPLETED", "FAILED", "ABORTED"})
_QUIESCENCE_PEERS = (
    "companion",
    "ardupilot_sitl",
    "gazebo",
    "electromagnet",
    "scorekeeper",
)
_PHASE2_RUN_STATE_NODES = frozenset(
    {
        "artifacts_runtime",
        "synthetic_companion",
        "synthetic_ardupilot_sitl",
        "synthetic_gazebo",
        "synthetic_electromagnet",
        "synthetic_scorekeeper",
    }
)
_PHASE3_RUN_STATE_NODES = frozenset(
    {
        "artifacts_runtime",
        "drone_sim_companion",
        "drone_sim_gazebo_lifecycle",
        "drone_sim_electromagnet",
        "drone_sim_scorekeeper",
    }
)


class _Protocol(Protocol):
    def read_status(self, name: str) -> dict[str, Any] | None: ...
    def read_finalize_request(self) -> dict[str, Any] | None: ...
    def read_terminal_committed(self) -> dict[str, Any] | None: ...
    def write_status(self, name: str, document: dict[str, Any]) -> Any: ...
    def write_quiescence(self, module: str) -> Any: ...
    def read_quiescence(self, module: str) -> dict[str, Any] | None: ...


@dataclass(frozen=True)
class RuntimeStateEvent:
    run_id: str
    state: str
    sim_timestamp_ns: int
    reason: str
    config_sha256: str


@dataclass(frozen=True)
class RunStateSubscriber:
    """Discovery attributes needed to identify a lifecycle archive consumer."""

    node_name: str
    topic_type: str
    reliable: bool
    transient_local: bool


class RunStateTransportBarrier:
    """Require every intended lifecycle consumer before publishing STARTING."""

    _TYPE = "simulation_interfaces/msg/RunState"

    def __init__(
        self,
        *,
        deadline: float,
        failure: Callable[[str], None],
        required_nodes: Iterable[str] = _PHASE2_RUN_STATE_NODES,
    ) -> None:
        self._deadline = deadline
        self._failure = failure
        self._required_nodes = frozenset(required_nodes)
        if not self._required_nodes:
            raise ValueError("run-state transport barrier requires at least one node")
        self.ready = False
        self.failed = False
        self.preempted = False

    def poll(
        self,
        subscribers: Iterable[RunStateSubscriber],
        *,
        now: float,
        finalizing: bool,
    ) -> bool:
        if self.ready:
            return True
        if self.failed or self.preempted:
            return False
        if finalizing:
            self.preempted = True
            return False
        valid_names = {
            item.node_name
            for item in subscribers
            if item.topic_type == self._TYPE and item.reliable and item.transient_local
        }
        runtime_consumers = valid_names & self._required_nodes
        recorder_present = any(name.startswith("rosbag2_recorder_") for name in valid_names)
        if runtime_consumers == self._required_nodes and recorder_present:
            self.ready = True
            return True
        if now >= self._deadline:
            self.failed = True
            self._failure("run-state transport discovery deadline expired")
        return False


def start_runtime_after_transport_barrier(
    *,
    barrier: RunStateTransportBarrier,
    subscribers: Callable[[], Iterable[RunStateSubscriber]],
    finalize_requested: Callable[[], bool],
    monotonic: Callable[[], float],
    spin_once: Callable[[], None],
    runtime_start: Callable[[], None],
    runtime_ok: Callable[[], bool],
) -> bool:
    """Cross the infrastructure barrier before the first lifecycle publish."""
    while runtime_ok():
        if barrier.poll(
            subscribers(), now=monotonic(), finalizing=finalize_requested()
        ):
            runtime_start()
            return True
        if barrier.failed or barrier.preempted:
            return False
        spin_once()
    return False


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
        required_durable_readiness: tuple[str, ...] = (),
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
        allowed_readiness = {"ardupilot-ready", "companion-ready"}
        if (
            len(required_durable_readiness) != len(set(required_durable_readiness))
            or any(name not in allowed_readiness for name in required_durable_readiness)
        ):
            raise ValueError("durable readiness names are invalid")
        self._required_durable_readiness = required_durable_readiness
        self.state = "CREATED"
        self.last_sim_timestamp_ns = 0
        self._clock_observed = False
        self._terminal = False
        self._quiescence_written = False
        self._freeze_written = False

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
        self._clock_observed = True
        if self.state == "READY" and not self._required_durable_readiness:
            self._start_running()

    def _start_running(self) -> None:
        self._emit("RUNNING")
        self._protocol.write_status(
            "runtime-running",
            {
                "run_id": self.run_id,
                "state": "RUNNING",
                "sim_timestamp_ns": self.last_sim_timestamp_ns,
            },
        )

    def _flight_peers_ready(self) -> bool:
        return self._clock_observed and all(
            self._protocol.read_status(name) is not None
            for name in self._required_durable_readiness
        )

    def poll(self) -> bool:
        """Observe durable controls once; return true after terminal acknowledgement."""
        if self._terminal:
            return True
        if self.state in _PRETERMINAL:
            request = self._protocol.read_finalize_request()
            if request is not None:
                self._emit("FINALIZING", request["reason"])
                self._protocol.write_quiescence("orchestration")
                self._quiescence_written = True
            elif self.state == "READY" and self._flight_peers_ready():
                self._start_running()
        if self.state == "FINALIZING":
            if not self._quiescence_written:
                raise RuntimeError("orchestration quiescence marker was not written")
            if not self._freeze_written:
                if not all(
                    self._protocol.read_quiescence(module) is not None
                    for module in _QUIESCENCE_PEERS
                ):
                    return False
                self._protocol.write_status(
                    "runtime-frozen", {"run_id": self.run_id, "frozen": True}
                )
                self._freeze_written = True
            committed = self._protocol.read_terminal_committed()
            if committed is not None:
                terminal = committed["terminal_status"]
                if terminal not in _TERMINAL:
                    raise RuntimeError("protocol returned an invalid terminal state")
                self.state = terminal
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
        required_durable_readiness=(
            ("ardupilot-ready", "companion-ready")
            if config.get("runtime_profile") == "phase3"
            else ()
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

    def run_state_subscribers() -> list[RunStateSubscriber]:
        return [
            RunStateSubscriber(
                endpoint.node_name,
                endpoint.topic_type,
                endpoint.qos_profile.reliability == ReliabilityPolicy.RELIABLE,
                endpoint.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL,
            )
            for endpoint in node.get_subscriptions_info_by_topic("/simulation/run_state")
        ]

    startup_barrier = RunStateTransportBarrier(
        deadline=time.monotonic() + float(config["startup_wall_seconds"]),
        required_nodes=(
            _PHASE3_RUN_STATE_NODES
            if config.get("runtime_profile") == "phase3"
            else _PHASE2_RUN_STATE_NODES
        ),
        failure=lambda reason: protocol.write_status(
            "runtime-failure",
            {
                "run_id": run_id,
                "module": "orchestration",
                "reason": reason,
                "diagnostic_paths": ["logs/docker/orchestration.log.partial"],
            },
        ),
    )
    try:
        started = start_runtime_after_transport_barrier(
            barrier=startup_barrier,
            subscribers=run_state_subscribers,
            finalize_requested=lambda: protocol.read_finalize_request() is not None,
            monotonic=time.monotonic,
            spin_once=lambda: rclpy.spin_once(node, timeout_sec=0.05),
            runtime_start=runtime.start,
            runtime_ok=rclpy.ok,
        )
        if not started:
            return
        while rclpy.ok() and not runtime.poll():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        protocol.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


__all__ = [
    "OrchestrationRuntime",
    "RunStateSubscriber",
    "RunStateTransportBarrier",
    "RuntimeStateEvent",
    "start_runtime_after_transport_barrier",
    "main",
]
