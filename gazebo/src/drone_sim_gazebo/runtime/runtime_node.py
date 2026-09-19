"""Run the paused-first Gazebo server, bridges, ROS adapter, and lifecycle."""

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
from decimal import Decimal
import os
from pathlib import Path
import sys
from threading import Event, Thread
import time

from artifacts.runtime_protocol import RuntimeProtocol
from artifacts.runtime_status import (
    ArtifactsReadyStatus,
    MissionCommandDeliveredStatus,
    MissionExecutionReadyStatus,
    RuntimeStatus,
)
from artifacts.structured_log import StructuredEvent, write_event
from orchestration.config import load_run_config

from ..server import GazeboServer, server_spec
from ..worlds import WorldConfig, resolve_world
from .children import ChildSupervisor, gazebo_child_specs
from .entrypoint import (
    ActionExecutor,
    FinalizationDeadlineLatch,
    GazeboTransport,
    TransportError,
)
from .model import (
    AdapterCompleted,
    ArtifactsReady,
    ChildExited,
    FinalizationRequested,
    GazeboReady,
    RunStateEvent,
    RuntimeModel,
    ServerStopFailed,
)
from .paths import bridge_config_for_world


_RUN_STATE_NAMES = (
    "CREATED",
    "STARTING",
    "READY",
    "RUNNING",
    "FINALIZING",
    "COMPLETED",
    "FAILED",
    "ABORTED",
)
_EPOCH_CLOCK_LOOKAHEAD_NS = 50_000_000


def _event(run_id: str, name: str, *, fields=None, sim_timestamp_ns=None) -> None:
    write_event(
        sys.stdout,
        StructuredEvent(
            run_id=run_id,
            module="gazebo",
            severity="INFO",
            event=name,
            wall_timestamp=datetime.now(timezone.utc),
            sim_timestamp=(
                Decimal(sim_timestamp_ns) / Decimal(1_000_000_000)
                if sim_timestamp_ns is not None
                else None
            ),
            fields=fields or {},
        ),
    )


def _record_adapter_fault(run_id: str, inbox, reason: str) -> None:
    _event(run_id, "adapter_fault", fields={"reason": reason})
    inbox.append(ChildExited(run_id, "adapter", 1))


def _lifecycle_node(run_id: str, inbox: deque):
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from simulation_interfaces.msg import RunState

    class LifecycleNode(Node):
        def __init__(self):
            super().__init__("drone_sim_gazebo_lifecycle")
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(RunState, "/simulation/run_state", self._accept, qos)

        def _accept(self, message):
            if message.run_id != run_id:
                return
            state = int(message.state)
            if not 0 <= state < len(_RUN_STATE_NAMES):
                inbox.append(ChildExited(run_id, "adapter", 2))
                return
            inbox.append(RunStateEvent(run_id, _RUN_STATE_NAMES[state]))

    return LifecycleNode()


def _wait_transport(
    transport: GazeboTransport,
    *,
    deadline: float,
    monotonic=time.monotonic,
) -> None:
    last_error: Exception | None = None
    while monotonic() < deadline:
        try:
            transport.assert_ready()
            return
        except TransportError as error:
            last_error = error
            time.sleep(0.1)
    raise TransportError(f"Gazebo endpoints did not become ready: {last_error}")


def _probe_flight_exchange(
    transport: GazeboTransport,
    *,
    deadline: float,
    monotonic=time.monotonic,
):
    remaining = deadline - monotonic()
    if remaining <= 0:
        raise TransportError(
            "Gazebo flight exchange did not become ready before startup deadline"
        )
    return transport.ready_flight_exchange(timeout=min(2.0, remaining))


def _start_server_ready(
    server,
    transport: GazeboTransport,
    *,
    startup_deadline: float,
    cleanup_deadline: float,
    wait_transport=_wait_transport,
) -> None:
    """Start and discover the server without leaking it on startup failure."""
    server.start()
    try:
        wait_transport(transport, deadline=startup_deadline)
    except BaseException:
        try:
            server.stop(cleanup_deadline)
        except BaseException:
            pass
        raise


class PublicEpochRendezvous:
    """Hold physics at public zero until the first command is delivered."""

    def __init__(
        self,
        *,
        transport: GazeboTransport,
        protocol: RuntimeProtocol,
        public_epoch_native_ns: int,
        activate_output,
        prepare_output,
        epoch_reached,
        release_status_type: type[RuntimeStatus] = MissionCommandDeliveredStatus,
    ) -> None:
        self._transport = transport
        self._protocol = protocol
        self._target_ns = public_epoch_native_ns
        self._activate_output = activate_output
        self._prepare_output = prepare_output
        self._epoch_reached = epoch_reached
        self._release_status_type = release_status_type
        self._begun = False
        self._run_to_requested = False
        self._released = False
        self._unpause_done = Event()
        self._unpause_error: Exception | None = None
        self._unpause_thread: Thread | None = None
        self._unpause_attempts_started = 0

    def _unpause(self) -> None:
        try:
            self._transport.set_paused(False)
        except Exception as error:
            self._unpause_error = error
        finally:
            self._unpause_done.set()

    def _start_unpause(self) -> None:
        self._unpause_done = Event()
        self._unpause_error = None
        self._unpause_thread = Thread(
            target=self._unpause,
            name=f"gazebo-public-epoch-unpause-{self._unpause_attempts_started + 1}",
            daemon=True,
        )
        self._unpause_attempts_started += 1
        self._unpause_thread.start()

    def begin(self) -> None:
        if self._begun:
            return
        self._activate_output()
        self._begun = True

    def start_warmup(self) -> None:
        """Advance private simulation to the epoch and let Gazebo pause there."""
        if self._run_to_requested:
            return
        self._prepare_output()
        # Gazebo may stop before its terminal clock sample crosses the ROS bridge.
        # Stay well inside the 50 ms camera grid while making the epoch observable.
        try:
            self._transport.run_to_sim_time(
                self._target_ns + _EPOCH_CLOCK_LOOKAHEAD_NS
            )
        except TransportError:
            # Gazebo may execute a long run-to request but reject or omit the
            # service reply under load.  The adapter's independently observed
            # native clock remains the authority for whether the epoch was hit.
            pass
        self._run_to_requested = True

    def release_if_delivered(self) -> bool:
        if not self._begun or self._released:
            return False
        if not self._run_to_requested:
            return False
        if self._protocol.read_status(self._release_status_type) is None:
            return False
        if self._unpause_attempts_started == 0:
            self._start_unpause()
            return False
        if not self._unpause_done.is_set():
            return False
        epoch_reached = self._epoch_reached()
        if self._unpause_error is not None and not epoch_reached:
            raise self._unpause_error
        if self._unpause_attempts_started == 1:
            if not epoch_reached:
                return False
            # Gazebo can apply world control but fail to answer its service call
            # under load.  Reaching the epoch proves the first request took
            # effect.  The scheduled run-to pauses there, so issue the final
            # unpause and release without waiting for another unreliable reply.
            self._start_unpause()
            self._released = True
            return True
        self._released = True
        return True


def _release_status_type_for_mission(mission: str) -> type[RuntimeStatus]:
    if mission == "configured":
        return MissionExecutionReadyStatus
    return MissionCommandDeliveredStatus


def main() -> int:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    from ..ros_adapter.node import GazeboAdapterNode

    run_id = os.environ["SIM_RUN_ID"]
    run_directory = Path(os.environ["SIM_RUN_DIRECTORY"]).resolve(strict=True)
    config_path = Path(
        os.environ.get(
            "SIM_CONFIG_PATH", str(run_directory / "configuration/run.json")
        )
    )
    config = load_run_config(config_path)
    if config.run_id != run_id or config.runtime_profile != "phase3" or config.simulation is None:
        raise ValueError("Gazebo runtime requires the current resolved phase3 run")
    resource_root = Path(
        os.environ.get("DRONE_SIM_GAZEBO_RESOURCES", "/opt/drone_sim/gazebo/resources")
    )
    resolved = resolve_world(WorldConfig(config.world, config.vehicle), package_root=resource_root)
    spec = server_spec(
        run_id=run_id,
        run_directory=run_directory,
        resolved_world=resolved,
        config=config.simulation,
    )
    server = GazeboServer(spec)
    transport = GazeboTransport(
        environment=spec.environment, world_name=resolved.world_name
    )
    transport.assert_typed_readiness_supported()
    startup_deadline = time.monotonic() + config.startup_wall_seconds
    _start_server_ready(
        server,
        transport,
        startup_deadline=startup_deadline,
        cleanup_deadline=startup_deadline + config.finalization_wall_seconds,
    )

    inbox: deque = deque()
    model = RuntimeModel(run_id=run_id, expected_frames=config.expected_camera_frames)
    protocol = RuntimeProtocol(run_directory, run_id)
    deadline_latch = FinalizationDeadlineLatch(config.finalization_wall_seconds)
    children = ChildSupervisor()
    rclpy.init()
    adapter = GazeboAdapterNode(
        run_id=run_id,
        expected_frames=config.expected_camera_frames,
        public_epoch_native_ns=config.simulation.public_epoch_native_ns,
        world_name=resolved.world_name,
        width_px=config.recording.width_px,
        height_px=config.recording.height_px,
        observer_width_px=getattr(
            config.recording, "observer_width_px", config.recording.width_px
        ),
        observer_height_px=getattr(
            config.recording, "observer_height_px", config.recording.height_px
        ),
        require_competition_recorders=config.scenario
        in {"competition_v1", "search_delivery_v1"},
        on_completed=lambda summary: inbox.append(AdapterCompleted(run_id, summary)),
        on_fault=lambda reason: _record_adapter_fault(run_id, inbox, reason),
    )
    lifecycle = _lifecycle_node(run_id, inbox)
    ros_executor = SingleThreadedExecutor()
    ros_executor.add_node(adapter)
    ros_executor.add_node(lifecycle)
    child_environment = dict(os.environ)
    child_environment.update(spec.environment)
    children.start(
        gazebo_child_specs(
            bridge_config=bridge_config_for_world(resolved.world_name),
            environment=child_environment,
            world_name=resolved.world_name,
        )
    )
    epoch_rendezvous = PublicEpochRendezvous(
        transport=transport,
        protocol=protocol,
        public_epoch_native_ns=config.simulation.public_epoch_native_ns,
        activate_output=adapter.activate_output,
        prepare_output=adapter.prepare_output_epoch,
        epoch_reached=adapter.public_epoch_reached,
        release_status_type=_release_status_type_for_mission(config.mission),
    )
    action_executor = ActionExecutor(
        run_id=run_id,
        protocol=protocol,
        transport=transport,
        children=children,
        server=server,
        activate_output=epoch_rendezvous.begin,
        start_warmup=epoch_rendezvous.start_warmup,
        observe=lambda action: _event(
            run_id, "runtime_action", fields={"action": type(action).__name__}
        ),
    )
    gazebo_ready_seen = False
    artifacts_ready_seen = False
    finalize_seen = False
    quiescent = False
    try:
        _event(run_id, "runtime_started", fields={"partition": spec.environment["GZ_PARTITION"]})
        while not quiescent:
            ros_executor.spin_once(timeout_sec=0.05)
            if epoch_rendezvous.release_if_delivered():
                _event(run_id, "public_epoch_released", sim_timestamp_ns=0)
            if not gazebo_ready_seen and adapter.transport_ready():
                flight_exchange = _probe_flight_exchange(
                    transport, deadline=startup_deadline
                )
                if flight_exchange is not None:
                    gazebo_ready_seen = True
                    inbox.append(GazeboReady(run_id, flight_exchange))
            if (
                not artifacts_ready_seen
                and protocol.read_status(ArtifactsReadyStatus) is not None
            ):
                if adapter.recorders_ready():
                    artifacts_ready_seen = True
                    inbox.append(ArtifactsReady(run_id))
            child_failure = children.poll_failure()
            if child_failure is not None:
                inbox.append(ChildExited(run_id, child_failure[0], child_failure[1]))
            if not finalize_seen:
                finalization = protocol.read_finalize_request()
                if finalization is not None:
                    finalize_seen = True
                    deadline = deadline_latch.deadline_for(finalization)
                    inbox.append(
                        FinalizationRequested(
                            run_id,
                            finalization["requested_terminal"],
                            finalization["reason"],
                            deadline,
                        )
                    )
            while inbox:
                actions = model.accept(inbox.popleft())
                if any(type(action).__name__ == "BeginFinalization" for action in actions):
                    adapter.freeze_output()
                followups = action_executor.apply(actions)
                for followup in followups:
                    inbox.append(followup)
                if any(type(action).__name__ == "WriteQuiescence" for action in actions):
                    quiescent = True
        _event(run_id, "runtime_quiescent")
        return 0
    except BaseException as error:
        _event(run_id, "runtime_exception", fields={"error": str(error)})
        if not finalize_seen:
            deadline = time.monotonic() + config.finalization_wall_seconds
            try:
                actions = model.accept(
                    FinalizationRequested(run_id, "FAILED", str(error) or "runtime_exception", deadline)
                )
                adapter.freeze_output()
                followups = action_executor.apply(actions)
                for followup in followups:
                    action_executor.apply(model.accept(followup))
            except BaseException:
                pass
        return 1
    finally:
        ros_executor.remove_node(lifecycle)
        ros_executor.remove_node(adapter)
        lifecycle.destroy_node()
        adapter.destroy_node()
        rclpy.shutdown()
        protocol.close()


if __name__ == "__main__":
    raise SystemExit(main())
