"""Live configured-mission host; ROS and MAVLink resources stay at this edge."""

from __future__ import annotations

import signal
import sys
import time

from artifacts.runtime_status import MissionExecutionReadyStatus, RuntimeFailureStatus

from .configured_mission import ConfiguredMission
from .lifecycle import CompanionLifecycle
from .mavlink_adapter import MavlinkAdapter
from .mission import MissionPhase, MissionState
from .operations import DroneOperations


class ConfiguredHost:
    def __init__(self, plan, vehicle, lifecycle, protocol, run_id, *, operations=None,
                 execution_ready=None, confirm_complete=None) -> None:
        self.operations = operations if operations is not None else DroneOperations(vehicle, lifecycle.emit)
        self._execution_ready = execution_ready or (lambda: True)
        self._confirm_complete = confirm_complete or (lambda: None)
        self.mission = ConfiguredMission(plan, self.operations)
        self.lifecycle = lifecycle
        self.protocol = protocol
        self.run_id = run_id
        self.started = False
        self.error: str | None = None
        self._recovery_id: str | None = None
        self._success_reported = False
        self._clock_ns = 0

    @property
    def recovery_pending(self) -> bool:
        return self._recovery_id is not None and self.operations.operation_status(self._recovery_id).state == "running"

    def observe(self, telemetry) -> None:
        self.operations.observe(telemetry)
        if self.operations.mission_ready:
            self.lifecycle.observe_mission_readiness(heartbeat_observed=True, prearm_checks_healthy=True)

    def tick(self, timestamp_ns: int | None, *, mission_running: bool) -> None:
        if timestamp_ns is None:
            return
        self._clock_ns = timestamp_ns
        self.operations.tick(timestamp_ns)
        if self.error is not None:
            return
        if not self.started:
            if not mission_running or not self.operations.mission_ready or not self._execution_ready():
                return
            self.protocol.write_status(
                MissionExecutionReadyStatus(self.run_id, timestamp_ns)
            )
            self.started = True
        self.mission.tick(timestamp_ns)
        if self.mission.state in {"failed", "cancelled"}:
            self.fail(self.mission.error)
        elif self.mission.state == "succeeded" and not self._success_reported:
            state = self.operations.read_vehicle_state()
            if state.get("landed") is not True or state.get("armed") is not False:
                self.fail("configured plan ended without observed landing and disarm")
                return
            self._confirm_complete()
            self.lifecycle.observe_terminal(MissionState(MissionPhase.LANDED, last_timestamp_ns=timestamp_ns))
            self._success_reported = True

    def fail(self, reason: str, *, attempt_recovery: bool = True) -> None:
        if self.error is not None:
            return
        self.error = reason
        self.mission.abort(reason)
        self.lifecycle.observe_terminal(MissionState(
            MissionPhase.FAILED, last_timestamp_ns=self._clock_ns, failure_reason=reason))
        if attempt_recovery:
            self._recovery_id = self.operations.recover_land()

    def finalize(self) -> None:
        if self.recovery_pending:
            self.operations.abort("recovery confirmation unavailable at shutdown")
        if self.error is not None:
            self.protocol.write_status(
                RuntimeFailureStatus(
                    self.run_id,
                    "companion",
                    self.error,
                    ("logs/docker/companion.log.partial",),
                )
            )
        self.lifecycle.finalize(self._clock_ns)


def run_configured(config) -> int:
    # Full plan validation happens in RuntimeConfig, before these live imports.
    if config.mission_plan is None:
        raise ValueError("configured runtime requires a validated mission plan")
    from .runtime_node import _ProductionProtocol, connect_mavlink, stamp_ns

    import rclpy
    from pymavlink import mavutil
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from simulation_interfaces.msg import RunState

    protocol = _ProductionProtocol(config)
    lifecycle = CompanionLifecycle(run_id=config.run_id, protocol=protocol, stream=sys.stdout)
    started = time.monotonic()
    overall_deadline = started + config.max_wall_seconds
    connection = None
    node = None
    host = None
    competition_io = None
    initialized = False
    latest_clock_ns = None
    mission_running = False
    finalizing = False
    requested_stop = False
    callback_error = None
    recovery_deadline = None
    previous_handlers = {}

    def stop(_signum, _frame):
        nonlocal requested_stop
        requested_stop = True

    def clock_callback(message):
        nonlocal latest_clock_ns, callback_error
        value = stamp_ns(message.clock)
        if value < 0 or (latest_clock_ns is not None and value < latest_clock_ns):
            callback_error = "authoritative simulation clock regressed"
        else:
            latest_clock_ns = value

    def state_callback(message):
        nonlocal mission_running, finalizing
        if message.run_id != config.run_id:
            return
        if message.state == RunState.RUNNING:
            mission_running = True
        elif message.state == RunState.FINALIZING:
            finalizing = True

    try:
        lifecycle.emit("starting", None, {"mission": "configured", "steps": len(config.mission_plan.steps)})
        connection = connect_mavlink(mavutil.mavlink_connection, config.mavlink_endpoint,
                                    deadline=min(started + config.startup_timeout_seconds, overall_deadline))
        vehicle = MavlinkAdapter(connection, mavutil)
        lifecycle.mark_transport_ready()
        rclpy.init()
        initialized = True
        node = Node("drone_sim_companion", parameter_overrides=[])
        node.create_subscription(Clock, "/clock", clock_callback,
                                 QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        node.create_subscription(RunState, "/simulation/run_state", state_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if getattr(config, 'scenario', 'descent_v1') in {'competition_v1', 'search_delivery_v1'}:
            import yaml
            from .configured_io import CompetitionIO
            from .configured_competition import CompetitionOperations
            competition_io = CompetitionIO(config, node, lifecycle.emit)
            course = yaml.safe_load(config.course_path.read_text(encoding='utf-8'))
            operations = CompetitionOperations(vehicle, competition_io, lifecycle.emit,
                                               release_agl_m=float(course['attempt']['release_agl_m']))
            host = ConfiguredHost(config.mission_plan, vehicle, lifecycle, protocol, config.run_id,
                                  operations=operations, execution_ready=lambda: competition_io.ready,
                                  confirm_complete=competition_io.flush)
        else:
            host = ConfiguredHost(config.mission_plan, vehicle, lifecycle, protocol, config.run_id)
        for signum in (signal.SIGTERM, signal.SIGINT):
            previous_handlers[signum] = signal.signal(signum, stop)
        telemetry_requested = False
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if competition_io is not None and latest_clock_ns is not None:
                competition_io.accept_clock(latest_clock_ns)
            for _ in range(100):
                telemetry = vehicle.poll(latest_clock_ns if latest_clock_ns is not None else 0)
                if telemetry is None:
                    break
                host.observe(telemetry)
                if telemetry.status_text is not None:
                    lifecycle.emit("mavlink_status_text", latest_clock_ns, {"text": telemetry.status_text})
                if telemetry.heartbeat and not telemetry_requested:
                    vehicle.request_telemetry(rate_hz=10)
                    telemetry_requested = True
            now = time.monotonic()
            if callback_error is not None or now >= overall_deadline:
                host.fail(callback_error or "configured mission wall deadline expired", attempt_recovery=False)
                break
            stopping = requested_stop or finalizing or protocol.read_finalize_request() is not None
            if stopping:
                if host.mission.state != "succeeded":
                    host.fail("configured mission interrupted before completion")
                elif host.error is None:
                    break
            if host.error is None and not stopping:
                host.tick(latest_clock_ns, mission_running=mission_running)
            elif latest_clock_ns is not None:
                host.tick(latest_clock_ns, mission_running=False)
            if host.error is not None:
                if recovery_deadline is None:
                    recovery_deadline = min(overall_deadline, now + config.finalization_wall_seconds)
                if not host.recovery_pending or now >= recovery_deadline:
                    break
        if host.mission.state != "succeeded" and host.error is None:
            host.fail("configured runtime stopped before completion", attempt_recovery=False)
    except Exception as error:
        if host is not None:
            host.fail(str(error), attempt_recovery=False)
        else:
            lifecycle.observe_terminal(MissionState(MissionPhase.FAILED, last_timestamp_ns=0, failure_reason=str(error)))
            protocol.write_status(
                RuntimeFailureStatus(
                    config.run_id,
                    "companion",
                    str(error),
                    ("logs/docker/companion.log.partial",),
                )
            )
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        if competition_io is not None:
            try:
                competition_io.close()
            except Exception as error:
                if host is not None:
                    host.fail(f'competition input cleanup failed: {error}', attempt_recovery=False)
        if node is not None:
            node.destroy_node()
        if connection is not None:
            connection.close()
        if initialized:
            rclpy.shutdown()
        if host is not None:
            host.finalize()
        else:
            lifecycle.finalize(latest_clock_ns)
        protocol.close()
    return 0 if host is not None and host.error is None and host.mission.state == "succeeded" else 1
