"""Live configured-mission host; ROS and MAVLink resources stay at this edge."""

from __future__ import annotations

import signal
import sys
import time
from pathlib import Path
from types import SimpleNamespace

from artifacts.runtime_status import MissionExecutionReadyStatus, RuntimeFailureStatus

from .configured_mission import ConfiguredMission
from .lifecycle import CompanionLifecycle
from .mavlink_adapter import MavlinkAdapter
from .mission import MissionPhase, MissionState
from .operations import DroneOperations


def _configure_vfs_resources(config, node, *, qos_profile, reliability, durability,
                             clock_ns):
    """Compose VFS-only ROS adapters without affecting version-1 configured runs."""
    from artifacts.vfs_contract import load_vfs_course, load_vfs_scenario
    from drone.sensors.camera._camera_manager import CameraManager
    from drone.sensors.camera.camera import Camera
    from sensor_msgs.msg import Image, LaserScan
    from simulation_interfaces.msg import (
        AttemptStatus, JudgeDecision, MissionEvent, VehicleStatus,
    )
    from simulation_interfaces.srv import PayloadCommand

    from .comp2026_host import RosFrameSource
    from .configured_vision import ConfiguredVision
    from .runtime_node import stamp_ns
    from .vfs_operations import (
        AttemptMonitor, ConfiguredRangefinder, PayloadActions, PhaseExchange,
        VehicleStatusStream,
    )

    course = load_vfs_course(config.course_path)
    scenario = load_vfs_scenario(config.scenario_path)
    reliable = lambda depth, transient=False: qos_profile(
        depth=depth,
        reliability=reliability.RELIABLE,
        durability=(durability.TRANSIENT_LOCAL if transient else durability.VOLATILE),
    )
    frame_source = RosFrameSource(
        width_px=scenario.camera.width_px,
        height_px=scenario.camera.height_px,
        clock_ns=clock_ns,
        max_exposure_age_ns=500_000_000,
    )
    manager = CameraManager(
        frame_source=frame_source,
        calibration_path=Path(__file__).with_name("gazebo_camera_calibration.json"),
        clock=lambda: clock_ns() or 0,
        max_exposure_age_ns=500_000_000,
    )
    camera = Camera(
        int(round(scenario.payload_geometry.marker_size_m * 1000)),
        manager=manager,
        mounting_path=Path(__file__).with_name("gazebo_camera_mounting.json"),
    )
    vision = ConfiguredVision(camera)
    rangefinder = ConfiguredRangefinder(clock_ns=clock_ns)
    payload_client = node.create_client(PayloadCommand, "/simulation/payload_command")
    payloads = PayloadActions(config.run_id, payload_client, PayloadCommand)
    event_publisher = node.create_publisher(
        MissionEvent, "/simulation/mission_events", reliable(100, transient=True)
    )
    event_sequence = 0

    def publish_event(phase, state, detail, timestamp_ns):
        nonlocal event_sequence
        message = MissionEvent()
        message.run_id = config.run_id
        message.sim_timestamp.sec = timestamp_ns // 1_000_000_000
        message.sim_timestamp.nanosec = timestamp_ns % 1_000_000_000
        message.event_id = event_sequence
        published_event_id = event_sequence
        event_sequence += 1
        message.phase = phase
        message.state = state
        message.detail = detail
        event_publisher.publish(message)
        return published_event_id

    status_stream = VehicleStatusStream()
    phases = PhaseExchange(
        config.run_id, publish_event, check_ready=status_stream.disarmed_ready
    )
    attempt = AttemptMonitor(config.run_id)
    node.create_subscription(
        Image, "/camera/onboard/image_raw", frame_source.accept_image, reliable(100)
    )
    node.create_subscription(
        LaserScan,
        "/competition/range/downward",
        lambda message: rangefinder.accept(message, stamp_ns(message.header.stamp)),
        reliable(1),
    )
    node.create_subscription(
        JudgeDecision, "/simulation/judge_decisions", phases.accept,
        reliable(100, transient=True),
    )
    node.create_subscription(
        AttemptStatus, "/simulation/attempt_status", attempt.accept, reliable(100)
    )
    status_publisher = node.create_publisher(
        VehicleStatus, "/simulation/vehicle_status", reliable(100)
    )
    return SimpleNamespace(
        dependencies=dict(
            course=course, scenario=scenario, vision=vision, rangefinder=rangefinder,
            payloads=payloads, phases=phases, attempt=attempt,
        ),
        conditions=phases,
        vision=vision,
        frame_source=frame_source,
        status_publisher=status_publisher,
        status_type=VehicleStatus,
        status_stream=status_stream,
        phases=phases,
    )


class ConfiguredHost:
    def __init__(self, plan, vehicle, lifecycle, protocol, run_id, *,
                 operation_dependencies=None, conditions=None) -> None:
        self.operations = DroneOperations(
            vehicle, lifecycle.emit, **(operation_dependencies or {})
        )
        self.mission = ConfiguredMission(
            plan, self.operations, conditions=conditions, emit=lifecycle.emit
        )
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
            if not mission_running or not self.operations.mission_ready:
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
            self.lifecycle.observe_terminal(MissionState(MissionPhase.LANDED, last_timestamp_ns=timestamp_ns))
            self._success_reported = True

    def fail(self, reason: str, *, attempt_recovery: bool = True) -> None:
        if self.error is not None:
            return
        self.error = reason
        owner_recovery = self.operations.recovery_id
        if owner_recovery is None:
            self.mission.abort(reason)
        self.lifecycle.observe_terminal(MissionState(
            MissionPhase.FAILED, last_timestamp_ns=self._clock_ns, failure_reason=reason))
        if owner_recovery is not None:
            self._recovery_id = owner_recovery
        elif attempt_recovery:
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
    readiness_deadline = min(
        overall_deadline, started + config.startup_timeout_seconds
    )
    connection = None
    node = None
    host = None
    initialized = False
    latest_clock_ns = None
    mission_running = False
    finalizing = False
    requested_stop = False
    callback_error = None
    recovery_deadline = None
    previous_handlers = {}
    vfs_resources = None

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
            if vfs_resources is not None:
                vfs_resources.frame_source.notify_clock()

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
        course_path = getattr(config, "course_path", None)
        scenario_path = getattr(config, "scenario_path", None)
        if course_path is not None or scenario_path is not None:
            if course_path is None or scenario_path is None:
                raise ValueError("configured VFS runtime requires course and scenario together")
            vfs_resources = _configure_vfs_resources(
                config,
                node,
                qos_profile=QoSProfile,
                reliability=ReliabilityPolicy,
                durability=DurabilityPolicy,
                clock_ns=lambda: latest_clock_ns,
            )
        host = ConfiguredHost(
            config.mission_plan,
            vehicle,
            lifecycle,
            protocol,
            config.run_id,
            operation_dependencies=(None if vfs_resources is None else vfs_resources.dependencies),
            conditions=(None if vfs_resources is None else vfs_resources.conditions),
        )
        node.create_subscription(Clock, "/clock", clock_callback,
                                 QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        node.create_subscription(RunState, "/simulation/run_state", state_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for signum in (signal.SIGTERM, signal.SIGINT):
            previous_handlers[signum] = signal.signal(signum, stop)
        telemetry_requested = False
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            for _ in range(100):
                telemetry = vehicle.poll(latest_clock_ns if latest_clock_ns is not None else 0)
                if telemetry is None:
                    break
                host.observe(telemetry)
                if vfs_resources is not None:
                    vfs_resources.status_stream.observe(telemetry)
                if telemetry.status_text is not None:
                    lifecycle.emit("mavlink_status_text", latest_clock_ns, {"text": telemetry.status_text})
                if telemetry.heartbeat and not telemetry_requested:
                    vehicle.request_telemetry(rate_hz=10)
                    if vfs_resources is not None:
                        vehicle.request_vfs_baseline()
                    telemetry_requested = True
            if vfs_resources is not None and latest_clock_ns is not None:
                status = vfs_resources.status_stream.sample(latest_clock_ns)
                if status is not None:
                    message = vfs_resources.status_type()
                    message.run_id = config.run_id
                    message.sim_timestamp.sec = status.timestamp_ns // 1_000_000_000
                    message.sim_timestamp.nanosec = status.timestamp_ns % 1_000_000_000
                    message.sequence_id = status.sequence_id
                    message.vehicle_id = "iris_competition_2027"
                    message.armed = status.armed
                    message.mode = status.mode
                    message.failsafe_active = status.failsafe_active
                    message.failsafe_reason = status.failsafe_reason
                    vfs_resources.status_publisher.publish(message)
                vfs_resources.phases.advance(latest_clock_ns)
            now = time.monotonic()
            if (vfs_resources is not None and not host.started
                    and now >= readiness_deadline):
                host.fail(
                    "configured VFS baseline readiness deadline expired",
                    attempt_recovery=False,
                )
                break
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
        if node is not None:
            node.destroy_node()
        if vfs_resources is not None:
            vfs_resources.frame_source.stop("configured runtime stopped")
            vfs_resources.vision.close()
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
