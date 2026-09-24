"""Live configured-mission host; ROS and MAVLink resources stay at this edge."""

from __future__ import annotations

import signal
import sys
import time
import math
from pathlib import Path
import threading

from artifacts.runtime_status import MissionExecutionReadyStatus, RuntimeFailureStatus

from .configured_mission import ConfiguredMission
from .lifecycle import CompanionLifecycle
from .mavlink_adapter import MavlinkAdapter
from .mission import MissionPhase, MissionState
from .moving_precision import MovingPrecisionLanding
from .operations import DroneOperations
from .comp2026_host import MissionEventRecord


MOVING_PRECISION_PARAMETERS = {
    "LAND_SPD_MS": 0.50,
    "PLND_ENABLED": 1.0,
    "PLND_TYPE": 1.0,
    "PLND_LAG": 0.08,
    "PLND_EST_TYPE": 1.0,
    "PLND_XY_DIST_MAX": 0.50,
    "PLND_STRICT": 2.0,
    "PLND_RET_MAX": 1.0,
    "PLND_TIMEOUT": 0.50,
    "PLND_ALT_MIN": 0.75,
    "PLND_ALT_MAX": 8.0,
    "PLND_OPTIONS": 5.0,
}


class ConfiguredHost:
    def __init__(
        self,
        plan,
        vehicle,
        lifecycle,
        protocol,
        run_id,
        *,
        moving_vision=None,
        publish_mission_event=None,
    ) -> None:
        self._moving_required = any(step.tool == "precision_land" for step in plan.steps)
        precision_step = next(
            (
                (index, step)
                for index, step in enumerate(plan.steps)
                if step.tool == "precision_land"
            ),
            None,
        )
        self._precision_step_index = (
            precision_step[0] if precision_step is not None else None
        )
        self._settle_by_ns = (
            int(float(precision_step[1].args["settle_by_sim_s"]) * 1e9)
            if precision_step is not None
            else None
        )
        self.operations = DroneOperations(
            vehicle,
            lifecycle.emit,
            moving_precision=MovingPrecisionLanding() if self._moving_required else None,
        )
        self.mission = ConfiguredMission(plan, self.operations)
        self.lifecycle = lifecycle
        self.protocol = protocol
        self.run_id = run_id
        self.started = False
        self.error: str | None = None
        self._recovery_id: str | None = None
        self._success_reported = False
        self._clock_ns = 0
        self._moving_vision = moving_vision
        self._parameters: dict[str, float] = {}
        self._publish_mission_event = publish_mission_event
        self._next_event_id = 0
        self._last_armed: bool | None = None

    @property
    def recovery_pending(self) -> bool:
        return self._recovery_id is not None and self.operations.operation_status(self._recovery_id).state == "running"

    def observe(self, telemetry) -> None:
        if (
            self._moving_required
            and telemetry.parameter_name in MOVING_PRECISION_PARAMETERS
        ):
            expected = MOVING_PRECISION_PARAMETERS[telemetry.parameter_name]
            value = telemetry.parameter_value
            if value is None or not math.isclose(value, expected, rel_tol=0.0, abs_tol=1e-6):
                self.fail(
                    f"effective precision parameter {telemetry.parameter_name} is {value}, expected {expected}",
                    attempt_recovery=False,
                )
                return
            self._parameters[telemetry.parameter_name] = value
        self.operations.observe(telemetry)
        if self.operations.mission_ready:
            self.lifecycle.observe_mission_readiness(heartbeat_observed=True, prearm_checks_healthy=True)
        if self._moving_required and telemetry.armed is not None:
            state = None
            if telemetry.armed is True and self._last_armed is not True:
                state = "ARMED"
            elif telemetry.armed is False and self._last_armed is True:
                state = "DISARMED"
            self._last_armed = telemetry.armed
            if state is not None and self._publish_mission_event is not None:
                self._publish_mission_event(MissionEventRecord(
                    self.run_id,
                    telemetry.timestamp_ns,
                    self._next_event_id,
                    "MOVING_PAD",
                    state,
                    "observed vehicle state",
                ))
                self._next_event_id += 1

    @property
    def precision_profile_ready(self) -> bool:
        return not self._moving_required or set(self._parameters) == set(MOVING_PRECISION_PARAMETERS)

    def tick(self, timestamp_ns: int | None, *, mission_running: bool) -> None:
        if timestamp_ns is None:
            return
        self._clock_ns = timestamp_ns
        if self._moving_vision is not None:
            observation = self._moving_vision.latest()
            if observation is not None:
                self.operations.observe_precision(observation)
        self.operations.tick(timestamp_ns)
        if self.error is not None:
            return
        if (
            mission_running
            and self._precision_step_index is not None
            and self._settle_by_ns is not None
            and self.mission.step_index < self._precision_step_index
            and timestamp_ns > self._settle_by_ns
        ):
            self.fail(
                "moving-pad approach did not reach precision settlement by "
                f"{self._settle_by_ns / 1e9:g} simulated seconds"
            )
            return
        if not self.started:
            if (
                not mission_running
                or not self.operations.mission_ready
                or not self.precision_profile_ready
            ):
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
        self.mission.abort(reason)
        self.lifecycle.observe_terminal(MissionState(
            MissionPhase.FAILED, last_timestamp_ns=self._clock_ns, failure_reason=reason))
        if attempt_recovery:
            self._recovery_id = self.operations.recover_land()

    def stop_for_finalization(self, reason: str) -> None:
        if self.error is None:
            self.fail(reason, attempt_recovery=False)
        if self.recovery_pending:
            self.operations.abort("recovery cancelled by global finalization")

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

    moving_required = any(
        step.tool == "precision_land" for step in config.mission_plan.steps
    )
    if moving_required:
        from drone.sensors.camera._camera_manager import CameraManager
        from drone.sensors.camera.camera import Camera
        from sensor_msgs.msg import Image, LaserScan
        from simulation_interfaces.msg import MissionEvent

        from .comp2026_host import RosFrameSource
        from .moving_precision import AttitudeEvidence, RangeEvidence
        from .moving_vision import MovingVision

    protocol = _ProductionProtocol(config)
    lifecycle = CompanionLifecycle(run_id=config.run_id, protocol=protocol, stream=sys.stdout)
    started = time.monotonic()
    overall_deadline = started + config.max_wall_seconds
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
    moving_vision = None
    frame_source = None
    sensor_lock = threading.Lock()
    latest_range = None
    latest_attitude = None
    mission_event_publisher = None

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

    def publish_mission_event(record):
        assert mission_event_publisher is not None
        message = MissionEvent()
        message.run_id = record.run_id
        message.sim_timestamp.sec = record.sim_timestamp_ns // 1_000_000_000
        message.sim_timestamp.nanosec = record.sim_timestamp_ns % 1_000_000_000
        message.event_id = record.event_id
        message.phase = record.phase
        message.state = record.state
        message.detail = record.detail
        mission_event_publisher.publish(message)

    def image_callback(message):
        nonlocal callback_error
        try:
            assert frame_source is not None
            frame_source.accept_image(message)
        except Exception as error:
            callback_error = f"moving camera input failed: {error}"

    def range_callback(message):
        nonlocal callback_error, latest_range
        try:
            values = list(message.ranges)
            if len(values) != 1:
                raise ValueError("downward range must contain one sample")
            distance = float(values[0])
            if (
                not math.isfinite(distance)
                or distance < float(message.range_min)
                or distance > float(message.range_max)
            ):
                raise ValueError("downward range sample is invalid")
            evidence = RangeEvidence(distance, stamp_ns(message.header.stamp))
            with sensor_lock:
                latest_range = evidence
        except Exception as error:
            callback_error = f"moving range input failed: {error}"

    def current_range():
        with sensor_lock:
            return latest_range

    def current_attitude():
        with sensor_lock:
            return latest_attitude

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
        publish_event = None
        if moving_required:
            mission_event_publisher = node.create_publisher(
                MissionEvent,
                "/simulation/mission_events",
                QoSProfile(
                    depth=1000,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
            publish_event = publish_mission_event
            frame_source = RosFrameSource(width_px=640, height_px=480)
            module_path = Path(__file__).parent
            manager = CameraManager(
                frame_source=frame_source,
                calibration_path=module_path / "moving_pad_camera_calibration.json",
                clock=lambda: latest_clock_ns if latest_clock_ns is not None else 0,
                max_exposure_age_ns=250_000_000,
            )
            camera = Camera(
                100,
                manager=manager,
                mounting_path=module_path / "moving_pad_camera_mounting.json",
            )
            moving_vision = MovingVision(
                camera,
                latest_range=current_range,
                latest_attitude=current_attitude,
            )
            node.create_subscription(
                Image,
                "/camera/onboard/image_raw",
                image_callback,
                QoSProfile(depth=4, reliability=ReliabilityPolicy.RELIABLE),
            )
            node.create_subscription(
                LaserScan,
                "/competition/range/downward",
                range_callback,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            )
            moving_vision.start()
        host = ConfiguredHost(
            config.mission_plan,
            vehicle,
            lifecycle,
            protocol,
            config.run_id,
            moving_vision=moving_vision,
            publish_mission_event=publish_event,
        )
        for signum in (signal.SIGTERM, signal.SIGINT):
            previous_handlers[signum] = signal.signal(signum, stop)
        telemetry_requested = False
        parameters_requested = False
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            for _ in range(100):
                telemetry = vehicle.poll(latest_clock_ns if latest_clock_ns is not None else 0)
                if telemetry is None:
                    break
                if moving_required and telemetry.attitude_rpy_rad is not None:
                    assert telemetry.attitude_timestamp_ns is not None
                    with sensor_lock:
                        latest_attitude = AttitudeEvidence(
                            telemetry.attitude_rpy_rad,
                            telemetry.attitude_timestamp_ns,
                        )
                host.observe(telemetry)
                if telemetry.status_text is not None:
                    lifecycle.emit("mavlink_status_text", latest_clock_ns, {"text": telemetry.status_text})
                if telemetry.heartbeat and not telemetry_requested:
                    vehicle.request_telemetry(rate_hz=10)
                    telemetry_requested = True
                if telemetry.heartbeat and moving_required and not parameters_requested:
                    vehicle.request_parameters(tuple(MOVING_PRECISION_PARAMETERS))
                    parameters_requested = True
            now = time.monotonic()
            if callback_error is not None or now >= overall_deadline:
                host.fail(callback_error or "configured mission wall deadline expired", attempt_recovery=False)
                break
            stopping = requested_stop or finalizing or protocol.read_finalize_request() is not None
            if stopping:
                if host.mission.state != "succeeded" or host.error is not None:
                    host.stop_for_finalization(
                        "configured mission interrupted before completion"
                    )
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
        if frame_source is not None:
            frame_source.stop("configured runtime teardown")
        if moving_vision is not None and not moving_vision.close(
            min(config.finalization_wall_seconds, 5.0)
        ):
            if host is not None:
                host.fail("moving camera worker did not stop", attempt_recovery=False)
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
