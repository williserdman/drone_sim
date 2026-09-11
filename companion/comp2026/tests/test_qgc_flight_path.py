"""Bounded, hardware-free traces through QGC admission and real flight methods.

All coordinates, identities, timing limits, and FC observations below are test
fixtures.  The fake FC confirms commands with independent FlightState updates;
accepting an outbound command alone never advances an operation.
"""

import json
import signal
import queue
import threading
import time as system_time
from dataclasses import replace
from types import MappingProxyType, SimpleNamespace

import pytest
from pymavlink import mavutil

from drone import timebase
from drone.auto_attempt import PHYSICAL_EVIDENCE_INTERVAL_SECONDS
from drone.control import drone_control as drone_control_module
from drone.control import mission_supervisor as mission_supervisor_module
from dronekit.mavlink import MAVWriter
from drone.common_types import GPSCoord, MissionHome, RelPosComplete
from drone.control.drone_control import CommandAckTracker, DroneControl, MissionAckTracker
from drone.control.flight_profile import FlightProfile, TelemetryRequest
from drone.control.flight_state import (
    FailsafeEvidence,
    FlightState,
    RCModeBand,
    SourceIdentity,
)
from drone.control.listener import (
    ACK_ACCEPTED,
    ACK_CANCELLED,
    ACK_DENIED,
    ACK_FAILED,
    ACK_IN_PROGRESS,
    CommandExecutionOwner,
    LiveComponentFactories,
    LiveListenerRuntime,
    QGCCommandListener,
    TelemetryStartupEvidence,
    build_live_listener,
    load_listener_artifacts,
)
from drone.control.listener_runtime import InjectedComponentConfig
from drone.control.mission_info import MissonTracker
from drone.control.mission_supervisor import (
    AuthorityLost,
    FM1,
    MissionAbort,
    MissionSupervisor,
    RecoveryPolicy,
)
from drone.control.mission_supervisor import FM3
from drone.control.mission_supervisor import FM2
from drone.control.stability import ReleaseStabilityConfig
from drone.missions.fm1 import fm1
from drone.missions.fm2 import fm2
from drone.mock_mission import PrecisionMissionPolicy, fm3
from drone.sensors.camera._camera_manager import FrameMetadata
from drone.sensors.lidar.clearance import ClearanceCalibration
from drone.sensors.lidar.lidar import LidarSample


BOUNDS = {name: 30.0 for name in (
    "heartbeat", "mode", "location", "velocity", "attitude",
    "landed_state", "armed", "home", "rc_input", "range", "failsafe",
)}
H = MissionHome(41.0, -81.0, 250.0)
L = GPSCoord(41.0001, -81.0001, 10.0)
DROP = GPSCoord(41.0002, -81.0002, 10.0)
WM1 = GPSCoord(41.0003, -81.0003, 10.0)
PICKUP = GPSCoord(41.0004, -81.0004, 10.0)


class AdvancingClock:
    def __init__(self):
        self.value = 100.0
        self.epoch_base = system_time.time()
        self.pending = []
        self.tick_callbacks = []
        self.error = None

    def monotonic(self):
        if self.error is not None:
            raise self.error
        return self.value

    def now(self):
        return self.value

    def epoch(self):
        return self.epoch_base + self.value - 100.0

    def sleep(self, seconds):
        self.value += seconds
        pending, self.pending = self.pending, []
        for callback in pending:
            callback()
        for callback in tuple(self.tick_callbacks):
            callback()

    def later(self, callback):
        self.pending.append(callback)


class Packet:
    def __init__(self, command=FM1, *, attempt_id=7):
        self.command = command
        self.target_system = 1
        self.target_component = 191
        for index, value in enumerate((float(attempt_id), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1):
            setattr(self, f"param{index}", value)

    def get_type(self):
        return "COMMAND_LONG"

    def get_srcSystem(self):
        return 200

    def get_srcComponent(self):
        return 190


class CallbackTransport:
    source_identity = SourceIdentity(1, 191)
    wire_protocol = "2.0"

    def __init__(self, *, wire_protocol="2.0"):
        self.wire_protocol = wire_protocol
        self.callbacks = {}
        self.acks = []

    def install_message_callback(self, names, callback):
        self.callbacks.update({name: callback for name in names})

    def send_command_ack(self, ack):
        self.acks.append(ack)

    def deliver(self, packet):
        self.callbacks[packet.get_type()](packet)


class InertMav:
    def __init__(self, harness):
        self.harness = harness

    def statustext_send(self, _severity, _text):
        pass

    def mission_item_int_send(self, *fields):
        guarded_writer = getattr(self.harness, "guarded_writer", None)
        if guarded_writer is not None:
            guarded_writer.write(repr(fields).encode())
        self.harness.events.append(("waypoint", fields[11], fields[12], fields[13]))
        self.harness.mission_ack()
        if self.harness.confirm_waypoint:
            self.harness.publish(
                location=(fields[11], fields[12], int(round(fields[13] * 1000)), 10_000)
            )
        callback = getattr(self.harness, "after_waypoint_output", None)
        if callback is not None:
            self.harness.after_waypoint_output = None
            callback()


class InertMessageFactory:
    def __init__(self, harness):
        self.harness = harness

    def command_long_encode(self, *fields):
        return ("command-long", fields)

    def set_position_target_local_ned_encode(self, *fields):
        return ("relative", fields)

    def landing_target_encode(self, *fields):
        return ("landing-target", fields)


class InertVehicle:
    """FC transport adapter: outputs and observations are deliberately separate."""

    def __init__(self, harness):
        self.harness = harness
        self._mode = SimpleNamespace(name="GUIDED")
        self._armed = False
        self.after_mode_output = None
        self.is_armable = True
        self._master = SimpleNamespace(
            mav=InertMav(harness), WIRE_PROTOCOL_VERSION="2.0"
        )
        self.message_factory = InertMessageFactory(harness)
        self._mode_mapping = {"GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}
        self.parameters = dict(drone_control_module.PRECISION_LANDING_PARAMETERS)
        self.listeners = {}
        self._handler = SimpleNamespace(target_system=1, target_component=1)

    def close(self):
        return None

    def on_message(self, name):
        def install(callback):
            self.listeners[name] = callback
            return callback

        return install

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        self._mode = value
        self.harness.events.append(("mode", value.name))
        self.harness.ack(mavutil.mavlink.MAV_CMD_DO_SET_MODE)
        self.harness.publish(mode=value.name)
        if self.after_mode_output is not None:
            self.after_mode_output()
        if value.name == "LAND" and self.harness.confirm_touchdown:
            self.harness.publish(landed_state=mavutil.mavlink.MAV_LANDED_STATE_LANDING)
            def touchdown():
                location = self.harness.state.snapshot().location.observation.value
                site_amsl_mm = {
                    410000000: 250000,
                    410001000: 251000,
                    410004000: 252000,
                }.get(location[0], 250000)
                self.harness.publish(
                    landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                    location=(location[0], location[1], site_amsl_mm, 0),
                )

            self.harness.clock.later(touchdown)

    @property
    def armed(self):
        return self._armed

    @armed.setter
    def armed(self, value):
        self._armed = value
        self.harness.events.append(("armed", value))
        self.harness.ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if value:
            location = self.harness.state.snapshot().location.observation.value
            if hasattr(self.harness, "causal_home_trace"):
                self.harness.publish(home=location[:3])
                self.harness.causal_home_trace.append(
                    ("premature-home", location[:3], self.harness.sequence)
                )
        self.harness.publish(armed=value)
        if value:
            if hasattr(self.harness, "causal_home_trace"):
                self.harness.causal_home_trace.append(
                    (
                        "armed-confirmed",
                        location[:3],
                        self.harness.sequence,
                        self.harness.permission_checks,
                    )
                )
            else:
                self.harness.publish(home=location[:3])

    @property
    def groundspeed(self):
        return 0.0

    @groundspeed.setter
    def groundspeed(self, value):
        self.harness.events.append(("groundspeed", value))
        self.harness.ack(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED)

    def simple_takeoff(self, fc_relative_altitude):
        if hasattr(self.harness, "causal_home_trace"):
            home = self.harness.state.snapshot().home
            self.harness.causal_home_trace.append(
                ("takeoff", fc_relative_altitude, home.observation.sequence)
            )
        self.harness.events.append(("takeoff", fc_relative_altitude))
        self.harness.ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        location = self.harness.state.snapshot().location.observation.value
        self.harness.publish(
            landed_state=mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
            location=(location[0], location[1], 260000, int(round(fc_relative_altitude * 1000))),
        )

    def send_mavlink(self, message):
        guarded_writer = getattr(self.harness, "guarded_writer", None)
        if guarded_writer is not None:
            guarded_writer.write(repr(message).encode())
        if message[0] == "command-long":
            fields = message[1]
            command = fields[2]
            if command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                mode_id = int(fields[5])
                name = next(
                    name for name, value in self._mode_mapping.items() if value == mode_id
                )
                self.mode = SimpleNamespace(name=name)
                return
            if command == mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
                self.groundspeed = fields[5]
                return
            if command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                self.armed = fields[4] == 1.0
                return
            if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                self.simple_takeoff(fields[10])
                return
            message_id = fields[4]
            assert fields[:4] == (
                1,
                1,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
            )
            assert message_id == float(mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION)
            assert fields[5:] == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            location = self.harness.state.snapshot().location.observation.value
            self.harness.causal_home_trace.append(
                ("home-request", location[:3], self.harness.permission_checks)
            )
            self.harness.ack(command)
            self.harness.causal_home_trace.append(("home-ack", command))

            def publish_requested_home():
                self.harness.publish(home=location[:3])
                self.harness.causal_home_trace.append(
                    ("home-response", location[:3], self.harness.sequence)
                )

            self.harness.clock.later(publish_requested_home)
            return
        self.harness.events.append(("mavlink", message[0]))
        if message[0] == "relative":
            location = self.harness.state.snapshot().location.observation.value
            self.harness.publish(location=location)

    def flush(self):
        self.harness.events.append(("flush",))


class InertLidar:
    def __init__(self, harness, distance_m=1.5):
        self.harness = harness
        self.distance_m = distance_m
        self.sequence = 0
        self.sample = None
        self.refresh()
        harness.clock.tick_callbacks.append(self.refresh)

    def refresh(self):
        self.sequence += 1
        self.harness.publish(
            location=self.harness.state.snapshot().location.observation.value,
            velocity=(0, 0, 0),
            attitude=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        self.sample = LidarSample(
            self.distance_m,
            self.harness.clock.monotonic(),
            self.sequence,
            0,
        )

    def get_sample(self):
        return self.sample

    def stop(self, *, timeout_seconds):
        return SimpleNamespace(worker_stopped=True, cleanup_completed=True)


class InertDropper:
    supports_attachment = True

    def __init__(self, events):
        self.events = events

    def drop(self):
        self.events.append(("payload-release-attempt", None))

    def attach(self, target_id):
        self.events.append(("payload-attached", target_id))
        return True


class InertCamera:
    def __init__(self, harness, *, prepared=True):
        self.harness = harness
        self.sequence = 0
        self.prepared = prepared

    def prepare_precision_readiness(self, *, timeout_s):
        self.harness.events.append(("camera-prepared", timeout_s))
        self.prepared = True
        return SimpleNamespace(ready=True, reasons=())

    def precision_readiness(self):
        return SimpleNamespace(ready=self.prepared, reasons=(() if self.prepared else ("no frame",)))

    def vec_to_marker_3d_bounded(
        self, target_id, *, timeout_s, after_sequence=0, quality=4
    ):
        self.sequence += 1
        self.harness.events.append(
            ("camera", target_id, self.sequence, after_sequence, quality)
        )
        now_ns = int(self.harness.clock.monotonic() * 1_000_000_000)
        return RelPosComplete(0.0, 0.0, 1.5), FrameMetadata(
            sequence=self.sequence,
            exposure_timestamp_ns=now_ns,
            receipt_timestamp_ns=now_ns,
            exposure_age_ns=0,
            exposure_age_bounded=True,
            raw_image_size_px=(640, 480),
            image_size_px=(640, 480),
        )


class FactoryRig:
    """Factories for the public live composition, all explicitly test-only."""

    def __init__(self, clock):
        self.clock = clock
        self.events = []
        self.sequence = 0
        self.confirm_waypoint = True
        self.confirm_touchdown = True
        self.state = None
        self.controller = None
        self.transport = None
        self.permission_checks = 0
        self.causal_home_trace = []
        self.release_evidence = []
        self.release_source_times = []
        self.release_partitions = []
        self.release_boundaries = []
        self._release_partition_start = 0
        self.inject_release_invalidation = False
        self.release_invalidation_injected = False
        self.outbound = None
        self.guarded_writer = None
        self.accepted_simulation_ns = 0

    def publish(self, **values):
        self.sequence += 1
        assert self.state.update_many(
            values,
            received_at=self.clock.monotonic(),
            sequence=self.sequence,
            source_system=1,
            source_component=1,
        )

    def ack(self, command):
        self.controller._command_ack_tracker.observe(
            command=command,
            result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
            source_system=1,
            source_component=1,
            target_system=1,
            target_component=191,
        )

    def mission_ack(self):
        self.controller._mission_ack_tracker.observe_message(SimpleNamespace(
            type=mavutil.mavlink.MAV_MISSION_ACCEPTED,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            target_system=1,
            target_component=191,
            get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 1,
        ))

    def refresh(self):
        self.publish_rc_health()
        snapshot = self.state.snapshot()
        if (
            self.inject_release_invalidation
            and not self.release_invalidation_injected
            and self.release_evidence
        ):
            self.sequence += 1
            assert self.state.invalidate_evidence(
                ("attitude",),
                received_at=self.clock.monotonic(),
                sequence=self.sequence,
                source_system=1,
                source_component=1,
            )
            self.release_invalidation_injected = True
        self.publish(**{
            name: getattr(snapshot, name).observation.value
            for name in (
                "heartbeat", "mode", "location", "velocity", "attitude",
                "landed_state", "armed", "failsafe",
            )
        })
        self.sequence += 1
        assert self.state.observe_rc_input(
            channel=7, pwm=1500, signal_healthy=True,
            received_at=self.clock.monotonic(), sequence=self.sequence,
            source_system=1, source_component=1,
        )

    def publish_rc_health(self):
        self.decoders.observe_sys_status(FCMessage(
            onboard_control_sensors_present=65_536,
            onboard_control_sensors_enabled=65_536,
            onboard_control_sensors_health=65_536,
        ))

    def controller_factory(
        self, *, config, flight_state, permission_guard, decoders
    ):
        self.state = flight_state
        self.decoders = decoders
        self.outbound = queue.Queue()
        self.guarded_writer = drone_control_module._OutputGuardedWriter(
            MAVWriter(self.outbound)
        )
        vehicle = InertVehicle(self)
        controller = object.__new__(DroneControl)
        controller.vehicle = vehicle
        controller.source_identity = config.connection.source_identity
        controller.flight_controller_target = config.connection.target_identity
        controller.flight_state = flight_state
        def counted_permission_guard():
            self.permission_checks += 1
            return permission_guard()

        controller.permission_guard = counted_permission_guard
        controller.mission_home_check = config.operating_site.mission_home_check
        controller.fc_home_position_tolerance_m = config.fc_home_position_tolerance_m
        controller.fc_home_altitude_tolerance_m = config.fc_home_altitude_tolerance_m
        controller.clearance_calibration = config.clearance_calibration
        controller.release_stability_config = config.release_stability
        controller.home_request_timeout_s = config.telemetry.home_request_timeout_s
        controller.telemetry_poll_interval_s = config.telemetry.poll_interval_s
        controller.cruise_alt = config.cruise_altitude_m
        controller._mission_home = None
        controller._last_arm_boundary_sequence = None
        controller._release_hold_confirmation = None
        controller._command_ack_tracker = CommandAckTracker(
            wire_protocol="2.0", source_system=1, source_component=1,
            target_system=1, target_component=191,
        )
        controller._mission_ack_tracker = MissionAckTracker(
            source_system=1, source_component=1,
            target_system=1, target_component=191,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        controller._mission_ack_transaction_lock = threading.Lock()
        controller._telemetry_transaction_lock = threading.Lock()
        controller._transport_output_transaction = self.guarded_writer.transaction
        original_release_evidence = controller._release_evidence

        def record_release_evidence(*args, **kwargs):
            evidence = original_release_evidence(*args, **kwargs)
            self.release_evidence.append(evidence)
            snapshot, _coord, lidar, _required_agl_m = args
            range_sample = lidar.get_sample()
            self.release_source_times.append((
                snapshot.location.observation.received_at,
                snapshot.velocity.observation.received_at,
                snapshot.attitude.observation.received_at,
                range_sample.sampled_at,
            ))
            return evidence

        controller._release_evidence = record_release_evidence
        original_actuate_guarded = controller._actuate_guarded

        def record_release_partition(output, *, validate_snapshot=None):
            output_times = []

            def record_actual_output():
                output_times.append(self.clock.monotonic())
                return output()

            boundary = original_actuate_guarded(
                record_actual_output, validate_snapshot=validate_snapshot
            )
            if validate_snapshot is not None:
                self.release_partitions.append(
                    tuple(self.release_evidence[self._release_partition_start:])
                )
                self.release_boundaries.append((
                    output_times[0],
                    boundary,
                    self.release_evidence[-1],
                    self.release_source_times[-1],
                ))
                self._release_partition_start = len(self.release_evidence)
            return boundary

        controller._actuate_guarded = record_release_partition
        self.controller = controller
        return controller

    def telemetry_collector_factory(self, **_kwargs):
        def publish_baseline():
            self.publish_rc_health()
            self.publish(
                heartbeat="fixture",
                mode="GUIDED",
                location=(410000000, -810000000, 250000, 0),
                velocity=(0, 0, 0),
                attitude=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                armed=False,
                home=(410000000, -810000000, 250000),
                failsafe=FailsafeEvidence(active=False, reason="test clear"),
            )
            self.sequence += 1
            assert self.state.observe_rc_input(
                channel=7, pwm=1500, signal_healthy=True,
                received_at=self.clock.monotonic(), sequence=self.sequence,
                source_system=1, source_component=1,
            )

        def configure_and_collect():
            publish_baseline()
            self.clock.tick_callbacks.append(self.refresh)
            return TelemetryStartupEvidence(MappingProxyType({}), 1.0)

        def prepare():
            publish_baseline()
            self.clock.tick_callbacks.append(self.refresh)

        def verify_after_guided():
            assert self.accepted_simulation_ns > 0
            self.events.append(("telemetry-verified-after-guided", self.accepted_simulation_ns))
            return TelemetryStartupEvidence(MappingProxyType({}), 1.0)

        return SimpleNamespace(
            configure_and_collect=configure_and_collect,
            prepare=prepare,
            verify_after_guided=verify_after_guided,
            close=lambda: None,
        )

    def factories(self, *, staged=False):
        def ack_transport_factory(**_kwargs):
            self.transport = CallbackTransport(wire_protocol="2.0")
            return self.transport

        return LiveComponentFactories(
            backend=(
                "drone-sim-ros-confirmed-v1"
                if staged
                else "test-injected-components-v1"
            ),
            controller_factory=self.controller_factory,
            ack_transport_factory=ack_transport_factory,
            telemetry_collector_factory=self.telemetry_collector_factory,
            lidar_factory=lambda **_kwargs: InertLidar(self),
            dropper_factory=lambda **_kwargs: InertDropper(self.events),
            camera_factory=lambda **_kwargs: InertCamera(self, prepared=False),
            supports_attachment=True,
            telemetry_startup_mode=("staged_simulation" if staged else "complete"),
        )


def flight_profile():
    return FlightProfile(
        profile_id="qgc-flight-path-fixture",
        raw_sha256="a" * 64,
        firmware="ArduCopter test fixture",
        qgc_source=SourceIdentity(200, 190),
        companion_target=SourceIdentity(1, 191),
        flight_controller=SourceIdentity(1, 1),
        freshness_bounds=MappingProxyType(BOUNDS),
        rc_health_max_age=1.0,
        rc_channel=7,
        rc_mode_bands=(
            RCModeBand("companion", 1400, 1600, "GUIDED"),
            RCModeBand("pilot", 1800, 2100, "LOITER"),
        ),
        heartbeat_type=2,
        heartbeat_autopilot=3,
        copter_modes=MappingProxyType({0: "STABILIZE", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}),
        failsafe_active_statuses=frozenset((5,)),
        failsafe_clear_statuses=frozenset((3, 4)),
        decoder_contract_version=1,
        decoder_contract_evidence="sha256:" + "b" * 64,
        decoder_contract_reference="test fixture",
        telemetry_requests=(TelemetryRequest(0, 500_000),),
    )


class FlightPathHarness:
    def __init__(
        self,
        *,
        confirm_waypoint=True,
        confirm_touchdown=True,
        real_recovery=False,
    ):
        self.clock = AdvancingClock()
        self.events = []
        self.sequence = 0
        self.confirm_waypoint = confirm_waypoint
        self.confirm_touchdown = confirm_touchdown
        self.real_recovery = real_recovery
        self.after_waypoint_output = None
        self.state = FlightState(
            source_system=1,
            source_component=1,
            freshness_bounds=BOUNDS,
            rc_channel=7,
            rc_mode_mapping=flight_profile().rc_mode_bands,
            clock=self.clock.monotonic,
        )
        self.vehicle = InertVehicle(self)
        self.controller = object.__new__(DroneControl)
        self.controller.vehicle = self.vehicle
        self.controller.source_identity = SourceIdentity(1, 191)
        self.controller.flight_controller_target = SourceIdentity(1, 1)
        self.controller.flight_state = self.state
        self.controller.permission_guard = lambda: self.supervisor.check_permission()
        self.controller.mission_home_check = lambda _home: None
        self.controller.fc_home_position_tolerance_m = 2.0
        self.controller.fc_home_altitude_tolerance_m = 2.0
        self.controller.cruise_alt = 10.0
        self.controller._mission_home = H
        self.controller._last_arm_boundary_sequence = None
        self.controller._release_hold_confirmation = None
        self.controller.clearance_calibration = ClearanceCalibration(
            beam_direction_body_frd=(0.0, 0.0, 1.0),
            measured_reference_offset_body_frd_m=(0.0, 0.0, 0.0),
            lidar_mounting_offset_already_applied=True,
            max_tilt_rad=0.2,
            max_age_seconds=0.25,
            max_skew_seconds=0.05,
            locally_horizontal_planar_surface=True,
        )
        self.controller.release_stability_config = ReleaseStabilityConfig(
            hold_seconds=0.2,
            timeout_seconds=2.0,
            max_horizontal_speed_m_s=0.2,
            max_vertical_speed_m_s=0.2,
            max_roll_rad=0.1,
            max_pitch_rad=0.1,
            horizontal_position_tolerance_m=0.3,
            vertical_position_tolerance_m=0.2,
            max_observation_skew_seconds=0.05,
            max_observation_gap_seconds=0.2,
            poll_interval_seconds=0.05,
            waypoint_reissue_interval_seconds=0.5,
        )
        self.controller._command_ack_tracker = CommandAckTracker(
            wire_protocol="2.0", source_system=1, source_component=1,
            target_system=1, target_component=191,
        )
        self.controller._mission_ack_tracker = MissionAckTracker(
            source_system=1, source_component=1,
            target_system=1, target_component=191,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        self.controller._mission_ack_transaction_lock = threading.Lock()
        self.publish(
            heartbeat="fixture",
            mode="GUIDED",
            location=(410000000, -810000000, 250000, 0),
            velocity=(0, 0, 0),
            attitude=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
            armed=False,
            failsafe=FailsafeEvidence(active=False, reason="test clear"),
        )
        self.sequence += 1
        assert self.state.observe_rc_input(
            channel=7, pwm=1500, signal_healthy=True,
            received_at=self.clock.monotonic(), sequence=self.sequence,
            source_system=1, source_component=1,
        )
        assert self.state.acquire_initial_companion_authority(now=self.clock.monotonic())
        self.clock.tick_callbacks.append(self.refresh_flight_observations)
        self.consumed = []
        self.supervisor = MissionSupervisor(
            7,
            admission_check=lambda _envelope: None,
            attempt_consumer=lambda token: self.consumed.append(token),
            permission_check=self.permission_check,
            recovery_policy=RecoveryPolicy(
                check=lambda _operation, _home, _altitude: None,
                timeout_s=300.0,
                local_land_reserve_s=10.0,
                clock=self.clock.monotonic,
            ),
        )
        self.controller.install_output_transactions(
            dependency_transaction=lambda operation: operation(),
            supervisor_transaction=self.supervisor.output_transaction,
            transport_transaction=lambda operation, enqueue_check: (
                enqueue_check(), operation()
            )[1],
        )
        self.controller.install_startup_telemetry_verifier(None, verified=True)
        self.commands = queue.Queue()
        self.transport = CallbackTransport()
        self.listener = QGCCommandListener(
            supervisor=self.supervisor,
            profile=flight_profile(),
            command_queue=self.commands,
            clock=self.clock.monotonic,
            ack_transport=self.transport,
            wire_protocol="2.0",
        )
        self.listener.install()
        tracker = MissonTracker(600)
        tracker.begin_mission()
        self.lidar = InertLidar(self)
        self.dropper = InertDropper(self.events)
        self.camera = InertCamera(self)
        self.precision_policy = PrecisionMissionPolicy(
            clearance_calibration=self.controller.clearance_calibration,
            clock=timebase.monotonic,
            max_exposure_age_s=0.2,
            max_image_attitude_skew_s=0.2,
            max_image_location_skew_s=0.2,
            max_attitude_transport_latency_s=0.01,
            max_location_transport_latency_s=0.01,
            acquisition_timeout_s=5.0,
            frame_timeout_s=0.2,
            observation_period_s=0.05,
            target_hover_height_m=1.5,
            hover_tolerance_m=0.2,
            centered_tolerance_m=0.2,
            correction_gain=0.5,
            cruise_altitude_m=10.0,
            desired_drop_height_m=1.5,
            landing_timeout_s=60.0,
            target_loss_timeout_s=0.5,
            reacquisition_count=5,
        )
        self.tracker = tracker
        self.handler_errors = []
        self.recoveries = []
        self.owner = CommandExecutionOwner(
            supervisor=self.supervisor,
            command_queue=self.commands,
            handlers={
                FM1: lambda _envelope: fm1(tracker, self.controller, 10, L),
                FM2: lambda _envelope: self.run_fm2(tracker),
                FM3: lambda _envelope: self.run_fm3(),
            },
            terminal_ack=self.listener.acknowledge_terminal,
            recovery=self.recover,
            attempt_timeout_s=600,
            idle_poll_s=0.01,
            clock=self.clock.monotonic,
        )

    def permission_check(self):
        if not self.state.ordinary_commands_permitted():
            from drone.control.mission_supervisor import AuthorityLost
            raise AuthorityLost("fixture authority is not companion")

    def run_fm2(self, tracker):
        try:
            return fm2(
                tracker,
                self.controller,
                10,
                DROP,
                self.dropper,
                self.lidar,
                desired_drop_height_m=1.5,
            )
        except Exception as error:
            self.handler_errors.append(
                f"{error!r}; cause={error.__cause__!r}; context={error.__context__!r}"
            )
            raise

    def refresh_flight_observations(self):
        snapshot = self.state.snapshot()
        values = {
            name: getattr(snapshot, name).observation.value
            for name in (
                "heartbeat", "mode", "location", "velocity", "attitude",
                "landed_state", "armed", "failsafe",
            )
        }
        if snapshot.home is not None:
            values["home"] = snapshot.home.observation.value
        self.publish(**values)
        self.sequence += 1
        assert self.state.observe_rc_input(
            channel=7, pwm=1500, signal_healthy=True,
            received_at=self.clock.monotonic(), sequence=self.sequence,
            source_system=1, source_component=1,
        )

    def recover(self):
        if not self.real_recovery:
            self.recoveries.append("recovery")
            return None
        self.confirm_waypoint = True
        snapshot = self.state.snapshot()
        if snapshot.authority.name == "COMPANION":
            self.publish(
                heartbeat="fixture",
                mode="GUIDED",
                location=snapshot.location.observation.value,
                landed_state=mavutil.mavlink.MAV_LANDED_STATE_IN_AIR,
                armed=True,
            )
        try:
            return self.supervisor.recover(self.controller, H, 10.0)
        finally:
            self.recoveries.append(self.supervisor.recovery_outcome)

    def run_fm3(self):
        if fm3(
            self.tracker,
            self.controller,
            self.camera,
            self.lidar,
            self.dropper,
            {3},
            PICKUP,
            WM1,
            precision_policy=self.precision_policy,
        ) is not True:
            return False
        assert self.controller.goto_waypoint(GPSCoord(H.lat, H.lon, 10.0)) == 0
        assert self.controller.simple_land() == 0
        assert self.controller.disarm() == 0
        return True

    def publish(self, **values):
        self.sequence += 1
        assert self.state.update_many(
            values,
            received_at=self.clock.monotonic(),
            sequence=self.sequence,
            source_system=1,
            source_component=1,
        )

    def ack(self, command):
        self.controller._command_ack_tracker.observe(
            command=command,
            result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
            source_system=1,
            source_component=1,
            target_system=1,
            target_component=191,
        )

    def mission_ack(self):
        self.controller._mission_ack_tracker.observe_message(SimpleNamespace(
            type=mavutil.mavlink.MAV_MISSION_ACCEPTED,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            target_system=1,
            target_component=191,
            get_srcSystem=lambda: 1,
            get_srcComponent=lambda: 1,
        ))


def test_fresh_qgc_fm1_uses_real_controller_observations_before_landing():
    """Break caught: ACK-only takeoff/navigation must not complete FM1."""
    harness = FlightPathHarness()

    with timebase.configured(harness.clock):
        harness.transport.deliver(Packet())
        assert harness.owner.process_next(timeout_s=0) == "SUCCEEDED"

    assert harness.consumed == [7]
    assert [ack.result for ack in harness.transport.acks] == [ACK_IN_PROGRESS, ACK_ACCEPTED]
    assert harness.events == [
        ("mode", "GUIDED"),
        ("armed", True),
        ("takeoff", 10.0),
        ("groundspeed", 10.0),
        ("waypoint", 410001000, -810001000, 260.0),
        ("mode", "LAND"),
        ("armed", False),
    ]
    assert harness.supervisor.status(FM1) == "TERMINAL"
    assert harness.supervisor.terminal_result is None
    assert harness.recoveries == []


def test_qgc_fm1_ack_without_waypoint_observation_fails_and_never_lands():
    """Break caught: transport acceptance must not substitute for arrival evidence."""
    harness = FlightPathHarness(confirm_waypoint=False)

    with timebase.configured(harness.clock):
        harness.transport.deliver(Packet())
        assert harness.owner.process_next(timeout_s=0) == "FAILED"

    assert [ack.result for ack in harness.transport.acks] == [ACK_IN_PROGRESS, ACK_FAILED]
    assert ("mode", "LAND") not in harness.events
    assert ("armed", False) not in harness.events
    assert harness.supervisor.terminal_result == "FAILED"
    assert harness.recoveries == ["recovery"]


def test_qgc_fm1_land_ack_without_touchdown_fails_and_never_disarms():
    """Break caught: accepted LAND mode must not stand in for touchdown."""
    harness = FlightPathHarness(confirm_touchdown=False)

    with timebase.configured(harness.clock):
        harness.transport.deliver(Packet())
        assert harness.owner.process_next(timeout_s=0) == "FAILED"

    assert ("mode", "LAND") in harness.events
    assert ("armed", False) not in harness.events
    assert harness.supervisor.terminal_result == "FAILED"
    assert harness.recoveries == ["recovery"]
    assert [ack.result for ack in harness.transport.acks] == [ACK_IN_PROGRESS, ACK_FAILED]


def test_mission_failure_is_separate_from_single_real_home_land_recovery():
    """Break caught: a recovered landing must not turn mission failure into success."""
    harness = FlightPathHarness(confirm_waypoint=False, real_recovery=True)

    with timebase.configured(harness.clock):
        harness.transport.deliver(Packet())
        assert harness.owner.process_next(timeout_s=0) == "FAILED"
        assert harness.owner.process_next(timeout_s=0) is None

    assert harness.supervisor.terminal_result == "FAILED"
    assert harness.supervisor.recovery_outcome == "HOME_LANDED"
    assert harness.recoveries == ["HOME_LANDED"]
    assert harness.events.count(("mode", "LAND")) == 1


def test_fm2_requires_fresh_request_and_continuous_evidence_before_payload_attempt():
    """Break caught: FM1 completion must not auto-run FM2 or release early."""
    harness = FlightPathHarness()

    with timebase.configured(harness.clock):
        harness.transport.deliver(Packet(FM1))
        assert harness.owner.process_next(timeout_s=0) == "SUCCEEDED"
        events_after_fm1 = tuple(harness.events)
        assert harness.owner.process_next(timeout_s=0) is None
        assert tuple(harness.events) == events_after_fm1

        harness.transport.deliver(Packet(FM2))
        result = harness.owner.process_next(timeout_s=0)
        assert result == "SUCCEEDED", harness.handler_errors

    release_index = harness.events.index(("payload-release-attempt", None))
    hold_waypoints = [
        event for event in harness.events[:release_index]
        if event[0] == "waypoint" and event[1:3] == (410002000, -810002000)
    ]
    assert len(hold_waypoints) >= 3
    assert harness.events[-1] == ("payload-release-attempt", None)
    assert harness.state.snapshot().armed.observation.value is True
    assert [ack.result for ack in harness.transport.acks] == [
        ACK_IN_PROGRESS,
        ACK_ACCEPTED,
        ACK_IN_PROGRESS,
        ACK_ACCEPTED,
    ]
    assert harness.supervisor.terminal_result is None


def test_fresh_qgc_fm3_attaches_after_touchdown_then_rearms_and_returns_original_home():
    """Break caught: FM3 must not attach airborne or return to a changed FC home."""
    harness = FlightPathHarness()

    with timebase.configured(harness.clock):
        for command in (FM1, FM2, FM3):
            harness.transport.deliver(Packet(command))
            result = harness.owner.process_next(timeout_s=0)
            assert result == "SUCCEEDED", "\n".join(map(str, harness.events))

    attachment = harness.events.index(("payload-attached", 3))
    rearm = harness.events.index(("armed", True), attachment)
    assert harness.events[attachment - 1] == ("armed", False)
    assert attachment < rearm
    assert harness.events[-2:] == [("mode", "LAND"), ("armed", False)]
    home_waypoints = [
        event for event in harness.events
        if event[:3] == ("waypoint", 410000000, -810000000)
    ]
    assert home_waypoints[-1][3] == 260.0
    assert [event[1] for event in harness.events if event[0] == "takeoff"] == [
        10.0,
        9.0,
        8.0,
    ]
    assert harness.supervisor.terminal_result == "SUCCEEDED"
    assert harness.recoveries == []


def test_fresh_pilot_rc_edge_suppresses_queued_fm1_outputs():
    """Break caught: a queued request must not output after confirmed RC takeover."""
    harness = FlightPathHarness()
    harness.transport.deliver(Packet())
    harness.sequence += 1
    assert harness.state.observe_rc_input(
        channel=7, pwm=1900, signal_healthy=True,
        received_at=harness.clock.monotonic(), sequence=harness.sequence,
        source_system=1, source_component=1,
    )
    assert harness.state.observe_mode(
        "LOITER", received_at=harness.clock.monotonic(), sequence=harness.sequence + 1,
        source_system=1, source_component=1,
    )

    with timebase.configured(harness.clock):
        assert harness.owner.process_next(timeout_s=0) == "FAILED"

    assert harness.events == []
    assert harness.recoveries == ["recovery"]
    assert [ack.result for ack in harness.transport.acks] == [ACK_IN_PROGRESS, ACK_FAILED]


def test_stale_permission_after_callback_prevents_all_fm1_outputs():
    """Break caught: queued work must not refresh or assume stale authority."""
    harness = FlightPathHarness()
    harness.transport.deliver(Packet())
    harness.clock.value += 31.0

    with timebase.configured(harness.clock):
        assert harness.owner.process_next(timeout_s=0) == "FAILED"

    assert harness.events == []
    assert harness.recoveries == ["recovery"]


def test_sigint_abort_during_message_pack_prevents_writer_enqueue():
    """A reentrant terminal signal aborts before DroneKit's queue boundary."""
    harness = FlightPathHarness()
    harness.publish(mode="LAND", armed=True, landed_state=1)
    outbound = queue.Queue()
    guarded_writer = drone_control_module._OutputGuardedWriter(MAVWriter(outbound))
    harness.controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=harness.supervisor.output_transaction,
        transport_transaction=guarded_writer.transaction,
    )

    class PackedDisarm:
        def pack(self, _encoder, **_kwargs):
            harness.supervisor.request_abort("SIGINT")
            return b"would-be-disarm"

    harness.vehicle.message_factory.command_long_encode = lambda *_fields: PackedDisarm()

    def send_mavlink(message):
        packet = message.pack(object())
        guarded_writer.write(packet)

    harness.vehicle.send_mavlink = send_mavlink

    with timebase.configured(harness.clock), pytest.raises(MissionAbort):
        harness.controller.disarm(timeout=0.3)

    assert outbound.empty()


def test_public_sigint_at_real_mavwriter_entry_prevents_queue_insertion(monkeypatch):
    """Break caught: completed SIGINT cannot precede a new MAVWriter enqueue."""
    harness = FlightPathHarness()
    harness.publish(
        mode="LAND",
        armed=True,
        landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
        failsafe=FailsafeEvidence(active=False, reason="test clear"),
    )
    outbound = queue.Queue()
    delegate = MAVWriter(outbound)
    guarded_writer = drone_control_module._OutputGuardedWriter(delegate)
    harness.controller.install_output_transactions(
        dependency_transaction=lambda operation: operation(),
        supervisor_transaction=harness.supervisor.output_transaction,
        transport_transaction=guarded_writer.transaction,
    )

    class PackedDisarm:
        @staticmethod
        def pack(_encoder, **_kwargs):
            return b"would-be-disarm"

    harness.vehicle.message_factory.command_long_encode = lambda *_fields: PackedDisarm()
    harness.vehicle.send_mavlink = lambda message: guarded_writer.write(
        message.pack(object())
    )
    original_write = MAVWriter.write

    def signal_at_writer_entry(writer, packet):
        signal.raise_signal(signal.SIGINT)
        return original_write(writer, packet)

    monkeypatch.setattr(MAVWriter, "write", signal_at_writer_entry)
    runtime = LiveListenerRuntime(
        listener=harness.listener,
        owner=SimpleNamespace(
            run=lambda: harness.controller.disarm(timeout=0.3),
            _idle_poll_s=0.01,
        ),
        supervisor=harness.supervisor,
        controller=harness.controller,
        flight_state=harness.state,
        tracker=SimpleNamespace(),
        lidar=SimpleNamespace(),
        camera=None,
        dropper=SimpleNamespace(),
        cleanup_timeout_s=0.1,
        diagnostics=lambda _message: None,
        monitoring_stop=threading.Event(),
    )

    with timebase.configured(harness.clock), pytest.raises(MissionAbort):
        runtime.run()

    assert harness.supervisor.abort_reason == "SIGINT"
    assert outbound.empty()


def test_public_sigint_before_output_owner_publish_prevents_gpio(monkeypatch):
    """A completed SIGINT cannot be followed by a guarded GPIO actuation."""
    harness = FlightPathHarness()
    gpio_events = []
    real_get_ident = threading.get_ident
    signal_delivered = False

    def signal_during_owner_publish():
        nonlocal signal_delivered
        if not signal_delivered:
            signal_delivered = True
            signal.raise_signal(signal.SIGINT)
        return real_get_ident()

    monkeypatch.setattr(
        mission_supervisor_module.threading,
        "get_ident",
        signal_during_owner_publish,
    )

    class UnexpectedCompletion(RuntimeError):
        pass

    def run_gpio():
        harness.controller._actuate_guarded(lambda: gpio_events.append("GPIO"))
        raise UnexpectedCompletion("GPIO transaction returned after SIGINT")

    runtime = LiveListenerRuntime(
        listener=harness.listener,
        owner=SimpleNamespace(run=run_gpio, _idle_poll_s=0.01),
        supervisor=harness.supervisor,
        controller=harness.controller,
        flight_state=harness.state,
        tracker=SimpleNamespace(),
        lidar=SimpleNamespace(),
        camera=None,
        dropper=SimpleNamespace(),
        cleanup_timeout_s=0.1,
        diagnostics=lambda _message: None,
        monitoring_stop=threading.Event(),
    )

    with timebase.configured(harness.clock), pytest.raises(MissionAbort, match="SIGINT"):
        runtime.run()

    assert signal_delivered
    assert harness.supervisor.abort_reason == "SIGINT"
    assert harness.supervisor._output_thread_id is None
    assert gpio_events == []


def test_real_decoder_writer_boundary_rejects_expired_vital_evidence():
    """Break caught: fresh SYS_STATUS cannot preserve expired flight vitals."""
    harness = FlightPathHarness()
    harness.state.freshness_bounds = MappingProxyType(
        {name: 1.0 for name in BOUNDS}
    )
    profile = replace(
        flight_profile(),
        freshness_bounds=harness.state.freshness_bounds,
        rc_health_max_age=5.0,
    )
    decoders = profile.decoders(
        clock=harness.clock.monotonic,
        flight_state=harness.state,
    )
    decoders.observe_sys_status(
        FCMessage(
            onboard_control_sensors_present=65_536,
            onboard_control_sensors_enabled=65_536,
            onboard_control_sensors_health=65_536,
        )
    )
    outbound = queue.Queue()
    guarded_writer = drone_control_module._OutputGuardedWriter(MAVWriter(outbound))
    harness.controller.install_output_transactions(
        dependency_transaction=decoders.output_transaction,
        supervisor_transaction=harness.supervisor.output_transaction,
        transport_transaction=guarded_writer.transaction,
    )

    class PackedMode:
        @staticmethod
        def pack(_encoder, **_kwargs):
            harness.clock.value += 1.6
            return b"would-be-guided-mode"

    harness.vehicle.message_factory.command_long_encode = lambda *_fields: PackedMode()
    harness.vehicle.send_mavlink = lambda message: guarded_writer.write(
        message.pack(object())
    )

    with timebase.configured(harness.clock), pytest.raises(
        AuthorityLost, match="vital flight evidence"
    ):
        harness.controller.set_guided_mode(timeout=0.3)

    assert decoders.check_rc_health() is None
    assert outbound.empty()
    assert harness.state.snapshot().authority.name == "UNKNOWN"

    harness.publish(
        heartbeat="refreshed",
        mode="GUIDED",
        failsafe=FailsafeEvidence(active=False, reason="test clear"),
    )
    harness.sequence += 1
    assert harness.state.observe_rc_input(
        channel=7,
        pwm=1500,
        signal_healthy=True,
        received_at=harness.clock.monotonic(),
        sequence=harness.sequence,
        source_system=1,
        source_component=1,
    )
    decoders.observe_sys_status(
        FCMessage(
            onboard_control_sensors_present=65_536,
            onboard_control_sensors_enabled=65_536,
            onboard_control_sensors_health=65_536,
        )
    )
    gpio_events = []

    with pytest.raises(AuthorityLost):
        harness.controller._actuate_guarded(lambda: gpio_events.append("GPIO"))
    with pytest.raises(AuthorityLost):
        harness.controller._send_guarded(lambda: guarded_writer.write(b"later"))

    assert harness.state.snapshot().authority.name == "UNKNOWN"
    assert gpio_events == []
    assert outbound.empty()


class FCMessage:
    def __init__(self, **fields):
        self.__dict__.update(fields)

    def get_srcSystem(self):
        return 1

    def get_srcComponent(self):
        return 1


def test_real_controller_callbacks_classify_takeover_failsafe_and_unknown(monkeypatch):
    """Break caught: callback evidence must never default UNKNOWN to companion."""
    clock = AdvancingClock()
    vehicle = InertVehicle(SimpleNamespace(events=[], state=None, clock=clock))
    state = FlightState(
        source_system=1,
        source_component=1,
        freshness_bounds=BOUNDS,
        rc_channel=7,
        rc_mode_mapping=flight_profile().rc_mode_bands,
        clock=clock.monotonic,
    )
    monkeypatch.setattr(
        drone_control_module, "connect", lambda *_args, **_kwargs: vehicle
    )
    with timebase.configured(clock):
        controller = DroneControl(
            "inert:test",
            source_identity=SourceIdentity(1, 191),
            flight_controller_target=SourceIdentity(1, 1),
            wire_protocol="2.0",
            wait_ready=False,
            flight_state=state,
            permission_guard=lambda: None,
            heartbeat_mode_decoder=lambda message: message.decoded_mode,
            rc_health_decoder=lambda _message, _channel, _pwm: True,
            failsafe_decoders={
                "HEARTBEAT": lambda message: FailsafeEvidence(
                    active=message.system_status == 5,
                    reason="test FC status",
                )
            },
        )
        heartbeat = lambda mode, status=3: FCMessage(
            type=2, autopilot=3, base_mode=0, custom_mode=4,
            system_status=status, decoded_mode=mode,
        )
        vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", heartbeat("GUIDED"))
        vehicle.listeners["EXTENDED_SYS_STATE"](
            vehicle,
            "EXTENDED_SYS_STATE",
            FCMessage(landed_state=mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND),
        )
        vehicle.listeners["RC_CHANNELS"](
            vehicle, "RC_CHANNELS", FCMessage(chan7_raw=1500)
        )
        assert state.acquire_initial_companion_authority(now=clock.monotonic())

        vehicle.listeners["RC_CHANNELS"](
            vehicle, "RC_CHANNELS", FCMessage(chan7_raw=1900)
        )
        vehicle.listeners["HEARTBEAT"](vehicle, "HEARTBEAT", heartbeat("LOITER"))
        assert state.snapshot().authority.name == "PILOT"
        with pytest.raises(AuthorityLost):
            controller.check_permission()

        state_for_failsafe = FlightState(
            source_system=1, source_component=1, freshness_bounds=BOUNDS,
            rc_channel=7, rc_mode_mapping=flight_profile().rc_mode_bands,
            clock=clock.monotonic,
        )
        vehicle2 = InertVehicle(SimpleNamespace(events=[], state=None, clock=clock))
        monkeypatch.setattr(
            drone_control_module, "connect", lambda *_args, **_kwargs: vehicle2
        )
        controller2 = DroneControl(
            "inert:test",
            source_identity=SourceIdentity(1, 191),
            flight_controller_target=SourceIdentity(1, 1),
            wire_protocol="2.0",
            wait_ready=False,
            flight_state=state_for_failsafe,
            permission_guard=lambda: None,
            heartbeat_mode_decoder=lambda message: message.decoded_mode,
            rc_health_decoder=lambda _message, _channel, _pwm: True,
            failsafe_decoders={
                "HEARTBEAT": lambda message: FailsafeEvidence(
                    active=message.system_status == 5,
                    reason="test FC status",
                )
            },
        )
        vehicle2.listeners["HEARTBEAT"](
            vehicle2, "HEARTBEAT", heartbeat(None, status=3)
        )
        snapshot = state_for_failsafe.snapshot()
        assert snapshot.authority.name == "UNKNOWN"
        assert snapshot.commands_suspended is True
        with pytest.raises(AuthorityLost):
            controller2.check_permission()

        vehicle2.listeners["HEARTBEAT"](
            vehicle2, "HEARTBEAT", heartbeat("GUIDED", status=5)
        )
        assert state_for_failsafe.snapshot().authority.name == "FC_FAILSAFE"
        with pytest.raises(AuthorityLost):
            controller2.check_permission()


@pytest.mark.parametrize(
    ("scenario", "expected_authority", "expected_recovery"),
    [
        ("pilot", "PILOT", "PILOT"),
        ("failsafe", "FC_FAILSAFE", "FC_FAILSAFE"),
        ("unknown", "UNKNOWN", "UNCONFIRMED"),
    ],
)
def test_controller_callback_revocation_stops_real_owner_and_recovery_outputs(
    monkeypatch, scenario, expected_authority, expected_recovery
):
    """Break caught: callback revocation must suppress mission and recovery output."""
    harness = FlightPathHarness(real_recovery=True)
    with timebase.configured(harness.clock):
        for command in (FM1, FM2):
            harness.transport.deliver(Packet(command))
            assert harness.owner.process_next(timeout_s=0) == "SUCCEEDED"

        callback_vehicle = InertVehicle(harness)
        monkeypatch.setattr(
            drone_control_module,
            "connect",
            lambda *_args, **_kwargs: callback_vehicle,
        )
        observer = DroneControl(
            "inert:test",
            source_identity=SourceIdentity(1, 191),
            flight_controller_target=SourceIdentity(1, 1),
            wire_protocol="2.0",
            wait_ready=False,
            flight_state=harness.state,
            permission_guard=harness.supervisor.check_permission,
            heartbeat_mode_decoder=lambda message: message.decoded_mode,
            rc_health_decoder=lambda _message, _channel, _pwm: True,
            failsafe_decoders={
                "HEARTBEAT": lambda message: FailsafeEvidence(
                    active=message.system_status == 5,
                    reason="test FC status",
                )
            },
        )
        heartbeat = lambda mode, status=3: FCMessage(
            type=2, autopilot=3, base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
            custom_mode=4, system_status=status, decoded_mode=mode,
        )
        for _ in range(harness.sequence + 5):
            callback_vehicle.listeners["HEARTBEAT"](
                callback_vehicle, "HEARTBEAT", heartbeat("GUIDED")
            )
        if scenario == "pilot":
            callback_vehicle.listeners["RC_CHANNELS"](
                callback_vehicle, "RC_CHANNELS", FCMessage(chan7_raw=1900)
            )
            callback_vehicle.listeners["HEARTBEAT"](
                callback_vehicle, "HEARTBEAT", heartbeat("LOITER")
            )
        elif scenario == "failsafe":
            callback_vehicle.listeners["HEARTBEAT"](
                callback_vehicle, "HEARTBEAT", heartbeat("GUIDED", status=5)
            )
        else:
            callback_vehicle.listeners["HEARTBEAT"](
                callback_vehicle, "HEARTBEAT", heartbeat(None)
            )
        before_abort = tuple(harness.events)
        assert harness.state.snapshot().authority.name == expected_authority
        assert harness.owner.process_next(timeout_s=0) == "ABORTED"

    assert observer.flight_state is harness.state
    assert tuple(harness.events) == before_abort
    assert harness.recoveries == [expected_recovery]


def test_abort_latched_during_fm1_stops_outputs_and_duplicates_cannot_restart():
    """Break caught: a mid-phase abort or duplicate must never reach arming."""
    harness = FlightPathHarness()
    harness.vehicle.after_mode_output = lambda: harness.listener.request_abort(
        "test operator abort"
    )

    with timebase.configured(harness.clock):
        harness.transport.deliver(Packet())
        assert harness.owner.process_next(timeout_s=0) == "ABORTED"
        harness.transport.deliver(Packet())

    assert harness.events == [("mode", "GUIDED")]
    assert harness.recoveries == ["recovery"]
    assert harness.commands.empty()
    assert [ack.result for ack in harness.transport.acks] == [
        ACK_IN_PROGRESS,
        ACK_CANCELLED,
        ACK_CANCELLED,
    ]


def test_abort_while_airborne_idle_after_fm2_runs_one_recovery_only():
    """Break caught: idle abort must not restart a phase or repeat recovery."""
    harness = FlightPathHarness(real_recovery=True)

    with timebase.configured(harness.clock):
        for command in (FM1, FM2):
            harness.transport.deliver(Packet(command))
            assert harness.owner.process_next(timeout_s=0) == "SUCCEEDED"
        before_abort = tuple(harness.events)
        harness.listener.request_abort("test airborne idle abort")
        assert harness.owner.process_next(timeout_s=0) == "ABORTED"
        after_recovery = tuple(harness.events)
        assert harness.owner.process_next(timeout_s=0) == "ABORTED"

    assert harness.events[len(before_abort):] == [
        ("groundspeed", 10.0),
        ("waypoint", 410000000, -810000000, 260.0),
        ("mode", "LAND"),
        ("armed", False),
    ]
    assert tuple(harness.events) == after_recovery
    assert harness.recoveries == ["HOME_LANDED"]
    assert harness.supervisor.terminal_result == "ABORTED"
    assert harness.supervisor.recovery_outcome == "HOME_LANDED"


def test_attempt_deadline_includes_airborne_fm2_idle_time():
    """Break caught: the 600-second attempt budget must not reset after FM2."""
    harness = FlightPathHarness()

    with timebase.configured(harness.clock):
        fm1_started_at = harness.clock.monotonic()
        harness.transport.deliver(Packet(FM1))
        assert harness.owner.process_next(timeout_s=0) == "SUCCEEDED"
        fm1_deadline = harness.owner._deadline
        assert fm1_deadline == pytest.approx(fm1_started_at + 600.0)

        harness.transport.deliver(Packet(FM2))
        assert harness.owner.process_next(timeout_s=0) == "SUCCEEDED"
        assert harness.owner._deadline == fm1_deadline

        harness.clock.value = fm1_deadline + 0.001
        assert harness.owner.process_next(timeout_s=0) == "ABORTED"

    assert harness.supervisor.abort_reason == "attempt deadline expired"
    assert harness.recoveries == ["recovery"]


def test_phase_deadline_unwinds_before_real_original_home_recovery():
    """Break caught: mission deadline context must not cancel later recovery."""
    harness = FlightPathHarness(real_recovery=True)

    def expire_during_waypoint():
        harness.clock.value = harness.owner._deadline + 0.001
        harness.refresh_flight_observations()

    harness.after_waypoint_output = expire_during_waypoint
    with timebase.configured(harness.clock):
        harness.transport.deliver(Packet(FM1))
        assert harness.owner.process_next(timeout_s=0) == "ABORTED"

    assert harness.supervisor.terminal_result == "ABORTED"
    assert harness.supervisor.recovery_outcome == "HOME_LANDED"
    assert harness.recoveries == ["HOME_LANDED"]
    mission_waypoint = harness.events.index(
        ("waypoint", 410001000, -810001000, 260.0)
    )
    recovery_waypoint = harness.events.index(
        ("waypoint", 410000000, -810000000, 260.0)
    )
    assert mission_waypoint < recovery_waypoint
    assert harness.events[recovery_waypoint + 1] == ("mode", "LAND")


def test_outer_clock_error_is_preserved_when_recovery_also_fails():
    """Break caught: a clock cancellation must not trigger a later fallback path."""
    harness = FlightPathHarness()
    recovery_attempts = []

    def failed_recovery():
        recovery_attempts.append("attempt")
        raise RuntimeError("test recovery failure")

    harness.owner._recovery = failed_recovery
    with timebase.configured(harness.clock):
        harness.transport.deliver(Packet(FM1))
        assert harness.owner.process_next(timeout_s=0) == "SUCCEEDED"
        cancellation = timebase.ClockError("test outer clock loss")
        harness.clock.error = cancellation
        with pytest.raises(timebase.ClockError) as caught:
            harness.owner.process_next(timeout_s=0)

    assert caught.value is cancellation
    assert recovery_attempts == ["attempt"]


def test_failed_real_recovery_stays_unconfirmed_and_does_not_change_mission_failure():
    """Break caught: failed recovery must not overwrite the mission result."""
    harness = FlightPathHarness(confirm_touchdown=False, real_recovery=True)

    with timebase.configured(harness.clock):
        harness.transport.deliver(Packet())
        assert harness.owner.process_next(timeout_s=0) == "FAILED"

    assert harness.supervisor.terminal_result == "FAILED"
    assert harness.supervisor.recovery_outcome == "UNCONFIRMED"
    assert harness.recoveries == ["UNCONFIRMED"]


def build_factory_runtime(
    tmp_path, *, wm_count=6, staged=False, phase_observer=None
):
    from test_listener import prepared_startup_files
    from test_listener_runtime import runtime_configuration

    attempt_dir = tmp_path / "attempt"
    attempt_dir.mkdir()
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    files, prepared = prepared_startup_files(attempt_dir)
    artifacts = load_listener_artifacts(files)
    config = runtime_configuration(runtime_dir)
    clock = AdvancingClock()
    records = {
        "L": L,
        "TARGET": DROP,
        "WA": GPSCoord(PICKUP.lat, PICKUP.long, -6.0),
    }
    for index in range(1, wm_count + 1):
        records[f"WM{index}"] = GPSCoord(
            WM1.lat + (index - 1) * 0.0001,
            WM1.long - (index - 1) * 0.0001,
            4_000.0,
        )
    config.waypoint_path.write_text(json.dumps({
        name: {
            "coords": {"lat": coord.lat, "long": coord.long, "alt": coord.alt},
            "loaded_at": clock.epoch(),
        }
        for name, coord in records.items()
    }))
    config = replace(
        config,
        connection=replace(config.connection, wire_protocol="2.0"),
        components=(
            InjectedComponentConfig(
                "drone-sim-ros-confirmed-v1", "test staged simulator binding"
            )
            if staged
            else config.components
        ),
        precision_policy=replace(
            config.precision_policy,
            target_hover_height_m=1.51,
        ),
    )
    rig = FactoryRig(clock)
    diagnostics = []
    with timebase.configured(clock):
        runtime = build_live_listener(
            artifacts,
            config,
            factories=rig.factories(staged=staged),
            diagnostics=diagnostics.append,
            phase_observer=phase_observer,
        )
    return runtime, rig, prepared, config, diagnostics


def test_first_guided_delivery_allows_post_gate_camera_readiness(tmp_path):
    runtime, rig, prepared, config, diagnostics = build_factory_runtime(
        tmp_path, wm_count=1, staged=True
    )
    assert not any(event[0] == "camera-prepared" for event in rig.events)

    def open_public_simulation():
        rig.accepted_simulation_ns = 50_000_000
        rig.events.append(("guided-delivered", rig.accepted_simulation_ns))
        rig.clock.value += 0.05

    runtime.controller.guided_output_delivery_callback = open_public_simulation
    runtime.controller._guided_output_delivery_reported = False
    with timebase.configured(rig.clock):
        rig.transport.deliver(Packet(FM1, attempt_id=prepared.attempt_id))
        assert runtime.owner.process_next(timeout_s=0) == "SUCCEEDED", diagnostics

    guided_index = rig.events.index(("guided-delivered", 50_000_000))
    telemetry_index = rig.events.index(
        ("telemetry-verified-after-guided", 50_000_000)
    )
    camera_index = rig.events.index(
        ("camera-prepared", config.precision_policy.frame_timeout_s)
    )
    assert guided_index < telemetry_index < camera_index


def test_public_live_factory_runs_complete_callback_fm1_fm2_fm3_trace(tmp_path):
    """Break caught: composition must not bypass its listener, owner, or missions."""
    runtime, rig, prepared, config, diagnostics = build_factory_runtime(tmp_path)

    with timebase.configured(rig.clock):
        for command in (FM1, FM2, FM3):
            if command == FM2:
                rig.inject_release_invalidation = True
            rig.transport.deliver(Packet(command, attempt_id=prepared.attempt_id))
            result = runtime.owner.process_next(timeout_s=0)
            assert result == "SUCCEEDED", (rig.transport.acks, diagnostics)

    assert runtime.supervisor.terminal_result == "SUCCEEDED"
    assert runtime.tracker.waypoint_path == config.waypoint_path
    assert runtime.controller is rig.controller
    assert runtime.flight_state is rig.state
    assert rig.outbound.qsize() > 0
    assert rig.release_invalidation_injected is True
    assert len(rig.release_partitions) == 3
    fm2_release_evidence = rig.release_partitions[0]
    generations = [
        evidence.attitude_invalidation_generation
        for evidence in fm2_release_evidence
    ]
    assert 0 in generations and 1 in generations
    post_reset = []
    for evidence in fm2_release_evidence:
        if evidence.attitude_invalidation_generation != 1:
            continue
        sequences = runtime.controller._evidence_sequences(evidence)
        if post_reset and not all(
            current > previous
            for current, previous in zip(
                sequences,
                runtime.controller._evidence_sequences(post_reset[-1]),
            )
        ):
            continue
        post_reset.append(evidence)
    assert len(post_reset) >= 5
    assert all(
        later.observed_at >= earlier.observed_at
        for earlier, later in zip(post_reset, post_reset[1:])
    )
    assert (
        post_reset[-1].observed_at - post_reset[0].observed_at
        >= config.release_stability.hold_seconds
    )
    assert all(
        all(current > previous for current, previous in zip(
            runtime.controller._evidence_sequences(later),
            runtime.controller._evidence_sequences(earlier),
        ))
        for earlier, later in zip(post_reset, post_reset[1:])
    )
    final_release_evidence = fm2_release_evidence[-1]
    assert final_release_evidence.observed_at >= post_reset[-1].observed_at
    assert runtime.controller._release_evidence_is_stable(
        final_release_evidence,
        config.release_stability,
        config.desired_drop_height_m,
    )
    release_output_at, release_boundary, recorded_evidence, source_times = (
        rig.release_boundaries[0]
    )
    assert recorded_evidence is final_release_evidence
    assert release_boundary.location.observation.sequence == (
        final_release_evidence.location_sequence
    )
    stream_names = ("location", "velocity", "attitude")
    for name, sampled_at in zip(stream_names, source_times[:3]):
        age = release_output_at - sampled_at
        assert 0.0 <= age <= runtime.flight_state.freshness_bounds[name]
    range_age = release_output_at - source_times[3]
    assert 0.0 <= range_age <= config.clearance_calibration.max_age_seconds
    assert max(source_times) - min(source_times) <= (
        config.release_stability.max_observation_skew_seconds
    )
    assert abs(source_times[2] - source_times[3]) <= (
        config.clearance_calibration.max_skew_seconds
    )
    assert ("camera-prepared", config.precision_policy.frame_timeout_s) in rig.events
    attachment = rig.events.index(("payload-attached", 3))
    assert rig.events[attachment - 1] == ("armed", False)
    assert rig.events.index(("armed", True), attachment) > attachment
    takeoff_altitudes = [event[1] for event in rig.events if event[0] == "takeoff"]
    assert takeoff_altitudes[:3] == [10.0, 9.0, 8.0]
    assert len(takeoff_altitudes) == 4
    expected_rearm_homes = takeoff_altitudes
    assert [event[0] for event in rig.causal_home_trace] == [
        event
        for _ in expected_rearm_homes
        for event in (
            "premature-home",
            "armed-confirmed",
            "home-request",
            "home-ack",
            "home-response",
            "takeoff",
        )
    ]
    for index, expected_takeoff in enumerate(expected_rearm_homes):
        premature, armed, request, ack, response, takeoff = (
            rig.causal_home_trace[index * 6:(index + 1) * 6]
        )
        assert premature[1] == armed[1] == request[1] == response[1]
        assert premature[2] < armed[2] < response[2]
        assert request[2] > armed[3]
        assert ack == ("home-ack", mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE)
        assert takeoff[1] == expected_takeoff
        assert takeoff[2] == response[2]
    attachments = [event[1] for event in rig.events if event[0] == "payload-attached"]
    assert attachments == [3, 4]
    pickup_sites = [
        (410004000, -810004000),
        (410003000, -810003000),
    ]
    previous_delivery = -1
    for marker_id, pickup_site in zip(range(3, 5), pickup_sites):
        attachment_index = rig.events.index(("payload-attached", marker_id))
        pickup_index = max(
            index
            for index, event in enumerate(rig.events[:attachment_index])
            if event[0:3] == ("waypoint", *pickup_site) and event[3] == 260.0
        )
        delivery_index = next(
            index
            for index, event in enumerate(rig.events[attachment_index + 1:], attachment_index + 1)
            if event[0:3] == ("waypoint", 410002000, -810002000)
            and event[3] == 260.0
        )
        assert previous_delivery < pickup_index < attachment_index < delivery_index
        previous_delivery = delivery_index
    assert rig.events[-3:] == [
        ("waypoint", 410000000, -810000000, 260.0),
        ("mode", "LAND"),
        ("armed", False),
    ]
    assert [ack.result for ack in rig.transport.acks] == [
        ACK_IN_PROGRESS,
        ACK_ACCEPTED,
        ACK_IN_PROGRESS,
        ACK_ACCEPTED,
        ACK_IN_PROGRESS,
        ACK_ACCEPTED,
    ]


def test_full_mode_emits_eleven_events_and_uses_fm3_home_as_recovery(tmp_path):
    """Missing an FM3 or HOME boundary would leave official evidence incomplete."""
    phases = []
    runtime, rig, prepared, _config, diagnostics = build_factory_runtime(
        tmp_path,
        wm_count=1,
        phase_observer=lambda phase, state: phases.append(
            (phase, state, rig.clock.monotonic())
        ),
    )

    with timebase.configured(rig.clock):
        rig.transport.deliver(Packet(FM1, attempt_id=prepared.attempt_id))
        assert runtime.owner.process_next(timeout_s=0) == "SUCCEEDED", diagnostics

    releases = []
    delegate = runtime.dropper._delegate
    assert delegate is not None
    original_drop = delegate.drop

    def record_release():
        original_drop()
        releases.append(rig.clock.monotonic())

    delegate.drop = record_release
    with timebase.configured(rig.clock):
        rig.transport.deliver(Packet(FM2, attempt_id=prepared.attempt_id))
        assert runtime.owner.process_next(timeout_s=0) == "SUCCEEDED", diagnostics
    assert runtime.supervisor.terminal_result is None
    assert runtime.supervisor.recovery_outcome is None
    assert not any(phase == "HOME" for phase, _state, _at in phases)

    with timebase.configured(rig.clock):
        rig.transport.deliver(Packet(FM3, attempt_id=prepared.attempt_id))
        assert runtime.owner.process_next(timeout_s=0) == "SUCCEEDED", diagnostics

    assert [(phase, state) for phase, state, _at in phases] == [
        ("FM1", "STARTED"),
        ("FM1", "COMPLETE"),
        ("FM2", "STARTED"),
        ("FM2", "COMPLETE"),
        ("FM3_3", "STARTED"),
        ("FM3_3", "COMPLETE"),
        ("FM3_4", "STARTED"),
        ("FM3_4", "COMPLETE"),
        ("HOME", "STARTED"),
        ("HOME", "DISARMED"),
        ("HOME", "COMPLETE"),
    ]
    assert runtime.supervisor.terminal_result == "SUCCEEDED"
    assert runtime.supervisor.recovery_outcome == "HOME_LANDED"
    completed_at = {
        phase: observed_at
        for phase, state, observed_at in phases
        if state == "COMPLETE" and phase in {"FM2", "FM3_3", "FM3_4"}
    }
    assert len(releases) == 3
    assert all(
        completed_at[phase] - released_at
        >= PHYSICAL_EVIDENCE_INTERVAL_SECONDS - 1e-9
        for phase, released_at in zip(("FM2", "FM3_3", "FM3_4"), releases)
    )
    assert [event for event in rig.events if event[0] == "payload-attached"] == [
        ("payload-attached", 3),
        ("payload-attached", 4),
    ]
    assert [
        event
        for event in rig.events
        if event[:3] == ("waypoint", 410000000, -810000000)
    ] == [("waypoint", 410000000, -810000000, 260.0)]


@pytest.mark.parametrize(
    "failure",
    ("clock", "deadline", "permission", "evidence_wait"),
)
def test_full_fm2_evidence_wait_failure_withholds_complete_and_success(
    tmp_path, failure
):
    """A failed post-release interval cannot certify full-mode FM2."""
    phases = []
    runtime, rig, prepared, _config, diagnostics = build_factory_runtime(
        tmp_path,
        wm_count=1,
        phase_observer=lambda phase, state: phases.append((phase, state)),
    )
    with timebase.configured(rig.clock):
        rig.transport.deliver(Packet(FM1, attempt_id=prepared.attempt_id))
        assert runtime.owner.process_next(timeout_s=0) == "SUCCEEDED", diagnostics

    delegate = runtime.dropper._delegate
    assert delegate is not None
    original_drop = delegate.drop

    def fail_after_release():
        original_drop()
        if failure == "clock":
            runtime.owner._clock = lambda: float("nan")
        elif failure == "deadline":
            runtime.owner._deadline = (
                rig.clock.monotonic() + PHYSICAL_EVIDENCE_INTERVAL_SECONDS / 2
            )
        elif failure == "permission":
            runtime.supervisor._permission_check = lambda: (_ for _ in ()).throw(
                AuthorityLost("test permission lost")
            )
        else:
            rig.clock.sleep = lambda _seconds: (_ for _ in ()).throw(
                RuntimeError("test evidence sleep failed")
            )

    delegate.drop = fail_after_release
    with timebase.configured(rig.clock):
        rig.transport.deliver(Packet(FM2, attempt_id=prepared.attempt_id))
        try:
            result = runtime.owner.process_next(timeout_s=0)
        except (AuthorityLost, timebase.ClockError):
            result = "FAILED"

    assert result != "SUCCEEDED"
    assert ("FM2", "COMPLETE") not in phases
    assert runtime.supervisor.terminal_result != "SUCCEEDED"


@pytest.mark.parametrize(
    ("failure", "last_event"),
    (
        ("payload3", ("FM3_3", "STARTED")),
        ("payload4", ("FM3_4", "STARTED")),
        ("home_navigation", ("HOME", "STARTED")),
        ("home_landing", ("HOME", "STARTED")),
        ("home_disarm", ("HOME", "STARTED")),
        ("home_interval", ("HOME", "DISARMED")),
        ("observer", ("FM3_3", "STARTED")),
    ),
)
def test_full_mode_failure_withholds_later_completion_and_success(
    tmp_path, failure, last_event
):
    """Each FM3 and HOME failure boundary must end the attempt fail closed."""
    phases = []

    def observe(phase, state):
        phases.append((phase, state))
        if failure == "observer" and (phase, state) == ("FM3_3", "STARTED"):
            raise RuntimeError("mission event publication failed")

    runtime, rig, prepared, _config, diagnostics = build_factory_runtime(
        tmp_path, wm_count=1, phase_observer=observe
    )
    with timebase.configured(rig.clock):
        for command in (FM1, FM2):
            rig.transport.deliver(Packet(command, attempt_id=prepared.attempt_id))
            assert runtime.owner.process_next(timeout_s=0) == "SUCCEEDED", diagnostics

    delegate = runtime.dropper._delegate
    assert delegate is not None
    original_drop = delegate.drop
    fm3_releases = 0

    def maybe_fail_drop():
        nonlocal fm3_releases
        fm3_releases += 1
        if failure == "payload3" and fm3_releases == 1:
            raise RuntimeError("payload 3 failed")
        if failure == "payload4" and fm3_releases == 2:
            raise RuntimeError("payload 4 failed")
        return original_drop()

    delegate.drop = maybe_fail_drop
    original_goto = runtime.controller.goto_waypoint
    original_land = runtime.controller.simple_land
    original_disarm = runtime.controller.disarm

    def at_home() -> bool:
        location = runtime.flight_state.snapshot().location.observation.value
        return location[:2] == (410000000, -810000000)

    def maybe_fail_goto(target, *args, **kwargs):
        if failure == "home_navigation" and (target.lat, target.long) == (H.lat, H.lon):
            return 1
        return original_goto(target, *args, **kwargs)

    def maybe_fail_land(*args, **kwargs):
        if failure == "home_landing" and at_home():
            return 1
        return original_land(*args, **kwargs)

    def maybe_fail_disarm(*args, **kwargs):
        if failure == "home_disarm" and at_home():
            return 1
        result = original_disarm(*args, **kwargs)
        if failure == "home_interval" and at_home():
            rig.clock.tick_callbacks.clear()
        return result

    runtime.controller.goto_waypoint = maybe_fail_goto
    runtime.controller.simple_land = maybe_fail_land
    runtime.controller.disarm = maybe_fail_disarm

    with timebase.configured(rig.clock):
        rig.transport.deliver(Packet(FM3, attempt_id=prepared.attempt_id))
        if failure == "observer":
            with pytest.raises(RuntimeError, match="mission event publication failed"):
                runtime.owner.process_next(timeout_s=0)
        else:
            assert runtime.owner.process_next(timeout_s=0) == "FAILED"

    assert phases[-1] == last_event
    assert runtime.supervisor.terminal_result == "FAILED"
    assert ("HOME", "COMPLETE") not in phases
    assert not any(
        phase == last_event[0] and state == "COMPLETE"
        for phase, state in phases[phases.index(last_event) + 1 :]
    )


def test_public_live_factory_rejects_wa_only_full_phase_route(tmp_path):
    """Break caught: a full runtime must freeze both competition FM3 cycles."""
    runtime, rig, prepared, _config, diagnostics = build_factory_runtime(
        tmp_path, wm_count=0
    )

    with timebase.configured(rig.clock):
        rig.transport.deliver(Packet(FM1, attempt_id=prepared.attempt_id))

    assert runtime.owner.process_next(timeout_s=0) is None
    assert rig.transport.acks[-1].result == ACK_DENIED
    assert diagnostics == []


def test_live_runtime_stale_vitals_become_unknown_and_deny_next_phase(tmp_path):
    """Break caught: stale live observations must not retain companion authority."""
    runtime, rig, prepared, _config, diagnostics = build_factory_runtime(tmp_path)

    with timebase.configured(rig.clock):
        rig.transport.deliver(Packet(FM1, attempt_id=prepared.attempt_id))
        assert runtime.owner.process_next(timeout_s=0) == "SUCCEEDED"
        prior_events = tuple(rig.events)
        rig.clock.value += 31.0
        rig.transport.deliver(Packet(FM2, attempt_id=prepared.attempt_id))
        authority = runtime.flight_state.snapshot().authority.name

    assert tuple(rig.events) == prior_events
    assert authority == "UNKNOWN"
    assert rig.transport.acks[-1].command == FM2
    assert rig.transport.acks[-1].result == ACK_DENIED
    assert diagnostics == []
