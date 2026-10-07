"""Guarded composition for the original Comp2026 automatic simulation."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Callable

from pymavlink import mavutil

from drone import timebase
from drone.control.drone_control import DroneControl
from drone.control.flight_state import FlightState, RCInput
from drone.control.listener import (
    GroundTelemetryPending,
    TelemetryStartupCollector,
    _mission_home_from_snapshot,
    _require_fresh_ground_snapshot,
    _require_fresh_vital_permission,
)
from drone.control.mission_supervisor import (
    CommandEnvelope,
    CommandRejected,
    FM1,
    FM2,
    FM3,
    MAX_ATTEMPT_ID,
    MissionSupervisor,
)


class SimulationCompetitionControl:
    """Compose imported observation, authority, mission, and output guards."""

    def __init__(
        self,
        policy,
        *,
        run_id: str,
        run_directory: Path,
        endpoint: str,
        heartbeat_timeout: float,
        guided_output_delivery_callback: Callable[[], None],
        controller_factory=DroneControl,
        telemetry_factory=TelemetryStartupCollector,
    ) -> None:
        if not isinstance(run_id, str) or not run_id:
            raise ValueError("run_id must be a non-empty string")
        self.policy = policy
        self.run_id = run_id
        self.run_directory = Path(run_directory)
        profile = policy.flight_profile
        self.flight_state = FlightState(
            source_system=profile.flight_controller.system_id,
            source_component=profile.flight_controller.component_id,
            freshness_bounds=profile.freshness_bounds,
            rc_channel=profile.rc_channel,
            rc_mode_mapping=profile.rc_mode_bands,
            clock=timebase.monotonic,
        )
        self.decoders = profile.decoders(
            clock=timebase.monotonic, flight_state=self.flight_state
        )
        digest = hashlib.sha256(run_id.encode("utf-8")).digest()
        self._attempt_id = int.from_bytes(digest[:3], "big") % MAX_ATTEMPT_ID + 1
        self._pending_home = None
        self._ground_telemetry_pending_reasons: tuple[str, ...] = ()
        self._phase_events = (
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
        )
        self._phase_event_index = 0

        def permission_check() -> None:
            self.decoders.check_authority_dependency()
            _require_fresh_vital_permission(self.flight_state)

        def admission_check(envelope: CommandEnvelope) -> None:
            if envelope.command == FM1:
                snapshot = self._ready_snapshot()
                self._pending_home = _mission_home_from_snapshot(snapshot)
                if policy.mission_home_check(self._pending_home) is not None:
                    raise CommandRejected(
                        "mission-home check violated its success contract"
                    )
            else:
                permission_check()

        def consume_attempt(attempt_id: int) -> None:
            self._consume_attempt(attempt_id)
            if self.flight_state.acquire_initial_companion_authority() is not True:
                raise RuntimeError("initial companion authority was not acquired")
            if self._pending_home is None:
                raise RuntimeError("launch snapshot is unavailable")
            self.controller.set_mission_home(self._pending_home)

        self.supervisor = MissionSupervisor(
            self._attempt_id,
            admission_check=admission_check,
            attempt_consumer=consume_attempt,
            permission_check=permission_check,
            recovery_policy=policy.recovery_policy,
            enabled_phases=(FM1, FM2, FM3),
        )
        # A later wire-version upgrade would replace DroneKit's queue writer.
        os.environ["MAVLINK20"] = "1"
        mavutil.set_dialect("ardupilotmega")
        self.controller = controller_factory(
            endpoint,
            source_identity=profile.companion_target,
            flight_controller_target=profile.flight_controller,
            wire_protocol="2.0",
            wait_ready=False,
            heartbeat_timeout=heartbeat_timeout,
            flight_state=self.flight_state,
            permission_guard=self.supervisor.check_permission,
            heartbeat_mode_decoder=self.decoders.heartbeat_mode_decoder,
            rc_health_decoder=self.decoders.rc_health_decoder,
            sys_status_observer=self.decoders.observe_sys_status,
            failsafe_decoders={
                "HEARTBEAT": self.decoders.heartbeat_failsafe_decoder,
            },
            mission_home_check=policy.mission_home_check,
            fc_home_position_tolerance_m=policy.fc_home_position_tolerance_m,
            fc_home_altitude_tolerance_m=policy.fc_home_altitude_tolerance_m,
            clearance_calibration=policy.clearance_calibration,
            release_stability_config=policy.release_stability,
            home_request_timeout_s=policy.startup_timeout_s,
            telemetry_poll_interval_s=policy.telemetry_poll_interval_s,
            guided_output_delivery_callback=guided_output_delivery_callback,
        )
        self.controller.install_output_transactions(
            dependency_transaction=self.decoders.output_transaction,
            supervisor_transaction=self.supervisor.output_transaction,
        )
        self.telemetry_collector = telemetry_factory(
            controller=self.controller,
            flight_profile=profile,
            policy=policy.telemetry_policy,
            autopilot_version=policy.autopilot_version_contract,
            clock=timebase.monotonic,
        )

    def _ready_snapshot(self):
        snapshot = _require_fresh_ground_snapshot(self.flight_state)
        self.decoders.check_rc_health()
        rc_input = snapshot.rc_input.observation.value
        if not isinstance(rc_input, RCInput) or rc_input.slot != "companion":
            raise CommandRejected("initial RC selection must be the companion slot")
        return snapshot

    def ready_for_initial_command(self) -> bool:
        try:
            self._ready_snapshot()
        except GroundTelemetryPending as error:
            self._ground_telemetry_pending_reasons = error.reasons
            return False
        self._ground_telemetry_pending_reasons = ()
        return True

    @property
    def ground_telemetry_pending_reasons(self) -> tuple[str, ...]:
        return self._ground_telemetry_pending_reasons

    def prepare(self) -> None:
        self.telemetry_collector.prepare()

        def verify() -> None:
            self.telemetry_collector.verify_after_guided()
            return None

        self.controller.install_startup_telemetry_verifier(verify, verified=False)

    def _envelope(self, command: int) -> CommandEnvelope:
        profile = self.policy.flight_profile
        return CommandEnvelope(
            source_system=profile.qgc_source.system_id,
            source_component=profile.qgc_source.component_id,
            target_system=profile.companion_target.system_id,
            target_component=profile.companion_target.component_id,
            command=command,
            attempt_id=self._attempt_id,
            params=(float(self._attempt_id), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            received_at=timebase.monotonic(),
        )

    def begin_attempt(self) -> None:
        if self.supervisor.admit(self._envelope(FM1)) is not True:
            raise CommandRejected("FM1 was already admitted")
        self.supervisor.begin(FM1)

    def _consume_attempt(self, attempt_id: int) -> None:
        self.run_directory.mkdir(parents=True, exist_ok=True)
        path = self.run_directory / ".comp2026-attempt-consumed.json"
        payload = (json.dumps({"run_id": self.run_id, "attempt_id": attempt_id}) + "\n").encode()
        try:
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError as error:
            raise RuntimeError("run-scoped attempt token is already consumed") from error
        try:
            os.write(descriptor, payload)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        directory = os.open(self.run_directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory)
        finally:
            os.close(directory)

    def phase_event(self, phase: str, state: str) -> None:
        event = (phase, state)
        if (
            self._phase_event_index >= len(self._phase_events)
            or event != self._phase_events[self._phase_event_index]
        ):
            raise RuntimeError(f"unexpected automatic mission event {phase}/{state}")
        self._phase_event_index += 1
        if (phase, state) == ("FM1", "COMPLETE"):
            self.supervisor.finish(FM1, "SUCCEEDED")
            self.supervisor.admit(self._envelope(FM2))
            self.supervisor.begin(FM2)
        elif (phase, state) == ("FM2", "COMPLETE"):
            self.supervisor.finish(FM2, "SUCCEEDED")
            self.supervisor.admit(self._envelope(FM3))
            self.supervisor.begin(FM3)
        elif (phase, state) == ("HOME", "COMPLETE"):
            self.supervisor.finish(FM3, "SUCCEEDED")

    def abort(self, reason: str) -> None:
        self.supervisor.request_abort(reason)

    def recover(self) -> str:
        home = self.controller.mission_home
        if home is None:
            raise RuntimeError("recovery has no pinned mission home")
        timebase.monotonic()
        return self.supervisor.recover(self.controller, home, 10.0)

    def close(self) -> None:
        self.telemetry_collector.close()


__all__ = ["SimulationCompetitionControl"]
