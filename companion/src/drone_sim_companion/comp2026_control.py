"""Testbench transport ownership around the upstream DroneControl object."""
from __future__ import annotations

import threading
import time
from pathlib import Path

from dronekit import connect, VehicleMode
from pymavlink import mavutil
from drone.control.drone_control import DroneControl

from .comp2026_policy import MissionHome
from .dronekit_sim import SimulationVehicle
from .flight_authority import DroneKitAuthorityWriter, FlightAuthorityGate, FlightAuthorityLost
from .mission import Telemetry
from .qgc_transport import configure_dronekit_mavlink2


class _CompetitionWriter(DroneKitAuthorityWriter):
    """Retain the existing packet guard and add mission/recovery ownership."""
    def __init__(self, delegate, owner, mavlink):
        super().__init__(delegate, owner.authority, mavlink)
        self.owner = owner

    def write(self, packet):
        message = self._decode(packet)
        passive = message.get_type() in self._OBSERVATION_MESSAGES or (
            message.get_type() == "COMMAND_LONG"
            and int(message.command) in self._OBSERVATION_COMMANDS
        )
        if passive:
            return super().write(packet)
        return self.owner.actuate(lambda: super(_CompetitionWriter, self).write(packet))


class SimulationCompetitionControl:
    """Own admission, native observations and output scope, never flight logic."""
    _OBSERVED = ("HEARTBEAT", "SYS_STATUS", "EXTENDED_SYS_STATE", "GLOBAL_POSITION_INT", "AUTOPILOT_VERSION")

    def __init__(self, *, policy, run_id: str, run_directory: Path, endpoint: str,
                 heartbeat_timeout: float, clock, guided_output_delivery_callback, recovery_clock=None,
                 controller_factory=DroneControl, connection_factory=connect):
        self.policy = policy
        self.clock = clock
        self.recovery_clock = recovery_clock or clock
        self.run_id = run_id
        self.run_directory = Path(run_directory)
        self.authority = FlightAuthorityGate()
        self._lock = threading.RLock()
        self._facts = {}
        self._last_output_ns = -1
        self._native_previous = None
        self._native_advancing = False
        self._native_clock_failed = False
        self._mission_thread = None
        self._recovery_thread = None
        self._cancelled = False
        self._finalizing = False
        self._initial_delivered = False
        self._shutdown_transaction = lambda operation: operation()
        self._delivery = guided_output_delivery_callback
        self._ground_telemetry_pending_reasons = ()
        self.mission_home = None
        configure_dronekit_mavlink2()
        self.vehicle = connection_factory(
            endpoint, wait_ready=False, heartbeat_timeout=heartbeat_timeout,
            source_system=1, source_component=191, vehicle_class=SimulationVehicle,
        )
        try:
            encoder = self.vehicle._master.mav
            encoder.file = _CompetitionWriter(encoder.file, self, mavutil.mavlink)
            self.vehicle.install_fc_message_observer(self._observe)
            self.controller = controller_factory(endpoint, vehicle=self.vehicle, command_listener=False)
        except BaseException:
            self.vehicle.close()
            raise

    def install_output_guard(self, transaction):
        self._shutdown_transaction = transaction

    def _observe(self, _vehicle, name, message):
        if name not in self._OBSERVED or message.get_srcSystem() != 1 or message.get_srcComponent() != 1:
            return
        now = time.monotonic_ns()
        with self._lock:
            self._facts[name] = (now, message)
            if name == "GLOBAL_POSITION_INT":
                native_tick = int(message.time_boot_ms)
                if self._native_previous is not None:
                    if native_tick < self._native_previous:
                        self._native_clock_failed = True
                        if self.authority.admitted:
                            self.authority.revoke("flight-controller native clock regressed")
                    elif native_tick > self._native_previous:
                        self._native_advancing = True
                self._native_previous = native_tick
            elif name == "HEARTBEAT":
                if self.authority.admitted and (message.type != 2 or message.autopilot != 3):
                    self.authority.revoke("flight-controller heartbeat identity changed")
                mode = {0: "STABILIZE", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}.get(int(message.custom_mode), str(message.custom_mode))
                self.authority.observe(Telemetry(now, heartbeat=True, mode=mode,
                    armed=bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)))
                if self.authority.admitted and getattr(message, "system_status", 0) >= 5:
                    self.authority.revoke("flight controller reported an active failsafe")

    def _fresh(self, name):
        fact = self._facts.get(name)
        return fact[1] if fact is not None and 0 <= time.monotonic_ns() - fact[0] <= 2_000_000_000 else None

    def prepare(self):
        mav = self.vehicle._master.mav
        mav.param_request_list_send(1, 1)
        mav.command_long_send(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0)
        for message_id, interval in self.policy.telemetry_intervals_us:
            mav.command_long_send(1, 1, 511, 0, message_id, interval, 0, 0, 0, 0, 0)

    def ready_for_initial_command(self):
        with self._lock:
            reasons = []
            heartbeat = self._fresh("HEARTBEAT")
            status = self._fresh("SYS_STATUS")
            landed = self._fresh("EXTENDED_SYS_STATE")
            position = self._fresh("GLOBAL_POSITION_INT")
            version_fact = self._facts.get("AUTOPILOT_VERSION")
            version = version_fact[1] if version_fact else None
            if heartbeat is None or bool(heartbeat.base_mode & 128) or getattr(heartbeat, "system_status", 0) >= 5:
                reasons.append("fresh healthy disarmed heartbeat absent")
            if heartbeat is not None and (heartbeat.type != 2 or heartbeat.autopilot != 3):
                reasons.append("ArduCopter heartbeat identity not verified")
            prearm = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
            if status is None or not (status.onboard_control_sensors_enabled & prearm and status.onboard_control_sensors_health & prearm):
                reasons.append("fresh healthy prearm status absent")
            if landed is None or landed.landed_state != 1:
                reasons.append("fresh grounded telemetry absent")
            if position is None or not self._native_advancing or self._native_clock_failed:
                reasons.append("advancing native position time absent")
            if version is None or version.flight_sw_version != self.policy.flight_sw_version or bytes(version.flight_custom_version) != self.policy.flight_custom_version:
                reasons.append("pinned flight-controller firmware not verified")
            if position is not None:
                home = MissionHome(position.lat / 1e7, position.lon / 1e7, (position.alt - position.relative_alt) / 1000)
                try:
                    self.policy.mission_home_check(home)
                except ValueError as error:
                    reasons.append(str(error))
            self._ground_telemetry_pending_reasons = tuple(reasons)
            return not reasons

    @property
    def ground_telemetry_pending_reasons(self):
        return self._ground_telemetry_pending_reasons

    def admit(self, admission_sha256):
        self.authority.admit(admission_sha256)

    def begin_attempt(self):
        with self._lock:
            if self._mission_thread is not None or self._cancelled:
                raise FlightAuthorityLost("attempt is already consumed or cancelled")
            if not self.ready_for_initial_command():
                raise FlightAuthorityLost("ground readiness changed before mission start")
            if not self.authority.admitted:
                raise FlightAuthorityLost("attempt has no testbench admission")
            self.run_directory.mkdir(parents=True, exist_ok=True)
            with (self.run_directory / ".comp2026-attempt-consumed.json").open("x") as output:
                output.write('{"consumed":true}\n')
            position = self._fresh("GLOBAL_POSITION_INT")
            self.mission_home = MissionHome(position.lat / 1e7, position.lon / 1e7, (position.alt - position.relative_alt) / 1000)
            self._mission_thread = threading.get_ident()

    def _require_live_heartbeat(self):
        if self._fresh("HEARTBEAT") is None:
            self.authority.revoke("flight-controller heartbeat is stale")
            raise FlightAuthorityLost("flight-controller heartbeat is stale")

    def check_active(self):
        with self._lock:
            if self._finalizing:
                raise FlightAuthorityLost("competition runtime is finalizing")
            if self._cancelled or threading.get_ident() != self._mission_thread:
                raise FlightAuthorityLost("upstream mission output is cancelled or has no owner")
            self._require_live_heartbeat()
            with self.authority.output_transaction():
                pass

    def actuate(self, operation):
        with self._lock:
            if self._finalizing:
                raise FlightAuthorityLost("competition runtime is finalizing")
            recovery = threading.get_ident() == self._recovery_thread
            if not recovery:
                self.check_active()
            else:
                self._require_live_heartbeat()
            return self._shutdown_transaction(lambda: self._authorized(operation))

    def _authorized(self, operation):
        with self.authority.output_transaction():
            result = operation()
            self._last_output_ns = time.monotonic_ns()
            return result

    def deliver_initial_guided(self):
        def enqueue(timestamp_ns):
            if timestamp_ns > 50_000_000:
                raise RuntimeError("initial GUIDED command missed the 50 ms delivery window")
            with self._lock:
                self.check_active()
                if self._initial_delivered:
                    raise RuntimeError("initial GUIDED was already delivered")
                # DroneKit's setter always enqueues. Upstream set_guided_mode can
                # skip a packet when cached GUIDED, which cannot release the epoch.
                def deliver():
                    self.vehicle.mode = VehicleMode("GUIDED")
                    self._delivery()
                    self._initial_delivered = True
                self._shutdown_transaction(deliver)
        if not self.clock.run_at_current_timestamp(enqueue):
            raise RuntimeError("initial GUIDED requires the public simulation clock")

    def wait_for_guided(self):
        """Observe the epoch-opening mode before upstream can request STABILIZE."""
        started = self.clock.now()
        while self.clock.now() - started <= 10:
            self.check_active()
            with self._lock:
                heartbeat = self._fresh("HEARTBEAT")
                if heartbeat is not None and heartbeat.custom_mode == 4:
                    return
            self.clock.sleep(0.05)
        raise TimeoutError("initial GUIDED mode was not observed")

    def confirm_landed(self):
        with self._lock:
            heartbeat = self._fresh("HEARTBEAT")
            landed = self._fresh("EXTENDED_SYS_STATE")
            return bool(
                heartbeat is not None and not heartbeat.base_mode & 128
                and landed is not None and landed.landed_state == 1
                and self._facts["HEARTBEAT"][0] > self._last_output_ns
                and self._facts["EXTENDED_SYS_STATE"][0] > self._last_output_ns
            )

    def wait_for_landed(self):
        """Observe touchdown/auto-disarm after upstream returns, without output."""
        started = self.clock.now()
        while self.clock.now() - started <= 15:
            self.check_active()
            if self.confirm_landed():
                return True
            self.clock.sleep(0.05)
        return False

    def finish_attempt(self):
        with self._lock:
            self._mission_thread = None
            self.authority.clear_expected()

    def abort(self, _reason):
        with self._lock:
            self._cancelled = True
            self.authority.clear_expected()

    def recover(self):
        with self._lock:
            if self._finalizing or self.authority.revoked or not self.authority.admitted:
                return "UNCONFIRMED"
            if self.confirm_landed():
                return "LOCAL_LANDED"
            heartbeat = self._fresh("HEARTBEAT")
            if heartbeat is None or not heartbeat.base_mode & 128:
                return "UNCONFIRMED"
            self._recovery_thread = threading.get_ident()
        try:
            self.vehicle.mode = VehicleMode("LAND")
            started = self.recovery_clock.now()
            while self.recovery_clock.now() - started <= 60:
                if self.authority.revoked or self._finalizing:
                    return "UNCONFIRMED"
                if self.confirm_landed():
                    return "LOCAL_LANDED"
                self.recovery_clock.sleep(0.05)
            return "UNCONFIRMED"
        finally:
            with self._lock:
                self._recovery_thread = None
                self.authority.clear_expected()

    def begin_finalizing(self):
        with self._lock:
            self._finalizing = True
            self.authority.begin_finalizing()

    def close(self):
        self.begin_finalizing()
        self.vehicle.remove_fc_message_observer(self._observe)
