"""Simulation-only DroneKit behavior needed for long native parameter streams."""

import math
import threading
import time
from collections import deque

from dronekit import Vehicle

from .mavlink_adapter import (
    FLIGHT_CONTROLLER_COMPONENT_ID,
    FLIGHT_CONTROLLER_SYSTEM_ID,
    is_flight_controller_message,
    request_flight_controller_heartbeat,
)
from .qgc_transport import VehicleFactObservation


class SimulationVehicle(Vehicle):
    """Keep DroneKit's real parameter cache without its indexed retry bursts."""

    def __init__(self, handler):
        super().__init__(handler)
        self._qgc_message_callback = None
        self._operator_observations = deque(maxlen=64)
        self._operator_observations_lock = threading.Lock()
        self._params_duration = math.inf
        self.add_message_listener(
            "PARAM_VALUE", self._suppress_automatic_parameter_retry
        )

    def initialize(self, rate=4, heartbeat_timeout=30):
        self._master.target_system = FLIGHT_CONTROLLER_SYSTEM_ID
        self._master.target_component = FLIGHT_CONTROLLER_COMPONENT_ID
        self._handler.target_system = FLIGHT_CONTROLLER_SYSTEM_ID
        super().initialize(rate=rate, heartbeat_timeout=heartbeat_timeout)
        from pymavlink import mavutil

        request_flight_controller_heartbeat(
            self._master.mav, mavutil.mavlink, rate_hz=10
        )

    def notify_message_listeners(self, name, message):
        if self._qgc_message_callback is not None:
            self._qgc_message_callback(message)
        if not is_flight_controller_message(message):
            return
        self._observe_operator_fact(name, message)
        super().notify_message_listeners(name, message)

    def install_qgc_message_callback(self, callback):
        if not callable(callback):
            raise TypeError("QGC message callback must be callable")
        if self._qgc_message_callback is not None:
            raise RuntimeError("QGC message callback is already installed")
        self._qgc_message_callback = callback

    def drain_operator_observations(self):
        with self._operator_observations_lock:
            observations = tuple(self._operator_observations)
            self._operator_observations.clear()
            return observations

    def _observe_operator_fact(self, name, message):
        from pymavlink import mavutil

        values = {}
        if name == "HEARTBEAT":
            values = {
                "heartbeat_live": True,
                "armed": bool(
                    message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                ),
                "mode": str(mavutil.mode_string_v10(message)).upper(),
            }
        elif name == "SYS_STATUS":
            prearm = int(mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK)
            values = {
                "prearm_checks_healthy": bool(
                    int(message.onboard_control_sensors_enabled) & prearm
                )
                and bool(int(message.onboard_control_sensors_health) & prearm)
            }
        elif name == "EXTENDED_SYS_STATE":
            values = {
                "landed": int(message.landed_state)
                == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            }
        if values:
            with self._operator_observations_lock:
                self._operator_observations.append(
                    VehicleFactObservation(time.time_ns(), **values)
                )

    def _suppress_automatic_parameter_retry(
        self, _vehicle, _name, _message
    ) -> None:
        # DroneKit's base listener runs first and may reset this private delay.
        self._params_duration = math.inf


__all__ = ["SimulationVehicle"]
