"""Simulation-only DroneKit behavior needed for long native parameter streams."""

import math

from dronekit import Vehicle

from .mavlink_adapter import (
    FLIGHT_CONTROLLER_COMPONENT_ID,
    FLIGHT_CONTROLLER_SYSTEM_ID,
    is_flight_controller_message,
    request_flight_controller_heartbeat,
)


class SimulationVehicle(Vehicle):
    """Keep DroneKit's real parameter cache without its indexed retry bursts."""

    def __init__(self, handler):
        super().__init__(handler)
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
        if not is_flight_controller_message(message):
            return
        super().notify_message_listeners(name, message)

    def _suppress_automatic_parameter_retry(
        self, _vehicle, _name, _message
    ) -> None:
        # DroneKit's base listener runs first and may reset this private delay.
        self._params_duration = math.inf


__all__ = ["SimulationVehicle"]
