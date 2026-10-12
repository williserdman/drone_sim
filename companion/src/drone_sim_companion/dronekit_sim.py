"""Simulation-only DroneKit behavior needed for long native parameter streams."""

import math

from dronekit import Vehicle


class SimulationVehicle(Vehicle):
    """Keep DroneKit's real parameter cache without its indexed retry bursts."""

    def __init__(self, handler):
        super().__init__(handler)
        self._params_duration = math.inf
        self.add_message_listener(
            "PARAM_VALUE", self._suppress_automatic_parameter_retry
        )

    def _suppress_automatic_parameter_retry(
        self, _vehicle, _name, _message
    ) -> None:
        # DroneKit's base listener runs first and may reset this private delay.
        self._params_duration = math.inf


__all__ = ["SimulationVehicle"]
