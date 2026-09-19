"""Competition operations layered on the shared single-owner flight interface."""

from __future__ import annotations

import math

from .mission_plan import COMPETITION_TOOLS
from .operations import DroneOperations


def _format_failure(error: Exception) -> str:
    message = str(error)
    cause = error.__cause__
    if cause is None:
        return message
    return f'{message}; caused by {type(cause).__name__}: {_format_failure(cause)}'


class CompetitionOperations(DroneOperations):
    def __init__(self, vehicle, io, emit=None, *, release_agl_m=10.0):
        super().__init__(vehicle, emit)
        self.io = io
        self.release_agl_m = release_agl_m
        self._precision = None
        self._payload_dispatched = False
        self._stable_since = None
        self._last_event_ns = -1
        self._last_waypoint = None
        self._release_waypoint = None
        self._last_correction_ns = -1_000_000_000

    def _dispatch(self, tool, args):
        if tool not in COMPETITION_TOOLS:
            super()._dispatch(tool, args)
            if tool == 'goto_waypoint':
                self._last_waypoint = (args['latitude_deg'], args['longitude_deg'], args['altitude_m'])
            return
        if not self._fresh('heartbeat'):
            raise RuntimeError('fresh vehicle heartbeat required')
        assert self._active is not None
        if tool == 'mission_event':
            if ((args['phase'] == 'HOME' and args['state'] != 'STARTED')
                    or (args['phase'] in {'FM1', 'SEARCH'} and args['state'] == 'COMPLETE')):
                if not self._fresh('landed', 'armed') or not self._values.get('landed') or self._values.get('armed'):
                    raise RuntimeError('landing phase completion requires observed landing and disarm')
            return
        if tool == 'attach_payload':
            if not self._fresh('landed', 'armed') or self._values.get('landed') is not True or self._values.get('armed') is not False:
                raise RuntimeError('attachment requires observed landing and disarm')
            self.io.payloads.start(args['aruco_id'], 'ATTACH', self._active.status.operation_id, self._clock_ns)
            self._payload_dispatched = True
            return
        if self._values.get('armed') is not True or self._values.get('mode') != 'GUIDED':
            raise RuntimeError(f'{tool} requires observed armed GUIDED state')
        if tool == 'precision_land':
            from .configured_precision import PrecisionLanding
            self._precision = PrecisionLanding(
                self._vehicle, aruco_id=args['aruco_id'], started_at_ns=self._clock_ns,
                timeout_sim_s=(self._active.deadline_ns - self._clock_ns) / 1e9,
                emit=self._emit,
            )
        elif tool == 'release_payload':
            if self.io.payloads.attached_id != args['aruco_id']:
                raise RuntimeError('release requires the requested payload to be observed attached')
            if not self._fresh('latitude_deg', 'longitude_deg', 'relative_altitude_m'):
                raise RuntimeError('release requires fresh position')
            self._release_waypoint = self._last_waypoint or (
                self._values['latitude_deg'], self._values['longitude_deg'], self._values['relative_altitude_m'])
            self._stable_since = None
            self._payload_dispatched = False

    def observe(self, telemetry):
        if self._precision is not None and telemetry.ack is not None:
            try:
                self._precision.observe_ack(telemetry.ack)
            except Exception as error:
                self._finish('failed', _format_failure(error))
        super().observe(telemetry)

    def _evaluate(self):
        active = self._active
        if active is None or active.status.tool not in COMPETITION_TOOLS:
            return super()._evaluate()
        try:
            if self._clock_ns >= active.deadline_ns:
                raise TimeoutError(f'{active.status.tool} simulation timeout')
            if not self._fresh('heartbeat'):
                raise RuntimeError('vehicle heartbeat became stale')
            tool, args = active.status.tool, active.args
            if tool == 'mission_event':
                if self.io.ready and self._clock_ns > self._last_event_ns:
                    self.io.publish_event(args['phase'], args['state'], self._clock_ns)
                    self._last_event_ns = self._clock_ns
                    self._finish('succeeded')
                return
            if tool == 'attach_payload':
                if self._values.get('armed') is not False or self._values.get('landed') is not True:
                    raise RuntimeError('landed disarmed state lost during attachment')
                if self.io.payloads.tick(self._clock_ns):
                    self._finish('succeeded')
                return
            state = self.read_vehicle_state()
            clearance = self.io.clearance(state, self._clock_ns)
            clearance_m = None
            if clearance is not None:
                clearance_m, source_ns = clearance
                state['observed_at_ns']['clearance_m'] = source_ns
            if tool == 'precision_land':
                marker = self.io.marker(args['aruco_id'])
                if self._precision.tick(self._clock_ns, state, marker, clearance_m):
                    self._finish('succeeded')
            elif tool == 'release_payload':
                if self._values.get('armed') is not True or self._values.get('mode') != 'GUIDED':
                    raise RuntimeError('armed GUIDED state lost during release')
                if not self._payload_dispatched and self._release_stable(state, clearance_m):
                    self.io.payloads.start(args['aruco_id'], 'RELEASE', active.status.operation_id, self._clock_ns)
                    self._payload_dispatched = True
                if self._payload_dispatched and self.io.payloads.tick(self._clock_ns):
                    self._finish('succeeded')
        except Exception as error:
            self._finish('failed', _format_failure(error))

    def _release_stable(self, state, clearance_m):
        names = ('latitude_deg', 'longitude_deg', 'relative_altitude_m',
                 'horizontal_speed_m_s', 'vertical_speed_m_s', 'roll_rad', 'pitch_rad')
        fresh = all(name in state['observed_at_ns'] and
                    0 <= self._clock_ns - state['observed_at_ns'][name] <= 500_000_000 for name in names)
        if not fresh or clearance_m is None:
            self._stable_since = None
            return False
        latitude, longitude, _ = self._release_waypoint
        north = math.radians(state['latitude_deg'] - latitude) * 6_371_000
        east = math.radians(state['longitude_deg'] - longitude) * 6_371_000 * math.cos(math.radians(latitude))
        if clearance_m < self.release_agl_m + .05 and self._clock_ns - self._last_correction_ns >= 500_000_000:
            altitude = state['relative_altitude_m'] + self.release_agl_m + .20 - clearance_m
            self._vehicle.send_waypoint(latitude, longitude, altitude)
            self._last_correction_ns = self._clock_ns
        stable = (clearance_m >= self.release_agl_m + .02
                  and math.hypot(north, east) <= .08
                  and state['horizontal_speed_m_s'] <= .06
                  and abs(state['vertical_speed_m_s']) <= .05
                  and max(abs(state['roll_rad']), abs(state['pitch_rad'])) <= .1)
        if not stable:
            self._stable_since = None
            return False
        if self._stable_since is None:
            self._stable_since = self._clock_ns
        return self._clock_ns - self._stable_since >= 2_000_000_000

    def _finish(self, state, error=''):
        if self._active is not None and self._active.status.tool in COMPETITION_TOOLS:
            if state != 'succeeded':
                self.io.payloads.cancel()
            self._precision = None
        super()._finish(state, error)
