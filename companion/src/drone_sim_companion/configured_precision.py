"""Pure nonblocking precision-landing policy for configured missions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real

from .mission import Ack, CommandKind


_EARTH_RADIUS_M = 6_378_137.0
_FRESH_NS = 500_000_000
_ACQUISITION_AGL_M = 4.572
_ACQUISITION_TOLERANCE_M = 0.15
_CENTER_TOLERANCE_M = 0.05
_ANCHOR_DRIFT_LIMIT_M = 0.20
_CORRECTION_GAIN = 0.5
_MAX_OBSERVATIONS_PER_CELL = 10
_TARGET_HEALTH_TIMEOUT_NS = 500_000_000
_HOLD_TIMEOUT_NS = 5_000_000_000
_HOLD_COMMAND_PERIOD_NS = 200_000_000
_PRECISION_FLOOR_M = 0.75
_REQUIRED_ANCHOR_FRAMES = 5
_REQUIRED_REACQUIRE_FRAMES = 5
_GRID_OFFSETS_NE = (
    (0.0, 0.0),
    (0.0, 1.5),
    (1.5, 1.5),
    (1.5, 0.0),
    (1.5, -1.5),
    (0.0, -1.5),
    (-1.5, -1.5),
    (-1.5, 0.0),
    (-1.5, 1.5),
)


def _finite(value: object) -> bool:
    return (
        isinstance(value, Real)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


@dataclass(frozen=True)
class MarkerObservation:
    timestamp_ns: int
    sequence: int
    aruco_id: int
    forward_m: float
    right_m: float
    down_m: float

    def __post_init__(self) -> None:
        if (
            isinstance(self.timestamp_ns, bool)
            or not isinstance(self.timestamp_ns, int)
            or self.timestamp_ns < 0
        ):
            raise ValueError("marker timestamp_ns must be a nonnegative integer")
        if (
            isinstance(self.sequence, bool)
            or not isinstance(self.sequence, int)
            or self.sequence <= 0
        ):
            raise ValueError("marker sequence must be a positive integer")
        if (
            isinstance(self.aruco_id, bool)
            or not isinstance(self.aruco_id, int)
            or self.aruco_id < 0
        ):
            raise ValueError("marker aruco_id must be a nonnegative integer")
        if not all(_finite(value) for value in (self.forward_m, self.right_m, self.down_m)):
            raise ValueError("marker coordinates must be finite")


class PrecisionLanding:
    """Drive one precision landing from armed GUIDED flight using supplied evidence."""

    def __init__(
        self,
        vehicle,
        *,
        aruco_id: int,
        started_at_ns: int,
        timeout_sim_s: float,
        emit=None,
    ) -> None:
        if isinstance(aruco_id, bool) or not isinstance(aruco_id, int) or aruco_id < 0:
            raise ValueError("aruco_id must be a nonnegative integer")
        if (
            isinstance(started_at_ns, bool)
            or not isinstance(started_at_ns, int)
            or started_at_ns < 0
        ):
            raise ValueError("started_at_ns must be a nonnegative integer")
        if not _finite(timeout_sim_s) or float(timeout_sim_s) <= 0:
            raise ValueError("timeout_sim_s must be finite and positive")
        if emit is not None and not callable(emit):
            raise TypeError("emit must be callable")
        self._vehicle = vehicle
        self._aruco_id = aruco_id
        self._deadline_ns = started_at_ns + int(float(timeout_sim_s) * 1_000_000_000)
        self._emit = emit or (lambda _event, _stamp, _fields: None)
        self._last_timestamp_ns = started_at_ns
        self._stage = "approach"
        self._expected_ack: CommandKind | None = None
        self._acknowledged = False
        self._mode_commanded_at_ns: int | None = None
        self._acquisition_target: tuple[float, float, float] | None = None
        self._grid_origin: tuple[float, float, float] | None = None
        self._grid_index = 0
        self._grid_target: tuple[float, float, float] | None = None
        self._observations_at_cell = 0
        self._last_search_tick_ns: int | None = None
        self._last_marker_sequence = 0
        self._anchor_samples: list[tuple[float, float]] = []
        self._anchor: tuple[float, float] | None = None
        self._land_commanded = False
        self._land_commanded_at_ns: int | None = None
        self._last_healthy_ns: int | None = None
        self._hold_target: tuple[float, float, float] | None = None
        self._hold_started_ns: int | None = None
        self._next_hold_command_ns: int | None = None
        self._reacquired_frames = 0
        self._retry_used = False

    def observe_ack(self, ack: Ack) -> None:
        if not isinstance(ack, Ack):
            raise TypeError("ack must be an Ack")
        if ack.command is not self._expected_ack:
            return
        if not ack.accepted:
            name = "LAND" if ack.command is CommandKind.LAND else "GUIDED"
            raise RuntimeError(f"{name} command rejected: result {ack.result}")
        self._acknowledged = True

    def tick(
        self,
        timestamp_ns: int,
        state: dict,
        marker: MarkerObservation | None,
        clearance_m: float | None,
    ) -> bool:
        self._validate_tick(timestamp_ns, state, marker, clearance_m)
        if timestamp_ns >= self._deadline_ns:
            raise TimeoutError("precision landing simulation timeout")
        self._last_timestamp_ns = timestamp_ns

        if (
            self._land_commanded
            and self._fresh(state, timestamp_ns, "armed", "landed")
            and self._observed_after(state, "armed", self._land_commanded_at_ns)
            and self._observed_after(state, "landed", self._land_commanded_at_ns)
        ):
            if state.get("landed") is True:
                if state.get("armed") is False:
                    self._transition("complete", timestamp_ns)
                    return True
                return False

        if self._stage == "complete":
            return True
        if self._stage in {"approach", "search", "search_move", "retry_approach"}:
            if not self._require_flight_state(state, timestamp_ns, "GUIDED"):
                return False
        elif self._stage == "land":
            if not self._require_flight_state(state, timestamp_ns, "LAND"):
                return False
        elif self._stage == "hold":
            if not self._require_flight_state(state, timestamp_ns, "GUIDED"):
                return False

        if self._stage == "approach":
            return self._tick_approach(timestamp_ns, state, marker, clearance_m)
        if self._stage == "search_move":
            return self._tick_search_move(timestamp_ns, state, marker, clearance_m)
        if self._stage == "search":
            return self._tick_search(timestamp_ns, state, marker, clearance_m)
        if self._stage == "wait_land":
            return self._tick_wait_mode(timestamp_ns, state, marker, clearance_m, "LAND")
        if self._stage == "land":
            return self._tick_land(timestamp_ns, state, marker, clearance_m)
        if self._stage == "wait_guided":
            return self._tick_wait_mode(timestamp_ns, state, marker, clearance_m, "GUIDED")
        if self._stage == "hold":
            return self._tick_hold(timestamp_ns, state, marker, clearance_m)
        if self._stage == "retry_approach":
            return self._tick_retry_approach(timestamp_ns, state, marker, clearance_m)
        raise RuntimeError(f"unknown precision landing stage {self._stage!r}")

    def _tick_approach(self, timestamp_ns, state, marker, clearance_m) -> bool:
        if not self._evidence_fresh(state, timestamp_ns, clearance=True):
            return False
        if self._acquisition_target is None:
            target_altitude = self._corrected_altitude(state, clearance_m)
            self._acquisition_target = (
                float(state["latitude_deg"]),
                float(state["longitude_deg"]),
                target_altitude,
            )
            self._send_waypoint(self._acquisition_target)
            return False
        if not self._at_waypoint(state, self._acquisition_target, clearance_m):
            return False
        self._grid_origin = self._acquisition_target
        self._reset_search()
        self._transition("search", timestamp_ns)
        return self._tick_search(timestamp_ns, state, marker, clearance_m)

    def _tick_search_move(self, timestamp_ns, state, marker, clearance_m) -> bool:
        if not self._evidence_fresh(state, timestamp_ns, clearance=True):
            return False
        assert self._grid_target is not None
        if not self._at_waypoint(state, self._grid_target, clearance_m):
            return False
        self._anchor_samples.clear()
        self._observations_at_cell = 0
        self._transition("search", timestamp_ns)
        return self._tick_search(timestamp_ns, state, marker, clearance_m)

    def _tick_search(self, timestamp_ns, state, marker, clearance_m) -> bool:
        if not self._evidence_fresh(state, timestamp_ns, clearance=True):
            return False
        new_frame, evidence = self._marker_evidence(timestamp_ns, state, marker)
        if evidence is not None:
            north, east = evidence
            if math.hypot(north, east) > _CENTER_TOLERANCE_M:
                target = self._offset_waypoint(
                    float(state["latitude_deg"]),
                    float(state["longitude_deg"]),
                    float(state["relative_altitude_m"]),
                    north * _CORRECTION_GAIN,
                    east * _CORRECTION_GAIN,
                )
                self._grid_target = target
                self._send_waypoint(target)
                self._anchor_samples.clear()
                self._transition("search_move", timestamp_ns)
                return False
            projected = self._offset_gps(
                float(state["latitude_deg"]),
                float(state["longitude_deg"]),
                north,
                east,
            )
            if any(
                self._horizontal_distance(projected, prior) > _ANCHOR_DRIFT_LIMIT_M
                for prior in self._anchor_samples
            ):
                self._anchor_samples = [projected]
            else:
                self._anchor_samples.append(projected)
            if len(self._anchor_samples) >= _REQUIRED_ANCHOR_FRAMES:
                latitudes = sorted(point[0] for point in self._anchor_samples)
                longitudes = sorted(point[1] for point in self._anchor_samples)
                middle = len(self._anchor_samples) // 2
                self._anchor = (latitudes[middle], longitudes[middle])
                self._last_healthy_ns = timestamp_ns
                self._send_mode(CommandKind.LAND, timestamp_ns, "wait_land")
                self._land_commanded = True
                self._land_commanded_at_ns = timestamp_ns
                return False
        if marker is not None and not new_frame:
            return False
        if self._last_search_tick_ns != timestamp_ns:
            self._last_search_tick_ns = timestamp_ns
            self._observations_at_cell += 1
        if self._observations_at_cell >= _MAX_OBSERVATIONS_PER_CELL:
            self._advance_grid(timestamp_ns)
        return False

    def _tick_wait_mode(
        self, timestamp_ns, state, marker, clearance_m, expected_mode: str
    ) -> bool:
        if not self._acknowledged:
            return False
        if not self._fresh(state, timestamp_ns, "mode"):
            return False
        if not self._observed_after(state, "mode", self._mode_commanded_at_ns):
            return False
        if state.get("mode") != expected_mode:
            return False
        self._expected_ack = None
        self._acknowledged = False
        if expected_mode == "LAND":
            self._transition("land", timestamp_ns)
            return self._tick_land(timestamp_ns, state, marker, clearance_m)
        self._transition("hold", timestamp_ns)
        self._hold_started_ns = timestamp_ns
        self._next_hold_command_ns = timestamp_ns
        return self._tick_hold(timestamp_ns, state, marker, clearance_m)

    def _tick_land(self, timestamp_ns, state, marker, clearance_m) -> bool:
        if self._clearance_fresh(state, timestamp_ns, clearance_m):
            assert clearance_m is not None
            if float(clearance_m) <= _PRECISION_FLOOR_M:
                self._last_healthy_ns = timestamp_ns
                return False
        _, evidence = self._healthy_landing_marker(timestamp_ns, state, marker)
        if evidence is not None:
            assert marker is not None
            self._vehicle.send_landing_target(
                float(marker.forward_m), float(marker.right_m), float(marker.down_m)
            )
            self._last_healthy_ns = timestamp_ns
            return False
        assert self._last_healthy_ns is not None
        if timestamp_ns - self._last_healthy_ns < _TARGET_HEALTH_TIMEOUT_NS:
            return False
        if not self._fresh(
            state,
            timestamp_ns,
            "latitude_deg",
            "longitude_deg",
            "relative_altitude_m",
        ):
            return False
        self._hold_target = (
            float(state["latitude_deg"]),
            float(state["longitude_deg"]),
            float(state["relative_altitude_m"]),
        )
        self._reacquired_frames = 0
        self._send_mode(CommandKind.SET_GUIDED, timestamp_ns, "wait_guided")
        return False

    def _tick_hold(self, timestamp_ns, state, marker, clearance_m) -> bool:
        assert self._hold_target is not None
        assert self._hold_started_ns is not None
        assert self._next_hold_command_ns is not None
        if timestamp_ns >= self._next_hold_command_ns:
            self._send_waypoint(self._hold_target)
            self._next_hold_command_ns = timestamp_ns + _HOLD_COMMAND_PERIOD_NS
        new_frame, evidence = self._healthy_landing_marker(
            timestamp_ns, state, marker, require_centered=True
        )
        if new_frame:
            if evidence is None:
                self._reacquired_frames = 0
            else:
                self._reacquired_frames += 1
                if self._reacquired_frames >= _REQUIRED_REACQUIRE_FRAMES:
                    self._last_healthy_ns = timestamp_ns
                    self._send_mode(CommandKind.LAND, timestamp_ns, "wait_land")
                    return False
        if timestamp_ns - self._hold_started_ns < _HOLD_TIMEOUT_NS:
            return False
        if self._retry_used:
            raise RuntimeError("precision landing target was not reacquired")
        if not self._evidence_fresh(state, timestamp_ns, clearance=True):
            return False
        assert self._anchor is not None
        retry_target = (
            self._anchor[0],
            self._anchor[1],
            self._corrected_altitude(state, clearance_m),
        )
        self._retry_used = True
        self._acquisition_target = retry_target
        self._send_waypoint(retry_target)
        self._transition("retry_approach", timestamp_ns)
        return False

    def _tick_retry_approach(self, timestamp_ns, state, marker, clearance_m) -> bool:
        if not self._evidence_fresh(state, timestamp_ns, clearance=True):
            return False
        assert self._acquisition_target is not None
        if not self._at_waypoint(state, self._acquisition_target, clearance_m):
            return False
        self._grid_origin = self._acquisition_target
        self._grid_index = 0
        self._reset_search()
        self._transition("search", timestamp_ns)
        return self._tick_search(timestamp_ns, state, marker, clearance_m)

    def _advance_grid(self, timestamp_ns: int) -> None:
        self._grid_index += 1
        if self._grid_index >= len(_GRID_OFFSETS_NE):
            raise RuntimeError("precision landing target was not acquired")
        assert self._grid_origin is not None
        north, east = _GRID_OFFSETS_NE[self._grid_index]
        self._grid_target = self._offset_waypoint(*self._grid_origin, north, east)
        self._send_waypoint(self._grid_target)
        self._anchor_samples.clear()
        self._transition("search_move", timestamp_ns)

    def _reset_search(self) -> None:
        self._grid_target = None
        self._observations_at_cell = 0
        self._last_search_tick_ns = None
        self._anchor_samples.clear()

    def _send_mode(self, command: CommandKind, timestamp_ns: int, stage: str) -> None:
        self._vehicle.send(command, None)
        self._expected_ack = command
        self._acknowledged = False
        self._mode_commanded_at_ns = timestamp_ns
        if command is CommandKind.LAND:
            self._land_commanded_at_ns = timestamp_ns
        self._transition(stage, timestamp_ns)

    def _send_waypoint(self, target: tuple[float, float, float]) -> None:
        self._vehicle.send_waypoint(*target)

    def _transition(self, stage: str, timestamp_ns: int) -> None:
        self._stage = stage
        self._emit("precision_stage", timestamp_ns, {"stage": stage, "aruco_id": self._aruco_id})

    def _corrected_altitude(self, state: dict, clearance_m: float | None) -> float:
        if not _finite(clearance_m):
            raise RuntimeError("fresh precision clearance is unavailable")
        target = float(state["relative_altitude_m"]) - (
            float(clearance_m) - _ACQUISITION_AGL_M
        )
        if not math.isfinite(target) or target <= 0:
            raise RuntimeError("precision acquisition altitude is invalid")
        return target

    def _at_waypoint(
        self,
        state: dict,
        target: tuple[float, float, float],
        clearance_m: float | None,
    ) -> bool:
        if not _finite(clearance_m):
            return False
        current = (float(state["latitude_deg"]), float(state["longitude_deg"]))
        return (
            self._horizontal_distance(current, target[:2]) <= _ACQUISITION_TOLERANCE_M
            and abs(float(state["relative_altitude_m"]) - target[2])
            <= _ACQUISITION_TOLERANCE_M
            and abs(float(clearance_m) - _ACQUISITION_AGL_M)
            <= _ACQUISITION_TOLERANCE_M
        )

    def _marker_evidence(
        self, timestamp_ns: int, state: dict, marker: MarkerObservation | None
    ) -> tuple[bool, tuple[float, float] | None]:
        if marker is None or marker.sequence <= self._last_marker_sequence:
            return False, None
        self._last_marker_sequence = marker.sequence
        if (
            marker.aruco_id != self._aruco_id
            or timestamp_ns - marker.timestamp_ns < 0
            or timestamp_ns - marker.timestamp_ns >= _FRESH_NS
            or marker.down_m <= 0
            or not self._fresh(
                state,
                timestamp_ns,
                "latitude_deg",
                "longitude_deg",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
            )
        ):
            return True, None
        return True, self._marker_offset_ne(marker, state)

    def _healthy_landing_marker(
        self,
        timestamp_ns: int,
        state: dict,
        marker: MarkerObservation | None,
        *,
        require_centered: bool = False,
    ) -> tuple[bool, tuple[float, float] | None]:
        new_frame, evidence = self._marker_evidence(timestamp_ns, state, marker)
        if evidence is None or self._anchor is None:
            return new_frame, None
        north, east = evidence
        if require_centered and math.hypot(north, east) > _CENTER_TOLERANCE_M:
            return True, None
        projected = self._offset_gps(
            float(state["latitude_deg"]),
            float(state["longitude_deg"]),
            north,
            east,
        )
        if self._horizontal_distance(projected, self._anchor) > _ANCHOR_DRIFT_LIMIT_M:
            return True, None
        return True, evidence

    @staticmethod
    def _marker_offset_ne(marker: MarkerObservation, state: dict) -> tuple[float, float]:
        roll = float(state["roll_rad"])
        pitch = float(state["pitch_rad"])
        yaw = float(state["yaw_rad"])
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        north = (
            cy * cp * marker.forward_m
            + (cy * sp * sr - sy * cr) * marker.right_m
            + (cy * sp * cr + sy * sr) * marker.down_m
        )
        east = (
            sy * cp * marker.forward_m
            + (sy * sp * sr + cy * cr) * marker.right_m
            + (sy * sp * cr - cy * sr) * marker.down_m
        )
        return north, east

    @classmethod
    def _offset_waypoint(
        cls, latitude: float, longitude: float, altitude: float, north: float, east: float
    ) -> tuple[float, float, float]:
        target = cls._offset_gps(latitude, longitude, north, east)
        return target[0], target[1], altitude

    @staticmethod
    def _offset_gps(
        latitude: float, longitude: float, north: float, east: float
    ) -> tuple[float, float]:
        return (
            latitude + math.degrees(north / _EARTH_RADIUS_M),
            longitude
            + math.degrees(
                east / (_EARTH_RADIUS_M * math.cos(math.radians(latitude)))
            ),
        )

    @staticmethod
    def _horizontal_distance(
        first: tuple[float, float], second: tuple[float, float]
    ) -> float:
        latitude = math.radians((first[0] + second[0]) / 2.0)
        north = math.radians(second[0] - first[0]) * _EARTH_RADIUS_M
        east = (
            math.radians(second[1] - first[1])
            * math.cos(latitude)
            * _EARTH_RADIUS_M
        )
        return math.hypot(north, east)

    def _require_flight_state(
        self, state: dict, timestamp_ns: int, mode: str
    ) -> bool:
        if not self._fresh(state, timestamp_ns, "mode", "armed", "landed"):
            return False
        if state.get("armed") is not True or state.get("landed") is not False:
            raise RuntimeError("precision landing lost confirmed airborne armed state")
        if state.get("mode") != mode:
            raise RuntimeError(f"precision landing requires observed {mode} mode")
        return True

    def _evidence_fresh(
        self, state: dict, timestamp_ns: int, *, clearance: bool = False
    ) -> bool:
        names = [
            "latitude_deg",
            "longitude_deg",
            "relative_altitude_m",
            "roll_rad",
            "pitch_rad",
            "yaw_rad",
        ]
        if clearance:
            names.append("clearance_m")
        return self._fresh(state, timestamp_ns, *names)

    def _clearance_fresh(
        self, state: dict, timestamp_ns: int, clearance_m: float | None
    ) -> bool:
        return _finite(clearance_m) and self._fresh(state, timestamp_ns, "clearance_m")

    @staticmethod
    def _fresh(state: dict, timestamp_ns: int, *names: str) -> bool:
        observed = state.get("observed_at_ns")
        if not isinstance(observed, dict):
            return False
        for name in names:
            stamp = observed.get(name)
            value = state.get(name)
            if (
                isinstance(stamp, bool)
                or not isinstance(stamp, int)
                or stamp < 0
                or timestamp_ns - stamp < 0
                or timestamp_ns - stamp >= _FRESH_NS
                or (name != "clearance_m" and value is None)
            ):
                return False
        return True

    @staticmethod
    def _observed_after(state: dict, name: str, boundary_ns: int | None) -> bool:
        if boundary_ns is None:
            return False
        observed = state.get("observed_at_ns")
        if not isinstance(observed, dict):
            return False
        stamp = observed.get(name)
        return (
            isinstance(stamp, int)
            and not isinstance(stamp, bool)
            and stamp > boundary_ns
        )

    def _validate_tick(
        self,
        timestamp_ns: int,
        state: dict,
        marker: MarkerObservation | None,
        clearance_m: float | None,
    ) -> None:
        if (
            isinstance(timestamp_ns, bool)
            or not isinstance(timestamp_ns, int)
            or timestamp_ns < self._last_timestamp_ns
        ):
            raise ValueError("precision landing timestamp regressed or is invalid")
        if not isinstance(state, dict):
            raise TypeError("state must be a dictionary")
        if marker is not None and not isinstance(marker, MarkerObservation):
            raise TypeError("marker must be a MarkerObservation or None")
        if clearance_m is not None and not _finite(clearance_m):
            raise ValueError("clearance_m must be finite or None")
        for name in (
            "latitude_deg",
            "longitude_deg",
            "relative_altitude_m",
            "roll_rad",
            "pitch_rad",
            "yaw_rad",
            "horizontal_speed_m_s",
            "vertical_speed_m_s",
        ):
            value = state.get(name)
            if value is not None and not _finite(value):
                raise ValueError(f"state {name} must be finite")


__all__ = ["MarkerObservation", "PrecisionLanding"]
