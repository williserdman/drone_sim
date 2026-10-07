"""Nonblocking, single-owner diagnostic drone operations driven by telemetry."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, replace
import math
from typing import Any, Protocol

from .mission import CommandKind, Telemetry
from .mission_plan import positive_seconds, validate_arguments
from .precision_landing import (
    ANCHOR_DRIFT_M,
    BodyFrd,
    HOLD_REFRESH_NS,
    HOLD_TIMEOUT_NS,
    MarkerAnchor,
    NEAR_GROUND_M,
    PRECISION_TIMEOUT_NS,
    REACQUIRE_FRAMES,
    TARGET_HEALTH_NS,
    horizontal_distance_m,
    marker_offset_ne,
    offset_gps,
    profile_matches,
)


class Vehicle(Protocol):
    def send(self, command: CommandKind, altitude_m: float | None) -> None: ...
    def send_waypoint(self, latitude_deg: float, longitude_deg: float, altitude_m: float) -> None: ...
    def send_mode(self, mode: str) -> None: ...
    def send_landing_target(self, vector: BodyFrd) -> None: ...


@dataclass(frozen=True)
class OperationStatus:
    operation_id: str
    tool: str
    state: str
    started_at_ns: int
    finished_at_ns: int | None = None
    error: str = ""


@dataclass
class _Active:
    status: OperationStatus
    args: dict[str, Any]
    deadline_ns: int
    baseline_sequence: int
    expected_ack: CommandKind | None = None
    acknowledged: bool = False
    context: dict[str, Any] = field(default_factory=dict)


class DroneOperations:
    """Call on the runtime owner thread; reads never consume telemetry or block."""

    def __init__(
        self,
        vehicle: Vehicle,
        emit: Any = None,
        *,
        course: Any = None,
        scenario: Any = None,
        vision: Any = None,
        rangefinder: Any = None,
        payloads: Any = None,
        phases: Any = None,
        attempt: Any = None,
    ) -> None:
        self._vehicle = vehicle
        self._emit = emit or (lambda _event, _stamp, _fields: None)
        self._clock_ns = 0
        self._sequence = 0
        self._values: dict[str, Any] = {}
        self._observed: dict[str, tuple[int, int]] = {}
        self._pose_history: dict[str, deque[tuple[int, float]]] = {
            name: deque(maxlen=32)
            for name in ("latitude_deg", "longitude_deg", "roll_rad", "pitch_rad", "yaw_rad")
        }
        self._operations: dict[str, OperationStatus] = {}
        self._active: _Active | None = None
        self._stopped_reason: str | None = None
        self._recovery_id: str | None = None
        self.armed_at_ns: int | None = None
        self._course = course
        self._scenario = scenario
        self._vision = vision
        self._rangefinder = rangefinder
        self._payloads = payloads
        self._phases = phases
        self._attempt = attempt
        self._course_origin: tuple[float, float] | None = None
        self._marker_anchors: dict[int, MarkerAnchor] = {}
        self._last_marker_sequence = 0
        self._last_precision_touchdown_id: int | None = None
        self._request_sequence = 0

    @property
    def mission_ready(self) -> bool:
        base = (self._fresh("heartbeat", "prearm_checks_healthy")
                and self._values.get("prearm_checks_healthy") is True)
        if not base or self._course is None:
            return base
        return (self._course_origin is not None
                and profile_matches(self._values.get("precision_parameters", {})))

    @property
    def recovery_id(self) -> str | None:
        return self._recovery_id

    def read_vehicle_state(self) -> dict[str, Any]:
        return {**self._values, "sim_timestamp_ns": self._clock_ns,
                "armed_at_ns": self.armed_at_ns,
                "observed_at_ns": {name: stamp for name, (stamp, _) in self._observed.items()}}

    def operation_status(self, operation_id: str) -> OperationStatus:
        return self._operations[operation_id]

    def _fresh(self, *names: str) -> bool:
        return all(name in self._observed
                   and 0 <= self._clock_ns - self._observed[name][0] <= 3_000_000_000
                   for name in names)

    def _after_start(self, *names: str) -> bool:
        assert self._active is not None
        return self._fresh(*names) and all(
            self._observed[name][1] > self._active.baseline_sequence for name in names)

    def _finish(self, state: str, error: str = "") -> None:
        if self._active is None:
            return
        tool = self._active.status.tool
        status = replace(self._active.status, state=state, finished_at_ns=self._clock_ns, error=error)
        self._operations[status.operation_id] = status
        self._active = None
        if tool in {"find_marker", "precision_land"} and self._vision is not None:
            self._vision.cancel()
        if tool in {"attach_payload", "release_payload"} and state != "succeeded":
            cancel = getattr(self._payloads, "cancel", None)
            if cancel is not None:
                cancel()
        if state != "succeeded":
            self._stopped_reason = error or state
        self._emit("operation_finished", self._clock_ns, {
            "operation_id": status.operation_id, "tool": status.tool,
            "state": state, "error": error,
        })

    def start(self, tool: str, args: dict[str, Any], *, timeout_sim_s: float = 60) -> str:
        args = validate_arguments(tool, args)
        timeout = positive_seconds(timeout_sim_s, "timeout_sim_s")
        if self._active is not None:
            raise RuntimeError("another operation is active")
        if self._stopped_reason is not None:
            raise RuntimeError(f"operations stopped: {self._stopped_reason}")
        operation_id = str(len(self._operations) + 1)
        status = OperationStatus(operation_id, tool, "running", self._clock_ns)
        self._operations[operation_id] = status
        self._active = _Active(status, args, self._clock_ns + int(timeout * 1e9), self._sequence)
        self._emit("operation_started", self._clock_ns, {
            "operation_id": operation_id, "tool": tool, "args": args,
            "timeout_sim_s": timeout,
        })
        expiry = self._attempt_expiry_reason()
        if expiry is not None:
            self._finish("failed", expiry)
            self._start_home_recovery()
            return operation_id
        try:
            self._dispatch(tool, args)
            self._evaluate()
        except Exception as error:
            self._finish("failed", str(error))
        return operation_id

    def _dispatch(self, tool: str, args: dict[str, Any]) -> None:
        assert self._active is not None
        if tool == "wait_for_state":
            return
        if tool in {"begin_phase", "check_phase"}:
            self._begin_phase_tool(tool, args)
            return
        if not self._fresh("heartbeat"):
            raise RuntimeError("fresh vehicle heartbeat required")
        if tool in {"takeoff", "goto_waypoint", "goto_course_waypoint", "find_marker", "precision_land", "hold"}:
            if self._values.get("armed") is not True or self._values.get("mode") != "GUIDED":
                raise RuntimeError(f"{tool} requires observed armed GUIDED state")
        if tool == "arm":
            if self._values.get("mode") != "GUIDED":
                raise RuntimeError("arm requires observed GUIDED mode")
            if not self._fresh("prearm_checks_healthy") or self._values.get("prearm_checks_healthy") is not True:
                raise RuntimeError("arm requires healthy prearm checks")
        if tool == "hold":
            return
        if tool == "goto_course_waypoint":
            latitude, longitude = self._resolve_course_target(args)
            self._active.context.update(
                latitude_deg=latitude,
                longitude_deg=longitude,
                stable_since_ns=None,
            )
            self._vehicle.send_waypoint(latitude, longitude, args["altitude_m"])
            return
        if tool == "find_marker":
            if self._vision is None or self._rangefinder is None:
                raise RuntimeError("find_marker requires configured vision and range")
            self._active.context.update(samples=[], last_sequence=self._last_marker_sequence)
            self._vision.request(args["aruco_id"], self._last_marker_sequence)
            return
        if tool == "precision_land":
            self._begin_precision(args["aruco_id"])
            return
        if tool in {"attach_payload", "release_payload"}:
            self._begin_payload(tool, args)
            return
        if tool == "goto_waypoint":
            self._vehicle.send_waypoint(args["latitude_deg"], args["longitude_deg"], args["altitude_m"])
            return
        if tool == "land" and self._fresh("landed") and self._values.get("landed") is True and self._values.get("armed") is False:
            self._finish("succeeded")
            return
        command = {"set_mode": CommandKind.SET_GUIDED, "arm": CommandKind.ARM,
                   "takeoff": CommandKind.TAKEOFF, "land": CommandKind.LAND}[tool]
        self._active.expected_ack = command
        self._vehicle.send(command, args.get("altitude_m"))

    def observe(self, telemetry: Telemetry) -> None:
        self.tick(telemetry.timestamp_ns, evaluate=False)
        self._sequence += 1
        if telemetry.heartbeat:
            self._values["heartbeat"] = True
            self._observed["heartbeat"] = (self._clock_ns, self._sequence)
        for name in ("mode", "armed", "landed", "relative_altitude_m", "latitude_deg",
                     "longitude_deg", "vertical_speed_m_s", "prearm_checks_healthy",
                     "horizontal_velocity_north_m_s", "horizontal_velocity_east_m_s",
                     "roll_rad", "pitch_rad", "yaw_rad", "home_latitude_deg",
                     "home_longitude_deg", "home_altitude_m", "failsafe"):
            value = getattr(telemetry, name)
            if value is None:
                continue
            if isinstance(value, float) and not math.isfinite(value):
                self._finish("failed", f"invalid {name} telemetry")
                continue
            self._values[name] = value
            self._observed[name] = (self._clock_ns, self._sequence)
            if name in self._pose_history:
                history = self._pose_history[name]
                if history and history[-1][0] == self._clock_ns:
                    history.pop()
                history.append((self._clock_ns, float(value)))
        if telemetry.failsafe_reason is not None:
            self._values["failsafe_reason"] = telemetry.failsafe_reason
        if telemetry.precision_parameters:
            parameters = dict(self._values.get("precision_parameters", {}))
            parameters.update(telemetry.precision_parameters)
            self._values["precision_parameters"] = parameters
            self._observed["precision_parameters"] = (self._clock_ns, self._sequence)
        if (self._course_origin is None
                and telemetry.home_latitude_deg is not None
                and telemetry.home_longitude_deg is not None):
            self._course_origin = (
                float(telemetry.home_latitude_deg),
                float(telemetry.home_longitude_deg),
            )
        if telemetry.armed is True and self.armed_at_ns is None:
            self.armed_at_ns = self._clock_ns
            self._emit("mission_armed", self._clock_ns, {})
        if self._active is not None and telemetry.ack is not None:
            if telemetry.ack.command is self._active.expected_ack:
                if telemetry.ack.accepted:
                    self._active.acknowledged = True
                else:
                    self._finish("failed", f"command rejected: {telemetry.ack.command.value}, result {telemetry.ack.result}")
        self._evaluate()

    def tick(self, timestamp_ns: int, *, evaluate: bool = True) -> None:
        if type(timestamp_ns) is not int or timestamp_ns < self._clock_ns:
            self._stopped_reason = "simulation timestamp regressed or invalid"
            self._finish("failed", self._stopped_reason)
            raise ValueError(self._stopped_reason)
        self._clock_ns = timestamp_ns
        if evaluate:
            self._evaluate()

    def _evaluate(self) -> None:
        active = self._active
        if active is None:
            return
        expiry = self._attempt_expiry_reason()
        if active.status.tool != "attempt_recovery" and expiry is not None:
            self._finish("failed", expiry)
            self._start_home_recovery()
            return
        if self._clock_ns >= active.deadline_ns:
            self._finish("failed", f"{active.status.tool} simulation timeout")
            return
        tool, args = active.status.tool, active.args
        values = self._values
        if tool == "attempt_recovery":
            self._evaluate_home_recovery(active)
            return
        if tool not in {"wait_for_state", "begin_phase", "check_phase"} and not self._fresh("heartbeat"):
            self._finish("failed", "vehicle heartbeat became stale")
            return
        if tool in {"takeoff", "goto_waypoint", "goto_course_waypoint", "find_marker", "hold"} and (
                values.get("mode") != "GUIDED" or values.get("armed") is not True):
            self._finish("failed", "armed GUIDED state lost during operation")
            return
        if active.expected_ack is not None and not active.acknowledged:
            return
        complete = False
        if tool == "wait_for_state":
            complete = self._fresh("heartbeat", *args) and all(values.get(key) == value for key, value in args.items())
        elif tool == "set_mode":
            complete = self._after_start("mode") and values.get("mode") == args["mode"]
        elif tool == "arm":
            complete = self._after_start("armed") and values.get("armed") is True
        elif tool == "takeoff":
            tolerance = args.get("tolerance_m", min(0.15, args["altitude_m"] * 0.1))
            complete = self._after_start("relative_altitude_m") and values["relative_altitude_m"] >= args["altitude_m"] - tolerance
        elif tool == "goto_waypoint":
            if self._after_start("latitude_deg", "longitude_deg", "relative_altitude_m"):
                north = math.radians(values["latitude_deg"] - args["latitude_deg"]) * 6_371_000
                longitude_delta = (values["longitude_deg"] - args["longitude_deg"] + 180) % 360 - 180
                east = math.radians(longitude_delta) * 6_371_000 * math.cos(math.radians(args["latitude_deg"]))
                tolerance = args.get("tolerance_m", 1.0)
                complete = math.hypot(north, east) <= tolerance and abs(values["relative_altitude_m"] - args["altitude_m"]) <= tolerance
        elif tool == "goto_course_waypoint":
            try:
                complete = self._evaluate_course_waypoint(active, args)
            except Exception as error:
                self._finish("failed", str(error))
                return
        elif tool == "find_marker":
            try:
                complete = self._evaluate_find_marker(active, args["aruco_id"])
            except Exception as error:
                self._finish("failed", str(error))
                return
        elif tool == "precision_land":
            try:
                complete = self._evaluate_precision(active, args["aruco_id"])
            except Exception as error:
                self._finish("failed", str(error))
                return
        elif tool in {"attach_payload", "release_payload"}:
            try:
                complete = self._evaluate_payload(active)
            except Exception as error:
                self._finish("failed", str(error))
                return
        elif tool in {"begin_phase", "check_phase"}:
            try:
                complete = self._evaluate_phase_tool(active)
            except Exception as error:
                self._finish("failed", str(error))
                return
        elif tool == "hold":
            complete = self._clock_ns - active.status.started_at_ns >= int(args["duration_sim_s"] * 1e9)
        elif tool == "land":
            complete = self._after_start("landed", "armed") and values.get("landed") is True and values.get("armed") is False
        if complete:
            self._finish("succeeded")

    def abort(self, reason: str = "operator abort") -> None:
        self._stopped_reason = reason
        self._finish("cancelled", reason)

    def _resolve_course_target(self, args: dict[str, Any]) -> tuple[float, float]:
        if self._course is None:
            raise RuntimeError("course navigation requires a frozen course")
        if self._course_origin is None:
            raise RuntimeError("course navigation requires pinned home")
        if "waypoint" in args:
            waypoint = self._course.waypoints.get(args["waypoint"])
            if waypoint is None:
                raise RuntimeError(f"unknown course waypoint {args['waypoint']!r}")
            east, north = waypoint.center_xy_m
        else:
            if self._scenario is None:
                raise RuntimeError("payload target requires a frozen scenario")
            try:
                from artifacts.vfs_contract import resolve_waypoint_xy

                east, north = resolve_waypoint_xy(
                    self._course,
                    self._scenario,
                    payload_target=args["payload_target"],
                )
            except (ImportError, AttributeError):
                payload = self._scenario.payload(args["payload_target"])
                waypoint = self._course.waypoints.get(payload.delivery)
                if waypoint is None:
                    raise RuntimeError(f"unknown payload target {payload.delivery!r}")
                offset_east, offset_north = payload.release_offset_xy_m
                east = waypoint.center_xy_m[0] + offset_east
                north = waypoint.center_xy_m[1] + offset_north
        latitude, longitude = self._course_origin
        return offset_gps(latitude, longitude, float(north), float(east))

    def _evaluate_course_waypoint(self, active: _Active, args: dict[str, Any]) -> bool:
        names = (
            "latitude_deg", "longitude_deg", "relative_altitude_m",
            "horizontal_velocity_north_m_s", "horizontal_velocity_east_m_s",
        )
        if not self._after_start(*names):
            return False
        target = (active.context["latitude_deg"], active.context["longitude_deg"])
        current = (self._values["latitude_deg"], self._values["longitude_deg"])
        tolerance = args.get("tolerance_m", 1.0)
        arrived = (
            horizontal_distance_m(current, target) <= tolerance
            and abs(self._values["relative_altitude_m"] - args["altitude_m"]) <= tolerance
        )
        speed = math.hypot(
            self._values["horizontal_velocity_north_m_s"],
            self._values["horizontal_velocity_east_m_s"],
        )
        if not arrived or speed >= 0.10:
            active.context["stable_since_ns"] = None
            return False
        if active.context["stable_since_ns"] is None:
            active.context["stable_since_ns"] = self._clock_ns
            return False
        return self._clock_ns - active.context["stable_since_ns"] >= 1_000_000_000

    def _marker_candidate(self, observation, aruco_id: int, *, acquisition_height=True):
        assert self._active is not None
        assert self._rangefinder is not None
        assert self._course is not None
        self._active.context["marker_rejection"] = "no_observation"
        self._active.context["marker_pose_timestamps"] = {}
        if observation is None or observation.aruco_id != aruco_id:
            return None
        if observation.sequence <= self._active.context.get("last_sequence", 0):
            self._active.context["marker_rejection"] = "repeated_frame"
            return None
        self._active.context["last_sequence"] = observation.sequence
        self._last_marker_sequence = max(self._last_marker_sequence, observation.sequence)
        if not observation.exposure_timestamp_ns <= observation.receipt_timestamp_ns <= self._clock_ns:
            self._active.context["marker_rejection"] = "frame_clock_order"
            return None
        if self._clock_ns - observation.exposure_timestamp_ns > 500_000_000:
            self._active.context["marker_rejection"] = "frame_age"
            return None
        required = ("latitude_deg", "longitude_deg", "roll_rad", "pitch_rad", "yaw_rad")
        if not self._fresh(*required):
            self._active.context["marker_rejection"] = "telemetry_unavailable"
            return None
        pose = {}
        for name in required:
            stamp, value = min(
                self._pose_history[name],
                key=lambda row: abs(row[0] - observation.exposure_timestamp_ns),
            )
            self._active.context["marker_pose_timestamps"][name] = stamp
            if abs(stamp - observation.exposure_timestamp_ns) > 200_000_000:
                self._active.context["marker_rejection"] = "telemetry_skew"
                return None
            pose[name] = value
        self._active.context["marker_pose"] = pose
        if observation.down_m <= 0:
            self._active.context["marker_rejection"] = "invalid_depth"
            return None
        try:
            range_observation = self._rangefinder.latest(self._clock_ns)
        except RuntimeError:
            self._active.context["marker_rejection"] = "range_unavailable"
            return None
        acquisition = getattr(self._course.attempt, "acquisition_agl_m", 4.572)
        if (not 0 <= self._clock_ns - range_observation.timestamp_ns <= 500_000_000
                or (acquisition_height
                    and abs(range_observation.distance_m - acquisition) > 1.0)):
            self._active.context["marker_rejection"] = "range_age_or_height"
            return None
        self._active.context["marker_rejection"] = "healthy"
        north, east = marker_offset_ne(
            observation,
            roll_rad=pose["roll_rad"],
            pitch_rad=pose["pitch_rad"],
            yaw_rad=pose["yaw_rad"],
        )
        return offset_gps(
            pose["latitude_deg"], pose["longitude_deg"], north, east
        )

    def _evaluate_find_marker(self, active: _Active, aruco_id: int) -> bool:
        assert self._vision is not None
        candidate = self._marker_candidate(self._vision.poll(), aruco_id)
        if candidate is None:
            return False
        samples = active.context["samples"]
        if any(horizontal_distance_m(candidate, prior) > ANCHOR_DRIFT_M for prior in samples):
            samples.clear()
        samples.append(candidate)
        if len(samples) < 5:
            return False
        latitude = sorted(sample[0] for sample in samples)[len(samples) // 2]
        longitude = sorted(sample[1] for sample in samples)[len(samples) // 2]
        self._marker_anchors[aruco_id] = MarkerAnchor(
            aruco_id, latitude, longitude, self._clock_ns,
            active.context["last_sequence"],
        )
        return True

    def _begin_precision(self, aruco_id: int) -> None:
        if self._vision is None or self._rangefinder is None:
            raise RuntimeError("precision_land requires configured vision and range")
        anchor = self._marker_anchors.get(aruco_id)
        if anchor is None:
            raise RuntimeError("precision_land requires a current matching marker acquisition")
        if not profile_matches(self._values.get("precision_parameters", {})):
            raise RuntimeError("live precision-landing profile does not match the approved profile")
        assert self._active is not None
        self._active.deadline_ns = min(
            self._active.deadline_ns, self._clock_ns + PRECISION_TIMEOUT_NS
        )
        self._active.context.update(
            anchor=anchor,
            phase="await_first",
            last_sequence=self._last_marker_sequence,
            last_healthy_ns=self._clock_ns,
            hold_started_ns=None,
            next_hold_ns=None,
            reacquired=0,
            retry=0,
        )
        self._vision.request(aruco_id, self._last_marker_sequence)

    def _evaluate_precision(self, active: _Active, aruco_id: int) -> bool:
        assert self._rangefinder is not None
        assert self._vision is not None
        assert self._course is not None
        if (self._after_start("landed", "armed")
                and self._values.get("landed") is True
                and self._values.get("armed") is False):
            self._last_precision_touchdown_id = aruco_id
            return True
        if self._values.get("failsafe") is True:
            self._finish("failed", "failsafe observed during precision landing")
            return False
        if self._values.get("mode") not in {"GUIDED", "LAND"}:
            self._finish("failed", "operator mode change revoked precision landing")
            return False
        try:
            range_observation = self._rangefinder.latest(self._clock_ns)
        except RuntimeError:
            range_observation = None
        phase = active.context["phase"]
        if phase == "retry_transit":
            if self._clock_ns >= active.context["next_hold_ns"]:
                self._vehicle.send_waypoint(*active.context["hold_target"])
                active.context["next_hold_ns"] = self._clock_ns + HOLD_REFRESH_NS
            latitude, longitude, altitude = active.context["hold_target"]
            arrived = (
                self._fresh("latitude_deg", "longitude_deg", "relative_altitude_m")
                and horizontal_distance_m(
                    (self._values["latitude_deg"], self._values["longitude_deg"]),
                    (latitude, longitude),
                ) <= 0.25
                and abs(self._values["relative_altitude_m"] - altitude) <= 0.25
            )
            if not arrived:
                self._vision.poll()
                return False
            phase = "hold"
            active.context.update(phase=phase, hold_started_ns=self._clock_ns, reacquired=0)
            self._emit("precision_recovery", self._clock_ns, {
                "aruco_id": aruco_id, "phase": "search_arrived", "retry": active.context["retry"],
            })
        if (phase == "tracking"
                and self._values.get("mode") == "LAND"
                and range_observation is not None
                and range_observation.distance_m <= NEAR_GROUND_M):
            active.context["last_healthy_ns"] = self._clock_ns
            return False
        candidate_observation = self._vision.poll()
        candidate = self._marker_candidate(
            candidate_observation, aruco_id, acquisition_height=False
        )
        healthy = False
        reason = active.context["marker_rejection"]
        if candidate is not None:
            anchor = active.context["anchor"]
            healthy = horizontal_distance_m(
                candidate, (anchor.latitude_deg, anchor.longitude_deg)
            ) <= ANCHOR_DRIFT_M
            if not healthy:
                reason = "anchor_drift"
            if healthy and phase in {"await_first", "hold"}:
                pose = active.context["marker_pose"]
                north, east = marker_offset_ne(
                    candidate_observation,
                    roll_rad=pose["roll_rad"],
                    pitch_rad=pose["pitch_rad"],
                    yaw_rad=pose["yaw_rad"],
                )
                healthy = math.hypot(north, east) <= 0.50
                if not healthy:
                    reason = "not_centered"
        if candidate_observation is not None:
            self._emit("precision_observation", self._clock_ns, {
                "aruco_id": aruco_id, "phase": phase, "retry": active.context["retry"],
                "frame_sequence": candidate_observation.sequence,
                "exposure_timestamp_ns": candidate_observation.exposure_timestamp_ns,
                "receipt_timestamp_ns": candidate_observation.receipt_timestamp_ns,
                "healthy": healthy, "reason": reason,
                "reacquired_frames_before": active.context["reacquired"],
                "telemetry_timestamps_ns": active.context["marker_pose_timestamps"],
                "range_timestamp_ns": None if range_observation is None else range_observation.timestamp_ns,
                "range_m": None if range_observation is None else range_observation.distance_m,
                "forward_m": candidate_observation.forward_m,
                "right_m": candidate_observation.right_m,
                "down_m": candidate_observation.down_m,
            })
        if phase == "await_first":
            if not healthy:
                return False
            self._vehicle.send_mode("LAND")
            active.context["phase"] = "tracking"
            active.context["last_healthy_ns"] = self._clock_ns
            return False
        if phase == "tracking":
            if healthy and candidate_observation is not None:
                if self._values.get("mode") != "LAND":
                    return False
                self._vehicle.send_landing_target(BodyFrd(
                    candidate_observation.forward_m,
                    candidate_observation.right_m,
                    candidate_observation.down_m,
                ))
                active.context["last_healthy_ns"] = self._clock_ns
                return False
            if (range_observation is not None
                    and range_observation.distance_m <= NEAR_GROUND_M):
                active.context["last_healthy_ns"] = self._clock_ns
                return False
            if self._clock_ns - active.context["last_healthy_ns"] < TARGET_HEALTH_NS:
                return False
            self._vehicle.send_mode("GUIDED")
            active.context.update(
                phase="hold", hold_started_ns=self._clock_ns,
                next_hold_ns=self._clock_ns, reacquired=0,
                hold_target=(self._values["latitude_deg"],
                             self._values["longitude_deg"],
                             self._values["relative_altitude_m"]),
            )
            return False
        if self._clock_ns >= active.context["next_hold_ns"]:
            self._vehicle.send_waypoint(*active.context["hold_target"])
            active.context["next_hold_ns"] = self._clock_ns + HOLD_REFRESH_NS
        if candidate_observation is not None:
            active.context["reacquired"] = (
                active.context["reacquired"] + 1 if healthy else 0
            )
        if active.context["reacquired"] >= REACQUIRE_FRAMES:
            self._vehicle.send_mode("LAND")
            active.context.update(
                phase="tracking", last_healthy_ns=self._clock_ns, reacquired=0,
            )
            return False
        if self._clock_ns - active.context["hold_started_ns"] >= HOLD_TIMEOUT_NS:
            if active.context["retry"] >= 1:
                self._finish("failed", "precision target reacquisition failed")
                return False
            anchor = active.context["anchor"]
            self._vehicle.send_waypoint(
                anchor.latitude_deg, anchor.longitude_deg,
                getattr(self._course.attempt, "acquisition_agl_m", 4.572),
            )
            active.context.update(
                phase="retry_transit", retry=1, hold_started_ns=None,
                next_hold_ns=self._clock_ns + HOLD_REFRESH_NS, reacquired=0,
                hold_target=(anchor.latitude_deg, anchor.longitude_deg,
                             getattr(self._course.attempt, "acquisition_agl_m", 4.572)),
            )
        return False

    def _next_request_id(self, prefix: str) -> str:
        self._request_sequence += 1
        return f"{prefix}:{self._request_sequence}"

    def _begin_payload(self, tool: str, args: dict[str, Any]) -> None:
        if self._payloads is None:
            raise RuntimeError(f"{tool} requires payload command transport")
        assert self._active is not None
        aruco_id = args["aruco_id"]
        if tool == "attach_payload":
            if (self._values.get("landed") is not True
                    or self._values.get("armed") is not False
                    or self._last_precision_touchdown_id != aruco_id):
                raise RuntimeError("attach_payload requires matching aligned touchdown and disarm")
            action, mode = "attach", None
        else:
            mode = args["delivery_mode"]
            if mode == "airborne":
                assert self._course is not None
                minimum = getattr(self._course.attempt, "minimum_agl_m", 9.144)
                if (self._values.get("armed") is not True
                        or self._values.get("landed") is not False
                        or self._values.get("mode") != "GUIDED"
                        or not self._fresh("relative_altitude_m")
                        or self._values["relative_altitude_m"] < minimum):
                    raise RuntimeError("airborne release prerequisites are not satisfied")
            elif self._values.get("landed") is not True or self._values.get("armed") is not False:
                raise RuntimeError("grounded release requires landed and disarmed state")
            action = "release"
        request_id = self._next_request_id(action)
        self._active.context["request_id"] = request_id
        self._payloads.request(action, aruco_id, request_id, mode)

    def _evaluate_payload(self, active: _Active) -> bool:
        assert self._payloads is not None
        confirmation = self._payloads.confirmation(active.context["request_id"])
        if confirmation is None:
            return False
        if confirmation.request_id != active.context["request_id"]:
            return False
        if confirmation.confirmed is not True:
            self._finish("failed", confirmation.reason or "payload action was not confirmed")
            return False
        return True

    def _begin_phase_tool(self, tool: str, args: dict[str, Any]) -> None:
        if self._phases is None:
            raise RuntimeError(f"{tool} requires phase exchange")
        assert self._active is not None
        request_id = self._next_request_id(args["phase"])
        self._active.context["request_id"] = request_id
        if tool == "begin_phase":
            self._phases.begin(args["phase"], request_id, self._clock_ns)
        else:
            self._phases.check(
                args["phase"], args.get("checkpoint", ""), request_id, self._clock_ns
            )

    def _evaluate_phase_tool(self, active: _Active) -> bool:
        if active.status.tool == "begin_phase":
            return True
        assert self._phases is not None
        decision = self._phases.decision(active.context["request_id"])
        if decision is None:
            return False
        expected_checkpoint = active.args.get("checkpoint", "")
        if (decision.request_id != active.context["request_id"]
                or decision.phase != active.args["phase"]
                or decision.checkpoint != expected_checkpoint):
            return False
        if decision.approved is not True:
            self._finish("failed", decision.reason or "judge rejected phase check")
            return False
        return True

    def recover_land(self, *, timeout_sim_s: float = 60) -> str | None:
        """Host-only, one local LAND attempt; never resume the failed sequence."""
        if self._recovery_id is not None:
            return self._recovery_id
        if (not self._fresh("heartbeat") or self._values.get("armed") is not True
                or self._values.get("mode") not in {"GUIDED", "LAND"}):
            return None
        reason = self._stopped_reason or "recovery requested"
        self.abort(reason)
        self._stopped_reason = None
        try:
            self._recovery_id = self.start("land", {}, timeout_sim_s=timeout_sim_s)
        finally:
            self._stopped_reason = reason
        return self._recovery_id

    def _attempt_expiry_reason(self) -> str | None:
        if self._attempt is None or self._course is None:
            return None
        duration = getattr(self._course.attempt, "duration_seconds", 600)
        if self._attempt.active_elapsed_ns >= int(float(duration) * 1e9):
            return "active attempt deadline expired"
        approval_timestamp_ns = getattr(self._attempt, "approval_timestamp_ns", 0)
        approval_timeout_s = getattr(self._course.attempt, "approval_timeout_s", 60.0)
        if (approval_timestamp_ns > 0
                and self._clock_ns - approval_timestamp_ns >= int(approval_timeout_s * 1e9)):
            return "approval-to-takeoff deadline expired"
        return None

    def _start_home_recovery(self) -> str | None:
        if self._recovery_id is not None:
            return self._recovery_id
        if (not self._fresh("heartbeat", "mode", "armed", "landed")
                or self._values.get("failsafe") is True
                or self._values.get("mode") not in {"GUIDED", "LAND"}):
            return None
        if self._values.get("landed") is True and self._values.get("armed") is False:
            return None
        reason = self._stopped_reason or "attempt recovery"
        operation_id = str(len(self._operations) + 1)
        status = OperationStatus(operation_id, "attempt_recovery", "running", self._clock_ns)
        timeout = getattr(self._course.attempt, "recovery_timeout_s", 60.0)
        active = _Active(
            status, {}, self._clock_ns + int(float(timeout) * 1e9), self._sequence
        )
        self._operations[operation_id] = status
        self._active = active
        self._recovery_id = operation_id
        self._stopped_reason = reason
        if self._values.get("mode") == "LAND":
            active.context["phase"] = "land"
            return operation_id
        try:
            latitude, longitude = self._resolve_course_target({"waypoint": "H"})
        except Exception:
            self._active = None
            self._operations[operation_id] = replace(
                status, state="failed", finished_at_ns=self._clock_ns,
                error="home recovery target is unavailable",
            )
            return operation_id
        altitude = max(
            float(self._values.get("relative_altitude_m", 0.0)),
            float(getattr(self._course.attempt, "transit_agl_m", 10.0)),
        )
        active.context.update(
            phase="return", latitude_deg=latitude, longitude_deg=longitude,
            altitude_m=altitude,
        )
        self._vehicle.send_waypoint(latitude, longitude, altitude)
        return operation_id

    def _evaluate_home_recovery(self, active: _Active) -> None:
        if self._clock_ns >= active.deadline_ns:
            self._finish("failed", "attempt recovery simulation timeout")
            return
        if (not self._fresh("heartbeat", "mode", "armed", "landed")
                or self._values.get("failsafe") is True
                or self._values.get("mode") not in {"GUIDED", "LAND"}):
            self._finish("failed", "attempt recovery authority is unavailable")
            return
        if self._values.get("landed") is True and self._values.get("armed") is False:
            self._finish("succeeded")
            return
        if active.context["phase"] == "land":
            return
        names = (
            "latitude_deg", "longitude_deg", "relative_altitude_m",
            "horizontal_velocity_north_m_s", "horizontal_velocity_east_m_s",
        )
        if not self._fresh(*names):
            return
        current = (self._values["latitude_deg"], self._values["longitude_deg"])
        target = (active.context["latitude_deg"], active.context["longitude_deg"])
        speed = math.hypot(
            self._values["horizontal_velocity_north_m_s"],
            self._values["horizontal_velocity_east_m_s"],
        )
        if horizontal_distance_m(current, target) <= 1.0 and speed < 0.10:
            self._vehicle.send(CommandKind.LAND, None)
            active.context["phase"] = "land"
