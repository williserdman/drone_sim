"""Nonblocking, single-owner diagnostic drone operations driven by telemetry."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Protocol

from .mission import CommandKind, Telemetry
from .mission_plan import positive_seconds, validate_arguments


class Vehicle(Protocol):
    def send(self, command: CommandKind, altitude_m: float | None) -> None: ...
    def send_waypoint(self, latitude_deg: float, longitude_deg: float, altitude_m: float) -> None: ...


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
    args: dict
    deadline_ns: int
    baseline_sequence: int
    expected_ack: CommandKind | None = None
    acknowledged: bool = False


class DroneOperations:
    """Call on the runtime owner thread; reads never consume telemetry or block."""

    def __init__(self, vehicle: Vehicle, emit=None) -> None:
        self._vehicle = vehicle
        self._emit = emit or (lambda _event, _stamp, _fields: None)
        self._clock_ns = 0
        self._sequence = 0
        self._values: dict[str, object] = {}
        self._observed: dict[str, tuple[int, int]] = {}
        self._operations: dict[str, OperationStatus] = {}
        self._active: _Active | None = None
        self._stopped_reason: str | None = None
        self._recovery_id: str | None = None
        self.armed_at_ns: int | None = None

    @property
    def mission_ready(self) -> bool:
        return self._fresh("heartbeat", "prearm_checks_healthy") and self._values.get("prearm_checks_healthy") is True

    def read_vehicle_state(self) -> dict:
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
        status = replace(self._active.status, state=state, finished_at_ns=self._clock_ns, error=error)
        self._operations[status.operation_id] = status
        self._active = None
        if state != "succeeded":
            self._stopped_reason = error or state
        self._emit("operation_finished", self._clock_ns, {
            "operation_id": status.operation_id, "tool": status.tool,
            "state": state, "error": error,
        })

    def start(self, tool: str, args: dict, *, timeout_sim_s: float = 60) -> str:
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
        try:
            self._dispatch(tool, args)
            self._evaluate()
        except Exception as error:
            self._finish("failed", str(error))
        return operation_id

    def _dispatch(self, tool: str, args: dict) -> None:
        assert self._active is not None
        if tool == "wait_for_state":
            return
        if not self._fresh("heartbeat"):
            raise RuntimeError("fresh vehicle heartbeat required")
        if tool in {"takeoff", "goto_waypoint", "hold"}:
            if self._values.get("armed") is not True or self._values.get("mode") != "GUIDED":
                raise RuntimeError(f"{tool} requires observed armed GUIDED state")
        if tool == "arm":
            if self._values.get("mode") != "GUIDED":
                raise RuntimeError("arm requires observed GUIDED mode")
            if not self._fresh("prearm_checks_healthy") or self._values.get("prearm_checks_healthy") is not True:
                raise RuntimeError("arm requires healthy prearm checks")
        if tool == "hold":
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
                     "horizontal_speed_m_s", "roll_rad", "pitch_rad", "yaw_rad"):
            value = getattr(telemetry, name)
            if value is None:
                continue
            if isinstance(value, float) and not math.isfinite(value):
                self._finish("failed", f"invalid {name} telemetry")
                continue
            self._values[name] = value
            self._observed[name] = (self._clock_ns, self._sequence)
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
        if self._clock_ns >= active.deadline_ns:
            self._finish("failed", f"{active.status.tool} simulation timeout")
            return
        tool, args = active.status.tool, active.args
        values = self._values
        if tool != "wait_for_state" and not self._fresh("heartbeat"):
            self._finish("failed", "vehicle heartbeat became stale")
            return
        if tool in {"takeoff", "goto_waypoint", "hold"} and (
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
        elif tool == "hold":
            complete = self._clock_ns - active.status.started_at_ns >= int(args["duration_sim_s"] * 1e9)
        elif tool == "land":
            complete = self._after_start("landed", "armed") and values.get("landed") is True and values.get("armed") is False
        if complete:
            self._finish("succeeded")

    def abort(self, reason: str = "operator abort") -> None:
        self._stopped_reason = reason
        self._finish("cancelled", reason)

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
