"""Validated tool calls shared by configured missions and future callers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping


def _number(*, minimum=0, maximum=None, exclusive=False):
    schema = {"type": "number", "exclusiveMinimum" if exclusive else "minimum": minimum}
    if maximum is not None:
        schema["maximum"] = maximum
    return schema


_POSITIVE = _number(exclusive=True)
_GUIDED = {"type": "string", "enum": ["GUIDED"]}
COMPETITION_TOOLS = frozenset({"precision_land", "attach_payload", "release_payload", "mission_event"})
_ARGUMENTS = {
    "set_mode": ({"mode": _GUIDED}, ["mode"]),
    "arm": ({}, []),
    "wait_for_state": ({
        "armed": {"type": "boolean"},
        "landed": {"type": "boolean"},
        "mode": {"type": "string", "minLength": 1},
    }, []),
    "takeoff": ({"altitude_m": _POSITIVE, "tolerance_m": _POSITIVE}, ["altitude_m"]),
    "goto_waypoint": ({
        "latitude_deg": _number(minimum=-90, maximum=90),
        "longitude_deg": _number(minimum=-180, maximum=180),
        "altitude_m": _POSITIVE,
        "tolerance_m": _POSITIVE,
    }, ["latitude_deg", "longitude_deg", "altitude_m"]),
    "hold": ({"duration_sim_s": _POSITIVE}, ["duration_sim_s"]),
    "land": ({}, []),
    "precision_land": ({"aruco_id": {"type": "integer", "minimum": 0, "maximum": 249}}, ["aruco_id"]),
    "attach_payload": ({"aruco_id": {"type": "integer", "enum": [3, 4]}}, ["aruco_id"]),
    "release_payload": ({"aruco_id": {"type": "integer", "enum": [2, 3, 4]}}, ["aruco_id"]),
    "mission_event": ({
        "phase": {"type": "string", "enum": ["FM1", "FM2", "FM3_3", "FM3_4", "HOME", "SEARCH", "DELIVERY"]},
        "state": {"type": "string", "enum": ["STARTED", "COMPLETE", "DISARMED"]},
    }, ["phase", "state"]),
}


def tool_definitions() -> list[dict]:
    """JSON-compatible definitions; the runner validates this same vocabulary."""
    import copy

    return [{"name": name, "parameters": {
        "type": "object", "properties": copy.deepcopy(properties),
        "required": list(required), "additionalProperties": False,
        **({"minProperties": 1} if name == "wait_for_state" else {}),
    }} for name, (properties, required) in _ARGUMENTS.items()]


def positive_seconds(value: object, name: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def validate_arguments(tool: object, args: object) -> dict:
    if not isinstance(tool, str) or tool not in _ARGUMENTS:
        raise ValueError(f"unsupported tool: {tool!r}")
    if not isinstance(args, Mapping):
        raise ValueError(f"{tool} args must be an object")
    properties, required = _ARGUMENTS[tool]
    if not set(required) <= set(args) or not set(args) <= set(properties):
        raise ValueError(f"{tool} args have missing or unknown fields")
    if tool == "wait_for_state" and not args:
        raise ValueError("wait_for_state requires at least one condition")
    for name, value in args.items():
        schema = properties[name]
        valid = {
            "number": type(value) in (int, float),
            "integer": type(value) is int,
            "boolean": type(value) is bool,
            "string": isinstance(value, str) and bool(value),
        }[schema["type"]]
        if not valid:
            raise ValueError(f"{tool}.{name} must be a {schema['type']}")
        if schema["type"] in {"number", "integer"}:
            if not math.isfinite(value):
                raise ValueError(f"{tool}.{name} must be finite")
            for key, invalid in (
                ("minimum", lambda bound: value < bound),
                ("maximum", lambda bound: value > bound),
                ("exclusiveMinimum", lambda bound: value <= bound),
            ):
                if key in schema and invalid(schema[key]):
                    raise ValueError(f"{tool}.{name} is outside its allowed range")
        if "enum" in schema and value not in schema["enum"]:
            raise ValueError(f"unsupported {tool}.{name}: {value!r}")
    if tool == "takeoff" and args.get("tolerance_m", 0) >= args["altitude_m"]:
        raise ValueError("takeoff tolerance_m must be less than altitude_m")
    return dict(args)


@dataclass(frozen=True)
class MissionStep:
    tool: str
    args: Mapping[str, object]
    timeout_sim_s: float = 60.0


@dataclass(frozen=True)
class MissionPlan:
    steps: tuple[MissionStep, ...]


def parse_mission_plan(document: object) -> MissionPlan:
    if not isinstance(document, dict) or set(document) != {"schema_version", "steps"}:
        raise ValueError("mission_plan requires schema_version and steps only")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        raise ValueError("mission_plan schema_version must be 1")
    raw_steps = document["steps"]
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ValueError("mission_plan steps must be a nonempty list")
    steps = []
    for index, raw in enumerate(raw_steps):
        if (not isinstance(raw, dict) or not {"tool", "args"} <= set(raw)
                or not set(raw) <= {"tool", "args", "timeout_sim_s"}):
            raise ValueError(f"mission step {index + 1} requires tool, args, optional timeout_sim_s")
        args = validate_arguments(raw["tool"], raw["args"])
        timeout = positive_seconds(raw.get("timeout_sim_s", 60), "timeout_sim_s")
        if raw["tool"] == "hold" and args["duration_sim_s"] >= timeout:
            raise ValueError("hold duration_sim_s must be less than timeout_sim_s")
        steps.append(MissionStep(raw["tool"], MappingProxyType(args), timeout))
    return MissionPlan(tuple(steps))
