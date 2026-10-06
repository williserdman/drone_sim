"""Validated tool calls shared by configured missions and future callers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from types import MappingProxyType
from typing import Mapping, TypeAlias


def _number(*, minimum=0, maximum=None, exclusive=False):
    schema = {"type": "number", "exclusiveMinimum" if exclusive else "minimum": minimum}
    if maximum is not None:
        schema["maximum"] = maximum
    return schema


_POSITIVE = _number(exclusive=True)
_GUIDED = {"type": "string", "enum": ["GUIDED"]}
_PAYLOAD_ID = {"type": "integer", "enum": [2, 3, 4]}
_PHASE = {"type": "string", "enum": ["FM1", "FM2", "FM3"]}
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
    "goto_course_waypoint": ({
        "waypoint": {"type": "string", "minLength": 1},
        "payload_target": _PAYLOAD_ID,
        "altitude_m": _POSITIVE,
        "tolerance_m": _POSITIVE,
    }, ["altitude_m"]),
    "find_marker": ({"aruco_id": _PAYLOAD_ID}, ["aruco_id"]),
    "precision_land": ({"aruco_id": _PAYLOAD_ID}, ["aruco_id"]),
    "attach_payload": ({"aruco_id": _PAYLOAD_ID}, ["aruco_id"]),
    "release_payload": ({
        "aruco_id": _PAYLOAD_ID,
        "delivery_mode": {"type": "string", "enum": ["airborne", "grounded"]},
    }, ["aruco_id", "delivery_mode"]),
    "begin_phase": ({"phase": _PHASE}, ["phase"]),
    "check_phase": ({
        "phase": _PHASE,
        "checkpoint": {"type": "string", "enum": ["D", "H"]},
    }, ["phase"]),
}


def tool_definitions() -> list[dict]:
    """JSON-compatible definitions; the runner validates this same vocabulary."""
    import copy

    definitions = []
    for name, (properties, required) in _ARGUMENTS.items():
        parameters = {
            "type": "object",
            "properties": copy.deepcopy(properties),
            "required": list(required),
            "additionalProperties": False,
            **({"minProperties": 1} if name == "wait_for_state" else {}),
        }
        if name == "goto_course_waypoint":
            parameters["oneOf"] = [
                {"required": ["waypoint"], "not": {"required": ["payload_target"]}},
                {"required": ["payload_target"], "not": {"required": ["waypoint"]}},
            ]
        if name == "check_phase":
            parameters["allOf"] = [{
                "if": {"properties": {"phase": {"const": "FM1"}}, "required": ["phase"]},
                "then": {"required": ["checkpoint"]},
                "else": {"not": {"required": ["checkpoint"]}},
            }]
        definitions.append({"name": name, "parameters": parameters})
    return definitions


def positive_seconds(value: object, name: str) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value <= 0):
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
        if schema["type"] == "number":
            assert isinstance(value, (int, float)) and not isinstance(value, bool)
            numeric_value = float(value)
            if not math.isfinite(numeric_value):
                raise ValueError(f"{tool}.{name} must be finite")
            for key, invalid in (
                ("minimum", lambda bound: numeric_value < bound),
                ("maximum", lambda bound: numeric_value > bound),
                ("exclusiveMinimum", lambda bound: numeric_value <= bound),
            ):
                if key in schema and invalid(schema[key]):
                    raise ValueError(f"{tool}.{name} is outside its allowed range")
        if "enum" in schema and value not in schema["enum"]:
            raise ValueError(f"unsupported {tool}.{name}: {value!r}")
    if tool == "takeoff" and args.get("tolerance_m", 0) >= args["altitude_m"]:
        raise ValueError("takeoff tolerance_m must be less than altitude_m")
    if tool == "goto_course_waypoint":
        references = {"waypoint", "payload_target"} & set(args)
        if len(references) != 1:
            raise ValueError("goto_course_waypoint requires exactly one of waypoint or payload_target")
    if tool == "check_phase":
        if args["phase"] == "FM1" and "checkpoint" not in args:
            raise ValueError("check_phase FM1 requires checkpoint D or H")
        if args["phase"] != "FM1" and "checkpoint" in args:
            raise ValueError("check_phase checkpoint is only valid for FM1")
    return dict(args)


@dataclass(frozen=True)
class MissionStep:
    tool: str
    args: Mapping[str, object]
    timeout_sim_s: float = 60.0


@dataclass(frozen=True)
class Condition:
    kind: str
    payload_id: int | None = None


@dataclass(frozen=True)
class ConditionalGroup:
    when: Condition
    steps: tuple[PlanItem, ...]


PlanItem: TypeAlias = MissionStep | ConditionalGroup


@dataclass(frozen=True)
class ConditionResult:
    matched: bool
    reason: str


@dataclass(frozen=True)
class MissionPlan:
    steps: tuple[PlanItem, ...]
    schema_version: int = 1


def _parse_condition(raw: object, path: str) -> Condition:
    if not isinstance(raw, dict) or len(raw) != 1:
        raise ValueError(f"{path}.when must contain one known predicate")
    if raw == {"any_payload_eligible": True}:
        return Condition("any_payload_eligible")
    if set(raw) == {"payload_eligible"} and type(raw["payload_eligible"]) is int \
            and raw["payload_eligible"] in {2, 3, 4}:
        return Condition("payload_eligible", raw["payload_eligible"])
    raise ValueError(f"{path}.when must be any_payload_eligible true or payload_eligible 2, 3, or 4")


def _parse_items(raw_items: object, *, version: int, depth: int, path: str) -> tuple[PlanItem, ...]:
    if not isinstance(raw_items, list) or not raw_items:
        raise ValueError(f"{path} must be a nonempty list")
    items: list[PlanItem] = []
    for index, raw in enumerate(raw_items):
        item_path = f"{path}[{index}]"
        if not isinstance(raw, dict):
            raise ValueError(f"{item_path} must be an object")
        if set(raw) == {"when", "steps"}:
            if version != 2:
                raise ValueError(f"{item_path} conditional groups require schema_version 2")
            if depth >= 2:
                raise ValueError(f"{item_path} exceeds the maximum conditional group depth")
            condition = _parse_condition(raw["when"], item_path)
            children = _parse_items(raw["steps"], version=version, depth=depth + 1,
                                    path=f"{item_path}.steps")
            items.append(ConditionalGroup(condition, children))
            continue
        if (not {"tool", "args"} <= set(raw)
                or not set(raw) <= {"tool", "args", "timeout_sim_s"}):
            raise ValueError(f"{item_path} requires tool, args, optional timeout_sim_s")
        args = validate_arguments(raw["tool"], raw["args"])
        timeout = positive_seconds(raw.get("timeout_sim_s", 60), "timeout_sim_s")
        if raw["tool"] == "hold" and args["duration_sim_s"] >= timeout:
            raise ValueError("hold duration_sim_s must be less than timeout_sim_s")
        items.append(MissionStep(raw["tool"], MappingProxyType(args), timeout))
    return tuple(items)


def parse_mission_plan(document: object) -> MissionPlan:
    if not isinstance(document, dict) or set(document) != {"schema_version", "steps"}:
        raise ValueError("mission_plan requires schema_version and steps only")
    version = document["schema_version"]
    if type(version) is not int or version not in {1, 2}:
        raise ValueError("mission_plan schema_version must be 1 or 2")
    steps = _parse_items(document["steps"], version=version, depth=0, path="steps")
    return MissionPlan(steps, version)
