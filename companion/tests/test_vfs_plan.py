from __future__ import annotations

import pytest
from jsonschema import ValidationError, validate

from drone_sim_companion.configured_mission import ConfiguredMission
from drone_sim_companion.mission import Ack, CommandKind, Telemetry
from drone_sim_companion.mission_plan import (
    ConditionResult,
    MissionPlan,
    tool_definitions,
    parse_mission_plan,
)
from drone_sim_companion.operations import DroneOperations


class Vehicle:
    def __init__(self) -> None:
        self.commands: list[tuple[object, object]] = []

    def send(self, command, altitude_m) -> None:
        self.commands.append((command, altitude_m))

    def send_waypoint(self, latitude_deg, longitude_deg, altitude_m) -> None:
        self.commands.append(("waypoint", latitude_deg, longitude_deg, altitude_m))


class Conditions:
    def evaluate(self, condition):
        assert condition.kind == "payload_eligible"
        assert condition.payload_id == 2
        return False


def ready(operations, *, armed=False):
    operations.observe(Telemetry(
        0,
        heartbeat=True,
        mode="GUIDED",
        armed=armed,
        landed=not armed,
        prearm_checks_healthy=True,
    ))


def v2(*items):
    return parse_mission_plan({"schema_version": 2, "steps": list(items)})


def test_v2_skips_ineligible_retrieval_without_commands():
    plan = parse_mission_plan({
        "schema_version": 2,
        "steps": [{
            "when": {"payload_eligible": 2},
            "steps": [{"tool": "arm", "args": {}}],
        }],
    })
    vehicle = Vehicle()
    events = []
    mission = ConfiguredMission(
        plan,
        DroneOperations(vehicle),
        conditions=Conditions(),
        emit=lambda event, stamp, fields: events.append((event, stamp, fields)),
    )

    mission.tick(123)

    assert mission.state == "succeeded"
    assert vehicle.commands == []
    assert events == [(
        "conditional_group_skipped",
        123,
        {
            "path": "steps[0]",
            "condition": {"payload_eligible": 2},
            "reason": "payload 2 is not eligible",
        },
    )]


def test_nested_eligibility_runs_only_the_selected_payload_branch():
    plan = v2({
        "when": {"any_payload_eligible": True},
        "steps": [
            {
                "when": {"payload_eligible": 2},
                "steps": [{"tool": "arm", "args": {}}],
            },
            {
                "when": {"payload_eligible": 3},
                "steps": [{"tool": "arm", "args": {}}],
            },
        ],
    })

    class Selective:
        def evaluate(self, condition):
            if condition.kind == "any_payload_eligible":
                return ConditionResult(True, "FM2 inspection approved payload 3")
            return ConditionResult(condition.payload_id == 3, f"payload {condition.payload_id} eligibility frozen")

    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    ready(operations)
    mission = ConfiguredMission(plan, operations, conditions=Selective())

    mission.tick(0)

    assert vehicle.commands == [(CommandKind.ARM, None)]


@pytest.mark.parametrize("evaluator, expected", [
    (None, "requires an eligibility evaluator"),
    (lambda _condition: (_ for _ in ()).throw(RuntimeError("stale FM2 decision")), "stale FM2 decision"),
    (lambda _condition: (_ for _ in ()).throw(RuntimeError("decision belongs to another run")), "another run"),
    (lambda _condition: ConditionResult(False, ""), "clear reason"),
])
def test_missing_stale_foreign_or_reasonless_decision_fails_without_commands(evaluator, expected):
    class Adapter:
        def evaluate(self, condition):
            return evaluator(condition)

    vehicle = Vehicle()
    conditions = None if evaluator is None else Adapter()
    mission = ConfiguredMission(
        v2({
            "when": {"payload_eligible": 2},
            "steps": [{"tool": "arm", "args": {}}],
        }),
        DroneOperations(vehicle),
        conditions=conditions,
    )

    mission.tick(0)

    assert mission.state == "failed"
    assert expected in mission.error
    assert vehicle.commands == []


@pytest.mark.parametrize("bad", [
    {"when": {"payload_eligible": 2}, "steps": []},
    {"when": {"payload_eligible": 5}, "steps": [{"tool": "arm", "args": {}}]},
    {"when": {"any_payload_eligible": False}, "steps": [{"tool": "arm", "args": {}}]},
    {"when": {"payload_eligible": 2}, "tool": "arm", "args": {}, "steps": []},
    {
        "when": {"any_payload_eligible": True},
        "steps": [{
            "when": {"payload_eligible": 2},
            "steps": [{
                "when": {"payload_eligible": 2},
                "steps": [{"tool": "arm", "args": {}}],
            }],
        }],
    },
    {
        "when": {"payload_eligible": 2},
        "steps": [{"tool": "not_a_tool", "args": {}}],
    },
])
def test_invalid_or_too_deep_groups_fail_during_plan_parsing(bad):
    with pytest.raises(ValueError):
        v2(bad)


@pytest.mark.parametrize("tool,args", [
    ("goto_course_waypoint", {"waypoint": "H", "altitude_m": 10}),
    ("goto_course_waypoint", {"payload_target": 2, "altitude_m": 10, "tolerance_m": 0.5}),
    ("find_marker", {"aruco_id": 2}),
    ("precision_land", {"aruco_id": 3}),
    ("attach_payload", {"aruco_id": 4}),
    ("release_payload", {"aruco_id": 2, "delivery_mode": "airborne"}),
    ("begin_phase", {"phase": "FM2"}),
    ("check_phase", {"phase": "FM1", "checkpoint": "D"}),
    ("check_phase", {"phase": "FM1", "checkpoint": "H"}),
    ("check_phase", {"phase": "FM3"}),
])
def test_vfs_tool_calls_are_registered_and_parse(tool, args):
    parsed = v2({"tool": tool, "args": args})

    assert parsed.schema_version == 2
    assert parsed.steps[0].tool == tool
    schema = next(
        definition["parameters"]
        for definition in tool_definitions()
        if definition["name"] == tool
    )
    validate(args, schema)


@pytest.mark.parametrize("tool,args", [
    ("goto_course_waypoint", {"altitude_m": 10}),
    ("goto_course_waypoint", {"waypoint": "H", "payload_target": 2, "altitude_m": 10}),
    ("find_marker", {"aruco_id": 1}),
    ("release_payload", {"aruco_id": 2, "delivery_mode": "hovering"}),
    ("check_phase", {"phase": "FM1"}),
    ("check_phase", {"phase": "FM2", "checkpoint": "D"}),
])
def test_vfs_tool_calls_reject_ambiguous_or_out_of_contract_arguments(tool, args):
    with pytest.raises(ValueError):
        v2({"tool": tool, "args": args})

    schema = next(
        definition["parameters"]
        for definition in tool_definitions()
        if definition["name"] == tool
    )
    with pytest.raises(ValidationError):
        validate(args, schema)


def test_v1_plan_constructor_and_flat_parser_remain_compatible():
    parsed = parse_mission_plan({
        "schema_version": 1,
        "steps": [{"tool": "arm", "args": {}}],
    })

    assert parsed == MissionPlan(parsed.steps)
    with pytest.raises(ValueError, match="conditional groups require schema_version 2"):
        parse_mission_plan({
            "schema_version": 1,
            "steps": [{
                "when": {"payload_eligible": 2},
                "steps": [{"tool": "arm", "args": {}}],
            }],
        })


def test_v2_operation_failure_stops_before_later_conditional_command():
    class Eligible:
        def evaluate(self, condition):
            return True

    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    ready(operations)
    mission = ConfiguredMission(v2(
        {"tool": "arm", "args": {}},
        {
            "when": {"payload_eligible": 2},
            "steps": [{"tool": "takeoff", "args": {"altitude_m": 2}}],
        },
    ), operations, conditions=Eligible())

    mission.tick(0)
    operations.observe(Telemetry(1, ack=Ack(CommandKind.ARM, False, 4)))
    mission.tick(1)

    assert mission.state == "failed"
    assert vehicle.commands == [(CommandKind.ARM, None)]
