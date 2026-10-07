from __future__ import annotations

import pytest
from jsonschema import ValidationError, validate

from drone_sim_companion.mission import Telemetry
from drone_sim_companion.mission_plan import parse_mission_plan, tool_definitions
from drone_sim_companion.operations import DroneOperations


ARGS = {
    "transit_speed_m_s": 12.0,
    "horizontal_acceleration_m_s2": 3.0,
    "horizontal_jerk_m_s3": 2.0,
    "landing_high_speed_m_s": 1.5,
    "landing_transition_altitude_m": 2.0,
}
PARAMETERS = {
    "WP_SPD": 12.0, "WP_ACC": 3.0, "WP_JERK": 2.0,
    "LAND_SPD_HIGH_MS": 1.5, "LAND_ALT_LOW_M": 2.0,
}


class Vehicle:
    def __init__(self):
        self.configurations = []

    def configure_flight(self, parameters):
        self.configurations.append(parameters)


def observe(operations, stamp=0, *, armed=False, landed=True, parameters=None):
    operations.observe(Telemetry(
        stamp, heartbeat=True, armed=armed, landed=landed,
        precision_parameters=parameters,
    ))


def test_tuning_waits_for_every_readback_and_ignores_preexisting_cache():
    vehicle = Vehicle()
    events = []
    operations = DroneOperations(vehicle, lambda *event: events.append(event))
    observe(operations, parameters=PARAMETERS)

    operation_id = operations.start("configure_flight", ARGS)

    assert vehicle.configurations == [PARAMETERS]
    assert operations.operation_status(operation_id).state == "running"
    for index, (name, value) in enumerate(PARAMETERS.items(), 1):
        observe(operations, index * 100_000_000, parameters={name: value})
        expected = "succeeded" if index == len(PARAMETERS) else "running"
        assert operations.operation_status(operation_id).state == expected
    confirmed = next(event for event in events if event[0] == "flight_configuration_confirmed")
    assert confirmed[2]["parameters"] == PARAMETERS


@pytest.mark.parametrize("armed,landed", [(True, True), (True, False), (False, False)])
def test_tuning_requires_landed_disarmed_before_any_parameter_writes(armed, landed):
    vehicle = Vehicle()
    operations = DroneOperations(vehicle)
    observe(operations, armed=armed, landed=landed)

    operation_id = operations.start("configure_flight", ARGS)

    assert operations.operation_status(operation_id).state == "failed"
    assert "landed/disarmed" in operations.operation_status(operation_id).error
    assert vehicle.configurations == []


def test_mismatched_readback_times_out_without_claiming_configuration_success():
    operations = DroneOperations(Vehicle())
    observe(operations)
    operation_id = operations.start("configure_flight", ARGS, timeout_sim_s=1)
    observe(operations, 100_000_000, parameters={**PARAMETERS, "WP_SPD": 10.0})
    assert operations.operation_status(operation_id).state == "running"

    operations.tick(1_000_000_000)

    assert operations.operation_status(operation_id).state == "failed"
    assert "timeout" in operations.operation_status(operation_id).error


def test_tuning_fails_if_vehicle_arms_while_confirmation_is_pending():
    operations = DroneOperations(Vehicle())
    observe(operations)
    operation_id = operations.start("configure_flight", ARGS)

    observe(operations, 100_000_000, armed=True, parameters=PARAMETERS)

    assert operations.operation_status(operation_id).state == "failed"


def test_tuning_is_available_in_the_existing_plan_and_exported_tool_schema():
    document = {"schema_version": 2, "steps": [{"tool": "configure_flight", "args": ARGS}]}
    plan = parse_mission_plan(document)
    assert dict(plan.steps[0].args) == ARGS
    schema = next(d["parameters"] for d in tool_definitions() if d["name"] == "configure_flight")
    validate(ARGS, schema)


@pytest.mark.parametrize("changes", [
    {"transit_speed_m_s": 0}, {"transit_speed_m_s": 16},
    {"horizontal_acceleration_m_s2": True}, {"horizontal_jerk_m_s3": -1},
    {"landing_high_speed_m_s": 4}, {"landing_transition_altitude_m": 0.5},
    {"LAND_SPD_MS": 2},
])
def test_invalid_tuning_is_rejected_before_flight(changes):
    args = {**ARGS, **changes}
    with pytest.raises(ValueError):
        parse_mission_plan({"schema_version": 2, "steps": [{"tool": "configure_flight", "args": args}]})
    schema = next(d["parameters"] for d in tool_definitions() if d["name"] == "configure_flight")
    with pytest.raises(ValidationError):
        validate(args, schema)
