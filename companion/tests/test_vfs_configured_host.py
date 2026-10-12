from __future__ import annotations

from types import SimpleNamespace

from drone_sim_companion.configured_runtime import ConfiguredHost
from drone_sim_companion.mission import Telemetry
from drone_sim_companion.mission_plan import parse_mission_plan
from drone_sim_companion.precision_landing import PRECISION_PARAMETERS

from .test_vfs_operations import Attempt, Payloads, Vehicle, course


class Lifecycle:
    def emit(self, *_args) -> None:
        pass

    def observe_mission_readiness(self, **_kwargs) -> None:
        pass

    def observe_terminal(self, _state) -> None:
        pass


class Protocol:
    def write_status(self, _status) -> None:
        pass


def test_attempt_expiry_home_recovery_survives_mission_failure_propagation() -> None:
    attempt = Attempt()
    payloads = Payloads()
    vehicle = Vehicle()
    plan = parse_mission_plan({
        "schema_version": 2,
        "steps": [{
            "tool": "release_payload",
            "args": {"aruco_id": 2, "delivery_mode": "airborne"},
        }],
    })
    host = ConfiguredHost(
        plan,
        vehicle,
        Lifecycle(),
        Protocol(),
        "00000000-0000-4000-8000-000000000001",
        operation_dependencies={
            "course": course(),
            "payloads": payloads,
            "attempt": attempt,
        },
    )
    host.observe(Telemetry(
        0,
        heartbeat=True,
        prearm_checks_healthy=True,
        armed=True,
        landed=False,
        mode="GUIDED",
        relative_altitude_m=10.0,
        latitude_deg=41.0,
        longitude_deg=-81.0,
        horizontal_velocity_north_m_s=0.0,
        horizontal_velocity_east_m_s=0.0,
        home_latitude_deg=41.0,
        home_longitude_deg=-81.0,
        precision_parameters=dict(PRECISION_PARAMETERS),
    ))
    host.tick(0, mission_running=True)
    attempt.active_elapsed_ns = 600_000_000_000

    host.tick(1, mission_running=True)

    assert host.error == "active attempt deadline expired"
    assert host.recovery_pending
    assert any(command[0] == "waypoint" for command in vehicle.commands)
    assert vehicle.commands[-1][0].value == "LAND"
