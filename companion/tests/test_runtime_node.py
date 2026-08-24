from __future__ import annotations

from types import SimpleNamespace

import pytest

from drone_sim_companion.runtime_node import (
    RuntimeConfig,
    connect_mavlink,
    stamp_ns,
    vertical_truth,
)


def test_runtime_config_uses_compose_network_mavlink_endpoint() -> None:
    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": "00000000-0000-4000-8000-000000000001",
            "SIM_RUN_DIRECTORY": "/runs/00000000-0000-4000-8000-000000000001",
        }
    )
    assert config.mavlink_endpoint == "tcp:ardupilot-sitl:5760"
    assert config.startup_timeout_seconds == 60.0


@pytest.mark.parametrize(
    "environment",
    [
        {"SIM_RUN_ID": "bad", "SIM_RUN_DIRECTORY": "/runs/bad"},
        {
            "SIM_RUN_ID": "00000000-0000-4000-8000-000000000001",
            "SIM_RUN_DIRECTORY": "/runs/x",
            "SIM_COMPANION_STARTUP_TIMEOUT_SECONDS": "0",
        },
    ],
)
def test_runtime_config_rejects_invalid_infrastructure_configuration(
    environment: dict[str, str],
) -> None:
    with pytest.raises(ValueError):
        RuntimeConfig.from_environment(environment)


def test_ros_ground_truth_conversion_preserves_native_simulation_stamp() -> None:
    message = SimpleNamespace(
        sim_timestamp=SimpleNamespace(sec=2, nanosec=50_000_000),
        pose=SimpleNamespace(position=SimpleNamespace(z=1.25)),
        twist=SimpleNamespace(linear=SimpleNamespace(z=-0.2)),
        in_contact=False,
    )
    assert stamp_ns(message.sim_timestamp) == 2_050_000_000
    assert vertical_truth(message) == (2_050_000_000, 1.25, -0.2, False)


def test_mavlink_connect_retries_only_within_wall_infrastructure_deadline() -> None:
    attempts = 0
    clock = iter((0.0, 0.2, 0.4))

    def factory(endpoint: str, **options: object) -> object:
        nonlocal attempts
        attempts += 1
        assert endpoint == "tcp:ardupilot-sitl:5760"
        assert options == {"autoreconnect": False, "source_system": 255}
        if attempts < 3:
            raise ConnectionRefusedError("not listening")
        return "connected"

    pauses: list[float] = []
    assert (
        connect_mavlink(
            factory,
            "tcp:ardupilot-sitl:5760",
            deadline=1.0,
            now=lambda: next(clock),
            pause=pauses.append,
        )
        == "connected"
    )
    assert pauses == [0.1, 0.1]
