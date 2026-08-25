from __future__ import annotations

from pathlib import Path

import pytest

from drone_sim_ardupilot.config import RuntimeConfig, resolve_gazebo_address


RUN_ID = "123e4567-e89b-42d3-a456-426614174000"


def test_descent_parameters_disable_rc_flight_mode_override() -> None:
    parameter_file = Path(__file__).parents[1] / "params/descent.parm"
    parameters = {
        name: value
        for line in parameter_file.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
        for name, value in (line.split(),)
    }

    assert parameters["FLTMODE_CH"] == "0"


def test_runtime_config_builds_lockstep_json_and_network_only_mavlink_argv(tmp_path: Path) -> None:
    config = RuntimeConfig(
        run_id=RUN_ID,
        run_directory=tmp_path,
        executable=Path("/opt/ardupilot/bin/arducopter"),
        parameter_file=Path("/opt/drone_sim/ardupilot/params/descent.parm"),
    )

    assert config.argv == (
        "/opt/ardupilot/bin/arducopter",
        "--model",
        "JSON",
        "--speedup",
        "1",
        "--sim-address",
        "gazebo-runtime",
        "--sim-port-in",
        "9003",
        "--sim-port-out",
        "9002",
        "--serial0",
        "tcp:0.0.0.0:5760",
        "--defaults",
        "/opt/drone_sim/ardupilot/params/descent.parm",
        "--home",
        "37.4003371,-122.0800351,0,0",
        "--wipe",
    )
    assert "--no-lockstep" not in config.argv


@pytest.mark.parametrize(
    "replacement",
    [
        {"run_id": "not-a-uuid"},
        {"run_id": "123E4567-E89B-42D3-A456-426614174000"},
        {"gazebo_host": ""},
        {"gazebo_port": 0},
        {"mavlink_port": 65536},
    ],
)
def test_runtime_config_rejects_ambiguous_identity_and_endpoints(
    tmp_path: Path, replacement: dict[str, object]
) -> None:
    values: dict[str, object] = {"run_id": RUN_ID, "run_directory": tmp_path}
    values.update(replacement)

    with pytest.raises(ValueError):
        RuntimeConfig(**values)  # type: ignore[arg-type]


def test_gazebo_service_name_is_resolved_for_upstream_numeric_only_socket() -> None:
    observed: list[str] = []

    def resolve(name: str) -> str:
        observed.append(name)
        return "172.23.0.2"

    assert resolve_gazebo_address("gazebo-runtime", resolver=resolve) == "172.23.0.2"
    assert observed == ["gazebo-runtime"]


def test_gazebo_resolution_rejects_non_ipv4_result() -> None:
    with pytest.raises(ValueError, match="IPv4"):
        resolve_gazebo_address("gazebo-runtime", resolver=lambda _name: "not-an-address")
