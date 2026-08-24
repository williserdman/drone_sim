"""Static contract for the first Compose-launched Gazebo runtime milestone."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
RUN_ID = "00000000-0000-4000-8000-000000000001"
RUN_DIRECTORY = f"/tmp/drone-sim/runs/{RUN_ID}"
CONFIG_PATH = f"{RUN_DIRECTORY}/configuration/run.json"
PHASE3_SERVICES = {
    "orchestration-runtime",
    "artifacts-runtime",
    "synthetic-companion",
    "synthetic-ardupilot-sitl",
    "gazebo-runtime",
    "synthetic-electromagnet",
    "synthetic-scorekeeper",
}


def _phase3_document() -> dict:
    environment = os.environ.copy()
    for name in (
        "SIM_RUN_ID",
        "SIM_RUN_DIRECTORY",
        "SIM_CONFIG_PATH",
        "SIM_PHASE2_FAULT",
        "SIM_SYNTHETIC_WALL_DELAY_MS",
        "SIM_SYNTHETIC_QUIESCENCE_DELAY_MS",
    ):
        environment.pop(name, None)
    result = subprocess.run(
        ["docker", "compose", "--profile", "phase3", "config", "--format", "json"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


def test_phase3_profile_has_one_real_gazebo_and_six_retained_runtime_services() -> None:
    document = _phase3_document()
    assert set(document["services"]) == PHASE3_SERVICES
    assert "synthetic-gazebo" not in document["services"]


def test_gazebo_runtime_is_run_scoped_unprivileged_and_has_no_host_port() -> None:
    service = _phase3_document()["services"]["gazebo-runtime"]
    assert service["profiles"] == ["phase3"]
    assert service["image"] == "drone-sim-gazebo-runtime:phase3"
    assert service["build"] == {
        "context": str(ROOT),
        "dockerfile": "gazebo/Dockerfile",
        "target": "runtime",
    }
    assert service["command"] == ["drone-sim-gazebo-runtime"]
    assert service["environment"]["SIM_MODULE"] == "gazebo"
    assert service["environment"]["SIM_RUN_ID"] == RUN_ID
    assert service["environment"]["SIM_RUN_DIRECTORY"] == RUN_DIRECTORY
    assert service["environment"]["SIM_CONFIG_PATH"] == CONFIG_PATH
    assert "ports" not in service
    assert "network_mode" not in service
    assert "privileged" not in service
    assert "devices" not in service
    assert "cap_add" not in service
    assert service["init"] is True
    assert service["restart"] == "no"

    bindings = service["volumes"]
    run_binding = next(item for item in bindings if item["target"] == RUN_DIRECTORY)
    config_binding = next(item for item in bindings if item["target"] == CONFIG_PATH)
    assert run_binding["type"] == "bind"
    assert run_binding["source"] == RUN_DIRECTORY
    assert run_binding.get("read_only", False) is False
    assert config_binding == {
        "type": "bind",
        "source": CONFIG_PATH,
        "target": CONFIG_PATH,
        "read_only": True,
    }


def test_phase2_profile_remains_exactly_the_original_seven_services() -> None:
    environment = os.environ.copy()
    environment["COMPOSE_PROFILES"] = "phase2"
    result = subprocess.run(
        ["docker", "compose", "config", "--services"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert set(result.stdout.splitlines()) == {
        "orchestration-runtime",
        "artifacts-runtime",
        "synthetic-companion",
        "synthetic-ardupilot-sitl",
        "synthetic-gazebo",
        "synthetic-electromagnet",
        "synthetic-scorekeeper",
    }
