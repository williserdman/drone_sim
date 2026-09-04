"""Static contract for the seven-service production Phase 3 topology."""

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
    "companion-runtime",
    "ardupilot-sitl",
    "gazebo-runtime",
    "electromagnet-runtime",
    "scorekeeper-runtime",
}


def _phase3_document(*compose_files: str) -> dict:
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
    file_arguments = [argument for path in compose_files for argument in ("-f", path)]
    result = subprocess.run(
        [
            "docker",
            "compose",
            *file_arguments,
            "--profile",
            "phase3",
            "config",
            "--format",
            "json",
        ],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    return json.loads(result.stdout)


def test_gpu_override_is_opt_in_and_reserved_only_for_gazebo() -> None:
    base_services = _phase3_document()["services"]
    assert all(
        not service.get("deploy", {}).get("resources", {}).get("reservations", {}).get("devices")
        for service in base_services.values()
    )

    gpu_services = _phase3_document("compose.yaml", "compose.gpu.yaml")["services"]
    gpu_request = gpu_services["gazebo-runtime"]["deploy"]["resources"]["reservations"][
        "devices"
    ]
    assert gpu_request == [
        {
            "capabilities": ["gpu"],
            "count": 1,
            "driver": "nvidia",
        }
    ]
    assert gpu_services["gazebo-runtime"]["environment"]["NVIDIA_DRIVER_CAPABILITIES"] == (
        "compute,graphics,utility"
    )
    assert gpu_services["gazebo-runtime"]["environment"][
        "__EGL_VENDOR_LIBRARY_FILENAMES"
    ] == "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    assert [
        {
            "source": Path(volume["source"]).relative_to(ROOT).as_posix(),
            "target": volume["target"],
            "read_only": volume["read_only"],
        }
        for volume in gpu_services["gazebo-runtime"]["volumes"]
        if volume["target"] == "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    ] == [
        {
            "source": "gazebo/config/10_nvidia.json",
            "target": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
            "read_only": True,
        }
    ]
    for name, service in gpu_services.items():
        if name != "gazebo-runtime":
            assert not service.get("deploy", {}).get("resources", {}).get("reservations", {}).get(
                "devices"
            )


def test_gpu_override_nvidia_egl_descriptor_selects_the_driver_library() -> None:
    descriptor = json.loads((ROOT / "gazebo/config/10_nvidia.json").read_text())

    assert descriptor == {
        "file_format_version": "1.0.0",
        "ICD": {"library_path": "libEGL_nvidia.so.0"},
    }


def test_phase3_profile_has_exact_seven_production_services_and_no_synthetic_roles() -> None:
    document = _phase3_document()
    assert set(document["services"]) == PHASE3_SERVICES
    assert not any(name.startswith("synthetic-") for name in document["services"])


def test_companion_starts_only_after_ardupilot_container() -> None:
    companion = _phase3_document()["services"]["companion-runtime"]

    assert companion["depends_on"] == {
        "ardupilot-sitl": {
            "condition": "service_started",
            "required": True,
        }
    }


def test_phase3_production_images_builds_commands_and_modules_are_exact() -> None:
    services = _phase3_document()["services"]
    expected = {
        "orchestration-runtime": (
            "drone-sim-orchestration-runtime:phase2",
            "orchestration/Dockerfile",
            ["python3", "-m", "orchestration.runtime_node"],
            "orchestration",
        ),
        "artifacts-runtime": (
            "drone-sim-artifacts-runtime:phase2",
            "artifacts/Dockerfile",
            ["python3", "-m", "artifacts.runtime_node"],
            "artifacts",
        ),
        "companion-runtime": (
            "drone-sim-companion-runtime:phase3",
            "companion/Dockerfile",
            ["drone-sim-companion-runtime"],
            "companion",
        ),
        "ardupilot-sitl": (
            "drone-sim-ardupilot-runtime:phase3",
            "ardupilot_sitl/Dockerfile",
            None,
            "ardupilot_sitl",
        ),
        "gazebo-runtime": (
            "drone-sim-gazebo-runtime:phase3",
            "gazebo/Dockerfile",
            ["drone-sim-gazebo-runtime"],
            "gazebo",
        ),
        "electromagnet-runtime": (
            "drone-sim-electromagnet-runtime:phase3",
            "electromagnet/Dockerfile",
            ["drone-sim-electromagnet-runtime"],
            "electromagnet",
        ),
        "scorekeeper-runtime": (
            "drone-sim-scorekeeper-runtime:phase3",
            "scorekeeper/Dockerfile",
            ["drone-sim-scorekeeper-runtime"],
            "scorekeeper",
        ),
    }
    for name, (image, dockerfile, command, module) in expected.items():
        service = services[name]
        assert service["profiles"] == (
            ["phase2", "phase3"]
            if name in {"orchestration-runtime", "artifacts-runtime"}
            else ["phase3"]
        )
        assert service["image"] == image
        assert service["build"]["context"] == str(ROOT)
        assert service["build"]["dockerfile"] == dockerfile
        if command is None:
            assert service["command"] is None
            assert service["entrypoint"] is None
        else:
            assert service["command"] == command
        assert service["environment"]["SIM_MODULE"] == module


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


def test_phase3_internal_flight_endpoints_are_exact_and_never_published_to_host() -> None:
    services = _phase3_document()["services"]
    for service in services.values():
        assert "ports" not in service
        assert "network_mode" not in service
    assert services["gazebo-runtime"]["expose"] == ["9002/udp"]
    assert set(services["ardupilot-sitl"]["expose"]) == {"5760/tcp", "9003/udp"}
    assert services["ardupilot-sitl"]["environment"]["SIM_GAZEBO_HOST"] == "gazebo-runtime"
    assert (
        services["companion-runtime"]["environment"]["SIM_MAVLINK_ENDPOINT"]
        == "tcp:ardupilot-sitl:5760"
    )


def test_ardupilot_starts_after_gazebo_service_without_dependency_cycle() -> None:
    services = _phase3_document()["services"]

    ardupilot_dependencies = services["ardupilot-sitl"]["depends_on"]
    assert set(ardupilot_dependencies) == {"gazebo-runtime"}
    assert ardupilot_dependencies["gazebo-runtime"]["condition"] == "service_started"

    graph = {
        name: set(service.get("depends_on", {}))
        for name, service in services.items()
    }
    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(name: str) -> None:
        assert name not in visiting, f"Compose dependency cycle at {name}"
        if name in visited:
            return
        visiting.add(name)
        for dependency in graph[name]:
            visit(dependency)
        visiting.remove(name)
        visited.add(name)

    for service_name in graph:
        visit(service_name)


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
