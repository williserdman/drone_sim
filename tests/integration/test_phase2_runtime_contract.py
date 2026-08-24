"""Static contract checks for the Phase 2 container topology.

These checks intentionally inspect the Compose and image definitions rather
than starting containers.  Runtime behavior belongs to the runtime-node and
end-to-end tests; this seam protects the immutable container boundary those
tests depend on.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
BASE_IMAGE = (
    "ros:jazzy-ros-base@sha256:"
    "2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4"
)
RUN_ID_DEFAULT = "00000000-0000-4000-8000-000000000001"
RUN_DIRECTORY_DEFAULT = f"/tmp/drone-sim/runs/{RUN_ID_DEFAULT}"
CONFIG_PATH_DEFAULT = f"{RUN_DIRECTORY_DEFAULT}/configuration/run.json"
PHASE2_SERVICES = (
    ("orchestration-runtime", "orchestration"),
    ("artifacts-runtime", "artifacts"),
    ("synthetic-companion", "companion"),
    ("synthetic-ardupilot-sitl", "ardupilot_sitl"),
    ("synthetic-gazebo", "gazebo"),
    ("synthetic-electromagnet", "electromagnet"),
    ("synthetic-scorekeeper", "scorekeeper"),
)


@pytest.fixture(scope="module")
def compose_document() -> dict:
    environment = os.environ.copy()
    for name in (
        "SIM_RUN_ID",
        "SIM_RUN_DIRECTORY",
        "SIM_CONFIG_PATH",
        "SIM_PHASE2_FAULT",
        "SIM_SYNTHETIC_WALL_DELAY_MS",
    ):
        environment.pop(name, None)
    result = subprocess.run(
        ["docker", "compose", "--profile", "phase2", "config", "--format", "json"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    document = json.loads(result.stdout)
    assert isinstance(document, dict)
    return document


@pytest.fixture(scope="module")
def compose_text() -> str:
    return (ROOT / "compose.yaml").read_text(encoding="utf-8")


def _phase2(compose_document: dict) -> dict[str, dict]:
    services = compose_document["services"]
    return {name: services[name] for name, _module in PHASE2_SERVICES}


def test_container_contract_compose_has_frozen_service_order(
    compose_document: dict, compose_text: str
) -> None:
    service_text = compose_text.split("services:\n", 1)[1]
    service_order = re.findall(r"^  ([a-z][a-z0-9-]+):$", service_text, flags=re.MULTILINE)
    assert service_order == [
        "foundation",
        *(name for name, _module in PHASE2_SERVICES),
    ]

    services = compose_document["services"]
    for name, module in PHASE2_SERVICES:
        assert services[name]["environment"]["SIM_MODULE"] == module


def test_container_contract_phase2_services_are_profile_scoped_and_unprivileged(
    compose_document: dict,
) -> None:
    # No explicit network means Compose's ordinary default network is the only
    # network available to the profile.  An explicit custom network can make
    # a seemingly harmless service escape that boundary.
    assert set(compose_document.get("networks", {})) == {"default"}

    for service in _phase2(compose_document).values():
        assert service["profiles"] == ["phase2"]
        assert service["init"] is True
        assert service["restart"] == "no"
        assert "network_mode" not in service
        assert "privileged" not in service
        assert "cap_add" not in service
        assert "cap_drop" not in service
        volumes = service.get("volumes", [])
        assert all(
            "/var/run/docker.sock" not in str(volume)
            for volume in volumes
        )


def test_container_contract_services_share_safe_absolute_run_configuration_bindings(
    compose_document: dict,
    compose_text: str,
) -> None:
    phase2 = _phase2(compose_document)
    first_environment = phase2[PHASE2_SERVICES[0][0]]["environment"]
    for name, _module in PHASE2_SERVICES:
        environment = phase2[name]["environment"]
        assert environment["SIM_RUN_ID"] == RUN_ID_DEFAULT
        assert environment["SIM_RUN_DIRECTORY"] == first_environment["SIM_RUN_DIRECTORY"]
        assert environment["SIM_CONFIG_PATH"] == first_environment["SIM_CONFIG_PATH"]
        assert environment["SIM_RUN_DIRECTORY"] == RUN_DIRECTORY_DEFAULT
        assert environment["SIM_CONFIG_PATH"] == CONFIG_PATH_DEFAULT
        assert environment["SIM_PHASE2_FAULT"] == ""
        assert environment["SIM_SYNTHETIC_WALL_DELAY_MS"] == "0"

        run_directory = environment["SIM_RUN_DIRECTORY"]
        config_path = environment["SIM_CONFIG_PATH"]
        assert run_directory.startswith("/")
        assert config_path.startswith("/")
        assert config_path.endswith("/configuration/run.json")
        assert "${SIM_RUN_ID:-" in compose_text
        assert "${SIM_RUN_DIRECTORY:-/" in compose_text
        assert "${SIM_CONFIG_PATH:-/" in compose_text
        assert "${SIM_PHASE2_FAULT:-" in compose_text
        assert "${SIM_SYNTHETIC_WALL_DELAY_MS:-" in compose_text

        bindings = phase2[name]["volumes"]
        assert all(isinstance(binding, dict) for binding in bindings)
        run_bindings = [binding for binding in bindings if binding["target"] == run_directory]
        config_bindings = [binding for binding in bindings if binding["target"] == config_path]
        assert len(run_bindings) == 1
        assert len(config_bindings) == 1
        assert run_bindings[0]["type"] == "bind"
        assert run_bindings[0]["source"] == run_directory
        assert run_bindings[0]["target"] == run_directory
        assert config_bindings[0] == {
            "type": "bind",
            "source": config_path,
            "target": config_path,
            "read_only": True,
        }
        assert "read_only: false" in compose_text


def test_container_contract_dockerfiles_use_pinned_ros_and_owned_build_contexts() -> None:
    orchestration = (ROOT / "orchestration/Dockerfile").read_text(encoding="utf-8")
    artifacts = (ROOT / "artifacts/Dockerfile").read_text(encoding="utf-8")
    phase2 = (ROOT / "tests/phase2/Dockerfile").read_text(encoding="utf-8")

    assert orchestration.startswith(f"FROM {BASE_IMAGE} AS runtime")
    assert f"FROM {BASE_IMAGE} AS runtime" in artifacts
    assert phase2.startswith(f"FROM {BASE_IMAGE}")
    for dockerfile in (orchestration, artifacts, phase2):
        assert "simulation_interfaces" in dockerfile
        assert "colcon build" in dockerfile
        assert "companion/comp2026" not in dockerfile
        assert "docker.sock" not in dockerfile

    assert "FROM runtime AS test" in orchestration
    assert "COPY conftest.py ./conftest.py" in orchestration
    assert "COPY orchestration/src" in orchestration
    assert "COPY orchestration/pyproject.toml" in orchestration
    assert "COPY config config" in orchestration
    assert 'git -c user.name="Test Fixture"' in orchestration
    assert "COPY artifacts/ffmpeg-packages.lock" in artifacts
    assert "ros-jazzy-rosbag2-storage-mcap" in artifacts
    assert "ffmpeg" in artifacts
    assert "ffprobe" in artifacts
    assert "libx264" in artifacts
    assert "FROM runtime AS test" in artifacts
    assert "COPY conftest.py ./conftest.py" in artifacts
    for dockerfile in (orchestration, artifacts):
        assert "COPY artifacts/pyproject.toml artifacts/pyproject.toml" in dockerfile
        assert "COPY orchestration/pyproject.toml orchestration/pyproject.toml" in dockerfile
        assert "uv sync --frozen --inexact --no-install-workspace" in dockerfile
        assert 'CMD ["uv", "run", "--no-sync", "pytest"' in dockerfile


def test_container_contract_phase2_image_and_entrypoint_are_deterministic() -> None:
    dockerfile = (ROOT / "tests/phase2/Dockerfile").read_text(encoding="utf-8")
    entrypoint = (ROOT / "tests/phase2/entrypoint.sh").read_text(encoding="utf-8")

    assert "COPY tests/phase2/" in dockerfile
    assert "ENTRYPOINT" in dockerfile
    assert "source /opt/ros/jazzy/setup.bash" in entrypoint
    assert "source /ros_ws/install/setup.bash" in entrypoint
    assert "exec" in entrypoint


def test_synthetic_runtime_contract_has_exact_timeline_and_wall_delay_independence() -> None:
    phase2 = ROOT / "tests/phase2"
    sys.path.insert(0, str(phase2))
    try:
        from synthetic_gazebo import SyntheticGazeboModel
    finally:
        sys.path.remove(str(phase2))

    def capture(delay_ms: int):
        events = []
        finished = []
        model = SyntheticGazeboModel(
            RUN_ID_DEFAULT,
            wall_delay_ms=delay_ms,
            publish=lambda kind, value: events.append((kind, value)),
            sleep=lambda _seconds: None,
            source_finished=finished.append,
        )
        assert model.step() is False
        model.accept_run_state(RUN_ID_DEFAULT, "READY")
        assert model.step() is True
        model.accept_run_state(RUN_ID_DEFAULT, "RUNNING")
        while not model.finished and not model.stalled:
            model.step()
            if model.next_frame_id > 0:
                frame_id = model.next_frame_id - 1
                model.accept_pair_ack(
                    RUN_ID_DEFAULT,
                    frame_id,
                    (frame_id + 1) * 50_000_000,
                    "aggregate",
                )
        return events, finished

    immediate, immediate_finished = capture(0)
    delayed, delayed_finished = capture(11)
    assert delayed == immediate
    assert delayed_finished == immediate_finished == [2_000_000_000]
    assert [value for kind, value in immediate if kind == "clock"] == list(
        range(0, 2_000_000_001, 50_000_000)
    )
    expected_stamps = list(range(50_000_000, 2_000_000_001, 50_000_000))
    for stream in ("onboard", "observer"):
        frames = [value for kind, value in immediate if kind == stream]
        assert [item.frame_id for item in frames] == list(range(40))
        assert [item.sim_timestamp_ns for item in frames] == expected_stamps
        assert all(
            item.encoding == "rgb8"
            and item.width == 320
            and item.height == 240
            and len(item.payload) == 320 * 240 * 3
            for item in frames
        )
    ground_truth = [value for kind, value in immediate if kind == "ground_truth"]
    assert [item.sim_timestamp_ns for item in ground_truth] == expected_stamps


def test_phase2_test_image_contains_real_ros_contract_harness() -> None:
    dockerfile = (ROOT / "tests/phase2/Dockerfile").read_text(encoding="utf-8")
    harness = (ROOT / "tests/phase2/ros_contract_harness.py").read_text(encoding="utf-8")
    assert "uv==0.12.1" in dockerfile
    assert "pytest==8.4.2" in dockerfile
    assert "COPY tests/phase2/ros_contract_harness.py" in dockerfile
    assert "synthetic source published before READY" in harness
    assert "node.clocks == list(range(0, 2_000_000_001, 50_000_000))" in harness


def test_camera_pair_ack_is_internal_backpressure_not_bag_inventory() -> None:
    from artifacts._adapters.rosbag import FIXED_TOPICS

    artifact_runtime = (ROOT / "artifacts/src/artifacts/runtime_node.py").read_text()
    gazebo = (ROOT / "tests/phase2/synthetic_gazebo.py").read_text()
    assert "/simulation/camera_pair_ack" not in FIXED_TOPICS
    assert '"/simulation/camera_pair_ack"' in artifact_runtime
    assert 'acknowledgement.stream = "aggregate"' in artifact_runtime
    assert "ReliabilityPolicy.RELIABLE" in artifact_runtime
    assert '"/simulation/camera_pair_ack"' in gazebo


def test_phase2_archival_camera_transport_is_reliable_without_changing_inventory() -> None:
    from artifacts._adapters.rosbag import FIXED_TOPICS

    qos = (ROOT / "config/recording-qos.yaml").read_text(encoding="utf-8")
    artifact_runtime = (ROOT / "artifacts/src/artifacts/runtime_node.py").read_text()
    gazebo = (ROOT / "tests/phase2/synthetic_gazebo.py").read_text()
    camera_section = qos.split("/camera/onboard/image_raw:", 1)[1]

    assert "reliability: reliable" in camera_section
    assert "reliability: best_effort" not in camera_section
    assert "def camera_recorder_qos" in artifact_runtime
    assert "qos_factory=camera_recorder_qos" in artifact_runtime
    assert "frame_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)" in gazebo
    assert len(FIXED_TOPICS) == 10
    assert "/simulation/camera_pair_ack" not in FIXED_TOPICS


def test_aggregate_runtime_destroys_its_injected_ros_node_only_once() -> None:
    artifact_runtime = (ROOT / "artifacts/src/artifacts/runtime_node.py").read_text()

    assert "video_node.destroy_node()" not in artifact_runtime
    assert artifact_runtime.count("node.destroy_node()") == 1
