"""Contracts for the native Colima deployment and read-only Gazebo desktop."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
LOCAL_COMPOSE = ROOT / "compose.local.yaml"
LOCAL_BUILD = ROOT / "scripts/local-build.sh"
VIEWER_ROOT = ROOT / "gazebo/local-viewer"

RUNTIME_SERVICES = {
    "orchestration-runtime",
    "artifacts-runtime",
    "companion-runtime",
    "ardupilot-sitl",
    "gazebo-runtime",
    "electromagnet-runtime",
    "scorekeeper-runtime",
}


def _local_compose() -> dict:
    assert LOCAL_COMPOSE.is_file(), "the local Compose overlay is missing"
    return yaml.safe_load(LOCAL_COMPOSE.read_text(encoding="utf-8"))


def test_local_overlay_publishes_only_the_three_loopback_operator_endpoints() -> None:
    services = _local_compose()["services"]

    assert services["ardupilot-sitl"]["ports"] == ["127.0.0.1:5762:5762/tcp"]
    assert services["gazebo-runtime"]["ports"] == ["127.0.0.1:8081:8081/tcp"]
    assert services["gazebo-viewer"]["ports"] == ["127.0.0.1:6080:6080/tcp"]
    assert all(
        not service.get("ports")
        for name, service in services.items()
        if name not in {"ardupilot-sitl", "gazebo-runtime", "gazebo-viewer"}
    )


def test_local_overlay_uses_linux_volume_for_atomic_media_spool() -> None:
    compose = _local_compose()
    artifacts = compose["services"]["artifacts-runtime"]

    assert artifacts["environment"]["SIM_MEDIA_SPOOL_DIRECTORY"] == (
        "/var/lib/drone-sim/media"
    )
    assert artifacts["volumes"] == ["artifacts-media:/var/lib/drone-sim/media"]
    assert compose["volumes"] == {"artifacts-media": None}


def test_viewer_joins_the_run_partition_network_and_immutable_assets() -> None:
    viewer = _local_compose()["services"]["gazebo-viewer"]

    assert viewer["profiles"] == ["local"]
    assert viewer["image"] == "drone-sim-gazebo-viewer:local"
    assert viewer["build"] == {
        "context": ".",
        "dockerfile": "gazebo/local-viewer/Dockerfile",
    }
    assert viewer["depends_on"] == {
        "gazebo-runtime": {"condition": "service_started", "required": True}
    }
    assert viewer["environment"]["SIM_RUN_ID"] == (
        "${SIM_RUN_ID:-00000000-0000-4000-8000-000000000001}"
    )
    assert viewer["environment"]["GZ_SIM_RESOURCE_PATH"] == (
        "/opt/drone_sim/gazebo/resources"
    )
    assert "network_mode" not in viewer
    assert "networks" not in viewer

    entrypoint = (VIEWER_ROOT / "entrypoint.sh").read_text(encoding="utf-8")
    assert 'GZ_PARTITION="drone_sim_${SIM_RUN_ID//-/_}"' in entrypoint
    assert "export GZ_PARTITION" in entrypoint


def test_viewer_gui_has_inspection_without_ungoverned_world_mutations() -> None:
    config = (VIEWER_ROOT / "gui.config").read_text(encoding="utf-8")
    plugins = set(re.findall(r'<plugin filename="([^"]+)"', config))

    first_plugin = re.search(r'<plugin filename="', config)
    assert first_plugin is not None
    assert config.index("</window>") < first_plugin.start()
    assert plugins == {
        "MinimalScene",
        "GzSceneManager",
        "InteractiveViewControl",
        "CameraTracking",
        "SelectEntities",
        "ReadOnlyInspector",
    }
    for forbidden in (
        "WorldControl",
        "WorldStats",
        "Spawn",
        "TransformControl",
        "AlignTool",
        "EntityContextMenuPlugin",
        "ComponentInspector",
        "EntityTree",
    ):
        assert forbidden not in config
    assert '<drawer visible="false"/>' in config
    assert '<plugins visible="false" from_paths="false"/>' in config

    inspector = (VIEWER_ROOT / "read_only_inspector.cc").read_text(
        encoding="utf-8"
    )
    qml = (VIEWER_ROOT / "ReadOnlyInspector.qml").read_text(encoding="utf-8")
    assert "EntitiesSelected" in inspector
    assert "components::Pose" in inspector
    assert "components::Name" in inspector
    assert "Node.Request" not in inspector
    assert "OnInsert" not in inspector
    assert "readOnly: true" in qml


def test_viewer_desktop_is_reconnectable_and_uses_matching_gazebo_runtime() -> None:
    dockerfile = (VIEWER_ROOT / "Dockerfile").read_text(encoding="utf-8")
    entrypoint = (VIEWER_ROOT / "entrypoint.sh").read_text(encoding="utf-8")

    assert dockerfile.startswith(
        "FROM drone-sim-gazebo-runtime:phase3 AS viewer\n"
    )
    assert "viewer-packages.${TARGETARCH}.lock" in dockerfile
    assert (
        "COPY gazebo/local-viewer/operator.html /usr/share/novnc/operator.html"
        in dockerfile
    )
    assert "-forever" in entrypoint
    assert "-shared" in entrypoint
    assert "--web=/usr/share/novnc" in entrypoint
    assert "gz sim -g" in entrypoint
    assert '--gui-config "${GZ_GUI_CONFIG_PATH}"' in entrypoint
    assert 'grep --quiet \'"Gazebo Sim"\'' in entrypoint
    assert entrypoint.index("gz sim -g") < entrypoint.index("x11vnc")
    assert entrypoint.index("x11vnc") < entrypoint.index("websockify")

    viewer_lock = (VIEWER_ROOT / "viewer-packages.arm64.lock").read_text(
        encoding="utf-8"
    ).splitlines()
    assert viewer_lock == sorted(set(viewer_lock))
    for package in ("xvfb", "openbox", "x11vnc", "novnc", "websockify"):
        assert any(item.startswith(f"{package}=") for item in viewer_lock)

    amd_lock = VIEWER_ROOT / "viewer-packages.amd64.lock"
    assert amd_lock.read_bytes() == (
        VIEWER_ROOT / "viewer-packages.arm64.lock"
    ).read_bytes()


def test_local_build_selects_host_native_platform_and_all_runtime_images_deterministically() -> None:
    from orchestration.local_build import IMAGE_REFS, RUNTIME_SERVICES as BUILD_SERVICES

    script = LOCAL_BUILD.read_text(encoding="utf-8")

    assert set(BUILD_SERVICES) == RUNTIME_SERVICES
    assert len(IMAGE_REFS) == 8
    assert IMAGE_REFS[-1] == "drone-sim-gazebo-viewer:local"
    assert "python -m orchestration.local_build" in script


def test_metadata_verified_lock_candidates_are_applied_exactly() -> None:
    expected = {
        ROOT / "artifacts/ffmpeg-packages.lock": "2048c831d649fde6dd167f20709706cddf6c93bb1d09dcbb7ccafa452b9de71d",
        ROOT / "gazebo/harmonic-packages.lock": "36c26dc16b8a60c8ddd011c0f4916b2ca53301c2d60663a174f9bef9d15d854a",
        ROOT / "gazebo/local-viewer/viewer-packages.amd64.lock": "fb84a33dc95ca2a2cbb60f46b3173428926ee7b1aa4811cbb5e2fa72dd0fd6ec",
        ROOT / "gazebo/harmonic-packages.arm64.lock": "b6837cbb39a03ba0ab4244233df9aef829ca0f9c88d5a11830985b289171d7ba",
        ROOT / "gazebo/local-viewer/viewer-packages.arm64.lock": "fb84a33dc95ca2a2cbb60f46b3173428926ee7b1aa4811cbb5e2fa72dd0fd6ec",
        ROOT / "ardupilot_sitl/build-packages.arm64.lock": "379fcfd0604fa90dea439196826c362a72c59dd3c6aee0135bed39f1f06c0f7c",
    }

    for path, digest in expected.items():
        assert hashlib.sha256(path.read_bytes()).hexdigest() == digest

    artifacts = (ROOT / "artifacts/Dockerfile").read_text(encoding="utf-8")
    gazebo = (ROOT / "gazebo/Dockerfile").read_text(encoding="utf-8")
    assert "ros-jazzy-rosbag2=0.26.11-1noble.20261006.034645" in artifacts
    assert (
        "ros-jazzy-rosbag2-storage-mcap=0.26.11-1noble.20260911.143733"
        in artifacts
    )
    assert (
        "ROSBAG2_PACKAGE_ARM64="
        "ros-jazzy-rosbag2=0.26.11-1noble.20261006.034453"
        in artifacts
    )
    assert (
        "ROSBAG2_STORAGE_MCAP_PACKAGE_ARM64="
        "ros-jazzy-rosbag2-storage-mcap=0.26.11-1noble.20260911.095159"
        in artifacts
    )
    assert 'case "${TARGETARCH}" in' in artifacts
    assert "DRONE_SIM_APT_PACKAGE_LOCK_FILE=" in artifacts
    for package in (
        "ros-jazzy-ros-gz=1.0.24-1noble.20261006.041805",
        "ros-jazzy-ros-gz-bridge=1.0.24-1noble.20260920.234644",
        "ros-jazzy-ros-gz-image=1.0.24-1noble.20260921.023911",
        "ros-jazzy-ros-gz-interfaces=1.0.24-1noble.20260911.110630",
        "ros-jazzy-ros-gz-sim=1.0.24-1noble.20261006.035056",
        "ros-jazzy-gz-sim-vendor=0.0.13-1noble.20260905.073557",
        "ros-jazzy-sdformat-vendor=0.0.14-1noble.20260905.033543",
    ):
        assert package in gazebo


def test_amd64_lock_resolution_evidence_and_resolver_are_retained() -> None:
    summary = json.loads(
        (ROOT / "docs/amd64-lock-resolution-summary.json").read_text(encoding="utf-8")
    )
    phases = {item["phase"]: item for item in summary["phases"]}

    assert summary["apt_architecture"] == "amd64"
    assert summary["native_amd64_acceptance"] is False
    assert summary["docker_inventory_unchanged"] is True
    assert len(summary["indexes"]) == 22
    assert phases["artifacts-ffmpeg"]["candidate_sha256"] == (
        "2048c831d649fde6dd167f20709706cddf6c93bb1d09dcbb7ccafa452b9de71d"
    )
    assert phases["gazebo-harmonic"]["candidate_sha256"] == (
        "6bb600d901816d3733a0e5e46eca413edc715aff93a6ede40c4fbcd5849c3192"
    )
    assert phases["viewer"]["candidate_sha256"] == (
        "d29308041de14bb20d97208c0a647de1277655139ab292beadf69a1b327e3cbb"
    )

    help_result = subprocess.run(
        [sys.executable, str(ROOT / "scripts/resolve-amd64-locks.py"), "--help"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert help_result.returncode == 0
    assert "--skip-downloads" in help_result.stdout


def test_local_build_has_a_read_only_reuse_check_and_ignored_atomic_record() -> None:
    script = LOCAL_BUILD.read_text(encoding="utf-8")
    ignore = (ROOT / ".gitignore").read_text(encoding="utf-8").splitlines()

    assert ".local-runtime/" in ignore
    assert 'exec uv run --locked python -m orchestration.local_build "$@"' in script
    assert "DRONE_SIM_WORKSTATION_LOCK_HELD" not in script
    assert "fcntl" not in script


def test_gazebo_runtime_chooses_a_complete_lock_for_the_target_architecture() -> None:
    dockerfile = (ROOT / "gazebo/Dockerfile").read_text(encoding="utf-8")
    arm_lock = ROOT / "gazebo/harmonic-packages.arm64.lock"

    assert "ARG TARGETARCH" in dockerfile
    assert arm_lock.is_file()
    assert "harmonic-packages.${TARGETARCH}.lock" in dockerfile
    assert "sed -E 's/:(amd64|arm64)=/=/g'" in dockerfile
    locked = arm_lock.read_text(encoding="utf-8").splitlines()
    assert locked == sorted(set(locked))
    assert locked
    assert all(re.fullmatch(r"[a-z0-9][a-z0-9+.-]*=[^\s=]+", item) for item in locked)
    assert any(item.startswith("python3.12-venv=") for item in locked)
    assert any(item.startswith("python3-wheel=") for item in locked)


def test_artifacts_runtime_chooses_an_exact_ffmpeg_lock_for_target_architecture() -> None:
    dockerfile = (ROOT / "artifacts/Dockerfile").read_text(encoding="utf-8")
    arm_lock = ROOT / "artifacts/ffmpeg-packages.arm64.lock"

    assert "ARG TARGETARCH" in dockerfile
    assert arm_lock.is_file()
    assert "ffmpeg-packages.${TARGETARCH}.lock" in dockerfile
    assert "sed -E 's/:(amd64|arm64)=/=/g'" in dockerfile
    locked = arm_lock.read_text(encoding="utf-8").splitlines()
    assert locked == sorted(set(locked))
    assert locked
    assert all(re.fullmatch(r"[a-z0-9][a-z0-9+.-]*=[^\s=]+", item) for item in locked)
    for package in ("ffmpeg", "libavcodec60", "libx264-164"):
        assert any(item.startswith(f"{package}=") for item in locked)


def test_ardupilot_uses_a_native_locked_arm64_base_without_changing_amd64() -> None:
    dockerfile = (ROOT / "ardupilot_sitl/Dockerfile").read_text(encoding="utf-8")
    arm_lock = ROOT / "ardupilot_sitl/build-packages.arm64.lock"

    assert "ARG TARGETARCH" in dockerfile
    assert (
        "FROM ardupilot/ardupilot-dev-base@"
        "sha256:576cd622957308469d6e72528befe5de5862457b264e61d60aaa4b8f29de85b6 "
        "AS upstream-amd64"
    ) in dockerfile
    assert (
        "FROM ros:jazzy-ros-base@"
        "sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4 "
        "AS upstream-arm64"
    ) in dockerfile
    assert "FROM upstream-${TARGETARCH} AS source" in dockerfile
    assert arm_lock.is_file()
    assert "build-packages.arm64.lock" in dockerfile
    locked = arm_lock.read_text(encoding="utf-8").splitlines()
    assert locked == sorted(set(locked))
    assert locked
    assert all(re.fullmatch(r"[a-z0-9][a-z0-9+.-]*=[^\s=]+", item) for item in locked)
