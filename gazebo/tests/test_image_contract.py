"""Host-visible contracts for the reproducible Gazebo Harmonic image."""

from __future__ import annotations

from pathlib import Path
import re
import tomllib


ROOT = Path(__file__).resolve().parents[2]
DOCKERFILE = ROOT / "gazebo/Dockerfile"
PACKAGE_LOCK = ROOT / "gazebo/harmonic-packages.lock"
PROJECT = ROOT / "gazebo/pyproject.toml"
IMAGE_TEST = ROOT / "gazebo/tests/test_gazebo_image"
OGRE_WORKER_PATCH = ROOT / "gazebo/patches/gz-rendering8-inline-workers.patch"

BASE = (
    "ros:jazzy-ros-base@"
    "sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4"
)
DIRECT_PACKAGES = {
    "ros-jazzy-ros-gz": "1.0.22-1noble.20260616.074726",
    "ros-jazzy-ros-gz-bridge": "1.0.22-1noble.20260615.142443",
    "ros-jazzy-ros-gz-image": "1.0.22-1noble.20260615.145009",
    "ros-jazzy-ros-gz-interfaces": "1.0.22-1noble.20260615.112415",
    "ros-jazzy-ros-gz-sim": "1.0.22-1noble.20260615.173223",
    "ros-jazzy-gz-sim-vendor": "0.0.10-1noble.20260604.111001",
    "ros-jazzy-sdformat-vendor": "0.0.11-1noble.20260604.104102",
    "zstd": "1.5.5+dfsg2-2build1.1",
}
UPGRADED_BASE_PACKAGES = {
    "bsdutils": "1:2.39.3-9ubuntu6.6",
    "bzip2": "1.0.8-5.1ubuntu0.1",
    "libblkid1": "2.39.3-9ubuntu6.6",
    "libbz2-1.0": "1.0.8-5.1ubuntu0.1",
    "libmount1": "2.39.3-9ubuntu6.6",
    "libsmartcols1": "2.39.3-9ubuntu6.6",
    "libuuid1": "2.39.3-9ubuntu6.6",
    "mount": "2.39.3-9ubuntu6.6",
    "util-linux": "2.39.3-9ubuntu6.6",
    "uuid-dev": "2.39.3-9ubuntu6.6",
    "zlib1g-dev": "1:1.3.dfsg-3.1ubuntu2.2",
    "zlib1g": "1:1.3.dfsg-3.1ubuntu2.2",
}


def _dockerfile() -> str:
    assert DOCKERFILE.is_file(), "the pinned Gazebo Dockerfile is missing"
    return DOCKERFILE.read_text(encoding="utf-8")


def _locked_packages() -> list[str]:
    assert PACKAGE_LOCK.is_file(), "the complete Harmonic apt delta lock is missing"
    return PACKAGE_LOCK.read_text(encoding="utf-8").splitlines()


def test_runtime_inherits_only_the_approved_immutable_ros_base():
    """A mutable or legacy base image would make runtime bytes float."""
    from_lines = [line for line in _dockerfile().splitlines() if line.startswith("FROM ")]

    assert from_lines[0] == f"FROM {BASE} AS runtime"
    assert all("ardupilot" not in line.lower() for line in from_lines)
    assert all("gazebo:" not in line.lower() for line in from_lines)


def test_all_direct_gazebo_packages_are_installed_at_approved_versions():
    """An unversioned direct dependency could change the Gazebo runtime ABI."""
    dockerfile = _dockerfile()

    for package, version in DIRECT_PACKAGES.items():
        assert f"{package}={version}" in dockerfile
    assert not re.search(r"apt-get install[^\n]*(?:gz-harmonic|gazebo)", dockerfile)


def test_complete_added_package_delta_is_compared_with_the_lock():
    """Pinning only direct packages would leave transitive runtime bytes mutable."""
    dockerfile = _dockerfile()
    locked = _locked_packages()

    assert locked == sorted(locked)
    assert len(locked) == len(set(locked))
    assert locked
    assert all(re.fullmatch(r"[a-z0-9][a-z0-9+.-]*=[^\s=]+", line) for line in locked)
    assert "COPY gazebo/harmonic-packages.lock" in dockerfile
    assert dockerfile.count("dpkg-query -W") >= 2
    assert "comm -13" in dockerfile
    assert "diff --unified" in dockerfile


def test_upgraded_base_packages_are_part_of_the_exact_runtime_lock():
    """Security upgrades from the pinned base are runtime changes, not additions."""
    locked = set(_locked_packages())

    assert {
        f"{package}={version}" for package, version in UPGRADED_BASE_PACKAGES.items()
    } <= locked


def test_runtime_copies_local_resources_and_only_fetches_pinned_sources():
    """Runtime resources stay local while both build fetches are immutable."""
    dockerfile = _dockerfile().lower()

    assert "copy artifacts orchestration gazebo ros_ws/src/simulation_interfaces /opt/drone_sim/source/" in dockerfile
    assert "cp -a /opt/drone_sim/source/resources /opt/drone_sim/gazebo/resources" in dockerfile
    assert "gz_sim_resource_path=/opt/drone_sim/gazebo/resources" in dockerfile
    assert "http://" not in dockerfile
    assert dockerfile.count("https://") == 2
    assert "https://github.com/ardupilot/ardupilot_gazebo.git" in dockerfile
    assert "fetch --depth=1 origin \"${ardupilot_gazebo_commit}\"" in dockerfile
    assert "fetch --depth=1 origin \"${gz_rendering_commit}\"" in dockerfile
    assert "git -c /tmp/ardupilot_gazebo apply" in dockerfile
    assert "0001-paused-initial-json.patch" in dockerfile
    assert "display=" not in dockerfile


def test_gpu_image_backports_the_pinned_ogre_worker_fix() -> None:
    dockerfile = _dockerfile()
    assert OGRE_WORKER_PATCH.is_file()
    assert "ARG GZ_RENDERING_COMMIT=f1249da6e07f0d6b7adaba28109fa7af727fd322" in dockerfile
    assert "https://github.com/gazebosim/gz-rendering.git" in dockerfile
    assert "git -C /tmp/gz-rendering apply" in dockerfile
    assert "gz-rendering8-inline-workers.patch" in dockerfile
    assert "cmake --build /tmp/gz-rendering/build --target gz-rendering8-ogre2" in dockerfile
    assert "GZ_RENDERING_OGRE2_WORKER_THREADS" in IMAGE_TEST.read_text()


def test_runtime_installs_the_competition_bridge_configuration():
    """The competition world must not silently reuse a smaller bridge surface."""
    dockerfile = _dockerfile()

    assert (
        "cp /opt/drone_sim/source/config/bridge-competition.yaml "
        "/etc/drone_sim/gazebo-bridge-competition.yaml"
    ) in dockerfile


def test_test_target_proves_harmonic_bridge_and_pinned_plugin_offline():
    """The image must self-identify Sim 8 and its pinned flight plugin offline."""
    dockerfile = _dockerfile()
    assert IMAGE_TEST.is_file(), "the offline image test executable is missing"
    image_test = IMAGE_TEST.read_text(encoding="utf-8")

    assert "FROM runtime AS test" in dockerfile
    assert "/opt/drone_sim/image-tests/test_gazebo_image" in dockerfile
    assert "RUN <<" not in dockerfile
    assert "gz sim --versions" in image_test
    assert "ros2 pkg prefix ros_gz_bridge" in image_test
    assert "ArduPilotPlugin" in image_test
    assert "DRONE_SIM_REAL_PLUGIN_TEST=1" in image_test
    assert "test_plugin_udp.py" in image_test
    assert 'CMD ["/opt/drone_sim/image-tests/test_gazebo_image"]' in dockerfile


def test_entrypoint_sources_the_cmake_installed_interface_package():
    """A direct CMake install has a package setup file, not a colcon workspace one."""
    dockerfile = _dockerfile()

    assert (
        "source /opt/drone_sim/ros_ws/install/simulation_interfaces/"
        "share/simulation_interfaces/local_setup.bash"
    ) in dockerfile
    assert (
        "export PYTHONPATH=/opt/drone_sim/ros_ws/install/simulation_interfaces/"
        "lib/python3.12/site-packages:${PYTHONPATH}"
    ) in dockerfile
    assert (
        "export LD_LIBRARY_PATH=/opt/drone_sim/ros_ws/install/"
        "simulation_interfaces/lib:${LD_LIBRARY_PATH}"
    ) in dockerfile


def test_gazebo_project_is_installable_and_reserves_the_runtime_entry_point():
    """An image-only package would leave Task 5 without an installable runtime seam."""
    assert PROJECT.is_file(), "the Gazebo Python project is missing"
    project = tomllib.loads(PROJECT.read_text(encoding="utf-8"))

    assert project["project"] == {
        "name": "drone-sim-gazebo",
        "version": "0.1.0",
        "requires-python": ">=3.12",
        "dependencies": ["drone-sim-artifacts", "PyYAML>=6.0,<7"],
        "scripts": {
            "drone-sim-gazebo-runtime": "drone_sim_gazebo.runtime.runtime_node:main"
        },
    }
    assert project["tool"]["uv"]["sources"] == {
        "drone-sim-artifacts": {"workspace": True}
    }
    assert project["build-system"]["build-backend"] == "setuptools.build_meta"


def test_root_workspace_locks_the_gazebo_project_as_a_local_dependency():
    """Leaving Gazebo outside the workspace would let its local dependency drift."""
    root_project = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))

    assert "drone-sim-gazebo" in root_project["project"]["dependencies"]
    assert "gazebo" in root_project["tool"]["uv"]["workspace"]["members"]
    assert root_project["tool"]["uv"]["sources"]["drone-sim-gazebo"] == {
        "workspace": True
    }
