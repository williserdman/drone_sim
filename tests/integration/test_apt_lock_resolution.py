"""Boundaries for an explicit Ubuntu-only refresh of exact apt locks."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = spec_from_file_location("apt_lock_resolver", ROOT / "scripts/resolve-amd64-locks.py")
assert SPEC is not None and SPEC.loader is not None
resolver = module_from_spec(SPEC)
sys.modules[SPEC.name] = resolver
SPEC.loader.exec_module(resolver)


def policy(name, candidate, versions):
    lines = [f"{name}:", "  Installed: (none)", f"  Candidate: {candidate}", "  Version table:"]
    for version, url in versions:
        lines.extend([f"     {version} 500", f"        500 {url} noble/main arm64 Packages"])
    return "\n".join(lines) + "\n"


UBUNTU = "http://ports.ubuntu.com/ubuntu-ports"
ROS = "http://packages.ros.org/ros2/ubuntu"


def test_refresh_preserves_available_pins_and_changes_only_unavailable_ubuntu_rows():
    inventory = resolver._parse_policy(
        policy("libz", "2", [("2", UBUNTU), ("1", UBUNTU)])
        + policy("libold", "4", [("4", UBUNTU)])
        + policy("ros-jazzy-example", "9", [("9", ROS), ("8", ROS)])
    )
    roots, changes = resolver._refresh_roots(
        ("libz=1", "libold=3", "ros-jazzy-example=8"), inventory, {}
    )
    assert roots == ("libz=1", "libold=4", "ros-jazzy-example=8")
    assert changes == [{"package": "libold", "before": "3", "after": "4", "origin": "Ubuntu"}]


@pytest.mark.parametrize("package,url", [("ros-jazzy-example", ROS), ("python3-rospkg-modules", ROS), ("libold", "https://example.com/ubuntu")])
def test_refresh_rejects_missing_ros_and_unknown_origin(package, url):
    inventory = resolver._parse_policy(policy(package, "2", [("2", url)]))
    with pytest.raises(RuntimeError):
        resolver._refresh_roots((f"{package}=1",), inventory, {})


def test_inherited_installed_version_needs_no_new_download():
    inventory = resolver._parse_policy(policy("libold", "2", [("2", UBUNTU)]))
    roots, changes = resolver._refresh_roots(("libold=1",), inventory, {("libold", "arm64"): "1"})
    assert roots == ("libold=1",)
    assert changes == []


@pytest.mark.parametrize("architecture,url", [("arm64", "ports.ubuntu.com/ubuntu-ports"), ("amd64", "archive.ubuntu.com/ubuntu")])
def test_sources_select_architecture_without_emulation(tmp_path, architecture, url):
    resolver._write_sources(tmp_path, architecture)
    sources = (tmp_path / "sources.list").read_text()
    assert f"arch={architecture}" in sources
    assert url in sources
    assert "trusted=yes" not in sources


def test_parse_plan_normalizes_both_architectures_and_rejects_removal():
    assert resolver._parse_plan("Inst foo:arm64 (1 Ubuntu:24.04/noble [arm64])\n") == [("foo", "1")]
    with pytest.raises(RuntimeError, match="removal"):
        resolver._parse_plan("Remv foo [1]\n")


def test_artifacts_direct_pins_come_from_matching_dockerfile_architecture():
    arm = resolver._artifacts_direct(ROOT, "arm64")
    amd = resolver._artifacts_direct(ROOT, "amd64")
    assert "ros-jazzy-rosbag2=0.26.11-1noble.20261006.034453" in arm
    assert "ros-jazzy-rosbag2=0.26.11-1noble.20261006.034645" in amd
    assert "python3.12-venv=3.12.3-1ubuntu0.17" in arm
    assert len(arm) == len(amd) == 8


def test_complete_delta_includes_new_base_dependencies_and_child_uses_parent_state(tmp_path):
    before = tmp_path / "base.status"
    before.write_text("")
    parent = tmp_path / "parent.status"
    child = tmp_path / "child.status"

    class Runner:
        run_directory = tmp_path
        architecture = "arm64"

        def __init__(self, package, dependency):
            self.package = package
            self.dependency = dependency

        def run(self, action, *, status, roots=None):
            if action == "simulate":
                installed = resolver._installed(status)
                return "".join(
                    f"Inst {package} (1 Ubuntu:24.04/noble [arm64])\n"
                    for package in (self.package, self.dependency)
                    if (package, "arm64") not in installed
                )
            if action == "show":
                return "\n\n".join(
                    f"Package: {row.split('=')[0]}\nVersion: 1\nArchitecture: arm64\n"
                    f"Filename: pool/{row.split('=')[0]}.deb\nSize: 1\nSHA256: {'a' * 64}"
                    for row in roots.read_text().splitlines()
                )
            raise AssertionError(action)

    parent_lock = tmp_path / "parent.lock"
    resolver._resolve_phase(
        Runner("foo", "libsystemd0"), phase="parent", before=before, after=parent,
        roots=("foo=1",), candidate_path=parent_lock, verify_downloads=False,
    )
    assert parent_lock.read_text() == "foo=1\nlibsystemd0=1\n"
    child_lock = tmp_path / "child.lock"
    resolver._resolve_phase(
        Runner("viewer", "libsystemd0"), phase="child", before=parent, after=child,
        roots=("viewer=1",), candidate_path=child_lock, verify_downloads=False,
    )
    assert child_lock.read_text() == "viewer=1\n"
    assert resolver._installed(child)[("libsystemd0", "arm64")] == "1"


def test_policy_does_not_treat_an_installed_status_record_as_downloadable():
    text = "libold:\n  Installed: 1\n  Candidate: 2\n  Version table:\n     2 500\n        500 http://ports.ubuntu.com/ubuntu-ports noble/main arm64 Packages\n *** 1 100\n        100 /audit/status\n"
    inventory = resolver._parse_policy(text)
    assert inventory["libold"].versions == {"2": {"Ubuntu"}}


def test_downgrade_rejection_prevents_simulation_and_candidate_publication(tmp_path):
    before = tmp_path / "base.status"
    before.write_text("")
    candidate = tmp_path / "candidate.lock"
    calls = []

    class Runner:
        run_directory = tmp_path
        architecture = "arm64"

        def run(self, action, *, status, roots=None):
            calls.append(action)
            if action == "policy":
                return policy("libold", "1", [("1", UBUNTU)])
            if action == "check-upgrades":
                assert roots.read_text() == "libold 2 1\n"
                raise RuntimeError("replacement is not an upgrade")
            pytest.fail(f"must reject before {action}")

    with pytest.raises(RuntimeError, match="not an upgrade"):
        resolver._resolve_phase(
            Runner(), phase="parent", before=before, after=tmp_path / "after.status",
            roots=("libold=2",), candidate_path=candidate, verify_downloads=False,
            refresh_unavailable_ubuntu=True,
        )
    assert calls == ["policy", "check-upgrades"]
    assert not candidate.exists()


def test_resume_rejects_completed_run_without_changing_published_evidence(tmp_path):
    runs = tmp_path / "runs"
    completed = runs / "finished"
    completed.mkdir(parents=True)
    summary = completed / "resolution-summary.json"
    candidate = completed / "harmonic-packages.arm64.candidate.lock"
    pointer = tmp_path / "latest-run-path.txt"
    summary.write_bytes(b'{"completed": true}\n')
    candidate.write_bytes(b'libexample=1\n')
    pointer.write_text(str(completed) + "\n")
    before = {path: path.read_bytes() for path in (summary, candidate, pointer)}

    with pytest.raises(RuntimeError, match="completed"):
        resolver._resume_base(completed, runs, "arm64")

    assert {path: path.read_bytes() for path in before} == before
    assert sorted(path.name for path in completed.iterdir()) == sorted(
        [summary.name, candidate.name]
    )


@pytest.mark.parametrize("verify_downloads", [False, True])
def test_solver_downgrade_is_rejected_even_when_available_pin_was_preserved(tmp_path, verify_downloads):
    before = tmp_path / "base.status"
    before.write_text("Package: libold\nStatus: install ok installed\nVersion: 2\nArchitecture: arm64\n")
    candidate = tmp_path / "candidate.lock"
    calls = []

    class Runner:
        run_directory = tmp_path
        architecture = "arm64"

        def run(self, action, *, status, roots=None):
            calls.append(action)
            if action == "simulate":
                return "Inst libold [2] (1 Ubuntu:24.04/noble [arm64])\n"
            if action == "check-upgrades":
                assert roots.read_text() == "libold 2 1\n"
                raise RuntimeError("replacement is not an upgrade")
            pytest.fail(f"must reject solver downgrade before {action}")

    with pytest.raises(RuntimeError, match="not an upgrade"):
        resolver._resolve_phase(
            Runner(), phase="child", before=before, after=tmp_path / "after.status",
            roots=("libold=1",), candidate_path=candidate,
            verify_downloads=verify_downloads,
        )
    assert calls == ["simulate", "check-upgrades"]
    assert not candidate.exists()
