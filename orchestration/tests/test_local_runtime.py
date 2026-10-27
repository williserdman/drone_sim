from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from artifacts.qgc_operator import MavlinkIdentity
from orchestration.host_config import HostFacts
from orchestration.host_integration import HostIssue, LaunchResult, QgcInstallation
from orchestration.local_runtime import LocalRuntime


class FakeHost:
    def __init__(self, state_directory: Path) -> None:
        self.facts = HostFacts("linux", "amd64", "linux/amd64", "docker")
        self.state_directory = state_directory
        self.installation = QgcInstallation(
            "/usr/bin/QGroundControl",
            "/home/operator/.config/QGroundControl/QGroundControl.ini",
            "/home/operator/QGroundControl",
            "5.0.8",
            MavlinkIdentity(255, 190),
        )
        self.opened = []

    def preflight(self, _paths):
        return ()

    def inspect_qgc(self) -> QgcInstallation:
        return self.installation

    def open_qgc(self, installation: QgcInstallation) -> LaunchResult:
        self.opened.append(installation)
        return LaunchResult(True, "focused")


def test_local_runtime_uses_injected_host_for_qgc_launch(tmp_path: Path) -> None:
    host = FakeHost(tmp_path / "state")
    runtime = LocalRuntime(tmp_path, host=host)

    result = runtime.open_qgc()

    assert result == LaunchResult(True, "focused")
    assert host.opened == [host.installation]


def test_local_runtime_build_calls_python_builder_with_shared_workstation(
    tmp_path: Path, monkeypatch,
) -> None:
    import orchestration.local_runtime as local_runtime

    host = FakeHost(tmp_path / "state")
    workstation = object()
    calls = []
    monkeypatch.setattr(
        local_runtime,
        "build_images",
        lambda project, selected_host, selected_workstation: calls.append(
            (project, selected_host, selected_workstation)
        ),
    )
    runtime = LocalRuntime(tmp_path, host=host, workstation=workstation)
    runtime.checks = lambda: {"ok": True}

    assert runtime.build() == {"ok": True}
    assert calls == [(tmp_path.resolve(), host, workstation)]
    assert "DRONE_SIM_WORKSTATION_LOCK_HELD" not in __import__("os").environ


def test_local_runtime_accepts_native_amd64_daemon_for_amd64_host(
    tmp_path: Path, monkeypatch,
) -> None:
    import orchestration.local_runtime as local_runtime

    host = FakeHost(tmp_path / "state")
    runtime = LocalRuntime(tmp_path, host=host)
    runtime._run = lambda _command: SimpleNamespace(
        returncode=0,
        stdout='{"OSType":"linux","Architecture":"x86_64","NCPU":6,"MemTotal":12884901888}',
        stderr="",
    )
    monkeypatch.setattr(
        local_runtime,
        "check_build",
        lambda _project, _host: {"ok": True, "reasons": []},
    )

    result = runtime.checks(active=True)

    daemon = next(
        row for row in result["checks"] if row["name"] == "Local Linux runtime"
    )
    assert daemon["ok"] is True
    assert result["provenance"]["ok"] is True


def test_checks_report_selected_host_and_runtime_provider(tmp_path: Path, monkeypatch) -> None:
    import orchestration.local_runtime as local_runtime

    host = FakeHost(tmp_path / "state")
    host.facts = HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")
    runtime = LocalRuntime(tmp_path, host=host)
    runtime._run = lambda _command: SimpleNamespace(
        returncode=0,
        stdout='{"OSType":"linux","Architecture":"x86_64","NCPU":6,"MemTotal":12884901888}',
        stderr="",
    )
    monkeypatch.setattr(local_runtime, "check_build", lambda *_args: {"ok": True, "reasons": []})

    result = runtime.checks(active=True)

    assert result["host"] == {
        "kind": "wsl2",
        "machine": "amd64",
        "image_platform": "linux/amd64",
        "provider": "docker-desktop",
    }


@pytest.mark.parametrize(
    "facts",
    [
        HostFacts("macos", "arm64", "linux/arm64", "docker-desktop"),
        HostFacts("linux", "amd64", "linux/amd64", "docker"),
        HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop"),
    ],
)
def test_empty_docker_error_names_selected_provider_and_native_platform(
    tmp_path: Path, facts: HostFacts
) -> None:
    host = FakeHost(tmp_path / "state")
    host.facts = facts
    runtime = LocalRuntime(tmp_path, host=host)
    runtime._run = lambda _command: SimpleNamespace(
        returncode=1,
        stdout="",
        stderr="",
    )

    result = runtime.checks(active=True)

    daemon = next(
        row for row in result["checks"] if row["name"] == "Local Linux runtime"
    )
    assert daemon == {
        "name": "Local Linux runtime",
        "ok": False,
        "detail": (
            f"Docker provider {facts.provider} is unavailable for "
            f"{facts.image_platform}; start or repair the selected local runtime"
        ),
    }


def test_checks_report_host_policy_and_interop_prerequisite_failures(
    tmp_path: Path, monkeypatch
) -> None:
    host = FakeHost(tmp_path / "state")
    host.preflight = lambda _paths: (
        HostIssue(
            "helper_policy_blocked",
            "Windows PowerShell policy blocked the packaged helper",
        ),
    )
    runtime = LocalRuntime(tmp_path, host=host)
    monkeypatch.setattr("orchestration.local_runtime.shutil.which", lambda _name: None)

    result = runtime.checks(active=True)

    check = next(row for row in result["checks"] if row["name"] == "Host integration")
    assert check == {
        "name": "Host integration",
        "ok": False,
        "detail": "helper_policy_blocked: Windows PowerShell policy blocked the packaged helper",
    }
