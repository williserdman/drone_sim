from __future__ import annotations

from pathlib import Path

from artifacts.qgc_operator import MavlinkIdentity
from orchestration.host_config import HostFacts
from orchestration.host_integration import LaunchResult, QgcInstallation
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
