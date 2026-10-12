from __future__ import annotations

import platform
from pathlib import Path
import os
import shutil

import pytest

from orchestration._adapters.wsl_host import WslHost
from orchestration.host_config import HostConfig, HostFacts


_IS_WSL2 = platform.system() == "Linux" and "microsoft-standard" in platform.release().lower()


@pytest.mark.skipif(
    not _IS_WSL2 or shutil.which("powershell.exe") is None,
    reason="requires WSL2 interoperability with Windows PowerShell 5.1",
)
def test_native_wsl_adapter_probe() -> None:
    host = WslHost(
        Path.cwd(),
        HostConfig("wsl2", None, None),
        HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop"),
        environ=os.environ,
    )

    info = host.helper_info()

    assert info.version == "1"
    assert len(info.source_sha256) == 64
