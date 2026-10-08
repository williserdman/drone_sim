"""Local process prerequisites, image provenance, and native QGC launch."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
from typing import Any

from .workstation import workstation_lock


class LocalRuntime:
    def __init__(self, project_directory: Path) -> None:
        self.project_directory = project_directory.resolve()

    def _run(self, command: list[str], *, timeout: float = 15) -> subprocess.CompletedProcess[str]:
        return subprocess.run(command, cwd=self.project_directory, capture_output=True, text=True,
                              timeout=timeout, check=False)

    def remaining_resources(self) -> list[dict[str, str]]:
        result = self._run(["docker", "ps", "--all", "--format", "{{json .}}",
                            "--filter", "label=com.docker.compose.project"])
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or "could not inspect local containers")
        return [row for line in result.stdout.splitlines() if line
                if "drone-sim-" in (row := json.loads(line)).get("Names", "")]

    def checks(self, *, active: bool = False) -> dict[str, Any]:
        rows: list[dict[str, Any]] = []

        def add(name: str, ok: bool, detail: str) -> None:
            rows.append({"name": name, "ok": ok, "detail": detail})

        add("Docker", shutil.which("docker") is not None, "Docker CLI must be installed")
        daemon_ok = False
        try:
            result = self._run(["docker", "info", "--format", "{{json .}}"])
            if result.returncode:
                raise RuntimeError(result.stderr.strip() or "Start Colima with colima start --cpu 6 --memory 12")
            info = json.loads(result.stdout)
            daemon_ok = info["OSType"] == "linux" and info["Architecture"] in {"aarch64", "arm64"}
            add("Local Linux runtime", daemon_ok, f'{info["OSType"]}/{info["Architecture"]}; {info["NCPU"]} CPUs; {info["MemTotal"] / 2**30:.1f} GiB RAM')
            add("Runtime resources", info["NCPU"] >= 4 and info["MemTotal"] >= 8 * 2**30,
                "At least 4 CPUs and 8 GiB RAM; 6 CPUs and 12 GiB recommended")
        except (OSError, ValueError, KeyError, RuntimeError, subprocess.TimeoutExpired) as error:
            add("Local Linux runtime", False, str(error))
        qgc = Path("/Applications/QGroundControl.app")
        add("QGroundControl", qgc.is_dir(), str(qgc))
        free = shutil.disk_usage(self.project_directory).free
        add("Disk space", free >= 8 * 2**30, f"{free / 2**30:.1f} GiB free; at least 8 GiB required for run evidence")
        if not active:
            for port in (5762, 6080, 8081):
                try:
                    with socket.socket() as probe:
                        probe.bind(("127.0.0.1", port))
                    add(f"Local port {port}", True, "available")
                except OSError:
                    add(f"Local port {port}", False, "already in use; finish the owning session first")
            if daemon_ok:
                try:
                    resources = self.remaining_resources()
                    add("Previous cleanup", not resources,
                        ", ".join(row.get("Names", "unknown") for row in resources) or "no remaining simulation containers")
                except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
                    add("Previous cleanup", False, str(error))
        provenance: dict[str, Any] = {"ok": False, "reasons": ["runtime images have not been built"]}
        if daemon_ok:
            try:
                result = self._run([str(self.project_directory / "scripts/local-build.sh"), "--check"], timeout=90)
                provenance = json.loads(result.stdout)
                if result.returncode and provenance.get("ok"):
                    raise ValueError("image check failed despite success response")
            except (OSError, ValueError, subprocess.TimeoutExpired) as error:
                provenance = {"ok": False, "reasons": [str(error)]}
        add("Matching runtime images", provenance.get("ok") is True,
            "source and image identity match" if provenance.get("ok") is True
            else "Build matching images to continue. Details are under Source & image identity.")
        return {"ok": all(row["ok"] for row in rows), "checks": rows, "provenance": provenance}

    def open_qgc(self) -> None:
        result = self._run(["open", "-a", "/Applications/QGroundControl.app"])
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or "QGroundControl could not be opened")

    def build(self) -> dict[str, Any]:
        directory = self.project_directory / ".local-runtime"
        directory.mkdir(exist_ok=True)
        with workstation_lock(allow_nested=True), (directory / "build.log").open("w") as log:
            result = subprocess.run([str(self.project_directory / "scripts/local-build.sh"), "--build"],
                                    cwd=self.project_directory, stdout=log, stderr=subprocess.STDOUT,
                                    env={**os.environ, "DRONE_SIM_WORKSTATION_LOCK_HELD": "1"},
                                    timeout=7200, check=False)
        if result.returncode:
            raise RuntimeError("Runtime build failed; open the build log for details")
        return self.checks()
