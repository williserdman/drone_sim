"""Local process prerequisites, image provenance, and native QGC launch."""

from __future__ import annotations

import json
import os
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
import re
import shutil
import socket
import subprocess
from typing import Any

from .host_integration import HostIntegration, LaunchResult, resolve_host
from .local_build import build_images, check_build
from .workstation import Workstation


_RUN_CONTAINER = re.compile(r"drone-sim-[0-9a-f]{32}-.+")
_RUN_MEDIA_VOLUME = re.compile(r"drone-sim-[0-9a-f]{32}_artifacts-media")


class LocalRuntime:
    def __init__(
        self,
        project_directory: Path,
        *,
        host: HostIntegration | None = None,
        workstation: Workstation | None = None,
    ) -> None:
        self.project_directory = project_directory.resolve()
        self.host = host or resolve_host(self.project_directory)
        self.workstation = workstation or Workstation(self.host)

    def _run(self, command: list[str], *, timeout: float = 15) -> subprocess.CompletedProcess[str]:
        return subprocess.run(command, cwd=self.project_directory, capture_output=True, text=True,
                              timeout=timeout, check=False)

    def remaining_resources(self) -> list[dict[str, str]]:
        containers = self._run([
            "docker", "ps", "--all", "--format", "{{json .}}",
            "--filter", "label=com.docker.compose.project",
        ])
        if containers.returncode:
            raise RuntimeError(
                containers.stderr.strip() or "could not inspect local containers"
            )
        volumes = self._run([
            "docker", "volume", "ls", "--format", "{{json .}}",
            "--filter", "label=com.docker.compose.project",
            "--filter", "label=com.docker.compose.volume=artifacts-media",
        ])
        if volumes.returncode:
            raise RuntimeError(
                volumes.stderr.strip() or "could not inspect local media volumes"
            )
        resources = [
            row
            for line in containers.stdout.splitlines()
            if line
            if _RUN_CONTAINER.fullmatch((row := json.loads(line)).get("Names", ""))
        ]
        resources.extend(
            {"Names": name, "ResourceType": "volume"}
            for line in volumes.stdout.splitlines()
            if line
            if _RUN_MEDIA_VOLUME.fullmatch(
                name := json.loads(line).get("Name", "")
            )
        )
        return resources

    def checks(self, *, active: bool = False) -> dict[str, Any]:
        rows: list[dict[str, Any]] = []

        def add(name: str, ok: bool, detail: str) -> None:
            rows.append({"name": name, "ok": ok, "detail": detail})

        issues = self.host.preflight(
            (
                self.project_directory,
                self.project_directory / "runs",
                self.host.state_directory,
            )
        )
        add(
            "Host integration",
            not issues,
            (
                "; ".join(f"{issue.code}: {issue.message}" for issue in issues)
                if issues
                else f"{self.host.facts.kind} host integration is available"
            ),
        )
        add("Docker", shutil.which("docker") is not None, "Docker CLI must be installed")
        daemon_ok = False
        try:
            result = self._run(["docker", "info", "--format", "{{json .}}"])
            if result.returncode:
                raise RuntimeError(
                    result.stderr.strip()
                    or (
                        f"Docker provider {self.host.facts.provider} is unavailable for "
                        f"{self.host.facts.image_platform}; start or repair the selected "
                        "local runtime"
                    )
                )
            info = json.loads(result.stdout)
            architecture = {
                "aarch64": "arm64",
                "arm64": "arm64",
                "amd64": "amd64",
                "x86_64": "amd64",
            }.get(info["Architecture"])
            daemon_platform = f"linux/{architecture}" if architecture else None
            daemon_ok = (
                info["OSType"] == "linux"
                and daemon_platform == self.host.facts.image_platform
            )
            add(
                "Local Linux runtime",
                daemon_ok,
                f'{info["OSType"]}/{info["Architecture"]}; '
                f'{info["NCPU"]} CPUs; {info["MemTotal"] / 2**30:.1f} GiB RAM; '
                f'expected {self.host.facts.image_platform}',
            )
            add("Runtime resources", info["NCPU"] >= 4 and info["MemTotal"] >= 8 * 2**30,
                "At least 4 CPUs and 8 GiB RAM; 6 CPUs and 12 GiB recommended")
        except (OSError, ValueError, KeyError, RuntimeError, subprocess.TimeoutExpired) as error:
            add("Local Linux runtime", False, str(error))
        try:
            installation = self.host.inspect_qgc()
            add(
                "QGroundControl",
                True,
                f"{installation.version} at {installation.executable}",
            )
        except (OSError, ValueError, RuntimeError) as error:
            add("QGroundControl", False, str(error))
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
                provenance = check_build(self.project_directory, self.host)
            except (OSError, ValueError, subprocess.TimeoutExpired) as error:
                provenance = {"ok": False, "reasons": [str(error)]}
        add("Matching runtime images", provenance.get("ok") is True,
            "source and image identity match" if provenance.get("ok") is True
            else "Build matching images to continue. Details are under Source & image identity.")
        facts = self.host.facts
        return {
            "ok": all(row["ok"] for row in rows),
            "host": {
                "kind": facts.kind,
                "machine": facts.machine,
                "image_platform": facts.image_platform,
                "provider": facts.provider,
            },
            "checks": rows,
            "provenance": provenance,
        }

    def open_qgc(self) -> LaunchResult:
        installation = self.host.inspect_qgc()
        return self.host.open_qgc(installation)

    def build(self) -> dict[str, Any]:
        directory = self.project_directory / ".local-runtime"
        directory.mkdir(exist_ok=True)
        with (directory / "build.log").open("w") as log:
            try:
                with redirect_stdout(log), redirect_stderr(log):
                    build_images(
                        self.project_directory,
                        self.host,
                        self.workstation,
                    )
            except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
                print(str(error), file=log)
                raise RuntimeError(
                    "Runtime build failed; open the build log for details"
                ) from error
        return self.checks()
