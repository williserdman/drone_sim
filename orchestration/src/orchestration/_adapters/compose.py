"""Policy-free, shell-free Docker Compose subprocess boundary."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import os
from pathlib import Path
import re
import subprocess
import time
from typing import Any
from uuid import UUID

from artifacts import DockerLogCommandResult, ImageDigest, SourceRevision
from orchestration.config import RuntimeTopology


_SERVICE_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
_DIGEST_PATTERN = re.compile(r"(?:sha256:)?([0-9a-f]{64})")
_COMPANION_IMAGE = "drone-sim-companion-runtime:phase3"
_COMP2026_REVISION_LABEL = "org.opencontainers.image.comp2026.revision"
_AMBIENT_COMPOSE_SELECTORS = frozenset(
    {
        "COMPOSE_FILE",
        "COMPOSE_ENV_FILES",
        "COMPOSE_PATH_SEPARATOR",
        "COMPOSE_PROFILES",
        "COMPOSE_PROJECT_NAME",
        "COMPOSE_PROJECT_DIR",
        "COMPOSE_PROJECT_DIRECTORY",
        "COMPOSE_DISABLE_ENV_FILE",
    }
)


@dataclass(frozen=True)
class ComposeCommandResult:
    returncode: int
    output: bytes

    def __post_init__(self) -> None:
        if isinstance(self.returncode, bool) or not isinstance(self.returncode, int):
            raise TypeError("returncode must be an integer")
        if not isinstance(self.output, bytes):
            raise TypeError("output must be bytes")


class ComposeRuntimeError(RuntimeError):
    def __init__(self, message: str, result: ComposeCommandResult | None = None) -> None:
        self.result = result
        super().__init__(message)


Runner = Callable[..., Any]


def _production_runner(
    command: list[str], *, env: Mapping[str, str], timeout: float
) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        shell=False,
        env=dict(env),
        timeout=timeout,
    )


class ComposeRuntime:
    """One run's deterministic Compose command construction and execution."""

    def __init__(
        self,
        *,
        project_directory: Path | str,
        run_id: str,
        run_directory: Path | str,
        config_path: Path | str,
        topology: RuntimeTopology,
        runner: Runner = _production_runner,
        base_environment: Mapping[str, str] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if not isinstance(topology, RuntimeTopology):
            raise TypeError("topology must be a RuntimeTopology")
        try:
            parsed = UUID(run_id)
        except (TypeError, ValueError, AttributeError) as exc:
            raise ValueError("run_id must be a canonical UUID") from exc
        if str(parsed) != run_id:
            raise ValueError("run_id must be a canonical UUID")
        self.project_directory = Path(project_directory)
        self.run_directory = Path(run_directory)
        self.config_path = Path(config_path)
        if not all(
            path.is_absolute()
            for path in (self.project_directory, self.run_directory, self.config_path)
        ):
            raise ValueError("Compose paths must be absolute")
        if self.config_path != self.run_directory / "configuration/run.json":
            raise ValueError("config_path must be the run's resolved configuration")
        self.run_id = run_id
        self.project_name = f"drone-sim-{run_id.replace('-', '')}"
        self.topology = topology
        self._services = frozenset(service for service, _module in topology.ownership)
        self._runner = runner
        self._monotonic = monotonic
        environment = dict(os.environ if base_environment is None else base_environment)
        for selector in _AMBIENT_COMPOSE_SELECTORS:
            environment.pop(selector, None)
        self.environment = {
            **environment,
            "COMPOSE_DISABLE_ENV_FILE": "1",
            "COMPOSE_PROFILES": topology.profile,
            "SIM_RUN_ID": run_id,
            "SIM_RUN_DIRECTORY": str(self.run_directory),
            "SIM_CONFIG_PATH": str(self.config_path),
        }
        if topology.profile == "phase2":
            self.environment["SIM_PHASE2_PROFILE"] = "1"
        self._base = [
            "docker",
            "compose",
            "--file",
            str(self.project_directory / "compose.yaml"),
            "--project-directory",
            str(self.project_directory),
            "-p",
            self.project_name,
        ]

    @staticmethod
    def _timeout(value: float) -> float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError("timeout must be a number")
        if value < 0:
            raise ValueError("timeout must be nonnegative")
        return float(value)

    def _invoke(self, command: list[str], timeout: float) -> ComposeCommandResult:
        bounded = self._timeout(timeout)
        completed = self._runner(command, env=self.environment, timeout=bounded)
        try:
            returncode = completed.returncode
            output = completed.output if hasattr(completed, "output") else completed.stdout
        except AttributeError as exc:
            raise TypeError("Compose runner must return returncode and output/stdout") from exc
        if isinstance(output, str):
            output = output.encode("utf-8", errors="surrogateescape")
        return ComposeCommandResult(returncode, output)

    def _compose(self, arguments: Sequence[str], timeout: float) -> ComposeCommandResult:
        return self._invoke([*self._base, *arguments], timeout)

    def up(self, timeout: float) -> ComposeCommandResult:
        return self._compose(["up", "--detach", "--no-build"], timeout)

    def bind_source_revisions(
        self,
        revisions: Sequence[SourceRevision],
        timeout: float,
    ) -> ComposeCommandResult:
        """Bind Compose interpolation and verify the built nested-source label."""
        records = tuple(revisions)
        if [record.name for record in records] != ["drone_sim", "comp2026"]:
            raise ValueError("source revisions must be ordered drone_sim then comp2026")
        nested = records[1].revision
        if not isinstance(nested, str) or not nested:
            raise ValueError("comp2026 source revision must be nonempty")
        self.environment["SIM_COMP2026_REVISION"] = nested
        result = self._invoke(
            [
                "docker",
                "image",
                "inspect",
                "--format",
                f'{{{{ index .Config.Labels "{_COMP2026_REVISION_LABEL}" }}}}',
                _COMPANION_IMAGE,
            ],
            timeout,
        )
        if result.returncode != 0:
            raise ComposeRuntimeError("companion image revision label unavailable", result)
        try:
            label = result.output.decode("ascii").strip()
        except UnicodeDecodeError as error:
            raise ComposeRuntimeError("companion image revision label is not ASCII", result) from error
        if label != nested:
            raise ComposeRuntimeError(
                "companion image revision label does not match comp2026 source",
                result,
            )
        return result

    def logs(self, command: list[str], timeout: float) -> DockerLogCommandResult:
        """Run exactly one Task 5 frozen per-service log command with safe context."""
        if not isinstance(command, list) or len(command) != 8:
            raise ValueError("log command does not match the frozen Task 5 contract")
        expected_prefix = [
            "docker",
            "compose",
            "-p",
            self.project_name,
            "logs",
            "--no-color",
            "--no-log-prefix",
        ]
        service = command[-1]
        if (
            command[:-1] != expected_prefix
            or _SERVICE_PATTERN.fullmatch(service) is None
            or service not in self._services
        ):
            raise ValueError("log command does not match the frozen Task 5 contract")
        result = self._compose(command[4:], timeout)
        return DockerLogCommandResult(result.returncode, result.output)

    def stop_services(
        self, services: Sequence[str], timeout: float
    ) -> ComposeCommandResult:
        names = list(services)
        if not names or any(
            not isinstance(name, str) or _SERVICE_PATTERN.fullmatch(name) is None
            for name in names
        ):
            raise ValueError("stop_services requires safe service names")
        if any(name not in self._services for name in names):
            raise ValueError("stop_services requires services from the selected topology")
        return self._compose(["stop", *names], timeout)

    def down(self, timeout: float) -> ComposeCommandResult:
        return self._compose(["down", "--remove-orphans"], timeout)

    def ps(self, timeout: float) -> ComposeCommandResult:
        return self._compose(["ps", "--all", "--format", "json"], timeout)

    def image_digests(self, timeout: float) -> tuple[ImageDigest, ...]:
        """Resolve immutable local image IDs within one caller-supplied budget."""
        total = self._timeout(timeout)
        deadline = self._monotonic() + total
        config_result = self._compose(
            ["config", "--images"], max(0.0, deadline - self._monotonic())
        )
        if config_result.returncode != 0:
            raise ComposeRuntimeError("docker compose config --images failed", config_result)
        try:
            images = tuple(
                dict.fromkeys(
                    line.decode("utf-8").strip()
                    for line in config_result.output.splitlines()
                    if line.strip()
                )
            )
        except UnicodeDecodeError as exc:
            raise ComposeRuntimeError("Compose image list is not UTF-8", config_result) from exc
        if not images:
            raise ComposeRuntimeError("Compose profile produced no images", config_result)
        inspect = self._invoke(
            ["docker", "image", "inspect", "--format", "{{.Id}}", *images],
            max(0.0, deadline - self._monotonic()),
        )
        if inspect.returncode != 0:
            raise ComposeRuntimeError("docker image inspect failed", inspect)
        lines = inspect.output.splitlines()
        if len(lines) != len(images):
            raise ComposeRuntimeError("docker image inspect returned the wrong digest count", inspect)
        records: list[ImageDigest] = []
        for image, line in zip(images, lines, strict=True):
            try:
                value = line.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise ComposeRuntimeError("Docker image digest is not ASCII", inspect) from exc
            match = _DIGEST_PATTERN.fullmatch(value)
            if match is None:
                raise ComposeRuntimeError(f"Docker image {image!r} has no immutable SHA-256 ID", inspect)
            records.append(ImageDigest(image, match.group(1)))
        return tuple(records)


__all__ = ["ComposeCommandResult", "ComposeRuntime", "ComposeRuntimeError"]
