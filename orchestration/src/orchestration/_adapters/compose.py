"""Shell-free Docker Compose subprocess boundary."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import errno
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import subprocess
import time
from typing import Any
from uuid import UUID

from artifacts import DockerLogCommandResult, ImageDigest, SourceRevision
from orchestration.config import QGCSources, RuntimeTopology


_SERVICE_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
_DIGEST_PATTERN = re.compile(r"(?:sha256:)?([0-9a-f]{64})")
_COMPANION_IMAGE = "drone-sim-companion-runtime:phase3"
_COMP2026_REVISION_LABEL = "org.opencontainers.image.comp2026.revision"
_QGC_STATE_ROOT = Path("/var/lib/drone-sim/comp2026-attempt-state")
_QGC_STATE_ENV = "SIM_QGC_ATTEMPT_STATE_DIRECTORY"
_QGC_ORIGIN_ENV = "SIM_LAUNCH_ORIGIN_JSON"
_LOCAL_DOCKER_HOST = "unix:///var/run/docker.sock"
_DOCKER_DAEMON_ENV = frozenset(
    {
        "DOCKER_HOST",
        "DOCKER_CONTEXT",
        "DOCKER_TLS",
        "DOCKER_TLS_VERIFY",
        "DOCKER_CERT_PATH",
    }
)
_QGC_STATE_ENTRIES = frozenset({"attempt-ledger.json", "attempt-ledger.json.lock"})
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


@dataclass(frozen=True, slots=True)
class _LaunchOrigin:
    latitude_deg: float
    longitude_deg: float
    amsl_m: float
    heading_deg: float

    @property
    def canonical_json(self) -> str:
        return json.dumps(
            {
                "latitude_deg": self.latitude_deg,
                "longitude_deg": self.longitude_deg,
                "amsl_m": self.amsl_m,
                "heading_deg": self.heading_deg,
            },
            sort_keys=True,
            separators=(",", ":"),
        )


_DIAGNOSTIC_LAUNCH_ORIGIN = _LaunchOrigin(
    latitude_deg=37.4003371,
    longitude_deg=-122.0800351,
    amsl_m=0.0,
    heading_deg=0.0,
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


def _canonical_absolute_path(value: Path | str, *, label: str) -> Path:
    try:
        raw = os.fspath(value)
    except TypeError as exc:
        raise ValueError(f"{label} must be a filesystem path") from exc
    if not isinstance(raw, str) or "\0" in raw:
        raise ValueError(f"{label} must be a valid text filesystem path")
    if raw.startswith("//"):
        raise ValueError(f"{label} must use a single-slash filesystem anchor")
    path = Path(raw)
    try:
        canonical = Path(os.path.abspath(raw))
    except (OSError, ValueError) as exc:
        raise ValueError(f"{label} is malformed") from exc
    if not path.is_absolute() or path != canonical:
        raise ValueError(f"{label} must be canonical and absolute")
    return canonical


def _directory_flags() -> int:
    return (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )


def _open_real_directory(path: Path, *, label: str) -> int:
    flags = _directory_flags()
    try:
        current_fd = os.open("/", flags)
    except (OSError, ValueError) as exc:  # pragma: no cover - fixed host root
        raise ValueError("cannot inspect the local filesystem root") from exc
    try:
        for component in path.parts[1:]:
            next_fd = os.open(component, flags, dir_fd=current_fd)
            os.close(current_fd)
            current_fd = next_fd
    except (OSError, ValueError) as exc:
        os.close(current_fd)
        raise ValueError(f"{label} must be an existing real directory") from exc
    except BaseException:
        os.close(current_fd)
        raise
    return current_fd


def _validate_regular_at(directory_fd: int, name: str, *, required: bool) -> None:
    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
        | getattr(os, "O_NONBLOCK", 0)
    )
    try:
        descriptor = os.open(name, flags, dir_fd=directory_fd)
    except OSError as exc:
        if not required and exc.errno == errno.ENOENT:
            return
        raise ValueError(f"attempt state {name} must be a regular non-symlink file") from exc
    try:
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            raise ValueError(f"attempt state {name} must be a regular non-symlink file")
    finally:
        os.close(descriptor)


def _validate_qgc_state(root: Path, state_id: str) -> None:
    root_fd = _open_real_directory(root, label="attempt state root")
    try:
        try:
            state_fd = os.open(state_id, _directory_flags(), dir_fd=root_fd)
        except (OSError, ValueError) as exc:
            raise ValueError("attempt state directory must be an existing real directory") from exc
        try:
            entries = frozenset(os.listdir(state_fd))
            if entries - _QGC_STATE_ENTRIES:
                raise ValueError("attempt state directory contains an unexpected filename")
            _validate_regular_at(state_fd, "attempt-ledger.json", required=True)
            _validate_regular_at(state_fd, "attempt-ledger.json.lock", required=True)
        finally:
            os.close(state_fd)
    finally:
        os.close(root_fd)


def _qgc_launch_origin(payload: bytes) -> str:
    try:
        policy = json.loads(payload.decode("utf-8"))
        origin = policy["simulator_launch_origin"]
    except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError) as exc:
        raise ValueError("simulator launch origin is missing or invalid") from exc
    fields = {"latitude_deg", "longitude_deg", "amsl_m", "heading_deg"}
    if not isinstance(origin, dict) or set(origin) != fields:
        raise ValueError("simulator launch origin must contain exactly four fields")
    normalized: dict[str, float] = {}
    for name in fields:
        value = origin[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"simulator launch origin {name} must be a finite JSON number")
        try:
            normalized[name] = float(value)
        except OverflowError as exc:
            raise ValueError(
                f"simulator launch origin {name} must be a finite JSON number"
            ) from exc
        if not math.isfinite(normalized[name]):
            raise ValueError(f"simulator launch origin {name} must be a finite JSON number")
    if not -90 <= normalized["latitude_deg"] <= 90:
        raise ValueError("simulator launch origin latitude is outside [-90, 90]")
    if not -180 <= normalized["longitude_deg"] <= 180:
        raise ValueError("simulator launch origin longitude is outside [-180, 180]")
    if not 0 <= normalized["heading_deg"] < 360:
        raise ValueError("simulator launch origin heading is outside [0, 360)")
    return _LaunchOrigin(**normalized).canonical_json


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
        qgc: QGCSources | None = None,
        runner: Runner = _production_runner,
        base_environment: Mapping[str, str] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        test_only_qgc_state_root: Path | str | None = None,
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
        if qgc is not None and not isinstance(qgc, QGCSources):
            raise TypeError("qgc must be QGCSources or None")
        self._services = frozenset(service for service, _module in topology.ownership)
        self._runner = runner
        self._monotonic = monotonic
        environment = dict(os.environ if base_environment is None else base_environment)
        for selector in _AMBIENT_COMPOSE_SELECTORS:
            environment.pop(selector, None)
        overlay = environment.pop("SIM_COMPOSE_OVERLAY", "")
        if overlay not in {"", "gpu"}:
            raise ValueError("SIM_COMPOSE_OVERLAY must be empty or 'gpu'")
        environment.pop(_QGC_ORIGIN_ENV, None)
        if qgc is not None:
            environment.pop(_QGC_STATE_ENV, None)
            for name in _DOCKER_DAEMON_ENV:
                environment.pop(name, None)
        self.environment = {
            **environment,
            "COMPOSE_DISABLE_ENV_FILE": "1",
            "COMPOSE_PROFILES": topology.profile,
            "SIM_RUN_ID": run_id,
            "SIM_RUN_DIRECTORY": str(self.run_directory),
            "SIM_CONFIG_PATH": str(self.config_path),
            _QGC_ORIGIN_ENV: _DIAGNOSTIC_LAUNCH_ORIGIN.canonical_json,
        }
        if topology.profile == "phase2":
            self.environment["SIM_PHASE2_PROFILE"] = "1"
        self._qgc_state_root: Path | None = None
        self._qgc_state_id: str | None = None
        if qgc is not None:
            digest = hashlib.sha256(qgc.deployment_profile).hexdigest()
            self._qgc_state_id = f"sha256-{digest}"
            self._qgc_state_root = _canonical_absolute_path(
                _QGC_STATE_ROOT
                if test_only_qgc_state_root is None
                else test_only_qgc_state_root,
                label="attempt state root",
            )
            self.environment[_QGC_STATE_ENV] = str(_QGC_STATE_ROOT / self._qgc_state_id)
            self.environment[_QGC_ORIGIN_ENV] = _qgc_launch_origin(qgc.runtime_policy)
        self._docker = (
            ["docker", "--host", _LOCAL_DOCKER_HOST]
            if qgc is not None
            else ["docker"]
        )
        self._base = [
            *self._docker,
            "compose",
            "--file",
            str(self.project_directory / "compose.yaml"),
            *(
                ["--file", str(self.project_directory / "compose.gpu.yaml")]
                if overlay == "gpu"
                else []
            ),
            *(
                ["--file", str(self.project_directory / "compose.qgc.yaml")]
                if qgc is not None
                else []
            ),
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
        if self._qgc_state_root is not None and self._qgc_state_id is not None:
            _validate_qgc_state(self._qgc_state_root, self._qgc_state_id)
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
                *self._docker,
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
            [*self._docker, "image", "inspect", "--format", "{{.Id}}", *images],
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
