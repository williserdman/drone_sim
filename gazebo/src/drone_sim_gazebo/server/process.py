"""Fail-closed construction and supervision of one paused Gazebo server."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path, PurePosixPath
import signal
import stat
import subprocess
import time
from types import MappingProxyType
from typing import Any, Protocol
from uuid import UUID

from orchestration.config import CAMERA_INTERVAL_NS, SimulationConfig

from ..worlds import ResolvedWorld


_DIRECTORY_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_DIRECTORY", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_READ_FILE_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
    | getattr(os, "O_NONBLOCK", 0)
)
_LOG_FLAGS = (
    os.O_WRONLY
    | os.O_APPEND
    | os.O_CREAT
    | os.O_EXCL
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_SNAPSHOT_FIELDS = (
    "st_dev",
    "st_ino",
    "st_mode",
    "st_nlink",
    "st_size",
    "st_mtime_ns",
    "st_ctime_ns",
)
_HEX = frozenset("0123456789abcdef")
_AUTHORITATIVE_PATH = (
    "/opt/ros/jazzy/opt/gz_msgs_vendor/bin:"
    "/opt/ros/jazzy/opt/gz_tools_vendor/bin:"
    "/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:"
    "/opt/ros/jazzy/bin:/opt/drone_sim/venv/bin:"
    "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
)
_AUTHORITATIVE_GZ_CONFIG_PATH = (
    "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz:"
    "/opt/ros/jazzy/opt/sdformat_vendor/share/gz:"
    "/opt/ros/jazzy/opt/gz_gui_vendor/share/gz:"
    "/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:"
    "/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz:"
    "/opt/ros/jazzy/opt/gz_plugin_vendor/share/gz:"
    "/opt/ros/jazzy/opt/gz_fuel_tools_vendor/share/gz:"
    "/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz:"
    "/opt/ros/jazzy/opt/gz_common_vendor/share/gz"
)
_AUTHORITATIVE_LD_LIBRARY_PATH = (
    "/opt/ros/jazzy/opt/gz_sim_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_sensors_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_physics_vendor/lib:"
    "/opt/ros/jazzy/opt/sdformat_vendor/lib:"
    "/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:"
    "/opt/ros/jazzy/lib/x86_64-linux-gnu:"
    "/opt/ros/jazzy/opt/gz_gui_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_transport_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_rendering_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_plugin_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_msgs_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_common_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_math_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_utils_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_tools_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:"
    "/opt/ros/jazzy/opt/gz_cmake_vendor/lib:"
    "/opt/ros/jazzy/lib"
)
_STATIC_ENVIRONMENT = MappingProxyType(
    {
        "PATH": _AUTHORITATIVE_PATH,
        "GZ_CONFIG_PATH": _AUTHORITATIVE_GZ_CONFIG_PATH,
        "LD_LIBRARY_PATH": _AUTHORITATIVE_LD_LIBRARY_PATH,
        "HOME": "/tmp",
    }
)
_PASSIVE_ENVIRONMENT_KEYS = frozenset(
    {
        "PATH",
        "GZ_CONFIG_PATH",
        "LD_LIBRARY_PATH",
        "HOME",
        "GZ_PARTITION",
        "GZ_SIM_RESOURCE_PATH",
    }
)
_FLIGHT_ENVIRONMENT_KEYS = _PASSIVE_ENVIRONMENT_KEYS | {
    "GZ_SIM_SYSTEM_PLUGIN_PATH"
}
_FLIGHT_PLUGIN_PATH = "/opt/drone_sim/gazebo/plugins"
_GROUP_POLL_SECONDS = 0.01


class ServerProcessError(RuntimeError):
    """The server boundary could not safely complete its lifecycle."""


class _Process(Protocol):
    pid: int
    returncode: int | None

    def poll(self) -> int | None: ...

    def wait(self, timeout: float) -> int: ...


PopenFactory = Callable[..., _Process]
Monotonic = Callable[[], float]
Sleep = Callable[[float], None]
SignalProcessGroup = Callable[[int, int], None]
GetProcessGroup = Callable[[int], int]
ProcessGroupExists = Callable[[int, int], bool]
ObserveLeaderExit = Callable[[int], bool]


def _observe_leader_exit_without_reap(pid: int) -> bool:
    """Observe child exit while retaining its PID / group identity as a zombie."""
    try:
        result = os.waitid(
            os.P_PID,
            pid,
            os.WEXITED | os.WNOHANG | os.WNOWAIT,
        )
    except ChildProcessError as error:
        raise OSError("Gazebo leader identity was reaped outside its owner") from error
    if result is None:
        return False
    if result.si_pid != pid:
        raise OSError("waitid returned an unexpected Gazebo leader identity")
    return True


def _linux_process_group_exists(process_group_id: int, session_id: int) -> bool:
    """Return whether the retained Linux session still has a live group member."""
    proc = Path("/proc")
    if not proc.is_dir():
        raise OSError("safe process-group probing requires Linux /proc")
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "stat").read_text(encoding="ascii")
        except (FileNotFoundError, ProcessLookupError, PermissionError, OSError):
            continue
        closing = raw.rfind(")")
        if closing < 0:
            continue
        fields = raw[closing + 2 :].split()
        if len(fields) < 4:
            continue
        state = fields[0]
        try:
            member_group = int(fields[2])
            member_session = int(fields[3])
        except ValueError:
            continue
        if (
            state not in {"Z", "X"}
            and member_group == process_group_id
            and member_session == session_id
        ):
            return True
    return False


def _canonical_run_id(value: object) -> str:
    if not isinstance(value, str):
        raise ValueError("run_id must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (ValueError, TypeError, AttributeError) as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise ValueError("run_id must be a canonical UUID")
    return value


def _positive_deadline(value: object) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value <= 0
    ):
        raise ValueError("deadline must be a positive finite monotonic value")
    return float(value)


def _digest(value: object, *, field: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in _HEX for character in value)
    ):
        raise ValueError(f"{field} must be a lowercase SHA-256 digest")
    return value


def _same_snapshot(first: os.stat_result, second: os.stat_result) -> bool:
    return all(
        getattr(first, field) == getattr(second, field)
        for field in _SNAPSHOT_FIELDS
    )


def _same_entry(first: os.stat_result, second: os.stat_result) -> bool:
    return all(
        getattr(first, field) == getattr(second, field)
        for field in ("st_dev", "st_ino", "st_mode")
    )


def _absolute_lexical_path(value: object, *, field: str) -> Path:
    if not isinstance(value, Path):
        raise TypeError(f"{field} must be a Path")
    if not value.is_absolute():
        raise ValueError(f"{field} must be absolute")
    if ".." in value.parts:
        raise ValueError(f"{field} must not contain parent traversal")
    return value


def _safe_existing_path(value: object, *, field: str, directory: bool) -> Path:
    path = _absolute_lexical_path(value, field=field)
    try:
        named = path.lstat()
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise ValueError(f"{field} is unreadable") from error
    if stat.S_ISLNK(named.st_mode) or resolved != path:
        raise ValueError(f"{field} must not traverse a symlink")
    expected = stat.S_ISDIR(named.st_mode) if directory else stat.S_ISREG(named.st_mode)
    if not expected:
        kind = "directory" if directory else "regular file"
        raise ValueError(f"{field} must be a {kind}")
    if not directory and named.st_nlink != 1:
        raise ValueError(f"{field} must have exactly one hard link")
    return path


def _validate_resource_hashes(value: object) -> tuple[tuple[str, str], ...]:
    if type(value) is not tuple:
        raise TypeError("resource_sha256s must be an immutable tuple")
    result: list[tuple[str, str]] = []
    for item in value:
        if type(item) is not tuple or len(item) != 2:
            raise TypeError("resource_sha256s entries must be immutable pairs")
        relative, digest = item
        if not isinstance(relative, str) or not relative or "\\" in relative:
            raise ValueError("resource checksum paths must be safe relative POSIX paths")
        path = PurePosixPath(relative)
        if path.is_absolute() or ".." in path.parts or path.parts in ((), (".",)):
            raise ValueError("resource checksum paths must be safe relative POSIX paths")
        result.append((relative, _digest(digest, field="resource checksum")))
    immutable = tuple(result)
    if not immutable or immutable != tuple(sorted(immutable)):
        raise ValueError("resource_sha256s must be nonempty, unique, and sorted")
    if len({path for path, _digest_value in immutable}) != len(immutable):
        raise ValueError("resource_sha256s must not contain duplicate paths")
    return immutable


def _validate_config(config: object) -> SimulationConfig:
    if not isinstance(config, SimulationConfig):
        raise TypeError("config must be a SimulationConfig")
    if type(config.seed) is not int or not 0 <= config.seed <= 4_294_967_295:
        raise ValueError("simulation seed must be an unsigned 32-bit integer")
    if (
        type(config.duration_ns) is not int
        or config.duration_ns <= 0
        or config.duration_ns % CAMERA_INTERVAL_NS != 0
    ):
        raise ValueError("simulation duration must contain exact camera intervals")
    if (
        isinstance(config.target_real_time_factor, bool)
        or not isinstance(config.target_real_time_factor, (int, float))
        or not math.isfinite(config.target_real_time_factor)
        or config.target_real_time_factor not in {0.1, 0.25}
    ):
        raise ValueError("target_real_time_factor must be exactly 0.1 or 0.25")
    return config


def _validate_world(value: object) -> ResolvedWorld:
    if not isinstance(value, ResolvedWorld):
        raise TypeError("resolved_world must be a ResolvedWorld")
    if (value.world_name, value.vehicle_id) not in {
        ("phase3_foundation", "iris"),
        ("vertical_descent", "iris_flight"),
        ("competition_mission", "iris_competition"),
    }:
        raise ValueError("server supports only approved local Iris worlds")
    path = _safe_existing_path(value.path, field="world path", directory=False)
    expected_filename = f"{value.world_name}.sdf"
    if path.name != expected_filename:
        raise ValueError(f"world path must identify {expected_filename}")
    _safe_existing_path(value.resource_path, field="resource path", directory=True)
    world_digest = _digest(value.world_sha256, field="world_sha256")
    resources = _validate_resource_hashes(value.resource_sha256s)
    if dict(resources).get(f"worlds/{expected_filename}") != world_digest:
        raise ValueError("world checksum must match the immutable resource inventory")
    return value


@dataclass(frozen=True)
class ServerSpec:
    """All immutable authority required to launch and diagnose one server."""

    run_id: str
    seed: int
    world_sha256: str
    resource_sha256s: tuple[tuple[str, str], ...]
    argv: tuple[str, ...]
    environment: Mapping[str, str]
    partial_log_path: Path
    final_log_path: Path
    native_state_path: Path

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)
        if type(self.seed) is not int or not 0 <= self.seed <= 4_294_967_295:
            raise ValueError("seed must be an unsigned 32-bit integer")
        _digest(self.world_sha256, field="world_sha256")
        _validate_resource_hashes(self.resource_sha256s)
        if type(self.argv) is not tuple or any(type(value) is not str for value in self.argv):
            raise TypeError("argv must be an immutable tuple of strings")
        if "-r" in self.argv:
            raise ValueError("Gazebo server must start paused")
        if not isinstance(self.environment, MappingProxyType):
            raise TypeError("environment must be an immutable mapping")
        environment = MappingProxyType(dict(self.environment))
        object.__setattr__(self, "environment", environment)
        flight = bool(self.argv) and self.argv[-1].endswith(
            ("/vertical_descent.sdf", "/competition_mission.sdf")
        )
        expected_environment_keys = (
            _FLIGHT_ENVIRONMENT_KEYS if flight else _PASSIVE_ENVIRONMENT_KEYS
        )
        if set(environment) != expected_environment_keys:
            raise ValueError("environment must contain only authoritative Gazebo keys")
        for key, expected in _STATIC_ENVIRONMENT.items():
            if environment[key] != expected:
                raise ValueError(f"{key} must match the pinned runtime image")
        if flight and environment["GZ_SIM_SYSTEM_PLUGIN_PATH"] != _FLIGHT_PLUGIN_PATH:
            raise ValueError("flight plugin path must match the pinned runtime image")
        for field, value in (
            ("partial_log_path", self.partial_log_path),
            ("final_log_path", self.final_log_path),
            ("native_state_path", self.native_state_path),
        ):
            _absolute_lexical_path(value, field=field)
        run_directory = self.partial_log_path.parent.parent
        if (
            run_directory.name != self.run_id
            or self.final_log_path != run_directory / "gazebo/server.log"
            or self.partial_log_path != run_directory / "gazebo/server.log.partial"
            or self.native_state_path != run_directory / "gazebo/state/state.tlog"
        ):
            raise ValueError("server paths must be canonical paths inside the current run")
        expected_partition = "drone_sim_" + self.run_id.replace("-", "_")
        if environment["GZ_PARTITION"] != expected_partition:
            raise ValueError("GZ_PARTITION must be derived from the current run_id")
        resource_path = _safe_existing_path(
            Path(environment["GZ_SIM_RESOURCE_PATH"]),
            field="GZ_SIM_RESOURCE_PATH",
            directory=True,
        )
        if resource_path.name != "models":
            raise ValueError("GZ_SIM_RESOURCE_PATH must identify the local models directory")
        if len(self.argv) != 9:
            raise ValueError("argv must be the exact paused Gazebo server command")
        world_path = _safe_existing_path(
            Path(self.argv[-1]), field="server argv world path", directory=False
        )
        expected_argv = (
            "gz",
            "sim",
            "-s",
            "--headless-rendering",
            "--seed",
            str(self.seed),
            "--record-path",
            str(self.native_state_path.parent),
            str(world_path),
        )
        if self.argv != expected_argv:
            raise ValueError("argv must be the exact paused Gazebo server command")
        if (
            world_path.name
            not in {
                "phase3_foundation.sdf",
                "vertical_descent.sdf",
                "competition_mission.sdf",
            }
            or world_path.parent.name != "worlds"
            or world_path.parent.parent != resource_path.parent
        ):
            raise ValueError("argv world and environment resources must share the local root")


@dataclass(frozen=True)
class NativeArtifactSummary:
    """Validated immutable Gazebo-native output and shutdown facts."""

    server_log_path: Path
    state_log_path: Path
    server_returncode: int
    graceful: bool

    def __post_init__(self) -> None:
        server_log = _absolute_lexical_path(
            self.server_log_path, field="server_log_path"
        )
        state_log = _absolute_lexical_path(self.state_log_path, field="state_log_path")
        if server_log.name != "server.log" or server_log.parent.name != "gazebo":
            raise ValueError("server_log_path must be the canonical Gazebo server log")
        if (
            state_log.name != "state.tlog"
            or state_log.parent.name != "state"
            or state_log.parent.parent != server_log.parent
        ):
            raise ValueError("state_log_path must be the canonical Gazebo native state log")
        _canonical_run_id(server_log.parent.parent.name)
        if type(self.server_returncode) is not int:
            raise TypeError("server_returncode must be an integer")
        if type(self.graceful) is not bool:
            raise TypeError("graceful must be a boolean")


def server_spec(
    *,
    run_id: str,
    run_directory: Path,
    resolved_world: ResolvedWorld,
    config: SimulationConfig,
) -> ServerSpec:
    """Validate inputs and derive the exact paused, run-isolated server spec."""
    canonical_run_id = _canonical_run_id(run_id)
    run_directory = _safe_existing_path(
        run_directory, field="run directory", directory=True
    )
    if run_directory.name != canonical_run_id:
        raise ValueError("run directory name must match run_id")
    resolved_world = _validate_world(resolved_world)
    config = _validate_config(config)
    expected_factor = (
        0.25 if resolved_world.world_name == "competition_mission" else 0.1
    )
    if config.target_real_time_factor != expected_factor:
        raise ValueError(
            f"{resolved_world.world_name} target_real_time_factor must be "
            f"exactly {expected_factor}"
        )
    gazebo_directory = run_directory / "gazebo"
    state_directory = gazebo_directory / "state"
    environment_values = {
        **_STATIC_ENVIRONMENT,
        "GZ_PARTITION": "drone_sim_" + canonical_run_id.replace("-", "_"),
        "GZ_SIM_RESOURCE_PATH": str(resolved_world.resource_path),
    }
    if resolved_world.world_name in {"vertical_descent", "competition_mission"}:
        environment_values["GZ_SIM_SYSTEM_PLUGIN_PATH"] = _FLIGHT_PLUGIN_PATH
    environment = MappingProxyType(environment_values)
    return ServerSpec(
        run_id=canonical_run_id,
        seed=config.seed,
        world_sha256=resolved_world.world_sha256,
        resource_sha256s=resolved_world.resource_sha256s,
        argv=(
            "gz",
            "sim",
            "-s",
            "--headless-rendering",
            "--seed",
            str(config.seed),
            "--record-path",
            str(state_directory),
            str(resolved_world.path),
        ),
        environment=environment,
        partial_log_path=gazebo_directory / "server.log.partial",
        final_log_path=gazebo_directory / "server.log",
        native_state_path=state_directory / "state.tlog",
    )


class GazeboServer:
    """Own one child process group and publish its native evidence once."""

    def __init__(
        self,
        spec: ServerSpec,
        *,
        popen_factory: PopenFactory = subprocess.Popen,
        monotonic: Monotonic = time.monotonic,
        sleep: Sleep = time.sleep,
        get_process_group: GetProcessGroup = os.getpgid,
        process_group_exists: ProcessGroupExists = _linux_process_group_exists,
        observe_leader_exit: ObserveLeaderExit = _observe_leader_exit_without_reap,
        signal_process_group: SignalProcessGroup = os.killpg,
    ) -> None:
        if not isinstance(spec, ServerSpec):
            raise TypeError("spec must be a ServerSpec")
        self._spec = spec
        self._popen_factory = popen_factory
        self._monotonic = monotonic
        self._sleep = sleep
        self._get_process_group = get_process_group
        self._process_group_exists = process_group_exists
        self._observe_leader_exit = observe_leader_exit
        self._signal_process_group = signal_process_group
        self._process: _Process | None = None
        self._process_group_id: int | None = None
        self._session_id: int | None = None
        self._log_stream: Any | None = None
        self._log_identity: os.stat_result | None = None
        self._run_fd: int | None = None
        self._gazebo_fd: int | None = None
        self._state_fd: int | None = None
        self._state_log_fd: int | None = None
        self._run_identity: os.stat_result | None = None
        self._gazebo_identity: os.stat_result | None = None
        self._state_identity: os.stat_result | None = None
        self._state_log_identity: os.stat_result | None = None
        self._summary: NativeArtifactSummary | None = None
        self._failure: ServerProcessError | None = None
        self._started = False

    @property
    def spec(self) -> ServerSpec:
        """Return the immutable launch authority; replacement is not supported."""
        return self._spec

    def _latch(self, error: BaseException, *, context: str) -> ServerProcessError:
        if self._failure is None:
            self._failure = ServerProcessError(
                f"{context}: {type(error).__name__}: {error}"
            )
        return self._failure

    def _raise_failure(self) -> None:
        if self._failure is not None:
            raise ServerProcessError(str(self._failure))

    @staticmethod
    def _open_or_create_directory(parent_fd: int, name: str) -> tuple[int, os.stat_result]:
        try:
            before = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            os.mkdir(name, 0o755, dir_fd=parent_fd)
            os.fsync(parent_fd)
            before = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        if stat.S_ISLNK(before.st_mode) or not stat.S_ISDIR(before.st_mode):
            raise OSError(f"unsafe owned directory collision at {name}")
        descriptor = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
        opened = os.fstat(descriptor)
        if not _same_entry(before, opened):
            os.close(descriptor)
            raise OSError(f"unsafe owned directory changed while opening: {name}")
        return descriptor, opened

    def _prepare_directories(self) -> None:
        run_directory = self.spec.partial_log_path.parent.parent
        named_run = run_directory.lstat()
        self._run_fd = os.open(run_directory, _DIRECTORY_FLAGS)
        opened_run = os.fstat(self._run_fd)
        if not stat.S_ISDIR(named_run.st_mode) or not _same_entry(named_run, opened_run):
            raise OSError("run directory is unsafe")
        self._run_identity = opened_run
        self._gazebo_fd, self._gazebo_identity = self._open_or_create_directory(
            self._run_fd, "gazebo"
        )
        gazebo_entries = set(os.listdir(self._gazebo_fd))
        if gazebo_entries:
            raise FileExistsError("unsafe Gazebo owned-path collision")
        for name in ("server.log.partial", "server.log"):
            try:
                os.stat(name, dir_fd=self._gazebo_fd, follow_symlinks=False)
            except FileNotFoundError:
                continue
            raise FileExistsError(f"unsafe server log collision: {name}")

    def _preamble(self) -> bytes:
        document = {
            "argv": list(self.spec.argv),
            "event": "gazebo_server_start",
            "partition": self.spec.environment["GZ_PARTITION"],
            "resource_sha256s": [list(item) for item in self.spec.resource_sha256s],
            "run_id": self.spec.run_id,
            "seed": self.spec.seed,
            "world_sha256": self.spec.world_sha256,
        }
        return (
            json.dumps(
                document,
                allow_nan=False,
                ensure_ascii=True,
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n"
        ).encode("utf-8")

    @staticmethod
    def _write_all(descriptor: int, payload: bytes) -> None:
        written = 0
        while written < len(payload):
            count = os.write(descriptor, payload[written:])
            if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
                raise OSError("diagnostic preamble write made no progress")
            written += count

    def start(self) -> None:
        """Start the child once after durably committing its diagnostic preamble."""
        self._raise_failure()
        if self._started or self._summary is not None:
            raise ServerProcessError("Gazebo server can be started only once")
        descriptor: int | None = None
        try:
            self._prepare_directories()
            assert self._gazebo_fd is not None
            descriptor = os.open(
                "server.log.partial",
                _LOG_FLAGS,
                0o644,
                dir_fd=self._gazebo_fd,
            )
            self._write_all(descriptor, self._preamble())
            os.fsync(descriptor)
            os.fsync(self._gazebo_fd)
            self._log_identity = os.fstat(descriptor)
            self._log_stream = os.fdopen(descriptor, "ab", buffering=0)
            descriptor = None
            process = self._popen_factory(
                self.spec.argv,
                stdin=subprocess.DEVNULL,
                stdout=self._log_stream,
                stderr=self._log_stream,
                env=dict(self.spec.environment),
                shell=False,
                start_new_session=True,
            )
            if type(process.pid) is not int or process.pid <= 0:
                raise TypeError("spawned process must expose a positive integer pid")
            self._process = process
            # start_new_session=True makes the child's PID both its session and
            # process-group identity before exec. Retain that identity rather
            # than deriving it later from a possibly reaped / reused leader.
            self._process_group_id = process.pid
            self._session_id = process.pid
            self._started = True
        except BaseException as error:
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
            try:
                self._close_log_handle()
            except OSError:
                pass
            self._close_directories()
            raise self._latch(error, context="Gazebo server start failed") from error

    def _seal_log(self, deadline: float) -> None:
        stream = self._log_stream
        if stream is None:
            raise OSError("startup log descriptor is unavailable")
        self._check_deadline(deadline, operation="partial server log flush")
        stream.flush()
        self._check_deadline(deadline, operation="partial server log fsync")
        os.fsync(stream.fileno())

    def _close_log_handle(self) -> None:
        stream = self._log_stream
        self._log_stream = None
        if stream is not None:
            stream.close()

    def _close_log_handle_best_effort(self) -> None:
        """Release the committed log handle without changing its success result."""
        stream = self._log_stream
        self._log_stream = None
        if stream is None:
            return
        descriptor: int | None = None
        try:
            descriptor = stream.fileno()
        except Exception:
            pass
        try:
            stream.close()
        except Exception:
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass

    def _abandon_log_after_failure(self) -> None:
        """Drop the parent handle without starting new work after a stop fault."""
        stream = self._log_stream
        self._log_stream = None
        if stream is not None:
            try:
                stream.close()
            except OSError:
                pass

    def _close_directories(self) -> None:
        for attribute in ("_state_log_fd", "_state_fd", "_gazebo_fd", "_run_fd"):
            descriptor = getattr(self, attribute)
            setattr(self, attribute, None)
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass

    def _remaining(self, deadline: float) -> float:
        current = self._monotonic()
        if (
            isinstance(current, bool)
            or not isinstance(current, (int, float))
            or not math.isfinite(current)
        ):
            raise ValueError("monotonic clock returned a non-finite value")
        return max(0.0, deadline - float(current))

    def _check_deadline(self, deadline: float, *, operation: str) -> None:
        if self._remaining(deadline) <= 0:
            raise TimeoutError(
                f"absolute monotonic deadline expired before {operation}"
            )

    @staticmethod
    def _returncode(value: object) -> int:
        if type(value) is not int:
            raise TypeError("child return code must be an integer")
        return value

    def _group_exists(self) -> bool:
        process_group_id = self._process_group_id
        session_id = self._session_id
        if process_group_id is None or session_id is None:
            raise OSError("safe process-group identity is unavailable")
        exists = self._process_group_exists(process_group_id, session_id)
        if type(exists) is not bool:
            raise TypeError("process-group probe must return a boolean")
        return exists

    def _leader_has_exited(self, process: _Process) -> bool:
        exited = self._observe_leader_exit(process.pid)
        if type(exited) is not bool:
            raise TypeError("leader-exit observer must return a boolean")
        return exited

    def _verify_retained_leader_group(self, process: _Process) -> None:
        try:
            actual = self._get_process_group(process.pid)
        except ProcessLookupError as error:
            raise OSError("retained Gazebo leader identity disappeared") from error
        if type(actual) is not int or actual != self._process_group_id:
            raise OSError("Gazebo process-group identity changed before signaling")

    def _wait_for_group_empty(self, phase_deadline: float) -> bool:
        while True:
            if self._remaining(phase_deadline) <= 0:
                return False
            group_exists = self._group_exists()
            remaining = self._remaining(phase_deadline)
            if not group_exists:
                return True
            if remaining <= 0:
                return False
            self._sleep(min(_GROUP_POLL_SECONDS, remaining))

    def _wait_for_leader_exit(self, process: _Process, deadline: float) -> None:
        """Wait for the retained child to become waitable without reaping it."""
        while True:
            self._check_deadline(deadline, operation="Gazebo leader exit observation")
            if self._leader_has_exited(process):
                self._check_deadline(
                    deadline, operation="post-observation leader verification"
                )
                return
            remaining = self._remaining(deadline)
            if remaining <= 0:
                raise TimeoutError(
                    "absolute monotonic deadline expired before Gazebo leader exit"
                )
            self._sleep(min(_GROUP_POLL_SECONDS, remaining))

    def _reap_leader(self, process: _Process, deadline: float) -> int:
        self._check_deadline(deadline, operation="Gazebo leader reap")
        remaining = self._remaining(deadline)
        if remaining <= 0:
            raise TimeoutError(
                "absolute monotonic deadline expired before Gazebo leader reap"
            )
        return self._returncode(process.wait(timeout=remaining))

    def _stop_process(self, deadline: float) -> tuple[int, bool]:
        process = self._process
        if process is None:
            raise ServerProcessError("Gazebo server has not been started")
        self._check_deadline(deadline, operation="Gazebo leader exit observation")
        leader_exited = self._leader_has_exited(process)
        self._check_deadline(deadline, operation="Gazebo process-group probe")
        group_exists = self._group_exists()
        self._check_deadline(deadline, operation="post-probe shutdown decision")
        if not group_exists:
            if not leader_exited:
                self._wait_for_leader_exit(process, deadline)
            return self._reap_leader(process, deadline), False

        signals_sent: list[int] = []
        for index, signum in enumerate((signal.SIGTERM, signal.SIGKILL)):
            self._check_deadline(deadline, operation="Gazebo process-group shutdown")
            self._leader_has_exited(process)
            self._check_deadline(deadline, operation="process-group probe")
            group_exists = self._group_exists()
            self._check_deadline(deadline, operation="post-probe signal decision")
            if not group_exists:
                break
            self._check_deadline(deadline, operation="process-group identity validation")
            self._verify_retained_leader_group(process)
            self._check_deadline(deadline, operation=f"signal {signum}")
            try:
                assert self._process_group_id is not None
                self._signal_process_group(self._process_group_id, signum)
            except ProcessLookupError:
                self._check_deadline(deadline, operation="process-group disappearance probe")
                group_exists = self._group_exists()
                self._check_deadline(
                    deadline, operation="post-disappearance probe decision"
                )
                if not group_exists:
                    break
                raise
            signals_sent.append(signum)
            remaining = self._remaining(deadline)
            if remaining <= 0:
                raise TimeoutError(
                    "absolute monotonic deadline expired before process-group wait"
                )
            timeout = remaining / (2 - index)
            phase_deadline = deadline - remaining + timeout
            if self._wait_for_group_empty(phase_deadline):
                break

        self._check_deadline(deadline, operation="final process-group quiescence probe")
        group_exists = self._group_exists()
        self._check_deadline(deadline, operation="post-quiescence probe decision")
        if group_exists:
            raise subprocess.TimeoutExpired("Gazebo process-group shutdown", 0)
        if not self._leader_has_exited(process):
            self._wait_for_leader_exit(process, deadline)
        self._check_deadline(deadline, operation="post-quiescence leader verification")
        returncode = self._reap_leader(process, deadline)
        graceful = signals_sent == [signal.SIGTERM]
        return returncode, graceful

    def _verify_owned_directories(self) -> None:
        values = (
            (self._run_fd, self._run_identity, None, None),
            (self._gazebo_fd, self._gazebo_identity, self._run_fd, "gazebo"),
            (self._state_fd, self._state_identity, self._gazebo_fd, "state"),
        )
        for descriptor, identity, parent_fd, name in values:
            if descriptor is None or identity is None:
                raise OSError("owned directory descriptor is unavailable")
            opened = os.fstat(descriptor)
            named = (
                self.spec.partial_log_path.parent.parent.lstat()
                if parent_fd is None
                else os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
            )
            if not _same_entry(identity, opened) or not _same_entry(identity, named):
                raise OSError("owned Gazebo directory changed during lifecycle")

    def _acquire_native_state_directory(self, deadline: float) -> None:
        assert self._gazebo_fd is not None
        self._check_deadline(deadline, operation="native state directory validation")
        try:
            named = os.stat("state", dir_fd=self._gazebo_fd, follow_symlinks=False)
        except FileNotFoundError as error:
            raise OSError("Gazebo did not create its native state directory") from error
        if stat.S_ISLNK(named.st_mode) or not stat.S_ISDIR(named.st_mode):
            raise OSError("Gazebo native state path is not a safe directory")
        self._check_deadline(deadline, operation="native state directory open")
        descriptor = os.open("state", _DIRECTORY_FLAGS, dir_fd=self._gazebo_fd)
        opened = os.fstat(descriptor)
        if not _same_entry(named, opened):
            os.close(descriptor)
            raise OSError("Gazebo native state directory changed while opening")
        self._state_fd = descriptor
        self._state_identity = opened

    def _validate_native_state(self, deadline: float) -> None:
        assert self._state_fd is not None
        self._check_deadline(deadline, operation="native state validation")
        try:
            before = os.stat("state.tlog", dir_fd=self._state_fd, follow_symlinks=False)
        except FileNotFoundError as error:
            raise OSError("native state.tlog is missing") from error
        if (
            stat.S_ISLNK(before.st_mode)
            or not stat.S_ISREG(before.st_mode)
            or before.st_nlink != 1
            or before.st_size <= 0
        ):
            raise OSError("native state.tlog must be a nonempty single-link regular file")
        descriptor = os.open("state.tlog", _READ_FILE_FLAGS, dir_fd=self._state_fd)
        try:
            opened = os.fstat(descriptor)
            if not _same_snapshot(before, opened):
                raise OSError("native state.tlog changed while opening")
            self._check_deadline(deadline, operation="native state fsync")
            os.fsync(descriptor)
            self._check_deadline(deadline, operation="native state verification")
            after = os.fstat(descriptor)
            named_after = os.stat(
                "state.tlog", dir_fd=self._state_fd, follow_symlinks=False
            )
            if not _same_snapshot(opened, after) or not _same_snapshot(after, named_after):
                raise OSError("native state.tlog changed during validation")
            self._state_log_fd = descriptor
            self._state_log_identity = after
        except BaseException:
            os.close(descriptor)
            raise
        self._check_deadline(deadline, operation="native state directory fsync")
        os.fsync(self._state_fd)
        self._check_deadline(deadline, operation="native state durability")

    def _verify_native_state_binding(self, deadline: float) -> None:
        assert self._gazebo_fd is not None
        assert self._state_fd is not None
        descriptor = self._state_log_fd
        identity = self._state_log_identity
        if descriptor is None or identity is None:
            raise OSError("native state descriptor identity is unavailable")
        self._check_deadline(deadline, operation="final native state inventory")
        entries = set(os.listdir(self._gazebo_fd))
        if (
            not {"state", "server.log.partial"} <= entries
            or entries - {"state", "server.log.partial", "server.log"}
        ):
            raise OSError("Gazebo directory inventory changed before publication")
        self._check_deadline(deadline, operation="final native state parent identity")
        self._verify_owned_directories()
        self._check_deadline(deadline, operation="final native state file identity")
        opened = os.fstat(descriptor)
        named = os.stat("state.tlog", dir_fd=self._state_fd, follow_symlinks=False)
        if not _same_snapshot(identity, opened) or not _same_snapshot(opened, named):
            raise OSError("canonical native state identity changed before publication")

    def _publish_log(self, deadline: float) -> None:
        assert self._gazebo_fd is not None
        stream = self._log_stream
        identity = self._log_identity
        if stream is None or identity is None:
            raise OSError("startup log descriptor identity is unavailable")
        self._check_deadline(deadline, operation="server log publication")
        try:
            final = os.stat("server.log", dir_fd=self._gazebo_fd, follow_symlinks=False)
        except FileNotFoundError:
            final = None
        if final is not None:
            raise FileExistsError("server.log already exists; refusing to clobber it")
        before = os.stat(
            "server.log.partial", dir_fd=self._gazebo_fd, follow_symlinks=False
        )
        opened = os.fstat(stream.fileno())
        if (
            stat.S_ISLNK(before.st_mode)
            or not stat.S_ISREG(before.st_mode)
            or before.st_nlink != 1
            or before.st_size <= 0
            or not stat.S_ISREG(opened.st_mode)
            or opened.st_nlink != 1
            or opened.st_size <= 0
            or not _same_entry(identity, opened)
            or not _same_entry(opened, before)
        ):
            raise OSError(
                "server.log.partial changed identity or is not the retained "
                "nonempty single-link regular startup log"
            )
        self._check_deadline(deadline, operation="server log no-clobber link")
        os.link(
            "server.log.partial",
            "server.log",
            src_dir_fd=self._gazebo_fd,
            dst_dir_fd=self._gazebo_fd,
            follow_symlinks=False,
        )
        self._check_deadline(deadline, operation="linked server log validation")
        retained = os.fstat(stream.fileno())
        partial = os.stat(
            "server.log.partial", dir_fd=self._gazebo_fd, follow_symlinks=False
        )
        linked = os.stat("server.log", dir_fd=self._gazebo_fd, follow_symlinks=False)
        if (
            retained.st_nlink != 2
            or partial.st_nlink != 2
            or linked.st_nlink != 2
            or not _same_entry(identity, retained)
            or not _same_entry(retained, partial)
            or not _same_entry(retained, linked)
        ):
            raise OSError("temporary no-clobber server log link has an invalid identity")
        self._check_deadline(deadline, operation="linked server log directory fsync")
        os.fsync(self._gazebo_fd)
        # Removing the partial name is the transaction commit point. All
        # potentially blocking durability and identity checks precede it. Once
        # this exact unlink succeeds, no later clock observation can reverse
        # the already durable no-clobber publication.
        self._check_deadline(deadline, operation="server log publication commit")
        os.unlink("server.log.partial", dir_fd=self._gazebo_fd)

    def stop(self, deadline: float) -> NativeArtifactSummary:
        """Bound shutdown and no-clobber publication by one absolute deadline."""
        deadline = _positive_deadline(deadline)
        if self._summary is not None:
            return self._summary
        self._raise_failure()
        if not self._started:
            raise ServerProcessError("Gazebo server has not been started")
        try:
            self._check_deadline(deadline, operation="Gazebo shutdown")
            returncode, graceful = self._stop_process(deadline)
            self._check_deadline(deadline, operation="Gazebo artifact finalization")
            self._seal_log(deadline)
            self._acquire_native_state_directory(deadline)
            self._check_deadline(deadline, operation="owned directory validation")
            self._verify_owned_directories()
            self._validate_native_state(deadline)
            self._verify_native_state_binding(deadline)
            self._publish_log(deadline)
            self._close_log_handle_best_effort()
            self._summary = NativeArtifactSummary(
                self.spec.final_log_path,
                self.spec.native_state_path,
                returncode,
                graceful,
            )
            return self._summary
        except BaseException as error:
            self._abandon_log_after_failure()
            raise self._latch(error, context="Gazebo server stop failed") from error
        finally:
            self._close_directories()


__all__ = [
    "GazeboServer",
    "NativeArtifactSummary",
    "ServerProcessError",
    "ServerSpec",
    "server_spec",
]
