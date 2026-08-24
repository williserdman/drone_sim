"""Host-side capture of raw Compose logs and owned structured events."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from decimal import Decimal
import json
import os
from pathlib import Path
import re
import stat
import subprocess
from typing import Any

from ..structured_log import COMMON_FIELDS, StructuredEvent


REQUIRED_MODULES = (
    "orchestration",
    "artifacts",
    "companion",
    "ardupilot_sitl",
    "gazebo",
    "electromagnet",
    "scorekeeper",
)
HOST_EVENT_PATH = "logs/orchestration-host.jsonl.partial"

_EVENT_KEYS = COMMON_FIELDS | {"fields"}
_PROJECT_PATTERN = re.compile(r"[a-z0-9][a-z0-9_-]*")
_SERVICE_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
_WALL_TIMESTAMP_PATTERN = re.compile(
    r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}"
    r"(?:\.\d{1,6})?(?:Z|[+-]\d{2}:\d{2})"
)
_TIMEZONE_MARKER_PATTERN = re.compile(r"(?:Z|[+-]\d{2}:\d{2})")
_DIRECTORY_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_DIRECTORY", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_CANDIDATE_FLAGS = (
    os.O_WRONLY
    | os.O_CREAT
    | os.O_EXCL
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_HOST_FILE_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
    | getattr(os, "O_NONBLOCK", 0)
)
_FILE_SNAPSHOT_FIELDS = (
    "st_dev",
    "st_ino",
    "st_mode",
    "st_nlink",
    "st_size",
    "st_mtime_ns",
    "st_ctime_ns",
)


@dataclass(frozen=True)
class DockerLogCommandResult:
    """One merged byte stream returned by a shell-free Compose invocation."""

    returncode: int
    output: bytes

    def __post_init__(self) -> None:
        if isinstance(self.returncode, bool) or not isinstance(self.returncode, int):
            raise TypeError("returncode must be an integer")
        if not isinstance(self.output, bytes):
            raise TypeError("output must be bytes")


@dataclass(frozen=True)
class DockerLogDiagnostic:
    """Stable diagnostic suitable for a controller failure report."""

    kind: str
    detail: str
    service: str | None = None
    module: str | None = None
    line_number: int | None = None


@dataclass(frozen=True)
class DockerLogCaptureResult:
    """Immutable publication facts for one capture attempt."""

    succeeded: bool
    raw_paths: tuple[str, ...]
    structured_paths: tuple[str, ...]
    diagnostics: tuple[DockerLogDiagnostic, ...]
    command_failures: tuple[str, ...]
    missing_modules: tuple[str, ...]
    recovery_paths: tuple[str, ...] = ()
    leftover_partials: tuple[str, ...] = ()


class DockerLogCaptureError(RuntimeError):
    """Typed fail-closed capture error carrying all recoverable diagnostics."""

    def __init__(self, result: DockerLogCaptureResult) -> None:
        self.result = result
        detail = (
            result.diagnostics[0].detail
            if result.diagnostics
            else "Docker log capture failed"
        )
        super().__init__(detail)


CommandRunner = Callable[[list[str]], DockerLogCommandResult]


def _run_compose_logs(command: list[str]) -> DockerLogCommandResult:
    completed = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
        shell=False,
    )
    return DockerLogCommandResult(completed.returncode, completed.stdout)


def _same_entry(first: os.stat_result, second: os.stat_result) -> bool:
    return (
        first.st_dev == second.st_dev
        and first.st_ino == second.st_ino
        and first.st_mode == second.st_mode
    )


def _same_file_snapshot(first: os.stat_result, second: os.stat_result) -> bool:
    return all(
        getattr(first, field) == getattr(second, field)
        for field in _FILE_SNAPSHOT_FIELDS
    )


@dataclass
class _Candidate:
    directory_fd: int
    final_name: str
    partial_name: str
    descriptor: int
    identity: os.stat_result
    relative_path: str
    published: bool = False

    @classmethod
    def create(
        cls,
        directory_fd: int,
        final_name: str,
        payload: bytes,
        relative_path: str,
    ) -> _Candidate:
        partial_name = f"{final_name}.partial"
        descriptor = os.open(
            partial_name,
            _CANDIDATE_FLAGS,
            0o600,
            dir_fd=directory_fd,
        )
        try:
            view = memoryview(payload)
            written = 0
            while written < len(view):
                count = os.write(descriptor, view[written:])
                if count <= 0:
                    raise OSError("candidate write made no progress")
                written += count
            os.fsync(descriptor)
            os.fchmod(descriptor, 0o444)
            os.fsync(descriptor)
            identity = os.fstat(descriptor)
            if not stat.S_ISREG(identity.st_mode) or identity.st_nlink != 1:
                raise OSError("candidate is not an exclusively owned regular file")
            named = os.stat(
                partial_name,
                dir_fd=directory_fd,
                follow_symlinks=False,
            )
            if not _same_entry(identity, named):
                raise OSError("candidate path changed before publication")
            return cls(
                directory_fd,
                final_name,
                partial_name,
                descriptor,
                identity,
                relative_path,
            )
        except BaseException:
            try:
                current = os.stat(
                    partial_name,
                    dir_fd=directory_fd,
                    follow_symlinks=False,
                )
            except OSError:
                current = None
            try:
                opened = os.fstat(descriptor)
            except OSError:
                opened = None
            if (
                current is not None
                and opened is not None
                and _same_entry(current, opened)
            ):
                try:
                    os.unlink(partial_name, dir_fd=directory_fd)
                except OSError:
                    pass
            os.close(descriptor)
            raise

    def publish(self) -> None:
        opened = os.fstat(self.descriptor)
        named = os.stat(
            self.partial_name,
            dir_fd=self.directory_fd,
            follow_symlinks=False,
        )
        if (
            not _same_entry(self.identity, opened)
            or not _same_entry(self.identity, named)
            or opened.st_nlink != 1
        ):
            raise OSError("candidate changed before publication")
        os.link(
            self.partial_name,
            self.final_name,
            src_dir_fd=self.directory_fd,
            dst_dir_fd=self.directory_fd,
            follow_symlinks=False,
        )
        linked = os.stat(
            self.final_name,
            dir_fd=self.directory_fd,
            follow_symlinks=False,
        )
        if not _same_entry(self.identity, linked):
            raise OSError("published path does not identify the candidate")
        os.unlink(self.partial_name, dir_fd=self.directory_fd)
        final = os.stat(
            self.final_name,
            dir_fd=self.directory_fd,
            follow_symlinks=False,
        )
        if not _same_entry(self.identity, final) or final.st_nlink != 1:
            raise OSError("published file has an invalid identity")
        os.fsync(self.directory_fd)
        self.published = True

    def close_and_clean(
        self,
    ) -> tuple[tuple[DockerLogDiagnostic, ...], tuple[str, ...]]:
        diagnostics: list[DockerLogDiagnostic] = []
        partial_path = f"{self.relative_path}.partial"
        if not self.published:
            try:
                named = os.stat(
                    self.partial_name,
                    dir_fd=self.directory_fd,
                    follow_symlinks=False,
                )
                opened = os.fstat(self.descriptor)
            except OSError as exc:
                named = None
                opened = None
                diagnostics.append(
                    DockerLogDiagnostic(
                        "cleanup",
                        f"candidate cleanup ownership inspection failed: {exc}",
                    )
                )
            if (
                named is not None
                and opened is not None
                and _same_entry(self.identity, named)
                and _same_entry(self.identity, opened)
            ):
                try:
                    os.unlink(self.partial_name, dir_fd=self.directory_fd)
                except OSError as exc:
                    diagnostics.append(
                        DockerLogDiagnostic(
                            "cleanup",
                            f"candidate cleanup unlink failed for {partial_path}: {exc}",
                        )
                    )
                else:
                    try:
                        os.fsync(self.directory_fd)
                    except OSError as exc:
                        diagnostics.append(
                            DockerLogDiagnostic(
                                "cleanup",
                                f"candidate cleanup fsync failed for {partial_path}: {exc}",
                            )
                        )
        try:
            os.close(self.descriptor)
        except OSError as exc:
            diagnostics.append(
                DockerLogDiagnostic(
                    "cleanup",
                    f"candidate cleanup close failed for {self.relative_path}: {exc}",
                )
            )

        leftover: tuple[str, ...] = ()
        if not self.published:
            try:
                os.stat(
                    self.partial_name,
                    dir_fd=self.directory_fd,
                    follow_symlinks=False,
                )
            except FileNotFoundError:
                pass
            except OSError as exc:
                diagnostics.append(
                    DockerLogDiagnostic(
                        "cleanup",
                        f"candidate cleanup leftover inspection failed for {partial_path}: {exc}",
                    )
                )
                leftover = (partial_path,)
            else:
                leftover = (partial_path,)
        return tuple(diagnostics), leftover

    def final_is_linked(self) -> bool:
        """Return whether the final name already identifies this candidate."""
        try:
            final = os.stat(
                self.final_name,
                dir_fd=self.directory_fd,
                follow_symlinks=False,
            )
            opened = os.fstat(self.descriptor)
        except OSError:
            return False
        return _same_entry(self.identity, final) and _same_entry(self.identity, opened)


class _ObjectPairs(list[tuple[str, Any]]):
    pass


def _convert_pairs(value: Any) -> Any:
    if isinstance(value, _ObjectPairs):
        converted: dict[str, Any] = {}
        for key, item in value:
            if key in converted:
                raise ValueError(f"duplicate JSON object key: {key}")
            converted[key] = _convert_pairs(item)
        return converted
    if isinstance(value, list):
        return [_convert_pairs(item) for item in value]
    return value


def _parse_attempted_object(line: bytes) -> tuple[Mapping[str, Any] | None, str | None]:
    try:
        decoded = line.decode("utf-8")
        parsed = json.loads(
            decoded,
            object_pairs_hook=_ObjectPairs,
        )
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None, None
    except (MemoryError, RecursionError) as exc:
        return None, f"JSON parser resource failure: {exc}"
    except (OverflowError, ValueError) as exc:
        return None, f"JSON parser integer/resource failure: {exc}"
    if not isinstance(parsed, _ObjectPairs):
        return None, None
    claimed = any(key in COMMON_FIELDS for key, _value in parsed)
    if not claimed:
        return None, None
    try:
        converted = _convert_pairs(parsed)
    except (MemoryError, RecursionError) as exc:
        return None, f"JSON normalization resource failure: {exc}"
    except ValueError as exc:
        return None, str(exc)
    return converted, None


def _event_from_payload(
    payload: Mapping[str, Any],
    *,
    run_id: str,
    owning_module: str,
) -> StructuredEvent:
    if set(payload) != _EVENT_KEYS:
        missing = sorted(_EVENT_KEYS - set(payload))
        extra = sorted(set(payload) - _EVENT_KEYS)
        raise ValueError(
            f"attempted event has invalid top-level keys; missing={missing}, extra={extra}"
        )
    if payload["run_id"] != run_id:
        raise ValueError("attempted event run_id does not match capture run_id")
    module = payload["module"]
    if module not in REQUIRED_MODULES:
        raise ValueError("attempted event module is not a required module")
    if module != owning_module:
        raise ValueError("attempted event module violates explicit service ownership")
    for name in ("severity", "event"):
        if not isinstance(payload[name], str) or not payload[name]:
            raise ValueError(f"attempted event {name} must be a nonempty string")
    sim_timestamp = payload["sim_timestamp"]
    if (
        sim_timestamp is not None
        and (
            isinstance(sim_timestamp, bool)
            or not isinstance(sim_timestamp, (int, float, Decimal))
        )
    ):
        raise ValueError("attempted event sim_timestamp must be null or a finite number")
    wall_value = payload["wall_timestamp"]
    if not isinstance(wall_value, str):
        raise ValueError("attempted event wall_timestamp must be an ISO-8601 string")
    if _WALL_TIMESTAMP_PATTERN.fullmatch(wall_value) is None:
        if _TIMEZONE_MARKER_PATTERN.search(wall_value) is None:
            raise ValueError("attempted event wall_timestamp must be timezone-aware")
        raise ValueError("attempted event wall_timestamp does not match the required profile")
    try:
        wall_timestamp = datetime.fromisoformat(wall_value)
    except ValueError as exc:
        raise ValueError("attempted event wall_timestamp is invalid") from exc
    fields = payload["fields"]
    if not isinstance(fields, Mapping):
        raise ValueError("attempted event fields must be a JSON object")
    return StructuredEvent(
        run_id=payload["run_id"],
        module=module,
        severity=payload["severity"],
        event=payload["event"],
        sim_timestamp=sim_timestamp,
        wall_timestamp=wall_timestamp,
        fields=fields,
    )


@dataclass(frozen=True)
class _CapturedEvent:
    event: StructuredEvent
    encoded: bytes
    source_rank: int
    source_order: int


class DockerLogCapture:
    """Capture each service once, retain raw bytes, and publish valid events."""

    def __init__(
        self,
        run_directory: Path | str,
        project_name: str,
        ownership: Mapping[str, str] | Iterable[tuple[str, str]],
        run_id: str,
        command_runner: CommandRunner | None = None,
        *,
        host_events: bool = False,
    ) -> None:
        self.run_directory = Path(run_directory)
        self.project_name = project_name
        self.ownership = (
            tuple(ownership.items())
            if isinstance(ownership, Mapping)
            else tuple(ownership)
        )
        self.run_id = run_id
        self.command_runner = command_runner or _run_compose_logs
        self.host_events = host_events

    def _validate_inputs(self) -> None:
        if not self.run_directory.is_absolute():
            raise ValueError("run directory must be absolute")
        if (
            not isinstance(self.project_name, str)
            or _PROJECT_PATTERN.fullmatch(self.project_name) is None
        ):
            raise ValueError("project identifier is unsafe")
        if not isinstance(self.run_id, str) or not self.run_id:
            raise ValueError("run_id must be a nonempty string")
        if not isinstance(self.host_events, bool):
            raise ValueError("host_events must be a boolean")
        services: list[str] = []
        modules: list[str] = []
        for item in self.ownership:
            if not isinstance(item, tuple) or len(item) != 2:
                raise ValueError("ownership must contain service-module pairs")
            service, module = item
            if (
                not isinstance(service, str)
                or _SERVICE_PATTERN.fullmatch(service) is None
            ):
                raise ValueError("service identifier is unsafe")
            if not isinstance(module, str):
                raise ValueError("ownership module must be a string")
            services.append(service)
            modules.append(module)
        if len(set(services)) != len(services):
            raise ValueError("ownership services must be unique")
        if len(set(modules)) != len(modules):
            raise ValueError("ownership modules must be unique")
        if set(modules) != set(REQUIRED_MODULES) or len(modules) != len(REQUIRED_MODULES):
            raise ValueError("ownership must cover every required module exactly once")

    @staticmethod
    def _open_directory_at(
        parent_fd: int,
        name: str,
        *,
        create: bool,
    ) -> tuple[int, os.stat_result]:
        try:
            before = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            if not create:
                raise
            os.mkdir(name, 0o755, dir_fd=parent_fd)
            os.fsync(parent_fd)
            before = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        if stat.S_ISLNK(before.st_mode) or not stat.S_ISDIR(before.st_mode):
            raise OSError(f"{name} is not a safe directory")
        descriptor = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
        opened = os.fstat(descriptor)
        if not _same_entry(before, opened):
            os.close(descriptor)
            raise OSError(f"{name} directory changed while opening")
        return descriptor, opened

    def _open_directories(self) -> tuple[int, int, int, tuple[os.stat_result, ...]]:
        try:
            root_before = os.stat(self.run_directory, follow_symlinks=False)
            if stat.S_ISLNK(root_before.st_mode) or not stat.S_ISDIR(root_before.st_mode):
                raise OSError("run directory is not a safe directory")
            root_fd = os.open(self.run_directory, _DIRECTORY_FLAGS)
            root_opened = os.fstat(root_fd)
            if not _same_entry(root_before, root_opened):
                os.close(root_fd)
                raise OSError("run directory changed while opening")
            try:
                logs_fd, logs_identity = self._open_directory_at(
                    root_fd,
                    "logs",
                    create=True,
                )
            except BaseException:
                os.close(root_fd)
                raise
            try:
                docker_fd, docker_identity = self._open_directory_at(
                    logs_fd, "docker", create=True
                )
            except BaseException:
                os.close(logs_fd)
                os.close(root_fd)
                raise
            return (
                root_fd,
                logs_fd,
                docker_fd,
                (root_opened, logs_identity, docker_identity),
            )
        except OSError as exc:
            diagnostic = DockerLogDiagnostic("filesystem", f"run directory setup failed: {exc}")
            raise DockerLogCaptureError(
                DockerLogCaptureResult(False, (), (), (diagnostic,), (), ())
            ) from exc

    def _directories_stable(
        self,
        root_fd: int,
        logs_fd: int,
        docker_fd: int,
        identities: tuple[os.stat_result, ...],
    ) -> bool:
        try:
            path_root = os.stat(self.run_directory, follow_symlinks=False)
            named_logs = os.stat("logs", dir_fd=root_fd, follow_symlinks=False)
            named_docker = os.stat("docker", dir_fd=logs_fd, follow_symlinks=False)
            opened = (os.fstat(root_fd), os.fstat(logs_fd), os.fstat(docker_fd))
        except OSError:
            return False
        named = (path_root, named_logs, named_docker)
        return all(
            _same_entry(expected, by_name) and _same_entry(expected, by_fd)
            for expected, by_name, by_fd in zip(identities, named, opened, strict=True)
        )

    @staticmethod
    def _require_absent(directory_fd: int, name: str) -> None:
        try:
            os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        except FileNotFoundError:
            return
        except OSError as exc:
            raise OSError(f"could not inspect target {name!r}") from exc
        raise FileExistsError(f"preexisting target is forbidden: {name}")

    def _preflight_targets(self, logs_fd: int, docker_fd: int) -> None:
        for service, _module in self.ownership:
            final = f"{service}.log"
            self._require_absent(docker_fd, final)
            self._require_absent(docker_fd, f"{final}.partial")
        for module in REQUIRED_MODULES:
            final = f"{module}.jsonl"
            self._require_absent(logs_fd, final)
            self._require_absent(logs_fd, f"{final}.partial")

    @staticmethod
    def _diagnostic_bytes(value: object) -> bytes:
        if isinstance(value, bytes):
            return value
        if isinstance(value, str):
            return value.encode("utf-8", errors="surrogateescape")
        return b""

    @classmethod
    def _exception_output(cls, exc: BaseException) -> bytes:
        output = getattr(exc, "output", None)
        if output is None:
            output = getattr(exc, "stdout", None)
        stderr = getattr(exc, "stderr", None)
        return cls._diagnostic_bytes(output) + cls._diagnostic_bytes(stderr)

    def _collect_commands(
        self,
    ) -> tuple[dict[str, bytes], list[DockerLogDiagnostic], tuple[str, ...]]:
        outputs: dict[str, bytes] = {}
        diagnostics: list[DockerLogDiagnostic] = []
        failed: list[str] = []
        for service, module in self.ownership:
            command = [
                "docker",
                "compose",
                "-p",
                self.project_name,
                "logs",
                "--no-color",
                "--no-log-prefix",
                service,
            ]
            try:
                result = self.command_runner(command)
                if not isinstance(result, DockerLogCommandResult):
                    raise TypeError("command runner must return DockerLogCommandResult")
            except Exception as exc:
                outputs[service] = self._exception_output(exc)
                diagnostics.append(
                    DockerLogDiagnostic(
                        "command",
                        f"Compose log command raised {type(exc).__name__}: {exc}",
                        service,
                        module,
                    )
                )
                failed.append(service)
                continue
            outputs[service] = result.output
            if result.returncode != 0:
                diagnostics.append(
                    DockerLogDiagnostic(
                        "command",
                        f"Compose log command exited with status {result.returncode}",
                        service,
                        module,
                    )
                )
                failed.append(service)
        return outputs, diagnostics, tuple(failed)

    @staticmethod
    def _read_host_events(
        logs_fd: int,
    ) -> tuple[bytes | None, list[DockerLogDiagnostic], tuple[str, ...]]:
        name = Path(HOST_EVENT_PATH).name
        descriptor: int | None = None
        diagnostics: list[DockerLogDiagnostic] = []
        try:
            try:
                named_before = os.stat(
                    name,
                    dir_fd=logs_fd,
                    follow_symlinks=False,
                )
            except FileNotFoundError:
                diagnostics.append(
                    DockerLogDiagnostic(
                        "host_event",
                        f"requested host event source is missing: {HOST_EVENT_PATH}",
                        module="orchestration",
                    )
                )
                return None, diagnostics, ()
            except OSError as exc:
                diagnostics.append(
                    DockerLogDiagnostic(
                        "host_event",
                        f"requested host event source could not be inspected: {exc}",
                        module="orchestration",
                    )
                )
                return None, diagnostics, (HOST_EVENT_PATH,)
            if stat.S_ISLNK(named_before.st_mode) or not stat.S_ISREG(
                named_before.st_mode
            ):
                diagnostics.append(
                    DockerLogDiagnostic(
                        "host_event",
                        "requested host event source must be a regular non-symlink file",
                        module="orchestration",
                    )
                )
                return None, diagnostics, (HOST_EVENT_PATH,)
            if named_before.st_nlink != 1:
                diagnostics.append(
                    DockerLogDiagnostic(
                        "host_event",
                        "requested host event source must have exactly one hard link",
                        module="orchestration",
                    )
                )
                return None, diagnostics, (HOST_EVENT_PATH,)
            try:
                descriptor = os.open(name, _HOST_FILE_FLAGS, dir_fd=logs_fd)
            except OSError as exc:
                diagnostics.append(
                    DockerLogDiagnostic(
                        "host_event",
                        f"requested host event source could not be opened: {exc}",
                        module="orchestration",
                    )
                )
                return None, diagnostics, (HOST_EVENT_PATH,)
            try:
                opened_before = os.fstat(descriptor)
            except OSError as exc:
                diagnostics.append(
                    DockerLogDiagnostic(
                        "host_event",
                        f"requested host event source could not be verified: {exc}",
                        module="orchestration",
                    )
                )
                return None, diagnostics, (HOST_EVENT_PATH,)
            if not _same_file_snapshot(named_before, opened_before):
                diagnostics.append(
                    DockerLogDiagnostic(
                        "host_event",
                        "requested host event source changed while opening",
                        module="orchestration",
                    )
                )
                return None, diagnostics, (HOST_EVENT_PATH,)
            chunks: list[bytes] = []
            try:
                while chunk := os.read(descriptor, 1024 * 1024):
                    chunks.append(chunk)
            except OSError as exc:
                diagnostics.append(
                    DockerLogDiagnostic(
                        "host_event",
                        f"requested host event source could not be read: {exc}",
                        module="orchestration",
                    )
                )
                return None, diagnostics, (HOST_EVENT_PATH,)
            try:
                opened_after = os.fstat(descriptor)
            except OSError as exc:
                diagnostics.append(
                    DockerLogDiagnostic(
                        "host_event",
                        f"requested host event source could not be reverified: {exc}",
                        module="orchestration",
                    )
                )
                return None, diagnostics, (HOST_EVENT_PATH,)
            try:
                named_after = os.stat(
                    name,
                    dir_fd=logs_fd,
                    follow_symlinks=False,
                )
            except OSError:
                named_after = None
            if (
                named_after is None
                or not _same_file_snapshot(opened_before, opened_after)
                or not _same_file_snapshot(opened_after, named_after)
            ):
                diagnostics.append(
                    DockerLogDiagnostic(
                        "host_event",
                        "requested host event source changed during capture",
                        module="orchestration",
                    )
                )
                return None, diagnostics, (HOST_EVENT_PATH,)
            return b"".join(chunks), diagnostics, (HOST_EVENT_PATH,)
        finally:
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError as exc:
                    diagnostics.append(
                        DockerLogDiagnostic(
                            "cleanup",
                            f"host event descriptor cleanup close failed: {exc}",
                            module="orchestration",
                        )
                    )

    def _validated_line(
        self,
        line: bytes,
        *,
        service: str | None,
        owning_module: str,
        line_number: int,
        source_rank: int,
        strict: bool,
    ) -> tuple[_CapturedEvent | None, DockerLogDiagnostic | None]:
        payload, parse_error = _parse_attempted_object(line)
        if parse_error is not None:
            return None, DockerLogDiagnostic(
                "host_event" if strict else "event",
                parse_error,
                service,
                owning_module,
                line_number,
            )
        if payload is None:
            if strict:
                return None, DockerLogDiagnostic(
                    "host_event",
                    "host event source contains a blank, non-JSON, or unclaimed line",
                    module=owning_module,
                    line_number=line_number,
                )
            return None, None
        try:
            event = _event_from_payload(
                payload,
                run_id=self.run_id,
                owning_module=owning_module,
            )
            encoded = event.to_json_line().encode("utf-8")
        except (TypeError, ValueError, OverflowError) as exc:
            return None, DockerLogDiagnostic(
                "host_event" if strict else "event",
                str(exc),
                service,
                owning_module,
                line_number,
            )
        return (
            _CapturedEvent(event, encoded, source_rank, line_number),
            None,
        )

    def _partition(
        self,
        outputs: Mapping[str, bytes],
        host_output: bytes | None,
    ) -> tuple[dict[str, bytes], list[DockerLogDiagnostic], tuple[str, ...]]:
        routed: dict[str, list[_CapturedEvent]] = {
            module: [] for module in REQUIRED_MODULES
        }
        diagnostics: list[DockerLogDiagnostic] = []
        for service, owning_module in self.ownership:
            for line_number, line in enumerate(outputs[service].split(b"\n"), start=1):
                if line == b"":
                    continue
                captured, diagnostic = self._validated_line(
                    line,
                    service=service,
                    owning_module=owning_module,
                    line_number=line_number,
                    source_rank=0,
                    strict=False,
                )
                if diagnostic is not None:
                    diagnostics.append(diagnostic)
                if captured is not None:
                    routed[captured.event.module].append(captured)

        if host_output is not None:
            host_lines = [] if host_output == b"" else host_output.split(b"\n")
            if host_lines and host_lines[-1] == b"":
                host_lines.pop()
            for line_number, line in enumerate(host_lines, start=1):
                captured, diagnostic = self._validated_line(
                    line,
                    service=None,
                    owning_module="orchestration",
                    line_number=line_number,
                    source_rank=1,
                    strict=True,
                )
                if diagnostic is not None:
                    diagnostics.append(diagnostic)
                if captured is not None:
                    routed["orchestration"].append(captured)

        missing = tuple(module for module in REQUIRED_MODULES if not routed[module])
        if missing:
            diagnostics.append(
                DockerLogDiagnostic(
                    "coverage",
                    f"missing required structured module streams: {', '.join(missing)}",
                )
            )
        if host_output is not None:
            routed["orchestration"].sort(
                key=lambda captured: (
                    captured.event.wall_timestamp.astimezone(timezone.utc),
                    captured.source_rank,
                    captured.source_order,
                )
            )
        return (
            {
                module: b"".join(captured.encoded for captured in events)
                for module, events in routed.items()
            },
            diagnostics,
            missing,
        )

    @staticmethod
    def _result(
        succeeded: bool,
        raw_paths: Sequence[str],
        structured_paths: Sequence[str],
        diagnostics: Sequence[DockerLogDiagnostic],
        command_failures: Sequence[str],
        missing_modules: Sequence[str],
        recovery_paths: Sequence[str] = (),
        leftover_partials: Sequence[str] = (),
    ) -> DockerLogCaptureResult:
        return DockerLogCaptureResult(
            succeeded,
            tuple(raw_paths),
            tuple(structured_paths),
            tuple(diagnostics),
            tuple(command_failures),
            tuple(missing_modules),
            tuple(recovery_paths),
            tuple(leftover_partials),
        )

    def capture(self) -> DockerLogCaptureResult:
        """Capture all raw streams, then publish structured logs only if valid."""
        self._validate_inputs()
        root_fd, logs_fd, docker_fd, identities = self._open_directories()
        candidates: list[_Candidate] = []
        raw_paths: list[str] = []
        structured_paths: list[str] = []
        diagnostics: list[DockerLogDiagnostic] = []
        command_failures: tuple[str, ...] = ()
        missing_modules: tuple[str, ...] = ()
        recovery_paths: tuple[str, ...] = ()
        result: DockerLogCaptureResult | None = None
        capture_error: DockerLogCaptureError | None = None
        cleanup_diagnostics: list[DockerLogDiagnostic] = []
        leftover_partials: list[str] = []
        try:
            try:
                self._preflight_targets(logs_fd, docker_fd)
            except OSError as exc:
                diagnostic = DockerLogDiagnostic("filesystem", str(exc))
                raise DockerLogCaptureError(
                    self._result(False, (), (), (diagnostic,), (), ())
                ) from exc

            outputs, command_diagnostics, command_failures = self._collect_commands()
            diagnostics.extend(command_diagnostics)

            for service, _module in self.ownership:
                relative_path = f"logs/docker/{service}.log"
                try:
                    candidate = _Candidate.create(
                        docker_fd,
                        f"{service}.log",
                        outputs[service],
                        relative_path,
                    )
                except OSError as exc:
                    diagnostics.append(
                        DockerLogDiagnostic(
                            "publication",
                            f"raw candidate creation failed: {exc}",
                            service,
                        )
                    )
                    raise DockerLogCaptureError(
                        self._result(
                            False,
                            raw_paths,
                            structured_paths,
                            diagnostics,
                            command_failures,
                            missing_modules,
                        )
                    ) from exc
                candidates.append(candidate)

            host_output: bytes | None = None
            host_diagnostics: list[DockerLogDiagnostic] = []
            if self.host_events:
                host_output, host_diagnostics, recovery_paths = self._read_host_events(
                    logs_fd
                )
                diagnostics.extend(host_diagnostics)

            routed, event_diagnostics, missing_modules = self._partition(
                outputs,
                host_output,
            )
            diagnostics.extend(event_diagnostics)

            raw_publication_failed = False
            for candidate in tuple(candidates):
                try:
                    if not self._directories_stable(
                        root_fd, logs_fd, docker_fd, identities
                    ):
                        raise OSError("retained run directory chain changed")
                    candidate.publish()
                    raw_paths.append(candidate.relative_path)
                except OSError as exc:
                    raw_publication_failed = True
                    if candidate.final_is_linked():
                        raw_paths.append(candidate.relative_path)
                    service = Path(candidate.relative_path).stem
                    diagnostics.append(
                        DockerLogDiagnostic(
                            "publication",
                            f"raw publication failed: {exc}",
                            service,
                        )
                    )

            if (
                command_failures
                or host_diagnostics
                or event_diagnostics
                or missing_modules
                or raw_publication_failed
                or len(raw_paths) != len(self.ownership)
            ):
                raise DockerLogCaptureError(
                    self._result(
                        False,
                        raw_paths,
                        structured_paths,
                        diagnostics,
                        command_failures,
                        missing_modules,
                        recovery_paths,
                    )
                )

            structured_candidates: list[_Candidate] = []
            for module in REQUIRED_MODULES:
                relative_path = f"logs/{module}.jsonl"
                try:
                    candidate = _Candidate.create(
                        logs_fd,
                        f"{module}.jsonl",
                        routed[module],
                        relative_path,
                    )
                except OSError as exc:
                    diagnostics.append(
                        DockerLogDiagnostic(
                            "publication",
                            f"structured candidate creation failed: {exc}",
                            module=module,
                        )
                    )
                    raise DockerLogCaptureError(
                        self._result(
                            False,
                            raw_paths,
                            structured_paths,
                            diagnostics,
                            command_failures,
                            missing_modules,
                            recovery_paths,
                        )
                    ) from exc
                candidates.append(candidate)
                structured_candidates.append(candidate)

            for candidate in structured_candidates:
                try:
                    if not self._directories_stable(
                        root_fd, logs_fd, docker_fd, identities
                    ):
                        raise OSError("retained run directory chain changed")
                    candidate.publish()
                    structured_paths.append(candidate.relative_path)
                except OSError as exc:
                    if candidate.final_is_linked():
                        structured_paths.append(candidate.relative_path)
                    diagnostics.append(
                        DockerLogDiagnostic(
                            "publication",
                            f"structured publication failed: {exc}",
                            module=Path(candidate.relative_path).stem,
                        )
                    )
                    raise DockerLogCaptureError(
                        self._result(
                            False,
                            raw_paths,
                            structured_paths,
                            diagnostics,
                            command_failures,
                            missing_modules,
                            recovery_paths,
                        )
                    ) from exc

            result = self._result(
                True,
                raw_paths,
                structured_paths,
                diagnostics,
                command_failures,
                missing_modules,
                recovery_paths,
            )
        except DockerLogCaptureError as exc:
            capture_error = exc
            result = exc.result
        finally:
            for candidate in candidates:
                candidate_diagnostics, candidate_leftovers = (
                    candidate.close_and_clean()
                )
                cleanup_diagnostics.extend(candidate_diagnostics)
                leftover_partials.extend(candidate_leftovers)
            for descriptor, label in (
                (docker_fd, "Docker log directory"),
                (logs_fd, "logs directory"),
                (root_fd, "run directory"),
            ):
                try:
                    os.close(descriptor)
                except OSError as exc:
                    cleanup_diagnostics.append(
                        DockerLogDiagnostic(
                            "cleanup",
                            f"{label} cleanup close failed: {exc}",
                        )
                    )

        if result is None:
            raise RuntimeError("capture produced no result")
        if cleanup_diagnostics or leftover_partials:
            result = replace(
                result,
                succeeded=False,
                diagnostics=result.diagnostics + tuple(cleanup_diagnostics),
                leftover_partials=tuple(dict.fromkeys(leftover_partials)),
            )
            raise DockerLogCaptureError(result) from capture_error
        if capture_error is not None:
            raise DockerLogCaptureError(result) from capture_error
        return result


__all__ = [
    "DockerLogCapture",
    "DockerLogCaptureError",
    "DockerLogCaptureResult",
    "DockerLogCommandResult",
    "DockerLogDiagnostic",
    "REQUIRED_MODULES",
]
