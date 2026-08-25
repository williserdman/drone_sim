"""Descriptor-safe durable files shared by the Phase 2 runtime processes."""

from __future__ import annotations

from collections.abc import Mapping
import fcntl
import json
import os
from pathlib import Path, PurePosixPath
import stat
from typing import Any
from uuid import UUID, uuid4


_DIRECTORY_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_DIRECTORY", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_FILE_FLAGS = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
_MAX_BYTES = 4 * 1024 * 1024
_TERMINAL_STATES = frozenset({"COMPLETED", "FAILED", "ABORTED"})
_QUIESCENCE_MODULES = frozenset(
    {"orchestration", "companion", "ardupilot_sitl", "gazebo", "electromagnet", "scorekeeper"}
)
_STATUS_NAMES = frozenset(
    {
        "artifacts-ready",
        "gazebo-ready",
        "ardupilot-ready",
        "companion-ready",
        "runtime-running",
        "source-finished",
        "mission-finished",
        "score-finished",
        "runtime-failure",
        "runtime-frozen",
        "artifacts-final",
        "terminal-notified",
    }
)
_FLIGHT_EXCHANGE_KEYS = frozenset(
    {
        "online",
        "servo_packets_received",
        "motor_updates",
        "duplicate_servo_packets",
        "servo_frame_gaps",
        "json_states_sent",
        "json_send_errors",
        "last_servo_frame",
        "last_json_sim_time_ns",
    }
)


class ProtocolError(RuntimeError):
    """A runtime protocol path or document violates the frozen contract."""


def canonical_run_id(value: str) -> str:
    if not isinstance(value, str):
        raise ValueError("run_id must be a canonical UUID")
    try:
        parsed = UUID(value)
    except (ValueError, TypeError, AttributeError) as error:
        raise ValueError("run_id must be a canonical UUID") from error
    if str(parsed) != value:
        raise ValueError("run_id must be a canonical UUID")
    return value


def _duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def _invalid_constant(value: str) -> None:
    raise ValueError(f"non-standard JSON constant: {value}")


def _canonical_json(document: Mapping[str, Any]) -> bytes:
    try:
        return (
            json.dumps(
                dict(document),
                allow_nan=False,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError, OverflowError, RecursionError) as error:
        raise ProtocolError(f"protocol document is not valid JSON: {error}") from error


def _same_snapshot(first: os.stat_result, second: os.stat_result) -> bool:
    return all(
        getattr(first, field) == getattr(second, field)
        for field in (
            "st_dev",
            "st_ino",
            "st_mode",
            "st_nlink",
            "st_size",
            "st_mtime_ns",
            "st_ctime_ns",
        )
    )


def _safe_relative_path(value: Any) -> bool:
    if not isinstance(value, str) or not value or "\\" in value:
        return False
    path = PurePosixPath(value)
    return not path.is_absolute() and path.parts not in ((), (".",)) and ".." not in path.parts


def _nonnegative_integer(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _valid_flight_exchange(value: Any) -> bool:
    return bool(
        isinstance(value, dict)
        and set(value) == _FLIGHT_EXCHANGE_KEYS
        and value["online"] is True
        and all(
            type(value[key]) is int and value[key] >= 0
            for key in _FLIGHT_EXCHANGE_KEYS - {"online"}
        )
        and value["servo_packets_received"] >= 2
        and value["motor_updates"] >= 2
        and value["json_states_sent"] >= 1
        and value["servo_frame_gaps"] == 0
        and value["json_send_errors"] == 0
    )


def _validate_status(name: str, document: Mapping[str, Any], run_id: str) -> None:
    if name not in _STATUS_NAMES:
        raise ValueError("runtime status name is not part of the frozen protocol")
    if not isinstance(document, dict) or document.get("run_id") != run_id:
        raise ProtocolError("runtime status has the wrong run_id")
    if name == "artifacts-ready":
        valid = set(document) == {"run_id", "ready"} and document["ready"] is True
    elif name == "gazebo-ready":
        valid = (
            set(document) == {"run_id", "ready", "flight_exchange"}
            and document["ready"] is True
            and _valid_flight_exchange(document["flight_exchange"])
        )
    elif name == "ardupilot-ready":
        valid = (
            set(document)
            == {"run_id", "ready", "json_exchange", "mavlink_endpoint"}
            and document["ready"] is True
            and document["json_exchange"] is True
            and document["mavlink_endpoint"] == "tcp://ardupilot-sitl:5760"
        )
    elif name == "companion-ready":
        valid = (
            set(document)
            == {
                "run_id",
                "ready",
                "mavlink_endpoint",
                "mavlink_transport_connected",
            }
            and document["ready"] is True
            and document["mavlink_endpoint"] == "tcp://ardupilot-sitl:5760"
            and document["mavlink_transport_connected"] is True
        )
    elif name == "runtime-running":
        valid = (
            set(document) == {"run_id", "state", "sim_timestamp_ns"}
            and document["state"] == "RUNNING"
            and _nonnegative_integer(document["sim_timestamp_ns"])
        )
    elif name == "source-finished":
        valid = (
            set(document) == {"run_id", "finished", "sim_timestamp_ns"}
            and document["finished"] is True
            and _nonnegative_integer(document["sim_timestamp_ns"])
        )
    elif name == "mission-finished":
        valid = (
            set(document) == {
                "run_id",
                "finished",
                "sim_timestamp_ns",
                "outcome",
            }
            and document["finished"] is True
            and _nonnegative_integer(document["sim_timestamp_ns"])
            and document["outcome"] == "LANDED"
        )
    elif name == "score-finished":
        valid = (
            set(document) == {"run_id", "finished", "sim_timestamp_ns"}
            and document["finished"] is True
            and _nonnegative_integer(document["sim_timestamp_ns"])
        )
    elif name == "runtime-failure":
        paths = document.get("diagnostic_paths")
        valid = (
            set(document) == {"run_id", "module", "reason", "diagnostic_paths"}
            and isinstance(document.get("module"), str)
            and bool(document["module"])
            and isinstance(document.get("reason"), str)
            and bool(document["reason"])
            and isinstance(paths, list)
            and all(isinstance(path, str) for path in paths)
            and len(paths) == len(set(paths))
            and all(_safe_relative_path(path) for path in paths)
        )
    elif name == "runtime-frozen":
        valid = set(document) == {"run_id", "frozen"} and document["frozen"] is True
    elif name == "terminal-notified":
        valid = set(document) == {"run_id", "notified"} and document["notified"] is True
    else:
        valid = _valid_artifacts_final(document, run_id)
    if not valid:
        raise ProtocolError(f"runtime status {name!r} has an invalid schema")


def _valid_artifacts_final(document: Mapping[str, Any], run_id: str) -> bool:
    if set(document) != {"run_id", "complete", "records"}:
        return False
    if document.get("run_id") != run_id or not isinstance(document.get("complete"), bool):
        return False
    records = document.get("records")
    required = ("video/onboard.mp4", "video/observer.mp4", "rosbag")
    if not isinstance(records, list) or len(records) != 3:
        return False
    paths: list[str] = []
    for record in records:
        if not isinstance(record, dict) or set(record) != {
            "relative_path", "status", "detail", "size_bytes", "sha256", "semantic"
        }:
            return False
        paths.append(record.get("relative_path"))
        if record.get("status") not in {"valid", "missing", "invalid"}:
            return False
        if not isinstance(record.get("detail"), str):
            return False
        if record.get("size_bytes") is not None and not _nonnegative_integer(record["size_bytes"]):
            return False
        digest = record.get("sha256")
        if digest is not None and (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            return False
        if not isinstance(record.get("semantic"), dict) or not record["semantic"]:
            return False
    return tuple(paths) == required and document["complete"] == all(
        record["status"] == "valid" for record in records
    )


def _validate_control(name: str, document: Mapping[str, Any], run_id: str) -> None:
    if not isinstance(document, dict) or document.get("run_id") != run_id:
        raise ProtocolError(f"{name} has the wrong run_id")
    if name == "finalize-request":
        valid = (
            set(document) == {"run_id", "requested_terminal", "reason"}
            and document.get("requested_terminal") in _TERMINAL_STATES
            and isinstance(document.get("reason"), str)
            and bool(document["reason"])
        )
    elif name == "terminal-committed":
        valid = (
            set(document) == {"run_id", "terminal_status", "reason", "manifest_path"}
            and document.get("terminal_status") in _TERMINAL_STATES
            and isinstance(document.get("reason"), str)
            and document.get("manifest_path") == "manifest.json"
        )
    else:
        raise ValueError("unknown host control")
    if not valid:
        raise ProtocolError(f"{name} has an invalid schema")


class RuntimeProtocol:
    """Policy-free access to one already allocated run's durable protocol."""

    def __init__(self, run_directory: Path | str, run_id: str) -> None:
        self.run_id = canonical_run_id(run_id)
        self.run_directory = Path(run_directory)
        try:
            before = self.run_directory.lstat()
            descriptor = os.open(self.run_directory, _DIRECTORY_FLAGS)
            opened = os.fstat(descriptor)
        except OSError as error:
            raise ProtocolError(f"run directory is unsafe: {error}") from error
        if not stat.S_ISDIR(before.st_mode) or (before.st_dev, before.st_ino) != (
            opened.st_dev,
            opened.st_ino,
        ):
            os.close(descriptor)
            raise ProtocolError("run directory is unsafe")
        self._run_fd = descriptor
        self._observed_controls: dict[str, bytes] = {}

    def close(self) -> None:
        descriptor = getattr(self, "_run_fd", None)
        if descriptor is not None:
            os.close(descriptor)
            self._run_fd = None

    def __enter__(self) -> "RuntimeProtocol":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()

    def _open_directory(self, name: str) -> int:
        return self._open_directory_at(self._run_fd, name)

    @staticmethod
    def _open_directory_at(parent_fd: int, name: str) -> int:
        try:
            before = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
            descriptor = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
            opened = os.fstat(descriptor)
        except OSError as error:
            raise ProtocolError(f"protocol directory {name!r} is unsafe: {error}") from error
        if not stat.S_ISDIR(before.st_mode) or (before.st_dev, before.st_ino) != (
            opened.st_dev,
            opened.st_ino,
        ):
            os.close(descriptor)
            raise ProtocolError(f"protocol directory {name!r} is unsafe")
        return descriptor

    @staticmethod
    def _inspect(directory_fd: int, name: str) -> os.stat_result | None:
        try:
            metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        except FileNotFoundError:
            return None
        except OSError as error:
            raise ProtocolError(f"could not inspect protocol file {name!r}: {error}") from error
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise ProtocolError(f"protocol file {name!r} must be regular with one link")
        return metadata

    @classmethod
    def _read_at(cls, directory_fd: int, name: str) -> dict[str, Any] | None:
        before = cls._inspect(directory_fd, name)
        if before is None:
            return None
        descriptor: int | None = None
        try:
            descriptor = os.open(name, _FILE_FLAGS, dir_fd=directory_fd)
            opened = os.fstat(descriptor)
            if not _same_snapshot(before, opened):
                raise ProtocolError(f"protocol file {name!r} changed while opening")
            if opened.st_size > _MAX_BYTES:
                raise ProtocolError(f"protocol file {name!r} is too large")
            chunks: list[bytes] = []
            remaining = _MAX_BYTES + 1
            while remaining:
                chunk = os.read(descriptor, min(65536, remaining))
                if not chunk:
                    break
                chunks.append(chunk)
                remaining -= len(chunk)
            if sum(map(len, chunks)) > _MAX_BYTES:
                raise ProtocolError(f"protocol file {name!r} is too large")
            after = os.fstat(descriptor)
            named_after = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
            if not _same_snapshot(opened, after) or not _same_snapshot(after, named_after):
                raise ProtocolError(f"protocol file {name!r} changed while reading")
            try:
                document = json.loads(
                    b"".join(chunks).decode("utf-8"),
                    object_pairs_hook=_duplicate_pairs,
                    parse_constant=_invalid_constant,
                )
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError, RecursionError) as error:
                raise ProtocolError(f"protocol file {name!r} contains invalid JSON: {error}") from error
            if not isinstance(document, dict):
                raise ProtocolError(f"protocol file {name!r} must contain an object")
            return document
        except ProtocolError:
            raise
        except OSError as error:
            raise ProtocolError(f"could not read protocol file {name!r}: {error}") from error
        finally:
            if descriptor is not None:
                os.close(descriptor)

    @classmethod
    def _write_at(
        cls, directory_fd: int, name: str, document: Mapping[str, Any]
    ) -> bool:
        payload = _canonical_json(document)
        temporary = f".{name}.{uuid4().hex}.tmp"
        descriptor: int | None = None
        fcntl.flock(directory_fd, fcntl.LOCK_EX)
        try:
            existing = cls._read_at(directory_fd, name)
            if existing is not None:
                if existing == dict(document):
                    return False
                raise ProtocolError(f"protocol file {name!r} conflicts with existing value")
            descriptor = os.open(
                temporary,
                os.O_WRONLY
                | os.O_CREAT
                | os.O_EXCL
                | getattr(os, "O_CLOEXEC", 0)
                | getattr(os, "O_NOFOLLOW", 0),
                # Runtime containers normally run as root while the host
                # controller does not.  Status documents contain no secrets
                # and must cross that ownership boundary through the bind
                # mount; controls remain host-owned and are only read here.
                0o644,
                dir_fd=directory_fd,
            )
            os.fchmod(descriptor, 0o644)
            written = 0
            while written < len(payload):
                count = os.write(descriptor, payload[written:])
                if count <= 0:
                    raise OSError("protocol write made no progress")
                written += count
            os.fsync(descriptor)
            os.close(descriptor)
            descriptor = None
            os.replace(temporary, name, src_dir_fd=directory_fd, dst_dir_fd=directory_fd)
            os.fsync(directory_fd)
            return True
        except ProtocolError:
            raise
        except OSError as error:
            raise ProtocolError(f"could not persist protocol file {name!r}: {error}") from error
        finally:
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
            try:
                os.unlink(temporary, dir_fd=directory_fd)
            except OSError:
                pass
            fcntl.flock(directory_fd, fcntl.LOCK_UN)

    def write_status(self, name: str, document: Mapping[str, Any]) -> Path:
        _validate_status(name, document, self.run_id)
        status_fd = self._open_directory(".status")
        try:
            self._write_at(status_fd, f"{name}.json", document)
        finally:
            os.close(status_fd)
        return self.run_directory / ".status" / f"{name}.json"

    def read_status(self, name: str) -> dict[str, Any] | None:
        if name not in _STATUS_NAMES:
            raise ValueError("runtime status name is not part of the frozen protocol")
        status_fd = self._open_directory(".status")
        try:
            document = self._read_at(status_fd, f"{name}.json")
        finally:
            os.close(status_fd)
        if document is not None:
            _validate_status(name, document, self.run_id)
        return document

    @staticmethod
    def _validate_quiescence_module(module: str) -> None:
        if module not in _QUIESCENCE_MODULES:
            raise ValueError("quiescence module is not part of the frozen protocol")

    def write_quiescence(self, module: str) -> Path:
        self._validate_quiescence_module(module)
        document = {"run_id": self.run_id, "module": module, "quiescent": True}
        status_fd = self._open_directory(".status")
        try:
            quiescence_fd = self._open_directory_at(status_fd, "quiescence")
            try:
                self._write_at(quiescence_fd, f"{module}.json", document)
            finally:
                os.close(quiescence_fd)
        finally:
            os.close(status_fd)
        return self.run_directory / ".status" / "quiescence" / f"{module}.json"

    def read_quiescence(self, module: str) -> dict[str, Any] | None:
        self._validate_quiescence_module(module)
        status_fd = self._open_directory(".status")
        try:
            quiescence_fd = self._open_directory_at(status_fd, "quiescence")
            try:
                document = self._read_at(quiescence_fd, f"{module}.json")
            finally:
                os.close(quiescence_fd)
        finally:
            os.close(status_fd)
        if document is not None and document != {
            "run_id": self.run_id,
            "module": module,
            "quiescent": True,
        }:
            raise ProtocolError(f"quiescence marker for {module!r} has an invalid schema")
        return document

    def _read_control(self, name: str) -> dict[str, Any] | None:
        control_fd = self._open_directory(".control")
        try:
            document = self._read_at(control_fd, f"{name}.json")
        finally:
            os.close(control_fd)
        if document is None:
            return None
        _validate_control(name, document, self.run_id)
        encoded = _canonical_json(document)
        previous = self._observed_controls.setdefault(name, encoded)
        if previous != encoded:
            raise ProtocolError(f"host control {name!r} changed after first observation")
        return document

    def read_finalize_request(self) -> dict[str, Any] | None:
        return self._read_control("finalize-request")

    def read_terminal_committed(self) -> dict[str, Any] | None:
        return self._read_control("terminal-committed")

    def read_manifest_status(self) -> dict[str, Any]:
        """Read only the final live-status facts from immutable manifest authority."""
        document = self._read_at(self._run_fd, "manifest.json")
        if document is None:
            raise ProtocolError("manifest.json is missing after terminal commit")
        required = {
            "schema_version",
            "run_id",
            "terminal_status",
            "artifacts",
            "incomplete_paths",
        }
        if not required.issubset(document) or document.get("schema_version") != 1:
            raise ProtocolError("manifest status schema is invalid")
        if document.get("run_id") != self.run_id:
            raise ProtocolError("manifest status has the wrong run_id")
        if document.get("terminal_status") not in _TERMINAL_STATES:
            raise ProtocolError("manifest status has an invalid terminal state")
        artifacts = document.get("artifacts")
        incomplete = document.get("incomplete_paths")
        if not isinstance(artifacts, list) or not isinstance(incomplete, list):
            raise ProtocolError("manifest status inventory is invalid")
        validations: dict[str, str] = {}
        for record in artifacts:
            if not isinstance(record, dict):
                raise ProtocolError("manifest status artifact record is invalid")
            relative_path = record.get("relative_path")
            validation = record.get("validation")
            if (
                not _safe_relative_path(relative_path)
                or relative_path in validations
                or validation not in {"valid", "missing", "invalid"}
            ):
                raise ProtocolError("manifest status artifact record is invalid")
            validations[relative_path] = validation
        if (
            any(not _safe_relative_path(path) for path in incomplete)
            or len(incomplete) != len(set(incomplete))
            or any(validations.get(path) not in {"missing", "invalid"} for path in incomplete)
        ):
            raise ProtocolError("manifest incomplete paths are invalid")
        return {
            "run_id": self.run_id,
            "complete": not incomplete,
            "missing": sorted(incomplete),
            "manifest_path": "manifest.json",
        }


__all__ = ["ProtocolError", "RuntimeProtocol", "canonical_run_id"]
