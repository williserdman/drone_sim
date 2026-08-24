"""Durable host-side run allocation and control/status protocol."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import stat
from typing import Any, Callable, Mapping
from uuid import UUID, uuid4

from artifacts import validate_regular_file, validate_tree
from artifacts.manifest import (
    ArtifactRecord,
    ConfigurationRecord,
    ImageDigest,
    REQUIRED_ARTIFACT_PATHS,
    RunManifest,
    SimulationTiming,
    SourceRevision,
    WallTiming,
    validate_manifest,
)


_DIRECTORY_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_DIRECTORY", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_FILE_FLAGS = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
_RUNTIME_STATUS_NAMES = frozenset(
    {
        "artifacts-ready",
        "runtime-running",
        "source-finished",
        "runtime-failure",
        "runtime-frozen",
        "artifacts-final",
        "terminal-notified",
    }
)
_LIFECYCLE_STATES = frozenset(
    {"CREATED", "STARTING", "READY", "RUNNING", "FINALIZING", "COMPLETED", "FAILED", "ABORTED"}
)
_TERMINAL_STATES = frozenset({"COMPLETED", "FAILED", "ABORTED"})
_MAX_PROTOCOL_BYTES = 4 * 1024 * 1024


class ProtocolFileError(RuntimeError):
    """A durable protocol path or document violates its frozen contract."""


@dataclass(frozen=True)
class TerminalCause:
    kind: str
    reason: str
    module: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.kind, str) or not self.kind:
            raise ValueError("terminal cause kind must be nonempty")
        if not isinstance(self.reason, str) or not self.reason:
            raise ValueError("terminal cause reason must be nonempty")
        if self.module is not None and (not isinstance(self.module, str) or not self.module):
            raise ValueError("terminal cause module must be null or nonempty")

    def to_dict(self) -> dict[str, str | None]:
        return {"kind": self.kind, "reason": self.reason, "module": self.module}

    @classmethod
    def from_dict(cls, document: Any) -> "TerminalCause":
        if not isinstance(document, dict) or set(document) != {"kind", "reason", "module"}:
            raise ProtocolFileError("terminal cause has invalid keys")
        try:
            return cls(document["kind"], document["reason"], document["module"])
        except (TypeError, ValueError) as exc:
            raise ProtocolFileError(f"invalid terminal cause: {exc}") from exc


@dataclass(frozen=True)
class OperatorStatus:
    run_id: str
    state: str
    reason: str
    manifest_path: str | None
    primary_cause: TerminalCause | None = None
    diagnostics: tuple[TerminalCause, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        _canonical_run_id(self.run_id)
        if self.state not in _LIFECYCLE_STATES:
            raise ValueError("operator state is invalid")
        if not isinstance(self.reason, str):
            raise ValueError("operator reason must be a string")
        if self.manifest_path not in (None, "manifest.json"):
            raise ValueError("operator manifest_path must be null or manifest.json")
        if self.primary_cause is not None and not isinstance(self.primary_cause, TerminalCause):
            raise TypeError("primary_cause must be a TerminalCause or None")
        if any(not isinstance(value, TerminalCause) for value in self.diagnostics):
            raise TypeError("diagnostics must contain TerminalCause values")

    def to_dict(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "state": self.state,
            "reason": self.reason,
            "manifest_path": self.manifest_path,
            "primary_cause": None if self.primary_cause is None else self.primary_cause.to_dict(),
            "diagnostics": [item.to_dict() for item in self.diagnostics],
        }

    @classmethod
    def from_dict(cls, document: Any) -> "OperatorStatus":
        expected = {"run_id", "state", "reason", "manifest_path", "primary_cause", "diagnostics"}
        if not isinstance(document, dict) or set(document) != expected:
            raise ProtocolFileError("operator state has invalid keys")
        diagnostics = document["diagnostics"]
        if not isinstance(diagnostics, list):
            raise ProtocolFileError("operator diagnostics must be an array")
        try:
            return cls(
                document["run_id"],
                document["state"],
                document["reason"],
                document["manifest_path"],
                None
                if document["primary_cause"] is None
                else TerminalCause.from_dict(document["primary_cause"]),
                tuple(TerminalCause.from_dict(value) for value in diagnostics),
            )
        except (TypeError, ValueError) as exc:
            raise ProtocolFileError(f"invalid operator state: {exc}") from exc


def _canonical_run_id(run_id: str) -> str:
    if not isinstance(run_id, str):
        raise ValueError("run_id must be a canonical UUID")
    try:
        parsed = UUID(run_id)
    except (ValueError, AttributeError) as exc:
        raise ValueError("run_id must be a canonical UUID") from exc
    canonical = str(parsed)
    if run_id != canonical:
        raise ValueError("run_id must be a canonical UUID")
    return canonical


def _reject_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    document: dict[str, Any] = {}
    for key, value in pairs:
        if key in document:
            raise ValueError(f"duplicate JSON key: {key}")
        document[key] = value
    return document


def _reject_json_constant(value: str) -> None:
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
    except (TypeError, ValueError, OverflowError) as exc:
        raise ProtocolFileError(f"protocol document is not valid JSON: {exc}") from exc


class StatusStore:
    """Own only host allocation, operator state, and control documents."""

    def __init__(self, output_root: Path | str) -> None:
        self.output_root = Path(output_root)
        if not self.output_root.is_absolute():
            raise ValueError("output root must be absolute")
        if ".." in self.output_root.parts:
            raise ValueError("output root must not contain parent traversal")

    def _open_output_root(self, *, create: bool) -> int:
        current_fd: int | None = None
        try:
            current_fd = os.open(self.output_root.anchor, _DIRECTORY_FLAGS)
            for part in self.output_root.parts[1:]:
                try:
                    metadata = os.stat(
                        part, dir_fd=current_fd, follow_symlinks=False
                    )
                except FileNotFoundError:
                    if not create:
                        raise ProtocolFileError("output root does not exist") from None
                    os.mkdir(part, 0o755, dir_fd=current_fd)
                    os.fsync(current_fd)
                    metadata = os.stat(
                        part, dir_fd=current_fd, follow_symlinks=False
                    )
                if stat.S_ISLNK(metadata.st_mode):
                    raise ProtocolFileError(
                        "output root must not traverse symlink components"
                    )
                if not stat.S_ISDIR(metadata.st_mode):
                    raise ProtocolFileError(
                        "output root must be a non-symlink directory"
                    )
                next_fd = os.open(part, _DIRECTORY_FLAGS, dir_fd=current_fd)
                opened = os.fstat(next_fd)
                if (metadata.st_dev, metadata.st_ino) != (
                    opened.st_dev,
                    opened.st_ino,
                ):
                    os.close(next_fd)
                    raise ProtocolFileError("output root changed while opening")
                os.close(current_fd)
                current_fd = next_fd
            descriptor = current_fd
            current_fd = None
            return descriptor
        except ProtocolFileError:
            raise
        except OSError as exc:
            raise ProtocolFileError(f"output root is unsafe: {exc}") from exc
        finally:
            if current_fd is not None:
                os.close(current_fd)

    def run_directory(self, run_id: str) -> Path:
        canonical = _canonical_run_id(run_id)
        candidate = self.output_root / canonical
        if candidate.parent != self.output_root:
            raise ValueError("run directory escapes output root")
        return candidate

    def allocate(self, run_id: str) -> Path:
        canonical = _canonical_run_id(run_id)
        root_fd = self._open_output_root(create=True)
        run_fd: int | None = None
        try:
            os.mkdir(canonical, 0o755, dir_fd=root_fd)
            os.chmod(canonical, 0o755, dir_fd=root_fd, follow_symlinks=False)
            os.fsync(root_fd)
            run_fd = os.open(canonical, _DIRECTORY_FLAGS, dir_fd=root_fd)
            for name in (".control", ".status", "configuration"):
                os.mkdir(name, 0o755, dir_fd=run_fd)
                os.chmod(name, 0o755, dir_fd=run_fd, follow_symlinks=False)
            status_fd = os.open(".status", _DIRECTORY_FLAGS, dir_fd=run_fd)
            try:
                os.mkdir("quiescence", 0o755, dir_fd=status_fd)
                os.chmod(
                    "quiescence", 0o755, dir_fd=status_fd, follow_symlinks=False
                )
                os.fsync(status_fd)
            finally:
                os.close(status_fd)
            os.fsync(run_fd)
        finally:
            if run_fd is not None:
                os.close(run_fd)
            os.close(root_fd)
        return self.run_directory(canonical)

    def _open_run(self, run_id: str) -> int:
        canonical = _canonical_run_id(run_id)
        root_fd = self._open_output_root(create=False)
        try:
            descriptor = os.open(canonical, _DIRECTORY_FLAGS, dir_fd=root_fd)
            metadata = os.stat(canonical, dir_fd=root_fd, follow_symlinks=False)
            opened = os.fstat(descriptor)
            if not stat.S_ISDIR(metadata.st_mode) or (
                metadata.st_dev,
                metadata.st_ino,
            ) != (opened.st_dev, opened.st_ino):
                os.close(descriptor)
                raise ProtocolFileError("run directory is unsafe")
            return descriptor
        except FileNotFoundError:
            raise ProtocolFileError(f"run does not exist: {canonical}") from None
        except ProtocolFileError:
            raise
        except OSError as exc:
            raise ProtocolFileError(f"run directory is unsafe: {exc}") from exc
        finally:
            os.close(root_fd)

    @staticmethod
    def _open_child_directory(parent_fd: int, name: str) -> int:
        try:
            descriptor = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
            metadata = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
            opened = os.fstat(descriptor)
            if not stat.S_ISDIR(metadata.st_mode) or (
                metadata.st_dev,
                metadata.st_ino,
            ) != (opened.st_dev, opened.st_ino):
                raise ProtocolFileError(f"protocol directory {name!r} is unsafe")
            return descriptor
        except ProtocolFileError:
            try:
                os.close(descriptor)
            except (OSError, UnboundLocalError):
                pass
            raise
        except OSError as exc:
            raise ProtocolFileError(f"protocol directory {name!r} is unsafe: {exc}") from exc

    @staticmethod
    def _inspect_existing(directory_fd: int, name: str) -> os.stat_result | None:
        try:
            metadata = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
        except FileNotFoundError:
            return None
        except OSError as exc:
            raise ProtocolFileError(f"could not inspect protocol file {name!r}: {exc}") from exc
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
            raise ProtocolFileError(f"protocol file {name!r} must be regular and non-symlink")
        if metadata.st_nlink != 1:
            raise ProtocolFileError(f"protocol file {name!r} must have exactly one hard link")
        return metadata

    @classmethod
    def _read_document_at(
        cls,
        directory_fd: int,
        name: str,
        deadline_check: Callable[[], None] | None = None,
    ) -> dict[str, Any] | None:
        if deadline_check is not None:
            deadline_check()
        before = cls._inspect_existing(directory_fd, name)
        if before is None:
            return None
        descriptor: int | None = None
        try:
            descriptor = os.open(name, _FILE_FLAGS, dir_fd=directory_fd)
            opened = os.fstat(descriptor)
            if (before.st_dev, before.st_ino) != (opened.st_dev, opened.st_ino):
                raise ProtocolFileError(f"protocol file {name!r} changed while opening")
            if opened.st_size > _MAX_PROTOCOL_BYTES:
                raise ProtocolFileError(f"protocol file {name!r} is too large")
            chunks: list[bytes] = []
            remaining = _MAX_PROTOCOL_BYTES + 1
            while remaining:
                if deadline_check is not None:
                    deadline_check()
                chunk = os.read(descriptor, min(65536, remaining))
                if deadline_check is not None:
                    deadline_check()
                if not chunk:
                    break
                chunks.append(chunk)
                remaining -= len(chunk)
            if remaining == 0 and os.read(descriptor, 1):
                raise ProtocolFileError(f"protocol file {name!r} is too large")
            after = os.fstat(descriptor)
            named_after = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
            if any(
                getattr(opened, field) != getattr(after, field)
                or getattr(after, field) != getattr(named_after, field)
                for field in ("st_dev", "st_ino", "st_mode", "st_nlink", "st_size", "st_mtime_ns", "st_ctime_ns")
            ):
                raise ProtocolFileError(f"protocol file {name!r} changed while reading")
            try:
                if deadline_check is not None:
                    deadline_check()
                document = json.loads(
                    b"".join(chunks).decode("utf-8"),
                    object_pairs_hook=_reject_duplicate_pairs,
                    parse_constant=_reject_json_constant,
                )
                if deadline_check is not None:
                    deadline_check()
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError, RecursionError) as exc:
                raise ProtocolFileError(f"protocol file {name!r} contains invalid JSON: {exc}") from exc
            if not isinstance(document, dict):
                raise ProtocolFileError(f"protocol file {name!r} must contain a JSON object")
            return document
        except ProtocolFileError:
            raise
        except TimeoutError:
            raise
        except OSError as exc:
            raise ProtocolFileError(f"could not read protocol file {name!r}: {exc}") from exc
        finally:
            if descriptor is not None:
                os.close(descriptor)

    @classmethod
    def _write_document_at(
        cls,
        directory_fd: int,
        name: str,
        document: Mapping[str, Any],
        *,
        first_wins: bool,
    ) -> tuple[dict[str, Any], bool]:
        payload = _canonical_json(document)
        temporary = f".{name}.{uuid4().hex}.tmp"
        descriptor: int | None = None
        fcntl.flock(directory_fd, fcntl.LOCK_EX)
        try:
            existing = cls._read_document_at(directory_fd, name)
            if existing is not None and first_wins:
                return existing, False
            if existing is not None:
                cls._inspect_existing(directory_fd, name)
            descriptor = os.open(
                temporary,
                os.O_WRONLY
                | os.O_CREAT
                | os.O_EXCL
                | getattr(os, "O_CLOEXEC", 0)
                | getattr(os, "O_NOFOLLOW", 0),
                0o600,
                dir_fd=directory_fd,
            )
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
            return dict(document), True
        except ProtocolFileError:
            raise
        except OSError as exc:
            raise ProtocolFileError(f"could not persist protocol file {name!r}: {exc}") from exc
        finally:
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
            try:
                os.unlink(temporary, dir_fd=directory_fd)
            except FileNotFoundError:
                pass
            except OSError:
                pass
            fcntl.flock(directory_fd, fcntl.LOCK_UN)

    def _with_protocol_directory(self, run_id: str, directory_name: str) -> tuple[int, int]:
        run_fd = self._open_run(run_id)
        try:
            child_fd = self._open_child_directory(run_fd, directory_name)
            return run_fd, child_fd
        except BaseException:
            os.close(run_fd)
            raise

    def write_operator_status(self, status: OperatorStatus) -> Path:
        run_fd, status_fd = self._with_protocol_directory(status.run_id, ".status")
        try:
            self._write_document_at(
                status_fd, "operator-state.json", status.to_dict(), first_wins=False
            )
        finally:
            os.close(status_fd)
            os.close(run_fd)
        return self.run_directory(status.run_id) / ".status/operator-state.json"

    def read_operator_status(self, run_id: str) -> OperatorStatus:
        run_fd, status_fd = self._with_protocol_directory(run_id, ".status")
        try:
            document = self._read_document_at(status_fd, "operator-state.json")
        finally:
            os.close(status_fd)
            os.close(run_fd)
        if document is None:
            raise ProtocolFileError("operator state is missing")
        status = OperatorStatus.from_dict(document)
        if status.run_id != run_id:
            raise ProtocolFileError("operator state run_id does not match requested run")
        return status

    def request_finalization(
        self, run_id: str, requested_terminal: str, reason: str
    ) -> dict[str, Any]:
        _canonical_run_id(run_id)
        if requested_terminal not in _TERMINAL_STATES:
            raise ValueError("requested terminal state is invalid")
        if not isinstance(reason, str) or not reason:
            raise ValueError("finalization reason must be nonempty")
        document = {
            "run_id": run_id,
            "requested_terminal": requested_terminal,
            "reason": reason,
        }
        run_fd, control_fd = self._with_protocol_directory(run_id, ".control")
        try:
            persisted, _created = self._write_document_at(
                control_fd, "finalize-request.json", document, first_wins=True
            )
        finally:
            os.close(control_fd)
            os.close(run_fd)
        self._validate_finalize_request(run_id, persisted)
        return persisted

    @staticmethod
    def _validate_finalize_request(run_id: str, document: Any) -> None:
        if not isinstance(document, dict) or set(document) != {
            "run_id",
            "requested_terminal",
            "reason",
        }:
            raise ProtocolFileError("finalize request has invalid keys")
        if document["run_id"] != run_id:
            raise ProtocolFileError("finalize request run_id does not match requested run")
        if document["requested_terminal"] not in _TERMINAL_STATES:
            raise ProtocolFileError("finalize request terminal state is invalid")
        if not isinstance(document["reason"], str) or not document["reason"]:
            raise ProtocolFileError("finalize request reason is invalid")

    def read_finalize_request(self, run_id: str) -> dict[str, Any] | None:
        run_fd, control_fd = self._with_protocol_directory(run_id, ".control")
        try:
            document = self._read_document_at(control_fd, "finalize-request.json")
        finally:
            os.close(control_fd)
            os.close(run_fd)
        if document is not None:
            self._validate_finalize_request(run_id, document)
        return document

    def write_terminal_committed(self, run_id: str, document: Mapping[str, Any]) -> Path:
        expected = {"run_id", "terminal_status", "reason", "manifest_path"}
        if set(document) != expected or document.get("run_id") != run_id:
            raise ValueError("terminal commit document is invalid")
        if document.get("terminal_status") not in _TERMINAL_STATES:
            raise ValueError("terminal commit state is invalid")
        if document.get("manifest_path") != "manifest.json":
            raise ValueError("terminal commit manifest_path must be manifest.json")
        run_fd, control_fd = self._with_protocol_directory(run_id, ".control")
        try:
            persisted, created = self._write_document_at(
                control_fd, "terminal-committed.json", document, first_wins=True
            )
        finally:
            os.close(control_fd)
            os.close(run_fd)
        if not created and persisted != dict(document):
            raise ProtocolFileError("terminal commit already exists differently")
        return self.run_directory(run_id) / ".control/terminal-committed.json"

    def read_runtime_status(
        self,
        run_id: str,
        name: str,
        deadline_check: Callable[[], None] | None = None,
    ) -> dict[str, Any] | None:
        if name not in _RUNTIME_STATUS_NAMES:
            raise ValueError("runtime status name is not part of the frozen protocol")
        run_fd, status_fd = self._with_protocol_directory(run_id, ".status")
        try:
            document = self._read_document_at(
                status_fd,
                f"{name}.json",
                deadline_check,
            )
        finally:
            os.close(status_fd)
            os.close(run_fd)
        if document is not None and document.get("run_id") != run_id:
            raise ProtocolFileError(f"runtime status {name!r} has the wrong run_id")
        return document

    def validated_manifest_path(
        self,
        run_id: str,
        deadline_check: Callable[[], None] | None = None,
    ) -> Path:
        run_directory = self.run_directory(run_id)
        run_fd = self._open_run(run_id)
        try:
            document = self._read_document_at(
                run_fd, "manifest.json", deadline_check
            )
        finally:
            os.close(run_fd)
        if document is None:
            raise ProtocolFileError("manifest.json is missing")
        required_keys = {
            "schema_version",
            "run_id",
            "terminal_status",
            "reason",
            "simulation_timing",
            "wall_timing",
            "source_revisions",
            "image_digests",
            "configurations",
            "artifacts",
            "incomplete_paths",
            "scoring",
        }
        if set(document) != required_keys or document.get("schema_version") != 1:
            raise ProtocolFileError("manifest has an invalid schema")
        if document.get("run_id") != run_id:
            raise ProtocolFileError("manifest run_id does not match requested run")
        if document.get("terminal_status") not in _TERMINAL_STATES:
            raise ProtocolFileError("manifest terminal status is invalid")
        try:
            simulation = document["simulation_timing"]
            wall = document["wall_timing"]
            scoring = document["scoring"]
            if set(simulation) != {"start_ns", "end_ns", "duration_ns"}:
                raise ValueError("simulation timing keys")
            if set(wall) != {"started_at", "ended_at", "duration_seconds"}:
                raise ValueError("wall timing keys")
            if set(scoring) != {
                "achieved_score",
                "maximum_available_score",
                "scoring_checksum",
                "evidence_paths",
            }:
                raise ValueError("scoring keys")
            source_revisions = tuple(
                SourceRevision(value["name"], value["revision"], value["dirty"])
                for value in document["source_revisions"]
                if isinstance(value, dict)
                and set(value) == {"name", "revision", "dirty"}
            )
            if len(source_revisions) != len(document["source_revisions"]):
                raise ValueError("source revision keys")
            image_digests = tuple(
                ImageDigest(value["name"], value["digest"])
                for value in document["image_digests"]
                if isinstance(value, dict) and set(value) == {"name", "digest"}
            )
            if len(image_digests) != len(document["image_digests"]):
                raise ValueError("image digest keys")
            configurations = tuple(
                ConfigurationRecord(value["relative_path"], value["sha256"])
                for value in document["configurations"]
                if isinstance(value, dict)
                and set(value) == {"relative_path", "sha256"}
            )
            if len(configurations) != len(document["configurations"]):
                raise ValueError("configuration keys")
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolFileError(f"manifest metadata schema is invalid: {exc}") from exc
        artifacts = document.get("artifacts")
        if not isinstance(artifacts, list):
            raise ProtocolFileError("manifest artifacts must be an array")
        paths: set[str] = set()
        validations: dict[str, str] = {}
        for record in artifacts:
            if deadline_check is not None:
                deadline_check()
            if not isinstance(record, dict) or set(record) != {
                "relative_path",
                "size_bytes",
                "sha256",
                "validation",
                "detail",
            }:
                raise ProtocolFileError("manifest artifact record is invalid")
            relative_path = record["relative_path"]
            if (
                not isinstance(relative_path, str)
                or not relative_path
                or Path(relative_path).is_absolute()
                or ".." in Path(relative_path).parts
                or relative_path in paths
            ):
                raise ProtocolFileError("manifest artifact path is invalid or duplicated")
            paths.add(relative_path)
            validations[relative_path] = record["validation"]
            if record["validation"] == "valid":
                target = run_directory / relative_path
                validator = validate_tree if target.is_dir() else validate_regular_file
                result = validator(
                    run_directory,
                    relative_path,
                    deadline_check=deadline_check,
                )
                if (
                    result.status.value != "valid"
                    or result.size_bytes != record["size_bytes"]
                    or result.sha256 != record["sha256"]
                ):
                    raise ProtocolFileError(
                        f"manifest artifact {relative_path!r} no longer matches its checksum"
                    )
            elif record["validation"] not in {"missing", "invalid"}:
                raise ProtocolFileError("manifest artifact validation is invalid")
        if not set(REQUIRED_ARTIFACT_PATHS).issubset(paths):
            raise ProtocolFileError("manifest required inventory is incomplete")
        expected_incomplete = [
            path
            for path in REQUIRED_ARTIFACT_PATHS
            if validations[path] != "valid"
        ]
        if document.get("incomplete_paths") != expected_incomplete:
            raise ProtocolFileError("manifest incomplete_paths disagrees with required inventory")
        if document["terminal_status"] == "COMPLETED" and expected_incomplete:
            raise ProtocolFileError("completed manifest has incomplete required inventory")
        try:
            domain_manifest = RunManifest(
                run_id=document["run_id"],
                terminal_status=document["terminal_status"],
                reason=document["reason"],
                simulation_timing=SimulationTiming(
                    simulation["start_ns"],
                    simulation["end_ns"],
                    simulation["duration_ns"],
                ),
                wall_timing=WallTiming(
                    datetime.fromisoformat(wall["started_at"]),
                    datetime.fromisoformat(wall["ended_at"]),
                    wall["duration_seconds"],
                ),
                source_revisions=source_revisions,
                image_digests=image_digests,
                configurations=configurations,
                artifacts=tuple(
                    ArtifactRecord(
                        value["relative_path"],
                        value["size_bytes"],
                        value["sha256"],
                        value["validation"],
                        value["detail"],
                    )
                    for value in artifacts
                ),
                incomplete_paths=tuple(document["incomplete_paths"]),
                achieved_score=scoring["achieved_score"],
                maximum_available_score=scoring["maximum_available_score"],
                scoring_checksum=scoring["scoring_checksum"],
                evidence_paths=tuple(scoring["evidence_paths"]),
                schema_version=document["schema_version"],
            )
            validate_manifest(domain_manifest, deadline_check=deadline_check)
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolFileError(f"manifest domain validation failed: {exc}") from exc
        return run_directory / "manifest.json"

    def cleanup(self, run_id: str) -> None:
        descriptor = self._open_run(run_id)
        os.close(descriptor)


__all__ = [
    "OperatorStatus",
    "ProtocolFileError",
    "StatusStore",
    "TerminalCause",
]
