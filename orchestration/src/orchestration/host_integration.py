"""Platform boundary for native QGC files, processes, and datagrams."""

from __future__ import annotations

from contextlib import AbstractContextManager
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, Sequence
from uuid import UUID

from artifacts.qgc_operator import MavlinkIdentity

from .host_config import (
    HostFacts,
    detect_host,
    load_host_config,
)
from .qgc_actions import QgcActionMaterial


@dataclass(frozen=True, slots=True)
class QgcInstallation:
    executable: str
    settings_file: str
    save_path: str
    version: str
    source: MavlinkIdentity


@dataclass(frozen=True, slots=True)
class PublicationReceipt:
    native_path: str
    filename: str
    sha256: str


@dataclass(frozen=True, slots=True)
class HostIssue:
    code: str
    message: str


@dataclass(frozen=True, slots=True)
class HostHelperInfo:
    version: str
    source_sha256: str

    def __post_init__(self) -> None:
        if not isinstance(self.version, str) or not self.version:
            raise ValueError("helper version must be nonempty")
        if (
            not isinstance(self.source_sha256, str)
            or len(self.source_sha256) != 64
            or self.source_sha256.lower() != self.source_sha256
            or any(
                character not in "0123456789abcdef"
                for character in self.source_sha256
            )
        ):
            raise ValueError("helper source digest must be lowercase SHA-256 text")


@dataclass(frozen=True, slots=True)
class LaunchResult:
    opened: bool
    detail: str


@dataclass(frozen=True, slots=True)
class ControllerIdentity:
    installation_id: UUID
    ledger_id: UUID


@dataclass(frozen=True, slots=True)
class ControllerRegistration:
    active_run_id: str | None
    recovery_required: bool

    def __post_init__(self) -> None:
        if self.active_run_id is not None:
            try:
                parsed = UUID(self.active_run_id)
            except (ValueError, TypeError, AttributeError) as error:
                raise ValueError("active run identity must be a canonical UUID") from error
            if str(parsed) != self.active_run_id:
                raise ValueError("active run identity must be a canonical UUID")
        if type(self.recovery_required) is not bool:
            raise TypeError("recovery_required must be boolean")


class HostTransportError(RuntimeError):
    """The native QGC datagram boundary failed or was already closed."""


class DatagramTransport(Protocol):
    def send(self, payload: bytes) -> None: ...

    def receive(self, timeout_s: float) -> bytes | None: ...

    def close(self) -> None: ...


class HostLease(Protocol):
    def register_controller(
        self, identity: ControllerIdentity, *, pristine: bool
    ) -> ControllerRegistration: ...

    def begin_run(self, run_id: str) -> None: ...

    def verify_resume(
        self, run_id: str, publication: PublicationReceipt
    ) -> None: ...

    def finish_run(self, run_id: str, *, verified_cleanup: bool) -> None: ...


class HostIntegration(Protocol):
    facts: HostFacts

    @property
    def state_directory(self) -> Path: ...

    def preflight(self, paths: Sequence[Path]) -> tuple[HostIssue, ...]: ...

    def helper_info(self) -> HostHelperInfo | None: ...

    def inspect_qgc(self) -> QgcInstallation: ...

    def acquire_native_lease(self) -> AbstractContextManager[HostLease]: ...

    def publish_actions(
        self,
        material: QgcActionMaterial,
        installation: QgcInstallation,
        lease: HostLease,
    ) -> PublicationReceipt: ...

    def open_qgc(self, installation: QgcInstallation) -> LaunchResult: ...

    def open_browser(self, url: str) -> LaunchResult: ...

    def open_datagrams(
        self,
        lease: HostLease,
        run_id: str,
        publication: PublicationReceipt,
    ) -> DatagramTransport: ...


def resolve_host(
    project_directory: Path, config_path: Path | None = None
) -> HostIntegration:
    """Resolve one host adapter for the lifetime of a local console."""

    config = load_host_config(project_directory, config_path)
    facts = detect_host(config)
    if facts.kind == "wsl2":
        from ._adapters.wsl_host import WslHost

        return WslHost(project_directory, config, facts)
    from ._adapters.posix_host import PosixHost

    return PosixHost(project_directory, config, facts)


__all__ = [
    "ControllerIdentity",
    "ControllerRegistration",
    "DatagramTransport",
    "HostIntegration",
    "HostHelperInfo",
    "HostIssue",
    "HostLease",
    "HostTransportError",
    "LaunchResult",
    "PublicationReceipt",
    "QgcInstallation",
    "resolve_host",
]
