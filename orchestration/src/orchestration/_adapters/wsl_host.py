"""WSL adapter for native Windows QGroundControl operations."""

from __future__ import annotations

import base64
from contextlib import contextmanager
import hashlib
import ntpath
import os
from pathlib import Path
import threading
from typing import Callable, Iterator, Mapping, Sequence, cast
from urllib.parse import urlparse
from uuid import uuid4

from artifacts.qgc_operator import MavlinkIdentity

from ..host_config import HostConfig, HostConfigurationError, HostFacts, validate_state_paths
from ..host_integration import (
    ControllerIdentity,
    ControllerRegistration,
    HostHelperInfo,
    HostIssue,
    HostLease,
    HostTransportError,
    LaunchResult,
    PublicationReceipt,
    QgcInstallation,
)
from ..qgc_actions import QgcActionMaterial
from ..windows_helper import WindowsHelper, WindowsHelperError


_QGC_VERSION = "5.0.8"
_QGC_SOURCE = MavlinkIdentity(255, 190)


class _WslLease:
    def __init__(self, owner: WslHost, helper: WindowsHelper, distro_name: str) -> None:
        self.owner = owner
        self.helper = helper
        self._distro_name = distro_name

    def register_controller(
        self, identity: ControllerIdentity, *, pristine: bool
    ) -> ControllerRegistration:
        result = self.helper.request(
            "register_controller",
            {
                "installation_id": str(identity.installation_id),
                "ledger_id": str(identity.ledger_id),
                "distro_name": self._distro_name,
                "pristine": pristine,
            },
        )
        active_run_id = result.get("active_run_id")
        if active_run_id is not None and not isinstance(active_run_id, str):
            raise RuntimeError(
                "Windows helper result field active_run_id must be null or a string"
            )
        return ControllerRegistration(
            active_run_id=active_run_id,
            recovery_required=_boolean(result, "recovery_required"),
        )

    def begin_run(self, run_id: str) -> None:
        self.helper.request("begin_run", {"run_id": run_id})

    def verify_resume(
        self, run_id: str, publication: PublicationReceipt
    ) -> None:
        self.helper.request(
            "verify_resume",
            {"run_id": run_id, "publication": _publication_payload(publication)},
        )

    def finish_run(self, run_id: str, *, verified_cleanup: bool) -> None:
        self.helper.request(
            "finish_run",
            {"run_id": run_id, "verified_cleanup": verified_cleanup},
        )


class _WslDatagramTransport:
    def __init__(self, helper: WindowsHelper) -> None:
        self._helper = helper
        self._closed = False

    def send(self, payload: bytes) -> None:
        if self._closed:
            raise HostTransportError("QGC datagram transport is closed")
        try:
            self._helper.send(payload)
        except WindowsHelperError as error:
            raise HostTransportError(str(error)) from error

    def receive(self, timeout_s: float) -> bytes | None:
        if self._closed:
            raise HostTransportError("QGC datagram transport is closed")
        try:
            return self._helper.receive(timeout_s)
        except WindowsHelperError as error:
            raise HostTransportError(str(error)) from error

    def close(self) -> None:
        if self._closed:
            return
        try:
            self._helper.request("close_transport", {})
        except WindowsHelperError as error:
            raise HostTransportError(str(error)) from error
        finally:
            self._closed = True


class WslHost:
    """Native Windows host operations reached through bounded WSL interop."""

    def __init__(
        self,
        project_directory: Path,
        config: HostConfig,
        facts: HostFacts,
        *,
        home: Path | None = None,
        environ: Mapping[str, str] | None = None,
        helper_factory: Callable[..., WindowsHelper] = WindowsHelper.start,
    ) -> None:
        if facts.kind != "wsl2":
            raise HostConfigurationError("WslHost requires a WSL2 host")
        self.project_directory = project_directory.resolve()
        self.config = config
        self.facts = facts
        self._home = home or Path.home()
        self._environ = dict(os.environ if environ is None else environ)
        self._helper_factory = helper_factory
        distro_name = self._environ.get("WSL_DISTRO_NAME", "")
        if not distro_name:
            raise HostConfigurationError(
                "WSL_DISTRO_NAME is missing; start the console inside its registered WSL2 distribution"
            )
        self._distro_name = distro_name
        self._active_helper_info: HostHelperInfo | None = None
        self._lease_lock = threading.Lock()

    @property
    def state_directory(self) -> Path:
        root = Path(
            self._environ.get("XDG_STATE_HOME", str(self._home / ".local/state"))
        )
        return root / "drone-sim/qgc"

    def _start_helper(self) -> WindowsHelper:
        return self._helper_factory(session_id=str(uuid4()))

    def preflight(self, paths: Sequence[Path]) -> tuple[HostIssue, ...]:
        try:
            if paths:
                validate_state_paths(self.facts, paths)
        except HostConfigurationError as error:
            return (HostIssue("unsupported-state-path", str(error)),)
        helper: WindowsHelper | None = None
        try:
            helper = self._start_helper()
            helper.request("probe", {})
        except WindowsHelperError as error:
            return (HostIssue(error.code, str(error)),)
        finally:
            if helper is not None:
                helper.close()
        return ()

    def helper_info(self) -> HostHelperInfo:
        if self._active_helper_info is not None:
            return self._active_helper_info
        helper = self._start_helper()
        try:
            result = helper.request("probe", {})
            info = helper.info
            if (
                _string(result, "helper_version") != info.version
                or _string(result, "source_sha256") != info.source_sha256
            ):
                raise RuntimeError("Windows helper probe identity changed after startup")
            return info
        finally:
            helper.close()

    def _short_request(
        self, operation: str, payload: Mapping[str, object]
    ) -> Mapping[str, object]:
        helper = self._start_helper()
        try:
            return helper.request(operation, payload)
        finally:
            helper.close()

    def inspect_qgc(self) -> QgcInstallation:
        result = self._short_request(
            "inspect_qgc",
            {
                "executable": self.config.qgc_executable or "",
                "settings_file": self.config.qgc_settings_file or "",
            },
        )
        source = _mapping(result, "source")
        installation = QgcInstallation(
            executable=_string(result, "executable"),
            settings_file=_string(result, "settings_file"),
            save_path=_string(result, "save_path"),
            version=_string(result, "version"),
            source=MavlinkIdentity(
                _integer(source, "system_id"), _integer(source, "component_id")
            ),
        )
        if installation.version != _QGC_VERSION or installation.source != _QGC_SOURCE:
            raise RuntimeError("Windows helper returned an unexpected QGroundControl identity")
        return installation

    @contextmanager
    def acquire_native_lease(self) -> Iterator[HostLease]:
        if not self._lease_lock.acquire(blocking=False):
            raise RuntimeError("this WSL host already has an active native lease")
        helper: WindowsHelper | None = None
        acquired = False
        try:
            helper = self._start_helper()
            helper.request("acquire_lease", {})
            acquired = True
            self._active_helper_info = helper.info
            yield _WslLease(self, helper, self._distro_name)
        finally:
            try:
                if helper is not None and acquired:
                    try:
                        helper.request("release_lease", {})
                    finally:
                        helper.close()
                elif helper is not None:
                    helper.close()
            finally:
                self._active_helper_info = None
                self._lease_lock.release()

    def publish_actions(
        self,
        material: QgcActionMaterial,
        installation: QgcInstallation,
        lease: HostLease,
    ) -> PublicationReceipt:
        native_lease = self._require_lease(lease)
        if installation.version != _QGC_VERSION or installation.source != _QGC_SOURCE:
            raise RuntimeError("QGroundControl installation identity changed before publication")
        digest = hashlib.sha256(material.action_bytes).hexdigest()
        result = native_lease.helper.request(
            "publish_actions",
            {
                "directory": ntpath.join(installation.save_path, "MavlinkActions"),
                "filename": material.published_name,
                "data_base64": base64.b64encode(material.action_bytes).decode("ascii"),
                "sha256": digest,
            },
        )
        receipt = PublicationReceipt(
            native_path=_string(result, "native_path"),
            filename=_string(result, "filename"),
            sha256=_string(result, "sha256"),
        )
        if receipt.filename != material.published_name or receipt.sha256 != digest:
            raise RuntimeError("Windows helper returned a mismatched publication receipt")
        return receipt

    def open_qgc(self, installation: QgcInstallation) -> LaunchResult:
        result = self._short_request(
            "open_qgc", {"installation": _installation_payload(installation)}
        )
        return LaunchResult(_boolean(result, "opened"), _string(result, "detail"))

    def open_browser(self, url: str) -> LaunchResult:
        parsed = urlparse(url)
        if parsed.scheme not in {"http", "https"} or not parsed.netloc:
            raise ValueError("browser URL must be an absolute HTTP or HTTPS URL")
        result = self._short_request(
            "open_browser", {"url": url, "probe_timeout_ms": 5000}
        )
        return LaunchResult(_boolean(result, "opened"), _string(result, "detail"))

    def open_datagrams(
        self,
        lease: HostLease,
        run_id: str,
        publication: PublicationReceipt,
    ) -> _WslDatagramTransport:
        native_lease = self._require_lease(lease)
        result = native_lease.helper.request(
            "open_transport",
            {"run_id": run_id, "publication": _publication_payload(publication)},
        )
        if _integer(result, "local_port") < 1 or _string(result, "peer") != "127.0.0.1:14550":
            raise RuntimeError("Windows helper returned an invalid UDP transport endpoint")
        return _WslDatagramTransport(native_lease.helper)

    def _require_lease(self, lease: HostLease) -> _WslLease:
        if not isinstance(lease, _WslLease) or lease.owner is not self:
            raise RuntimeError("Windows host operation requires its active native lease")
        return lease


def _publication_payload(receipt: PublicationReceipt) -> dict[str, object]:
    return {
        "native_path": receipt.native_path,
        "filename": receipt.filename,
        "sha256": receipt.sha256,
    }


def _installation_payload(installation: QgcInstallation) -> dict[str, object]:
    return {
        "executable": installation.executable,
        "settings_file": installation.settings_file,
        "save_path": installation.save_path,
        "version": installation.version,
        "source": {
            "system_id": installation.source.system_id,
            "component_id": installation.source.component_id,
        },
    }


def _mapping(value: Mapping[str, object], field: str) -> Mapping[str, object]:
    selected = value.get(field)
    if not isinstance(selected, dict):
        raise RuntimeError(f"Windows helper result field {field} must be an object")
    return selected


def _string(value: Mapping[str, object], field: str) -> str:
    selected = value.get(field)
    if not isinstance(selected, str) or not selected:
        raise RuntimeError(f"Windows helper result field {field} must be a nonempty string")
    return selected


def _integer(value: Mapping[str, object], field: str) -> int:
    selected = value.get(field)
    if type(selected) is not int:
        raise RuntimeError(f"Windows helper result field {field} must be an integer")
    return cast(int, selected)


def _boolean(value: Mapping[str, object], field: str) -> bool:
    selected = value.get(field)
    if type(selected) is not bool:
        raise RuntimeError(f"Windows helper result field {field} must be boolean")
    return cast(bool, selected)


__all__ = ["WslHost"]
