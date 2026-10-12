"""Native macOS and Linux QGC integration."""

from __future__ import annotations

import configparser
from contextlib import contextmanager
import hashlib
import os
from pathlib import Path
import plistlib
import shutil
import socket
import stat
import subprocess
import time
from typing import Any, Callable, Iterator, Mapping, Sequence
from urllib.parse import urlparse
import webbrowser
import xml.etree.ElementTree as ElementTree

from artifacts.qgc_operator import MavlinkIdentity

from ..host_config import HostConfig, HostConfigurationError, HostFacts, validate_state_paths
from ..host_integration import (
    ControllerIdentity,
    HostHelperInfo,
    HostIssue,
    HostLease,
    HostTransportError,
    LaunchResult,
    PublicationReceipt,
    QgcInstallation,
)
from ..qgc_actions import QgcActionMaterial
from ..qgc_publication import publish_posix


_QGC_VERSION = "5.0.8"
_QGC_SOURCE = MavlinkIdentity(255, 190)
_OFFICIAL_APPIMAGE_SIZE = 180_816_376
_OFFICIAL_APPIMAGE_SHA256 = (
    "06969c67ef58ea063def0a8271447a1cc385438c4a7df36813315b4475146737"
)
_APPSTREAM_NAME = "org.mavlink.qgroundcontrol.metainfo.xml"
_ELF_MACHINES = {"amd64": 62, "arm64": 183}
_MAX_DATAGRAM = 65_507


class _PosixLease:
    def register_controller(
        self, identity: ControllerIdentity, *, pristine: bool
    ) -> None:
        del identity, pristine

    def begin_run(self, run_id: str) -> None:
        del run_id

    def verify_resume(
        self, run_id: str, publication: PublicationReceipt
    ) -> None:
        del run_id, publication

    def finish_run(self, run_id: str, *, verified_cleanup: bool) -> None:
        del run_id, verified_cleanup


class _PosixDatagramTransport:
    def __init__(self, peer_port: int) -> None:
        self._peer = ("127.0.0.1", peer_port)
        self._socket: socket.socket | None = None
        try:
            active = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError as error:
            raise HostTransportError(
                f"QGC datagram socket creation failed: {error}"
            ) from error
        try:
            active.bind(("127.0.0.1", 0))
        except OSError as error:
            try:
                active.close()
            except OSError as close_error:
                raise HostTransportError(
                    "QGC datagram bind failed: "
                    f"{error}; socket cleanup failed: {close_error}"
                ) from error
            raise HostTransportError(f"QGC datagram bind failed: {error}") from error
        self._socket = active

    def _active_socket(self) -> socket.socket:
        if self._socket is None:
            raise HostTransportError("QGC datagram transport is closed")
        return self._socket

    def send(self, payload: bytes) -> None:
        if not isinstance(payload, bytes):
            raise TypeError("datagram payload must be bytes")
        if len(payload) > _MAX_DATAGRAM:
            raise HostTransportError("QGC datagram payload exceeds 65,507 bytes")
        try:
            sent = self._active_socket().sendto(payload, self._peer)
        except OSError as error:
            raise HostTransportError(f"QGC datagram send failed: {error}") from error
        if sent != len(payload):
            raise HostTransportError("QGC datagram send was incomplete")

    def receive(self, timeout_s: float) -> bytes | None:
        if not isinstance(timeout_s, (int, float)) or timeout_s < 0:
            raise ValueError("datagram timeout must be a nonnegative number")
        active = self._active_socket()
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                active.settimeout(remaining)
            except OSError as error:
                raise HostTransportError(
                    f"QGC datagram receive failed: {error}"
                ) from error
            try:
                payload, peer = active.recvfrom(_MAX_DATAGRAM + 1)
            except socket.timeout:
                return None
            except OSError as error:
                raise HostTransportError(f"QGC datagram receive failed: {error}") from error
            if peer != self._peer:
                continue
            if len(payload) > _MAX_DATAGRAM:
                raise HostTransportError("QGC datagram payload exceeds 65,507 bytes")
            return payload

    def close(self) -> None:
        active, self._socket = self._socket, None
        if active is not None:
            try:
                active.close()
            except OSError as error:
                raise HostTransportError(
                    f"QGC datagram close failed: {error}"
                ) from error


class PosixHost:
    """Read-only QGC discovery plus POSIX publication and launch operations."""

    def __init__(
        self,
        project_directory: Path,
        config: HostConfig,
        facts: HostFacts,
        *,
        home: Path | None = None,
        environ: Mapping[str, str] | None = None,
        runner: Callable[..., Any] | None = None,
        launcher: Callable[..., Any] | None = None,
        qgc_port: int = 14550,
    ) -> None:
        if facts.kind not in {"macos", "linux"}:
            raise HostConfigurationError("PosixHost supports only macOS and native Linux")
        self.project_directory = project_directory.resolve()
        self.config = config
        self.facts = facts
        self._home = home or Path.home()
        self._environ = dict(os.environ if environ is None else environ)
        self._runner: Callable[..., Any] = runner or subprocess.run
        self._launcher: Callable[..., Any] = launcher or subprocess.Popen
        self._qgc_port = qgc_port

    @property
    def state_directory(self) -> Path:
        if self.facts.kind == "macos":
            return self._home / "Library/Application Support/drone-sim/qgc"
        root = Path(
            self._environ.get("XDG_STATE_HOME", str(self._home / ".local/state"))
        )
        return root / "drone-sim/qgc"

    @property
    def settings_file(self) -> Path:
        if self.config.qgc_settings_file is not None:
            return Path(self.config.qgc_settings_file).expanduser()
        root = Path(
            self._environ.get("XDG_CONFIG_HOME", str(self._home / ".config"))
        )
        organization = (
            "qgroundcontrol.com" if self.facts.kind == "macos" else "QGroundControl"
        )
        return root / organization / "QGroundControl.ini"

    def preflight(self, paths: Sequence[Path]) -> tuple[HostIssue, ...]:
        try:
            validate_state_paths(self.facts, paths)
        except HostConfigurationError as error:
            return (HostIssue("unsupported-state-path", str(error)),)
        return ()

    def helper_info(self) -> HostHelperInfo | None:
        return None

    def inspect_qgc(self) -> QgcInstallation:
        executable, version = (
            self._inspect_mac_executable()
            if self.facts.kind == "macos"
            else self._inspect_linux_executable()
        )
        settings_path = self.settings_file
        snapshot = _read_regular_snapshot(settings_path, "QGroundControl settings")
        save_path, source = _parse_settings(snapshot, settings_path)
        return QgcInstallation(
            executable=str(executable),
            settings_file=str(settings_path),
            save_path=save_path,
            version=version,
            source=source,
        )

    def _inspect_mac_executable(self) -> tuple[Path, str]:
        selected = Path(
            self.config.qgc_executable or "/Applications/QGroundControl.app"
        ).expanduser()
        if selected.suffix == ".app":
            bundle = selected
        elif selected.name == "QGroundControl" and selected.parent.name == "MacOS":
            bundle = selected.parents[2]
        else:
            raise RuntimeError(
                f"selected macOS QGroundControl path {selected} is not an application bundle"
            )
        info_path = bundle / "Contents/Info.plist"
        try:
            document = plistlib.loads(
                _read_regular_snapshot(info_path, "QGroundControl bundle metadata")
            )
        except (plistlib.InvalidFileException, ValueError) as error:
            raise RuntimeError(f"QGroundControl metadata {info_path} is invalid") from error
        required = {
            "CFBundleIdentifier": "org.qgroundcontrol.QGroundControl",
            "CFBundleExecutable": "QGroundControl",
            "CFBundleVersion": _QGC_VERSION,
            "CFBundleLongVersionString": _QGC_VERSION,
        }
        if any(document.get(key) != value for key, value in required.items()):
            raise RuntimeError(
                f"QGroundControl bundle {bundle} does not identify exact version {_QGC_VERSION}"
            )
        executable = bundle / "Contents/MacOS/QGroundControl"
        _require_executable(executable)
        return executable, _QGC_VERSION

    def _inspect_linux_executable(self) -> tuple[Path, str]:
        if self.config.qgc_executable is not None:
            selected = Path(self.config.qgc_executable).expanduser()
        else:
            candidates: list[Path] = []
            discovered = shutil.which("QGroundControl")
            if discovered:
                candidates.append(Path(discovered))
            for candidate in (
                self._home / "Applications/QGroundControl.AppImage",
                Path("/opt/QGroundControl/QGroundControl.AppImage"),
            ):
                if candidate.exists():
                    candidates.append(candidate)
            unique = list(dict.fromkeys(path.resolve() for path in candidates))
            if not unique:
                raise RuntimeError(
                    "no QGroundControl installation was found; set qgc.executable to "
                    "a native 5.0.8 installation or the pinned official AppImage"
                )
            if len(unique) != 1:
                joined = ", ".join(str(path) for path in unique)
                raise RuntimeError(
                    f"QGroundControl installation is ambiguous ({joined}); select qgc.executable"
                )
            selected = unique[0]
        _require_executable(selected)

        metadata = selected.stat()
        if metadata.st_size == _OFFICIAL_APPIMAGE_SIZE:
            digest = _hash_stable_file(selected)
            if digest == _OFFICIAL_APPIMAGE_SHA256:
                if self.facts.machine != "amd64":
                    raise RuntimeError(
                        "the pinned official QGroundControl AppImage requires amd64 architecture"
                    )
                return selected, _QGC_VERSION

        if selected.name != "QGroundControl" or selected.parent.name != "bin":
            raise RuntimeError(
                f"selected QGroundControl binary {selected} has no paired AppStream metadata"
            )
        _validate_elf_architecture(selected, self.facts.machine)
        prefix = selected.parent.parent
        appstream = prefix / "share/metainfo" / _APPSTREAM_NAME
        _validate_appstream(appstream)
        return selected, _QGC_VERSION

    @contextmanager
    def acquire_native_lease(self) -> Iterator[HostLease]:
        yield _PosixLease()

    def publish_actions(
        self,
        material: QgcActionMaterial,
        installation: QgcInstallation,
        lease: HostLease,
    ) -> PublicationReceipt:
        del lease
        if installation.source != _QGC_SOURCE or installation.version != _QGC_VERSION:
            raise RuntimeError("QGroundControl installation identity changed before publication")
        return publish_posix(material, Path(installation.save_path) / "MavlinkActions")

    def open_qgc(self, installation: QgcInstallation) -> LaunchResult:
        if self.inspect_qgc() != installation:
            raise RuntimeError("QGroundControl installation identity changed before launch")
        executable = Path(installation.executable)
        if self.facts.kind == "macos":
            bundle = executable.parents[2]
            command = ["open", "-a", str(bundle)]
            try:
                result = self._runner(
                    command,
                    cwd=self.project_directory,
                    capture_output=True,
                    text=True,
                    timeout=15,
                    check=False,
                )
            except (OSError, subprocess.TimeoutExpired) as error:
                raise RuntimeError(f"QGroundControl could not be opened: {error}") from error
            if result is not None:
                returncode = int(getattr(result, "returncode", 0))
                if returncode:
                    detail = str(getattr(result, "stderr", "")).strip()
                    raise RuntimeError(detail or "QGroundControl could not be opened")
        else:
            command = [str(executable)]
            try:
                self._launcher(
                    command,
                    cwd=self.project_directory,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except OSError as error:
                raise RuntimeError(f"QGroundControl could not be opened: {error}") from error
        return LaunchResult(True, "QGroundControl launch or focus request completed")

    def open_browser(self, url: str) -> LaunchResult:
        parsed = urlparse(url)
        if parsed.scheme not in {"http", "https"} or not parsed.netloc:
            raise ValueError("browser URL must be an absolute HTTP or HTTPS URL")
        opened = webbrowser.open(url)
        return LaunchResult(opened, "browser open request completed" if opened else "browser did not accept the URL")

    def open_datagrams(
        self,
        lease: HostLease,
        run_id: str,
        publication: PublicationReceipt,
    ) -> _PosixDatagramTransport:
        del lease, run_id, publication
        return _PosixDatagramTransport(self._qgc_port)


def _read_regular_snapshot(path: Path, label: str) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise RuntimeError(f"{label} {path} is missing, unsafe, or unreadable") from error
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_nlink != 1:
            raise RuntimeError(f"{label} {path} must be one unambiguous regular file")
        payload = bytearray()
        while len(payload) <= before.st_size:
            chunk = os.read(descriptor, before.st_size + 1 - len(payload))
            if not chunk:
                break
            payload.extend(chunk)
        after = os.fstat(descriptor)
        if (
            (before.st_dev, before.st_ino, before.st_size)
            != (after.st_dev, after.st_ino, after.st_size)
            or len(payload) != before.st_size
        ):
            raise RuntimeError(f"{label} {path} changed while being inspected")
        return bytes(payload)
    finally:
        os.close(descriptor)


def _parse_settings(payload: bytes, path: Path) -> tuple[str, MavlinkIdentity]:
    parser = configparser.ConfigParser(interpolation=None, strict=True)
    try:
        text = payload.decode("utf-8")
        parser.read_string(text)
    except (UnicodeError, configparser.Error) as error:
        raise RuntimeError(f"QGroundControl settings {path} are ambiguous or invalid") from error

    def option(section: str, *names: str) -> str | None:
        if not parser.has_section(section):
            return None
        matches = [parser.get(section, name) for name in names if parser.has_option(section, name)]
        if len(matches) > 1:
            raise RuntimeError(f"QGroundControl settings {path} contain ambiguous {names[0]}")
        return matches[0] if matches else None

    version = option("General", "SettingsVersion")
    if version is not None and version != "9":
        raise RuntimeError(
            f"QGroundControl settings {path} use incompatible SettingsVersion {version}; expected 9"
        )
    save_path = option("General", "savePath")
    if save_path is None:
        raise RuntimeError(f"QGroundControl settings {path} do not contain General/savePath")
    if not Path(save_path).is_absolute():
        raise RuntimeError(f"QGroundControl savePath in {path} must be absolute")
    option("MavlinkActions", "flyViewActionsFile")

    configured_id = option("General", "gcsMavlinkSystemID")
    legacy_id = option("QGC_MAVLINK_PROTOCOL", "GCS_SYSTEM_ID")
    raw_id = configured_id if configured_id is not None else legacy_id
    if raw_id is None:
        system_id = 255
    else:
        try:
            system_id = int(raw_id)
        except ValueError as error:
            raise RuntimeError(f"QGroundControl system ID in {path} is not an integer") from error
        if not 1 <= system_id <= 255:
            raise RuntimeError(f"QGroundControl system ID in {path} must be 1 through 255")
    if system_id != 255:
        raise RuntimeError(
            f"QGroundControl system ID in {path} is {system_id}; this console requires 255"
        )
    return save_path, MavlinkIdentity(system_id, 190)


def _require_executable(path: Path) -> None:
    try:
        metadata = path.stat(follow_symlinks=False)
    except OSError as error:
        raise RuntimeError(f"QGroundControl executable {path} is missing or unreadable") from error
    if not stat.S_ISREG(metadata.st_mode) or not os.access(path, os.X_OK):
        raise RuntimeError(f"QGroundControl executable {path} must be an executable regular file")


def _hash_stable_file(path: Path) -> str:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        before = os.fstat(descriptor)
        digest = hashlib.sha256()
        while chunk := os.read(descriptor, 1024 * 1024):
            digest.update(chunk)
        after = os.fstat(descriptor)
        if (
            (before.st_dev, before.st_ino, before.st_size)
            != (after.st_dev, after.st_ino, after.st_size)
            or before.st_size != _OFFICIAL_APPIMAGE_SIZE
        ):
            raise RuntimeError(f"QGroundControl AppImage {path} changed while being inspected")
        return digest.hexdigest()
    finally:
        os.close(descriptor)


def _validate_elf_architecture(path: Path, machine: str) -> None:
    with path.open("rb") as source:
        header = source.read(20)
    if len(header) < 20 or header[:4] != b"\x7fELF" or header[4:6] != b"\x02\x01":
        raise RuntimeError(f"QGroundControl executable {path} is not a supported native ELF file")
    elf_machine = int.from_bytes(header[18:20], "little")
    if elf_machine != _ELF_MACHINES[machine]:
        raise RuntimeError(
            f"QGroundControl executable {path} architecture does not match host {machine}"
        )


def _validate_appstream(path: Path) -> None:
    try:
        root = ElementTree.fromstring(
            _read_regular_snapshot(path, "QGroundControl AppStream metadata")
        )
    except ElementTree.ParseError as error:
        raise RuntimeError(f"QGroundControl AppStream metadata {path} is invalid") from error
    application_ids = [
        (element.text or "").strip()
        for element in root
        if element.tag.rsplit("}", 1)[-1] == "id"
    ]
    release_groups = [
        element
        for element in root
        if element.tag.rsplit("}", 1)[-1] == "releases"
    ]
    releases = list(release_groups[0]) if len(release_groups) == 1 else []
    if (
        root.tag.rsplit("}", 1)[-1] != "component"
        or root.attrib.get("type") != "desktop-application"
        or application_ids != ["org.mavlink.qgroundcontrol"]
        or len(release_groups) != 1
        or len(releases) != 1
        or releases[0].tag.rsplit("}", 1)[-1] != "release"
        or releases[0].attrib.get("version") != _QGC_VERSION
    ):
        raise RuntimeError(
            f"QGroundControl AppStream metadata {path} does not identify version {_QGC_VERSION}"
        )


__all__ = ["PosixHost"]
