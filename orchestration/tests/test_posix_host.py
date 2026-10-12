from __future__ import annotations

from dataclasses import FrozenInstanceError
from pathlib import Path
import plistlib
import socket
import stat
from types import SimpleNamespace

import pytest

from artifacts.qgc_operator import MavlinkIdentity
from orchestration.host_config import HostConfig, HostFacts
from orchestration.host_integration import HostTransportError, QgcInstallation
import orchestration.host_integration as host_integration_module
from orchestration.qgc_actions import QgcActionStore
import orchestration._adapters.posix_host as posix_module
from orchestration._adapters.posix_host import PosixHost


RUN_ID = "11111111-1111-4111-8111-111111111111"
QGC_SOURCE = MavlinkIdentity(255, 190)


def _facts(kind: str = "linux", machine: str = "amd64") -> HostFacts:
    return HostFacts(
        kind=kind,  # type: ignore[arg-type]
        machine=machine,  # type: ignore[arg-type]
        image_platform="linux/amd64" if machine == "amd64" else "linux/arm64",
        provider="docker" if kind == "linux" else "colima",
    )


def _settings(path: Path, save_path: Path, *, extra_general: str = "") -> bytes:
    payload = (
        "[General]\n"
        "SettingsVersion=9\n"
        f"savePath={save_path}\n"
        f"{extra_general}"
        "\n[MavlinkActions]\n"
        "flyViewActionsFile=custom_missions.json\n"
    ).encode()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    return payload


def _elf(path: Path, machine: str) -> None:
    payload = bytearray(64)
    payload[:6] = b"\x7fELF\x02\x01"
    payload[18:20] = (62 if machine == "amd64" else 183).to_bytes(2, "little")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def _linux_installation(root: Path, machine: str = "amd64") -> Path:
    executable = root / "usr/bin/QGroundControl"
    _elf(executable, machine)
    metadata = root / "usr/share/metainfo/org.mavlink.qgroundcontrol.metainfo.xml"
    metadata.parent.mkdir(parents=True)
    metadata.write_text(
        "<component type='desktop-application'>"
        "<id>org.mavlink.qgroundcontrol</id>"
        "<releases><release version='5.0.8'/></releases>"
        "</component>",
        encoding="utf-8",
    )
    return executable


def _mac_installation(root: Path) -> tuple[Path, Path]:
    bundle = root / "QGroundControl.app"
    executable = bundle / "Contents/MacOS/QGroundControl"
    executable.parent.mkdir(parents=True)
    executable.write_bytes(b"native executable")
    executable.chmod(executable.stat().st_mode | stat.S_IXUSR)
    info = {
        "CFBundleIdentifier": "org.qgroundcontrol.QGroundControl",
        "CFBundleExecutable": "QGroundControl",
        "CFBundleVersion": "5.0.8",
        "CFBundleLongVersionString": "5.0.8",
    }
    with (bundle / "Contents/Info.plist").open("wb") as target:
        plistlib.dump(info, target)
    return bundle, executable


def test_result_contracts_are_immutable() -> None:
    installation = QgcInstallation(
        executable="/usr/bin/QGroundControl",
        settings_file="/home/operator/.config/QGroundControl/QGroundControl.ini",
        save_path="/home/operator/QGroundControl",
        version="5.0.8",
        source=QGC_SOURCE,
    )
    with pytest.raises(FrozenInstanceError):
        installation.version = "5.0.9"  # type: ignore[misc]


def test_posix_state_and_settings_paths_follow_host_conventions(
    tmp_path: Path,
) -> None:
    mac = PosixHost(
        tmp_path,
        HostConfig("macos", None, None),
        _facts("macos", "arm64"),
        home=tmp_path / "mac-home",
        environ={},
    )
    linux_default = PosixHost(
        tmp_path,
        HostConfig("linux", None, None),
        _facts(),
        home=tmp_path / "linux-home",
        environ={},
    )
    linux_xdg = PosixHost(
        tmp_path,
        HostConfig("linux", None, None),
        _facts(),
        home=tmp_path / "linux-home",
        environ={
            "XDG_STATE_HOME": str(tmp_path / "state"),
            "XDG_CONFIG_HOME": str(tmp_path / "config"),
        },
    )

    assert mac.state_directory == (
        tmp_path / "mac-home/Library/Application Support/drone-sim/qgc"
    )
    assert mac.settings_file == (
        tmp_path / "mac-home/.config/qgroundcontrol.com/QGroundControl.ini"
    )
    assert linux_default.state_directory == (
        tmp_path / "linux-home/.local/state/drone-sim/qgc"
    )
    assert linux_default.settings_file == (
        tmp_path / "linux-home/.config/QGroundControl/QGroundControl.ini"
    )
    assert linux_xdg.state_directory == tmp_path / "state/drone-sim/qgc"
    assert linux_xdg.settings_file == (
        tmp_path / "config/QGroundControl/QGroundControl.ini"
    )
    assert mac.helper_info() is None
    assert linux_default.helper_info() is None
    assert linux_xdg.helper_info() is None


def test_mac_upgrade_preserves_existing_counter(tmp_path: Path) -> None:
    host = PosixHost(
        tmp_path,
        HostConfig("macos", None, None),
        _facts("macos", "arm64"),
        home=tmp_path / "home",
        environ={},
    )
    store = QgcActionStore(host.state_directory, QGC_SOURCE)
    previous_token = store.allocate(RUN_ID).descriptor.attempt_token

    next_token = QgcActionStore(host.state_directory, QGC_SOURCE).allocate(
        "22222222-2222-4222-8222-222222222222"
    ).descriptor.attempt_token

    assert next_token == previous_token + 1


def test_mac_discovery_preserves_qgc_preferences_and_selection(tmp_path: Path) -> None:
    bundle, executable = _mac_installation(tmp_path)
    settings_file = tmp_path / "QGroundControl.ini"
    save_path = tmp_path / "QGC Save"
    before = _settings(settings_file, save_path)
    host = PosixHost(
        tmp_path,
        HostConfig("macos", str(bundle), str(settings_file)),
        _facts("macos", "arm64"),
        home=tmp_path,
        environ={},
    )

    installation = host.inspect_qgc()
    material = QgcActionStore(host.state_directory, QGC_SOURCE).allocate(RUN_ID)
    with host.acquire_native_lease() as lease:
        receipt = host.publish_actions(material, installation, lease)

    published = save_path / "MavlinkActions" / material.published_name
    assert installation.executable == str(executable)
    assert installation.version == "5.0.8"
    assert installation.source == QGC_SOURCE
    assert settings_file.read_bytes() == before
    assert published.read_bytes() == material.action_bytes
    assert receipt.sha256 == __import__("hashlib").sha256(published.read_bytes()).hexdigest()


def test_linux_discovers_native_appdir_from_paired_appstream_metadata(
    tmp_path: Path,
) -> None:
    executable = _linux_installation(tmp_path / "QGroundControl.AppDir")
    settings_file = tmp_path / "QGroundControl.ini"
    _settings(settings_file, tmp_path / "save")
    host = PosixHost(
        tmp_path,
        HostConfig("linux", str(executable), str(settings_file)),
        _facts(),
        home=tmp_path,
        environ={},
    )

    installation = host.inspect_qgc()

    assert installation.executable == str(executable)
    assert installation.version == "5.0.8"
    assert installation.source == QGC_SOURCE


def test_linux_rejects_newer_appstream_release_with_historical_5_0_8(
    tmp_path: Path,
) -> None:
    executable = _linux_installation(tmp_path / "QGroundControl.AppDir")
    metadata = (
        executable.parents[1]
        / "share/metainfo/org.mavlink.qgroundcontrol.metainfo.xml"
    )
    metadata.write_text(
        "<component type='desktop-application'>"
        "<id>org.mavlink.qgroundcontrol</id>"
        "<releases>"
        "<release version='5.1.0'/>"
        "<release version='5.0.8'/>"
        "</releases>"
        "</component>",
        encoding="utf-8",
    )
    settings_file = tmp_path / "QGroundControl.ini"
    _settings(settings_file, tmp_path / "save")
    host = PosixHost(
        tmp_path,
        HostConfig("linux", str(executable), str(settings_file)),
        _facts(),
        home=tmp_path,
        environ={},
    )

    with pytest.raises(RuntimeError, match="version 5.0.8"):
        host.inspect_qgc()


def test_linux_rejects_wrong_native_elf_architecture(tmp_path: Path) -> None:
    executable = _linux_installation(tmp_path / "QGroundControl.AppDir", "arm64")
    settings_file = tmp_path / "QGroundControl.ini"
    _settings(settings_file, tmp_path / "save")
    host = PosixHost(
        tmp_path,
        HostConfig("linux", str(executable), str(settings_file)),
        _facts("linux", "amd64"),
        home=tmp_path,
        environ={},
    )

    with pytest.raises(RuntimeError, match="architecture"):
        host.inspect_qgc()


def test_linux_rejects_loose_binary_without_paired_metadata(tmp_path: Path) -> None:
    executable = tmp_path / "QGroundControl"
    _elf(executable, "amd64")
    settings_file = tmp_path / "QGroundControl.ini"
    _settings(settings_file, tmp_path / "save")
    host = PosixHost(
        tmp_path,
        HostConfig("linux", str(executable), str(settings_file)),
        _facts(),
        home=tmp_path,
        environ={},
    )

    with pytest.raises(RuntimeError, match="paired AppStream"):
        host.inspect_qgc()


def test_linux_official_appimage_uses_pinned_bytes_without_execution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    appimage = tmp_path / "renamed"
    payload = b"official fixture"
    appimage.write_bytes(payload)
    appimage.chmod(appimage.stat().st_mode | stat.S_IXUSR)
    settings_file = tmp_path / "QGroundControl.ini"
    _settings(settings_file, tmp_path / "save")
    monkeypatch.setattr(posix_module, "_OFFICIAL_APPIMAGE_SIZE", len(payload))
    monkeypatch.setattr(
        posix_module,
        "_OFFICIAL_APPIMAGE_SHA256",
        __import__("hashlib").sha256(payload).hexdigest(),
    )
    commands: list[list[str]] = []
    host = PosixHost(
        tmp_path,
        HostConfig("linux", str(appimage), str(settings_file)),
        _facts(),
        home=tmp_path,
        environ={},
        runner=lambda command, **_kwargs: commands.append(command),
    )

    assert host.inspect_qgc().version == "5.0.8"
    assert commands == []


def test_qgc_artifact_is_revalidated_immediately_before_launch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    appimage = tmp_path / "renamed"
    payload = b"official fixture"
    appimage.write_bytes(payload)
    appimage.chmod(appimage.stat().st_mode | stat.S_IXUSR)
    settings_file = tmp_path / "QGroundControl.ini"
    _settings(settings_file, tmp_path / "save")
    monkeypatch.setattr(posix_module, "_OFFICIAL_APPIMAGE_SIZE", len(payload))
    monkeypatch.setattr(
        posix_module,
        "_OFFICIAL_APPIMAGE_SHA256",
        __import__("hashlib").sha256(payload).hexdigest(),
    )
    launches: list[list[str]] = []
    host = PosixHost(
        tmp_path,
        HostConfig("linux", str(appimage), str(settings_file)),
        _facts(),
        home=tmp_path,
        environ={},
        launcher=lambda command, **_kwargs: launches.append(command),
    )
    installation = host.inspect_qgc()
    appimage.write_bytes(b"modified fixture")

    with pytest.raises(RuntimeError, match="AppStream|identity|version"):
        host.open_qgc(installation)
    assert launches == []


def test_linux_launch_uses_argument_array_and_returns_without_waiting(
    tmp_path: Path,
) -> None:
    executable = _linux_installation(tmp_path / "QGroundControl.AppDir")
    settings_file = tmp_path / "QGroundControl.ini"
    _settings(settings_file, tmp_path / "save")
    launches: list[tuple[list[str], dict[str, object]]] = []

    def launcher(command, **kwargs):
        launches.append((command, kwargs))
        return SimpleNamespace(pid=123)

    host = PosixHost(
        tmp_path,
        HostConfig("linux", str(executable), str(settings_file)),
        _facts(),
        home=tmp_path,
        environ={},
        launcher=launcher,
    )
    result = host.open_qgc(host.inspect_qgc())

    assert result.opened is True
    assert launches[0][0] == [str(executable)]
    assert "--version" not in launches[0][0]
    assert launches[0][1]["start_new_session"] is True


@pytest.mark.parametrize(
    ("extra", "message"),
    [
        ("SettingsVersion=8\n", "SettingsVersion"),
        ("gcsMavlinkSystemID=254\n", "system ID"),
    ],
)
def test_discovery_rejects_incompatible_qgc_settings(
    tmp_path: Path, extra: str, message: str,
) -> None:
    executable = _linux_installation(tmp_path / "QGroundControl.AppDir")
    settings_file = tmp_path / "QGroundControl.ini"
    _settings(settings_file, tmp_path / "save", extra_general=extra)
    if extra.startswith("SettingsVersion="):
        settings_file.write_bytes(
            settings_file.read_bytes().replace(
                b"SettingsVersion=9\n",
                b"",
                1,
            )
        )
    host = PosixHost(
        tmp_path,
        HostConfig("linux", str(executable), str(settings_file)),
        _facts(),
        home=tmp_path,
        environ={},
    )

    with pytest.raises(RuntimeError, match=message):
        host.inspect_qgc()


def test_discovery_rejects_missing_and_ambiguous_settings(tmp_path: Path) -> None:
    executable = _linux_installation(tmp_path / "QGroundControl.AppDir")
    missing = tmp_path / "missing.ini"
    host = PosixHost(
        tmp_path,
        HostConfig("linux", str(executable), str(missing)),
        _facts(),
        home=tmp_path,
        environ={},
    )
    with pytest.raises(RuntimeError, match=str(missing)):
        host.inspect_qgc()

    ambiguous = tmp_path / "ambiguous.ini"
    ambiguous.write_text(
        "[General]\nSettingsVersion=9\nsavePath=/one\nsavePath=/two\n",
        encoding="utf-8",
    )
    host = PosixHost(
        tmp_path,
        HostConfig("linux", str(executable), str(ambiguous)),
        _facts(),
        home=tmp_path,
        environ={},
    )
    with pytest.raises(RuntimeError, match="ambiguous"):
        host.inspect_qgc()


def test_qgc_system_id_uses_general_then_legacy_then_default(tmp_path: Path) -> None:
    executable = _linux_installation(tmp_path / "QGroundControl.AppDir")
    cases = (
        ("gcsMavlinkSystemID=255\n", "GCS_SYSTEM_ID=254\n"),
        ("", "GCS_SYSTEM_ID=255\n"),
        ("", ""),
    )
    for index, (general, legacy) in enumerate(cases):
        settings_file = tmp_path / f"settings-{index}.ini"
        _settings(settings_file, tmp_path / "save", extra_general=general)
        if legacy:
            with settings_file.open("a", encoding="utf-8") as target:
                target.write(f"\n[QGC_MAVLINK_PROTOCOL]\n{legacy}")
        host = PosixHost(
            tmp_path,
            HostConfig("linux", str(executable), str(settings_file)),
            _facts(),
            home=tmp_path,
            environ={},
        )

        assert host.inspect_qgc().source == QGC_SOURCE


def test_resolver_keeps_wsl_unavailable_until_windows_adapter(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = HostConfig("wsl2", None, None)
    facts = HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")
    monkeypatch.setattr(host_integration_module, "load_host_config", lambda *_args: config)
    monkeypatch.setattr(host_integration_module, "detect_host", lambda _config: facts)

    with pytest.raises(ValueError, match="not available yet"):
        host_integration_module.resolve_host(tmp_path)


def test_mac_open_command_focuses_selected_existing_application(tmp_path: Path) -> None:
    bundle, executable = _mac_installation(tmp_path)
    settings_file = tmp_path / "QGroundControl.ini"
    _settings(settings_file, tmp_path / "save")
    calls: list[list[str]] = []

    def runner(command, **_kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    host = PosixHost(
        tmp_path,
        HostConfig("macos", str(bundle), str(settings_file)),
        _facts("macos", "arm64"),
        home=tmp_path,
        environ={},
        runner=runner,
    )
    result = host.open_qgc(host.inspect_qgc())

    assert result.opened is True
    assert calls == [["open", "-a", str(bundle)]]
    assert str(executable) not in calls[0]


def test_posix_udp_ignores_unpinned_peer_and_close_is_idempotent(
    tmp_path: Path,
) -> None:
    pinned = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pinned.bind(("127.0.0.1", 0))
    unpinned = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    host = PosixHost(
        tmp_path,
        HostConfig("linux", None, None),
        _facts(),
        home=tmp_path,
        environ={},
        qgc_port=pinned.getsockname()[1],
    )
    publication = SimpleNamespace(native_path="/save/action", filename="action", sha256="0" * 64)

    with host.acquire_native_lease() as lease:
        transport = host.open_datagrams(lease, RUN_ID, publication)
        transport.send(b"outbound")
        outbound, transport_address = pinned.recvfrom(65535)
        unpinned.sendto(b"foreign", transport_address)
        pinned.sendto(b"pinned", transport_address)
        received = [transport.receive(1)]
        assert transport.receive(0.01) is None
        transport.close()
        transport.close()

    assert outbound == b"outbound"
    assert received == [b"pinned"]
    with pytest.raises(HostTransportError, match="closed"):
        transport.send(b"late")
    pinned.close()
    unpinned.close()


class _SocketFailureFixture:
    def __init__(
        self,
        *,
        bind_error: OSError | None = None,
        timeout_error: OSError | None = None,
    ) -> None:
        self.bind_error = bind_error
        self.timeout_error = timeout_error
        self.closed = False

    def bind(self, _address: tuple[str, int]) -> None:
        if self.bind_error is not None:
            raise self.bind_error

    def settimeout(self, _timeout_s: float) -> None:
        if self.timeout_error is not None:
            raise self.timeout_error

    def close(self) -> None:
        self.closed = True


def _transport_host(tmp_path: Path) -> PosixHost:
    return PosixHost(
        tmp_path,
        HostConfig("linux", None, None),
        _facts(),
        home=tmp_path,
        environ={},
    )


def _publication() -> SimpleNamespace:
    return SimpleNamespace(
        native_path="/save/action",
        filename="action",
        sha256="0" * 64,
    )


def test_posix_udp_translates_socket_initialization_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    def fail_socket(*_args: object, **_kwargs: object) -> None:
        raise OSError("socket unavailable")

    monkeypatch.setattr(posix_module.socket, "socket", fail_socket)
    host = _transport_host(tmp_path)

    with host.acquire_native_lease() as lease:
        with pytest.raises(HostTransportError, match="creation failed") as caught:
            host.open_datagrams(lease, RUN_ID, _publication())

    assert isinstance(caught.value.__cause__, OSError)


def test_posix_udp_closes_socket_and_translates_bind_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    failed_socket = _SocketFailureFixture(bind_error=OSError("bind denied"))
    monkeypatch.setattr(
        posix_module.socket,
        "socket",
        lambda *_args, **_kwargs: failed_socket,
    )
    host = _transport_host(tmp_path)

    with host.acquire_native_lease() as lease:
        with pytest.raises(HostTransportError, match="bind failed") as caught:
            host.open_datagrams(lease, RUN_ID, _publication())

    assert isinstance(caught.value.__cause__, OSError)
    assert failed_socket.closed is True


def test_posix_udp_translates_receive_setup_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    failed_socket = _SocketFailureFixture(
        timeout_error=OSError("timeout setup failed")
    )
    monkeypatch.setattr(
        posix_module.socket,
        "socket",
        lambda *_args, **_kwargs: failed_socket,
    )
    host = _transport_host(tmp_path)

    with host.acquire_native_lease() as lease:
        transport = host.open_datagrams(lease, RUN_ID, _publication())
        with pytest.raises(HostTransportError, match="receive failed") as caught:
            transport.receive(1)
        transport.close()

    assert isinstance(caught.value.__cause__, OSError)
    assert failed_socket.closed is True
