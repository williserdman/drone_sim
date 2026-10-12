from __future__ import annotations

from dataclasses import FrozenInstanceError
import json
from pathlib import Path
from types import ModuleType

import pytest

from orchestration import host_config as host_config_module


@pytest.fixture
def host_config() -> ModuleType:
    return host_config_module


def _write_config(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document), encoding="utf-8")


def _set_host(monkeypatch: pytest.MonkeyPatch, host_config: ModuleType, *, system: str, machine: str, release: str) -> None:
    monkeypatch.setattr(host_config.platform, "system", lambda: system)
    monkeypatch.setattr(host_config.platform, "machine", lambda: machine)
    monkeypatch.setattr(host_config.platform, "release", lambda: release)


def _mount_line(mount_point: Path, filesystem: str) -> str:
    encoded = str(mount_point).replace("\\", r"\134").replace(" ", r"\040")
    return f"36 25 0:32 / {encoded} rw,relatime - {filesystem} source rw\n"


def _use_mountinfo(monkeypatch: pytest.MonkeyPatch, host_config: ModuleType, path: Path, contents: str) -> None:
    path.write_text(contents, encoding="utf-8")
    monkeypatch.setattr(host_config, "_MOUNTINFO_PATH", path)


def test_missing_default_config_uses_auto_discovery(tmp_path: Path, host_config: ModuleType):
    config = host_config.load_host_config(tmp_path)

    assert config == host_config.HostConfig(
        platform="auto",
        qgc_executable=None,
        qgc_settings_file=None,
    )


def test_existing_default_config_is_loaded_from_project_directory(tmp_path: Path, host_config: ModuleType):
    _write_config(
        tmp_path / ".local-runtime/host.json",
        {"schema_version": 1, "platform": "linux"},
    )

    config = host_config.load_host_config(tmp_path)

    assert config == host_config.HostConfig(
        platform="linux",
        qgc_executable=None,
        qgc_settings_file=None,
    )


def test_broken_default_config_symlink_is_rejected(tmp_path: Path, host_config: ModuleType):
    path = tmp_path / ".local-runtime/host.json"
    path.parent.mkdir()
    path.symlink_to(tmp_path / "missing-target.json")

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.load_host_config(tmp_path)

    assert str(path) in str(caught.value)
    assert "repair" in str(caught.value).lower()


def test_explicit_config_preserves_native_windows_path_strings(tmp_path: Path, host_config: ModuleType):
    path = tmp_path / "operator host.json"
    _write_config(
        path,
        {
            "schema_version": 1,
            "platform": "wsl2",
            "qgc": {
                "executable": r"C:\Program Files\Équipe\QGroundControl.exe",
                "settings_file": r"C:\Users\Zoë\AppData\Roaming\QGroundControl\QGroundControl.ini",
            },
        },
    )

    config = host_config.load_host_config(tmp_path, path)

    assert config.qgc_executable == r"C:\Program Files\Équipe\QGroundControl.exe"
    assert config.qgc_settings_file == r"C:\Users\Zoë\AppData\Roaming\QGroundControl\QGroundControl.ini"


def test_missing_explicit_config_reports_path_and_remediation(tmp_path: Path, host_config: ModuleType):
    path = tmp_path / "missing.json"

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.load_host_config(tmp_path, path)

    assert str(path) in str(caught.value)
    assert "create" in str(caught.value).lower()


@pytest.mark.parametrize("unknown", ["provider", "qgc_settings"])
def test_unknown_top_level_keys_are_rejected(tmp_path: Path, host_config: ModuleType, unknown: str):
    path = tmp_path / "host.json"
    _write_config(path, {"schema_version": 1, "platform": "auto", unknown: "value"})

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.load_host_config(tmp_path, path)

    assert unknown in str(caught.value)
    assert "remove" in str(caught.value).lower()


def test_unknown_qgc_key_is_rejected(tmp_path: Path, host_config: ModuleType):
    path = tmp_path / "host.json"
    _write_config(
        path,
        {"schema_version": 1, "platform": "auto", "qgc": {"save_path": "C:\\QGC"}},
    )

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.load_host_config(tmp_path, path)

    assert "qgc.save_path" in str(caught.value)
    assert "remove" in str(caught.value).lower()


@pytest.mark.parametrize(
    ("document", "field"),
    [
        ({"schema_version": True, "platform": "auto"}, "schema_version"),
        ({"schema_version": 1, "platform": 2}, "platform"),
        ({"schema_version": 1, "platform": "auto", "qgc": []}, "qgc"),
        (
            {"schema_version": 1, "platform": "auto", "qgc": {"executable": 4}},
            "qgc.executable",
        ),
        (
            {"schema_version": 1, "platform": "auto", "qgc": {"settings_file": False}},
            "qgc.settings_file",
        ),
    ],
)
def test_wrong_scalar_types_are_rejected(
    tmp_path: Path,
    host_config: ModuleType,
    document: object,
    field: str,
):
    path = tmp_path / "host.json"
    _write_config(path, document)

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.load_host_config(tmp_path, path)

    assert field in str(caught.value)
    assert str(path) in str(caught.value)


@pytest.mark.parametrize("value", [None, "", "windows", "MacOS"])
def test_invalid_platform_values_are_rejected(tmp_path: Path, host_config: ModuleType, value: object):
    path = tmp_path / "host.json"
    _write_config(path, {"schema_version": 1, "platform": value})

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.load_host_config(tmp_path, path)

    assert "platform" in str(caught.value)
    assert "auto" in str(caught.value)


def test_unsupported_schema_version_is_rejected(tmp_path: Path, host_config: ModuleType):
    path = tmp_path / "host.json"
    _write_config(path, {"schema_version": 2, "platform": "auto"})

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.load_host_config(tmp_path, path)

    assert "schema_version" in str(caught.value)
    assert "1" in str(caught.value)


def test_malformed_json_reports_selected_file(tmp_path: Path, host_config: ModuleType):
    path = tmp_path / "host.json"
    path.write_text('{"schema_version":', encoding="utf-8")

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.load_host_config(tmp_path, path)

    assert str(path) in str(caught.value)
    assert "json" in str(caught.value).lower()


def test_non_object_document_is_rejected(tmp_path: Path, host_config: ModuleType):
    path = tmp_path / "host.json"
    _write_config(path, [{"schema_version": 1, "platform": "auto"}])

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.load_host_config(tmp_path, path)

    assert str(path) in str(caught.value)
    assert "object" in str(caught.value)


@pytest.mark.parametrize(
    ("value", "field"),
    [
        pytest.param(host_config_module.HostConfig("auto", None, None), "platform", id="config"),
        pytest.param(
            host_config_module.HostFacts("linux", "amd64", "linux/amd64", "docker"),
            "kind",
            id="facts",
        ),
    ],
)
def test_host_values_are_immutable(value: object, field: str):
    with pytest.raises(FrozenInstanceError):
        setattr(value, field, "wsl2")


def test_explicit_platform_must_match_detected_host(
    tmp_path: Path,
    host_config: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
):
    path = tmp_path / "host.json"
    _write_config(path, {"schema_version": 1, "platform": "linux"})
    config = host_config.load_host_config(tmp_path, path)
    _set_host(monkeypatch, host_config, system="Darwin", machine="arm64", release="25.0.0")

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.detect_host(config)

    assert "platform" in str(caught.value)
    assert "macos" in str(caught.value)
    assert "auto" in str(caught.value)


def test_wsl_is_detected_before_linux(host_config: ModuleType, monkeypatch: pytest.MonkeyPatch):
    _set_host(
        monkeypatch,
        host_config,
        system="Linux",
        machine="x86_64",
        release="6.6.87.2-microsoft-standard-WSL2",
    )

    facts = host_config.detect_host(host_config.HostConfig("auto", None, None))

    assert facts.kind == "wsl2"
    assert facts.machine == "amd64"
    assert facts.image_platform == "linux/amd64"
    assert facts.provider == "docker-desktop"


def test_wsl1_is_rejected_instead_of_classified_as_wsl2(
    host_config: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
):
    _set_host(
        monkeypatch,
        host_config,
        system="Linux",
        machine="x86_64",
        release="4.4.0-19041-Microsoft",
    )

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.detect_host(host_config.HostConfig("auto", None, None))

    assert "WSL1" in str(caught.value)
    assert "WSL2" in str(caught.value)
    assert "4.4.0-19041-Microsoft" in str(caught.value)


@pytest.mark.parametrize(
    ("system", "machine", "release", "kind", "normalized", "image", "provider"),
    [
        ("Darwin", "x86_64", "25.0.0", "macos", "amd64", "linux/amd64", "colima"),
        ("Darwin", "aarch64", "25.0.0", "macos", "arm64", "linux/arm64", "colima"),
        ("Linux", "AMD64", "6.8.0-generic", "linux", "amd64", "linux/amd64", "docker"),
        ("Linux", "arm64", "6.8.0-generic", "linux", "arm64", "linux/arm64", "docker"),
    ],
)
def test_supported_hosts_resolve_native_image_platform(
    host_config: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
    system: str,
    machine: str,
    release: str,
    kind: str,
    normalized: str,
    image: str,
    provider: str,
):
    _set_host(monkeypatch, host_config, system=system, machine=machine, release=release)

    facts = host_config.detect_host(host_config.HostConfig("auto", None, None))

    assert facts == host_config.HostFacts(kind, normalized, image, provider)


@pytest.mark.parametrize(
    ("system", "machine", "release", "expected"),
    [
        ("FreeBSD", "amd64", "14.1", "FreeBSD"),
        ("Windows", "AMD64", "11", "Windows"),
        ("Linux", "riscv64", "6.8.0", "riscv64"),
        ("Linux", "aarch64", "6.6.87.2-microsoft-standard-WSL2", "arm64"),
    ],
)
def test_unsupported_host_or_cpu_reports_remediation(
    host_config: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
    system: str,
    machine: str,
    release: str,
    expected: str,
):
    _set_host(monkeypatch, host_config, system=system, machine=machine, release=release)

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.detect_host(host_config.HostConfig("auto", None, None))

    assert expected in str(caught.value)
    assert "supported" in str(caught.value).lower()


def test_non_wsl_hosts_do_not_apply_wsl_storage_rules(
    tmp_path: Path,
    host_config: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
):
    missing_mountinfo = tmp_path / "absent-mountinfo"
    monkeypatch.setattr(host_config, "_MOUNTINFO_PATH", missing_mountinfo)
    facts = host_config.HostFacts("linux", "amd64", "linux/amd64", "docker")

    host_config.validate_state_paths(facts, [tmp_path / "runs"])


def test_wsl_accepts_nonexistent_state_path_on_supported_linux_mount(
    tmp_path: Path,
    host_config: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
):
    mount = tmp_path / "Linux state"
    mount.mkdir()
    mountinfo = tmp_path / "mountinfo"
    _use_mountinfo(monkeypatch, host_config, mountinfo, _mount_line(mount, "ext4"))
    facts = host_config.HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")

    host_config.validate_state_paths(facts, [mount / "drone-sim" / "tokens"])


@pytest.mark.parametrize("filesystem", ["drvfs", "9p"])
def test_wsl_rejects_windows_storage_before_allocation(
    tmp_path: Path,
    host_config: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
    filesystem: str,
):
    linux_mount = tmp_path / "linux"
    windows_mount = tmp_path / "windows mount"
    linux_mount.mkdir()
    windows_mount.mkdir()
    mountinfo = tmp_path / "mountinfo"
    _use_mountinfo(
        monkeypatch,
        host_config,
        mountinfo,
        _mount_line(linux_mount, "ext4") + _mount_line(windows_mount, filesystem),
    )
    facts = host_config.HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")
    allocations: list[str] = []
    token_writes: list[str] = []

    def allocate() -> None:
        host_config.validate_state_paths(facts, [linux_mount / "checkout", windows_mount / "tokens"])
        allocations.append("run")
        token_writes.append("token")

    with pytest.raises(host_config.HostConfigurationError) as caught:
        allocate()

    assert str(windows_mount / "tokens") in str(caught.value)
    assert filesystem in str(caught.value)
    assert "linux filesystem" in str(caught.value).lower()
    assert allocations == []
    assert token_writes == []


def test_wsl_rejects_unsupported_linux_filesystem(
    tmp_path: Path,
    host_config: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
):
    mount = tmp_path / "volatile"
    mount.mkdir()
    mountinfo = tmp_path / "mountinfo"
    _use_mountinfo(monkeypatch, host_config, mountinfo, _mount_line(mount, "tmpfs"))
    facts = host_config.HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.validate_state_paths(facts, [mount / "runs"])

    assert "tmpfs" in str(caught.value)
    assert "ext4" in str(caught.value)


def test_wsl_resolves_symlink_before_selecting_containing_mount(
    tmp_path: Path,
    host_config: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
):
    linux_mount = tmp_path / "linux"
    windows_mount = tmp_path / "windows"
    linux_mount.mkdir()
    windows_mount.mkdir()
    (linux_mount / "linked").symlink_to(windows_mount, target_is_directory=True)
    mountinfo = tmp_path / "mountinfo"
    _use_mountinfo(
        monkeypatch,
        host_config,
        mountinfo,
        _mount_line(tmp_path, "ext4") + _mount_line(windows_mount, "drvfs"),
    )
    facts = host_config.HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.validate_state_paths(facts, [linux_mount / "linked" / "runs"])

    assert "drvfs" in str(caught.value)
    assert str(linux_mount / "linked" / "runs") in str(caught.value)


def test_wsl_rejects_path_when_mount_cannot_be_identified(
    tmp_path: Path,
    host_config: ModuleType,
    monkeypatch: pytest.MonkeyPatch,
):
    mountinfo = tmp_path / "mountinfo"
    _use_mountinfo(monkeypatch, host_config, mountinfo, "")
    facts = host_config.HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")

    with pytest.raises(host_config.HostConfigurationError) as caught:
        host_config.validate_state_paths(facts, [tmp_path / "runs"])

    assert str(tmp_path / "runs") in str(caught.value)
    assert "mount" in str(caught.value).lower()
