"""Strict host configuration, platform detection, and WSL state-path checks."""

from __future__ import annotations

from dataclasses import dataclass
import json
import platform
from pathlib import Path
from typing import Literal, Sequence, cast


ConfiguredPlatform = Literal["auto", "macos", "linux", "wsl2"]
HostKind = Literal["macos", "linux", "wsl2"]
HostMachine = Literal["arm64", "amd64"]
ImagePlatform = Literal["linux/arm64", "linux/amd64"]

_CONFIGURED_PLATFORMS = frozenset({"auto", "macos", "linux", "wsl2"})
_ARCHITECTURE_ALIASES: dict[str, HostMachine] = {
    "aarch64": "arm64",
    "arm64": "arm64",
    "amd64": "amd64",
    "x86_64": "amd64",
}
_PROVIDERS: dict[HostKind, str] = {
    "macos": "docker-desktop",
    "linux": "docker",
    "wsl2": "docker-desktop",
}
_SUPPORTED_WSL_STATE_FILESYSTEMS = frozenset({"ext4"})
_WINDOWS_MOUNT_FILESYSTEMS = frozenset({"9p", "drvfs"})
_MOUNTINFO_PATH = Path("/proc/self/mountinfo")


class HostConfigurationError(ValueError):
    """An external host setting or detected host fact is not supported."""


@dataclass(frozen=True, slots=True)
class HostConfig:
    platform: ConfiguredPlatform
    qgc_executable: str | None
    qgc_settings_file: str | None


@dataclass(frozen=True, slots=True)
class HostFacts:
    kind: HostKind
    machine: HostMachine
    image_platform: ImagePlatform
    provider: str


@dataclass(frozen=True, slots=True)
class _Mount:
    point: Path
    filesystem: str


def load_host_config(project_directory: Path, config_path: Path | None = None) -> HostConfig:
    """Load a version-1 host document without interpreting native QGC paths."""

    selected = (
        config_path
        if config_path is not None
        else project_directory / ".local-runtime/host.json"
    )
    if not selected.exists():
        if selected.is_symlink():
            raise HostConfigurationError(
                f"host config {selected} is a broken symlink; repair or remove the link"
            )
        if config_path is None:
            return HostConfig(platform="auto", qgc_executable=None, qgc_settings_file=None)
        raise HostConfigurationError(
            f"host config {selected} does not exist; create that file or omit "
            "--host-config to use automatic discovery"
        )
    if not selected.is_file():
        raise HostConfigurationError(
            f"host config {selected} is not a regular file; select a readable version-1 JSON file"
        )

    try:
        document: object = json.loads(selected.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise HostConfigurationError(
            f"host config {selected} is not readable UTF-8 JSON; "
            f"repair the file or select another path: {error}"
        ) from error
    if not isinstance(document, dict):
        raise HostConfigurationError(
            f"host config {selected} must contain a JSON object; use schema_version, "
            "platform, and optional qgc fields"
        )

    allowed_top = {"schema_version", "platform", "qgc"}
    unknown_top = set(document) - allowed_top
    if unknown_top:
        field = sorted(str(key) for key in unknown_top)[0]
        raise HostConfigurationError(
            f"host config {selected} has unknown field {field}; remove it and use "
            "only schema_version, platform, and qgc"
        )

    schema_version = document.get("schema_version")
    if type(schema_version) is not int or schema_version != 1:
        raise HostConfigurationError(
            f"host config {selected} field schema_version must be integer 1; "
            "update the document to the version-1 schema"
        )

    selected_platform = document.get("platform")
    if not isinstance(selected_platform, str) or selected_platform not in _CONFIGURED_PLATFORMS:
        raise HostConfigurationError(
            f"host config {selected} field platform must be one of auto, macos, "
            "linux, or wsl2; correct the field"
        )

    qgc_document = document.get("qgc", {})
    if not isinstance(qgc_document, dict):
        raise HostConfigurationError(
            f"host config {selected} field qgc must be an object; use executable "
            "and settings_file string fields"
        )
    allowed_qgc = {"executable", "settings_file"}
    unknown_qgc = set(qgc_document) - allowed_qgc
    if unknown_qgc:
        field = sorted(str(key) for key in unknown_qgc)[0]
        raise HostConfigurationError(
            f"host config {selected} has unknown field qgc.{field}; remove it and "
            "use only executable and settings_file"
        )

    executable = _optional_native_path(selected, qgc_document, "executable")
    settings_file = _optional_native_path(selected, qgc_document, "settings_file")
    return HostConfig(
        platform=cast(ConfiguredPlatform, selected_platform),
        qgc_executable=executable,
        qgc_settings_file=settings_file,
    )


def detect_host(config: HostConfig) -> HostFacts:
    """Resolve the supported host and its native Linux image architecture."""

    system = platform.system()
    release = platform.release()
    if system == "Darwin":
        kind: HostKind = "macos"
    elif system == "Linux":
        kind = _linux_host_kind(release)
    else:
        raise HostConfigurationError(
            f"detected unsupported host {system}; run the controller on macOS, "
            "Ubuntu Linux, or Ubuntu under WSL2"
        )

    if config.platform != "auto" and config.platform != kind:
        raise HostConfigurationError(
            f"host config field platform selects {config.platform}, but detection "
            f"found {kind}; set platform to {kind} or auto"
        )

    raw_machine = platform.machine()
    machine = _ARCHITECTURE_ALIASES.get(raw_machine.lower())
    if machine is None:
        raise HostConfigurationError(
            f"detected unsupported CPU {raw_machine}; use an arm64 or amd64 host "
            "from the supported platform matrix"
        )
    if kind == "wsl2" and machine != "amd64":
        raise HostConfigurationError(
            f"detected unsupported WSL2 CPU {machine}; use Windows 11 x86-64 "
            "with an amd64 WSL2 distribution"
        )

    image_platform: ImagePlatform = "linux/arm64" if machine == "arm64" else "linux/amd64"
    return HostFacts(
        kind=kind,
        machine=machine,
        image_platform=image_platform,
        provider=_PROVIDERS[kind],
    )


def validate_state_paths(facts: HostFacts, paths: Sequence[Path]) -> None:
    """Reject WSL state paths that resolve outside the distribution's ext4 storage."""

    if facts.kind != "wsl2":
        return
    mounts = _read_mounts()
    for supplied in paths:
        try:
            resolved = supplied.resolve(strict=False)
        except (OSError, RuntimeError) as error:
            raise HostConfigurationError(
                f"state path {supplied} cannot be resolved safely; repair its "
                f"parent directories or symlinks: {error}"
            ) from error
        containing = _containing_mount(resolved, mounts)
        if containing is None:
            raise HostConfigurationError(
                f"state path {supplied} resolves to {resolved}, but its containing "
                "mount cannot be identified; keep the checkout and all simulator "
                "state inside the WSL distribution's ext4 filesystem"
            )
        if containing.filesystem in _WINDOWS_MOUNT_FILESYSTEMS:
            raise HostConfigurationError(
                f"state path {supplied} resolves to {resolved} on "
                f"{containing.filesystem} Windows-mounted storage; "
                "move the checkout and all simulator state to the WSL Linux filesystem"
            )
        if containing.filesystem not in _SUPPORTED_WSL_STATE_FILESYSTEMS:
            raise HostConfigurationError(
                f"state path {supplied} resolves to {resolved} on unsupported "
                f"filesystem {containing.filesystem}; "
                "move it to the WSL distribution's ext4 filesystem"
            )


def _optional_native_path(
    config_path: Path,
    qgc_document: dict[object, object],
    field: str,
) -> str | None:
    if field not in qgc_document:
        return None
    value = qgc_document[field]
    if not isinstance(value, str) or not value:
        raise HostConfigurationError(
            f"host config {config_path} field qgc.{field} must be a non-empty string; "
            "provide the native QGC path or remove the field for automatic discovery"
        )
    return value


def _linux_host_kind(release: str) -> HostKind:
    lowered = release.lower()
    if "microsoft" not in lowered and "wsl" not in lowered:
        return "linux"
    if "microsoft-standard" in lowered or "wsl2" in lowered:
        return "wsl2"
    raise HostConfigurationError(
        f"detected unsupported WSL1 kernel release {release}; convert the "
        "distribution to WSL2 before running the controller"
    )


def _read_mounts() -> tuple[_Mount, ...]:
    try:
        lines = _MOUNTINFO_PATH.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        raise HostConfigurationError(
            f"cannot inspect WSL mounts at {_MOUNTINFO_PATH}; restore procfs before "
            f"creating simulator state: {error}"
        ) from error

    mounts: list[_Mount] = []
    for line in lines:
        before, separator, after = line.partition(" - ")
        fields = before.split()
        after_fields = after.split()
        if not separator or len(fields) < 5 or not after_fields:
            continue
        point = Path(_decode_mount_field(fields[4])).resolve(strict=False)
        mounts.append(_Mount(point=point, filesystem=after_fields[0].lower()))
    return tuple(mounts)


def _decode_mount_field(value: str) -> str:
    return (
        value.replace(r"\040", " ")
        .replace(r"\011", "\t")
        .replace(r"\012", "\n")
        .replace(r"\134", "\\")
    )


def _containing_mount(path: Path, mounts: Sequence[_Mount]) -> _Mount | None:
    matches = (mount for mount in mounts if path == mount.point or mount.point in path.parents)
    return max(matches, key=lambda mount: len(mount.point.parts), default=None)
