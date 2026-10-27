#!/usr/bin/env python3
"""Resolve exact ARM64 or AMD64 apt deltas without executing foreign binaries.

The pinned host-native ROS image supplies apt, while an isolated target package
database supplies the base state. Refreshing unavailable Ubuntu versions is an
explicit maintenance operation; ordinary builds keep their exact package locks.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import UTC, datetime
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tarfile
import urllib.parse
import urllib.request


ROS_IMAGE = (
    "ros:jazzy-ros-base@"
    "sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4"
)
ROS_INDEX_DIGEST = "sha256:2589a8fba5257307857890173c069852c2abf913a0be7970f172478baecb09e4"
REGISTRY_REPOSITORY = "library/ros"
REGISTRY = "https://registry-1.docker.io"

APT_SHELL = r"""
set -euo pipefail
apt_options=(
  -o APT::Architecture="${AUDIT_ARCHITECTURE}"
  -o APT::Architectures::="${AUDIT_ARCHITECTURE}"
  -o Dir::State=/audit/state
  -o Dir::State::status="/audit/${AUDIT_STATUS}"
  -o Dir::State::lists=/audit/state/lists
  -o Dir::Cache=/audit/cache
  -o Dir::Cache::archives=/audit/cache/archives
  -o Dir::Etc::sourcelist=/audit/sources.list
  -o Dir::Etc::sourceparts=/audit/sourceparts
  -o Debug::NoLocking=true
  -o Acquire::Retries=5
  -o Acquire::http::Timeout=30
  -o APT::Update::Error-Mode=any
  -o APT::Sandbox::User=root
)

case "${AUDIT_ACTION}" in
  architecture)
    machine="$(uname -m)"
    case "${AUDIT_UTILITY_ARCHITECTURE}:${machine}" in
      arm64:aarch64|arm64:arm64|amd64:x86_64) ;;
      *) printf 'unexpected utility architecture: %s\n' "${machine}" >&2; exit 2 ;;
    esac
    printf '%s\n' "${machine}"
    ;;
  copy-ros-source)
    cp /etc/apt/sources.list.d/ros2.sources /audit/sourceparts/ros2.sources
    ;;
  update)
    apt-get "${apt_options[@]}" update
    ;;
  simulate)
    mapfile -t packages < "/audit/${AUDIT_ROOTS}"
    apt-get "${apt_options[@]}" \
      --simulate --no-install-recommends --no-remove \
      install "${packages[@]}"
    ;;
  show)
    mapfile -t packages < "/audit/${AUDIT_ROOTS}"
    apt-cache "${apt_options[@]}" show "${packages[@]}"
    ;;
  policy)
    mapfile -t packages < <(cut -d= -f1 "/audit/${AUDIT_ROOTS}")
    apt-cache "${apt_options[@]}" policy "${packages[@]}"
    ;;
  check-upgrades)
    while read -r package previous next; do
      if ! dpkg --compare-versions "${next}" gt "${previous}"; then
        printf 'replacement is not an upgrade: %s %s -> %s\n' \
          "${package}" "${previous}" "${next}" >&2
        exit 2
      fi
    done < "/audit/${AUDIT_ROOTS}"
    ;;
  download)
    mapfile -t packages < "/audit/${AUDIT_ROOTS}"
    apt-get "${apt_options[@]}" \
      --yes --download-only --no-install-recommends --no-remove \
      install "${packages[@]}"
    ;;
  *)
    printf 'unsupported audit action: %s\n' "${AUDIT_ACTION}" >&2
    exit 2
    ;;
esac
"""


@dataclass(frozen=True)
class PackageRecord:
    package: str
    version: str
    architecture: str
    stanza: str
    filename: str
    size: int
    sha256: str

    @property
    def lock_row(self) -> str:
        return f"{self.package}={self.version}"


def _run(
    arguments: list[str],
    *,
    check: bool = True,
    capture_output: bool = True,
    timeout: float = 120,
) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            arguments,
            check=check,
            capture_output=capture_output,
            text=True,
            timeout=timeout,
        )
    except subprocess.CalledProcessError as error:
        raise RuntimeError(
            f"command failed with exit {error.returncode}: {arguments[:8]}\n"
            f"stdout:\n{error.stdout}\nstderr:\n{error.stderr}"
        ) from error


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _registry_request(path: str, *, accept: str | None = None) -> bytes:
    query = urllib.parse.urlencode(
        {
            "service": "registry.docker.io",
            "scope": f"repository:{REGISTRY_REPOSITORY}:pull",
        }
    )
    with urllib.request.urlopen(
        f"https://auth.docker.io/token?{query}", timeout=60
    ) as response:
        token = json.load(response)["token"]
    headers = {"Authorization": f"Bearer {token}"}
    if accept is not None:
        headers["Accept"] = accept
    request = urllib.request.Request(f"{REGISTRY}{path}", headers=headers)
    with urllib.request.urlopen(request, timeout=120) as response:
        return response.read()


def _fetch_base_status(run_directory: Path, architecture: str) -> tuple[Path, dict[str, object]]:
    index_bytes = _registry_request(
        f"/v2/{REGISTRY_REPOSITORY}/manifests/{ROS_INDEX_DIGEST}",
        accept="application/vnd.oci.image.index.v1+json",
    )
    actual_index_digest = f"sha256:{_sha256_bytes(index_bytes)}"
    if actual_index_digest != ROS_INDEX_DIGEST:
        raise RuntimeError(
            f"ROS index digest mismatch: {actual_index_digest} != {ROS_INDEX_DIGEST}"
        )
    index = json.loads(index_bytes)
    children = [
        item
        for item in index["manifests"]
        if item.get("platform", {}).get("os") == "linux"
        and item.get("platform", {}).get("architecture") == architecture
    ]
    if len(children) != 1:
        raise RuntimeError(f"expected one {architecture} ROS child manifest, found {len(children)}")
    child_digest = children[0]["digest"]
    manifest_bytes = _registry_request(
        f"/v2/{REGISTRY_REPOSITORY}/manifests/{child_digest}",
        accept="application/vnd.oci.image.manifest.v1+json",
    )
    actual_child_digest = f"sha256:{_sha256_bytes(manifest_bytes)}"
    if actual_child_digest != child_digest:
        raise RuntimeError(
            f"ROS {architecture} child digest mismatch: {actual_child_digest} != {child_digest}"
        )
    manifest = json.loads(manifest_bytes)

    status_data: bytes | None = None
    status_layer: str | None = None
    layer_path = run_directory / f"{architecture}-base-layer.tar.gz"
    for layer in reversed(manifest["layers"]):
        blob = _registry_request(
            f"/v2/{REGISTRY_REPOSITORY}/blobs/{layer['digest']}"
        )
        actual_layer_digest = f"sha256:{_sha256_bytes(blob)}"
        if actual_layer_digest != layer["digest"]:
            raise RuntimeError(
                f"base layer digest mismatch: {actual_layer_digest} != {layer['digest']}"
            )
        layer_path.write_bytes(blob)
        with tarfile.open(layer_path, mode="r:gz") as archive:
            try:
                member = archive.getmember("var/lib/dpkg/status")
            except KeyError:
                continue
            extracted = archive.extractfile(member)
            if extracted is None:
                raise RuntimeError("base status member is not a regular file")
            status_data = extracted.read()
            status_layer = layer["digest"]
            break
    layer_path.unlink(missing_ok=True)
    if status_data is None or status_layer is None:
        raise RuntimeError(f"no /var/lib/dpkg/status found in the {architecture} ROS image")

    status_path = run_directory / "states" / "base.status"
    status_path.parent.mkdir(parents=True, exist_ok=True)
    status_path.write_bytes(status_data)
    evidence = {
        "index_digest": ROS_INDEX_DIGEST,
        "architecture": architecture,
        "child_manifest_digest": child_digest,
        "config_digest": manifest["config"]["digest"],
        "status_layer_digest": status_layer,
        "status_sha256": _sha256_bytes(status_data),
    }
    (run_directory / "base-manifest.json").write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return status_path, evidence


def _docker_snapshot() -> dict[str, list[str]]:
    tags = _run(
        [
            "docker",
            "image",
            "ls",
            "--no-trunc",
            "--format",
            "{{.Repository}}:{{.Tag}} {{.ID}}",
        ]
    ).stdout.splitlines()
    containers = _run(
        ["docker", "ps", "--format", "{{.Names}} {{.Image}} {{.ID}}"]
    ).stdout.splitlines()
    return {"tags": sorted(tags), "running_containers": sorted(containers)}


class AptRunner:
    def __init__(self, run_directory: Path, architecture: str, utility_architecture: str) -> None:
        self.run_directory = run_directory
        self.architecture = architecture
        self.utility_architecture = utility_architecture
        self.counter = 0

    def run(
        self,
        action: str,
        *,
        status: Path,
        roots: Path | None = None,
    ) -> str:
        self.counter += 1
        name = f"drone-sim-{self.architecture}-lock-{self.counter}-{os.getpid()}"
        command = [
            "docker",
            "run",
            "--rm",
            "--platform",
            f"linux/{self.utility_architecture}",
            "--name",
            name,
            "--mount",
            f"type=bind,src={self.run_directory},dst=/audit",
            "-e",
            f"AUDIT_ARCHITECTURE={self.architecture}",
            "-e",
            f"AUDIT_UTILITY_ARCHITECTURE={self.utility_architecture}",
            "-e",
            "LC_ALL=C",
            "-e",
            f"AUDIT_ACTION={action}",
            "-e",
            f"AUDIT_STATUS={status.relative_to(self.run_directory)}",
            "-e",
            f"AUDIT_ROOTS={roots.relative_to(self.run_directory) if roots else ''}",
            ROS_IMAGE,
            "bash",
            "-lc",
            APT_SHELL,
        ]
        try:
            completed = _run(command, timeout=1800 if action == "download" else 600)
        except subprocess.TimeoutExpired as error:
            # docker run's client can time out while its disposable helper lives.
            # The generated name belongs only to this resolver invocation.
            _run(["docker", "rm", "--force", name], check=False)
            raise RuntimeError(f"apt helper timed out during {action}: {name}") from error
        return completed.stdout


def _deb822_stanzas(text: str) -> list[str]:
    return [stanza.strip() for stanza in re.split(r"\n\s*\n", text) if stanza.strip()]


def _fields(stanza: str) -> dict[str, str]:
    result: dict[str, str] = {}
    current: str | None = None
    for line in stanza.splitlines():
        if line.startswith((" ", "\t")) and current is not None:
            result[current] += "\n" + line
            continue
        if ": " not in line:
            current = None
            continue
        current, value = line.split(": ", 1)
        result[current] = value
    return result


def _installed(status: Path) -> dict[tuple[str, str], str]:
    result = {}
    for stanza in _deb822_stanzas(status.read_text(encoding="utf-8")):
        fields = _fields(stanza)
        if fields.get("Status") != "install ok installed":
            continue
        result[(fields["Package"], fields["Architecture"])] = fields["Version"]
    return result


def _parse_plan(simulation: str) -> list[tuple[str, str]]:
    if any(line.startswith("Remv ") for line in simulation.splitlines()):
        raise RuntimeError("apt proposed a package removal")
    selected: dict[str, str] = {}
    pattern = re.compile(r"^Inst\s+(\S+)(?:\s+\[[^]]+\])?\s+\((\S+)")
    for line in simulation.splitlines():
        match = pattern.match(line)
        if match is None:
            continue
        package = re.sub(r":(?:amd64|arm64)$", "", match.group(1))
        version = match.group(2)
        previous = selected.setdefault(package, version)
        if previous != version:
            raise RuntimeError(f"apt selected two versions of {package}")
    return sorted(selected.items())


@dataclass
class PackageAvailability:
    candidate: str | None
    versions: dict[str, set[str]]


def _repository_origin(url: str) -> str:
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme not in {"http", "https"} or parsed.username or parsed.password:
        return "unknown"
    location = (parsed.hostname, parsed.path.rstrip("/"))
    if location in {
        ("ports.ubuntu.com", "/ubuntu-ports"),
        ("archive.ubuntu.com", "/ubuntu"),
        ("security.ubuntu.com", "/ubuntu"),
    }:
        return "Ubuntu"
    if location == ("packages.ros.org", "/ros2/ubuntu"):
        return "ROS"
    return "unknown"


def _parse_policy(text: str) -> dict[str, PackageAvailability]:
    """Read apt's C-locale policy output, keeping downloadable origins separate."""
    result: dict[str, PackageAvailability] = {}
    current: PackageAvailability | None = None
    version: str | None = None
    for line in text.splitlines():
        if line and not line[0].isspace() and line.endswith(":"):
            package = re.sub(r":(?:amd64|arm64)$", "", line[:-1])
            current = PackageAvailability(None, {})
            result[package] = current
            version = None
        elif current is not None:
            candidate = re.fullmatch(r"\s+Candidate: (\S+)", line)
            selected = re.fullmatch(r"\s+(?:\*\*\*\s+)?(\S+)\s+-?\d+", line)
            source = re.fullmatch(r"\s+-?\d+\s+(\S+)\s+.+\s+Packages", line)
            if candidate:
                current.candidate = None if candidate[1] == "(none)" else candidate[1]
            elif selected:
                version = selected[1]
            elif source and version is not None:
                current.versions.setdefault(version, set()).add(_repository_origin(source[1]))
    return result


def _refresh_roots(
    roots: tuple[str, ...],
    inventory: dict[str, PackageAvailability],
    installed: dict[tuple[str, str], str],
) -> tuple[tuple[str, ...], list[dict[str, str]]]:
    refreshed = []
    changes = []
    for row in roots:
        package, version = row.split("=", 1)
        if any(name == package and value == version for (name, _arch), value in installed.items()):
            refreshed.append(row)
            continue
        available = inventory.get(package)
        origins = available.versions.get(version, set()) if available else set()
        if origins:
            if "unknown" in origins:
                raise RuntimeError(f"unknown repository origin for {row}")
            refreshed.append(row)
            continue
        if package.startswith("ros-"):
            raise RuntimeError(f"immutable ROS package is unavailable: {row}")
        candidate = available.candidate if available else None
        candidate_origins = available.versions.get(candidate, set()) if available and candidate else set()
        if candidate is None or candidate_origins != {"Ubuntu"}:
            raise RuntimeError(f"no signed Ubuntu replacement for {row}: {candidate_origins}")
        refreshed.append(f"{package}={candidate}")
        changes.append({"package": package, "before": version, "after": candidate, "origin": "Ubuntu"})
    return tuple(refreshed), changes


def _read_lock(path: Path) -> tuple[str, ...]:
    rows = tuple(path.read_text(encoding="utf-8").splitlines())
    if not rows or any(not re.fullmatch(r"[a-z0-9][a-z0-9+.-]*=[^\s=]+", row) for row in rows):
        raise RuntimeError(f"invalid exact package lock: {path}")
    if len({row.split("=", 1)[0] for row in rows}) != len(rows):
        raise RuntimeError(f"duplicate package in {path}")
    return rows


def _artifacts_direct(project: Path, architecture: str) -> tuple[str, ...]:
    dockerfile = (project / "artifacts/Dockerfile").read_text(encoding="utf-8")
    common = tuple(dict.fromkeys(re.findall(r"^\s+([a-z0-9][a-z0-9+.-]*=[^\s=]+)\s+\\$", dockerfile, re.M)))
    suffix = "_ARM64" if architecture == "arm64" else ""
    packages = []
    for argument in ("ROSBAG2_PACKAGE", "ROSBAG2_STORAGE_MCAP_PACKAGE"):
        match = re.search(rf"^ARG {argument}{suffix}=(\S+)$", dockerfile, re.M)
        if match is None:
            raise RuntimeError(f"missing {architecture} artifact direct pin: {argument}")
        packages.append(match[1])
    if not common:
        raise RuntimeError("no artifacts direct Ubuntu pins found")
    return common + tuple(packages)


def _parse_records(text: str, expected: list[tuple[str, str]]) -> list[PackageRecord]:
    wanted = set(expected)
    records: dict[tuple[str, str], PackageRecord] = {}
    for stanza in _deb822_stanzas(text):
        fields = _fields(stanza)
        key = (fields.get("Package", ""), fields.get("Version", ""))
        if key not in wanted:
            continue
        for required in ("Architecture", "Filename", "Size", "SHA256"):
            if not fields.get(required):
                raise RuntimeError(f"{key[0]}={key[1]} has no {required} field")
        records[key] = PackageRecord(
            package=key[0],
            version=key[1],
            architecture=fields["Architecture"],
            stanza=stanza,
            filename=fields["Filename"],
            size=int(fields["Size"]),
            sha256=fields["SHA256"],
        )
    missing = wanted - set(records)
    if missing:
        raise RuntimeError(f"apt-cache did not return records for {sorted(missing)}")
    return [records[key] for key in expected]


def _installed_stanza(record: PackageRecord) -> str:
    lines = [line for line in record.stanza.splitlines() if not line.startswith("Status: ")]
    lines.insert(1, "Status: install ok installed")
    return "\n".join(lines)


def _materialize_status(
    before: Path,
    after: Path,
    records: list[PackageRecord],
) -> None:
    replacements = {(record.package, record.architecture): record for record in records}
    output: list[str] = []
    replaced: set[tuple[str, str]] = set()
    for stanza in _deb822_stanzas(before.read_text(encoding="utf-8")):
        fields = _fields(stanza)
        key = (fields.get("Package", ""), fields.get("Architecture", ""))
        record = replacements.get(key)
        if record is None:
            output.append(stanza)
            continue
        output.append(_installed_stanza(record))
        replaced.add(key)
    for key, record in replacements.items():
        if key not in replaced:
            output.append(_installed_stanza(record))
    after.write_text("\n\n".join(output) + "\n", encoding="utf-8")


def _write_roots(run_directory: Path, phase: str, roots: tuple[str, ...]) -> Path:
    path = run_directory / f"{phase}.roots"
    path.write_text("\n".join(roots) + "\n", encoding="utf-8")
    return path


def _write_package_manifest(
    path: Path,
    records: list[PackageRecord],
    verified_downloads: dict[str, str] | None,
) -> None:
    document = []
    for record in records:
        item = {
            "package": record.package,
            "version": record.version,
            "architecture": record.architecture,
            "filename": record.filename,
            "size": record.size,
            "sha256": record.sha256,
        }
        if verified_downloads is not None:
            item["verified_deb"] = verified_downloads[record.sha256]
        document.append(item)
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _verify_downloads(
    runner: AptRunner,
    status: Path,
    roots_path: Path,
    records: list[PackageRecord],
) -> dict[str, str]:
    runner.run("download", status=status, roots=roots_path)
    by_digest = {}
    for package_path in sorted((runner.run_directory / "cache" / "archives").glob("*.deb")):
        by_digest[_sha256_file(package_path)] = package_path.name
    missing = sorted(record.sha256 for record in records if record.sha256 not in by_digest)
    if missing:
        raise RuntimeError(f"downloaded package checksums missing: {missing}")
    return {record.sha256: by_digest[record.sha256] for record in records}


def _resolve_phase(
    runner: AptRunner,
    *,
    phase: str,
    before: Path,
    after: Path,
    roots: tuple[str, ...],
    candidate_path: Path | None,
    verify_downloads: bool,
    refresh_unavailable_ubuntu: bool = False,
) -> dict[str, object]:
    roots_path = _write_roots(runner.run_directory, phase, roots)
    original_roots = roots
    changes: list[dict[str, str]] = []
    if refresh_unavailable_ubuntu:
        policy = runner.run("policy", status=before, roots=roots_path)
        (runner.run_directory / f"{phase}.policy.txt").write_text(policy, encoding="utf-8")
        roots, changes = _refresh_roots(roots, _parse_policy(policy), _installed(before))
        if changes:
            upgrade_path = runner.run_directory / f"{phase}.upgrades"
            upgrade_path.write_text(
                "".join(f"{item['package']} {item['before']} {item['after']}\n" for item in changes),
                encoding="utf-8",
            )
            runner.run("check-upgrades", status=before, roots=upgrade_path)
        roots_path = _write_roots(runner.run_directory, f"{phase}.refreshed", roots)
    simulation = runner.run("simulate", status=before, roots=roots_path)
    (runner.run_directory / f"{phase}.simulation.txt").write_text(
        simulation, encoding="utf-8"
    )
    selected = _parse_plan(simulation)
    installed_before = _installed(before)
    version_changes = [
        f"{package} {previous} {version}\n"
        for package, version in selected
        for (installed_package, installed_architecture), previous in installed_before.items()
        if installed_package == package
        and installed_architecture in {runner.architecture, "all"}
        and previous != version
    ]
    if version_changes:
        plan_upgrades = runner.run_directory / f"{phase}.plan-upgrades"
        plan_upgrades.write_text("".join(version_changes), encoding="utf-8")
        runner.run("check-upgrades", status=before, roots=plan_upgrades)
    selected_roots = tuple(f"{package}={version}" for package, version in selected)
    selected_path = _write_roots(runner.run_directory, f"{phase}.selected", selected_roots)
    if selected:
        records_text = runner.run("show", status=before, roots=selected_path)
        records = sorted(
            _parse_records(records_text, selected), key=lambda record: record.lock_row
        )
        if any(record.architecture not in {runner.architecture, "all"} for record in records):
            raise RuntimeError(f"{phase}: apt selected a foreign architecture")
    else:
        records = []
    _materialize_status(before, after, records)

    installed_after = _installed(after)
    for root in roots:
        package, version = root.split("=", 1)
        actual = {
            installed_version
            for (installed_package, _architecture), installed_version in installed_after.items()
            if installed_package == package
        }
        if version not in actual:
            raise RuntimeError(f"{phase}: root {root} is not installed in synthetic state")

    convergence = runner.run("simulate", status=after, roots=roots_path)
    if _parse_plan(convergence):
        raise RuntimeError(f"{phase}: synthetic state does not converge")
    if any(line.startswith("Remv ") for line in convergence.splitlines()):
        raise RuntimeError(f"{phase}: converged state proposes a removal")

    downloaded = None
    if verify_downloads and records:
        downloaded = _verify_downloads(runner, before, roots_path, records)
    _write_package_manifest(
        runner.run_directory / f"{phase}.packages.json", records, downloaded
    )

    rows = [record.lock_row for record in records]
    if rows != sorted(rows):
        raise RuntimeError(f"{phase}: apt selection is not sorted")
    if candidate_path is not None:
        candidate_path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    return {
        "phase": phase,
        "roots": list(roots),
        "original_roots": list(original_roots),
        "ubuntu_refreshes": changes,
        "selected_count": len(records),
        "selected_download_bytes": sum(record.size for record in records),
        "candidate": str(candidate_path) if candidate_path is not None else None,
        "candidate_sha256": _sha256_file(candidate_path) if candidate_path else None,
        "downloads_verified": verify_downloads,
    }


def _write_sources(run_directory: Path, architecture: str) -> None:
    if architecture not in {"amd64", "arm64"}:
        raise RuntimeError(f"unsupported package architecture: {architecture}")
    archive = "http://ports.ubuntu.com/ubuntu-ports" if architecture == "arm64" else "http://archive.ubuntu.com/ubuntu"
    security = archive if architecture == "arm64" else "http://security.ubuntu.com/ubuntu"
    sources = "\n".join(
        (
            f"deb [arch={architecture}] {archive} noble main universe restricted multiverse",
            f"deb [arch={architecture}] {archive} noble-updates main universe restricted multiverse",
            f"deb [arch={architecture}] {archive} noble-backports main universe restricted multiverse",
            f"deb [arch={architecture}] {security} noble-security main universe restricted multiverse",
        )
    )
    (run_directory / "sources.list").write_text(sources + "\n", encoding="utf-8")
    for directory in (
        run_directory / "sourceparts",
        run_directory / "state" / "lists" / "partial",
        run_directory / "cache" / "archives" / "partial",
        run_directory / "states",
    ):
        directory.mkdir(parents=True, exist_ok=True)


def _index_evidence(run_directory: Path) -> list[dict[str, object]]:
    result = []
    for path in sorted((run_directory / "state" / "lists").iterdir()):
        if not path.is_file():
            continue
        result.append(
            {"file": path.name, "size": path.stat().st_size, "sha256": _sha256_file(path)}
        )
    return result


def _resume_base(
    requested: Path, runs_directory: Path, architecture: str,
) -> tuple[Path, Path, dict[str, object]]:
    run_directory = requested.resolve()
    if run_directory.parent != runs_directory.resolve():
        raise RuntimeError(f"resume path must be an immediate child of {runs_directory}")
    if (run_directory / "resolution-summary.json").exists():
        raise RuntimeError("cannot resume a completed resolution; start a new run")
    base_status = run_directory / "states/base.status"
    base_evidence = json.loads((run_directory / "base-manifest.json").read_text())
    if base_evidence["architecture"] != architecture or base_evidence["index_digest"] != ROS_INDEX_DIGEST:
        raise RuntimeError("resume base architecture or pinned image differs")
    if _sha256_file(base_status) != base_evidence["status_sha256"]:
        raise RuntimeError("resume base status checksum differs")
    return run_directory, base_status, base_evidence


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--architecture", choices=("arm64", "amd64"), default="amd64")
    parser.add_argument(
        "--refresh-unavailable-ubuntu", action="store_true",
        help="replace unavailable Ubuntu pins with signed current candidates; preserve ROS pins",
    )
    parser.add_argument(
        "--skip-downloads", action="store_true",
        help="resolve metadata without downloading and hashing every selected deb",
    )
    parser.add_argument("--resume", type=Path, help="reuse an unfinished run and its package cache; completed runs are immutable")
    arguments = parser.parse_args()

    project_directory = Path(__file__).resolve().parent.parent
    architecture = arguments.architecture
    output_directory = project_directory / ".local-runtime/apt-lock-resolution" / architecture
    runs_directory = output_directory / "runs"
    resumed = (
        _resume_base(arguments.resume, runs_directory, architecture)
        if arguments.resume is not None else None
    )
    runs_directory.mkdir(parents=True, exist_ok=True)
    engine_architecture = _run(["docker", "info", "--format", "{{.Architecture}}"]).stdout.strip()
    aliases = {"aarch64": "arm64", "arm64": "arm64", "x86_64": "amd64", "amd64": "amd64"}
    if engine_architecture not in aliases:
        raise RuntimeError(f"unsupported native Docker architecture: {engine_architecture}")
    utility_architecture = aliases[engine_architecture]
    utility = _run(
        ["docker", "image", "inspect", "--format", "{{.Architecture}}", ROS_IMAGE],
        check=False,
    )
    if utility.returncode != 0 or utility.stdout.strip() != utility_architecture:
        _run(["docker", "pull", "--platform", f"linux/{utility_architecture}", ROS_IMAGE], timeout=1800)
    docker_before = _docker_snapshot()
    if resumed is None:
        timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
        run_directory = runs_directory / f"{timestamp}-{os.getpid()}"
        run_directory.mkdir()
        _write_sources(run_directory, architecture)
        base_status, base_evidence = _fetch_base_status(run_directory, architecture)
    else:
        run_directory, base_status, base_evidence = resumed
    runner = AptRunner(run_directory, architecture, utility_architecture)
    native_machine = runner.run("architecture", status=base_status).strip()
    runner.run("copy-ros-source", status=base_status)
    update_output = runner.run("update", status=base_status)
    (run_directory / "apt-update.txt").write_text(update_output, encoding="utf-8")

    suffix = ".arm64" if architecture == "arm64" else ""
    artifact_direct = _artifacts_direct(project_directory, architecture)
    direct_state = run_directory / "states/artifacts-direct.status"
    gazebo_state = run_directory / "states/gazebo-runtime.status"
    # Child image deltas must be resolved against their own refreshed parent.
    specifications = [
        ("artifacts-direct", base_status, direct_state, artifact_direct,
         f"artifacts-direct.{architecture}.candidate.delta"),
        ("artifacts-ffmpeg", direct_state, run_directory / "states/artifacts-runtime.status",
         _read_lock(project_directory / f"artifacts/ffmpeg-packages{suffix}.lock"),
         f"ffmpeg-packages.{architecture}.candidate.lock"),
        ("gazebo-harmonic", base_status, gazebo_state,
         _read_lock(project_directory / f"gazebo/harmonic-packages{suffix}.lock"),
         f"harmonic-packages.{architecture}.candidate.lock"),
        ("viewer", gazebo_state, run_directory / "states/viewer.status",
         _read_lock(project_directory / f"gazebo/local-viewer/viewer-packages.{architecture}.lock"),
         f"viewer-packages.{architecture}.candidate.lock"),
    ]
    if architecture == "arm64":
        specifications.append((
            "ardupilot-build", base_status, run_directory / "states/ardupilot-build.status",
            _read_lock(project_directory / "ardupilot_sitl/build-packages.arm64.lock"),
            "build-packages.arm64.candidate.lock",
        ))
    phases = []
    for phase, before, after, roots, filename in specifications:
        print(f"Resolving {architecture} {phase}", file=sys.stderr, flush=True)
        phases.append(_resolve_phase(
            runner, phase=phase, before=before, after=after, roots=roots,
            candidate_path=run_directory / filename,
            verify_downloads=not arguments.skip_downloads,
            refresh_unavailable_ubuntu=arguments.refresh_unavailable_ubuntu,
        ))
    (run_directory / f"artifacts-direct.{architecture}.candidate.pins").write_text(
        "\n".join(phases[0]["roots"]) + "\n", encoding="utf-8"
    )
    docker_after = _docker_snapshot()
    if docker_after != docker_before:
        raise RuntimeError("Docker tags or running-container inventory changed during resolution")
    summary = {
        "generated_at": datetime.now(UTC).isoformat(),
        "utility_architecture": native_machine,
        "apt_architecture": architecture,
        "refresh_unavailable_ubuntu": arguments.refresh_unavailable_ubuntu,
        "base": base_evidence,
        "indexes": _index_evidence(run_directory),
        "phases": phases,
        "docker_inventory_unchanged": True,
        "native_acceptance": False,
        "run_directory": str(run_directory),
    }
    # Publish candidates only when every layer and archive check has succeeded.
    (run_directory / "resolution-summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output_directory / "latest-run-path.txt").write_text(str(run_directory) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
