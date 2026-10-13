#!/usr/bin/env python3
"""Resolve AMD64 apt deltas with ARM64 apt and the pinned AMD64 base status.

The script never executes an AMD64 binary. It runs apt from the pinned ARM64
ROS image with APT::Architecture=amd64, an isolated package database, and the
official Ubuntu and ROS repositories. Each simulated install is materialized
as a synthetic dpkg status file before the next child-image layer is resolved.
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

ARTIFACTS_DIRECT = (
    "python3-venv=3.12.3-0ubuntu2.1",
    "python3.12-venv=3.12.3-1ubuntu0.17",
    "python3-pip-whl=24.0+dfsg-1ubuntu1.3",
    "python3-setuptools-whl=68.1.2-2ubuntu1.2",
    "python3-wheel=0.42.0-2",
    "zstd=1.5.5+dfsg2-2build1.1",
    "ros-jazzy-rosbag2=0.26.11-1noble.20261006.034645",
    "ros-jazzy-rosbag2-storage-mcap=0.26.11-1noble.20260911.143733",
)
ARTIFACTS_FFMPEG = ("ffmpeg=7:6.1.1-3ubuntu5",)
GAZEBO_DIRECT = (
    "ros-jazzy-ros-gz=1.0.24-1noble.20261006.041805",
    "ros-jazzy-ros-gz-bridge=1.0.24-1noble.20260920.234644",
    "ros-jazzy-ros-gz-image=1.0.24-1noble.20260921.023911",
    "ros-jazzy-ros-gz-interfaces=1.0.24-1noble.20260911.110630",
    "ros-jazzy-ros-gz-sim=1.0.24-1noble.20261006.035056",
    "ros-jazzy-gz-sim-vendor=0.0.13-1noble.20260905.073557",
    "ros-jazzy-sdformat-vendor=0.0.14-1noble.20260905.033543",
    "zstd=1.5.5+dfsg2-2build1.1",
)

APT_SHELL = r"""
set -euo pipefail
apt_options=(
  -o APT::Architecture=amd64
  -o APT::Architectures::=amd64
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
  -o APT::Sandbox::User=root
)

case "${AUDIT_ACTION}" in
  architecture)
    machine="$(uname -m)"
    test "${machine}" = "aarch64" -o "${machine}" = "arm64"
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
  download)
    mapfile -t packages < "/audit/${AUDIT_ROOTS}"
    apt-get "${apt_options[@]}" \
      --yes --download-only --no-install-recommends --no-remove \
      install "${packages[@]}"
    ;;
  cleanup-cache)
    find /audit/cache/archives -maxdepth 1 -type f -name '*.deb' -delete
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
) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            arguments,
            check=check,
            capture_output=capture_output,
            text=True,
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


def _fetch_base_status(run_directory: Path) -> tuple[Path, dict[str, object]]:
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
        and item.get("platform", {}).get("architecture") == "amd64"
    ]
    if len(children) != 1:
        raise RuntimeError(f"expected one AMD64 ROS child manifest, found {len(children)}")
    child_digest = children[0]["digest"]
    manifest_bytes = _registry_request(
        f"/v2/{REGISTRY_REPOSITORY}/manifests/{child_digest}",
        accept="application/vnd.oci.image.manifest.v1+json",
    )
    actual_child_digest = f"sha256:{_sha256_bytes(manifest_bytes)}"
    if actual_child_digest != child_digest:
        raise RuntimeError(
            f"ROS AMD64 child digest mismatch: {actual_child_digest} != {child_digest}"
        )
    manifest = json.loads(manifest_bytes)

    status_data: bytes | None = None
    status_layer: str | None = None
    layer_path = run_directory / "amd64-base-layer.tar.gz"
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
                raise RuntimeError("AMD64 status member is not a regular file")
            status_data = extracted.read()
            status_layer = layer["digest"]
            break
    layer_path.unlink(missing_ok=True)
    if status_data is None or status_layer is None:
        raise RuntimeError("no /var/lib/dpkg/status found in the AMD64 ROS image")

    status_path = run_directory / "states" / "base.status"
    status_path.parent.mkdir(parents=True, exist_ok=True)
    status_path.write_bytes(status_data)
    evidence = {
        "index_digest": ROS_INDEX_DIGEST,
        "amd64_child_manifest_digest": child_digest,
        "amd64_config_digest": manifest["config"]["digest"],
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
    def __init__(self, run_directory: Path) -> None:
        self.run_directory = run_directory
        self.counter = 0

    def run(
        self,
        action: str,
        *,
        status: Path,
        roots: Path | None = None,
    ) -> str:
        self.counter += 1
        name = f"drone-sim-amd64-lock-{self.counter}-{os.getpid()}"
        command = [
            "docker",
            "run",
            "--rm",
            "--platform",
            "linux/arm64",
            "--name",
            name,
            "--mount",
            f"type=bind,src={self.run_directory},dst=/audit",
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
        completed = _run(command)
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
        package = match.group(1).removesuffix(":amd64")
        version = match.group(2)
        previous = selected.setdefault(package, version)
        if previous != version:
            raise RuntimeError(f"apt selected two versions of {package}")
    return sorted(selected.items())


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
) -> dict[str, object]:
    roots_path = _write_roots(runner.run_directory, phase, roots)
    simulation = runner.run("simulate", status=before, roots=roots_path)
    (runner.run_directory / f"{phase}.simulation.txt").write_text(
        simulation, encoding="utf-8"
    )
    selected = _parse_plan(simulation)
    selected_roots = tuple(f"{package}={version}" for package, version in selected)
    selected_path = _write_roots(runner.run_directory, f"{phase}.selected", selected_roots)
    if selected:
        records_text = runner.run("show", status=before, roots=selected_path)
        records = sorted(
            _parse_records(records_text, selected), key=lambda record: record.lock_row
        )
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
        "selected_count": len(records),
        "selected_download_bytes": sum(record.size for record in records),
        "candidate": str(candidate_path) if candidate_path is not None else None,
        "candidate_sha256": _sha256_file(candidate_path) if candidate_path else None,
        "downloads_verified": verify_downloads,
    }


def _write_sources(run_directory: Path) -> None:
    sources = "\n".join(
        (
            "deb [arch=amd64] http://archive.ubuntu.com/ubuntu noble main universe restricted multiverse",
            "deb [arch=amd64] http://archive.ubuntu.com/ubuntu noble-updates main universe restricted multiverse",
            "deb [arch=amd64] http://archive.ubuntu.com/ubuntu noble-backports main universe restricted multiverse",
            "deb [arch=amd64] http://security.ubuntu.com/ubuntu noble-security main universe restricted multiverse",
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--skip-downloads",
        action="store_true",
        help="resolve metadata without downloading and hashing every selected deb",
    )
    parser.add_argument(
        "--resume",
        type=Path,
        help="reuse a prior run directory and its verified package cache",
    )
    arguments = parser.parse_args()

    project_directory = Path(__file__).resolve().parent.parent
    output_directory = project_directory / ".local-runtime/amd64-lock-resolution"
    runs_directory = output_directory / "runs"
    runs_directory.mkdir(parents=True, exist_ok=True)
    docker_before = _docker_snapshot()
    if arguments.resume is None:
        timestamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
        run_directory = runs_directory / f"{timestamp}-{os.getpid()}"
        run_directory.mkdir(parents=True)
        _write_sources(run_directory)
        base_status, base_evidence = _fetch_base_status(run_directory)
    else:
        run_directory = arguments.resume.resolve()
        if run_directory.parent != runs_directory.resolve():
            raise RuntimeError(f"resume path must be an immediate child of {runs_directory}")
        base_status = run_directory / "states/base.status"
        if not base_status.is_file():
            raise RuntimeError(f"resume path has no base status: {base_status}")
        base_evidence = json.loads(
            (run_directory / "base-manifest.json").read_text(encoding="utf-8")
        )
    runner = AptRunner(run_directory)
    architecture = runner.run("architecture", status=base_status).strip()
    runner.run("copy-ros-source", status=base_status)
    update_output = runner.run("update", status=base_status)
    (run_directory / "apt-update.txt").write_text(update_output, encoding="utf-8")

    viewer_roots = tuple(
        line
        for line in (
            project_directory / "gazebo/local-viewer/viewer-packages.arm64.lock"
        ).read_text(encoding="utf-8").splitlines()
        if line
    )
    verify_downloads = not arguments.skip_downloads
    phases = []
    phases.append(
        _resolve_phase(
            runner,
            phase="artifacts-direct",
            before=base_status,
            after=run_directory / "states/artifacts-direct.status",
            roots=ARTIFACTS_DIRECT,
            candidate_path=output_directory / "artifacts-direct.amd64.candidate.delta",
            verify_downloads=verify_downloads,
        )
    )
    phases.append(
        _resolve_phase(
            runner,
            phase="artifacts-ffmpeg",
            before=run_directory / "states/artifacts-direct.status",
            after=run_directory / "states/artifacts-runtime.status",
            roots=ARTIFACTS_FFMPEG,
            candidate_path=output_directory / "ffmpeg-packages.amd64.candidate.lock",
            verify_downloads=verify_downloads,
        )
    )
    phases.append(
        _resolve_phase(
            runner,
            phase="gazebo-harmonic",
            before=base_status,
            after=run_directory / "states/gazebo-runtime.status",
            roots=GAZEBO_DIRECT,
            candidate_path=output_directory / "harmonic-packages.amd64.candidate.lock",
            verify_downloads=verify_downloads,
        )
    )
    phases.append(
        _resolve_phase(
            runner,
            phase="viewer",
            before=run_directory / "states/gazebo-runtime.status",
            after=run_directory / "states/viewer.status",
            roots=viewer_roots,
            candidate_path=output_directory / "viewer-packages.amd64.candidate.lock",
            verify_downloads=verify_downloads,
        )
    )

    (output_directory / "gazebo-direct.amd64.candidate.pins").write_text(
        "\n".join(GAZEBO_DIRECT) + "\n", encoding="utf-8"
    )
    (output_directory / "artifacts-direct.amd64.candidate.pins").write_text(
        "\n".join(ARTIFACTS_DIRECT) + "\n", encoding="utf-8"
    )
    runner.run("cleanup-cache", status=base_status)
    docker_after = _docker_snapshot()
    if docker_after != docker_before:
        raise RuntimeError("Docker tags or running-container inventory changed during resolution")

    summary = {
        "generated_at": datetime.now(UTC).isoformat(),
        "utility_architecture": architecture,
        "apt_architecture": "amd64",
        "base": base_evidence,
        "indexes": _index_evidence(run_directory),
        "phases": phases,
        "docker_inventory_unchanged": True,
        "native_amd64_acceptance": False,
        "run_directory": str(run_directory),
    }
    (output_directory / "resolution-summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output_directory / "latest-run-path.txt").write_text(
        str(run_directory) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
