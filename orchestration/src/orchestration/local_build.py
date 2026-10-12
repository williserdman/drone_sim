"""Shared local image build and provenance checks."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any, Mapping, Sequence

from .host_integration import HostIntegration, resolve_host
from .workstation import Workstation


EXPECTED_PLATFORM = "linux/arm64"
RUNTIME_SERVICES = (
    "orchestration-runtime",
    "artifacts-runtime",
    "companion-runtime",
    "ardupilot-sitl",
    "gazebo-runtime",
    "electromagnet-runtime",
    "scorekeeper-runtime",
)
IMAGE_REFS = (
    "drone-sim-orchestration-runtime:phase2",
    "drone-sim-artifacts-runtime:phase2",
    "drone-sim-companion-runtime:phase3",
    "drone-sim-ardupilot-runtime:phase3",
    "drone-sim-gazebo-runtime:phase3",
    "drone-sim-electromagnet-runtime:phase3",
    "drone-sim-scorekeeper-runtime:phase3",
    "drone-sim-gazebo-viewer:local",
)
_INPUT_PATHS = (
    ".dockerignore",
    "compose.yaml",
    "compose.local.yaml",
    "config",
    "pyproject.toml",
    "uv.lock",
    "scripts/local-build.sh",
    "ardupilot_sitl",
    "artifacts",
    "companion",
    "electromagnet",
    "gazebo",
    "orchestration",
    "ros_ws",
    "scorekeeper",
)


def _check_output(command: Sequence[str], **kwargs: Any) -> str:
    return subprocess.check_output(command, **kwargs)


def _revision(project_directory: Path) -> str:
    return _check_output(
        ["git", "-C", str(project_directory), "rev-parse", "HEAD"],
        stderr=subprocess.DEVNULL,
        text=True,
    ).strip()


def runtime_input_digest(project_directory: Path) -> str:
    """Return the exact legacy local-build input fingerprint."""
    root = project_directory.resolve()
    selected = subprocess.check_output(
        [
            "git",
            "-C",
            str(root),
            "ls-files",
            "--cached",
            "--others",
            "--exclude-standard",
            "-z",
            "--",
            *_INPUT_PATHS,
        ],
        stderr=subprocess.DEVNULL,
    )
    paths = sorted(path for path in selected.split(b"\0") if path)
    digest = hashlib.sha256()
    root_bytes = os.fsencode(root)
    for relative_bytes in paths:
        digest.update(relative_bytes)
        digest.update(b"\0")
        absolute = os.path.join(root_bytes, relative_bytes)
        if os.path.isfile(absolute):
            inner = hashlib.sha256()
            with open(absolute, "rb") as source:
                for chunk in iter(lambda: source.read(1024 * 1024), b""):
                    inner.update(chunk)
            if b"\\" in relative_bytes or b"\n" in relative_bytes:
                digest.update(b"\\")
            digest.update(inner.hexdigest().encode("ascii"))
            digest.update(b"\n")
        else:
            digest.update(b"deleted\n")
    return digest.hexdigest()


def _inspect_image(image_ref: str, revision: str) -> dict[str, object]:
    document = json.loads(
        _check_output(
            ["docker", "image", "inspect", image_ref],
            stderr=subprocess.DEVNULL,
            text=True,
        )
    )[0]
    platform = f'{document["Os"]}/{document["Architecture"]}'
    if image_ref == "drone-sim-companion-runtime:phase3":
        labels = document.get("Config", {}).get("Labels") or {}
        if labels.get("org.opencontainers.image.comp2026.revision") != revision:
            raise RuntimeError(
                "companion image source revision label does not match HEAD"
            )
    return {
        "image": image_ref,
        "image_id": document["Id"],
        "repo_digests": sorted(document.get("RepoDigests") or []),
        "os_architecture": platform,
    }


def check_build(
    project_directory: Path, host: HostIntegration
) -> dict[str, object]:
    """Inspect the existing ARM64 image set without changing it."""
    del host  # Task 7 uses host facts when extending the platform gate.
    root = project_directory.resolve()
    revision = _revision(root)
    tree_digest = runtime_input_digest(root)
    record_path = root / ".local-runtime/build.json"
    reasons: list[str] = []
    try:
        docker_context: str | None = _check_output(
            ["docker", "context", "show"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        docker_context = None
        reasons.append("Docker context is unavailable")
    try:
        server_platform: str | None = _check_output(
            ["docker", "info", "--format", "{{.OSType}}/{{.Architecture}}"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        if server_platform == "linux/aarch64":
            server_platform = EXPECTED_PLATFORM
        if server_platform != EXPECTED_PLATFORM:
            reasons.append(f"Docker engine has platform {server_platform}")
    except (OSError, subprocess.CalledProcessError):
        server_platform = None
        reasons.append("Docker engine is unavailable")
    try:
        record: Mapping[str, object] | None = json.loads(
            record_path.read_text(encoding="utf-8")
        )
    except FileNotFoundError:
        record = None
        reasons.append("local image build record is missing")
    except (OSError, json.JSONDecodeError) as error:
        record = None
        reasons.append(f"local image build record is unreadable: {error}")

    current_images: list[dict[str, object]] = []
    for image_ref in IMAGE_REFS:
        try:
            inspection = _inspect_image(image_ref, revision)
        except (
            OSError,
            RuntimeError,
            subprocess.CalledProcessError,
            json.JSONDecodeError,
            IndexError,
            KeyError,
            TypeError,
        ) as error:
            if str(error) == "companion image source revision label does not match HEAD":
                reasons.append(str(error))
            else:
                reasons.append(f"local image is missing or unreadable: {image_ref}")
            continue
        current_images.append(inspection)
        if inspection["os_architecture"] != EXPECTED_PLATFORM:
            reasons.append(
                f'local image has platform {inspection["os_architecture"]}: {image_ref}'
            )
    if record is not None:
        if record.get("schema_version") != 1:
            reasons.append("local image build record schema is unsupported")
        if record.get("source_revision") != revision:
            reasons.append("source revision changed since the local image build")
        if record.get("source_tree_sha256") != tree_digest:
            reasons.append("runtime build inputs changed since the local image build")
        if record.get("docker_context") != docker_context:
            reasons.append("Docker context changed since the local image build")
        if record.get("target_platform") != EXPECTED_PLATFORM:
            reasons.append("local image build record is not for linux/arm64")
        if record.get("images") != current_images:
            reasons.append("local image identities changed since the build record")
    return {
        "ok": not reasons,
        "record_path": str(record_path),
        "reasons": reasons,
        "current_source": {"revision": revision, "tree_sha256": tree_digest},
        "current_platform": server_platform,
        "current_context": docker_context,
        "images": current_images,
    }


def _run(command: Sequence[str], *, root: Path, env: Mapping[str, str]) -> None:
    subprocess.run(
        command,
        cwd=root,
        env=env,
        stdout=sys.stderr,
        stderr=subprocess.STDOUT,
        check=True,
    )


def build_images(
    project_directory: Path,
    host: HostIntegration,
    workstation: Workstation,
) -> None:
    """Build and record the eight legacy ARM64 runtime images under one lease."""
    root = project_directory.resolve()
    with workstation.hold(allow_nested=True):
        revision = _revision(root)
        tree_digest = runtime_input_digest(root)
        try:
            architecture = _check_output(
                ["docker", "version", "--format", "{{.Server.Arch}}"],
                text=True,
            ).strip()
        except (OSError, subprocess.CalledProcessError) as error:
            raise RuntimeError(
                "local build requires a running arm64 Docker engine"
            ) from error
        if architecture not in {"arm64", "aarch64"}:
            raise RuntimeError(
                f"local build requires an arm64 Docker engine; found {architecture}"
            )
        environment = {
            **os.environ,
            "DOCKER_DEFAULT_PLATFORM": EXPECTED_PLATFORM,
            "SIM_COMP2026_REVISION": revision,
        }
        _run(
            [
                "docker", "compose", "--file", "compose.yaml", "--file",
                "compose.local.yaml", "--profile", "phase3", "build",
                *RUNTIME_SERVICES,
            ],
            root=root,
            env=environment,
        )
        _run(
            [
                "docker", "compose", "--file", "compose.yaml", "--file",
                "compose.local.yaml", "--profile", "phase3", "--profile",
                "local", "build", "gazebo-viewer",
            ],
            root=root,
            env=environment,
        )
        if _revision(root) != revision or runtime_input_digest(root) != tree_digest:
            raise RuntimeError(
                "runtime build inputs changed during the build; refusing to publish a mixed-source build record"
            )
        images = [_inspect_image(image_ref, revision) for image_ref in IMAGE_REFS]
        for image in images:
            if image["os_architecture"] != EXPECTED_PLATFORM:
                raise RuntimeError(
                    f'image {image["image"]} has platform {image["os_architecture"]}, expected {EXPECTED_PLATFORM}'
                )
        docker_context = _check_output(
            ["docker", "context", "show"], text=True
        ).strip()
        record = {
            "schema_version": 1,
            "source_revision": revision,
            "source_tree_sha256": tree_digest,
            "docker_context": docker_context,
            "target_platform": EXPECTED_PLATFORM,
            "images": images,
        }
        record_directory = root / ".local-runtime"
        record_directory.mkdir(exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix="build.json.tmp.", dir=record_directory
        )
        temporary = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as output:
                json.dump(record, output, indent=2, sort_keys=True)
                output.write("\n")
                output.flush()
                os.fsync(output.fileno())
            os.chmod(temporary, 0o600)
            if _revision(root) != revision or runtime_input_digest(root) != tree_digest:
                raise RuntimeError(
                    "runtime build inputs changed before record publication; refusing to publish a mixed-source build record"
                )
            os.replace(temporary, record_directory / "build.json")
        finally:
            temporary.unlink(missing_ok=True)
        diagnostic = check_build(root, host)
        if diagnostic["ok"] is not True:
            reasons = diagnostic["reasons"]
            if not isinstance(reasons, list) or not all(
                isinstance(reason, str) for reason in reasons
            ):
                raise RuntimeError("local build check returned invalid diagnostics")
            raise RuntimeError("; ".join(reasons))


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="local-build.sh")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--build", action="store_true")
    mode.add_argument("--check", action="store_true")
    arguments = parser.parse_args(list(argv) if argv is not None else None)
    root = Path(__file__).resolve().parents[3]
    try:
        host = resolve_host(root)
        if arguments.build:
            build_images(root, host, Workstation(host))
        diagnostic = check_build(root, host)
        print(json.dumps(diagnostic, sort_keys=True, separators=(",", ":")))
        return 0 if diagnostic["ok"] is True else 1
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "IMAGE_REFS",
    "build_images",
    "check_build",
    "main",
    "runtime_input_digest",
]
