"""Shared local image build and provenance checks."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
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
_ARCHITECTURE_ALIASES = {
    "aarch64": "arm64",
    "arm64": "arm64",
    "amd64": "amd64",
    "x86_64": "amd64",
}


@dataclass(frozen=True, slots=True)
class _DockerState:
    context: str
    endpoint: str
    platform: str


def _check_output(command: Sequence[str], **kwargs: Any) -> str:
    return subprocess.check_output(command, **kwargs)


def _revision(project_directory: Path) -> str:
    return _check_output(
        ["git", "-C", str(project_directory), "rev-parse", "HEAD"],
        stderr=subprocess.DEVNULL,
        text=True,
    ).strip()


def _target_platform(host: HostIntegration) -> str:
    platform = host.facts.image_platform
    if platform not in {"linux/arm64", "linux/amd64"}:
        raise RuntimeError(f"host selected unsupported image platform {platform}")
    return platform


def _platform(os_type: object, architecture: object) -> str:
    if not isinstance(os_type, str) or not isinstance(architecture, str):
        raise RuntimeError("Docker engine returned invalid platform metadata")
    normalized = _ARCHITECTURE_ALIASES.get(architecture.lower())
    if os_type.lower() != "linux" or normalized is None:
        raise RuntimeError(f"Docker engine has platform {os_type}/{architecture}")
    return f"linux/{normalized}"


def _context_document(context: str) -> Mapping[str, object]:
    raw = json.loads(
        _check_output(
            ["docker", "context", "inspect", context],
            stderr=subprocess.DEVNULL,
            text=True,
        )
    )
    if not isinstance(raw, list) or len(raw) != 1 or not isinstance(raw[0], dict):
        raise RuntimeError("Docker context inspection returned invalid metadata")
    return raw[0]


def _docker_state(host: HostIntegration) -> _DockerState:
    target_platform = _target_platform(host)
    shown_context = _check_output(
        ["docker", "context", "show"],
        stderr=subprocess.DEVNULL,
        text=True,
    ).strip()
    configured_context = os.environ.get("DOCKER_CONTEXT", "").strip()
    context = configured_context or shown_context
    if not context:
        raise RuntimeError("Docker context is unavailable")
    if configured_context and shown_context != configured_context:
        raise RuntimeError("Docker context selection does not match DOCKER_CONTEXT")

    document = _context_document(context)
    configured_host = os.environ.get("DOCKER_HOST", "").strip()
    context_supplies_endpoint = bool(configured_context) or not configured_host
    context_tls = False
    if context_supplies_endpoint:
        endpoints = document.get("Endpoints")
        if not isinstance(endpoints, dict):
            raise RuntimeError("Docker context has no endpoint metadata")
        docker_endpoint = endpoints.get("docker")
        if not isinstance(docker_endpoint, dict):
            raise RuntimeError("Docker context has no Docker endpoint")
        endpoint = docker_endpoint.get("Host")
        if not isinstance(endpoint, str) or not endpoint:
            raise RuntimeError("Docker context has an invalid Docker endpoint")
        context_tls = (
            docker_endpoint.get("SkipTLSVerify") is True
            or bool(document.get("TLSMaterial"))
        )
    else:
        endpoint = configured_host
    if not endpoint.startswith("unix://"):
        raise RuntimeError(
            "local Docker endpoint is required; effective endpoint is remote"
        )

    tls_environment = tuple(
        name
        for name in ("DOCKER_TLS", "DOCKER_TLS_VERIFY", "DOCKER_CERT_PATH")
        if os.environ.get(name, "").strip()
    )
    if tls_environment or context_tls:
        raise RuntimeError(
            "local Docker endpoint cannot use TLS environment or context material"
        )

    if host.facts.provider == "colima" and context != "colima":
        raise RuntimeError(
            f"host provider colima requires Docker context colima; found {context}"
        )
    if host.facts.provider not in {"colima", "docker", "docker-desktop"}:
        raise RuntimeError(f"host selected unsupported Docker provider {host.facts.provider}")

    configured_platform = os.environ.get("DOCKER_DEFAULT_PLATFORM", "").strip()
    if configured_platform and configured_platform != target_platform:
        raise RuntimeError(
            "DOCKER_DEFAULT_PLATFORM selects "
            f"{configured_platform}, expected native {target_platform}"
        )

    info = json.loads(
        _check_output(
            ["docker", "info", "--format", "{{json .}}"],
            stderr=subprocess.DEVNULL,
            text=True,
        )
    )
    if not isinstance(info, dict):
        raise RuntimeError("Docker engine returned invalid information")
    platform = _platform(info.get("OSType"), info.get("Architecture"))
    if platform != target_platform:
        raise RuntimeError(
            f"Docker engine has platform {platform}, expected native {target_platform}"
        )
    return _DockerState(context=context, endpoint=endpoint, platform=platform)


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
    platform = _platform(document["Os"], document["Architecture"])
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
    """Inspect the existing native image set without changing it."""
    root = project_directory.resolve()
    revision = _revision(root)
    tree_digest = runtime_input_digest(root)
    target_platform = _target_platform(host)
    record_path = root / ".local-runtime/build.json"
    reasons: list[str] = []
    docker_state: _DockerState | None = None
    try:
        docker_state = _docker_state(host)
        docker_context: str | None = docker_state.context
        server_platform: str | None = docker_state.platform
    except (
        OSError,
        RuntimeError,
        subprocess.CalledProcessError,
        json.JSONDecodeError,
    ) as error:
        try:
            docker_context = _check_output(
                ["docker", "context", "show"],
                stderr=subprocess.DEVNULL,
                text=True,
            ).strip() or None
        except (OSError, subprocess.CalledProcessError):
            docker_context = None
        server_platform = None
        reasons.append(str(error) or "Docker engine is unavailable")
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
    for image_ref in IMAGE_REFS if docker_state is not None else ():
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
        if inspection["os_architecture"] != target_platform:
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
        if record.get("target_platform") != target_platform:
            reasons.append(f"local image build record is not for {target_platform}")
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
    """Build and record the eight native runtime images under one lease."""
    root = project_directory.resolve()
    with workstation.hold(allow_nested=True):
        target_platform = _target_platform(host)
        docker_state = _docker_state(host)
        revision = _revision(root)
        tree_digest = runtime_input_digest(root)
        environment = {
            **os.environ,
            "DOCKER_DEFAULT_PLATFORM": target_platform,
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
            if image["os_architecture"] != target_platform:
                raise RuntimeError(
                    f'image {image["image"]} has platform {image["os_architecture"]}, expected {target_platform}'
                )
        final_docker_state = _docker_state(host)
        if final_docker_state != docker_state:
            raise RuntimeError(
                "Docker context, endpoint, or platform changed during the build"
            )
        record = {
            "schema_version": 1,
            "source_revision": revision,
            "source_tree_sha256": tree_digest,
            "docker_context": docker_state.context,
            "target_platform": target_platform,
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
