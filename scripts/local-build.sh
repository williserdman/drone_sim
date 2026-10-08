#!/usr/bin/env bash
set -euo pipefail

EXPECTED_ARCH="arm64"
EXPECTED_PLATFORM="linux/${EXPECTED_ARCH}"
SCRIPT_DIRECTORY="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIRECTORY}/.." && pwd)"
BUILD_RECORD="${PROJECT_ROOT}/.local-runtime/build.json"

if [[ $# -ne 1 || ( "$1" != "--build" && "$1" != "--check" ) ]]; then
    printf 'usage: %s --build|--check\n' "$0" >&2
    exit 2
fi
mode="$1"

if [[ "${mode}" == "--build" \
    && "${DRONE_SIM_WORKSTATION_LOCK_HELD:-}" != "1" ]]; then
    exec python3 - "$0" "$@" <<'PY'
from __future__ import annotations

import fcntl
import os
from pathlib import Path
import stat
import subprocess
import sys


script, *arguments = sys.argv[1:]
lock_path = Path(f"/tmp/drone-sim-suite-{os.getuid()}.lock")
try:
    descriptor = os.open(
        lock_path,
        os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC,
        0o600,
    )
except OSError as error:
    print(f"could not open the workstation lease: {error}", file=sys.stderr)
    raise SystemExit(1)

try:
    metadata = os.fstat(descriptor)
    if (
        not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != os.getuid()
        or metadata.st_nlink != 1
    ):
        print("unsafe workstation lease file", file=sys.stderr)
        raise SystemExit(1)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(
            "another drone-sim session already owns this workstation (already running)",
            file=sys.stderr,
        )
        raise SystemExit(1)
    environment = dict(os.environ)
    environment["DRONE_SIM_WORKSTATION_LOCK_HELD"] = "1"
    raise SystemExit(subprocess.run([script, *arguments], env=environment).returncode)
finally:
    os.close(descriptor)
PY
fi

compute_source_tree_sha256() {
    {
        git -C "${PROJECT_ROOT}" ls-files \
            --cached --others --exclude-standard -z -- \
            .dockerignore compose.yaml compose.local.yaml config \
            pyproject.toml uv.lock scripts/local-build.sh \
            ardupilot_sitl artifacts companion electromagnet gazebo \
            orchestration ros_ws scorekeeper \
            | LC_ALL=C sort -z \
            | while IFS= read -r -d '' relative_path; do
                printf '%s\0' "${relative_path}"
                if [[ -f "${PROJECT_ROOT}/${relative_path}" ]]; then
                    shasum -a 256 "${PROJECT_ROOT}/${relative_path}" \
                        | awk '{print $1}'
                else
                    printf 'deleted\n'
                fi
            done
    } | shasum -a 256 | awk '{print $1}'
}

revision="$(git -C "${PROJECT_ROOT}" rev-parse HEAD)"
source_tree_sha256="$(compute_source_tree_sha256)"

runtime_services=(
    orchestration-runtime
    artifacts-runtime
    companion-runtime
    ardupilot-sitl
    gazebo-runtime
    electromagnet-runtime
    scorekeeper-runtime
)
image_refs=(
    drone-sim-orchestration-runtime:phase2
    drone-sim-artifacts-runtime:phase2
    drone-sim-companion-runtime:phase3
    drone-sim-ardupilot-runtime:phase3
    drone-sim-gazebo-runtime:phase3
    drone-sim-electromagnet-runtime:phase3
    drone-sim-scorekeeper-runtime:phase3
    drone-sim-gazebo-viewer:local
)

check_build() {
    SOURCE_TREE_SHA256="${source_tree_sha256}" \
    SOURCE_REVISION="${revision}" \
    BUILD_RECORD="${BUILD_RECORD}" \
    IMAGE_REFS="$(IFS=,; printf '%s' "${image_refs[*]}")" \
    python3 - <<'PY'
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys


record_path = Path(os.environ["BUILD_RECORD"])
reasons: list[str] = []

try:
    docker_context = subprocess.check_output(
        ["docker", "context", "show"], stderr=subprocess.DEVNULL, text=True
    ).strip()
except (OSError, subprocess.CalledProcessError):
    docker_context = None
    reasons.append("Docker context is unavailable")

try:
    server_platform = subprocess.check_output(
        ["docker", "info", "--format", "{{.OSType}}/{{.Architecture}}"],
        stderr=subprocess.DEVNULL,
        text=True,
    ).strip()
    if server_platform == "linux/aarch64":
        server_platform = "linux/arm64"
    if server_platform != "linux/arm64":
        reasons.append(f"Docker engine has platform {server_platform}")
except (OSError, subprocess.CalledProcessError):
    server_platform = None
    reasons.append("Docker engine is unavailable")

try:
    record = json.loads(record_path.read_text(encoding="utf-8"))
except FileNotFoundError:
    record = None
    reasons.append("local image build record is missing")
except (OSError, json.JSONDecodeError) as error:
    record = None
    reasons.append(f"local image build record is unreadable: {error}")

expected_images = os.environ["IMAGE_REFS"].split(",")
current_images = []
for image_ref in expected_images:
    try:
        inspection = json.loads(
            subprocess.check_output(
                ["docker", "image", "inspect", image_ref],
                stderr=subprocess.DEVNULL,
                text=True,
            )
        )[0]
    except (subprocess.CalledProcessError, json.JSONDecodeError, IndexError, KeyError):
        reasons.append(f"local image is missing or unreadable: {image_ref}")
        continue
    platform = f'{inspection["Os"]}/{inspection["Architecture"]}'
    current_images.append(
        {
            "image": image_ref,
            "image_id": inspection["Id"],
            "repo_digests": sorted(inspection.get("RepoDigests") or []),
            "os_architecture": platform,
        }
    )
    if platform != "linux/arm64":
        reasons.append(f"local image has platform {platform}: {image_ref}")
    if image_ref == "drone-sim-companion-runtime:phase3":
        labels = inspection.get("Config", {}).get("Labels") or {}
        if labels.get("org.opencontainers.image.comp2026.revision") != os.environ["SOURCE_REVISION"]:
            reasons.append("companion image source revision label does not match HEAD")

if record is not None:
    if record.get("schema_version") != 1:
        reasons.append("local image build record schema is unsupported")
    if record.get("source_revision") != os.environ["SOURCE_REVISION"]:
        reasons.append("source revision changed since the local image build")
    if record.get("source_tree_sha256") != os.environ["SOURCE_TREE_SHA256"]:
        reasons.append("runtime build inputs changed since the local image build")
    if record.get("docker_context") != docker_context:
        reasons.append("Docker context changed since the local image build")
    if record.get("target_platform") != "linux/arm64":
        reasons.append("local image build record is not for linux/arm64")
    recorded_images = record.get("images")
    if recorded_images != current_images:
        reasons.append("local image identities changed since the build record")

diagnostic = {
    "ok": not reasons,
    "record_path": str(record_path),
    "reasons": reasons,
    "current_source": {
        "revision": os.environ["SOURCE_REVISION"],
        "tree_sha256": os.environ["SOURCE_TREE_SHA256"],
    },
    "current_platform": server_platform,
    "current_context": docker_context,
    "images": current_images,
}
print(json.dumps(diagnostic, sort_keys=True, separators=(",", ":")))
raise SystemExit(0 if diagnostic["ok"] else 1)
PY
}

if [[ "${mode}" == "--check" ]]; then
    check_build
    exit $?
fi

if ! actual_arch="$(docker version --format '{{.Server.Arch}}')"; then
    printf 'local build requires a running arm64 Docker engine\n' >&2
    exit 1
fi
case "${actual_arch}" in
    arm64|aarch64) ;;
    *)
        printf 'local build requires an arm64 Docker engine; found %s\n' \
            "${actual_arch}" >&2
        exit 1
        ;;
esac

cd "${PROJECT_ROOT}"
export DOCKER_DEFAULT_PLATFORM="${EXPECTED_PLATFORM}"
export SIM_COMP2026_REVISION="${revision}"

docker compose \
    --file compose.yaml \
    --file compose.local.yaml \
    --profile phase3 \
    build "${runtime_services[@]}" >&2
docker compose \
    --file compose.yaml \
    --file compose.local.yaml \
    --profile phase3 \
    --profile local \
    build gazebo-viewer >&2

revision_after_build="$(git -C "${PROJECT_ROOT}" rev-parse HEAD)"
source_tree_sha256_after_build="$(compute_source_tree_sha256)"
if [[ "${revision_after_build}" != "${revision}" \
    || "${source_tree_sha256_after_build}" != "${source_tree_sha256}" ]]; then
    printf '%s\n' \
        'runtime build inputs changed during the build; refusing to publish a mixed-source build record' >&2
    exit 1
fi

for image_ref in "${image_refs[@]}"; do
    image_platform="$(docker image inspect --format '{{.Os}}/{{.Architecture}}' "${image_ref}")"
    if [[ "${image_platform}" != "${EXPECTED_PLATFORM}" ]]; then
        printf 'image %s has platform %s, expected %s\n' \
            "${image_ref}" "${image_platform}" "${EXPECTED_PLATFORM}" >&2
        exit 1
    fi
done

mkdir -p "$(dirname -- "${BUILD_RECORD}")"
record_tmp="$(mktemp "${BUILD_RECORD}.tmp.XXXXXX")"
trap 'rm -f -- "${record_tmp}"' EXIT
SOURCE_TREE_SHA256="${source_tree_sha256}" \
SOURCE_REVISION="${revision}" \
DOCKER_CONTEXT="$(docker context show)" \
BUILD_RECORD_TMP="${record_tmp}" \
IMAGE_REFS="$(IFS=,; printf '%s' "${image_refs[*]}")" \
python3 - <<'PY'
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess


images = []
for image_ref in os.environ["IMAGE_REFS"].split(","):
    inspection = json.loads(
        subprocess.check_output(
            ["docker", "image", "inspect", image_ref], text=True
        )
    )[0]
    images.append(
        {
            "image": image_ref,
            "image_id": inspection["Id"],
            "repo_digests": sorted(inspection.get("RepoDigests") or []),
            "os_architecture": f'{inspection["Os"]}/{inspection["Architecture"]}',
        }
    )
    if image_ref == "drone-sim-companion-runtime:phase3":
        labels = inspection.get("Config", {}).get("Labels") or {}
        if labels.get("org.opencontainers.image.comp2026.revision") != os.environ["SOURCE_REVISION"]:
            raise RuntimeError("companion image source revision label does not match HEAD")

document = {
    "schema_version": 1,
    "source_revision": os.environ["SOURCE_REVISION"],
    "source_tree_sha256": os.environ["SOURCE_TREE_SHA256"],
    "docker_context": os.environ["DOCKER_CONTEXT"],
    "target_platform": "linux/arm64",
    "images": images,
}
record_tmp = Path(os.environ["BUILD_RECORD_TMP"])
pending = record_tmp.with_suffix(record_tmp.suffix + ".pending")
pending.write_text(
    json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
os.replace(pending, record_tmp)
PY
chmod 0600 "${record_tmp}"

revision_before_publish="$(git -C "${PROJECT_ROOT}" rev-parse HEAD)"
source_tree_sha256_before_publish="$(compute_source_tree_sha256)"
if [[ "${revision_before_publish}" != "${revision}" \
    || "${source_tree_sha256_before_publish}" != "${source_tree_sha256}" ]]; then
    printf '%s\n' \
        'runtime build inputs changed before record publication; refusing to publish a mixed-source build record' >&2
    exit 1
fi

mv -f -- "${record_tmp}" "${BUILD_RECORD}"
trap - EXIT
check_build
