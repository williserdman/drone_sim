#!/usr/bin/env bash
set -euo pipefail

root=$(git -C "$(dirname "${BASH_SOURCE[0]}")/.." rev-parse --show-toplevel)
image=${DRONE_SIM_LINUX_TEST_IMAGE:-drone-sim-linux-test:python3.12}
volume=${DRONE_SIM_LINUX_TEST_VOLUME:-drone-sim-linux-test}

if [[ ! $volume =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
    echo "DRONE_SIM_LINUX_TEST_VOLUME is not a valid Docker volume name: $volume" >&2
    exit 2
fi

for command in docker git python3; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "required command is unavailable: $command" >&2
        exit 2
    fi
done

snapshot_parent=$root/.local-runtime/test-linux
mkdir -p "$snapshot_parent"
snapshot=$(mktemp -d "$snapshot_parent/snapshot.XXXXXX")
cleanup() {
    rm -rf "$snapshot"
}
trap cleanup EXIT

source_head=$(git -C "$root" rev-parse HEAD)
printf '%s\n' "$source_head" > "$snapshot/source-head"
git -C "$root" status --porcelain=v1 --untracked-files=all > "$snapshot/source-status"
git -C "$root" bundle create "$snapshot/repository.bundle" HEAD

SNAPSHOT_ROOT=$root SNAPSHOT_TAR=$snapshot/source.tar python3 <<'PY'
from __future__ import annotations

import os
from pathlib import Path, PurePosixPath
import subprocess
import tarfile


root = Path(os.environ["SNAPSHOT_ROOT"])
archive_path = Path(os.environ["SNAPSHOT_TAR"])
excluded_components = {
    ".local-runtime",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".venv",
    "__pycache__",
    "build",
    "cache",
    "runs",
}
listed = subprocess.check_output(
    [
        "git",
        "-C",
        str(root),
        "ls-files",
        "-z",
        "--cached",
        "--others",
        "--exclude-standard",
    ]
).split(b"\0")


def normalized(info: tarfile.TarInfo) -> tarfile.TarInfo:
    info.uid = 0
    info.gid = 0
    info.uname = "root"
    info.gname = "root"
    return info


with tarfile.open(archive_path, "w") as archive:
    for encoded in listed:
        if not encoded:
            continue
        relative = os.fsdecode(encoded)
        path = PurePosixPath(relative)
        if path.is_absolute() or ".." in path.parts:
            raise SystemExit(f"unsafe Git path in source snapshot: {relative!r}")
        if any(component in excluded_components for component in path.parts):
            continue
        source = root / relative
        if not source.exists() and not source.is_symlink():
            continue
        archive.add(
            source,
            arcname=relative,
            recursive=False,
            filter=normalized,
        )
PY

if [[ $(git -C "$root" rev-parse HEAD) != "$source_head" ]]; then
    echo "source HEAD changed while the Linux test snapshot was created; retry" >&2
    exit 2
fi

docker build \
    --file "$root/tests/linux/Dockerfile" \
    --tag "$image" \
    "$root/tests/linux"
docker volume create "$volume" >/dev/null

docker run --rm \
    --mount "type=volume,src=$volume,dst=/linux-test" \
    --mount "type=bind,src=$snapshot,dst=/snapshot,readonly" \
    --workdir / \
    --env "SOURCE_HEAD=$source_head" \
    "$image" \
    bash -euc '
        rm -rf /linux-test/project.next /linux-test/project.previous
        git -c init.defaultBranch=main clone --quiet --no-checkout \
            /snapshot/repository.bundle /linux-test/project.next
        git -C /linux-test/project.next checkout --quiet --detach "$SOURCE_HEAD"
        find /linux-test/project.next -mindepth 1 -maxdepth 1 ! -name .git -exec rm -rf -- {} +
        tar --extract --file=/snapshot/source.tar --directory=/linux-test/project.next
        tar --compare --file=/snapshot/source.tar --directory=/linux-test/project.next
        test "$(git -C /linux-test/project.next rev-parse HEAD)" = "$SOURCE_HEAD"
        if [ -d /linux-test/project ]; then
            mv /linux-test/project /linux-test/project.previous
        fi
        mv /linux-test/project.next /linux-test/project
        rm -rf /linux-test/project.previous
        mkdir -p /linux-test/home /linux-test/provenance /linux-test/tmp /linux-test/uv-cache
        cp /snapshot/source-head /snapshot/source-status /linux-test/provenance/
        sha256sum /snapshot/source.tar > /linux-test/provenance/source-tar.sha256
        git -C /linux-test/project status --porcelain=v1 --untracked-files=all \
            > /linux-test/provenance/snapshot-status
    '

docker run --rm \
    --mount "type=volume,src=$volume,dst=/linux-test" \
    --workdir / \
    --env HOME=/linux-test/home \
    "$image" \
    bash -euc '
        cd /linux-test/project
        uv sync --frozen
    '

echo "Linux source snapshot: $source_head"
if [[ -s $snapshot/source-status ]]; then
    echo "Linux source snapshot includes tracked or untracked worktree changes."
else
    echo "Linux source snapshot matches the recorded HEAD."
fi
echo "Linux test volume: $volume"

docker run --rm \
    --mount "type=volume,src=$volume,dst=/linux-test" \
    --workdir / \
    --env HOME=/linux-test/home \
    --env DRONE_SIM_LINUX_DOCKER_DAEMON=0 \
    --env PYTHONPATH=/linux-test/project/tests/linux \
    "$image" \
    bash -euc '
        run_tmp=$(mktemp -d /linux-test/tmp/pytest.XXXXXX)
        cleanup_test_tmp() {
            rm -rf "$run_tmp"
        }
        trap cleanup_test_tmp EXIT
        mkdir -p "$run_tmp/media"
        export TMPDIR=$run_tmp
        export DRONE_SIM_TEST_RUN_VOLUME=$run_tmp/media
        cd /linux-test/project
        if /linux-test/venv/bin/python -m pytest \
            -p pytest_plugin \
            -p no:cacheprovider \
            --basetemp="$run_tmp/pytest" \
            "$@"; then
            status=0
        else
            status=$?
        fi
        exit "$status"
    ' bash "$@"
