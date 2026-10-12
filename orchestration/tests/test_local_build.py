from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from orchestration.local_build import runtime_input_digest


EXPECTED = {
    "baseline": "96bca58be5c193746ce28394d198f6619fc3d2392eb77f609accf707a483292a",
    "changed_tracked_contents": "b0f1b738cb5527bc64976838ee336620cc8efbba19371e6dea91ef088ac10252",
    "tracked_deletion": "31ad972eb3c4280d904e061f5ee83885c0dfd9ce14fb1e3f7af478ade296eafa",
    "changed_subdirectory": "471a04afd99e082504670712e5a076c4bc7a03f68f74767c381e59cc7ee0e991",
    "changed_untracked_contents": "0714c46899fee82f3189742c58097004632ee936a1cd9f590c4c774e35abced1",
    "changed_symlink_target": "e68e239fa3f02b9f4670cb68f2d8b08d52a31ebc4f90a2faac1c60367984c770",
    "changed_outside_symlink_target": "60f9eb1ae2ef735d489d1de3437d4337d8db1d5a7ad70402ed224e7fd861062b",
    "excluded_only_changes": "96bca58be5c193746ce28394d198f6619fc3d2392eb77f609accf707a483292a",
}


def _write(root: Path, relative: str, payload: bytes) -> None:
    target = root / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(payload)


def _fixture(root: Path) -> None:
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    subprocess.run(["git", "-C", str(root), "config", "user.email", "fixture@example.invalid"], check=True)
    subprocess.run(["git", "-C", str(root), "config", "user.name", "fixture"], check=True)
    payloads = {
        ".gitignore": b"gazebo/ignored.txt\n",
        ".dockerignore": b"fixture dockerignore\n",
        "compose.yaml": b"services: {}\n",
        "compose.local.yaml": b"services: {}\n",
        "pyproject.toml": b'[project]\nname = "legacy-fixture"\n',
        "uv.lock": b"version = 1\n",
        "scripts/local-build.sh": b"fixture legacy input\n",
        "config/00-ascii.txt": b"ascii\n",
        "config/ß.txt": b"latin unicode\n",
        "config/Ω.txt": b"greek unicode\n",
        "config/中.txt": b"cjk unicode\n",
        "config/😀.txt": b"emoji unicode\n",
        "config/line\nbreak.txt": b"newline filename\n",
        "config/back\\slash.txt": b"backslash filename\n",
        "config/deep/nested.txt": b"nested baseline\n",
        "config/symlink-target.txt": b"symlink target baseline\n",
        "outside-target.txt": b"outside target baseline\n",
        "ardupilot_sitl/tracked.txt": b"ardupilot\n",
        "artifacts/tracked.txt": b"artifacts\n",
        "companion/tracked.txt": b"companion\n",
        "electromagnet/tracked.txt": b"electromagnet\n",
        "gazebo/tracked.txt": b"gazebo\n",
        "orchestration/tracked.txt": b"orchestration\n",
        "ros_ws/tracked.txt": b"ros workspace\n",
        "scorekeeper/tracked.txt": b"scorekeeper\n",
        "scorekeeper/delete-me.txt": b"tracked before deletion\n",
        "gazebo/untracked.txt": b"eligible untracked baseline\n",
        "gazebo/ignored.txt": b"ignored baseline\n",
        "docs/outside-tracked.txt": b"tracked outside pathspec\n",
        "docs/outside-untracked.txt": b"untracked outside pathspec\n",
    }
    for relative, payload in payloads.items():
        _write(root, relative, payload)
    os.symlink("symlink-target.txt", root / "config/link-to-file")
    os.symlink("../outside-target.txt", root / "config/link-to-outside")
    os.symlink("missing-target.txt", root / "config/broken-link")
    os.symlink("deep", root / "config/link-to-directory")
    tracked = [
        ".gitignore", ".dockerignore", "compose.yaml", "compose.local.yaml",
        "pyproject.toml", "uv.lock", "scripts/local-build.sh", "config",
        "ardupilot_sitl/tracked.txt", "artifacts/tracked.txt",
        "companion/tracked.txt", "electromagnet/tracked.txt", "gazebo/tracked.txt",
        "orchestration/tracked.txt", "ros_ws/tracked.txt", "scorekeeper/tracked.txt",
        "scorekeeper/delete-me.txt", "docs/outside-tracked.txt",
    ]
    subprocess.run(["git", "-C", str(root), "add", "--", *tracked], check=True)


def _mutate(root: Path, scenario: str) -> None:
    if scenario == "baseline":
        return
    if scenario == "changed_tracked_contents":
        _write(root, "config/00-ascii.txt", b"ascii changed\n")
    elif scenario == "tracked_deletion":
        (root / "scorekeeper/delete-me.txt").unlink()
    elif scenario == "changed_subdirectory":
        _write(root, "config/deep/nested.txt", b"nested changed\n")
    elif scenario == "changed_untracked_contents":
        _write(root, "gazebo/untracked.txt", b"eligible untracked changed\n")
    elif scenario == "changed_symlink_target":
        _write(root, "config/symlink-target.txt", b"symlink target changed\n")
    elif scenario == "changed_outside_symlink_target":
        _write(root, "outside-target.txt", b"outside target changed\n")
    elif scenario == "excluded_only_changes":
        _write(root, "gazebo/ignored.txt", b"ignored changed\n")
        _write(root, "docs/outside-tracked.txt", b"tracked outside changed\n")
        _write(root, "docs/outside-untracked.txt", b"untracked outside changed\n")


@pytest.mark.parametrize("scenario", EXPECTED)
def test_runtime_input_digest_matches_captured_legacy_oracle(
    tmp_path: Path, scenario: str
) -> None:
    fixture = tmp_path / scenario
    fixture.mkdir()
    _fixture(fixture)
    _mutate(fixture, scenario)

    assert runtime_input_digest(fixture) == EXPECTED[scenario]


def test_check_build_preserves_record_schema_and_eight_image_order(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import orchestration.local_build as local_build

    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    _write(tmp_path, "scripts/local-build.sh", b"launcher\n")
    subprocess.run(["git", "-C", str(tmp_path), "add", "scripts/local-build.sh"], check=True)
    subprocess.run(
        ["git", "-C", str(tmp_path), "-c", "user.name=test", "-c", "user.email=test@example.invalid", "commit", "-qm", "fixture"],
        check=True,
    )
    revision = subprocess.check_output(["git", "-C", str(tmp_path), "rev-parse", "HEAD"], text=True).strip()
    images = []
    for index, image in enumerate(local_build.IMAGE_REFS):
        images.append({
            "image": image,
            "image_id": f"sha256:{index:064x}",
            "repo_digests": [],
            "os_architecture": "linux/arm64",
        })
    record = {
        "schema_version": 1,
        "source_revision": revision,
        "source_tree_sha256": runtime_input_digest(tmp_path),
        "docker_context": "colima",
        "target_platform": "linux/arm64",
        "images": images,
    }
    (tmp_path / ".local-runtime").mkdir()
    (tmp_path / ".local-runtime/build.json").write_text(json.dumps(record))

    real_output = local_build._check_output

    def output(command, **kwargs):
        if command[1:3] == ["context", "show"]:
            return "colima\n"
        if command[1:3] == ["info", "--format"]:
            return "linux/aarch64\n"
        if command[1:3] == ["image", "inspect"]:
            image = command[-1]
            index = local_build.IMAGE_REFS.index(image)
            labels = ({"org.opencontainers.image.comp2026.revision": revision}
                      if image == "drone-sim-companion-runtime:phase3" else {})
            return json.dumps([{
                "Id": f"sha256:{index:064x}", "Os": "linux", "Architecture": "arm64",
                "RepoDigests": [], "Config": {"Labels": labels},
            }])
        return real_output(command, **kwargs)

    monkeypatch.setattr(local_build, "_check_output", output)

    result = local_build.check_build(tmp_path, object())

    assert result["ok"] is True
    assert [row["image"] for row in result["images"]] == list(local_build.IMAGE_REFS)


def test_standalone_builder_loses_real_process_contention_before_docker(
    tmp_path: Path,
) -> None:
    from contextlib import contextmanager
    from types import SimpleNamespace

    from orchestration.workstation import Workstation

    lock_path = tmp_path / "workstation.lock"

    class Host:
        facts = SimpleNamespace(kind="linux")
        state_directory = tmp_path / "state"

        @contextmanager
        def acquire_native_lease(self):
            yield object()

    with Workstation(Host(), lock_path=lock_path).hold():
        code = (
            "from contextlib import contextmanager\n"
            "from pathlib import Path\n"
            "from types import SimpleNamespace\n"
            "from orchestration.local_build import build_images\n"
            "from orchestration.workstation import Workstation\n"
            "class H:\n"
            "  facts=SimpleNamespace(kind='linux')\n"
            "  state_directory=Path('/unused')\n"
            "  @contextmanager\n"
            "  def acquire_native_lease(self): yield object()\n"
            "host=H()\n"
            f"build_images(Path({str(tmp_path)!r}), host, "
            f"Workstation(host, lock_path=Path({str(lock_path)!r})))\n"
        )
        result = subprocess.run(
            [sys.executable, "-c", code], capture_output=True, text=True
        )

    assert result.returncode != 0
    assert "already owns" in result.stderr
    assert "docker" not in result.stderr.lower()
