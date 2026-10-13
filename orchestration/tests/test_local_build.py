from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import time
from contextlib import contextmanager
from types import SimpleNamespace
from typing import Iterator

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


@pytest.fixture(autouse=True)
def _clean_docker_environment(monkeypatch: pytest.MonkeyPatch) -> None:
    for name in (
        "DOCKER_CONTEXT",
        "DOCKER_HOST",
        "DOCKER_TLS",
        "DOCKER_TLS_VERIFY",
        "DOCKER_CERT_PATH",
        "DOCKER_DEFAULT_PLATFORM",
    ):
        monkeypatch.delenv(name, raising=False)


class _Workstation:
    @contextmanager
    def hold(self, *, allow_nested: bool = False) -> Iterator[None]:
        assert allow_nested is True
        yield


def _host(platform: str = "linux/arm64", *, provider: str = "colima"):
    machine = "arm64" if platform == "linux/arm64" else "amd64"
    kind = "macos" if provider == "colima" else "linux"
    return SimpleNamespace(
        facts=SimpleNamespace(
            image_platform=platform,
            machine=machine,
            provider=provider,
            kind=kind,
        )
    )


def _committed_fixture(root: Path) -> str:
    subprocess.run(["git", "init", "-q", str(root)], check=True)
    _write(root, "scripts/local-build.sh", b"launcher\n")
    subprocess.run(
        ["git", "-C", str(root), "add", "scripts/local-build.sh"], check=True
    )
    subprocess.run(
        [
            "git", "-C", str(root), "-c", "user.name=test",
            "-c", "user.email=test@example.invalid", "commit", "-qm", "fixture",
        ],
        check=True,
    )
    return subprocess.check_output(
        ["git", "-C", str(root), "rev-parse", "HEAD"], text=True
    ).strip()


def _image_rows(local_build, platform: str) -> list[dict[str, object]]:
    return [
        {
            "image": image,
            "image_id": f"sha256:{index:064x}",
            "repo_digests": [],
            "os_architecture": platform,
        }
        for index, image in enumerate(local_build.IMAGE_REFS)
    ]


def _write_build_record(
    root: Path,
    local_build,
    revision: str,
    platform: str,
    *,
    context: str = "colima",
) -> Path:
    record_path = root / ".local-runtime/build.json"
    record_path.parent.mkdir()
    record_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "source_revision": revision,
                "source_tree_sha256": runtime_input_digest(root),
                "docker_context": context,
                "target_platform": platform,
                "images": _image_rows(local_build, platform),
            }
        )
    )
    return record_path


def _docker_output(
    local_build,
    revision: str,
    *,
    architecture: str = "aarch64",
    context: str = "colima",
    endpoint: str = "unix:///Users/test/.colima/default/docker.sock",
    missing_image: str | None = None,
    replaced_image: str | None = None,
    wrong_image_architecture: str | None = None,
    companion_revision: str | None = None,
):
    real_output = local_build._check_output

    def output(command, **kwargs):
        if command[1:3] == ["context", "show"]:
            return f"{context}\n"
        if command[1:3] == ["context", "inspect"]:
            return json.dumps(
                [{
                    "Name": context,
                    "Endpoints": {
                        "docker": {"Host": endpoint, "SkipTLSVerify": False}
                    },
                    "TLSMaterial": {},
                    "Storage": {"TLSPath": "/Users/test/.docker/contexts/tls/local"},
                }]
            )
        if command[1:3] == ["info", "--format"]:
            return json.dumps(
                {"OSType": "linux", "Architecture": architecture, "Name": context}
            )
        if command[1:3] == ["image", "inspect"]:
            image = command[-1]
            if image == missing_image:
                raise subprocess.CalledProcessError(1, command)
            index = local_build.IMAGE_REFS.index(image)
            labels = (
                {
                    "org.opencontainers.image.comp2026.revision": (
                        companion_revision or revision
                    )
                }
                if image == "drone-sim-companion-runtime:phase3"
                else {}
            )
            image_architecture = architecture
            if image == wrong_image_architecture:
                image_architecture = (
                    "amd64" if architecture in {"arm64", "aarch64"} else "arm64"
                )
            image_id_index = index + 20 if image == replaced_image else index
            return json.dumps(
                [{
                    "Id": f"sha256:{image_id_index:064x}",
                    "Os": "linux",
                    "Architecture": image_architecture,
                    "RepoDigests": [],
                    "Config": {"Labels": labels},
                }]
            )
        return real_output(command, **kwargs)

    return output


@contextmanager
def _standalone_builder_holding_lease(tmp_path: Path):
    lock_path = tmp_path / "workstation.lock"
    ready = tmp_path / "builder-ready"
    release = tmp_path / "builder-release"
    code = f"""
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace
import time
import orchestration.local_build as local_build
from orchestration.workstation import Workstation
class Host:
    facts = SimpleNamespace(
        kind='linux', machine='arm64', image_platform='linux/arm64', provider='docker'
    )
    state_directory = Path({str(tmp_path / 'builder-state')!r})
    @contextmanager
    def acquire_native_lease(self):
        yield object()
def revision(_root):
    Path({str(ready)!r}).write_text('ready')
    while not Path({str(release)!r}).exists():
        time.sleep(0.01)
    raise RuntimeError('builder released')
local_build._revision = revision
local_build._docker_state = lambda _host: local_build._DockerState(
    context='default', endpoint='unix:///var/run/docker.sock', platform='linux/arm64'
)
host = Host()
local_build.build_images(
    Path({str(tmp_path)!r}), host,
    Workstation(host, lock_path=Path({str(lock_path)!r}))
)
"""
    process = subprocess.Popen(
        [sys.executable, "-c", code], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True,
    )
    deadline = time.monotonic() + 5
    while not ready.exists() and process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert ready.exists(), process.communicate(timeout=1)
    try:
        yield lock_path
    finally:
        release.write_text("release")
        _stdout, stderr = process.communicate(timeout=5)
        assert process.returncode != 0
        assert "builder released" in stderr


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

    monkeypatch.delenv("DOCKER_DEFAULT_PLATFORM", raising=False)
    monkeypatch.setattr(
        local_build, "_check_output", _docker_output(local_build, revision)
    )

    result = local_build.check_build(tmp_path, _host())

    assert result["ok"] is True
    assert [row["image"] for row in result["images"]] == list(local_build.IMAGE_REFS)


@pytest.mark.parametrize(
    ("platform", "daemon_architecture", "provider", "context", "endpoint"),
    [
        (
            "linux/arm64", "arm64", "colima", "colima",
            "unix:///Users/test/.colima/default/docker.sock",
        ),
        (
            "linux/arm64", "aarch64", "docker", "default",
            "unix:///var/run/docker.sock",
        ),
        (
            "linux/amd64", "amd64", "docker", "default",
            "unix:///var/run/docker.sock",
        ),
        (
            "linux/amd64", "x86_64", "docker-desktop", "default",
            "unix:///var/run/docker.sock",
        ),
    ],
)
def test_check_build_accepts_native_docker_architecture_aliases(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    platform: str,
    daemon_architecture: str,
    provider: str,
    context: str,
    endpoint: str,
) -> None:
    import orchestration.local_build as local_build

    revision = _committed_fixture(tmp_path)
    _write_build_record(tmp_path, local_build, revision, platform, context=context)
    monkeypatch.delenv("DOCKER_HOST", raising=False)
    monkeypatch.delenv("DOCKER_CONTEXT", raising=False)
    monkeypatch.delenv("DOCKER_DEFAULT_PLATFORM", raising=False)
    monkeypatch.setattr(
        local_build,
        "_check_output",
        _docker_output(
            local_build,
            revision,
            architecture=daemon_architecture,
            context=context,
            endpoint=endpoint,
        ),
    )

    result = local_build.check_build(
        tmp_path, _host(platform, provider=provider)
    )

    assert result["ok"] is True
    assert result["current_platform"] == platform


def test_environment_cannot_hide_remote_daemon(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import orchestration.local_build as local_build

    revision = _committed_fixture(tmp_path)
    monkeypatch.setenv("DOCKER_CONTEXT", "colima")
    monkeypatch.delenv("DOCKER_HOST", raising=False)
    monkeypatch.setattr(
        local_build,
        "_check_output",
        _docker_output(
            local_build,
            revision,
            context="colima",
            endpoint="ssh://builder@example.invalid",
        ),
    )
    build_calls: list[list[str]] = []
    monkeypatch.setattr(
        local_build,
        "_run",
        lambda command, **_kwargs: build_calls.append(list(command)),
    )

    result = local_build.check_build(tmp_path, _host())
    with pytest.raises(RuntimeError, match="local Docker endpoint"):
        local_build.build_images(tmp_path, _host(), _Workstation())

    assert result["ok"] is False
    assert build_calls == []


def test_explicit_local_context_overrides_ambient_remote_docker_host(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import orchestration.local_build as local_build

    revision = _committed_fixture(tmp_path)
    _write_build_record(tmp_path, local_build, revision, "linux/arm64")
    monkeypatch.setenv("DOCKER_CONTEXT", "colima")
    monkeypatch.setenv("DOCKER_HOST", "tcp://remote.example.invalid:2376")
    monkeypatch.setattr(
        local_build,
        "_check_output",
        _docker_output(local_build, revision),
    )

    assert local_build.check_build(tmp_path, _host())["ok"] is True


@pytest.mark.parametrize(
    ("name", "value", "reason"),
    [
        ("DOCKER_HOST", "tcp://remote.example.invalid:2376", "local Docker endpoint"),
        ("DOCKER_TLS", "1", "TLS environment"),
        ("DOCKER_TLS_VERIFY", "1", "TLS environment"),
        ("DOCKER_CERT_PATH", "/tmp/docker-certificates", "TLS environment"),
    ],
)
def test_ambient_remote_or_tls_docker_environment_fails_closed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    name: str,
    value: str,
    reason: str,
) -> None:
    import orchestration.local_build as local_build

    revision = _committed_fixture(tmp_path)
    for variable in (
        "DOCKER_CONTEXT",
        "DOCKER_HOST",
        "DOCKER_TLS",
        "DOCKER_TLS_VERIFY",
        "DOCKER_CERT_PATH",
        "DOCKER_DEFAULT_PLATFORM",
    ):
        monkeypatch.delenv(variable, raising=False)
    monkeypatch.setenv(name, value)
    calls: list[list[str]] = []
    output = _docker_output(local_build, revision)

    def recording_output(command, **kwargs):
        calls.append(list(command))
        return output(command, **kwargs)

    monkeypatch.setattr(local_build, "_check_output", recording_output)

    result = local_build.check_build(tmp_path, _host())

    assert result["ok"] is False
    assert any(reason in item for item in result["reasons"])
    assert not any(command[1:3] == ["image", "inspect"] for command in calls)


def test_mismatching_default_platform_fails_before_build(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import orchestration.local_build as local_build

    revision = _committed_fixture(tmp_path)
    monkeypatch.setenv("DOCKER_DEFAULT_PLATFORM", "linux/amd64")
    monkeypatch.setattr(
        local_build,
        "_check_output",
        _docker_output(local_build, revision),
    )
    build_calls: list[list[str]] = []
    monkeypatch.setattr(
        local_build,
        "_run",
        lambda command, **_kwargs: build_calls.append(list(command)),
    )

    result = local_build.check_build(tmp_path, _host("linux/arm64"))
    with pytest.raises(RuntimeError, match="DOCKER_DEFAULT_PLATFORM"):
        local_build.build_images(tmp_path, _host("linux/arm64"), _Workstation())

    assert result["ok"] is False
    assert build_calls == []


def test_source_mutation_mid_build_prevents_record_publication(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import orchestration.local_build as local_build

    revision = _committed_fixture(tmp_path)
    monkeypatch.delenv("DOCKER_DEFAULT_PLATFORM", raising=False)
    monkeypatch.setattr(
        local_build,
        "_check_output",
        _docker_output(local_build, revision),
    )
    calls = 0

    def mutate_after_first_build(_command, **_kwargs):
        nonlocal calls
        calls += 1
        if calls == 1:
            _write(tmp_path, "scripts/local-build.sh", b"mutated during build\n")

    monkeypatch.setattr(local_build, "_run", mutate_after_first_build)

    with pytest.raises(RuntimeError, match="changed during the build"):
        local_build.build_images(tmp_path, _host(), _Workstation())

    assert not (tmp_path / ".local-runtime/build.json").exists()


@pytest.mark.parametrize(
    ("failure", "reason"),
    [
        ("wrong_architecture", "local image has platform"),
        ("stale_companion", "companion image source revision label"),
        ("missing", "local image is missing or unreadable"),
        ("replaced", "local image identities changed"),
    ],
)
def test_check_build_rejects_invalid_actual_image_set(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure: str,
    reason: str,
) -> None:
    import orchestration.local_build as local_build

    revision = _committed_fixture(tmp_path)
    _write_build_record(tmp_path, local_build, revision, "linux/arm64")
    target = local_build.IMAGE_REFS[3]
    options = {
        "wrong_image_architecture": target if failure == "wrong_architecture" else None,
        "companion_revision": "0" * 40 if failure == "stale_companion" else None,
        "missing_image": target if failure == "missing" else None,
        "replaced_image": target if failure == "replaced" else None,
    }
    monkeypatch.delenv("DOCKER_DEFAULT_PLATFORM", raising=False)
    monkeypatch.setattr(
        local_build,
        "_check_output",
        _docker_output(local_build, revision, **options),
    )

    result = local_build.check_build(tmp_path, _host())

    assert result["ok"] is False
    assert any(reason in item for item in result["reasons"])


def test_matching_arm_record_is_checked_without_relabeling(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import orchestration.local_build as local_build

    revision = _committed_fixture(tmp_path)
    record = _write_build_record(tmp_path, local_build, revision, "linux/arm64")
    before = record.read_bytes()
    monkeypatch.delenv("DOCKER_DEFAULT_PLATFORM", raising=False)
    monkeypatch.setattr(
        local_build,
        "_check_output",
        _docker_output(local_build, revision),
    )

    assert local_build.check_build(tmp_path, _host())["ok"] is True
    assert record.read_bytes() == before


def test_standalone_builder_loses_real_process_contention_before_docker(
    tmp_path: Path,
) -> None:
    from contextlib import contextmanager
    from types import SimpleNamespace

    from orchestration.workstation import Workstation

    lock_path = tmp_path / "workstation.lock"

    class Host:
        facts = SimpleNamespace(
            kind="linux",
            machine="arm64",
            image_platform="linux/arm64",
            provider="docker",
        )
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
            "  facts=SimpleNamespace(kind='linux', machine='arm64', "
            "image_platform='linux/arm64', provider='docker')\n"
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


def test_standalone_build_blocks_prepare_direct_suite_and_cleanup_entrypoints(
    tmp_path: Path,
) -> None:
    from orchestration._adapters.suite import SuiteRuntime
    from orchestration.controller import RunController
    from orchestration.operator_session import OperatorSession
    from orchestration.workstation import Workstation

    class Host:
        facts = SimpleNamespace(
            kind="linux",
            machine="arm64",
            image_platform="linux/arm64",
            provider="docker",
        )
        state_directory = tmp_path / "state"

        @contextmanager
        def acquire_native_lease(self):
            yield object()

        def preflight(self, _paths):
            return ()

    host = Host()
    with _standalone_builder_holding_lease(tmp_path) as lock_path:
        workstation = Workstation(host, lock_path=lock_path)

        prepare = OperatorSession(
            Path(__file__).resolve().parents[2],
            state_directory=tmp_path / "prepare-state",
            runtime=SimpleNamespace(
                checks=lambda: pytest.fail("prepare reached runtime checks")
            ),
            controller=SimpleNamespace(),
            host=host,
            workstation=workstation,
        )
        prepare.prepare(
            preset_id="configured-descent",
            request_id="00000000-0000-4000-8000-000000000701",
        )
        assert prepare._worker is not None
        prepare._worker.join(5)
        assert "already owns" in prepare.snapshot()["error"]

        with pytest.raises(RuntimeError, match="already owns"):
            RunController(
                project_directory=Path(__file__).resolve().parents[2],
                workstation=workstation,
            ).start(tmp_path / "unread-direct-template.json")

        suite = SuiteRuntime(
            project_directory=tmp_path,
            controller=SimpleNamespace(workstation=workstation),
            workstation=workstation,
        )
        with pytest.raises(RuntimeError, match="already owns"):
            with suite.lock():
                pytest.fail("suite acquired the standalone builder lease")

        cleanup = OperatorSession(
            Path(__file__).resolve().parents[2],
            state_directory=tmp_path / "cleanup-state",
            runtime=SimpleNamespace(),
            controller=SimpleNamespace(),
            host=host,
            workstation=workstation,
        )
        cleanup._record.update(
            state="ORPHANED",
            run_id="00000000-0000-4000-8000-000000000702",
            cleanup="incomplete",
        )
        cleanup.stop(run_id="00000000-0000-4000-8000-000000000702")
        assert cleanup._worker is not None
        cleanup._worker.join(5)
        assert cleanup.snapshot()["state"] == "ORPHANED"
        assert "already owns" in cleanup.snapshot()["error"]


def test_initial_build_enrolls_and_reacquires_without_consuming_action_token(
    tmp_path: Path, monkeypatch
) -> None:
    import orchestration.local_build as local_build
    from orchestration.host_integration import ControllerRegistration
    from orchestration.workstation import Workstation

    events: list[bool] = []

    class Lease:
        def register_controller(self, _identity, *, pristine):
            events.append(pristine)
            return ControllerRegistration(None, False)

    class Host:
        facts = SimpleNamespace(
            kind="wsl2",
            machine="amd64",
            image_platform="linux/amd64",
            provider="docker-desktop",
        )
        state_directory = tmp_path / "native-state"

        @contextmanager
        def acquire_native_lease(self):
            yield Lease()

    host = Host()
    workstation = Workstation(host, lock_path=tmp_path / "lock")
    monkeypatch.setattr(
        local_build,
        "_revision",
        lambda _root: (_ for _ in ()).throw(RuntimeError("stop after enrollment")),
    )
    monkeypatch.setattr(
        local_build,
        "_docker_state",
        lambda _host: local_build._DockerState(
            "default", "unix:///var/run/docker.sock", "linux/amd64"
        ),
    )

    for _ in range(2):
        with pytest.raises(RuntimeError, match="stop after enrollment"):
            local_build.build_images(tmp_path, host, workstation)

    assert events == [True, False]
    assert json.loads(
        (host.state_directory / "attempt-ledger.json").read_text()
    ) == {"schema_version": 1, "last_allocated_token": 0}
