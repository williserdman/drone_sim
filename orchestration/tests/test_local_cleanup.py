from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from artifacts.runtime_protocol import RuntimeProtocol
from artifacts.runtime_status import MediaSpoolPublishedStatus
from orchestration._adapters.compose import ComposeRuntime
from orchestration.config import PHASE3_TOPOLOGY
from orchestration.local_runtime import LocalRuntime


RUN_ID = "11111111-1111-4111-8111-111111111111"
PROJECT_NAME = "drone-sim-11111111111141118111111111111111"
VOLUME_NAME = f"{PROJECT_NAME}_artifacts-media"


def _runtime(tmp_path: Path) -> LocalRuntime:
    host = SimpleNamespace(state_directory=tmp_path / "qgc-state")
    return LocalRuntime(tmp_path, host=host)


def _compose(tmp_path: Path, calls: list[list[str]], *, overlay: str) -> ComposeRuntime:
    run_directory = (tmp_path / "runs" / RUN_ID).resolve()
    (run_directory / "configuration").mkdir(parents=True)
    (run_directory / ".status").mkdir()

    def runner(command, **_kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout=b"")

    return ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=run_directory,
        config_path=run_directory / "configuration/run.json",
        topology=PHASE3_TOPOLOGY,
        runner=runner,
        base_environment={"SIM_COMPOSE_OVERLAY": overlay} if overlay else {},
    )


def test_local_down_removes_named_volume_after_verified_media_publication(
    tmp_path: Path,
) -> None:
    calls: list[list[str]] = []
    compose = _compose(tmp_path, calls, overlay="local")
    with RuntimeProtocol(compose.run_directory, RUN_ID) as protocol:
        protocol.write_status(MediaSpoolPublishedStatus(RUN_ID))

    compose.down(20)

    assert calls[0][-3:] == ["down", "--remove-orphans", "--volumes"]


def test_local_down_preserves_named_volume_without_publication_proof(
    tmp_path: Path,
) -> None:
    calls: list[list[str]] = []
    compose = _compose(tmp_path, calls, overlay="local")

    compose.down(20)

    assert calls[0][-2:] == ["down", "--remove-orphans"]


def test_local_down_preserves_named_volume_when_publication_proof_is_invalid(
    tmp_path: Path,
) -> None:
    calls: list[list[str]] = []
    compose = _compose(tmp_path, calls, overlay="local")
    marker = compose.run_directory / ".status/media-spool-published.json"
    marker.write_text(
        json.dumps({"run_id": RUN_ID, "closed_media_verified": False}),
        encoding="utf-8",
    )

    compose.down(20)

    assert calls[0][-2:] == ["down", "--remove-orphans"]


def test_nonlocal_down_never_removes_named_volumes(tmp_path: Path) -> None:
    calls: list[list[str]] = []
    compose = _compose(tmp_path, calls, overlay="")
    with RuntimeProtocol(compose.run_directory, RUN_ID) as protocol:
        protocol.write_status(MediaSpoolPublishedStatus(RUN_ID))

    compose.down(20)

    assert calls[0][-2:] == ["down", "--remove-orphans"]


def test_nonlocal_media_cleanup_is_verified_by_compose_topology(tmp_path: Path) -> None:
    calls: list[list[str]] = []
    compose = _compose(tmp_path, calls, overlay="")

    assert compose.media_cleanup_verified(20) is True
    assert calls == []


def test_local_media_cleanup_requires_absent_exact_owned_volume(tmp_path: Path) -> None:
    calls: list[list[str]] = []
    compose = _compose(tmp_path, calls, overlay="local")

    assert compose.media_cleanup_verified(20) is True
    assert calls == [[
        "docker",
        "volume",
        "ls",
        "--quiet",
        "--filter",
        f"label=com.docker.compose.project={PROJECT_NAME}",
        "--filter",
        "label=com.docker.compose.volume=artifacts-media",
    ]]


def test_local_media_cleanup_fails_for_retained_or_uninspectable_owned_volume(
    tmp_path: Path,
) -> None:
    run_directory = (tmp_path / "runs" / RUN_ID).resolve()
    (run_directory / "configuration").mkdir(parents=True)
    results = iter(
        (
            SimpleNamespace(returncode=0, stdout=(VOLUME_NAME + "\n").encode()),
            SimpleNamespace(returncode=1, stdout=b"inventory failed"),
            SimpleNamespace(returncode=0, stdout=b"\xff"),
        )
    )
    compose = ComposeRuntime(
        project_directory=tmp_path.resolve(),
        run_id=RUN_ID,
        run_directory=run_directory,
        config_path=run_directory / "configuration/run.json",
        topology=PHASE3_TOPOLOGY,
        runner=lambda *_args, **_kwargs: next(results),
        base_environment={"SIM_COMPOSE_OVERLAY": "local"},
    )

    assert compose.media_cleanup_verified(20) is False
    assert compose.media_cleanup_verified(20) is False
    assert compose.media_cleanup_verified(20) is False


def test_remaining_resources_reports_run_owned_containers_and_media_volumes(
    tmp_path: Path,
) -> None:
    runtime = _runtime(tmp_path)
    calls: list[list[str]] = []

    def run(command, **_kwargs):
        calls.append(command)
        if command[1] == "ps":
            return SimpleNamespace(
                returncode=0,
                stdout=json.dumps({"Names": f"{PROJECT_NAME}-gazebo-runtime-1"}) + "\n",
                stderr="",
            )
        return SimpleNamespace(
            returncode=0,
            stdout="\n".join(
                (
                    json.dumps({"Name": VOLUME_NAME}),
                    json.dumps({"Name": "unrelated_artifacts-media"}),
                )
            ),
            stderr="",
        )

    runtime._run = run

    assert runtime.remaining_resources() == [
        {"Names": f"{PROJECT_NAME}-gazebo-runtime-1"},
        {"Names": VOLUME_NAME, "ResourceType": "volume"},
    ]
    assert calls[1] == [
        "docker",
        "volume",
        "ls",
        "--format",
        "{{json .}}",
        "--filter",
        "label=com.docker.compose.project",
        "--filter",
        "label=com.docker.compose.volume=artifacts-media",
    ]


def test_remaining_resources_fails_closed_when_volume_inventory_fails(
    tmp_path: Path,
) -> None:
    runtime = _runtime(tmp_path)
    results = iter(
        (
            SimpleNamespace(returncode=0, stdout="", stderr=""),
            SimpleNamespace(returncode=1, stdout="", stderr="volume query failed"),
        )
    )
    runtime._run = lambda *_args, **_kwargs: next(results)

    try:
        runtime.remaining_resources()
    except RuntimeError as error:
        assert str(error) == "volume query failed"
    else:
        raise AssertionError("failed volume inventory must block cleanup completion")
