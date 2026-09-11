"""Executable Phase 2 acceptance gate for terminal synthetic run bundles."""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
from typing import Any

import jsonschema
import pytest

from artifacts import StructuredEvent, validate_regular_file, validate_tree
from artifacts.manifest import MODULE_LOGS, REQUIRED_ARTIFACT_PATHS, REQUIRED_DIRECTORY_PATHS


ROOT = Path(__file__).resolve().parents[2]
INSPECTOR = ROOT / "tests/phase2/inspect_bundle.py"
ARTIFACTS_IMAGE = "drone-sim-artifacts-runtime:phase2"
MODULES = (
    "orchestration",
    "artifacts",
    "companion",
    "ardupilot_sitl",
    "gazebo",
    "electromagnet",
    "scorekeeper",
)
SERVICES = (
    "orchestration-runtime",
    "artifacts-runtime",
    "synthetic-companion",
    "synthetic-ardupilot-sitl",
    "synthetic-gazebo",
    "synthetic-electromagnet",
    "synthetic-scorekeeper",
)
EXACT_COUNTS = {
    "/clock": 41,
    "/simulation/run_state": 4,
    "/simulation/artifact_status": 2,
    "/simulation/ground_truth": 40,
    "/simulation/scenario_events": 1,
    "/simulation/score_events": 1,
    "/camera/onboard/frame_metadata": 40,
    "/camera/observer/frame_metadata": 40,
}
COMPLETED_PARTIALS = {
    "logs/docker/ffmpeg-onboard.log.partial",
    "logs/docker/ffmpeg-observer.log.partial",
    "logs/docker/rosbag2.log.partial",
}
RECOVERY_PARTIALS = {
    "video/onboard.mp4.partial",
    "video/observer.mp4.partial",
}


def _compose_environment() -> dict[str, str]:
    environment = os.environ.copy()
    for name in (
        "COMPOSE_FILE", "COMPOSE_ENV_FILES", "COMPOSE_PATH_SEPARATOR",
        "COMPOSE_PROFILES", "COMPOSE_PROJECT_NAME", "COMPOSE_PROJECT_DIR",
        "COMPOSE_PROJECT_DIRECTORY", "COMPOSE_DISABLE_ENV_FILE",
    ):
        environment.pop(name, None)
    environment["COMPOSE_DISABLE_ENV_FILE"] = "1"
    return environment


def _run(
    command: list[str],
    *,
    env: dict[str, str] | None = None,
    timeout: float = 180,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
        timeout=timeout,
    )


@pytest.fixture(scope="session", autouse=True)
def stable_phase2_images() -> None:
    if os.environ.get("DRONE_SIM_PHASE2_IMAGES_BUILT") == "1":
        return
    built = _run(
        [
            "docker", "compose", "--file", "compose.yaml",
            "--project-directory", str(ROOT), "--profile", "phase2", "build",
        ],
        env=_compose_environment(),
        timeout=900,
    )
    assert built.returncode == 0, built.stdout + built.stderr


@pytest.fixture(scope="module")
def output_root(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return tmp_path_factory.mktemp("phase2-output").resolve()


def _template(directory: Path, output_root: Path) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "run-template.json"
    path.write_text(
        json.dumps(
            {
                "world": "synthetic-phase2-fixture",
                "vehicle": "synthetic-vehicle",
                "mission": "synthetic-lifecycle-only",
                "scenario": "synthetic-artifact-acceptance",
                "output_root": str(output_root),
                "max_wall_seconds": 90,
                "startup_wall_seconds": 45,
                "finalization_wall_seconds": 45,
                "recording": {
                    "width_px": 320,
                    "height_px": 240,
                    "fps": 20,
                    "encoding": "rgb8",
                },
            },
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return path


def _project(run_id: str) -> str:
    return f"drone-sim-{run_id.replace('-', '')}"


def _new_bundle(output_root: Path, before: set[Path]) -> Path:
    candidates = {
        path for path in output_root.iterdir() if path.is_dir() and path not in before
    }
    assert len(candidates) == 1, sorted(str(path) for path in candidates)
    return candidates.pop()


def _parse_start(result: subprocess.CompletedProcess[str]) -> dict[str, Any]:
    lines = [json.loads(line) for line in result.stdout.splitlines() if line.strip()]
    assert lines
    assert lines[-1].get("result_type") == "run_result"
    assert sum(item.get("result_type") == "run_result" for item in lines) == 1
    for item in lines[:-1]:
        assert set(item) == {
            "run_id",
            "module",
            "severity",
            "event",
            "sim_timestamp",
            "wall_timestamp",
            "fields",
        }
        StructuredEvent(
            run_id=item["run_id"],
            module=item["module"],
            severity=item["severity"],
            event=item["event"],
            sim_timestamp=item["sim_timestamp"],
            wall_timestamp=datetime.fromisoformat(item["wall_timestamp"].replace("Z", "+00:00")),
            fields=item["fields"],
        )
    return lines[-1]


def _parse_single(result: subprocess.CompletedProcess[str]) -> dict[str, Any]:
    lines = [line for line in result.stdout.splitlines() if line.strip()]
    assert len(lines) == 1, result.stdout
    value = json.loads(lines[0])
    assert value["result_type"] == "run_result"
    return value


def _start(
    tmp_path: Path,
    output_root: Path,
    *,
    wall_delay_ms: int,
    fault: str = "",
) -> tuple[subprocess.CompletedProcess[str], Path, dict[str, Any]]:
    before = {path for path in output_root.iterdir() if path.is_dir()}
    environment = os.environ.copy()
    environment["SIM_SYNTHETIC_WALL_DELAY_MS"] = str(wall_delay_ms)
    environment["SIM_PHASE2_FAULT"] = fault
    result = _run(
        ["uv", "run", "drone-sim", "start", "--config", str(_template(tmp_path, output_root))],
        env=environment,
    )
    bundle = _new_bundle(output_root, before)
    return result, bundle, _parse_start(result)


def _cleanup_project(run_id: str) -> None:
    _run(
        [
            "docker",
            "compose",
            "--file",
            "compose.yaml",
            "--project-directory",
            str(ROOT),
            "--profile",
            "phase2",
            "-p",
            _project(run_id),
            "down",
            "--remove-orphans",
        ],
        env=_compose_environment(),
        timeout=60,
    )


def _assert_clean(run_id: str) -> None:
    project = _project(run_id)
    containers = _run(
        [
            "docker",
            "ps",
            "--all",
            "--quiet",
            "--filter",
            f"label=com.docker.compose.project={project}",
        ]
    )
    assert containers.returncode == 0
    assert containers.stdout.strip() == ""
    network = _run(["docker", "network", "inspect", f"{project}_default"])
    assert network.returncode != 0


def _inspect(bundle: Path) -> dict[str, Any]:
    result = _run(
        [
            "docker",
            "run",
            "--rm",
            "--network",
            "none",
            "--mount",
            f"type=bind,src={bundle},dst=/bundle,readonly",
            "--mount",
            f"type=bind,src={INSPECTOR},dst=/inspect_bundle.py,readonly",
            ARTIFACTS_IMAGE,
            "python3",
            "/inspect_bundle.py",
            "/bundle",
        ],
        timeout=180,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert len(result.stdout.splitlines()) == 1
    return json.loads(result.stdout)


def _manifest(bundle: Path) -> dict[str, Any]:
    value = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
    schema = json.loads((ROOT / "artifacts/schemas/manifest.schema.json").read_text())
    jsonschema.Draft202012Validator(schema).validate(value)
    return value


def _assert_manifest_hashes(bundle: Path, manifest: dict[str, Any]) -> None:
    for item in manifest["artifacts"]:
        if item["validation"] != "valid":
            assert item["size_bytes"] is None and item["sha256"] is None
            continue
        relative = item["relative_path"]
        result = (
            validate_tree(bundle, relative)
            if (bundle / relative).is_dir()
            else validate_regular_file(bundle, relative)
        )
        assert result.status.value == "valid"
        assert (result.size_bytes, result.sha256) == (item["size_bytes"], item["sha256"])


def _assert_logs(bundle: Path, run_id: str) -> None:
    for module in MODULES:
        path = bundle / f"logs/{module}.jsonl"
        lines = path.read_text(encoding="utf-8").splitlines()
        assert lines
        for line in lines:
            item = json.loads(line)
            assert item["run_id"] == run_id
            assert item["module"] == module
            StructuredEvent(
                run_id=item["run_id"],
                module=item["module"],
                severity=item["severity"],
                event=item["event"],
                sim_timestamp=item["sim_timestamp"],
                wall_timestamp=datetime.fromisoformat(item["wall_timestamp"].replace("Z", "+00:00")),
                fields=item["fields"],
            )
    assert tuple(f"logs/{module}.jsonl" for module in MODULES) == MODULE_LOGS
    for service in SERVICES:
        raw = bundle / f"logs/docker/{service}.log"
        assert raw.is_file()
        assert raw.stat().st_size > 0


def _assert_inventory(bundle: Path, manifest: dict[str, Any]) -> None:
    paths = [item["relative_path"] for item in manifest["artifacts"]]
    assert len(paths) == len(set(paths))
    assert not any(path.startswith(".control/") or path.startswith(".status/") for path in paths)
    partials = {
        path.relative_to(bundle).as_posix()
        for path in bundle.rglob("*.partial")
        if path.is_file()
        and path.relative_to(bundle).parts[0] not in {".control", ".status"}
    }
    assert partials <= set(paths)
    terminal_status = manifest["terminal_status"]
    if terminal_status == "COMPLETED":
        assert partials == COMPLETED_PARTIALS
    else:
        assert terminal_status in {"FAILED", "ABORTED"}
        assert COMPLETED_PARTIALS <= partials
        assert partials <= COMPLETED_PARTIALS | RECOVERY_PARTIALS
    assert not any(path.name.startswith(".manifest.json.") for path in bundle.iterdir())
    assert not any(
        path.endswith(".partial") and path not in partials
        for path in paths
    )
    directory_records = {
        path for path in paths if path in REQUIRED_DIRECTORY_PATHS or path == "configuration"
    }
    for candidate in bundle.rglob("*"):
        if not candidate.is_file():
            continue
        relative = candidate.relative_to(bundle).as_posix()
        if relative == "manifest.json" or relative.split("/", 1)[0] in {
            ".control",
            ".status",
        }:
            continue
        assert relative in paths or any(
            relative.startswith(f"{directory}/") for directory in directory_records
        ), relative


def _assert_provenance(bundle: Path, manifest: dict[str, Any]) -> None:
    assert manifest["source_revisions"]
    assert len(manifest["image_digests"]) == 7
    assert {item["name"] for item in manifest["image_digests"]} == {
        "drone-sim-orchestration-runtime:phase2",
        "drone-sim-artifacts-runtime:phase2",
        "drone-sim-synthetic-companion:phase2",
        "drone-sim-synthetic-ardupilot-sitl:phase2",
        "drone-sim-synthetic-gazebo:phase2",
        "drone-sim-synthetic-electromagnet:phase2",
        "drone-sim-synthetic-scorekeeper:phase2",
    }
    resolved = (bundle / "configuration/run.json").read_bytes()
    resolved_document = json.loads(resolved)
    resolved_schema = json.loads((ROOT / "config/run.schema.json").read_text())
    jsonschema.Draft202012Validator(resolved_schema).validate(resolved_document)
    checksum = resolved_document.pop("config_sha256")
    canonical = json.dumps(resolved_document, sort_keys=True, separators=(",", ":")).encode()
    assert hashlib.sha256(canonical).hexdigest() == checksum
    assert manifest["configurations"] == [
        {"relative_path": "configuration/run.json", "sha256": checksum}
    ]


def _image_digest_mapping(manifest: dict[str, Any]) -> dict[str, str]:
    return {
        item["name"]: item["digest"]
        for item in manifest["image_digests"]
    }


def _finalization_intervals(bundle: Path) -> dict[str, float]:
    host_events = [
        json.loads(line)
        for line in (bundle / "logs/orchestration.jsonl").read_text().splitlines()
    ]
    timestamps = {
        item["event"]: datetime.fromisoformat(
            item["wall_timestamp"].replace("Z", "+00:00")
        )
        for item in host_events
        if item["event"] in {"run_finalizing", "log_capture_starting"}
    }
    assert set(timestamps) == {"run_finalizing", "log_capture_starting"}
    manifest_at = datetime.fromtimestamp(
        (bundle / "manifest.json").stat().st_mtime_ns / 1_000_000_000,
        tz=timezone.utc,
    )
    finalizing_at = timestamps["run_finalizing"]
    capture_at = timestamps["log_capture_starting"]
    intervals = {
        "FINALIZING_to_capture": (capture_at - finalizing_at).total_seconds(),
        "capture_to_manifest": (manifest_at - capture_at).total_seconds(),
        "FINALIZING_to_manifest": (manifest_at - finalizing_at).total_seconds(),
    }
    assert all(value >= 0 for value in intervals.values())
    return intervals


def _canonical_terminal_result(bundle: Path) -> dict[str, Any]:
    manifest = _manifest(bundle)
    return {
        "result_type": "run_result",
        "run_id": manifest["run_id"],
        "state": manifest["terminal_status"],
        "reason": manifest["reason"],
        "manifest_path": "manifest.json",
    }


def _assert_collect_read_only(bundle: Path, *, include_abort: bool = False) -> None:
    def snapshot() -> dict[str, tuple[int, int, bytes]]:
        return {
            path.relative_to(bundle).as_posix(): (
                path.stat().st_mtime_ns,
                path.stat().st_size,
                path.read_bytes(),
            )
            for path in bundle.rglob("*")
            if path.is_file()
            and path.relative_to(bundle).parts[0] not in {".control", ".status"}
        }

    before = snapshot()
    commands = []
    if include_abort:
        commands.append(["abort", bundle.name])
    commands.extend(
        [
            ["collect-results", bundle.name],
            ["collect-results", bundle.name],
            ["status", bundle.name],
        ]
    )
    expected = _canonical_terminal_result(bundle)
    for command in commands:
        result = _run(
            [
                "uv",
                "run",
                "drone-sim",
                *command,
                "--output-root",
                str(bundle.parent),
            ]
        )
        assert result.returncode == 0, result.stderr
        assert _parse_single(result) == expected
    assert snapshot() == before


def _assert_video_semantics(video: dict[str, Any], required_record: dict[str, Any]) -> None:
    frame_count = video["recorder_frame_count"]
    assert frame_count is None or (
        isinstance(frame_count, int)
        and not isinstance(frame_count, bool)
        and frame_count >= 0
    )
    if frame_count:
        assert video["probe_ok"] is True
        assert video["decode_ok"] is True
        assert len(video["streams"]) == 1
        assert video["streams"][0] == {
            "codec_type": "video",
            "codec_name": "h264",
            "pix_fmt": "yuv420p",
            "width": 320,
            "height": 240,
            "avg_frame_rate": "20/1",
            "nb_read_frames": str(frame_count),
        }
        assert len(video["decoded_frame_hashes"]) == frame_count
        assert video["validator"] == {
            "status": "valid",
            "detail": "valid H.264 yuv420p 20-FPS video with full decode",
            "frame_count": frame_count,
        }
        return
    assert required_record["validation"] in {"missing", "invalid"}
    assert video["recorder_status"] in {"missing", "invalid"}
    assert video["validator"]["status"] in {"missing", "invalid"}


def _assert_completed_bundle(bundle: Path) -> dict[str, Any]:
    manifest = _manifest(bundle)
    assert manifest["terminal_status"] == "COMPLETED"
    assert manifest["incomplete_paths"] == []
    assert all(item["validation"] == "valid" for item in manifest["artifacts"])
    assert manifest["scoring"] == {
        "achieved_score": 0.0,
        "maximum_available_score": 0.0,
        "scoring_checksum": hashlib.sha256(
            (ROOT / "tests/phase2/scoring.json").read_bytes()
        ).hexdigest(),
        "evidence_paths": ["scoring/events.jsonl"],
    }
    scoring = json.loads((bundle / "scoring/result.json").read_text())
    assert scoring["fixture"] is True
    assert scoring["scoring_checksum"] == manifest["scoring"]["scoring_checksum"]
    _assert_manifest_hashes(bundle, manifest)
    _assert_logs(bundle, manifest["run_id"])
    _assert_inventory(bundle, manifest)
    _assert_provenance(bundle, manifest)

    inspected = _inspect(bundle)
    records = {item["relative_path"]: item for item in manifest["artifacts"]}
    for stream, video in inspected["videos"].items():
        assert video["recorder_frame_count"] == 40
        _assert_video_semantics(video, records[f"video/{stream}.mp4"])

    bag = inspected["bag"]
    assert bag["structurally_readable"] is True
    assert bag["storage_id"] == "mcap"
    assert bag["semantic"]["status"] == "valid"
    assert {item["name"]: item["decoded_count"] for item in bag["topics"]} == EXACT_COUNTS
    assert bag["lifecycle"] == ["STARTING", "READY", "RUNNING", "FINALIZING"]
    assert [
        {key: value for key, value in item.items() if key != "record_index"}
        for item in bag["artifact_statuses"]
    ] == [
        {
            "sim_timestamp_ns": 0,
            "ready": False,
            "complete": False,
            "missing": ["onboard", "observer", "rosbag"],
            "manifest_path": "",
        },
        {
            "sim_timestamp_ns": 0,
            "ready": True,
            "complete": False,
            "missing": [],
            "manifest_path": "",
        },
    ]
    assert [item["record_index"] for item in bag["artifact_statuses"]] == sorted(
        item["record_index"] for item in bag["artifact_statuses"]
    )
    assert all(
        item["record_index"] < bag["first_clock_record_index"]
        for item in bag["artifact_statuses"]
    )
    assert bag["artifact_ready_record_index"] < bag["first_clock_record_index"]
    assert len(bag["scenario_events"]) == len(bag["score_events"]) == 1
    expected_ids = list(range(40))
    expected_stamps = list(range(50_000_000, 2_000_000_001, 50_000_000))
    by_topic = {item["name"]: item for item in bag["topics"]}
    for stream in ("onboard", "observer"):
        metadata = f"/camera/{stream}/frame_metadata"
        assert bag["frame_ids"][metadata] == expected_ids
        assert by_topic[metadata]["sim_timestamps_ns"] == expected_stamps
    return inspected


def _normalized(inspected: dict[str, Any]) -> dict[str, Any]:
    bag = inspected["bag"]
    return {
        "topic_stamps": {
            item["name"]: item["sim_timestamps_ns"] for item in bag["topics"]
        },
        "frame_ids": bag["frame_ids"],
        "image_payload_hashes": bag["image_payload_hashes"],
        "scenario_events": bag["scenario_events"],
        "score_events": bag["score_events"],
        "video_frames": {
            stream: value["decoded_frame_hashes"]
            for stream, value in inspected["videos"].items()
        },
    }


def test_completed_runs_are_semantically_identical_across_wall_delays(
    tmp_path: Path, output_root: Path
) -> None:
    runs = []
    try:
        for index, delay in enumerate((0, 17)):
            result, bundle, terminal = _start(
                tmp_path / f"completed-{index}", output_root, wall_delay_ms=delay
            )
            runs.append(bundle)
            assert result.returncode == 0, result.stdout + result.stderr
            assert terminal == {
                "result_type": "run_result",
                "run_id": bundle.name,
                "state": "COMPLETED",
                "reason": "mission_complete",
                "manifest_path": "manifest.json",
            }
        inspected = [_assert_completed_bundle(bundle) for bundle in runs]
        assert _normalized(inspected[0]) == _normalized(inspected[1])
        for bundle in runs:
            _assert_collect_read_only(bundle)
            _assert_clean(bundle.name)
    finally:
        for bundle in runs:
            _cleanup_project(bundle.name)


def test_observer_encoder_failure_preserves_readable_recovery_bundle(
    tmp_path: Path, output_root: Path
) -> None:
    bundle: Path | None = None
    try:
        result, bundle, terminal = _start(
            tmp_path / "failed",
            output_root,
            wall_delay_ms=0,
            fault="observer_encoder_after_5",
        )
        assert result.returncode == 1, result.stdout + result.stderr
        assert terminal["state"] == "FAILED"
        manifest = _manifest(bundle)
        assert manifest["terminal_status"] == "FAILED"
        records = {item["relative_path"]: item for item in manifest["artifacts"]}
        assert records["video/observer.mp4"]["validation"] in {"missing", "invalid"}
        assert "video/observer.mp4" in manifest["incomplete_paths"]
        _assert_manifest_hashes(bundle, manifest)
        _assert_logs(bundle, bundle.name)
        _assert_inventory(bundle, manifest)
        _assert_provenance(bundle, manifest)
        inspected = _inspect(bundle)
        assert inspected["bag"]["structurally_readable"] is True
        for stream, video in inspected["videos"].items():
            _assert_video_semantics(video, records[f"video/{stream}.mp4"])
        observer_partials = {
            item["relative_path"]
            for item in manifest["artifacts"]
            if "observer" in item["relative_path"] and item["relative_path"].endswith(".partial")
        }
        assert observer_partials
        assert all((bundle / path).is_file() for path in observer_partials)
        _assert_collect_read_only(bundle)
        _assert_clean(bundle.name)
    finally:
        if bundle is not None:
            _cleanup_project(bundle.name)


def test_abort_is_durable_idempotent_and_preserves_readable_bundle(
    tmp_path: Path, output_root: Path
) -> None:
    before = {path for path in output_root.iterdir() if path.is_dir()}
    environment = os.environ.copy()
    environment["SIM_SYNTHETIC_WALL_DELAY_MS"] = "40"
    environment["SIM_PHASE2_FAULT"] = ""
    process = subprocess.Popen(
        [
            "uv",
            "run",
            "drone-sim",
            "start",
            "--config",
            str(_template(tmp_path / "aborted", output_root)),
        ],
        cwd=ROOT,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    bundle: Path | None = None
    try:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            current = {
                path for path in output_root.iterdir() if path.is_dir() and path not in before
            }
            if len(current) == 1:
                bundle = current.pop()
                status_path = bundle / ".status/operator-state.json"
                if status_path.is_file():
                    try:
                        if json.loads(status_path.read_text())["state"] == "RUNNING":
                            break
                    except (OSError, json.JSONDecodeError, KeyError):
                        pass
            if process.poll() is not None:
                break
            time.sleep(0.05)
        assert bundle is not None
        assert json.loads((bundle / ".status/operator-state.json").read_text())["state"] == "RUNNING"

        abort = _run(
            [
                "uv",
                "run",
                "drone-sim",
                "abort",
                bundle.name,
                "--output-root",
                str(output_root),
            ]
        )
        assert abort.returncode == 0
        assert _parse_single(abort)["run_id"] == bundle.name
        stdout, stderr = process.communicate(timeout=180)
        started = subprocess.CompletedProcess(process.args, process.returncode, stdout, stderr)
        terminal = _parse_start(started)
        assert started.returncode == 130, stdout + stderr
        assert terminal["state"] == "ABORTED"

        manifest_path = bundle / "manifest.json"
        immutable = manifest_path.read_bytes()
        _assert_collect_read_only(bundle, include_abort=True)
        assert manifest_path.read_bytes() == immutable

        manifest = _manifest(bundle)
        assert manifest["terminal_status"] == "ABORTED"
        _assert_manifest_hashes(bundle, manifest)
        _assert_logs(bundle, bundle.name)
        _assert_inventory(bundle, manifest)
        _assert_provenance(bundle, manifest)
        inspected = _inspect(bundle)
        assert inspected["bag"]["structurally_readable"] is True
        records = {item["relative_path"]: item for item in manifest["artifacts"]}
        for stream, video in inspected["videos"].items():
            _assert_video_semantics(video, records[f"video/{stream}.mp4"])
        _assert_clean(bundle.name)
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=10)
        if bundle is not None:
            _cleanup_project(bundle.name)


def test_all_four_terminal_manifests_share_exact_image_mapping_and_timing_evidence(
    output_root: Path,
) -> None:
    bundles = sorted(path.parent for path in output_root.glob("*/manifest.json"))
    assert len(bundles) == 4
    manifests = [_manifest(bundle) for bundle in bundles]
    mappings = [_image_digest_mapping(manifest) for manifest in manifests]

    assert len(mappings[0]) == 7
    assert all(mapping == mappings[0] for mapping in mappings[1:])
    assert sorted(manifest["terminal_status"] for manifest in manifests) == [
        "ABORTED",
        "COMPLETED",
        "COMPLETED",
        "FAILED",
    ]
    for bundle in bundles:
        intervals = _finalization_intervals(bundle)
        assert intervals["FINALIZING_to_manifest"] == pytest.approx(
            intervals["FINALIZING_to_capture"] + intervals["capture_to_manifest"],
            abs=1e-6,
        )
        notification = bundle / ".status/terminal-notified.json"
        assert json.loads(notification.read_text()) == {
            "run_id": bundle.name,
            "notified": True,
        }
        assert notification.stat().st_mtime_ns >= (bundle / "manifest.json").stat().st_mtime_ns


@pytest.mark.parametrize(
    "relative_path",
    [
        "logs/docker/orchestration-runtime.log.partial",
        "logs/orchestration.jsonl.partial",
        "logs/orchestration-host.jsonl.partial",
        "logs/docker/unknown-recorder.log.partial",
        "unknown.partial",
    ],
    ids=(
        "raw-log-publication",
        "structured-log-publication",
        "host-source-after-capture",
        "unknown-recorder",
        "unknown",
    ),
)
def test_terminal_inventory_rejects_non_recorder_partial_candidates(
    tmp_path: Path, relative_path: str
) -> None:
    bundle = tmp_path / "bundle"
    candidate = bundle / relative_path
    candidate.parent.mkdir(parents=True, exist_ok=True)
    candidate.write_bytes(b"leftover")
    manifest = {
        "terminal_status": "FAILED",
        "artifacts": [{"relative_path": relative_path}],
    }

    with pytest.raises(AssertionError):
        _assert_inventory(bundle, manifest)


def test_terminal_inventory_rejects_manifest_publication_candidate(tmp_path: Path) -> None:
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    artifacts = []
    for relative_path in COMPLETED_PARTIALS:
        candidate = bundle / relative_path
        candidate.parent.mkdir(parents=True, exist_ok=True)
        candidate.write_bytes(b"recorder diagnostic")
        artifacts.append({"relative_path": relative_path})
    (bundle / ".manifest.json.fixed.tmp").write_bytes(b"candidate")

    with pytest.raises(AssertionError):
        _assert_inventory(
            bundle, {"terminal_status": "ABORTED", "artifacts": artifacts}
        )


@pytest.mark.parametrize("terminal_status", ["FAILED", "ABORTED"])
def test_terminal_inventory_allows_only_inventoried_recorder_recovery_partials(
    tmp_path: Path, terminal_status: str
) -> None:
    bundle = tmp_path / "bundle"
    artifacts = []
    for relative_path in COMPLETED_PARTIALS | {"video/observer.mp4.partial"}:
        candidate = bundle / relative_path
        candidate.parent.mkdir(parents=True, exist_ok=True)
        candidate.write_bytes(b"frozen recorder output")
        artifacts.append({"relative_path": relative_path})

    _assert_inventory(
        bundle,
        {"terminal_status": terminal_status, "artifacts": artifacts},
    )


def test_positive_recorder_frame_count_requires_full_video_semantics() -> None:
    corrupt_positive = {
        "recorder_frame_count": 5,
        "probe_ok": False,
        "decode_ok": True,
        "streams": [],
        "decoded_frame_hashes": [],
        "validator": {"status": "invalid", "frame_count": None},
    }
    record = {"validation": "invalid"}

    with pytest.raises(AssertionError):
        _assert_video_semantics(corrupt_positive, record)


def test_zero_recorder_frame_count_requires_explicit_invalid_or_missing_record() -> None:
    zero_frame = {
        "recorder_frame_count": 0,
        "recorder_status": "invalid",
        "probe_ok": False,
        "decode_ok": False,
        "streams": [],
        "decoded_frame_hashes": [],
        "validator": {"status": "invalid", "frame_count": None},
    }

    with pytest.raises(AssertionError):
        _assert_video_semantics(zero_frame, {"validation": "valid"})
    _assert_video_semantics(zero_frame, {"validation": "invalid"})
