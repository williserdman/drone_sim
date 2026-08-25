from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from artifacts import (
    ArtifactSession,
    ConfigurationRecord,
    FinalizationInput,
    ImageDigest,
    SourceRevision,
)
from artifacts.manifest import MODULE_LOGS, REQUIRED_ARTIFACT_PATHS
from artifacts._adapters.rosbag import (
    GroundTruthEvidence,
    PhysicalBagEvidence,
    ScoreEventEvidence,
)
from artifacts.validation import validate_tree


RUN_ID = "00000000-0000-4000-8000-000000000606"
RULES_PATH = Path(__file__).parents[2] / "scorekeeper/rules/descent_v1.json"
PHASE3_IMAGE_NAMES = (
    "drone-sim-orchestration-runtime:phase2",
    "drone-sim-artifacts-runtime:phase2",
    "drone-sim-companion-runtime:phase3",
    "drone-sim-ardupilot-runtime:phase3",
    "drone-sim-gazebo-runtime:phase3",
    "drone-sim-electromagnet-runtime:phase3",
    "drone-sim-scorekeeper-runtime:phase3",
)


def _completed_bundle(
    run_directory: Path,
    *,
    achieved: float = 60.0,
    with_optional_artifact: bool = False,
    log_mutator=None,
) -> None:
    modules = tuple(Path(path).stem for path in MODULE_LOGS)
    for relative_path in REQUIRED_ARTIFACT_PATHS:
        target = run_directory / relative_path
        if relative_path in {"configuration", "gazebo/state", "rosbag"}:
            target.mkdir(parents=True, exist_ok=True)
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(b"artifact")
    (run_directory / "gazebo/state/state.tlog").write_bytes(b"native state")
    (run_directory / "rosbag/data.mcap").write_bytes(b"mcap")

    configuration = {
        "run_id": RUN_ID,
        "world": "competition",
        "vehicle": "iris",
        "mission": "descent",
        "scenario": "maximum_score",
        "output_root": str(run_directory.parent),
        "max_wall_seconds": 30,
        "startup_wall_seconds": 10,
        "finalization_wall_seconds": 10,
        "runtime_profile": "phase3",
        "recording": {
            "width_px": 320,
            "height_px": 240,
            "fps": 20,
            "encoding": "rgb8",
        },
        "simulation": {
            "seed": 9,
            "duration_sim_seconds": 2.0,
            "target_real_time_factor": 0.1,
        },
    }
    config_sha = hashlib.sha256(
        json.dumps(configuration, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    configuration["config_sha256"] = config_sha
    config_path = run_directory / "configuration/run.json"
    config_path.write_text(json.dumps(configuration), encoding="utf-8")

    def log_row(module, event, fields, sim_timestamp=None):
        return {
            "run_id": RUN_ID,
            "module": module,
            "severity": "INFO",
            "event": event,
            "sim_timestamp": sim_timestamp,
            "wall_timestamp": "2026-08-24T00:00:00Z",
            "fields": fields,
        }

    companion_events = [
        ("heartbeat_observed", {}),
        *(("command_issued", {"command": command}) for command in (
            "SET_GUIDED", "ARM", "TAKEOFF", "LAND"
        )),
        *(("command_acknowledged", {"command": command}) for command in (
            "SET_GUIDED", "ARM", "TAKEOFF", "LAND"
        )),
        ("descent_observed", {}),
        ("touchdown_observed", {}),
        ("vehicle_disarmed", {}),
        ("mission_landed", {}),
        ("mission_finished", {"outcome": "LANDED"}),
    ]
    gazebo_actions = (
        "PublishGazeboReady",
        "RequestSteps",
        "SetPaused",
        "WriteSourceFinished",
        "BeginFinalization",
        "StopServer",
        "WriteQuiescence",
    )
    documents_by_module = {
        "companion": [
            log_row("companion", event, fields, float(index) / 10)
            for index, (event, fields) in enumerate(companion_events)
        ],
        "ardupilot_sitl": [
            log_row("ardupilot_sitl", "starting", {}),
            log_row(
                "ardupilot_sitl",
                "ready",
                {"json_exchange": True, "mavlink_listening": True},
            ),
            log_row("ardupilot_sitl", "stopped", {"return_code": 0}),
        ],
        "gazebo": [
            log_row("gazebo", "runtime_started", {"partition": RUN_ID}),
            *(
                log_row("gazebo", "runtime_action", {"action": action})
                for action in gazebo_actions
            ),
            log_row("gazebo", "runtime_quiescent", {}),
        ],
    }
    for module, relative_path in zip(modules, MODULE_LOGS, strict=True):
        documents = documents_by_module.get(
            module, [log_row(module, "captured", {})]
        )
        (run_directory / relative_path).write_text(
            "".join(json.dumps(document) + "\n" for document in documents),
            encoding="utf-8",
        )
    if log_mutator is not None:
        log_mutator(run_directory)
    if with_optional_artifact:
        optional = run_directory / "logs/docker/rosbag.log"
        optional.parent.mkdir(parents=True, exist_ok=True)
        optional.write_text("recorder diagnostic\n", encoding="utf-8")

    rule_ids = (
        "airborne_then_contact",
        "touchdown_precision",
        "safe_preimpact_speed",
        "stable_contact",
    )
    available = (20.0, 40.0, 20.0, 20.0)
    awarded = (20.0, 40.0, 20.0, achieved - 80.0) if achieved >= 80 else (
        20.0,
        40.0,
        0.0,
        0.0,
    )
    events = [
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 2_000_000_000,
            "event_id": index,
            "event_type": f"descent.{rule_id}",
            "value": value,
            "evidence_ref": f"scoring/events.jsonl#event-{index}",
        }
        for index, (rule_id, value) in enumerate(zip(rule_ids, awarded, strict=True))
    ]
    events.append(
        {
            "run_id": RUN_ID,
            "sim_timestamp_ns": 2_000_000_000,
            "event_id": 4,
            "event_type": "score.finalized",
            "value": achieved,
            "evidence_ref": "scoring/events.jsonl#event-4",
        }
    )
    (run_directory / "scoring/events.jsonl").write_text(
        "".join(json.dumps(event) + "\n" for event in events), encoding="utf-8"
    )
    checksum = hashlib.sha256(RULES_PATH.read_bytes()).hexdigest()
    evidence = tuple(
        [*(f"scoring/events.jsonl#event-{index}" for index in range(5)),
         "rosbag#/simulation/ground_truth"]
    )
    (run_directory / "scoring/result.json").write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "ruleset_id": "descent_v1",
                "complete": True,
                "achieved_score": achieved,
                "maximum_available_score": 100.0,
                "scoring_checksum": checksum,
                "evidence_paths": list(evidence),
                "rule_results": [
                    {
                        "rule_id": rule_id,
                        "passed": value > 0,
                        "awarded_points": value,
                        "available_points": points,
                        "evidence_ref": f"rosbag#/simulation/ground_truth:{rule_id}",
                    }
                    for rule_id, value, points in zip(
                        rule_ids, awarded, available, strict=True
                    )
                ],
                "diagnostic": None,
            }
        ),
        encoding="utf-8",
    )
    now = datetime(2026, 8, 24, tzinfo=timezone.utc)
    ArtifactSession(run_directory, physical_gazebo=True).finalize(
        FinalizationInput(
            run_id=RUN_ID,
            requested_terminal="COMPLETED",
            reason="mission_complete",
            sim_start_ns=0,
            sim_end_ns=2_000_000_000,
            wall_started_at=now,
            wall_ended_at=now,
            source_revisions=(SourceRevision("drone_sim", "abc123", False),),
            image_digests=tuple(
                ImageDigest(name, f"{index + 1:064x}")
                for index, name in enumerate(PHASE3_IMAGE_NAMES)
            ),
            configuration_records=(ConfigurationRecord("configuration/run.json", config_sha),),
            achieved_score=achieved,
            maximum_available_score=100.0,
            scoring_checksum=checksum,
            evidence_paths=evidence,
        )
    )


def _physical_evidence(run_directory: Path, *, achieved: float) -> PhysicalBagEvidence:
    if achieved == 100.0:
        first_timestamp = 1_400_000_000
        samples = tuple(
            GroundTruthEvidence(
                sim_timestamp_ns=first_timestamp + index * 50_000_000,
                vehicle_id="iris",
                position_xyz=(0.0, 0.0, 1.0 if index == 0 else 0.0),
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_velocity_xyz=(0.0, 0.0, -0.5 if index == 1 else 0.0),
                angular_velocity_xyz=(0.0, 0.0, 0.0),
                in_contact=index >= 2,
            )
            for index in range(13)
        )
    else:
        samples = tuple(
            GroundTruthEvidence(
                sim_timestamp_ns=1_900_000_000 + index * 50_000_000,
                vehicle_id="iris",
                position_xyz=(0.0, 0.0, 1.0 if index == 0 else 0.0),
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                linear_velocity_xyz=(0.0, 0.0, -2.0 if index == 1 else 0.0),
                angular_velocity_xyz=(0.0, 0.0, 0.0),
                in_contact=index == 2,
            )
            for index in range(3)
        )
    events = [
        json.loads(line)
        for line in (run_directory / "scoring/events.jsonl").read_text().splitlines()
    ]
    bag_sha256 = validate_tree(run_directory, "rosbag").sha256
    assert bag_sha256 is not None
    return PhysicalBagEvidence(
        bag_sha256,
        hashlib.sha256(
            json.dumps(
                {
                    key: value
                    for key, value in json.loads(
                        (run_directory / "configuration/run.json").read_text()
                    ).items()
                    if key != "config_sha256"
                },
                sort_keys=True,
                separators=(",", ":"),
            ).encode()
        ).hexdigest(),
        ("STARTING", "READY", "RUNNING", "FINALIZING"),
        samples,
        tuple(
            ScoreEventEvidence(
                event["sim_timestamp_ns"],
                event["event_id"],
                event["event_type"],
                event["value"],
                event["evidence_ref"],
            )
            for event in events
        ),
    )


def test_acceptance_inspector_accepts_complete_partial_bundle_read_only(tmp_path):
    from artifacts.acceptance import inspect_phase3_bundle

    _completed_bundle(tmp_path)
    before = {
        path.relative_to(tmp_path): path.read_bytes()
        for path in tmp_path.rglob("*") if path.is_file()
    }

    report = inspect_phase3_bundle(
        tmp_path,
        rules_path=RULES_PATH,
        compose_resources=lambda _project: (),
        semantic_check=lambda *_args: _physical_evidence(tmp_path, achieved=60.0),
    )

    after = {
        path.relative_to(tmp_path): path.read_bytes()
        for path in tmp_path.rglob("*") if path.is_file()
    }
    assert report.run_id == RUN_ID
    assert report.achieved_score == 60.0
    assert report.compose_project.endswith(RUN_ID.replace("-", ""))
    assert before == after


def test_acceptance_inspector_optionally_requires_maximum_score(tmp_path):
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    _completed_bundle(tmp_path)

    with pytest.raises(BundleAcceptanceError, match="100/100"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            require_maximum_score=True,
            compose_resources=lambda _project: (),
            semantic_check=lambda *_args: _physical_evidence(tmp_path, achieved=60.0),
        )


def test_acceptance_inspector_rejects_leftover_compose_resources(tmp_path):
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    _completed_bundle(tmp_path)

    with pytest.raises(BundleAcceptanceError, match="Compose resources"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            compose_resources=lambda _project: ("container:deadbeef",),
            semantic_check=lambda *_args: _physical_evidence(tmp_path, achieved=60.0),
        )


def test_acceptance_inspector_accepts_maximum_score_when_required(tmp_path):
    from artifacts.acceptance import inspect_phase3_bundle

    _completed_bundle(tmp_path, achieved=100.0)

    report = inspect_phase3_bundle(
        tmp_path,
        rules_path=RULES_PATH,
        require_maximum_score=True,
        compose_resources=lambda _project: (),
        semantic_check=lambda *_args: _physical_evidence(tmp_path, achieved=100.0),
    )

    assert report.achieved_score == 100.0


def test_acceptance_rejects_completed_manifest_with_wrong_simulation_timing(tmp_path):
    """A completed bundle must bind manifest time to the configured public epoch."""
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    _completed_bundle(tmp_path)
    manifest_path = tmp_path / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["simulation_timing"] = {
        "start_ns": 50_000_000,
        "end_ns": 2_050_000_000,
        "duration_ns": 2_000_000_000,
    }
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(BundleAcceptanceError, match="simulation timing"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            compose_resources=lambda _project: (),
            semantic_check=lambda *_args: _physical_evidence(
                tmp_path, achieved=60.0
            ),
        )


@pytest.mark.parametrize(
    "mutation",
    (
        lambda manifest: manifest["source_revisions"][0].update(name="other"),
        lambda manifest: manifest["image_digests"].pop(),
        lambda manifest: manifest["image_digests"][0].update(name="other:image"),
        lambda manifest: manifest["image_digests"][1].update(
            digest=manifest["image_digests"][0]["digest"]
        ),
    ),
    ids=(
        "wrong-source",
        "missing-image",
        "wrong-image-name",
        "duplicate-image-digest",
    ),
)
def test_acceptance_rejects_incomplete_phase3_provenance(tmp_path, mutation):
    """A nonempty provenance list is insufficient for a seven-service run."""
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    _completed_bundle(tmp_path)
    manifest_path = tmp_path / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    mutation(manifest)
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(BundleAcceptanceError, match="provenance"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            compose_resources=lambda _project: (),
            semantic_check=lambda *_args: _physical_evidence(
                tmp_path, achieved=60.0
            ),
        )


def test_acceptance_inspector_rejects_changed_artifact_checksum(tmp_path):
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    _completed_bundle(tmp_path)
    (tmp_path / "video/onboard.mp4").write_bytes(b"changed")

    with pytest.raises(BundleAcceptanceError, match="checksum"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            compose_resources=lambda _project: (),
            semantic_check=lambda _run, _run_id, _frames: None,
        )


def test_acceptance_inspector_rejects_eighth_module_log(tmp_path):
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    _completed_bundle(tmp_path)
    (tmp_path / "logs/extra.jsonl").write_text("{}\n", encoding="utf-8")

    with pytest.raises(BundleAcceptanceError, match="exactly seven"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            compose_resources=lambda _project: (),
            semantic_check=lambda _run, _run_id, _frames: None,
        )


def test_acceptance_requires_digest_bound_physical_evidence(tmp_path):
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    _completed_bundle(tmp_path)

    with pytest.raises(BundleAcceptanceError, match="physical evidence"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            compose_resources=lambda _project: (),
            semantic_check=lambda _run, _run_id, _frames, _checksum: None,
        )


def test_acceptance_rehashes_optional_manifest_artifacts(tmp_path):
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    _completed_bundle(tmp_path, with_optional_artifact=True)
    (tmp_path / "logs/docker/rosbag.log").write_text(
        "changed diagnostic\n", encoding="utf-8"
    )

    with pytest.raises(BundleAcceptanceError, match="checksum"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            compose_resources=lambda _project: (),
            semantic_check=lambda *_args: _physical_evidence(
                tmp_path, achieved=60.0
            ),
        )


def test_acceptance_applies_complete_manifest_domain_invariants(tmp_path):
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    _completed_bundle(tmp_path)
    manifest_path = tmp_path / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["wall_timing"]["duration_seconds"] = -1.0
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")

    with pytest.raises(BundleAcceptanceError, match="manifest domain"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            compose_resources=lambda _project: (),
            semantic_check=lambda *_args: _physical_evidence(
                tmp_path, achieved=60.0
            ),
        )


def test_acceptance_requires_completed_companion_flight_evidence(tmp_path):
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    def remove_land_ack(run_directory: Path) -> None:
        path = run_directory / "logs/companion.jsonl"
        rows = [json.loads(line) for line in path.read_text().splitlines()]
        rows = [
            row for row in rows
            if not (
                row["event"] == "command_acknowledged"
                and row["fields"] == {"command": "LAND"}
            )
        ]
        path.write_text(
            "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
        )

    _completed_bundle(tmp_path, log_mutator=remove_land_ack)

    with pytest.raises(BundleAcceptanceError, match="companion flight evidence"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            compose_resources=lambda _project: (),
            semantic_check=lambda *_args: _physical_evidence(
                tmp_path, achieved=60.0
            ),
        )


def test_semantic_inspector_container_command_uses_pinned_image_and_read_only_mounts(
    tmp_path,
):
    from artifacts.acceptance import semantic_container_command

    bundle = tmp_path / "run"
    rules = tmp_path / "descent_v1.json"

    command = semantic_container_command(
        bundle,
        rules_path=rules,
        require_maximum_score=True,
    )

    assert command == (
        "docker",
        "run",
        "--rm",
        "--network",
        "none",
        "--read-only",
        "--mount",
        f"type=bind,src={bundle.resolve()},dst=/bundle,readonly",
        "--mount",
        f"type=bind,src={rules.resolve()},dst=/rules/descent_v1.json,readonly",
        "drone-sim-artifacts-runtime:phase2",
        "python3",
        "-m",
        "artifacts.acceptance",
        "/bundle",
        "--rules-path",
        "/rules/descent_v1.json",
        "--semantic-only",
        "--require-maximum-score",
    )


def test_host_inventory_uses_exact_label_filtered_read_only_docker_commands():
    from artifacts.acceptance import docker_compose_resources

    calls = []

    def runner(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout="")

    assert docker_compose_resources("drone-sim-run", runner=runner) == ()
    assert [call[0] for call in calls] == [
        [
            "docker", "ps", "-a", "--filter",
            "label=com.docker.compose.project=drone-sim-run",
            "--format", "{{.ID}}",
        ],
        [
            "docker", "network", "ls", "--filter",
            "label=com.docker.compose.project=drone-sim-run",
            "--format", "{{.ID}}",
        ],
        [
            "docker", "volume", "ls", "--filter",
            "label=com.docker.compose.project=drone-sim-run",
            "--format", "{{.Name}}",
        ],
    ]
    assert all(
        kwargs["shell"] is False
        and kwargs["check"] is False
        and kwargs["timeout"] == 30
        for _command, kwargs in calls
    )


def test_host_acceptance_runs_semantics_in_container_then_inventories_compose(tmp_path):
    from artifacts.acceptance import inspect_phase3_via_container, semantic_container_command

    calls = []
    projects = []

    def runner(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(
            returncode=0,
            stdout=json.dumps(
                {
                    "accepted": True,
                    "run_id": RUN_ID,
                    "achieved_score": 60.0,
                    "maximum_available_score": 100.0,
                    "compose_project": "drone-sim-" + RUN_ID.replace("-", ""),
                    "manifest_sha256": "b" * 64,
                }
            )
            + "\n",
        )

    report = inspect_phase3_via_container(
        tmp_path / "run",
        rules_path=tmp_path / "descent_v1.json",
        runner=runner,
        compose_resources=lambda project: projects.append(project) or (),
    )

    assert report.run_id == RUN_ID
    assert calls[0][0] == list(
        semantic_container_command(
            tmp_path / "run", rules_path=tmp_path / "descent_v1.json"
        )
    )
    assert calls[0][1] == {
        "stdout": subprocess.PIPE,
        "stderr": subprocess.STDOUT,
        "text": True,
        "shell": False,
        "check": False,
        "timeout": 300,
    }
    assert projects == ["drone-sim-" + RUN_ID.replace("-", "")]


def test_make_inspect_phase3_invokes_host_orchestrator_cli_exactly(tmp_path):
    root = Path(__file__).parents[2]
    run_directory = tmp_path / "run"

    result = subprocess.run(
        [
            "make",
            "-n",
            "inspect-phase3",
            f"RUN_DIRECTORY={run_directory}",
            "REQUIRE_MAXIMUM_SCORE=1",
        ],
        cwd=root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0
    assert result.stdout.splitlines() == [
        f'test -n "{run_directory}"',
        (
            f'uv run python -m artifacts.acceptance "{run_directory}" '
            "--rules-path scorekeeper/rules/descent_v1.json "
            "--require-maximum-score"
        ),
    ]
