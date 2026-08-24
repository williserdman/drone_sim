from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import pytest

from artifacts import (
    ArtifactSession,
    ConfigurationRecord,
    FinalizationInput,
    ImageDigest,
    SourceRevision,
)
from artifacts.manifest import MODULE_LOGS, REQUIRED_ARTIFACT_PATHS


RUN_ID = "00000000-0000-4000-8000-000000000606"
RULES_PATH = Path(__file__).parents[2] / "scorekeeper/rules/descent_v1.json"


def _completed_bundle(run_directory: Path, *, achieved: float = 60.0) -> None:
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

    for module, relative_path in zip(modules, MODULE_LOGS, strict=True):
        (run_directory / relative_path).write_text(
            json.dumps(
                {
                    "run_id": RUN_ID,
                    "module": module,
                    "severity": "INFO",
                    "event": "captured",
                    "sim_timestamp": None,
                    "wall_timestamp": "2026-08-24T00:00:00Z",
                    "fields": {},
                }
            )
            + "\n",
            encoding="utf-8",
        )

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
            image_digests=(ImageDigest("phase3", "a" * 64),),
            configuration_records=(ConfigurationRecord("configuration/run.json", config_sha),),
            achieved_score=achieved,
            maximum_available_score=100.0,
            scoring_checksum=checksum,
            evidence_paths=evidence,
        )
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
        semantic_check=lambda _run, _run_id, _frames: None,
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
            semantic_check=lambda _run, _run_id, _frames: None,
        )


def test_acceptance_inspector_rejects_leftover_compose_resources(tmp_path):
    from artifacts.acceptance import BundleAcceptanceError, inspect_phase3_bundle

    _completed_bundle(tmp_path)

    with pytest.raises(BundleAcceptanceError, match="Compose resources"):
        inspect_phase3_bundle(
            tmp_path,
            rules_path=RULES_PATH,
            compose_resources=lambda _project: ("container:deadbeef",),
            semantic_check=lambda _run, _run_id, _frames: None,
        )


def test_acceptance_inspector_accepts_maximum_score_when_required(tmp_path):
    from artifacts.acceptance import inspect_phase3_bundle

    _completed_bundle(tmp_path, achieved=100.0)

    report = inspect_phase3_bundle(
        tmp_path,
        rules_path=RULES_PATH,
        require_maximum_score=True,
        compose_resources=lambda _project: (),
        semantic_check=lambda _run, _run_id, _frames: None,
    )

    assert report.achieved_score == 100.0


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
