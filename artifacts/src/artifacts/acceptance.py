"""Read-only acceptance inspection for a finalized Phase 3 run bundle."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
from typing import Any

from ._adapters.rosbag import PhysicalBagEvidence, RosbagValidator
from ._adapters.video import VideoValidator
from .manifest import (
    MODULE_LOGS,
    REQUIRED_ARTIFACT_PATHS,
    REQUIRED_DIRECTORY_PATHS,
    ArtifactRecord,
    ConfigurationRecord,
    ImageDigest,
    RunManifest,
    SimulationTiming,
    SourceRevision,
    WallTiming,
    validate_manifest,
)
from .runtime_configuration import resolve_recording_runtime_config
from .score_validation import ScoreValidationError, validate_descent_score_outputs
from .validation import (
    ValidationStatus,
    read_regular_file_bytes,
    validate_gazebo_state,
    validate_nonempty_regular_file,
    validate_regular_file,
    validate_tree,
)


_LOG_FIELDS = {
    "run_id",
    "module",
    "severity",
    "event",
    "sim_timestamp",
    "wall_timestamp",
    "fields",
}
_ARTIFACTS_RUNTIME_IMAGE = "drone-sim-artifacts-runtime:phase2"
_FRAME_INTERVAL_NS = 50_000_000
_PHASE3_IMAGE_NAMES = frozenset(
    {
        "drone-sim-orchestration-runtime:phase2",
        "drone-sim-artifacts-runtime:phase2",
        "drone-sim-companion-runtime:phase3",
        "drone-sim-ardupilot-runtime:phase3",
        "drone-sim-gazebo-runtime:phase3",
        "drone-sim-electromagnet-runtime:phase3",
        "drone-sim-scorekeeper-runtime:phase3",
    }
)


class BundleAcceptanceError(RuntimeError):
    """A finalized bundle does not meet the Phase 3 acceptance contract."""


@dataclass(frozen=True)
class BundleAcceptanceReport:
    run_id: str
    achieved_score: float
    maximum_available_score: float
    compose_project: str
    manifest_sha256: str

    def to_dict(self) -> dict[str, object]:
        return {
            "accepted": True,
            "run_id": self.run_id,
            "achieved_score": self.achieved_score,
            "maximum_available_score": self.maximum_available_score,
            "compose_project": self.compose_project,
            "manifest_sha256": self.manifest_sha256,
        }


ComposeResources = Callable[[str], Sequence[str]]
SemanticCheck = Callable[[Path, str, int, str], PhysicalBagEvidence]


def _exact_dict(value: object, keys: set[str], label: str) -> dict[str, Any]:
    if not isinstance(value, dict) or set(value) != keys:
        raise BundleAcceptanceError(f"manifest domain {label} is invalid")
    return value


def _validate_manifest_domain(document: dict[str, Any]) -> RunManifest:
    top = _exact_dict(
        document,
        {
            "schema_version",
            "run_id",
            "terminal_status",
            "reason",
            "simulation_timing",
            "wall_timing",
            "source_revisions",
            "image_digests",
            "configurations",
            "artifacts",
            "incomplete_paths",
            "scoring",
        },
        "document",
    )
    try:
        simulation = _exact_dict(
            top["simulation_timing"],
            {"start_ns", "end_ns", "duration_ns"},
            "simulation timing",
        )
        wall = _exact_dict(
            top["wall_timing"],
            {"started_at", "ended_at", "duration_seconds"},
            "wall timing",
        )
        scoring = _exact_dict(
            top["scoring"],
            {
                "achieved_score",
                "maximum_available_score",
                "scoring_checksum",
                "evidence_paths",
            },
            "scoring",
        )
        source_revisions = tuple(
            SourceRevision(**_exact_dict(row, {"name", "revision", "dirty"}, "source"))
            for row in top["source_revisions"]
        )
        image_digests = tuple(
            ImageDigest(**_exact_dict(row, {"name", "digest"}, "image digest"))
            for row in top["image_digests"]
        )
        configurations = tuple(
            ConfigurationRecord(
                **_exact_dict(row, {"relative_path", "sha256"}, "configuration")
            )
            for row in top["configurations"]
        )
        artifacts = tuple(
            ArtifactRecord(
                **_exact_dict(
                    row,
                    {
                        "relative_path",
                        "size_bytes",
                        "sha256",
                        "validation",
                        "detail",
                    },
                    "artifact",
                )
            )
            for row in top["artifacts"]
        )
        manifest = RunManifest(
            run_id=top["run_id"],
            terminal_status=top["terminal_status"],
            reason=top["reason"],
            simulation_timing=SimulationTiming(**simulation),
            wall_timing=WallTiming(
                datetime.fromisoformat(wall["started_at"].replace("Z", "+00:00")),
                datetime.fromisoformat(wall["ended_at"].replace("Z", "+00:00")),
                wall["duration_seconds"],
            ),
            source_revisions=source_revisions,
            image_digests=image_digests,
            configurations=configurations,
            artifacts=artifacts,
            incomplete_paths=tuple(top["incomplete_paths"]),
            achieved_score=scoring["achieved_score"],
            maximum_available_score=scoring["maximum_available_score"],
            scoring_checksum=scoring["scoring_checksum"],
            evidence_paths=tuple(scoring["evidence_paths"]),
            schema_version=top["schema_version"],
        )
        validate_manifest(manifest)
    except BundleAcceptanceError:
        raise
    except (AttributeError, KeyError, TypeError, ValueError) as error:
        raise BundleAcceptanceError(f"manifest domain is invalid: {error}") from error
    return manifest


def _read_json(run_directory: Path, relative_path: str) -> tuple[dict[str, Any], str]:
    validation, payload = read_regular_file_bytes(run_directory, relative_path)
    if validation.status is not ValidationStatus.VALID or payload is None:
        raise BundleAcceptanceError(f"{relative_path} is not safe readable evidence")
    try:
        document = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise BundleAcceptanceError(f"{relative_path} is not valid JSON") from error
    if not isinstance(document, dict):
        raise BundleAcceptanceError(f"{relative_path} must contain a JSON object")
    return document, hashlib.sha256(payload).hexdigest()


def _validate_config(run_directory: Path, run_id: str) -> tuple[dict[str, Any], int]:
    document, _digest = _read_json(run_directory, "configuration/run.json")
    checksum = document.get("config_sha256")
    without_checksum = {
        key: value for key, value in document.items() if key != "config_sha256"
    }
    expected_checksum = hashlib.sha256(
        json.dumps(without_checksum, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    if checksum != expected_checksum:
        raise BundleAcceptanceError("configuration checksum is invalid")
    if document.get("run_id") != run_id or document.get("runtime_profile") != "phase3":
        raise BundleAcceptanceError("configuration is not for this Phase 3 run")
    try:
        contract = resolve_recording_runtime_config(document)
    except ValueError as error:
        raise BundleAcceptanceError(f"recording configuration is invalid: {error}") from error
    if not contract.physical_run:
        raise BundleAcceptanceError("configuration does not select physical validation")
    return document, contract.expected_camera_frames


def _validate_phase3_provenance(manifest: dict[str, Any]) -> None:
    sources = manifest["source_revisions"]
    if len(sources) != 1 or sources[0]["name"] != "drone_sim":
        raise BundleAcceptanceError("manifest source provenance is incomplete")
    images = manifest["image_digests"]
    names = {record["name"] for record in images}
    digests = {record["digest"] for record in images}
    if (
        len(images) != len(_PHASE3_IMAGE_NAMES)
        or names != _PHASE3_IMAGE_NAMES
        or len(digests) != len(images)
    ):
        raise BundleAcceptanceError("manifest image provenance is incomplete")


def _validate_simulation_timing(
    manifest: dict[str, Any], expected_camera_frames: int
) -> None:
    duration_ns = expected_camera_frames * _FRAME_INTERVAL_NS
    if manifest["simulation_timing"] != {
        "start_ns": 0,
        "end_ns": duration_ns,
        "duration_ns": duration_ns,
    }:
        raise BundleAcceptanceError(
            "manifest simulation timing does not match the configured public epoch"
        )


def _validate_manifest_inventory(run_directory: Path, manifest: dict[str, Any]) -> None:
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, list):
        raise BundleAcceptanceError("manifest artifact inventory is invalid")
    records: dict[str, dict[str, Any]] = {}
    expected_keys = {
        "relative_path",
        "size_bytes",
        "sha256",
        "validation",
        "detail",
    }
    for record in artifacts:
        if (
            not isinstance(record, dict)
            or set(record) != expected_keys
            or not isinstance(record.get("relative_path"), str)
        ):
            raise BundleAcceptanceError("manifest artifact record is invalid")
        relative_path = record["relative_path"]
        path = Path(relative_path)
        if (
            not relative_path
            or path.is_absolute()
            or ".." in path.parts
            or path.as_posix() != relative_path
        ):
            raise BundleAcceptanceError("manifest artifact path is unsafe")
        if relative_path in records:
            raise BundleAcceptanceError("manifest artifact paths are not unique")
        records[relative_path] = record
    if not set(REQUIRED_ARTIFACT_PATHS).issubset(records):
        raise BundleAcceptanceError("manifest omits required artifacts")

    for relative_path, record in records.items():
        validator = (
            validate_tree
            if relative_path in REQUIRED_DIRECTORY_PATHS
            else validate_regular_file
        )
        actual = validator(run_directory, relative_path)
        if (
            record.get("validation") != actual.status.value
            or record.get("size_bytes") != actual.size_bytes
            or record.get("sha256") != actual.sha256
        ):
            raise BundleAcceptanceError(
                f"artifact checksum does not match manifest: {relative_path}"
            )


def _validate_module_logs(run_directory: Path, run_id: str) -> None:
    logs_directory = run_directory / "logs"
    expected_names = {Path(path).name for path in MODULE_LOGS}
    try:
        actual_names = {path.name for path in logs_directory.glob("*.jsonl")}
    except OSError as error:
        raise BundleAcceptanceError("module logs could not be inventoried") from error
    if actual_names != expected_names:
        raise BundleAcceptanceError("bundle must contain exactly seven JSONL module logs")
    documents_by_module: dict[str, list[dict[str, Any]]] = {}
    for relative_path in MODULE_LOGS:
        validation, payload = read_regular_file_bytes(run_directory, relative_path)
        if validation.status is not ValidationStatus.VALID or payload is None:
            raise BundleAcceptanceError(f"module log is unsafe: {relative_path}")
        module = Path(relative_path).stem
        try:
            lines = payload.decode("utf-8").splitlines()
            documents = [json.loads(line) for line in lines]
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise BundleAcceptanceError(f"module log is invalid JSONL: {relative_path}") from error
        if not documents or any(
            not isinstance(document, dict)
            or set(document) != _LOG_FIELDS
            or document.get("run_id") != run_id
            or document.get("module") != module
            for document in documents
        ):
            raise BundleAcceptanceError(f"module log contract is invalid: {relative_path}")
        documents_by_module[module] = documents
    _validate_production_log_evidence(documents_by_module)


def _validate_production_log_evidence(
    documents_by_module: dict[str, list[dict[str, Any]]],
) -> None:
    companion = documents_by_module["companion"]
    companion_events = {row["event"] for row in companion}
    required_facts = {
        "heartbeat_observed",
        "descent_observed",
        "touchdown_observed",
        "vehicle_disarmed",
        "mission_landed",
        "mission_finished",
    }
    commands = {"SET_GUIDED", "ARM", "TAKEOFF", "LAND"}
    issued = {
        row["fields"].get("command")
        for row in companion
        if row["event"] == "command_issued" and isinstance(row["fields"], dict)
    }
    acknowledged = {
        row["fields"].get("command")
        for row in companion
        if row["event"] == "command_acknowledged" and isinstance(row["fields"], dict)
    }
    mission_finished = any(
        row["event"] == "mission_finished"
        and row["fields"] == {"outcome": "LANDED"}
        for row in companion
    )
    if (
        not required_facts.issubset(companion_events)
        or issued != commands
        or acknowledged != commands
        or not mission_finished
    ):
        raise BundleAcceptanceError("companion flight evidence is incomplete")

    ardupilot = documents_by_module["ardupilot_sitl"]
    if not (
        any(row["event"] == "starting" for row in ardupilot)
        and any(
            row["event"] == "ready"
            and isinstance(row["fields"], dict)
            and row["fields"].get("json_exchange") is True
            and row["fields"].get("mavlink_listening") is True
            for row in ardupilot
        )
        and any(row["event"] == "stopped" for row in ardupilot)
    ):
        raise BundleAcceptanceError("ArduPilot JSON exchange evidence is incomplete")

    gazebo = documents_by_module["gazebo"]
    required_actions = {
        "PublishGazeboReady",
        "RequestSteps",
        "SetPaused",
        "WriteSourceFinished",
        "BeginFinalization",
        "StopServer",
        "WriteQuiescence",
    }
    actions = {
        row["fields"].get("action")
        for row in gazebo
        if row["event"] == "runtime_action" and isinstance(row["fields"], dict)
    }
    if not (
        any(row["event"] == "runtime_started" for row in gazebo)
        and required_actions.issubset(actions)
        and any(row["event"] == "runtime_quiescent" for row in gazebo)
    ):
        raise BundleAcceptanceError("Gazebo runtime action evidence is incomplete")


def _production_semantic_check(
    run_directory: Path,
    run_id: str,
    expected_camera_frames: int,
    config_sha256: str,
) -> PhysicalBagEvidence:
    deadline = time.monotonic() + 120.0
    video_validator = VideoValidator()
    for stream in ("onboard", "observer"):
        result = video_validator.validate(
            run_directory,
            f"video/{stream}.mp4",
            expected_frame_count=expected_camera_frames,
            outcome="COMPLETED",
            deadline=deadline,
        )
        if result.status is not ValidationStatus.VALID:
            raise BundleAcceptanceError(f"{stream} video is invalid: {result.detail}")
    bag = RosbagValidator(
        run_id,
        expected_camera_frames=expected_camera_frames,
        physical_run=True,
        config_sha256=config_sha256,
    ).validate(run_directory, "rosbag")
    if bag.status is not ValidationStatus.VALID:
        raise BundleAcceptanceError(f"physical MCAP is invalid: {bag.detail}")
    state = validate_gazebo_state(run_directory, "gazebo/state")
    server_log = validate_nonempty_regular_file(run_directory, "gazebo/server.log")
    if state.status is not ValidationStatus.VALID:
        raise BundleAcceptanceError(f"Gazebo state is invalid: {state.detail}")
    if server_log.status is not ValidationStatus.VALID:
        raise BundleAcceptanceError(f"Gazebo log is invalid: {server_log.detail}")
    if bag.physical_evidence is None:
        raise BundleAcceptanceError("physical MCAP produced no decoded evidence")
    return bag.physical_evidence


def semantic_container_command(
    run_directory: Path | str,
    *,
    rules_path: Path | str,
    require_maximum_score: bool = False,
) -> tuple[str, ...]:
    command = (
        "docker",
        "run",
        "--rm",
        "--network",
        "none",
        "--read-only",
        "--mount",
        f"type=bind,src={Path(run_directory).resolve()},dst=/bundle,readonly",
        "--mount",
        f"type=bind,src={Path(rules_path).resolve()},dst=/rules/descent_v1.json,readonly",
        _ARTIFACTS_RUNTIME_IMAGE,
        "python3",
        "-m",
        "artifacts.acceptance",
        "/bundle",
        "--rules-path",
        "/rules/descent_v1.json",
        "--semantic-only",
    )
    return command + (("--require-maximum-score",) if require_maximum_score else ())


def inspect_phase3_via_container(
    run_directory: Path | str,
    *,
    rules_path: Path | str,
    require_maximum_score: bool = False,
    runner: Callable[..., Any] = subprocess.run,
    compose_resources: ComposeResources | None = None,
) -> BundleAcceptanceReport:
    result = runner(
        list(
            semantic_container_command(
                run_directory,
                rules_path=rules_path,
                require_maximum_score=require_maximum_score,
            )
        ),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        shell=False,
        check=False,
        timeout=300,
    )
    if result.returncode != 0:
        raise BundleAcceptanceError(
            "container semantic inspection failed: " + result.stdout.strip()
        )
    try:
        payload = json.loads(next(line for line in reversed(result.stdout.splitlines()) if line))
    except (StopIteration, json.JSONDecodeError) as error:
        raise BundleAcceptanceError(
            "container semantic inspection returned invalid JSON"
        ) from error
    expected_keys = {
        "accepted",
        "run_id",
        "achieved_score",
        "maximum_available_score",
        "compose_project",
        "manifest_sha256",
    }
    if (
        not isinstance(payload, dict)
        or set(payload) != expected_keys
        or payload["accepted"] is not True
    ):
        raise BundleAcceptanceError("container semantic inspection did not accept the bundle")
    run_id = payload["run_id"]
    expected_project = (
        "drone-sim-" + run_id.replace("-", "") if isinstance(run_id, str) else None
    )
    if payload["compose_project"] != expected_project:
        raise BundleAcceptanceError("container semantic report has wrong Compose project")
    inventory = compose_resources or docker_compose_resources
    leftovers = tuple(inventory(payload["compose_project"]))
    if leftovers:
        raise BundleAcceptanceError(
            "leftover Compose resources: " + ", ".join(leftovers)
        )
    try:
        return BundleAcceptanceReport(
            run_id,
            float(payload["achieved_score"]),
            float(payload["maximum_available_score"]),
            payload["compose_project"],
            payload["manifest_sha256"],
        )
    except (TypeError, ValueError) as error:
        raise BundleAcceptanceError("container semantic report has invalid fields") from error


def docker_compose_resources(
    project_name: str,
    *,
    runner: Callable[..., Any] = subprocess.run,
) -> tuple[str, ...]:
    """Return project-labelled Docker resources without modifying them."""
    commands = (
        ("container", ["docker", "ps", "-a"], "{{.ID}}"),
        ("network", ["docker", "network", "ls"], "{{.ID}}"),
        ("volume", ["docker", "volume", "ls"], "{{.Name}}"),
    )
    resources: list[str] = []
    for kind, command, output_format in commands:
        result = runner(
            [
                *command,
                "--filter",
                f"label=com.docker.compose.project={project_name}",
                "--format",
                output_format,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            shell=False,
            check=False,
            timeout=30,
        )
        if result.returncode != 0:
            raise BundleAcceptanceError(
                f"Docker {kind} inventory failed: {result.stdout.strip()}"
            )
        resources.extend(
            f"{kind}:{identifier}"
            for identifier in result.stdout.splitlines()
            if identifier.strip()
        )
    return tuple(resources)


def inspect_phase3_semantics(
    run_directory: Path | str,
    *,
    rules_path: Path | str,
    require_maximum_score: bool = False,
    semantic_check: SemanticCheck = _production_semantic_check,
) -> BundleAcceptanceReport:
    """Assert bundle semantics without accessing host Docker inventory."""
    if not isinstance(require_maximum_score, bool):
        raise TypeError("require_maximum_score must be a boolean")
    directory = Path(run_directory).resolve()
    manifest, manifest_sha256 = _read_json(directory, "manifest.json")
    _validate_manifest_domain(manifest)
    run_id = manifest.get("run_id")
    if not isinstance(run_id, str) or not run_id:
        raise BundleAcceptanceError("manifest run_id is invalid")
    if manifest.get("schema_version") != 1 or manifest.get("terminal_status") != "COMPLETED":
        raise BundleAcceptanceError("manifest is not a completed schema-v1 run")
    if manifest.get("incomplete_paths") != []:
        raise BundleAcceptanceError("completed manifest contains incomplete artifacts")
    _validate_phase3_provenance(manifest)

    configuration, expected_frames = _validate_config(directory, run_id)
    _validate_simulation_timing(manifest, expected_frames)
    configurations = manifest.get("configurations")
    if configurations != [
        {
            "relative_path": "configuration/run.json",
            "sha256": configuration["config_sha256"],
        }
    ]:
        raise BundleAcceptanceError("manifest configuration provenance is invalid")
    _validate_manifest_inventory(directory, manifest)
    _validate_module_logs(directory, run_id)
    physical_evidence = semantic_check(
        directory,
        run_id,
        expected_frames,
        configuration["config_sha256"],
    )
    if not isinstance(physical_evidence, PhysicalBagEvidence):
        raise BundleAcceptanceError("semantic inspection returned no physical evidence")

    try:
        score = validate_descent_score_outputs(
            directory,
            run_id=run_id,
            rules_path=rules_path,
            physical_evidence=physical_evidence,
        )
    except ScoreValidationError as error:
        raise BundleAcceptanceError(f"score evidence is invalid: {error}") from error
    scoring = manifest.get("scoring")
    if scoring != {
        "achieved_score": score.achieved_score,
        "maximum_available_score": score.maximum_available_score,
        "scoring_checksum": score.scoring_checksum,
        "evidence_paths": list(score.evidence_paths),
    }:
        raise BundleAcceptanceError("manifest scoring provenance does not match score evidence")
    if require_maximum_score and not (
        math.isclose(score.achieved_score, 100.0)
        and math.isclose(score.maximum_available_score, 100.0)
    ):
        raise BundleAcceptanceError("bundle did not achieve the required 100/100 score")

    compose_project = "drone-sim-" + run_id.replace("-", "")
    return BundleAcceptanceReport(
        run_id,
        score.achieved_score,
        score.maximum_available_score,
        compose_project,
        manifest_sha256,
    )


def inspect_phase3_bundle(
    run_directory: Path | str,
    *,
    rules_path: Path | str,
    require_maximum_score: bool = False,
    compose_resources: ComposeResources = docker_compose_resources,
    semantic_check: SemanticCheck = _production_semantic_check,
) -> BundleAcceptanceReport:
    """Compatibility helper combining semantics with host Docker inventory."""
    report = inspect_phase3_semantics(
        run_directory,
        rules_path=rules_path,
        require_maximum_score=require_maximum_score,
        semantic_check=semantic_check,
    )
    leftovers = tuple(compose_resources(report.compose_project))
    if leftovers:
        raise BundleAcceptanceError(
            "leftover Compose resources: " + ", ".join(leftovers)
        )
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--rules-path", type=Path)
    parser.add_argument("--require-maximum-score", action="store_true")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--semantic-only", action="store_true")
    modes.add_argument("--inventory-only", action="store_true")
    arguments = parser.parse_args(argv)
    try:
        if arguments.inventory_only:
            manifest, _digest = _read_json(
                arguments.run_directory.resolve(), "manifest.json"
            )
            run_id = manifest.get("run_id")
            if not isinstance(run_id, str) or not run_id:
                raise BundleAcceptanceError("manifest run_id is invalid")
            compose_project = "drone-sim-" + run_id.replace("-", "")
            leftovers = docker_compose_resources(compose_project)
            if leftovers:
                raise BundleAcceptanceError(
                    "leftover Compose resources: " + ", ".join(leftovers)
                )
            print(
                json.dumps(
                    {"accepted": True, "compose_project": compose_project},
                    sort_keys=True,
                )
            )
            return 0
        if arguments.rules_path is None:
            parser.error("--rules-path is required for semantic acceptance")
        inspector = (
            inspect_phase3_semantics
            if arguments.semantic_only
            else inspect_phase3_via_container
        )
        report = inspector(
            arguments.run_directory,
            rules_path=arguments.rules_path,
            require_maximum_score=arguments.require_maximum_score,
        )
    except BundleAcceptanceError as error:
        print(json.dumps({"accepted": False, "detail": str(error)}, sort_keys=True))
        return 1
    print(json.dumps(report.to_dict(), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "BundleAcceptanceError",
    "BundleAcceptanceReport",
    "docker_compose_resources",
    "inspect_phase3_bundle",
    "inspect_phase3_semantics",
    "inspect_phase3_via_container",
    "main",
    "semantic_container_command",
]
