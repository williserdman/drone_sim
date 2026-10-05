"""Read-only acceptance inspection for a finalized Phase 3 run bundle."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
from typing import Any

from ._adapters.rosbag import GroundTruthEvidence, PhysicalBagEvidence, RosbagValidator
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
from .score_validation import ScoreValidationError, validate_score_outputs
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
_MODULE_LOG_MAX_BYTES = 32 * 1024 * 1024
_PHASE3_IMAGE_NAMES = (
    "drone-sim-orchestration-runtime:phase2",
    "drone-sim-artifacts-runtime:phase2",
    "drone-sim-companion-runtime:phase3",
    "drone-sim-ardupilot-runtime:phase3",
    "drone-sim-gazebo-runtime:phase3",
    "drone-sim-electromagnet-runtime:phase3",
    "drone-sim-scorekeeper-runtime:phase3",
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


def _expected_sources(
    *,
    expected_source_revision: str | None,
    expected_source_dirty: bool | Mapping[str, bool] | None,
    expected_source_revisions: Mapping[str, str] | None,
) -> tuple[dict[str, str], dict[str, bool]]:
    """Normalize the legacy single-source and current two-source interfaces."""
    if expected_source_revisions is not None:
        if expected_source_revision is not None or not isinstance(
            expected_source_dirty, Mapping
        ):
            raise TypeError(
                "source revision and dirty expectations must both be mappings"
            )
        revisions = dict(expected_source_revisions)
        dirty = dict(expected_source_dirty)
    else:
        if not isinstance(expected_source_revision, str) or type(
            expected_source_dirty
        ) is not bool:
            raise TypeError("legacy source provenance expectation is incomplete")
        revisions = {"drone_sim": expected_source_revision}
        dirty = {"drone_sim": expected_source_dirty}
    if tuple(revisions) != tuple(dirty) or not revisions:
        raise TypeError("source revision and dirty expectation names must match")
    if any(not isinstance(name, str) or not isinstance(value, str) for name, value in revisions.items()):
        raise TypeError("source revision expectations are invalid")
    if any(type(value) is not bool for value in dirty.values()):
        raise TypeError("source dirty expectations are invalid")
    return revisions, dirty


def _parse_boolean(value: str) -> bool:
    if value == "true":
        return True
    if value == "false":
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def _parse_image_digest(value: str) -> tuple[str, str]:
    name, separator, digest = value.partition("=")
    if (
        not separator
        or not name
        or len(digest) != 64
        or any(character not in "0123456789abcdef" for character in digest)
    ):
        raise argparse.ArgumentTypeError("expected IMAGE=64-lowercase-hex-digest")
    return name, digest


def _parse_named_source(value: str) -> tuple[str, str]:
    name, separator, revision = value.partition("=")
    if not separator or not name or not revision:
        raise argparse.ArgumentTypeError("expected SOURCE=REVISION")
    return name, revision


def _parse_named_dirty(value: str) -> tuple[str, bool]:
    name, separator, dirty = value.partition("=")
    if not separator or not name:
        raise argparse.ArgumentTypeError("expected SOURCE=true|false")
    return name, _parse_boolean(dirty)


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


def _validate_phase3_provenance(
    manifest: dict[str, Any],
    *,
    expected_source_revisions: Mapping[str, str],
    expected_source_dirty: Mapping[str, bool],
    expected_image_digests: Mapping[str, str],
) -> None:
    sources = manifest["source_revisions"]
    if tuple(expected_source_revisions) != tuple(expected_source_dirty):
        raise BundleAcceptanceError("expected source provenance names do not match")
    expected_sources = [
        {"name": name, "revision": revision, "dirty": expected_source_dirty[name]}
        for name, revision in expected_source_revisions.items()
    ]
    if sources != expected_sources:
        raise BundleAcceptanceError(
            "manifest source provenance does not match external expectation"
        )
    images = manifest["image_digests"]
    names = {record["name"] for record in images}
    digests = {record["digest"] for record in images}
    if (
        len(images) != len(_PHASE3_IMAGE_NAMES)
        or names != set(_PHASE3_IMAGE_NAMES)
        or len(digests) != len(images)
    ):
        raise BundleAcceptanceError("manifest image provenance is incomplete")
    actual_image_digests = {
        record["name"]: record["digest"] for record in images
    }
    if actual_image_digests != dict(expected_image_digests):
        raise BundleAcceptanceError(
            "manifest image provenance does not match external expectation"
        )


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


def _validate_module_logs(
    run_directory: Path, run_id: str, *, ruleset_id: str,
    mission: str = "controlled_descent", mission_plan: Mapping[str, Any] | None = None,
) -> list[dict[str, Any]]:
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
        validation, payload = read_regular_file_bytes(
            run_directory,
            relative_path,
            max_bytes=_MODULE_LOG_MAX_BYTES,
        )
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
    _validate_production_log_evidence(
        documents_by_module, ruleset_id=ruleset_id, mission=mission,
        mission_plan=mission_plan,
    )
    return documents_by_module["companion"]


def _configured_operation_pairs(companion, mission_plan):
    steps = mission_plan.get("steps", []) if isinstance(mission_plan, Mapping) else []
    rows = [row for row in companion if row["event"] in {
        "operation_started", "operation_finished"}]
    if not steps or len(rows) != 2 * len(steps):
        raise BundleAcceptanceError("configured operation evidence is incomplete")
    pairs = []
    previous_end = -1.0
    identities = set()
    for step, start, finish in zip(steps, rows[::2], rows[1::2], strict=True):
        started, finished = start["fields"], finish["fields"]
        start_time, end_time = start["sim_timestamp"], finish["sim_timestamp"]
        valid = (
            isinstance(started, dict) and isinstance(finished, dict)
            and start["event"] == "operation_started"
            and finish["event"] == "operation_finished"
            and started.get("tool") == finished.get("tool") == step["tool"]
            and started.get("args") == step["args"]
            and isinstance(started.get("operation_id"), str)
            and started["operation_id"] not in identities
            and started["operation_id"] == finished.get("operation_id")
            and finished.get("state") == "succeeded"
            and all(type(value) in (float, int) and math.isfinite(value)
                    for value in (start_time, end_time))
            and previous_end <= start_time <= end_time
        )
        if not valid:
            raise BundleAcceptanceError("configured operation sequence does not match the plan")
        identities.add(started["operation_id"])
        previous_end = end_time
        pairs.append((step, start_time, end_time))
    return pairs


def _validate_calibration_hover(
    samples: Sequence[GroundTruthEvidence], *, hold_start_ns: int,
    hold_end_ns: int, altitude_m: float,
) -> None:
    """Require five seconds of stable world-frame motion in the recorded hold."""
    if not samples:
        raise BundleAcceptanceError("calibration validation hover has no ground truth")
    origin_z = samples[0].position_xyz[2]
    stable_start = previous = None
    for sample in samples:
        stamp = sample.sim_timestamp_ns
        if stamp < hold_start_ns or stamp > hold_end_ns:
            continue
        x, y, z, w = sample.orientation_xyzw
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm == 0:
            stable_start = previous = None
            continue
        x, y, z, w = (value / norm for value in (x, y, z, w))
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
        # descent_v1 keeps its historical body-frame evidence. Rotate here only.
        vx, vy, vz = sample.linear_velocity_xyz
        world_x = (1-2*(y*y+z*z))*vx + 2*(x*y-z*w)*vy + 2*(x*z+y*w)*vz
        world_y = 2*(x*y+z*w)*vx + (1-2*(x*x+z*z))*vy + 2*(y*z-x*w)*vz
        world_z = 2*(x*z-y*w)*vx + 2*(y*z+x*w)*vy + (1-2*(x*x+y*y))*vz
        stable = (
            not sample.in_contact
            and abs(sample.position_xyz[2] - origin_z - altitude_m) <= 0.5
            and math.hypot(world_x, world_y) <= 0.2
            and abs(world_z) <= 0.2
            and abs(roll) <= math.radians(5) and abs(pitch) <= math.radians(5)
        )
        if not stable:
            stable_start = previous = None
            continue
        if previous is None or stamp - previous != _FRAME_INTERVAL_NS:
            stable_start = stamp
        previous = stamp
        if stamp - stable_start >= 5_000_000_000:
            return
    raise BundleAcceptanceError("calibration validation hover was not stable for five seconds")


def _requires_reload_validation(configuration):
    calibration = configuration.get("calibration_json")
    return calibration is not None and (
        configuration.get("calibration_validation", False)
        or calibration["profile"].get("schema_version", 1) == 1
    )


def _validate_operator_evidence(run_directory, configuration, companion):
    checked, payload = read_regular_file_bytes(run_directory, "logs/docker/operator.jsonl")
    try:
        rows = [json.loads(line) for line in payload.splitlines()] if payload else []
        pairs = _configured_operation_pairs(companion, configuration["mission_plan"])
        step, started, finished = pairs[0]
        if step["tool"] != "wait_for_state" or step["args"] != {"armed": True, "mode": "GUIDED"}:
            raise ValueError("frozen first operation is not the operator wait")
        sequence = [(row["event"], row.get("command")) for row in rows]
        if sequence != [("operator_command", "SET_GUIDED"), ("operator_acknowledgement", "SET_GUIDED"),
                        ("operator_observed_guided", None), ("operator_command", "ARM"),
                        ("operator_acknowledgement", "ARM"), ("operator_observed_armed", None)]:
            raise ValueError("command/ACK/observed-state sequence differs")
        stamps = [row["sim_timestamp_ns"] for row in rows]
        if any(row["run_id"] != configuration["run_id"] for row in rows) or any(type(stamp) is not int or stamp < 0 for stamp in stamps):
            raise ValueError("run identity or public timestamps differ")
        if stamps != sorted(stamps) or stamps[0] < round(started*1e9) or stamps[3] > round(finished*1e9):
            raise ValueError("operator commands do not fall within the started wait")
    except (ValueError, KeyError, TypeError, IndexError) as error:
        raise BundleAcceptanceError(f"operator evidence is invalid: {error}") from error


def _validate_calibration_inputs(run_directory, calibration, companion):
    for relative, field in (
        ("configuration/calibration.parm", "source_artifact_sha256"),
        ("configuration/calibration-manifest.json", "source_manifest_sha256"),
    ):
        checked, payload = read_regular_file_bytes(run_directory, relative)
        if payload is None or checked.sha256 != calibration.get(field):
            raise BundleAcceptanceError(f"calibration input checksum mismatch: {relative}")
    from .calibration import read_calibration_parameters

    try:
        source_id, gains = read_calibration_parameters(run_directory / "configuration/calibration.parm")
        source_manifest, _ = _read_json(run_directory, "configuration/calibration-manifest.json")
        if (source_id != calibration["source_run_id"]
                or source_manifest["run_id"] != source_id or gains != calibration["gains"]):
            raise ValueError("frozen source identity or gain values differ")
        profile = calibration["profile"]
        baseline = profile["effective_baseline_parameters"] if profile.get("schema_version") == 2 else profile["baseline_parameters"]
        expected = {**baseline, **gains}
    except (OSError, KeyError, ValueError) as error:
        raise BundleAcceptanceError(f"calibration input is invalid: {error}") from error
    # Match a durable readback before any arm operation, never after touchdown.
    for row in companion:
        fields = row["fields"]
        if (row["event"] == "operation_started" and fields.get("tool") != "wait_for_state") or row["event"] in {
            "command_issued", "autotune_command_status", "hover_command_status", "mission_command_delivered"
        } or (row["event"] in {"hover_phase", "autotune_phase"} and fields.get("phase") != "WAIT_READY") or (
            row["event"] == "mission_event" and fields.get("state") == "STARTED"
        ):
            break
        if (row["event"] == "calibration_parameters_verified"
                and fields.get("stage") == "pre_arm"):
            observed = fields.get("parameters")
            if (isinstance(observed, dict) and set(observed) == set(expected)
                    and all(type(observed[key]) in (int, float)
                            and math.isclose(observed[key], value, rel_tol=1e-5, abs_tol=1e-7)
                            for key, value in expected.items())):
                return
    raise BundleAcceptanceError("calibration validation lacks matching pre-arm parameter readback")


def _validate_calibration_profile(run_directory, profile, manifest):
    if not isinstance(profile, dict):
        raise BundleAcceptanceError("calibration aircraft profile is missing")
    payload = {key: value for key, value in profile.items() if key != "profile_sha256"}
    checksum = hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    if profile.get("profile_sha256") != checksum:
        raise BundleAcceptanceError("calibration aircraft profile checksum differs")
    checked, contents = read_regular_file_bytes(run_directory, "gazebo/server.log")
    try:
        preamble = json.loads(contents.splitlines()[0]) if contents else {}
        resource = profile.get("consumer_model_resource", "gazebo/resources/models/iris_flight/model.sdf")
        if not resource.startswith("gazebo/resources/models/"):
            raise ValueError("consumer model resource is invalid")
        model = dict(preamble["resource_sha256s"])[resource.removeprefix("gazebo/resources/")]
        images = {row["name"]: row["digest"] for row in manifest["image_digests"]}
        if (model != profile.get("consumer_model_sha256", profile["physical_model_sha256"])
                or not profile["image_digests"]
                or any(images.get(name) != digest for name, digest in profile["image_digests"].items())):
            raise ValueError("runtime aircraft/image identity differs")
    except (KeyError, TypeError, ValueError, IndexError) as error:
        raise BundleAcceptanceError(f"calibration aircraft profile differs from recorded runtime: {error}") from error


def _validate_production_log_evidence(
    documents_by_module: dict[str, list[dict[str, Any]]],
    *,
    ruleset_id: str,
    mission: str = "controlled_descent",
    mission_plan: Mapping[str, Any] | None = None,
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
    descent_evidence_valid = (
        required_facts.issubset(companion_events)
        and issued == commands
        and acknowledged == commands
    )
    if not mission_finished or (
        ruleset_id == "descent_v1" and mission not in {"configured", "hover_roll", "autotune_roll"} and not descent_evidence_valid
    ):
        raise BundleAcceptanceError("companion flight evidence is incomplete")
    if mission == "configured":
        _configured_operation_pairs(companion, mission_plan)
    if mission in {"hover_roll", "autotune_roll"}:
        from .diagnostic_acceptance import validate_diagnostic_logs
        validate_diagnostic_logs(mission, companion)
    if ruleset_id == "calibration_v1":
        statuses = [row["fields"].get("text") for row in companion
                    if row["event"] == "ardupilot_status_text"]
        required = ("AutoTune: Success", "AutoTune: Pilot Testing gains for Roll Pitch Yaw(E)",
                    "AutoTune: Saved gains for Roll Pitch Yaw(E)")
        next_required = iter(required)
        expected = next(next_required)
        for status in statuses:
            if status == expected:
                expected = next(next_required, None)
        phases = [row["fields"].get("phase") for row in companion
                  if row["event"] == "autotune_phase"]
        if (mission != "autotune" or expected is not None
                or not {"autotune_aux_ack", "autotune_disarmed"}.issubset(companion_events)
                or "LANDING" not in phases or not phases or phases[-1] != "COMPLETE"):
            raise BundleAcceptanceError("all-axis AutoTune flight evidence is incomplete")

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
    required_action_sequence = (
        "PublishGazeboReady",
        "SetPaused",
        "ActivateOutput",
        "SetPaused",
        "WriteSourceFinished",
        "BeginFinalization",
        "StopServer",
        "WriteQuiescence",
    )
    actions = [
        row["fields"].get("action")
        for row in gazebo
        if row["event"] == "runtime_action" and isinstance(row["fields"], dict)
    ]
    required_index = 0
    for action in actions:
        if action == required_action_sequence[required_index]:
            required_index += 1
            if required_index == len(required_action_sequence):
                break
    if not (
        any(row["event"] == "runtime_started" for row in gazebo)
        and required_index == len(required_action_sequence)
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
    configuration, _digest = _read_json(run_directory, "configuration/run.json")
    try:
        contract = resolve_recording_runtime_config(configuration)
    except ValueError as error:
        raise BundleAcceptanceError(
            f"recording configuration is invalid: {error}"
        ) from error
    video_validator = VideoValidator(
        width_px=contract.width_px,
        height_px=contract.height_px,
        fps=contract.fps,
    )
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
        ruleset_id=contract.ruleset_id,
        width_px=contract.width_px,
        height_px=contract.height_px,
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
    expected_source_revision: str | None = None,
    expected_source_dirty: bool | Mapping[str, bool] | None = None,
    expected_source_revisions: Mapping[str, str] | None = None,
    expected_image_digests: Mapping[str, str],
    require_maximum_score: bool = False,
) -> tuple[str, ...]:
    if set(expected_image_digests) != set(_PHASE3_IMAGE_NAMES):
        raise ValueError("expected_image_digests must contain exact Phase 3 image names")
    source_revisions, source_dirty = _expected_sources(
        expected_source_revision=expected_source_revision,
        expected_source_dirty=expected_source_dirty,
        expected_source_revisions=expected_source_revisions,
    )
    if expected_source_revisions is None:
        provenance_arguments = (
            "--expected-source-revision",
            source_revisions["drone_sim"],
            "--expected-source-dirty",
            str(source_dirty["drone_sim"]).lower(),
        )
    else:
        provenance_arguments = tuple(
            argument
            for name in source_revisions
            for argument in (
                "--expected-source",
                f"{name}={source_revisions[name]}",
                "--expected-source-dirty-entry",
                f"{name}={str(source_dirty[name]).lower()}",
            )
        )
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
        f"type=bind,src={Path(rules_path).resolve()},dst=/rules/rules.json,readonly",
        _ARTIFACTS_RUNTIME_IMAGE,
        "python3",
        "-m",
        "artifacts.acceptance",
        "/bundle",
        "--rules-path",
        "/rules/rules.json",
        "--semantic-only",
        *provenance_arguments,
        *(
            argument
            for name in _PHASE3_IMAGE_NAMES
            for argument in (
                "--expected-image-digest",
                f"{name}={expected_image_digests[name]}",
            )
        ),
    )
    return command + (("--require-maximum-score",) if require_maximum_score else ())


def inspect_phase3_via_container(
    run_directory: Path | str,
    *,
    rules_path: Path | str,
    expected_source_revision: str | None = None,
    expected_source_dirty: bool | Mapping[str, bool] | None = None,
    expected_source_revisions: Mapping[str, str] | None = None,
    expected_image_digests: Mapping[str, str],
    require_maximum_score: bool = False,
    runner: Callable[..., Any] = subprocess.run,
    compose_resources: ComposeResources | None = None,
) -> BundleAcceptanceReport:
    result = runner(
        list(
            semantic_container_command(
                run_directory,
                rules_path=rules_path,
                expected_source_revision=expected_source_revision,
                expected_source_dirty=expected_source_dirty,
                expected_source_revisions=expected_source_revisions,
                expected_image_digests=expected_image_digests,
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
    expected_image_digests: Mapping[str, str],
    expected_source_revision: str | None = None,
    expected_source_dirty: bool | Mapping[str, bool] | None = None,
    expected_source_revisions: Mapping[str, str] | None = None,
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
    source_revisions, source_dirty = _expected_sources(
        expected_source_revision=expected_source_revision,
        expected_source_dirty=expected_source_dirty,
        expected_source_revisions=expected_source_revisions,
    )
    _validate_phase3_provenance(
        manifest,
        expected_source_revisions=source_revisions,
        expected_source_dirty=source_dirty,
        expected_image_digests=expected_image_digests,
    )

    configuration, expected_frames = _validate_config(directory, run_id)
    _validate_simulation_timing(manifest, expected_frames)
    configurations = manifest.get("configurations")
    expected_configurations = [
        {
            "relative_path": "configuration/run.json",
            "sha256": configuration["config_sha256"],
        }
    ]
    calibration = configuration.get("calibration_json")
    if calibration is not None:
        expected_configurations.extend([
            {"relative_path": "configuration/calibration.parm",
             "sha256": calibration.get("source_artifact_sha256")},
            {"relative_path": "configuration/calibration-manifest.json",
             "sha256": calibration.get("source_manifest_sha256")},
        ])
    if configurations != expected_configurations:
        raise BundleAcceptanceError("manifest configuration provenance is invalid")
    _validate_manifest_inventory(directory, manifest)
    scenario = configuration.get("scenario", "descent_v1")
    ruleset_id = (
        scenario
        if scenario in {"descent_v1", "competition_v1", "moving_pad_v1", "calibration_v1"}
        else "descent_v1"
    )
    companion_logs = _validate_module_logs(
        directory, run_id, ruleset_id=ruleset_id,
        mission=configuration.get("mission", "controlled_descent"),
        mission_plan=configuration.get("mission_plan"),
    )
    if (directory / "logs/docker/operator.jsonl").exists():
        if not any(row["relative_path"] == "logs/docker/operator.jsonl" for row in manifest["artifacts"]):
            raise BundleAcceptanceError("operator evidence is absent from manifest inventory")
        _validate_operator_evidence(directory, configuration, companion_logs)
    physical_evidence = semantic_check(
        directory,
        run_id,
        expected_frames,
        configuration["config_sha256"],
    )
    if not isinstance(physical_evidence, PhysicalBagEvidence):
        raise BundleAcceptanceError("semantic inspection returned no physical evidence")
    if ruleset_id == "calibration_v1":
        from .calibration import validate_calibration_artifact

        _validate_calibration_profile(directory, configuration.get("calibration_profile"), manifest)
        try:
            validate_calibration_artifact(directory)
        except (OSError, ValueError) as error:
            raise BundleAcceptanceError(f"calibration artifact is invalid: {error}") from error
    if calibration is not None:
        _validate_calibration_inputs(directory, calibration, companion_logs)
        _validate_calibration_profile(directory, calibration.get("profile"), manifest)
        if calibration["profile"].get("schema_version") == 2 and calibration["profile"].get("consumer_vehicle") != configuration["vehicle"]:
            raise BundleAcceptanceError("calibration consumer aircraft differs from frozen run")
    if _requires_reload_validation(configuration):
        pairs = _configured_operation_pairs(companion_logs, configuration.get("mission_plan"))
        altitude = None
        verified_hover = False
        for step, start, end in pairs:
            if step["tool"] == "takeoff":
                altitude = step["args"].get("altitude_m")
            if (step["tool"] == "hold" and altitude == 5.0
                    and step["args"].get("duration_sim_s") == 10.0):
                _validate_calibration_hover(
                    physical_evidence.ground_truth, hold_start_ns=round(start * 1e9),
                    hold_end_ns=round(end * 1e9), altitude_m=altitude,
                )
                verified_hover = True
        if not verified_hover:
            raise BundleAcceptanceError("calibration validation requires a recorded 5 m, 10 s hover")
    if configuration.get("mission") == "hover_roll":
        phases = [(row["fields"].get("phase"), row["sim_timestamp"]) for row in companion_logs if row["event"] == "hover_phase"]
        start = next((stamp for phase, stamp in phases if phase == "HOVERING"), None)
        end = next((stamp for phase, stamp in phases if phase == "WAIT_LAND" and start is not None and stamp >= start), None)
        if start is None or end is None or not math.isclose(end - start, 10.0, abs_tol=0.1):
            raise BundleAcceptanceError("hover diagnostic requires its recorded ten-second ALT_HOLD")
        _validate_calibration_hover(physical_evidence.ground_truth, hold_start_ns=round(start*1e9), hold_end_ns=round(end*1e9), altitude_m=5.0)
    if configuration.get("mission") == "autotune_roll":
        from .diagnostic_acceptance import validate_roll_gain_artifact
        validate_roll_gain_artifact(directory)

    try:
        score = validate_score_outputs(
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
    diagnostic = configuration.get("mission") in {"hover_roll", "autotune_roll"}
    if diagnostic:
        from .diagnostic_acceptance import validate_diagnostic_landing
        score_document, _ = _read_json(directory, "scoring/result.json")
        validate_diagnostic_landing(score_document["rule_results"])
    if (require_maximum_score or ruleset_id == "calibration_v1" or (calibration is not None and not diagnostic)) and not math.isclose(
        score.achieved_score, score.maximum_available_score
    ):
        required = "150/150" if ruleset_id == "competition_v1" else "100/100"
        raise BundleAcceptanceError(
            f"bundle did not achieve the required {required} score"
        )

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
    expected_image_digests: Mapping[str, str],
    expected_source_revision: str | None = None,
    expected_source_dirty: bool | Mapping[str, bool] | None = None,
    expected_source_revisions: Mapping[str, str] | None = None,
    require_maximum_score: bool = False,
    compose_resources: ComposeResources = docker_compose_resources,
    semantic_check: SemanticCheck = _production_semantic_check,
) -> BundleAcceptanceReport:
    """Compatibility helper combining semantics with host Docker inventory."""
    report = inspect_phase3_semantics(
        run_directory,
        rules_path=rules_path,
        expected_source_revision=expected_source_revision,
        expected_source_dirty=expected_source_dirty,
        expected_source_revisions=expected_source_revisions,
        expected_image_digests=expected_image_digests,
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
    parser.add_argument("--expected-source-revision")
    parser.add_argument("--expected-source-dirty", type=_parse_boolean)
    parser.add_argument(
        "--expected-source", type=_parse_named_source, action="append", default=[]
    )
    parser.add_argument(
        "--expected-source-dirty-entry",
        type=_parse_named_dirty,
        action="append",
        default=[],
    )
    parser.add_argument(
        "--expected-image-digest", type=_parse_image_digest, action="append", default=[]
    )
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
        expected_image_digests = dict(arguments.expected_image_digest)
        named_revisions = dict(arguments.expected_source)
        named_dirty = dict(arguments.expected_source_dirty_entry)
        named_mode = bool(arguments.expected_source or arguments.expected_source_dirty_entry)
        legacy_valid = (
            arguments.expected_source_revision is not None
            and arguments.expected_source_dirty is not None
        )
        named_valid = (
            named_mode
            and len(named_revisions) == len(arguments.expected_source)
            and len(named_dirty) == len(arguments.expected_source_dirty_entry)
            and tuple(named_revisions) == tuple(named_dirty)
            and arguments.expected_source_revision is None
            and arguments.expected_source_dirty is None
        )
        if (
            not (legacy_valid or named_valid)
            or len(arguments.expected_image_digest) != len(_PHASE3_IMAGE_NAMES)
            or set(expected_image_digests) != set(_PHASE3_IMAGE_NAMES)
        ):
            parser.error(
                "semantic acceptance requires matching source revision/dirty "
                "expectations and exact seven Phase 3 image digests"
            )
        common = {
            "rules_path": arguments.rules_path,
            "expected_image_digests": expected_image_digests,
            "require_maximum_score": arguments.require_maximum_score,
        }
        if arguments.semantic_only and named_valid:
            report = inspect_phase3_semantics(
                arguments.run_directory,
                expected_source_revisions=named_revisions,
                expected_source_dirty=named_dirty,
                **common,
            )
        elif arguments.semantic_only:
            report = inspect_phase3_semantics(
                arguments.run_directory,
                expected_source_revision=arguments.expected_source_revision,
                expected_source_dirty=arguments.expected_source_dirty,
                **common,
            )
        elif named_valid:
            report = inspect_phase3_via_container(
                arguments.run_directory,
                expected_source_revisions=named_revisions,
                expected_source_dirty=named_dirty,
                **common,
            )
        else:
            report = inspect_phase3_via_container(
                arguments.run_directory,
                expected_source_revision=arguments.expected_source_revision,
                expected_source_dirty=arguments.expected_source_dirty,
                **common,
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
