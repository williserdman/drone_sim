import hashlib
import json
import math

import pytest

from artifacts import acceptance
from artifacts._adapters.rosbag import GroundTruthEvidence


PLAN = {"steps": [
    {"tool": "set_mode", "args": {"mode": "GUIDED"}},
    {"tool": "arm", "args": {}},
    {"tool": "takeoff", "args": {"altitude_m": 5.0}},
    {"tool": "hold", "args": {"duration_sim_s": 10.0}},
    {"tool": "land", "args": {}},
]}


def logs():
    def row(event, fields=None, stamp=0):
        return {"event": event, "fields": fields or {}, "sim_timestamp": stamp}

    companion = [row("heartbeat_observed")]
    for index, step in enumerate(PLAN["steps"]):
        companion.extend([
            row("operation_started", {"operation_id": str(index + 1), **step}, index * 12),
            row("operation_finished", {"operation_id": str(index + 1),
                "tool": step["tool"], "state": "succeeded", "error": ""}, index * 12 + 10),
        ])
    companion.append(row("mission_finished", {"outcome": "LANDED"}, 60))
    return {
        "companion": companion,
        "ardupilot_sitl": [row("starting"), row("ready", {
            "json_exchange": True, "mavlink_listening": True}), row("stopped")],
        "gazebo": [row("runtime_started"), *[
            row("runtime_action", {"action": action}) for action in (
                "PublishGazeboReady", "SetPaused", "ActivateOutput", "SetPaused",
                "WriteSourceFinished", "BeginFinalization", "StopServer", "WriteQuiescence")
        ], row("runtime_quiescent")],
    }


def validate(documents):
    acceptance._validate_production_log_evidence(
        documents, ruleset_id="descent_v1", mission="configured", mission_plan=PLAN)


def test_configured_mission_accepts_observed_operations_without_legacy_commands():
    validate(logs())


@pytest.mark.parametrize("mutation", ["missing", "failed", "wrong_args", "out_of_order"])
def test_configured_acceptance_requires_exact_successful_sequence(mutation):
    documents = logs()
    companion = documents["companion"]
    if mutation == "missing":
        companion.pop(4)
    elif mutation == "failed":
        companion[4]["fields"]["state"] = "failed"
    elif mutation == "wrong_args":
        companion[5]["fields"]["args"] = {"altitude_m": 1.5}
    else:
        companion[3], companion[4] = companion[4], companion[3]
    with pytest.raises(acceptance.BundleAcceptanceError, match="configured"):
        validate(documents)


def hover_samples(*, speed=0.0, roll=0.0, altitude=5.195):
    return tuple(GroundTruthEvidence(
        sim_timestamp_ns=index * 50_000_000,
        vehicle_id="iris",
        position_xyz=(0.0, 0.0, altitude if index else 0.195),
        orientation_xyzw=(math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2)),
        linear_velocity_xyz=(speed, 0.0, 0.0),
        angular_velocity_xyz=(0.0, 0.0, 0.0),
        in_contact=index == 0,
    ) for index in range(401))


def test_validation_accepts_five_continuous_seconds_of_recorded_stable_hover():
    acceptance._validate_calibration_hover(
        hover_samples(), hold_start_ns=5_000_000_000,
        hold_end_ns=15_000_000_000, altitude_m=5.0)


@pytest.mark.parametrize("change", ["speed", "tilt", "altitude", "gap"])
def test_validation_rejects_unstable_hover_despite_safe_landing(change):
    samples = hover_samples(**{
        "speed": {"speed": 0.21}, "tilt": {"roll": math.radians(5.1)},
        "altitude": {"altitude": 4.6}, "gap": {},
    }[change])
    if change == "gap":
        samples = tuple(sample for index, sample in enumerate(samples) if index % 40)
    with pytest.raises(acceptance.BundleAcceptanceError, match="hover"):
        acceptance._validate_calibration_hover(
            samples, hold_start_ns=5_000_000_000,
            hold_end_ns=15_000_000_000, altitude_m=5.0)


@pytest.mark.parametrize("missing", ["testing", "ack", "disarm", "land"])
def test_calibration_requires_gain_activation_and_native_landing_evidence(missing):
    documents = logs()
    companion = documents["companion"]
    statuses = ["AutoTune: Success", "AutoTune: Pilot Testing gains for Roll Pitch Yaw(E)",
                "AutoTune: Saved gains for Roll Pitch Yaw(E)"]
    if missing == "testing":
        statuses.pop(1)
    companion.extend({"event": "ardupilot_status_text", "fields": {"text": text}}
                     for text in statuses)
    companion.extend({"event": event, "fields": {}} for event in (
        "autotune_aux_ack", "autotune_disarmed")
        if event != {"ack": "autotune_aux_ack", "disarm": "autotune_disarmed"}.get(missing))
    companion.extend({"event": "autotune_phase", "fields": {"phase": phase}}
                     for phase in (["COMPLETE"] if missing == "land" else ["LANDING", "COMPLETE"]))
    with pytest.raises(acceptance.BundleAcceptanceError, match="AutoTune"):
        acceptance._validate_production_log_evidence(
            documents, ruleset_id="calibration_v1", mission="autotune")


def test_imported_parameter_bytes_must_match_frozen_source_digest(tmp_path):
    directory = tmp_path / "configuration"
    directory.mkdir()
    (directory / "calibration.parm").write_text("changed")
    with pytest.raises(acceptance.BundleAcceptanceError, match="checksum"):
        acceptance._validate_calibration_inputs(tmp_path, {
            "source_artifact_sha256": hashlib.sha256(b"original").hexdigest()}, [])


def test_calibration_profile_rejects_actual_model_different_from_frozen_input(tmp_path):
    profile = {"physical_model_sha256": "a" * 64, "image_digests": {"gazebo": "c" * 64}}
    profile["profile_sha256"] = hashlib.sha256(
        json.dumps(profile, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    (tmp_path / "gazebo").mkdir()
    (tmp_path / "gazebo/server.log").write_text(json.dumps({
        "resource_sha256s": [["models/iris_flight/model.sdf", "b" * 64]]}) + "\n")
    with pytest.raises(acceptance.BundleAcceptanceError, match="aircraft"):
        acceptance._validate_calibration_profile(tmp_path, profile,
            {"image_digests": [{"name": "gazebo", "digest": "c" * 64}]})
