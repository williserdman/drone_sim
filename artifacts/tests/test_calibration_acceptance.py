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


@pytest.mark.parametrize('role,version,expected', [(False,2,False),(True,2,True),(False,1,True)])
def test_reload_hover_role_does_not_apply_to_every_calibrated_mission(role, version, expected):
    config = {'mission':'configured', 'calibration_json': {'profile': {'schema_version':version}},
              'calibration_validation':role}
    assert acceptance._requires_reload_validation(config) is expected


def test_version_two_profile_checks_actual_consumer_model(tmp_path):
    profile = {'schema_version':2,'physical_model_sha256':'a'*64,
               'consumer_vehicle':'iris_moving_pad','consumer_model_resource':'gazebo/resources/models/iris_moving_pad/model.sdf',
               'consumer_model_sha256':'b'*64,'image_digests':{'gazebo':'c'*64}}
    profile['profile_sha256'] = hashlib.sha256(json.dumps(profile, sort_keys=True, separators=(',',':')).encode()).hexdigest()
    (tmp_path / 'gazebo').mkdir()
    (tmp_path / 'gazebo/server.log').write_text(json.dumps({'resource_sha256s':[
        ['models/iris_flight/model.sdf','a'*64], ['models/iris_moving_pad/model.sdf','b'*64]]})+'\n')
    acceptance._validate_calibration_profile(tmp_path,profile,{'image_digests':[{'name':'gazebo','digest':'c'*64}]})


def test_version_two_profile_rejects_wrong_consumer_even_if_source_model_matches(tmp_path):
    profile = {'schema_version':2,'physical_model_sha256':'a'*64,
               'consumer_vehicle':'iris_moving_pad','consumer_model_resource':'gazebo/resources/models/iris_moving_pad/model.sdf',
               'consumer_model_sha256':'b'*64,'image_digests':{'gazebo':'c'*64}}
    profile['profile_sha256'] = hashlib.sha256(json.dumps(profile, sort_keys=True, separators=(',',':')).encode()).hexdigest()
    (tmp_path / 'gazebo').mkdir()
    (tmp_path / 'gazebo/server.log').write_text(json.dumps({'resource_sha256s':[
        ['models/iris_flight/model.sdf','a'*64], ['models/iris_moving_pad/model.sdf','f'*64]]})+'\n')
    with pytest.raises(acceptance.BundleAcceptanceError, match='aircraft'):
        acceptance._validate_calibration_profile(tmp_path,profile,{'image_digests':[{'name':'gazebo','digest':'c'*64}]})


def test_hover_uses_its_actual_phase_grammar():
    documents = logs()
    documents['companion'] = [{'event':'hover_phase','fields':{'phase':phase},'sim_timestamp':stamp}
        for phase,stamp in [('HOVERING',10),('WAIT_LAND',20),('COMPLETE',30)]]
    documents['companion'].append({'event':'mission_finished','fields':{'outcome':'LANDED'},'sim_timestamp':30})
    acceptance._validate_production_log_evidence(documents, ruleset_id='descent_v1',mission='hover_roll')


@pytest.mark.parametrize('first_event,fields', [
    ('operation_started', {'tool':'arm'}),
    ('command_issued', {'command':'SET_GUIDED'}),
    ('hover_phase', {'phase':'WAIT_GUIDED'}),
    ('autotune_phase', {'phase':'WAIT_GUIDED'}),
    ('mission_event', {'state':'STARTED'}),
])
def test_post_command_readback_cannot_satisfy_pre_arm_gate(tmp_path, first_event, fields):
    from artifacts.calibration import CALIBRATION_PARAMETERS, write_calibration_parameters
    run_id = '00000000-0000-4000-8000-000000000707'
    gains = {name: 0.1 for name in CALIBRATION_PARAMETERS}
    gains['ATC_RAT_YAW_I'] = 0.01
    artifact = write_calibration_parameters(tmp_path, run_id, gains)
    configuration = tmp_path / 'configuration'
    configuration.mkdir()
    (configuration / 'calibration.parm').write_bytes(artifact.read_bytes())
    manifest = json.dumps({'run_id':run_id}).encode()
    (configuration / 'calibration-manifest.json').write_bytes(manifest)
    calibration = {'source_run_id':run_id,'gains':gains,
        'source_artifact_sha256':hashlib.sha256(artifact.read_bytes()).hexdigest(),
        'source_manifest_sha256':hashlib.sha256(manifest).hexdigest(),
        'profile':{'schema_version':2,'effective_baseline_parameters':{'PLND_LAG':0.04}}}
    verified = {'event':'calibration_parameters_verified','fields':{
        'stage':'pre_arm','parameters':{**gains,'PLND_LAG':0.04}}}
    acceptance._validate_calibration_inputs(tmp_path, calibration, [verified, {'event':first_event,'fields':fields}])
    with pytest.raises(acceptance.BundleAcceptanceError,match='pre-arm'):
        acceptance._validate_calibration_inputs(tmp_path, calibration, [{'event':first_event,'fields':fields},verified])


def operator_fixture(tmp_path):
    run_id = '00000000-0000-4000-8000-000000000707'
    plan={'schema_version':1,'steps':[{'tool':'wait_for_state','args':{'armed':True,'mode':'GUIDED'},'timeout_sim_s':60}]}
    companion=[{'event':'operation_started','fields':{'operation_id':'1',**plan['steps'][0]},'sim_timestamp':0.1},
               {'event':'operation_finished','fields':{'operation_id':'1','tool':'wait_for_state','state':'succeeded','error':''},'sim_timestamp':0.7}]
    events=[('operator_command','SET_GUIDED'),('operator_acknowledgement','SET_GUIDED'),
            ('operator_observed_guided',None),('operator_command','ARM'),
            ('operator_acknowledgement','ARM'),('operator_observed_armed',None)]
    rows=[{'run_id':run_id,'event':event,'sim_timestamp_ns':200_000_000+index*50_000_000,
           **({'command':command} if command else {})} for index,(event,command) in enumerate(events)]
    target=tmp_path/'logs/docker/operator.jsonl'
    target.parent.mkdir(parents=True)
    target.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    return {'run_id':run_id,'mission_plan':plan},companion,target,rows


def test_operator_evidence_requires_ordered_ack_and_observed_states(tmp_path):
    config,companion,target,rows=operator_fixture(tmp_path)
    acceptance._validate_operator_evidence(tmp_path,config,companion)
    rows[2],rows[3]=rows[3],rows[2]
    target.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(acceptance.BundleAcceptanceError,match='operator'):
        acceptance._validate_operator_evidence(tmp_path,config,companion)


def test_operator_cannot_command_before_started_wait(tmp_path):
    config,companion,target,rows=operator_fixture(tmp_path)
    rows[0]['sim_timestamp_ns']=50_000_000
    target.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(acceptance.BundleAcceptanceError,match='operator'):
        acceptance._validate_operator_evidence(tmp_path,config,companion)
