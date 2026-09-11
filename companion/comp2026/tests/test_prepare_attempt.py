import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

from drone.control.attempt_setup import (
    AttemptLedger,
    LedgerError,
    PreparationError,
    initialize_ledger,
    load_deployment_profile_bytes,
    load_prepared_session_bytes,
    load_prepared_session,
    prepare_attempt,
)
from drone.control.mission_supervisor import (
    CommandEnvelope,
    CommandRejected,
    MissionSupervisor,
)


COMMANDS = {
    "FM1": 31000,
    "FM2": 31001,
    "FM3": 31002,
    "UPDATE_WA": 31003,
    "UPDATE_WM1": 31004,
    "UPDATE_WM2": 31005,
    "UPDATE_WM3": 31006,
    "UPDATE_WM4": 31007,
    "UPDATE_WM5": 31008,
    "UPDATE_WM6": 31009,
    "UPDATE_L": 31010,
    "UPDATE_TARGET": 31011,
    "CLEAR_PICKUPS": 31012,
    "CLEAR_ALL": 31013,
    "ABORT_AND_RECOVER": 31015,
}

GATES = {
    "identity": True,
    "protocol": True,
    "firmware": True,
    "qgc_fixed_params": True,
    "qgc_retry_behavior": True,
    "rc_mapping_and_freshness": True,
    "command_registry": True,
    "telemetry_rates": True,
    "sensor_configuration": True,
    "positive_preflight_evidence": True,
    "fc_failsafe": True,
    "rc_precedence": True,
}


def profile_data():
    return {
        "schema_version": 1,
        "profile_id": "bench-aircraft-2026-09-06",
        "verified_for_live_use": True,
        "identity": {
            "source_system": 200,
            "source_component": 190,
            "target_system": 1,
            "target_component": 191,
            "flight_controller_system": 1,
            "flight_controller_component": 1,
        },
        "software": {
            "mavlink_dialect": "ardupilotmega",
            "mavlink_wire_protocol": "2.0",
            "firmware": "ArduCopter 4.5.7",
            "qgc_version": "QGroundControl 4.4.3-test+abcdef0",
        },
        "rc": {
            "protocol": "test-protocol",
            "mode_channel": 7,
            "freshness_seconds": 0.5,
            "mode_mapping": {
                "STABILIZE": [900, 1200],
                "GUIDED": [1400, 1600],
                "LOITER": [1800, 2100],
            },
        },
        "observation": {
            "decoder_contract_version": 1,
            "decoder_contract_evidence": "sha256:" + "b" * 64,
            "decoder_contract_reference": (
                "ArduPilot/Copter-4.5.7:GCS_Mavlink.cpp:70-86;"
                "GCS_Copter.cpp:41-48"
            ),
            "expected_heartbeat_type": 2,
            "expected_heartbeat_autopilot": 3,
            "mode_mapping": {
                "0": "STABILIZE",
                "4": "GUIDED",
                "5": "LOITER",
            },
            "failsafe_active_system_statuses": [5],
            "failsafe_clear_system_statuses": [3, 4],
            "freshness_seconds": {
                "heartbeat": 1.0,
                "mode": 1.0,
                "location": 1.0,
                "velocity": 1.0,
                "attitude": 1.0,
                "landed_state": 1.0,
                "armed": 1.0,
                "home": 1.0,
                "rc_input": 0.5,
                "range": 1.0,
                "failsafe": 1.0,
            },
            "sys_status_rc_health_freshness_seconds": 0.4,
            "telemetry_requests": [
                {"message_id": 0, "interval_us": 500000},
                {"message_id": 1, "interval_us": 200000},
                {"message_id": 30, "interval_us": 100000},
                {"message_id": 33, "interval_us": 200000},
                {"message_id": 65, "interval_us": 200000},
                {"message_id": 132, "interval_us": 100000},
                {"message_id": 242, "interval_us": 500000},
                {"message_id": 245, "interval_us": 200000},
            ],
        },
        "command_registry": {
            "commands": dict(COMMANDS),
            "abort_31015_available": True,
            "collision_check_evidence": "sha256:" + "a" * 64,
        },
        "safety_gates": dict(GATES),
    }


def write_profile(path, data=None):
    path.write_text(json.dumps(profile_data() if data is None else data))


def paths(tmp_path):
    return (
        tmp_path / "deployment.json",
        tmp_path / "attempt-ledger.json",
        tmp_path / "listener-session.json",
        tmp_path / "qgc-actions.json",
    )


def test_preparation_api_imports_from_an_ordinary_package(capsys):
    import drone.control.attempt_setup as setup

    assert setup.prepare_attempt is prepare_attempt
    assert capsys.readouterr() == ("", "")


def test_offline_script_preserves_preparation_reexports(capsys):
    script_path = Path(__file__).parents[1] / "src" / "gc" / "prepare_attempt.py"
    specification = importlib.util.spec_from_file_location(
        "offline_prepare_attempt", script_path
    )
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)

    assert module.prepare_attempt is prepare_attempt
    assert module.COMMANDS == COMMANDS
    assert module.SUPPORTED_MAVLINK_DIALECTS == frozenset(("ardupilotmega",))
    assert capsys.readouterr() == ("", "")


def test_ledger_initialization_is_explicit_and_never_overwrites(tmp_path):
    ledger_path = tmp_path / "ledger.json"
    initialize_ledger(ledger_path)
    original = ledger_path.read_bytes()

    with pytest.raises(FileExistsError):
        initialize_ledger(ledger_path)
    assert ledger_path.read_bytes() == original
    assert json.loads(original) == {
        "schema_version": 1,
        "last_prepared_attempt_id": 0,
        "consumed_through_attempt_id": 0,
    }


def test_prepare_requires_existing_valid_ledger_and_explicit_ground_ack(tmp_path):
    profile, ledger, session, actions = paths(tmp_path)
    write_profile(profile)

    with pytest.raises(PreparationError, match="ground-preparation acknowledgement"):
        prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=False)
    with pytest.raises(LedgerError, match="does not exist"):
        prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)

    ledger.write_text("not json")
    with pytest.raises(LedgerError, match="corrupt"):
        prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)


def test_prepare_rejects_overlapping_input_and_output_paths(tmp_path):
    profile, ledger, _session, actions = paths(tmp_path)
    write_profile(profile)
    initialize_ledger(ledger)
    before = ledger.read_bytes()

    with pytest.raises(PreparationError, match="distinct"):
        prepare_attempt(
            profile, ledger, ledger, actions, acknowledge_on_ground=True
        )
    assert ledger.read_bytes() == before


def test_prepare_emits_matched_qgc_and_session_generation(tmp_path):
    profile, ledger, session, actions = paths(tmp_path)
    write_profile(profile)
    initialize_ledger(ledger)

    prepared = prepare_attempt(
        profile, ledger, session, actions, acknowledge_on_ground=True
    )
    loaded = load_prepared_session(session, actions, profile)
    action_data = json.loads(actions.read_text())

    assert prepared.attempt_id == loaded.attempt_id == 1
    assert prepared.generation == loaded.generation
    assert action_data["version"] == 1
    assert action_data["fileType"] == "MavlinkActions"
    assert loaded.generation in action_data["title"]
    assert "generation" not in action_data
    assert "attemptId" not in action_data
    assert {action["mavCmd"] for action in action_data["actions"]} == set(
        COMMANDS.values()
    )
    assert all(action["compId"] == 191 for action in action_data["actions"])
    assert all(action["param1"] == 1 for action in action_data["actions"])
    assert all(
        action[f"param{number}"] == 0 for action in action_data["actions"]
        for number in range(2, 8)
    )
    assert all("target system 1" in action["description"] for action in action_data["actions"])
    assert all("targetSystem" not in action for action in action_data["actions"])
    assert loaded.actions_sha256 == hashlib.sha256(actions.read_bytes()).hexdigest()


def test_byte_loaders_reject_boolean_integer_fields(tmp_path):
    profile, ledger, session, actions = paths(tmp_path)
    write_profile(profile)
    initialize_ledger(ledger)
    prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)
    deployment = load_deployment_profile_bytes(profile.read_bytes())

    profile_document = json.loads(profile.read_text())
    profile_document["schema_version"] = True
    with pytest.raises(PreparationError):
        load_deployment_profile_bytes(json.dumps(profile_document).encode())

    session_data = json.loads(session.read_text())
    session_data["schema_version"] = True
    with pytest.raises(PreparationError):
        load_prepared_session_bytes(
            json.dumps(session_data).encode(), actions.read_bytes(), deployment
        )

    session_data = json.loads(session.read_text())
    session_data["required_source"]["system"] = True
    with pytest.raises(PreparationError):
        load_prepared_session_bytes(
            json.dumps(session_data).encode(), actions.read_bytes(), deployment
        )

    actions_data = json.loads(actions.read_text())
    actions_data["version"] = True
    with pytest.raises(PreparationError):
        load_prepared_session_bytes(
            session.read_bytes(), json.dumps(actions_data).encode(), deployment
        )


def test_byte_loaders_reject_ambiguous_json(tmp_path):
    profile, ledger, session, actions = paths(tmp_path)
    write_profile(profile)
    initialize_ledger(ledger)
    prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)
    deployment = load_deployment_profile_bytes(profile.read_bytes())

    duplicate_session = session.read_bytes().replace(
        b'"schema_version": 1', b'"schema_version": 2, "schema_version": 1'
    )
    with pytest.raises(PreparationError, match="duplicate"):
        load_prepared_session_bytes(
            duplicate_session, actions.read_bytes(), deployment
        )

    nonstandard_actions = actions.read_bytes().replace(
        b'"param2": 0', b'"param2": NaN'
    )
    with pytest.raises(PreparationError, match="non-standard"):
        load_prepared_session_bytes(
            session.read_bytes(), nonstandard_actions, deployment
        )

    actions_data = json.loads(actions.read_text())
    actions_data["actions"][0]["param2"] = False
    with pytest.raises(PreparationError):
        load_prepared_session_bytes(
            session.read_bytes(), json.dumps(actions_data).encode(), deployment
        )


def test_prepare_is_monotonic_and_old_unconsumed_output_becomes_stale(tmp_path):
    profile, ledger, session, actions = paths(tmp_path)
    write_profile(profile)
    initialize_ledger(ledger)
    first = prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)
    old_session = tmp_path / "old-session.json"
    old_actions = tmp_path / "old-actions.json"
    old_session.write_bytes(session.read_bytes())
    old_actions.write_bytes(actions.read_bytes())

    second = prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)
    assert (first.attempt_id, second.attempt_id) == (1, 2)
    with pytest.raises(PreparationError, match="stale attempt"):
        load_prepared_session(old_session, old_actions, profile, ledger_path=ledger)


def test_consumption_is_durable_and_replay_fails_after_restart(tmp_path):
    profile, ledger_path, session, actions = paths(tmp_path)
    write_profile(profile)
    initialize_ledger(ledger_path)
    prepared = prepare_attempt(
        profile, ledger_path, session, actions, acknowledge_on_ground=True
    )

    AttemptLedger(ledger_path).consume(prepared.attempt_id)
    persisted = json.loads(ledger_path.read_text())
    assert persisted["consumed_through_attempt_id"] == 1
    with pytest.raises(LedgerError, match="already consumed"):
        AttemptLedger(ledger_path).consume(prepared.attempt_id)


def test_symlink_alias_uses_one_canonical_ledger_and_cannot_replay(tmp_path):
    real_ledger = tmp_path / "real-ledger.json"
    alias_ledger = tmp_path / "alias-ledger.json"
    initialize_ledger(real_ledger)
    alias_ledger.symlink_to(real_ledger)

    with AttemptLedger(alias_ledger).prepare_next() as attempt_id:
        assert attempt_id == 1

    assert alias_ledger.is_symlink()
    assert json.loads(real_ledger.read_text())["last_prepared_attempt_id"] == 1
    AttemptLedger(real_ledger).consume(1)
    with pytest.raises(LedgerError, match="already consumed"):
        AttemptLedger(alias_ledger).consume(1)


def test_alias_ledger_rejects_canonical_lock_output_before_mutation(tmp_path):
    """Break caught: an output must never replace the lock used by the ledger."""
    profile = tmp_path / "profile.json"
    real_ledger = tmp_path / "real-ledger.json"
    alias_ledger = tmp_path / "alias-ledger.json"
    session = tmp_path / "session.json"
    canonical_lock = tmp_path / "real-ledger.json.lock"
    write_profile(profile)
    initialize_ledger(real_ledger)
    alias_ledger.symlink_to(real_ledger)
    canonical_lock.write_bytes(b"existing lock sentinel")
    before_ledger = real_ledger.read_bytes()
    before_lock = canonical_lock.read_bytes()

    with pytest.raises(PreparationError, match="paths must be distinct"):
        prepare_attempt(
            profile,
            alias_ledger,
            session,
            canonical_lock,
            acknowledge_on_ground=True,
        )

    assert real_ledger.read_bytes() == before_ledger
    assert canonical_lock.read_bytes() == before_lock
    assert not session.exists()


def test_symlinked_ledger_lock_target_cannot_be_an_output(tmp_path):
    """Break caught: preparation must not replace a lock symlink's target."""
    profile = tmp_path / "profile.json"
    ledger = tmp_path / "ledger.json"
    lock = tmp_path / "ledger.json.lock"
    lock_target = tmp_path / "actual-lock"
    session = tmp_path / "session.json"
    write_profile(profile)
    initialize_ledger(ledger)
    lock_target.write_bytes(b"existing lock target")
    lock.unlink()
    lock.symlink_to(lock_target)
    before_ledger = ledger.read_bytes()
    before_target = lock_target.read_bytes()

    with pytest.raises(PreparationError, match="paths must be distinct"):
        prepare_attempt(
            profile,
            ledger,
            session,
            lock_target,
            acknowledge_on_ground=True,
        )

    assert ledger.read_bytes() == before_ledger
    assert lock.is_symlink()
    assert lock.resolve() == lock_target
    assert lock_target.read_bytes() == before_target
    assert not session.exists()


def test_supervisor_persists_consumption_before_fm1_admission(tmp_path):
    profile, ledger_path, session, actions = paths(tmp_path)
    write_profile(profile)
    initialize_ledger(ledger_path)
    prepared = prepare_attempt(
        profile, ledger_path, session, actions, acknowledge_on_ground=True
    )
    request = CommandEnvelope(
        200, 190, 1, 191, 31000, prepared.attempt_id,
        (float(prepared.attempt_id), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0,
    )
    first_process = MissionSupervisor(
        prepared.attempt_id,
        admission_check=lambda envelope: None,
        attempt_consumer=AttemptLedger(ledger_path).consume,
        permission_check=lambda: None,
    )
    assert first_process.admit(request) is True
    assert json.loads(ledger_path.read_text())["consumed_through_attempt_id"] == 1

    restarted_process = MissionSupervisor(
        prepared.attempt_id,
        admission_check=lambda envelope: None,
        attempt_consumer=AttemptLedger(ledger_path).consume,
        permission_check=lambda: None,
    )
    with pytest.raises(CommandRejected, match="already consumed"):
        restarted_process.admit(request)
    assert restarted_process.status(31000) is None


def test_canonical_actions_are_a_denied_token_template_without_abort():
    canonical = Path(__file__).parents[1] / "src" / "gc" / "custom_missions.json"
    data = json.loads(canonical.read_text())

    assert data["version"] == 1
    assert data["fileType"] == "MavlinkActions"
    assert "TEMPLATE" in data["title"]
    commands = {action["mavCmd"] for action in data["actions"]}
    assert commands == set(range(31000, 31014))
    assert 31014 not in commands
    assert 31015 not in commands
    assert all(action["param1"] == 0 for action in data["actions"])
    assert all(
        action[f"param{number}"] == 0 for action in data["actions"]
        for number in range(2, 8)
    )
    assert all("cannot authorize flight" in action["description"] for action in data["actions"])


def test_consume_rejects_unprepared_or_superseded_attempt(tmp_path):
    profile, ledger_path, session, actions = paths(tmp_path)
    write_profile(profile)
    initialize_ledger(ledger_path)
    prepare_attempt(profile, ledger_path, session, actions, acknowledge_on_ground=True)
    prepare_attempt(profile, ledger_path, session, actions, acknowledge_on_ground=True)

    with pytest.raises(LedgerError, match="current prepared attempt"):
        AttemptLedger(ledger_path).consume(1)
    with pytest.raises(LedgerError, match="current prepared attempt"):
        AttemptLedger(ledger_path).consume(3)


def test_unknown_profile_values_and_open_gates_fail_closed(tmp_path):
    profile, ledger, session, actions = paths(tmp_path)
    initialize_ledger(ledger)
    data = profile_data()
    data["software"]["firmware"] = "UNKNOWN"
    write_profile(profile, data)
    with pytest.raises(PreparationError, match="firmware"):
        prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)

    data = profile_data()
    data["safety_gates"]["telemetry_rates"] = False
    write_profile(profile, data)
    with pytest.raises(PreparationError, match="telemetry_rates"):
        prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("firmware", "ArduCopter 4.x.x", "firmware"),
        ("firmware", "ArduCopter exact-test-build", "firmware"),
        ("qgc_version", "master", "qgc_version"),
        ("qgc_version", "QGC latest", "qgc_version"),
        ("mavlink_wire_protocol", "3.0", "wire protocol"),
        ("mavlink_wire_protocol", "1.0", "MAVLink 2"),
        ("mavlink_dialect", "not-a-supported-dialect", "dialect"),
    ],
)
def test_unsupported_or_wildcard_software_is_rejected(
    tmp_path, field, value, message
):
    profile, ledger, session, actions = paths(tmp_path)
    initialize_ledger(ledger)
    data = profile_data()
    data["software"][field] = value
    write_profile(profile, data)

    with pytest.raises(PreparationError, match=message):
        prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)


def test_abort_collision_check_is_required(tmp_path):
    profile, ledger, session, actions = paths(tmp_path)
    initialize_ledger(ledger)
    data = profile_data()
    data["command_registry"]["abort_31015_available"] = False
    write_profile(profile, data)

    with pytest.raises(PreparationError, match="31015"):
        prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)


def test_mixed_generation_or_modified_action_file_fails_closed(tmp_path):
    profile, ledger, session, actions = paths(tmp_path)
    write_profile(profile)
    initialize_ledger(ledger)
    prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)
    action_data = json.loads(actions.read_text())
    action_data["title"] = "interrupted-other-generation"
    actions.write_text(json.dumps(action_data))

    with pytest.raises(PreparationError, match="hash|generation"):
        load_prepared_session(session, actions, profile)


def test_exhausted_ledger_does_not_wrap_or_reset(tmp_path):
    profile, ledger, session, actions = paths(tmp_path)
    write_profile(profile)
    ledger.write_text(json.dumps({
        "schema_version": 1,
        "last_prepared_attempt_id": 16_777_215,
        "consumed_through_attempt_id": 16_777_215,
    }))

    with pytest.raises(LedgerError, match="exhausted"):
        prepare_attempt(profile, ledger, session, actions, acknowledge_on_ground=True)
    assert json.loads(ledger.read_text())["last_prepared_attempt_id"] == 16_777_215


def test_failed_atomic_consumption_preserves_previous_ledger(tmp_path, monkeypatch):
    profile, ledger_path, session, actions = paths(tmp_path)
    write_profile(profile)
    initialize_ledger(ledger_path)
    prepared = prepare_attempt(
        profile, ledger_path, session, actions, acknowledge_on_ground=True
    )
    before = ledger_path.read_bytes()

    from drone.control import mission_supervisor as module

    def fail_replace(_source, _destination):
        raise OSError("disk failure")

    monkeypatch.setattr(module.os, "replace", fail_replace)
    with pytest.raises(LedgerError, match="disk failure"):
        AttemptLedger(ledger_path).consume(prepared.attempt_id)
    assert ledger_path.read_bytes() == before
