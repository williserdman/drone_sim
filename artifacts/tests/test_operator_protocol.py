import json
import os
from dataclasses import asdict
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator

from artifacts import protocol_files
from artifacts.operator_protocol import (
    FlightAbortStatus,
    MissionStartReadiness,
    OperatorProtocol,
    OperatorProtocolConflict,
    OperatorProtocolError,
    PreflightStatus,
    StartAdmission,
    validate_admission_binding,
)
from artifacts.qgc_operator import (
    MAV_RESULT_DENIED,
    MavlinkIdentity,
    QgcCommandEnvelope,
    QgcCommandResponse,
    QgcCommandStatus,
)


RUN_ID = "11111111-1111-4111-8111-111111111111"
OTHER_RUN_ID = "22222222-2222-4222-8222-222222222222"
REQUEST_ID = "33333333-3333-4333-8333-333333333333"
OTHER_REQUEST_ID = "44444444-4444-4444-8444-444444444444"
CONFIG_SHA256 = "a" * 64


def test_start_admission_schema_matches_protocol_domain() -> None:
    schema_path = Path(__file__).parents[1] / "schemas/start-admission.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    admission = StartAdmission(
        RUN_ID,
        CONFIG_SHA256,
        REQUEST_ID,
        "b" * 64,
        50_000_000,
    )

    validator = Draft202012Validator(schema)
    assert list(validator.iter_errors(asdict(admission))) == []
    canonical_nil_uuid = "00000000-0000-0000-0000-000000000000"
    protocol_valid_nil = StartAdmission(
        canonical_nil_uuid,
        CONFIG_SHA256,
        canonical_nil_uuid,
        "b" * 64,
        50_000_000,
    )
    assert list(validator.iter_errors(asdict(protocol_valid_nil))) == []
    invalid = {**asdict(admission), "native_epoch_ns": 50_000_001}
    assert list(validator.iter_errors(invalid))


def test_qgc_command_schema_and_protocol_preserve_nonfinite_rejection(
    run_directory: Path,
) -> None:
    schema_path = Path(__file__).parents[1] / "schemas/qgc-command-status.schema.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    envelope = QgcCommandEnvelope(
        2,
        MavlinkIdentity(255, 190),
        MavlinkIdentity(1, 191),
        31020,
        (float("nan"), float("inf"), float("-inf"), 0, 0, 0, 0),
        10,
    )
    value = QgcCommandStatus(
        RUN_ID,
        CONFIG_SHA256,
        0,
        envelope,
        None,
        None,
        None,
        11,
        QgcCommandResponse(
            31020,
            MavlinkIdentity(255, 190),
            MAV_RESULT_DENIED,
            100,
            "parameters are invalid",
        ),
    )

    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        assert protocol.write_qgc_command_status(value) == value
        assert protocol.read_qgc_command_statuses() == (value,)
    document = json.loads((run_directory / ".status/qgc-command-0000.json").read_text())
    assert document["envelope"]["parameters"][:3] == [
        "NaN",
        "Infinity",
        "-Infinity",
    ]
    assert list(Draft202012Validator(schema).iter_errors(document)) == []


def test_qgc_command_statuses_require_contiguous_immutable_sequence(
    run_directory: Path,
) -> None:
    source = MavlinkIdentity(255, 190)
    envelope = QgcCommandEnvelope(
        2, source, MavlinkIdentity(1, 191), 31020, (1, 0, 0, 0, 0, 0, 0), 10
    )
    skipped = QgcCommandStatus(
        RUN_ID,
        CONFIG_SHA256,
        1,
        envelope,
        None,
        None,
        None,
        11,
        QgcCommandResponse(31020, source, MAV_RESULT_DENIED, 100, "denied"),
    )
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        with pytest.raises(OperatorProtocolConflict, match="contiguous"):
            protocol.write_qgc_command_status(skipped)


def test_qgc_command_status_append_scans_history_once(
    run_directory: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = MavlinkIdentity(255, 190)
    envelope = QgcCommandEnvelope(
        2, source, MavlinkIdentity(1, 191), 31020, (1, 0, 0, 0, 0, 0, 0), 10
    )
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        scans = 0
        read_all = protocol.read_qgc_command_statuses

        def counted_read_all():
            nonlocal scans
            scans += 1
            return read_all()

        monkeypatch.setattr(protocol, "read_qgc_command_statuses", counted_read_all)
        for sequence in range(4):
            protocol.write_qgc_command_status(
                QgcCommandStatus(
                    RUN_ID,
                    CONFIG_SHA256,
                    sequence,
                    envelope,
                    None,
                    None,
                    None,
                    sequence + 11,
                    QgcCommandResponse(
                        31020, source, MAV_RESULT_DENIED, 100, "denied"
                    ),
                )
            )

        next_status = QgcCommandStatus(
            RUN_ID,
            CONFIG_SHA256,
            4,
            envelope,
            None,
            None,
            None,
            15,
            QgcCommandResponse(31020, source, MAV_RESULT_DENIED, 100, "denied"),
        )
        target = run_directory / ".status/qgc-command-0004.json"
        conflicting = asdict(next_status)
        conflicting["response"]["status_text"] = "different"
        target.write_text(json.dumps(conflicting), encoding="utf-8")
        before = target.read_bytes()
        with pytest.raises(OperatorProtocolError, match="conflicts"):
            protocol.write_qgc_command_status(next_status)
        assert target.read_bytes() == before

        target.unlink()
        (run_directory / ".status/qgc-command-0003.json").unlink()
        with pytest.raises(OperatorProtocolConflict, match="predecessor"):
            protocol.write_qgc_command_status(next_status)
        assert scans == 1


@pytest.fixture
def run_directory(tmp_path: Path) -> Path:
    run = tmp_path / RUN_ID
    (run / ".status").mkdir(parents=True)
    (run / ".control").mkdir()
    (run / "configuration").mkdir()
    return run


def test_start_request_equal_retry_is_idempotent_and_conflicting_retry_is_rejected(
    run_directory: Path,
) -> None:
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        request = protocol.write_start_request(REQUEST_ID)

        assert protocol.write_start_request(REQUEST_ID) == request
        assert protocol.read_start_request() == request
        assert len(request.sha256) == 64
        with pytest.raises(OperatorProtocolConflict, match="mission start request"):
            protocol.write_start_request(OTHER_REQUEST_ID)

    assert json.loads((run_directory / ".control/mission-start-request.json").read_text()) == {
        "config_sha256": CONFIG_SHA256,
        "request_id": REQUEST_ID,
        "run_id": RUN_ID,
        "schema_version": 1,
    }


def test_start_request_rejects_foreign_run_and_config_documents(
    run_directory: Path,
) -> None:
    path = run_directory / ".control/mission-start-request.json"
    path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "run_id": OTHER_RUN_ID,
                "request_id": REQUEST_ID,
                "config_sha256": CONFIG_SHA256,
            }
        )
    )
    path.chmod(0o600)

    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        with pytest.raises(OperatorProtocolError, match="run_id"):
            protocol.read_start_request()


def test_start_admission_is_immutable_grid_aligned_and_bound_to_request(
    run_directory: Path,
) -> None:
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        request = protocol.write_start_request(REQUEST_ID)
        admission = protocol.write_start_admission(
            request,
            native_epoch_ns=180_050_000_000,
        )

        assert protocol.read_start_admission() == admission
        assert protocol.start_admission_configuration_record() == (
            "configuration/start-admission.json",
            admission.sha256,
        )
        assert admission.request_sha256 == request.sha256
        assert admission.public_epoch_ns == 0
        assert len(admission.sha256) == 64
        assert protocol.write_start_admission(
            request,
            native_epoch_ns=180_050_000_000,
        ) == admission
        with pytest.raises(OperatorProtocolConflict, match="start admission"):
            protocol.write_start_admission(
                request,
                native_epoch_ns=180_100_000_000,
            )
        with pytest.raises(OperatorProtocolError, match="50 ms"):
            protocol.write_start_admission(request, native_epoch_ns=180_050_000_001)


def test_run_state_admission_binding_requires_exact_operator_digest(
    run_directory: Path,
) -> None:
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        request = protocol.write_start_request(REQUEST_ID)
        admission = protocol.write_start_admission(
            request,
            native_epoch_ns=180_050_000_000,
        )

        assert (
            validate_admission_binding(protocol, "operator", admission.sha256)
            == admission
        )
        with pytest.raises(OperatorProtocolError, match="digest"):
            validate_admission_binding(protocol, "operator", "b" * 64)
        assert validate_admission_binding(protocol, "unattended", "") is None
        with pytest.raises(OperatorProtocolError, match="unattended"):
            validate_admission_binding(protocol, "unattended", admission.sha256)


def test_ready_companion_decision_requires_disarmed_current_vehicle(
    run_directory: Path,
) -> None:
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        request = protocol.write_start_request(REQUEST_ID)
        with pytest.raises(OperatorProtocolError, match="disarmed"):
            protocol.write_start_readiness(
                MissionStartReadiness(
                    RUN_ID,
                    CONFIG_SHA256,
                    REQUEST_ID,
                    request.sha256,
                    1,
                    True,
                    True,
                    True,
                    True,
                    "ready",
                )
            )


def test_preflight_updates_advance_sequence_and_reject_regression(
    run_directory: Path,
) -> None:
    first = PreflightStatus(
        RUN_ID,
        CONFIG_SHA256,
        "gazebo",
        1,
        100,
        native_timestamp_ns=90_000_000_000,
        physics_advancing=True,
        camera_native_timestamp_ns=90_000_000_000,
        ready=True,
        detail="live",
    )
    second = PreflightStatus(
        RUN_ID,
        CONFIG_SHA256,
        "gazebo",
        2,
        200,
        native_timestamp_ns=90_050_000_000,
        physics_advancing=True,
        camera_native_timestamp_ns=90_050_000_000,
        ready=True,
        detail="live",
    )
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        assert protocol.write_preflight(first) == first
        assert protocol.write_preflight(second) == second
        assert protocol.read_preflight("gazebo") == second
        with pytest.raises(OperatorProtocolConflict, match="sequence regressed"):
            protocol.write_preflight(first)


@pytest.mark.parametrize(
    "replacement_stage",
    (
        "opening",
        "reading",
        "post_read_snapshot",
        "decoding",
        "post_decode_snapshot",
    ),
)
def test_read_preflight_retries_one_real_atomic_replacement(
    run_directory: Path, monkeypatch, replacement_stage: str,
) -> None:
    first = PreflightStatus(
        RUN_ID, CONFIG_SHA256, "gazebo", 1, 100,
        native_timestamp_ns=90_000_000_000, physics_advancing=True,
        camera_native_timestamp_ns=90_000_000_000, ready=True, detail="live",
    )
    second = PreflightStatus(
        RUN_ID, CONFIG_SHA256, "gazebo", 2, 200,
        native_timestamp_ns=90_050_000_000, physics_advancing=True,
        camera_native_timestamp_ns=90_050_000_000, ready=True, detail="live",
    )
    target = run_directory / ".status/preflight-gazebo.json"
    replacement = run_directory / ".status/replacement.json"
    replacement.write_text(json.dumps(asdict(second)), encoding="utf-8")
    real_inspect = protocol_files._inspect_existing
    real_read = protocol_files.os.read
    real_loads = protocol_files.json.loads
    replaced = False
    target_inspections = 0

    def inspect_then_replace(directory_fd, name):
        nonlocal replaced, target_inspections
        if name == target.name:
            target_inspections += 1
        if (
            not replaced
            and (
                replacement_stage == "post_read_snapshot" and target_inspections == 2
                or replacement_stage == "post_decode_snapshot"
                and target_inspections == 3
            )
        ):
            os.replace(replacement, target)
            replaced = True
        metadata = real_inspect(directory_fd, name)
        if replacement_stage == "opening" and name == target.name and not replaced:
            os.replace(replacement, target)
            replaced = True
        return metadata

    def read_then_replace(descriptor, size):
        nonlocal replaced
        chunk = real_read(descriptor, size)
        if replacement_stage == "reading" and chunk and not replaced:
            os.replace(replacement, target)
            replaced = True
        return chunk

    def decode_then_replace(*args, **kwargs):
        nonlocal replaced
        document = real_loads(*args, **kwargs)
        if replacement_stage == "decoding" and not replaced:
            os.replace(replacement, target)
            replaced = True
        return document

    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        protocol.write_preflight(first)
        monkeypatch.setattr(protocol_files, "_inspect_existing", inspect_then_replace)
        monkeypatch.setattr(protocol_files.os, "read", read_then_replace)
        monkeypatch.setattr(protocol_files.json, "loads", decode_then_replace)
        assert protocol.read_preflight("gazebo") == second


@pytest.mark.parametrize(
    "replacement_kind", ("malformed", "foreign", "symlink", "hardlink")
)
def test_read_preflight_retry_still_rejects_hostile_replacement(
    run_directory: Path, monkeypatch, replacement_kind: str,
) -> None:
    first = PreflightStatus(
        RUN_ID, CONFIG_SHA256, "gazebo", 1, 100,
        native_timestamp_ns=90_000_000_000, physics_advancing=True,
        camera_native_timestamp_ns=90_000_000_000, ready=True, detail="live",
    )
    target = run_directory / ".status/preflight-gazebo.json"
    replacement = run_directory / ".status/replacement.json"
    replacement_document = asdict(first)
    if replacement_kind == "foreign":
        replacement_document["run_id"] = OTHER_RUN_ID
    replacement.write_text(
        "{" if replacement_kind == "malformed" else json.dumps(replacement_document)
    )
    real_inspect = protocol_files._inspect_existing
    replaced = False

    def inspect_then_replace(directory_fd, name):
        nonlocal replaced
        metadata = real_inspect(directory_fd, name)
        if name == target.name and not replaced:
            if replacement_kind in {"malformed", "foreign"}:
                os.replace(replacement, target)
            else:
                target.unlink()
                if replacement_kind == "symlink":
                    target.symlink_to(replacement.name)
                else:
                    os.link(replacement, target)
            replaced = True
        return metadata

    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        protocol.write_preflight(first)
        monkeypatch.setattr(protocol_files, "_inspect_existing", inspect_then_replace)
        with pytest.raises(OperatorProtocolError):
            protocol.read_preflight("gazebo")


def test_immutable_control_read_does_not_retry_atomic_replacement(
    run_directory: Path, monkeypatch,
) -> None:
    target = run_directory / ".control/mission-start-request.json"
    replacement = run_directory / ".control/replacement.json"
    replacement.write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "config_sha256": CONFIG_SHA256,
                "request_id": OTHER_REQUEST_ID,
                "schema_version": 1,
            }
        ),
        encoding="utf-8",
    )
    real_inspect = protocol_files._inspect_existing
    replaced = False

    def inspect_then_replace(directory_fd, name):
        nonlocal replaced
        metadata = real_inspect(directory_fd, name)
        if name == target.name and not replaced:
            os.replace(replacement, target)
            replaced = True
        return metadata

    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        protocol.write_start_request(REQUEST_ID)
        monkeypatch.setattr(protocol_files, "_inspect_existing", inspect_then_replace)
        with pytest.raises(OperatorProtocolError, match="changed while opening"):
            protocol.read_start_request()


def test_flight_abort_progress_is_append_only_and_admission_bound(
    run_directory: Path,
) -> None:
    with OperatorProtocol(run_directory, RUN_ID, CONFIG_SHA256) as protocol:
        request = protocol.write_start_request(REQUEST_ID)
        admission = protocol.write_start_admission(
            request,
            native_epoch_ns=180_050_000_000,
        )
        abort = protocol.write_flight_abort_request(
            OTHER_REQUEST_ID,
            admission.sha256,
            "operator_abort",
        )
        first = FlightAbortStatus(
            RUN_ID,
            CONFIG_SHA256,
            abort.request_id,
            admission.sha256,
            0,
            "accepted",
            100,
            1_000_000_000,
            "abort accepted",
        )
        second = FlightAbortStatus(
            RUN_ID,
            CONFIG_SHA256,
            abort.request_id,
            admission.sha256,
            1,
            "recovering",
            200,
            1_050_000_000,
            "LAND requested",
        )

        assert protocol.write_flight_abort_status(first) == first
        assert protocol.write_flight_abort_status(first) == first
        assert protocol.write_flight_abort_status(second) == second
        assert protocol.read_flight_abort_statuses() == (first, second)
        assert protocol.read_latest_flight_abort_status() == second

        skipped = FlightAbortStatus(
            RUN_ID,
            CONFIG_SHA256,
            abort.request_id,
            admission.sha256,
            3,
            "failed",
            300,
            None,
            "timeout",
        )
        with pytest.raises(OperatorProtocolConflict, match="contiguous"):
            protocol.write_flight_abort_status(skipped)
