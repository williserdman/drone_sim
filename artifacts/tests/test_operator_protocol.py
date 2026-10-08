import json
from dataclasses import asdict
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator

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
    invalid = {**asdict(admission), "native_epoch_ns": 50_000_001}
    assert list(validator.iter_errors(invalid))


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
