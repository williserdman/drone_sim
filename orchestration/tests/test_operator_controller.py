from dataclasses import replace
from pathlib import Path
import time
from uuid import uuid4

import pytest

from artifacts.operator_protocol import OperatorProtocol, OperatorProtocolConflict, PreflightStatus
from orchestration.config import resolve_run_config, write_resolved_config
from orchestration.controller import ControllerError, RunController
from orchestration.status_store import OperatorStatus, StatusStore


ROOT = Path(__file__).resolve().parents[2]


def prepared(tmp_path):
    config = resolve_run_config(ROOT / "config/configured-descent-run.json",
                                launch_mode="operator", output_root=tmp_path)
    store = StatusStore(tmp_path)
    directory = store.allocate(config.run_id)
    write_resolved_config(directory, config)
    store.write_operator_status(OperatorStatus(config.run_id, "PREPARED", "", None))
    with OperatorProtocol(directory, config.run_id, config.config_sha256) as protocol:
        protocol.write_preflight(PreflightStatus(config.run_id, config.config_sha256, "gazebo", 1,
            time.time_ns(), native_timestamp_ns=90_000_000_000, physics_advancing=True,
            camera_native_timestamp_ns=90_000_000_000, ready=True, detail="live"))
        protocol.write_preflight(PreflightStatus(config.run_id, config.config_sha256, "companion", 1,
            time.time_ns(), heartbeat_live=True, prearm_checks_healthy=True, armed=False, ready=True, detail="ready"))
    return config, store, directory


def test_explicit_start_is_idempotent_and_keeps_frozen_configuration(tmp_path):
    config, store, directory = prepared(tmp_path)
    frozen = (directory / "configuration/run.json").read_bytes()
    controller = RunController(project_directory=ROOT)
    request = str(uuid4())
    first = controller.start_mission(config.run_id, tmp_path, request_id=request)
    assert first.state == "STARTING_MISSION"
    store.write_operator_status(OperatorStatus(config.run_id, "RUNNING", "", None))
    second = controller.start_mission(config.run_id, tmp_path, request_id=request)
    assert second.state == "RUNNING"
    assert (directory / "configuration/run.json").read_bytes() == frozen
    with pytest.raises(OperatorProtocolConflict, match="request|conflict"):
        controller.start_mission(config.run_id, tmp_path, request_id=str(uuid4()))


def test_stale_preflight_rejects_start_without_writing_request(tmp_path):
    config, store, directory = prepared(tmp_path)
    with OperatorProtocol(directory, config.run_id, config.config_sha256) as protocol:
        before = protocol.read_preflight("companion")
        protocol.write_preflight(replace(before, sequence=2, observed_wall_time_ns=time.time_ns() - 10_000_000_000))
    with pytest.raises(ControllerError, match="stale"):
        RunController(project_directory=ROOT).start_mission(config.run_id, tmp_path, request_id=str(uuid4()))
    assert not (directory / ".control/mission-start-request.json").exists()


def test_flight_abort_requires_admission_and_does_not_request_teardown(tmp_path):
    config, store, directory = prepared(tmp_path)
    controller = RunController(project_directory=ROOT)
    with pytest.raises(ControllerError, match="running"):
        controller.abort_flight(config.run_id, tmp_path, request_id=str(uuid4()))
    with OperatorProtocol(directory, config.run_id, config.config_sha256) as protocol:
        request = protocol.write_start_request(str(uuid4()))
        protocol.write_start_admission(request, native_epoch_ns=91_000_000_000)
    store.write_operator_status(OperatorStatus(config.run_id, "RUNNING", "", None))
    request_id = str(uuid4())
    controller.abort_flight(config.run_id, tmp_path, request_id=request_id)
    controller.abort_flight(config.run_id, tmp_path, request_id=request_id)
    assert (directory / ".control/flight-abort-request.json").exists()
    assert not (directory / ".control/finalize-request.json").exists()
