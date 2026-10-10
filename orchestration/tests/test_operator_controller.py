from dataclasses import replace
import json
from pathlib import Path
import time
from uuid import UUID, uuid4

import pytest

from artifacts.qgc_operator import MavlinkIdentity
from artifacts.operator_protocol import OperatorProtocol, OperatorProtocolConflict, PreflightStatus
from orchestration.config import resolve_run_config, write_resolved_config
from orchestration.controller import ControllerError, RunController
from orchestration.qgc_actions import QgcActionStore
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


def test_simulation_stop_records_shutdown_distinct_from_recovery(tmp_path):
    config, store, directory = prepared(tmp_path)
    RunController(project_directory=ROOT).abort(config.run_id, tmp_path, reason="simulation_stop")
    assert store.read_finalize_request(config.run_id)["reason"] == "simulation_stop"
    assert not (directory / ".control/flight-abort-request.json").exists()


def test_qgc_runs_reject_host_flight_actions_before_writing_requests(tmp_path):
    actions = QgcActionStore(tmp_path / "qgc-state", MavlinkIdentity(255, 190))
    config = resolve_run_config(
        ROOT / "config/configured-descent-run.json",
        launch_mode="operator",
        output_root=tmp_path,
        operator_control_factory=actions.allocate,
    )
    store = StatusStore(tmp_path)
    directory = store.allocate(config.run_id)
    write_resolved_config(directory, config)
    store.write_operator_status(OperatorStatus(config.run_id, "PREPARED", "", None))
    controller = RunController(project_directory=ROOT)

    with pytest.raises(ControllerError, match="QGroundControl"):
        controller.start_mission(config.run_id, tmp_path, request_id=str(uuid4()))
    assert not (directory / ".control/mission-start-request.json").exists()

    store.write_operator_status(OperatorStatus(config.run_id, "RUNNING", "", None))
    with pytest.raises(ControllerError, match="QGroundControl"):
        controller.abort_flight(config.run_id, tmp_path, request_id=str(uuid4()))
    assert not (directory / ".control/flight-abort-request.json").exists()


def test_qgc_configuration_is_published_before_compose_and_bound_in_manifest(tmp_path):
    run_id = "00000000-0000-4000-8000-000000000605"
    actions = QgcActionStore(tmp_path / "qgc-state", MavlinkIdentity(255, 190))
    events = []

    def configuration_ready(config, directory):
        assert (directory / "configuration/run.json").is_file()
        assert (directory / "configuration/qgc-actions.json").read_bytes() == config.operator_actions.action_bytes
        published = actions.publish(config.operator_actions, tmp_path / "QGroundControl/MavlinkActions")
        events.append(("published", published))

    def compose_factory(*_args, **_kwargs):
        events.append(("compose", None))
        raise RuntimeError("compose disabled in unit test")

    result = RunController(
        project_directory=ROOT,
        uuid_factory=lambda: UUID(run_id),
        compose_factory=compose_factory,
    ).start(
        ROOT / "config/configured-descent-run.json",
        launch_mode="operator",
        output_root=tmp_path / "runs",
        operator_control_factory=actions.allocate,
        on_allocated=lambda allocated_run_id, directory: events.append(
            ("allocated", (allocated_run_id, directory))
        ),
        on_configuration_ready=configuration_ready,
    )

    assert [event[0] for event in events] == ["allocated", "published", "compose"]
    manifest = json.loads((tmp_path / "runs" / run_id / "manifest.json").read_text())
    config_records = {row["relative_path"]: row["sha256"] for row in manifest["configurations"]}
    assert config_records["configuration/qgc-actions.json"] == json.loads(
        (tmp_path / "runs" / run_id / "configuration/run.json").read_text()
    )["operator_control"]["actions_sha256"]
    assert Path(events[1][1]).name == f"drone-sim-{run_id}.json"
    assert result.state == "FAILED"
