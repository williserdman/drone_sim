from dataclasses import replace
from contextlib import nullcontext
from pathlib import Path
import hashlib
import threading
import time
from types import SimpleNamespace
from uuid import UUID

import pytest

from artifacts.qgc_operator import MavlinkIdentity
from orchestration.operator_session import OperatorSession
from orchestration.config import resolve_run_config, write_resolved_config
from orchestration.controller import RunResult
from orchestration.host_config import HostFacts
from orchestration.host_integration import LaunchResult, QgcInstallation
from orchestration.qgc_actions import QgcActionStore
from orchestration.qgc_publication import publish_posix
from orchestration.status_store import OperatorStatus, StatusStore
from orchestration.workstation import workstation_lock


ROOT = Path(__file__).resolve().parents[2]
QGC_SOURCE = MavlinkIdentity(255, 190)


@pytest.fixture(autouse=True)
def isolated_workstation_lease(tmp_path, monkeypatch):
    monkeypatch.setattr("orchestration.operator_session.workstation_lock",
                        lambda: workstation_lock(tmp_path / "workstation.lock"))
    monkeypatch.setattr(
        "orchestration.operator_session.resolve_host",
        lambda _project: SimpleNamespace(state_directory=tmp_path / "qgc-state"),
    )


class Runtime:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()

    def checks(self, **kwargs):
        self.entered.set()
        self.release.wait(3)
        return {"ok": False, "checks": [{"name": "Matching runtime images", "ok": False, "detail": "build needed"}]}


class TransitionRuntime:
    def __init__(self, *, blocked_call=1):
        self.first_entered = threading.Event()
        self.first_release = threading.Event()
        self.active_arguments = []
        self.blocked_call = blocked_call

    def checks(self, *, active=False):
        self.active_arguments.append(active)
        if len(self.active_arguments) == self.blocked_call:
            self.first_entered.set()
            self.first_release.wait(3)
        return {
            "ok": active,
            "checks": [
                {
                    "name": "Local ports",
                    "ok": active,
                    "detail": "owned by active session" if active else "port 5762 is busy",
                }
            ],
        }


class FakeBridge:
    def __init__(self, _directory):
        self.started = False

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def status(self):
        return {"fresh": False, "qgc_fresh": False, "detail": "stale telemetry"}


class PrepareRuntime:
    def __init__(self):
        self.events = []

    def checks(self, **_kwargs):
        return {"ok": True, "checks": []}

    def remaining_resources(self):
        return []


class PrepareHost:
    def __init__(self, action_directory: Path, state_directory: Path):
        self.facts = HostFacts("macos", "arm64", "linux/arm64", "colima")
        self.state_directory = state_directory
        self.action_directory = action_directory
        self.events = []

    def inspect_qgc(self):
        self.events.append("settings")
        return QgcInstallation(
            executable="/Applications/QGroundControl.app/Contents/MacOS/QGroundControl",
            settings_file="/tmp/QGroundControl.ini",
            save_path=str(self.action_directory.parent),
            version="5.0.8",
            source=QGC_SOURCE,
        )

    def acquire_native_lease(self):
        return nullcontext(SimpleNamespace())

    def publish_actions(self, material, installation, lease):
        del installation, lease
        return publish_posix(material, self.action_directory)

    def open_qgc(self, installation):
        del installation
        self.events.append("open-qgc")
        return LaunchResult(True, "focused")


class PrepareController:
    def __init__(self, output_root: Path, run_ids):
        self.output_root = output_root
        self.run_ids = iter(run_ids)
        self.event_stream = None
        self.compose_attempts = 0
        self.configs = []

    def start(self, source, **kwargs):
        config = resolve_run_config(
            source,
            run_id_factory=lambda: next(self.run_ids),
            launch_mode=kwargs["launch_mode"],
            preparation_wall_seconds=kwargs["preparation_wall_seconds"],
            output_root=kwargs["output_root"],
            operator_control_factory=kwargs["operator_control_factory"],
        )
        directory = StatusStore(self.output_root).allocate(config.run_id)
        kwargs["on_allocated"](config.run_id, directory)
        write_resolved_config(directory, config)
        kwargs["on_configuration_ready"](config, directory)
        self.compose_attempts += 1
        self.configs.append(config)
        return RunResult(config.run_id, "ABORTED", "prepared_cancelled", None)

    def status(self, run_id, _output_root):
        return RunResult(run_id, "ABORTED", "prepared_cancelled", None)


def _wait_for_worker(session: OperatorSession) -> None:
    assert session._worker is not None
    session._worker.join(3)
    assert not session._worker.is_alive()


def test_prepare_is_single_owned_attempt_and_failed_prerequisites_are_actionable(tmp_path):
    runtime = Runtime()
    session = OperatorSession(ROOT, state_directory=tmp_path, runtime=runtime)
    first = session.prepare(preset_id="configured-descent", request_id="11111111-1111-4111-8111-111111111111")
    assert first["state"] == "PREPARING"
    assert runtime.entered.wait(2)
    retry = session.prepare(preset_id="configured-descent", request_id="11111111-1111-4111-8111-111111111111")
    assert retry["prepare_request_id"] == first["prepare_request_id"]
    with pytest.raises(ValueError, match="parameters"):
        session.prepare(preset_id="configured-descent", request_id="11111111-1111-4111-8111-111111111111",
                        preparation_wall_seconds=3600)
    with pytest.raises(RuntimeError, match="active"):
        session.prepare(preset_id="moving-pad", request_id="22222222-2222-4222-8222-222222222222")
    runtime.release.set()
    deadline = time.monotonic() + 3
    while session.snapshot()["state"] == "PREPARING" and time.monotonic() < deadline:
        time.sleep(0.01)
    final = session.snapshot()
    assert final["state"] == "FAILED"
    assert "build needed" in final["error"]
    assert final["run_id"] is None
    session.shutdown()


def test_unknown_preset_never_starts_a_worker(tmp_path):
    session = OperatorSession(ROOT, state_directory=tmp_path, runtime=Runtime())
    with pytest.raises(ValueError, match="unknown preset"):
        session.prepare(preset_id="../../arbitrary.json")
    assert session.snapshot()["state"] == "IDLE"


def test_host_state_directory_is_the_default_action_ledger(tmp_path):
    host = PrepareHost(tmp_path / "actions", tmp_path / "native-state")

    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=Runtime(),
        host=host,
    )

    assert session.qgc_action_store.state_directory == host.state_directory


def test_checks_retry_when_a_new_owner_starts_during_slow_prerequisites(tmp_path):
    runtime = TransitionRuntime()
    session = OperatorSession(ROOT, state_directory=tmp_path, runtime=runtime)
    owner_release = threading.Event()
    owner_entered = threading.Event()

    def owner_worker(*_args):
        owner_entered.set()
        owner_release.wait(3)

    session._prepare_worker = owner_worker
    returned = []
    request = threading.Thread(target=lambda: returned.append(session.checks()))
    request.start()
    assert runtime.first_entered.wait(2)

    session.prepare(
        preset_id="configured-descent",
        request_id="11111111-1111-4111-8111-111111111111",
    )
    assert owner_entered.wait(2)
    runtime.first_release.set()
    request.join(3)

    assert runtime.active_arguments == [False, True]
    assert returned[0]["ok"] is True
    assert session.snapshot()["checks"] == returned[0]
    owner_release.set()
    session._worker.join(3)


def test_checks_retry_when_owner_finishes_during_slow_prerequisites(tmp_path):
    runtime = TransitionRuntime(blocked_call=2)
    session = OperatorSession(ROOT, state_directory=tmp_path, runtime=runtime)
    owner_release = threading.Event()

    def owner_worker(*_args):
        owner_release.wait(3)
        with session._lock:
            session._record["state"] = "FAILED"

    session._prepare_worker = owner_worker
    session.prepare(
        preset_id="configured-descent",
        request_id="11111111-1111-4111-8111-111111111111",
    )
    assert session.checks()["ok"] is True
    returned = []
    request = threading.Thread(target=lambda: returned.append(session.checks()))
    request.start()
    assert runtime.first_entered.wait(2)

    owner_release.set()
    session._worker.join(3)
    runtime.first_release.set()
    request.join(3)

    assert runtime.active_arguments == [True, True, False]
    assert returned[0]["ok"] is False
    assert session.snapshot()["checks"] == returned[0]


def test_prepare_publishes_actions_before_qgc_launch_and_run_again_is_fresh(tmp_path, monkeypatch):
    monkeypatch.setattr("orchestration.operator_session.GcsBridge", FakeBridge)
    published = tmp_path / "QGroundControl" / "MavlinkActions"
    published.mkdir(parents=True)
    existing = published / "custom_missions.json"
    existing.write_bytes(b"user action file\n")
    runtime = PrepareRuntime()
    host = PrepareHost(published, tmp_path / "qgc-state")
    run_ids = (
        UUID("00000000-0000-4000-8000-000000000601"),
        UUID("00000000-0000-4000-8000-000000000602"),
    )
    output_root = tmp_path / "runs"
    controller = PrepareController(output_root, run_ids)
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=runtime,
        controller=controller,
        qgc_action_store=QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE),
        host=host,
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="11111111-1111-4111-8111-111111111111",
    )
    _wait_for_worker(session)
    first = session.snapshot()
    first_config = controller.configs[0]
    first_path = Path(first["qgc_action_path"])
    assert first_path.name == f"drone-sim-{first_config.run_id}.json"
    assert first_path.read_bytes() == first_config.operator_actions.action_bytes
    assert hashlib.sha256(first_path.read_bytes()).hexdigest() == first_config.operator_control.actions_sha256
    assert (output_root / first_config.run_id / "configuration/qgc-actions.json").read_bytes() == first_path.read_bytes()
    assert existing.read_bytes() == b"user action file\n"
    assert host.events == ["settings", "open-qgc"]

    session.prepare(
        preset_id="configured-descent",
        request_id="22222222-2222-4222-8222-222222222222",
    )
    _wait_for_worker(session)
    second = session.snapshot()
    second_config = controller.configs[1]
    assert second["run_id"] != first["run_id"]
    assert second_config.operator_control.attempt_token > first_config.operator_control.attempt_token
    assert second["qgc_action_path"] != first["qgc_action_path"]
    assert Path(second["qgc_action_path"]).is_file()


def test_failed_action_publication_retains_identity_without_launching_qgc(tmp_path, monkeypatch):
    monkeypatch.setattr("orchestration.operator_session.GcsBridge", FakeBridge)
    action_directory = tmp_path / "QGroundControl" / "MavlinkActions"
    action_directory.parent.mkdir(parents=True)
    action_directory.symlink_to(tmp_path / "missing")
    runtime = PrepareRuntime()
    host = PrepareHost(action_directory, tmp_path / "qgc-state")
    controller = PrepareController(
        tmp_path / "runs",
        [UUID("00000000-0000-4000-8000-000000000603")],
    )
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=runtime,
        controller=controller,
        qgc_action_store=QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE),
        host=host,
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="33333333-3333-4333-8333-333333333333",
    )
    _wait_for_worker(session)
    snapshot = session.snapshot()
    assert snapshot["state"] == "FAILED"
    assert snapshot["run_id"] == "00000000-0000-4000-8000-000000000603"
    assert "symlink" in snapshot["error"]
    assert snapshot["cleanup"] == "complete"
    assert host.events == ["settings"]
    assert controller.compose_attempts == 0


def test_run_again_requires_complete_cleanup(tmp_path):
    session = OperatorSession(ROOT, state_directory=tmp_path, runtime=Runtime())
    session._record.update(
        state="FAILED",
        run_id="00000000-0000-4000-8000-000000000604",
        cleanup="incomplete",
    )
    with pytest.raises(RuntimeError, match="cleanup"):
        session.prepare(
            preset_id="configured-descent",
            request_id="44444444-4444-4444-8444-444444444444",
        )


def test_qgc_admission_does_not_depend_on_browser_heartbeat(tmp_path, monkeypatch):
    monkeypatch.setattr("orchestration.operator_session.GcsBridge", FakeBridge)
    actions = QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE)
    config = resolve_run_config(
        ROOT / "config/configured-descent-run.json",
        launch_mode="operator",
        output_root=tmp_path / "runs",
        operator_control_factory=actions.allocate,
    )
    store = StatusStore(tmp_path / "runs")
    directory = store.allocate(config.run_id)
    write_resolved_config(directory, config)
    store.write_operator_status(OperatorStatus(config.run_id, "PREPARED", "", None))
    from artifacts.operator_protocol import OperatorProtocol, PreflightStatus
    with OperatorProtocol(directory, config.run_id, config.config_sha256) as protocol:
        protocol.write_preflight(PreflightStatus(
            config.run_id, config.config_sha256, "gazebo", 1, time.time_ns(),
            native_timestamp_ns=90_000_000_000, physics_advancing=True,
            camera_native_timestamp_ns=90_000_000_000, ready=True, detail="live",
        ))
        protocol.write_preflight(PreflightStatus(
            config.run_id, config.config_sha256, "companion", 1, time.time_ns(),
            heartbeat_live=True, prearm_checks_healthy=True, armed=False,
            ready=True, detail="ready",
        ))
    controller = SimpleNamespace(status=lambda run_id, _root: RunResult(run_id, "PREPARED", "", None))
    session = OperatorSession(ROOT, state_directory=tmp_path / "session", runtime=Runtime(), controller=controller)
    session._record.update(
        state="PREPARED", run_id=config.run_id, run_directory=str(directory),
        qgc_action_path=str(tmp_path / "QGroundControl/MavlinkActions" / config.operator_actions.published_name),
    )
    session._worker = SimpleNamespace(is_alive=lambda: True)
    session._bridge = FakeBridge(directory)
    monkeypatch.setattr("orchestration.operator_session.urlopen", lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError()))

    snapshot = session.snapshot()
    assert snapshot["qgc_admission_ready"] is True
    assert snapshot["views"]["qgc"]["qgc_fresh"] is False
    assert snapshot["views"]["gazebo"]["connected"] is False
    assert snapshot["views"]["camera"]["fresh"] is False
    with OperatorProtocol(directory, config.run_id, config.config_sha256) as protocol:
        companion = protocol.read_preflight("companion")
        protocol.write_preflight(replace(
            companion,
            sequence=2,
            observed_wall_time_ns=time.time_ns() - 10_000_000_000,
        ))
    assert session.snapshot()["qgc_admission_ready"] is False
