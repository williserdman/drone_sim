from dataclasses import replace
from contextlib import contextmanager, nullcontext
import ntpath
from pathlib import Path
import hashlib
import json
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
from orchestration.host_integration import (
    HostHelperInfo,
    LaunchResult,
    PublicationReceipt,
    QgcInstallation,
)
from orchestration.qgc_actions import QgcActionStore
from orchestration.qgc_publication import publish_posix
from orchestration.status_store import OperatorStatus, StatusStore, TerminalCause


ROOT = Path(__file__).resolve().parents[2]
QGC_SOURCE = MavlinkIdentity(255, 190)


def test_session_adopts_one_workstation_from_injected_runtime_and_controller(
    tmp_path,
):
    host = SimpleNamespace(state_directory=tmp_path / "qgc-state")
    workstation = SimpleNamespace(host=host)
    runtime = SimpleNamespace(workstation=workstation)
    controller = SimpleNamespace(workstation=workstation)

    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=runtime,
        controller=controller,
    )

    assert session.workstation is workstation
    assert session.host is host


def test_session_rejects_split_workstations_in_injected_dependencies(tmp_path):
    host = SimpleNamespace(state_directory=tmp_path / "qgc-state")
    runtime = SimpleNamespace(workstation=SimpleNamespace(host=host))
    controller = SimpleNamespace(workstation=SimpleNamespace(host=host))

    with pytest.raises(ValueError, match="same workstation"):
        OperatorSession(
            ROOT,
            state_directory=tmp_path / "session",
            runtime=runtime,
            controller=controller,
        )


@pytest.fixture(autouse=True)
def isolated_workstation_lease(tmp_path, monkeypatch):
    class Lease:
        def begin_run(self, _run_id):
            pass

        def finish_run(self, _run_id, *, verified_cleanup):
            pass

    class Workstation:
        def __init__(self, _host):
            self.lease = Lease()
            self._owned = False

        @contextmanager
        def hold(self, *, allow_nested=False, recovery_run_id=None):
            self._owned = True
            try:
                yield self.lease
            finally:
                self._owned = False

        def owned(self):
            assert self._owned
            return self.lease

    monkeypatch.setattr("orchestration.operator_session.Workstation", Workstation)
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


class FakeTransport:
    def __init__(self, events):
        self.events = events
        self.closed = False

    def close(self):
        if not self.closed:
            self.events.append("transport-close")
            self.closed = True


class FakeBridge:
    def __init__(self, _directory, *, transport):
        self.transport = transport
        self.started = False

        self.transport.events.append("bridge-constructed")

    def start(self):
        self.started = True

        self.transport.events.append("bridge-start")

    def stop(self):
        if self.started:
            self.transport.events.append("bridge-stop")
        self.started = False
        self.transport.close()

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
        self.facts = HostFacts("macos", "arm64", "linux/arm64", "docker-desktop")
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

    def helper_info(self):
        return None

    def acquire_native_lease(self):
        return nullcontext(SimpleNamespace())

    def publish_actions(self, material, installation, lease):
        del installation, lease
        receipt = publish_posix(material, self.action_directory)
        self.events.append("publish")
        return receipt

    def open_datagrams(self, lease, run_id, publication):
        del lease, run_id, publication
        self.events.append("transport-open")
        return FakeTransport(self.events)

    def open_qgc(self, installation):
        del installation
        self.events.append("open-qgc")
        return LaunchResult(True, "focused")


class PrepareController:
    def __init__(
        self,
        output_root: Path,
        run_ids,
        *,
        teardown_failure=False,
        events=None,
    ):
        self.output_root = output_root
        self.run_ids = iter(run_ids)
        self.event_stream = None
        self.compose_attempts = 0
        self.configs = []
        self.teardown_failure = teardown_failure
        self.events = events if events is not None else []

    def start(self, source, **kwargs):
        def identify():
            generated = next(self.run_ids)
            kwargs["on_run_identified"](str(generated))
            return generated

        config = resolve_run_config(
            source,
            run_id_factory=identify,
            launch_mode=kwargs["launch_mode"],
            preparation_wall_seconds=kwargs["preparation_wall_seconds"],
            output_root=kwargs["output_root"],
            operator_control_factory=kwargs["operator_control_factory"],
        )
        directory = StatusStore(self.output_root).allocate(config.run_id)
        kwargs["on_allocated"](config.run_id, directory)
        StatusStore(self.output_root).write_operator_status(
            OperatorStatus(config.run_id, "CREATED", "", None)
        )
        write_resolved_config(directory, config)
        kwargs["on_configuration_ready"](config, directory)
        self.compose_attempts += 1
        self.events.append("compose-up")
        self.configs.append(config)
        StatusStore(self.output_root).write_operator_status(
            OperatorStatus(
                config.run_id,
                "ABORTED",
                "prepared_cancelled",
                None,
                diagnostics=(
                    (TerminalCause("teardown", "native_owner_clear_failed"),)
                    if self.teardown_failure
                    else ()
                ),
            )
        )
        self.events.append("artifacts-final")
        kwargs["on_before_inventory"]()
        self.events.append("inventory")
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
    controller = PrepareController(output_root, run_ids, events=host.events)
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
    assert host.events[:9] == [
        "settings",
        "publish",
        "transport-open",
        "bridge-constructed",
        "bridge-start",
        "open-qgc",
        "compose-up",
        "artifacts-final",
        "bridge-stop",
    ]
    assert host.events.index("transport-open") < host.events.index("compose-up")
    assert host.events.index("bridge-stop") < host.events.index("inventory")
    assert host.events.index("transport-close") < host.events.index("inventory")

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


def test_native_publication_mismatch_never_starts_compose(tmp_path, monkeypatch):
    monkeypatch.setattr("orchestration.operator_session.GcsBridge", FakeBridge)
    published = tmp_path / "QGroundControl" / "MavlinkActions"
    published.mkdir(parents=True)

    class MismatchingHost(PrepareHost):
        def publish_actions(self, material, installation, lease):
            receipt = super().publish_actions(material, installation, lease)
            return PublicationReceipt(receipt.native_path, receipt.filename, "f" * 64)

    host = MismatchingHost(published, tmp_path / "qgc-state")
    controller = PrepareController(
        tmp_path / "runs",
        [UUID("00000000-0000-4000-8000-000000000609")],
    )
    store = QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE)
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=PrepareRuntime(),
        controller=controller,
        qgc_action_store=store,
        host=host,
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="99999999-9999-4999-8999-999999999999",
    )
    _wait_for_worker(session)
    consumed_token = 1
    next_material = store.allocate("00000000-0000-4000-8000-000000000610")

    assert controller.compose_attempts == 0
    assert (tmp_path / "runs" / "00000000-0000-4000-8000-000000000609").exists()
    assert next_material.descriptor.attempt_token > consumed_token
    assert session.snapshot()["cleanup"] == "complete"


def test_native_publication_path_mismatch_never_starts_compose(tmp_path, monkeypatch):
    monkeypatch.setattr("orchestration.operator_session.GcsBridge", FakeBridge)
    published = tmp_path / "QGroundControl" / "MavlinkActions"
    published.mkdir(parents=True)

    class MismatchingPathHost(PrepareHost):
        def publish_actions(self, material, installation, lease):
            receipt = super().publish_actions(material, installation, lease)
            return PublicationReceipt(
                str(tmp_path / "other" / receipt.filename),
                receipt.filename,
                receipt.sha256,
            )

    controller = PrepareController(
        tmp_path / "runs",
        [UUID("00000000-0000-4000-8000-000000000612")],
    )
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=PrepareRuntime(),
        controller=controller,
        qgc_action_store=QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE),
        host=MismatchingPathHost(published, tmp_path / "qgc-state"),
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb",
    )
    _wait_for_worker(session)

    assert controller.compose_attempts == 0
    assert "native path" in session.snapshot()["error"]


@pytest.mark.parametrize(
    ("save_path", "receipt_root"),
    [
        (
            "C:/Users/Pilot/Documents/QGroundControl",
            r"C:\Users\Pilot\Documents\QGroundControl",
        ),
        (
            r"C:\Users\Pilot\Documents\QGroundControl",
            r"c:\users\pilot\documents\qgroundcontrol",
        ),
    ],
)
def test_wsl_publication_accepts_equivalent_windows_path_spelling(
    tmp_path, monkeypatch, save_path, receipt_root
):
    monkeypatch.setattr("orchestration.operator_session.GcsBridge", FakeBridge)

    class EquivalentWindowsPathHost(PrepareHost):
        def __init__(self, state_directory):
            super().__init__(tmp_path / "unused-actions", state_directory)
            self.facts = HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")

        def inspect_qgc(self):
            self.events.append("settings")
            return QgcInstallation(
                executable=r"C:\Program Files\QGroundControl\bin\QGroundControl.exe",
                settings_file=r"C:\Users\Pilot\AppData\Roaming\QGroundControl\QGroundControl.ini",
                save_path=save_path,
                version="5.0.8",
                source=QGC_SOURCE,
            )

        def publish_actions(self, material, installation, lease):
            del installation, lease
            self.events.append("publish")
            return PublicationReceipt(
                ntpath.join(receipt_root, "MavlinkActions", material.published_name),
                material.published_name,
                hashlib.sha256(material.action_bytes).hexdigest(),
            )

    controller = PrepareController(
        tmp_path / "runs",
        [UUID("00000000-0000-4000-8000-000000000614")],
    )
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=PrepareRuntime(),
        controller=controller,
        qgc_action_store=QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE),
        host=EquivalentWindowsPathHost(tmp_path / "qgc-state"),
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="cccccccc-cccc-4ccc-8ccc-cccccccccccc",
    )
    _wait_for_worker(session)

    assert controller.compose_attempts == 1
    assert session.snapshot()["state"] == "ABORTED"


@pytest.mark.parametrize(
    ("save_path", "receipt_path"),
    [
        (
            r"C:\Users\Pilot\Documents\QGroundControl",
            r"C:\Users\Pilot\Documents\Other\MavlinkActions\{filename}",
        ),
        (
            r"C:\Users\Pilot\Documents\QGroundControl",
            r"MavlinkActions\{filename}",
        ),
        (
            r"C:\Users\Pilot\Documents\QGroundControl",
            r"C:\Users\Pilot\Documents\QGroundControl\MavlinkActions\..\..\Other\MavlinkActions\{filename}",
        ),
        (
            r"relative\QGroundControl",
            r"C:\Users\Pilot\Documents\QGroundControl\MavlinkActions\{filename}",
        ),
        (
            r"C:\Users\Pilot\Documents\QGroundControl",
            r"C:\Users\Pilot\Documents\QGroundControl\MavlinkActions\{upper_filename}",
        ),
    ],
)
def test_wsl_publication_rejects_nonabsolute_wrong_or_inexact_windows_path(
    tmp_path, monkeypatch, save_path, receipt_path
):
    monkeypatch.setattr("orchestration.operator_session.GcsBridge", FakeBridge)

    class InvalidWindowsPathHost(PrepareHost):
        def __init__(self, state_directory):
            super().__init__(tmp_path / "unused-actions", state_directory)
            self.facts = HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")

        def inspect_qgc(self):
            self.events.append("settings")
            return QgcInstallation(
                executable=r"C:\Program Files\QGroundControl\bin\QGroundControl.exe",
                settings_file=r"C:\Users\Pilot\AppData\Roaming\QGroundControl\QGroundControl.ini",
                save_path=save_path,
                version="5.0.8",
                source=QGC_SOURCE,
            )

        def publish_actions(self, material, installation, lease):
            del installation, lease
            self.events.append("publish")
            return PublicationReceipt(
                receipt_path.format(
                    filename=material.published_name,
                    upper_filename=material.published_name.upper(),
                ),
                material.published_name,
                hashlib.sha256(material.action_bytes).hexdigest(),
            )

    controller = PrepareController(
        tmp_path / "runs",
        [UUID("00000000-0000-4000-8000-000000000615")],
    )
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=PrepareRuntime(),
        controller=controller,
        qgc_action_store=QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE),
        host=InvalidWindowsPathHost(tmp_path / "qgc-state"),
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="dddddddd-dddd-4ddd-8ddd-dddddddddddd",
    )
    _wait_for_worker(session)

    assert controller.compose_attempts == 0
    assert "native path" in session.snapshot()["error"]


def test_qgc_launch_refusal_is_visible_without_stopping_prepare(tmp_path, monkeypatch):
    monkeypatch.setattr("orchestration.operator_session.GcsBridge", FakeBridge)
    published = tmp_path / "QGroundControl" / "MavlinkActions"
    published.mkdir(parents=True)

    class LaunchRefusedHost(PrepareHost):
        def open_qgc(self, installation):
            del installation
            self.events.append("open-qgc")
            return LaunchResult(False, "desktop declined the launch request")

    host = LaunchRefusedHost(published, tmp_path / "qgc-state")
    controller = PrepareController(
        tmp_path / "runs",
        [UUID("00000000-0000-4000-8000-000000000613")],
        events=host.events,
    )
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=PrepareRuntime(),
        controller=controller,
        qgc_action_store=QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE),
        host=host,
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="cccccccc-cccc-4ccc-8ccc-cccccccccccc",
    )
    _wait_for_worker(session)

    snapshot = session.snapshot()
    assert controller.compose_attempts == 1
    assert snapshot["qgc_launch"] == {
        "opened": False,
        "detail": "desktop declined the launch request",
    }


def test_host_evidence_is_allowlisted_and_closed_before_inventory(tmp_path, monkeypatch):
    class ObservedBridge(FakeBridge):
        def status(self):
            self.transport.events.append("bridge-status")
            return {
                "connected": True,
                "detail": "connected to SITL GCS channel",
                "qgc_connected": True,
                "qgc_detail": "QGC datagram transport open",
                "fresh": True,
                "qgc_fresh": True,
                "armed": False,
                "mode": "GUIDED",
                "relative_altitude_m": 3.25,
                "landed_state": 2,
                "untrusted_extra": "must not be persisted",
            }

    monkeypatch.setattr("orchestration.operator_session.GcsBridge", ObservedBridge)
    published = tmp_path / "QGroundControl" / "MavlinkActions"
    published.mkdir(parents=True)

    class EvidenceHost(PrepareHost):
        def helper_info(self):
            return HostHelperInfo("1", "a" * 64)

        def inspect_qgc(self):
            self.events.append("settings")
            return QgcInstallation(
                executable=r"C:\Program Files\QGroundControl\bin\QGroundControl.exe",
                settings_file=r"C:\Users\pilot\AppData\Roaming\QGroundControl\QGroundControl.ini",
                save_path=r"C:\Users\pilot\Documents\QGroundControl",
                version="5.0.8",
                source=QGC_SOURCE,
            )

        def publish_actions(self, material, installation, lease):
            del lease
            self.events.append("publish")
            return PublicationReceipt(
                rf"{installation.save_path}\MavlinkActions\{material.published_name}",
                material.published_name,
                hashlib.sha256(material.action_bytes).hexdigest(),
            )

    host = EvidenceHost(published, tmp_path / "qgc-state")
    host.facts = HostFacts("wsl2", "amd64", "linux/amd64", "docker-desktop")
    controller = PrepareController(
        tmp_path / "runs",
        [UUID("00000000-0000-4000-8000-000000000611")],
        events=host.events,
    )
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=PrepareRuntime(),
        controller=controller,
        qgc_action_store=QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE),
        host=host,
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa",
    )
    _wait_for_worker(session)

    evidence_path = (
        tmp_path
        / "runs/00000000-0000-4000-8000-000000000611/logs/docker/host-integration.jsonl"
    )
    rows = [json.loads(line) for line in evidence_path.read_text().splitlines()]
    assert rows[-1] == {"event": "ownership", "state": "cleanup_pending"}
    assert rows[-2] == {
        "event": "bridge_observation",
        "sitl_connected": True,
        "sitl_detail": "connected to SITL GCS channel",
        "vehicle_telemetry_fresh": True,
        "qgc_connected": True,
        "qgc_detail": "QGC datagram transport open",
        "qgc_traffic_fresh": True,
        "armed": False,
        "mode": "GUIDED",
        "relative_altitude_m": 3.25,
        "landed_state": 2,
    }
    assert rows[0] == {
        "event": "host",
        "platform": "wsl2",
        "provider": "docker-desktop",
        "image_platform": "linux/amd64",
        "transport": "windows-helper-udp",
    }
    assert any(row.get("event") == "qgc" and row["version"] == "5.0.8" for row in rows)
    assert any(row.get("event") == "helper" and row["source_sha256"] == "a" * 64 for row in rows)
    assert any(row.get("event") == "publication" and row["native_path"].endswith(".json") for row in rows)
    assert all("environment" not in row and "credential" not in row for row in rows)
    assert all("untrusted_extra" not in row for row in rows)
    assert host.events.index("artifacts-final") < host.events.index("bridge-status")
    assert host.events.index("bridge-status") < host.events.index("bridge-stop")
    assert host.events.index("bridge-stop") < host.events.index("inventory")
    with pytest.raises(ValueError, match="closed"):
        session._host_evidence.write("ownership", state="after_inventory")


def test_bridge_observation_failure_still_stops_and_closes_before_inventory(
    tmp_path, monkeypatch
):
    class FailedObservationBridge(FakeBridge):
        def status(self):
            self.transport.events.append("bridge-status")
            raise RuntimeError("bridge observation unavailable")

    monkeypatch.setattr(
        "orchestration.operator_session.GcsBridge", FailedObservationBridge
    )
    published = tmp_path / "QGroundControl" / "MavlinkActions"
    published.mkdir(parents=True)
    run_id = UUID("00000000-0000-4000-8000-000000000614")
    host = PrepareHost(published, tmp_path / "qgc-state")
    controller = PrepareController(tmp_path / "runs", [run_id], events=host.events)
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=PrepareRuntime(),
        controller=controller,
        qgc_action_store=QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE),
        host=host,
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="dddddddd-dddd-4ddd-8ddd-dddddddddddd",
    )
    _wait_for_worker(session)

    assert session.snapshot()["state"] == "FAILED"
    assert host.events.index("bridge-status") < host.events.index("bridge-stop")
    assert "inventory" not in host.events
    rows = [
        json.loads(line)
        for line in (
            tmp_path / "runs" / str(run_id) / "logs/docker/host-integration.jsonl"
        ).read_text().splitlines()
    ]
    assert rows[-1] == {"event": "ownership", "state": "cleanup_pending"}
    with pytest.raises(ValueError, match="closed"):
        session._host_evidence.write("ownership", state="after_failure")


def test_native_owner_clear_failure_keeps_session_cleanup_incomplete(
    tmp_path, monkeypatch
):
    monkeypatch.setattr("orchestration.operator_session.GcsBridge", FakeBridge)
    published = tmp_path / "QGroundControl" / "MavlinkActions"
    published.mkdir(parents=True)
    controller = PrepareController(
        tmp_path / "runs",
        [UUID("00000000-0000-4000-8000-000000000607")],
        teardown_failure=True,
    )
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=PrepareRuntime(),
        controller=controller,
        qgc_action_store=QgcActionStore(tmp_path / "qgc-state", QGC_SOURCE),
        host=PrepareHost(published, tmp_path / "qgc-state"),
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="77777777-7777-4777-8777-777777777777",
    )
    _wait_for_worker(session)

    assert session.snapshot()["cleanup"] == "incomplete"


def test_run_identity_receipt_survives_failure_before_directory_allocation(
    tmp_path,
):
    run_id = "00000000-0000-4000-8000-000000000608"

    class Controller:
        event_stream = None

        def start(self, _source, **kwargs):
            kwargs["on_run_identified"](run_id)
            raise RuntimeError("token allocation failed")

    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=PrepareRuntime(),
        controller=Controller(),
    )

    session.prepare(
        preset_id="configured-descent",
        request_id="55555555-5555-4555-8555-555555555555",
    )
    _wait_for_worker(session)

    snapshot = session.snapshot()
    assert snapshot["run_id"] == run_id
    assert snapshot.get("run_directory") is None
    assert snapshot["cleanup"] == "incomplete"
    persisted = __import__("json").loads(
        (tmp_path / "session/session.json").read_text()
    )
    assert persisted["run_id"] == run_id

    assert session.results() == {
        "cleanup": "incomplete",
        "manifest": None,
        "physical_outcome": None,
        "score": None,
    }
    stopping = session.stop(run_id=run_id)
    assert stopping["state"] == "FINALIZING"
    _wait_for_worker(session)
    finished = session.snapshot()
    assert finished["state"] == "INTERRUPTED"
    assert finished["cleanup"] == "complete"


def test_orphan_stop_reports_finalizing_while_cleanup_worker_is_active(tmp_path):
    run_id = "00000000-0000-4000-8000-000000000610"
    output_root = tmp_path / "runs"
    config = resolve_run_config(
        ROOT / "config/configured-descent-run.json",
        run_id_factory=lambda: UUID(run_id),
        launch_mode="operator",
        output_root=output_root,
    )
    directory = StatusStore(output_root).allocate(run_id)
    write_resolved_config(directory, config)
    StatusStore(output_root).write_operator_status(
        OperatorStatus(run_id, "READY", "", None)
    )

    entered = threading.Event()
    release = threading.Event()

    class Workstation:
        @contextmanager
        def hold(self, **_kwargs):
            entered.set()
            release.wait(3)
            raise RuntimeError("end focused cleanup")
            yield

    class Controller:
        def status(self, owned_run_id, owned_output_root):
            assert owned_run_id == run_id
            assert owned_output_root == output_root
            return RunResult(run_id, "READY", "", None)

        def abort(self, owned_run_id, owned_output_root, *, reason):
            assert owned_run_id == run_id
            assert owned_output_root == output_root
            assert reason == "prepared_cancelled"

    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=SimpleNamespace(),
        controller=Controller(),
        workstation=Workstation(),
    )
    session._record.update(
        state="ORPHANED",
        run_id=run_id,
        run_directory=str(directory),
        cleanup="pending",
    )

    stopping = session.stop(run_id=run_id)

    try:
        assert entered.wait(2)
        assert stopping["state"] == "FINALIZING"
    finally:
        release.set()
        _wait_for_worker(session)


def test_restart_preserves_terminal_session_after_verified_cleanup(tmp_path):
    run_id = "00000000-0000-4000-8000-000000000611"
    output_root = ROOT / "runs"
    run_directory = output_root / run_id
    state_directory = tmp_path / "session"
    state_directory.mkdir()
    (state_directory / "session.json").write_text(json.dumps({
        "state": "INTERRUPTED",
        "run_id": run_id,
        "run_directory": str(run_directory),
        "preset_id": "configured-descent",
        "prepare_request_id": "55555555-5555-4555-8555-555555555556",
        "error": "previous owner stopped",
        "cleanup": "complete",
    }))

    class Controller:
        def status(self, owned_run_id, owned_output_root):
            assert owned_run_id == run_id
            assert owned_output_root == output_root
            return RunResult(run_id, "READY", "", None)

    runtime = Runtime()
    session = OperatorSession(
        ROOT,
        state_directory=state_directory,
        runtime=runtime,
        controller=Controller(),
    )

    assert session.snapshot()["state"] == "INTERRUPTED"
    try:
        preparing = session.prepare(
            preset_id="configured-descent",
            request_id="55555555-5555-4555-8555-555555555557",
        )
        assert preparing["state"] == "PREPARING"
        assert runtime.entered.wait(2)
    finally:
        runtime.release.set()
        _wait_for_worker(session)


def test_restart_blocks_uncertain_session_without_verified_cleanup(tmp_path):
    run_id = "00000000-0000-4000-8000-000000000612"
    output_root = ROOT / "runs"
    run_directory = output_root / run_id
    state_directory = tmp_path / "session"
    state_directory.mkdir()
    (state_directory / "session.json").write_text(json.dumps({
        "state": "INTERRUPTED",
        "run_id": run_id,
        "run_directory": str(run_directory),
        "preset_id": "configured-descent",
        "prepare_request_id": "55555555-5555-4555-8555-555555555558",
        "error": "previous owner stopped",
        "cleanup": "incomplete",
    }))

    controller = SimpleNamespace(
        status=lambda _run_id, _root: RunResult(run_id, "READY", "", None)
    )
    session = OperatorSession(
        ROOT,
        state_directory=state_directory,
        runtime=Runtime(),
        controller=controller,
    )

    assert session.snapshot()["state"] == "ORPHANED"
    with pytest.raises(RuntimeError, match="active"):
        session.prepare(
            preset_id="configured-descent",
            request_id="55555555-5555-4555-8555-555555555559",
        )


@pytest.mark.parametrize(
    ("lifecycle_state", "expected_cleanup", "expected_finish"),
    (("CREATED", "complete", True), ("STARTING", "incomplete", False)),
)
def test_orphan_cleanup_distinguishes_precompose_failure_from_missing_media_receipt(
    tmp_path,
    monkeypatch,
    lifecycle_state,
    expected_cleanup,
    expected_finish,
):
    run_id = "00000000-0000-4000-8000-000000000609"
    output_root = tmp_path / "runs"
    config = resolve_run_config(
        ROOT / "config/configured-descent-run.json",
        run_id_factory=lambda: UUID(run_id),
        launch_mode="operator",
        output_root=output_root,
    )
    store = StatusStore(output_root)
    directory = store.allocate(run_id)
    write_resolved_config(directory, config)
    store.write_operator_status(OperatorStatus(run_id, lifecycle_state, "", None))
    if lifecycle_state != "CREATED":
        (directory / ".status/artifacts-final.json").write_text("{}")

    class Lease:
        finished = False

        def finish_run(self, owned_run_id, *, verified_cleanup):
            assert owned_run_id == run_id
            assert verified_cleanup is True
            self.finished = True

    lease = Lease()

    class Workstation:
        @contextmanager
        def hold(self, *, recovery_run_id=None, **_kwargs):
            assert recovery_run_id == run_id
            yield lease

    class Runtime:
        def remaining_resources(self):
            return []

    class Compose:
        def __init__(self, **_kwargs):
            pass

        def down(self, _timeout):
            return SimpleNamespace(returncode=0)

        def ps(self, _timeout):
            return SimpleNamespace(returncode=0, output=b"[]")

        def media_cleanup_verified(self, _timeout):
            return False

    monkeypatch.setattr("orchestration._adapters.compose.ComposeRuntime", Compose)
    session = OperatorSession(
        ROOT,
        state_directory=tmp_path / "session",
        runtime=Runtime(),
        controller=SimpleNamespace(),
        workstation=Workstation(),
    )
    session._record.update(
        state="ORPHANED",
        run_id=run_id,
        run_directory=str(directory),
        cleanup="incomplete",
    )

    session._cleanup_orphan()

    assert session._record["cleanup"] == expected_cleanup
    assert lease.finished is expected_finish


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
    session._bridge = FakeBridge(directory, transport=FakeTransport([]))
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
