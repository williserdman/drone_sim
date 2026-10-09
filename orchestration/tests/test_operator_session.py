from pathlib import Path
import threading
import time

import pytest

from orchestration.operator_session import OperatorSession
from orchestration.workstation import workstation_lock


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def isolated_workstation_lease(tmp_path, monkeypatch):
    monkeypatch.setattr("orchestration.operator_session.workstation_lock",
                        lambda: workstation_lock(tmp_path / "workstation.lock"))


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
