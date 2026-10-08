from pathlib import Path
import threading
import time

import pytest

from orchestration.operator_session import OperatorSession


ROOT = Path(__file__).resolve().parents[2]


class Runtime:
    def __init__(self):
        self.entered = threading.Event()
        self.release = threading.Event()

    def checks(self, **kwargs):
        self.entered.set()
        self.release.wait(3)
        return {"ok": False, "checks": [{"name": "Matching runtime images", "ok": False, "detail": "build needed"}]}


def test_prepare_is_single_owned_attempt_and_failed_prerequisites_are_actionable(tmp_path):
    runtime = Runtime()
    session = OperatorSession(ROOT, state_directory=tmp_path, runtime=runtime)
    first = session.prepare(preset_id="configured-descent", request_id="11111111-1111-4111-8111-111111111111")
    assert first["state"] == "PREPARING"
    assert runtime.entered.wait(2)
    retry = session.prepare(preset_id="configured-descent", request_id="11111111-1111-4111-8111-111111111111")
    assert retry["prepare_request_id"] == first["prepare_request_id"]
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
