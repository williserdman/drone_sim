from dataclasses import dataclass

import pytest

from orchestration.runtime_node import OrchestrationRuntime, RuntimeStateEvent


RUN_ID = "11111111-1111-4111-8111-111111111111"


class FakeProtocol:
    def __init__(self):
        self.finalize = None
        self.terminal = None
        self.statuses = []

    def read_finalize_request(self):
        return self.finalize

    def read_terminal_committed(self):
        return self.terminal

    def write_status(self, name, document):
        self.statuses.append((name, document))


def _runtime():
    protocol = FakeProtocol()
    published = []
    diagnostics = []
    runtime = OrchestrationRuntime(
        RUN_ID,
        "a" * 64,
        protocol=protocol,
        publish=published.append,
        diagnostic=diagnostics.append,
    )
    return runtime, protocol, published, diagnostics


def test_starting_ready_running_order_and_exact_first_clock_stamp():
    runtime, protocol, published, _ = _runtime()
    runtime.start()
    runtime.accept_clock(99)
    runtime.accept_artifact_status("stale", True)
    runtime.accept_artifact_status(RUN_ID, False)
    runtime.accept_artifact_status(RUN_ID, True)
    runtime.accept_clock(0)
    runtime.accept_clock(50_000_000)

    assert [(item.state, item.sim_timestamp_ns) for item in published] == [
        ("STARTING", 0),
        ("READY", 0),
        ("RUNNING", 0),
    ]
    assert protocol.statuses == [
        (
            "runtime-running",
            {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": 0},
        )
    ]
    assert runtime.last_sim_timestamp_ns == 50_000_000


@pytest.mark.parametrize("preterminal", ["STARTING", "READY", "RUNNING"])
@pytest.mark.parametrize("terminal", ["COMPLETED", "FAILED", "ABORTED"])
def test_finalize_from_every_preterminal_state_and_acknowledge_silently(preterminal, terminal):
    runtime, protocol, published, _ = _runtime()
    runtime.start()
    if preterminal in {"READY", "RUNNING"}:
        runtime.accept_artifact_status(RUN_ID, True)
    if preterminal == "RUNNING":
        runtime.accept_clock(123)
    protocol.finalize = {
        "run_id": RUN_ID,
        "requested_terminal": terminal,
        "reason": "operator requested",
    }
    assert runtime.poll() is False
    assert published[-1] == RuntimeStateEvent(
        RUN_ID, "FINALIZING", 123 if preterminal == "RUNNING" else 0, "operator requested", "a" * 64
    )
    protocol.terminal = {
        "run_id": RUN_ID,
        "terminal_status": terminal,
        "reason": "committed reason",
        "manifest_path": "manifest.json",
    }
    assert runtime.poll() is True
    assert published[-1].state == terminal
    assert published[-1].sim_timestamp_ns == (123 if preterminal == "RUNNING" else 0)
    assert protocol.statuses[-1] == (
        "terminal-notified",
        {"run_id": RUN_ID, "notified": True},
    )
    count = len(published)
    assert runtime.poll() is True
    assert len(published) == count


def test_stale_ids_are_diagnosed_before_quiescence_and_ignored():
    runtime, _, published, diagnostics = _runtime()
    runtime.start()
    runtime.accept_artifact_status("22222222-2222-4222-8222-222222222222", True)
    assert [item.state for item in published] == ["STARTING"]
    assert diagnostics == ["ignored stale artifact status"]


def test_clock_requires_nonnegative_integer_and_readiness():
    runtime, _, _, _ = _runtime()
    runtime.start()
    with pytest.raises(ValueError):
        runtime.accept_clock(-1)
    with pytest.raises(TypeError):
        runtime.accept_clock(True)
