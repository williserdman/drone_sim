from pathlib import Path

from drone_sim_gazebo.runtime import (
    ActivateOutput,
    BeginFinalization,
    PublishGazeboReady,
    RequestSteps,
    SetPaused,
    StopServer,
    WriteQuiescence,
    WriteRuntimeFailure,
    WriteSourceFinished,
)
from drone_sim_gazebo.runtime.entrypoint import ActionExecutor
from drone_sim_gazebo.server import NativeArtifactSummary


RUN_ID = "11111111-1111-4111-8111-111111111111"


class Protocol:
    def __init__(self):
        self.statuses = []
        self.quiescence = []

    def write_status(self, name, document):
        self.statuses.append((name, document))

    def write_quiescence(self, module):
        self.quiescence.append(module)


class Status:
    def __init__(self):
        self.ready = []

    def write_gazebo_ready(self):
        self.ready.append(True)


class Transport:
    def __init__(self):
        self.calls = []

    def request_steps(self, count):
        self.calls.append(("step", count))

    def set_paused(self, paused):
        self.calls.append(("pause", paused))


class Children:
    def __init__(self):
        self.deadlines = []
        self.error = None

    def stop(self, deadline):
        self.deadlines.append(deadline)
        if self.error is not None:
            raise self.error


class Server:
    def __init__(self, summary):
        self.summary = summary
        self.deadlines = []

    def stop(self, deadline):
        self.deadlines.append(deadline)
        return self.summary


def _executor(tmp_path, *, activate_output=lambda: None):
    summary = NativeArtifactSummary(
        tmp_path / RUN_ID / "gazebo/server.log",
        tmp_path / RUN_ID / "gazebo/state/state.tlog",
        0,
        True,
    )
    protocol, status, transport, children, server = (
        Protocol(), Status(), Transport(), Children(), Server(summary)
    )
    executor = ActionExecutor(
        run_id=RUN_ID,
        protocol=protocol,
        status=status,
        transport=transport,
        children=children,
        server=server,
        activate_output=activate_output,
    )
    return executor, protocol, status, transport, children, server, summary


def test_action_executor_maps_readiness_control_and_durable_facts(tmp_path):
    executor, protocol, status, transport, *_ = _executor(tmp_path)

    followups = executor.apply(
        (
            PublishGazeboReady(),
            RequestSteps(1),
            SetPaused(False),
            WriteSourceFinished(2_000_000_000),
            WriteRuntimeFailure("broken", ("gazebo/server.log.partial",)),
            BeginFinalization("FAILED", "broken"),
        )
    )

    assert followups == ()
    assert status.ready == [True]
    assert transport.calls == [("step", 1), ("pause", False)]
    assert protocol.statuses == [
        ("source-finished", {"run_id": RUN_ID, "finished": True, "sim_timestamp_ns": 2_000_000_000}),
        ("runtime-failure", {"run_id": RUN_ID, "module": "gazebo", "reason": "broken", "diagnostic_paths": ["gazebo/server.log.partial"]}),
    ]


def test_unpause_does_not_activate_public_output_during_private_warmup(tmp_path):
    ordering = []
    executor, _protocol, _status, transport, *_ = _executor(
        tmp_path, activate_output=lambda: ordering.append("activate")
    )
    original = transport.set_paused

    def record_unpause(paused):
        ordering.append(("pause", paused))
        original(paused)

    transport.set_paused = record_unpause

    executor.apply((SetPaused(False),))

    assert ordering == [("pause", False)]


def test_explicit_activate_output_action_does_not_unpause_a_second_time(tmp_path):
    ordering = []
    executor, _protocol, _status, transport, *_ = _executor(
        tmp_path, activate_output=lambda: ordering.append("activate")
    )

    executor.apply((ActivateOutput(),))

    assert ordering == ["activate"]
    assert transport.calls == []


def test_stop_uses_same_absolute_deadline_for_children_and_server(tmp_path):
    executor, _protocol, _status, _transport, children, server, summary = _executor(tmp_path)

    followups = executor.apply((StopServer(123.0),))

    assert children.deadlines == [123.0]
    assert server.deadlines == [123.0]
    assert followups[0].run_id == RUN_ID
    assert followups[0].native_artifacts == summary


def test_stop_still_stops_server_when_child_cleanup_fails(tmp_path):
    executor, _protocol, _status, _transport, children, server, _summary = _executor(tmp_path)
    children.error = RuntimeError("bridge cleanup failed")

    followups = executor.apply((StopServer(123.0),))

    assert children.deadlines == [123.0]
    assert server.deadlines == [123.0]
    assert len(followups) == 1
    assert followups[0].run_id == RUN_ID
    assert followups[0].reason == "bridge cleanup failed"


def test_quiescence_is_written_only_by_explicit_model_action(tmp_path):
    executor, protocol, *_rest, summary = _executor(tmp_path)
    assert protocol.quiescence == []

    executor.apply((WriteQuiescence(summary),))

    assert protocol.quiescence == ["gazebo"]
