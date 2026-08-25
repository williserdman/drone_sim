from drone_sim_gazebo.runtime.entrypoint import TransportError
from drone_sim_gazebo.runtime.model import ChildExited
from drone_sim_gazebo.runtime.runtime_node import _record_adapter_fault, _start_server_ready


RUN_ID = "11111111-1111-4111-8111-111111111111"


class Server:
    def __init__(self):
        self.started = 0
        self.stopped = []

    def start(self):
        self.started += 1

    def stop(self, deadline):
        self.stopped.append(deadline)


def test_transport_discovery_failure_stops_started_server():
    server = Server()

    def fail_discovery(_transport, *, deadline):
        assert deadline == 10.0
        raise TransportError("missing endpoint")

    try:
        _start_server_ready(
            server,
            object(),
            startup_deadline=10.0,
            cleanup_deadline=15.0,
            wait_transport=fail_discovery,
        )
    except TransportError as error:
        assert str(error) == "missing endpoint"
    else:
        raise AssertionError("transport discovery failure was not propagated")

    assert server.started == 1
    assert server.stopped == [15.0]


def test_adapter_fault_retains_exact_reason_in_structured_log(monkeypatch):
    events = []
    inbox = []
    monkeypatch.setattr(
        "drone_sim_gazebo.runtime.runtime_node._event",
        lambda run_id, name, **values: events.append((run_id, name, values)),
    )

    _record_adapter_fault(RUN_ID, inbox, "odometry and contact timestamps do not align")

    assert events == [
        (
            RUN_ID,
            "adapter_fault",
            {"fields": {"reason": "odometry and contact timestamps do not align"}},
        )
    ]
    assert inbox == [ChildExited(RUN_ID, "adapter", 1)]
