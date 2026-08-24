from drone_sim_gazebo.runtime.entrypoint import TransportError
from drone_sim_gazebo.runtime.runtime_node import _start_server_ready


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
