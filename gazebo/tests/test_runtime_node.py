import pytest
import yaml
from pathlib import Path

from drone_sim_gazebo.runtime.entrypoint import TransportError
from drone_sim_gazebo.runtime.model import ChildExited
from drone_sim_gazebo.runtime.runtime_node import (
    PublicEpochRendezvous,
    _probe_flight_exchange,
    _record_adapter_fault,
    _start_server_ready,
)


RUN_ID = "11111111-1111-4111-8111-111111111111"


def test_competition_bridge_is_minimal_and_directional():
    """A missing or bidirectional bridge would break physical authority boundaries."""
    bridge = yaml.safe_load(
        (Path(__file__).parents[1] / "config/bridge-competition.yaml").read_text(
            encoding="utf-8"
        )
    )
    by_ros_topic = {item["ros_topic_name"]: item for item in bridge}
    expected = {
        "/gazebo/private/clock",
        "/gazebo/private/iris/odometry",
        "/world/competition_mission/model/ground_plane/link/ground_link/sensor/iris_ground_contact/contact",
        "/gazebo/private/range/downward",
    }
    for aruco_id in (2, 3, 4):
        expected.update(
            {
                f"/gazebo/private/payload_{aruco_id}/pose",
                f"/gazebo/private/payload_{aruco_id}/contacts",
                f"/gazebo/private/payload_{aruco_id}/command",
                f"/gazebo/private/payload_{aruco_id}/joint_state",
                f"/gazebo/private/payload_{aruco_id}/result",
            }
        )

    assert set(by_ros_topic) == expected
    assert all(
        not token[0].isdigit()
        for topic in by_ros_topic
        for token in topic.split("/")
        if token
    )
    assert all(
        by_ros_topic[f"/gazebo/private/payload_{aruco_id}/command"]["direction"]
        == "ROS_TO_GZ"
        for aruco_id in (2, 3, 4)
    )
    for aruco_id in (2, 3, 4):
        for suffix in ("contacts", "command", "joint_state", "result"):
            assert (
                by_ros_topic[f"/gazebo/private/payload_{aruco_id}/{suffix}"]
                ["gz_topic_name"]
                == f"/gazebo/private/payload/{aruco_id}/{suffix}"
            )
    assert all(
        item["direction"] == "GZ_TO_ROS"
        for topic, item in by_ros_topic.items()
        if not topic.endswith("/command")
    )


class Server:
    def __init__(self):
        self.started = 0
        self.stopped = []

    def start(self):
        self.started += 1

    def stop(self, deadline):
        self.stopped.append(deadline)


class FlightTransport:
    def __init__(self, result=None):
        self.result = result
        self.timeouts = []

    def ready_flight_exchange(self, *, timeout):
        self.timeouts.append(timeout)
        return self.result


def test_flight_exchange_probe_uses_only_remaining_startup_budget():
    transport = FlightTransport(result={"online": True})

    assert _probe_flight_exchange(
        transport, deadline=10.25, monotonic=lambda: 10.0
    ) == {"online": True}
    assert transport.timeouts == [pytest.approx(0.25)]


def test_flight_exchange_probe_fails_after_existing_startup_deadline():
    transport = FlightTransport()

    with pytest.raises(TransportError, match="startup deadline"):
        _probe_flight_exchange(transport, deadline=10.0, monotonic=lambda: 10.0)
    assert transport.timeouts == []


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


def test_public_epoch_rendezvous_steps_to_target_then_waits_for_delivery_ack():
    calls = []

    class Transport:
        def set_paused(self, paused):
            calls.append(("paused", paused))

        def paused_sim_time_ns(self):
            calls.append(("stats",))
            return 44_000_000_000

        def run_to_sim_time(self, target_ns):
            calls.append(("run_to", target_ns))

    class Protocol:
        delivered = False

        def read_status(self, name):
            assert name == "mission-command-delivered"
            return {"delivered": True} if self.delivered else None

    protocol = Protocol()
    rendezvous = PublicEpochRendezvous(
        transport=Transport(),
        protocol=protocol,
        public_epoch_native_ns=90_000_000_000,
        activate_output=lambda: calls.append(("activate",)),
    )

    rendezvous.begin()
    assert calls == [
        ("activate",),
        ("paused", True),
    ]
    assert rendezvous.release_if_delivered() is False
    assert calls[-2:] == [
        ("stats",),
        ("run_to", 90_000_000_000),
    ]
    assert rendezvous.release_if_delivered() is False
    protocol.delivered = True
    assert rendezvous.release_if_delivered() is True
    assert calls[-1] == ("paused", False)


def test_public_epoch_rendezvous_waits_for_confirmed_pause_before_run_to():
    calls = []

    class Transport:
        def set_paused(self, paused):
            calls.append(("paused", paused))

        def paused_sim_time_ns(self):
            calls.append(("stats",))
            return None

        def run_to_sim_time(self, target_ns):
            calls.append(("run_to", target_ns))

    rendezvous = PublicEpochRendezvous(
        transport=Transport(),
        protocol=object(),
        public_epoch_native_ns=90_000_000_000,
        activate_output=lambda: calls.append(("activate",)),
    )

    rendezvous.begin()
    assert rendezvous.release_if_delivered() is False

    assert calls == [("activate",), ("paused", True), ("stats",)]
