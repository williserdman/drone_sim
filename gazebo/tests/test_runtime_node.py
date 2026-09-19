import pytest
import yaml
from pathlib import Path
from threading import Event, Thread
import time

from artifacts.runtime_status import (
    FlightExchange,
    MissionCommandDeliveredStatus,
    MissionExecutionReadyStatus,
)
from drone_sim_gazebo.runtime.entrypoint import TransportError
from drone_sim_gazebo.runtime.model import ChildExited
from drone_sim_gazebo.runtime.runtime_node import (
    PublicEpochRendezvous,
    _release_status_type_for_mission,
    _probe_flight_exchange,
    _record_adapter_fault,
    _start_server_ready,
)


RUN_ID = "11111111-1111-4111-8111-111111111111"


@pytest.mark.parametrize(
    ("mission", "expected"),
    [
        ("configured", MissionExecutionReadyStatus),
        ("comp2026_auto", MissionCommandDeliveredStatus),
        ("vertical_descent", MissionCommandDeliveredStatus),
    ],
)
def test_release_status_type_is_specific_to_configured_mission(mission, expected):
    assert _release_status_type_for_mission(mission) is expected


def test_competition_bridge_is_minimal_and_directional():
    """A missing or bidirectional bridge would break physical authority boundaries."""
    bridge = yaml.safe_load(
        (Path(__file__).parents[1] / "config/bridge-competition.yaml").read_text(
            encoding="utf-8"
        )
    )
    by_ros_topic = {item["ros_topic_name"]: item for item in bridge}
    expected = {
        "/gazebo/native/clock",
        "/gazebo/private/iris/odometry",
        "/gazebo/private/iris/contact",
        "/gazebo/private/range/downward",
    }
    for aruco_id in (2, 3, 4):
        expected.update(
            {
                f"/gazebo/private/payload_{aruco_id}/pose",
                f"/gazebo/private/payload_{aruco_id}/contact_state",
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
        for suffix in ("command", "joint_state", "result"):
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


def test_competition_bridge_uses_unique_recurrent_contact_truth_sources():
    """The 1 kHz event-only sensor stream must not feed exact 20 Hz truth."""
    from drone_sim_gazebo.ros_adapter.topics import gazebo_topics_for_world

    bridge = yaml.safe_load(
        (Path(__file__).parents[1] / "config/bridge-competition.yaml").read_text(
            encoding="utf-8"
        )
    )
    by_ros_topic = {item["ros_topic_name"]: item for item in bridge}
    required = gazebo_topics_for_world("competition_mission")

    for aruco_id in (2, 3, 4):
        native_topic = f"/gazebo/private/payload/{aruco_id}/contact_state"
        assert (
            by_ros_topic[f"/gazebo/private/payload_{aruco_id}/contact_state"]
            ["gz_topic_name"]
            == native_topic
        )
        assert native_topic in required
        assert all(
            f"model/payload_{aruco_id}/link/body/sensor/ground_contact" not in
            item["gz_topic_name"]
            for item in bridge
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
    flight_exchange = FlightExchange(True, 1, 1, 0, 0, 1, 0, 0, 0)
    transport = FlightTransport(result=flight_exchange)

    assert _probe_flight_exchange(
        transport, deadline=10.25, monotonic=lambda: 10.0
    ) == flight_exchange
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
    epoch_reached = False

    class Transport:
        def set_paused(self, paused):
            calls.append(("paused", paused))

        def run_to_sim_time(self, target_ns):
            calls.append(("run_to", target_ns))

    class Protocol:
        delivered = False

        def read_status(self, status_type):
            assert status_type is MissionCommandDeliveredStatus
            return MissionCommandDeliveredStatus(RUN_ID, 0) if self.delivered else None

    protocol = Protocol()
    rendezvous = PublicEpochRendezvous(
        transport=Transport(),
        protocol=protocol,
        public_epoch_native_ns=90_000_000_000,
        activate_output=lambda: calls.append(("activate",)),
        prepare_output=lambda: calls.append(("prepare",)),
        epoch_reached=lambda: epoch_reached,
    )

    rendezvous.start_warmup()
    assert calls == [
        ("prepare",),
        ("run_to", 90_050_000_000),
    ]
    rendezvous.begin()
    assert calls[-1] == ("activate",)
    assert rendezvous.release_if_delivered() is False
    protocol.delivered = True
    assert rendezvous.release_if_delivered() is False
    epoch_reached = True
    assert rendezvous.release_if_delivered() is True
    deadline = time.monotonic() + 2.0
    while calls.count(("paused", False)) < 2:
        assert time.monotonic() < deadline
        time.sleep(0.01)
    assert calls.count(("paused", False)) == 2


def test_public_epoch_rendezvous_can_wait_for_execution_readiness():
    observed_types = []

    class Transport:
        def run_to_sim_time(self, _target_ns):
            return None

    class Protocol:
        def read_status(self, status_type):
            observed_types.append(status_type)
            return None

    rendezvous = PublicEpochRendezvous(
        transport=Transport(),
        protocol=Protocol(),
        public_epoch_native_ns=0,
        activate_output=lambda: None,
        prepare_output=lambda: None,
        epoch_reached=lambda: True,
        release_status_type=MissionExecutionReadyStatus,
    )
    rendezvous.start_warmup()
    rendezvous.begin()

    assert rendezvous.release_if_delivered() is False
    assert observed_types == [MissionExecutionReadyStatus]


def test_public_epoch_rendezvous_keeps_run_to_after_rejected_reply():
    calls = []

    class Transport:
        def run_to_sim_time(self, target_ns):
            calls.append(target_ns)
            raise TransportError(
                "Gazebo world control rejected request: "
                "run_to_sim_time { sec: 15 nsec: 50000000 }"
            )

    rendezvous = PublicEpochRendezvous(
        transport=Transport(),
        protocol=object(),
        public_epoch_native_ns=15_000_000_000,
        activate_output=lambda: None,
        prepare_output=lambda: None,
        epoch_reached=lambda: False,
    )

    rendezvous.start_warmup()
    rendezvous.start_warmup()

    assert calls == [15_050_000_000]


def test_public_epoch_rendezvous_does_not_block_ros_callback_progress_during_unpause():
    calls = []
    unpause_started = Event()
    allow_unpause_reply = Event()

    class Transport:
        def set_paused(self, paused):
            calls.append(("paused", paused))
            if not paused:
                unpause_started.set()
                if not allow_unpause_reply.wait(2.0):
                    raise RuntimeError("test did not release world-control reply")

        def paused_sim_time_ns(self):
            calls.append(("stats",))
            return 90_000_000_000

        def run_to_sim_time(self, target_ns):
            calls.append(("run_to", target_ns))

    class Protocol:
        def read_status(self, status_type):
            assert status_type is MissionCommandDeliveredStatus
            return MissionCommandDeliveredStatus(RUN_ID, 0)

    rendezvous = PublicEpochRendezvous(
        transport=Transport(),
        protocol=Protocol(),
        public_epoch_native_ns=90_000_000_000,
        activate_output=lambda: calls.append(("activate",)),
        prepare_output=lambda: calls.append(("prepare",)),
        epoch_reached=lambda: True,
    )
    rendezvous.start_warmup()
    rendezvous.begin()
    assert rendezvous.release_if_delivered() is False

    outcomes = []
    caller = Thread(target=lambda: outcomes.append(rendezvous.release_if_delivered()))
    caller.start()
    assert unpause_started.wait(0.5)
    try:
        caller.join(0.1)
        assert not caller.is_alive(), "world-control reply blocked ROS callback progress"
        assert outcomes == [False]
        assert rendezvous.release_if_delivered() is False
    finally:
        allow_unpause_reply.set()
        caller.join(2.0)

    deadline = time.monotonic() + 2.0
    while not rendezvous.release_if_delivered():
        assert time.monotonic() < deadline
        time.sleep(0.01)
    assert calls.count(("paused", False)) == 2


def test_public_epoch_rendezvous_propagates_unpause_failure_before_epoch():
    unpause_attempted = Event()

    class Transport:
        def set_paused(self, paused):
            if not paused:
                unpause_attempted.set()
                raise TransportError("unpause control failed")

        def paused_sim_time_ns(self):
            return 90_000_000_000

        def run_to_sim_time(self, _target_ns):
            return None

    class Protocol:
        def read_status(self, status_type):
            assert status_type is MissionCommandDeliveredStatus
            return MissionCommandDeliveredStatus(RUN_ID, 0)

    rendezvous = PublicEpochRendezvous(
        transport=Transport(),
        protocol=Protocol(),
        public_epoch_native_ns=90_000_000_000,
        activate_output=lambda: None,
        prepare_output=lambda: None,
        epoch_reached=lambda: False,
    )
    rendezvous.start_warmup()
    rendezvous.begin()
    assert rendezvous.release_if_delivered() is False
    assert unpause_attempted.wait(0.5)

    with pytest.raises(TransportError, match="unpause control failed"):
        rendezvous.release_if_delivered()


def test_public_epoch_rendezvous_tolerates_rejected_reply_after_epoch_progress():
    calls = []

    class Transport:
        def set_paused(self, paused):
            calls.append(("paused", paused))
            if len(calls) == 1:
                raise TransportError("Gazebo world control rejected request: pause: false")

        def run_to_sim_time(self, _target_ns):
            return None

    class Protocol:
        def read_status(self, status_type):
            assert status_type is MissionCommandDeliveredStatus
            return MissionCommandDeliveredStatus(RUN_ID, 0)

    rendezvous = PublicEpochRendezvous(
        transport=Transport(),
        protocol=Protocol(),
        public_epoch_native_ns=90_000_000_000,
        activate_output=lambda: None,
        prepare_output=lambda: None,
        epoch_reached=lambda: True,
    )
    rendezvous.start_warmup()
    rendezvous.begin()
    assert rendezvous.release_if_delivered() is False

    deadline = time.monotonic() + 2.0
    while not rendezvous.release_if_delivered():
        assert time.monotonic() < deadline
        time.sleep(0.01)

    while len(calls) < 2:
        assert time.monotonic() < deadline
        time.sleep(0.01)
    assert calls == [("paused", False), ("paused", False)]


def test_public_epoch_rendezvous_waits_for_command_delivery():
    calls = []

    class Transport:
        def set_paused(self, paused):
            calls.append(("paused", paused))

        def run_to_sim_time(self, target_ns):
            calls.append(("run_to", target_ns))

    class Protocol:
        def read_status(self, status_type):
            assert status_type is MissionCommandDeliveredStatus
            return None

    rendezvous = PublicEpochRendezvous(
        transport=Transport(),
        protocol=Protocol(),
        public_epoch_native_ns=90_000_000_000,
        activate_output=lambda: calls.append(("activate",)),
        prepare_output=lambda: calls.append(("prepare",)),
        epoch_reached=lambda: True,
    )

    rendezvous.start_warmup()
    rendezvous.begin()
    assert rendezvous.release_if_delivered() is False

    assert calls == [
        ("prepare",),
        ("run_to", 90_050_000_000),
        ("activate",),
    ]
