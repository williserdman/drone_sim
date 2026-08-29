from __future__ import annotations

import json
from pathlib import Path
import sys
from threading import Event, Thread
from types import ModuleType, SimpleNamespace

import pytest

import drone_sim_electromagnet.runtime_node as runtime_node
from drone_sim_electromagnet.controller import PayloadGateway
from drone_sim_electromagnet.payload import PayloadRequest
from drone_sim_electromagnet.runtime_node import (
    RuntimeConfig,
    _competition_main,
    assign_stamp,
    parse_physical_result,
    stamp_ns,
)


RUN_ID = "00000000-0000-4000-8000-000000000001"
ROOT = Path(__file__).parents[2]


def test_ros_timestamp_conversion_is_exact() -> None:
    source = SimpleNamespace(sec=12, nanosec=345)
    destination = SimpleNamespace(sec=0, nanosec=0)
    assert stamp_ns(source) == 12_000_000_345
    assign_stamp(destination, 12_000_000_345)
    assert (destination.sec, destination.nanosec) == (12, 345)


def test_competition_exposes_inert_scenario_event_type_for_exact_bag_inventory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Publisher:
        def __init__(self) -> None:
            self.messages: list[object] = []

        def publish(self, message: object) -> None:
            self.messages.append(message)

    class Node:
        last: "Node"

        def __init__(self, _name: str) -> None:
            Node.last = self
            self.publishers: list[tuple[object, str, object, Publisher]] = []

        def create_publisher(self, message_type, topic, qos):
            publisher = Publisher()
            self.publishers.append((message_type, topic, qos, publisher))
            return publisher

        def create_subscription(self, *_args, **_kwargs):
            return object()

        def create_service(self, *_args, **_kwargs):
            return object()

        def count_publishers(self, _topic: str) -> int:
            return 0

        def destroy_node(self) -> None:
            pass

    class Executor:
        def __init__(self, *, num_threads: int) -> None:
            assert num_threads == 4

        def add_node(self, _node: object) -> None:
            pass

        def spin_once(self, *, timeout_sec: float) -> None:
            assert timeout_sec == 0.05

        def shutdown(self, *, timeout_sec: float) -> None:
            assert timeout_sec == 6.0

    class Protocol:
        def __init__(self, _run_directory: Path, _run_id: str) -> None:
            pass

        def read_finalize_request(self):
            return {"requested_terminal": "FAILED"}

        def write_quiescence(self, module: str) -> None:
            assert module == "electromagnet"

        def close(self) -> None:
            pass

    class QoSProfile:
        def __init__(self, *, depth, reliability, durability) -> None:
            self.depth = depth
            self.reliability = reliability
            self.durability = durability

    class DurabilityPolicy:
        TRANSIENT_LOCAL = "transient"
        VOLATILE = "volatile"

    class ReliabilityPolicy:
        RELIABLE = "reliable"

    class PayloadCommand:
        class Request:
            ATTACH = 1
            RELEASE = 2

    rclpy = ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    callback_groups = ModuleType("rclpy.callback_groups")
    callback_groups.ReentrantCallbackGroup = object
    executors = ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = Executor
    node_module = ModuleType("rclpy.node")
    node_module.Node = Node
    qos_module = ModuleType("rclpy.qos")
    qos_module.DurabilityPolicy = DurabilityPolicy
    qos_module.QoSProfile = QoSProfile
    qos_module.ReliabilityPolicy = ReliabilityPolicy
    message_module = ModuleType("simulation_interfaces.msg")
    for name in (
        "GroundTruth",
        "PayloadEvent",
        "PayloadState",
        "RunState",
        "ScenarioEvent",
    ):
        setattr(message_module, name, type(name, (), {}))
    service_module = ModuleType("simulation_interfaces.srv")
    service_module.PayloadCommand = PayloadCommand
    string_module = ModuleType("std_msgs.msg")
    string_module.String = type("String", (), {})
    for name, module in {
        "rclpy": rclpy,
        "rclpy.callback_groups": callback_groups,
        "rclpy.executors": executors,
        "rclpy.node": node_module,
        "rclpy.qos": qos_module,
        "simulation_interfaces.msg": message_module,
        "simulation_interfaces.srv": service_module,
        "std_msgs.msg": string_module,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(runtime_node, "RuntimeProtocol", Protocol)
    monkeypatch.setattr(runtime_node.signal, "signal", lambda *_args: None)

    assert _competition_main(RuntimeConfig.competition_defaults(RUN_ID)) == 0

    by_topic = {
        topic: (message_type, qos, publisher)
        for message_type, topic, qos, publisher in Node.last.publishers
    }
    message_type, qos, publisher = by_topic["/simulation/scenario_events"]
    assert message_type.__name__ == "ScenarioEvent"
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert publisher.messages == []


def write_config(run_directory: Path, scenario: str) -> Path:
    configuration = run_directory / "configuration"
    configuration.mkdir(parents=True)
    document: dict[str, object] = {"run_id": RUN_ID, "scenario": scenario}
    if scenario == "competition_v1":
        (configuration / "course.yaml").write_bytes((ROOT / "config/course.yaml").read_bytes())
        (configuration / "scenario.yaml").write_bytes(
            (ROOT / "config/scenario.yaml").read_bytes()
        )
        document["competition"] = {
            "course": "course.yaml",
            "scenario": "scenario.yaml",
        }
    path = configuration / "run.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def test_runtime_config_loads_authority_from_current_resolved_competition(
    tmp_path: Path,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_config(run_directory, "competition_v1")

    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )

    assert config.scenario == "competition_v1"
    assert config.payload_capacity == 1
    assert config.max_center_error_m == 0.075
    assert config.payload_zones == {2: None, 3: "WA", 4: "WM"}
    assert config.pickup_zones["WA"].contains((-45.72, -9.144))


def test_runtime_config_keeps_descent_without_competition_sources(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = write_config(run_directory, "descent_v1")
    config = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )
    assert config.scenario == "descent_v1"


def test_physical_result_parser_accepts_only_exact_coordinator_wire() -> None:
    result = parse_physical_result(
        "payload-result-v1|run:3:attach:1|confirmed|attached|OK"
    )
    assert (result.command_id, result.status, result.state, result.code) == (
        "run:3:attach:1",
        "confirmed",
        "attached",
        "OK",
    )
    with pytest.raises(ValueError, match="payload-result-v1"):
        parse_physical_result("payload-result-v1|too|few")


def ready_gateway(
    *,
    commands: list[tuple[int, str]],
    events: list[object],
    confirmation: str | None,
    timeout_seconds: float = 0.05,
) -> PayloadGateway:
    config = RuntimeConfig.competition_defaults(RUN_ID)
    gateway_ref: list[PayloadGateway] = []

    def publish_command(marker: int, wire: str) -> None:
        commands.append((marker, wire))
        if confirmation is not None:
            gateway_ref[0].accept_result(marker, confirmation)

    gateway = PayloadGateway(
        config.authority(),
        publish_command=publish_command,
        publish_event=events.append,
        confirmation_timeout_seconds=timeout_seconds,
    )
    gateway_ref.append(gateway)
    gateway.accept_vehicle(RUN_ID, 50_000_000, (-45.72, -9.144), True)
    gateway.accept_payload(RUN_ID, 50_000_000, 2, (0.0, 0.0), False, False)
    gateway.accept_payload(RUN_ID, 50_000_000, 3, (-45.72, -9.144), True, False)
    gateway.accept_payload(RUN_ID, 50_000_000, 4, (-45.72, 9.144), True, False)
    return gateway


def seed_gateway(
    gateway: PayloadGateway,
    *,
    timestamp_ns: int = 50_000_000,
    vehicle_xy: tuple[float, float] = (-45.72, -9.144),
    attached: frozenset[int] = frozenset(),
) -> None:
    gateway.accept_vehicle(RUN_ID, timestamp_ns, vehicle_xy, True)
    gateway.accept_payload(
        RUN_ID, timestamp_ns, 2, (0.0, 0.0), False, 2 in attached
    )
    gateway.accept_payload(
        RUN_ID, timestamp_ns, 3, (-45.72, -9.144), True, 3 in attached
    )
    gateway.accept_payload(
        RUN_ID, timestamp_ns, 4, (-45.72, 9.144), True, 4 in attached
    )


def test_gateway_returns_only_after_physical_confirmation_and_replays_exactly() -> None:
    commands: list[tuple[int, str]] = []
    events: list[object] = []
    wire = "payload-result-v1|run:3:attach:1|confirmed|attached|OK"
    gateway = ready_gateway(commands=commands, events=events, confirmation=wire)
    request = PayloadRequest(RUN_ID, 3, "attach", "run:3:attach:1")

    first = gateway.execute(request)
    replay = gateway.execute(request)

    assert (first.accepted, first.code, first.response_sequence) == (True, "OK", 1)
    assert replay == first
    assert commands == [(3, "payload-command-v1|run:3:attach:1|attach")]
    assert len(events) == 1
    assert (events[0].aruco_id, events[0].action, events[0].state) == (
        3,
        "attach",
        "attached",
    )


def test_gateway_timeout_fails_without_event_or_guessed_attachment() -> None:
    commands: list[tuple[int, str]] = []
    events: list[object] = []
    gateway = ready_gateway(commands=commands, events=events, confirmation=None)

    result = gateway.execute(PayloadRequest(RUN_ID, 3, "attach", "timeout:1"))

    assert (result.accepted, result.code) == (False, "PHYSICAL_CONFIRMATION_TIMEOUT")
    assert commands == [(3, "payload-command-v1|timeout:1|attach")]
    assert events == []
    assert gateway.attached_id is None


def test_gateway_rejects_nonmatching_physical_confirmation() -> None:
    commands: list[tuple[int, str]] = []
    events: list[object] = []
    gateway = ready_gateway(
        commands=commands,
        events=events,
        confirmation="payload-result-v1|run:3:attach:1|confirmed|detached|OK",
    )

    result = gateway.execute(
        PayloadRequest(RUN_ID, 3, "attach", "run:3:attach:1")
    )

    assert (result.accepted, result.code) == (
        False,
        "PHYSICAL_CONFIRMATION_MISMATCH",
    )
    assert events == []
    assert gateway.attached_id is None


def test_concurrent_exact_duplicate_waits_and_replays_identical_response() -> None:
    commands: list[tuple[int, str]] = []
    events: list[object] = []
    command_published = Event()
    duplicate_called = Event()
    gateway = PayloadGateway(
        RuntimeConfig.competition_defaults(RUN_ID).authority(),
        publish_command=lambda marker, wire: (
            commands.append((marker, wire)),
            command_published.set(),
        ),
        publish_event=events.append,
        confirmation_timeout_seconds=2.0,
    )
    seed_gateway(gateway)
    request = PayloadRequest(RUN_ID, 3, "attach", "concurrent:duplicate")
    responses: list[object] = []

    first = Thread(target=lambda: responses.append(gateway.execute(request)))

    def call_duplicate() -> None:
        duplicate_called.set()
        responses.append(gateway.execute(request))

    duplicate = Thread(target=call_duplicate)
    first.start()
    assert command_published.wait(1.0)
    duplicate.start()
    assert duplicate_called.wait(1.0)
    assert duplicate.is_alive()
    gateway.accept_result(
        3,
        "payload-result-v1|concurrent:duplicate|confirmed|attached|OK",
    )
    first.join(1.0)
    duplicate.join(1.0)

    assert first.is_alive() is False
    assert duplicate.is_alive() is False
    assert responses[0] == responses[1]
    assert responses[0].response_sequence == 1
    assert commands == [
        (3, "payload-command-v1|concurrent:duplicate|attach")
    ]
    assert len(events) == 1


def test_conflicting_reuse_cannot_poison_original_response_cache() -> None:
    commands: list[tuple[int, str]] = []
    command_published = Event()
    conflict_called = Event()
    gateway = PayloadGateway(
        RuntimeConfig.competition_defaults(RUN_ID).authority(),
        publish_command=lambda marker, wire: (
            commands.append((marker, wire)),
            command_published.set(),
        ),
        publish_event=lambda _event: None,
        confirmation_timeout_seconds=2.0,
    )
    seed_gateway(gateway)
    original = PayloadRequest(RUN_ID, 3, "attach", "concurrent:conflict")
    conflict = PayloadRequest(RUN_ID, 4, "attach", "concurrent:conflict")
    outcomes: dict[str, object] = {}
    first = Thread(
        target=lambda: outcomes.setdefault("first", gateway.execute(original))
    )

    def call_conflict() -> None:
        conflict_called.set()
        outcomes.setdefault("conflict", gateway.execute(conflict))

    second = Thread(target=call_conflict)
    first.start()
    assert command_published.wait(1.0)
    second.start()
    assert conflict_called.wait(1.0)
    assert second.is_alive()
    gateway.accept_result(
        3,
        "payload-result-v1|concurrent:conflict|confirmed|attached|OK",
    )
    first.join(1.0)
    second.join(1.0)

    replay = gateway.execute(original)
    assert replay == outcomes["first"]
    assert (outcomes["conflict"].accepted, outcomes["conflict"].code) == (
        False,
        "COMMAND_ID_CONFLICT",
    )
    assert commands == [(3, "payload-command-v1|concurrent:conflict|attach")]


def test_distinct_physical_commands_serialize_before_second_validation() -> None:
    commands: list[tuple[int, str]] = []
    events: list[object] = []
    first_published = Event()
    gateway = PayloadGateway(
        RuntimeConfig.competition_defaults(RUN_ID).authority(),
        publish_command=lambda marker, wire: (
            commands.append((marker, wire)),
            first_published.set(),
        ),
        publish_event=events.append,
        confirmation_timeout_seconds=2.0,
    )
    seed_gateway(gateway)
    requests = (
        PayloadRequest(RUN_ID, 3, "attach", "serialized:1"),
        PayloadRequest(RUN_ID, 3, "attach", "serialized:2"),
    )
    outcomes: dict[str, object] = {}
    first = Thread(
        target=lambda: outcomes.setdefault("first", gateway.execute(requests[0]))
    )
    second = Thread(
        target=lambda: outcomes.setdefault("second", gateway.execute(requests[1]))
    )
    first.start()
    assert first_published.wait(1.0)
    second.start()
    assert len(commands) == 1
    gateway.accept_result(
        3, "payload-result-v1|serialized:1|confirmed|attached|OK"
    )
    first.join(1.0)
    second.join(1.0)

    assert (outcomes["first"].accepted, outcomes["first"].code) == (True, "OK")
    assert (outcomes["second"].accepted, outcomes["second"].code) == (
        False,
        "CAPACITY_OCCUPIED",
    )
    assert commands == [(3, "payload-command-v1|serialized:1|attach")]
    assert len(events) == 1


def test_recurrent_payload_truth_updates_attachment_after_confirmation() -> None:
    commands: list[tuple[int, str]] = []
    events: list[object] = []
    gateway = ready_gateway(
        commands=commands,
        events=events,
        confirmation="payload-result-v1|truth:attach|confirmed|attached|OK",
    )
    assert gateway.execute(
        PayloadRequest(RUN_ID, 3, "attach", "truth:attach")
    ).accepted
    assert gateway.attached_id == 3

    seed_gateway(gateway, timestamp_ns=100_000_000)

    assert gateway.attached_id is None


def test_timestamp_regressions_do_not_overwrite_current_authorization_facts() -> None:
    commands: list[tuple[int, str]] = []
    events: list[object] = []
    gateway = ready_gateway(
        commands=commands,
        events=events,
        confirmation="payload-result-v1|monotonic:1|confirmed|attached|OK",
    )
    gateway.accept_vehicle(RUN_ID, 40_000_000, (-40.0, -9.144), False)
    gateway.accept_payload(
        RUN_ID, 40_000_000, 3, (-45.72, 9.144), False, False
    )

    result = gateway.execute(PayloadRequest(RUN_ID, 3, "attach", "monotonic:1"))

    assert result.accepted is True


def test_common_tick_older_than_half_a_sim_second_fails_closed() -> None:
    commands: list[tuple[int, str]] = []
    gateway = ready_gateway(commands=commands, events=[], confirmation=None)
    gateway.accept_vehicle(RUN_ID, 600_000_001, (-45.72, -9.144), True)

    result = gateway.execute(PayloadRequest(RUN_ID, 3, "attach", "stale:1"))

    assert (result.accepted, result.code) == (False, "STALE_PHYSICAL_STATE")
    assert commands == []


def test_gateway_uses_latest_recent_exact_common_tick_when_payloads_arrive_ahead() -> None:
    commands: list[tuple[int, str]] = []
    events: list[object] = []
    gateway_ref: list[PayloadGateway] = []

    def publish_command(marker: int, wire: str) -> None:
        commands.append((marker, wire))
        gateway_ref[0].accept_result(
            marker,
            "payload-result-v1|run:2:release:1|confirmed|detached|OK",
        )

    gateway = PayloadGateway(
        RuntimeConfig.competition_defaults(RUN_ID).authority(),
        publish_command=publish_command,
        publish_event=events.append,
        confirmation_timeout_seconds=0.05,
    )
    gateway_ref.append(gateway)
    seed_gateway(
        gateway,
        timestamp_ns=162_650_000_000,
        vehicle_xy=(0.0, 0.0),
        attached=frozenset({2}),
    )
    gateway.accept_payload(
        RUN_ID, 162_850_000_000, 2, (0.01, 0.0), False, True
    )
    gateway.accept_payload(
        RUN_ID, 162_850_000_000, 3, (-45.72, -9.144), True, False
    )
    gateway.accept_payload(
        RUN_ID, 162_850_000_000, 4, (-45.72, 9.144), True, False
    )

    result = gateway.execute(
        PayloadRequest(RUN_ID, 2, "release", "run:2:release:1")
    )

    assert (result.accepted, result.code) == (True, "OK")
    assert commands == [
        (2, "payload-command-v1|run:2:release:1|detach")
    ]
    assert len(events) == 1


def test_latest_common_tick_cannot_fall_back_to_an_older_valid_world() -> None:
    commands: list[tuple[int, str]] = []
    gateway = PayloadGateway(
        RuntimeConfig.competition_defaults(RUN_ID).authority(),
        publish_command=lambda marker, wire: commands.append((marker, wire)),
        publish_event=lambda _event: None,
        confirmation_timeout_seconds=0.05,
    )
    seed_gateway(gateway, timestamp_ns=1_000_000_000)
    gateway.accept_vehicle(
        RUN_ID, 1_050_000_000, (-45.72, -9.144), False
    )
    gateway.accept_payload(
        RUN_ID, 1_050_000_000, 2, (0.0, 0.0), False, False
    )
    gateway.accept_payload(
        RUN_ID, 1_050_000_000, 3, (-45.72, -9.144), True, False
    )
    gateway.accept_payload(
        RUN_ID, 1_050_000_000, 4, (-45.72, 9.144), True, False
    )

    result = gateway.execute(
        PayloadRequest(RUN_ID, 3, "attach", "latest:moving")
    )

    assert (result.accepted, result.code) == (False, "NOT_LANDED")
    assert commands == []


def test_changed_grounded_or_attachment_truth_after_common_tick_fails_closed() -> None:
    for mutation in ("grounded", "attachment"):
        commands: list[tuple[int, str]] = []
        gateway = PayloadGateway(
            RuntimeConfig.competition_defaults(RUN_ID).authority(),
            publish_command=lambda marker, wire: commands.append((marker, wire)),
            publish_event=lambda _event: None,
            confirmation_timeout_seconds=0.05,
        )
        seed_gateway(
            gateway,
            timestamp_ns=1_000_000_000,
            vehicle_xy=(0.0, 0.0),
            attached=frozenset({2}),
        )
        if mutation == "grounded":
            gateway.accept_vehicle(RUN_ID, 1_050_000_000, (0.0, 0.0), False)
        else:
            gateway.accept_payload(
                RUN_ID, 1_050_000_000, 2, (0.0, 0.0), False, False
            )

        result = gateway.execute(
            PayloadRequest(RUN_ID, 2, "release", f"changed:{mutation}")
        )

        assert (result.accepted, result.code) == (
            False,
            "STALE_PHYSICAL_STATE",
        )
        assert commands == []


def test_inconsistent_latest_attachment_history_fails_closed() -> None:
    commands: list[tuple[int, str]] = []
    gateway = PayloadGateway(
        RuntimeConfig.competition_defaults(RUN_ID).authority(),
        publish_command=lambda marker, wire: commands.append((marker, wire)),
        publish_event=lambda _event: None,
        confirmation_timeout_seconds=0.05,
    )
    seed_gateway(
        gateway,
        timestamp_ns=1_000_000_000,
        vehicle_xy=(0.0, 0.0),
        attached=frozenset({2}),
    )
    gateway.accept_payload(
        RUN_ID, 1_050_000_000, 3, (-45.72, -9.144), True, True
    )

    result = gateway.execute(
        PayloadRequest(RUN_ID, 2, "release", "inconsistent:attachment")
    )

    assert (result.accepted, result.code) == (False, "INVALID_PHYSICAL_STATE")
    assert commands == []


def test_pruned_history_cannot_be_restored_by_a_regressing_sample() -> None:
    commands: list[tuple[int, str]] = []
    gateway = PayloadGateway(
        RuntimeConfig.competition_defaults(RUN_ID).authority(),
        publish_command=lambda marker, wire: commands.append((marker, wire)),
        publish_event=lambda _event: None,
        confirmation_timeout_seconds=0.05,
    )
    seed_gateway(
        gateway,
        timestamp_ns=1_000_000_000,
        vehicle_xy=(0.0, 0.0),
        attached=frozenset({2}),
    )
    for marker, xy, grounded, attached in (
        (2, (0.0, 0.0), False, True),
        (3, (-45.72, -9.144), True, False),
        (4, (-45.72, 9.144), True, False),
    ):
        gateway.accept_payload(
            RUN_ID, 1_500_000_001, marker, xy, grounded, attached
        )
    gateway.accept_payload(
        RUN_ID, 1_000_000_000, 2, (0.0, 0.0), False, True
    )

    result = gateway.execute(
        PayloadRequest(RUN_ID, 2, "release", "pruned:regression")
    )

    assert (result.accepted, result.code) == (False, "STALE_PHYSICAL_STATE")
    assert commands == []


def test_multiple_attached_payloads_are_invalid_not_free_capacity() -> None:
    commands: list[tuple[int, str]] = []
    gateway = PayloadGateway(
        RuntimeConfig.competition_defaults(RUN_ID).authority(),
        publish_command=lambda marker, wire: commands.append((marker, wire)),
        publish_event=lambda _event: None,
        confirmation_timeout_seconds=0.05,
    )
    seed_gateway(
        gateway,
        vehicle_xy=(-45.72, 9.144),
        attached=frozenset({2, 3}),
    )

    result = gateway.execute(PayloadRequest(RUN_ID, 4, "attach", "invalid:capacity"))

    assert (result.accepted, result.code) == (False, "INVALID_PHYSICAL_STATE")
    assert commands == []


def test_competition_readiness_requires_all_current_facts_and_result_publishers() -> None:
    gateway = PayloadGateway(
        RuntimeConfig.competition_defaults(RUN_ID).authority(),
        publish_command=lambda _marker, _wire: None,
        publish_event=lambda _event: None,
    )
    assert gateway.ready(result_publishers=frozenset({2, 3, 4}), service_ready=True) is False
    gateway.accept_vehicle(RUN_ID, 50_000_000, (0.0, 0.0), True)
    for marker in (2, 3, 4):
        gateway.accept_payload(RUN_ID, 50_000_000, marker, (0.0, 0.0), True, marker == 2)
    assert gateway.ready(result_publishers=frozenset({2, 3}), service_ready=True) is False
    assert gateway.ready(result_publishers=frozenset({2, 3, 4}), service_ready=False) is False
    assert gateway.ready(result_publishers=frozenset({2, 3, 4}), service_ready=True) is True
    gateway.accept_payload(
        RUN_ID, 100_000_000, 2, (0.0, 0.0), True, True
    )
    assert gateway.ready(result_publishers=frozenset({2, 3, 4}), service_ready=True) is False
