from __future__ import annotations

import json
from pathlib import Path
from threading import Event, Thread
from types import SimpleNamespace

import pytest

from drone_sim_electromagnet.controller import PayloadGateway
from drone_sim_electromagnet.payload import PayloadRequest
from drone_sim_electromagnet.runtime_node import (
    RuntimeConfig,
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


def test_mixed_timestamp_world_fails_closed_without_a_command() -> None:
    commands: list[tuple[int, str]] = []
    gateway = ready_gateway(commands=commands, events=[], confirmation=None)
    gateway.accept_vehicle(RUN_ID, 100_000_000, (-45.72, -9.144), True)

    result = gateway.execute(PayloadRequest(RUN_ID, 3, "attach", "stale:1"))

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
