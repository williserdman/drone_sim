from __future__ import annotations

import json
from pathlib import Path
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
