from __future__ import annotations

import importlib.util
import importlib.metadata
import ipaddress
import json
import os
from pathlib import Path
import signal
import socket
import struct
import threading
from types import SimpleNamespace
import builtins

import pytest
import yaml

from drone_sim_ardupilot import json_peer


JsonPeer = json_peer.JsonPeer


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("qgc_sitl_probe", HERE / "listener_probe.py")
assert SPEC is not None and SPEC.loader is not None
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)

HOST_SPEC = importlib.util.spec_from_file_location("qgc_sitl_host", HERE / "host_runner.py")
assert HOST_SPEC is not None and HOST_SPEC.loader is not None
host = importlib.util.module_from_spec(HOST_SPEC)
HOST_SPEC.loader.exec_module(host)

START_SPEC = importlib.util.spec_from_file_location(
    "qgc_sitl_start_arducopter", HERE / "start_arducopter.py"
)
assert START_SPEC is not None and START_SPEC.loader is not None
start_arducopter = importlib.util.module_from_spec(START_SPEC)
START_SPEC.loader.exec_module(start_arducopter)


def _passing_result() -> dict[str, object]:
    return {
        "schema_version": 1,
        "status": "passed",
        "stage": "complete",
        "reason": None,
        "identities": {
            "injector": [200, 190],
            "companion": [1, 191],
            "flight_controller": [1, 1],
            "wire_protocol": "2.0",
        },
        "firmware": {
            "flight_sw_version": 0x040507FF,
            "flight_custom_version_hex": "3261336463346237",
            "discovery_observed": True,
            "validation_observed": True,
        },
        "passive_telemetry_receipts": {
            "meaning": probe.PASSIVE_RECEIPT_MEANING,
            "observer_sha256": probe.hash_file(HERE / "listener_probe.py"),
            "collection_started_at": 0.0,
            "collection_ended_at": 1.0,
            "requested_message_ids": {
                str(message_id): {"count": 2, "recent_received_at": [0.25, 0.75]}
                for message_id in probe.TELEMETRY_MESSAGE_IDS
            },
            "malformed_or_observer_error_count": 0,
            "recent_errors": [],
            "registration_error": None,
            "detach_error": None,
        },
        "acks": [
            {"request": "first", "source_system": 1, "source_component": 191, "target_system": 200, "target_component": 190, "command": 31010, "result": 5, "progress": 0, "wire_magic": 253},
            {"request": "first", "source_system": 1, "source_component": 191, "target_system": 200, "target_component": 190, "command": 31010, "result": 0, "progress": 0, "wire_magic": 253},
            {"request": "duplicate", "source_system": 1, "source_component": 191, "target_system": 200, "target_component": 190, "command": 31010, "result": 0, "progress": 0, "wire_magic": 253},
            {"request": "command_int", "source_system": 1, "source_component": 191, "target_system": 200, "target_component": 190, "command": 31010, "result": 3, "progress": 0, "wire_magic": 253},
        ],
        "packets": [
            {"request": "first", "message_type": "COMMAND_LONG", "source_system": 200, "source_component": 190, "target_system": 1, "target_component": 191, "command": 31010, "params": [7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "wire_magic": 253},
            {"request": "duplicate", "message_type": "COMMAND_LONG", "source_system": 200, "source_component": 190, "target_system": 1, "target_component": 191, "command": 31010, "params": [7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "wire_magic": 253},
            {"request": "wrong_source", "message_type": "COMMAND_LONG", "source_system": 201, "source_component": 190, "target_system": 1, "target_component": 191, "command": 31010, "params": [7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "wire_magic": 253},
            {"request": "command_int", "message_type": "COMMAND_INT", "source_system": 200, "source_component": 190, "target_system": 1, "target_component": 191, "command": 31010, "params": [7.0, 0.0, 0.0, 0.0], "wire_magic": 253},
        ],
        "wrong_source_admitted": False,
        "handler_count": 1,
        "owner": {"process_next_calls": 1, "thread_joined": True},
        "clock": {
            "accepted_frames": 4,
            "retransmissions": 1,
            "advance_count": 4,
            "elapsed_seconds": 0.01,
        },
        "provenance": {
            "ardupilot_commit": probe.ARDUPILOT_COMMIT,
            "base_image_digest": probe.ARDUPILOT_BASE_IMAGE,
            "arducopter_image_id": "sha256:" + "a" * 64,
            "probe_image_id": "sha256:" + "b" * 64,
            "dependency_image_id": "sha256:" + "c" * 64,
            "source_hashes": {
                "comp2026/timebase.py": "d" * 64,
                "qgc_sitl/start_arducopter.py": "f" * 64,
                "qgc_sitl/listener_probe.py": probe.hash_file(
                    HERE / "listener_probe.py"
                ),
            },
        },
        "inputs": {
            "argv": list(probe.arducopter_argv("172.28.0.3")),
            "launcher_sha256": "f" * 64,
            "parameters": {
                "SYSID_THISMAV": "1",
                "SERIAL0_PROTOCOL": "2",
                "SERIAL1_PROTOCOL": "2",
            },
            "parameters_sha256": "e" * 64,
        },
        "cleanup": {
            "connections_closed": True,
            "json_peer_closed": True,
            "owner_thread_joined": True,
        },
        "limitations": list(probe.EVIDENCE_LIMITATIONS),
    }


def test_build_recipe_and_manifest_pin_official_copter_457():
    manifest = json.loads((HERE / "ardupilot-4.5.7.json").read_text())
    dockerfile = (HERE / "Dockerfile.arducopter-4.5.7").read_text()

    assert manifest == {
        "origin": "https://github.com/ArduPilot/ardupilot.git",
        "tag": "Copter-4.5.7",
        "peeled_commit": probe.ARDUPILOT_COMMIT,
        "base_image": probe.ARDUPILOT_BASE_IMAGE,
        "license_path": "/opt/ardupilot/COPYING.txt",
        "build_target": "waf copter",
        "built_image_id": None,
    }
    assert dockerfile.count(probe.ARDUPILOT_BASE_IMAGE) == 2
    assert f"ARG ARDUPILOT_COMMIT={probe.ARDUPILOT_COMMIT}" in dockerfile
    assert 'test "$(git rev-parse HEAD)" = "${ARDUPILOT_COMMIT}"' in dockerfile
    assert "git submodule status --recursive" in dockerfile
    assert "COPYING.txt" in dockerfile
    assert 'GSCALAR(sysid_this_mav, "SYSID_THISMAV",' in dockerfile
    assert r'AP_GROUPINFO\("0_PROTOCOL",[[:space:]]+11,' in dockerfile
    assert r'AP_GROUPINFO\("1_PROTOCOL",[[:space:]]+1,' in dockerfile
    assert r'state\[0\]\.protocol,[[:space:]]+SerialProtocol_MAVLink2' in dockerfile
    assert r'state\[1\]\.protocol,[[:space:]]+DEFAULT_SERIAL1_PROTOCOL' in dockerfile
    assert "grep -Fq 'AP_GROUPINFO(\"0_PROTOCOL\", 11" not in dockerfile


def test_compose_topology_is_private_and_has_no_production_join():
    document = yaml.safe_load((HERE / "compose.yaml").read_text())
    services = document["services"]

    assert set(services) == {"arducopter-457", "listener-probe"}
    assert document["networks"] == {"default": {"internal": True}}
    for service in services.values():
        assert "ports" not in service
        assert "network_mode" not in service
        assert "devices" not in service
        assert service.get("privileged") is not True
        assert service["networks"] == ["default"]
        assert service["security_opt"] == ["no-new-privileges:true"]
        assert service["cap_drop"] == ["ALL"]
        assert service["user"] == (
            "${QGC_SITL_UID:?set to the numeric result-owner UID}:"
            "${QGC_SITL_GID:?set to the numeric result-owner GID}"
        )
    assert services["arducopter-457"]["image"] == probe.ARDUCOPTER_IMAGE_TAG
    assert services["listener-probe"]["image"] == probe.PROBE_IMAGE_TAG
    assert probe.PROBE_IMAGE_TAG == (
        "drone-sim-qgc-listener-probe:4.5.7-2a3dc4b7-diag1"
    )
    for mount in services["listener-probe"]["volumes"][1:]:
        assert mount["read_only"] is True
    assert services["arducopter-457"]["entrypoint"] == [
        "python3", "/opt/qgc-sitl/start_arducopter.py"
    ]
    assert "command" not in services["arducopter-457"]
    assert services["listener-probe"]["environment"]["QGC_SITL_ARDUCOPTER_IMAGE_ID"] == "${QGC_SITL_ARDUCOPTER_IMAGE_ID:-}"


def test_arducopter_argv_and_parameters_preserve_inert_lockstep_contract():
    parameters = probe.read_parameter_file(HERE / "listener.parm")

    argv = probe.arducopter_argv("172.28.0.3")
    assert argv == (
        "/opt/ardupilot/bin/arducopter",
        "--model", "JSON",
        "--speedup", "1",
        "--sim-address", "172.28.0.3",
        "--sim-port-in", "9003",
        "--sim-port-out", "9002",
        "--serial0", "tcp:5760",
        "--serial1", "tcp:5762",
        "--defaults", "/opt/qgc-sitl/listener.parm",
        "--home", "37.4003371,-122.0800351,0,0",
        "--wipe",
    )
    assert parameters == {
        "SYSID_THISMAV": "1",
        "SERIAL0_PROTOCOL": "2",
        "SERIAL1_PROTOCOL": "2",
    }
    assert "ARMING_CHECK" not in parameters
    assert ipaddress.ip_address(argv[argv.index("--sim-address") + 1]).is_private
    assert start_arducopter.argv("172.28.0.3") == argv
    with pytest.raises(ValueError, match="owned-network"):
        probe.arducopter_argv("8.8.8.8")


def test_simulation_clock_accumulates_actual_frame_periods_and_ignores_retries():
    clock = probe.AcceptedFrameClock()

    first_frame = SimpleNamespace(
        frame_count=0, frame_rate_hz=1200, retransmission=False
    )
    first = clock.candidate_timestamp(first_frame)
    assert clock.now() == 0.0
    clock.commit(first_frame, first)
    clock.observe(
        SimpleNamespace(frame_count=0, frame_rate_hz=1200, retransmission=True)
    )
    second_frame = SimpleNamespace(
        frame_count=1, frame_rate_hz=1199, retransmission=False
    )
    second = clock.candidate_timestamp(second_frame)
    clock.commit(second_frame, second)
    third_frame = SimpleNamespace(
        frame_count=2, frame_rate_hz=400, retransmission=False
    )
    third = clock.candidate_timestamp(third_frame)
    clock.commit(third_frame, third)

    assert first == pytest.approx(1 / 1200)
    assert second == pytest.approx(1 / 1200 + 1 / 1199)
    assert third == pytest.approx(1 / 1200 + 1 / 1199 + 1 / 400)
    assert clock.now() == third
    assert clock.snapshot() == {
        "accepted_frames": 3,
        "retransmissions": 1,
        "advance_count": 3,
        "elapsed_seconds": third,
    }
    with pytest.raises(ValueError, match="frame rate"):
        clock.candidate_timestamp(
            SimpleNamespace(frame_count=3, frame_rate_hz=0, retransmission=False)
        )


def test_json_exchange_uses_real_adaptive_headers_and_one_clock_on_retries():
    clock = probe.AcceptedFrameClock()
    with JsonPeer("127.0.0.1", 0) as peer, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
        client.settimeout(1.0)
        responses = []
        for frame_count, frame_rate_hz in ((0, 1200), (0, 1200), (1, 1199), (2, 400)):
            packet = struct.pack(
                "<HHI16H", 18458, frame_rate_hz, frame_count, *([1000] * 16)
            )
            client.sendto(packet, peer.address)
            probe.exchange_peer_once(peer, clock, timeout_seconds=0.5)
            responses.append(json.loads(client.recv(4096))["timestamp"])

    assert responses == pytest.approx([
        1 / 1200,
        1 / 1200,
        1 / 1200 + 1 / 1199,
        1 / 1200 + 1 / 1199 + 1 / 400,
    ])
    assert clock.now() == pytest.approx(responses[-1])
    assert clock.snapshot() == {
        "accepted_frames": 3,
        "retransmissions": 1,
        "advance_count": 3,
        "elapsed_seconds": pytest.approx(responses[-1]),
    }


def test_send_timeout_does_not_commit_peer_or_application_clock_twice():
    packet = struct.pack("<HHI16H", 18458, 1200, 0, *([1000] * 16))

    class InMemorySocket:
        def __init__(self):
            self.packets = [packet, packet]
            self.responses = []
            self.send_attempts = 0

        def settimeout(self, _timeout):
            pass

        def recvfrom(self, _size):
            return self.packets.pop(0), ("127.0.0.1", 9003)

        def sendto(self, payload, _sender):
            self.send_attempts += 1
            if self.send_attempts == 1:
                raise TimeoutError("inert send timeout")
            self.responses.append(payload)

    peer = JsonPeer.__new__(JsonPeer)
    peer._socket = InMemorySocket()
    peer._next_frame = 0
    peer._last_sim_timestamp = None
    clock = probe.AcceptedFrameClock()

    with pytest.raises(json_peer.ResponseSendError, match="response send failed"):
        probe.exchange_peer_once(peer, clock, timeout_seconds=0.5)
    assert peer._next_frame == 0
    assert peer._last_sim_timestamp is None
    assert clock.snapshot() == {
        "accepted_frames": 0,
        "retransmissions": 0,
        "advance_count": 0,
        "elapsed_seconds": 0.0,
    }

    frame = probe.exchange_peer_once(peer, clock, timeout_seconds=0.5)
    response = json.loads(peer._socket.responses[0])

    assert not frame.retransmission
    assert response["timestamp"] == pytest.approx(1 / 1200)
    assert clock.snapshot() == {
        "accepted_frames": 1,
        "retransmissions": 0,
        "advance_count": 1,
        "elapsed_seconds": pytest.approx(1 / 1200),
    }


def test_only_one_nonflight_mutation_command_is_admitted():
    assert probe.TEST_COMMAND == probe.UPDATE_L_COMMAND == 31010
    assert probe.ALLOWED_OUTBOUND_COMMANDS == frozenset(
        {probe.MAV_CMD_SET_MESSAGE_INTERVAL, probe.MAV_CMD_REQUEST_MESSAGE}
    )
    assert probe.FORBIDDEN_COMMANDS.isdisjoint(probe.ALLOWED_OUTBOUND_COMMANDS)
    assert probe.TEST_COMMAND not in probe.FORBIDDEN_COMMANDS


class _InertVehicle:
    def __init__(self) -> None:
        self.listeners = []
        self.removed = []

    def add_message_listener(self, name, callback):
        self.listeners.append((name, callback))

    def remove_message_listener(self, name, callback):
        self.removed.append((name, callback))
        self.listeners.remove((name, callback))

    def emit(self, message) -> None:
        for name, callback in tuple(self.listeners):
            callback(self, name, message)


def _message(message_id: int, *, system: int = 1, component: int = 1):
    return SimpleNamespace(
        get_srcSystem=lambda: system,
        get_srcComponent=lambda: component,
        get_msgId=lambda: message_id,
    )


def test_passive_receipt_observer_filters_source_bounds_memory_and_detaches():
    vehicle = _InertVehicle()
    now = [10.0]
    observer = probe.PassiveTelemetryReceiptObserver(
        vehicle=vehicle,
        message_ids=(0, 1),
        source_system=1,
        source_component=1,
        clock=lambda: now[0],
        recent_limit=2,
        error_limit=2,
        observer_sha256="a" * 64,
    )

    observer.start()
    vehicle.emit(_message(0, system=2))
    vehicle.emit(_message(99))
    for timestamp in (10.25, 10.5, 10.75):
        now[0] = timestamp
        vehicle.emit(_message(0))
    observer.stop()
    vehicle.emit(_message(0))

    snapshot = observer.snapshot()
    assert vehicle.listeners == []
    assert vehicle.removed == [("*", observer.observe)]
    assert snapshot == {
        "meaning": probe.PASSIVE_RECEIPT_MEANING,
        "observer_sha256": "a" * 64,
        "collection_started_at": 10.0,
        "collection_ended_at": 10.75,
        "requested_message_ids": {
            "0": {"count": 3, "recent_received_at": [10.5, 10.75]},
            "1": {"count": 0, "recent_received_at": []},
        },
        "malformed_or_observer_error_count": 0,
        "recent_errors": [],
        "registration_error": None,
        "detach_error": None,
    }


def test_passive_receipt_observer_records_bounded_callback_errors_thread_safely():
    vehicle = _InertVehicle()
    observer = probe.PassiveTelemetryReceiptObserver(
        vehicle=vehicle,
        message_ids=(0,),
        source_system=1,
        source_component=1,
        clock=lambda: 4.0,
        recent_limit=2,
        error_limit=2,
        observer_sha256="b" * 64,
    )
    malformed = SimpleNamespace(
        get_srcSystem=lambda: (_ for _ in ()).throw(ValueError("x" * 1000))
    )
    observer.start()
    threads = [threading.Thread(target=vehicle.emit, args=(malformed,)) for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    vehicle.emit(_message(True))
    observer.stop()

    snapshot = observer.snapshot()
    assert snapshot["malformed_or_observer_error_count"] == 5
    assert len(snapshot["recent_errors"]) == 2
    assert all(len(error) <= 256 for error in snapshot["recent_errors"])
    assert snapshot["recent_errors"][-1] == "ValueError: malformed MAVLink message ID"


def test_failed_collection_retains_passive_receipts_and_original_exception(tmp_path):
    vehicle = _InertVehicle()
    now = [20.0]
    result = {"firmware": {"validation_observed": False}}

    class FailingCollector:
        def configure_and_collect(self):
            now[0] = 20.5
            vehicle.emit(_message(0))
            now[0] = 21.0
            raise TimeoutError("original cadence failure")

    with pytest.raises(TimeoutError, match="^original cadence failure$"):
        probe.collect_with_passive_receipts(
            collector=FailingCollector(),
            vehicle=vehicle,
            message_ids=(0, 1),
            source_system=1,
            source_component=1,
            clock=lambda: now[0],
            observer_sha256="c" * 64,
            result=result,
            result_dir=tmp_path,
        )

    assert result["firmware"]["validation_observed"] is False
    assert result["passive_telemetry_receipts"]["requested_message_ids"] == {
        "0": {"count": 1, "recent_received_at": [20.5]},
        "1": {"count": 0, "recent_received_at": []},
    }
    assert result["passive_telemetry_receipts"]["collection_ended_at"] == 21.0
    assert json.loads((tmp_path / "progress.json").read_text()) == result
    assert vehicle.listeners == []


def test_successful_collection_returns_evidence_and_persists_passive_receipts(tmp_path):
    vehicle = _InertVehicle()
    now = [30.0]
    result = {"firmware": {"validation_observed": False}}
    evidence = object()

    class SuccessfulCollector:
        def configure_and_collect(self):
            now[0] = 30.5
            vehicle.emit(_message(1))
            return evidence

    returned = probe.collect_with_passive_receipts(
        collector=SuccessfulCollector(),
        vehicle=vehicle,
        message_ids=(0, 1),
        source_system=1,
        source_component=1,
        clock=lambda: now[0],
        observer_sha256="d" * 64,
        result=result,
        result_dir=tmp_path,
    )

    assert returned is evidence
    assert result["firmware"]["validation_observed"] is False
    assert result["passive_telemetry_receipts"]["requested_message_ids"] == {
        "0": {"count": 0, "recent_received_at": []},
        "1": {"count": 1, "recent_received_at": [30.5]},
    }
    assert json.loads((tmp_path / "progress.json").read_text()) == result
    assert vehicle.listeners == []


def test_result_validator_rejects_pass_shaped_fallbacks():
    result = _passing_result()
    assert probe.validate_result(result) is result

    for mutation in (
        lambda value: value["firmware"].update(validation_observed=False),
        lambda value: value.update(handler_count=2),
        lambda value: value["owner"].update(process_next_calls=0),
        lambda value: value["cleanup"].update(connections_closed=False),
        lambda value: value["provenance"].update(arducopter_image_id="mutable:tag"),
        lambda value: value.update(wrong_source_admitted=True),
        lambda value: value["acks"][1].update(target_component=1),
    ):
        candidate = json.loads(json.dumps(result))
        mutation(candidate)
        with pytest.raises(ValueError):
            probe.validate_result(candidate)


def test_failure_result_names_stage_reason_and_never_validates_as_pass():
    result = probe.failure_result("telemetry-validation", "AUTOPILOT_VERSION missing")

    assert result["status"] == "failed"
    assert result["stage"] == "telemetry-validation"
    assert result["reason"] == "AUTOPILOT_VERSION missing"
    with pytest.raises(ValueError):
        probe.validate_result(result)


def test_failure_evidence_retains_known_inputs_and_discovery(tmp_path, monkeypatch):
    for name, value in {
        "QGC_SITL_ARDUCOPTER_IMAGE_ID": "sha256:" + "a" * 64,
        "QGC_SITL_PROBE_IMAGE_ID": "sha256:" + "b" * 64,
        "QGC_SITL_DEPENDENCY_IMAGE_ID": "sha256:" + "c" * 64,
    }.items():
        monkeypatch.setenv(name, value)
    result = probe.initial_result(tmp_path)
    result["firmware"].update(
        flight_sw_version=0x040507FF,
        flight_custom_version_hex="3261336463346237",
        discovery_observed=True,
    )
    probe.fail_result(result, "telemetry-validation", "cadence missing")

    assert result["status"] == "failed"
    assert result["firmware"]["discovery_observed"] is True
    assert result["inputs"]["parameters"] == probe.read_parameter_file(HERE / "listener.parm")
    assert result["acks"] == []


def test_rejected_firmware_is_checkpointed_before_identity_validation(tmp_path):
    result = probe.initial_result(tmp_path)
    version = SimpleNamespace(
        flight_sw_version=0x040506FF,
        flight_custom_version=b"badrev00",
    )

    with pytest.raises(RuntimeError, match="official Copter 4.5.7"):
        probe.record_firmware_discovery(tmp_path, result, version)

    retained = json.loads((tmp_path / "progress.json").read_text())
    assert retained["firmware"] == {
        "flight_sw_version": 0x040506FF,
        "flight_custom_version_hex": "6261647265763030",
        "discovery_observed": True,
        "validation_observed": False,
    }


def test_launcher_evidence_hashes_the_actual_executed_mount():
    evidence = start_arducopter.startup_evidence("172.28.0.3")

    assert evidence["status"] == "exec"
    assert evidence["argv"] == start_arducopter.argv("172.28.0.3")
    assert evidence["launcher_sha256"] == probe.hash_file(HERE / "start_arducopter.py")


def test_failed_startup_retains_actual_launcher_hash_before_validation(tmp_path):
    result = probe.initial_result(tmp_path)
    launcher_sha256 = probe.hash_file(HERE / "start_arducopter.py")

    with pytest.raises(RuntimeError, match="ArduCopter startup failed"):
        probe.record_startup_evidence(
            tmp_path,
            result,
            {
                "status": "failed",
                "stage": "peer-resolution",
                "reason": "TimeoutError: DNS unavailable",
                "launcher_sha256": launcher_sha256,
            },
        )

    retained = json.loads((tmp_path / "progress.json").read_text())
    assert retained["inputs"]["launcher_sha256"] == launcher_sha256
    assert retained["provenance"]["source_hashes"][
        "qgc_sitl/start_arducopter.py"
    ] == launcher_sha256


def test_import_failure_is_structured_and_keeps_preimport_provenance(tmp_path, monkeypatch):
    for name, value in {
        "QGC_SITL_ARDUCOPTER_IMAGE_ID": "sha256:" + "a" * 64,
        "QGC_SITL_PROBE_IMAGE_ID": "sha256:" + "b" * 64,
        "QGC_SITL_DEPENDENCY_IMAGE_ID": "sha256:" + "c" * 64,
    }.items():
        monkeypatch.setenv(name, value)
    monkeypatch.setattr(probe, "_hash_sources", lambda _roots: {"source.py": "d" * 64})
    real_import = builtins.__import__

    def fail_drone_import(name, *args, **kwargs):
        if name == "drone":
            raise ImportError("missing test dependency")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fail_drone_import)

    result = probe._run_probe(tmp_path)

    assert result["status"] == "failed"
    assert result["stage"] == "imports"
    assert "missing test dependency" in result["reason"]
    assert result["provenance"]["source_hashes"] == {
        "source.py": "d" * 64,
        "qgc_sitl/listener_probe.py": probe.hash_file(HERE / "listener_probe.py"),
    }
    assert result["cleanup"]["owner_thread_joined"] is False
    assert json.loads((tmp_path / "progress.json").read_text()) == result


def test_import_failure_retains_preexisting_launcher_startup_record(tmp_path, monkeypatch):
    for name, value in {
        "QGC_SITL_ARDUCOPTER_IMAGE_ID": "sha256:" + "a" * 64,
        "QGC_SITL_PROBE_IMAGE_ID": "sha256:" + "b" * 64,
        "QGC_SITL_DEPENDENCY_IMAGE_ID": "sha256:" + "c" * 64,
    }.items():
        monkeypatch.setenv(name, value)
    launcher_sha256 = probe.hash_file(HERE / "start_arducopter.py")
    startup_dir = tmp_path / "arducopter"
    startup_dir.mkdir()
    (startup_dir / "startup.json").write_text(json.dumps({
        "status": "exec",
        "argv": list(probe.arducopter_argv("172.28.0.3")),
        "launcher_sha256": launcher_sha256,
    }))
    monkeypatch.setattr(probe, "_hash_sources", lambda _roots: {"source.py": "d" * 64})
    real_import = builtins.__import__

    def fail_drone_import(name, *args, **kwargs):
        if name == "drone":
            raise ImportError("original import failure")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fail_drone_import)

    result = probe._run_probe(tmp_path)

    assert result["status"] == "failed"
    assert result["stage"] == "imports"
    assert "original import failure" in result["reason"]
    assert result["inputs"]["launcher_sha256"] == launcher_sha256
    assert result["provenance"]["source_hashes"] == {
        "source.py": "d" * 64,
        "qgc_sitl/listener_probe.py": probe.hash_file(HERE / "listener_probe.py"),
        "qgc_sitl/start_arducopter.py": launcher_sha256,
    }
    assert json.loads((tmp_path / "progress.json").read_text()) == result


def test_setup_failure_still_returns_structured_result(tmp_path, monkeypatch):
    monkeypatch.setattr(
        probe,
        "initial_result",
        lambda _directory: (_ for _ in ()).throw(OSError("parameter unavailable")),
    )

    result = probe._run_probe(tmp_path)

    assert result["status"] == "failed"
    assert result["stage"] == "setup"
    assert "parameter unavailable" in result["reason"]


def test_cleanup_or_validation_error_persists_failed_semantics():
    result = _passing_result()
    result["cleanup"]["connections_closed"] = False

    probe.finalize_result(result)

    assert result["status"] == "failed"
    assert result["stage"] == "cleanup"
    assert "connections_closed" in result["reason"]


def test_ack_row_uses_actual_decoded_message_buffer():
    mavlink = pytest.importorskip("pymavlink.dialects.v20.ardupilotmega")
    assert importlib.metadata.version("pymavlink") == "2.4.49"
    encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=191)
    packet = mavlink.MAVLink_command_ack_message(31010, 0, 0, 0, 200, 190).pack(encoder)
    decoder = mavlink.MAVLink(None)
    decoded = decoder.parse_char(packet)

    row = probe._ack_row(decoded, "first")

    assert row["wire_magic"] == 0xFD
    assert decoded.get_msgbuf()[0] == 0xFD
    assert not hasattr(decoded._header, "magic")


def test_result_writer_never_replaces_existing_evidence(tmp_path):
    path = tmp_path / "result.json"
    probe._write_json_exclusive(path, {"status": "first"})

    with pytest.raises(FileExistsError):
        probe._write_json_exclusive(path, {"status": "replacement"})

    assert json.loads(path.read_text()) == {"status": "first"}


def test_probe_main_refuses_to_replace_existing_result(tmp_path, monkeypatch):
    result_path = tmp_path / "result.json"
    result_path.write_text('{"status":"historical"}\n')
    monkeypatch.setenv("QGC_SITL_RESULT_DIR", str(tmp_path))
    monkeypatch.setattr(
        probe,
        "_run_probe",
        lambda _directory: probe.failure_result("test", "expected failure"),
    )

    with pytest.raises(FileExistsError):
        probe.main()

    assert json.loads(result_path.read_text()) == {"status": "historical"}


def test_cleanup_commands_name_only_owned_resources():
    commands = probe.cleanup_commands("qgc-sitl-20260906-a1")

    assert commands == (
        ("docker", "compose", "--project-name", "qgc-sitl-20260906-a1", "-f", str(HERE / "compose.yaml"), "rm", "--force", "--stop", "--volumes", "arducopter-457", "listener-probe"),
        ("docker", "network", "rm", "qgc-sitl-20260906-a1_default"),
    )
    assert all("prune" not in command and "down" not in command for command in commands)


def test_host_preflight_requires_numeric_owner_and_private_owned_mounts(
    tmp_path, monkeypatch
):
    result_dir = tmp_path / "new-result"
    result_dir.mkdir(mode=0o700)
    (result_dir / "arducopter").mkdir(mode=0o700)
    monkeypatch.setenv("QGC_SITL_UID", str(os.getuid()))
    monkeypatch.setenv("QGC_SITL_GID", str(os.getgid()))

    host.validate_result_owner(result_dir)

    monkeypatch.setenv("QGC_SITL_UID", "root")
    with pytest.raises(ValueError, match="numeric"):
        host.validate_result_owner(result_dir)

    monkeypatch.setenv("QGC_SITL_UID", str(os.getuid() + 1))
    with pytest.raises(ValueError, match="owner"):
        host.validate_result_owner(result_dir)

    monkeypatch.setenv("QGC_SITL_UID", str(os.getuid()))
    result_dir.chmod(0o755)
    with pytest.raises(ValueError, match="private"):
        host.validate_result_owner(result_dir)

    calls = []
    result_dir.chmod(0o700)
    monkeypatch.setenv("QGC_SITL_UID", "not-numeric")
    monkeypatch.setattr(
        host, "run_isolated_project", lambda **_kwargs: calls.append("docker")
    )
    monkeypatch.setattr(
        "sys.argv", ["host_runner.py", "qgc-sitl-20260906-owner", str(result_dir)]
    )
    with pytest.raises(ValueError, match="numeric"):
        host.main()
    assert calls == []


def test_host_runner_attempts_all_cleanup_after_capture_failure(tmp_path):
    calls = []

    def run(command, **_kwargs):
        calls.append(tuple(command))
        if "logs" in command:
            raise RuntimeError("capture failed")
        return SimpleNamespace(returncode=0, stdout="[]", stderr="")

    result = host.run_isolated_project(
        project="qgc-sitl-20260906-a1",
        result_dir=tmp_path,
        runner=run,
        operation_timeout_s=3,
        run_timeout_s=5,
    )

    assert any("rm" in command for command in calls)
    assert any(command[:3] == ("docker", "network", "rm") for command in calls)
    assert result["logs"]["status"] == "error"
    assert result["service_cleanup"]["returncode"] == 0
    assert result["network_cleanup"]["returncode"] == 0
    assert json.loads((tmp_path / "host-status.json").read_text()) == result


def test_host_runner_routes_sigterm_through_bounded_cleanup(tmp_path):
    calls = []

    def run(command, **_kwargs):
        calls.append(tuple(command))
        if "up" in command:
            signal.raise_signal(signal.SIGTERM)
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    result = host.run_isolated_project(
        project="qgc-sitl-20260906-term",
        result_dir=tmp_path,
        runner=run,
        operation_timeout_s=3,
        run_timeout_s=5,
    )

    assert result["run"]["status"] == "error"
    assert any("rm" in command for command in calls)
    assert any(command[:3] == ("docker", "network", "rm") for command in calls)
    assert json.loads((tmp_path / "host-status.json").read_text()) == result
    assert not host.host_result_succeeded(result)


def test_host_runner_persists_capture_error_and_fails_result(tmp_path, monkeypatch):
    real_write_text = Path.write_text

    def fail_network_capture(path, data, *args, **kwargs):
        if path.name == "network-cleanup.stdout":
            raise OSError("evidence disk unavailable")
        return real_write_text(path, data, *args, **kwargs)

    monkeypatch.setattr(Path, "write_text", fail_network_capture)

    result = host.run_isolated_project(
        project="qgc-sitl-20260906-capture",
        result_dir=tmp_path,
        runner=lambda _command, **_kwargs: SimpleNamespace(
            returncode=0, stdout="", stderr=""
        ),
    )

    assert "evidence disk unavailable" in result["network_cleanup"]["capture_error"]
    assert json.loads((tmp_path / "host-status.json").read_text()) == result
    assert not host.host_result_succeeded(result)
    tmp_path.chmod(0o700)
    (tmp_path / "arducopter").mkdir(mode=0o700)
    monkeypatch.setenv("QGC_SITL_UID", str(os.getuid()))
    monkeypatch.setenv("QGC_SITL_GID", str(os.getgid()))
    monkeypatch.setattr(host, "run_isolated_project", lambda **_kwargs: result)
    monkeypatch.setattr(
        "sys.argv", ["host_runner.py", "qgc-sitl-20260906-capture", str(tmp_path)]
    )
    assert host.main() == 1


def test_owner_is_not_started_until_progress_has_been_retained():
    calls = []

    progress, terminal = probe.run_first_command(
        send=lambda: calls.append("send") or {"request": "first"},
        receive_progress=lambda: calls.append("progress") or "progress-ack",
        start_owner=lambda: calls.append("owner-start"),
        receive_terminal=lambda: calls.append("terminal") or "terminal-ack",
    )

    assert (progress, terminal) == ("progress-ack", "terminal-ack")
    assert calls == ["send", "progress", "owner-start", "terminal"]


def test_integration_is_opt_in_and_never_runs_from_ordinary_pytest(monkeypatch):
    monkeypatch.delenv("QGC_SITL_RUN_INTEGRATION", raising=False)
    assert probe.integration_requested() is False


@pytest.mark.qgc_sitl_integration
@pytest.mark.skipif(
    not probe.integration_requested(),
    reason="set QGC_SITL_RUN_INTEGRATION=1 after building the isolated images",
)
def test_isolated_sitl_result():
    result_path = Path(probe.required_environment("QGC_SITL_RESULT_PATH"))
    result = json.loads(result_path.read_text())

    probe.validate_result(result)
