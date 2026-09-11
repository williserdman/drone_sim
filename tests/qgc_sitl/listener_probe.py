#!/usr/bin/env python3
"""Bounded, test-only QGC packet probe for an isolated Copter 4.5.7 SITL."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import queue
import signal
import threading
import time
from types import MappingProxyType
from typing import Any, Mapping


ARDUPILOT_COMMIT = "2a3dc4b7bf2507120f7378a7b2fde73185e0c325"
ARDUPILOT_BASE_IMAGE = (
    "ardupilot/ardupilot-dev-base@"
    "sha256:576cd622957308469d6e72528befe5de5862457b264e61d60aaa4b8f29de85b6"
)
ARDUCOPTER_IMAGE_TAG = "drone-sim-qgc-listener-arducopter:4.5.7-2a3dc4b7"
PROBE_IMAGE_TAG = "drone-sim-qgc-listener-probe:4.5.7-2a3dc4b7-diag1"
TELEMETRY_MESSAGE_IDS = (0, 1, 30, 33, 65, 132, 242, 245)
PASSIVE_RECEIPT_MEANING = (
    "Passive source-filtered MAVLink receipts only; counts and timestamps do not "
    "prove collector acceptance or valid payload fields."
)
UPDATE_L_COMMAND = 31010
TEST_COMMAND = UPDATE_L_COMMAND
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_REQUEST_MESSAGE = 512
ALLOWED_OUTBOUND_COMMANDS = frozenset(
    {MAV_CMD_SET_MESSAGE_INTERVAL, MAV_CMD_REQUEST_MESSAGE}
)
FORBIDDEN_COMMANDS = frozenset(
    {
        16,  # MAV_CMD_NAV_WAYPOINT
        20,  # MAV_CMD_NAV_RETURN_TO_LAUNCH
        21,  # MAV_CMD_NAV_LAND
        22,  # MAV_CMD_NAV_TAKEOFF
        176,  # MAV_CMD_DO_SET_MODE
        400,  # MAV_CMD_COMPONENT_ARM_DISARM
        31000,  # FM1
        31001,  # FM2
        31002,  # FM3
    }
)
EVIDENCE_LIMITATIONS = (
    "The pymavlink injector does not verify the QGroundControl application.",
    "The inert JsonPeer is not flight physics or physical-flight evidence.",
    "No FM1, FM2, FM3, arm, mode, navigation, landing, RTL, or payload command ran.",
    "This lane does not verify Gazebo, scoring, recording, or competition readiness.",
)


class AcceptedFrameClock:
    """Advance from accepted JSON frames and ignore retransmissions."""

    def __init__(self, *, wall_stall_limit_s: float = 5.0) -> None:
        self._condition = threading.Condition()
        self._seconds = 0.0
        self._accepted_frames = 0
        self._retransmissions = 0
        self._advance_count = 0
        self._wall_stall_limit_s = wall_stall_limit_s

    def candidate_timestamp(self, frame: object) -> float:
        frame_count = getattr(frame, "frame_count", -1)
        frame_rate_hz = getattr(frame, "frame_rate_hz", 0)
        retransmission = getattr(frame, "retransmission", True)
        if frame_count < 0 or frame_rate_hz <= 0:
            raise ValueError("JSON frame rate and count must be positive")
        if retransmission:
            raise ValueError("a retransmission cannot select a new timestamp")
        with self._condition:
            candidate = self._seconds + (1.0 / frame_rate_hz)
            if candidate <= self._seconds:
                raise ValueError("accepted JSON time did not advance")
            return candidate

    def commit(self, frame: object, timestamp: float) -> None:
        frame_rate_hz = getattr(frame, "frame_rate_hz", 0)
        if getattr(frame, "retransmission", True) or frame_rate_hz <= 0:
            raise ValueError("only a new positive-rate frame can advance time")
        with self._condition:
            candidate = self._seconds + (1.0 / frame_rate_hz)
            if timestamp != candidate:
                raise ValueError("committed JSON timestamp does not match frame period")
            self._seconds = candidate
            self._accepted_frames += 1
            self._advance_count += 1
            self._condition.notify_all()
            return candidate

    def observe(self, frame: object) -> None:
        if not getattr(frame, "retransmission", False):
            return
        with self._condition:
            self._retransmissions += 1
            self._condition.notify_all()

    def now(self) -> float:
        with self._condition:
            return self._seconds

    def sleep(self, seconds: float) -> None:
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("clock sleep must be finite and nonnegative")
        with self._condition:
            target = self._seconds + seconds
            wall_deadline = time.monotonic() + self._wall_stall_limit_s
            while self._seconds < target:
                remaining = wall_deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("accepted-frame clock stalled")
                self._condition.wait(timeout=remaining)

    def snapshot(self) -> dict[str, int | float]:
        with self._condition:
            return {
                "accepted_frames": self._accepted_frames,
                "retransmissions": self._retransmissions,
                "advance_count": self._advance_count,
                "elapsed_seconds": self._seconds,
            }


class PassiveTelemetryReceiptObserver:
    """Retain bounded receipt facts without participating in validation."""

    _MAX_COUNT = (1 << 63) - 1

    def __init__(
        self,
        *,
        vehicle: object,
        message_ids: tuple[int, ...],
        source_system: int,
        source_component: int,
        clock: Any,
        observer_sha256: str,
        recent_limit: int = 4,
        error_limit: int = 4,
    ) -> None:
        if (
            not message_ids
            or len(set(message_ids)) != len(message_ids)
            or any(not isinstance(value, int) or isinstance(value, bool) for value in message_ids)
            or not callable(clock)
            or not callable(getattr(vehicle, "add_message_listener", None))
            or recent_limit <= 0
            or error_limit <= 0
            or len(observer_sha256) != 64
            or any(character not in "0123456789abcdef" for character in observer_sha256)
        ):
            raise ValueError("passive telemetry observer configuration is invalid")
        self._vehicle = vehicle
        self._message_ids = message_ids
        self._message_id_set = frozenset(message_ids)
        self._source_system = source_system
        self._source_component = source_component
        self._clock = clock
        self._observer_sha256 = observer_sha256
        self._lock = threading.Lock()
        self._counts = {message_id: 0 for message_id in message_ids}
        self._recent = {
            message_id: deque(maxlen=recent_limit) for message_id in message_ids
        }
        self._errors: deque[str] = deque(maxlen=error_limit)
        self._error_count = 0
        self._started_at: float | None = None
        self._ended_at: float | None = None
        self._registration_error: str | None = None
        self._detach_error: str | None = None
        self._active = False
        self._attached = False

    def start(self) -> None:
        with self._lock:
            if self._active or self._attached or self._started_at is not None:
                raise RuntimeError("passive telemetry observer is one-shot")
            self._active = True
        try:
            started_at = self._now()
        except BaseException as error:
            self._record_error(error)
        else:
            with self._lock:
                self._started_at = started_at
        try:
            self._vehicle.add_message_listener("*", self.observe)  # type: ignore[attr-defined]
        except BaseException as error:
            with self._lock:
                self._registration_error = self._error_text(error)
                self._active = False
        else:
            with self._lock:
                self._attached = True

    def observe(self, _vehicle: object, _name: str, message: object) -> None:
        try:
            with self._lock:
                if not self._active:
                    return
            source_system = message.get_srcSystem()  # type: ignore[attr-defined]
            source_component = message.get_srcComponent()  # type: ignore[attr-defined]
            if (
                not isinstance(source_system, int)
                or isinstance(source_system, bool)
                or not isinstance(source_component, int)
                or isinstance(source_component, bool)
            ):
                raise ValueError("malformed MAVLink source identity")
            if (
                source_system != self._source_system
                or source_component != self._source_component
            ):
                return
            message_id = message.get_msgId()  # type: ignore[attr-defined]
            if not isinstance(message_id, int) or isinstance(message_id, bool):
                raise ValueError("malformed MAVLink message ID")
            if message_id not in self._message_id_set:
                return
            observed_at = self._now()
            with self._lock:
                if not self._active:
                    return
                self._counts[message_id] = min(
                    self._MAX_COUNT, self._counts[message_id] + 1
                )
                self._recent[message_id].append(observed_at)
        except BaseException as error:
            self._record_error(error)

    def stop(self) -> None:
        with self._lock:
            self._active = False
            attached = self._attached
            self._attached = False
        if attached:
            remove = getattr(self._vehicle, "remove_message_listener", None)
            if not callable(remove):
                with self._lock:
                    self._detach_error = "RuntimeError: vehicle cannot detach message listener"
            else:
                try:
                    remove("*", self.observe)
                except BaseException as error:
                    with self._lock:
                        self._detach_error = self._error_text(error)
        try:
            ended_at = self._now()
        except BaseException as error:
            self._record_error(error)
        else:
            with self._lock:
                self._ended_at = ended_at

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return {
                "meaning": PASSIVE_RECEIPT_MEANING,
                "observer_sha256": self._observer_sha256,
                "collection_started_at": self._started_at,
                "collection_ended_at": self._ended_at,
                "requested_message_ids": {
                    str(message_id): {
                        "count": self._counts[message_id],
                        "recent_received_at": list(self._recent[message_id]),
                    }
                    for message_id in self._message_ids
                },
                "malformed_or_observer_error_count": self._error_count,
                "recent_errors": list(self._errors),
                "registration_error": self._registration_error,
                "detach_error": self._detach_error,
            }

    def _now(self) -> float:
        value = self._clock()
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(value)
        ):
            raise ValueError("passive telemetry observer clock returned a nonfinite value")
        return float(value)

    def _record_error(self, error: BaseException) -> None:
        with self._lock:
            self._error_count = min(self._MAX_COUNT, self._error_count + 1)
            self._errors.append(self._error_text(error))

    @staticmethod
    def _error_text(error: BaseException) -> str:
        return f"{type(error).__name__}: {error}"[:256]


def arducopter_argv(peer_ipv4: str) -> tuple[str, ...]:
    parsed = ipaddress.IPv4Address(peer_ipv4)
    if not parsed.is_private or parsed.is_unspecified or parsed.is_loopback:
        raise ValueError("JSON peer must use an owned-network IPv4 address")
    return (
        "/opt/ardupilot/bin/arducopter",
        "--model", "JSON",
        "--speedup", "1",
        "--sim-address", str(parsed),
        "--sim-port-in", "9003",
        "--sim-port-out", "9002",
        "--serial0", "tcp:5760",
        "--serial1", "tcp:5762",
        "--defaults", "/opt/qgc-sitl/listener.parm",
        "--home", "37.4003371,-122.0800351,0,0",
        "--wipe",
    )


def exchange_peer_once(peer: object, clock: AcceptedFrameClock, *, timeout_seconds: float):
    selected_timestamp: float | None = None

    def select_timestamp(frame: object) -> float:
        nonlocal selected_timestamp
        selected_timestamp = clock.candidate_timestamp(frame)
        return selected_timestamp

    frame = peer.exchange_once(  # type: ignore[attr-defined]
        timeout_seconds=timeout_seconds,
        sim_timestamp=select_timestamp,
    )
    if frame.retransmission:
        clock.observe(frame)
    elif selected_timestamp is None:
        raise RuntimeError("JSON peer did not select a timestamp for a new frame")
    else:
        clock.commit(frame, selected_timestamp)
    return frame


def read_parameter_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.partition("#")[0].strip()
        if not line:
            continue
        fields = line.replace(",", " ").split()
        if len(fields) != 2 or fields[0] in values:
            raise ValueError(f"invalid parameter row: {raw_line}")
        values[fields[0]] = fields[1]
    return values


def required_environment(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise RuntimeError(f"required environment variable is missing: {name}")
    return value


def integration_requested() -> bool:
    return os.environ.get("QGC_SITL_RUN_INTEGRATION") == "1"


def cleanup_commands(project_name: str) -> tuple[tuple[str, ...], ...]:
    if not project_name.startswith("qgc-sitl-") or any(
        character not in "abcdefghijklmnopqrstuvwxyz0123456789-"
        for character in project_name
    ):
        raise ValueError("cleanup requires a unique qgc-sitl project name")
    compose_path = str(Path(__file__).resolve().with_name("compose.yaml"))
    return (
        (
            "docker",
            "compose",
            "--project-name",
            project_name,
            "-f",
            compose_path,
            "rm",
            "--force",
            "--stop",
            "--volumes",
            "arducopter-457",
            "listener-probe",
        ),
        ("docker", "network", "rm", f"{project_name}_default"),
    )


def failure_result(stage: str, reason: str) -> dict[str, object]:
    if not stage or not reason:
        raise ValueError("failure stage and reason must be explicit")
    return {
        "schema_version": 1,
        "status": "failed",
        "stage": stage,
        "reason": reason,
        "limitations": list(EVIDENCE_LIMITATIONS),
    }


def initial_result(_result_dir: Path) -> dict[str, object]:
    parameter_path = Path(__file__).resolve().with_name("listener.parm")
    observer_sha256 = hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()
    return {
        "schema_version": 1,
        "status": "running",
        "stage": "setup",
        "reason": None,
        "identities": {
            "injector": [200, 190],
            "companion": [1, 191],
            "flight_controller": [1, 1],
            "wire_protocol": "2.0",
        },
        "firmware": {
            "flight_sw_version": None,
            "flight_custom_version_hex": None,
            "discovery_observed": False,
            "validation_observed": False,
        },
        "passive_telemetry_receipts": {
            "meaning": PASSIVE_RECEIPT_MEANING,
            "observer_sha256": observer_sha256,
            "collection_started_at": None,
            "collection_ended_at": None,
            "requested_message_ids": {
                str(message_id): {"count": 0, "recent_received_at": []}
                for message_id in TELEMETRY_MESSAGE_IDS
            },
            "malformed_or_observer_error_count": 0,
            "recent_errors": [],
            "registration_error": None,
            "detach_error": None,
        },
        "acks": [],
        "packets": [],
        "wrong_source_admitted": None,
        "handler_count": 0,
        "owner": {"process_next_calls": 0, "thread_joined": False},
        "clock": {
            "accepted_frames": 0,
            "retransmissions": 0,
            "advance_count": 0,
            "elapsed_seconds": 0.0,
        },
        "provenance": {
            "ardupilot_commit": ARDUPILOT_COMMIT,
            "base_image_digest": ARDUPILOT_BASE_IMAGE,
            "arducopter_image_id": os.environ.get("QGC_SITL_ARDUCOPTER_IMAGE_ID", ""),
            "probe_image_id": os.environ.get("QGC_SITL_PROBE_IMAGE_ID", ""),
            "dependency_image_id": os.environ.get("QGC_SITL_DEPENDENCY_IMAGE_ID", ""),
            "source_hashes": {},
        },
        "inputs": {
            "argv": [],
            "launcher_sha256": None,
            "parameters": read_parameter_file(parameter_path),
            "parameters_sha256": hashlib.sha256(parameter_path.read_bytes()).hexdigest(),
        },
        "cleanup": {
            "connections_closed": False,
            "json_peer_closed": False,
            "owner_thread_joined": False,
        },
        "limitations": list(EVIDENCE_LIMITATIONS),
    }


def fail_result(result: dict[str, object], stage: str, reason: str) -> dict[str, object]:
    if not stage or not reason:
        raise ValueError("failure stage and reason must be explicit")
    result.update(status="failed", stage=stage, reason=reason)
    return result


def finalize_result(result: dict[str, object]) -> dict[str, object]:
    if result.get("status") == "passed":
        cleanup = result.get("cleanup")
        if not isinstance(cleanup, dict):
            return fail_result(result, "cleanup", "cleanup evidence is missing")
        incomplete = [
            field
            for field in (
                "connections_closed", "json_peer_closed", "owner_thread_joined"
            )
            if cleanup.get(field) is not True
        ]
        if incomplete:
            return fail_result(
                result, "cleanup", f"cleanup incomplete: {', '.join(incomplete)}"
            )
        try:
            validate_result(result)
        except ValueError as error:
            return fail_result(result, "result-validation", str(error))
    return result


def _checkpoint(result_dir: Path, result: dict[str, object]) -> None:
    path = result_dir / "progress.json"
    temporary = result_dir / ".progress.json.tmp"
    temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def collect_with_passive_receipts(
    *,
    collector: object,
    vehicle: object,
    message_ids: tuple[int, ...],
    source_system: int,
    source_component: int,
    clock: Any,
    observer_sha256: str,
    result: dict[str, object],
    result_dir: Path,
) -> object:
    """Collect normally while persisting separate passive receipt evidence."""
    observer = PassiveTelemetryReceiptObserver(
        vehicle=vehicle,
        message_ids=message_ids,
        source_system=source_system,
        source_component=source_component,
        clock=clock,
        observer_sha256=observer_sha256,
    )
    observer.start()
    telemetry: object | None = None
    collector_error: BaseException | None = None
    try:
        telemetry = collector.configure_and_collect()  # type: ignore[attr-defined]
    except BaseException as error:
        collector_error = error
    observer.stop()
    result["passive_telemetry_receipts"] = observer.snapshot()
    try:
        _checkpoint(result_dir, result)
    except BaseException:
        if collector_error is None:
            raise
    if collector_error is not None:
        raise collector_error
    return telemetry


def _sha256_id(value: object, field: str) -> None:
    if not isinstance(value, str) or not value.startswith("sha256:"):
        raise ValueError(f"{field} must be an immutable sha256 image ID")
    digest = value[7:]
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise ValueError(f"{field} must be an immutable sha256 image ID")


def validate_result(result: dict[str, object]) -> dict[str, object]:
    if result.get("schema_version") != 1 or result.get("status") != "passed":
        raise ValueError("only a complete passing result validates")
    if result.get("stage") != "complete" or result.get("reason") is not None:
        raise ValueError("passing result has invalid terminal semantics")
    if result.get("identities") != {
        "injector": [200, 190],
        "companion": [1, 191],
        "flight_controller": [1, 1],
        "wire_protocol": "2.0",
    }:
        raise ValueError("wire identities do not match the fixed lane")
    firmware = result.get("firmware")
    if not isinstance(firmware, dict) or firmware.get("flight_sw_version") != 0x040507FF:
        raise ValueError("firmware packed version does not match Copter 4.5.7 stable")
    custom = firmware.get("flight_custom_version_hex")
    if (
        not isinstance(custom, str)
        or len(custom) != 16
        or any(character not in "0123456789abcdef" for character in custom)
        or firmware.get("discovery_observed") is not True
        or firmware.get("validation_observed") is not True
    ):
        raise ValueError("firmware discovery and bound validation are incomplete")
    expected_acks = [
        ("first", 5),
        ("first", 0),
        ("duplicate", 0),
        ("command_int", 3),
    ]
    acks = result.get("acks")
    if not isinstance(acks, list) or [
        (row.get("request"), row.get("result"))
        for row in acks
        if isinstance(row, dict)
    ] != expected_acks:
        raise ValueError("ACK sequence is incomplete")
    if any(
        not isinstance(row, dict)
        or row.get("source_system") != 1
        or row.get("source_component") != 191
        or row.get("target_system") != 200
        or row.get("target_component") != 190
        or row.get("command") != TEST_COMMAND
        or row.get("progress") != 0
        or row.get("wire_magic") != 0xFD
        for row in acks
    ):
        raise ValueError("ACK identities or MAVLink 2 encoding are invalid")
    packets = result.get("packets")
    expected_packets = (
        ("first", "COMMAND_LONG", 200),
        ("duplicate", "COMMAND_LONG", 200),
        ("wrong_source", "COMMAND_LONG", 201),
        ("command_int", "COMMAND_INT", 200),
    )
    if not isinstance(packets, list) or tuple(
        (row.get("request"), row.get("message_type"), row.get("source_system"))
        for row in packets
        if isinstance(row, dict)
    ) != expected_packets:
        raise ValueError("injected packet sequence is incomplete")
    if any(
        not isinstance(row, dict)
        or row.get("source_component") != 190
        or row.get("target_system") != 1
        or row.get("target_component") != 191
        or row.get("command") != TEST_COMMAND
        or row.get("wire_magic") != 0xFD
        or row.get("params")
        != ([7.0, 0.0, 0.0, 0.0] if row.get("message_type") == "COMMAND_INT" else [7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        for row in packets
    ):
        raise ValueError("injected packet fields or MAVLink 2 encoding are invalid")
    if result.get("wrong_source_admitted") is not False or result.get("handler_count") != 1:
        raise ValueError("admission or exactly-once execution failed")
    owner = result.get("owner")
    if not isinstance(owner, dict) or owner != {
        "process_next_calls": 1,
        "thread_joined": True,
    }:
        raise ValueError("synchronous owner evidence is incomplete")
    clock = result.get("clock")
    if (
        not isinstance(clock, dict)
        or not isinstance(clock.get("accepted_frames"), int)
        or clock["accepted_frames"] <= 0
        or clock.get("advance_count") != clock.get("accepted_frames")
        or not isinstance(clock.get("elapsed_seconds"), (int, float))
        or not math.isfinite(clock["elapsed_seconds"])
        or clock["elapsed_seconds"] <= 0
    ):
        raise ValueError("accepted-frame clock evidence is invalid")
    provenance = result.get("provenance")
    if not isinstance(provenance, dict):
        raise ValueError("provenance is missing")
    if provenance.get("ardupilot_commit") != ARDUPILOT_COMMIT:
        raise ValueError("ArduPilot commit does not match the fixed lane")
    if provenance.get("base_image_digest") != ARDUPILOT_BASE_IMAGE:
        raise ValueError("base image digest does not match the fixed lane")
    for field in ("arducopter_image_id", "probe_image_id", "dependency_image_id"):
        _sha256_id(provenance.get(field), field)
    hashes = provenance.get("source_hashes")
    if not isinstance(hashes, dict) or not hashes:
        raise ValueError("mounted source byte hashes are missing")
    if any(
        not isinstance(name, str)
        or not isinstance(digest, str)
        or len(digest) != 64
        or any(character not in "0123456789abcdef" for character in digest)
        for name, digest in hashes.items()
    ):
        raise ValueError("mounted source byte hashes are invalid")
    passive = result.get("passive_telemetry_receipts")
    if not isinstance(passive, dict) or passive.get("meaning") != PASSIVE_RECEIPT_MEANING:
        raise ValueError("passive telemetry receipt meaning is missing")
    observer_sha256 = passive.get("observer_sha256")
    if (
        not isinstance(observer_sha256, str)
        or hashes.get("qgc_sitl/listener_probe.py") != observer_sha256
    ):
        raise ValueError("passive telemetry observer byte hash is inconsistent")
    started_at = passive.get("collection_started_at")
    ended_at = passive.get("collection_ended_at")
    if (
        not isinstance(started_at, (int, float))
        or isinstance(started_at, bool)
        or not math.isfinite(started_at)
        or not isinstance(ended_at, (int, float))
        or isinstance(ended_at, bool)
        or not math.isfinite(ended_at)
        or ended_at < started_at
    ):
        raise ValueError("passive telemetry collection window is invalid")
    receipts = passive.get("requested_message_ids")
    if not isinstance(receipts, dict) or set(receipts) != {
        str(message_id) for message_id in TELEMETRY_MESSAGE_IDS
    }:
        raise ValueError("passive telemetry receipt IDs are incomplete")
    for row in receipts.values():
        count = row.get("count") if isinstance(row, dict) else None
        recent = row.get("recent_received_at") if isinstance(row, dict) else None
        if (
            not isinstance(count, int)
            or isinstance(count, bool)
            or count < 0
            or not isinstance(recent, list)
            or len(recent) > 4
            or count < len(recent)
            or any(
                not isinstance(value, (int, float))
                or isinstance(value, bool)
                or not math.isfinite(value)
                or not started_at <= value <= ended_at
                for value in recent
            )
        ):
            raise ValueError("passive telemetry receipt row is invalid")
    error_count = passive.get("malformed_or_observer_error_count")
    recent_errors = passive.get("recent_errors")
    if (
        not isinstance(error_count, int)
        or isinstance(error_count, bool)
        or error_count < 0
        or not isinstance(recent_errors, list)
        or len(recent_errors) > 4
        or error_count < len(recent_errors)
        or any(not isinstance(value, str) or not value for value in recent_errors)
        or any(
            value is not None and (not isinstance(value, str) or not value)
            for value in (passive.get("registration_error"), passive.get("detach_error"))
        )
    ):
        raise ValueError("passive telemetry observer errors are invalid")
    inputs = result.get("inputs")
    argv = inputs.get("argv") if isinstance(inputs, dict) else None
    if not isinstance(argv, list):
        raise ValueError("resolved ArduCopter argv does not match the fixed lane")
    try:
        address_index = argv.index("--sim-address") + 1
        expected_argv = list(arducopter_argv(argv[address_index]))
    except (IndexError, TypeError, ValueError):
        raise ValueError("resolved ArduCopter argv does not match the fixed lane") from None
    if argv != expected_argv:
        raise ValueError("resolved ArduCopter argv does not match the fixed lane")
    launcher_sha256 = inputs.get("launcher_sha256")
    if (
        not isinstance(launcher_sha256, str)
        or len(launcher_sha256) != 64
        or any(character not in "0123456789abcdef" for character in launcher_sha256)
        or hashes.get("qgc_sitl/start_arducopter.py") != launcher_sha256
    ):
        raise ValueError("mounted launcher byte hash is missing or inconsistent")
    if inputs.get("parameters") != {
        "SYSID_THISMAV": "1",
        "SERIAL0_PROTOCOL": "2",
        "SERIAL1_PROTOCOL": "2",
    }:
        raise ValueError("resolved Copter parameters do not match the fixed lane")
    cleanup = result.get("cleanup")
    if not isinstance(cleanup, dict) or any(
        cleanup.get(field) is not True
        for field in ("connections_closed", "json_peer_closed", "owner_thread_joined")
    ):
        raise ValueError("in-process cleanup is incomplete")
    if result.get("limitations") != list(EVIDENCE_LIMITATIONS):
        raise ValueError("evidence limitations are missing")
    return result


def _hash_sources(roots: tuple[tuple[str, Path], ...]) -> dict[str, str]:
    hashes: dict[str, str] = {}
    for label, root in roots:
        if root.is_file():
            files = (root,)
            base = root.parent
        else:
            files = tuple(sorted(path for path in root.rglob("*.py") if path.is_file()))
            base = root
        if not files:
            raise RuntimeError(f"source input is empty: {root}")
        for path in files:
            name = f"{label}/{path.relative_to(base)}"
            hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return hashes


def hash_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def record_startup_evidence(
    result_dir: Path, result: dict[str, object], startup: dict[str, object]
) -> None:
    launcher_sha256 = startup.get("launcher_sha256")
    if (
        not isinstance(launcher_sha256, str)
        or len(launcher_sha256) != 64
        or any(character not in "0123456789abcdef" for character in launcher_sha256)
    ):
        raise RuntimeError("ArduCopter launcher did not record its mounted-byte hash")
    result["inputs"]["launcher_sha256"] = launcher_sha256  # type: ignore[index]
    result["provenance"]["source_hashes"][  # type: ignore[index]
        "qgc_sitl/start_arducopter.py"
    ] = launcher_sha256
    _checkpoint(result_dir, result)
    if startup.get("status") != "exec" or not isinstance(startup.get("argv"), list):
        raise RuntimeError(f"ArduCopter startup failed: {startup}")
    result["inputs"]["argv"] = startup["argv"]  # type: ignore[index]
    _checkpoint(result_dir, result)


def record_firmware_discovery(
    result_dir: Path, result: dict[str, object], version: object
) -> tuple[int, bytes]:
    custom = bytes(getattr(version, "flight_custom_version"))
    packed = int(getattr(version, "flight_sw_version"))
    result["firmware"].update(  # type: ignore[union-attr]
        flight_sw_version=packed,
        flight_custom_version_hex=custom.hex(),
        discovery_observed=True,
    )
    _checkpoint(result_dir, result)
    if packed != 0x040507FF or len(custom) != 8:
        raise RuntimeError("discovered firmware does not identify official Copter 4.5.7")
    return packed, custom


def _profile():
    from drone.control.flight_profile import FlightProfile, TelemetryRequest
    from drone.control.flight_state import RCModeBand, SourceIdentity

    fields = {
        name: 2.0
        for name in (
            "heartbeat", "mode", "location", "velocity", "attitude",
            "landed_state", "armed", "home", "rc_input", "range", "failsafe",
        )
    }
    return FlightProfile(
        profile_id="qgc-sitl-copter-4.5.7",
        raw_sha256="0" * 64,
        firmware="ArduCopter 4.5.7",
        qgc_source=SourceIdentity(200, 190),
        companion_target=SourceIdentity(1, 191),
        flight_controller=SourceIdentity(1, 1),
        freshness_bounds=MappingProxyType(fields),
        rc_health_max_age=2.0,
        rc_channel=7,
        rc_mode_bands=(RCModeBand("STABILIZE", 900, 1199, "STABILIZE"),),
        heartbeat_type=2,
        heartbeat_autopilot=3,
        copter_modes=MappingProxyType(
            {0: "STABILIZE", 4: "GUIDED", 5: "LOITER", 6: "RTL", 9: "LAND"}
        ),
        failsafe_active_statuses=frozenset((5,)),
        failsafe_clear_statuses=frozenset((3, 4)),
        decoder_contract_version=1,
        decoder_contract_evidence="sha256:" + "0" * 64,
        decoder_contract_reference=(
            "ArduPilot/Copter-4.5.7:GCS_Mavlink.cpp:70-86;GCS_Copter.cpp:41-48"
        ),
        telemetry_requests=tuple(
            TelemetryRequest(message_id, 500_000)
            for message_id in TELEMETRY_MESSAGE_IDS
        ),
    )


def _ack_row(message: object, request: str) -> dict[str, object]:
    message_buffer = message.get_msgbuf()  # type: ignore[attr-defined]
    if not isinstance(message_buffer, (bytes, bytearray)) or not message_buffer:
        raise ValueError("decoded ACK has no wire buffer")
    return {
        "request": request,
        "source_system": message.get_srcSystem(),  # type: ignore[attr-defined]
        "source_component": message.get_srcComponent(),  # type: ignore[attr-defined]
        "target_system": getattr(message, "target_system", None),
        "target_component": getattr(message, "target_component", None),
        "command": getattr(message, "command", None),
        "result": getattr(message, "result", None),
        "progress": getattr(message, "progress", None),
        "wire_magic": message_buffer[0],
    }


def _receive_ack(link: object, *, command: int, result: int, timeout_s: float) -> object:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = link.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.25)  # type: ignore[attr-defined]
        if (
            message is not None
            and getattr(message, "command", None) == command
            and getattr(message, "result", None) == result
            and message.get_srcSystem() == 1
            and message.get_srcComponent() == 191
            and getattr(message, "target_system", None) == 200
            and getattr(message, "target_component", None) == 190
        ):
            return message
    raise TimeoutError(f"COMMAND_ACK {command}/{result} was not routed to the injector")


def _send_long(
    link: object, *, request: str, source_system: int = 200
) -> dict[str, object]:
    original = link.mav.srcSystem  # type: ignore[attr-defined]
    link.mav.srcSystem = source_system  # type: ignore[attr-defined]
    try:
        message = link.mav.command_long_encode(  # type: ignore[attr-defined]
            1, 191, TEST_COMMAND, 0, 7.0, 0, 0, 0, 0, 0, 0
        )
        encoded = message.pack(link.mav, force_mavlink1=False)  # type: ignore[attr-defined]
        link.mav.send(message, force_mavlink1=False)  # type: ignore[attr-defined]
        return {
            "request": request,
            "message_type": "COMMAND_LONG",
            "source_system": source_system,
            "source_component": 190,
            "target_system": 1,
            "target_component": 191,
            "command": TEST_COMMAND,
            "params": [7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "wire_magic": encoded[0],
        }
    finally:
        link.mav.srcSystem = original  # type: ignore[attr-defined]


def _send_int(link: object) -> dict[str, object]:
    message = link.mav.command_int_encode(  # type: ignore[attr-defined]
        1, 191, 0, TEST_COMMAND, 0, 0, 7.0, 0, 0, 0, 0, 0, 0
    )
    encoded = message.pack(link.mav, force_mavlink1=False)  # type: ignore[attr-defined]
    link.mav.send(message, force_mavlink1=False)  # type: ignore[attr-defined]
    return {
        "request": "command_int",
        "message_type": "COMMAND_INT",
        "source_system": 200,
        "source_component": 190,
        "target_system": 1,
        "target_component": 191,
        "command": TEST_COMMAND,
        "params": [7.0, 0.0, 0.0, 0.0],
        "wire_magic": encoded[0],
    }


def _write_json_exclusive(path: Path, value: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as output:
        json.dump(value, output, indent=2, sort_keys=True)
        output.write("\n")
        output.flush()
        os.fsync(output.fileno())


def _bounded_call(operation: Any, *, timeout_s: float) -> bool:
    completed = threading.Event()
    errors: list[BaseException] = []

    def invoke() -> None:
        try:
            operation()
        except BaseException as error:
            errors.append(error)
        finally:
            completed.set()

    thread = threading.Thread(target=invoke, name="qgc-sitl-bounded-cleanup", daemon=True)
    thread.start()
    thread.join(timeout=timeout_s)
    return completed.is_set() and not errors


def run_first_command(
    *,
    send: Any,
    receive_progress: Any,
    start_owner: Any,
    receive_terminal: Any,
) -> tuple[object, object]:
    send()
    progress = receive_progress()
    start_owner()
    terminal = receive_terminal()
    return progress, terminal


def _run_probe(result_dir: Path) -> dict[str, object]:
    try:
        result = initial_result(result_dir)
    except BaseException as error:
        result = failure_result("setup", f"{type(error).__name__}: {error}")
        result["cleanup"] = {
            "connections_closed": True,
            "json_peer_closed": True,
            "owner_thread_joined": False,
        }
        _checkpoint(result_dir, result)
        return result
    _checkpoint(result_dir, result)
    clock = AcceptedFrameClock()
    stop_peer = threading.Event()
    peer_error: list[BaseException] = []
    peer: Any = None
    controller: Any = None
    injector: Any = None
    owner_thread: threading.Thread | None = None
    peer_thread: threading.Thread | None = None
    owner_joined = False
    handler_count = 0
    owner_calls = 0
    startup_path = result_dir / "arducopter/startup.json"
    try:
        os.environ["MAVLINK20"] = "1"
        result["stage"] = "provenance"
        source_hashes = _hash_sources(
            (
                ("comp2026/control", Path("/opt/drone_sim/comp2026/src/drone/control")),
                ("comp2026/lidar", Path("/opt/drone_sim/comp2026/src/drone/sensors/lidar")),
                ("comp2026", Path("/opt/drone_sim/comp2026/src/drone/timebase.py")),
                ("comp2026", Path("/opt/drone_sim/comp2026/src/drone/common_types.py")),
                ("ardupilot_sitl", Path("/opt/drone_sim/ardupilot_sitl/src/drone_sim_ardupilot")),
            )
        )
        source_hashes["qgc_sitl/listener_probe.py"] = hash_file(Path(__file__).resolve())
        result["provenance"]["source_hashes"] = source_hashes  # type: ignore[index]
        for field in (
            "QGC_SITL_ARDUCOPTER_IMAGE_ID",
            "QGC_SITL_PROBE_IMAGE_ID",
            "QGC_SITL_DEPENDENCY_IMAGE_ID",
        ):
            required_environment(field)
        _checkpoint(result_dir, result)

        result["stage"] = "imports"
        _checkpoint(result_dir, result)
        from drone import timebase
        from drone.control.drone_control import DroneControl
        from drone.control.flight_state import FlightState, SourceIdentity
        from drone.control.listener import (
            ACK_ACCEPTED,
            ACK_IN_PROGRESS,
            ACK_UNSUPPORTED,
            CommandExecutionOwner,
            DroneKitQGCAckTransport,
            QGCCommandListener,
            TelemetryStartupCollector,
        )
        from drone.control.listener_runtime import (
            AutopilotVersionContract,
            TelemetryStartupPolicy,
        )
        from drone.control.mission_supervisor import MissionSupervisor
        from drone_sim_ardupilot.json_peer import JsonPeer, ResponseSendError
        from pymavlink import mavutil

        result["stage"] = "arducopter-startup"
        _checkpoint(result_dir, result)
        deadline = time.monotonic() + 15
        while not startup_path.is_file() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not startup_path.is_file():
            raise TimeoutError("ArduCopter startup argv was not recorded")
        startup = json.loads(startup_path.read_text())
        record_startup_evidence(result_dir, result, startup)

        profile = _profile()
        flight_state = FlightState(
            source_system=1,
            source_component=1,
            freshness_bounds=profile.freshness_bounds,
            rc_channel=profile.rc_channel,
            rc_mode_mapping=profile.rc_mode_bands,
            clock=timebase.monotonic,
        )
        result["stage"] = "json-peer"
        peer = JsonPeer("0.0.0.0", 9002)

        def exchange() -> None:
            while not stop_peer.is_set():
                try:
                    exchange_peer_once(peer, clock, timeout_seconds=0.5)
                except ResponseSendError as error:
                    peer_error.append(error)
                    return
                except TimeoutError:
                    continue
                except OSError:
                    if not stop_peer.is_set():
                        peer_error.append(RuntimeError("JSON peer socket failed"))
                    return
                except BaseException as error:
                    peer_error.append(error)
                    return

        peer_thread = threading.Thread(
            target=exchange, name="qgc-sitl-json-peer", daemon=True
        )
        peer_thread.start()
        result["stage"] = "connections"
        _checkpoint(result_dir, result)
        connection_deadline = time.monotonic() + 15
        while injector is None and time.monotonic() < connection_deadline:
            try:
                injector = mavutil.mavlink_connection(
                    "tcp:arducopter-457:5762",
                    source_system=200,
                    source_component=190,
                    dialect="ardupilotmega",
                    autoreconnect=False,
                )
            except OSError:
                time.sleep(0.1)
        if injector is None:
            raise TimeoutError("injector could not connect to the private SITL link")
        if injector.wait_heartbeat(timeout=15) is None:
            raise TimeoutError("injector link did not receive a flight-controller heartbeat")
        controller = DroneControl(
            "tcp:arducopter-457:5760",
            source_identity=SourceIdentity(1, 191),
            flight_controller_target=SourceIdentity(1, 1),
            wait_ready=False,
            heartbeat_timeout=15,
            flight_state=flight_state,
        )
        heartbeat = controller.vehicle.message_factory.heartbeat_encode(  # type: ignore[attr-defined]
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
            3,
        )
        controller.vehicle.send_mavlink(heartbeat)
        controller.vehicle.flush()

        result["stage"] = "firmware-discovery"
        _checkpoint(result_dir, result)
        injector.mav.command_long_send(
            1, 1, MAV_CMD_REQUEST_MESSAGE, 0, 148.0, 0, 0, 0, 0, 0, 0
        )
        version = injector.recv_match(
            type="AUTOPILOT_VERSION", blocking=True, timeout=15
        )
        if version is None:
            raise TimeoutError("AUTOPILOT_VERSION discovery returned no message")
        packed, custom = record_firmware_discovery(result_dir, result, version)

        result["stage"] = "telemetry-validation"
        _checkpoint(result_dir, result)
        version_contract = AutopilotVersionContract(
            firmware_label="ArduCopter 4.5.7",
            flight_sw_version=packed,
            flight_custom_version=custom,
            evidence_reference=(
                f"isolated image {required_environment('QGC_SITL_ARDUCOPTER_IMAGE_ID')} "
                "AUTOPILOT_VERSION discovery pass"
            ),
        )
        policy = TelemetryStartupPolicy(
            command_ack_timeout_s=5.0,
            collection_timeout_s=15.0,
            home_request_timeout_s=5.0,
            poll_interval_s=0.05,
            minimum_distinct_samples=2,
            maximum_interval_error_fraction=1.0,
        )
        with timebase.configured(clock):
            collector = TelemetryStartupCollector(
                controller=controller,
                flight_profile=profile,
                policy=policy,
                autopilot_version=version_contract,
            )
            telemetry = collect_with_passive_receipts(
                collector=collector,
                vehicle=controller.vehicle,
                message_ids=tuple(
                    request.message_id for request in profile.telemetry_requests
                ),
                source_system=profile.flight_controller.system_id,
                source_component=profile.flight_controller.component_id,
                clock=clock.now,
                observer_sha256=hash_file(Path(__file__).resolve()),
                result=result,
                result_dir=result_dir,
            )
            result["firmware"].update(  # type: ignore[union-attr]
                validation_observed=telemetry.autopilot_version_received_at >= 0,  # type: ignore[attr-defined]
                telemetry_received_at={
                    str(key): list(value)
                    for key, value in telemetry.received_at_by_message_id.items()  # type: ignore[attr-defined]
                },
            )
            _checkpoint(result_dir, result)

            result["stage"] = "listener"
            _checkpoint(result_dir, result)
            supervisor = MissionSupervisor(
                7,
                admission_check=lambda _envelope: None,
                attempt_consumer=lambda _attempt: None,
                permission_check=lambda: None,
            )
            command_queue: queue.Queue[Any] = queue.Queue()
            transport = DroneKitQGCAckTransport(
                controller.vehicle,
                expected_source=profile.companion_target,
                wire_protocol="2.0",
            )
            listener = QGCCommandListener(
                supervisor=supervisor,
                profile=profile,
                command_queue=command_queue,
                clock=timebase.monotonic,
                ack_transport=transport,
                wire_protocol="2.0",
            )
            listener.install()
            def record_envelope(envelope: object) -> bool:
                nonlocal handler_count
                handler_count += 1
                _write_json_exclusive(
                    result_dir / "executed-envelope.json", asdict(envelope)
                )
                return True

            def process_one() -> None:
                nonlocal owner_calls
                owner_calls += 1
                owner.process_next(timeout_s=10.0)

            owner = CommandExecutionOwner(
                supervisor=supervisor,
                command_queue=command_queue,
                handlers={TEST_COMMAND: record_envelope},
                terminal_ack=listener.acknowledge_terminal,
                recovery=lambda: None,
                attempt_timeout_s=30.0,
                idle_poll_s=0.05,
                clock=timebase.monotonic,
            )
            owner_thread = threading.Thread(
                target=process_one, name="qgc-sitl-owner", daemon=True
            )
            packets: list[dict[str, object]] = result["packets"]  # type: ignore[assignment]
            acks: list[dict[str, object]] = result["acks"]  # type: ignore[assignment]

            def send_first() -> None:
                packets.append(_send_long(injector, request="first"))
                _checkpoint(result_dir, result)

            def receive_progress() -> object:
                message = _receive_ack(
                    injector, command=TEST_COMMAND, result=ACK_IN_PROGRESS, timeout_s=10
                )
                acks.append(_ack_row(message, "first"))
                _checkpoint(result_dir, result)
                return message

            def start_owner() -> None:
                owner_thread.start()

            def receive_terminal() -> object:
                message = _receive_ack(
                    injector, command=TEST_COMMAND, result=ACK_ACCEPTED, timeout_s=10
                )
                acks.append(_ack_row(message, "first"))
                _checkpoint(result_dir, result)
                return message

            progress, terminal = run_first_command(
                send=send_first,
                receive_progress=receive_progress,
                start_owner=start_owner,
                receive_terminal=receive_terminal,
            )
            owner_thread.join(timeout=10)
            owner_joined = not owner_thread.is_alive()
            if not owner_joined:
                raise TimeoutError("one-shot command owner did not join")
            packets.append(_send_long(injector, request="duplicate"))
            duplicate = _receive_ack(
                injector, command=TEST_COMMAND, result=ACK_ACCEPTED, timeout_s=10
            )
            acks.append(_ack_row(duplicate, "duplicate"))
            _checkpoint(result_dir, result)
            packets.append(
                _send_long(injector, request="wrong_source", source_system=201)
            )
            wrong_source_admitted = (
                injector.recv_match(type="COMMAND_ACK", blocking=True, timeout=1) is not None
            )
            packets.append(_send_int(injector))
            unsupported = _receive_ack(
                injector, command=TEST_COMMAND, result=ACK_UNSUPPORTED, timeout_s=10
            )
            acks.append(_ack_row(unsupported, "command_int"))
            result["wrong_source_admitted"] = wrong_source_admitted
            result["handler_count"] = handler_count
            result["owner"] = {
                "process_next_calls": owner_calls,
                "thread_joined": owner_joined,
            }
            result["clock"] = clock.snapshot()
            _checkpoint(result_dir, result)
            if handler_count != 1 or wrong_source_admitted:
                raise RuntimeError("exactly-once or wrong-source admission check failed")

        result.update(status="passed", stage="complete", reason=None)
    except BaseException as error:
        failure_stage = str(result.get("stage", "setup"))
        failure_reason = f"{type(error).__name__}: {error}"
        fail_result(result, failure_stage, failure_reason)
        inputs = result.get("inputs")
        if (
            isinstance(inputs, dict)
            and inputs.get("launcher_sha256") is None
            and startup_path.is_file()
        ):
            try:
                startup = json.loads(startup_path.read_text())
                record_startup_evidence(result_dir, result, startup)
            except BaseException as evidence_error:
                result["startup_evidence_error"] = (
                    f"{type(evidence_error).__name__}: {evidence_error}"
                )
            fail_result(result, failure_stage, failure_reason)
        if peer_error:
            result["json_peer_error"] = f"{type(peer_error[0]).__name__}: {peer_error[0]}"
    finally:
        if owner_thread is not None and owner_thread.is_alive():
            owner_thread.join(timeout=2)
        owner_joined = owner_thread is not None and not owner_thread.is_alive()
        connection_cleanup_ok = True
        for connection in (controller, injector):
            if connection is controller and connection is not None:
                connection_cleanup_ok = _bounded_call(
                    connection.vehicle.close, timeout_s=2
                ) and connection_cleanup_ok
            elif connection is not None:
                connection_cleanup_ok = _bounded_call(
                    connection.close, timeout_s=2
                ) and connection_cleanup_ok
        stop_peer.set()
        peer_cleanup_ok = True
        if peer is not None:
            try:
                peer.close()
            except Exception:
                peer_cleanup_ok = False
        peer_closed = peer_cleanup_ok
        if peer_thread is not None:
            peer_thread.join(timeout=2)
            peer_closed = peer_closed and not peer_thread.is_alive()
        result["handler_count"] = handler_count
        result["owner"] = {
            "process_next_calls": owner_calls,
            "thread_joined": owner_joined,
        }
        result["clock"] = clock.snapshot()
        result["cleanup"] = {
            "connections_closed": connection_cleanup_ok,
            "json_peer_closed": peer_closed,
            "owner_thread_joined": owner_joined,
        }
        finalize_result(result)
        _checkpoint(result_dir, result)
    return result


def main() -> int:
    result_dir = Path(required_environment("QGC_SITL_RESULT_DIR"))
    result_dir.mkdir(parents=True, exist_ok=True)
    result_path = result_dir / "result.json"

    def wall_timeout(_signum: int, _frame: object) -> None:
        raise TimeoutError("probe exceeded its 160 second wall limit")

    signal.signal(signal.SIGALRM, wall_timeout)
    signal.alarm(160)
    result = _run_probe(result_dir)
    signal.alarm(0)
    try:
        validate_result(result)
    except ValueError:
        exit_code = 1
    else:
        exit_code = 0
    _write_json_exclusive(result_path, result)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
