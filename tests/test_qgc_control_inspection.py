from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
from typing import Any, Callable

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink

sys.path.insert(0, str(Path(__file__).parents[1]))

from artifacts import ArtifactSession, ConfigurationRecord, FinalizationInput
from artifacts.manifest import MODULE_LOGS, REQUIRED_ARTIFACT_PATHS
from artifacts.operator_protocol import FlightAbortStatus, OperatorProtocol
from artifacts.qgc_operator import (
    MAV_RESULT_ACCEPTED,
    MAV_RESULT_DENIED,
    MAV_RESULT_IN_PROGRESS,
    MavlinkIdentity,
    QGC_ABORT_COMMAND,
    QGC_ACTIONS_KIND,
    QGC_COMPANION_IDENTITY,
    QGC_FLIGHT_CONTROLLER_IDENTITY,
    QGC_START_COMMAND,
    QgcCommandEnvelope,
    QgcCommandResponse,
    QgcCommandStatus,
    QgcOperatorDescriptor,
)
from artifacts.runtime_protocol import RuntimeProtocol
from artifacts.runtime_status import RuntimeRunningStatus
from orchestration.qgc_actions import _render_actions
from scripts.inspect_qgc_control_run import inspect_qgc_control_run


RUN_ID = "00000000-0000-4000-8000-000000000727"
START_REQUEST_ID = "00000000-0000-4000-8000-000000000728"
ABORT_REQUEST_ID = "00000000-0000-4000-8000-000000000729"
QGC_SOURCE = MavlinkIdentity(255, 190)
TOKEN = 27
NATIVE_EPOCH_NS = 90_000_000_000
WALL_EPOCH_NS = 1_760_148_000_000_000_000
STIMULUS_REQUEST_TEXT = (
    'name: "moving_pad" position { x: 55 y: 0 z: 0 } '
    "orientation { x: 0 y: 0 z: 0 w: 1 }"
)
PROFILE = {
    "LAND_SPEED": 50,
    "PLND_ENABLED": 1.0,
    "PLND_TYPE": 1.0,
    "PLND_EST_TYPE": 0.0,
    "PLND_LAG": 0.08,
    "PLND_XY_DIST_MAX": 0.50,
    "PLND_STRICT": 2.0,
    "PLND_RET_MAX": 1.0,
    "PLND_TIMEOUT": 0.50,
    "PLND_ALT_MIN": 0.75,
    "PLND_ALT_MAX": 8.0,
    "PLND_OPTIONS": 4.0,
}


class Message:
    def __init__(self, message_type: str, time_us: int, **fields: object) -> None:
        self._document = {
            "mavpackettype": message_type,
            "TimeUS": time_us,
            **fields,
        }

    def to_dict(self) -> dict[str, object]:
        return dict(self._document)


class Connection:
    def __init__(self, rows: list[Message]) -> None:
        self._rows = iter(rows)

    def recv_match(self, *, blocking: bool = False) -> Message | None:
        assert blocking is False
        return next(self._rows, None)

    def close(self) -> None:
        return None


@dataclass
class RunFixture:
    directory: Path
    descriptor: QgcOperatorDescriptor
    dataflash_rows: list[Message]

    def dataflash_connection(self, path: str) -> Connection:
        expected = self.directory / "ardupilot_sitl/logs/00000001.BIN"
        assert Path(path).resolve() == expected.resolve()
        return Connection(self.dataflash_rows)


def _log_row(
    event: str,
    fields: dict[str, object],
    sim_timestamp: float | None,
) -> dict[str, object]:
    return {
        "run_id": RUN_ID,
        "module": "companion",
        "severity": "INFO",
        "event": event,
        "sim_timestamp": sim_timestamp,
        "wall_timestamp": "2026-10-10T22:00:00+00:00",
        "fields": fields,
    }


def _mavlink_packet(
    source: MavlinkIdentity,
    message: Any,
    wall_time_ns: int,
) -> bytes:
    encoder = mavlink.MAVLink(
        None,
        srcSystem=source.system_id,
        srcComponent=source.component_id,
    )
    return struct.pack(">Q", wall_time_ns // 1_000) + message.pack(encoder)


def _native_exchange(
    *,
    command: int,
    result: int,
    progress: int,
    status_text: str,
    wall_time_ns: int,
) -> bytes:
    companion = mavlink.MAVLink(
        None,
        srcSystem=QGC_COMPANION_IDENTITY.system_id,
        srcComponent=QGC_COMPANION_IDENTITY.component_id,
    )
    qgc = mavlink.MAVLink(
        None,
        srcSystem=QGC_SOURCE.system_id,
        srcComponent=QGC_SOURCE.component_id,
    )
    return b"".join(
        (
            _mavlink_packet(
                QGC_COMPANION_IDENTITY,
                companion.heartbeat_encode(0, 0, 0, 0, 0, 3),
                wall_time_ns - 100_000_000,
            ),
            _mavlink_packet(
                QGC_SOURCE,
                qgc.command_long_encode(
                    1,
                    191,
                    command,
                    0,
                    float(TOKEN),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ),
                wall_time_ns,
            ),
            _mavlink_packet(
                QGC_COMPANION_IDENTITY,
                companion.command_ack_encode(
                    command,
                    result,
                    progress,
                    0,
                    QGC_SOURCE.system_id,
                    QGC_SOURCE.component_id,
                ),
                wall_time_ns + 10_000_000,
            ),
            _mavlink_packet(
                QGC_COMPANION_IDENTITY,
                companion.statustext_encode(
                    6,
                    status_text.encode("ascii"),
                    0,
                    0,
                ),
                wall_time_ns + 20_000_000,
            ),
        )
    )


def _intervention(command: int, wall_time_ns: int) -> dict[str, object]:
    return {
        "event": "qgc_manual_command",
        "wall_time_ns": wall_time_ns,
        "message_type": "COMMAND_LONG",
        "command": command,
        "source_system": 255,
        "source_component": 190,
        "fields": {
            "mavpackettype": "COMMAND_LONG",
            "target_system": 1,
            "target_component": 191,
            "command": command,
            "confirmation": 0,
            "param1": float(TOKEN),
            "param2": 0.0,
            "param3": 0.0,
            "param4": 0.0,
            "param5": 0.0,
            "param6": 0.0,
            "param7": 0.0,
        },
    }


def _profile_events(
    *, restore: bool, restore_timestamp: float = 8.0
) -> list[dict[str, object]]:
    events: list[dict[str, object]] = []
    for parameter, value in PROFILE.items():
        events.append(
            _log_row(
                "landing_profile_transition",
                {
                    "parameter": parameter,
                    "action": "read",
                    "stage": "requested",
                    "value": value,
                    "generation": 1,
                },
                1.0,
            )
        )
    verified_generations = (4, 8, 3, 12, 6, 10, 5, 13, 7, 11, 2, 9)
    for index, ((parameter, value), generation) in enumerate(
        zip(PROFILE.items(), verified_generations, strict=True)
    ):
        events.append(
            _log_row(
                "landing_profile_transition",
                {
                    "parameter": parameter,
                    "action": "read",
                    "stage": "verified",
                    "value": value,
                    "generation": generation,
                },
                1.1 + index * 0.01,
            )
        )
    events.extend(
        (
            _log_row(
                "landing_profile_transition",
                {
                    "parameter": "PLND_ENABLED",
                    "action": "set",
                    "stage": "requested",
                    "value": 0.0,
                    "generation": 13,
                },
                2.0,
            ),
            _log_row(
                "landing_profile_transition",
                {
                    "parameter": "PLND_ENABLED",
                    "action": "set",
                    "stage": "verified",
                    "value": 0.0,
                    "generation": 14,
                },
                2.1,
            ),
        )
    )
    if restore:
        events.extend(
            (
                _log_row(
                    "landing_profile_transition",
                    {
                        "parameter": "PLND_ENABLED",
                        "action": "set",
                        "stage": "requested",
                        "value": 1.0,
                        "generation": 14,
                    },
                    restore_timestamp,
                ),
                _log_row(
                    "landing_profile_transition",
                    {
                        "parameter": "PLND_ENABLED",
                        "action": "set",
                        "stage": "verified",
                        "value": 1.0,
                        "generation": 15,
                    },
                    restore_timestamp + 0.1,
                ),
            )
        )
    return events


def _precision_events() -> list[dict[str, object]]:
    return [
        _log_row(
            "operation_started",
            {
                "operation_id": "5",
                "tool": "precision_land",
                "args": {
                    "marker_id": 7,
                    "settle_by_sim_s": 45.0,
                    "acquire_by_sim_s": 60.0,
                },
                "timeout_sim_s": 90.0,
            },
            1.0,
        ),
        _log_row(
            "precision_mode_requested",
            {"operation_id": "5", "mode": "LAND"},
            2.0,
        ),
        _log_row(
            "precision_mode_requested",
            {"operation_id": "5", "mode": "GUIDED"},
            4.0,
        ),
        _log_row(
            "operation_finished",
            {
                "operation_id": "5",
                "tool": "precision_land",
                "state": "failed",
                "error": "precision target tracking was lost above final clearance",
            },
            4.1,
        ),
    ]


def _stimulus_record() -> dict[str, object]:
    pose_35 = {
        "native_timestamp_ns": 93_000_000_000,
        "position": [35.0, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }
    pose_55 = {
        "native_timestamp_ns": 93_100_000_000,
        "position": [55.0, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }
    persistent = {
        **pose_55,
        "native_timestamp_ns": 93_700_000_001,
    }
    return {
        "schema_version": 1,
        "status": "succeeded",
        "detail": "moving_pad remained at x=55",
        "run_id": RUN_ID,
        "config_sha256": "CONFIG_SHA256",
        "world": "moving_pad_stationary",
        "entity": "moving_pad",
        "partition": "drone_sim_" + RUN_ID.replace("-", "_"),
        "container": {
            "id": "0123456789ab",
            "project": "drone-sim-" + RUN_ID.replace("-", ""),
            "service": "gazebo-runtime",
            "companion_id": "abcdef012345",
        },
        "pre_command": {
            "pad_odometry": pose_35,
            "range": {
                "native_timestamp_ns": 93_000_000_000,
                "distance_m": 4.8,
            },
            "native_horizon_ns": 93_000_000_000,
            "mission": {
                "operation_id": "5",
                "sim_timestamp": 1.0,
                "native_start_ns": 91_000_000_000,
                "land_request_sim_timestamp": 2.0,
                "native_land_request_ns": 92_000_000_000,
                "marker_id": 7,
                "source": "live_companion_container_log",
                "source_container_id": "abcdef012345",
                "source_tail_lines": 4096,
            },
            "qgc_interventions": {},
            "dataflash": {
                "precision": {
                    "time_us": 92_800_000,
                    "acquired": True,
                    "last_measurement_ms": 92_800,
                },
                "mode": {"time_us": 92_100_000, "mode": 9},
                "precision_max_age_ns": 500_000_000,
                "land_max_age_ns": 5_000_000_000,
            },
        },
        "service": {
            "name": "/world/moving_pad_stationary/set_pose",
            "request_type": "gz.msgs.Pose",
            "response_type": "gz.msgs.Boolean",
            "request_text": STIMULUS_REQUEST_TEXT,
            "request_utf8_hex": STIMULUS_REQUEST_TEXT.encode().hex(),
            "request_sha256": hashlib.sha256(STIMULUS_REQUEST_TEXT.encode()).hexdigest(),
            "requested_wall_time_ns": WALL_EPOCH_NS + 2_000_000_000,
            "outcome": "returned",
            "returncode": 0,
            "response": "data: true\n",
            "stderr": "",
        },
        "post_command": {
            "first": pose_55,
            "persistence": [persistent],
            "displacement_m": 20.0,
            "minimum_persistence_ns": 500_000_000,
            "threshold_comparison": "strictly_greater_than",
            "qgc_interventions": {},
        },
    }


def _write_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
        encoding="utf-8",
    )


def _make_run(
    tmp_path: Path,
    *,
    admitted: bool = True,
    include_native_tlog: bool = True,
    dataflash_rows: list[Message] | None = None,
    companion_events: list[dict[str, object]] | None = None,
    abort: bool = False,
    abort_recovery: bool = False,
    abort_native_regression: bool = False,
    abort_accepted_native_missing: bool = False,
    ordinary: bool = False,
    restore: bool = True,
    restore_before_touchdown: bool = False,
    precision: bool = False,
    precision_complete_phases: bool = True,
    stimulus_mutator: Callable[[dict[str, object]], None] | None = None,
) -> RunFixture:
    run_directory = tmp_path / RUN_ID
    for relative in ("configuration", ".control", ".status/quiescence", "logs/docker"):
        (run_directory / relative).mkdir(parents=True, exist_ok=True)

    action_bytes = _render_actions(RUN_ID, TOKEN)
    descriptor = QgcOperatorDescriptor(
        kind=QGC_ACTIONS_KIND,
        start_command=QGC_START_COMMAND,
        abort_command=QGC_ABORT_COMMAND,
        attempt_token=TOKEN,
        start_request_id=START_REQUEST_ID,
        abort_request_id=ABORT_REQUEST_ID,
        qgc_source=QGC_SOURCE,
        companion=QGC_COMPANION_IDENTITY,
        flight_controller=QGC_FLIGHT_CONTROLLER_IDENTITY,
        actions_path="configuration/qgc-actions.json",
        actions_sha256=hashlib.sha256(action_bytes).hexdigest(),
    )
    configuration: dict[str, object] = {
        "run_id": RUN_ID,
        "world": "moving_pad_stationary" if precision else "competition",
        "vehicle": "iris_moving_pad" if precision else "iris",
        "mission": "configured" if precision else "controlled_descent",
        "scenario": "moving_pad_v1" if precision else "competition_v1",
        "output_root": str(tmp_path),
        "max_wall_seconds": 30,
        "startup_wall_seconds": 10,
        "finalization_wall_seconds": 10,
        "runtime_profile": "phase3",
        "recording": {
            "width_px": 640 if precision else 320,
            "height_px": 480 if precision else 240,
            "fps": 20,
            "encoding": "rgb8",
        },
        "simulation": {
            "seed": 9,
            "duration_sim_seconds": 10.0,
            "target_real_time_factor": 0.1,
            "public_epoch_native_sim_seconds": 90.0,
        },
        "launch_mode": "operator",
        "preparation_wall_seconds": 1800,
        "operator_control": asdict(descriptor),
    }
    if precision:
        configuration["mission_plan"] = {
            "schema_version": 1,
            "steps": [
                {
                    "tool": "precision_land",
                    "args": {
                        "marker_id": 7,
                        "settle_by_sim_s": 45.0,
                        "acquire_by_sim_s": 60.0,
                    },
                    "timeout_sim_s": 90.0,
                }
            ],
        }
    config_sha256 = hashlib.sha256(
        json.dumps(configuration, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    configuration["config_sha256"] = config_sha256
    (run_directory / "configuration/qgc-actions.json").write_bytes(action_bytes)
    (run_directory / "configuration/run.json").write_text(
        json.dumps(configuration), encoding="utf-8"
    )

    configuration_records = [
        ConfigurationRecord("configuration/run.json", config_sha256),
        ConfigurationRecord(
            "configuration/qgc-actions.json", descriptor.actions_sha256
        ),
    ]
    statuses: list[tuple[QgcCommandStatus, int, int, int, str]] = []
    with OperatorProtocol(run_directory, RUN_ID, config_sha256) as protocol:
        if admitted:
            request = protocol.write_start_request(START_REQUEST_ID)
            admission = protocol.write_start_admission(
                request, native_epoch_ns=NATIVE_EPOCH_NS
            )
            configuration_records.append(
                ConfigurationRecord(
                    "configuration/start-admission.json", admission.sha256
                )
            )
            for result, progress, text in (
                (MAV_RESULT_IN_PROGRESS, 0, "Start reserved"),
                (MAV_RESULT_ACCEPTED, 100, "Start admitted"),
            ):
                sequence = len(statuses)
                envelope = QgcCommandEnvelope(
                    2,
                    QGC_SOURCE,
                    QGC_COMPANION_IDENTITY,
                    QGC_START_COMMAND,
                    (TOKEN, 0, 0, 0, 0, 0, 0),
                    WALL_EPOCH_NS + sequence * 1_000_000_000,
                )
                status = QgcCommandStatus(
                    RUN_ID,
                    config_sha256,
                    sequence,
                    envelope,
                    request.request_id,
                    request.sha256,
                    admission.sha256 if result == MAV_RESULT_ACCEPTED else None,
                    envelope.received_wall_time_ns + 20_000_000,
                    QgcCommandResponse(
                        QGC_START_COMMAND, QGC_SOURCE, result, progress, text
                    ),
                )
                protocol.write_qgc_command_status(status)
                statuses.append((status, result, progress, QGC_START_COMMAND, text))
        else:
            envelope = QgcCommandEnvelope(
                2,
                QGC_SOURCE,
                QGC_COMPANION_IDENTITY,
                QGC_ABORT_COMMAND,
                (TOKEN, 0, 0, 0, 0, 0, 0),
                WALL_EPOCH_NS,
            )
            status = QgcCommandStatus(
                RUN_ID,
                config_sha256,
                0,
                envelope,
                None,
                None,
                None,
                WALL_EPOCH_NS + 20_000_000,
                QgcCommandResponse(
                    QGC_ABORT_COMMAND,
                    QGC_SOURCE,
                    MAV_RESULT_DENIED,
                    100,
                    "Abort requires Start",
                ),
            )
            protocol.write_qgc_command_status(status)
            statuses.append(
                (
                    status,
                    MAV_RESULT_DENIED,
                    100,
                    QGC_ABORT_COMMAND,
                    "Abort requires Start",
                )
            )

        if abort:
            assert admitted
            abort_request = protocol.write_flight_abort_request(
                ABORT_REQUEST_ID, admission.sha256
            )
            sequence = len(statuses)
            envelope = QgcCommandEnvelope(
                2,
                QGC_SOURCE,
                QGC_COMPANION_IDENTITY,
                QGC_ABORT_COMMAND,
                (TOKEN, 0, 0, 0, 0, 0, 0),
                WALL_EPOCH_NS + sequence * 1_000_000_000,
            )
            response = QgcCommandResponse(
                QGC_ABORT_COMMAND,
                QGC_SOURCE,
                MAV_RESULT_ACCEPTED,
                100,
                "Recovery landed",
            )
            status = QgcCommandStatus(
                RUN_ID,
                config_sha256,
                sequence,
                envelope,
                abort_request.request_id,
                abort_request.sha256,
                admission.sha256,
                envelope.received_wall_time_ns + 20_000_000,
                response,
                "landed_disarmed",
            )
            protocol.write_qgc_command_status(status)
            statuses.append(
                (
                    status,
                    MAV_RESULT_ACCEPTED,
                    100,
                    QGC_ABORT_COMMAND,
                    "Recovery landed",
                )
            )
            if abort_recovery:
                accepted_native_ns: int | None
                if abort_accepted_native_missing:
                    accepted_native_ns = None
                elif abort_native_regression:
                    accepted_native_ns = 100_000_000_000
                else:
                    accepted_native_ns = 94_000_000_000
                for sequence, state, native_ns in (
                    (0, "accepted", accepted_native_ns),
                    (1, "recovering", 94_100_000_000),
                    (2, "landed_disarmed", 98_000_000_000),
                ):
                    protocol.write_flight_abort_status(
                        FlightAbortStatus(
                            RUN_ID,
                            config_sha256,
                            abort_request.request_id,
                            admission.sha256,
                            sequence,
                            state,
                            WALL_EPOCH_NS + 5_000_000_000 + sequence,
                            native_ns,
                            state,
                        )
                    )

    if admitted:
        with RuntimeProtocol(run_directory, RUN_ID) as runtime:
            runtime.write_status(RuntimeRunningStatus(RUN_ID, 0))

    interventions = [
        _intervention(command, status.envelope.received_wall_time_ns)
        for status, _result, _progress, command, _text in statuses
    ]
    _write_jsonl(
        run_directory / "logs/docker/qgc-interventions.jsonl", interventions
    )
    if include_native_tlog:
        (run_directory / "logs/docker/native-qgc.tlog").write_bytes(
            b"".join(
                _native_exchange(
                    command=command,
                    result=result,
                    progress=progress,
                    status_text=text,
                    wall_time_ns=status.envelope.received_wall_time_ns,
                )
                for status, result, progress, command, text in statuses
            )
        )

    companion = list(companion_events or [])
    if ordinary:
        companion.extend(
            _profile_events(
                restore=restore,
                restore_timestamp=6.0 if restore_before_touchdown else 8.0,
            )
        )
    if precision:
        companion.extend(_precision_events())
        stimulus = _stimulus_record()
        stimulus["config_sha256"] = config_sha256
        qgc_summary = {
            "path": "logs/docker/qgc-interventions.jsonl",
            "start_wall_time_ns": WALL_EPOCH_NS + 1_000_000_000,
            "accepted_start_received_wall_time_ns": WALL_EPOCH_NS + 1_000_000_000,
            "rows_checked": 2,
        }
        pre_command = stimulus["pre_command"]
        post_command = stimulus["post_command"]
        assert isinstance(pre_command, dict)
        assert isinstance(post_command, dict)
        pre_command["qgc_interventions"] = qgc_summary
        post_command["qgc_interventions"] = qgc_summary
        if stimulus_mutator is not None:
            stimulus_mutator(stimulus)
        attempted = deepcopy(stimulus)
        attempted["status"] = "attempted"
        attempted["detail"] = "set-pose service dispatch pending"
        attempted["post_command"] = None
        assert isinstance(attempted["service"], dict)
        attempted["service"]["outcome"] = "pending"
        attempted["service"]["returncode"] = None
        attempted["service"]["response"] = None
        attempted["service"]["stderr"] = None
        response_received = deepcopy(stimulus)
        response_received["status"] = "response_received"
        response_received["detail"] = "set-pose service returned"
        response_received["post_command"] = None
        _write_jsonl(
            run_directory / "logs/docker/precision-loss-stimulus.jsonl",
            (
                [attempted, response_received, stimulus]
                if precision_complete_phases
                else [stimulus]
            ),
        )

    for relative_path in REQUIRED_ARTIFACT_PATHS:
        target = run_directory / relative_path
        if relative_path in {"configuration", "gazebo/state", "rosbag"}:
            target.mkdir(parents=True, exist_ok=True)
            if relative_path != "configuration":
                (target / "evidence").write_bytes(b"evidence")
        elif relative_path == "logs/companion.jsonl":
            _write_jsonl(target, companion or [_log_row("finalizing", {}, 9.0)])
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(b"evidence")
    dataflash_path = run_directory / "ardupilot_sitl/logs/00000001.BIN"
    dataflash_path.parent.mkdir(parents=True, exist_ok=True)
    dataflash_path.write_bytes(b"dataflash")

    rows = list(dataflash_rows or [])
    if ordinary and dataflash_rows is None:
        rows = [
            Message("ARM", 90_100_000, ArmState=1),
            Message("CTUN", 91_000_000, Alt=5.0, DAlt=5.0),
            Message("PARM", 92_100_000, Name="PLND_ENABLED", Value=0.0),
            Message("MODE", 92_200_000, Mode=9, ModeNum=9),
            Message("CTUN", 94_000_000, Alt=2.0, DAlt=0.0),
            *(
                [Message("PARM", 96_100_000, Name="PLND_ENABLED", Value=1.0)]
                if restore and restore_before_touchdown
                else []
            ),
            Message("RFND", 97_000_000, Dist=0.1),
            *(
                [Message("PARM", 98_100_000, Name="PLND_ENABLED", Value=1.0)]
                if restore and not restore_before_touchdown
                else []
            ),
            Message("ARM", 98_200_000, ArmState=0),
        ]
    if precision and dataflash_rows is None:
        rows = [
            Message("PARM", 90_100_000, Name="PLND_ENABLED", Value=1.0),
            Message("PARM", 90_100_001, Name="PLND_STRICT", Value=2.0),
            Message("MODE", 92_100_000, Mode=9, ModeNum=9),
            Message("PL", 92_800_000, TAcq=1, LastMeasMS=92800),
            Message("RFND", 93_000_000, Dist=4.8),
            Message("PL", 93_700_001, TAcq=0, LastMeasMS=93000),
            Message("MODE", 94_100_000, Mode=4, ModeNum=4),
        ]

    for module in (
        "orchestration",
        "companion",
        "ardupilot_sitl",
        "gazebo",
        "electromagnet",
        "scorekeeper",
    ):
        (run_directory / ".status/quiescence" / f"{module}.json").write_text(
            json.dumps({"run_id": RUN_ID, "module": module, "quiescent": True}),
            encoding="utf-8",
        )

    terminal = "FAILED" if precision else ("ABORTED" if not admitted else "COMPLETED")
    reason = "precision target tracking was lost" if precision else "test finalization"
    (run_directory / ".control/finalize-request.json").write_text(
        json.dumps(
            {"run_id": RUN_ID, "requested_terminal": terminal, "reason": reason}
        ),
        encoding="utf-8",
    )
    now = datetime(2026, 10, 10, tzinfo=timezone.utc)
    ArtifactSession(run_directory).finalize(
        FinalizationInput(
            RUN_ID,
            terminal,
            reason,
            0,
            10_000_000_000,
            now,
            now,
            (),
            (),
            tuple(configuration_records),
            None,
            None,
            None,
            (),
        )
    )
    (run_directory / ".control/terminal-committed.json").write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "terminal_status": terminal,
                "reason": reason,
                "manifest_path": "manifest.json",
            }
        ),
        encoding="utf-8",
    )
    return RunFixture(run_directory, descriptor, rows)


def _inspect(run: RunFixture, expectation: str) -> dict[str, Any]:
    return inspect_qgc_control_run(
        run.directory,
        expectation,
        dataflash_connection_factory=run.dataflash_connection,
    )


def test_routing_accepts_rejected_round_trip_without_start_admission(tmp_path: Path) -> None:
    run = _make_run(tmp_path, admitted=False)

    report = _inspect(run, "routing")

    assert report["accepted"] is True
    assert report["findings"]["command"]["status"] == "passed"
    assert report["findings"]["physical"] == {
        "status": "not_required",
        "detail": "routing does not require physical flight proof",
    }
    assert report["findings"]["profile"] == {
        "status": "not_required",
        "detail": "routing does not require landing-profile proof",
    }
    assert report["findings"]["cleanup"]["status"] == "passed"


def test_cli_emits_json_and_success_exit_for_routing_proof(tmp_path: Path) -> None:
    run = _make_run(tmp_path, admitted=False)

    completed = subprocess.run(
        [
            sys.executable,
            "scripts/inspect_qgc_control_run.py",
            str(run.directory),
            "--expect",
            "routing",
        ],
        cwd=Path(__file__).parents[1],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0
    assert completed.stderr == ""
    assert json.loads(completed.stdout)["accepted"] is True


def test_start_rejects_accepted_ack_without_independent_flight(tmp_path: Path) -> None:
    run = _make_run(tmp_path)

    report = _inspect(run, "start")

    assert report["accepted"] is False
    assert report["findings"]["command"]["status"] == "passed"
    assert report["findings"]["physical"]["status"] == "failed"
    assert "armed" in report["findings"]["physical"]["detail"]


def test_start_accepts_armed_and_airborne_after_native_epoch(tmp_path: Path) -> None:
    run = _make_run(
        tmp_path,
        dataflash_rows=[
            Message("ARM", 90_100_000, ArmState=1),
            Message("CTUN", 91_000_000, Alt=1.5, DAlt=1.5),
        ],
    )

    report = _inspect(run, "start")

    assert report["accepted"] is True
    assert report["findings"]["physical"]["status"] == "passed"


def test_start_rejects_vehicle_rows_before_admitted_native_epoch(tmp_path: Path) -> None:
    run = _make_run(
        tmp_path,
        dataflash_rows=[
            Message("ARM", 89_900_000, ArmState=1),
            Message("CTUN", 89_950_000, Alt=2.0, DAlt=2.0),
        ],
    )

    report = _inspect(run, "start")

    assert report["accepted"] is False
    assert report["findings"]["physical"]["status"] == "failed"
    assert "native epoch" in report["findings"]["physical"]["detail"]


def test_inspection_rejects_old_action_bytes_after_manifest_binding(tmp_path: Path) -> None:
    run = _make_run(tmp_path, admitted=False)
    (run.directory / "configuration/qgc-actions.json").write_bytes(b"old action\n")

    report = _inspect(run, "routing")

    assert report["accepted"] is False
    assert report["findings"]["command"]["status"] == "failed"
    assert "action" in report["findings"]["command"]["detail"]


def test_inspection_rejects_corrupted_immutable_run_configuration(
    tmp_path: Path,
) -> None:
    run = _make_run(tmp_path, admitted=False)
    path = run.directory / "configuration/run.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    document["max_wall_seconds"] += 1
    path.write_text(json.dumps(document), encoding="utf-8")

    report = _inspect(run, "routing")

    assert report["accepted"] is False
    assert report["findings"]["command"]["status"] == "failed"
    assert "immutable run configuration" in report["findings"]["command"]["detail"]


def test_routing_rejects_command_status_added_after_manifest_commit(
    tmp_path: Path,
) -> None:
    run = _make_run(tmp_path, admitted=False)
    configuration = json.loads(
        (run.directory / "configuration/run.json").read_text(encoding="utf-8")
    )
    with OperatorProtocol(run.directory, RUN_ID, configuration["config_sha256"]) as protocol:
        existing = protocol.read_qgc_command_statuses()
        protocol.write_qgc_command_status(
            replace(
                existing[-1],
                sequence=1,
                responded_wall_time_ns=existing[-1].responded_wall_time_ns + 1,
            )
        )

    report = _inspect(run, "routing")

    assert report["accepted"] is False
    assert report["findings"]["command"]["status"] == "failed"
    assert "command evidence inventory" in report["findings"]["command"]["detail"]


def test_routing_rejects_incomplete_native_source_evidence(tmp_path: Path) -> None:
    run = _make_run(tmp_path, admitted=False, include_native_tlog=False)

    report = _inspect(run, "routing")

    assert report["accepted"] is False
    assert report["findings"]["command"]["status"] == "failed"
    assert "native-qgc.tlog" in report["findings"]["command"]["detail"]


def test_inspection_reports_missing_terminal_commit_as_cleanup_failure(tmp_path: Path) -> None:
    run = _make_run(tmp_path, admitted=False)
    (run.directory / ".control/terminal-committed.json").unlink()

    report = _inspect(run, "routing")

    assert report["accepted"] is False
    assert report["findings"]["command"]["status"] == "passed"
    assert report["findings"]["cleanup"]["status"] == "failed"
    assert "terminal commit" in report["findings"]["cleanup"]["detail"]


def test_abort_ack_without_ordered_recovery_and_flight_evidence_fails(tmp_path: Path) -> None:
    run = _make_run(tmp_path, abort=True)

    report = _inspect(run, "abort")

    assert report["accepted"] is False
    assert report["findings"]["physical"]["status"] == "failed"
    assert "recovery" in report["findings"]["physical"]["detail"]


def test_abort_requires_touchdown_and_disarm_after_recovery_begins(tmp_path: Path) -> None:
    run = _make_run(
        tmp_path,
        abort=True,
        abort_recovery=True,
        dataflash_rows=[
            Message("ARM", 90_100_000, ArmState=1),
            Message("CTUN", 91_000_000, Alt=2.0, DAlt=2.0),
            Message("RFND", 97_000_000, Dist=0.1),
            Message("ARM", 97_100_000, ArmState=0),
        ],
    )

    report = _inspect(run, "abort")

    assert report["accepted"] is True
    assert report["findings"]["physical"]["status"] == "passed"


def test_abort_rejects_regressing_present_native_status_timestamps(
    tmp_path: Path,
) -> None:
    run = _make_run(
        tmp_path,
        abort=True,
        abort_recovery=True,
        abort_native_regression=True,
        dataflash_rows=[
            Message("ARM", 90_100_000, ArmState=1),
            Message("CTUN", 91_000_000, Alt=2.0, DAlt=2.0),
            Message("RFND", 97_000_000, Dist=0.1),
            Message("ARM", 97_100_000, ArmState=0),
        ],
    )

    report = _inspect(run, "abort")

    assert report["accepted"] is False
    assert report["findings"]["physical"]["status"] == "failed"
    assert "native timestamps regress" in report["findings"]["physical"]["detail"]


def test_abort_accepts_optional_native_timestamp_absent_before_recovery(
    tmp_path: Path,
) -> None:
    run = _make_run(
        tmp_path,
        abort=True,
        abort_recovery=True,
        abort_accepted_native_missing=True,
        dataflash_rows=[
            Message("ARM", 90_100_000, ArmState=1),
            Message("CTUN", 91_000_000, Alt=2.0, DAlt=2.0),
            Message("RFND", 97_000_000, Dist=0.1),
            Message("ARM", 97_100_000, ArmState=0),
        ],
    )

    report = _inspect(run, "abort")

    assert report["accepted"] is True
    assert report["findings"]["physical"]["status"] == "passed"


def test_ordinary_landing_rejects_touchdown_without_verified_restore(tmp_path: Path) -> None:
    run = _make_run(tmp_path, ordinary=True, restore=False)

    report = _inspect(run, "ordinary-land")

    assert report["accepted"] is False
    assert report["findings"]["physical"]["status"] == "passed"
    assert report["findings"]["profile"]["status"] == "failed"
    assert "restore" in report["findings"]["profile"]["detail"]


def test_ordinary_landing_accepts_task5_batched_read_and_touchdown_restore(
    tmp_path: Path,
) -> None:
    run = _make_run(tmp_path, ordinary=True)

    report = _inspect(run, "ordinary-land")

    assert report["accepted"] is True
    assert {name: value["status"] for name, value in report["findings"].items()} == {
        "command": "passed",
        "physical": "passed",
        "profile": "passed",
        "cleanup": "passed",
    }


def test_ordinary_landing_rejects_restore_before_independent_touchdown(
    tmp_path: Path,
) -> None:
    run = _make_run(tmp_path, ordinary=True, restore_before_touchdown=True)

    report = _inspect(run, "ordinary-land")

    assert report["accepted"] is False
    assert report["findings"]["physical"]["status"] == "failed"
    assert "restore precedes independent touchdown" in report["findings"]["physical"]["detail"]


def test_precision_loss_requires_acquisition_loss_policy_and_persistent_stimulus(
    tmp_path: Path,
) -> None:
    run = _make_run(tmp_path, precision=True)

    report = _inspect(run, "precision-loss")

    assert report["accepted"] is True
    assert report["findings"]["physical"]["status"] == "passed"
    assert report["findings"]["profile"]["status"] == "passed"


def test_precision_loss_rejects_terminal_success_without_durable_prior_phases(
    tmp_path: Path,
) -> None:
    run = _make_run(tmp_path, precision=True, precision_complete_phases=False)

    report = _inspect(run, "precision-loss")

    assert report["accepted"] is False
    assert report["findings"]["physical"]["status"] == "failed"
    assert "three" in report["findings"]["physical"]["detail"]


def test_precision_loss_rejects_stimulus_bound_to_another_operation(
    tmp_path: Path,
) -> None:
    def mutate(record: dict[str, object]) -> None:
        pre = record["pre_command"]
        assert isinstance(pre, dict)
        mission = pre["mission"]
        assert isinstance(mission, dict)
        mission["operation_id"] = "foreign"

    run = _make_run(tmp_path, precision=True, stimulus_mutator=mutate)

    report = _inspect(run, "precision-loss")

    assert report["accepted"] is False
    assert report["findings"]["physical"]["status"] == "failed"
    assert "mission binding" in report["findings"]["physical"]["detail"]


def test_precision_loss_rejects_weakened_persistence_contract(tmp_path: Path) -> None:
    def mutate(record: dict[str, object]) -> None:
        post = record["post_command"]
        assert isinstance(post, dict)
        post["threshold_comparison"] = "greater_than_or_equal"

    run = _make_run(tmp_path, precision=True, stimulus_mutator=mutate)

    report = _inspect(run, "precision-loss")

    assert report["accepted"] is False
    assert report["findings"]["physical"]["status"] == "failed"
    assert "persistence contract" in report["findings"]["physical"]["detail"]


def test_precision_loss_rejects_stimulus_that_did_not_persist_over_half_second(
    tmp_path: Path,
) -> None:
    run = _make_run(tmp_path, precision=True)
    path = run.directory / "logs/docker/precision-loss-stimulus.jsonl"
    records = [json.loads(line) for line in path.read_text().splitlines()]
    records[-1]["post_command"]["persistence"][-1]["native_timestamp_ns"] = 93_600_000_000
    _write_jsonl(path, records)

    report = _inspect(run, "precision-loss")

    assert report["accepted"] is False
    assert report["findings"]["physical"]["status"] == "failed"
    assert (
        "checksum" in report["findings"]["cleanup"]["detail"]
        or "persist" in report["findings"]["physical"]["detail"]
    )
