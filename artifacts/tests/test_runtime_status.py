from collections import UserDict
from copy import deepcopy
from dataclasses import dataclass
from typing import ClassVar

import pytest

from artifacts.protocol_files import WritePolicy
from artifacts.runtime_status import (
    ArduPilotReadyStatus,
    ArtifactFinalRecord,
    ArtifactsFinalStatus,
    ArtifactsReadyStatus,
    CompanionReadyStatus,
    FlightExchange,
    GazeboReadyStatus,
    MissionCommandDeliveredStatus,
    MissionFinishedStatus,
    MissionReadyStatus,
    RuntimeFailureStatus,
    RuntimeFrozenStatus,
    RuntimeRunningStatus,
    RuntimeStatus,
    ScoreFinishedStatus,
    SourceFinishedStatus,
    TerminalNotifiedStatus,
    RuntimeStatusError,
    canonical_run_id,
    parse_status,
    status_document,
    status_name,
    status_write_policy,
)
from artifacts.validation import ValidationStatus


RUN_ID = "11111111-1111-4111-8111-111111111111"
OTHER_RUN_ID = "22222222-2222-4222-8222-222222222222"
DIGEST_A = "a" * 64
DIGEST_B = "b" * 64
DIGEST_C = "c" * 64

FLIGHT_EXCHANGE = FlightExchange(
    online=True,
    servo_packets_received=2,
    motor_updates=2,
    duplicate_servo_packets=0,
    servo_frame_gaps=0,
    json_states_sent=2,
    json_send_errors=0,
    last_servo_frame=1,
    last_json_sim_time_ns=40_000_000,
)

FINAL_RECORDS = (
    ArtifactFinalRecord(
        "video/onboard.mp4",
        ValidationStatus.VALID,
        "valid video",
        10,
        DIGEST_A,
        {"codec": "h264"},
    ),
    ArtifactFinalRecord(
        "video/observer.mp4",
        ValidationStatus.VALID,
        "valid video",
        20,
        DIGEST_B,
        {"frames": [1, 2]},
    ),
    ArtifactFinalRecord(
        "rosbag",
        ValidationStatus.VALID,
        "valid bag",
        30,
        DIGEST_C,
        {"topics": ["/clock"]},
    ),
)

CASES = (
    (
        ArtifactsReadyStatus(RUN_ID),
        "artifacts-ready",
        {"run_id": RUN_ID, "ready": True},
    ),
    (
        GazeboReadyStatus(RUN_ID, FLIGHT_EXCHANGE),
        "gazebo-ready",
        {
            "run_id": RUN_ID,
            "ready": True,
            "flight_exchange": {
                "online": True,
                "servo_packets_received": 2,
                "motor_updates": 2,
                "duplicate_servo_packets": 0,
                "servo_frame_gaps": 0,
                "json_states_sent": 2,
                "json_send_errors": 0,
                "last_servo_frame": 1,
                "last_json_sim_time_ns": 40_000_000,
            },
        },
    ),
    (
        ArduPilotReadyStatus(RUN_ID),
        "ardupilot-ready",
        {
            "run_id": RUN_ID,
            "ready": True,
            "json_exchange": True,
            "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
        },
    ),
    (
        CompanionReadyStatus(RUN_ID),
        "companion-ready",
        {
            "run_id": RUN_ID,
            "ready": True,
            "mavlink_endpoint": "tcp://ardupilot-sitl:5760",
            "mavlink_transport_connected": True,
        },
    ),
    (
        MissionReadyStatus(RUN_ID),
        "mission-ready",
        {
            "run_id": RUN_ID,
            "ready": True,
            "heartbeat_observed": True,
            "prearm_checks_healthy": True,
        },
    ),
    (
        MissionCommandDeliveredStatus(RUN_ID, 50_000_000),
        "mission-command-delivered",
        {
            "run_id": RUN_ID,
            "command": "SET_GUIDED",
            "sim_timestamp_ns": 50_000_000,
            "delivered": True,
        },
    ),
    (
        RuntimeRunningStatus(RUN_ID, 1),
        "runtime-running",
        {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": 1},
    ),
    (
        SourceFinishedStatus(RUN_ID, 2),
        "source-finished",
        {"run_id": RUN_ID, "finished": True, "sim_timestamp_ns": 2},
    ),
    (
        MissionFinishedStatus(RUN_ID, 3),
        "mission-finished",
        {
            "run_id": RUN_ID,
            "finished": True,
            "sim_timestamp_ns": 3,
            "outcome": "LANDED",
        },
    ),
    (
        ScoreFinishedStatus(RUN_ID, 4),
        "score-finished",
        {"run_id": RUN_ID, "finished": True, "sim_timestamp_ns": 4},
    ),
    (
        RuntimeFailureStatus(
            RUN_ID, "gazebo", "exchange stopped", ("logs/gazebo.log",)
        ),
        "runtime-failure",
        {
            "run_id": RUN_ID,
            "module": "gazebo",
            "reason": "exchange stopped",
            "diagnostic_paths": ["logs/gazebo.log"],
        },
    ),
    (
        RuntimeFrozenStatus(RUN_ID),
        "runtime-frozen",
        {"run_id": RUN_ID, "frozen": True},
    ),
    (
        ArtifactsFinalStatus(RUN_ID, FINAL_RECORDS),
        "artifacts-final",
        {
            "run_id": RUN_ID,
            "complete": True,
            "records": [
                {
                    "relative_path": "video/onboard.mp4",
                    "status": "valid",
                    "detail": "valid video",
                    "size_bytes": 10,
                    "sha256": DIGEST_A,
                    "semantic": {"codec": "h264"},
                },
                {
                    "relative_path": "video/observer.mp4",
                    "status": "valid",
                    "detail": "valid video",
                    "size_bytes": 20,
                    "sha256": DIGEST_B,
                    "semantic": {"frames": [1, 2]},
                },
                {
                    "relative_path": "rosbag",
                    "status": "valid",
                    "detail": "valid bag",
                    "size_bytes": 30,
                    "sha256": DIGEST_C,
                    "semantic": {"topics": ["/clock"]},
                },
            ],
        },
    ),
    (
        TerminalNotifiedStatus(RUN_ID),
        "terminal-notified",
        {"run_id": RUN_ID, "notified": True},
    ),
)


@pytest.mark.parametrize(("status", "name", "document"), CASES)
def test_registered_status_round_trip(status, name, document):
    assert status_name(type(status)) == name
    assert type(status).name == name
    assert status_document(status) == document
    parsed = parse_status(type(status), document, expected_run_id=RUN_ID)
    assert type(parsed) is type(status)
    assert parsed == status


def test_registered_status_names_are_unique():
    assert len({name for _status, name, _document in CASES}) == len(CASES)


@pytest.mark.parametrize(("status", "name", "document"), CASES)
def test_registered_status_write_policy(status, name, document):
    del name, document
    expected = (
        WritePolicy.FIRST_WINS
        if type(status) is RuntimeFailureStatus
        else WritePolicy.IDENTICAL
    )
    assert status_write_policy(type(status)) is expected


@pytest.mark.parametrize(("status", "name", "document"), CASES)
@pytest.mark.parametrize("replacement", [None, [], "mapping"])
def test_schema_rejects_non_exact_dictionary(status, name, document, replacement):
    del name
    malformed = UserDict(document) if replacement == "mapping" else replacement
    with pytest.raises(RuntimeStatusError):
        parse_status(type(status), malformed, expected_run_id=RUN_ID)


@pytest.mark.parametrize(("status", "name", "document"), CASES)
def test_schema_rejects_missing_and_extra_top_level_keys(status, name, document):
    del name
    missing = deepcopy(document)
    missing.pop(next(iter(missing)))
    extra = deepcopy(document)
    extra["extra"] = 1
    with pytest.raises(RuntimeStatusError):
        parse_status(type(status), missing, expected_run_id=RUN_ID)
    with pytest.raises(RuntimeStatusError):
        parse_status(type(status), extra, expected_run_id=RUN_ID)


INVALID_RUN_IDS = (
    "AAAAAAAA-AAAA-4AAA-8AAA-AAAAAAAAAAAA",
    "{aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa}",
    "aaaaaaaaaaaa4aaa8aaaaaaaaaaaaaaa",
    "not-a-uuid",
    None,
    7,
)


def test_canonical_run_id_accepts_only_canonical_text():
    assert canonical_run_id(RUN_ID) == RUN_ID
    for value in INVALID_RUN_IDS:
        with pytest.raises(RuntimeStatusError):
            canonical_run_id(value)
        with pytest.raises(RuntimeStatusError):
            ArtifactsReadyStatus(value)


@pytest.mark.parametrize(("status", "name", "document"), CASES)
@pytest.mark.parametrize("bad_run_id", (*INVALID_RUN_IDS, OTHER_RUN_ID))
def test_run_id_is_canonical_and_matches_expected(status, name, document, bad_run_id):
    del name
    malformed = deepcopy(document)
    malformed["run_id"] = bad_run_id
    with pytest.raises(RuntimeStatusError):
        parse_status(type(status), malformed, expected_run_id=RUN_ID)


@pytest.mark.parametrize(("status", "name", "document"), CASES)
def test_run_id_rejects_cross_run_expectation(status, name, document):
    del name
    with pytest.raises(RuntimeStatusError):
        parse_status(type(status), document, expected_run_id=OTHER_RUN_ID)


CONSTANT_MUTATIONS = (
    (0, ("ready",), False),
    (1, ("ready",), False),
    (1, ("flight_exchange", "online"), False),
    (2, ("ready",), False),
    (2, ("json_exchange",), False),
    (2, ("mavlink_endpoint",), "udp://wrong"),
    (3, ("ready",), False),
    (3, ("mavlink_endpoint",), "udp://wrong"),
    (3, ("mavlink_transport_connected",), False),
    (4, ("ready",), False),
    (4, ("heartbeat_observed",), False),
    (4, ("prearm_checks_healthy",), False),
    (5, ("command",), "WRONG"),
    (5, ("delivered",), False),
    (6, ("state",), "READY"),
    (7, ("finished",), False),
    (8, ("finished",), False),
    (8, ("outcome",), "CRASHED"),
    (9, ("finished",), False),
    (11, ("frozen",), False),
    (13, ("notified",), False),
)


@pytest.mark.parametrize(("case_index", "path", "replacement"), CONSTANT_MUTATIONS)
def test_constant_fields_are_exact(case_index, path, replacement):
    status, _name, document = CASES[case_index]
    malformed = deepcopy(document)
    target = malformed
    for component in path[:-1]:
        target = target[component]
    target[path[-1]] = replacement
    with pytest.raises(RuntimeStatusError):
        parse_status(type(status), malformed, expected_run_id=RUN_ID)


TIMESTAMP_CASES = (
    (MissionCommandDeliveredStatus, 5),
    (RuntimeRunningStatus, 6),
    (SourceFinishedStatus, 7),
    (MissionFinishedStatus, 8),
    (ScoreFinishedStatus, 9),
)


@pytest.mark.parametrize(("status_type", "case_index"), TIMESTAMP_CASES)
@pytest.mark.parametrize("timestamp", [True, False, -1])
def test_timestamp_rejects_booleans_and_negative_values(
    status_type, case_index, timestamp
):
    with pytest.raises(RuntimeStatusError):
        status_type(RUN_ID, timestamp)
    document = deepcopy(CASES[case_index][2])
    document["sim_timestamp_ns"] = timestamp
    with pytest.raises(RuntimeStatusError):
        parse_status(status_type, document, expected_run_id=RUN_ID)


def test_mission_command_timestamp_rejects_value_above_startup_window():
    with pytest.raises(RuntimeStatusError):
        MissionCommandDeliveredStatus(RUN_ID, 50_000_001)
    document = deepcopy(CASES[5][2])
    document["sim_timestamp_ns"] = 50_000_001
    with pytest.raises(RuntimeStatusError):
        parse_status(MissionCommandDeliveredStatus, document, expected_run_id=RUN_ID)


FLIGHT_FIELDS = tuple(FLIGHT_EXCHANGE.__dataclass_fields__)
FLIGHT_COUNTERS = tuple(field for field in FLIGHT_FIELDS if field != "online")


def _flight_values(**changes):
    values = CASES[1][2]["flight_exchange"].copy()
    values.update(changes)
    return values


@pytest.mark.parametrize("field", FLIGHT_FIELDS)
def test_flight_schema_rejects_missing_nested_field(field):
    document = deepcopy(CASES[1][2])
    del document["flight_exchange"][field]
    with pytest.raises(RuntimeStatusError):
        parse_status(GazeboReadyStatus, document, expected_run_id=RUN_ID)


def test_flight_schema_rejects_extra_nested_field():
    document = deepcopy(CASES[1][2])
    document["flight_exchange"]["extra"] = 1
    with pytest.raises(RuntimeStatusError):
        parse_status(GazeboReadyStatus, document, expected_run_id=RUN_ID)


@pytest.mark.parametrize("online", [False, 1])
def test_flight_online_requires_exact_true(online):
    with pytest.raises(RuntimeStatusError):
        FlightExchange(**_flight_values(online=online))
    document = deepcopy(CASES[1][2])
    document["flight_exchange"]["online"] = online
    with pytest.raises(RuntimeStatusError):
        parse_status(GazeboReadyStatus, document, expected_run_id=RUN_ID)


@pytest.mark.parametrize("field", FLIGHT_COUNTERS)
@pytest.mark.parametrize("value", [True, -1])
def test_flight_counters_reject_booleans_and_negative_values(field, value):
    with pytest.raises(RuntimeStatusError):
        FlightExchange(**_flight_values(**{field: value}))
    document = deepcopy(CASES[1][2])
    document["flight_exchange"][field] = value
    with pytest.raises(RuntimeStatusError):
        parse_status(GazeboReadyStatus, document, expected_run_id=RUN_ID)


@pytest.mark.parametrize(
    "field", ["servo_packets_received", "motor_updates", "json_states_sent"]
)
def test_flight_progress_counters_require_progress(field):
    with pytest.raises(RuntimeStatusError):
        FlightExchange(**_flight_values(**{field: 0}))
    document = deepcopy(CASES[1][2])
    document["flight_exchange"][field] = 0
    with pytest.raises(RuntimeStatusError):
        parse_status(GazeboReadyStatus, document, expected_run_id=RUN_ID)


@pytest.mark.parametrize("field", ["servo_frame_gaps", "json_send_errors"])
def test_flight_zero_counters_require_zero(field):
    with pytest.raises(RuntimeStatusError):
        FlightExchange(**_flight_values(**{field: 1}))
    document = deepcopy(CASES[1][2])
    document["flight_exchange"][field] = 1
    with pytest.raises(RuntimeStatusError):
        parse_status(GazeboReadyStatus, document, expected_run_id=RUN_ID)


def test_flight_status_constructor_requires_exact_exchange_type():
    with pytest.raises(RuntimeStatusError):
        GazeboReadyStatus(RUN_ID, CASES[1][2]["flight_exchange"])


@pytest.mark.parametrize(
    ("field", "value"),
    [("module", ""), ("module", 1), ("reason", ""), ("reason", None)],
)
def test_failure_requires_nonempty_module_and_reason(field, value):
    arguments = {
        "module": "gazebo",
        "reason": "exchange stopped",
        "diagnostic_paths": ("logs/gazebo.log",),
    }
    arguments[field] = value
    with pytest.raises(RuntimeStatusError):
        RuntimeFailureStatus(RUN_ID, **arguments)
    document = deepcopy(CASES[10][2])
    document[field] = value
    with pytest.raises(RuntimeStatusError):
        parse_status(RuntimeFailureStatus, document, expected_run_id=RUN_ID)


def test_failure_diagnostic_container_shape_is_directional():
    with pytest.raises(RuntimeStatusError):
        RuntimeFailureStatus(RUN_ID, "gazebo", "stopped", ["logs/x"])
    document = deepcopy(CASES[10][2])
    document["diagnostic_paths"] = ("logs/x",)
    with pytest.raises(RuntimeStatusError):
        parse_status(RuntimeFailureStatus, document, expected_run_id=RUN_ID)


INVALID_DIAGNOSTIC_PATHS = (
    "",
    "/tmp/x",
    "a\\b",
    "../x",
    "a/../b",
    "a//b",
    "a/",
    "./a",
    7,
)


@pytest.mark.parametrize("path", INVALID_DIAGNOSTIC_PATHS)
def test_failure_diagnostic_paths_reject_noncanonical_values(path):
    with pytest.raises(RuntimeStatusError):
        RuntimeFailureStatus(RUN_ID, "gazebo", "stopped", (path,))
    document = deepcopy(CASES[10][2])
    document["diagnostic_paths"] = [path]
    with pytest.raises(RuntimeStatusError):
        parse_status(RuntimeFailureStatus, document, expected_run_id=RUN_ID)


def test_failure_diagnostic_paths_must_be_unique():
    paths = ("logs/x", "logs/x")
    with pytest.raises(RuntimeStatusError):
        RuntimeFailureStatus(RUN_ID, "gazebo", "stopped", paths)
    document = deepcopy(CASES[10][2])
    document["diagnostic_paths"] = list(paths)
    with pytest.raises(RuntimeStatusError):
        parse_status(RuntimeFailureStatus, document, expected_run_id=RUN_ID)


@pytest.mark.parametrize("paths", [(), ("logs/x",), ("logs/a/b.partial",)])
def test_failure_diagnostic_paths_accept_relative_posix_paths(paths):
    status = RuntimeFailureStatus(RUN_ID, "gazebo", "stopped", paths)
    document = status_document(status)
    assert parse_status(RuntimeFailureStatus, document, expected_run_id=RUN_ID) == status


def _make_records(
    *,
    first_status=ValidationStatus.VALID,
    first_size=10,
    first_sha=DIGEST_A,
    first_detail="valid video",
    first_semantic=None,
):
    semantic = {"codec": "h264"} if first_semantic is None else first_semantic
    return (
        ArtifactFinalRecord(
            "video/onboard.mp4",
            first_status,
            first_detail,
            first_size,
            first_sha,
            semantic,
        ),
        FINAL_RECORDS[1],
        FINAL_RECORDS[2],
    )


@pytest.mark.parametrize(
    ("record_status", "size", "digest"),
    [
        (ValidationStatus.VALID, 10, DIGEST_A),
        (ValidationStatus.MISSING, None, None),
        (ValidationStatus.MISSING, 10, DIGEST_A),
        (ValidationStatus.INVALID, None, None),
        (ValidationStatus.INVALID, 10, DIGEST_A),
    ],
)
def test_artifact_record_accepts_status_size_hash_combinations(
    record_status, size, digest
):
    records = _make_records(
        first_status=record_status, first_size=size, first_sha=digest
    )
    status = ArtifactsFinalStatus(RUN_ID, records)
    assert parse_status(
        ArtifactsFinalStatus, status_document(status), expected_run_id=RUN_ID
    ) == status


@pytest.mark.parametrize(
    ("record_status", "size", "digest"),
    [
        (ValidationStatus.VALID, None, None),
        (ValidationStatus.VALID, 10, None),
        (ValidationStatus.VALID, None, DIGEST_A),
        (ValidationStatus.MISSING, 10, None),
        (ValidationStatus.MISSING, None, DIGEST_A),
        (ValidationStatus.INVALID, 10, None),
        (ValidationStatus.INVALID, None, DIGEST_A),
    ],
)
def test_artifact_record_rejects_status_size_hash_disagreements(
    record_status, size, digest
):
    with pytest.raises(RuntimeStatusError):
        ArtifactFinalRecord(
            "video/onboard.mp4",
            record_status,
            "detail",
            size,
            digest,
            {"codec": "h264"},
        )


@pytest.mark.parametrize(
    ("record_status", "size", "digest"),
    [
        ("valid", None, None),
        ("valid", 10, None),
        ("valid", None, DIGEST_A),
        ("missing", 10, None),
        ("missing", None, DIGEST_A),
        ("invalid", 10, None),
        ("invalid", None, DIGEST_A),
    ],
)
def test_artifact_parser_rejects_status_size_hash_disagreements(
    record_status, size, digest
):
    document = deepcopy(CASES[12][2])
    document["records"][0].update(
        status=record_status, size_bytes=size, sha256=digest
    )
    document["complete"] = record_status == "valid"
    with pytest.raises(RuntimeStatusError):
        parse_status(ArtifactsFinalStatus, document, expected_run_id=RUN_ID)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("status", "valid"),
        ("detail", ""),
        ("semantic", {}),
        ("size_bytes", -1),
        ("size_bytes", True),
        ("sha256", "A" * 64),
        ("sha256", "a" * 63),
        ("sha256", "g" * 64),
        ("sha256", 7),
    ],
)
def test_artifact_record_constructor_rejects_invalid_fields(field, value):
    values = {
        "relative_path": "video/onboard.mp4",
        "status": ValidationStatus.VALID,
        "detail": "valid video",
        "size_bytes": 10,
        "sha256": DIGEST_A,
        "semantic": {"codec": "h264"},
    }
    values[field] = value
    with pytest.raises(RuntimeStatusError):
        ArtifactFinalRecord(**values)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("status", "unknown"),
        ("detail", ""),
        ("semantic", {}),
        ("size_bytes", -1),
        ("size_bytes", True),
        ("sha256", "A" * 64),
        ("sha256", "a" * 63),
        ("sha256", "g" * 64),
        ("sha256", 7),
    ],
)
def test_artifact_record_parser_rejects_invalid_fields(field, value):
    document = deepcopy(CASES[12][2])
    document["records"][0][field] = value
    with pytest.raises(RuntimeStatusError):
        parse_status(ArtifactsFinalStatus, document, expected_run_id=RUN_ID)


def test_artifact_status_constructor_requires_exact_tuple_and_records():
    with pytest.raises(RuntimeStatusError):
        ArtifactsFinalStatus(RUN_ID, list(FINAL_RECORDS))
    with pytest.raises(RuntimeStatusError):
        ArtifactsFinalStatus(RUN_ID, (object(), *FINAL_RECORDS[1:]))


@pytest.mark.parametrize("mutation", ["fewer", "more", "reorder", "wrong_path"])
def test_artifact_status_rejects_wrong_record_inventory(mutation):
    records = list(FINAL_RECORDS)
    if mutation == "fewer":
        records.pop()
    elif mutation == "more":
        records.append(FINAL_RECORDS[-1])
    elif mutation == "reorder":
        records[0], records[1] = records[1], records[0]
    else:
        records[0] = ArtifactFinalRecord(
            "wrong", ValidationStatus.VALID, "detail", 10, DIGEST_A, {"x": 1}
        )
    with pytest.raises(RuntimeStatusError):
        ArtifactsFinalStatus(RUN_ID, tuple(records))


@pytest.mark.parametrize(
    "mutation", ["fewer", "more", "reorder", "wrong_path", "non_dict", "missing", "extra"]
)
def test_artifact_parser_rejects_wrong_record_schema_and_inventory(mutation):
    document = deepcopy(CASES[12][2])
    records = document["records"]
    if mutation == "fewer":
        records.pop()
    elif mutation == "more":
        records.append(deepcopy(records[-1]))
    elif mutation == "reorder":
        records[0], records[1] = records[1], records[0]
    elif mutation == "wrong_path":
        records[0]["relative_path"] = "wrong"
    elif mutation == "non_dict":
        records[0] = UserDict(records[0])
    elif mutation == "missing":
        records[0].pop("detail")
    else:
        records[0]["extra"] = 1
    with pytest.raises(RuntimeStatusError):
        parse_status(ArtifactsFinalStatus, document, expected_run_id=RUN_ID)


@pytest.mark.parametrize(("complete", "record_status"), [(None, None), (1, None), (False, None), (True, "missing")])
def test_artifact_complete_requires_exact_computed_boolean(complete, record_status):
    document = deepcopy(CASES[12][2])
    if complete is None:
        del document["complete"]
    else:
        document["complete"] = complete
    if record_status is not None:
        document["records"][0].update(
            status=record_status, size_bytes=None, sha256=None
        )
    with pytest.raises(RuntimeStatusError):
        parse_status(ArtifactsFinalStatus, document, expected_run_id=RUN_ID)


@pytest.mark.parametrize(
    "semantic",
    [
        {"none": None, "bool": True, "integer": 1, "float": 1.5, "text": "x"},
        {"mapping": UserDict({"nested": (1, [2, 3])})},
    ],
)
def test_semantic_accepts_json_values_and_converts_containers(semantic):
    status = ArtifactsFinalStatus(RUN_ID, _make_records(first_semantic=semantic))
    document = status_document(status)
    assert type(document["records"][0]["semantic"]) is dict
    if "mapping" in semantic:
        assert document["records"][0]["semantic"] == {
            "mapping": {"nested": [1, [2, 3]]}
        }


@pytest.mark.parametrize(
    "semantic",
    [
        {1: "value"},
        {"bad": {1, 2}},
        {"bad": object()},
        {"bad": float("nan")},
        {"bad": float("inf")},
        {"bad": float("-inf")},
        {"bad": ValidationStatus.VALID},
    ],
)
def test_semantic_rejects_non_json_values(semantic):
    with pytest.raises(RuntimeStatusError):
        ArtifactFinalRecord(
            "video/onboard.mp4",
            ValidationStatus.VALID,
            "detail",
            10,
            DIGEST_A,
            semantic,
        )


@pytest.mark.parametrize("container_type", [dict, list])
def test_semantic_rejects_recursive_containers(container_type):
    if container_type is dict:
        recursive = {}
        recursive["self"] = recursive
        semantic = recursive
    else:
        recursive = []
        recursive.append(recursive)
        semantic = {"recursive": recursive}
    with pytest.raises(RuntimeStatusError):
        ArtifactFinalRecord(
            "video/onboard.mp4",
            ValidationStatus.VALID,
            "detail",
            10,
            DIGEST_A,
            semantic,
        )


def test_semantic_translates_recursion_depth_error():
    semantic = {}
    cursor = semantic
    for _ in range(2_000):
        nested = {}
        cursor["nested"] = nested
        cursor = nested
    with pytest.raises(RuntimeStatusError):
        ArtifactFinalRecord(
            "video/onboard.mp4",
            ValidationStatus.VALID,
            "detail",
            10,
            DIGEST_A,
            semantic,
        )


def test_artifact_document_is_fresh_and_retained_mutation_fails_closed():
    semantic = {"nested": {"frames": [1, 2]}}
    status = ArtifactsFinalStatus(RUN_ID, _make_records(first_semantic=semantic))
    first = status_document(status)
    first["records"][0]["semantic"]["nested"]["frames"].append(3)
    assert status_document(status)["records"][0]["semantic"] == {
        "nested": {"frames": [1, 2]}
    }

    semantic.clear()
    with pytest.raises(RuntimeStatusError):
        status_document(status)


@dataclass(frozen=True)
class UnregisteredStatus(RuntimeStatus):
    name: ClassVar[str] = "../escape"


def test_unregistered_status_subclass_is_rejected():
    value = UnregisteredStatus(RUN_ID)
    with pytest.raises(RuntimeStatusError):
        status_name(UnregisteredStatus)
    with pytest.raises(RuntimeStatusError):
        status_write_policy(UnregisteredStatus)
    with pytest.raises(RuntimeStatusError):
        parse_status(UnregisteredStatus, {"run_id": RUN_ID}, expected_run_id=RUN_ID)
    with pytest.raises(RuntimeStatusError):
        status_document(value)
