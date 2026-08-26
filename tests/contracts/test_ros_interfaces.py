from pathlib import Path
import re

import pytest


ROOT = Path(__file__).parents[2]
MSG = ROOT / "ros_ws/src/simulation_interfaces/msg"
SRV = ROOT / "ros_ws/src/simulation_interfaces/srv"

EXPECTED = {
    "RunState.msg": [
        "string run_id",
        "builtin_interfaces/Time sim_timestamp",
        "uint8 state",
        "string reason",
        "string config_sha256",
    ],
    "FrameMetadata.msg": [
        "string run_id",
        "builtin_interfaces/Time sim_timestamp",
        "uint64 frame_id",
        "string stream",
    ],
    "GroundTruth.msg": [
        "string run_id",
        "builtin_interfaces/Time sim_timestamp",
        "string vehicle_id",
        "geometry_msgs/Pose pose",
        "geometry_msgs/Twist twist",
        "bool in_contact",
    ],
    "ScenarioEvent.msg": [
        "string run_id",
        "builtin_interfaces/Time sim_timestamp",
        "uint64 event_id",
        "string magnet_id",
        "string state",
    ],
    "ScoreEvent.msg": [
        "string run_id",
        "builtin_interfaces/Time sim_timestamp",
        "uint64 event_id",
        "string event_type",
        "float64 value",
        "string evidence_ref",
    ],
    "ArtifactStatus.msg": [
        "string run_id",
        "builtin_interfaces/Time sim_timestamp",
        "bool ready",
        "bool complete",
        "string[] missing",
        "string manifest_path",
    ],
    "PayloadState.msg": [
        "string run_id",
        "builtin_interfaces/Time sim_timestamp",
        "uint16 aruco_id",
        "geometry_msgs/Pose pose",
        "geometry_msgs/Twist twist",
        "bool grounded",
        "bool attached",
    ],
    "PayloadEvent.msg": [
        "string run_id",
        "builtin_interfaces/Time sim_timestamp",
        "uint64 event_id",
        "uint16 aruco_id",
        "string command_id",
        "string action",
        "string state",
        "string code",
    ],
    "MissionEvent.msg": [
        "string run_id",
        "builtin_interfaces/Time sim_timestamp",
        "uint64 event_id",
        "string phase",
        "string state",
        "string detail",
    ],
}


@pytest.mark.parametrize("filename,declarations", EXPECTED.items())
def test_message_contract(filename: str, declarations: list[str]) -> None:
    actual = []
    for raw_line in (MSG / filename).read_text().splitlines():
        declaration = raw_line.split("#", 1)[0].strip()
        if declaration and not re.fullmatch(r"uint8 [A-Z]+=[0-9]+", declaration):
            actual.append(declaration)

    assert actual == declarations


def test_run_state_lifecycle_constants_are_in_order() -> None:
    text = (MSG / "RunState.msg").read_text()
    expected = [
        ("CREATED", "0"),
        ("STARTING", "1"),
        ("READY", "2"),
        ("RUNNING", "3"),
        ("FINALIZING", "4"),
        ("COMPLETED", "5"),
        ("FAILED", "6"),
        ("ABORTED", "7"),
    ]
    actual = re.findall(r"^uint8 ([A-Z]+)=([0-9]+)$", text, re.MULTILINE)
    assert actual == expected


def test_payload_command_service_contract() -> None:
    request, response = (part.strip() for part in (SRV / "PayloadCommand.srv").read_text().split("---", 1))
    request_fields = [line.strip() for line in request.splitlines() if line.strip()]
    response_fields = [line.strip() for line in response.splitlines() if line.strip()]

    assert request_fields == [
        "string run_id",
        "uint16 aruco_id",
        "uint8 ATTACH=1",
        "uint8 RELEASE=2",
        "uint8 action",
        "string command_id",
    ]
    assert response_fields == [
        "bool accepted",
        "string code",
        "string detail",
        "string command_id",
        "uint64 response_sequence",
    ]
