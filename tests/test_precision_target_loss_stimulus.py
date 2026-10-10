from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from uuid import UUID

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))

from artifacts.operator_protocol import OperatorProtocol
from artifacts.qgc_operator import (
    MAV_RESULT_ACCEPTED,
    MavlinkIdentity,
    QGC_START_COMMAND,
    QgcCommandEnvelope,
    QgcCommandResponse,
    QgcCommandStatus,
)
from orchestration.config import resolve_run_config, write_resolved_config
from orchestration.qgc_actions import QgcActionStore
from orchestration.status_store import OperatorStatus, StatusStore
from scripts.induce_precision_target_loss import (
    StimulusError,
    induce_precision_target_loss,
    parse_dataflash_mode_csv,
    parse_dataflash_precision_csv,
    parse_gz_odometry,
    parse_gz_range,
)


ODOMETRY_AT_35 = """header {
  stamp { sec: 94 nsec: 250000000 }
  data { key: "frame_id" value: "moving_pad_stationary" }
}
pose {
  position { x: 35 }
  orientation { w: 1 }
}
twist {
  linear { }
  angular { }
}
"""

RANGE_AT_FIVE_METERS = """header {
  stamp { sec: 94 nsec: 300000000 }
  data { key: "frame_id" value: "iris/rangefinder" }
}
frame: "iris/rangefinder"
angle_min: -0.01
angle_max: 0.01
range_min: 0.20
range_max: 30
ranges: 4.82
intensities: 1
"""

PL_CSV = """timestamp,TimeUS,Heal,TAcq,pX,pY,vX,vY,mX,mY,mZ,LastMeasMS,E
1760148000.10000000,94200000,1,0,0,0,0,0,0,0,0,0,0
1760148001.15000000,95150000,1,1,0.1,-0.1,0,0,0,0,4.8,95120,0
"""

MODE_CSV = """timestamp,TimeUS,Mode,ModeNum,Rsn
1760147999.00000000,93000000,4,4,2
1760148001.00000000,95100000,9,9,3
"""

RUN_ID = "00000000-0000-4000-8000-000000000717"
ROOT = Path(__file__).parents[1]
STATIONARY_TEMPLATE = ROOT / "tests/fixtures/configured-stationary-pad-run.json"


def _companion_event() -> dict[str, object]:
    return {
        "run_id": RUN_ID,
        "module": "companion",
        "severity": "INFO",
        "event": "operation_started",
        "sim_timestamp": 4.5,
        "wall_timestamp": "2026-10-10T22:00:00Z",
        "fields": {
            "operation_id": "5",
            "tool": "precision_land",
            "args": {
                "marker_id": 7,
                "settle_by_sim_s": 45.0,
                "acquire_by_sim_s": 60.0,
            },
            "timeout_sim_s": 90.0,
        },
    }


def _odometry(x: float, nanoseconds: int) -> str:
    seconds, remainder = divmod(nanoseconds, 1_000_000_000)
    return f"""header {{ stamp {{ sec: {seconds} nsec: {remainder} }} }}
pose {{
  position {{ x: {x} }}
  orientation {{ w: 1 }}
}}
twist {{ linear {{ }} angular {{ }} }}
"""


@dataclass
class RunFixture:
    directory: Path
    config_sha256: str
    token: int
    admission_sha256: str
    container_id: str
    companion_container_id: str
    project: str


def _prepare_run(tmp_path: Path) -> RunFixture:
    output_root = tmp_path / "runs"
    run_directory = output_root / RUN_ID
    material_store = QgcActionStore(
        tmp_path / "qgc-state", MavlinkIdentity(255, 190)
    )
    resolved = resolve_run_config(
        STATIONARY_TEMPLATE,
        run_id_factory=lambda: UUID(RUN_ID),
        launch_mode="operator",
        output_root=output_root,
        operator_control_factory=material_store.allocate,
    )
    write_resolved_config(run_directory, resolved)
    for relative in (".control", ".status", "logs/docker", "ardupilot_sitl/logs"):
        (run_directory / relative).mkdir(parents=True, exist_ok=True)
    assert resolved.operator_control is not None
    with OperatorProtocol(run_directory, RUN_ID, resolved.config_sha256) as protocol:
        request = protocol.write_start_request(
            resolved.operator_control.start_request_id
        )
        admission = protocol.write_start_admission(
            request, native_epoch_ns=90_000_000_000
        )
        envelope = QgcCommandEnvelope(
            2,
            resolved.operator_control.qgc_source,
            resolved.operator_control.companion,
            QGC_START_COMMAND,
            (resolved.operator_control.attempt_token, 0, 0, 0, 0, 0, 0),
            1_760_148_000_000_000_000,
        )
        protocol.write_qgc_command_status(
            QgcCommandStatus(
                RUN_ID,
                resolved.config_sha256,
                0,
                envelope,
                request.request_id,
                request.sha256,
                admission.sha256,
                1_760_148_000_100_000_000,
                QgcCommandResponse(
                    QGC_START_COMMAND,
                    resolved.operator_control.qgc_source,
                    MAV_RESULT_ACCEPTED,
                    100,
                    "Start admitted",
                ),
            )
        )
    StatusStore(output_root).write_operator_status(
        OperatorStatus(RUN_ID, "RUNNING", "", None)
    )
    (run_directory / "ardupilot_sitl/logs/00000001.BIN").write_bytes(b"live")
    project = "drone-sim-" + RUN_ID.replace("-", "")
    return RunFixture(
        run_directory,
        resolved.config_sha256,
        resolved.operator_control.attempt_token,
        admission.sha256,
        "0123456789ab",
        "abcdef012345",
        project,
    )


class FakeRunner:
    def __init__(self, run: RunFixture) -> None:
        self.run = run
        self.calls: list[tuple[tuple[str, ...], dict[str, object]]] = []
        self.odometry = [
            _odometry(35.0, 95_250_000_000),
            _odometry(55.0, 95_300_000_000),
            _odometry(55.0, 95_450_000_000),
            _odometry(55.0, 95_600_000_000),
        ]
        self.range_output = RANGE_AT_FIVE_METERS.replace(
            "sec: 94 nsec: 300000000", "sec: 95 nsec: 250000000"
        )
        self.service_output = "data: true\n"
        self.service_returncode = 0
        self.precision_csv = PL_CSV.replace("94200000", "94800000").replace(
            "95150000", "95150000"
        )
        self.mode_csv = MODE_CSV.replace("93000000", "94000000")
        self.companion_log = json.dumps(_companion_event()) + "\n"

    def _inspect(self, container_id: str) -> str:
        config_path = self.run.directory / "configuration/run.json"
        service = (
            "gazebo-runtime"
            if container_id == self.run.container_id
            else "companion-runtime"
        )
        return json.dumps(
            [
                {
                    "Id": container_id,
                    "Name": f"/{self.run.project}-{service}-1",
                    "State": {"Running": True},
                    "Config": {
                        "Image": f"drone-sim-{service}:phase3",
                        "Env": [
                            f"SIM_RUN_ID={RUN_ID}",
                            f"SIM_RUN_DIRECTORY={self.run.directory}",
                            f"SIM_CONFIG_PATH={config_path}",
                        ],
                        "Labels": {
                            "com.docker.compose.project": self.run.project,
                            "com.docker.compose.service": service,
                            "com.docker.compose.oneoff": "False",
                        },
                    },
                    "Mounts": [
                        {
                            "Source": str(self.run.directory),
                            "Destination": str(self.run.directory),
                            "RW": True,
                        },
                        {
                            "Source": str(config_path),
                            "Destination": str(config_path),
                            "RW": False,
                        },
                    ],
                }
            ]
        )

    def __call__(self, argv: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        command = tuple(str(value) for value in argv)
        self.calls.append((command, kwargs))
        if command[:2] == ("docker", "ps"):
            service_filter = next(
                item for item in command if item.startswith("label=com.docker.compose.service=")
            )
            service = service_filter.rsplit("=", 1)[-1]
            container_id = (
                self.run.container_id
                if service == "gazebo-runtime"
                else self.run.companion_container_id
            )
            output = json.dumps(
                {
                    "ID": container_id,
                    "Names": f"{self.run.project}-{service}-1",
                }
            ) + "\n"
        elif command[:2] == ("docker", "inspect"):
            output = self._inspect(command[-1])
        elif command[:3] == ("docker", "logs", "--tail"):
            assert command[-1] == self.run.companion_container_id
            output = self.companion_log
        elif "mavlogdump.py" in command[0]:
            output = self.precision_csv if "PL" in command else self.mode_csv
        elif command[-3:] == ("gz", "service", "-l"):
            output = "/world/moving_pad_stationary/set_pose\n"
        elif "topic" in command and "-t" in command:
            topic = command[command.index("-t") + 1]
            if topic == "/gazebo/private/range/downward":
                output = self.range_output
            elif topic == "/gazebo/private/moving_pad/odometry":
                output = self.odometry.pop(0)
            else:
                raise AssertionError(f"unexpected topic: {topic}")
        elif "--reqtype" in command:
            return subprocess.CompletedProcess(
                argv,
                self.service_returncode,
                self.service_output,
                "" if self.service_returncode == 0 else "request rejected",
            )
        else:
            raise AssertionError(f"unexpected command: {command}")
        return subprocess.CompletedProcess(argv, 0, output, "")


def test_parses_real_gazebo_odometry_text_with_omitted_zero_scalars() -> None:
    sample = parse_gz_odometry(ODOMETRY_AT_35)

    assert sample.native_timestamp_ns == 94_250_000_000
    assert sample.position == (35.0, 0.0, 0.0)
    assert sample.orientation == (0.0, 0.0, 0.0, 1.0)


def test_parses_real_gazebo_laserscan_text() -> None:
    sample = parse_gz_range(RANGE_AT_FIVE_METERS)

    assert sample.native_timestamp_ns == 94_300_000_000
    assert sample.distance_m == 4.82


@pytest.mark.parametrize(
    "payload",
    (
        "header { stamp { sec: 1 } } pose { orientation { w: 1 } }",
        "header { stamp { sec: 1 } } pose { position { x: nan } orientation { w: 1 } }",
    ),
)
def test_odometry_parser_rejects_missing_or_nonfinite_position(payload: str) -> None:
    with pytest.raises(StimulusError, match="odometry"):
        parse_gz_odometry(payload)


def test_dataflash_parsers_retain_native_rows_from_real_csv_shape() -> None:
    precision = parse_dataflash_precision_csv(PL_CSV)
    modes = parse_dataflash_mode_csv(MODE_CSV)

    assert [(row.time_us, row.acquired, row.last_measurement_ms) for row in precision] == [
        (94_200_000, False, 0),
        (95_150_000, True, 95_120),
    ]
    assert [(row.time_us, row.mode) for row in modes] == [
        (93_000_000, 4),
        (95_100_000, 9),
    ]


def test_dataflash_parser_rejects_a_partial_live_row_instead_of_guessing() -> None:
    with pytest.raises(StimulusError, match="DataFlash PL"):
        parse_dataflash_precision_csv(
            "timestamp,TimeUS,Heal,TAcq,LastMeasMS\n1760148001,95150000,1,,95120\n"
        )


def test_success_moves_only_the_bound_pad_and_records_persistent_physical_evidence(
    tmp_path: Path,
) -> None:
    run = _prepare_run(tmp_path)
    runner = FakeRunner(run)

    result = induce_precision_target_loss(
        run.directory,
        runner=runner,
        wall_time_ns=lambda: 1_760_148_002_000_000_000,
    )

    record_path = run.directory / "logs/docker/precision-loss-stimulus.jsonl"
    assert result["status"] == "succeeded"
    assert record_path.read_text(encoding="utf-8").endswith("\n")
    assert json.loads(record_path.read_text(encoding="utf-8")) == result
    assert result["run_id"] == RUN_ID
    assert result["config_sha256"] == run.config_sha256
    assert result["world"] == "moving_pad_stationary"
    assert result["entity"] == "moving_pad"
    assert result["partition"] == "drone_sim_" + RUN_ID.replace("-", "_")
    assert result["service"]["name"] == "/world/moving_pad_stationary/set_pose"
    assert result["service"]["request_type"] == "gz.msgs.Pose"
    assert result["service"]["response_type"] == "gz.msgs.Boolean"
    request_text = result["service"]["request_text"]
    assert bytes.fromhex(result["service"]["request_utf8_hex"]).decode() == request_text
    assert 'name: "moving_pad"' in request_text
    assert "x: 55" in request_text
    assert result["service"]["returncode"] == 0
    assert result["service"]["response"] == "data: true\n"
    assert result["pre_command"]["range"]["distance_m"] == 4.82
    assert result["pre_command"]["pad_odometry"]["position"] == [35.0, 0.0, 0.0]
    assert result["pre_command"]["mission"]["source_container_id"] == run.companion_container_id
    assert result["post_command"]["displacement_m"] == pytest.approx(20.0)
    assert len(result["post_command"]["persistence"]) == 2
    assert all(row["position"] == [55.0, 0.0, 0.0]
               for row in result["post_command"]["persistence"])

    service_call = next(
        command for command, _kwargs in runner.calls if "--reqtype" in command
    )
    assert service_call[:5] == (
        "docker",
        "exec",
        "--env",
        "GZ_PARTITION=drone_sim_00000000_0000_4000_8000_000000000717",
        run.container_id,
    )
    assert service_call[5:9] == (
        "gz",
        "service",
        "-s",
        "/world/moving_pad_stationary/set_pose",
    )
    assert "bash" not in service_call
    assert all(kwargs["shell"] is False for _command, kwargs in runner.calls)
    assert all(float(kwargs["timeout"]) <= 15.0 for _command, kwargs in runner.calls)
    first_dataflash = next(
        index
        for index, (command, _kwargs) in enumerate(runner.calls)
        if "mavlogdump.py" in command[0]
    )
    first_native_sample = next(
        index
        for index, (command, _kwargs) in enumerate(runner.calls)
        if "topic" in command
    )
    assert first_dataflash < first_native_sample
    assert not (run.directory / "logs/companion.jsonl").exists()
    assert any(command[:3] == ("docker", "logs", "--tail") for command, _ in runner.calls)


@pytest.mark.parametrize(
    ("mutation", "detail"),
    (
        ("wrong_world", "moving_pad_stationary"),
        ("wrong_run_binding", "run directory"),
        ("missing_admission", "admission"),
        ("finalizing", "RUNNING"),
        ("finalize_requested", "finalization"),
        ("missing_acquisition", "TAcq=1"),
        ("stale_acquisition", "stale"),
        ("stale_measurement", "LastMeasMS"),
        ("stale_land", "stale"),
        ("no_precision_operation", "precision_land"),
        ("foreign_container", "owned"),
        ("wrong_pre_pose", "x=35"),
        ("low_range", "0.75"),
        ("missing_range", "range"),
    ),
)
def test_rejects_unsafe_or_unbound_inputs_before_set_pose(
    tmp_path: Path, mutation: str, detail: str
) -> None:
    run = _prepare_run(tmp_path)
    runner = FakeRunner(run)
    config_path = run.directory / "configuration/run.json"
    if mutation == "wrong_world":
        document = json.loads(config_path.read_text(encoding="utf-8"))
        document["world"] = "moving_pad_landing"
        without_checksum = {
            key: value for key, value in document.items() if key != "config_sha256"
        }
        document["config_sha256"] = hashlib.sha256(
            json.dumps(without_checksum, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        config_path.write_text(json.dumps(document), encoding="utf-8")
    elif mutation == "wrong_run_binding":
        renamed = run.directory.parent / "00000000-0000-4000-8000-000000000718"
        run.directory.rename(renamed)
        run.directory = renamed
    elif mutation == "missing_admission":
        (run.directory / "configuration/start-admission.json").unlink()
    elif mutation == "finalizing":
        StatusStore(run.directory.parent).write_operator_status(
            OperatorStatus(RUN_ID, "FINALIZING", "stop", None)
        )
    elif mutation == "finalize_requested":
        (run.directory / ".control/finalize-request.json").write_text("{}\n")
    elif mutation == "missing_acquisition":
        runner.precision_csv = runner.precision_csv.replace(",1,0.1", ",0,0.1")
    elif mutation == "stale_acquisition":
        runner.precision_csv = (
            PL_CSV.splitlines()[0]
            + "\n1760148001.15000000,94600000,1,1,0.1,-0.1,0,0,0,0,4.8,94590,0\n"
        )
    elif mutation == "stale_measurement":
        runner.precision_csv = runner.precision_csv.replace("95120,0", "94000,0")
    elif mutation == "stale_land":
        event = _companion_event()
        event["sim_timestamp"] = 0.0
        runner.companion_log = json.dumps(event) + "\n"
        runner.mode_csv = (
            MODE_CSV.splitlines()[0]
            + "\n1760148001.00000000,90100000,9,9,3\n"
        )
    elif mutation == "no_precision_operation":
        runner.companion_log = ""
    elif mutation == "foreign_container":
        original_inspect = runner._inspect

        def foreign_inspect(container_id: str) -> str:
            document = json.loads(original_inspect(container_id))
            document[0]["Config"]["Env"][0] = (
                "SIM_RUN_ID=00000000-0000-4000-8000-000000000999"
            )
            return json.dumps(document)

        runner._inspect = foreign_inspect  # type: ignore[method-assign]
    elif mutation == "wrong_pre_pose":
        runner.odometry[0] = _odometry(36.0, 95_250_000_000)
    elif mutation == "low_range":
        runner.range_output = runner.range_output.replace("ranges: 4.82", "ranges: 0.75")
    elif mutation == "missing_range":
        runner.range_output = runner.range_output.replace("ranges: 4.82", "")

    with pytest.raises(StimulusError, match=detail):
        induce_precision_target_loss(run.directory, runner=runner)

    assert not any("--reqtype" in command for command, _kwargs in runner.calls)


def test_rejected_service_is_recorded_and_never_accepted_from_return_code_alone(
    tmp_path: Path,
) -> None:
    run = _prepare_run(tmp_path)
    runner = FakeRunner(run)
    runner.service_output = "data: false\n"

    with pytest.raises(StimulusError, match="rejected"):
        induce_precision_target_loss(run.directory, runner=runner)

    record = json.loads(
        (run.directory / "logs/docker/precision-loss-stimulus.jsonl").read_text()
    )
    assert record["status"] == "failed"
    assert record["service"]["returncode"] == 0
    assert record["service"]["response"] == "data: false\n"
    assert "rejected" in record["detail"]


@pytest.mark.parametrize(
    ("positions", "detail"),
    (
        ((35.0, 35.0, 35.0), "did not move"),
        ((55.0, 35.0, 35.0), "did not persist"),
    ),
)
def test_true_service_response_requires_persistent_twenty_meter_displacement(
    tmp_path: Path, positions: tuple[float, ...], detail: str
) -> None:
    run = _prepare_run(tmp_path)
    runner = FakeRunner(run)
    runner.odometry = [
        _odometry(35.0, 95_250_000_000),
        *(
            _odometry(position, timestamp)
            for position, timestamp in zip(
                positions,
                (95_300_000_000, 95_450_000_000, 95_600_000_000),
                strict=True,
            )
        ),
    ]

    with pytest.raises(StimulusError, match=detail):
        induce_precision_target_loss(run.directory, runner=runner)

    record = json.loads(
        (run.directory / "logs/docker/precision-loss-stimulus.jsonl").read_text()
    )
    assert record["status"] == "failed"
    assert record["service"]["response"] == "data: true\n"


def test_existing_stimulus_record_blocks_every_external_command(tmp_path: Path) -> None:
    run = _prepare_run(tmp_path)
    record = run.directory / "logs/docker/precision-loss-stimulus.jsonl"
    record.write_text('{"status":"failed"}\n', encoding="utf-8")
    runner = FakeRunner(run)

    with pytest.raises(FileExistsError, match="precision-loss-stimulus"):
        induce_precision_target_loss(run.directory, runner=runner)

    assert runner.calls == []
    assert record.read_text(encoding="utf-8") == '{"status":"failed"}\n'
