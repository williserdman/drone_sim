from __future__ import annotations

from pathlib import Path
import hashlib
import json
from uuid import UUID

import pytest

from artifacts.calibration import (
    CALIBRATION_PARAMETERS,
    PRESERVED_PARAMETERS,
    read_calibration_baseline,
    read_calibration_parameters,
    read_saved_calibration_parameters,
    validate_calibration_parameters,
    validate_calibration_artifact,
    write_calibration_parameters,
)


RUN_ID = "12345678-1234-5678-9234-56781234abcd"


def gains() -> dict[str, float]:
    return {
        "ATC_ANG_RLL_P": 8.0,
        "ATC_RAT_RLL_P": 0.08,
        "ATC_RAT_RLL_I": 0.08,
        "ATC_RAT_RLL_D": 0.004,
        "ATC_ACC_R_MAX": 1000.0,
        "ATC_ANG_PIT_P": 8.5,
        "ATC_RAT_PIT_P": 0.09,
        "ATC_RAT_PIT_I": 0.09,
        "ATC_RAT_PIT_D": 0.005,
        "ATC_ACC_P_MAX": 1100.0,
        "ATC_ANG_YAW_P": 6.0,
        "ATC_RAT_YAW_P": 0.18,
        "ATC_RAT_YAW_I": 0.018,
        "ATC_RAT_YAW_FLTE": 2.5,
        "ATC_ACC_Y_MAX": 300.0,
    }


def test_round_trip_preserves_all_15_values(tmp_path: Path) -> None:
    target = write_calibration_parameters(tmp_path, RUN_ID, gains())

    assert target == tmp_path / "ardupilot_sitl/autotune.parm"
    assert read_calibration_parameters(target) == (RUN_ID, gains())
    assert target.read_text(encoding="utf-8").splitlines() == [
        "# drone_sim AutoTune calibration v1",
        f"# run_id {RUN_ID}",
        "# autotune_axes 7",
        *(f"{name} {gains()[name]:g}" for name in CALIBRATION_PARAMETERS),
    ]


@pytest.mark.parametrize(
    "change",
    [
        lambda values: values.pop("ATC_ANG_PIT_P"),
        lambda values: values.__setitem__("ATC_RAT_RLL_I", 0.07),
        lambda values: values.__setitem__("ATC_RAT_PIT_I", 0.08),
        lambda values: values.__setitem__("ATC_RAT_YAW_I", 0.18),
        lambda values: values.__setitem__("EXTRA", 1.0),
    ],
)
def test_rejects_partial_axes(change) -> None:
    values = gains()
    change(values)
    with pytest.raises(ValueError):
        validate_calibration_parameters(values)


class Message:
    def __init__(self, message_type: str, **fields: object) -> None:
        self.message_type = message_type
        self.fields = {"mavpackettype": message_type, **fields}

    def to_dict(self) -> dict[str, object]:
        return self.fields


class Connection:
    def __init__(self, messages: list[Message]) -> None:
        self.messages = iter(messages)

    def recv_match(self, *, blocking: bool = False):
        assert blocking is False
        return next(self.messages, None)


def parm(time_us: int, name: str, value: float) -> Message:
    return Message("PARM", TimeUS=time_us, Name=name, Value=value)


def saved_epoch(
    values: dict[str, float], *, marker_us: int = 1_000_000
) -> list[Message]:
    return [
        Message(
            "MSG",
            TimeUS=marker_us,
            Message="AutoTune: Saved gains for Roll Pitch Yaw(E)",
        ),
        Message("EV", TimeUS=marker_us, Id=11),
        Message("EV", TimeUS=marker_us, Id=37),
        *[
            parm(marker_us + 1_000 + index * 20_000, name, values[name])
            for index, name in enumerate(CALIBRATION_PARAMETERS)
        ],
    ]


def dataflash_reader(tmp_path: Path, messages: list[Message]) -> dict[str, float]:
    log = tmp_path / "ardupilot_sitl/logs/00000001.BIN"
    log.parent.mkdir(parents=True)
    log.write_bytes(b"dataflash")
    return read_saved_calibration_parameters(
        tmp_path,
        connection_factory=lambda path: (
            Connection(messages)
            if path == str(log)
            else pytest.fail("wrong DataFlash path")
        ),
    )


def test_reads_one_post_disarm_saved_epoch_with_axis_timestamp_spread(
    tmp_path: Path,
) -> None:
    values = gains()
    messages = saved_epoch(values)
    # A preserved zero yaw D is valid and is not a sixteenth tuned output.
    messages.append(parm(1_010_000, "ATC_RAT_YAW_D", 0.0))

    assert dataflash_reader(tmp_path, messages) == values


def test_rejects_mixed_save_epochs(tmp_path: Path) -> None:
    values = gains()
    current = saved_epoch(values)
    missing_name = "ATC_ACC_Y_MAX"
    current = [
        message
        for message in current
        if not (
            message.message_type == "PARM"
            and message.fields.get("Name") == missing_name
        )
    ]
    stale = parm(900_000, missing_name, values[missing_name])

    with pytest.raises(ValueError, match="incomplete"):
        dataflash_reader(tmp_path, [stale, *current])


def test_rejects_saved_text_without_disarm_and_saved_event(tmp_path: Path) -> None:
    messages = saved_epoch(gains())
    messages = [message for message in messages if message.message_type != "EV"]

    with pytest.raises(ValueError, match="evidence"):
        dataflash_reader(tmp_path, messages)


def test_rejects_duplicate_parameter_in_save_epoch(tmp_path: Path) -> None:
    messages = saved_epoch(gains())
    messages.append(parm(1_100_000, CALIBRATION_PARAMETERS[0], gains()[CALIBRATION_PARAMETERS[0]]))

    with pytest.raises(ValueError, match="duplicate"):
        dataflash_reader(tmp_path, messages)


def test_rejects_noncanonical_artifact_text(tmp_path: Path) -> None:
    target = write_calibration_parameters(tmp_path, RUN_ID, gains())
    target.write_text(target.read_text() + "ATC_RAT_YAW_D 0\n")

    with pytest.raises(ValueError):
        read_calibration_parameters(target)


def test_rejects_noncanonical_run_id(tmp_path: Path) -> None:
    upper = str(UUID(RUN_ID)).upper()
    with pytest.raises(ValueError, match="canonical UUID"):
        write_calibration_parameters(tmp_path, upper, gains())


def preserved() -> dict[str, float]:
    values = {name: 0.0 for name in PRESERVED_PARAMETERS}
    values["ATC_RATE_FF_ENAB"] = 1.0
    return values


def write_parameter_evidence(tmp_path: Path, *, post_yaw_d: float = 0.0) -> None:
    target = tmp_path / "logs/companion.jsonl"
    target.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for timestamp, stage in enumerate(("baseline", "activation", "post_disarm"), 1):
        parameters = {**gains(), **preserved()}
        if stage == "post_disarm":
            parameters["ATC_RAT_YAW_D"] = post_yaw_d
        rows.append(json.dumps({
            "run_id": RUN_ID,
            "module": "companion",
            "severity": "INFO",
            "event": "calibration_parameters_verified",
            "sim_timestamp": float(timestamp),
            "wall_timestamp": f"2026-09-29T00:00:0{timestamp}Z",
            "fields": {"stage": stage, "parameters": parameters},
        }))
    target.write_text("\n".join(rows) + "\n", encoding="utf-8")


def test_preserves_zero_baseline_yaw_d(tmp_path: Path) -> None:
    write_parameter_evidence(tmp_path)

    baseline = read_calibration_baseline(tmp_path)

    assert set(baseline) == set(PRESERVED_PARAMETERS)
    assert baseline["ATC_RAT_YAW_D"] == 0.0


def test_rejects_changed_preserved_parameter(tmp_path: Path) -> None:
    write_parameter_evidence(tmp_path, post_yaw_d=0.001)
    write_calibration_parameters(tmp_path, RUN_ID, gains())
    payload = (tmp_path / "ardupilot_sitl/autotune.parm").read_bytes()
    config = tmp_path / "configuration/run.json"
    config.parent.mkdir(parents=True)
    config.write_text(json.dumps({
        "run_id": RUN_ID, "mission": "autotune", "scenario": "calibration_v1",
    }))
    log = tmp_path / "ardupilot_sitl/logs/00000001.BIN"
    log.parent.mkdir(parents=True)
    log.write_bytes(b"dataflash")
    (tmp_path / "manifest.json").write_text(json.dumps({
        "run_id": RUN_ID,
        "terminal_status": "COMPLETED",
        "artifacts": [
            {
                "relative_path": "ardupilot_sitl/autotune.parm",
                "size_bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
                "validation": "valid",
                "detail": "valid regular file",
            },
            {
                "relative_path": "ardupilot_sitl/logs/00000001.BIN",
                "size_bytes": len(b"dataflash"),
                "sha256": hashlib.sha256(b"dataflash").hexdigest(),
                "validation": "valid",
                "detail": "valid regular file",
            },
        ],
    }))

    with pytest.raises(ValueError, match="preserved"):
        validate_calibration_artifact(
            tmp_path,
            connection_factory=lambda _path: Connection(saved_epoch(gains())),
        )
