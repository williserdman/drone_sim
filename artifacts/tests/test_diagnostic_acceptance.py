from __future__ import annotations

from pathlib import Path

import pytest

from artifacts.acceptance import BundleAcceptanceError
from artifacts.diagnostic_acceptance import (
    ROLL_GAIN_PARAMETERS,
    validate_diagnostic_landing,
    validate_diagnostic_logs,
    validate_roll_gain_artifact,
)


ROLL_GAINS = {
    "ATC_ANG_RLL_P": 4.25,
    "ATC_RAT_RLL_P": 0.041,
    "ATC_RAT_RLL_I": 0.041,
    "ATC_RAT_RLL_D": 0.0011,
    "ATC_ACC_R_MAX": 72000.0,
}


def row(event: str, **fields: object) -> dict[str, object]:
    return {"event": event, "fields": fields}


def test_accepts_terminal_hover_and_ordered_roll_autotune_logs() -> None:
    validate_diagnostic_logs(
        "hover_roll",
        [
            row("hover_phase", phase="HOVERING"),
            row("hover_phase", phase="WAIT_LAND"),
            row("hover_phase", phase="COMPLETE"),
        ],
    )
    validate_diagnostic_logs(
        "autotune_roll",
        [
            row("autotune_phase", phase="TUNING"),
            row("autotune_phase", phase="LANDING_TO_SAVE"),
            row("ardupilot_status_text", text="AutoTune: Success"),
            row("ardupilot_status_text", text="AutoTune: Saved gains for Roll"),
            row("autotune_phase", phase="COMPLETE"),
        ],
    )


@pytest.mark.parametrize(
    "mission, rows",
    [
        ("hover_roll", [row("hover_phase", phase="HOVERING")]),
        (
            "autotune_roll",
            [
                row("ardupilot_status_text", text="AutoTune: Saved gains for Roll"),
                row("ardupilot_status_text", text="AutoTune: Success"),
                row("autotune_phase", phase="TUNING"),
                row("autotune_phase", phase="LANDING_TO_SAVE"),
                row("autotune_phase", phase="COMPLETE"),
            ],
        ),
    ],
)
def test_rejects_incomplete_or_out_of_order_diagnostic_logs(
    mission: str, rows: list[dict[str, object]]
) -> None:
    with pytest.raises(BundleAcceptanceError):
        validate_diagnostic_logs(mission, rows)


def test_diagnostic_landing_ignores_touchdown_precision() -> None:
    rows = [
        {"rule_id": name, "passed": True}
        for name in ("airborne_then_contact", "safe_preimpact_speed", "stable_contact")
    ]
    rows.append({"rule_id": "touchdown_precision", "passed": False})

    validate_diagnostic_landing(rows)

    rows[1]["passed"] = False
    with pytest.raises(BundleAcceptanceError, match="safe_preimpact_speed"):
        validate_diagnostic_landing(rows)


class Message:
    def __init__(self, timestamp: int, name: str, value: float) -> None:
        self.fields = {
            "mavpackettype": "PARM",
            "TimeUS": timestamp,
            "Name": name,
            "Value": value,
        }

    def to_dict(self) -> dict[str, object]:
        return self.fields


class NativeMarker:
    def __init__(self, message_type: str, timestamp: int, **fields: object) -> None:
        self.fields = {
            "mavpackettype": message_type,
            "TimeUS": timestamp,
            **fields,
        }

    def to_dict(self) -> dict[str, object]:
        return self.fields


class Connection:
    def __init__(self, messages: list[Message | NativeMarker]) -> None:
        self.messages = iter(messages)

    def recv_match(self, **_kwargs: object) -> Message | NativeMarker | None:
        return next(self.messages, None)


def native_save(values: dict[str, float]) -> list[Message | NativeMarker]:
    marker = 1_000_000
    return [
        NativeMarker("EV", marker, Id=11),
        NativeMarker("MSG", marker, Message="AutoTune: Saved gains for Roll"),
        NativeMarker("EV", marker, Id=37),
        *[
            Message(marker + 1_000 + index * 20_000, name, value)
            for index, (name, value) in enumerate(values.items())
        ],
    ]


def write_roll_artifact(run_directory: Path, values: dict[str, float]) -> None:
    artifact = run_directory / "ardupilot_sitl/autotune-roll.parm"
    artifact.parent.mkdir(parents=True)
    artifact.write_text(
        "# Roll gains saved by ArduPilot AutoTune\n"
        "# run_id 12345678-1234-5678-9234-56781234abcd\n"
        + "".join(f"{name} {values[name]:g}\n" for name in ROLL_GAIN_PARAMETERS),
        encoding="utf-8",
    )
    log = run_directory / "ardupilot_sitl/logs/00000001.BIN"
    log.parent.mkdir(parents=True)
    log.write_bytes(b"dataflash")


def test_roll_gain_artifact_matches_one_native_save(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    write_roll_artifact(tmp_path, ROLL_GAINS)
    messages = native_save(ROLL_GAINS)
    monkeypatch.setattr(
        "artifacts.diagnostic_acceptance.mavutil.mavlink_connection",
        lambda _path: Connection(messages),
    )

    assert validate_roll_gain_artifact(tmp_path) == ROLL_GAINS


@pytest.mark.parametrize(
    "mutate, error",
    [
        (lambda values: values.__setitem__("ATC_RAT_RLL_I", 0.04), "I gain"),
        (lambda values: values.__setitem__("ATC_RAT_RLL_D", 0.0), "positive"),
    ],
)
def test_roll_gain_artifact_rejects_invalid_values(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mutate,
    error: str,
) -> None:
    values = dict(ROLL_GAINS)
    mutate(values)
    write_roll_artifact(tmp_path, values)
    messages = [Message(1_000_000, name, value) for name, value in values.items()]
    monkeypatch.setattr(
        "artifacts.diagnostic_acceptance.mavutil.mavlink_connection",
        lambda _path: Connection(messages),
    )

    with pytest.raises(BundleAcceptanceError, match=error):
        validate_roll_gain_artifact(tmp_path)


def test_roll_gain_artifact_rejects_mismatched_or_mixed_native_save(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    write_roll_artifact(tmp_path, ROLL_GAINS)
    messages = native_save(ROLL_GAINS)
    messages[-1].fields["TimeUS"] = 2_000_001
    monkeypatch.setattr(
        "artifacts.diagnostic_acceptance.mavutil.mavlink_connection",
        lambda _path: Connection(messages),
    )

    with pytest.raises(BundleAcceptanceError, match="incomplete"):
        validate_roll_gain_artifact(tmp_path)


def test_roll_gain_artifact_rejects_unanchored_parameter_snapshot(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    write_roll_artifact(tmp_path, ROLL_GAINS)
    messages = [Message(1_001_000, name, value) for name, value in ROLL_GAINS.items()]
    monkeypatch.setattr(
        "artifacts.diagnostic_acceptance.mavutil.mavlink_connection",
        lambda _path: Connection(messages),
    )

    with pytest.raises(BundleAcceptanceError, match="save evidence"):
        validate_roll_gain_artifact(tmp_path)
