from __future__ import annotations

from pathlib import Path
import subprocess
import sys


RUN_ID = "00000000-0000-4000-8000-000000000001"
SCRIPT = Path(__file__).parents[2] / "scripts/promote_roll_autotune.py"


def _artifact(run_directory: Path, extra: str = "") -> None:
    path = run_directory / "ardupilot_sitl/autotune-roll.parm"
    path.parent.mkdir(parents=True)
    path.write_text(
        "# Roll gains saved by ArduPilot AutoTune\n"
        f"# run_id {RUN_ID}\n"
        "ATC_ANG_RLL_P 4.25\n"
        "ATC_RAT_RLL_P 0.041\n"
        "ATC_RAT_RLL_I 0.041\n"
        "ATC_RAT_RLL_D 0.0011\n"
        "ATC_ACC_R_MAX 72000\n"
        f"{extra}",
        encoding="utf-8",
    )


def test_promotes_exact_saved_roll_gains_without_changing_pitch(tmp_path: Path) -> None:
    _artifact(tmp_path)
    parameters = tmp_path / "descent.parm"
    parameters.write_text(
        "ATC_RAT_RLL_P 0.0675\n"
        "ATC_RAT_RLL_I 0.0675\n"
        "ATC_RAT_RLL_D 0.0018\n"
        "ATC_RAT_PIT_P 0.0675\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(tmp_path),
            "--parameter-file",
            str(parameters),
        ],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    contents = parameters.read_text(encoding="utf-8")
    assert "ATC_ANG_RLL_P 4.25" in contents
    assert "ATC_RAT_RLL_P 0.041" in contents
    assert "ATC_RAT_RLL_I 0.041" in contents
    assert "ATC_RAT_RLL_D 0.0011" in contents
    assert "ATC_ACC_R_MAX 72000" in contents
    assert "ATC_RAT_PIT_P 0.0675" in contents
    assert all(contents.count(name) == 1 for name in (
        "ATC_ANG_RLL_P",
        "ATC_RAT_RLL_P",
        "ATC_RAT_RLL_I",
        "ATC_RAT_RLL_D",
        "ATC_ACC_R_MAX",
    ))


def test_rejects_extra_artifact_parameters_without_touching_destination(
    tmp_path: Path,
) -> None:
    _artifact(tmp_path, "ATC_RAT_PIT_P 9\n")
    parameters = tmp_path / "descent.parm"
    parameters.write_text("ATC_RAT_RLL_P 0.0675\n", encoding="utf-8")
    before = parameters.read_bytes()

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(tmp_path),
            "--parameter-file",
            str(parameters),
        ],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode != 0
    assert parameters.read_bytes() == before
