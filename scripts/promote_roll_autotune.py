#!/usr/bin/env python3
"""Promote one run's saved roll AutoTune gains into the versioned overlay."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import stat
import tempfile
from uuid import UUID


ROLL_GAIN_PARAMETERS = (
    "ATC_ANG_RLL_P",
    "ATC_RAT_RLL_P",
    "ATC_RAT_RLL_I",
    "ATC_RAT_RLL_D",
    "ATC_ACC_R_MAX",
)


def read_saved_gains(path: Path) -> tuple[str, dict[str, float]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    if len(lines) != 7 or lines[0] != "# Roll gains saved by ArduPilot AutoTune":
        raise ValueError("AutoTune artifact has an unexpected format")
    prefix = "# run_id "
    if not lines[1].startswith(prefix):
        raise ValueError("AutoTune artifact is missing its run ID")
    run_id = lines[1][len(prefix) :]
    try:
        parsed_run_id = UUID(run_id)
    except ValueError as error:
        raise ValueError("AutoTune artifact run ID is invalid") from error
    if str(parsed_run_id) != run_id:
        raise ValueError("AutoTune artifact run ID is not canonical")

    values: dict[str, float] = {}
    for line in lines[2:]:
        parts = line.split()
        if len(parts) != 2:
            raise ValueError("AutoTune artifact contains an invalid parameter line")
        name, raw_value = parts
        if name not in ROLL_GAIN_PARAMETERS or name in values:
            raise ValueError(f"AutoTune artifact contains unexpected parameter {name}")
        value = float(raw_value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"AutoTune parameter {name} must be finite and positive")
        values[name] = value
    if tuple(values) != ROLL_GAIN_PARAMETERS:
        raise ValueError("AutoTune artifact parameters are missing or out of order")
    if not math.isclose(
        values["ATC_RAT_RLL_I"],
        values["ATC_RAT_RLL_P"],
        rel_tol=1e-9,
        abs_tol=1e-12,
    ):
        raise ValueError("saved roll I gain must equal the roll P gain")
    return run_id, values


def promote(parameter_file: Path, values: dict[str, float]) -> None:
    original = parameter_file.read_text(encoding="utf-8")
    remaining = set(ROLL_GAIN_PARAMETERS)
    output: list[str] = []
    for line in original.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] in remaining:
            name = parts[0]
            output.append(f"{name} {values[name]:g}")
            remaining.remove(name)
        elif len(parts) == 2 and parts[0] in ROLL_GAIN_PARAMETERS:
            raise ValueError(f"destination contains duplicate parameter {parts[0]}")
        else:
            output.append(line)
    if remaining:
        output.extend(f"{name} {values[name]:g}" for name in ROLL_GAIN_PARAMETERS if name in remaining)
    rendered = "\n".join(output) + "\n"

    destination_mode = stat.S_IMODE(parameter_file.stat().st_mode)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=parameter_file.parent,
            prefix=f".{parameter_file.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(destination_mode)
        os.replace(temporary, parameter_file)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument(
        "--parameter-file",
        type=Path,
        default=Path(__file__).parents[1] / "ardupilot_sitl/params/descent.parm",
    )
    args = parser.parse_args()
    artifact = args.run_directory / "ardupilot_sitl/autotune-roll.parm"
    try:
        run_id, values = read_saved_gains(artifact)
        promote(args.parameter_file, values)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "run_id": run_id,
                "artifact": str(artifact),
                "parameter_file": str(args.parameter_file),
                "promoted": values,
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
