#!/usr/bin/env python3
"""Write a concise Markdown handoff note from an accepted competition bundle."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def _document(path: Path) -> dict[str, object]:
    try:
        document = json.loads(path.read_bytes())
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"could not read accepted evidence: {path}") from error
    if not isinstance(document, dict):
        raise ValueError(f"accepted evidence must be an object: {path}")
    return document


def _score_text(value: float) -> str:
    return str(int(value)) if value.is_integer() else str(value)


def write_competition_verification(
    run_directory: Path | str, output: Path | str
) -> Path:
    """Read an already accepted bundle and write its evidence pointers only."""
    directory = Path(run_directory).resolve()
    manifest = _document(directory / "manifest.json")
    result = _document(directory / "scoring/result.json")
    scoring = manifest.get("scoring")
    source_rows = manifest.get("source_revisions")
    rule_rows = result.get("rule_results")
    accepted = (
        manifest.get("terminal_status") == "COMPLETED"
        and isinstance(scoring, dict)
        and scoring.get("achieved_score") == 150.0
        and scoring.get("maximum_available_score") == 150.0
        and result.get("ruleset_id") == "competition_v1"
        and result.get("complete") is True
        and result.get("achieved_score") == 150.0
        and result.get("maximum_available_score") == 150.0
        and result.get("scoring_checksum") == scoring.get("scoring_checksum")
        and isinstance(source_rows, list)
        and [row.get("name") for row in source_rows if isinstance(row, dict)]
        == ["drone_sim", "comp2026"]
        and isinstance(rule_rows, list)
        and len(rule_rows) == 7
    )
    if not accepted:
        raise ValueError("verification note requires an accepted 150/150 manifest")
    try:
        awarded = [float(row["awarded_points"]) for row in rule_rows]
        checkpoints = (sum(awarded[:4]), sum(awarded[:6]), sum(awarded))
        timing = manifest["simulation_timing"]
        elapsed_ns = int(timing["duration_ns"])
        run_id = str(manifest["run_id"])
        checksum = str(scoring["scoring_checksum"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("verification note requires an accepted 150/150 manifest") from error
    if checkpoints != (80.0, 145.0, 150.0) or elapsed_ns < 0:
        raise ValueError("verification note requires an accepted 150/150 manifest")
    elapsed = elapsed_ns / 1_000_000_000
    elapsed_text = f"{elapsed:g} s"
    revisions = {
        row["name"]: (row["revision"], row["dirty"])
        for row in source_rows
        if isinstance(row, dict)
    }
    note = f"""# comp2026 MVP verification

- Run ID: `{run_id}`
- Terminal state: `COMPLETED`
- Elapsed simulated time: `{elapsed_text}`
- Score: `150/150`
- Rules checksum: `{checksum}`
- Parent revision: `{revisions['drone_sim'][0]}` (dirty: `{str(revisions['drone_sim'][1]).lower()}`)
- Nested comp2026 revision: `{revisions['comp2026'][0]}` (dirty: `{str(revisions['comp2026'][1]).lower()}`)
- Checkpoints: `{_score_text(checkpoints[0])}/150`, `{_score_text(checkpoints[1])}/150`, `{_score_text(checkpoints[2])}/150`
- Manifest: `{directory / 'manifest.json'}`
- Onboard video: `{directory / 'video/onboard.mp4'}`
- Observer video: `{directory / 'video/observer.mp4'}`
- Rosbag: `{directory / 'rosbag'}`
"""
    target = Path(output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(note, encoding="utf-8")
    return target


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("output", type=Path)
    arguments = parser.parse_args()
    try:
        write_competition_verification(arguments.run_directory, arguments.output)
    except ValueError as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
