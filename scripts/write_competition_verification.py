#!/usr/bin/env python3
"""Write a concise Markdown handoff note from an accepted competition bundle."""

from __future__ import annotations

import argparse
from collections.abc import Mapping
import hashlib
import json
from pathlib import Path


def _canonical_inspection(
    run_directory: Path,
    *,
    expected_image_digests: Mapping[str, str] | None = None,
) -> Mapping[str, object]:
    from inspect_competition_run import inspect_competition_run

    return inspect_competition_run(
        run_directory,
        expected_image_digests=expected_image_digests,
    )


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
    run_directory: Path | str,
    output: Path | str,
    *,
    expected_image_digests: Mapping[str, str] | None = None,
) -> Path:
    """Pass canonical read-only acceptance, then write its evidence pointers."""
    directory = Path(run_directory).resolve()
    try:
        if expected_image_digests is None:
            inspection = _canonical_inspection(directory)
        else:
            inspection = _canonical_inspection(
                directory,
                expected_image_digests=expected_image_digests,
            )
    except Exception as error:
        raise ValueError(f"canonical competition inspection failed: {error}") from error
    expected_inspection_keys = {
        "accepted",
        "run_id",
        "achieved_score",
        "maximum_available_score",
        "compose_project",
        "manifest_sha256",
    }
    if (
        not isinstance(inspection, Mapping)
        or set(inspection) != expected_inspection_keys
        or inspection.get("accepted") is not True
        or inspection.get("achieved_score") != 150.0
        or inspection.get("maximum_available_score") != 150.0
    ):
        raise ValueError("canonical competition inspection failed")
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
        and inspection.get("run_id") == manifest.get("run_id")
        and inspection.get("manifest_sha256")
        == hashlib.sha256((directory / "manifest.json").read_bytes()).hexdigest()
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
    parser.add_argument("--expected-image-digests", type=Path)
    arguments = parser.parse_args()
    try:
        expected_image_digests = None
        if arguments.expected_image_digests is not None:
            document = json.loads(arguments.expected_image_digests.read_text())
            if not isinstance(document, dict) or not all(
                isinstance(name, str) and isinstance(digest, str)
                for name, digest in document.items()
            ):
                raise ValueError("expected image digests must be a JSON object of strings")
            expected_image_digests = document
        write_competition_verification(
            arguments.run_directory,
            arguments.output,
            expected_image_digests=expected_image_digests,
        )
    except (OSError, json.JSONDecodeError, ValueError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
