#!/usr/bin/env python3
"""Inspect one completed local competition run against current provenance."""

from __future__ import annotations

import argparse
from collections.abc import Mapping
import json
from pathlib import Path
import subprocess
from typing import Any, Callable

from artifacts.acceptance import inspect_phase3_bundle


PROJECT_ROOT = Path(__file__).resolve().parents[1]
COMP2026_ROOT = PROJECT_ROOT / "companion/comp2026"
RULES_PATH = PROJECT_ROOT / "scorekeeper/rules/competition_v1.json"
PHASE3_IMAGES = (
    "drone-sim-orchestration-runtime:phase2",
    "drone-sim-artifacts-runtime:phase2",
    "drone-sim-companion-runtime:phase3",
    "drone-sim-ardupilot-runtime:phase3",
    "drone-sim-gazebo-runtime:phase3",
    "drone-sim-electromagnet-runtime:phase3",
    "drone-sim-scorekeeper-runtime:phase3",
)


def _output(runner: Callable[..., Any], command: list[str]) -> str:
    result = runner(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        shell=False,
        check=False,
        timeout=30,
    )
    if result.returncode != 0:
        detail = str(getattr(result, "stderr", "")).strip()
        raise RuntimeError(f"command failed: {' '.join(command)}: {detail}")
    return str(result.stdout).strip()


def _source(
    runner: Callable[..., Any], name: str, directory: Path
) -> tuple[str, str, bool]:
    revision = _output(
        runner, ["git", "-C", str(directory), "rev-parse", "HEAD"]
    )
    if len(revision) != 40 or any(character not in "0123456789abcdef" for character in revision):
        raise RuntimeError(f"{name} revision is not a full lowercase Git object ID")
    status = _output(
        runner,
        [
            "git",
            "-C",
            str(directory),
            "status",
            "--porcelain",
            "--untracked-files=normal",
        ],
    )
    return name, revision, bool(status)


def _image_digest(runner: Callable[..., Any], image: str) -> str:
    image_id = _output(
        runner,
        ["docker", "image", "inspect", "--format", "{{.Id}}", image],
    )
    prefix = "sha256:"
    digest = image_id[len(prefix) :] if image_id.startswith(prefix) else image_id
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise RuntimeError(f"image digest is invalid: {image}")
    return digest


def inspect_competition_run(
    run_directory: Path | str,
    *,
    runner: Callable[..., Any] = subprocess.run,
    bundle_inspector: Callable[..., Any] = inspect_phase3_bundle,
    expected_image_digests: Mapping[str, str] | None = None,
) -> dict[str, object]:
    """Return the canonical accepted report without modifying run evidence."""
    sources = (
        _source(runner, "drone_sim", PROJECT_ROOT),
        _source(runner, "comp2026", COMP2026_ROOT),
    )
    report = bundle_inspector(
        Path(run_directory).resolve(),
        rules_path=RULES_PATH,
        expected_source_revisions={name: revision for name, revision, _dirty in sources},
        expected_source_dirty={name: dirty for name, _revision, dirty in sources},
        expected_image_digests=(
            dict(expected_image_digests)
            if expected_image_digests is not None
            else {image: _image_digest(runner, image) for image in PHASE3_IMAGES}
        ),
        require_maximum_score=True,
    )
    return report.to_dict()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
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
        report = inspect_competition_run(
            arguments.run_directory,
            expected_image_digests=expected_image_digests,
        )
    except Exception as error:
        print(
            json.dumps(
                {"accepted": False, "detail": str(error)},
                sort_keys=True,
                separators=(",", ":"),
            )
        )
        return 1
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
