#!/usr/bin/env python3
"""Prepare one external QGC attempt and its drone-sim run template."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid


ROOT = Path(__file__).resolve().parents[1]
NESTED_SOURCE = ROOT / "companion/comp2026/src"
PREPARATION_SCRIPT = NESTED_SOURCE / "gc/prepare_attempt.py"
BASE_TEMPLATE = ROOT / "config/default-run.json"
POLICY = ROOT / "config/qgc-full-simulation-policy.json"


def _absolute_canonical(value: str, *, label: str) -> Path:
    if "\0" in value or value.startswith("//"):
        raise ValueError(f"{label} must be an absolute canonical path")
    path = Path(value)
    canonical = Path(os.path.abspath(value))
    if not path.is_absolute() or path != canonical or path == Path("/"):
        raise ValueError(f"{label} must be an absolute canonical path")
    return canonical


def _run_preparation(arguments: list[str]) -> subprocess.CompletedProcess[str]:
    environment = {
        "PATH": os.environ.get("PATH", os.defpath),
        "PYTHONPATH": str(NESTED_SOURCE),
    }
    return subprocess.run(
        [sys.executable, str(PREPARATION_SCRIPT), *arguments],
        cwd=ROOT,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--operator-directory", required=True)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--acknowledge-on-ground", action="store_true")
    return parser


def main() -> int:
    parser = _parser()
    arguments = parser.parse_args()
    try:
        profile = _absolute_canonical(arguments.profile, label="profile")
        operator_directory = _absolute_canonical(
            arguments.operator_directory, label="operator directory"
        )
        output_root = _absolute_canonical(arguments.output_root, label="output root")
        if arguments.acknowledge_on_ground is not True:
            raise ValueError("--acknowledge-on-ground is required")
        if not profile.is_file() or profile.is_symlink():
            raise ValueError("profile must be a regular non-symlink file")

        profile_digest = hashlib.sha256(profile.read_bytes()).hexdigest()
        state_root = operator_directory / "attempt-state"
        state_directory = state_root / f"sha256-{profile_digest}"
        state_directory.mkdir(parents=True, exist_ok=True)
        ledger = state_directory / "attempt-ledger.json"
        if not ledger.exists():
            initialized = _run_preparation(["initialize-ledger", str(ledger)])
            if initialized.returncode != 0:
                raise ValueError(initialized.stderr.strip() or "ledger initialization failed")

        attempt_directory = operator_directory / "attempts" / uuid.uuid4().hex
        attempt_directory.mkdir(parents=True)
        session = attempt_directory / "listener-session.json"
        actions = attempt_directory / "qgc-actions.json"
        prepared = _run_preparation(
            [
                "prepare",
                "--profile",
                str(profile),
                "--ledger",
                str(ledger),
                "--session",
                str(session),
                "--actions",
                str(actions),
                "--acknowledge-on-ground",
            ]
        )
        if prepared.returncode != 0:
            raise ValueError(prepared.stderr.strip() or "attempt preparation failed")
        preparation_result = json.loads(prepared.stdout)

        shutil.copyfile(ROOT / "config/course.yaml", attempt_directory / "course.yaml")
        shutil.copyfile(ROOT / "config/scenario.yaml", attempt_directory / "scenario.yaml")
        template = json.loads(BASE_TEMPLATE.read_text(encoding="utf-8"))
        template["output_root"] = str(output_root)
        template["qgc"] = {
            "deployment_profile": str(profile),
            "listener_session": str(session),
            "qgc_actions": str(actions),
            "runtime_policy": str(POLICY),
            "attempt_state_root": str(state_root),
        }
        run_template = attempt_directory / "qgc-run.json"
        run_template.write_text(
            json.dumps(template, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

        result = {
            **preparation_result,
            "run_template": str(run_template),
            "qgc_actions": str(actions),
            "listener_session": str(session),
            "attempt_state_root": str(state_root),
            "commands": {
                "start": [
                    "uv", "run", "--locked", "drone-sim", "start",
                    "--config", str(run_template),
                ],
                "status": [
                    "uv", "run", "--locked", "drone-sim", "status", "RUN_ID",
                    "--output-root", str(output_root),
                ],
                "abort": [
                    "uv", "run", "--locked", "drone-sim", "abort", "RUN_ID",
                    "--output-root", str(output_root),
                ],
            },
            "recordings": [
                str(output_root / "RUN_ID/video/onboard.mp4"),
                str(output_root / "RUN_ID/video/observer.mp4"),
            ],
        }
        print(json.dumps(result, sort_keys=True))
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
