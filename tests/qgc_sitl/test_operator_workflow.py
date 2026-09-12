from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(ROOT / "orchestration/src"))
sys.path.insert(0, str(ROOT / "companion/src"))
NESTED_SOURCE = ROOT / "companion/comp2026/src"
NESTED_TESTS = ROOT / "companion/comp2026/tests"
sys.path.insert(0, str(NESTED_SOURCE))
sys.path.insert(0, str(NESTED_TESTS))

from orchestration.config import resolve_run_config
from drone_sim_companion.qgc_runtime_policy import load_qgc_runtime_policy
from test_prepare_attempt import profile_data


def test_prepare_qgc_run_creates_valid_external_attempt_and_full_policy(tmp_path: Path) -> None:
    profile = tmp_path / "deployment-profile.json"
    profile.write_text(json.dumps(profile_data()), encoding="utf-8")
    operator_directory = tmp_path / "operator state"
    output_root = tmp_path / "runs"

    completed = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/prepare_qgc_run.py"),
            "--profile",
            str(profile),
            "--operator-directory",
            str(operator_directory),
            "--output-root",
            str(output_root),
            "--acknowledge-on-ground",
        ],
        cwd=ROOT,
        env={"PATH": os.environ["PATH"]},
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    prepared = json.loads(completed.stdout)
    run_template = Path(prepared["run_template"])
    actions = Path(prepared["qgc_actions"])
    session = Path(prepared["listener_session"])
    state_root = Path(prepared["attempt_state_root"])
    resolved = resolve_run_config(run_template)

    assert run_template.is_file()
    assert actions.is_file()
    assert session.is_file()
    assert resolved.qgc is not None
    assert resolved.qgc.attempt_state_root == state_root
    assert state_root.parent == operator_directory
    state_directory = state_root / resolved.qgc.attempt_state_id
    assert sorted(path.name for path in state_directory.iterdir()) == [
        "attempt-ledger.json",
        "attempt-ledger.json.lock",
    ]
    policy_path = Path(json.loads(run_template.read_text())["qgc"]["runtime_policy"])
    policy = load_qgc_runtime_policy(
        policy_path,
        expected_sha256=__import__("hashlib").sha256(policy_path.read_bytes()).hexdigest(),
    )
    assert policy.purpose == "drone-sim-comp2026-full"
    assert policy.enabled_phases == (31000, 31001, 31002)
    assert [action["mavCmd"] for action in json.loads(actions.read_text())["actions"][:3]] == [
        31000,
        31001,
        31002,
    ]
    assert prepared["commands"] == {
        "start": [
            "uv", "run", "--locked", "drone-sim", "start", "--config", str(run_template)
        ],
        "status": [
            "uv", "run", "--locked", "drone-sim", "status", "RUN_ID",
            "--output-root", str(output_root.resolve()),
        ],
        "abort": [
            "uv", "run", "--locked", "drone-sim", "abort", "RUN_ID",
            "--output-root", str(output_root.resolve()),
        ],
    }
    assert prepared["recordings"] == [
        str(output_root.resolve() / "RUN_ID/video/onboard.mp4"),
        str(output_root.resolve() / "RUN_ID/video/observer.mp4"),
    ]


def test_prepare_qgc_run_requires_absolute_external_directories(tmp_path: Path) -> None:
    profile = tmp_path / "deployment-profile.json"
    profile.write_text(json.dumps(profile_data()), encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/prepare_qgc_run.py"),
            "--profile",
            str(profile),
            "--operator-directory",
            "relative-state",
            "--output-root",
            str(tmp_path / "runs"),
            "--acknowledge-on-ground",
        ],
        cwd=ROOT,
        env={"PATH": os.environ["PATH"]},
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )

    assert completed.returncode == 2
    assert "absolute" in completed.stderr
