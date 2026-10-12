from pathlib import Path

import yaml


ROOT = Path(__file__).parents[2]
WORKFLOW = ROOT / ".github/workflows/mission-suite.yml"
DISPATCH_ROOT = (
    "/home/willis/projects/drone_sim/runs/ci/"
    "${{ github.run_id }}-${{ github.run_attempt }}"
)


def _workflow():
    return yaml.safe_load(WORKFLOW.read_text(encoding="utf-8"))


def test_workflow_is_manual_and_serializes_workstation_suites():
    workflow = _workflow()

    assert set(workflow.get("on", workflow.get(True))) == {"workflow_dispatch"}
    assert set(workflow["jobs"]) == {"suite"}
    assert workflow["concurrency"] == {
        "group": "drone-sim-mission-suite",
        "cancel-in-progress": False,
    }
    assert workflow["permissions"] == {"contents": "read"}

    job = workflow["jobs"]["suite"]
    assert job["runs-on"] == ["self-hosted", "linux", "x64", "drone-sim"]
    assert job["timeout-minutes"] == 1440


def test_workflow_runs_the_locked_suite_and_preserves_its_exit_code():
    job = _workflow()["jobs"]["suite"]
    steps = job["steps"]

    checkout = next(step for step in steps if step.get("name") == "Checkout")
    assert checkout["uses"] == "actions/checkout@3d3c42e5aac5ba805825da76410c181273ba90b1"
    assert checkout["with"]["persist-credentials"] is False

    sync = next(step for step in steps if step.get("name") == "Sync environment")
    assert sync["run"].strip() == "/home/willis/.local/bin/uv sync --locked"

    suite = next(step for step in steps if step.get("id") == "suite")
    assert "timeout-minutes" not in suite
    assert DISPATCH_ROOT in suite["env"].values()
    assert (
        "/home/willis/.local/bin/uv run --locked drone-sim suite "
        "--config config/ci-suite.json --output-root \"$OUTPUT_ROOT\""
    ) in suite["run"]
    assert "PIPESTATUS[0]" in suite["run"]
    assert "tee" in suite["run"]
    assert 'exit "$suite_exit"' in suite["run"]


def test_workflow_always_publishes_summary_and_compact_dispatch_artifacts():
    steps = _workflow()["jobs"]["suite"]["steps"]
    report = next(step for step in steps if step.get("id") == "report")
    summary = next(step for step in steps if step.get("name") == "Publish summary")
    upload = next(step for step in steps if step.get("name") == "Upload diagnostics")

    assert report["if"] == "${{ always() }}"
    assert 'value.get("result_type") == "suite_result"' in report["run"]
    assert 'value.get("report_path")' in report["run"]
    assert "report.md" in report["run"]
    assert summary["if"] == "${{ always() }}"
    assert upload["if"] == "${{ always() }}"
    assert upload["uses"] == "actions/upload-artifact@043fb46d1a93c77aae656e7c1c64a875d1fc6a0a"

    paths = upload["with"]["path"].splitlines()
    assert paths
    assert all(path.startswith(DISPATCH_ROOT + "/") for path in paths)
    assert any("report" in path for path in paths)
    assert any("manifest" in path for path in paths)
    assert any("/scoring/" in path for path in paths)
    assert any("logs" in path for path in paths)
    assert any("configuration" in path for path in paths)
    assert any("ardupilot_sitl" in path and ".parm" in path for path in paths)
    assert all(".mp4" not in path and ".bag" not in path for path in paths)
