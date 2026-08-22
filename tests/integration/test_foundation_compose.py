import json
import subprocess

import pytest


RUN_ID = "00000000-0000-4000-8000-000000000099"
FOUNDATION_TIMEOUT_SECONDS = 180
COMMON_FIELDS = {
    "run_id",
    "module",
    "severity",
    "event",
    "sim_timestamp",
    "wall_timestamp",
}


def _run_foundation_compose(command):
    try:
        return subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=FOUNDATION_TIMEOUT_SECONDS,
        )
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(
            f"foundation Compose run timed out after {FOUNDATION_TIMEOUT_SECONDS} seconds\n"
            f"stdout:\n{error.stdout or ''}\n"
            f"stderr:\n{error.stderr or ''}"
        ) from error


def test_foundation_subprocess_timeout_includes_captured_diagnostics(monkeypatch):
    def raise_timeout(*args, **kwargs):
        raise subprocess.TimeoutExpired(
            cmd=args[0],
            timeout=kwargs["timeout"],
            output="publisher output",
            stderr="discovery stalled",
        )

    monkeypatch.setattr(subprocess, "run", raise_timeout)

    with pytest.raises(RuntimeError) as error:
        _run_foundation_compose(["docker", "compose", "run"])

    message = str(error.value)
    assert "180 seconds" in message
    assert "publisher output" in message
    assert "discovery stalled" in message


def test_foundation_compose_emits_an_ordered_structured_lifecycle(tmp_path):
    output_dir = tmp_path / "output"
    output_dir.mkdir()

    result = _run_foundation_compose(
        [
            "docker",
            "compose",
            "run",
            "--rm",
            "-e",
            f"SIM_RUN_ID={RUN_ID}",
            "-e",
            "SIM_OUTPUT_ROOT=/output",
            "-v",
            f"{output_dir.resolve()}:/output",
            "foundation",
        ]
    )

    assert result.returncode == 0, result.stderr
    log_path = output_dir / "logs" / "foundation.jsonl"
    assert log_path.is_file()

    events = [json.loads(line) for line in log_path.read_text().splitlines()]
    assert len(events) == 5
    assert all(COMMON_FIELDS <= event.keys() for event in events)
    assert all(event["run_id"] == RUN_ID for event in events)
    assert [event["event"] for event in events[:4]] == [
        "starting",
        "ready",
        "clock_started",
        "finalizing",
    ]
    assert events[-1]["sim_timestamp"] is not None

    stdout_events = [json.loads(line) for line in result.stdout.splitlines() if line.strip()]
    assert stdout_events == events

    observation_path = output_dir / "foundation-observation.json"
    assert observation_path.is_file()
    observation = json.loads(observation_path.read_text(encoding="utf-8"))
    assert observation["discovered"] is True
    assert observation["complete"] is True
    assert observation["error"] is None
    assert observation["clock"] == [
        {"sec": 0, "nanosec": 0},
        {"sec": 0, "nanosec": 50_000_000},
        {"sec": 0, "nanosec": 100_000_000},
    ]
    assert observation["run_state"] == [
        {
            "run_id": RUN_ID,
            "sim_timestamp": {"sec": 0, "nanosec": 0},
            "state": 1,
            "reason": "",
            "config_sha256": "",
        },
        {
            "run_id": RUN_ID,
            "sim_timestamp": {"sec": 0, "nanosec": 0},
            "state": 2,
            "reason": "",
            "config_sha256": "",
        },
        {
            "run_id": RUN_ID,
            "sim_timestamp": {"sec": 0, "nanosec": 50_000_000},
            "state": 3,
            "reason": "",
            "config_sha256": "",
        },
        {
            "run_id": RUN_ID,
            "sim_timestamp": {"sec": 0, "nanosec": 100_000_000},
            "state": 4,
            "reason": "",
            "config_sha256": "",
        },
        {
            "run_id": RUN_ID,
            "sim_timestamp": {"sec": 0, "nanosec": 100_000_000},
            "state": 5,
            "reason": "",
            "config_sha256": "",
        },
    ]
    assert observation["qos"] == {
        "/clock": {
            "publisher": {
                "reliability": "BEST_EFFORT",
                "durability": "VOLATILE",
            },
            "subscriber": {
                "history": "KEEP_LAST",
                "depth": 1,
                "reliability": "BEST_EFFORT",
                "durability": "VOLATILE",
            },
            "compatible": True,
        },
        "/simulation/run_state": {
            "publisher": {
                "reliability": "RELIABLE",
                "durability": "TRANSIENT_LOCAL",
            },
            "subscriber": {
                "history": "KEEP_LAST",
                "depth": 1,
                "reliability": "RELIABLE",
                "durability": "TRANSIENT_LOCAL",
            },
            "compatible": True,
        },
    }
