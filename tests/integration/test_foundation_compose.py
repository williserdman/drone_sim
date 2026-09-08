import json
import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[2]
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


def _compose_environment():
    environment = os.environ.copy()
    for name in (
        "COMPOSE_FILE", "COMPOSE_ENV_FILES", "COMPOSE_PATH_SEPARATOR",
        "COMPOSE_PROFILES", "COMPOSE_PROJECT_NAME", "COMPOSE_PROJECT_DIR",
        "COMPOSE_PROJECT_DIRECTORY", "COMPOSE_DISABLE_ENV_FILE",
    ):
        environment.pop(name, None)
    environment["COMPOSE_DISABLE_ENV_FILE"] = "1"
    return environment


def _run_foundation_compose(command):
    try:
        return subprocess.run(
            command,
            cwd=ROOT,
            env=_compose_environment(),
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


def _parse_structured_stdout(stdout):
    events = []
    for line in stdout.splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and COMMON_FIELDS <= value.keys():
            events.append(value)
    return events


def test_foundation_subprocess_timeout_includes_captured_diagnostics(monkeypatch):
    compose_selectors = {
        "COMPOSE_FILE", "COMPOSE_ENV_FILES", "COMPOSE_PATH_SEPARATOR",
        "COMPOSE_PROFILES", "COMPOSE_PROJECT_NAME", "COMPOSE_PROJECT_DIR",
        "COMPOSE_PROJECT_DIRECTORY", "COMPOSE_DISABLE_ENV_FILE",
    }
    for name in compose_selectors:
        monkeypatch.setenv(name, "hostile")
    monkeypatch.setenv("DOCKER_HOST", "unix:///tmp/docker.sock")
    monkeypatch.setenv("SIM_RUN_ID", "explicit-run")

    def raise_timeout(*args, **kwargs):
        assert kwargs["cwd"] == ROOT
        assert kwargs["env"]["COMPOSE_DISABLE_ENV_FILE"] == "1"
        assert not (compose_selectors - {"COMPOSE_DISABLE_ENV_FILE"}) & kwargs["env"].keys()
        assert kwargs["env"]["DOCKER_HOST"] == "unix:///tmp/docker.sock"
        assert kwargs["env"]["SIM_RUN_ID"] == "explicit-run"
        raise subprocess.TimeoutExpired(
            cmd=args[0],
            timeout=kwargs["timeout"],
            output="publisher output",
            stderr="discovery stalled",
        )

    monkeypatch.setattr(subprocess, "run", raise_timeout)

    with pytest.raises(RuntimeError) as error:
        _run_foundation_compose(
            [
                "docker", "compose", "--file", "compose.yaml",
                "--project-directory", str(ROOT), "run",
            ]
        )

    message = str(error.value)
    assert "180 seconds" in message
    assert "publisher output" in message
    assert "discovery stalled" in message


def test_parse_structured_stdout_ignores_compose_progress():
    event = {
        "run_id": RUN_ID,
        "module": "foundation",
        "severity": "INFO",
        "event": "starting",
        "sim_timestamp": None,
        "wall_timestamp": "2026-08-23T00:00:00+00:00",
    }
    stdout = '#0 building with "default" instance using docker driver\n' + json.dumps(event)

    assert _parse_structured_stdout(stdout) == [event]


def test_foundation_compose_emits_an_ordered_structured_lifecycle(tmp_path):
    output_dir = tmp_path / "output"
    output_dir.mkdir()

    result = _run_foundation_compose(
        [
            "docker",
            "compose",
            "--file",
            "compose.yaml",
            "--project-directory",
            str(ROOT),
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

    stdout_events = _parse_structured_stdout(result.stdout)
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
