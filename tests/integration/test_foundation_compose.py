import json
import subprocess


RUN_ID = "00000000-0000-4000-8000-000000000099"
COMMON_FIELDS = {
    "run_id",
    "module",
    "severity",
    "event",
    "sim_timestamp",
    "wall_timestamp",
}


def test_foundation_compose_emits_an_ordered_structured_lifecycle(tmp_path):
    output_dir = tmp_path / "output"
    output_dir.mkdir()

    result = subprocess.run(
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
        ],
        capture_output=True,
        text=True,
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
