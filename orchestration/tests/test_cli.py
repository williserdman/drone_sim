from __future__ import annotations

import io
import json
from pathlib import Path

import pytest

from orchestration.cli import main
from orchestration.controller import ControllerError, RunResult


RUN_ID = "00000000-0000-4000-8000-000000000606"


class FakeController:
    def __init__(self, result: RunResult | None = None, error: Exception | None = None):
        self.result = result or RunResult(RUN_ID, "COMPLETED", "mission_complete", "manifest.json")
        self.error = error
        self.calls: list[tuple] = []

    def _return(self):
        if self.error is not None:
            raise self.error
        return self.result

    def start(self, config):
        self.calls.append(("start", Path(config)))
        return self._return()

    def status(self, run_id, output_root):
        self.calls.append(("status", run_id, Path(output_root)))
        return self._return()

    def abort(self, run_id, output_root):
        self.calls.append(("abort", run_id, Path(output_root)))
        return self._return()

    def collect_results(self, run_id, output_root):
        self.calls.append(("collect-results", run_id, Path(output_root)))
        return self._return()


def _invoke(argv, controller, *, cwd: Path | None = None):
    stdout = io.StringIO()
    stderr = io.StringIO()
    if cwd is not None:
        old = Path.cwd()
        try:
            import os

            os.chdir(cwd)
            code = main(argv, controller_factory=lambda **_kwargs: controller, stdout=stdout, stderr=stderr)
        finally:
            os.chdir(old)
    else:
        code = main(argv, controller_factory=lambda **_kwargs: controller, stdout=stdout, stderr=stderr)
    return code, stdout.getvalue(), stderr.getvalue()


@pytest.mark.parametrize(
    ("state", "expected_exit"),
    [("COMPLETED", 0), ("FAILED", 1), ("ABORTED", 130)],
)
def test_start_prints_one_final_typed_result_last_and_maps_terminal_exit(state, expected_exit):
    controller = FakeController(
        RunResult(RUN_ID, state, "terminal_reason", "manifest.json")
    )

    code, stdout, stderr = _invoke(["start", "--config", "template.json"], controller)

    assert code == expected_exit
    assert json.loads(stdout) == {
        "manifest_path": "manifest.json",
        "reason": "terminal_reason",
        "result_type": "run_result",
        "run_id": RUN_ID,
        "state": state,
    }
    assert stdout.endswith("\n")
    assert stderr == ""
    assert controller.calls == [("start", Path("template.json"))]


def test_start_defaults_to_repository_competition_config(tmp_path):
    controller = FakeController()

    code, _stdout, stderr = _invoke(["start"], controller, cwd=tmp_path)

    assert code == 0
    assert stderr == ""
    assert controller.calls == [("start", tmp_path / "config/default-run.json")]


@pytest.mark.parametrize("command", ["status", "abort", "collect-results"])
def test_run_directory_commands_default_to_resolved_invocation_runs_and_print_one_json(
    tmp_path, command
):
    controller = FakeController(RunResult(RUN_ID, "RUNNING", "", None))

    code, stdout, stderr = _invoke([command, RUN_ID], controller, cwd=tmp_path)

    assert code == 0
    assert stdout.count("\n") == 1
    assert json.loads(stdout) == {
        "manifest_path": None,
        "reason": "",
        "result_type": "run_result",
        "run_id": RUN_ID,
        "state": "RUNNING",
    }
    assert stderr == ""
    assert controller.calls == [(command, RUN_ID, tmp_path / "runs")]


@pytest.mark.parametrize("command", ["status", "abort", "collect-results"])
def test_run_directory_commands_accept_explicit_absolute_output_root(tmp_path, command):
    controller = FakeController()
    output_root = (tmp_path / "custom").resolve()

    code, _stdout, _stderr = _invoke(
        [command, RUN_ID, "--output-root", str(output_root)], controller
    )

    assert code == 0
    assert controller.calls == [(command, RUN_ID, output_root)]


@pytest.mark.parametrize("command", ["status", "abort", "collect-results"])
def test_explicit_relative_output_root_is_controlled_usage_error(command):
    controller = FakeController()

    code, stdout, stderr = _invoke(
        [command, RUN_ID, "--output-root", "relative"], controller
    )

    assert code == 2
    assert stdout == ""
    assert "absolute" in stderr
    assert "Traceback" not in stderr
    assert controller.calls == []


def test_controlled_controller_failure_exits_two_without_traceback():
    controller = FakeController(error=ControllerError("configuration is invalid"))

    code, stdout, stderr = _invoke(["start", "--config", "bad.json"], controller)

    assert code == 2
    assert stdout == ""
    assert stderr == "configuration is invalid\n"
    assert "Traceback" not in stderr


@pytest.mark.parametrize(
    "argv",
    [[], ["unknown"], ["status"], ["abort"], ["collect-results"]],
)
def test_parse_errors_exit_two_without_python_traceback(argv):
    code, stdout, stderr = _invoke(argv, FakeController())

    assert code == 2
    assert stdout == ""
    assert stderr
    assert "Traceback" not in stderr


def test_help_lists_exactly_the_four_operator_commands():
    code, stdout, stderr = _invoke(["--help"], FakeController())

    assert code == 0
    assert stderr == ""
    for command in ("start", "status", "abort", "collect-results"):
        assert command in stdout
    assert "{start,status,abort,collect-results}" in stdout
