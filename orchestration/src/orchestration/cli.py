"""Installed operator command-line interface."""

from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
import json
from pathlib import Path
import sys
from typing import Callable, Sequence, TextIO

from .controller import ControllerError, RunController
from .status_store import ProtocolFileError


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="drone-sim")
    commands = parser.add_subparsers(dest="command", required=True)

    start = commands.add_parser("start")
    start.add_argument("--config")

    for name in ("status", "abort", "collect-results"):
        command = commands.add_parser(name)
        command.add_argument("run_id")
        command.add_argument("--output-root")
    return parser


def _output_root(value: str | None) -> Path:
    if value is None:
        return (Path.cwd() / "runs").resolve()
    path = Path(value)
    if not path.is_absolute():
        raise ControllerError("explicit output root must be absolute")
    return path


def main(
    argv: Sequence[str] | None = None,
    *,
    controller_factory: Callable[..., RunController] = RunController,
    stdout: TextIO | None = None,
    stderr: TextIO | None = None,
) -> int:
    """Parse one operator command and return its process exit code."""
    output = stdout or sys.stdout
    errors = stderr or sys.stderr
    parser = _parser()
    try:
        with redirect_stdout(output), redirect_stderr(errors):
            arguments = parser.parse_args(list(argv) if argv is not None else None)
    except SystemExit as exc:
        return int(exc.code)

    try:
        controller = controller_factory(
            project_directory=Path.cwd().resolve(), event_stream=output
        )
        if arguments.command == "start":
            config = (
                Path(arguments.config)
                if arguments.config is not None
                else Path.cwd() / "config/default-run.json"
            )
            result = controller.start(config)
            exit_code = result.exit_code
        else:
            root = _output_root(arguments.output_root)
            operation = {
                "status": controller.status,
                "abort": controller.abort,
                "collect-results": controller.collect_results,
            }[arguments.command]
            result = operation(arguments.run_id, root)
            exit_code = 0
    except (ControllerError, ProtocolFileError, OSError, ValueError) as exc:
        errors.write(f"{exc}\n")
        errors.flush()
        return 2

    output.write(
        json.dumps(
            result.to_dict(),
            allow_nan=False,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    )
    output.flush()
    return exit_code


__all__ = ["main"]
