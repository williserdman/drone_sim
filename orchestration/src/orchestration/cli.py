"""Installed operator command-line interface."""

from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
import json
from pathlib import Path
import sys
from typing import Any, Callable, Sequence, TextIO
from uuid import uuid4

from .controller import ControllerError, RunController, RunResult
from .host_integration import resolve_host
from .status_store import ProtocolFileError
from .suite import SuiteRunner, SuiteResult
from .console_client import request_console
from .workstation import Workstation


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="drone-sim")
    commands = parser.add_subparsers(dest="command", required=True)

    start = commands.add_parser("start")
    start.add_argument("--config")
    suite = commands.add_parser("suite")
    suite.add_argument("--config")
    suite.add_argument("--output-root")

    for name in ("status", "abort", "collect-results"):
        command = commands.add_parser(name)
        command.add_argument("run_id")
        command.add_argument("--output-root")
    console = commands.add_parser("console", help="open the local simulation console")
    console.add_argument("--port", type=int, default=8765)
    console.add_argument("--no-browser", action="store_true")
    prepare = commands.add_parser("prepare", help="prepare one approved preset without starting flight")
    prepare.add_argument("--preset", default="configured-descent")
    prepare.add_argument("--request-id")
    prepare.add_argument("--calibration-source")
    prepare.add_argument("--preparation-wall-seconds", type=int, default=1800)
    prepare.add_argument("--port", type=int, default=8765)
    for name in ("start-mission", "abort-flight", "stop"):
        command = commands.add_parser(
            name,
            help=(
                "legacy descriptor-absent operator action"
                if name != "stop"
                else "finalize and stop simulation infrastructure"
            ),
        )
        command.add_argument("run_id", nargs="?" if name == "stop" else None)
        command.add_argument("--port", type=int, default=8765)
        if name != "stop":
            command.add_argument("--request-id")
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
    suite_runner_factory: Callable[..., SuiteRunner] = SuiteRunner,
    console_client: Callable[..., dict[str, Any]] = request_console,
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
        return exc.code if isinstance(exc.code, int) else 1

    try:
        if arguments.command == "console":
            from .console import serve_console
            serve_console(Path.cwd().resolve(), port=arguments.port, open_browser=not arguments.no_browser)
            return 0
        if arguments.command in {"prepare", "start-mission", "abort-flight", "stop"}:
            body: dict[str, Any]
            if arguments.command == "prepare":
                body = {"preset_id": arguments.preset, "request_id": arguments.request_id or str(uuid4()),
                        "preparation_wall_seconds": arguments.preparation_wall_seconds}
                if arguments.calibration_source:
                    body["calibration_source"] = arguments.calibration_source
            else:
                body = {"run_id": arguments.run_id}
                if arguments.command != "stop":
                    body["request_id"] = arguments.request_id or str(uuid4())
            document = console_client(arguments.command, body, port=arguments.port)
            output.write(json.dumps(document, allow_nan=False, sort_keys=True) + "\n")
            output.flush()
            return 0
        project_directory = Path.cwd().resolve()
        host = resolve_host(project_directory)
        workstation = Workstation(host)
        controller = controller_factory(
            project_directory=project_directory,
            event_stream=output,
            workstation=workstation,
        )
        result: RunResult | SuiteResult
        if arguments.command == "suite":
            catalog = Path(arguments.config).resolve() if arguments.config else Path.cwd() / "config/ci-suite.json"
            result = suite_runner_factory(
                project_directory=project_directory,
                controller=controller,
                workstation=workstation,
                event_stream=output,
            ).run(catalog, output_root=_output_root(arguments.output_root))
            exit_code = result.exit_code
        elif arguments.command == "start":
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
    except (ControllerError, ProtocolFileError, OSError, ValueError, RuntimeError) as exc:
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


if __name__ == "__main__":
    raise SystemExit(main())
