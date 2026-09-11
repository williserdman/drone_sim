#!/usr/bin/env python3
"""Bounded host lifecycle for the isolated QGC SITL Compose project."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
from typing import Any, Callable


HERE = Path(__file__).resolve().parent
SERVICES = ("arducopter-457", "listener-probe")


class TerminationRequested(BaseException):
    pass


def _record(path: Path, value: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def _call(
    runner: Callable[..., Any],
    command: tuple[str, ...],
    *,
    timeout_s: float,
) -> tuple[dict[str, object], str]:
    try:
        completed = runner(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except BaseException as error:
        return {
            "status": "error",
            "error": f"{type(error).__name__}: {error}",
        }, ""
    return {
        "status": "completed",
        "returncode": int(completed.returncode),
        "stderr": str(completed.stderr),
    }, str(completed.stdout)


def _capture_and_record(
    *,
    result_dir: Path,
    output_name: str,
    output: str,
    status_path: Path,
    status: dict[str, object],
    entry: dict[str, object],
) -> None:
    try:
        (result_dir / output_name).write_text(output)
    except OSError as error:
        entry["capture_error"] = f"{type(error).__name__}: {error}"
    try:
        _record(status_path, status)
    except OSError as error:
        entry["status_capture_error"] = f"{type(error).__name__}: {error}"


def host_result_succeeded(result: dict[str, object]) -> bool:
    fields = ("run", "logs", "service_status", "service_cleanup", "network_cleanup")
    return all(
        isinstance(result.get(field), dict)
        and result[field].get("status") == "completed"  # type: ignore[union-attr]
        and result[field].get("returncode") == 0  # type: ignore[union-attr]
        and "capture_error" not in result[field]  # type: ignore[operator]
        and "status_capture_error" not in result[field]  # type: ignore[operator]
        for field in fields
    )


def validate_result_owner(result_dir: Path) -> None:
    identities: dict[str, int] = {}
    for name in ("QGC_SITL_UID", "QGC_SITL_GID"):
        value = os.environ.get(name, "")
        if not value.isdecimal():
            raise ValueError(f"{name} must be a numeric ID")
        identities[name] = int(value)
    expected_owner = (identities["QGC_SITL_UID"], identities["QGC_SITL_GID"])
    for path in (result_dir, result_dir / "arducopter"):
        metadata = path.stat()
        if not stat.S_ISDIR(metadata.st_mode):
            raise ValueError(f"result mount is not a directory: {path}")
        if (metadata.st_uid, metadata.st_gid) != expected_owner:
            raise ValueError(f"result mount owner does not match QGC_SITL_UID:GID: {path}")
        if stat.S_IMODE(metadata.st_mode) & 0o077:
            raise ValueError(f"result mount must be private to its owner: {path}")


def run_isolated_project(
    *,
    project: str,
    result_dir: Path,
    runner: Callable[..., Any] = subprocess.run,
    operation_timeout_s: float = 30,
    run_timeout_s: float = 180,
) -> dict[str, object]:
    if not project.startswith("qgc-sitl-"):
        raise ValueError("project must use the qgc-sitl prefix")
    result_dir.mkdir(parents=True, exist_ok=True)
    compose = (
        "docker", "compose", "--project-name", project,
        "-f", str(HERE / "compose.yaml"),
    )
    status: dict[str, object] = {
        "schema_version": 1,
        "project": project,
        "run": {"status": "not_started"},
        "logs": {"status": "not_attempted"},
        "service_status": {"status": "not_attempted"},
        "service_cleanup": {"status": "not_attempted"},
        "network_cleanup": {"status": "not_attempted"},
    }
    status_path = result_dir / "host-status.json"
    if status_path.exists():
        raise FileExistsError(f"host status already exists: {status_path}")
    def terminate(signum: int, _frame: object) -> None:
        raise TerminationRequested(f"received signal {signum}")

    previous_sigint = signal.signal(signal.SIGINT, terminate)
    previous_sigterm = signal.signal(signal.SIGTERM, terminate)
    try:
        _record(status_path, status)
        try:
            status["run"], output = _call(
                runner,
                compose + (
                    "up", "--no-build", "--abort-on-container-exit",
                    "--exit-code-from", "listener-probe",
                ),
                timeout_s=run_timeout_s,
            )
            _capture_and_record(
                result_dir=result_dir,
                output_name="compose-up.stdout",
                output=output,
                status_path=status_path,
                status=status,
                entry=status["run"],  # type: ignore[arg-type]
            )
        except BaseException as error:
            status["run"] = {
                "status": "error",
                "error": f"{type(error).__name__}: {error}",
            }
            try:
                _record(status_path, status)
            except OSError as capture_error:
                status["run"]["status_capture_error"] = (  # type: ignore[index]
                    f"{type(capture_error).__name__}: {capture_error}"
                )
        finally:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            for field, suffix, output_name in (
                ("logs", ("logs", "--no-color"), "compose.log"),
                ("service_status", ("ps", "--all", "--format", "json"), "service-status.json"),
                (
                    "service_cleanup",
                    ("rm", "--force", "--stop", "--volumes", *SERVICES),
                    "service-cleanup.stdout",
                ),
            ):
                entry, output = _call(
                    runner, compose + suffix, timeout_s=operation_timeout_s
                )
                status[field] = entry
                _capture_and_record(
                    result_dir=result_dir,
                    output_name=output_name,
                    output=output,
                    status_path=status_path,
                    status=status,
                    entry=entry,
                )
            entry, output = _call(
                runner,
                ("docker", "network", "rm", f"{project}_default"),
                timeout_s=operation_timeout_s,
            )
            status["network_cleanup"] = entry
            _capture_and_record(
                result_dir=result_dir,
                output_name="network-cleanup.stdout",
                output=output,
                status_path=status_path,
                status=status,
                entry=entry,
            )
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
    return status


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project")
    parser.add_argument("result_dir", type=Path)
    arguments = parser.parse_args()
    result_dir = arguments.result_dir.resolve()
    validate_result_owner(result_dir)
    result = run_isolated_project(
        project=arguments.project,
        result_dir=result_dir,
    )
    return 0 if host_result_succeeded(result) else 1


if __name__ == "__main__":
    raise SystemExit(main())
