"""Thin process-side observability and durable diagnostic helpers."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import selectors
import subprocess
from typing import Any, Mapping, TextIO
from uuid import uuid4


@dataclass
class OutputFacts:
    json_exchange: bool = False
    mavlink_listening: bool = False

    @property
    def ready(self) -> bool:
        return self.json_exchange and self.mavlink_listening

    def observe(self, line: str) -> None:
        if "bind port 5760" in line:
            self.mavlink_listening = True
        if "JSON received:" in line:
            self.json_exchange = True


class EventWriter:
    def __init__(self, run_id: str, stream: TextIO) -> None:
        self._run_id = run_id
        self._stream = stream

    def emit(self, event: str, *, severity: str = "INFO", **fields: Any) -> None:
        payload = {
            "run_id": self._run_id,
            "module": "ardupilot_sitl",
            "severity": severity,
            "event": event,
            "sim_timestamp": None,
            "wall_timestamp": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "fields": fields,
        }
        self._stream.write(
            json.dumps(payload, allow_nan=False, sort_keys=True, separators=(",", ":")) + "\n"
        )
        self._stream.flush()


class SITLProcess:
    """Shell-free child process with bounded infrastructure I/O and shutdown."""

    def __init__(self, command: tuple[str, ...], working_directory: Path) -> None:
        self._command = command
        self._working_directory = working_directory
        self._process: subprocess.Popen[str] | None = None
        self._selector = selectors.DefaultSelector()

    def start(self) -> None:
        if self._process is not None:
            raise RuntimeError("SITL process already started")
        self._working_directory.mkdir(parents=True, exist_ok=True)
        self._process = subprocess.Popen(
            self._command,
            cwd=self._working_directory,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        assert self._process.stdout is not None and self._process.stderr is not None
        try:
            self._selector.register(self._process.stdout, selectors.EVENT_READ, "stdout")
            self._selector.register(self._process.stderr, selectors.EVENT_READ, "stderr")
        except BaseException as error:
            try:
                self.stop(10.0)
            except BaseException as cleanup_error:
                error.add_note(f"SITL cleanup after selector failure: {cleanup_error}")
            raise

    @property
    def return_code(self) -> int | None:
        return None if self._process is None else self._process.poll()

    def read_line(self, timeout_seconds: float) -> tuple[str, str] | None:
        for key, _mask in self._selector.select(timeout_seconds):
            line = key.fileobj.readline()
            if line:
                return str(key.data), line.rstrip("\r\n")
            self._selector.unregister(key.fileobj)
        return None

    def stop(self, timeout_seconds: float) -> int | None:
        if self._process is None:
            return None
        if self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=timeout_seconds)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=timeout_seconds)
        return int(self._process.returncode)


def atomic_document(path: Path, document: Mapping[str, Any]) -> None:
    payload = json.dumps(document, allow_nan=False, sort_keys=True, separators=(",", ":")).encode()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
    descriptor: int | None = None
    try:
        descriptor = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0),
            0o644,
        )
        written = 0
        while written < len(payload):
            count = os.write(descriptor, payload[written:])
            if count <= 0:
                raise OSError("status write made no progress")
            written += count
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


@dataclass(frozen=True)
class DiagnosticInventory:
    root: Path

    def relative_paths(self) -> tuple[str, ...]:
        candidates = [self.root / "eeprom.bin", self.root / "flash.dat", self.root / "failure.json"]
        candidates.extend((self.root / "logs").glob("*.BIN") if (self.root / "logs").is_dir() else ())
        return tuple(
            sorted(str(path.relative_to(self.root)) for path in candidates if path.is_file())
        )
