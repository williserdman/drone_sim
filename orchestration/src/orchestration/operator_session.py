"""Persistent owner shared by browser and CLI operator actions."""

from __future__ import annotations

from dataclasses import asdict
import json
import os
from pathlib import Path
import tempfile
import threading
import time
from typing import Any
from urllib.request import urlopen
from uuid import UUID, uuid4

from .catalog import PresetCatalog, Preset
from .config import load_run_config
from .controller import RunController
from .gcs_bridge import GcsBridge
from .local_runtime import LocalRuntime
from .operator_readiness import preflight_snapshot
from .status_store import StatusStore
from .workstation import workstation_lock


_TERMINAL = {"COMPLETED", "FAILED", "ABORTED", "INTERRUPTED"}


def _uuid(value: str) -> str:
    if str(UUID(value)) != value:
        raise ValueError("request identity must be a canonical UUID")
    return value


def _tail(path: Path, maximum: int = 16384) -> str:
    try:
        with path.open("rb") as source:
            source.seek(0, 2)
            source.seek(max(0, source.tell() - maximum))
            return source.read(maximum).decode(errors="replace")
    except FileNotFoundError:
        return ""


class OperatorSession:
    def __init__(self, project_directory: Path, *, state_directory: Path | None = None,
                 runtime: Any = None, controller: Any = None) -> None:
        self.project_directory = project_directory.resolve()
        self.state_directory = state_directory or self.project_directory / ".local-runtime"
        self.state_directory.mkdir(parents=True, exist_ok=True)
        self.catalog = PresetCatalog(self.project_directory)
        self.runtime = runtime or LocalRuntime(self.project_directory)
        self.controller = controller or RunController(project_directory=self.project_directory)
        self._lock = threading.RLock()
        self._worker: threading.Thread | None = None
        self._cancel = threading.Event()
        self._bridge: GcsBridge | None = None
        self._record: dict[str, Any] = {"state": "IDLE", "run_id": None, "preset_id": None,
                                         "prepare_request_id": None, "error": "", "cleanup": "not started"}
        self._checks: dict[str, Any] | None = None
        receipt = self.state_directory / "session.json"
        if receipt.exists():
            previous = json.loads(receipt.read_text())
            if previous.get("run_id"):
                _uuid(previous["run_id"])
                directory = Path(previous["run_directory"])
                if directory != self.project_directory / "runs" / previous["run_id"]:
                    raise ValueError("session receipt belongs to another output directory")
                self._record = previous
                try:
                    state = self.controller.status(previous["run_id"], directory.parent).state
                    self._record["state"] = state if state in _TERMINAL else "ORPHANED"
                except (OSError, ValueError, RuntimeError):
                    self._record["state"] = "ORPHANED"
                if self._record["state"] == "ORPHANED":
                    self._record["error"] = "The previous session owner stopped. Stop Simulation to retain diagnostics and clean up its containers."

    def _save(self) -> None:
        descriptor, temporary = tempfile.mkstemp(prefix="session-", dir=self.state_directory)
        try:
            with os.fdopen(descriptor, "w") as output:
                json.dump(self._record, output, allow_nan=False)
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary, self.state_directory / "session.json")
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def _active(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    def prepare(self, *, preset_id: str, request_id: str | None = None,
                preparation_wall_seconds: int = 1800, calibration_source: str | None = None) -> dict[str, Any]:
        preset = self.catalog.get(preset_id)
        request_id = _uuid(request_id or str(uuid4()))
        if type(preparation_wall_seconds) is not int or not 30 <= preparation_wall_seconds <= 86400:
            raise ValueError("preparation limit must be 30–86400 wall seconds")
        if preset.requires_calibration and not calibration_source:
            raise ValueError("this preset requires an accepted calibration run directory")
        if calibration_source and not preset.requires_calibration:
            raise ValueError("this preset does not request calibration import")
        with self._lock:
            if self._record.get("prepare_request_id") == request_id:
                if preset_id != self._record.get("preset_id"):
                    raise ValueError("prepare request identity belongs to another preset")
                return self.snapshot()
            if self._active() or self._record["state"] == "ORPHANED":
                raise RuntimeError("a session is active; stop and finalize it before preparing another")
            self._cancel.clear()
            self._record = {"state": "PREPARING", "run_id": None, "preset_id": preset_id,
                            "prepare_request_id": request_id, "error": "", "cleanup": "pending",
                            "preparation_wall_seconds": preparation_wall_seconds, "owner_pid": os.getpid()}
            self._save()
            self._worker = threading.Thread(target=self._prepare_worker,
                args=(preset, preparation_wall_seconds, calibration_source), name="simulation-owner")
            self._worker.start()
            return dict(self._record)

    def _prepare_worker(self, preset: Preset, preparation_wall_seconds: int, calibration_source: str | None) -> None:
        try:
            with workstation_lock():
                checks = self.runtime.checks()
                with self._lock:
                    self._checks = checks
                if not checks["ok"]:
                    raise RuntimeError("; ".join(f'{row["name"]}: {row["detail"]}' for row in checks["checks"] if not row["ok"]))
                if self._cancel.is_set():
                    with self._lock:
                        self._record.update(state="ABORTED", cleanup="complete")
                    return
                source = preset.template
                if calibration_source:
                    document = json.loads(source.read_text())
                    document["calibration"] = {"source_run_directory": str(Path(calibration_source).expanduser().resolve())}
                    source = self.state_directory / f'input-{self._record["prepare_request_id"]}.json'
                    source.write_text(json.dumps(document))
                self.runtime.open_qgc()

                def allocated(run_id: str, directory: Path) -> None:
                    with self._lock:
                        self._record.update(run_id=run_id, run_directory=str(directory))
                        self._save()
                        self._bridge = GcsBridge(directory)
                        self._bridge.start()
                    if self._cancel.is_set():
                        StatusStore(directory.parent).request_finalization(run_id, "ABORTED", "prepared_cancelled")

                with (self.state_directory / "owner.log").open("w") as events:
                    self.controller.event_stream = events
                    result = self.controller.start(source, launch_mode="operator",
                        preparation_wall_seconds=preparation_wall_seconds,
                        output_root=self.project_directory / "runs", auxiliary_services=("gazebo-viewer",),
                        on_allocated=allocated)
                if self._bridge:
                    self._bridge.stop()
                remaining = self.runtime.remaining_resources()
                with self._lock:
                    self._record.update(state=result.state, reason=result.reason,
                        cleanup="complete" if not remaining else "incomplete", remaining_resources=remaining)
        except (OSError, ValueError, RuntimeError) as error:
            with self._lock:
                self._record.update(state="FAILED", error=str(error))
        finally:
            if self._bridge:
                self._bridge.stop()
            with self._lock:
                self._save()

    def _current(self, run_id: str) -> Path:
        _uuid(run_id)
        if run_id != self._record.get("run_id"):
            raise ValueError("action does not match the active run")
        return Path(self._record["run_directory"])

    def start_mission(self, *, run_id: str, request_id: str) -> dict[str, Any]:
        with self._lock:
            directory = self._current(run_id)
            from artifacts.operator_protocol import OperatorProtocol
            config = load_run_config(directory / "configuration/run.json")
            with OperatorProtocol(directory, run_id, config.config_sha256) as protocol:
                existing = protocol.read_start_request()
            if existing is None:
                snapshot = self.snapshot()
                if not snapshot.get("can_start"):
                    raise RuntimeError("mission start requires fresh preflight and connected operator views")
            result = self.controller.start_mission(run_id, directory.parent, request_id=_uuid(request_id))
            return result.to_dict()

    def abort_flight(self, *, run_id: str, request_id: str) -> dict[str, Any]:
        with self._lock:
            directory = self._current(run_id)
            return self.controller.abort_flight(run_id, directory.parent, request_id=_uuid(request_id)).to_dict()

    def stop(self, *, run_id: str | None = None) -> dict[str, Any]:
        with self._lock:
            if run_id is not None:
                self._current(run_id)
            self._cancel.set()
            if self._record.get("run_id") is not None:
                directory = Path(self._record["run_directory"])
                self.controller.abort(self._record["run_id"], directory.parent)
                if self._record["state"] == "ORPHANED":
                    self._worker = threading.Thread(target=self._cleanup_orphan, name="orphan-cleanup")
                    self._record["state"] = "FINALIZING"
                    self._worker.start()
            return self.snapshot()

    def _cleanup_orphan(self) -> None:
        """Retain an interrupted run; do not invent a terminal manifest or landing."""
        from ._adapters.compose import ComposeRuntime

        directory = Path(self._record["run_directory"])
        try:
            with workstation_lock():
                config = load_run_config(directory / "configuration/run.json")
                compose = ComposeRuntime(project_directory=self.project_directory, run_id=config.run_id,
                    run_directory=directory, config_path=directory / "configuration/run.json", topology=config.topology,
                    auxiliary_services=("gazebo-viewer",), base_environment={**os.environ, "SIM_COMPOSE_OVERLAY": "local"})
                deadline = time.monotonic() + config.finalization_wall_seconds
                while time.monotonic() < deadline and not (directory / ".status/artifacts-final.json").exists():
                    time.sleep(0.5)
                result = compose.down(20)
                remaining = self.runtime.remaining_resources()
                with self._lock:
                    self._record.update(state="INTERRUPTED", cleanup="complete" if result.returncode == 0 and not remaining else "incomplete",
                        remaining_resources=remaining, error="Owner was interrupted; retained evidence has no new terminal validity claim.")
        except (OSError, RuntimeError, ValueError) as error:
            with self._lock:
                self._record.update(state="ORPHANED", cleanup="incomplete", error=str(error))
        finally:
            with self._lock:
                self._save()

    def _views(self) -> dict[str, Any]:
        views: dict[str, Any] = {"qgc": self._bridge.status() if self._bridge else {"fresh": False},
                                 "gazebo": {"connected": False}, "camera": {"fresh": False}}
        for name, url in (("gazebo", "http://127.0.0.1:6080/vnc.html"), ("camera", "http://127.0.0.1:8081/status")):
            try:
                with urlopen(url, timeout=0.5) as response:
                    if name == "camera":
                        views[name] = json.loads(response.read(65536))
                    else:
                        views[name] = {"connected": response.status == 200}
            except (OSError, ValueError):
                pass
        return views

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._record)
            result.update(can_start=False, checks=self._checks)
            run_id = result.get("run_id")
            if run_id is None:
                return result
            directory = Path(result["run_directory"])
            try:
                status = self.controller.status(run_id, directory.parent)
                if result["state"] not in {"ORPHANED", "INTERRUPTED"} and self._active():
                    result["state"] = status.state
                    result["reason"] = status.reason
                from artifacts.operator_protocol import OperatorProtocol
                config = load_run_config(directory / "configuration/run.json")
                result["configuration"] = json.loads((directory / "configuration/run.json").read_text())
                with OperatorProtocol(directory, run_id, config.config_sha256) as protocol:
                    result["preflight"] = preflight_snapshot(protocol)
                    request = protocol.read_start_request()
                    admission = protocol.read_start_admission()
                    recovery = protocol.read_latest_flight_abort_status()
                    result["start_request"] = asdict(request) if request else None
                    result["admission"] = asdict(admission) if admission else None
                    result["recovery"] = asdict(recovery) if recovery else None
                    if request and result["state"] == "PREPARED":
                        result["state"] = "STARTING_MISSION"
                if self._active():
                    result["views"] = self._views()
                    views = result["views"]
                    result["can_start"] = (result["state"] == "PREPARED" and result["preflight"]["ready"]
                        and views["qgc"].get("qgc_fresh", False) and views["gazebo"]["connected"]
                        and views["camera"].get("fresh", False))
            except (OSError, ValueError, RuntimeError) as error:
                result["observation_error"] = str(error)
            return result

    def checks(self) -> dict[str, Any]:
        result = self.runtime.checks(active=self._active())
        with self._lock:
            self._checks = result
        return result

    def open_qgc(self) -> dict[str, Any]:
        self.runtime.open_qgc()
        return {"opened": True}

    def build(self) -> dict[str, Any]:
        with self._lock:
            if self._active() or self._record["state"] == "ORPHANED":
                raise RuntimeError("finish the active session before building")
            self._record["state"] = "BUILDING"
            self._worker = threading.Thread(target=self._build_worker, name="runtime-build")
            self._worker.start()
            return dict(self._record)

    def _build_worker(self) -> None:
        try:
            with workstation_lock():
                checks = self.runtime.build()
                with self._lock:
                    self._checks = checks
                    self._record.update(state="IDLE", error="")
        except (OSError, RuntimeError, ValueError) as error:
            with self._lock:
                self._record.update(state="FAILED", error=str(error))

    def results(self) -> dict[str, Any]:
        with self._lock:
            result: dict[str, Any] = {"cleanup": self._record.get("cleanup"), "manifest": None,
                                      "physical_outcome": None, "score": None}
            if self._record.get("run_id"):
                directory = Path(self._record["run_directory"])
                for key, relative in (("manifest", "manifest.json"), ("physical_outcome", ".status/mission-finished.json"),
                                      ("score", "scoring/result.json")):
                    source = directory / relative
                    if source.is_file():
                        result[key] = json.loads(source.read_text())
                result["interventions"] = _tail(directory / "logs/docker/qgc-interventions.jsonl")
            return result

    def logs(self) -> dict[str, str]:
        with self._lock:
            result = {"build": _tail(self.state_directory / "build.log"), "owner": _tail(self.state_directory / "owner.log")}
            if self._record.get("run_directory"):
                directory = Path(self._record["run_directory"])
                for name in ("companion", "gazebo", "orchestration"):
                    result[name] = _tail(directory / f"logs/{name}.jsonl")
            return result

    def shutdown(self) -> None:
        if self._active():
            self.stop()
            if self._worker:
                self._worker.join()
