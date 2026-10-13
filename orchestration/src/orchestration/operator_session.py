"""Persistent owner shared by browser and CLI operator actions."""

from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
import ntpath
import os
from pathlib import Path
import stat
import tempfile
import threading
import time
from typing import Any, TextIO
from urllib.request import urlopen
from uuid import UUID, uuid4

from artifacts.qgc_operator import MavlinkIdentity
from artifacts.runtime_status import MissionFinishedStatus

from .catalog import PresetCatalog, Preset
from .config import RunConfig, load_run_config
from .controller import RunController
from .gcs_bridge import GcsBridge
from .host_integration import HostIntegration, LaunchResult, resolve_host
from .local_runtime import LocalRuntime
from .operator_readiness import preflight_snapshot
from .qgc_actions import QgcActionStore
from .status_store import StatusStore
from .workstation import Workstation, select_workstation


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


class _HostEvidenceLog:
    _FIELDS = {
        "host": frozenset({"platform", "provider", "image_platform", "transport"}),
        "ownership": frozenset({"state"}),
        "qgc": frozenset({"executable", "settings_file", "save_path", "version"}),
        "helper": frozenset({"version", "source_sha256"}),
        "publication": frozenset({"native_path", "filename", "sha256"}),
        "bridge_observation": frozenset({
            "sitl_connected",
            "sitl_detail",
            "vehicle_telemetry_fresh",
            "qgc_connected",
            "qgc_detail",
            "qgc_traffic_fresh",
            "armed",
            "mode",
            "relative_altitude_m",
            "landed_state",
        }),
    }

    def __init__(self, run_directory: Path) -> None:
        directory = run_directory / "logs/docker"
        directory.mkdir(mode=0o755, parents=True, exist_ok=True)
        metadata = directory.lstat()
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
            raise RuntimeError("host evidence directory is unsafe")
        self.path = directory / "host-integration.jsonl"
        self._stream: TextIO | None = self.path.open("x", encoding="utf-8")
        os.chmod(self.path, 0o600)

    def write(self, event: str, **fields: Any) -> None:
        if self._stream is None:
            raise ValueError("host evidence log is closed")
        expected = self._FIELDS.get(event)
        if expected is None or frozenset(fields) != expected:
            raise ValueError("host evidence fields are not allowlisted")
        document = {"event": event, **fields}
        self._stream.write(json.dumps(document, allow_nan=False, separators=(",", ":")) + "\n")
        self._stream.flush()
        os.fsync(self._stream.fileno())

    def write_bridge_observation(self, status: dict[str, Any]) -> None:
        armed = status.get("armed")
        mode = status.get("mode")
        altitude = status.get("relative_altitude_m")
        landed_state = status.get("landed_state")
        self.write(
            "bridge_observation",
            sitl_connected=status.get("connected") is True,
            sitl_detail=str(status.get("detail", "")),
            vehicle_telemetry_fresh=status.get("fresh") is True,
            qgc_connected=status.get("qgc_connected") is True,
            qgc_detail=str(status.get("qgc_detail", "")),
            qgc_traffic_fresh=status.get("qgc_fresh") is True,
            armed=armed if type(armed) is bool else None,
            mode=mode if isinstance(mode, str) else None,
            relative_altitude_m=(
                altitude
                if isinstance(altitude, (int, float)) and not isinstance(altitude, bool)
                else None
            ),
            landed_state=(
                landed_state
                if isinstance(landed_state, int) and not isinstance(landed_state, bool)
                else None
            ),
        )

    def close(self) -> None:
        stream, self._stream = self._stream, None
        if stream is not None:
            stream.flush()
            os.fsync(stream.fileno())
            stream.close()


class OperatorSession:
    def __init__(self, project_directory: Path, *, state_directory: Path | None = None,
                 runtime: Any = None, controller: Any = None,
                 qgc_action_store: QgcActionStore | None = None,
                 host: HostIntegration | None = None,
                 workstation: Workstation | None = None) -> None:
        self.project_directory = project_directory.resolve()
        selected_workstation = select_workstation(workstation, runtime, controller)
        self.host = (
            host
            or getattr(selected_workstation, "host", None)
            or resolve_host(self.project_directory)
        )
        self.workstation = selected_workstation or Workstation(self.host)
        self.state_directory = state_directory or self.project_directory / ".local-runtime"
        self.state_directory.mkdir(parents=True, exist_ok=True)
        self.catalog = PresetCatalog(self.project_directory)
        self.runtime = runtime or LocalRuntime(
            self.project_directory,
            host=self.host,
            workstation=self.workstation,
        )
        self.controller = controller or RunController(
            project_directory=self.project_directory,
            workstation=self.workstation,
        )
        self.qgc_action_store = qgc_action_store or QgcActionStore(
            self.host.state_directory,
            MavlinkIdentity(255, 190),
        )
        self._lock = threading.RLock()
        self._worker: threading.Thread | None = None
        self._cancel = threading.Event()
        self._bridge: GcsBridge | None = None
        self._host_evidence: _HostEvidenceLog | None = None
        self._desktop_observation: tuple[str, bool, float] | None = None
        self._record: dict[str, Any] = {"state": "IDLE", "run_id": None, "preset_id": None,
                                         "prepare_request_id": None, "error": "", "cleanup": "not started"}
        self._checks: dict[str, Any] | None = None
        self._checks_context: tuple[int, bool] | None = None
        self._owner_generation = 0
        self._console_url: str | None = None
        self._browser_launch = LaunchResult(False, "Browser launch has not been requested")
        receipt = self.state_directory / "session.json"
        if receipt.exists():
            previous = json.loads(receipt.read_text())
            if previous.get("run_id"):
                _uuid(previous["run_id"])
                self._record = previous
                run_directory = previous.get("run_directory")
                if run_directory is None:
                    self._record["state"] = "ORPHANED"
                else:
                    directory = Path(run_directory)
                    if directory != self.project_directory / "runs" / previous["run_id"]:
                        raise ValueError("session receipt belongs to another output directory")
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

    def set_console_status(self, url: str, launch: LaunchResult) -> None:
        with self._lock:
            self._console_url = url
            self._browser_launch = launch

    def _begin_owner(self) -> None:
        """Invalidate prerequisite observations made for an earlier owner."""
        self._owner_generation += 1
        self._checks = None
        self._checks_context = None

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
        parameters = {"preset_id": preset_id, "preparation_wall_seconds": preparation_wall_seconds,
                      "calibration_source": str(Path(calibration_source).expanduser().resolve()) if calibration_source else None}
        with self._lock:
            if self._record.get("prepare_request_id") == request_id:
                if parameters != self._record.get("prepare_parameters"):
                    raise ValueError("prepare request identity belongs to different parameters")
                return self.snapshot()
            if self._active() or self._record["state"] == "ORPHANED":
                raise RuntimeError("a session is active; stop and finalize it before preparing another")
            if self._record.get("run_id") and self._record.get("cleanup") != "complete":
                raise RuntimeError("previous run cleanup must complete before preparing another")
            self._begin_owner()
            self._cancel.clear()
            self._record = {"state": "PREPARING", "run_id": None, "preset_id": preset_id,
                            "prepare_request_id": request_id, "error": "", "cleanup": "pending",
                            "prepare_parameters": parameters,
                            "preparation_wall_seconds": preparation_wall_seconds, "owner_pid": os.getpid()}
            self._save()
            self._worker = threading.Thread(target=self._prepare_worker,
                args=(preset, preparation_wall_seconds, calibration_source), name="simulation-owner")
            self._worker.start()
            return dict(self._record)

    def _prepare_worker(self, preset: Preset, preparation_wall_seconds: int, calibration_source: str | None) -> None:
        try:
            with self.workstation.hold():
                checks = self.runtime.checks()
                with self._lock:
                    self._checks = checks
                    self._checks_context = (self._owner_generation, False)
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
                def identified(run_id: str) -> None:
                    with self._lock:
                        self._record.update(run_id=run_id)
                        self._save()

                def allocated(run_id: str, directory: Path) -> None:
                    with self._lock:
                        self._record.update(run_id=run_id, run_directory=str(directory))
                        self._save()
                    if self._cancel.is_set():
                        StatusStore(directory.parent).request_finalization(run_id, "ABORTED", "prepared_cancelled")

                def configuration_ready(config: RunConfig, _directory: Path) -> None:
                    material = config.operator_actions
                    if material is None:
                        raise RuntimeError("operator preparation did not allocate QGC actions")
                    evidence = _HostEvidenceLog(_directory)
                    facts = self.host.facts
                    evidence.write(
                        "host",
                        platform=facts.kind,
                        provider=facts.provider,
                        image_platform=facts.image_platform,
                        transport=(
                            "windows-helper-udp"
                            if facts.kind == "wsl2"
                            else "loopback-udp"
                        ),
                    )
                    evidence.write("ownership", state="active")
                    self._host_evidence = evidence
                    installation = self.host.inspect_qgc()
                    publication = self.host.publish_actions(
                        material,
                        installation,
                        self.workstation.owned(),
                    )
                    expected_sha256 = hashlib.sha256(material.action_bytes).hexdigest()
                    expected_native_path = (
                        ntpath.join(
                            installation.save_path,
                            "MavlinkActions",
                            material.published_name,
                        )
                        if self.host.facts.kind == "wsl2"
                        else str(
                            Path(installation.save_path)
                            / "MavlinkActions"
                            / material.published_name
                        )
                    )
                    if (
                        publication.filename != material.published_name
                        or publication.sha256 != expected_sha256
                        or publication.sha256 != material.descriptor.actions_sha256
                    ):
                        raise RuntimeError(
                            "native QGroundControl action publication does not match the bundled material"
                        )
                    if publication.native_path != expected_native_path:
                        raise RuntimeError(
                            "native QGroundControl action publication returned an unexpected native path"
                        )
                    if self._host_evidence is None:
                        raise RuntimeError("host evidence log was not created for the allocated run")
                    self._host_evidence.write(
                        "qgc",
                        executable=installation.executable,
                        settings_file=installation.settings_file,
                        save_path=installation.save_path,
                        version=installation.version,
                    )
                    helper_info = self.host.helper_info()
                    if helper_info is not None:
                        self._host_evidence.write(
                            "helper",
                            version=helper_info.version,
                            source_sha256=helper_info.source_sha256,
                        )
                    self._host_evidence.write(
                        "publication",
                        native_path=publication.native_path,
                        filename=publication.filename,
                        sha256=publication.sha256,
                    )
                    with self._lock:
                        self._record.update(
                            qgc_action_path=publication.native_path,
                            qgc_action_filename=publication.filename,
                            qgc_action_sha256=publication.sha256,
                        )
                        self._save()
                    transport = self.host.open_datagrams(
                        self.workstation.owned(),
                        config.run_id,
                        publication,
                    )
                    try:
                        bridge = GcsBridge(_directory, transport=transport)
                        bridge.start()
                    except BaseException:
                        transport.close()
                        raise
                    with self._lock:
                        self._bridge = bridge
                    try:
                        launch = self.host.open_qgc(installation)
                    except (OSError, RuntimeError, ValueError) as error:
                        launch = LaunchResult(
                            False,
                            f"QGroundControl could not be opened automatically: {error}",
                        )
                    with self._lock:
                        self._record["qgc_launch"] = asdict(launch)
                        self._save()

                with (self.state_directory / "owner.log").open("w") as events:
                    def before_inventory() -> None:
                        failure: BaseException | None = None
                        with self._lock:
                            bridge = self._bridge
                            evidence = self._host_evidence
                        if bridge is not None:
                            try:
                                if evidence is None:
                                    raise RuntimeError(
                                        "host evidence log is unavailable for final bridge observation"
                                    )
                                # status() is an in-memory, lock-bounded projection. Capture it
                                # while the transport still reflects its final live state.
                                evidence.write_bridge_observation(bridge.status())
                            except BaseException as error:
                                failure = error
                            try:
                                bridge.stop()
                            except BaseException as error:
                                failure = failure or error
                        if evidence is not None:
                            try:
                                evidence.write(
                                    "ownership", state="cleanup_pending"
                                )
                            except BaseException as error:
                                failure = failure or error
                            try:
                                evidence.close()
                            except BaseException as error:
                                failure = failure or error
                        try:
                            events.flush()
                            events.close()
                        except BaseException as error:
                            failure = failure or error
                        if failure is not None:
                            raise RuntimeError(
                                "operator writers did not quiesce before inventory"
                            ) from failure

                    self.controller.event_stream = events
                    result = self.controller.start(source, launch_mode="operator",
                        preparation_wall_seconds=preparation_wall_seconds,
                        output_root=self.project_directory / "runs", auxiliary_services=("gazebo-viewer",),
                        on_run_identified=identified,
                        on_allocated=allocated,
                        on_configuration_ready=configuration_ready,
                        on_before_inventory=before_inventory,
                        operator_control_factory=self.qgc_action_store.allocate)
                if self._bridge:
                    self._bridge.stop()
                remaining = self.runtime.remaining_resources()
                with self._lock:
                    terminal_output_root = Path(self._record["run_directory"]).parent
                terminal_status = StatusStore(terminal_output_root).read_operator_status(
                    result.run_id
                )
                teardown_failed = any(
                    diagnostic.kind == "teardown"
                    for diagnostic in terminal_status.diagnostics
                )
                with self._lock:
                    self._record.update(state=result.state, reason=result.reason,
                        cleanup="complete" if not remaining and not teardown_failed else "incomplete",
                        remaining_resources=remaining)
        except (OSError, ValueError, RuntimeError) as error:
            with self._lock:
                update: dict[str, Any] = {"state": "FAILED", "error": str(error)}
                if self._record.get("run_id") is not None:
                    try:
                        remaining = self.runtime.remaining_resources()
                        cleanup_complete = False
                        run_directory = self._record.get("run_directory")
                        if run_directory is not None:
                            status = StatusStore(
                                Path(run_directory).parent
                            ).read_operator_status(self._record["run_id"])
                            cleanup_complete = (
                                status.state == "CREATED"
                                and not remaining
                                and not any(
                                    diagnostic.kind == "teardown"
                                    for diagnostic in status.diagnostics
                                )
                            )
                        update.update(
                            cleanup=(
                                "complete" if cleanup_complete else "incomplete"
                            ),
                            remaining_resources=remaining,
                        )
                    except (OSError, ValueError, RuntimeError) as cleanup_error:
                        update.update(
                            cleanup="incomplete",
                            error=f"{error}; cleanup inspection failed: {cleanup_error}",
                        )
                self._record.update(update)
        finally:
            if self._bridge:
                self._bridge.stop()
            if self._host_evidence is not None:
                self._host_evidence.close()
            with self._lock:
                self._save()

    def _validate_current_run(self, run_id: str) -> None:
        _uuid(run_id)
        if run_id != self._record.get("run_id"):
            raise ValueError("action does not match the active run")

    def _current(self, run_id: str) -> Path:
        self._validate_current_run(run_id)
        if self._record.get("run_directory") is None:
            raise RuntimeError("run directory was not allocated; cleanup recovery is required")
        return Path(self._record["run_directory"])

    def start_mission(self, *, run_id: str, request_id: str) -> dict[str, Any]:
        with self._lock:
            directory = self._current(run_id)
            from artifacts.operator_protocol import OperatorProtocol
            config = load_run_config(directory / "configuration/run.json")
            if config.operator_control is not None:
                raise RuntimeError(
                    "descriptor-backed runs accept flight actions only in QGroundControl"
                )
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
            config = load_run_config(directory / "configuration/run.json")
            if config.operator_control is not None:
                raise RuntimeError(
                    "descriptor-backed runs accept flight actions only in QGroundControl"
                )
            return self.controller.abort_flight(run_id, directory.parent, request_id=_uuid(request_id)).to_dict()

    def stop(self, *, run_id: str | None = None) -> dict[str, Any]:
        with self._lock:
            if run_id is not None:
                self._validate_current_run(run_id)
            self._cancel.set()
            if self._record.get("run_id") is not None:
                run_directory = self._record.get("run_directory")
                if run_directory is not None:
                    directory = Path(run_directory)
                    status = self.controller.status(self._record["run_id"], directory.parent)
                    reason = ("prepared_cancelled" if status.state in {"CREATED", "STARTING", "READY", "PREPARED"}
                              else "simulation_stop")
                    self.controller.abort(self._record["run_id"], directory.parent, reason=reason)
                if (
                    not self._active()
                    and self._record["state"] in {"ORPHANED", "FAILED"}
                    and self._record.get("cleanup") != "complete"
                ):
                    self._begin_owner()
                    self._worker = threading.Thread(target=self._cleanup_orphan, name="orphan-cleanup")
                    self._record["state"] = "FINALIZING"
                    self._worker.start()
            return self.snapshot()

    def _cleanup_orphan(self) -> None:
        """Retain an interrupted run; do not invent a terminal manifest or landing."""
        from ._adapters.compose import ComposeRuntime

        run_id = self._record["run_id"]
        run_directory = self._record.get("run_directory")
        try:
            with self.workstation.hold(recovery_run_id=run_id) as lease:
                if run_directory is None:
                    lease.finish_run(run_id, verified_cleanup=True)
                    with self._lock:
                        self._record.update(
                            state="INTERRUPTED",
                            cleanup="complete",
                            remaining_resources=[],
                            error="Owner stopped before run-directory allocation; native ownership was cleared after cleanup-only recovery.",
                        )
                    return
                directory = Path(run_directory)
                config = load_run_config(directory / "configuration/run.json")
                lifecycle = StatusStore(directory.parent).read_operator_status(run_id)
                failed_before_compose = lifecycle.state == "CREATED"
                compose = ComposeRuntime(project_directory=self.project_directory, run_id=config.run_id,
                    run_directory=directory, config_path=directory / "configuration/run.json", topology=config.topology,
                    auxiliary_services=("gazebo-viewer",), base_environment={**os.environ, "SIM_COMPOSE_OVERLAY": "local"})
                deadline = time.monotonic() + config.finalization_wall_seconds
                while (not failed_before_compose and time.monotonic() < deadline
                       and not (directory / ".status/artifacts-final.json").exists()):
                    time.sleep(0.5)
                result = compose.down(20)
                remaining = self.runtime.remaining_resources()
                project = compose.ps(5)
                try:
                    project_rows = json.loads(project.output or b"[]")
                except (UnicodeError, json.JSONDecodeError) as error:
                    raise RuntimeError("could not verify exact run resource cleanup") from error
                cleanup_verified = (
                    result.returncode == 0
                    and project.returncode == 0
                    and project_rows == []
                    and not remaining
                    and (
                        failed_before_compose
                        or compose.media_cleanup_verified(5)
                    )
                )
                if cleanup_verified:
                    lease.finish_run(run_id, verified_cleanup=True)
                with self._lock:
                    self._record.update(state="INTERRUPTED", cleanup="complete" if cleanup_verified else "incomplete",
                        remaining_resources=remaining, error="Owner was interrupted; retained evidence has no new terminal validity claim.")
        except (OSError, RuntimeError, ValueError) as error:
            with self._lock:
                self._record.update(state="ORPHANED", cleanup="incomplete", error=str(error))
        finally:
            with self._lock:
                self._save()

    def _views(self) -> dict[str, Any]:
        views: dict[str, Any] = {"qgc": self._bridge.status() if self._bridge else {"fresh": False},
                                 "gazebo": {"available": False, "connected": False}, "camera": {"fresh": False}}
        for name, url in (("gazebo", "http://127.0.0.1:6080/vnc.html"), ("camera", "http://127.0.0.1:8081/status")):
            try:
                with urlopen(url, timeout=0.5) as response:
                    if name == "camera":
                        views[name] = json.loads(response.read(65536))
                    else:
                        observation = self._desktop_observation
                        connected = (observation is not None and observation[0] == self._record.get("run_id")
                            and observation[1] and time.monotonic() - observation[2] < 5)
                        views[name] = {"available": response.status == 200,
                                       "connected": response.status == 200 and connected}
            except (OSError, ValueError):
                pass
        return views

    def viewer_status(self, *, run_id: str, connected: bool) -> dict[str, bool]:
        if type(connected) is not bool:
            raise ValueError("desktop connection state must be boolean")
        with self._lock:
            self._current(run_id)
            self._desktop_observation = (run_id, connected, time.monotonic())
        return {"observed": True}

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._record)
            facts = getattr(self.host, "facts", None)
            result.update(
                can_start=False,
                qgc_admission_ready=False,
                checks=self._checks,
                console_url=self._console_url,
                browser_launch=asdict(self._browser_launch),
                host=(
                    {
                        "kind": facts.kind,
                        "machine": facts.machine,
                        "image_platform": facts.image_platform,
                        "provider": facts.provider,
                    }
                    if facts is not None
                    else {}
                ),
            )
            run_id = result.get("run_id")
            if run_id is None:
                return result
            if result.get("run_directory") is None:
                return result
            directory = Path(result["run_directory"])
            try:
                status = self.controller.status(run_id, directory.parent)
                if result["state"] not in {"ORPHANED", "INTERRUPTED", "BUILDING"} and self._active():
                    result["state"] = status.state
                    result["reason"] = status.reason
                from artifacts.operator_protocol import OperatorProtocol
                config = load_run_config(directory / "configuration/run.json")
                result["configuration"] = json.loads((directory / "configuration/run.json").read_text())
                with OperatorProtocol(directory, run_id, config.config_sha256) as protocol:
                    result["preflight"] = preflight_snapshot(protocol)
                    request = protocol.read_start_request()
                    readiness = protocol.read_start_readiness()
                    admission = protocol.read_start_admission()
                    recovery = protocol.read_latest_flight_abort_status()
                    result["start_request"] = asdict(request) if request else None
                    result["readiness"] = asdict(readiness) if readiness else None
                    result["admission"] = asdict(admission) if admission else None
                    result["recovery"] = asdict(recovery) if recovery else None
                    if request and result["state"] == "PREPARED":
                        result["state"] = "STARTING_MISSION"
                if self._active():
                    result["views"] = self._views()
                    views = result["views"]
                    preflight_ready = (
                        result["state"] == "PREPARED" and result["preflight"]["ready"]
                    )
                    result["qgc_admission_ready"] = preflight_ready
                    if config.operator_control is None:
                        result["can_start"] = (
                            preflight_ready
                            and views["qgc"].get("qgc_fresh", False)
                            and views["gazebo"]["connected"]
                            and views["camera"].get("fresh", False)
                        )
                mission = StatusStore(directory.parent).read_runtime_status(
                    run_id, MissionFinishedStatus
                )
                result["mission"] = asdict(mission) if mission else None
            except (OSError, ValueError, RuntimeError) as error:
                result["observation_error"] = str(error)
            return result

    def checks(self) -> dict[str, Any]:
        while True:
            with self._lock:
                generation = self._owner_generation
                active = self._active()
            result = self.runtime.checks(active=active)
            with self._lock:
                if generation == self._owner_generation and active == self._active():
                    self._checks = result
                    self._checks_context = (generation, active)
                    return result
                if (
                    self._checks is not None
                    and self._checks_context
                    == (self._owner_generation, self._active())
                ):
                    return self._checks

    def open_qgc(self) -> dict[str, Any]:
        try:
            result = self.host.open_qgc(self.host.inspect_qgc())
        except (OSError, RuntimeError, ValueError) as error:
            result = LaunchResult(
                False,
                f"QGroundControl could not be opened automatically: {error}",
            )
        document = asdict(result)
        with self._lock:
            self._record["qgc_launch"] = document
            self._save()
        return document

    def build(self) -> dict[str, Any]:
        with self._lock:
            if self._active() or self._record["state"] == "ORPHANED":
                raise RuntimeError("finish the active session before building")
            self._begin_owner()
            self._record["state"] = "BUILDING"
            self._worker = threading.Thread(target=self._build_worker, name="runtime-build")
            self._worker.start()
            return dict(self._record)

    def _build_worker(self) -> None:
        try:
            checks = self.runtime.build()
            with self._lock:
                self._checks = checks
                self._checks_context = (self._owner_generation, False)
                self._record.update(state="IDLE", error="")
        except (OSError, RuntimeError, ValueError) as error:
            with self._lock:
                self._record.update(state="FAILED", error=str(error))

    def results(self) -> dict[str, Any]:
        with self._lock:
            result: dict[str, Any] = {"cleanup": self._record.get("cleanup"), "manifest": None,
                                      "physical_outcome": None, "score": None}
            if self._record.get("run_id") and self._record.get("run_directory"):
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
