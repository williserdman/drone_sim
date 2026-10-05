"""Mission-suite catalog, temporary templates, and durable reports."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
from typing import Any
from uuid import uuid4


_ACCEPTANCE_CONTRACTS = {
    "calibration",
    "reload_validation",
    "configured_descent",
    "operator_wait",
    "controlled_descent",
    "hover_roll",
    "autotune_roll",
    "moving_pad",
    "competition",
}
_CASE_NAME = re.compile(r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
_FLIGHT_FIELDS = {"world", "vehicle", "mission"}
_REQUIRED_CASES = (
    ("calibration", "config/autotune-run.json", "calibration"),
    ("reload-validation", "config/calibration-validation-run.json", "reload_validation"),
    ("configured-descent", "config/configured-descent-run.json", "configured_descent"),
    ("operator-wait", "config/configured-operator-run.json", "operator_wait"),
    ("controlled-descent", "config/vertical-descent-run.json", "controlled_descent"),
    ("hover-roll", "config/hover-roll-run.json", "hover_roll"),
    ("autotune-roll", "config/autotune-roll-run.json", "autotune_roll"),
    ("moving-pad", "config/configured-moving-pad-run.json", "moving_pad"),
    ("stationary-pad", "tests/fixtures/configured-stationary-pad-run.json", "moving_pad"),
    ("competition-slow", "config/default-run.json", "competition"),
    ("competition-realtime", "config/realtime-run.json", "competition"),
)


@dataclass(frozen=True)
class SuiteCase:
    name: str
    template: Path
    acceptance: str


def _read_json_object(path: Path, description: str) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError(f"invalid {description}: {path}") from error
    if not isinstance(document, dict):
        raise ValueError(f"{description} must be a JSON object")
    return document


def _flight_templates(project_directory: Path) -> set[Path]:
    templates: set[Path] = set()
    config_directory = project_directory / "config"
    for path in config_directory.glob("*.json"):
        document = _read_json_object(path, "configuration")
        if _FLIGHT_FIELDS <= document.keys():
            templates.add(path.resolve())
    stationary = project_directory / "tests/fixtures/configured-stationary-pad-run.json"
    if stationary.is_file():
        templates.add(stationary.resolve())
    return templates


def load_suite_catalog(
    path: Path, *, project_directory: Path
) -> tuple[SuiteCase, ...]:
    """Load and validate the ordered suite catalog and flight-template coverage."""
    project_directory = project_directory.resolve()
    document = _read_json_object(Path(path), "suite catalog")
    if set(document) != {"schema_version", "cases"}:
        raise ValueError("suite catalog has missing or unknown keys")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        raise ValueError("suite catalog schema_version must be 1")
    entries = document["cases"]
    if not isinstance(entries, list):
        raise ValueError("suite catalog cases must be an array")

    cases: list[SuiteCase] = []
    names: set[str] = set()
    templates: set[Path] = set()
    for index, entry in enumerate(entries):
        if not isinstance(entry, dict) or set(entry) != {"name", "template", "acceptance"}:
            raise ValueError(f"suite catalog case {index} has missing or unknown keys")
        name = entry["name"]
        template_value = entry["template"]
        acceptance = entry["acceptance"]
        if not isinstance(name, str) or not _CASE_NAME.fullmatch(name):
            raise ValueError(f"suite catalog case {index} has an invalid name")
        if name in names:
            raise ValueError(f"suite catalog contains duplicate name: {name}")
        if not isinstance(template_value, str) or not template_value:
            raise ValueError(f"suite catalog case {name} has an invalid template")
        relative = Path(template_value)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"suite catalog case {name} template must be project-relative")
        template = (project_directory / relative).resolve()
        try:
            template.relative_to(project_directory)
        except ValueError as error:
            raise ValueError(f"suite catalog case {name} template escapes the project") from error
        if not template.is_file():
            raise ValueError(f"suite catalog case {name} template does not exist: {relative}")
        if template in templates:
            raise ValueError(f"suite catalog contains duplicate template: {relative}")
        if not isinstance(acceptance, str) or acceptance not in _ACCEPTANCE_CONTRACTS:
            raise ValueError(f"suite catalog case {name} has an invalid acceptance contract")
        names.add(name)
        templates.add(template)
        cases.append(SuiteCase(name, template, acceptance))

    actual = tuple(
        (case.name, case.template.relative_to(project_directory).as_posix(), case.acceptance)
        for case in cases
    )
    if actual != _REQUIRED_CASES:
        raise ValueError("suite catalog must contain the required cases in order")

    missing = _flight_templates(project_directory) - templates
    extra = templates - _flight_templates(project_directory)
    if missing:
        display = ", ".join(str(item.relative_to(project_directory)) for item in sorted(missing))
        raise ValueError(f"suite catalog is missing flight templates: {display}")
    if extra:
        display = ", ".join(str(item.relative_to(project_directory)) for item in sorted(extra))
        raise ValueError(f"suite catalog contains non-flight templates: {display}")
    return tuple(cases)


def prepare_suite_template(
    case: SuiteCase,
    *,
    suite_directory: Path,
    output_root: Path,
    calibration_source: Path | None,
) -> Path:
    """Relocate one template without changing its mission behavior."""
    source = case.template.resolve()
    document = _read_json_object(source, "run template")
    competition = document.get("competition")
    competition_payloads: dict[str, bytes] = {}
    if competition is not None:
        if not isinstance(competition, dict):
            raise ValueError("run template competition must be an object")
        for key in ("course", "scenario"):
            value = competition.get(key)
            if not isinstance(value, str):
                raise ValueError(f"run template competition.{key} must be a path")
            referenced = Path(value)
            if not referenced.is_absolute():
                referenced = source.parent / referenced
            referenced = referenced.resolve()
            try:
                competition_payloads[f"{key}.yaml"] = referenced.read_bytes()
            except OSError as error:
                raise ValueError(
                    f"could not read run template competition.{key}: {referenced}"
                ) from error
            competition[key] = f"{key}.yaml"

    calibration = document.get("calibration")
    if calibration is not None:
        if not isinstance(calibration, dict):
            raise ValueError("run template calibration must be an object")
        value = calibration.get("source_run_directory")
        if not isinstance(value, str):
            raise ValueError("run template calibration.source_run_directory must be a path")
        referenced = Path(value)
        if not referenced.is_absolute():
            referenced = source.parent / referenced
        calibration["source_run_directory"] = str(referenced.resolve())

    document["output_root"] = str(output_root.resolve())
    if calibration_source is not None:
        document["calibration"] = {
            "source_run_directory": str(calibration_source.resolve())
        }

    configuration_directory = Path(suite_directory) / "configuration" / case.name
    configuration_directory.mkdir(parents=True, exist_ok=True)
    for name, payload in competition_payloads.items():
        _atomic_write(configuration_directory / name, payload)
    destination = configuration_directory / "run.json"
    _atomic_write(destination, _json_bytes(document))
    return destination


def _json_bytes(document: Mapping[str, object]) -> bytes:
    try:
        return (json.dumps(dict(document), allow_nan=False, indent=2, sort_keys=True) + "\n").encode()
    except (TypeError, ValueError) as error:
        raise ValueError(f"suite report is not valid JSON: {error}") from error


def _atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.parent / f".{path.name}.{uuid4().hex}.tmp"
    try:
        with temporary.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _score_text(score: object) -> str:
    if not isinstance(score, Mapping):
        return "unknown"
    earned = score.get("earned", score.get("score"))
    maximum = score.get("maximum", score.get("maximum_score"))
    return "unknown" if earned is None or maximum is None else f"{earned}/{maximum}"


def _report_markdown(report: Mapping[str, object]) -> str:
    lines = [
        f"# Mission suite {report.get('suite_id', 'unknown')}",
        "",
        f"State: {report.get('state', 'unknown')}",
        f"Reason: {report.get('reason') or 'none'}",
        "",
        "| Case | Status | Lifecycle | Physical | Score | Acceptance | Bundle | Reason |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    cases = report.get("cases", [])
    if isinstance(cases, list):
        for case in cases:
            if not isinstance(case, Mapping):
                continue
            acceptance = case.get("artifact_acceptance")
            accepted = "unknown"
            acceptance_reason = ""
            if isinstance(acceptance, Mapping):
                flag = acceptance.get("accepted")
                accepted = "accepted" if flag is True else "rejected" if flag is False else "unknown"
                acceptance_reason = str(acceptance.get("reason") or "")
            bundle = case.get("bundle_path")
            bundle_text = "unknown" if not bundle else f"[{bundle}]({bundle})"
            reasons = [
                str(value)
                for value in (case.get("reason"), acceptance_reason)
                if value
            ]
            reason = "; ".join(dict.fromkeys(reasons))
            cells = (
                case.get("name"), case.get("status"), case.get("lifecycle"),
                case.get("physical_outcome"), _score_text(case.get("score")),
                accepted, bundle_text, reason,
            )
            lines.append("| " + " | ".join(str(value if value is not None else "unknown").replace("|", "\\|") for value in cells) + " |")
    return "\n".join(lines) + "\n"


def write_suite_report(
    suite_directory: Path, report: Mapping[str, object]
) -> None:
    """Atomically publish machine-readable and derived human-readable reports."""
    if not isinstance(report, Mapping):
        raise ValueError("suite report must be a mapping")
    suite_directory = Path(suite_directory)
    _atomic_write(suite_directory / "report.json", _json_bytes(report))
    _atomic_write(
        suite_directory / "report.md", _report_markdown(report).encode("utf-8")
    )


@dataclass(frozen=True)
class SuiteResult:
    suite_id: str
    state: str
    report_path: Path
    exit_code: int

    def to_dict(self) -> dict[str, object]:
        return {'result_type': 'suite_result', 'suite_id': self.suite_id,
                'state': self.state, 'report_path': str(self.report_path),
                'exit_code': self.exit_code}


class SuiteRunner:
    def __init__(self, *, project_directory: Path, controller, runtime=None, event_stream) -> None:
        from ._adapters.suite import SuiteRuntime
        self.project_directory = project_directory.resolve()
        self.controller = controller
        self.runtime = runtime or SuiteRuntime(project_directory=self.project_directory, controller=controller)
        self.event_stream = event_stream
        self._interrupted = False
        self._active = None
        self._abort_requested = False

    def _interrupt(self, signum, frame):
        self._interrupted = True
        if self._active is not None and not self._abort_requested:
            self._abort_requested = True
            run_id, directory = self._active
            try:
                self.controller.abort(run_id, directory.parent)
            except Exception as error:
                self.event_stream.write(f'suite abort request failed: {error}\n')
                self.event_stream.flush()

    def _allocated(self, run_id, directory):
        self._active = (run_id, directory)
        if self._interrupted:
            self._interrupt(None, None)

    def run(self, catalog_path: Path, *, output_root: Path) -> SuiteResult:
        import hashlib
        import signal
        import threading
        from datetime import datetime, timezone
        from .status_store import StatusStore
        from dataclasses import asdict

        def now(): return datetime.now(timezone.utc).isoformat()
        output_root = output_root.resolve()
        suite_id = str(uuid4())
        directory = output_root / 'suites' / suite_id
        directory.mkdir(parents=True)
        report = {'schema_version':1, 'suite_id':suite_id, 'state':'RUNNING', 'reason':None,
                  'started_at':now(), 'finished_at':None, 'source_revisions':[], 'image_digests':[],
                  'catalog_sha256':None, 'template_sha256s':{}, 'calibration_run_id':None,
                  'reload_validation_run_id':None, 'cases':[]}
        self._interrupted = self._abort_requested = False
        self._active = None
        previous_handlers = {}
        if threading.current_thread() is threading.main_thread():
            for signum in (signal.SIGINT, signal.SIGTERM):
                previous_handlers[signum] = signal.signal(signum, self._interrupt)
        exit_code = 2
        try:
            cases = load_suite_catalog(catalog_path, project_directory=self.project_directory)
            report['catalog_sha256'] = hashlib.sha256(catalog_path.read_bytes()).hexdigest()
            report['template_sha256s'] = {case.name:hashlib.sha256(case.template.read_bytes()).hexdigest() for case in cases}
            report['cases'] = [dict(name=case.name,template=case.template.relative_to(self.project_directory).as_posix(),
                acceptance_contract=case.acceptance,status='unrun',run_id=None,lifecycle=None,reason=None,
                physical_outcome=None,score=None,artifact_acceptance={'accepted':None,'reason':None,'report':None},
                teardown_diagnostics=[],bundle_path=None) for case in cases]
            write_suite_report(directory, report)
            with self.runtime.lock():
                frozen = self.runtime.build_and_freeze()
                report['source_revisions'] = [row.to_dict() for row in frozen.source_revisions]
                report['image_digests'] = [row.to_dict() for row in frozen.image_digests]
                calibration_source = None
                blocked_reason = None
                for case, row in zip(cases, report['cases'], strict=True):
                    if self._interrupted:
                        break
                    if blocked_reason:
                        row.update(status='blocked', reason=blocked_reason)
                        continue
                    try:
                        self.runtime.assert_unchanged(frozen)
                    except Exception as error:
                        blocked_reason = str(error)
                        row.update(status='blocked', reason=blocked_reason)
                        continue
                    template = prepare_suite_template(case, suite_directory=directory,
                        output_root=output_root, calibration_source=calibration_source)
                    self.event_stream.write(f'suite {suite_id}: starting {case.name}\n')
                    self.event_stream.flush()
                    self._abort_requested = False
                    try:
                        result = self.controller.start(template,
                            auxiliary_services=('operator-wait-runtime',) if case.acceptance=='operator_wait' else (),
                            on_allocated=self._allocated)
                        row.update(run_id=result.run_id, lifecycle=result.state, reason=result.reason)
                        run_directory = output_root / result.run_id
                        if result.state != 'COMPLETED':
                            raise RuntimeError(f'run lifecycle {result.state}: {result.reason}')
                        accepted = self.runtime.inspect_run(case, run_directory, frozen)
                        row.update(status='passed', physical_outcome='LANDED')
                        row['artifact_acceptance'] = {'accepted':True,'reason':None,'report':asdict(accepted)}
                    except Exception as error:
                        row.update(status='failed', reason=str(error))
                        row['artifact_acceptance'] = {'accepted':False,'reason':str(error),'report':None}
                    finally:
                        if self._active is not None:
                            run_id, run_directory = self._active
                            row['run_id'] = run_id
                            row['bundle_path'] = os.path.relpath(run_directory, directory)
                            manifest_path = run_directory / 'manifest.json'
                            if manifest_path.is_file():
                                try:
                                    manifest = _read_json_object(manifest_path, 'manifest')
                                    row['lifecycle'] = manifest.get('terminal_status',row['lifecycle'])
                                    row['score'] = manifest.get('scoring')
                                except ValueError as error:
                                    row.update(status='failed', reason=str(error), physical_outcome=None)
                                    row['artifact_acceptance'] = {'accepted':False,'reason':str(error),'report':None}
                            try:
                                status = StatusStore(output_root).read_operator_status(run_id)
                                diagnostics = [value.to_dict() for value in status.diagnostics]
                                row['teardown_diagnostics'] = diagnostics
                                if any(value['kind']=='teardown' for value in diagnostics):
                                    blocked_reason = 'current run teardown was not confirmed'
                                    row.update(status='failed',reason=blocked_reason)
                            except Exception as error:
                                blocked_reason = f'current run teardown status unavailable: {error}'
                                row.update(status='failed',reason=blocked_reason)
                        self._active = None
                    if row['status']=='passed':
                        if case.acceptance=='calibration':
                            calibration_source = output_root / row['run_id']
                            report['calibration_run_id'] = row['run_id']
                        elif case.acceptance=='reload_validation':
                            report['reload_validation_run_id'] = row['run_id']
                    elif case.acceptance in {'calibration','reload_validation'}:
                        blocked_reason = f'{case.name} gate failed: {row["reason"]}'
                    if not self._interrupted:
                        try:
                            self.runtime.assert_unchanged(frozen)
                        except Exception as error:
                            blocked_reason = str(error)
                            row.update(status='failed',reason=blocked_reason)
                    write_suite_report(directory, report)
                if self._interrupted:
                    report.update(state='INTERRUPTED',reason='operator interrupted suite')
                    exit_code = 130
                else:
                    passed = all(row['status']=='passed' for row in report['cases'])
                    report.update(state='PASSED' if passed else 'FAILED',reason=None if passed else 'one or more required cases failed or did not run')
                    exit_code = 0 if passed else 1
        except Exception as error:
            report.update(state='INTERRUPTED' if self._interrupted else 'SETUP_FAILED',reason=str(error))
            exit_code = 130 if self._interrupted else 2
        finally:
            report['finished_at'] = now()
            try:
                write_suite_report(directory, report)
            finally:
                for signum, handler in previous_handlers.items():
                    signal.signal(signum, handler)
        return SuiteResult(suite_id,report['state'],directory/'report.json',exit_code)
