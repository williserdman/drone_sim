import hashlib
import json
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
from uuid import UUID

import pytest
from jsonschema import Draft202012Validator, FormatChecker, ValidationError

from orchestration.config import (
    RecordingConfig,
    RunConfig,
    RunTemplate,
    load_run_config,
    resolve_run_config,
    write_resolved_config,
)


ROOT = Path(__file__).parents[1]
CONFIG = (ROOT / "../config").resolve()
DEFAULT_TEMPLATE = CONFIG / "default-run.json"
FIXED_RUN_ID = UUID("00000000-0000-4000-8000-000000000222")


def _load_validator(name: str) -> Draft202012Validator:
    schema = json.loads((CONFIG / name).read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema, format_checker=FormatChecker())


def _resolved_document() -> dict:
    template = json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
    document = {
        "run_id": str(FIXED_RUN_ID),
        **template,
        "output_root": str((ROOT / "../runs").resolve()),
    }
    canonical = json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
    document["config_sha256"] = hashlib.sha256(canonical).hexdigest()
    return document


def test_resolve_default_template_returns_frozen_phase_2_configuration():
    calls = 0

    def fixed_uuid() -> UUID:
        nonlocal calls
        calls += 1
        return FIXED_RUN_ID

    resolved = resolve_run_config(DEFAULT_TEMPLATE, run_id_factory=fixed_uuid)

    assert isinstance(resolved, RunConfig)
    assert resolved.run_id == "00000000-0000-4000-8000-000000000222"
    assert resolved.recording == RecordingConfig(
        width_px=320,
        height_px=240,
        fps=20,
        encoding="rgb8",
    )
    assert resolved.startup_wall_seconds == 120
    assert resolved.finalization_wall_seconds == 120
    assert resolved.output_root == (ROOT / "../runs").resolve()
    assert calls == 1
    with pytest.raises(FrozenInstanceError):
        resolved.world = "other"


def test_run_template_and_recording_are_frozen():
    resolved = resolve_run_config(DEFAULT_TEMPLATE, run_id_factory=lambda: FIXED_RUN_ID)
    template = RunTemplate(
        world=resolved.world,
        vehicle=resolved.vehicle,
        mission=resolved.mission,
        scenario=resolved.scenario,
        output_root=resolved.output_root,
        max_wall_seconds=resolved.max_wall_seconds,
        startup_wall_seconds=resolved.startup_wall_seconds,
        finalization_wall_seconds=resolved.finalization_wall_seconds,
        recording=resolved.recording,
    )

    with pytest.raises(FrozenInstanceError):
        template.world = "other"
    with pytest.raises(FrozenInstanceError):
        template.recording.fps = 10


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_config_schemas_are_valid_and_accept_phase_2_documents(schema_name):
    validator = _load_validator(schema_name)
    document = (
        json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
        if schema_name == "run-template.schema.json"
        else _resolved_document()
    )

    validator.validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
@pytest.mark.parametrize(
    "mutate",
    [
        pytest.param(
            lambda value: value["recording"].update({"width_px": 321}),
            id="odd-width",
        ),
        pytest.param(
            lambda value: value["recording"].update({"height_px": 239}),
            id="odd-height",
        ),
        pytest.param(
            lambda value: value["recording"].update({"fps": 19}), id="wrong-fps"
        ),
        pytest.param(
            lambda value: value["recording"].update({"encoding": "bgr8"}),
            id="wrong-encoding",
        ),
        pytest.param(
            lambda value: value["recording"].update({"width_px": True}),
            id="boolean-dimension",
        ),
        pytest.param(
            lambda value: value.update({"startup_wall_seconds": True}),
            id="boolean-deadline",
        ),
        pytest.param(
            lambda value: value.update({"finalization_wall_seconds": 0}),
            id="nonpositive-deadline",
        ),
        pytest.param(lambda value: value.update({"unexpected": 1}), id="unknown-key"),
    ],
)
def test_config_schemas_reject_invalid_phase_2_bindings(schema_name, mutate):
    validator = _load_validator(schema_name)
    document = (
        json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
        if schema_name == "run-template.schema.json"
        else _resolved_document()
    )
    mutate(document)

    with pytest.raises(ValidationError):
        validator.validate(document)


def test_template_schema_rejects_caller_supplied_run_id():
    document = json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
    document["run_id"] = str(FIXED_RUN_ID)

    with pytest.raises(ValidationError):
        _load_validator("run-template.schema.json").validate(document)


@pytest.mark.parametrize(
    "mutate",
    [
        pytest.param(lambda value: value.pop("world"), id="missing-key"),
        pytest.param(lambda value: value.update({"unexpected": True}), id="unknown-key"),
        pytest.param(lambda value: value.update({"run_id": str(FIXED_RUN_ID)}), id="run-id"),
        pytest.param(lambda value: value.update({"world": ""}), id="empty-string"),
        pytest.param(lambda value: value.update({"max_wall_seconds": 0}), id="zero-limit"),
        pytest.param(lambda value: value.update({"max_wall_seconds": True}), id="boolean-limit"),
        pytest.param(
            lambda value: value["recording"].update({"height_px": 239}),
            id="odd-dimension",
        ),
        pytest.param(
            lambda value: value["recording"].update({"fps": 30}), id="wrong-fps"
        ),
    ],
)
def test_resolve_run_config_rejects_invalid_templates(tmp_path, mutate):
    value = json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
    mutate(value)
    path = tmp_path / "run.json"
    path.write_text(json.dumps(value), encoding="utf-8")

    with pytest.raises(ValueError):
        resolve_run_config(path, run_id_factory=lambda: FIXED_RUN_ID)


def test_resolve_run_config_rejects_non_object_json(tmp_path):
    path = tmp_path / "run.json"
    path.write_text("[]", encoding="utf-8")

    with pytest.raises(ValueError):
        resolve_run_config(path, run_id_factory=lambda: FIXED_RUN_ID)


def test_resolve_run_config_uses_invoking_process_for_relative_output_root(
    tmp_path, monkeypatch
):
    invocation_directory = tmp_path / "invocation"
    invocation_directory.mkdir()
    monkeypatch.chdir(invocation_directory)

    resolved = resolve_run_config(DEFAULT_TEMPLATE, run_id_factory=lambda: FIXED_RUN_ID)

    assert resolved.output_root == invocation_directory / "runs"


def test_write_resolved_config_creates_schema_valid_exclusive_snapshot(tmp_path):
    resolved = resolve_run_config(DEFAULT_TEMPLATE, run_id_factory=lambda: FIXED_RUN_ID)
    run_dir = tmp_path / resolved.run_id

    written = write_resolved_config(run_dir, resolved)

    assert written == run_dir / "configuration/run.json"
    assert [path.relative_to(run_dir).as_posix() for path in run_dir.rglob("*")] == [
        "configuration",
        "configuration/run.json",
    ]
    document = json.loads(written.read_text(encoding="utf-8"))
    _load_validator("run.schema.json").validate(document)
    assert document == {
        "config_sha256": resolved.config_sha256,
        "finalization_wall_seconds": 120,
        "max_wall_seconds": 3600,
        "mission": "descent",
        "output_root": str((ROOT / "../runs").resolve()),
        "recording": {
            "encoding": "rgb8",
            "fps": 20,
            "height_px": 240,
            "width_px": 320,
        },
        "run_id": str(FIXED_RUN_ID),
        "scenario": "maximum_score",
        "startup_wall_seconds": 120,
        "vehicle": "iris",
        "world": "competition",
    }
    assert load_run_config(written) == resolved

    with pytest.raises(FileExistsError):
        write_resolved_config(run_dir, replace(resolved, world="other"))
    assert json.loads(written.read_text(encoding="utf-8")) == document


def test_load_run_config_rejects_checksum_mismatch(tmp_path):
    document = _resolved_document()
    document["world"] = "other"
    path = tmp_path / "run.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="config_sha256"):
        load_run_config(path)
