import hashlib
import json
from dataclasses import FrozenInstanceError
from pathlib import Path
from uuid import UUID

import pytest
from jsonschema import Draft202012Validator, FormatChecker, ValidationError

from orchestration.config import RunConfig, load_run_config


ROOT = Path(__file__).parents[1]


def test_run_schema_is_valid_and_default_run_validates():
    schema = json.loads((ROOT / "../config/run.schema.json").resolve().read_text())
    default = json.loads((ROOT / "../config/default-run.json").resolve().read_text())
    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema, format_checker=FormatChecker())
    validator.validate(default)
    invalid = {**default, "run_id": "not-a-uuid"}
    with pytest.raises(ValidationError):
        validator.validate(invalid)


def test_load_run_config_returns_frozen_config_with_canonical_checksum():
    path = (ROOT / "../config/default-run.json").resolve()
    raw = json.loads(path.read_text())
    config = load_run_config(path)

    assert isinstance(config, RunConfig)
    assert UUID(config.run_id).version == 4
    assert config.world == "competition"
    assert config.vehicle == "iris"
    assert config.mission == "descent"
    assert config.scenario == "maximum_score"
    assert config.output_root == (ROOT / "../runs").resolve()
    assert config.max_wall_seconds == 3600
    expected = hashlib.sha256(
        json.dumps(raw, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    assert config.config_sha256 == expected
    with pytest.raises(FrozenInstanceError):
        config.world = "other"


@pytest.mark.parametrize(
    "mutate",
    [
        lambda value: value.pop("world"),
        lambda value: value.update({"unexpected": True}),
        lambda value: value.update({"run_id": "not-a-uuid"}),
        lambda value: value.update({"world": ""}),
        lambda value: value.update({"max_wall_seconds": 0}),
        lambda value: value.update({"max_wall_seconds": True}),
    ],
)
def test_load_run_config_rejects_invalid_documents(tmp_path, mutate):
    value = json.loads((ROOT / "../config/default-run.json").resolve().read_text())
    mutate(value)
    path = tmp_path / "run.json"
    path.write_text(json.dumps(value))
    with pytest.raises(ValueError):
        load_run_config(path)


def test_load_run_config_rejects_non_object_json(tmp_path):
    path = tmp_path / "run.json"
    path.write_text("[]")
    with pytest.raises(ValueError):
        load_run_config(path)
