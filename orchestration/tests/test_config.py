import hashlib
import json
import os
import socket
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
from uuid import UUID

import pytest
import yaml
from jsonschema import Draft202012Validator, FormatChecker, ValidationError

import orchestration.config as config_module
from orchestration.config import (
    QGCSources,
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
REALTIME_TEMPLATE = CONFIG / "realtime-run.json"
VERTICAL_DESCENT_TEMPLATE = CONFIG / "vertical-descent-run.json"
ROLL_AUTOTUNE_TEMPLATE = CONFIG / "autotune-roll-run.json"
FIXED_RUN_ID = UUID("00000000-0000-4000-8000-000000000222")


COURSE_DOCUMENT = {
    "schema_version": 1,
    "units": "meters",
    "origin": "H",
    "waypoints": {
        "H": {"x": 0.0, "y": 0.0, "width": 4.572, "height": 4.572, "role": "home"},
        "L": {"x": -91.44, "y": 0.0, "width": 4.572, "height": 4.572, "role": "landing"},
        "F2": {"x": -152.40, "y": 0.0, "width": 0.9144, "height": 0.9144, "role": "fire"},
        "WA": {"x": -45.72, "y": -9.144, "width": 6.096, "height": 6.096, "role": "autonomous_pickup"},
        "WM": {"x": -45.72, "y": 9.144, "width": 6.096, "height": 6.096, "role": "manual_pickup"},
    },
    "attempt": {
        "duration_seconds": 600,
        "acquisition_agl_m": 4.572,
        "transit_agl_m": 10.0,
        "release_agl_m": 10.0,
    },
}
SCENARIO_DOCUMENT = {
    "schema_version": 1,
    "seed": 2026,
    "vehicle": {"payload_capacity": 1},
    "camera": {
        "width_px": 640,
        "height_px": 480,
        "update_rate_hz": 20,
        "horizontal_fov_rad": 0.60,
        "body_position_m": [0.0, 0.0, -0.10],
    },
    "range_sensor": {"update_rate_hz": 20},
    "payload_interaction": {
        "pickup_max_center_error_m": 0.075,
        "settle_position_tolerance_m": 0.01,
        "settle_time_s": 1.0,
    },
    "payload_geometry": {
        "size_in": [6, 6, 2],
        "mass_lb": 2.5,
        "marker_size_mm": 100,
    },
    "payloads": [
        {"aruco_id": 2, "color": "red", "initial": "attached"},
        {"aruco_id": 3, "color": "yellow", "initial": "WA"},
        {"aruco_id": 4, "color": "blue", "initial": "WM"},
    ],
    "mission": {
        "fm2_drop_zone": "F2",
        "fm3_cycles": [
            {"pickup_zone": "WA", "color": "yellow", "drop_zone": "F2"},
            {"pickup_zone": "WM", "color": "blue", "drop_zone": "F2"},
        ],
    },
}


def _load_validator(name: str) -> Draft202012Validator:
    schema = json.loads((CONFIG / name).read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema, format_checker=FormatChecker())


def _resolved_document() -> dict:
    template = json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
    template.pop("competition")
    document = {
        "run_id": str(FIXED_RUN_ID),
        **template,
        "output_root": str((ROOT / "../runs").resolve()),
        "competition": {
            "course": "course.yaml",
            "scenario": "scenario.yaml",
            "course_sha256": hashlib.sha256(
                (CONFIG / "course.yaml").read_bytes()
            ).hexdigest(),
            "scenario_sha256": hashlib.sha256(
                (CONFIG / "scenario.yaml").read_bytes()
            ).hexdigest(),
        },
    }
    canonical = json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
    document["config_sha256"] = hashlib.sha256(canonical).hexdigest()
    return document


def _phase2_document() -> dict:
    return {
        "world": "competition",
        "vehicle": "iris",
        "mission": "descent",
        "scenario": "maximum_score",
        "output_root": "runs",
        "max_wall_seconds": 3600,
        "startup_wall_seconds": 120,
        "finalization_wall_seconds": 600,
        "recording": {
            "width_px": 320,
            "height_px": 240,
            "fps": 20,
            "encoding": "rgb8",
        },
    }


def _competition_document() -> dict:
    document = _phase2_document()
    document.update(
        world="competition_mission",
        vehicle="iris_competition",
        mission="comp2026_auto",
        scenario="competition_v1",
        max_wall_seconds=5400,
        recording={
            "width_px": 640,
            "height_px": 480,
            "fps": 20,
            "encoding": "rgb8",
        },
        runtime_profile="phase3",
        simulation={
            "seed": 2026,
            "duration_sim_seconds": 600.0,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0.25,
        },
        competition={"course": "course.yaml", "scenario": "scenario.yaml"},
    )
    return document


def _write_competition_template(
    tmp_path: Path,
    *,
    course: dict = COURSE_DOCUMENT,
    scenario: dict = SCENARIO_DOCUMENT,
    include_qgc: bool = True,
) -> Path:
    (tmp_path / "course.yaml").write_text(json.dumps(course), encoding="utf-8")
    (tmp_path / "scenario.yaml").write_text(json.dumps(scenario), encoding="utf-8")
    document = _competition_document()
    if include_qgc:
        for field, name in QGC_SOURCE_NAMES.items():
            (tmp_path / name).write_bytes(QGC_PAYLOADS[field])
        document["qgc"] = dict(QGC_SOURCE_NAMES)
    return _write_template(tmp_path, document)


def _write_template(tmp_path: Path, document: dict, name: str = "run.json") -> Path:
    path = tmp_path / name
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


QGC_PAYLOADS = {
    "deployment_profile": b'{"kind":"deployment","value":1}\n',
    "listener_session": b'{"kind":"session","value":2}\n',
    "qgc_actions": b'{"kind":"actions","value":3}\n',
    "runtime_policy": b'{"kind":"policy","value":4}\n',
}
QGC_SOURCE_NAMES = {
    "deployment_profile": "operator-profile.json",
    "listener_session": "operator-session.json",
    "qgc_actions": "operator-actions.json",
    "runtime_policy": "operator-policy.json",
}
QGC_ARTIFACT_NAMES = {
    "deployment_profile": "deployment-profile.json",
    "listener_session": "listener-session.json",
    "qgc_actions": "qgc-actions.json",
    "runtime_policy": "qgc-runtime.json",
}


def _schema_competition_document(schema_name: str) -> dict:
    if schema_name == "run-template.schema.json":
        document = _competition_document()
        document["qgc"] = dict(QGC_SOURCE_NAMES)
        return document
    document = _resolved_document()
    digests = {
        f"{field}_sha256": hashlib.sha256(payload).hexdigest()
        for field, payload in QGC_PAYLOADS.items()
    }
    document["qgc"] = {
        **QGC_ARTIFACT_NAMES,
        **digests,
        "attempt_state_id": "sha256-" + digests["deployment_profile_sha256"],
    }
    _rewrite_resolved_checksum(document)
    return document


def _write_qgc_template(tmp_path: Path, document: dict | None = None) -> Path:
    (tmp_path / "course.yaml").write_text(json.dumps(COURSE_DOCUMENT), encoding="utf-8")
    (tmp_path / "scenario.yaml").write_text(
        json.dumps(SCENARIO_DOCUMENT), encoding="utf-8"
    )
    for field, name in QGC_SOURCE_NAMES.items():
        (tmp_path / name).write_bytes(QGC_PAYLOADS[field])
    document = _competition_document() if document is None else document
    document["qgc"] = dict(QGC_SOURCE_NAMES)
    return _write_template(tmp_path, document)


def _write_security_qgc_template(
    directory: Path,
    payloads: dict[str, bytes] = QGC_PAYLOADS,
    *,
    phase3: bool = True,
) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "course.yaml").write_bytes((CONFIG / "course.yaml").read_bytes())
    (directory / "scenario.yaml").write_bytes((CONFIG / "scenario.yaml").read_bytes())
    for field, name in QGC_SOURCE_NAMES.items():
        (directory / name).write_bytes(payloads[field])
    document = _competition_document()
    if not phase3:
        document.pop("runtime_profile")
        document.pop("simulation")
    document["qgc"] = dict(QGC_SOURCE_NAMES)
    return _write_template(directory, document)


def test_qgc_sources_resolve_to_exact_immutable_bytes_and_canonical_identity(tmp_path):
    template = _write_qgc_template(tmp_path)

    resolved = resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)
    written = write_resolved_config(tmp_path / "run", resolved)
    document = json.loads(written.read_text(encoding="utf-8"))

    assert resolved.qgc is not None
    for field, payload in QGC_PAYLOADS.items():
        assert getattr(resolved.qgc, field) == payload
        artifact_name = QGC_ARTIFACT_NAMES[field]
        assert (written.parent / artifact_name).read_bytes() == payload
        assert document["qgc"][field] == artifact_name
        assert document["qgc"][f"{field}_sha256"] == hashlib.sha256(payload).hexdigest()
    profile_digest = hashlib.sha256(QGC_PAYLOADS["deployment_profile"]).hexdigest()
    assert document["qgc"]["attempt_state_id"] == f"sha256-{profile_digest}"
    without_checksum = {
        key: value for key, value in document.items() if key != "config_sha256"
    }
    assert document["config_sha256"] == hashlib.sha256(
        json.dumps(without_checksum, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def test_qgc_snapshot_uses_bytes_retained_during_template_load(tmp_path):
    template = _write_qgc_template(tmp_path)
    resolved = resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)
    for name in QGC_SOURCE_NAMES.values():
        (tmp_path / name).write_bytes(b'{"changed":true}\n')

    written = write_resolved_config(tmp_path / "run", resolved)

    for field, payload in QGC_PAYLOADS.items():
        assert (written.parent / QGC_ARTIFACT_NAMES[field]).read_bytes() == payload


@pytest.mark.parametrize("unsafe", ["", ".", "../outside.json", "/outside.json"])
def test_qgc_template_rejects_unsafe_source_paths(tmp_path, unsafe):
    template = _write_qgc_template(tmp_path)
    document = json.loads(template.read_text(encoding="utf-8"))
    document["qgc"]["runtime_policy"] = unsafe
    template.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="safe relative path|relative path"):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)


def test_qgc_template_rejects_symlinked_source_directory(tmp_path):
    template = _write_qgc_template(tmp_path)
    actual = tmp_path / "actual"
    actual.mkdir()
    (actual / "policy.json").write_bytes(QGC_PAYLOADS["runtime_policy"])
    (tmp_path / "linked").symlink_to(actual, target_is_directory=True)
    document = json.loads(template.read_text(encoding="utf-8"))
    document["qgc"]["runtime_policy"] = "linked/policy.json"
    template.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="non-symlink"):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)


def test_load_run_config_retains_verified_qgc_snapshot_bytes(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    resolved = resolve_run_config(
        _write_qgc_template(source),
        run_id_factory=lambda: FIXED_RUN_ID,
    )
    written = write_resolved_config(tmp_path / "run", resolved)

    loaded = load_run_config(written)

    assert loaded.qgc == resolved.qgc


@pytest.mark.parametrize(
    "payload",
    [b"[]\n", b"null\n", b"{broken\n", b"\xff\n"],
    ids=["array", "null", "invalid-json", "invalid-utf8"],
)
def test_qgc_template_rejects_sources_that_are_not_utf8_json_objects(
    tmp_path, payload
):
    template = _write_qgc_template(tmp_path)
    (tmp_path / QGC_SOURCE_NAMES["runtime_policy"]).write_bytes(payload)

    with pytest.raises(ValueError, match="UTF-8 JSON|JSON object"):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)


@pytest.mark.parametrize("kind", ["symlink", "directory", "fifo", "socket"])
def test_qgc_template_rejects_non_regular_sources(tmp_path, kind):
    template = _write_qgc_template(tmp_path)
    source = tmp_path / QGC_SOURCE_NAMES["runtime_policy"]
    source.unlink()
    opened_socket = None
    if kind == "symlink":
        target = tmp_path / "real-policy.json"
        target.write_bytes(QGC_PAYLOADS["runtime_policy"])
        source.symlink_to(target)
    elif kind == "directory":
        source.mkdir()
    elif kind == "fifo":
        os.mkfifo(source)
    else:
        opened_socket = socket.socket(socket.AF_UNIX)
        opened_socket.bind(str(source))
    try:
        with pytest.raises(ValueError, match="regular non-symlink"):
            resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)
    finally:
        if opened_socket is not None:
            opened_socket.close()


@pytest.mark.parametrize("alias_kind", ["normalized-path", "hard-link"])
def test_qgc_template_requires_four_distinct_source_files(tmp_path, alias_kind):
    template = _write_qgc_template(tmp_path)
    document = json.loads(template.read_text(encoding="utf-8"))
    if alias_kind == "normalized-path":
        document["qgc"]["listener_session"] = "./operator-profile.json"
        template.write_text(json.dumps(document), encoding="utf-8")
    else:
        session = tmp_path / QGC_SOURCE_NAMES["listener_session"]
        session.unlink()
        os.link(tmp_path / QGC_SOURCE_NAMES["deployment_profile"], session)

    with pytest.raises(ValueError, match="pairwise distinct"):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)


def test_qgc_destination_collision_is_preflighted_before_any_snapshot_write(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    resolved = resolve_run_config(
        _write_qgc_template(source), run_id_factory=lambda: FIXED_RUN_ID
    )
    configuration = tmp_path / "run/configuration"
    configuration.mkdir(parents=True)
    outside = tmp_path / "outside.json"
    outside.write_bytes(b"unchanged")
    collision = configuration / "qgc-actions.json"
    collision.symlink_to(outside)

    with pytest.raises(FileExistsError):
        write_resolved_config(tmp_path / "run", resolved)

    assert sorted(path.name for path in configuration.iterdir()) == ["qgc-actions.json"]
    assert outside.read_bytes() == b"unchanged"


def _rewrite_resolved_checksum(document: dict) -> None:
    without_checksum = {
        key: value for key, value in document.items() if key != "config_sha256"
    }
    document["config_sha256"] = hashlib.sha256(
        json.dumps(without_checksum, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


@pytest.mark.parametrize(
    "mutate",
    [
        pytest.param(
            lambda qgc: qgc.update(runtime_policy="policy.json"), id="artifact-name"
        ),
        pytest.param(
            lambda qgc: qgc.update(runtime_policy_sha256="A" * 64), id="digest-case"
        ),
        pytest.param(
            lambda qgc: qgc.update(attempt_state_id="sha256-" + "0" * 64),
            id="state-relation",
        ),
        pytest.param(lambda qgc: qgc.update(ledger_path="ledger.json"), id="ledger"),
    ],
)
def test_load_run_config_rejects_tampered_qgc_metadata(tmp_path, mutate):
    source = tmp_path / "source"
    source.mkdir()
    resolved = resolve_run_config(
        _write_qgc_template(source), run_id_factory=lambda: FIXED_RUN_ID
    )
    written = write_resolved_config(tmp_path / "run", resolved)
    document = json.loads(written.read_text(encoding="utf-8"))
    mutate(document["qgc"])
    _rewrite_resolved_checksum(document)
    written.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError):
        load_run_config(written)


@pytest.mark.parametrize("tamper", ["bytes", "symlink"])
def test_load_run_config_rejects_tampered_qgc_artifact(tmp_path, tamper):
    source = tmp_path / "source"
    source.mkdir()
    resolved = resolve_run_config(
        _write_qgc_template(source), run_id_factory=lambda: FIXED_RUN_ID
    )
    written = write_resolved_config(tmp_path / "run", resolved)
    policy = written.parent / "qgc-runtime.json"
    if tamper == "bytes":
        policy.write_bytes(b'{"changed":true}\n')
    else:
        policy.unlink()
        target = tmp_path / "outside-policy.json"
        target.write_bytes(QGC_PAYLOADS["runtime_policy"])
        policy.symlink_to(target)

    with pytest.raises(ValueError):
        load_run_config(written)


def test_qgc_snapshot_never_creates_attempt_state_artifacts(tmp_path):
    resolved = resolve_run_config(
        _write_qgc_template(tmp_path), run_id_factory=lambda: FIXED_RUN_ID
    )
    written = write_resolved_config(tmp_path / "run", resolved)

    names = {path.name for path in written.parent.iterdir()}
    assert not any("ledger" in name or name.endswith(".lock") for name in names)


def test_competition_templates_without_qgc_select_automatic_path():
    for template in (DEFAULT_TEMPLATE, REALTIME_TEMPLATE):
        resolved = resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)

        assert resolved.mission == "comp2026_auto"
        assert resolved.qgc is None
        assert resolved.competition is not None


def test_qgc_sources_reject_non_object_payloads():
    with pytest.raises(ValueError, match="JSON object"):
        QGCSources(b"[]", b"{}", b"{}", b"{}")


def test_qgc_sources_are_frozen():
    sources = QGCSources(**QGC_PAYLOADS)

    with pytest.raises(FrozenInstanceError):
        sources.runtime_policy = b"{}"


@pytest.mark.parametrize("change", ["missing", "extra", "non-string"])
def test_qgc_template_requires_exact_source_fields(tmp_path, change):
    template = _write_qgc_template(tmp_path)
    document = json.loads(template.read_text(encoding="utf-8"))
    if change == "missing":
        document["qgc"].pop("runtime_policy")
    elif change == "extra":
        document["qgc"]["ledger"] = "ledger.json"
    else:
        document["qgc"]["runtime_policy"] = None
    template.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)


def test_qgc_write_rejects_symlinked_configuration_directory(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    resolved = resolve_run_config(
        _write_qgc_template(source), run_id_factory=lambda: FIXED_RUN_ID
    )
    outside = tmp_path / "outside"
    outside.mkdir()
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    (run_dir / "configuration").symlink_to(outside, target_is_directory=True)

    with pytest.raises(FileExistsError):
        write_resolved_config(run_dir, resolved)

    assert list(outside.iterdir()) == []


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_config_schemas_accept_phase3_qgc_bundle(schema_name):
    if schema_name == "run-template.schema.json":
        document = _competition_document()
        document["qgc"] = dict(QGC_SOURCE_NAMES)
    else:
        document = _resolved_document()
        digests = {
            f"{field}_sha256": hashlib.sha256(payload).hexdigest()
            for field, payload in QGC_PAYLOADS.items()
        }
        document["qgc"] = {
            **QGC_ARTIFACT_NAMES,
            **digests,
            "attempt_state_id": "sha256-"
            + digests["deployment_profile_sha256"],
        }
        _rewrite_resolved_checksum(document)

    _load_validator(schema_name).validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_comp2026_schema_accepts_automatic_run_without_qgc_bundle(schema_name):
    document = (
        _competition_document()
        if schema_name == "run-template.schema.json"
        else _resolved_document()
    )

    _load_validator(schema_name).validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_qgc_schema_requires_explicit_phase3_profile(schema_name):
    document = _phase2_document()
    if schema_name == "run-template.schema.json":
        document["qgc"] = dict(QGC_SOURCE_NAMES)
    else:
        digests = {
            f"{field}_sha256": hashlib.sha256(payload).hexdigest()
            for field, payload in QGC_PAYLOADS.items()
        }
        document.update(
            run_id=str(FIXED_RUN_ID),
            output_root=str((ROOT / "../runs").resolve()),
            qgc={
                **QGC_ARTIFACT_NAMES,
                **digests,
                "attempt_state_id": "sha256-"
                + digests["deployment_profile_sha256"],
            },
        )
        _rewrite_resolved_checksum(document)

    with pytest.raises(ValidationError):
        _load_validator(schema_name).validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
@pytest.mark.parametrize(
    "mission", ["controlled_descent", "autotune_roll", "hover_roll"]
)
def test_qgc_schema_requires_comp2026_mission(schema_name, mission):
    document = _schema_competition_document(schema_name)
    document["mission"] = mission
    document["recording"]["width_px"] = 320
    document["recording"]["height_px"] = 240
    if schema_name == "run.schema.json":
        _rewrite_resolved_checksum(document)

    with pytest.raises(ValidationError):
        _load_validator(schema_name).validate(document)


def test_resolved_schema_rejects_uppercase_uuid():
    document = _phase2_document()
    document.update(
        run_id="00000000-0000-4000-8000-000000000ABC",
        output_root=str((ROOT / "../runs").resolve()),
        config_sha256="a" * 64,
    )

    with pytest.raises(ValidationError):
        _load_validator("run.schema.json").validate(document)


def test_plain_resolved_schema_rejects_uuid_with_trailing_newline():
    schema = json.loads((CONFIG / "run.schema.json").read_text(encoding="utf-8"))
    document = _phase2_document()
    document.update(
        run_id=str(FIXED_RUN_ID) + "\n",
        output_root=str((ROOT / "../runs").resolve()),
        config_sha256="a" * 64,
    )

    with pytest.raises(ValidationError):
        Draft202012Validator(schema).validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
@pytest.mark.parametrize("change", ["missing", "extra", "unsafe-path-or-name"])
def test_config_schemas_reject_malformed_qgc_bundle(schema_name, change):
    if schema_name == "run-template.schema.json":
        document = _competition_document()
        qgc = dict(QGC_SOURCE_NAMES)
        if change == "unsafe-path-or-name":
            qgc["runtime_policy"] = "."
    else:
        document = _resolved_document()
        digests = {
            f"{field}_sha256": hashlib.sha256(payload).hexdigest()
            for field, payload in QGC_PAYLOADS.items()
        }
        qgc = {
            **QGC_ARTIFACT_NAMES,
            **digests,
            "attempt_state_id": "sha256-"
            + digests["deployment_profile_sha256"],
        }
        if change == "unsafe-path-or-name":
            qgc["runtime_policy"] = "policy.json"
    if change == "missing":
        qgc.pop("runtime_policy")
    elif change == "extra":
        qgc["ledger"] = "ledger.json"
    document["qgc"] = qgc

    with pytest.raises(ValidationError):
        _load_validator(schema_name).validate(document)


@pytest.mark.parametrize(
    "field",
    [
        "deployment_profile_sha256",
        "listener_session_sha256",
        "qgc_actions_sha256",
        "runtime_policy_sha256",
        "attempt_state_id",
    ],
)
@pytest.mark.parametrize("malformation", ["terminal-newline", "too-short"])
def test_resolved_schema_rejects_wrong_length_qgc_identities(field, malformation):
    document = _resolved_document()
    digests = {
        f"{name}_sha256": hashlib.sha256(payload).hexdigest()
        for name, payload in QGC_PAYLOADS.items()
    }
    document["qgc"] = {
        **QGC_ARTIFACT_NAMES,
        **digests,
        "attempt_state_id": "sha256-" + digests["deployment_profile_sha256"],
    }
    value = document["qgc"][field]
    document["qgc"][field] = (
        value + "\n" if malformation == "terminal-newline" else value[:-1]
    )
    _rewrite_resolved_checksum(document)

    with pytest.raises(ValidationError):
        _load_validator("run.schema.json").validate(document)


@pytest.mark.parametrize("resolved", [False, True], ids=["template", "resolved"])
def test_explicit_null_qgc_is_rejected(tmp_path, resolved):
    if not resolved:
        document = _phase2_document()
        document["qgc"] = None
        path = _write_template(tmp_path, document)
        with pytest.raises(ValueError, match="QGC configuration"):
            resolve_run_config(path, run_id_factory=lambda: FIXED_RUN_ID)
        return

    document = _competition_document()
    document.update(
        run_id=str(FIXED_RUN_ID),
        qgc=None,
    )
    _rewrite_resolved_checksum(document)
    path = _write_template(tmp_path, document)
    with pytest.raises(ValueError, match="resolved QGC configuration"):
        load_run_config(path)


@pytest.mark.parametrize(
    "payload",
    [
        b'{"value":NaN}',
        b'{"value":Infinity}',
        b'{"value":-Infinity}',
        b'{"value":1,"value":2}',
    ],
    ids=["nan", "infinity", "negative-infinity", "duplicate-key"],
)
def test_qgc_json_objects_reject_nonstandard_numbers_and_duplicate_keys(
    tmp_path, payload
):
    template = _write_qgc_template(tmp_path)
    (tmp_path / QGC_SOURCE_NAMES["runtime_policy"]).write_bytes(payload)

    with pytest.raises(ValueError, match="valid UTF-8 JSON"):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)
    with pytest.raises(ValueError, match="valid UTF-8 JSON"):
        QGCSources(
            QGC_PAYLOADS["deployment_profile"],
            QGC_PAYLOADS["listener_session"],
            QGC_PAYLOADS["qgc_actions"],
            payload,
        )


@pytest.mark.parametrize(
    "unsafe", ["./operator-policy.json", "./.", "nested\n/policy.json", "nested/.."]
)
def test_qgc_runtime_and_template_schema_reject_the_same_unsafe_paths(
    tmp_path, unsafe
):
    template = _write_qgc_template(tmp_path)
    document = json.loads(template.read_text(encoding="utf-8"))
    document["qgc"]["runtime_policy"] = unsafe
    template.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="safe relative path"):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)
    with pytest.raises(ValidationError):
        _load_validator("run-template.schema.json").validate(document)


def test_qgc_runtime_rejects_embedded_nul_as_an_unsafe_path(tmp_path):
    template = _write_qgc_template(tmp_path)
    document = json.loads(template.read_text(encoding="utf-8"))
    document["qgc"]["runtime_policy"] = "nested\x00/policy.json"
    template.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="safe relative path"):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)


def test_qgc_template_schema_rejects_embedded_nul_path():
    document = _competition_document()
    document["qgc"] = {
        **QGC_SOURCE_NAMES,
        "runtime_policy": "nested\x00/policy.json",
    }

    with pytest.raises(ValidationError):
        _load_validator("run-template.schema.json").validate(document)


def test_template_and_qgc_reads_stay_on_one_directory_descriptor(
    tmp_path, monkeypatch
):
    source = tmp_path / "source"
    template = _write_security_qgc_template(source)
    replacement_payloads = {
        field: json.dumps({"replacement": field}).encode()
        for field in QGC_PAYLOADS
    }
    replacement = tmp_path / "replacement"
    _write_security_qgc_template(replacement, replacement_payloads)
    held = tmp_path / "held-source"
    original = config_module._qgc_from_template

    def substitute_directory(document, *args):
        source.rename(held)
        replacement.rename(source)
        return original(document, *args)

    monkeypatch.setattr(config_module, "_qgc_from_template", substitute_directory)

    resolved = resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)

    assert resolved.qgc.runtime_policy == QGC_PAYLOADS["runtime_policy"]


def test_resolved_run_and_qgc_reads_stay_on_one_directory_descriptor(
    tmp_path, monkeypatch
):
    source = tmp_path / "source"
    resolved = resolve_run_config(
        _write_security_qgc_template(source), run_id_factory=lambda: FIXED_RUN_ID
    )
    written = write_resolved_config(tmp_path / "run", resolved)
    configuration = written.parent
    replacement = tmp_path / "replacement"
    replacement.mkdir()
    (replacement / "run.json").write_bytes(written.read_bytes())
    (replacement / "course.yaml").write_bytes((configuration / "course.yaml").read_bytes())
    (replacement / "scenario.yaml").write_bytes(
        (configuration / "scenario.yaml").read_bytes()
    )
    for field, artifact in QGC_ARTIFACT_NAMES.items():
        (replacement / artifact).write_bytes(
            json.dumps({"replacement": field}).encode()
        )
    held = tmp_path / "held-configuration"
    original = config_module._qgc_from_resolved

    def substitute_directory(document, *args):
        configuration.rename(held)
        replacement.rename(configuration)
        return original(document, *args)

    monkeypatch.setattr(config_module, "_qgc_from_resolved", substitute_directory)

    loaded = load_run_config(written)

    assert loaded.qgc == resolved.qgc


@pytest.mark.parametrize("resolved", [False, True], ids=["template", "resolved"])
def test_qgc_load_rejects_symlinked_owning_directory(tmp_path, resolved):
    source = tmp_path / "source"
    if resolved:
        inputs = tmp_path / "inputs"
        config = resolve_run_config(
            _write_security_qgc_template(inputs), run_id_factory=lambda: FIXED_RUN_ID
        )
        path = write_resolved_config(source.parent / "run", config)
        source = path.parent
    else:
        path = _write_security_qgc_template(source)
    alias = tmp_path / "alias"
    alias.symlink_to(source, target_is_directory=True)

    with pytest.raises(ValueError, match="non-symlink directory"):
        if resolved:
            load_run_config(alias / "run.json")
        else:
            resolve_run_config(alias / path.name, run_id_factory=lambda: FIXED_RUN_ID)


def test_snapshot_writes_stay_on_the_acquired_configuration_descriptor(
    tmp_path, monkeypatch
):
    source = tmp_path / "source"
    resolved = resolve_run_config(
        _write_security_qgc_template(source), run_id_factory=lambda: FIXED_RUN_ID
    )
    run_dir = tmp_path / "run"
    configuration = run_dir / "configuration"
    held = tmp_path / "held-configuration"
    outside = tmp_path / "outside"
    outside.mkdir()
    real_open = os.open
    substituted = False

    def substitute_after_directory_open(path, flags, mode=0o777, *, dir_fd=None):
        nonlocal substituted
        descriptor = real_open(path, flags, mode, dir_fd=dir_fd)
        path_text = os.fspath(path)
        opening_configuration = (
            not substituted
            and flags & os.O_DIRECTORY
            and (
                path_text == os.fspath(configuration)
                or (path_text == "configuration" and dir_fd is not None)
            )
        )
        if opening_configuration:
            assert not (configuration / "run.json").exists()
            configuration.rename(held)
            configuration.symlink_to(outside, target_is_directory=True)
            substituted = True
        return descriptor

    monkeypatch.setattr(config_module.os, "open", substitute_after_directory_open)

    write_resolved_config(run_dir, resolved)

    assert substituted is True
    assert list(outside.iterdir()) == []
    assert sorted(path.name for path in held.iterdir()) == [
        "course.yaml",
        "deployment-profile.json",
        "listener-session.json",
        "qgc-actions.json",
        "qgc-runtime.json",
        "run.json",
        "scenario.yaml",
    ]


def test_qgc_template_resolves_complete_competition_attempt(tmp_path):
    calls = 0

    def fixed_uuid() -> UUID:
        nonlocal calls
        calls += 1
        return FIXED_RUN_ID

    resolved = resolve_run_config(_write_qgc_template(tmp_path), run_id_factory=fixed_uuid)

    assert isinstance(resolved, RunConfig)
    assert resolved.run_id == "00000000-0000-4000-8000-000000000222"
    assert (resolved.world, resolved.vehicle, resolved.mission, resolved.scenario) == (
        "competition_mission",
        "iris_competition",
        "comp2026_auto",
        "competition_v1",
    )
    assert resolved.runtime_profile == "phase3"
    assert resolved.simulation == config_module.SimulationConfig(
        seed=2026,
        duration_ns=600_000_000_000,
        target_real_time_factor=0.25,
    )
    assert getattr(resolved.simulation, "public_epoch_native_ns", None) == 90_000_000_000
    assert resolved.expected_camera_frames == 12000
    assert resolved.recording == RecordingConfig(
        width_px=640,
        height_px=480,
        fps=20,
        encoding="rgb8",
    )
    assert resolved.startup_wall_seconds == 120
    assert resolved.max_wall_seconds == 5400
    assert resolved.finalization_wall_seconds == 600
    assert resolved.output_root == (Path.cwd() / "runs").resolve()
    assert resolved.competition is not None
    assert resolved.competition.course_source == tmp_path / "course.yaml"
    assert resolved.competition.scenario_source == tmp_path / "scenario.yaml"
    assert resolved.competition.course_sha256 == hashlib.sha256(
        (tmp_path / "course.yaml").read_bytes()
    ).hexdigest()
    assert resolved.competition.scenario_sha256 == hashlib.sha256(
        (tmp_path / "scenario.yaml").read_bytes()
    ).hexdigest()
    assert calls == 1
    with pytest.raises(FrozenInstanceError):
        resolved.world = "other"
    with pytest.raises(FrozenInstanceError):
        resolved.competition.course_source = CONFIG / "other.yaml"

    written = write_resolved_config(tmp_path, resolved)
    assert (written.parent / "course.yaml").read_bytes() == (
        tmp_path / "course.yaml"
    ).read_bytes()
    assert (written.parent / "scenario.yaml").read_bytes() == (
        tmp_path / "scenario.yaml"
    ).read_bytes()


def test_realtime_qgc_template_resolves_competition_attempt_at_one_x(tmp_path):
    document = json.loads(REALTIME_TEMPLATE.read_text(encoding="utf-8"))
    config = resolve_run_config(
        _write_qgc_template(tmp_path, document), run_id_factory=lambda: FIXED_RUN_ID
    )

    assert config.world == "competition_mission"
    assert config.simulation.target_real_time_factor == 1.0
    assert config.simulation.public_epoch_native_ns == 90_000_000_000


def test_authoritative_competition_sources_have_approved_physical_values():
    assert yaml.safe_load((CONFIG / "course.yaml").read_text(encoding="utf-8")) == (
        COURSE_DOCUMENT
    )
    assert yaml.safe_load((CONFIG / "scenario.yaml").read_text(encoding="utf-8")) == (
        SCENARIO_DOCUMENT
    )


def test_explicit_vertical_descent_template_preserves_prior_profile():
    resolved = resolve_run_config(
        VERTICAL_DESCENT_TEMPLATE, run_id_factory=lambda: FIXED_RUN_ID
    )

    assert (resolved.world, resolved.vehicle, resolved.mission, resolved.scenario) == (
        "vertical_descent",
        "iris_flight",
        "controlled_descent",
        "descent_v1",
    )
    assert resolved.recording == RecordingConfig(320, 240, 20, "rgb8")
    assert resolved.simulation == config_module.SimulationConfig(
        seed=1,
        duration_ns=60_000_000_000,
        target_real_time_factor=0.1,
    )
    assert resolved.competition is None


def test_roll_autotune_template_uses_the_lightweight_flight_world():
    resolved = resolve_run_config(
        ROLL_AUTOTUNE_TEMPLATE, run_id_factory=lambda: FIXED_RUN_ID
    )

    assert (resolved.world, resolved.vehicle, resolved.mission, resolved.scenario) == (
        "vertical_descent",
        "iris_flight",
        "autotune_roll",
        "descent_v1",
    )
    assert resolved.recording == RecordingConfig(320, 240, 20, "rgb8")
    assert resolved.simulation == config_module.SimulationConfig(
        seed=2026,
        duration_ns=120_000_000_000,
        target_real_time_factor=0.1,
        public_epoch_native_ns=15_000_000_000,
    )
    assert resolved.competition is None


def test_roll_autotune_template_and_resolved_snapshot_match_public_schemas(
    tmp_path: Path,
) -> None:
    template_document = json.loads(ROLL_AUTOTUNE_TEMPLATE.read_text(encoding="utf-8"))
    _load_validator("run-template.schema.json").validate(template_document)

    resolved = resolve_run_config(
        ROLL_AUTOTUNE_TEMPLATE, run_id_factory=lambda: FIXED_RUN_ID
    )
    resolved_path = write_resolved_config(tmp_path, resolved)
    resolved_document = json.loads(resolved_path.read_text(encoding="utf-8"))
    _load_validator("run.schema.json").validate(resolved_document)


@pytest.mark.parametrize(
    ("source_name", "mutate"),
    [
        ("course", lambda value: value["waypoints"]["F2"].update(x=-152.39)),
        ("course", lambda value: value.update(schema_version=True)),
        ("scenario", lambda value: value["payloads"][2].update(aruco_id=5)),
        ("scenario", lambda value: value["vehicle"].update(payload_capacity=True)),
    ],
)
def test_competition_sources_reject_wrong_course_or_payload_layout(
    tmp_path, source_name, mutate
):
    course = json.loads(json.dumps(COURSE_DOCUMENT))
    scenario = json.loads(json.dumps(SCENARIO_DOCUMENT))
    mutate({"course": course, "scenario": scenario}[source_name])

    with pytest.raises(ValueError, match=f"{source_name} configuration"):
        resolve_run_config(
            _write_competition_template(tmp_path, course=course, scenario=scenario),
            run_id_factory=lambda: FIXED_RUN_ID,
        )


def test_competition_sources_must_be_regular_non_symlink_files(tmp_path):
    template = _write_competition_template(tmp_path)
    (tmp_path / "course.yaml").unlink()
    (tmp_path / "course.yaml").symlink_to(CONFIG / "course.yaml")

    with pytest.raises(ValueError, match="regular non-symlink"):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)


def test_resolver_accepts_comp2026_without_qgc_inputs(tmp_path):
    template = _write_competition_template(tmp_path, include_qgc=False)

    resolved = resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)

    assert resolved.mission == "comp2026_auto"
    assert resolved.qgc is None


def test_resolver_rejects_qgc_inputs_outside_phase3(tmp_path):
    template = _write_security_qgc_template(tmp_path, phase3=False)

    with pytest.raises(ValueError, match="QGC.*phase3"):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)


@pytest.mark.parametrize("mission", ["controlled_descent", "autotune_roll", "hover_roll"])
def test_resolver_rejects_qgc_for_non_comp2026_before_source_reads(
    tmp_path, monkeypatch, mission
):
    document = _competition_document()
    document["mission"] = mission
    document["recording"] = {
        "width_px": 320,
        "height_px": 240,
        "fps": 20,
        "encoding": "rgb8",
    }
    template = _write_qgc_template(tmp_path, document)
    source_reads = 0

    def reject_source_read(*_args, **_kwargs):
        nonlocal source_reads
        source_reads += 1
        raise AssertionError("QGC sources must not be read")

    monkeypatch.setattr(config_module, "_qgc_from_template", reject_source_read)
    monkeypatch.setattr(config_module, "_competition_from_template", reject_source_read)

    with pytest.raises(ValueError, match="QGC.*comp2026_auto"):
        resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)
    assert source_reads == 0


@pytest.mark.parametrize("mission", ["controlled_descent", "autotune_roll", "hover_roll"])
def test_non_qgc_diagnostic_missions_still_resolve(tmp_path, mission):
    document = _phase2_document()
    document["mission"] = mission

    resolved = resolve_run_config(
        _write_template(tmp_path, document), run_id_factory=lambda: FIXED_RUN_ID
    )

    assert resolved.mission == mission
    assert resolved.qgc is None


def _write_resolved_competition_document(tmp_path: Path, document: dict) -> Path:
    configuration = tmp_path / "configuration"
    configuration.mkdir()
    (configuration / "course.yaml").write_bytes((CONFIG / "course.yaml").read_bytes())
    (configuration / "scenario.yaml").write_bytes(
        (CONFIG / "scenario.yaml").read_bytes()
    )
    path = configuration / "run.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def _resolved_phase2_qgc_document() -> dict:
    document = _phase2_document()
    digests = {
        f"{field}_sha256": hashlib.sha256(payload).hexdigest()
        for field, payload in QGC_PAYLOADS.items()
    }
    document.update(
        run_id=str(FIXED_RUN_ID),
        output_root=str((ROOT / "../runs").resolve()),
        qgc={
            **QGC_ARTIFACT_NAMES,
            **digests,
            "attempt_state_id": "sha256-" + digests["deployment_profile_sha256"],
        },
    )
    _rewrite_resolved_checksum(document)
    return document


def test_loader_rejects_qgc_inputs_outside_phase3(tmp_path):
    configuration = tmp_path / "configuration"
    configuration.mkdir()
    for field, name in QGC_ARTIFACT_NAMES.items():
        (configuration / name).write_bytes(QGC_PAYLOADS[field])
    path = configuration / "run.json"
    path.write_text(json.dumps(_resolved_phase2_qgc_document()), encoding="utf-8")

    with pytest.raises(ValueError, match="QGC.*phase3"):
        load_run_config(path)


@pytest.mark.parametrize("mission", ["controlled_descent", "autotune_roll", "hover_roll"])
def test_loader_rejects_qgc_for_non_comp2026_before_source_reads(
    tmp_path, monkeypatch, mission
):
    document = _schema_competition_document("run.schema.json")
    document["mission"] = mission
    document["recording"] = {
        "width_px": 320,
        "height_px": 240,
        "fps": 20,
        "encoding": "rgb8",
    }
    _rewrite_resolved_checksum(document)
    path = tmp_path / "run.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    source_reads = 0

    def reject_source_read(*_args, **_kwargs):
        nonlocal source_reads
        source_reads += 1
        raise AssertionError("resolved sources must not be read")

    monkeypatch.setattr(config_module, "_qgc_from_resolved", reject_source_read)
    monkeypatch.setattr(config_module, "_competition_from_resolved", reject_source_read)

    with pytest.raises(ValueError, match="QGC.*comp2026_auto"):
        load_run_config(path)
    assert source_reads == 0


def test_loader_accepts_comp2026_without_qgc_inputs(tmp_path):
    document = _resolved_document()

    resolved = load_run_config(
        _write_resolved_competition_document(tmp_path, document)
    )

    assert resolved.mission == "comp2026_auto"
    assert resolved.qgc is None


def test_loader_rejects_uppercase_uuid_instead_of_normalizing(tmp_path):
    document = _phase2_document()
    document.update(
        run_id="00000000-0000-4000-8000-000000000ABC",
        output_root=str((ROOT / "../runs").resolve()),
    )
    _rewrite_resolved_checksum(document)
    path = tmp_path / "run.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="canonical UUID"):
        load_run_config(path)


def test_loader_rejects_uuid_with_trailing_newline(tmp_path):
    document = _phase2_document()
    document.update(
        run_id=str(FIXED_RUN_ID) + "\n",
        output_root=str((ROOT / "../runs").resolve()),
    )
    _rewrite_resolved_checksum(document)
    path = tmp_path / "run.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="UUID"):
        load_run_config(path)


def test_loader_rejects_uppercase_uuid_before_reading_qgc_snapshots(
    tmp_path, monkeypatch
):
    document = _schema_competition_document("run.schema.json")
    document["run_id"] = "00000000-0000-4000-8000-000000000ABC"
    _rewrite_resolved_checksum(document)
    path = tmp_path / "run.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    qgc_reads = 0

    def reject_qgc_read(*_args, **_kwargs):
        nonlocal qgc_reads
        qgc_reads += 1
        raise AssertionError("QGC snapshots must not be read")

    monkeypatch.setattr(config_module, "_qgc_from_resolved", reject_qgc_read)

    with pytest.raises(ValueError, match="canonical UUID"):
        load_run_config(path)
    assert qgc_reads == 0


def test_resolve_hashes_same_source_bytes_that_passed_validation(
    tmp_path, monkeypatch
):
    template = _write_competition_template(tmp_path)
    course = tmp_path / "course.yaml"
    approved_payload = course.read_bytes()
    changed_course = json.loads(json.dumps(COURSE_DOCUMENT))
    changed_course["waypoints"]["F2"]["x"] = -152.39
    original_read_text = Path.read_text

    def change_course_after_validation_read(path, *args, **kwargs):
        payload = original_read_text(path, *args, **kwargs)
        if path == course:
            path.write_text(json.dumps(changed_course), encoding="utf-8")
        return payload

    monkeypatch.setattr(Path, "read_text", change_course_after_validation_read)

    resolved = resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)

    assert resolved.competition.course_sha256 == hashlib.sha256(
        approved_payload
    ).hexdigest()


def test_omitted_runtime_profile_resolves_to_frozen_phase_2_compatibility(tmp_path):
    resolved = resolve_run_config(
        _write_template(tmp_path, _phase2_document()),
        run_id_factory=lambda: FIXED_RUN_ID,
    )

    assert resolved.runtime_profile == "phase2"
    assert resolved.simulation is None
    assert resolved.expected_camera_frames == 40
    assert resolved.topology == config_module.RuntimeTopology(
        profile="phase2",
        ownership=config_module.PHASE2_OWNERSHIP,
    )
    with pytest.raises(FrozenInstanceError):
        resolved.topology.profile = "phase3"


def test_phase3_simulation_config_derives_exact_frame_count(tmp_path):
    value = _phase2_document()
    value["runtime_profile"] = "phase3"
    value["simulation"] = {
        "seed": 7,
        "duration_sim_seconds": 0.15,
        "public_epoch_native_sim_seconds": 90.0,
        "target_real_time_factor": 0.1,
    }

    config = resolve_run_config(
        _write_template(tmp_path, value),
        run_id_factory=lambda: FIXED_RUN_ID,
    )

    assert config.runtime_profile == "phase3"
    assert config.simulation.seed == 7
    assert config.simulation.duration_ns == 150_000_000
    assert config.simulation.target_real_time_factor == 0.1
    assert config.expected_camera_frames == 3
    assert config.topology == config_module.RuntimeTopology(
        profile="phase3",
        ownership=config_module.PHASE3_OWNERSHIP,
    )


def test_phase3_simulation_config_stores_public_epoch_as_exact_nanoseconds(tmp_path):
    value = _phase2_document()
    value.update(
        runtime_profile="phase3",
        simulation={
            "seed": 7,
            "duration_sim_seconds": 0.15,
            "public_epoch_native_sim_seconds": 90.05,
            "target_real_time_factor": 0.1,
        },
    )

    config = resolve_run_config(
        _write_template(tmp_path, value),
        run_id_factory=lambda: FIXED_RUN_ID,
    )

    assert config.simulation.public_epoch_native_ns == 90_050_000_000


@pytest.mark.parametrize(
    "epoch",
    [
        0,
        -0.05,
        float("nan"),
        90.0000000001,
        90.000000001,
        90.025,
    ],
)
def test_phase3_simulation_rejects_invalid_public_epoch_values(tmp_path, epoch):
    value = _phase2_document()
    value.update(
        runtime_profile="phase3",
        simulation={
            "seed": 1,
            "duration_sim_seconds": 2.0,
            "public_epoch_native_sim_seconds": epoch,
            "target_real_time_factor": 0.1,
        },
    )

    with pytest.raises(ValueError):
        resolve_run_config(
            _write_template(tmp_path, value),
            run_id_factory=lambda: FIXED_RUN_ID,
        )


def test_large_finite_integral_duration_normalizes_without_float_overflow(tmp_path):
    duration_seconds = 10**400
    value = _phase2_document()
    value.update(
        runtime_profile="phase3",
        simulation={
            "seed": 1,
            "duration_sim_seconds": duration_seconds,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0.1,
        },
    )

    config = resolve_run_config(
        _write_template(tmp_path, value),
        run_id_factory=lambda: FIXED_RUN_ID,
    )

    assert config.simulation.duration_ns == duration_seconds * 1_000_000_000


@pytest.mark.parametrize(
    ("profile", "ownership"),
    [
        ("phase3", config_module.PHASE2_OWNERSHIP),
        ("phase2", config_module.PHASE3_OWNERSHIP),
        ("ambient", config_module.PHASE2_OWNERSHIP),
    ],
)
def test_runtime_topology_rejects_noncanonical_profile_ownership_pairs(
    profile, ownership
):
    with pytest.raises(ValueError):
        config_module.RuntimeTopology(profile, ownership)


@pytest.mark.parametrize(
    "simulation",
    [
        {
            "seed": -1,
            "duration_sim_seconds": 2.0,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0.1,
        },
        {
            "seed": 2**32,
            "duration_sim_seconds": 2.0,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0.1,
        },
        {
            "seed": 1,
            "duration_sim_seconds": 0,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0.1,
        },
        {
            "seed": 1,
            "duration_sim_seconds": 0.075,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0.1,
        },
        {
            "seed": 1,
            "duration_sim_seconds": 2.0,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0,
        },
        {
            "seed": 1,
            "duration_sim_seconds": 2.0,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0.5,
        },
        {
            "seed": True,
            "duration_sim_seconds": 2.0,
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0.1,
        },
        {
            "seed": 1,
            "duration_sim_seconds": float("nan"),
            "public_epoch_native_sim_seconds": 90.0,
            "target_real_time_factor": 0.1,
        },
    ],
)
def test_invalid_phase3_timing_is_rejected_before_compose(tmp_path, simulation):
    value = _phase2_document()
    value.update(runtime_profile="phase3", simulation=simulation)

    with pytest.raises(ValueError):
        resolve_run_config(
            _write_template(tmp_path, value),
            run_id_factory=lambda: FIXED_RUN_ID,
        )


@pytest.mark.parametrize(
    "updates",
    [
        {"runtime_profile": "phase3"},
        {
            "simulation": {
                "seed": 1,
                "duration_sim_seconds": 2.0,
                "public_epoch_native_sim_seconds": 90.0,
                "target_real_time_factor": 0.1,
            }
        },
        {
            "runtime_profile": "phase2",
            "simulation": {
                "seed": 1,
                "duration_sim_seconds": 2.0,
                "public_epoch_native_sim_seconds": 90.0,
                "target_real_time_factor": 0.1,
            },
        },
    ],
)
def test_profile_and_simulation_must_form_a_valid_pair(tmp_path, updates):
    value = _phase2_document()
    value.update(updates)

    with pytest.raises(ValueError):
        resolve_run_config(
            _write_template(tmp_path, value),
            run_id_factory=lambda: FIXED_RUN_ID,
        )


def test_run_template_and_recording_are_frozen(tmp_path):
    resolved = resolve_run_config(
        _write_qgc_template(tmp_path), run_id_factory=lambda: FIXED_RUN_ID
    )
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
        runtime_profile=resolved.runtime_profile,
        simulation=resolved.simulation,
    )

    with pytest.raises(FrozenInstanceError):
        template.world = "other"
    with pytest.raises(FrozenInstanceError):
        template.recording.fps = 10


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_config_schemas_are_valid_and_accept_phase3_qgc_run(schema_name):
    validator = _load_validator(schema_name)
    document = _schema_competition_document(schema_name)

    validator.validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_config_schemas_accept_exact_three_frame_duration(schema_name):
    document = _schema_competition_document(schema_name)
    document["simulation"]["duration_sim_seconds"] = 0.15

    _load_validator(schema_name).validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_config_schemas_accept_public_epoch_on_exact_50_ms_grid(schema_name):
    document = _schema_competition_document(schema_name)
    document["simulation"]["public_epoch_native_sim_seconds"] = 90.0

    _load_validator(schema_name).validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
@pytest.mark.parametrize("epoch", [0, 90.025, 90.0000000001, 90.000000001])
def test_config_schemas_reject_invalid_public_epoch_values(schema_name, epoch):
    document = _schema_competition_document(schema_name)
    document["simulation"]["public_epoch_native_sim_seconds"] = epoch

    with pytest.raises(ValidationError):
        _load_validator(schema_name).validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_config_schemas_accept_omitted_phase_2_profile(schema_name):
    document = _phase2_document()
    if schema_name == "run.schema.json":
        document = {
            "run_id": str(FIXED_RUN_ID),
            **document,
            "output_root": str((ROOT / "../runs").resolve()),
            "config_sha256": "a" * 64,
        }

    _load_validator(schema_name).validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
@pytest.mark.parametrize(
    "updates",
    [
        pytest.param({"runtime_profile": "phase3"}, id="phase3-without-simulation"),
        pytest.param(
            {
                "simulation": {
                    "seed": 1,
                    "duration_sim_seconds": 2.0,
                    "target_real_time_factor": 0.1,
                }
            },
            id="omitted-profile-with-simulation",
        ),
        pytest.param(
            {
                "runtime_profile": "phase2",
                "simulation": {
                    "seed": 1,
                    "duration_sim_seconds": 2.0,
                    "public_epoch_native_sim_seconds": 90.0,
                    "target_real_time_factor": 0.1,
                },
            },
            id="phase2-with-simulation",
        ),
    ],
)
def test_config_schemas_reject_invalid_profile_simulation_pairs(schema_name, updates):
    document = _phase2_document()
    document.update(updates)
    if schema_name == "run.schema.json":
        document.update(run_id=str(FIXED_RUN_ID), config_sha256="a" * 64)

    with pytest.raises(ValidationError):
        _load_validator(schema_name).validate(document)


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
            lambda value: value["recording"].update({"width_px": 320}),
            id="wrong-even-width",
        ),
        pytest.param(
            lambda value: value["recording"].update({"height_px": 240}),
            id="wrong-even-height",
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
def test_config_schemas_reject_invalid_phase_3_bindings(schema_name, mutate):
    validator = _load_validator(schema_name)
    document = _schema_competition_document(schema_name)
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
            lambda value: value["recording"].update({"width_px": 320}),
            id="wrong-even-width",
        ),
        pytest.param(
            lambda value: value["recording"].update({"height_px": 240}),
            id="wrong-even-height",
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
    template = _write_qgc_template(tmp_path)
    invocation_directory = tmp_path / "invocation"
    invocation_directory.mkdir()
    monkeypatch.chdir(invocation_directory)

    resolved = resolve_run_config(template, run_id_factory=lambda: FIXED_RUN_ID)

    assert resolved.output_root == invocation_directory / "runs"


def test_write_resolved_config_creates_schema_valid_exclusive_snapshot(tmp_path):
    source = tmp_path / "source"
    source.mkdir()
    resolved = resolve_run_config(
        _write_qgc_template(source), run_id_factory=lambda: FIXED_RUN_ID
    )
    run_dir = tmp_path / resolved.run_id

    written = write_resolved_config(run_dir, resolved)

    assert written == run_dir / "configuration/run.json"
    assert sorted(
        path.relative_to(run_dir).as_posix() for path in run_dir.rglob("*")
    ) == [
        "configuration",
        "configuration/course.yaml",
        "configuration/deployment-profile.json",
        "configuration/listener-session.json",
        "configuration/qgc-actions.json",
        "configuration/qgc-runtime.json",
        "configuration/run.json",
        "configuration/scenario.yaml",
    ]
    document = json.loads(written.read_text(encoding="utf-8"))
    _load_validator("run.schema.json").validate(document)
    assert document["competition"] == {
        "course": "course.yaml",
        "scenario": "scenario.yaml",
        "course_sha256": resolved.competition.course_sha256,
        "scenario_sha256": resolved.competition.scenario_sha256,
    }
    loaded = load_run_config(written)
    assert replace(
        loaded,
        competition=replace(
            loaded.competition,
            course_source=resolved.competition.course_source,
            scenario_source=resolved.competition.scenario_source,
        ),
    ) == resolved

    with pytest.raises(FileExistsError):
        write_resolved_config(run_dir, resolved)
    assert json.loads(written.read_text(encoding="utf-8")) == document


def test_write_resolved_config_rejects_invalid_run_id_before_creating_snapshot(
    tmp_path,
):
    source = tmp_path / "source"
    source.mkdir()
    resolved = resolve_run_config(
        _write_qgc_template(source), run_id_factory=lambda: FIXED_RUN_ID
    )
    document = config_module._document_without_checksum(resolved)
    document["run_id"] = "not-a-uuid"
    checksum = hashlib.sha256(
        json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    invalid = replace(
        resolved,
        run_id="not-a-uuid",
        config_sha256=checksum,
    )

    with pytest.raises(ValueError, match="run_id"):
        write_resolved_config(tmp_path, invalid)
    assert not (tmp_path / "configuration/run.json").exists()


@pytest.mark.parametrize("malformation", ["phase2-qgc", "common"])
def test_write_rejects_structural_contract_before_creating_directories(
    tmp_path, monkeypatch, malformation
):
    source = tmp_path / "source"
    source.mkdir()
    resolved = resolve_run_config(
        _write_qgc_template(source), run_id_factory=lambda: FIXED_RUN_ID
    )
    if malformation == "phase2-qgc":
        invalid = replace(resolved, runtime_profile="phase2", simulation=None)
    else:
        invalid = replace(resolved, max_wall_seconds=0)
    invalid = replace(
        invalid,
        config_sha256=config_module._checksum(
            config_module._document_without_checksum(invalid)
        ),
    )
    run_directory = tmp_path / "run"
    side_effects: list[str] = []

    def reject_side_effect(name):
        def reject(*_args, **_kwargs):
            side_effects.append(name)
            raise AssertionError(f"unexpected {name}")

        return reject

    monkeypatch.setattr(config_module, "_open_directory_nofollow", reject_side_effect("open"))
    monkeypatch.setattr(config_module, "_require_source_file", reject_side_effect("read"))
    monkeypatch.setattr(config_module, "_write_exclusive", reject_side_effect("write"))
    original_mkdir = Path.mkdir

    def reject_run_mkdir(path, *args, **kwargs):
        if path == run_directory or run_directory in path.parents:
            side_effects.append("mkdir")
            raise AssertionError("unexpected mkdir")
        return original_mkdir(path, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", reject_run_mkdir)

    with pytest.raises(ValueError):
        write_resolved_config(run_directory, invalid)

    assert not run_directory.exists()
    assert side_effects == []


@pytest.mark.parametrize("mission", ["controlled_descent", "autotune_roll", "hover_roll"])
def test_write_rejects_qgc_for_non_comp2026_before_filesystem_effects(
    tmp_path, monkeypatch, mission
):
    source = tmp_path / "source"
    source.mkdir()
    resolved = resolve_run_config(
        _write_qgc_template(source), run_id_factory=lambda: FIXED_RUN_ID
    )
    invalid = replace(
        resolved,
        mission=mission,
        recording=RecordingConfig(320, 240, 20, "rgb8"),
    )
    invalid = replace(
        invalid,
        config_sha256=config_module._checksum(
            config_module._document_without_checksum(invalid)
        ),
    )
    run_directory = tmp_path / "run"
    side_effects: list[str] = []

    def reject_side_effect(name):
        def reject(*_args, **_kwargs):
            side_effects.append(name)
            raise AssertionError(f"unexpected {name}")

        return reject

    monkeypatch.setattr(config_module, "_open_directory_nofollow", reject_side_effect("open"))
    monkeypatch.setattr(config_module, "_require_source_file", reject_side_effect("read"))
    monkeypatch.setattr(config_module, "_write_exclusive", reject_side_effect("write"))
    original_mkdir = Path.mkdir

    def reject_run_mkdir(path, *args, **kwargs):
        if path == run_directory or run_directory in path.parents:
            side_effects.append("mkdir")
            raise AssertionError("unexpected mkdir")
        return original_mkdir(path, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", reject_run_mkdir)

    with pytest.raises(ValueError, match="QGC.*comp2026_auto"):
        write_resolved_config(run_directory, invalid)
    assert not run_directory.exists()
    assert side_effects == []


def test_load_run_config_rejects_checksum_mismatch(tmp_path):
    document = _phase2_document()
    document.update(
        run_id=str(FIXED_RUN_ID),
        output_root=str((ROOT / "../runs").resolve()),
        config_sha256="a" * 64,
    )
    document["world"] = "other"
    path = tmp_path / "run.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="config_sha256"):
        load_run_config(path)
