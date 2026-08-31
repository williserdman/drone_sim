import hashlib
import json
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
from uuid import UUID

import pytest
import yaml
from jsonschema import Draft202012Validator, FormatChecker, ValidationError

import orchestration.config as config_module
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
VERTICAL_DESCENT_TEMPLATE = CONFIG / "vertical-descent-run.json"
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
) -> Path:
    (tmp_path / "course.yaml").write_text(json.dumps(course), encoding="utf-8")
    (tmp_path / "scenario.yaml").write_text(json.dumps(scenario), encoding="utf-8")
    return _write_template(tmp_path, _competition_document())


def _write_template(tmp_path: Path, document: dict, name: str = "run.json") -> Path:
    path = tmp_path / name
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def test_default_template_resolves_complete_competition_attempt(tmp_path):
    calls = 0

    def fixed_uuid() -> UUID:
        nonlocal calls
        calls += 1
        return FIXED_RUN_ID

    resolved = resolve_run_config(DEFAULT_TEMPLATE, run_id_factory=fixed_uuid)

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
    assert resolved.startup_wall_seconds == 1800
    assert resolved.max_wall_seconds == 14400
    assert resolved.finalization_wall_seconds == 900
    assert resolved.output_root == (ROOT / "../runs").resolve()
    assert resolved.competition is not None
    assert resolved.competition.course_source == CONFIG / "course.yaml"
    assert resolved.competition.scenario_source == CONFIG / "scenario.yaml"
    assert resolved.competition.course_sha256 == hashlib.sha256(
        (CONFIG / "course.yaml").read_bytes()
    ).hexdigest()
    assert resolved.competition.scenario_sha256 == hashlib.sha256(
        (CONFIG / "scenario.yaml").read_bytes()
    ).hexdigest()
    assert calls == 1
    with pytest.raises(FrozenInstanceError):
        resolved.world = "other"
    with pytest.raises(FrozenInstanceError):
        resolved.competition.course_source = CONFIG / "other.yaml"

    written = write_resolved_config(tmp_path, resolved)
    assert (written.parent / "course.yaml").read_bytes() == (
        CONFIG / "course.yaml"
    ).read_bytes()
    assert (written.parent / "scenario.yaml").read_bytes() == (
        CONFIG / "scenario.yaml"
    ).read_bytes()


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
            "target_real_time_factor": 1.0,
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
        runtime_profile=resolved.runtime_profile,
        simulation=resolved.simulation,
    )

    with pytest.raises(FrozenInstanceError):
        template.world = "other"
    with pytest.raises(FrozenInstanceError):
        template.recording.fps = 10


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_config_schemas_are_valid_and_accept_phase_3_default(schema_name):
    validator = _load_validator(schema_name)
    document = (
        json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
        if schema_name == "run-template.schema.json"
        else _resolved_document()
    )

    validator.validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_config_schemas_accept_exact_three_frame_duration(schema_name):
    document = (
        json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
        if schema_name == "run-template.schema.json"
        else _resolved_document()
    )
    document["simulation"]["duration_sim_seconds"] = 0.15

    _load_validator(schema_name).validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
def test_config_schemas_accept_public_epoch_on_exact_50_ms_grid(schema_name):
    document = (
        json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
        if schema_name == "run-template.schema.json"
        else _resolved_document()
    )
    document["simulation"]["public_epoch_native_sim_seconds"] = 90.0

    _load_validator(schema_name).validate(document)


@pytest.mark.parametrize("schema_name", ["run-template.schema.json", "run.schema.json"])
@pytest.mark.parametrize("epoch", [0, 90.025, 90.0000000001, 90.000000001])
def test_config_schemas_reject_invalid_public_epoch_values(schema_name, epoch):
    document = (
        json.loads(DEFAULT_TEMPLATE.read_text(encoding="utf-8"))
        if schema_name == "run-template.schema.json"
        else _resolved_document()
    )
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
    assert sorted(
        path.relative_to(run_dir).as_posix() for path in run_dir.rglob("*")
    ) == [
        "configuration",
        "configuration/course.yaml",
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
        write_resolved_config(run_dir, replace(resolved, world="other"))
    assert json.loads(written.read_text(encoding="utf-8")) == document


def test_write_resolved_config_rejects_invalid_run_id_before_creating_snapshot(
    tmp_path,
):
    resolved = resolve_run_config(DEFAULT_TEMPLATE, run_id_factory=lambda: FIXED_RUN_ID)
    document = _resolved_document()
    document["run_id"] = "not-a-uuid"
    document.pop("config_sha256")
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


def test_load_run_config_rejects_checksum_mismatch(tmp_path):
    document = _resolved_document()
    document["world"] = "other"
    path = tmp_path / "run.json"
    path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="config_sha256"):
        load_run_config(path)
