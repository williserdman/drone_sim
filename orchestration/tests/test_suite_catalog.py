import hashlib
import json
from pathlib import Path

import pytest

from orchestration.config import (
    CALIBRATION_PARAMETERS,
    CalibrationImport,
    resolve_run_config,
)
from orchestration.suite import (
    load_suite_catalog,
    prepare_suite_template,
    write_suite_report,
)


ROOT = Path(__file__).resolve().parents[2]


def test_catalog_has_all_flight_setups_in_required_order():
    cases = load_suite_catalog(ROOT / "config/ci-suite.json", project_directory=ROOT)

    assert [case.name for case in cases] == [
        "calibration", "reload-validation", "configured-descent", "operator-wait",
        "controlled-descent", "hover-roll", "autotune-roll", "moving-pad",
        "stationary-pad", "competition-slow", "competition-realtime",
    ]
    assert [case.acceptance for case in cases] == [
        "calibration", "reload_validation", "configured_descent", "operator_wait",
        "controlled_descent", "hover_roll", "autotune_roll", "moving_pad",
        "moving_pad", "competition", "competition",
    ]
    assert len({case.template for case in cases}) == 11
    assert cases[-1].template == ROOT / "config/realtime-run.json"


def test_catalog_rejects_an_uncatalogued_top_level_flight_template(tmp_path):
    project = tmp_path / "project"
    config = project / "config"
    config.mkdir(parents=True)
    source_catalog = json.loads((ROOT / "config/ci-suite.json").read_text())
    for item in source_catalog["cases"]:
        source = ROOT / item["template"]
        target = project / item["template"]
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
    (config / "ci-suite.json").write_text(json.dumps(source_catalog))
    extra = json.loads((ROOT / "config/vertical-descent-run.json").read_text())
    (config / "new-flight.json").write_text(json.dumps(extra))

    with pytest.raises(ValueError, match="missing flight templates.*new-flight.json"):
        load_suite_catalog(config / "ci-suite.json", project_directory=project)


def test_prepared_competition_template_preserves_relative_inputs(tmp_path):
    case = load_suite_catalog(
        ROOT / "config/ci-suite.json", project_directory=ROOT
    )[-2]
    original = json.loads(case.template.read_text())
    output_root = (tmp_path / "runs").resolve()
    calibration = (tmp_path / "runs/calibration-id").resolve()

    prepared_path = prepare_suite_template(
        case,
        suite_directory=tmp_path / "runs/suites/suite-id",
        output_root=output_root,
        calibration_source=calibration,
    )
    prepared = json.loads(prepared_path.read_text())

    assert prepared_path == tmp_path / "runs/suites/suite-id/configuration/competition-slow/run.json"
    assert prepared["output_root"] == str(output_root)
    assert prepared["calibration"]["source_run_directory"] == str(calibration)
    assert prepared["competition"] == {
        "course": "course.yaml",
        "scenario": "scenario.yaml",
    }
    assert (prepared_path.parent / "course.yaml").read_bytes() == (ROOT / "config/course.yaml").read_bytes()
    assert (prepared_path.parent / "scenario.yaml").read_bytes() == (ROOT / "config/scenario.yaml").read_bytes()
    for key in ("mission_plan", "simulation", "recording"):
        assert prepared.get(key) == original.get(key)


def test_prepared_template_resolves_existing_calibration_reference_before_relocation(tmp_path):
    case = load_suite_catalog(
        ROOT / "config/ci-suite.json", project_directory=ROOT
    )[1]

    prepared_path = prepare_suite_template(
        case,
        suite_directory=tmp_path / "suite",
        output_root=tmp_path / "runs",
        calibration_source=None,
    )

    prepared = json.loads(prepared_path.read_text())
    assert prepared["calibration"]["source_run_directory"] == str(
        (ROOT / "runs/SOURCE_RUN_ID").resolve()
    )


def test_every_prepared_template_resolves_as_a_run_configuration(tmp_path):
    calibration_source = (tmp_path / "accepted-calibration").resolve()
    artifact = calibration_source / "ardupilot_sitl/autotune.parm"
    manifest = calibration_source / "manifest.json"
    artifact.parent.mkdir(parents=True)
    artifact.write_text("ATC_RAT_RLL_P 0.1\n")
    manifest.write_text("{}\n")
    calibration = {
        "schema_version": 1,
        "source_run_id": "00000000-0000-4000-8000-000000000001",
        "source_manifest_sha256": "a" * 64,
        "source_artifact_sha256": "b" * 64,
        "gains": {name: 0.1 for name in CALIBRATION_PARAMETERS},
        "profile": {},
    }

    def importer(_path, _vehicle, _scenario):
        return CalibrationImport(json.dumps(calibration), artifact, manifest)

    cases = load_suite_catalog(ROOT / "config/ci-suite.json", project_directory=ROOT)
    for case in cases:
        prepared = prepare_suite_template(
            case,
            suite_directory=tmp_path / "suite",
            output_root=tmp_path / "runs",
            calibration_source=None if case.acceptance == "calibration" else calibration_source,
        )
        resolved = resolve_run_config(
            prepared,
            calibration_importer=importer,
            calibration_profile_factory=lambda: {},
        )
        if resolved.competition is not None:
            course_bytes = (ROOT / "config/course.yaml").read_bytes()
            scenario_bytes = (ROOT / "config/scenario.yaml").read_bytes()
            assert resolved.competition.course_source.read_bytes() == course_bytes
            assert resolved.competition.scenario_source.read_bytes() == scenario_bytes
            assert resolved.competition.course_sha256 == hashlib.sha256(course_bytes).hexdigest()
            assert resolved.competition.scenario_sha256 == hashlib.sha256(scenario_bytes).hexdigest()


def test_report_json_and_markdown_keep_independent_outcomes_and_bundle_links(tmp_path):
    suite_directory = tmp_path / "runs/suites/suite-id"
    report = {
        "schema_version": 1,
        "suite_id": "suite-id",
        "state": "FAILED",
        "reason": "artifact validation failed",
        "started_at": "2026-10-05T10:00:00Z",
        "finished_at": "2026-10-05T11:00:00Z",
        "source_revisions": [],
        "image_digests": [],
        "catalog_sha256": "abc",
        "template_sha256s": {},
        "calibration_run_id": None,
        "reload_validation_run_id": None,
        "cases": [{
            "name": "hover-roll",
            "template": "config/hover-roll-run.json",
            "acceptance_contract": "hover_roll",
            "status": "failed",
            "run_id": "run-id",
            "lifecycle": "FAILED",
            "reason": "invalid artifacts",
            "physical_outcome": "LANDED",
            "score": {"earned": 60, "maximum": 100},
            "artifact_acceptance": {
                "accepted": False,
                "reason": "video missing",
                "report": {"score_accepted": True},
            },
            "teardown_diagnostics": None,
            "bundle_path": "../../run-id",
        }],
    }

    write_suite_report(suite_directory, report)

    assert json.loads((suite_directory / "report.json").read_text()) == report
    markdown = (suite_directory / "report.md").read_text()
    assert "FAILED" in markdown
    assert "60/100" in markdown
    assert "video missing" in markdown
    assert "[../../run-id](../../run-id)" in markdown
    assert not list(suite_directory.glob(".report.*.tmp"))


@pytest.mark.parametrize(
    "change, message",
    [
        ({"schema_version": "1"}, "schema_version"),
        ({"cases": "bad"}, "cases"),
        ({"cases": [{"name": "bad name", "template": "config/default-run.json", "acceptance": "competition"}]}, "name"),
        ({"cases": [{"name": "competition-slow", "template": "config/default-run.json", "acceptance": "unknown"}]}, "acceptance"),
    ],
)
def test_catalog_rejects_invalid_types_names_and_contracts(tmp_path, change, message):
    catalog = json.loads((ROOT / "config/ci-suite.json").read_text())
    catalog.update(change)
    path = tmp_path / "catalog.json"
    path.write_text(json.dumps(catalog))

    with pytest.raises(ValueError, match=message):
        load_suite_catalog(path, project_directory=ROOT)


def test_catalog_rejects_valid_but_wrong_case_name(tmp_path):
    catalog = json.loads((ROOT / "config/ci-suite.json").read_text())
    catalog["cases"][0]["name"] = "other-calibration"
    path = tmp_path / "catalog.json"
    path.write_text(json.dumps(catalog))

    with pytest.raises(ValueError, match="required cases in order"):
        load_suite_catalog(path, project_directory=ROOT)
