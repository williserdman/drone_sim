from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from drone_sim_companion.runtime_node import RuntimeConfig


RUN_ID = "00000000-0000-4000-8000-000000000001"


@pytest.mark.parametrize("tamper", [False, True])
def test_search_delivery_loads_checksum_bound_sources_before_live_io(tmp_path, tamper):
    repository = Path(__file__).parents[2]
    run_directory = tmp_path / RUN_ID
    configuration = run_directory / "configuration"
    configuration.mkdir(parents=True)
    sources = {}
    for label in ("course", "scenario"):
        content = (repository / f"config/{label}-search-delivery.yaml").read_bytes()
        (configuration / f"{label}.yaml").write_bytes(content)
        sources[label] = f"{label}.yaml"
        sources[f"{label}_sha256"] = hashlib.sha256(content).hexdigest()
    document = {
        "run_id": RUN_ID, "startup_wall_seconds": 120,
        "max_wall_seconds": 5400, "finalization_wall_seconds": 600,
        "mission": "configured", "scenario": "search_delivery_v1",
        "runtime_profile": "phase3", "simulation": {}, "competition": sources,
        "mission_plan": {"schema_version": 1, "steps": [
            {"tool": "mission_event", "args": {"phase": "SEARCH", "state": "STARTED"}},
            {"tool": "precision_land", "args": {"aruco_id": 3}},
        ]},
    }
    path = configuration / "run.json"
    path.write_text(json.dumps(document))
    environment = {"SIM_RUN_ID": RUN_ID, "SIM_RUN_DIRECTORY": str(run_directory),
                   "SIM_CONFIG_PATH": str(path)}
    if tamper:
        (configuration / "scenario.yaml").write_text("payloads: []\n")
        with pytest.raises(ValueError, match="SHA-256"):
            RuntimeConfig.from_environment(environment)
    else:
        config = RuntimeConfig.from_environment(environment)
        assert config.course_path == configuration / "course.yaml"
        assert config.scenario_path == configuration / "scenario.yaml"
        assert config.qgc is None


def test_configured_competition_loads_frozen_sources_without_qgc(tmp_path: Path) -> None:
    run_directory = tmp_path / RUN_ID
    configuration = run_directory / "configuration"
    configuration.mkdir(parents=True)
    repository = Path(__file__).parents[2]
    course = (repository / "config/course.yaml").read_bytes()
    scenario = (repository / "config/scenario.yaml").read_bytes()
    (configuration / "course.yaml").write_bytes(course)
    (configuration / "scenario.yaml").write_bytes(scenario)
    config_path = configuration / "run.json"
    config_path.write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "startup_wall_seconds": 120,
                "max_wall_seconds": 5400,
                "finalization_wall_seconds": 600,
                "mission": "configured",
                "scenario": "competition_v1",
                "runtime_profile": "phase3",
                "simulation": {
                    "seed": 2026,
                    "duration_sim_seconds": 600.0,
                    "public_epoch_native_sim_seconds": 90.0,
                    "target_real_time_factor": 0.25,
                },
                "competition": {
                    "course": "course.yaml",
                    "scenario": "scenario.yaml",
                    "course_sha256": hashlib.sha256(course).hexdigest(),
                    "scenario_sha256": hashlib.sha256(scenario).hexdigest(),
                },
                "mission_plan": {
                    "schema_version": 1,
                    "steps": [
                        {
                            "tool": "set_mode",
                            "args": {"mode": "GUIDED"},
                            "timeout_sim_s": 60,
                        }
                    ],
                },
            }
        ),
        encoding="utf-8",
    )

    loaded = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": RUN_ID,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )

    assert (loaded.mission, loaded.scenario) == ("configured", "competition_v1")
    assert loaded.course_path == configuration / "course.yaml"
    assert loaded.scenario_path == configuration / "scenario.yaml"
    assert loaded.qgc is None


def test_configured_competition_tools_require_competition_scenario(
    tmp_path: Path,
) -> None:
    run_directory = tmp_path / RUN_ID
    config_path = run_directory / "configuration/run.json"
    config_path.parent.mkdir(parents=True)
    config_path.write_text(
        json.dumps(
            {
                "run_id": RUN_ID,
                "startup_wall_seconds": 120,
                "max_wall_seconds": 3600,
                "finalization_wall_seconds": 120,
                "mission": "configured",
                "scenario": "descent_v1",
                "runtime_profile": "phase3",
                "simulation": {},
                "mission_plan": {
                    "schema_version": 1,
                    "steps": [
                        {
                            "tool": "release_payload",
                            "args": {"aruco_id": 2},
                            "timeout_sim_s": 60,
                        }
                    ],
                },
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="competition mission tools.*competition_v1"):
        RuntimeConfig.from_environment(
            {
                "SIM_RUN_ID": RUN_ID,
                "SIM_RUN_DIRECTORY": str(run_directory),
                "SIM_CONFIG_PATH": str(config_path),
            }
        )
