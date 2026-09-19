from __future__ import annotations

import json
from pathlib import Path
from uuid import UUID

from drone_sim_companion.runtime_node import RuntimeConfig
from orchestration.config import resolve_run_config, write_resolved_config


ROOT = Path(__file__).parents[2]
TEMPLATE = ROOT / "config/configured-competition-run.json"
RUN_ID = UUID("33333333-3333-4333-8333-333333333333")


def test_configured_competition_template_normalizes_and_parses_complete_plan(
    tmp_path: Path,
) -> None:
    resolved = resolve_run_config(TEMPLATE, run_id_factory=lambda: RUN_ID)
    run_directory = tmp_path / resolved.run_id
    config_path = write_resolved_config(run_directory, resolved)
    document = json.loads(config_path.read_text(encoding="utf-8"))
    loaded = RuntimeConfig.from_environment(
        {
            "SIM_RUN_ID": resolved.run_id,
            "SIM_RUN_DIRECTORY": str(run_directory),
            "SIM_CONFIG_PATH": str(config_path),
        }
    )

    assert (
        document["mission"],
        document["scenario"],
        document["world"],
        document["vehicle"],
    ) == (
        "configured",
        "competition_v1",
        "competition_mission",
        "iris_competition",
    )
    assert document["simulation"] == {
        "duration_sim_seconds": 420.0,
        "public_epoch_native_sim_seconds": 90.0,
        "seed": 2026,
        "target_real_time_factor": 1.0,
    }
    assert document["recording"] == {
        "width_px": 640,
        "height_px": 480,
        "fps": 20,
        "encoding": "rgb8",
    }
    assert (
        document["startup_wall_seconds"],
        document["max_wall_seconds"],
        document["finalization_wall_seconds"],
    ) == (1800, 14400, 900)
    assert document["competition"] == {
        "course": "course.yaml",
        "scenario": "scenario.yaml",
        "course_sha256": "03a92d6b91363b5f7e9d607219574657f72f08cbe6d558a5ec4ca9ec65ab0774",
        "scenario_sha256": "d45a2c605b2574696433fdf86e2d7e1b6928c3bcf86c2e531b5e10f76da1469b",
    }
    assert "qgc" not in document
    assert loaded.mission_plan is not None
    steps = loaded.mission_plan.steps
    assert tuple(step.tool for step in steps) == (
        "mission_event", "set_mode", "arm", "takeoff", "goto_waypoint", "land", "mission_event",
        "mission_event", "set_mode", "arm", "takeoff", "goto_waypoint", "release_payload", "hold", "mission_event",
        "mission_event", "goto_waypoint", "precision_land", "attach_payload", "set_mode", "arm", "takeoff", "goto_waypoint", "release_payload", "hold", "mission_event",
        "mission_event", "goto_waypoint", "precision_land", "attach_payload", "set_mode", "arm", "takeoff", "goto_waypoint", "release_payload", "hold", "mission_event",
        "mission_event", "goto_waypoint", "land", "mission_event", "mission_event",
    )
    assert [
        (step.args["phase"], step.args["state"])
        for step in steps
        if step.tool == "mission_event"
    ] == [
        ("FM1", "STARTED"),
        ("FM1", "COMPLETE"),
        ("FM2", "STARTED"),
        ("FM2", "COMPLETE"),
        ("FM3_3", "STARTED"),
        ("FM3_3", "COMPLETE"),
        ("FM3_4", "STARTED"),
        ("FM3_4", "COMPLETE"),
        ("HOME", "STARTED"),
        ("HOME", "DISARMED"),
        ("HOME", "COMPLETE"),
    ]
    assert [step.args["aruco_id"] for step in steps if step.tool == "release_payload"] == [2, 3, 4]
    assert [step.args["aruco_id"] for step in steps if step.tool == "attach_payload"] == [3, 4]
    assert [dict(step.args) for step in steps if step.tool == "goto_waypoint"] == [
        {"latitude_deg": 37.4003371, "longitude_deg": -122.08106909807805, "altitude_m": 10.0, "tolerance_m": 0.05},
        {"latitude_deg": 37.4003371, "longitude_deg": -122.08175843013008, "altitude_m": 10.0, "tolerance_m": 0.05},
        {"latitude_deg": 37.400254958050425, "longitude_deg": -122.08055209903902, "altitude_m": 10.0, "tolerance_m": 0.05},
        {"latitude_deg": 37.4003371, "longitude_deg": -122.08175843013008, "altitude_m": 10.0, "tolerance_m": 0.05},
        {"latitude_deg": 37.40041924194958, "longitude_deg": -122.08055209903902, "altitude_m": 10.0, "tolerance_m": 0.05},
        {"latitude_deg": 37.4003371, "longitude_deg": -122.08175843013008, "altitude_m": 10.0, "tolerance_m": 0.05},
        {"latitude_deg": 37.4003371, "longitude_deg": -122.0800351, "altitude_m": 10.0, "tolerance_m": 0.05},
    ]
    assert all(
        steps[index - 1].tool == "arm"
        for index, step in enumerate(steps)
        if step.tool == "takeoff"
    )
    assert tuple(step.timeout_sim_s for step in steps) == tuple(
        150.0 if step.tool == "precision_land" else 90.0 if step.tool == "goto_waypoint" else 60.0
        for step in steps
    )
