from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from types import ModuleType
from uuid import UUID

import pytest

import drone_sim_companion.runtime_node as runtime_node
from drone_sim_companion.mission_plan import MissionPlan
from drone_sim_companion.runtime_node import RuntimeConfig
from orchestration.config import resolve_run_config, write_resolved_config


ROOT = Path(__file__).parents[2]
CONFIG = ROOT / "config"
RUN_ID = UUID("22222222-2222-4222-8222-222222222222")


def _resolved_environment(tmp_path: Path, template: Path) -> dict[str, str]:
    resolved = resolve_run_config(template, run_id_factory=lambda: RUN_ID)
    run_directory = tmp_path / resolved.run_id
    config_path = write_resolved_config(run_directory, resolved)
    return {
        "SIM_RUN_ID": resolved.run_id,
        "SIM_RUN_DIRECTORY": str(run_directory),
        "SIM_CONFIG_PATH": str(config_path),
    }


def _rewrite_checksum(document: dict) -> None:
    without_checksum = {
        key: value for key, value in document.items() if key != "config_sha256"
    }
    canonical = json.dumps(
        without_checksum, sort_keys=True, separators=(",", ":")
    ).encode()
    document["config_sha256"] = hashlib.sha256(canonical).hexdigest()


def _configured_runtime_module(run_configured) -> ModuleType:
    module = ModuleType("drone_sim_companion.configured_runtime")
    module.run_configured = run_configured
    return module


@pytest.mark.parametrize(
    ("template_name", "expected_tools", "first_args"),
    [
        (
            "configured-descent-run.json",
            ("set_mode", "arm", "takeoff", "hold", "land"),
            {"mode": "GUIDED"},
        ),
        (
            "configured-operator-run.json",
            ("wait_for_state", "takeoff", "hold", "land"),
            {"armed": True, "mode": "GUIDED"},
        ),
    ],
)
def test_runtime_config_loads_resolved_configured_plan(
    tmp_path: Path,
    template_name: str,
    expected_tools: tuple[str, ...],
    first_args: dict[str, object],
) -> None:
    environment = _resolved_environment(tmp_path, CONFIG / template_name)

    config = RuntimeConfig.from_environment(environment)

    assert config.mission == "configured"
    assert isinstance(config.mission_plan, MissionPlan)
    assert tuple(step.tool for step in config.mission_plan.steps) == expected_tools
    assert dict(config.mission_plan.steps[0].args) == first_args
    assert tuple(step.timeout_sim_s for step in config.mission_plan.steps) == (
        60.0,
    ) * len(expected_tools)


def test_unsupported_precision_plan_fails_before_configured_runtime_dispatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    template = json.loads(
        (CONFIG / "configured-descent-run.json").read_text(encoding="utf-8")
    )
    template["mission_plan"]["steps"][0] = {
        "tool": "precision_land",
        "args": {"aruco_id": 7},
    }
    template_path = tmp_path / "precision-run.json"
    template_path.write_text(json.dumps(template), encoding="utf-8")
    environment = _resolved_environment(tmp_path, template_path)
    dispatched: list[RuntimeConfig] = []
    monkeypatch.setitem(
        sys.modules,
        "drone_sim_companion.configured_runtime",
        _configured_runtime_module(lambda config: dispatched.append(config)),
    )
    monkeypatch.setattr(
        runtime_node,
        "_run_comp2026",
        lambda _config: pytest.fail("invalid configured plan reached a live runtime"),
    )
    for name, value in environment.items():
        monkeypatch.setenv(name, value)

    with pytest.raises(ValueError, match="unsupported tool.*precision_land"):
        runtime_node.main()

    assert dispatched == []


def test_runtime_config_rejects_mission_plan_for_other_mission(tmp_path: Path) -> None:
    environment = _resolved_environment(tmp_path, CONFIG / "vertical-descent-run.json")
    config_path = Path(environment["SIM_CONFIG_PATH"])
    document = json.loads(config_path.read_text(encoding="utf-8"))
    document["mission_plan"] = json.loads(
        (CONFIG / "configured-descent-run.json").read_text(encoding="utf-8")
    )["mission_plan"]
    _rewrite_checksum(document)
    config_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises(ValueError, match="mission_plan.*configured"):
        RuntimeConfig.from_environment(environment)


def test_main_dispatches_configured_runtime_with_validated_plan(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    environment = _resolved_environment(
        tmp_path, CONFIG / "configured-descent-run.json"
    )
    dispatched: list[RuntimeConfig] = []
    expected_exit = 23
    monkeypatch.setitem(
        sys.modules,
        "drone_sim_companion.configured_runtime",
        _configured_runtime_module(
            lambda config: dispatched.append(config) or expected_exit
        ),
    )
    monkeypatch.setattr(
        runtime_node,
        "_run_comp2026",
        lambda _config: pytest.fail("configured mission used the QGC host"),
    )
    monkeypatch.setattr(
        runtime_node,
        "_run_controlled_descent",
        lambda _config: pytest.fail("configured mission used controlled descent"),
    )
    monkeypatch.setattr(
        runtime_node,
        "_run_autotune_roll",
        lambda _config: pytest.fail("configured mission used roll diagnostics"),
    )
    for name, value in environment.items():
        monkeypatch.setenv(name, value)

    assert runtime_node.main() == expected_exit
    assert len(dispatched) == 1
    assert dispatched[0].mission == "configured"
    assert isinstance(dispatched[0].mission_plan, MissionPlan)
