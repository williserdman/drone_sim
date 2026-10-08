from pathlib import Path
import importlib
import os
import shutil
import subprocess
import sys

from drone_sim_companion import comp2026_smoke


ROOT = Path(__file__).resolve().parents[2]


def test_comp2026_smoke_imports_full_auto_closure(monkeypatch, capsys):
    monkeypatch.syspath_prepend(str(ROOT / "companion/comp2026/src"))
    assert comp2026_smoke.main([]) == 0
    output = capsys.readouterr().out
    assert "FM1/FM2/FM3" in output
    assert "guarded control/policy" in output


def test_filtered_comp2026_build_context_passes_import_smoke(tmp_path):
    context = tmp_path / "context"
    shutil.copytree(ROOT / "companion/src", context / "companion/src")
    dockerignore = (ROOT / ".dockerignore").read_text().splitlines()
    admitted = [
        line[2:]
        for line in dockerignore
        if line.startswith("!/companion/comp2026/src/") and not line.endswith("/")
    ]
    for relative in admitted:
        source = ROOT / relative
        target = context / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)

    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(context / "companion/src"), str(context / "companion/comp2026/src"))
    )
    result = subprocess.run(
        [sys.executable, "-m", "drone_sim_companion.comp2026_smoke"],
        cwd=context,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "guarded control/policy" in result.stdout


def test_dockerfile_installs_policy_resources_at_canonical_paths(tmp_path):
    dockerfile = (ROOT / "companion/Dockerfile").read_text().splitlines()
    expected = {
        "config/course.yaml": "/opt/drone_sim/config/course.yaml",
        "config/scenario.yaml": "/opt/drone_sim/config/scenario.yaml",
        "ardupilot_sitl/provenance/ardupilot.json": (
            "/opt/drone_sim/ardupilot_sitl/provenance/ardupilot.json"
        ),
        "ardupilot_sitl/params/descent.parm": (
            "/opt/drone_sim/ardupilot_sitl/params/descent.parm"
        ),
        "ardupilot_sitl/params/competition.parm": (
            "/opt/drone_sim/ardupilot_sitl/params/competition.parm"
        ),
        "gazebo/resources/models/iris_competition/model.sdf": (
            "/opt/drone_sim/gazebo/resources/models/iris_competition/model.sdf"
        ),
        "gazebo/resources/worlds/competition_mission.sdf": (
            "/opt/drone_sim/gazebo/resources/worlds/competition_mission.sdf"
        ),
        "gazebo/resources/worlds/competition_mission_1x.sdf": (
            "/opt/drone_sim/gazebo/resources/worlds/competition_mission_1x.sdf"
        ),
    }
    copies = {
        parts[1]: parts[2]
        for line in dockerfile
        if (parts := line.split()) and parts[0] == "COPY" and len(parts) == 3
    }

    for source, destination in expected.items():
        assert copies.get(source) == destination
        target = tmp_path / destination.removeprefix("/opt/drone_sim/")
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / source, target)
        assert target.read_bytes() == (ROOT / source).read_bytes()


def test_comp2026_smoke_reports_missing_runtime_module(monkeypatch, capsys):
    original = importlib.import_module

    def missing(name):
        if name == "drone.control.drone_control":
            raise ModuleNotFoundError("No module named 'drone.control.stability'")
        return original(name)

    monkeypatch.setattr(importlib, "import_module", missing)
    assert comp2026_smoke.main([]) == 1
    assert "drone.control.stability" in capsys.readouterr().err
