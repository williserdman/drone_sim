"""Keep the imported mission changes limited to the documented testbench hooks."""

from pathlib import Path
import runpy
import shutil

import pytest


ROOT = Path(__file__).resolve().parents[1]
AUDIT = runpy.run_path(str(ROOT / "scripts/audit-comp2026-hooks.py"))["audit"]


@pytest.fixture
def source_copy(tmp_path):
    companion = tmp_path / "companion"
    companion.mkdir()
    shutil.copyfile(ROOT / "companion/comp2026-source.json", companion / "comp2026-source.json")
    shutil.copytree(ROOT / "companion/comp2026", companion / "comp2026",
                    ignore=shutil.ignore_patterns("__pycache__", ".pytest_cache"))
    return tmp_path


def test_actual_source_matches_documented_hook_boundary():
    result = AUDIT(ROOT)
    assert result["upstream_revision"] == "95dec5105573d1a18b2ad824975d5edf99893006"
    assert len(result["changed_upstream_files"]) == 7


def test_mission_edits_are_rejected(source_copy):
    path = source_copy / "companion/comp2026/src/drone/missions/fm2.py"
    path.write_bytes(path.read_bytes() + b"\nUNREVIEWED_MISSION_CHANGE = True\n")
    with pytest.raises(ValueError, match="changed outside hook scope.*fm2.py"):
        AUDIT(source_copy)


def test_flight_logic_edits_in_hooked_module_are_rejected(source_copy):
    path = source_copy / "companion/comp2026/src/drone/control/drone_control.py"
    path.write_bytes(path.read_bytes() + b"\nUNREVIEWED_FLIGHT_CHANGE = True\n")
    with pytest.raises(ValueError, match="changed flight logic outside injected hooks"):
        AUDIT(source_copy)


def test_undocumented_source_addition_is_rejected(source_copy):
    (source_copy / "companion/comp2026/src/drone/replacement.py").write_text("pass\n")
    with pytest.raises(ValueError, match="unexpected source addition"):
        AUDIT(source_copy)
