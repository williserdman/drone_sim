from pathlib import Path
import importlib

from drone_sim_companion import comp2026_smoke


ROOT = Path(__file__).resolve().parents[2]


def test_comp2026_smoke_imports_full_auto_closure(monkeypatch, capsys):
    monkeypatch.syspath_prepend(str(ROOT / "companion/comp2026/src"))
    assert comp2026_smoke.main([]) == 0
    assert "FM1/FM2/FM3" in capsys.readouterr().out


def test_comp2026_smoke_reports_missing_runtime_module(monkeypatch, capsys):
    original = importlib.import_module

    def missing(name):
        if name == "drone.control.drone_control":
            raise ModuleNotFoundError("No module named 'drone.control.stability'")
        return original(name)

    monkeypatch.setattr(importlib, "import_module", missing)
    assert comp2026_smoke.main([]) == 1
    assert "drone.control.stability" in capsys.readouterr().err
