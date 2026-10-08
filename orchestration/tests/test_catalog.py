from pathlib import Path

import pytest

from orchestration.catalog import PresetCatalog


ROOT = Path(__file__).resolve().parents[2]


def test_catalog_filters_complete_approved_tuples_and_identifies_manual_mission():
    catalog = PresetCatalog(ROOT)
    rows = catalog.list()
    assert len(rows) == 11
    assert {row.world for row in rows} == {
        "vertical_descent", "competition_mission", "moving_pad_landing", "moving_pad_stationary"
    }
    assert catalog.get("operator-wait").manual_arming is True
    assert catalog.get("configured-descent").manual_arming is False
    assert catalog.get("reload-validation").requires_calibration is True
    for row in catalog.list(world="competition_mission"):
        assert row.vehicle == "iris_competition"
        assert row.mission == "comp2026_auto"
        assert row.configuration["recording"]["width_px"] == 640


@pytest.mark.parametrize("name", ["foundation", "../config/default-run.json", "unknown"])
def test_catalog_rejects_unapproved_presets(name):
    with pytest.raises(ValueError, match="unknown preset"):
        PresetCatalog(ROOT).get(name)
