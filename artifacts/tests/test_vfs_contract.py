from pathlib import Path

import pytest


ROOT = Path(__file__).parents[2]


def test_vfs_target_override_changes_resolved_aimpoint(tmp_path):
    import yaml
    from artifacts.vfs_contract import (
        load_vfs_course,
        load_vfs_scenario,
        resolve_waypoint_xy,
    )

    course = load_vfs_course(ROOT / "config/course-2027.yaml")
    scenario = load_vfs_scenario(ROOT / "config/scenario-2027.yaml")
    assert resolve_waypoint_xy(course, scenario, payload_target=2) == pytest.approx(
        (-86.106, -2.768)
    )
    document = yaml.safe_load((ROOT / "config/scenario-2027.yaml").read_text())
    document["mission"]["targets"][2] = "A"
    overridden = tmp_path / "scenario.yaml"
    overridden.write_text(yaml.safe_dump(document))
    scenario = load_vfs_scenario(overridden)
    assert resolve_waypoint_xy(course, scenario, payload_target=2) == pytest.approx(
        (-86.106, 3.328)
    )


def test_vfs_configuration_is_frozen_and_cross_validated():
    from artifacts.vfs_contract import (
        load_vfs_course, load_vfs_scenario, validate_vfs_configuration,
    )

    course = load_vfs_course(ROOT / "config/course-2027.yaml")
    scenario = load_vfs_scenario(ROOT / "config/scenario-2027.yaml")
    validate_vfs_configuration(course, scenario)
    with pytest.raises(TypeError):
        scenario.targets[2] = "A"
    with pytest.raises(TypeError):
        course.waypoints["H"] = course.waypoints["D"]
    assert scenario.payload_geometry.total_mass_kg == pytest.approx(1.46077711)


@pytest.mark.parametrize("field,value", [
    ("water_mass_kg", 1.0),
    ("size_m", [0.08, 0.08, 0.0762]),
])
def test_vfs_rejects_payload_that_cannot_meet_contract(field, value):
    import yaml
    from artifacts.vfs_contract import validate_vfs_scenario

    document = yaml.safe_load((ROOT / "config/scenario-2027.yaml").read_text())
    document["payload_geometry"][field] = value
    with pytest.raises(ValueError):
        validate_vfs_scenario(document)


def test_vfs_rejects_source_payload_inside_home():
    import yaml
    from artifacts.vfs_contract import (
        load_vfs_scenario, validate_vfs_course, validate_vfs_configuration,
    )

    document = yaml.safe_load((ROOT / "config/course-2027.yaml").read_text())
    document["waypoints"]["P2"].update(x=0, y=0)
    course = validate_vfs_course(document)
    scenario = load_vfs_scenario(ROOT / "config/scenario-2027.yaml")
    with pytest.raises(ValueError, match="outside H"):
        validate_vfs_configuration(course, scenario)
