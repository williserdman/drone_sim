from pathlib import Path
import pytest

from artifacts.calibration import airframe_fingerprint

ROOT = Path(__file__).parents[2]


def test_stock_variants_have_identical_physics():
    fingerprints = {airframe_fingerprint((ROOT / f"gazebo/resources/models/{name}/model.sdf").read_bytes())
        for name in ("iris_flight", "iris_moving_pad", "iris_competition")}
    assert len(fingerprints) == 1


@pytest.mark.parametrize("old,new", [
    (b"<mass>1.5</mass>", b"<mass>1.6</mass>"),
    (b"<update_rate>1000.0</update_rate>", b"<update_rate>999.0</update_rate>"),
    (b"<ixx>0.008</ixx>", b"<ixx>0.009</ixx>"),
    (b"<mass>0.05</mass>", b"<mass>0.06</mass>"),
])
def test_changed_consumer_physics_is_incompatible(old, new):
    original = (ROOT / "gazebo/resources/models/iris_flight/model.sdf").read_bytes()
    assert old in original
    assert airframe_fingerprint(original) != airframe_fingerprint(original.replace(old, new))


def test_unknown_plugin_remains_bound():
    original = (ROOT / "gazebo/resources/models/iris_flight/model.sdf").read_bytes()
    changed = original.replace(b"</model>", b'<plugin name="unknown" filename="unknown.so"/></model>')
    assert airframe_fingerprint(original) != airframe_fingerprint(changed)


def test_competition_rc_input_does_not_change_physical_fingerprint():
    original = (ROOT / "gazebo/resources/models/iris_flight/model.sdf").read_bytes()
    configured = original.replace(
        b"<have_32_channels>0</have_32_channels>",
        b"<have_32_channels>0</have_32_channels>"
        b"<rc_input_pwm>1500 1500 1000 1500 1500 1500 1500</rc_input_pwm>",
    )

    assert airframe_fingerprint(original) == airframe_fingerprint(configured)
