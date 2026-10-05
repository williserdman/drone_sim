from pathlib import Path


ROOT = Path(__file__).parents[2]


def test_calibration_rules_are_packaged_with_scorekeeper():
    assert (ROOT / "scorekeeper/rules/calibration_v1.json").is_file()
    assert "COPY scorekeeper/rules /opt/drone_sim/scorekeeper/rules" in (
        ROOT / "scorekeeper/Dockerfile"
    ).read_text()


def test_calibration_uses_existing_ground_truth_wire_contract():
    declarations = [line.strip() for line in (
        ROOT / "ros_ws/src/simulation_interfaces/msg/GroundTruth.msg"
    ).read_text().splitlines() if line.strip()]
    assert "geometry_msgs/Pose pose" in declarations
    assert "geometry_msgs/Twist twist" in declarations
    assert "bool in_contact" in declarations


def test_roll_gain_allowlist_agrees_between_producer_and_inspector():
    from drone_sim_companion.autotune import ROLL_GAIN_PARAMETERS as producer
    from artifacts.diagnostic_acceptance import ROLL_GAIN_PARAMETERS as inspector
    assert producer == inspector
