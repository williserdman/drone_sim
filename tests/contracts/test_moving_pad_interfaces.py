from pathlib import Path


ROOT = Path(__file__).parents[2]
INTERFACES = ROOT / "ros_ws/src/simulation_interfaces"


def test_landing_pad_state_wire_contract_is_exact():
    declarations = [
        line.strip()
        for line in (INTERFACES / "msg/LandingPadState.msg").read_text().splitlines()
        if line.strip()
    ]

    assert declarations == [
        "string run_id",
        "builtin_interfaces/Time sim_timestamp",
        "uint32 marker_id",
        "geometry_msgs/Pose pose",
        "geometry_msgs/Twist twist",
        "bool vehicle_in_contact",
    ]


def test_interface_build_registers_landing_pad_state():
    cmake = (INTERFACES / "CMakeLists.txt").read_text()

    assert '"msg/LandingPadState.msg"' in cmake
