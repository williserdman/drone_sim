from types import SimpleNamespace
import math

import pytest
from pymavlink import mavutil

from drone_sim_companion.mavlink_adapter import MavlinkAdapter
from drone_sim_companion.mission_plan import parse_mission_plan


@pytest.mark.parametrize('tool,args', [
    ('precision_land', {'aruco_id': 7}),
    ('attach_payload', {'aruco_id': 3}),
    ('release_payload', {'aruco_id': 2}),
    ('mission_event', {'phase': 'HOME', 'state': 'DISARMED'}),
])
def test_competition_tools_use_the_same_validated_plan(tool, args):
    plan = parse_mission_plan({'schema_version': 1, 'steps': [{'tool': tool, 'args': args}]})
    assert plan.steps[0].tool == tool
    assert dict(plan.steps[0].args) == args


def test_attitude_and_horizontal_speed_are_observed_from_mavlink():
    messages = [
        mavutil.mavlink.MAVLink_attitude_message(1, .1, -.2, .3, 0, 0, 0),
        mavutil.mavlink.MAVLink_global_position_int_message(1, 374003371, -1220800351,
                                                         10000, 10000, 30, 40, -10, 0),
    ]
    adapter = MavlinkAdapter(SimpleNamespace(recv_match=lambda **kwargs: messages.pop(0)), mavutil)
    attitude = adapter.poll(100)
    position = adapter.poll(100)
    assert (attitude.roll_rad, attitude.pitch_rad, attitude.yaw_rad) == pytest.approx((.1, -.2, .3))
    assert position.horizontal_speed_m_s == pytest.approx(.5)


def test_landing_target_uses_camera_angles_for_body_frd_vector():
    calls = []
    connection = SimpleNamespace(mav=SimpleNamespace(landing_target_send=lambda *args: calls.append(args)))
    adapter = MavlinkAdapter(connection, mavutil)
    adapter.send_landing_target(1.0, 2.0, 3.0)
    assert calls == [(0, 0, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                      math.atan2(2.0, 3.0), math.atan2(-1.0, 3.0), math.sqrt(14), 0.0, 0.0)]
