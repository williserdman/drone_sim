from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'comp2026/src'))


def test_runner_calls_real_upstream_functions_without_synthetic_fm3_payload(monkeypatch):
    from drone_sim_companion.comp2026_runner import run_auto_attempt
    from drone.common_types import GPSCoord
    from drone import mock_mission
    from drone.control.mission_info import MissonTracker

    calls = []
    class Controller:
        vehicle = SimpleNamespace(_master=SimpleNamespace(mav=SimpleNamespace(statustext_send=lambda *a: None)), armed=True)
        force_arm_takeoff = lambda self, altitude: calls.append(('takeoff', altitude))
        goto_waypoint = lambda self, point, **kw: calls.append(('goto', point)) or 0
        simple_land = lambda self: calls.append(('land',))
        disarm = lambda self: calls.append(('disarm',)) or 0
        set_guided_mode = lambda self: calls.append(('guided',))
        climb = lambda self, height: calls.append(('climb', height))
        hold_waypoint_until_stable = lambda self, point: True
        is_landed = lambda self: False
    controller = Controller()
    # Exercise the real FM3 body through its normal unsuccessful-pickup path.
    monkeypatch.setattr(mock_mission, 'pickup_sequence', lambda *args: False)
    payload = SimpleNamespace(drop=lambda: calls.append(('drop',)))
    points = {key: GPSCoord(37.4, -122.08, 10) for key in ('H', 'L', 'F2', 'WA', 'WM')}
    events = []
    run_auto_attempt(tracker=MissonTracker(600), controller=controller,
                     camera=SimpleNamespace(save_frame_buffer_async=lambda: None),
                     lidar=SimpleNamespace(get_distance=lambda: 10), payloads={2: payload},
                     waypoints=points, emit=lambda *event: events.append(event),
                     check_active=lambda: None, confirm_landed=lambda: True)
    assert calls.count(('drop',)) == 1
    assert calls.count(('land',)) == 3  # Actual upstream FM1, FM2, and return-home calls.
    assert calls.count(('takeoff', 10)) == 3
    assert events[-2:] == [('HOME', 'DISARMED'), ('HOME', 'COMPLETE')]


def test_runner_cannot_report_home_complete_from_none_returns(monkeypatch):
    from drone_sim_companion.comp2026_runner import run_auto_attempt
    from drone.common_types import GPSCoord
    from drone import mock_mission
    from drone.control.mission_info import MissonTracker
    from importlib import import_module
    fm1 = import_module("drone.missions.fm1")
    fm2 = import_module("drone.missions.fm2")

    monkeypatch.setattr(fm1, 'fm1', lambda *args: None)
    monkeypatch.setattr(fm2, 'fm2', lambda *args: None)
    monkeypatch.setattr(mock_mission, 'fm3', lambda *args: None)
    controller = SimpleNamespace(force_arm_takeoff=lambda *args: None,
                                 goto_waypoint=lambda *args: None,
                                 simple_land=lambda: None, disarm=lambda: 0)
    events = []
    points = {key: GPSCoord(37.4, -122.08, 10) for key in ('H', 'L', 'F2', 'WA', 'WM')}
    with pytest.raises(RuntimeError, match='landed and disarmed'):
        run_auto_attempt(tracker=MissonTracker(), controller=controller, camera=None,
                         lidar=None, payloads={2: None}, waypoints=points,
                         emit=lambda *event: events.append(event), check_active=lambda: None,
                         confirm_landed=lambda: False)
    assert ('HOME', 'COMPLETE') not in events
