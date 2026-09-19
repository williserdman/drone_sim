from types import SimpleNamespace

import pytest

from drone_sim_companion.mission import Telemetry


class Vehicle:
    def __init__(self):
        self.commands = []

    def send(self, *args):
        self.commands.append(args)

    def send_waypoint(self, *args):
        self.commands.append(('waypoint', *args))


class Payloads:
    attached_id = 2
    done = False

    def __init__(self):
        self.calls = []

    def start(self, *args):
        self.calls.append(args)

    def tick(self, stamp):
        return self.done

    def cancel(self):
        pass


class IO:
    ready = True
    range_m = 10.2

    def __init__(self):
        self.payloads = Payloads()
        self.events = []

    def clearance(self, state, stamp):
        return self.range_m, stamp

    def publish_event(self, phase, state, stamp):
        self.events.append((phase, state, stamp))


class CameraFailureIO(IO):
    def marker(self, aruco_id):
        try:
            raise ValueError('image data length does not match its geometry')
        except ValueError as error:
            raise RuntimeError('Camera acquisition worker failed') from error


def observe(ops, stamp, *, speed=0.0, armed=True, landed=False):
    ops.observe(Telemetry(stamp, heartbeat=True, mode='GUIDED', armed=armed,
                          landed=landed, latitude_deg=37.4, longitude_deg=-122.08,
                          relative_altitude_m=10.0, horizontal_speed_m_s=speed,
                          vertical_speed_m_s=0.0, roll_rad=0.0, pitch_rad=0.0, yaw_rad=0.0))


def test_release_waits_for_continuous_stability_and_confirmed_detachment():
    from drone_sim_companion.configured_competition import CompetitionOperations
    io = IO()
    ops = CompetitionOperations(Vehicle(), io)
    observe(ops, 0, speed=.5)
    identifier = ops.start('release_payload', {'aruco_id': 2})
    assert io.payloads.calls == []
    for stamp in range(100_000_000, 2_100_000_000, 100_000_000):
        observe(ops, stamp)
    assert io.payloads.calls == []
    observe(ops, 2_100_000_000)
    assert len(io.payloads.calls) == 1
    assert ops.operation_status(identifier).state == 'running'
    io.payloads.done = True
    observe(ops, 2_200_000_000)
    assert ops.operation_status(identifier).state == 'succeeded'


def test_attachment_requires_landed_disarmed_state():
    from drone_sim_companion.configured_competition import CompetitionOperations
    io = IO()
    ops = CompetitionOperations(Vehicle(), io)
    observe(ops, 0)
    identifier = ops.start('attach_payload', {'aruco_id': 3})
    assert ops.operation_status(identifier).state == 'failed'
    assert io.payloads.calls == []


def test_precision_land_preserves_camera_failure_cause_in_terminal_error():
    from drone_sim_companion.configured_competition import CompetitionOperations

    io = CameraFailureIO()
    emitted = []
    ops = CompetitionOperations(Vehicle(), io, emit=lambda *event: emitted.append(event))
    observe(ops, 0)

    identifier = ops.start('precision_land', {'aruco_id': 3})

    status = ops.operation_status(identifier)
    assert status.state == 'failed'
    assert status.error == (
        'Camera acquisition worker failed; caused by ValueError: '
        'image data length does not match its geometry'
    )
    finished = [event for event in emitted if event[0] == 'operation_finished']
    assert len(finished) == 1
    assert finished[0][2]['state'] == 'failed'
    assert finished[0][2]['error'] == status.error


def test_phase_events_wait_for_distinct_simulation_timestamps():
    from drone_sim_companion.configured_competition import CompetitionOperations
    io = IO()
    ops = CompetitionOperations(Vehicle(), io)
    observe(ops, 0, armed=False, landed=True)
    first = ops.start('mission_event', {'phase': 'FM1', 'state': 'STARTED'})
    assert ops.operation_status(first).state == 'succeeded'
    second = ops.start('mission_event', {'phase': 'FM1', 'state': 'COMPLETE'})
    assert ops.operation_status(second).state == 'running'
    observe(ops, 50_000_000, armed=False, landed=True)
    assert io.events == [('FM1', 'STARTED', 0), ('FM1', 'COMPLETE', 50_000_000)]


@pytest.mark.parametrize('armed,landed,expected', [(True, False, 'failed'), (False, True, 'succeeded')])
def test_search_completion_requires_observed_landing_and_disarm(armed, landed, expected):
    from drone_sim_companion.configured_competition import CompetitionOperations
    io = IO()
    ops = CompetitionOperations(Vehicle(), io)
    observe(ops, 0, armed=armed, landed=landed)
    identifier = ops.start('mission_event', {'phase': 'SEARCH', 'state': 'COMPLETE'})
    assert ops.operation_status(identifier).state == expected
    assert len(io.events) == (1 if expected == 'succeeded' else 0)
