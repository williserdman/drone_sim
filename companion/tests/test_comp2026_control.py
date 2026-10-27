"""The actual upstream controller sends through the testbench writer boundary."""
from pathlib import Path
import sys
from types import SimpleNamespace
import threading

import pytest
from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).parents[1] / 'comp2026/src'))


class Wire:
    def __init__(self): self.packets = []
    def write(self, packet): self.packets.append(packet)


class Vehicle:
    def __init__(self):
        self.wire = Wire()
        self._master = SimpleNamespace(mav=mavutil.mavlink.MAVLink(self.wire, srcSystem=1, srcComponent=191))
        self.listeners = {}
        self.early_observer = None
        self.parameters = {}
        self.last_heartbeat = 0.0
        self.is_armable = True
        self._mode = SimpleNamespace(name='STABILIZE')
        self._armed = False
        self.location = SimpleNamespace(global_relative_frame=SimpleNamespace(lat=37.4003371, lon=-122.0800351, alt=0))
    def install_fc_message_observer(self, callback): self.early_observer = callback
    def remove_fc_message_observer(self, callback): self.early_observer = None
    def add_message_listener(self, name, callback): self.listeners.setdefault(name, []).append(callback)
    def remove_message_listener(self, name, callback): self.listeners[name].remove(callback)
    def on_message(self, names):
        def install(callback):
            for name in names if isinstance(names, list) else [names]: self.add_message_listener(name, callback)
            return callback
        return install
    def emit(self, name, *, source=(1, 1), **fields):
        message = SimpleNamespace(get_srcSystem=lambda: source[0], get_srcComponent=lambda: source[1], **fields)
        if self.early_observer is not None: self.early_observer(self, name, message)
        for callback in self.listeners.get(name, ()): callback(self, name, message)
    @property
    def mode(self): return self._mode
    @mode.setter
    def mode(self, value):
        number = {'STABILIZE': 0, 'GUIDED': 4, 'LOITER': 5, 'RTL': 6, 'LAND': 9}[value.name]
        self._master.mav.set_mode_send(1, 1, number)
        self._mode = value
    @property
    def armed(self): return self._armed
    @armed.setter
    def armed(self, value):
        self._master.mav.command_long_send(1, 1, 400, 0, int(value), 0, 0, 0, 0, 0, 0)
        self._armed = value
    def close(self): pass


class Clock:
    timestamp_ns = 50_000_000
    def now(self): return self.timestamp_ns / 1e9
    def sleep(self, seconds): self.timestamp_ns += int(seconds * 1e9)
    def run_at_current_timestamp(self, action):
        action(self.timestamp_ns)
        return True


def make_control(tmp_path):
    from drone_sim_companion.comp2026_control import SimulationCompetitionControl
    vehicle = Vehicle()
    clock = Clock()
    control = SimulationCompetitionControl(
        policy=SimpleNamespace(flight_sw_version=0x040603FF, flight_custom_version=b'92b0cd78',
                               telemetry_intervals_us=((0, 100_000),), mission_home_check=lambda home: None),
        run_id='test', run_directory=tmp_path, endpoint='tcp:sitl:5760', heartbeat_timeout=60,
        clock=clock, guided_output_delivery_callback=lambda: None,
        connection_factory=lambda *args, **kwargs: vehicle,
    )
    return control, vehicle, clock


def baseline(vehicle):
    vehicle.emit('HEARTBEAT', base_mode=0, custom_mode=0, type=2, autopilot=3, system_status=3)
    vehicle.emit('SYS_STATUS', onboard_control_sensors_enabled=268435456, onboard_control_sensors_health=268435456)
    vehicle.emit('EXTENDED_SYS_STATE', landed_state=1)
    vehicle.emit('AUTOPILOT_VERSION', flight_sw_version=0x040603FF, flight_custom_version=b'92b0cd78')
    for ms in (100, 200):
        vehicle.emit('GLOBAL_POSITION_INT', time_boot_ms=ms, lat=374003371, lon=-1220800351, alt=195, relative_alt=0)


def test_actual_upstream_commands_are_closed_before_admission_and_after_abort(tmp_path):
    control, vehicle, clock = make_control(tmp_path)
    baseline(vehicle)
    before = len(vehicle.wire.packets)
    with pytest.raises(RuntimeError): control.controller.set_land_mode()
    assert len(vehicle.wire.packets) == before
    control.admit("a" * 64)
    control.begin_attempt()
    control.deliver_initial_guided()
    control.controller.set_land_mode()
    assert len(vehicle.wire.packets) == before + 2
    control.abort('QGC abort')
    with pytest.raises(RuntimeError): control.controller.rtl()
    assert len(vehicle.wire.packets) == before + 2


def test_initial_mode_is_enqueued_even_if_cached_mode_is_guided(tmp_path):
    control, vehicle, clock = make_control(tmp_path)
    baseline(vehicle)
    vehicle._mode = SimpleNamespace(name='GUIDED')
    control.admit("a" * 64)
    control.begin_attempt()
    before = len(vehicle.wire.packets)
    control.deliver_initial_guided()
    assert len(vehicle.wire.packets) == before + 1


def test_late_initial_command_and_manual_takeover_never_emit(tmp_path):
    control, vehicle, clock = make_control(tmp_path)
    baseline(vehicle)
    control.admit("a" * 64)
    control.begin_attempt()
    clock.timestamp_ns = 50_000_001
    with pytest.raises(RuntimeError, match='50 ms'): control.deliver_initial_guided()
    assert not vehicle.wire.packets
    clock.timestamp_ns = 50_000_000
    control.deliver_initial_guided()
    vehicle.emit('HEARTBEAT', base_mode=0, custom_mode=5, type=2, autopilot=3, system_status=3)
    before = len(vehicle.wire.packets)
    with pytest.raises(RuntimeError): control.controller.rtl()
    control.abort('QGC abort after takeover')
    assert control.recover() == 'UNCONFIRMED'
    assert len(vehicle.wire.packets) == before


def test_readiness_requires_native_time_and_exact_firmware_and_ground(tmp_path):
    control, vehicle, _ = make_control(tmp_path)
    assert not control.ready_for_initial_command()
    baseline(vehicle)
    assert control.ready_for_initial_command()
    vehicle.emit('AUTOPILOT_VERSION', flight_sw_version=0x040507FF, flight_custom_version=b'badbuild')
    assert not control.ready_for_initial_command()
    assert 'firmware' in ' '.join(control.ground_telemetry_pending_reasons)


def test_foreign_telemetry_cannot_admit_or_revoke(tmp_path):
    control, vehicle, _ = make_control(tmp_path)
    vehicle.emit('HEARTBEAT', source=(255,190), base_mode=0, custom_mode=0)
    assert not control.ready_for_initial_command()
    baseline(vehicle)
    control.admit("a" * 64)
    control.begin_attempt()
    control.deliver_initial_guided()
    vehicle.emit('HEARTBEAT', source=(255,190), base_mode=0, custom_mode=5)
    control.controller.set_land_mode()


def test_abort_owner_may_land_but_worker_and_finalized_owner_cannot_write(tmp_path):
    control, vehicle, _ = make_control(tmp_path)
    baseline(vehicle)
    control.admit("a" * 64)
    control.begin_attempt()
    control.deliver_initial_guided()
    vehicle.armed = True
    vehicle.emit('HEARTBEAT', base_mode=128, custom_mode=4, type=2, autopilot=3, system_status=4)
    vehicle.emit('EXTENDED_SYS_STATE', landed_state=2)
    control.abort('operator request')
    # A separate owner is the only recovery producer. The simulated FC confirms LAND.
    def landed(_vehicle, _name, _message): pass
    original_sleep = control.clock.sleep
    def sleep(seconds):
        vehicle.emit('HEARTBEAT', base_mode=0, custom_mode=9, type=2, autopilot=3, system_status=3)
        vehicle.emit('EXTENDED_SYS_STATE', landed_state=1)
        original_sleep(seconds)
    control.clock.sleep = sleep
    results = []
    thread = threading.Thread(target=lambda: results.append(control.recover()))
    thread.start(); thread.join(1)
    assert results == ['LOCAL_LANDED']
    control.begin_finalizing()
    before = len(vehicle.wire.packets)
    with pytest.raises(RuntimeError): control.controller.set_land_mode()
    assert len(vehicle.wire.packets) == before


def test_ground_baseline_cannot_confirm_landing_after_flight_output(tmp_path):
    control, vehicle, _ = make_control(tmp_path)
    baseline(vehicle)
    control.admit('a' * 64)
    control.begin_attempt()
    control.deliver_initial_guided()
    control.controller.set_land_mode()
    assert not control.confirm_landed()
    vehicle.emit('HEARTBEAT', base_mode=0, custom_mode=9, type=2, autopilot=3, system_status=3)
    assert not control.confirm_landed()
    vehicle.emit('EXTENDED_SYS_STATE', landed_state=1)
    assert control.confirm_landed()


def test_upstream_rtl_owns_its_mode_and_auto_disarm_transition(tmp_path):
    control, vehicle, _ = make_control(tmp_path)
    baseline(vehicle)
    control.admit('a' * 64)
    control.begin_attempt()
    control.deliver_initial_guided()
    vehicle.armed = True
    vehicle.emit('HEARTBEAT', base_mode=128, custom_mode=4, type=2, autopilot=3, system_status=4)
    control.controller.rtl()
    vehicle.emit('HEARTBEAT', base_mode=128, custom_mode=6, type=2, autopilot=3, system_status=4)
    vehicle.emit('HEARTBEAT', base_mode=0, custom_mode=6, type=2, autopilot=3, system_status=3)
    assert not control.authority.revoked


def test_simulation_finalization_blocks_actual_upstream_parameter_and_mode_writes(tmp_path):
    from drone_sim_companion.runtime_node import _Comp2026ShutdownAdmission, _comp2026_shutdown_dependency
    control, vehicle, _ = make_control(tmp_path)
    baseline(vehicle)
    control.admit('a' * 64)
    control.begin_attempt()
    control.deliver_initial_guided()
    admission = _Comp2026ShutdownAdmission()
    control.install_output_guard(_comp2026_shutdown_dependency(admission, lambda action: action()))
    admission.request_stop_from_signal()
    before = len(vehicle.wire.packets)
    with pytest.raises(RuntimeError): control.controller.rtl()
    with pytest.raises(RuntimeError): vehicle._master.mav.param_set_send(1, 1, b'PLND_ENABLED', 0, 9)
    assert len(vehicle.wire.packets) == before


def test_attempt_cannot_acquire_its_own_admission(tmp_path):
    control, vehicle, _ = make_control(tmp_path)
    baseline(vehicle)
    with pytest.raises(RuntimeError, match='admission'): control.begin_attempt()
    assert not vehicle.wire.packets


def test_regressing_native_clock_cannot_be_readiness_evidence(tmp_path):
    control, vehicle, _ = make_control(tmp_path)
    baseline(vehicle)
    vehicle.emit('GLOBAL_POSITION_INT', time_boot_ms=150, lat=374003371, lon=-1220800351, alt=195, relative_alt=0)
    assert not control.ready_for_initial_command()
    assert 'native' in ' '.join(control.ground_telemetry_pending_reasons)


def test_initial_delivery_status_finishes_before_finalization_latches(tmp_path):
    from drone_sim_companion.runtime_node import _Comp2026ShutdownAdmission, _comp2026_shutdown_dependency
    control, vehicle, _ = make_control(tmp_path)
    baseline(vehicle)
    control.admit('a' * 64)
    admission = _Comp2026ShutdownAdmission()
    control.install_output_guard(_comp2026_shutdown_dependency(admission, lambda action: action()))
    delivery_entered = threading.Event()
    release = threading.Event()
    finalized = threading.Event()
    control._delivery = lambda: (delivery_entered.set(), release.wait(1))
    def worker():
        control.begin_attempt()
        control.deliver_initial_guided()
    thread = threading.Thread(target=worker)
    thread.start()
    assert delivery_entered.wait(1)
    finalizer = threading.Thread(target=lambda: (admission.begin_finalizing(), finalized.set()))
    finalizer.start()
    try:
        assert not finalized.wait(0.02)
    finally:
        release.set()
        thread.join(1)
        finalizer.join(1)
    assert finalized.is_set()


def test_observed_guided_gate_prevents_upstream_stabilize_startup_race(tmp_path):
    from dronekit import VehicleMode
    control, vehicle, clock = make_control(tmp_path)
    baseline(vehicle)
    control.admit('a' * 64)
    control.begin_attempt()
    control.deliver_initial_guided()
    original_sleep = clock.sleep
    def acknowledge_guided(seconds):
        original_sleep(seconds)
        vehicle.emit('HEARTBEAT', base_mode=0, custom_mode=4, type=2, autopilot=3, system_status=3)
    clock.sleep = acknowledge_guided
    control.wait_for_guided()
    vehicle.mode = VehicleMode('STABILIZE')
    vehicle.emit('HEARTBEAT', base_mode=0, custom_mode=0, type=2, autopilot=3, system_status=3)
    assert not control.authority.revoked


@pytest.mark.parametrize('identity', [(6, 3), (2, 8), (6, 8)])
def test_wrong_heartbeat_identity_cannot_replace_arducopter_readiness(tmp_path, identity):
    control, vehicle, _ = make_control(tmp_path)
    baseline(vehicle)
    vehicle.emit('HEARTBEAT', base_mode=0, custom_mode=0, type=identity[0], autopilot=identity[1], system_status=3)
    assert not control.ready_for_initial_command()
    assert 'identity' in ' '.join(control.ground_telemetry_pending_reasons)


def test_stale_native_heartbeat_irreversibly_closes_actual_upstream_output(tmp_path):
    control, vehicle, _ = make_control(tmp_path)
    baseline(vehicle)
    control.admit('a' * 64)
    control.begin_attempt()
    control.deliver_initial_guided()
    receipt, heartbeat = control._facts['HEARTBEAT']
    control._facts['HEARTBEAT'] = (receipt - 3_000_000_000, heartbeat)
    before = len(vehicle.wire.packets)
    with pytest.raises(RuntimeError, match='heartbeat'):
        control.controller.set_land_mode()
    assert len(vehicle.wire.packets) == before
    assert control.authority.revoked
    vehicle.emit('HEARTBEAT', base_mode=0, custom_mode=4, type=2, autopilot=3, system_status=3)
    with pytest.raises(RuntimeError): control.controller.set_land_mode()
    assert len(vehicle.wire.packets) == before
