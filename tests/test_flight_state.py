import math
import threading
from types import SimpleNamespace

import pytest

from drone.control.flight_state import (
    Authority,
    FlightState,
    Observation,
    RCModeBand,
    RCInput,
    is_fresh,
)
from drone.control.mission_supervisor import (
    AuthorityLost,
    FlightOperationError,
    MissionAbort,
)


SOURCE = (1, 1)
BOUNDS = {
    "heartbeat": 1.5,
    "mode": 1.5,
    "location": 0.5,
    "velocity": 0.5,
    "attitude": 0.5,
    "landed_state": 1.5,
    "armed": 1.5,
    "home": 1.5,
    "rc_input": 0.5,
    "range": 0.5,
    "failsafe": 1.5,
}
RC_MAPPING = (
    RCModeBand(slot="manual-low", minimum_pwm=900, maximum_pwm=1200, mode="STABILIZE"),
    RCModeBand(slot="companion", minimum_pwm=1400, maximum_pwm=1600, mode="GUIDED"),
    RCModeBand(slot="manual-high", minimum_pwm=1800, maximum_pwm=2100, mode="LOITER"),
)


def make_state(now=10.0):
    return FlightState(
        source_system=SOURCE[0],
        source_component=SOURCE[1],
        freshness_bounds=BOUNDS,
        rc_channel=7,
        rc_mode_mapping=RC_MAPPING,
        clock=lambda: now,
    )


def update(state, field, value, *, at=10.0, sequence=1, source=SOURCE):
    return state.update(
        field,
        value,
        received_at=at,
        sequence=sequence,
        source_system=source[0],
        source_component=source[1],
    )


class CoordinatedRLock:
    def __init__(self, reader_name):
        self._lock = threading.RLock()
        self._reader_name = reader_name
        self.reader_waiting = threading.Event()

    def __enter__(self):
        if threading.current_thread().name == self._reader_name:
            self.reader_waiting.set()
        self._lock.acquire()
        return self

    def __exit__(self, _type, _value, _traceback):
        self._lock.release()


def test_implicit_snapshot_time_is_captured_with_the_telemetry_state():
    class Clock:
        now = 10.0

        def __call__(self):
            return self.now

    clock = Clock()
    state = FlightState(
        source_system=SOURCE[0],
        source_component=SOURCE[1],
        freshness_bounds=BOUNDS,
        rc_channel=7,
        rc_mode_mapping=RC_MAPPING,
        clock=clock,
    )
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)
    coordinated_lock = CoordinatedRLock("snapshot-reader")
    state._lock = coordinated_lock
    captured = []

    def read_snapshot():
        captured.append(state.snapshot())

    with coordinated_lock:
        reader = threading.Thread(target=read_snapshot, name="snapshot-reader")
        reader.start()
        assert coordinated_lock.reader_waiting.wait(timeout=1.0)
        clock.now = 10.1
        assert update(state, "heartbeat", "new heartbeat", at=10.1, sequence=2)
    reader.join(timeout=1.0)

    assert not reader.is_alive()
    snapshot = captured[0]
    assert snapshot.heartbeat is not None
    assert snapshot.heartbeat.age == pytest.approx(0.0)
    assert snapshot.heartbeat.fresh
    assert snapshot.authority is Authority.COMPANION


def test_future_and_stale_observations_are_not_fresh():
    sample = Observation(value=10.0, received_at=5.0, sequence=1)

    assert is_fresh(sample, now=5.2, max_age=0.5)
    assert not is_fresh(sample, now=6.0, max_age=0.5)
    assert not is_fresh(sample, now=4.9, max_age=0.5)


@pytest.mark.parametrize("value", [None, math.nan, (1.0, math.nan)])
def test_missing_or_nan_measurements_are_rejected(value):
    state = make_state()

    assert not update(state, "location", value)
    assert state.snapshot(now=10.0).location is None


def test_wrong_source_cannot_replace_accepted_telemetry():
    state = make_state()
    assert update(state, "location", (1.0, 2.0, 3.0))

    assert not update(
        state,
        "location",
        (9.0, 9.0, 9.0),
        at=10.1,
        sequence=2,
        source=(42, 1),
    )

    location = state.snapshot(now=10.2).location
    assert location is not None
    assert location.observation.value == (1.0, 2.0, 3.0)
    assert location.source.system_id == 1
    assert location.source.component_id == 1


def test_each_snapshot_field_has_its_own_age_and_freshness():
    state = make_state()
    assert update(state, "heartbeat", "GUIDED", at=10.0)
    assert update(state, "location", (1.0, 2.0, 3.0), at=9.0)

    snapshot = state.snapshot(now=10.2)

    assert snapshot.heartbeat is not None and snapshot.heartbeat.fresh
    assert snapshot.heartbeat.age == pytest.approx(0.2)
    assert snapshot.location is not None and not snapshot.location.fresh
    assert snapshot.location.age == pytest.approx(1.2)


def test_out_of_order_field_update_does_not_create_a_mixed_snapshot():
    state = make_state()
    assert update(state, "velocity", (1.0, 2.0, 3.0), sequence=4)

    assert not update(
        state, "velocity", (8.0, 8.0, 8.0), at=10.1, sequence=3
    )

    velocity = state.snapshot(now=10.2).velocity
    assert velocity is not None
    assert velocity.observation.sequence == 4
    assert velocity.observation.value == (1.0, 2.0, 3.0)


@pytest.mark.parametrize(
    "exception_type", [FlightOperationError, MissionAbort, AuthorityLost]
)
def test_supervisor_exceptions_retain_reason(exception_type):
    error = exception_type("mode became unknown")

    assert error.reason == "mode became unknown"
    assert str(error) == "mode became unknown"
    assert type(error).__bases__ == (RuntimeError,)


def establish_ground_baseline(state, *, pwm=1500):
    assert update(state, "heartbeat", "alive", sequence=1)
    assert update(state, "mode", "GUIDED", sequence=1)
    assert update(state, "landed_state", 1, sequence=1)
    assert update(state, "armed", False, sequence=1)
    assert state.observe_rc_input(
        channel=7,
        pwm=pwm,
        signal_healthy=True,
        received_at=10.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )


def observe_mode(state, mode, *, at=10.0, sequence=2):
    return state.observe_mode(
        mode,
        received_at=at,
        sequence=sequence,
        source_system=1,
        source_component=1,
    )


def observe_rc(state, pwm, *, healthy=True, at=10.0, sequence=2, channel=7):
    return state.observe_rc_input(
        channel=channel,
        pwm=pwm,
        signal_healthy=healthy,
        received_at=at,
        sequence=sequence,
        source_system=1,
        source_component=1,
    )


def test_initial_companion_authority_requires_fresh_ground_baseline():
    state = make_state()

    assert not state.acquire_initial_companion_authority(now=10.0)
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert state.authority is Authority.COMPANION
    assert state.ordinary_commands_permitted()


def test_initial_authority_rejects_guided_heartbeat_with_physical_loiter_slot():
    state = make_state()
    establish_ground_baseline(state, pwm=1900)

    assert not state.acquire_initial_companion_authority(now=10.0)
    assert state.authority is not Authority.COMPANION
    assert not state.ordinary_commands_permitted()


def test_implicit_authority_time_is_captured_with_the_ground_snapshot():
    class Clock:
        now = 10.0

        def __call__(self):
            return self.now

    clock = Clock()
    state = FlightState(
        source_system=SOURCE[0],
        source_component=SOURCE[1],
        freshness_bounds=BOUNDS,
        rc_channel=7,
        rc_mode_mapping=RC_MAPPING,
        clock=clock,
    )
    establish_ground_baseline(state)
    coordinated_lock = CoordinatedRLock("authority-reader")
    state._lock = coordinated_lock
    acquired = []

    def acquire_authority():
        acquired.append(state.acquire_initial_companion_authority())

    with coordinated_lock:
        reader = threading.Thread(target=acquire_authority, name="authority-reader")
        reader.start()
        assert coordinated_lock.reader_waiting.wait(timeout=1.0)
        clock.now = 10.1
        assert state.update_many(
            {
                "heartbeat": "new heartbeat",
                "mode": "GUIDED",
                "landed_state": 1,
                "armed": False,
            },
            received_at=10.1,
            sequence=2,
            source_system=1,
            source_component=1,
        )
        assert observe_rc(state, 1500, at=10.1, sequence=2)
    reader.join(timeout=1.0)

    assert not reader.is_alive()
    assert acquired == [True]
    assert state.authority is Authority.COMPANION
    assert state.ordinary_commands_permitted()


@pytest.mark.parametrize("landed_state", [0, 2, 3, 4, True, False, 1.0])
def test_initial_companion_authority_requires_exact_mav_on_ground_enum(landed_state):
    state = make_state()
    establish_ground_baseline(state)
    assert update(state, "landed_state", landed_state, sequence=2)

    assert not state.acquire_initial_companion_authority(now=10.0)
    assert state.authority is Authority.UNKNOWN


@pytest.mark.parametrize(
    ("field", "value"),
    [("landed_state", 2), ("armed", True)],
)
def test_initial_authority_rejects_airborne_or_armed_state(field, value):
    state = make_state()
    establish_ground_baseline(state)
    assert update(state, field, value, sequence=2)

    assert not state.acquire_initial_companion_authority(now=10.0)
    assert state.authority is Authority.UNKNOWN


def test_unhealthy_rc_input_does_not_establish_initial_baseline():
    state = make_state()
    assert update(state, "heartbeat", "alive")
    assert update(state, "landed_state", 1)
    assert update(state, "armed", False)
    assert observe_rc(state, 1500, healthy=False, sequence=1)

    assert not state.acquire_initial_companion_authority(now=10.0)


def test_expected_mode_registration_does_not_grant_authority():
    state = make_state()

    token = state.register_expected_mode("GUIDED")
    assert observe_mode(state, "GUIDED")

    assert token > 0
    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()


def test_expected_mode_is_consumed_without_suspending_companion_commands():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    state.register_expected_mode("GUIDED")
    assert observe_mode(state, "GUIDED")

    assert state.snapshot(now=10.0).commands_suspended is False
    assert state.snapshot(now=10.0).expected_mode is None
    assert state.ordinary_commands_permitted()


def test_repeated_unchanged_mode_does_not_suspend_companion_commands():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert observe_mode(state, "GUIDED")

    assert state.authority is Authority.COMPANION
    assert state.ordinary_commands_permitted()


def test_old_mode_during_pending_transition_does_not_consume_expectation():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)
    state.register_expected_mode("LOITER")

    assert observe_mode(state, "GUIDED")

    snapshot = state.snapshot(now=10.0)
    assert snapshot.expected_mode == "LOITER"
    assert snapshot.authority is Authority.COMPANION
    assert snapshot.commands_suspended is False


def test_unexpected_mode_suspends_commands_without_claiming_pilot_takeover():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert observe_mode(state, "LOITER")

    snapshot = state.snapshot(now=10.0)
    assert snapshot.authority is Authority.UNKNOWN
    assert snapshot.commands_suspended
    assert not state.ordinary_commands_permitted()


@pytest.mark.parametrize("manual_mode", ["LOITER", "STABILIZE"])
def test_fresh_healthy_rc_edge_then_matching_manual_mode_establishes_pilot(
    manual_mode,
):
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)
    permission = state.command_permission()
    assert permission is not None
    pwm = 2000 if manual_mode == "LOITER" else 1000

    assert observe_rc(state, pwm)
    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()
    assert not state.permission_is_current(permission)

    assert observe_mode(state, manual_mode)

    assert state.authority is Authority.PILOT
    assert not state.ordinary_commands_permitted()


@pytest.mark.parametrize("manual_mode", ["LOITER", "STABILIZE"])
def test_manual_mode_before_rc_edge_needs_a_new_post_edge_mode_observation(
    manual_mode,
):
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)
    permission = state.command_permission()
    assert permission is not None
    pwm = 2000 if manual_mode == "LOITER" else 1000

    assert observe_mode(state, manual_mode, at=10.0, sequence=2)
    assert state.authority is Authority.UNKNOWN
    assert not state.permission_is_current(permission)

    assert observe_rc(state, pwm, at=10.0, sequence=2)
    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()

    assert observe_mode(state, manual_mode, at=10.0, sequence=3)
    assert state.authority is Authority.PILOT
    assert not state.ordinary_commands_permitted()


def test_mode_callback_with_pre_edge_receipt_time_cannot_establish_pilot():
    state = make_state(now=10.2)
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert observe_rc(state, 2000, at=10.2, sequence=2)
    assert observe_mode(state, "LOITER", at=10.1, sequence=2)

    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()


def test_wrong_source_mode_after_rc_edge_cannot_establish_pilot():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert observe_rc(state, 2000, at=10.0, sequence=2)
    assert not state.observe_mode(
        "LOITER",
        received_at=10.0,
        sequence=2,
        source_system=42,
        source_component=1,
    )

    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()


def test_mode_that_does_not_match_rc_mapping_cannot_establish_pilot():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert observe_rc(state, 2000, at=10.0, sequence=2)
    assert observe_mode(state, "STABILIZE", at=10.0, sequence=2)

    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()


def test_mode_after_rc_edge_freshness_window_cannot_establish_pilot():
    now = [10.0]
    state = FlightState(
        source_system=SOURCE[0],
        source_component=SOURCE[1],
        freshness_bounds=BOUNDS,
        rc_channel=7,
        rc_mode_mapping=RC_MAPPING,
        clock=lambda: now[0],
    )
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert observe_rc(state, 2000, at=10.0, sequence=2)
    assert state.authority is Authority.UNKNOWN
    now[0] = 10.6
    assert observe_mode(state, "LOITER", at=10.6, sequence=2)

    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()


@pytest.mark.parametrize(
    "rc_change",
    [
        lambda state: observe_rc(state, 1500),
        lambda state: observe_rc(state, 2000, healthy=False),
        lambda state: observe_rc(state, 2000, at=8.0),
        lambda state: observe_rc(state, 2000, channel=8),
    ],
    ids=["same-slot", "unhealthy", "stale", "wrong-channel"],
)
def test_mode_without_a_fresh_healthy_rc_edge_stays_unknown(rc_change):
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert rc_change(state) is not False or state.authority is Authority.COMPANION
    assert observe_mode(state, "LOITER")

    assert state.authority is Authority.UNKNOWN


def test_explicit_fc_failsafe_has_precedence_over_pilot_evidence():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert state.observe_failsafe(
        "battery",
        active=True,
        received_at=10.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )
    assert observe_rc(state, 2000)
    assert observe_mode(state, "LOITER")

    assert state.authority is Authority.FC_FAILSAFE
    assert not state.ordinary_commands_permitted()


@pytest.mark.parametrize("lost_authority", [Authority.PILOT, Authority.FC_FAILSAFE])
def test_lost_attempt_authority_cannot_be_reacquired_on_ground(lost_authority):
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)
    if lost_authority is Authority.PILOT:
        assert observe_rc(state, 2000)
        assert observe_mode(state, "LOITER")
    else:
        assert state.observe_failsafe(
            "battery",
            active=True,
            received_at=10.0,
            sequence=1,
            source_system=1,
            source_component=1,
        )
    assert update(state, "landed_state", 1, sequence=2)
    assert update(state, "armed", False, sequence=2)

    assert not state.acquire_initial_companion_authority(now=10.0)
    assert state.authority is lost_authority


def test_cancelled_failed_mode_request_does_not_hide_later_unexpected_mode():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    token = state.register_expected_mode("GUIDED")
    assert state.cancel_expected_mode(token)
    assert observe_mode(state, "LOITER")

    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()


def test_mutable_measurement_is_frozen_at_ingestion():
    state = make_state()
    incoming = {"xyz": [1.0, 2.0, 3.0]}
    assert update(state, "velocity", incoming)

    incoming["xyz"][0] = 99.0

    observed = state.snapshot(now=10.0).velocity
    assert observed is not None
    assert observed.observation.value["xyz"] == (1.0, 2.0, 3.0)


def test_mutable_object_measurement_is_copied_at_ingestion():
    state = make_state()
    incoming = SimpleNamespace(pitch=0.2)
    assert update(state, "attitude", incoming)

    incoming.pitch = 9.0

    observed = state.snapshot(now=10.0).attitude
    assert observed is not None
    assert observed.observation.value.pitch == 0.2


def test_permission_token_is_invalid_after_takeover_between_check_and_send():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)
    permission = state.command_permission()
    assert permission is not None

    assert observe_rc(state, 2000)
    assert observe_mode(state, "LOITER")

    assert not state.permission_is_current(permission)


@pytest.mark.parametrize(
    "change",
    [
        {"source_system": 0},
        {"source_component": 256},
        {"rc_channel": 0},
        {"freshness_bounds": {**BOUNDS, "location": 0.0}},
        {"rc_mode_mapping": (RCModeBand("companion", 1400, 1600, "GUIDED"),)},
        {"rc_mode_mapping": (RCModeBand("pilot", 1800, 2100, "LOITER"),)},
        {
            "rc_mode_mapping": (
                RCModeBand("one", 900, 1500, "GUIDED"),
                RCModeBand("two", 1400, 1800, "LOITER"),
            )
        },
    ],
)
def test_invalid_observation_profiles_are_rejected(change):
    arguments = {
        "source_system": 1,
        "source_component": 1,
        "freshness_bounds": BOUNDS,
        "rc_channel": 7,
        "rc_mode_mapping": RC_MAPPING,
    }
    arguments.update(change)

    with pytest.raises(ValueError):
        FlightState(**arguments)


def test_rc_change_after_a_missing_fresh_edge_stays_unknown():
    now = [10.0]
    state = FlightState(
        source_system=1,
        source_component=1,
        freshness_bounds=BOUNDS,
        rc_channel=7,
        rc_mode_mapping=RC_MAPPING,
        clock=lambda: now[0],
    )
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    now[0] = 11.0
    assert observe_rc(state, 2000, at=11.0)
    assert observe_mode(state, "LOITER", at=11.0)

    assert state.authority is Authority.UNKNOWN


def test_already_held_manual_slot_cannot_open_initial_companion_authority():
    state = make_state()
    establish_ground_baseline(state, pwm=2000)
    assert not state.acquire_initial_companion_authority(now=10.0)
    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()


def test_multi_field_update_rejects_the_whole_packet_when_one_value_is_invalid():
    state = make_state()

    assert not state.update_many(
        {"location": (1.0, 2.0, 3.0), "velocity": (4.0, math.nan, 6.0)},
        received_at=10.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )

    snapshot = state.snapshot(now=10.0)
    assert snapshot.location is None
    assert snapshot.velocity is None


def test_expected_mode_registered_before_authority_cannot_mask_a_mode_change():
    state = make_state()
    state.register_expected_mode("GUIDED")
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert observe_mode(state, "LOITER")

    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()


def test_unknown_after_attempt_start_cannot_reacquire_on_ground():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)
    assert observe_mode(state, "LOITER")
    assert state.authority is Authority.UNKNOWN
    assert update(state, "landed_state", 1, sequence=2)
    assert update(state, "armed", False, sequence=2)

    assert not state.acquire_initial_companion_authority(now=10.0)
    assert state.authority is Authority.UNKNOWN


@pytest.mark.parametrize("manual_mode", ["LOITER", "STABILIZE"])
def test_expected_manual_mode_does_not_suppress_corroborated_takeover(
    manual_mode,
):
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)
    state.register_expected_mode(manual_mode)
    pwm = 2000 if manual_mode == "LOITER" else 1000

    assert observe_rc(state, pwm, sequence=2)
    assert state.authority is Authority.UNKNOWN
    assert observe_mode(state, manual_mode, sequence=2)

    assert state.authority is Authority.PILOT
    assert state.snapshot(now=10.0).expected_mode is None
    assert not state.ordinary_commands_permitted()


@pytest.mark.parametrize(
    ("pwm", "health"),
    [(2000, False), (2000, None), (1700, True)],
    ids=["unhealthy", "unknown-health", "unmapped"],
)
def test_later_invalid_rc_sample_cancels_pending_takeover_edge(pwm, health):
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)
    assert observe_rc(state, 2000, healthy=True, sequence=2)

    assert observe_rc(state, pwm, healthy=health, sequence=3)
    assert observe_mode(state, "LOITER", sequence=2)

    assert state.authority is Authority.UNKNOWN
    assert not state.ordinary_commands_permitted()


def test_mutating_a_snapshot_object_cannot_change_later_snapshots():
    state = make_state()
    assert update(state, "attitude", SimpleNamespace(pitch=0.2))
    first = state.snapshot(now=10.0)
    assert first.attitude is not None

    first.attitude.observation.value.pitch = math.nan

    later = state.snapshot(now=10.0)
    assert later.attitude is not None
    assert later.attitude.observation.value.pitch == 0.2


def test_stale_first_failsafe_observation_does_not_claim_fc_authority():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert state.observe_failsafe(
        "old battery event",
        active=True,
        received_at=8.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )

    assert state.authority is Authority.COMPANION
    assert state.ordinary_commands_permitted()


def _snapshot_while_observation_callback_is_paused(state, monkeypatch, callback):
    published = threading.Event()
    release = threading.Event()
    snapshot_finished = threading.Event()
    captured = []
    original_update = state.update

    def paused_update(field, value, **metadata):
        accepted = original_update(field, value, **metadata)
        published.set()
        assert release.wait(timeout=1.0)
        return accepted

    monkeypatch.setattr(state, "update", paused_update)
    producer = threading.Thread(target=callback)
    producer.start()
    assert published.wait(timeout=1.0)

    def read_snapshot():
        captured.append(state.snapshot(now=10.0))
        snapshot_finished.set()

    reader = threading.Thread(target=read_snapshot)
    reader.start()
    snapshot_finished.wait(timeout=0.05)
    release.set()
    producer.join(timeout=1.0)
    reader.join(timeout=1.0)
    assert not producer.is_alive()
    assert not reader.is_alive()
    return captured[0]


def test_failsafe_publication_and_authority_revocation_are_atomic(monkeypatch):
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    snapshot = _snapshot_while_observation_callback_is_paused(
        state,
        monkeypatch,
        lambda: state.observe_failsafe(
            "battery",
            active=True,
            received_at=10.0,
            sequence=1,
            source_system=1,
            source_component=1,
        ),
    )

    assert snapshot.failsafe is not None
    assert snapshot.authority is Authority.FC_FAILSAFE
    assert snapshot.commands_suspended


def test_unknown_rc_publication_and_authority_revocation_are_atomic(monkeypatch):
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    snapshot = _snapshot_while_observation_callback_is_paused(
        state,
        monkeypatch,
        lambda: observe_rc(state, 1500, healthy=None, sequence=2),
    )

    assert snapshot.rc_input is not None
    assert snapshot.rc_input.observation.value.healthy is False
    assert snapshot.authority is Authority.UNKNOWN
    assert snapshot.commands_suspended


def test_old_unusable_heartbeat_cannot_invalidate_newer_authorizing_evidence():
    state = make_state()
    assert state.update_many(
        {
            "heartbeat": "new heartbeat",
            "mode": "GUIDED",
            "landed_state": 1,
            "armed": False,
        },
        received_at=10.0,
        sequence=2,
        source_system=1,
        source_component=1,
    )
    assert state.observe_rc_input(
        channel=7,
        pwm=1500,
        signal_healthy=True,
        received_at=10.0,
        sequence=2,
        source_system=1,
        source_component=1,
    )
    assert state.acquire_initial_companion_authority(now=10.0)
    permission = state.command_permission()
    assert permission is not None

    assert not state.observe_heartbeat(
        ("old", None),
        armed=False,
        mode=None,
        received_at=9.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )

    snapshot = state.snapshot(now=10.0)
    assert snapshot.heartbeat is not None
    assert snapshot.heartbeat.observation.value == "new heartbeat"
    assert snapshot.mode is not None
    assert snapshot.authority is Authority.COMPANION
    assert state.permission_is_current(permission)


def test_older_valid_heartbeat_cannot_repopulate_after_current_invalidation():
    state = make_state()
    establish_ground_baseline(state)
    assert state.acquire_initial_companion_authority(now=10.0)

    assert state.observe_heartbeat(
        (2, 3, 0, 4, 3),
        armed=False,
        mode=None,
        received_at=10.0,
        sequence=3,
        source_system=1,
        source_component=1,
    )
    assert not state.observe_heartbeat(
        (2, 3, 0, 4, 3),
        armed=False,
        mode="GUIDED",
        received_at=9.0,
        sequence=2,
        source_system=1,
        source_component=1,
    )

    snapshot = state.snapshot(now=10.0)
    assert snapshot.heartbeat is None
    assert snapshot.armed is None
    assert snapshot.mode is None
    assert snapshot.authority is Authority.UNKNOWN
