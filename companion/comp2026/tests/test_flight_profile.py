import json
import threading
from types import SimpleNamespace

import pytest

from drone.control.flight_profile import FlightProfileError, load_flight_profile
from drone.control.flight_state import Authority, FlightState, FailsafeEvidence
from drone.control.mission_supervisor import AuthorityLost
from test_prepare_attempt import profile_data


class Clock:
    def __init__(self, now=10.0):
        self.now = now

    def __call__(self):
        return self.now


class PausedOlderClock:
    def __init__(self, paused_value=10.0):
        self.now = 10.1
        self.paused_value = paused_value
        self.older_sampled = threading.Event()
        self.release_older = threading.Event()

    def __call__(self):
        if threading.current_thread().name == "older-sys-status":
            self.older_sampled.set()
            if not self.release_older.wait(1):
                raise TimeoutError("test did not release older SYS_STATUS")
            return self.paused_value
        return self.now


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


class Message(SimpleNamespace):
    def get_srcSystem(self):
        return self.src_system

    def get_srcComponent(self):
        return self.src_component


def write_profile(tmp_path, mutate=None):
    data = profile_data()
    if mutate is not None:
        mutate(data)
    path = tmp_path / "profile.json"
    path.write_text(json.dumps(data))
    return path, data


def message(**values):
    defaults = {
        "src_system": 1,
        "src_component": 1,
        "type": 2,
        "autopilot": 3,
        "custom_mode": 4,
        "system_status": 4,
        "onboard_control_sensors_present": 65536,
        "onboard_control_sensors_enabled": 65536,
        "onboard_control_sensors_health": 65536,
        "chan7_raw": 1500,
    }
    defaults.update(values)
    return Message(**defaults)


def flight_state(profile, clock):
    return FlightState(
        source_system=profile.flight_controller.system_id,
        source_component=profile.flight_controller.component_id,
        freshness_bounds=profile.freshness_bounds,
        rc_channel=profile.rc_channel,
        rc_mode_mapping=profile.rc_mode_bands,
        clock=clock,
    )


def acquire_state(profile, clock):
    state = flight_state(profile, clock)
    source = {
        "source_system": profile.flight_controller.system_id,
        "source_component": profile.flight_controller.component_id,
    }
    assert state.observe_heartbeat(
        "heartbeat",
        armed=False,
        mode="GUIDED",
        received_at=clock.now,
        sequence=1,
        **source,
    )
    assert state.update(
        "landed_state", 1, received_at=clock.now, sequence=1, **source
    )
    assert state.observe_rc_input(
        channel=profile.rc_channel,
        pwm=1500,
        signal_healthy=True,
        received_at=clock.now,
        sequence=1,
        **source,
    )
    assert state.acquire_initial_companion_authority(now=clock.now)
    assert state.authority is Authority.COMPANION
    return state


def publish_ground_evidence_and_acquire(state, profile, *, healthy, now):
    source = {
        "source_system": profile.flight_controller.system_id,
        "source_component": profile.flight_controller.component_id,
    }
    assert state.observe_heartbeat(
        "heartbeat",
        armed=False,
        mode="GUIDED",
        received_at=now,
        sequence=1,
        **source,
    )
    assert state.update("landed_state", 1, received_at=now, sequence=1, **source)
    assert state.observe_rc_input(
        channel=profile.rc_channel,
        pwm=1500,
        signal_healthy=healthy,
        received_at=now,
        sequence=1,
        **source,
    )
    return state.acquire_initial_companion_authority(now=now)


def test_profile_keeps_distinct_identities_and_exact_hash(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)

    assert (profile.qgc_source.system_id, profile.qgc_source.component_id) == (200, 190)
    assert (profile.companion_target.system_id, profile.companion_target.component_id) == (1, 191)
    assert (profile.flight_controller.system_id, profile.flight_controller.component_id) == (1, 1)
    assert profile.raw_sha256 == __import__("hashlib").sha256(path.read_bytes()).hexdigest()
    assert profile.firmware == "ArduCopter 4.5.7"
    assert [(item.message_id, item.interval_us) for item in profile.telemetry_requests] == [
        (0, 500000),
        (1, 200000),
        (30, 100000),
        (33, 200000),
        (65, 200000),
        (132, 100000),
        (242, 500000),
        (245, 200000),
    ]


@pytest.mark.parametrize(
    "mutate, match",
    [
        (
            lambda data: data["identity"].pop("flight_controller_component"),
            "flight_controller_component",
        ),
        (lambda data: data["identity"].update(flight_controller_component=191), "distinct"),
        (lambda data: data["identity"].update(source_system=1, source_component=1), "conflicting"),
        (
            lambda data: data["identity"].update(
                source_system=1, source_component=191
            ),
            "conflicting",
        ),
        (
            lambda data: data["observation"].update(decoder_contract_version=True),
            "contract version",
        ),
        (
            lambda data: data["observation"].update(
                decoder_contract_evidence="UNVERIFIED"
            ),
            "evidence",
        ),
        (lambda data: data["observation"]["freshness_seconds"].pop("home"), "freshness"),
        (
            lambda data: data["observation"].update(
                sys_status_rc_health_freshness_seconds=0
            ),
            "SYS_STATUS",
        ),
        (
            lambda data: data["observation"]["telemetry_requests"].append(
                {"message_id": 1, "interval_us": 5}
            ),
            "duplicate",
        ),
        (
            lambda data: data["observation"]["telemetry_requests"][0].update(
                interval_us=0
            ),
            "interval",
        ),
        (lambda data: data["observation"]["telemetry_requests"].pop(3), "required telemetry"),
        (lambda data: data["rc"]["mode_mapping"].update(OTHER=[1100, 1500]), "overlap"),
    ],
)
def test_profile_rejects_unapproved_or_malformed_observation_contract(tmp_path, mutate, match):
    path, _ = write_profile(tmp_path, mutate)
    with pytest.raises(FlightProfileError, match=match):
        load_flight_profile(path)


def test_heartbeat_decoders_require_trusted_source_and_exact_integer_fields(tmp_path):
    path, _ = write_profile(tmp_path)
    decoders = load_flight_profile(path).decoders(Clock())

    assert decoders.heartbeat_mode_decoder(message()) == "GUIDED"
    assert decoders.heartbeat_mode_decoder(message(custom_mode=99)) is None
    assert decoders.heartbeat_mode_decoder(message(src_component=2)) is None
    assert decoders.heartbeat_mode_decoder(message(type=True)) is None
    assert decoders.heartbeat_mode_decoder(message(custom_mode=4.0)) is None
    assert decoders.heartbeat_failsafe_decoder(
        message(system_status=5)
    ) == FailsafeEvidence(True, "FC system status CRITICAL")
    assert decoders.heartbeat_failsafe_decoder(
        message(system_status=4)
    ) == FailsafeEvidence(False, "FC system status ACTIVE")
    assert decoders.heartbeat_failsafe_decoder(message(system_status=6)) is None
    assert decoders.heartbeat_failsafe_decoder(message(system_status=True)) is None


def test_sys_status_health_owns_timestamp_and_rc_traffic_cannot_extend_it(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    clock = Clock()
    decoders = profile.decoders(clock)

    decoders.observe_sys_status(message())
    clock.now = 10.3
    assert decoders.rc_health_decoder(message(), 7, 1500) is True
    clock.now = 10.5
    assert decoders.rc_health_decoder(message(), 7, 1500) is None
    clock.now = 10.2
    assert decoders.rc_health_decoder(message(), 7, 1500) is None


def test_foreign_old_and_malformed_sys_status_cannot_corrupt_current_health(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    clock = Clock()
    decoders = profile.decoders(clock)

    decoders.observe_sys_status(message())
    clock.now = 9.0
    decoders.observe_sys_status(message(onboard_control_sensors_health=0))
    clock.now = 10.1
    decoders.observe_sys_status(message(src_system=2, onboard_control_sensors_health=0))
    assert decoders.rc_health_decoder(message(), 7, 1500) is True

    clock.now = 10.2
    decoders.observe_sys_status(message(onboard_control_sensors_health=0))
    assert decoders.rc_health_decoder(message(), 7, 1500) is False
    clock.now = 10.3
    decoders.observe_sys_status(message(onboard_control_sensors_health="bad"))
    assert decoders.rc_health_decoder(message(), 7, 1500) is None


def test_rc_decoder_requires_exact_current_channel_pwm_and_all_health_bits(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    decoders = profile.decoders(Clock())
    decoders.observe_sys_status(message())

    assert decoders.rc_health_decoder(message(), 6, 1500) is None
    assert decoders.rc_health_decoder(message(chan7_raw=1499), 7, 1500) is None
    assert decoders.rc_health_decoder(message(src_system=2), 7, 1500) is None
    assert decoders.rc_health_decoder(message(), 7, True) is None

    for field in (
        "onboard_control_sensors_present",
        "onboard_control_sensors_enabled",
        "onboard_control_sensors_health",
    ):
        unhealthy = message(**{field: 0})
        decoders.observe_sys_status(unhealthy)
        assert decoders.rc_health_decoder(message(), 7, 1500) is False
        decoders.observe_sys_status(message())


def test_authority_dependency_revokes_rc_evidence_when_sys_status_expires(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    clock = Clock()
    state = flight_state(profile, clock)
    decoders = profile.decoders(clock, flight_state=state)
    decoders.observe_sys_status(message())
    state.observe_rc_input(
        channel=7,
        pwm=1500,
        signal_healthy=True,
        received_at=10.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )

    clock.now = 10.5
    with pytest.raises(AuthorityLost, match="SYS_STATUS"):
        decoders.check_authority_dependency()
    assert state.snapshot(now=10.5).rc_input is None
    assert state.authority is Authority.UNKNOWN


@pytest.mark.parametrize("check_name", ["check_rc_health", "check_authority_dependency"])
def test_rc_health_check_captures_implicit_time_with_the_health_snapshot(
    tmp_path, check_name
):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    clock = Clock()
    state = acquire_state(profile, clock)
    decoders = profile.decoders(clock, flight_state=state)
    decoders.observe_sys_status(message())
    coordinated_lock = CoordinatedRLock("health-reader")
    decoders._lock = coordinated_lock
    failures = []

    def check_health():
        try:
            getattr(decoders, check_name)()
        except BaseException as error:
            failures.append(error)

    with coordinated_lock:
        reader = threading.Thread(target=check_health, name="health-reader")
        reader.start()
        assert coordinated_lock.reader_waiting.wait(timeout=1.0)
        clock.now = 10.1
        decoders.observe_sys_status(message())
    reader.join(timeout=1.0)

    assert not reader.is_alive()
    assert failures == []
    assert state.authority is Authority.COMPANION


def test_live_rc_callback_captures_time_with_the_health_snapshot(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    clock = Clock()
    state = acquire_state(profile, clock)
    decoders = profile.decoders(clock, flight_state=state)
    decoders.observe_sys_status(message())
    coordinated_lock = CoordinatedRLock("rc-callback")
    decoders._lock = coordinated_lock
    observed_health = []

    def deliver_rc_callback():
        health = decoders.rc_health_decoder(message(), 7, 1500)
        observed_health.append(health)
        state.observe_rc_input(
            channel=7,
            pwm=1500,
            signal_healthy=health,
            received_at=10.1,
            sequence=2,
            source_system=1,
            source_component=1,
        )

    with coordinated_lock:
        reader = threading.Thread(target=deliver_rc_callback, name="rc-callback")
        reader.start()
        assert coordinated_lock.reader_waiting.wait(timeout=1.0)
        clock.now = 10.1
        decoders.observe_sys_status(message())
    reader.join(timeout=1.0)

    assert not reader.is_alive()
    assert observed_health == [True]
    assert state.authority is Authority.COMPANION


def test_rc_health_check_succeeds_before_acquisition_without_granting_authority(
    tmp_path,
):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    clock = Clock()
    state = flight_state(profile, clock)
    decoders = profile.decoders(clock, flight_state=state)
    decoders.observe_sys_status(message())

    assert decoders.check_rc_health() is None
    assert state.authority is Authority.UNKNOWN
    with pytest.raises(AuthorityLost, match="authority"):
        decoders.check_authority_dependency()


@pytest.mark.parametrize("health_case", ["absent", "malformed", "unhealthy", "expired"])
def test_rc_health_check_rejects_without_current_sys_status_and_revokes_newer_pwm(
    tmp_path, health_case
):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    clock = Clock()
    state = flight_state(profile, clock)
    decoders = profile.decoders(clock, flight_state=state)

    if health_case == "malformed":
        decoders.observe_sys_status(
            message(onboard_control_sensors_health="malformed")
        )
    elif health_case == "unhealthy":
        decoders.observe_sys_status(message(onboard_control_sensors_health=0))
    elif health_case == "expired":
        decoders.observe_sys_status(message())
        clock.now = 10.5

    assert state.observe_rc_input(
        channel=7,
        pwm=1500,
        signal_healthy=True,
        received_at=clock.now,
        sequence=2,
        source_system=1,
        source_component=1,
    )
    with pytest.raises(AuthorityLost, match="SYS_STATUS"):
        decoders.check_rc_health()

    assert state.snapshot(now=clock.now).rc_input is None
    assert state.authority is Authority.UNKNOWN


def test_current_unhealthy_sys_status_immediately_revokes_rc_evidence(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    clock = Clock()
    state = flight_state(profile, clock)
    decoders = profile.decoders(clock, flight_state=state)
    decoders.observe_sys_status(message())
    state.observe_rc_input(
        channel=7,
        pwm=1500,
        signal_healthy=True,
        received_at=10.0,
        sequence=1,
        source_system=1,
        source_component=1,
    )

    clock.now = 10.1
    decoders.observe_sys_status(message(onboard_control_sensors_health="bad"))

    assert state.snapshot(now=10.1).rc_input is None
    assert state.authority is Authority.UNKNOWN


def test_trusted_sys_status_with_invalid_receipt_clock_revokes_acquired_authority(
    tmp_path,
):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    clock = Clock()
    state = acquire_state(profile, clock)
    decoders = profile.decoders(clock, flight_state=state)
    decoders.observe_sys_status(message())

    clock.now = float("nan")
    decoders.observe_sys_status(message(onboard_control_sensors_health=0))
    clock.now = 10.1

    with pytest.raises(AuthorityLost, match="SYS_STATUS"):
        decoders.check_authority_dependency()
    assert state.snapshot(now=10.1).rc_input is None
    assert state.authority is Authority.UNKNOWN


def test_unhealthy_revocation_finishes_before_newer_health_can_publish(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    clock = Clock()
    state = acquire_state(profile, clock)
    decoders = profile.decoders(clock, flight_state=state)
    decoders.observe_sys_status(message())
    invalidation_entered = threading.Event()
    release_invalidation = threading.Event()
    newer_finished = threading.Event()
    errors = []
    original_invalidate = state.invalidate_observation

    def paused_invalidate(*args, **kwargs):
        invalidation_entered.set()
        if not release_invalidation.wait(1):
            raise TimeoutError("test did not release SYS_STATUS invalidation")
        return original_invalidate(*args, **kwargs)

    state.invalidate_observation = paused_invalidate

    def publish_unhealthy():
        try:
            decoders.observe_sys_status(
                message(onboard_control_sensors_health=0)
            )
        except BaseException as error:
            errors.append(error)

    def publish_newer_healthy_and_rc():
        try:
            clock.now = 10.2
            decoders.observe_sys_status(message())
            health = decoders.rc_health_decoder(message(), 7, 1500)
            if health is True:
                state.observe_rc_input(
                    channel=7,
                    pwm=1500,
                    signal_healthy=health,
                    received_at=10.2,
                    sequence=2,
                    source_system=1,
                    source_component=1,
                )
        except BaseException as error:
            errors.append(error)
        finally:
            newer_finished.set()

    clock.now = 10.1
    unhealthy = threading.Thread(target=publish_unhealthy)
    unhealthy.start()
    assert invalidation_entered.wait(1)
    newer = threading.Thread(target=publish_newer_healthy_and_rc)
    newer.start()
    overtook_revoke = newer_finished.wait(0.1)
    release_invalidation.set()
    unhealthy.join(1)
    newer.join(1)

    assert not errors
    assert overtook_revoke is False
    assert state.snapshot(now=10.2).rc_input is not None
    assert state.authority is Authority.UNKNOWN
    with pytest.raises(AuthorityLost, match="authority"):
        decoders.check_authority_dependency()


def test_invalid_clock_tombstone_rejects_already_inflight_older_health(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    state_clock = Clock(10.2)
    state = flight_state(profile, state_clock)
    decoder_clock = PausedOlderClock()
    decoders = profile.decoders(decoder_clock, flight_state=state)
    errors = []

    def publish_older_health():
        try:
            decoders.observe_sys_status(message())
        except BaseException as error:
            errors.append(error)

    older = threading.Thread(
        target=publish_older_health, name="older-sys-status"
    )
    older.start()
    assert decoder_clock.older_sampled.wait(1)
    decoders.observe_sys_status(message())
    decoder_clock.now = float("nan")
    decoders.observe_sys_status(message(onboard_control_sensors_health=0))
    decoder_clock.now = 10.2
    decoder_clock.release_older.set()
    older.join(1)

    assert not older.is_alive()
    assert not errors
    health = decoders.rc_health_decoder(message(), 7, 1500)
    assert health is None
    assert publish_ground_evidence_and_acquire(
        state, profile, healthy=health, now=10.2
    ) is False
    assert state.authority is Authority.UNKNOWN
    with pytest.raises(AuthorityLost, match="SYS_STATUS"):
        decoders.check_authority_dependency()


def test_guard_failure_keeps_cutoff_against_already_inflight_older_health(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    state_clock = Clock(10.2)
    state = flight_state(profile, state_clock)
    decoder_clock = PausedOlderClock()
    decoders = profile.decoders(decoder_clock, flight_state=state)
    errors = []

    def publish_older_health():
        try:
            decoders.observe_sys_status(message())
        except BaseException as error:
            errors.append(error)

    older = threading.Thread(
        target=publish_older_health, name="older-sys-status"
    )
    older.start()
    assert decoder_clock.older_sampled.wait(1)
    decoders.observe_sys_status(message(onboard_control_sensors_health=0))
    decoder_clock.now = 10.2
    with pytest.raises(AuthorityLost, match="SYS_STATUS"):
        decoders.check_authority_dependency()
    decoder_clock.release_older.set()
    older.join(1)

    assert not older.is_alive()
    assert not errors
    health = decoders.rc_health_decoder(message(), 7, 1500)
    assert health is False
    assert publish_ground_evidence_and_acquire(
        state, profile, healthy=health, now=10.2
    ) is False
    assert state.authority is Authority.UNKNOWN
    with pytest.raises(AuthorityLost, match="SYS_STATUS"):
        decoders.check_authority_dependency()


def test_invalid_guard_clock_clears_usable_health_but_keeps_ordering_cutoff(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    state = flight_state(profile, Clock(10.1))
    decoder_clock = Clock(10.0)
    decoders = profile.decoders(decoder_clock, flight_state=state)
    decoders.observe_sys_status(message())

    decoder_clock.now = float("nan")
    with pytest.raises(AuthorityLost, match="SYS_STATUS"):
        decoders.check_authority_dependency()
    decoder_clock.now = 10.1

    assert decoders.rc_health_decoder(message(), 7, 1500) is None


def test_guard_invalidation_blocks_healthy_callback_already_inflight(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    state = flight_state(profile, Clock(10.2))
    decoder_clock = PausedOlderClock(paused_value=10.1)
    decoder_clock.now = 10.0
    decoders = profile.decoders(decoder_clock, flight_state=state)
    decoders.observe_sys_status(message())
    errors = []

    def publish_pending_health():
        try:
            decoders.observe_sys_status(message())
        except BaseException as error:
            errors.append(error)

    pending = threading.Thread(
        target=publish_pending_health, name="older-sys-status"
    )
    pending.start()
    assert decoder_clock.older_sampled.wait(1)
    decoder_clock.now = float("nan")
    with pytest.raises(AuthorityLost, match="SYS_STATUS"):
        decoders.check_authority_dependency()
    decoder_clock.now = 10.2
    decoder_clock.release_older.set()
    pending.join(1)

    assert not pending.is_alive()
    assert not errors
    health = decoders.rc_health_decoder(message(), 7, 1500)
    assert health is None
    assert publish_ground_evidence_and_acquire(
        state, profile, healthy=health, now=10.2
    ) is False
    assert state.authority is Authority.UNKNOWN


def test_numeric_old_rejection_does_not_suppress_pending_newer_unhealthy(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    state = flight_state(profile, Clock(10.2))
    decoder_clock = PausedOlderClock(paused_value=10.1)
    decoder_clock.now = 10.0
    decoders = profile.decoders(decoder_clock, flight_state=state)
    decoders.observe_sys_status(message())
    errors = []

    def publish_pending_unhealthy():
        try:
            decoders.observe_sys_status(
                message(onboard_control_sensors_health=0)
            )
        except BaseException as error:
            errors.append(error)

    pending = threading.Thread(
        target=publish_pending_unhealthy, name="older-sys-status"
    )
    pending.start()
    assert decoder_clock.older_sampled.wait(1)
    decoder_clock.now = 9.0
    decoders.observe_sys_status(message())
    decoder_clock.now = 10.2
    decoder_clock.release_older.set()
    pending.join(1)

    assert not pending.is_alive()
    assert not errors
    health = decoders.rc_health_decoder(message(), 7, 1500)
    assert health is False
    assert publish_ground_evidence_and_acquire(
        state, profile, healthy=health, now=10.2
    ) is False
    assert state.authority is Authority.UNKNOWN
    with pytest.raises(AuthorityLost, match="SYS_STATUS"):
        decoders.check_authority_dependency()


def test_expiry_barrier_blocks_healthy_callback_already_inflight(tmp_path):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    state = flight_state(profile, Clock(10.5))
    decoder_clock = PausedOlderClock(paused_value=10.4)
    decoder_clock.now = 10.0
    decoders = profile.decoders(decoder_clock, flight_state=state)
    decoders.observe_sys_status(message())
    errors = []

    def publish_pending_health():
        try:
            decoders.observe_sys_status(message())
        except BaseException as error:
            errors.append(error)

    pending = threading.Thread(
        target=publish_pending_health, name="older-sys-status"
    )
    pending.start()
    assert decoder_clock.older_sampled.wait(1)
    decoder_clock.now = 10.5
    assert decoders.rc_health_decoder(message(), 7, 1500) is None
    decoder_clock.release_older.set()
    pending.join(1)

    assert not pending.is_alive()
    assert not errors
    health = decoders.rc_health_decoder(message(), 7, 1500)
    assert health is None
    assert publish_ground_evidence_and_acquire(
        state, profile, healthy=health, now=10.5
    ) is False
    assert state.authority is Authority.UNKNOWN


def test_superseded_invalid_clock_cannot_revoke_newer_health_before_acquisition(
    tmp_path,
):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    state = flight_state(profile, Clock(10.2))
    decoder_clock = PausedOlderClock(paused_value=float("nan"))
    decoder_clock.now = 10.0
    decoders = profile.decoders(decoder_clock, flight_state=state)
    decoders.observe_sys_status(message())
    errors = []

    def publish_pending_invalid_clock():
        try:
            decoders.observe_sys_status(message())
        except BaseException as error:
            errors.append(error)

    pending = threading.Thread(
        target=publish_pending_invalid_clock, name="older-sys-status"
    )
    pending.start()
    assert decoder_clock.older_sampled.wait(1)
    decoder_clock.now = 10.15
    decoders.observe_sys_status(message())
    decoder_clock.now = 10.2
    decoder_clock.release_older.set()
    pending.join(1)

    assert not pending.is_alive()
    assert not errors
    health = decoders.rc_health_decoder(message(), 7, 1500)
    assert health is True
    assert publish_ground_evidence_and_acquire(
        state, profile, healthy=health, now=10.2
    ) is True
    assert state.authority is Authority.COMPANION
    decoders.check_authority_dependency()


def test_superseded_invalid_clock_cannot_revoke_newer_health_after_acquisition(
    tmp_path,
):
    path, _ = write_profile(tmp_path)
    profile = load_flight_profile(path)
    state_clock = Clock(10.0)
    state = flight_state(profile, state_clock)
    decoder_clock = PausedOlderClock(paused_value=float("nan"))
    decoder_clock.now = 10.0
    decoders = profile.decoders(decoder_clock, flight_state=state)
    decoders.observe_sys_status(message())
    assert publish_ground_evidence_and_acquire(
        state, profile, healthy=True, now=10.0
    ) is True
    errors = []

    def publish_pending_invalid_clock():
        try:
            decoders.observe_sys_status(message())
        except BaseException as error:
            errors.append(error)

    pending = threading.Thread(
        target=publish_pending_invalid_clock, name="older-sys-status"
    )
    pending.start()
    assert decoder_clock.older_sampled.wait(1)
    decoder_clock.now = 10.15
    decoders.observe_sys_status(message())
    state_clock.now = 10.15
    assert state.observe_rc_input(
        channel=7,
        pwm=1500,
        signal_healthy=True,
        received_at=10.15,
        sequence=2,
        source_system=1,
        source_component=1,
    )
    state_clock.now = 10.2
    decoder_clock.now = 10.2
    decoder_clock.release_older.set()
    pending.join(1)

    assert not pending.is_alive()
    assert not errors
    assert decoders.rc_health_decoder(message(), 7, 1500) is True
    assert state.snapshot(now=10.2).rc_input is not None
    assert state.authority is Authority.COMPANION
    decoders.check_authority_dependency()


def test_missing_dependencies_fail_closed_and_imports_are_inert(tmp_path, capsys):
    path, _ = write_profile(tmp_path)
    decoders = load_flight_profile(path).decoders(Clock())
    with pytest.raises(AuthorityLost, match="FlightState"):
        decoders.check_authority_dependency()
    assert capsys.readouterr() == ("", "")
