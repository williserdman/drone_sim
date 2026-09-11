import math
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from drone.sensors.lidar.lidar import (
    Lidar,
    LidarConfigurationError,
    LidarResource,
    LidarStartupError,
    StaleSensorError,
)


class MutableClock:
    def __init__(self, now=10.0):
        self.now = now

    def __call__(self):
        return self.now


class RepeatingSensor:
    def __init__(self, distance):
        self.value = distance

    @property
    def distance(self):
        if isinstance(self.value, Exception):
            raise self.value
        return self.value


def inert_resource(sensor):
    return LidarResource(sensor=sensor, cleanup=lambda: None)


def make_lidar(sensor, clock, **overrides):
    settings = {
        "sensor_factory": lambda: inert_resource(sensor),
        "raw_min_cm": 10.0,
        "raw_max_cm": 1_000.0,
        "mounting_offset_cm": 10.0,
        "stale_after_seconds": 0.5,
        "startup_timeout_seconds": 0.05,
        "poll_interval_seconds": 0.001,
        "clock": clock,
    }
    settings.update(overrides)
    return Lidar(**settings)


def test_stop_reports_worker_and_owned_cleanup_completion_in_order():
    events = []

    class RecordingSensor:
        @property
        def distance(self):
            events.append("read")
            return 110.0

    resource = LidarResource(
        sensor=RecordingSensor(), cleanup=lambda: events.append("cleanup")
    )
    lidar = make_lidar(
        resource.sensor,
        MutableClock(),
        sensor_factory=lambda: resource,
    )

    status = lidar.stop(timeout_seconds=0.1)

    assert status.worker_stopped is True
    assert status.cleanup_completed is True
    assert events[-1] == "cleanup"


def test_stop_reports_cleanup_exception_as_unconfirmed():
    def fail_cleanup():
        raise RuntimeError("cleanup failed")

    sensor = RepeatingSensor(110.0)
    lidar = make_lidar(
        sensor,
        MutableClock(),
        sensor_factory=lambda: LidarResource(sensor, fail_cleanup),
    )

    status = lidar.stop(timeout_seconds=0.1)

    assert status.worker_stopped is True
    assert status.cleanup_completed is False
    assert status.cleanup_error == "RuntimeError: cleanup failed"


def test_stop_reports_unconfirmed_resource_state_while_cleanup_is_blocked():
    cleanup_started = threading.Event()
    release_cleanup = threading.Event()

    def blocked_cleanup():
        cleanup_started.set()
        release_cleanup.wait(timeout=1.0)

    sensor = RepeatingSensor(110.0)
    lidar = make_lidar(
        sensor,
        MutableClock(),
        sensor_factory=lambda: LidarResource(sensor, blocked_cleanup),
    )

    status = lidar.stop(timeout_seconds=0.01)

    assert cleanup_started.is_set()
    assert status.worker_stopped is False
    assert status.cleanup_completed is False

    release_cleanup.set()
    status = lidar.stop(timeout_seconds=0.2)
    assert status.worker_stopped is True
    assert status.cleanup_completed is True


def test_stop_reports_unconfirmed_cleanup_while_native_read_is_blocked():
    second_read_started = threading.Event()
    release_read = threading.Event()
    cleaned = threading.Event()

    class BlockedAfterStartupSensor:
        def __init__(self):
            self.reads = 0

        @property
        def distance(self):
            self.reads += 1
            if self.reads == 1:
                return 110.0
            second_read_started.set()
            release_read.wait(timeout=1.0)
            return 110.0

    sensor = BlockedAfterStartupSensor()
    resource = LidarResource(sensor=sensor, cleanup=cleaned.set)
    lidar = make_lidar(sensor, MutableClock(), sensor_factory=lambda: resource)
    assert second_read_started.wait(timeout=0.2)

    status = lidar.stop(timeout_seconds=0.01)

    assert status.worker_stopped is False
    assert status.cleanup_completed is False
    assert not cleaned.is_set()

    release_read.set()
    status = lidar.stop(timeout_seconds=0.2)
    assert status.worker_stopped is True
    assert status.cleanup_completed is True
    assert cleaned.is_set()


def test_get_sample_cannot_return_snapshot_after_concurrent_cancellation():
    second_read_started = threading.Event()
    release_read = threading.Event()
    getter_clock_started = threading.Event()
    release_getter_clock = threading.Event()
    outcome = []

    class BlockedAfterStartupSensor:
        def __init__(self):
            self.reads = 0

        @property
        def distance(self):
            self.reads += 1
            if self.reads == 1:
                return 110.0
            second_read_started.set()
            release_read.wait(timeout=1.0)
            return 110.0

    class BlockingGetterClock:
        def __init__(self):
            self.calls = 0

        def __call__(self):
            self.calls += 1
            if self.calls > 1:
                getter_clock_started.set()
                release_getter_clock.wait(timeout=1.0)
            return 10.0

    sensor = BlockedAfterStartupSensor()
    lidar = make_lidar(sensor, BlockingGetterClock())
    assert second_read_started.wait(timeout=0.2)

    def get_sample():
        try:
            outcome.append(lidar.get_sample())
        except Exception as exc:
            outcome.append(exc)

    getter = threading.Thread(target=get_sample)
    getter.start()
    assert getter_clock_started.wait(timeout=0.2)

    lidar.stop(timeout_seconds=0.01)
    release_getter_clock.set()
    getter.join(timeout=0.2)

    assert len(outcome) == 1
    assert isinstance(outcome[0], StaleSensorError)

    release_read.set()
    lidar.stop(timeout_seconds=0.2)


def test_get_sample_rechecks_age_after_waiting_for_final_state_lock():
    second_read_started = threading.Event()
    release_read = threading.Event()
    getter_clock_started = threading.Event()
    release_getter_clock = threading.Event()
    final_lock_attempted = threading.Event()
    outcome = []

    class BlockedAfterStartupSensor:
        def __init__(self):
            self.reads = 0

        @property
        def distance(self):
            self.reads += 1
            if self.reads == 1:
                return 110.0
            second_read_started.set()
            release_read.wait(timeout=1.0)
            return 110.0

    class GateSecondClockCall:
        def __init__(self):
            self.calls = 0
            self.now = 10.0

        def __call__(self):
            self.calls += 1
            if self.calls == 2:
                getter_clock_started.set()
                release_getter_clock.wait(timeout=1.0)
            return self.now

    class TrackGetterLockAttempts:
        def __init__(self, lock):
            self.lock = lock
            self.getter_ident = None
            self.getter_attempts = 0

        def acquire(self, *args, **kwargs):
            if threading.get_ident() == self.getter_ident:
                self.getter_attempts += 1
                if self.getter_attempts == 2:
                    final_lock_attempted.set()
            return self.lock.acquire(*args, **kwargs)

        def release(self):
            self.lock.release()

        def __enter__(self):
            self.acquire()
            return self

        def __exit__(self, _exc_type, _exc, _traceback):
            self.release()

    sensor = BlockedAfterStartupSensor()
    clock = GateSecondClockCall()
    lidar = make_lidar(sensor, clock)
    assert second_read_started.wait(timeout=0.2)
    underlying_lock = lidar._sample_lock
    tracking_lock = TrackGetterLockAttempts(underlying_lock)
    lidar._sample_lock = tracking_lock

    def get_sample():
        tracking_lock.getter_ident = threading.get_ident()
        try:
            outcome.append(lidar.get_sample())
        except Exception as exc:
            outcome.append(exc)

    getter = threading.Thread(target=get_sample)
    getter.start()
    assert getter_clock_started.wait(timeout=0.2)
    underlying_lock.acquire()
    release_getter_clock.set()
    assert final_lock_attempted.wait(timeout=0.2)

    clock.now = 10.6
    underlying_lock.release()
    getter.join(timeout=0.2)

    assert len(outcome) == 1
    assert isinstance(outcome[0], StaleSensorError)
    assert "stale" in str(outcome[0])

    release_read.set()
    lidar.stop(timeout_seconds=0.2)


def test_cancelled_startup_cleans_owned_resource_after_blocked_read_returns():
    release_read = threading.Event()
    cleaned = threading.Event()

    class BlockedSensor:
        @property
        def distance(self):
            release_read.wait(timeout=1.0)
            return 110.0

    resource = LidarResource(sensor=BlockedSensor(), cleanup=cleaned.set)
    with pytest.raises(LidarStartupError) as raised:
        make_lidar(
            resource.sensor,
            MutableClock(),
            sensor_factory=lambda: resource,
            startup_timeout_seconds=0.02,
        )

    assert raised.value.shutdown_status.worker_stopped is False
    assert raised.value.shutdown_status.cleanup_completed is False
    assert not cleaned.is_set()
    release_read.set()
    assert cleaned.wait(timeout=0.2)


def test_default_factory_deinitializes_i2c_when_driver_construction_fails(monkeypatch):
    events = []

    class Bus:
        def deinit(self):
            events.append("deinit")

    bus = Bus()
    monkeypatch.setitem(sys.modules, "board", SimpleNamespace(SCL=1, SDA=2))
    monkeypatch.setitem(
        sys.modules, "busio", SimpleNamespace(I2C=lambda _scl, _sda: bus)
    )

    def fail_driver(_bus):
        events.append("driver")
        raise RuntimeError("driver construction failed")

    monkeypatch.setitem(
        sys.modules,
        "adafruit_lidarlite",
        SimpleNamespace(LIDARLite=fail_driver),
    )

    with pytest.raises(LidarStartupError):
        Lidar(
            raw_min_cm=10.0,
            raw_max_cm=1_000.0,
            mounting_offset_cm=10.0,
            stale_after_seconds=0.5,
            startup_timeout_seconds=0.02,
            poll_interval_seconds=0.001,
            clock=MutableClock(),
        )

    assert events == ["driver", "deinit"]


def test_failed_constructor_cleanup_is_reported_as_unconfirmed(monkeypatch):
    class Bus:
        def deinit(self):
            raise RuntimeError("bus remains open")

    monkeypatch.setitem(sys.modules, "board", SimpleNamespace(SCL=1, SDA=2))
    monkeypatch.setitem(
        sys.modules, "busio", SimpleNamespace(I2C=lambda _scl, _sda: Bus())
    )

    def fail_driver(_bus):
        raise RuntimeError("driver construction failed")

    monkeypatch.setitem(
        sys.modules,
        "adafruit_lidarlite",
        SimpleNamespace(LIDARLite=fail_driver),
    )

    with pytest.raises(LidarStartupError) as raised:
        Lidar(
            raw_min_cm=10.0,
            raw_max_cm=1_000.0,
            mounting_offset_cm=10.0,
            stale_after_seconds=0.5,
            startup_timeout_seconds=0.02,
            poll_interval_seconds=0.001,
            clock=MutableClock(),
        )

    status = raised.value.shutdown_status
    assert status.worker_stopped is True
    assert status.cleanup_completed is False
    assert status.cleanup_error == "RuntimeError: bus remains open"


def test_default_factory_closes_driver_before_i2c_after_reads_finish(monkeypatch):
    events = []

    class Bus:
        def deinit(self):
            events.append("bus cleanup")

    class Driver:
        @property
        def distance(self):
            events.append("read")
            return 110.0

        def deinit(self):
            events.append("driver cleanup")

    bus = Bus()
    monkeypatch.setitem(sys.modules, "board", SimpleNamespace(SCL=1, SDA=2))
    monkeypatch.setitem(
        sys.modules, "busio", SimpleNamespace(I2C=lambda _scl, _sda: bus)
    )
    monkeypatch.setitem(
        sys.modules,
        "adafruit_lidarlite",
        SimpleNamespace(LIDARLite=lambda _bus: Driver()),
    )
    lidar = Lidar(
        raw_min_cm=10.0,
        raw_max_cm=1_000.0,
        mounting_offset_cm=10.0,
        stale_after_seconds=0.5,
        startup_timeout_seconds=0.02,
        poll_interval_seconds=0.001,
        clock=MutableClock(),
    )

    status = lidar.stop(timeout_seconds=0.1)

    assert status.cleanup_completed is True
    assert events[-2:] == ["driver cleanup", "bus cleanup"]


def test_first_real_sample_is_corrected_and_published_as_one_snapshot():
    clock = MutableClock()
    lidar = make_lidar(RepeatingSensor(160), clock)
    try:
        sample = lidar.get_sample()
        assert sample.distance_m == 1.5
        assert sample.sampled_at == 10.0
        assert sample.sequence >= 1
        with pytest.raises(AttributeError):
            sample.distance_m = 99.0
    finally:
        lidar.stop()


@pytest.mark.parametrize(
    "raw_distance", [9.0, 1_001.0, -1.0, math.nan, math.inf, True, 10**400]
)
def test_invalid_raw_values_cannot_satisfy_first_sample(raw_distance):
    with pytest.raises(LidarStartupError):
        make_lidar(RepeatingSensor(raw_distance), MutableClock())


def test_negative_corrected_distance_cannot_satisfy_first_sample():
    with pytest.raises(LidarStartupError):
        make_lidar(
            RepeatingSensor(10.0),
            MutableClock(),
            raw_min_cm=0.0,
            mounting_offset_cm=10.1,
        )


def test_zero_corrected_distance_is_a_valid_range_measurement():
    lidar = make_lidar(RepeatingSensor(10.0), MutableClock())
    try:
        assert lidar.get_distance() == 0.0
    finally:
        lidar.stop()


def test_missing_and_stale_samples_never_return_a_fabricated_distance():
    clock = MutableClock()
    with pytest.raises(LidarStartupError):
        make_lidar(RepeatingSensor(RuntimeError("not a distance")), clock)

    sensor = RepeatingSensor(110.0)
    lidar = make_lidar(sensor, clock)
    try:
        sensor.value = RuntimeError("I2C unavailable")
        time.sleep(0.01)
        clock.now = 10.51
        with pytest.raises(StaleSensorError, match="no valid sample"):
            lidar.get_distance()
    finally:
        lidar.stop()


@pytest.mark.parametrize(
    "invalid_read", [math.nan, RuntimeError("I2C unavailable")]
)
def test_invalid_read_immediately_revokes_range_and_changes_next_generation(
    invalid_read,
):
    clock = MutableClock()
    sensor = RepeatingSensor(110.0)
    lidar = make_lidar(sensor, clock)
    try:
        first = lidar.get_sample()

        sensor.value = invalid_read
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            try:
                lidar.get_sample()
            except StaleSensorError:
                break
            time.sleep(0.001)
        else:
            pytest.fail("invalid hardware read did not revoke the usable sample")

        sensor.value = 120.0
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            try:
                recovered = lidar.get_sample()
            except StaleSensorError:
                time.sleep(0.001)
                continue
            if recovered.sequence > first.sequence:
                break
        else:
            pytest.fail("valid hardware reading was not republished")

        assert recovered.invalidation_generation > first.invalidation_generation
    finally:
        lidar.stop()


def test_clock_rollback_rejects_a_future_dated_sample():
    clock = MutableClock(20.0)
    sensor = RepeatingSensor(110.0)
    lidar = make_lidar(sensor, clock)
    try:
        sensor.value = RuntimeError("I2C unavailable")
        time.sleep(0.01)
        clock.now = 19.9
        with pytest.raises(StaleSensorError):
            lidar.get_sample()
    finally:
        lidar.stop()


def test_backward_sample_receipt_cannot_replace_invalidated_evidence():
    clock = MutableClock(20.0)
    sensor = RepeatingSensor(110.0)
    lidar = make_lidar(sensor, clock)
    try:
        sensor.value = RuntimeError("I2C unavailable")
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            try:
                lidar.get_sample()
            except StaleSensorError:
                break
            time.sleep(0.001)
        else:
            pytest.fail("invalid hardware read did not revoke the usable sample")

        clock.now = 19.0
        sensor.value = 120.0
        time.sleep(0.01)

        with pytest.raises(StaleSensorError, match="no valid sample"):
            lidar.get_sample()
    finally:
        lidar.stop()


@pytest.mark.parametrize("malformed_now", [True, "now", 10**400])
def test_malformed_current_clock_never_makes_range_available(malformed_now):
    clock = MutableClock(20.0)
    lidar = make_lidar(RepeatingSensor(110.0), clock)
    try:
        clock.now = malformed_now
        with pytest.raises(StaleSensorError, match="clock"):
            lidar.get_sample()
    finally:
        lidar.stop()


@pytest.mark.parametrize(
    ("override", "value"),
    [
        ("raw_min_cm", math.nan),
        ("raw_max_cm", math.inf),
        ("raw_max_cm", 10**400),
        ("mounting_offset_cm", -0.1),
        ("mounting_offset_cm", 1_000.1),
        ("stale_after_seconds", 0.0),
        ("startup_timeout_seconds", 0.0),
        ("poll_interval_seconds", 0.0),
    ],
)
def test_physical_and_timing_configuration_must_be_explicitly_valid(override, value):
    with pytest.raises(LidarConfigurationError):
        make_lidar(RepeatingSensor(100.0), MutableClock(), **{override: value})


def test_slow_sensor_factory_is_inside_the_single_startup_budget():
    factory_started = threading.Event()
    release_factory = threading.Event()

    def blocked_factory():
        factory_started.set()
        release_factory.wait(timeout=1.0)
        return inert_resource(RepeatingSensor(100.0))

    started_at = time.monotonic()
    try:
        with pytest.raises(LidarStartupError):
            make_lidar(
                RepeatingSensor(100.0),
                MutableClock(),
                sensor_factory=blocked_factory,
                startup_timeout_seconds=0.02,
                poll_interval_seconds=0.06,
            )
    finally:
        release_factory.set()

    assert factory_started.is_set()
    assert time.monotonic() - started_at < 0.08


def test_blocked_first_read_times_out_without_late_publication():
    release_read = threading.Event()
    read_returned = threading.Event()
    clock_calls = []

    class BlockedSensor:
        @property
        def distance(self):
            release_read.wait(timeout=1.0)
            read_returned.set()
            return 100.0

    def recording_clock():
        clock_calls.append(time.monotonic())
        return 10.0

    started_at = time.monotonic()
    try:
        with pytest.raises(LidarStartupError):
            make_lidar(
                BlockedSensor(),
                recording_clock,
                startup_timeout_seconds=0.02,
                poll_interval_seconds=0.06,
            )
    finally:
        release_read.set()

    assert time.monotonic() - started_at < 0.08
    assert read_returned.wait(timeout=0.2)
    time.sleep(0.01)
    assert clock_calls == []


class PausedPublicationLidar(Lidar):
    def __init__(self, *args, pause_publication_number=1, **kwargs):
        self.publication_reached = threading.Event()
        self.release_publication = threading.Event()
        self.publication_results = []
        self._publication_number = 0
        self._pause_publication_number = pause_publication_number
        super().__init__(*args, **kwargs)

    def _publish_sample(self, sample):
        self._publication_number += 1
        if self._publication_number == self._pause_publication_number:
            self.publication_reached.set()
            self.release_publication.wait(timeout=1.0)
        published = super()._publish_sample(sample)
        self.publication_results.append(published)
        return published


def publication_lidar_settings(**overrides):
    settings = {
        "raw_min_cm": 10.0,
        "raw_max_cm": 1_000.0,
        "mounting_offset_cm": 10.0,
        "stale_after_seconds": 0.5,
        "startup_timeout_seconds": 0.02,
        "poll_interval_seconds": 0.001,
        "sensor_factory": lambda: inert_resource(RepeatingSensor(100.0)),
        "clock": MutableClock(),
    }
    settings.update(overrides)
    return settings


def test_startup_timeout_cancellation_wins_final_publication_interleaving():
    holder = {}

    class CapturedPausedLidar(PausedPublicationLidar):
        def __init__(self, *args, **kwargs):
            holder["lidar"] = self
            super().__init__(*args, **kwargs)

    try:
        with pytest.raises(LidarStartupError):
            CapturedPausedLidar(**publication_lidar_settings())
    finally:
        lidar = holder["lidar"]
        lidar.release_publication.set()

    assert lidar.publication_reached.is_set()
    lidar._thread.join(timeout=0.2)
    assert lidar.publication_results == [False]
    assert lidar._sample is None
    assert not lidar._first_sample.is_set()


def test_normal_stop_cancellation_wins_final_publication_interleaving():
    lidar = PausedPublicationLidar(
        pause_publication_number=2,
        **publication_lidar_settings(startup_timeout_seconds=0.1),
    )
    lidar.get_sample()
    assert lidar.publication_reached.wait(timeout=0.2)

    lidar.stop()
    lidar.release_publication.set()
    lidar._thread.join(timeout=0.2)

    assert lidar.publication_results == [True, False]
    with pytest.raises(StaleSensorError, match="no valid sample"):
        lidar.get_sample()
