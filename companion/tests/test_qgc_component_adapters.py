from __future__ import annotations

import threading
from types import SimpleNamespace

import pytest

from drone_sim_companion.comp2026_host import (
    PayloadDropper,
    QgcFm2PayloadAdapter,
    QgcRangeIngress,
    QgcRosLidarAdapter,
    SimulationClock,
)


class InertRosLidar:
    def __init__(self) -> None:
        self.events: list[tuple[str, object]] = []
        self.revoked = False
        self.sample = object()

    def get_sample(self) -> object:
        return self.sample

    def get_distance(self) -> float:
        return 4.25

    def stop(self, reason: str) -> None:
        self.events.append(("stop", reason))
        self.revoked = True


class InertPayloadDropper:
    def __init__(
        self, *, aruco_id: object = 2, cleanup_result: object = True
    ) -> None:
        self.aruco_id = aruco_id
        self.calls: list[tuple[object, ...]] = []
        self.cleanup_result = cleanup_result

    def drop(self, delay_hold: float = 0.0) -> None:
        self.calls.append(("drop", delay_hold))

    def drop_with_guard(
        self,
        release_actuate,
        continuation_actuate,
        *,
        delay_hold: float = 0.0,
    ) -> None:
        self.calls.append(
            ("drop_with_guard", release_actuate, continuation_actuate, delay_hold)
        )

    def cleanup_passive(self) -> object:
        self.calls.append(("cleanup_passive",))
        return self.cleanup_result

    def attach(self, aruco_id: int) -> bool:
        self.calls.append(("attach", aruco_id))
        return True


class InertPayloadClient:
    def __init__(self) -> None:
        self.prepared: list[object] = []
        self.dispatched: list[object] = []

    def prepare(self, request: object) -> object:
        self.prepared.append(request)
        return request

    def dispatch(self, prepared: object) -> object:
        self.dispatched.append(prepared)
        return prepared

    def await_response(self, pending: object, *, cancelled: object) -> object:
        del cancelled
        return SimpleNamespace(
            accepted=True,
            command_id=pending.command_id,
            response_sequence=1,
        )


def inert_permission():
    def permission() -> bool:
        return True

    def actuate(output) -> None:
        output()

    permission.actuate = actuate
    return permission


def real_payload_dropper(aruco_id: object) -> tuple[PayloadDropper, InertPayloadClient]:
    client = InertPayloadClient()
    dropper = PayloadDropper(
        "00000000-0000-4000-8000-000000000001",
        aruco_id,
        client,
        SimulationClock(),
        permission=inert_permission(),
        delay_wall_timeout_seconds=1.0,
    )
    return dropper, client


def test_adapter_construction_is_inert_and_silent(capsys) -> None:
    lidar = InertRosLidar()
    dropper = InertPayloadDropper()

    QgcRosLidarAdapter(lidar)
    QgcFm2PayloadAdapter(dropper, aruco_id=2)

    captured = capsys.readouterr()
    assert captured.out == ""
    assert captured.err == ""
    assert lidar.events == []
    assert dropper.calls == []


def test_payload_dropper_exposes_read_only_aruco_identity() -> None:
    dropper, _client = real_payload_dropper(2)

    assert dropper.aruco_id == 2
    with pytest.raises(AttributeError):
        dropper.aruco_id = 3


@pytest.mark.parametrize("invalid_aruco_id", [True, False, 2.0, "2"])
def test_payload_dropper_rejects_non_integer_public_identity(
    invalid_aruco_id: object,
) -> None:
    with pytest.raises(TypeError, match="ArUco ID"):
        real_payload_dropper(invalid_aruco_id)


@pytest.mark.parametrize("invalid_aruco_id", [True, False, 2.0, "2", 1, 3])
def test_fm2_payload_adapter_rejects_every_id_except_integer_two(
    invalid_aruco_id: object,
) -> None:
    dropper = InertPayloadDropper()

    with pytest.raises((TypeError, ValueError)):
        QgcFm2PayloadAdapter(dropper, aruco_id=invalid_aruco_id)

    assert dropper.calls == []


@pytest.mark.parametrize(
    "dropper",
    [object(), InertPayloadDropper(aruco_id=True), InertPayloadDropper(aruco_id=3)],
)
def test_fm2_payload_adapter_rejects_missing_bool_or_wrong_delegate_identity(
    dropper: object,
) -> None:
    with pytest.raises((TypeError, ValueError)):
        QgcFm2PayloadAdapter(dropper, aruco_id=2)


def test_fm2_payload_adapter_rejects_real_id_three_dropper_before_request() -> None:
    dropper, client = real_payload_dropper(3)

    with pytest.raises(ValueError, match="delegate"):
        QgcFm2PayloadAdapter(dropper, aruco_id=2)

    assert client.prepared == []
    assert client.dispatched == []


@pytest.mark.parametrize("operation", ["drop", "drop_with_guard"])
def test_fm2_payload_adapter_real_delegate_emits_only_id_two(
    operation: str,
) -> None:
    dropper, client = real_payload_dropper(2)
    adapter = QgcFm2PayloadAdapter(dropper, aruco_id=2)

    if operation == "drop":
        adapter.drop()
    else:
        adapter.drop_with_guard(lambda output: output(), lambda output: output())

    assert len(client.prepared) == 1
    assert client.prepared[0].aruco_id == 2
    assert client.dispatched == client.prepared


def test_fm2_payload_adapter_never_delegates_attachment() -> None:
    dropper = InertPayloadDropper()
    adapter = QgcFm2PayloadAdapter(dropper, aruco_id=2)

    assert adapter.supports_attachment is False
    with pytest.raises(NotImplementedError, match="FM2"):
        adapter.attach(2)

    assert dropper.calls == []


def test_fm2_payload_adapter_delegates_one_drop() -> None:
    dropper = InertPayloadDropper()
    adapter = QgcFm2PayloadAdapter(dropper, aruco_id=2)

    adapter.drop(delay_hold=1.25)

    assert dropper.calls == [("drop", 1.25)]


def test_fm2_payload_adapter_preserves_guard_argument_identity() -> None:
    dropper = InertPayloadDropper()
    adapter = QgcFm2PayloadAdapter(dropper, aruco_id=2)
    release_actuate = object()
    continuation_actuate = object()

    adapter.drop_with_guard(
        release_actuate,
        continuation_actuate,
        delay_hold=0.75,
    )

    assert dropper.calls == [
        (
            "drop_with_guard",
            release_actuate,
            continuation_actuate,
            0.75,
        )
    ]


@pytest.mark.parametrize("cleanup_result", [False, 1, "true", None])
def test_fm2_payload_adapter_cleanup_fails_closed(cleanup_result: object) -> None:
    dropper = InertPayloadDropper(cleanup_result=cleanup_result)
    adapter = QgcFm2PayloadAdapter(dropper, aruco_id=2)

    assert adapter.cleanup_passive() is False
    assert dropper.calls == [("cleanup_passive",)]


def test_fm2_payload_adapter_accepts_literal_true_cleanup() -> None:
    dropper = InertPayloadDropper(cleanup_result=True)
    adapter = QgcFm2PayloadAdapter(dropper, aruco_id=2)

    assert adapter.cleanup_passive() is True
    assert dropper.calls == [("cleanup_passive",)]


def test_fm2_payload_adapter_repeats_confirmed_delegate_cleanup() -> None:
    dropper = InertPayloadDropper(cleanup_result=True)
    adapter = QgcFm2PayloadAdapter(dropper, aruco_id=2)

    assert adapter.cleanup_passive() is True
    assert adapter.cleanup_passive() is True
    assert dropper.calls == [("cleanup_passive",), ("cleanup_passive",)]


def test_fm2_payload_adapter_does_not_hide_delegate_cleanup_exception() -> None:
    class FailingCleanupDropper(InertPayloadDropper):
        def cleanup_passive(self) -> object:
            self.calls.append(("cleanup_passive",))
            raise RuntimeError("delegate cleanup failed")

    dropper = FailingCleanupDropper()
    adapter = QgcFm2PayloadAdapter(dropper, aruco_id=2)

    with pytest.raises(RuntimeError, match="delegate cleanup failed"):
        adapter.cleanup_passive()
    assert dropper.calls == [("cleanup_passive",)]


def test_qgc_lidar_adapter_preserves_sample_reads() -> None:
    lidar = InertRosLidar()
    adapter = QgcRosLidarAdapter(lidar)

    assert adapter.get_sample() is lidar.sample
    assert adapter.get_distance() == 4.25


def test_qgc_lidar_stop_revokes_before_reporting_synchronous_cleanup() -> None:
    lidar = InertRosLidar()
    adapter = QgcRosLidarAdapter(lidar)

    status = adapter.stop(timeout_seconds=0.0)

    assert lidar.events == [("stop", "QGC listener cleanup")]
    assert lidar.revoked is True
    assert status.worker_stopped is True
    assert status.cleanup_completed is True
    assert vars(status) == {
        "worker_stopped": True,
        "cleanup_completed": True,
    }


def test_qgc_lidar_stop_waits_for_inflight_range_then_closes_ingress() -> None:
    lidar = InertRosLidar()
    ingress = QgcRangeIngress()
    adapter = QgcRosLidarAdapter(lidar, range_ingress=ingress)
    entered = threading.Event()
    release = threading.Event()
    accepted = []

    def receive() -> None:
        ingress.accept(lambda: (entered.set(), release.wait(1.0), accepted.append("range"))[-1])

    receiver = threading.Thread(target=receive)
    stopper = threading.Thread(target=adapter.stop)
    receiver.start()
    assert entered.wait(1.0)
    stopper.start()
    stopper.join(0.05)
    assert stopper.is_alive()
    assert lidar.events == []
    release.set()
    receiver.join(1.0)
    stopper.join(1.0)

    assert not receiver.is_alive()
    assert not stopper.is_alive()
    assert accepted == ["range"]
    assert lidar.events == [("stop", "QGC listener cleanup")]
    assert ingress.accept(lambda: accepted.append("late")) is False
    assert accepted == ["range"]


def test_inflight_range_error_is_not_hidden_by_lidar_stop() -> None:
    lidar = InertRosLidar()
    ingress = QgcRangeIngress()
    adapter = QgcRosLidarAdapter(lidar, range_ingress=ingress)

    with pytest.raises(RuntimeError, match="range decode failed"):
        ingress.accept(
            lambda: (_ for _ in ()).throw(RuntimeError("range decode failed"))
        )

    adapter.stop()
    assert lidar.events == [("stop", "QGC listener cleanup")]


@pytest.mark.parametrize(
    "invalid_timeout",
    [
        True,
        False,
        "1",
        float("nan"),
        float("inf"),
        -0.01,
        pytest.param(10**10000, id="huge-integer"),
    ],
)
def test_qgc_lidar_stop_rejects_invalid_timeout_before_revocation(
    invalid_timeout: object,
) -> None:
    lidar = InertRosLidar()
    adapter = QgcRosLidarAdapter(lidar)

    with pytest.raises((TypeError, ValueError)):
        adapter.stop(timeout_seconds=invalid_timeout)

    assert lidar.events == []
    assert lidar.revoked is False


def test_qgc_lidar_stop_repeats_synchronous_revocation() -> None:
    lidar = InertRosLidar()
    adapter = QgcRosLidarAdapter(lidar)

    first = adapter.stop(timeout_seconds=0.1)
    second = adapter.stop(timeout_seconds=0.1)

    assert first.worker_stopped is True
    assert first.cleanup_completed is True
    assert second.worker_stopped is True
    assert second.cleanup_completed is True
    assert lidar.events == [
        ("stop", "QGC listener cleanup"),
        ("stop", "QGC listener cleanup"),
    ]


def test_qgc_lidar_stop_does_not_return_status_after_delegate_failure() -> None:
    class FailingStopLidar(InertRosLidar):
        def stop(self, reason: str) -> None:
            self.events.append(("stop", reason))
            raise RuntimeError("delegate stop failed")

    lidar = FailingStopLidar()
    adapter = QgcRosLidarAdapter(lidar)

    with pytest.raises(RuntimeError, match="delegate stop failed"):
        adapter.stop(timeout_seconds=0.1)
    assert lidar.events == [("stop", "QGC listener cleanup")]
    assert lidar.revoked is False


def test_qgc_lidar_stop_accepts_omitted_timeout() -> None:
    lidar = InertRosLidar()
    adapter = QgcRosLidarAdapter(lidar)

    status = adapter.stop()

    assert status.worker_stopped is True
    assert status.cleanup_completed is True
