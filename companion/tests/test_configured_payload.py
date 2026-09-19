from __future__ import annotations

from types import SimpleNamespace

import pytest

from drone_sim_companion.configured_payload import ConfiguredPayloadClient


RUN_ID = "00000000-0000-4000-8000-000000000701"


class Request:
    ATTACH = 1
    RELEASE = 2

    def __init__(self) -> None:
        self.run_id = ""
        self.aruco_id = 0
        self.action = 0
        self.command_id = ""


class Future:
    def __init__(self) -> None:
        self._done = False
        self._result = None
        self.cancelled = False

    def done(self) -> bool:
        return self._done

    def result(self):
        return self._result

    def complete(self, response) -> None:
        self._result = response
        self._done = True

    def cancel(self) -> None:
        self.cancelled = True


class ServiceClient:
    def __init__(self, *, ready: bool = True) -> None:
        self.ready = ready
        self.requests: list[Request] = []
        self.futures: list[Future] = []

    def service_is_ready(self) -> bool:
        return self.ready

    def call_async(self, request: Request) -> Future:
        future = Future()
        self.requests.append(request)
        self.futures.append(future)
        return future


def payload_state(
    aruco_id: int,
    timestamp_ns: int,
    *,
    attached: bool,
    grounded: bool,
    run_id: str = RUN_ID,
):
    sec, nanosec = divmod(timestamp_ns, 1_000_000_000)
    return SimpleNamespace(
        run_id=run_id,
        aruco_id=aruco_id,
        sim_timestamp=SimpleNamespace(sec=sec, nanosec=nanosec),
        attached=attached,
        grounded=grounded,
    )


def response(
    command_id: str,
    *,
    accepted: object = True,
    code: object = "OK",
    sequence: object = 1,
):
    return SimpleNamespace(
        accepted=accepted,
        code=code,
        detail="test response",
        command_id=command_id,
        response_sequence=sequence,
    )


def make_adapter(*, ready: bool = True):
    service = ServiceClient(ready=ready)
    return ConfiguredPayloadClient(RUN_ID, service, Request), service


def test_attach_waits_for_correlated_reply_and_newer_physical_state() -> None:
    adapter, service = make_adapter()
    adapter.observe(payload_state(3, 100, attached=False, grounded=True))

    adapter.start(3, "ATTACH", "7", 100)

    assert adapter.ready is True
    assert len(service.requests) == 1
    request = service.requests[0]
    assert (request.run_id, request.aruco_id, request.action) == (
        RUN_ID,
        3,
        Request.ATTACH,
    )
    assert request.command_id == f"{RUN_ID}:configured:7"
    service.futures[0].complete(response(request.command_id))
    assert adapter.tick(150) is False

    adapter.observe(payload_state(3, 150, attached=True, grounded=True))

    assert adapter.tick(150) is True
    assert adapter.attached_id == 3


def test_release_ignores_predispatch_and_other_payload_state() -> None:
    adapter, service = make_adapter()
    adapter.observe(payload_state(3, 100, attached=True, grounded=False))
    adapter.start(3, "RELEASE", "8", 100)
    command_id = service.requests[0].command_id
    service.futures[0].complete(response(command_id))

    adapter.observe(payload_state(4, 150, attached=False, grounded=True))
    assert adapter.tick(150) is False
    assert adapter.attached_id == 3

    adapter.observe(payload_state(3, 200, attached=False, grounded=False))

    assert adapter.tick(200) is True
    assert adapter.attached_id is None


def test_rejected_response_fails_once_and_keeps_confirmed_attachment() -> None:
    adapter, service = make_adapter()
    adapter.observe(payload_state(2, 100, attached=True, grounded=False))
    adapter.start(2, "RELEASE", "9", 100)
    request = service.requests[0]
    service.futures[0].complete(
        response(request.command_id, accepted=False, code="NOT_ATTACHED")
    )

    with pytest.raises(RuntimeError, match="NOT_ATTACHED"):
        adapter.tick(150)

    assert len(service.requests) == 1
    assert adapter.attached_id == 2


@pytest.mark.parametrize(
    "reply",
    [
        response("wrong-command"),
        response(f"{RUN_ID}:configured:10", accepted=1),
        response(f"{RUN_ID}:configured:10", code="NOT_OK"),
        response(f"{RUN_ID}:configured:10", sequence=0),
        response(f"{RUN_ID}:configured:10", sequence=True),
    ],
)
def test_malformed_or_mismatched_response_fails(reply) -> None:
    adapter, service = make_adapter()
    adapter.start(4, "RELEASE", "10", 100)
    service.futures[0].complete(reply)

    with pytest.raises(RuntimeError, match="payload response"):
        adapter.tick(150)


def test_wrong_physical_state_fails_when_public_truth_becomes_stale() -> None:
    adapter, service = make_adapter()
    adapter.start(3, "RELEASE", "11", 100)
    request = service.requests[0]
    service.futures[0].complete(response(request.command_id))
    adapter.observe(payload_state(3, 200, attached=True, grounded=False))

    assert adapter.tick(500_000_200) is False
    with pytest.raises(RuntimeError, match="stale"):
        adapter.tick(500_000_201)


def test_competing_action_and_attach_two_are_rejected() -> None:
    adapter, _service = make_adapter()

    with pytest.raises(ValueError, match="payload 2"):
        adapter.start(2, "ATTACH", "12", 100)

    adapter.start(3, "ATTACH", "13", 100)
    with pytest.raises(RuntimeError, match="already pending"):
        adapter.start(4, "RELEASE", "14", 100)


def test_cancel_cancels_future_without_changing_confirmed_state() -> None:
    adapter, service = make_adapter()
    adapter.observe(payload_state(3, 100, attached=True, grounded=False))
    adapter.start(3, "RELEASE", "15", 100)

    adapter.cancel()

    assert service.futures[0].cancelled is True
    assert adapter.attached_id == 3
    adapter.start(3, "RELEASE", "16", 200)
    assert len(service.requests) == 2


def test_observe_rejects_wrong_run_invalid_fields_and_nonmonotonic_time() -> None:
    adapter, _service = make_adapter()

    with pytest.raises(RuntimeError, match="run_id"):
        adapter.observe(
            payload_state(
                3,
                100,
                attached=False,
                grounded=True,
                run_id="00000000-0000-4000-8000-000000000702",
            )
        )
    with pytest.raises(RuntimeError, match="marker"):
        adapter.observe(payload_state(5, 100, attached=False, grounded=True))
    with pytest.raises(RuntimeError, match="boolean"):
        adapter.observe(payload_state(3, 100, attached=0, grounded=True))

    adapter.observe(payload_state(3, 100, attached=False, grounded=True))
    with pytest.raises(RuntimeError, match="increase"):
        adapter.observe(payload_state(3, 100, attached=False, grounded=True))
