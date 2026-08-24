import hashlib
import stat
from pathlib import Path
import sys

import pytest


PHASE2 = Path(__file__).resolve().parent
sys.path.insert(0, str(PHASE2))

from module_stub import write_bytes_atomic
from synthetic_gazebo import (
    BoundedPublicationQueue,
    CameraTransportBarrier,
    FRAME_PUBLICATION_ORDER,
    SyntheticGazeboModel,
    apply_durable_lifecycle,
)
from synthetic_scorekeeper import scoring_result


RUN_ID = "11111111-1111-4111-8111-111111111111"


def _capture(delay_ms=0, fault=""):
    events = []
    sleeps = []
    finished = []
    model = SyntheticGazeboModel(
        RUN_ID,
        fault=fault,
        wall_delay_ms=delay_ms,
        publish=lambda kind, value: events.append((kind, value)),
        sleep=lambda seconds: sleeps.append(seconds),
        source_finished=lambda stamp: finished.append(stamp),
    )
    assert model.step() is False
    assert events == []
    model.accept_run_state(RUN_ID, "READY")
    assert model.step() is True
    assert events == [("clock", 0)]
    model.accept_run_state(RUN_ID, "RUNNING")
    while not model.finished and not model.stalled:
        model.step()
        if model.next_frame_id > 0:
            frame_id = model.next_frame_id - 1
            model.accept_pair_ack(
                RUN_ID, frame_id, (frame_id + 1) * 50_000_000, "aggregate"
            )
    return events, sleeps, finished, model


def test_gazebo_model_has_no_pre_ready_output_then_exact_deterministic_timeline():
    events, sleeps, finished, model = _capture()
    clocks = [value for kind, value in events if kind == "clock"]
    onboard = [value for kind, value in events if kind == "onboard"]
    observer = [value for kind, value in events if kind == "observer"]
    ground_truth = [value for kind, value in events if kind == "ground_truth"]
    assert clocks == list(range(0, 2_000_000_001, 50_000_000))
    for stream in (onboard, observer):
        assert [item.frame_id for item in stream] == list(range(40))
        assert [item.sim_timestamp_ns for item in stream] == list(
            range(50_000_000, 2_000_000_001, 50_000_000)
        )
        assert all(item.width == 320 and item.height == 240 for item in stream)
        assert all(item.encoding == "rgb8" and len(item.payload) == 320 * 240 * 3 for item in stream)
    assert [item.stream for item in onboard] == ["onboard"] * 40
    assert [item.stream for item in observer] == ["observer"] * 40
    assert [item.sim_timestamp_ns for item in ground_truth] == [item.sim_timestamp_ns for item in onboard]
    assert finished == [2_000_000_000]
    assert model.finished is True
    assert sleeps == []


def test_wall_delay_changes_only_wall_sleeps_not_simulation_data_or_order():
    immediate, _, immediate_finished, _ = _capture(delay_ms=0)
    delayed, sleeps, delayed_finished, _ = _capture(delay_ms=7)
    assert delayed == immediate
    assert delayed_finished == immediate_finished
    assert sleeps and set(sleeps) == {0.007}


def test_clock_stall_fault_stops_after_frame_four_without_source_finished():
    events, _, finished, model = _capture(fault="clock_stall_after_5")
    assert [value for kind, value in events if kind == "clock"] == list(
        range(0, 250_000_001, 50_000_000)
    )
    assert [item.frame_id for kind, item in events if kind == "onboard"] == list(range(5))
    assert finished == []
    assert model.stalled is True


def test_gazebo_model_waits_for_aggregate_pair_ack_before_next_frame():
    events = []
    finished = []
    model = SyntheticGazeboModel(
        RUN_ID,
        publish=lambda kind, value: events.append((kind, value)),
        source_finished=finished.append,
    )
    model.accept_run_state(RUN_ID, "READY")
    assert model.step() is True
    model.accept_run_state(RUN_ID, "RUNNING")
    assert model.step() is True
    assert model.next_frame_id == 1
    assert model.step() is False
    model.accept_pair_ack(RUN_ID, 0, 50_000_000, "aggregate")
    assert model.step() is True
    assert model.next_frame_id == 2
    assert finished == []


def test_pair_ack_rejects_wrong_fields_and_ignores_stale_or_duplicate_ids():
    model = SyntheticGazeboModel(RUN_ID, publish=lambda *_args: None)
    model.accept_run_state(RUN_ID, "READY")
    model.step()
    model.accept_run_state(RUN_ID, "RUNNING")
    model.step()

    model.accept_pair_ack("22222222-2222-4222-8222-222222222222", 0, 50_000_000, "aggregate")
    assert model.last_pair_ack == -1
    with pytest.raises(ValueError, match="stream"):
        model.accept_pair_ack(RUN_ID, 0, 50_000_000, "onboard")
    with pytest.raises(ValueError, match="timestamp"):
        model.accept_pair_ack(RUN_ID, 0, 1, "aggregate")
    with pytest.raises(ValueError, match="contiguous"):
        model.accept_pair_ack(RUN_ID, 1, 100_000_000, "aggregate")

    model.accept_pair_ack(RUN_ID, 0, 50_000_000, "aggregate")
    model.accept_pair_ack(RUN_ID, 0, 50_000_000, "aggregate")
    assert model.last_pair_ack == 0


def test_finalization_preempts_wait_for_pair_ack_without_deadlock():
    model = SyntheticGazeboModel(RUN_ID, publish=lambda *_args: None)
    model.accept_run_state(RUN_ID, "READY")
    model.step()
    model.accept_run_state(RUN_ID, "RUNNING")
    model.step()
    assert model.last_pair_ack == -1

    model.accept_run_state(RUN_ID, "FINALIZING")

    assert model.step() is False
    assert model.finalizing is True


def test_durable_lifecycle_fallback_is_run_scoped_and_preempts_ack_wait():
    model = SyntheticGazeboModel(RUN_ID, publish=lambda *_args: None)
    apply_durable_lifecycle(
        model,
        {
            "run_id": "22222222-2222-4222-8222-222222222222",
            "state": "RUNNING",
            "sim_timestamp_ns": 0,
        },
        None,
    )
    assert model.running is False

    apply_durable_lifecycle(
        model,
        {"run_id": RUN_ID, "state": "RUNNING", "sim_timestamp_ns": 0},
        None,
    )
    assert model.running is True

    apply_durable_lifecycle(
        model,
        None,
        {"run_id": RUN_ID, "requested_terminal": "ABORTED", "reason": "operator_abort"},
    )
    assert model.finalizing is True


class _Publisher:
    def __init__(self, subscriptions):
        self.subscriptions = subscriptions

    def get_subscription_count(self):
        return self.subscriptions


def test_camera_transport_barrier_rejects_partial_endpoint_discovery():
    publishers = {
        "onboard_image": _Publisher(2),
        "onboard_metadata": _Publisher(2),
        "observer_image": _Publisher(2),
        "observer_metadata": _Publisher(1),
    }
    failures = []
    barrier = CameraTransportBarrier(publishers, deadline=10.0, failure=failures.append)

    assert barrier.poll(now=1.0, finalizing=False) is False
    assert barrier.ready is False
    assert failures == []

    publishers["observer_metadata"].subscriptions = 2
    assert barrier.poll(now=1.1, finalizing=False) is True
    assert barrier.ready is True


def test_camera_transport_barrier_is_preempted_by_finalization_and_deadline():
    publishers = {"onboard_image": _Publisher(0)}
    failures = []
    preempted = CameraTransportBarrier(
        publishers, deadline=10.0, failure=failures.append
    )

    assert preempted.poll(now=1.0, finalizing=True) is False
    assert preempted.preempted is True
    assert failures == []

    expired = CameraTransportBarrier(publishers, deadline=2.0, failure=failures.append)
    assert expired.poll(now=2.0, finalizing=False) is False
    assert expired.failed is True
    assert failures == ["camera transport discovery deadline expired"]
    assert expired.poll(now=3.0, finalizing=False) is False
    assert len(failures) == 1


def test_publication_queue_drains_one_message_per_executor_turn_and_preempts():
    published = []
    assert FRAME_PUBLICATION_ORDER == (
        "clock",
        "onboard_image",
        "onboard_metadata",
        "observer_image",
        "observer_metadata",
        "ground_truth",
    )
    queue = BoundedPublicationQueue(limit=6)
    for label in FRAME_PUBLICATION_ORDER:
        queue.enqueue(label, lambda value=label: published.append(value))

    with pytest.raises(RuntimeError, match="limit"):
        queue.enqueue("overflow", lambda: None)
    drained = [queue.drain_one() for _ in FRAME_PUBLICATION_ORDER]
    assert drained == list(FRAME_PUBLICATION_ORDER)
    assert published == list(FRAME_PUBLICATION_ORDER)
    assert queue.drain_one() is None

    queue.enqueue("late", lambda: published.append("late"))
    queue.preempt()
    assert queue.drain_one() is None
    with pytest.raises(RuntimeError, match="preempted"):
        queue.enqueue("later", lambda: None)


@pytest.mark.parametrize("fault", ["unknown", "observer_after_5"])
def test_gazebo_model_rejects_unknown_faults(fault):
    with pytest.raises(ValueError, match="SIM_PHASE2_FAULT"):
        SyntheticGazeboModel(RUN_ID, fault=fault, publish=lambda *_: None)


def test_synthetic_score_result_is_explicit_fixture_and_checksums_config(tmp_path):
    scoring = tmp_path / "scoring.json"
    scoring.write_bytes(b'{"fixture":true}\n')
    result = scoring_result(RUN_ID, scoring)
    assert result == {
        "run_id": RUN_ID,
        "fixture": True,
        "validity": "synthetic infrastructure evidence only",
        "achieved_score": 0.0,
        "maximum_available_score": 0.0,
        "scoring_checksum": hashlib.sha256(scoring.read_bytes()).hexdigest(),
        "evidence_paths": ["scoring/events.jsonl"],
    }


def test_fixture_output_is_readable_by_non_root_host_controller(tmp_path):
    path = tmp_path / "scoring/result.json"
    write_bytes_atomic(path, b"{}\n")

    assert path.stat().st_mode & stat.S_IROTH
