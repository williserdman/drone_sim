from pathlib import Path

import pytest

from drone_sim_gazebo.runtime.entrypoint import (
    FinalizationDeadlineLatch,
    GazeboReadyStatus,
    GazeboTransport,
    TransportError,
)


def test_transport_discovers_actual_phase3_topics_and_control_service():
    outputs = {
        ("gz", "topic", "-l"): "\n".join(
            (
                "/clock",
                "/gazebo/private/camera/onboard/image",
                "/gazebo/private/camera/observer/image",
                "/gazebo/private/iris/odometry",
                "/world/phase3_foundation/model/ground_plane/link/ground_link/sensor/iris_ground_contact/contact",
            )
        ),
        ("gz", "service", "-l"): "/world/phase3_foundation/control\n",
    }
    calls = []

    def run(argv, **kwargs):
        calls.append((tuple(argv), kwargs))
        return type("Result", (), {"returncode": 0, "stdout": outputs[tuple(argv)], "stderr": ""})()

    transport = GazeboTransport(environment={"GZ_PARTITION": "p"}, run=run)

    transport.assert_ready()

    assert [call[0] for call in calls] == [("gz", "topic", "-l"), ("gz", "service", "-l")]
    assert all(call[1]["env"] == {"GZ_PARTITION": "p"} for call in calls)


def test_transport_names_missing_actual_endpoint():
    def run(argv, **_kwargs):
        output = "/clock\n" if argv[1] == "topic" else "/world/phase3_foundation/control\n"
        return type("Result", (), {"returncode": 0, "stdout": output, "stderr": ""})()

    with pytest.raises(TransportError, match="camera/onboard"):
        GazeboTransport(environment={"GZ_PARTITION": "p"}, run=run).assert_ready()


def test_transport_applies_step_and_pause_with_shell_free_world_control():
    calls = []

    def run(argv, **kwargs):
        calls.append((tuple(argv), kwargs))
        return type("Result", (), {"returncode": 0, "stdout": "data: true", "stderr": ""})()

    transport = GazeboTransport(environment={"GZ_PARTITION": "p"}, run=run)
    transport.request_steps(1)
    transport.set_paused(False)

    assert "pause: true, multi_step: 1" in calls[0][0]
    assert "pause: false" in calls[1][0]
    assert all(call[1]["shell"] is False for call in calls)


def test_transport_rejects_negative_world_control_reply():
    def run(*_args, **_kwargs):
        return type("Result", (), {"returncode": 0, "stdout": "data: false", "stderr": ""})()

    with pytest.raises(TransportError, match="rejected"):
        GazeboTransport(environment={"GZ_PARTITION": "p"}, run=run).set_paused(True)


def test_finalization_deadline_is_latched_once_and_never_restarted():
    clock = iter((100.0, 999.0))
    latch = FinalizationDeadlineLatch(30, monotonic=lambda: next(clock))

    first = latch.deadline_for({"requested_terminal": "FAILED", "reason": "x"})
    second = latch.deadline_for({"requested_terminal": "FAILED", "reason": "x"})

    assert first == second == 130.0


def test_finalization_control_cannot_change_after_deadline_is_latched():
    latch = FinalizationDeadlineLatch(30, monotonic=lambda: 100.0)
    latch.deadline_for({"requested_terminal": "FAILED", "reason": "x"})

    with pytest.raises(ValueError, match="changed"):
        latch.deadline_for({"requested_terminal": "ABORTED", "reason": "y"})


def test_gazebo_ready_status_is_current_run_idempotent_and_conflict_safe(tmp_path):
    run_directory = tmp_path / "11111111-1111-4111-8111-111111111111"
    (run_directory / ".status").mkdir(parents=True)
    status = GazeboReadyStatus(run_directory, run_directory.name)

    first = status.write_gazebo_ready()
    second = status.write_gazebo_ready()

    assert first == second == run_directory / ".status/gazebo-ready.json"
    assert first.read_text() == '{"ready":true,"run_id":"11111111-1111-4111-8111-111111111111"}\n'


def test_gazebo_ready_status_refuses_conflicting_existing_fact(tmp_path):
    run_directory = tmp_path / "11111111-1111-4111-8111-111111111111"
    status_directory = run_directory / ".status"
    status_directory.mkdir(parents=True)
    (status_directory / "gazebo-ready.json").write_text("{}\n")

    with pytest.raises(RuntimeError, match="conflicts"):
        GazeboReadyStatus(run_directory, run_directory.name).write_gazebo_ready()
