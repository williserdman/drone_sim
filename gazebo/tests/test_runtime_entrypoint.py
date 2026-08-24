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


def test_flight_transport_discovers_and_controls_the_flight_world():
    """Using the passive world endpoint would falsely report the flight server ready."""
    contact = (
        "/world/vertical_descent/model/ground_plane/link/ground_link/sensor/"
        "iris_ground_contact/contact"
    )
    outputs = {
        ("gz", "topic", "-l"): "\n".join(
            (
                "/clock",
                "/gazebo/private/camera/onboard/image",
                "/gazebo/private/camera/observer/image",
                "/gazebo/private/iris/odometry",
                contact,
            )
        ),
        ("gz", "service", "-l"): (
            "/world/vertical_descent/control\n/model/iris/ardupilot/status\n"
        ),
    }
    calls = []

    def run(argv, **kwargs):
        calls.append((tuple(argv), kwargs))
        return type(
            "Result",
            (),
            {"returncode": 0, "stdout": outputs.get(tuple(argv), "data: true"), "stderr": ""},
        )()

    transport = GazeboTransport(
        environment={"GZ_PARTITION": "p"}, world_name="vertical_descent", run=run
    )
    transport.assert_ready()
    transport.request_steps(1)

    assert "/world/vertical_descent/control" in calls[-1][0]


def test_flight_transport_rejects_a_world_without_the_plugin_status_service():
    """A passive world must not satisfy the flight runtime's local readiness."""
    contact = (
        "/world/vertical_descent/model/ground_plane/link/ground_link/sensor/"
        "iris_ground_contact/contact"
    )

    def run(argv, **_kwargs):
        output = (
            "\n".join(
                (
                    "/clock",
                    "/gazebo/private/camera/onboard/image",
                    "/gazebo/private/camera/observer/image",
                    "/gazebo/private/iris/odometry",
                    contact,
                )
            )
            if argv[1] == "topic"
            else "/world/vertical_descent/control\n"
        )
        return type("Result", (), {"returncode": 0, "stdout": output, "stderr": ""})()

    transport = GazeboTransport(
        environment={"GZ_PARTITION": "p"}, world_name="vertical_descent", run=run
    )

    with pytest.raises(TransportError, match="ardupilot/status"):
        transport.assert_ready()


def test_flight_exchange_status_requires_real_bidirectional_zero_gap_counts():
    """Generic Gazebo endpoints cannot substitute for one real JSON exchange."""
    payload = (
        '{"online":true,"servo_packets_received":3,"motor_updates":2,'
        '"duplicate_servo_packets":1,"servo_frame_gaps":0,'
        '"json_states_sent":3,"json_send_errors":0,'
        '"last_servo_frame":2,"last_json_sim_time_ns":0}'
    )

    def run(_argv, **_kwargs):
        escaped = payload.replace('"', '\\"')
        return type(
            "Result",
            (),
            {"returncode": 0, "stdout": f'data: "{escaped}"\n', "stderr": ""},
        )()

    transport = GazeboTransport(
        environment={"GZ_PARTITION": "p"}, world_name="vertical_descent", run=run
    )

    assert transport.flight_exchange_status() == {
        "online": True,
        "servo_packets_received": 3,
        "motor_updates": 2,
        "duplicate_servo_packets": 1,
        "servo_frame_gaps": 0,
        "json_states_sent": 3,
        "json_send_errors": 0,
        "last_servo_frame": 2,
        "last_json_sim_time_ns": 0,
    }
    assert transport.ready_flight_exchange() is None


def test_flight_exchange_readiness_requires_progress_across_distinct_polls():
    samples = iter(
        (
            {
                "online": True,
                "servo_packets_received": 1,
                "motor_updates": 1,
                "duplicate_servo_packets": 0,
                "servo_frame_gaps": 0,
                "json_states_sent": 1,
                "json_send_errors": 0,
                "last_servo_frame": 0,
                "last_json_sim_time_ns": 0,
            },
            {
                "online": True,
                "servo_packets_received": 2,
                "motor_updates": 2,
                "duplicate_servo_packets": 0,
                "servo_frame_gaps": 0,
                "json_states_sent": 2,
                "json_send_errors": 0,
                "last_servo_frame": 1,
                "last_json_sim_time_ns": 0,
            },
        )
    )

    def run(_argv, **_kwargs):
        payload = __import__("json").dumps(next(samples), separators=(",", ":"))
        escaped = payload.replace('"', '\\"')
        return type("Result", (), {"returncode": 0, "stdout": f'data: "{escaped}"\n', "stderr": ""})()

    transport = GazeboTransport(
        environment={"GZ_PARTITION": "p"}, world_name="vertical_descent", run=run
    )

    assert transport.ready_flight_exchange() is None
    assert transport.ready_flight_exchange()["last_servo_frame"] == 1


def test_flight_exchange_readiness_rejects_healthy_history_after_peer_stops():
    samples = iter(
        (
            (1, 1, 1),
            (1, 1, 2),
        )
    )

    def run(_argv, **_kwargs):
        servo, motor, sent = next(samples)
        payload = (
            '{"online":true,"servo_packets_received":%d,"motor_updates":%d,'
            '"duplicate_servo_packets":0,"servo_frame_gaps":0,'
            '"json_states_sent":%d,"json_send_errors":0,'
            '"last_servo_frame":0,"last_json_sim_time_ns":0}'
        ) % (servo, motor, sent)
        escaped = payload.replace('"', '\\"')
        return type("Result", (), {"returncode": 0, "stdout": f'data: "{escaped}"\n', "stderr": ""})()

    transport = GazeboTransport(
        environment={"GZ_PARTITION": "p"}, world_name="vertical_descent", run=run
    )

    assert transport.ready_flight_exchange() is None
    assert transport.ready_flight_exchange() is None


@pytest.mark.parametrize(
    "changed",
    (
        {"online": False},
        {"servo_packets_received": 0},
        {"motor_updates": 0},
        {"json_states_sent": 0},
        {"servo_frame_gaps": 1},
        {"json_send_errors": 1},
    ),
)
def test_flight_exchange_readiness_fails_closed_on_incomplete_or_gapped_exchange(changed):
    """Every required exchange fact must be healthy before Gazebo flight-ready."""
    status = {
        "online": True,
        "servo_packets_received": 1,
        "motor_updates": 1,
        "duplicate_servo_packets": 0,
        "servo_frame_gaps": 0,
        "json_states_sent": 1,
        "json_send_errors": 0,
        "last_servo_frame": 0,
        "last_json_sim_time_ns": 0,
    }
    status.update(changed)

    polls = 0

    def run(_argv, **_kwargs):
        nonlocal polls
        polls += 1
        sample = dict(status)
        if polls > 1:
            for key in ("servo_packets_received", "motor_updates", "json_states_sent"):
                if key not in changed:
                    sample[key] += 1
        payload = __import__("json").dumps(sample, separators=(",", ":"))
        escaped = payload.replace('"', '\\"')
        return type(
            "Result",
            (),
            {"returncode": 0, "stdout": f'data: "{escaped}"\n', "stderr": ""},
        )()

    transport = GazeboTransport(
        environment={"GZ_PARTITION": "p"}, world_name="vertical_descent", run=run
    )

    assert transport.flight_exchange_ready() is False
    assert transport.flight_exchange_ready() is False


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


def test_gazebo_ready_fact_preserves_the_latched_flight_exchange_counts(tmp_path):
    """The current run must retain the counters that justified flight readiness."""
    run_directory = tmp_path / "11111111-1111-4111-8111-111111111111"
    (run_directory / ".status").mkdir(parents=True)
    status = GazeboReadyStatus(run_directory, run_directory.name)
    status.record_flight_exchange(
        {
            "online": True,
            "servo_packets_received": 1,
            "motor_updates": 1,
            "duplicate_servo_packets": 0,
            "servo_frame_gaps": 0,
            "json_states_sent": 1,
            "json_send_errors": 0,
            "last_servo_frame": 0,
            "last_json_sim_time_ns": 0,
        }
    )

    target = status.write_gazebo_ready()

    assert target.read_text() == (
        '{"flight_exchange":{"duplicate_servo_packets":0,'
        '"json_send_errors":0,"json_states_sent":1,'
        '"last_json_sim_time_ns":0,"last_servo_frame":0,'
        '"motor_updates":1,"online":true,"servo_frame_gaps":0,'
        '"servo_packets_received":1},"ready":true,'
        '"run_id":"11111111-1111-4111-8111-111111111111"}\n'
    )
