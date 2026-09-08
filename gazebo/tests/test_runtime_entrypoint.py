import subprocess

import pytest

from drone_sim_gazebo.runtime.entrypoint import (
    FinalizationDeadlineLatch,
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


def test_competition_transport_requires_every_physical_source_topic():
    """Gazebo-ready must not precede any payload, range, or joint publisher."""
    from drone_sim_gazebo.ros_adapter.topics import gazebo_topics_for_world

    topics = gazebo_topics_for_world("competition_mission")
    outputs = {
        ("gz", "topic", "-l"): "\n".join(topics),
        ("gz", "service", "-l"): (
            "/world/competition_mission/control\n/model/iris/ardupilot/status\n"
        ),
    }

    def run(argv, **_kwargs):
        return type(
            "Result",
            (),
            {"returncode": 0, "stdout": outputs[tuple(argv)], "stderr": ""},
        )()

    GazeboTransport(
        environment={"GZ_PARTITION": "p"},
        world_name="competition_mission",
        run=run,
    ).assert_ready()

    missing = topics[-1]
    outputs[("gz", "topic", "-l")] = "\n".join(topics[:-1])
    with pytest.raises(TransportError, match=missing):
        GazeboTransport(
            environment={"GZ_PARTITION": "p"},
            world_name="competition_mission",
            run=run,
        ).assert_ready()


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
    assert transport.ready_flight_exchange() == transport.flight_exchange_status()


@pytest.mark.parametrize(
    "failure",
    (
        subprocess.TimeoutExpired(("gz", "service"), 2.0),
        type(
            "UnavailableResult",
            (),
            {"returncode": 1, "stdout": "", "stderr": "service unavailable"},
        )(),
    ),
)
def test_flight_exchange_command_unavailability_is_retryable_not_ready(failure):
    """A cold status RPC must not terminate an otherwise healthy startup."""

    def run(_argv, **_kwargs):
        if isinstance(failure, BaseException):
            raise failure
        return failure

    transport = GazeboTransport(
        environment={"GZ_PARTITION": "p"}, world_name="vertical_descent", run=run
    )

    assert transport.ready_flight_exchange() is None


def test_flight_exchange_malformed_status_remains_fatal():
    def run(_argv, **_kwargs):
        return type(
            "Result", (), {"returncode": 0, "stdout": "not status data", "stderr": ""}
        )()

    transport = GazeboTransport(
        environment={"GZ_PARTITION": "p"}, world_name="vertical_descent", run=run
    )

    with pytest.raises(TransportError, match="malformed data"):
        transport.ready_flight_exchange()


def test_flight_exchange_readiness_accepts_stable_bounded_bootstrap_snapshot():
    sample = {
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

    def run(_argv, **_kwargs):
        payload = __import__("json").dumps(sample, separators=(",", ":"))
        escaped = payload.replace('"', '\\"')
        return type("Result", (), {"returncode": 0, "stdout": f'data: "{escaped}"\n', "stderr": ""})()

    transport = GazeboTransport(
        environment={"GZ_PARTITION": "p"}, world_name="vertical_descent", run=run
    )

    assert transport.ready_flight_exchange() == sample
    assert transport.ready_flight_exchange() == sample


def test_flight_exchange_readiness_rejects_json_sends_without_servo_exchange():
    samples = iter(
        (
            (0, 0, 1),
            (0, 0, 2),
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

    def run(_argv, **_kwargs):
        payload = __import__("json").dumps(status, separators=(",", ":"))
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
    transport.run_to_sim_time(90_050_000_000)

    assert "pause: true, multi_step: 1" in calls[0][0]
    assert "pause: false" in calls[1][0]
    assert "run_to_sim_time { sec: 90 nsec: 50000000 }" in calls[2][0]
    assert all(call[1]["shell"] is False for call in calls)
    assert all(
        call[0][call[0].index("--timeout") + 1] == "60000"
        for call in calls
    )
    assert all(call[1]["timeout"] == 62.0 for call in calls)


@pytest.mark.parametrize(
    ("output", "expected"),
    [
        ("sim_time {\n  sec: 44\n  nsec: 125000000\n}\npaused: true\n", 44_125_000_000),
        ("sim_time {\n  sec: 44\n}\npaused: false\n", None),
        ("sim_time {\n  sec: 44\n}\niterations: 44000\n", None),
    ],
)
def test_transport_reads_only_confirmed_paused_integer_world_time(output, expected):
    def run(argv, **_kwargs):
        assert ("-t", "/world/phase3_foundation/stats") == (
            argv[argv.index("-t")],
            argv[argv.index("-t") + 1],
        )
        return type("Result", (), {"returncode": 0, "stdout": output, "stderr": ""})()

    transport = GazeboTransport(environment={"GZ_PARTITION": "p"}, run=run)

    assert transport.paused_sim_time_ns() == expected


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
