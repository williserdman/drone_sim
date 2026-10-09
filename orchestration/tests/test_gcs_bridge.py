import socket
import threading
import time

from pymavlink.dialects.v20 import ardupilotmega as mavlink

from orchestration.gcs_bridge import GcsBridge


def test_dedicated_gcs_link_relays_real_frames_both_ways_and_records_manual_command(tmp_path):
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen()
    listener.settimeout(3)
    qgc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    qgc.bind(("127.0.0.1", 0))
    qgc.settimeout(3)
    bridge = GcsBridge(tmp_path, tcp_port=listener.getsockname()[1], qgc_port=qgc.getsockname()[1])
    bridge.start()
    connection, _ = listener.accept()
    connection.settimeout(3)
    def frame(message, *, system, component):
        return message.pack(
            mavlink.MAVLink(None, srcSystem=system, srcComponent=component)
        )

    controller_frames = [
        frame(
            mavlink.MAVLink_heartbeat_message(2, 3, 129, 4, 3, 3),
            system=1,
            component=1,
        ),
        frame(
            mavlink.MAVLink_global_position_int_message(
                1, 0, 0, 0, 12_500, 0, 0, 0, 0
            ),
            system=1,
            component=1,
        ),
        frame(
            mavlink.MAVLink_extended_sys_state_message(0, 2),
            system=1,
            component=1,
        ),
    ]
    connection.sendall(b"".join(controller_frames))
    relayed = [qgc.recvfrom(4096) for _frame in controller_frames]
    assert [payload for payload, _peer in relayed] == controller_frames
    peer = relayed[-1][1]
    deadline = time.monotonic() + 3
    while "landed_state" not in bridge.status() and time.monotonic() < deadline:
        time.sleep(0.01)
    controller_status = bridge.status()
    assert controller_status["mode"] == "GUIDED"
    assert controller_status["armed"] is True
    assert controller_status["relative_altitude_m"] == 12.5
    assert controller_status["landed_state"] == 2
    controller_freshness = controller_status["last_telemetry_wall_time"]

    foreign_frames = []
    for system, component in ((1, 191), (42, 1)):
        foreign_frames.extend(
            [
                frame(
                    mavlink.MAVLink_heartbeat_message(2, 3, 0, 0, 3, 3),
                    system=system,
                    component=component,
                ),
                frame(
                    mavlink.MAVLink_global_position_int_message(
                        2, 0, 0, 0, 999_000, 0, 0, 0, 0
                    ),
                    system=system,
                    component=component,
                ),
                frame(
                    mavlink.MAVLink_extended_sys_state_message(0, 1),
                    system=system,
                    component=component,
                ),
            ]
        )
    time.sleep(0.01)
    connection.sendall(b"".join(foreign_frames))
    foreign_relay = [qgc.recvfrom(4096)[0] for _frame in foreign_frames]
    assert foreign_relay == foreign_frames
    time.sleep(0.01)
    filtered_status = bridge.status()
    assert filtered_status["mode"] == "GUIDED"
    assert filtered_status["armed"] is True
    assert filtered_status["relative_altitude_m"] == 12.5
    assert filtered_status["landed_state"] == 2
    assert filtered_status["last_telemetry_wall_time"] == controller_freshness

    command = mavlink.MAVLink_command_long_message(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0).pack(
        mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
    )
    qgc.sendto(command, peer)
    assert connection.recv(4096) == command
    deadline = time.monotonic() + 3
    while not (tmp_path / "logs/docker/qgc-interventions.jsonl").exists() and time.monotonic() < deadline:
        time.sleep(0.01)
    bridge.stop()
    connection.close()
    listener.close()
    qgc.close()
    log = (tmp_path / "logs/docker/qgc-interventions.jsonl").read_text()
    assert '"command":400' in log
    assert '"source_system":255' in log
    assert bridge.status()["connected"] is False
