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
    encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    heartbeat = mavlink.MAVLink_heartbeat_message(2, 3, 0, 4, 3, 3).pack(encoder)
    connection.sendall(heartbeat)
    received, peer = qgc.recvfrom(4096)
    assert received == heartbeat
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
