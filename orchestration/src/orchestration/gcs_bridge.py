"""Dedicated SITL GCS transport to native QGC, preserving MAVLink frames."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import select
import socket
import threading
import time
from typing import Any

from pymavlink.dialects.v20 import ardupilotmega as mavlink
from pymavlink import mavutil

from .host_integration import DatagramTransport, HostTransportError


_INTERVENTION_TYPES = {
    "SET_MODE", "RC_CHANNELS_OVERRIDE", "MANUAL_CONTROL", "PARAM_SET",
    "PARAM_EXT_SET", "MISSION_COUNT", "MISSION_ITEM", "MISSION_ITEM_INT",
    "MISSION_SET_CURRENT", "MISSION_CLEAR_ALL", "SET_POSITION_TARGET_LOCAL_NED",
    "SET_POSITION_TARGET_GLOBAL_INT", "SET_ATTITUDE_TARGET",
}
# QGC uses these commands to request telemetry, not to change flight intent.
_OBSERVATION_COMMANDS = {510, 511, 512, 519, 520, 521, 522}
_FLIGHT_CONTROLLER_SOURCE = (1, 1)


class GcsBridge:
    def __init__(
        self,
        run_directory: Path,
        *,
        transport: DatagramTransport,
        tcp_port: int = 5762,
    ) -> None:
        self.run_directory = run_directory
        self.transport = transport
        self.tcp_port = tcp_port
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._transport_lock = threading.Lock()
        self._transport_closed = False
        self._thread: threading.Thread | None = None
        self._state: dict[str, Any] = {
            "connected": False,
            "detail": "waiting for SITL",
            "qgc_connected": True,
            "qgc_detail": "QGC datagram transport open",
            "last_telemetry_wall_time": None,
            "last_qgc_wall_time": None,
        }

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("GCS bridge already started")
        self._thread = threading.Thread(target=self._run, name="native-qgc-link", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        close_error = self._close_transport()
        if self._thread is not None:
            self._thread.join(timeout=4)
            if self._thread.is_alive():
                raise RuntimeError("GCS bridge worker did not stop")
        self._update(
            connected=False,
            detail="GCS link closed",
            qgc_connected=False,
            qgc_detail="QGC datagram transport closed",
        )
        if close_error is not None:
            raise close_error

    def _close_transport(self) -> HostTransportError | None:
        with self._transport_lock:
            if self._transport_closed:
                return None
            self._transport_closed = True
        try:
            self.transport.close()
        except HostTransportError as error:
            self._update(qgc_connected=False, qgc_detail=str(error))
            return error
        return None

    def status(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._state)
        stamp = result["last_telemetry_wall_time"]
        result["fresh"] = stamp is not None and time.time() - stamp < 5 and result["connected"]
        qgc_stamp = result["last_qgc_wall_time"]
        result["qgc_fresh"] = (
            qgc_stamp is not None
            and time.time() - qgc_stamp < 10
            and result["fresh"]
            and result["qgc_connected"]
        )
        return result

    def _update(self, **values: Any) -> None:
        with self._lock:
            self._state.update(values)

    def _record_intervention(self, message: Any) -> None:
        kind = message.get_type()
        command = getattr(message, "command", None)
        if kind not in _INTERVENTION_TYPES and not (
            kind in {"COMMAND_LONG", "COMMAND_INT"} and command not in _OBSERVATION_COMMANDS
        ):
            return
        path = self.run_directory / "logs/docker"
        path.mkdir(parents=True, exist_ok=True)
        fields = message.to_dict()
        # MAVLink uses NaN for unspecified command parameters; retain that meaning
        # without emitting nonstandard JSON into the evidence bundle.
        fields = {key: (str(value) if isinstance(value, float) and not math.isfinite(value) else value)
                  for key, value in fields.items()}
        event = {"event": "qgc_manual_command", "wall_time_ns": time.time_ns(),
                 "message_type": kind, "command": command,
                 "source_system": message.get_srcSystem(), "source_component": message.get_srcComponent(),
                 "fields": fields}
        descriptor = os.open(path / "qgc-interventions.jsonl", os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        try:
            payload = (json.dumps(event, allow_nan=False, sort_keys=True, separators=(",", ":")) + "\n").encode()
            while payload:
                count = os.write(descriptor, payload)
                if count <= 0:
                    raise OSError("intervention log append made no progress")
                payload = payload[count:]
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def _observe(self, message: Any) -> None:
        source = (message.get_srcSystem(), message.get_srcComponent())
        if source != _FLIGHT_CONTROLLER_SOURCE:
            return
        self._update(last_telemetry_wall_time=time.time())
        if message.get_type() == "HEARTBEAT":
            self._update(armed=bool(message.base_mode & 128), mode=mavutil.mode_string_v10(message))
        elif message.get_type() == "GLOBAL_POSITION_INT":
            self._update(relative_altitude_m=message.relative_alt / 1000)
        elif message.get_type() == "EXTENDED_SYS_STATE":
            self._update(landed_state=message.landed_state)

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                try:
                    with socket.create_connection(("127.0.0.1", self.tcp_port), timeout=2) as tcp:
                        tcp.settimeout(1)
                        self._update(connected=True, detail="connected to SITL GCS channel")
                        self._relay(tcp)
                except (OSError, ValueError) as error:
                    self._update(connected=False, detail=str(error))
                    self._stop.wait(1)
        finally:
            close_error = self._close_transport()
            self._update(
                connected=False,
                detail="GCS link closed",
                qgc_connected=False,
                qgc_detail=(
                    str(close_error)
                    if close_error is not None
                    else "QGC datagram transport closed"
                ),
            )

    def _relay(self, tcp: socket.socket) -> None:
        incoming = mavlink.MAVLink(None)
        outgoing = mavlink.MAVLink(None)
        incoming.robust_parsing = outgoing.robust_parsing = True
        transport_active = True
        while not self._stop.is_set():
            readable, _, _ = select.select([tcp], [], [], 0.05)
            if tcp in readable:
                data = tcp.recv(65536)
                if not data:
                    raise ConnectionError("SITL GCS channel disconnected")
                for message in incoming.parse_buffer(data) or []:
                    if message.get_type() != "BAD_DATA":
                        self._observe(message)
                        if transport_active:
                            try:
                                self.transport.send(bytes(message.get_msgbuf()))
                            except HostTransportError as error:
                                transport_active = False
                                self._update(
                                    qgc_connected=False,
                                    qgc_detail=str(error),
                                )
            if not transport_active:
                continue
            try:
                datagram = self.transport.receive(0.05)
            except HostTransportError as error:
                transport_active = False
                self._update(qgc_connected=False, qgc_detail=str(error))
                continue
            if datagram is None:
                continue
            for message in outgoing.parse_buffer(datagram) or []:
                if message.get_type() == "BAD_DATA":
                    continue
                self._update(last_qgc_wall_time=time.time())
                if self._finalizing():
                    continue
                self._record_intervention(message)
                # Finalization can begin while persistence is in progress. The
                # command boundary must therefore be checked again at the last
                # possible point before every TCP write.
                if self._finalizing():
                    continue
                tcp.sendall(bytes(message.get_msgbuf()))

    def _finalizing(self) -> bool:
        return self._stop.is_set() or (
            self.run_directory / ".control/finalize-request.json"
        ).exists()
