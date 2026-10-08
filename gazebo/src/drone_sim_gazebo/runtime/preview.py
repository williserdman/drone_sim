"""Bounded latest-frame HTTP preview for private Gazebo camera images."""

from __future__ import annotations

from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
import time
from typing import Callable


@dataclass(frozen=True)
class PreviewFrame:
    sequence: int
    native_timestamp_ns: int
    received_wall_time_ns: int
    width: int
    height: int
    source: str
    data: bytes


class LatestFramePreview:
    """Retain one immutable RGB frame without backpressuring its producer."""

    def __init__(
        self,
        *,
        freshness_ns: int = 2_000_000_000,
        wall_time_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        if type(freshness_ns) is not int or freshness_ns <= 0:
            raise ValueError("freshness_ns must be a positive integer")
        self._freshness_ns = freshness_ns
        self._wall_time_ns = wall_time_ns
        self._condition = threading.Condition()
        self._frame: PreviewFrame | None = None
        self._sequence = 0

    def update(
        self,
        *,
        native_timestamp_ns: int,
        width: int,
        height: int,
        encoding: str,
        data: bytes,
        source: str,
    ) -> PreviewFrame:
        if type(native_timestamp_ns) is not int or native_timestamp_ns < 0:
            raise ValueError("native_timestamp_ns must be a nonnegative integer")
        if type(width) is not int or width <= 0 or type(height) is not int or height <= 0:
            raise ValueError("preview dimensions must be positive integers")
        if encoding != "rgb8":
            raise ValueError("preview encoding must be rgb8")
        if type(data) is not bytes or len(data) != width * height * 3:
            raise ValueError("preview data must contain one immutable RGB frame")
        if not isinstance(source, str) or not source:
            raise ValueError("preview source must be nonempty")
        received = self._wall_time_ns()
        if type(received) is not int or received < 0:
            raise ValueError("wall clock must return nonnegative integer nanoseconds")
        with self._condition:
            self._sequence += 1
            frame = PreviewFrame(
                self._sequence,
                native_timestamp_ns,
                received,
                width,
                height,
                source,
                data,
            )
            self._frame = frame
            self._condition.notify_all()
            return frame

    def wait_for_frame(
        self, *, after_sequence: int, timeout: float | None
    ) -> PreviewFrame | None:
        if type(after_sequence) is not int or after_sequence < 0:
            raise ValueError("after_sequence must be a nonnegative integer")
        if timeout is not None and (
            isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or timeout < 0
        ):
            raise ValueError("timeout must be nonnegative or null")
        deadline = None if timeout is None else time.monotonic() + float(timeout)
        with self._condition:
            while self._frame is None or self._frame.sequence <= after_sequence:
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return self._frame

    def status(self) -> dict[str, object]:
        with self._condition:
            frame = self._frame
        if frame is None:
            return {
                "fresh": False,
                "native_timestamp_ns": None,
                "received_wall_time_ns": None,
                "sequence": 0,
                "width": None,
                "height": None,
                "source": None,
            }
        age = self._wall_time_ns() - frame.received_wall_time_ns
        return {
            "fresh": 0 <= age <= self._freshness_ns,
            "native_timestamp_ns": frame.native_timestamp_ns,
            "received_wall_time_ns": frame.received_wall_time_ns,
            "sequence": frame.sequence,
            "width": frame.width,
            "height": frame.height,
            "source": frame.source,
        }


def _encode_jpeg(frame: PreviewFrame) -> bytes:
    import cv2
    import numpy as np

    rgb = np.frombuffer(frame.data, dtype=np.uint8).reshape(
        (frame.height, frame.width, 3)
    )
    ok, encoded = cv2.imencode(
        ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85]
    )
    if not ok:
        raise RuntimeError("could not encode private camera preview")
    return bytes(encoded)


class PreviewServer:
    """Serve preview status and an MJPEG stream from a latest-frame buffer."""

    def __init__(
        self,
        preview: LatestFramePreview,
        *,
        host: str = "0.0.0.0",
        port: int = 8081,
        encode_jpeg: Callable[[PreviewFrame], bytes] = _encode_jpeg,
    ) -> None:
        if not isinstance(preview, LatestFramePreview):
            raise TypeError("preview must be a LatestFramePreview")
        if not isinstance(host, str) or not host:
            raise ValueError("host must be nonempty")
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError("port must be between 0 and 65535")
        self._preview = preview
        self._encode_jpeg = encode_jpeg
        owner = self

        class Server(ThreadingHTTPServer):
            daemon_threads = True

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def do_GET(self) -> None:
                if self.path == "/status":
                    payload = json.dumps(
                        owner._preview.status(),
                        sort_keys=True,
                        separators=(",", ":"),
                    ).encode("utf-8")
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(payload)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                if self.path != "/camera.mjpg":
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                self.send_response(HTTPStatus.OK)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                sequence = 0
                try:
                    while True:
                        frame = owner._preview.wait_for_frame(
                            after_sequence=sequence, timeout=5.0
                        )
                        if frame is None:
                            continue
                        jpeg = owner._encode_jpeg(frame)
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(jpeg)).encode("ascii")
                            + b"\r\n\r\n"
                            + jpeg
                            + b"\r\n"
                        )
                        self.wfile.flush()
                        sequence = frame.sequence
                except (BrokenPipeError, ConnectionResetError):
                    return

            def log_message(self, _format: str, *_args: object) -> None:
                return

        self._server = Server((host, port), Handler)
        self._thread: threading.Thread | None = None

    @property
    def port(self) -> int:
        return int(self._server.server_address[1])

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="gazebo-private-preview",
            daemon=True,
        )
        self._thread.start()

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None


__all__ = ["LatestFramePreview", "PreviewFrame", "PreviewServer"]
