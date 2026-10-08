"""Loopback HTTP boundary over the persistent operator session."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
from typing import Any
from urllib.error import URLError
from urllib.parse import urlsplit
from urllib.request import urlopen
import webbrowser

from .catalog import PresetCatalog


class ConsoleServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], session: Any) -> None:
        if address[0] != "127.0.0.1":
            raise ValueError("console must bind to 127.0.0.1")
        self.session = session
        self.token = secrets.token_urlsafe(32)
        self.preview_slots = threading.BoundedSemaphore(4)
        super().__init__(address, ConsoleHandler)


class ConsoleHandler(BaseHTTPRequestHandler):
    server: ConsoleServer
    protocol_version = "HTTP/1.1"

    def log_message(self, _format: str, *args: Any) -> None:
        pass

    def _respond(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "default-src 'self'; frame-src http://127.0.0.1:6080; style-src 'self'; script-src 'self'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, value: Any, status: int = 200) -> None:
        self._respond(status, json.dumps(value, allow_nan=False).encode(), "application/json; charset=utf-8")

    def _local_host(self) -> bool:
        port = self.server.server_address[1]
        return self.headers.get("Host") in {f"127.0.0.1:{port}", f"localhost:{port}"}

    def do_GET(self) -> None:
        if not self._local_host():
            self._json({"error": "local host required"}, 403)
            return
        path = urlsplit(self.path).path
        session = self.server.session
        try:
            if path == "/api/bootstrap":
                self._json({"token": self.server.token})
            elif path == "/api/session":
                self._json(session.snapshot())
            elif path == "/api/catalog":
                self._json({"presets": [row.to_dict() for row in PresetCatalog(session.project_directory).list()]})
            elif path == "/api/checks":
                self._json(session.checks())
            elif path == "/api/results":
                self._json(session.results())
            elif path == "/api/logs":
                self._json(session.logs())
            elif path == "/camera.mjpg":
                self._camera()
            elif path in {"/", "/console.js", "/console.css"}:
                name = "index.html" if path == "/" else path[1:]
                source = Path(__file__).with_name("console_static") / name
                media_type = {".html": "text/html", ".js": "text/javascript", ".css": "text/css"}[source.suffix]
                self._respond(200, source.read_bytes(), media_type + "; charset=utf-8")
            else:
                self._json({"error": "not found"}, 404)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except (OSError, ValueError, RuntimeError) as error:
            self._json({"error": str(error)}, 409)

    def _camera(self) -> None:
        if not self.server.preview_slots.acquire(blocking=False):
            self._json({"error": "all preview connections are in use"}, 503)
            return
        try:
            try:
                upstream = urlopen("http://127.0.0.1:8081/camera.mjpg", timeout=5)
            except (OSError, URLError) as error:
                self._json({"error": f"camera unavailable: {error}"}, 503)
                return
            with upstream:
                self.send_response(200)
                self.send_header("Content-Type", upstream.headers.get("Content-Type", "multipart/x-mixed-replace; boundary=frame"))
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "close")
                self.end_headers()
                self.close_connection = True
                try:
                    while chunk := upstream.read1(65536):
                        self.wfile.write(chunk)
                        self.wfile.flush()
                except (OSError, TimeoutError):
                    pass
        finally:
            self.server.preview_slots.release()

    def do_POST(self) -> None:
        port = self.server.server_address[1]
        origin = self.headers.get("Origin")
        if (not self._local_host()
            or origin not in {None, f"http://127.0.0.1:{port}", f"http://localhost:{port}"}
            or not secrets.compare_digest(self.headers.get("X-Console-Token", ""), self.server.token)):
            self.close_connection = True
            self._json({"error": "local origin and console token required"}, 403)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = -1
        if length < 0 or length > 16384 or self.headers.get("Transfer-Encoding"):
            self.close_connection = True
            self._json({"error": "request body exceeds 16 KiB"}, 413)
            return
        if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
            self._json({"error": "JSON body required"}, 415)
            self.close_connection = True
            return
        try:
            document = json.loads(self.rfile.read(length))
            if not isinstance(document, dict):
                raise ValueError("body must be an object")
            actions = {"/api/prepare": "prepare", "/api/start-mission": "start_mission",
                       "/api/abort-flight": "abort_flight", "/api/stop": "stop",
                       "/api/open-qgc": "open_qgc", "/api/build": "build"}
            action = actions.get(self.path)
            if action is None:
                self._json({"error": "unknown action"}, 404)
                return
            self._json(getattr(self.server.session, action)(**document))
        except (OSError, ValueError, RuntimeError, TypeError) as error:
            self._json({"error": str(error)}, 409)


def serve_console(project_directory: Path, *, port: int = 8765, open_browser: bool = True) -> None:
    from .operator_session import OperatorSession

    session = OperatorSession(project_directory)
    server = ConsoleServer(("127.0.0.1", port), session)
    url = f"http://127.0.0.1:{server.server_address[1]}"
    print(f"Simulation console: {url}", flush=True)
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Finalizing the active simulation before closing the console…", flush=True)
    finally:
        server.server_close()
        session.shutdown()
