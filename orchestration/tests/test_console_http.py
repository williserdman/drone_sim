import http.client
import json
from pathlib import Path
import threading
from types import SimpleNamespace

from orchestration.console import ConsoleServer
from orchestration.host_integration import LaunchResult


class Session:
    project_directory = Path(__file__).resolve().parents[2]

    def __init__(self):
        self.actions = []

    def snapshot(self):
        return {"state": "PREPARED", "run_id": "existing-run"}

    def prepare(self, **kwargs):
        self.actions.append(kwargs)
        return {"state": "PREPARING"}

    def start_mission(self, **kwargs):
        self.actions.append(kwargs)
        raise RuntimeError("Use the run-bound QGroundControl action")


def test_console_refresh_reads_same_session_and_mutations_require_token_and_local_origin():
    session = Session()
    server = ConsoleServer(("127.0.0.1", 0), session)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    port = server.server_address[1]

    def request(method, path, body=None, headers=None):
        connection = http.client.HTTPConnection("127.0.0.1", port, timeout=2)
        connection.request(method, path, body=body, headers=headers or {})
        result = connection.getresponse()
        payload = result.read()
        connection.close()
        return result.status, payload

    try:
        status, payload = request("GET", "/api/session")
        assert status == 200
        assert json.loads(payload)["run_id"] == "existing-run"
        assert request("GET", "/api/session")[1] == payload
        token = json.loads(request("GET", "/api/bootstrap")[1])["token"]
        body = json.dumps({"preset_id": "configured-descent"})
        headers = {"Content-Type": "application/json", "X-Console-Token": token,
                   "Origin": f"http://127.0.0.1:{port}"}
        assert request("POST", "/api/prepare", body, {"Content-Type": "application/json"})[0] == 403
        assert request("POST", "/api/prepare", body, {**headers, "Origin": "https://example.com"})[0] == 403
        assert request("POST", "/api/prepare", body, headers)[0] == 200
        assert len(session.actions) == 1
        start = json.dumps({"run_id": "existing-run", "request_id": "request"})
        status, payload = request("POST", "/api/start-mission", start, headers)
        assert status == 409
        assert "QGroundControl" in json.loads(payload)["error"]
        assert request("GET", "/../config/default-run.json")[0] == 404
        assert request("POST", "/api/prepare", "{}" * 20000, headers)[0] == 413
    finally:
        server.shutdown()
        server.server_close()
        worker.join()


def test_windows_browser_probe_runs_after_http_is_serving(monkeypatch, tmp_path):
    import orchestration.console as console

    events = []

    class Host:
        facts = SimpleNamespace(kind="wsl2")
        state_directory = tmp_path / "host-state"

        def open_browser(self, url):
            events.append("windows_url_probe")
            return LaunchResult(True, f"opened {url}")

    host = Host()
    workstation = SimpleNamespace(host=host)
    session_holder = {}

    class Session:
        def __init__(self, *_args, **kwargs):
            assert kwargs["host"] is host
            assert kwargs["workstation"] is workstation
            self.project_directory = tmp_path
            self.status = None
            session_holder["value"] = self

        def set_console_status(self, url, launch):
            self.status = (url, launch)

        def shutdown(self):
            pass

    class Server:
        server_address = ("127.0.0.1", 8765)

        def __init__(self, _address, _session):
            self.serving = False

        def serve_forever(self):
            self.serving = True
            events.append("http_ready")
            for _ in range(200):
                if "windows_url_probe" in events:
                    return
                threading.Event().wait(0.005)
            raise AssertionError("browser launch was not scheduled")

        def server_close(self):
            pass

    monkeypatch.setattr(console, "resolve_host", lambda *_args: host)
    monkeypatch.setattr(console, "Workstation", lambda selected: workstation)
    monkeypatch.setattr(console, "LocalRuntime", lambda *_args, **_kwargs: SimpleNamespace(workstation=workstation))
    monkeypatch.setattr(console, "RunController", lambda **_kwargs: SimpleNamespace(workstation=workstation))
    monkeypatch.setattr(console, "OperatorSession", Session)
    monkeypatch.setattr(console, "ConsoleServer", Server)
    monkeypatch.setattr(console, "_console_is_serving", lambda _url: "http_ready" in events)

    console.serve_console(tmp_path, open_browser=True)

    assert events.index("http_ready") < events.index("windows_url_probe")
    assert session_holder["value"].status == (
        "http://127.0.0.1:8765",
        LaunchResult(True, "opened http://127.0.0.1:8765"),
    )


def test_browser_open_failure_preserves_console_and_url(monkeypatch, tmp_path):
    import orchestration.console as console

    statuses = []
    session = SimpleNamespace(
        project_directory=tmp_path,
        set_console_status=lambda url, launch: statuses.append((url, launch)),
        shutdown=lambda: None,
    )
    host = SimpleNamespace(
        facts=SimpleNamespace(kind="macos"),
        state_directory=tmp_path / "state",
        open_browser=lambda _url: (_ for _ in ()).throw(RuntimeError("browser unavailable")),
    )
    server_is_serving = {"value": False}

    class Server:
        server_address = ("127.0.0.1", 8765)
        def __init__(self, _address, _session): pass
        def serve_forever(self):
            server_is_serving["value"] = True
            for _ in range(200):
                if statuses:
                    return
                threading.Event().wait(0.005)
        def server_close(self): pass

    monkeypatch.setattr(console, "resolve_host", lambda *_args: host)
    monkeypatch.setattr(console, "Workstation", lambda _host: SimpleNamespace(host=host))
    monkeypatch.setattr(console, "LocalRuntime", lambda *_args, **_kwargs: SimpleNamespace())
    monkeypatch.setattr(console, "RunController", lambda **_kwargs: SimpleNamespace())
    monkeypatch.setattr(console, "OperatorSession", lambda *_args, **_kwargs: session)
    monkeypatch.setattr(console, "ConsoleServer", Server)
    monkeypatch.setattr(console, "_console_is_serving", lambda _url: server_is_serving["value"])

    console.serve_console(tmp_path)

    assert server_is_serving["value"] is True
    assert statuses[-1][0] == "http://127.0.0.1:8765"
    assert statuses[-1][1].opened is False
