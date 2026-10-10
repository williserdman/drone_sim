import http.client
import json
from pathlib import Path
import threading

from orchestration.console import ConsoleServer


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
