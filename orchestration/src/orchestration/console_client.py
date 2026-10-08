"""CLI client for the same local operator boundary used by the browser."""

from __future__ import annotations

import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def request_console(action: str, body: dict[str, Any] | None = None, *, port: int = 8765) -> dict[str, Any]:
    if not 1 <= port <= 65535:
        raise ValueError("console port must be between 1 and 65535")
    base = f"http://127.0.0.1:{port}"
    try:
        if body is None:
            request = Request(f"{base}/api/{action}")
        else:
            with urlopen(f"{base}/api/bootstrap", timeout=5) as response:
                token = json.loads(response.read(65536))["token"]
            request = Request(f"{base}/api/{action}", data=json.dumps(body, allow_nan=False).encode(),
                headers={"Content-Type": "application/json", "X-Console-Token": token, "Origin": base}, method="POST")
        with urlopen(request, timeout=30) as response:
            result = json.loads(response.read(2 * 1024 * 1024))
            if not isinstance(result, dict):
                raise ValueError("invalid console response")
            return result
    except HTTPError as error:
        try:
            detail = json.loads(error.read(65536)).get("error", str(error))
        except ValueError:
            detail = str(error)
        raise RuntimeError(detail) from error
    except URLError as error:
        raise RuntimeError(f"Local console unavailable. Start `uv run drone-sim console --port {port}` in this checkout. {error.reason}") from error
