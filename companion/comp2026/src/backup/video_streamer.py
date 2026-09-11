"""Opt-in development camera streamer with lazy hardware dependencies."""

from __future__ import annotations

import argparse
import time
from collections.abc import Callable, Iterator
from typing import Any


AUTO_CONTROLS = {
    "auto_exposure": "1",
    "white_balance_automatic": "0",
    "focus_automatic_continuous": "0",
    "exposure_dynamic_framerate": "0",
    "power_line_frequency": "0",
}

MANUAL_CONTROLS = {
    "white_balance_temperature": "4000",
    "focus_absolute": "0",
    "sharpness": "128",
    "contrast": "32",
    "saturation": "128",
    "brightness": "0",
    "zoom_absolute": "100",
    "backlight_compensation": "0",
    "gain": "0",
}


def setup_camera(
    *,
    device: str,
    command_runner: Callable[..., Any] | None = None,
    settle: Callable[[float], None] = time.sleep,
) -> None:
    """Apply settings only when explicitly requested for one named device."""
    if not device:
        raise ValueError("device must be explicit and non-empty")
    if command_runner is None:
        import subprocess

        command_runner = subprocess.run
        stderr_target = subprocess.DEVNULL
    else:
        stderr_target = None

    for control, value in AUTO_CONTROLS.items():
        command_runner(
            ["v4l2-ctl", "-d", device, f"--set-ctrl={control}={value}"],
            check=True,
            stderr=stderr_target,
        )
    settle(0.2)
    command_runner(
        ["v4l2-ctl", "-d", device, "-p", "30"],
        check=True,
        stderr=stderr_target,
    )
    for control, value in MANUAL_CONTROLS.items():
        command_runner(
            ["v4l2-ctl", "-d", device, f"--set-ctrl={control}={value}"],
            check=True,
            stderr=stderr_target,
        )


def generate_frames(
    camera: Any, *, encoder: Callable[..., Any] | None = None
) -> Iterator[bytes]:
    if encoder is None:
        import cv2

        encoder = cv2.imencode
    while True:
        success, frame = camera.read()
        if not success:
            return
        encoded, buffer = encoder(".jpg", frame)
        if not encoded:
            continue
        yield (
            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
            + buffer.tobytes()
            + b"\r\n"
        )


def create_app(camera: Any, *, flask_module: Any = None, encoder=None):
    if flask_module is None:
        import flask as flask_module

    app = flask_module.Flask(__name__)

    @app.route("/video")
    def video_feed():
        return flask_module.Response(
            generate_frames(camera, encoder=encoder),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    return app


def run_server(
    *,
    device: str,
    host: str,
    port: int,
    apply_settings: bool = False,
    camera_factory: Callable[[str], Any] | None = None,
    app_factory: Callable[[Any], Any] | None = None,
) -> None:
    """Open the requested camera only for the lifetime of the explicit runner."""
    if apply_settings:
        setup_camera(device=device)
    if camera_factory is None:
        import cv2

        camera_factory = cv2.VideoCapture
    camera = camera_factory(device)
    try:
        app = app_factory(camera) if app_factory is not None else create_app(camera)
        app.run(host=host, port=port)
    finally:
        camera.release()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=4321)
    parser.add_argument("--apply-settings", action="store_true")
    args = parser.parse_args(argv)
    run_server(
        device=args.device,
        host=args.host,
        port=args.port,
        apply_settings=args.apply_settings,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
