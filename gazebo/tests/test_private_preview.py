import json
import urllib.request

from drone_sim_gazebo.runtime.preview import LatestFramePreview, PreviewServer


def test_preview_keeps_only_latest_frame_and_reports_wall_freshness():
    wall = [1_000_000_000]
    preview = LatestFramePreview(freshness_ns=2_000_000_000, wall_time_ns=lambda: wall[0])

    preview.update(
        native_timestamp_ns=90_050_000_000,
        width=2,
        height=1,
        encoding="rgb8",
        data=b"first!",
        source="onboard",
    )
    wall[0] += 1
    preview.update(
        native_timestamp_ns=90_100_000_000,
        width=2,
        height=1,
        encoding="rgb8",
        data=b"second",
        source="onboard",
    )

    frame = preview.wait_for_frame(after_sequence=0, timeout=0)
    assert frame is not None
    assert (frame.sequence, frame.native_timestamp_ns, frame.data) == (
        2,
        90_100_000_000,
        b"second",
    )
    assert preview.status() == {
        "fresh": True,
        "native_timestamp_ns": 90_100_000_000,
        "received_wall_time_ns": 1_000_000_001,
        "sequence": 2,
        "width": 2,
        "height": 1,
        "source": "onboard",
    }

    wall[0] += 2_000_000_001
    assert preview.status()["fresh"] is False


def test_preview_http_serves_status_and_multipart_latest_frame():
    preview = LatestFramePreview()
    server = PreviewServer(
        preview,
        host="127.0.0.1",
        port=0,
        encode_jpeg=lambda frame: b"\xff\xd8jpeg\xff\xd9",
    )
    server.start()
    try:
        preview.update(
            native_timestamp_ns=50_000_000,
            width=1,
            height=1,
            encoding="rgb8",
            data=b"\x01\x02\x03",
            source="onboard",
        )
        base = f"http://127.0.0.1:{server.port}"
        with urllib.request.urlopen(f"{base}/status", timeout=2) as response:
            status = json.load(response)
        assert status["fresh"] is True
        assert status["sequence"] == 1
        assert status["native_timestamp_ns"] == 50_000_000

        with urllib.request.urlopen(f"{base}/camera.mjpg", timeout=2) as response:
            body = response.read1(96)
        assert response.headers["Content-Type"] == (
            "multipart/x-mixed-replace; boundary=frame"
        )
        assert b"Content-Type: image/jpeg" in body
        assert b"\xff\xd8jpeg\xff\xd9" in body
    finally:
        server.close()
