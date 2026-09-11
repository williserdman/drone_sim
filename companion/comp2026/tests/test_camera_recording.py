from __future__ import annotations

import json
import threading

import numpy as np
import pytest

from drone.sensors.camera.camera import Camera


class Manager:
    last_frame_timestamp = None


class UnprintableRecordingFailure(Exception):
    def __str__(self):
        raise RuntimeError("diagnostic formatting failed")


def mounting_file(tmp_path):
    path = tmp_path / "mounting.json"
    path.write_text(
        json.dumps(
            {
                "camera_to_body_frd": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "down_offset_m": 0.0,
                "verified": True,
            }
        )
    )
    return path


class BlockingWriter:
    def __init__(self, entered, release_write):
        self.entered = entered
        self.release_write = release_write
        self.frames = []
        self.released = False

    def isOpened(self):
        return True

    def write(self, frame):
        self.entered.set()
        assert self.release_write.wait(1.0)
        self.frames.append(frame)

    def release(self):
        self.released = True


def test_encoding_does_not_hold_frame_buffer_lock(tmp_path):
    entered = threading.Event()
    release = threading.Event()
    writer = BlockingWriter(entered, release)
    camera = Camera(
        100,
        manager=Manager(),
        mounting_path=mounting_file(tmp_path),
        writer_factory=lambda *args: writer,
    )
    camera._buffer_frame(np.zeros((2, 2, 3), dtype=np.uint8))
    thread = camera.save_frame_buffer_async(str(tmp_path / "capture.mp4"))
    assert entered.wait(1.0)

    append = threading.Thread(
        target=camera._buffer_frame,
        args=(np.ones((2, 2, 3), dtype=np.uint8),),
    )
    append.start()
    append.join(0.2)
    release.set()
    thread.join(1.0)

    assert not append.is_alive()
    assert len(camera.frame_buffer) == 2
    assert writer.released is True
    assert camera.recording_errors == {}


def test_concurrent_recordings_cannot_overwrite_same_path(tmp_path):
    entered = threading.Event()
    release = threading.Event()
    camera = Camera(
        100,
        manager=Manager(),
        mounting_path=mounting_file(tmp_path),
        writer_factory=lambda *args: BlockingWriter(entered, release),
    )
    camera._buffer_frame(np.zeros((2, 2, 3), dtype=np.uint8))
    path = str(tmp_path / "capture.mp4")
    thread = camera.save_frame_buffer_async(path)
    assert entered.wait(1.0)

    with pytest.raises(RuntimeError, match="already in progress"):
        camera.save_frame_buffer_async(path)

    release.set()
    thread.join(1.0)


def test_equivalent_destination_spellings_share_one_writer_lock(tmp_path):
    entered = threading.Event()
    release = threading.Event()
    camera = Camera(
        100,
        manager=Manager(),
        mounting_path=mounting_file(tmp_path),
        writer_factory=lambda *args: BlockingWriter(entered, release),
    )
    camera._buffer_frame(np.zeros((2, 2, 3), dtype=np.uint8))
    direct = str(tmp_path / "capture.mp4")
    equivalent = f"{tmp_path}/./capture.mp4"
    thread = camera.save_frame_buffer_async(direct)
    assert entered.wait(1.0)

    with pytest.raises(RuntimeError, match="already in progress"):
        camera.save_frame_buffer_async(equivalent)

    release.set()
    thread.join(1.0)


def test_writer_open_failure_is_available_as_a_diagnostic(tmp_path):
    class ClosedWriter:
        def isOpened(self):
            return False

        def release(self):
            pass

    path = str(tmp_path / "capture.mp4")
    camera = Camera(
        100,
        manager=Manager(),
        mounting_path=mounting_file(tmp_path),
        writer_factory=lambda *args: ClosedWriter(),
    )
    camera._buffer_frame(np.zeros((2, 2, 3), dtype=np.uint8))

    thread = camera.save_frame_buffer_async(path)
    thread.join(1.0)

    assert "could not be opened" in camera.recording_errors[path]


@pytest.mark.parametrize(
    "write_failure",
    [KeyboardInterrupt(), UnprintableRecordingFailure()],
)
def test_release_failure_does_not_replace_write_failure(tmp_path, write_failure):
    class FailingWriter:
        release_calls = 0

        def isOpened(self):
            return True

        def write(self, _frame):
            raise write_failure

        def release(self):
            self.release_calls += 1
            raise SystemExit()

    writer = FailingWriter()
    camera = Camera(
        100,
        manager=Manager(),
        mounting_path=mounting_file(tmp_path),
        writer_factory=lambda *args: writer,
    )
    camera._buffer_frame(np.zeros((2, 2, 3), dtype=np.uint8))

    with pytest.raises(BaseException) as raised:
        camera._save_frame_buffer_to_disk(str(tmp_path / "capture.mp4"))

    traceback_functions = []
    traceback = raised.tb
    while traceback is not None:
        traceback_functions.append(traceback.tb_frame.f_code.co_name)
        traceback = traceback.tb_next
    assert raised.value is write_failure
    assert traceback_functions[-1] == "write"
    assert "release" not in traceback_functions
    assert writer.release_calls == 1
    assert write_failure.__notes__ == [
        "Video writer release also failed: SystemExit"
    ]


@pytest.mark.parametrize(
    ("write_failure", "release_failure", "expected_diagnostic"),
    [
        (
            KeyboardInterrupt(),
            UnprintableRecordingFailure(),
            "KeyboardInterrupt; Video writer release also failed: "
            "UnprintableRecordingFailure",
        ),
        (
            UnprintableRecordingFailure(),
            SystemExit(),
            "UnprintableRecordingFailure; Video writer release also failed: "
            "SystemExit",
        ),
    ],
)
def test_recording_reports_write_and_release_failures(
    tmp_path, write_failure, release_failure, expected_diagnostic
):
    class FailingWriter:
        release_calls = 0

        def isOpened(self):
            return True

        def write(self, _frame):
            raise write_failure

        def release(self):
            self.release_calls += 1
            raise release_failure

    writer = FailingWriter()
    path = str(tmp_path / "capture.mp4")
    camera = Camera(
        100,
        manager=Manager(),
        mounting_path=mounting_file(tmp_path),
        writer_factory=lambda *args: writer,
    )
    camera._buffer_frame(np.zeros((2, 2, 3), dtype=np.uint8))

    camera.save_frame_buffer_async(path)

    assert camera.wait_for_recordings(timeout_s=1.0) is True
    assert camera.recording_errors[path] == expected_diagnostic
    assert writer.release_calls == 1


def test_release_failure_after_successful_encoding_is_recorded(tmp_path):
    class ReleaseFailingWriter:
        def isOpened(self):
            return True

        def write(self, _frame):
            pass

        def release(self):
            raise UnprintableRecordingFailure()

    path = str(tmp_path / "capture.mp4")
    camera = Camera(
        100,
        manager=Manager(),
        mounting_path=mounting_file(tmp_path),
        writer_factory=lambda *args: ReleaseFailingWriter(),
    )
    camera._buffer_frame(np.zeros((2, 2, 3), dtype=np.uint8))

    camera.save_frame_buffer_async(path)

    assert camera.wait_for_recordings(timeout_s=1.0) is True
    assert camera.recording_errors[path] == "UnprintableRecordingFailure"


@pytest.mark.parametrize(
    ("failure", "expected_diagnostic"),
    [
        (KeyboardInterrupt(), "KeyboardInterrupt"),
        (SystemExit(), "SystemExit"),
        (UnprintableRecordingFailure(), "UnprintableRecordingFailure"),
    ],
)
def test_recording_worker_contains_base_exception_and_records_safe_diagnostic(
    tmp_path, monkeypatch, failure, expected_diagnostic
):
    uncaught = []
    monkeypatch.setattr(
        threading,
        "excepthook",
        lambda args: uncaught.append(args.exc_value),
    )

    def fail_writer(*_args):
        raise failure

    path = str(tmp_path / "capture.mp4")
    camera = Camera(
        100,
        manager=Manager(),
        mounting_path=mounting_file(tmp_path),
        writer_factory=fail_writer,
    )
    camera._buffer_frame(np.zeros((2, 2, 3), dtype=np.uint8))

    camera.save_frame_buffer_async(path)

    assert camera.wait_for_recordings(timeout_s=1.0) is True
    assert camera.recording_errors[path] == expected_diagnostic
    assert uncaught == []


def test_wait_for_recordings_has_a_bounded_timeout(tmp_path):
    entered = threading.Event()
    release = threading.Event()
    camera = Camera(
        100,
        manager=Manager(),
        mounting_path=mounting_file(tmp_path),
        writer_factory=lambda *args: BlockingWriter(entered, release),
    )
    camera._buffer_frame(np.zeros((2, 2, 3), dtype=np.uint8))
    camera.save_frame_buffer_async(str(tmp_path / "capture.mp4"))
    assert entered.wait(1.0)

    assert camera.wait_for_recordings(timeout_s=0.01) is False
    release.set()
    assert camera.wait_for_recordings(timeout_s=1.0) is True
