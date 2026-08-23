from dataclasses import FrozenInstanceError
from datetime import UTC, datetime
import io
import json
import os
from pathlib import Path
import shutil
from types import SimpleNamespace
import subprocess
import time

import pytest

from artifacts._adapters.video import (
    VideoDiagnostic,
    VideoFinalization,
    VideoStreamRecorder,
    VideoValidationResult,
    VideoValidator,
)
from artifacts.recorder_node import VIDEO_TOPICS, VideoRecorderNode
from artifacts.validation import ValidationStatus


RUN_ID = "run-7"
FRAME_BYTES = bytes(index % 251 for index in range(320 * 240 * 3))


def _stamp(timestamp_ns):
    return SimpleNamespace(sec=timestamp_ns // 1_000_000_000, nanosec=timestamp_ns % 1_000_000_000)


def _image(timestamp_ns=0, **changes):
    values = dict(header=SimpleNamespace(stamp=_stamp(timestamp_ns)), height=240, width=320,
                  encoding="rgb8", step=960, data=FRAME_BYTES)
    values.update(changes)
    return SimpleNamespace(**values)


def _metadata(timestamp_ns=0, frame_id=0, **changes):
    values = dict(run_id=RUN_ID, sim_timestamp=_stamp(timestamp_ns), frame_id=frame_id,
                  stream="onboard")
    values.update(changes)
    return SimpleNamespace(**values)


class FakeProcess:
    def __init__(self, wait_outcomes=(0,), *, advance=None):
        self.stdin = io.BytesIO()
        self.returncode = None
        self.wait_outcomes = list(wait_outcomes)
        self.wait_timeouts = []
        self.signals = []
        self.advance = advance or (lambda _seconds: None)

    def poll(self):
        return self.returncode

    def wait(self, timeout):
        self.wait_timeouts.append(timeout)
        outcome = self.wait_outcomes.pop(0) if self.wait_outcomes else 0
        if outcome == "timeout":
            self.advance(timeout)
            raise subprocess.TimeoutExpired("ffmpeg", timeout)
        self.returncode = outcome
        return outcome

    def send_signal(self, signum):
        self.signals.append(signum)


class BrokenCloseStream(io.BytesIO):
    def close(self):
        super().close()
        raise BrokenPipeError("encoder closed pipe")


class SlowCloseStream(io.BytesIO):
    def close(self):
        time.sleep(0.10)
        super().close()


class FakeProcessFactory:
    def __init__(self, process=None):
        self.process = process or FakeProcess()
        self.calls = []

    def __call__(self, command, **kwargs):
        self.calls.append((tuple(command), kwargs))
        return self.process


class FakeCommandRunner:
    def __init__(self, probe_document=None, *, decode_returncode=0):
        self.probe_document = probe_document or {"streams": [{
            "codec_type": "video", "codec_name": "h264", "pix_fmt": "yuv420p",
            "avg_frame_rate": "20/1", "width": 320, "height": 240,
            "nb_read_frames": "4",
        }]}
        self.decode_returncode = decode_returncode
        self.calls = []

    def __call__(self, command, **kwargs):
        command = tuple(command)
        self.calls.append((command, kwargs))
        if "-encoders" in command:
            return subprocess.CompletedProcess(command, 0, " V..... libx264 H.264\n", "")
        if command[:2] == ("ffprobe", "-version"):
            return subprocess.CompletedProcess(command, 0, "ffprobe version 6.1\n", "")
        if command[0] == "ffprobe":
            return subprocess.CompletedProcess(command, 0, json.dumps(self.probe_document), "")
        stderr = "decode failed" if self.decode_returncode else ""
        return subprocess.CompletedProcess(command, self.decode_returncode, "", stderr)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def _valid_result(frame_count=1):
    return VideoValidationResult(
        ValidationStatus.VALID, 100, "a" * 64, "valid video",
        (VideoDiagnostic("onboard", "video_probe", "valid video"),),
        "h264", "yuv420p", "20/1", 320, 240, frame_count,
    )


class FakeValidator:
    def __init__(self, result=None):
        self.result = result or _valid_result()
        self.calls = []

    def validate(
        self,
        run_directory,
        relative_path,
        *,
        expected_frame_count,
        outcome,
        deadline=None,
        descriptor=None,
    ):
        self.calls.append(
            (
                Path(run_directory),
                str(relative_path),
                expected_frame_count,
                outcome,
                deadline,
                descriptor,
            )
        )
        return self.result


def _recorder(tmp_path, **changes):
    values = dict(run_directory=tmp_path, run_id=RUN_ID, stream="onboard",
                  expected_frame_count=1,
                  process_factory=FakeProcessFactory(), command_runner=FakeCommandRunner(),
                  validator=FakeValidator())
    values.update(changes)
    recorder = VideoStreamRecorder(**values)
    recorder.start()
    return recorder


def test_command_is_exact_and_start_is_shell_free(tmp_path):
    factory = FakeProcessFactory()
    recorder = _recorder(tmp_path, process_factory=factory)
    assert recorder.command() == (
        "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error", "-y",
        "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "320x240",
        "-framerate", "20", "-i", "pipe:0", "-an", "-c:v", "libx264",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-f", "mp4",
        f"/proc/self/fd/{recorder._partial_fd}",
    )
    assert factory.calls[0][1]["shell"] is False
    assert factory.calls[0][1]["stdin"] is subprocess.PIPE


def test_start_reserves_single_link_partial_and_ffmpeg_writes_retained_inode(tmp_path):
    factory = FakeProcessFactory()
    recorder = _recorder(tmp_path, process_factory=factory)

    assert recorder.partial_path.is_file()
    assert recorder.partial_path.stat().st_nlink == 1
    command, kwargs = factory.calls[0]
    output_argument = command[-1]
    assert output_argument.startswith("/proc/self/fd/")
    output_fd = int(output_argument.rsplit("/", 1)[1])
    assert kwargs["pass_fds"] == (output_fd,)
    assert os.path.samefile(output_argument, recorder.partial_path)


def test_finalize_close_cannot_consume_past_caller_deadline(tmp_path):
    process = FakeProcess((0,))
    process.stdin = SlowCloseStream()
    recorder = _recorder(tmp_path, process_factory=FakeProcessFactory(process))
    recorder.partial_path.write_bytes(b"partial")
    started = time.monotonic()

    result = recorder.finalize(deadline=started + 0.01, outcome="COMPLETED")

    assert time.monotonic() - started < 0.05
    assert result.published is False
    assert any("deadline" in item.detail for item in recorder.diagnostics)


def test_validator_bounds_probe_and_decode_by_remaining_caller_deadline(tmp_path):
    _video_file(tmp_path)
    clock = FakeClock()
    runner = FakeCommandRunner()
    validator = VideoValidator(command_runner=runner, monotonic=clock.monotonic)

    result = validator.validate(
        tmp_path,
        "video/onboard.mp4",
        expected_frame_count=4,
        outcome="COMPLETED",
        deadline=9.0,
    )

    assert result.status is ValidationStatus.VALID
    assert runner.calls[0][1]["timeout"] == pytest.approx(9.0)
    assert runner.calls[1][1]["timeout"] == pytest.approx(9.0)


def test_validator_rejects_success_returned_after_caller_deadline(tmp_path):
    _video_file(tmp_path)
    clock = FakeClock()

    class DeadlineIgnoringRunner(FakeCommandRunner):
        def __call__(self, command, **kwargs):
            result = super().__call__(command, **kwargs)
            if command[0] == "ffmpeg":
                clock.advance(kwargs["timeout"] + 1)
            return result

    result = VideoValidator(
        command_runner=DeadlineIgnoringRunner(), monotonic=clock.monotonic
    ).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4,
        outcome="COMPLETED", deadline=9.0,
    )

    assert result.status is ValidationStatus.INVALID
    assert "deadline" in result.detail


def test_path_swap_after_validation_never_publishes_replacement_inode(tmp_path):
    class SwappingValidator(FakeValidator):
        def validate(self, run_directory, relative_path, **kwargs):
            result = super().validate(run_directory, relative_path, **kwargs)
            partial = Path(run_directory) / relative_path
            partial.rename(partial.with_name("retained-original"))
            partial.write_bytes(b"attacker replacement")
            return result

    recorder = _recorder(tmp_path, validator=SwappingValidator())
    recorder.partial_path.write_bytes(b"encoded original")

    recorder.finalize(deadline=time.monotonic() + 1, outcome="COMPLETED")

    assert not recorder.final_path.exists() or recorder.final_path.read_bytes() != b"attacker replacement"


def test_start_preflights_encoder_and_ffprobe_before_process(tmp_path):
    runner, factory = FakeCommandRunner(), FakeProcessFactory()
    _recorder(tmp_path, command_runner=runner, process_factory=factory)
    assert "-encoders" in runner.calls[0][0]
    assert runner.calls[1][0] == ("ffprobe", "-version")
    assert len(factory.calls) == 1


def test_start_appends_ffmpeg_stderr_to_hardened_owned_log(tmp_path):
    log_path = tmp_path / "logs/docker/ffmpeg-onboard.log.partial"
    log_path.parent.mkdir(parents=True)
    log_path.write_bytes(b"existing diagnostics\n")
    factory = FakeProcessFactory()

    _recorder(tmp_path, process_factory=factory)

    stderr = factory.calls[0][1]["stderr"]
    assert os.path.samefile(f"/proc/self/fd/{stderr.fileno()}", log_path)
    assert log_path.read_bytes() == b"existing diagnostics\n"


def test_start_rejects_symlinked_or_hardlinked_ffmpeg_log(tmp_path):
    outside = tmp_path.parent / "outside-ffmpeg.log"
    outside.write_bytes(b"outside")
    log_path = tmp_path / "logs/docker/ffmpeg-onboard.log.partial"
    log_path.parent.mkdir(parents=True)
    log_path.symlink_to(outside)
    factory = FakeProcessFactory()
    recorder = VideoStreamRecorder(tmp_path, run_id=RUN_ID, stream="onboard",
        process_factory=factory, command_runner=FakeCommandRunner())
    with pytest.raises(RuntimeError, match="unsafe recorder log path"):
        recorder.start()
    log_path.unlink()
    os.link(outside, log_path)
    with pytest.raises(RuntimeError, match="unsafe recorder log path"):
        recorder.start()
    assert outside.read_bytes() == b"outside" and factory.calls == []


@pytest.mark.parametrize("missing", ["encoder", "ffprobe"])
def test_preflight_failure_prevents_process_and_readiness(tmp_path, missing):
    class Runner(FakeCommandRunner):
        def __call__(self, command, **kwargs):
            result = super().__call__(command, **kwargs)
            if missing == "encoder" and "-encoders" in command:
                return subprocess.CompletedProcess(command, 0, "", "")
            if missing == "ffprobe" and tuple(command[:2]) == ("ffprobe", "-version"):
                return subprocess.CompletedProcess(command, 127, "", "missing")
            return result
    factory = FakeProcessFactory()
    recorder = VideoStreamRecorder(tmp_path, run_id=RUN_ID, stream="onboard",
                                   command_runner=Runner(), process_factory=factory)
    with pytest.raises(RuntimeError, match="preflight"):
        recorder.start()
    assert recorder.is_ready is False
    assert factory.calls == []


def test_preflight_requires_exact_libx264_encoder_name(tmp_path):
    class RgbOnlyRunner(FakeCommandRunner):
        def __call__(self, command, **kwargs):
            result = super().__call__(command, **kwargs)
            if "-encoders" in command:
                return subprocess.CompletedProcess(
                    command, 0, " V..... libx264rgb H.264 RGB\n", ""
                )
            return result

    recorder = VideoStreamRecorder(
        tmp_path, run_id=RUN_ID, stream="onboard", command_runner=RgbOnlyRunner()
    )
    with pytest.raises(RuntimeError, match="preflight"):
        recorder.start()


@pytest.mark.parametrize("order", [("image", "metadata"), ("metadata", "image")])
def test_frame_is_written_only_after_exact_pair_in_either_arrival_order(tmp_path, order):
    recorder = _recorder(tmp_path)
    process = recorder._process
    first = _image() if order[0] == "image" else _metadata()
    second = _image() if order[1] == "image" else _metadata()
    getattr(recorder, f"accept_{order[0]}")(first)
    assert process.stdin.getvalue() == b""
    assert recorder.pending_counts in ((1, 0), (0, 1))
    getattr(recorder, f"accept_{order[1]}")(second)
    assert process.stdin.getvalue() == FRAME_BYTES
    assert recorder.pending_counts == (0, 0)
    assert recorder.frame_count == 1


def test_multiple_pairs_preserve_bytes_and_integer_timestamps(tmp_path):
    recorder = _recorder(tmp_path)
    for frame_id, timestamp_ns in enumerate((0, 50_000_000, 100_000_000)):
        recorder.accept_metadata(_metadata(timestamp_ns, frame_id))
        recorder.accept_image(_image(timestamp_ns))
    assert recorder.frame_count == 3
    assert recorder.last_timestamp_ns == 100_000_000
    assert recorder._process.stdin.getvalue() == FRAME_BYTES * 3


@pytest.mark.parametrize(("image", "metadata", "detail"), [
    (_image(), _metadata(run_id="wrong"), "run_id"),
    (_image(), _metadata(stream="observer"), "stream"),
    (_image(1), _metadata(2), "stamp"),
    (_image(width=319), _metadata(), "even"),
    (_image(height=239), _metadata(), "even"),
    (_image(encoding="bgr8"), _metadata(), "encoding"),
    (_image(step=959), _metadata(), "step"),
    (_image(data=FRAME_BYTES[:-1]), _metadata(), "payload"),
    (_image(data=FRAME_BYTES + b"x"), _metadata(), "payload"),
])
def test_invalid_first_pair_fails_closed_with_immutable_diagnostic(tmp_path, image, metadata, detail):
    recorder = _recorder(tmp_path)
    with pytest.raises(ValueError, match=detail):
        recorder.accept_metadata(metadata)
        recorder.accept_image(image)
    assert recorder.failed is True
    assert recorder.frame_count == 0
    assert len(recorder.diagnostics) == 1
    with pytest.raises(FrozenInstanceError):
        recorder.diagnostics[0].detail = "changed"
    with pytest.raises(RuntimeError, match="failed closed"):
        recorder.accept_image(_image())


@pytest.mark.parametrize(("second_id", "second_stamp", "detail"), [
    (0, 50_000_000, "frame ID"), (2, 50_000_000, "frame ID"),
    (1, 0, "timestamp"), (1, 49_999_999, "50"), (1, 50_000_001, "50"),
])
def test_sequence_rejects_duplicates_gaps_and_wrong_stamp_delta(tmp_path, second_id, second_stamp, detail):
    recorder = _recorder(tmp_path)
    recorder.accept_image(_image())
    recorder.accept_metadata(_metadata())
    recorder.accept_image(_image(second_stamp))
    with pytest.raises(ValueError, match=detail):
        recorder.accept_metadata(_metadata(second_stamp, second_id))
    assert recorder.failed is True
    assert recorder.frame_count == 1


@pytest.mark.parametrize("kind", ["image", "metadata"])
def test_second_unmatched_input_fails_without_growing_pending_state(tmp_path, kind):
    recorder = _recorder(tmp_path)
    getattr(recorder, f"accept_{kind}")(_image() if kind == "image" else _metadata())
    with pytest.raises(ValueError, match="unmatched"):
        getattr(recorder, f"accept_{kind}")(
            _image(50_000_000) if kind == "image" else _metadata(50_000_000, 1)
        )
    assert sum(recorder.pending_counts) <= 1
    assert recorder.failed is True


def test_changed_dimensions_after_start_fail_closed(tmp_path):
    recorder = _recorder(tmp_path)
    recorder.accept_image(_image()); recorder.accept_metadata(_metadata())
    recorder.accept_metadata(_metadata(50_000_000, 1))
    with pytest.raises(ValueError, match="dimensions"):
        recorder.accept_image(_image(50_000_000, width=322, step=966, data=b"x" * 231840))


@pytest.mark.parametrize("collision", ["partial", "final"])
def test_start_refuses_existing_output_without_spawning(tmp_path, collision):
    video = tmp_path / "video"; video.mkdir()
    target = video / ("onboard.mp4.partial" if collision == "partial" else "onboard.mp4")
    target.write_bytes(b"do not overwrite")
    factory = FakeProcessFactory()
    recorder = VideoStreamRecorder(tmp_path, run_id=RUN_ID, stream="onboard",
                                   process_factory=factory, command_runner=FakeCommandRunner())
    with pytest.raises(FileExistsError):
        recorder.start()
    assert factory.calls == []
    assert target.read_bytes() == b"do not overwrite"


def test_start_rejects_symlinked_video_directory_without_external_write(tmp_path):
    outside = tmp_path.parent / "outside-video"; outside.mkdir()
    (tmp_path / "video").symlink_to(outside, target_is_directory=True)
    factory = FakeProcessFactory()
    recorder = VideoStreamRecorder(tmp_path, run_id=RUN_ID, stream="onboard",
                                   process_factory=factory, command_runner=FakeCommandRunner())
    with pytest.raises(RuntimeError, match="unsafe video path"):
        recorder.start()
    assert factory.calls == [] and list(outside.iterdir()) == []


def test_finalize_closes_stdin_validates_and_publishes_without_clobber(tmp_path):
    validator = FakeValidator()
    recorder = _recorder(tmp_path, validator=validator)
    recorder.accept_image(_image()); recorder.accept_metadata(_metadata())
    recorder.partial_path.write_bytes(b"encoded")
    result = recorder.finalize(deadline=10.0, outcome="completed")
    assert result == VideoFinalization(True, 0, (), False, True, "video finalized")
    assert recorder._process.stdin.closed is True
    assert recorder.final_path.read_bytes() == b"encoded"
    assert not recorder.partial_path.exists()
    assert validator.calls == [(
        tmp_path.resolve(), "video/onboard.mp4.partial", 1, "COMPLETED", 10.0,
        validator.calls[0][5],
    )]
    assert isinstance(validator.calls[0][5], int)


@pytest.mark.parametrize("returncode", [1, 137])
def test_nonzero_exit_retains_partial_and_diagnostic(tmp_path, returncode):
    recorder = _recorder(tmp_path, process_factory=FakeProcessFactory(FakeProcess((returncode,))))
    recorder.partial_path.write_bytes(b"partial output")
    result = recorder.finalize(deadline=10.0, outcome="failed")
    assert result.published is False and result.returncode == returncode
    assert recorder.partial_path.read_bytes() == b"partial output"
    assert any("return code" in item.detail for item in recorder.diagnostics)
    assert recorder._root_fd is None and recorder._video_fd is None


def test_broken_pipe_while_closing_fails_closed_but_still_reaps_process(tmp_path):
    process = FakeProcess((0,))
    process.stdin = BrokenCloseStream()
    recorder = _recorder(tmp_path, process_factory=FakeProcessFactory(process))
    recorder.partial_path.write_bytes(b"partial output")

    result = recorder.finalize(deadline=10.0, outcome="completed")

    assert result.exited is True and result.published is False
    assert process.returncode == 0
    assert recorder.partial_path.exists()
    assert any("close" in item.detail for item in recorder.diagnostics)
    assert recorder._root_fd is None and recorder._video_fd is None


def test_stuck_process_uses_only_shared_deadline_and_bounded_term_kill(tmp_path):
    clock = FakeClock()
    process = FakeProcess(("timeout", "timeout", "timeout"), advance=clock.advance)
    recorder = _recorder(tmp_path, process_factory=FakeProcessFactory(process),
                         monotonic=clock.monotonic)
    recorder.partial_path.write_bytes(b"partial output")
    result = recorder.finalize(deadline=9.0, outcome="aborted")
    assert result.exited is False
    assert result.signals == ("SIGTERM", "SIGKILL") and result.escalated is True
    assert sum(process.wait_timeouts) == pytest.approx(9.0)
    assert clock.now == pytest.approx(9.0) and recorder.partial_path.exists()
    assert recorder._root_fd is None and recorder._video_fd is None


def test_invalid_output_is_retained_as_partial(tmp_path):
    invalid = VideoValidationResult(ValidationStatus.INVALID, 7, "a" * 64, "video is corrupt")
    recorder = _recorder(tmp_path, validator=FakeValidator(invalid))
    recorder.partial_path.write_bytes(b"partial")
    result = recorder.finalize(deadline=10.0, outcome="failed")
    assert result.published is False and "corrupt" in result.detail
    assert recorder.partial_path.exists()
    assert recorder._root_fd is None and recorder._video_fd is None


def test_final_appearing_during_finalize_is_not_clobbered(tmp_path):
    class CollidingValidator(FakeValidator):
        def validate(self, *args, **kwargs):
            result = super().validate(*args, **kwargs)
            tmp_path.joinpath("video/onboard.mp4").write_bytes(b"winner")
            return result
    recorder = _recorder(tmp_path, validator=CollidingValidator())
    recorder.partial_path.write_bytes(b"partial")
    result = recorder.finalize(deadline=10.0, outcome="completed")
    assert result.published is False
    assert recorder.final_path.read_bytes() == b"winner"
    assert recorder.partial_path.read_bytes() == b"partial"


def _video_file(tmp_path, contents=b"not really mp4"):
    path = tmp_path / "video/onboard.mp4"; path.parent.mkdir(); path.write_bytes(contents)
    return path


def test_validator_uses_machine_json_and_full_decode_for_completed_count(tmp_path):
    _video_file(tmp_path); runner = FakeCommandRunner()
    result = VideoValidator(command_runner=runner).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4, outcome="COMPLETED",
        deadline=time.monotonic() + 10)
    assert result.status is ValidationStatus.VALID
    assert (result.codec_name, result.pix_fmt, result.avg_frame_rate, result.width,
            result.height, result.frame_count) == ("h264", "yuv420p", "20/1", 320, 240, 4)
    assert runner.calls[0][0][0] == "ffprobe" and "json" in runner.calls[0][0]
    assert runner.calls[1][0][0] == "ffmpeg" and runner.calls[1][0][-2:] == ("null", "-")


@pytest.mark.parametrize("outcome", ["failed", "aborted"])
def test_failed_and_aborted_accept_readable_nonempty_shorter_video(tmp_path, outcome):
    _video_file(tmp_path); document = FakeCommandRunner().probe_document
    document["streams"][0]["nb_read_frames"] = "2"
    result = VideoValidator(command_runner=FakeCommandRunner(document)).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4, outcome=outcome,
        deadline=time.monotonic() + 10)
    assert result.status is ValidationStatus.VALID and result.frame_count == 2


@pytest.mark.parametrize(("mutation", "detail"), [
    (lambda s: s.append(dict(s[0])), "exactly one"),
    (lambda s: s.append({"codec_type": "audio"}), "unexpected"),
    (lambda s: s[0].update(codec_name="hevc"), "codec"),
    (lambda s: s[0].update(pix_fmt="yuv444p"), "pixel"),
    (lambda s: s[0].update(avg_frame_rate="25/1"), "frame rate"),
    (lambda s: s[0].update(width=640), "dimensions"),
    (lambda s: s[0].update(nb_read_frames="0"), "zero"),
    (lambda s: s[0].update(nb_read_frames="N/A"), "trustworthy"),
    (lambda s: s[0].update(nb_read_frames="5"), "expected"),
])
def test_validator_rejects_wrong_format_inventory_or_count(tmp_path, mutation, detail):
    _video_file(tmp_path); document = FakeCommandRunner().probe_document
    mutation(document["streams"])
    result = VideoValidator(command_runner=FakeCommandRunner(document)).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4, outcome="FAILED",
        deadline=time.monotonic() + 10)
    assert result.status is ValidationStatus.INVALID and detail in result.detail


def test_validator_rejects_decode_error(tmp_path):
    _video_file(tmp_path)
    result = VideoValidator(command_runner=FakeCommandRunner(decode_returncode=1)).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4, outcome="COMPLETED",
        deadline=time.monotonic() + 10)
    assert result.status is ValidationStatus.INVALID and "decode" in result.detail


def test_strict_decode_rejects_error_stderr_even_when_ffmpeg_returns_zero(tmp_path):
    _video_file(tmp_path)

    class LenientExitRunner(FakeCommandRunner):
        def __call__(self, command, **kwargs):
            result = super().__call__(command, **kwargs)
            if command[0] == "ffmpeg":
                return subprocess.CompletedProcess(command, 0, "", "corrupt packet")
            return result

    runner = LenientExitRunner()
    result = VideoValidator(command_runner=runner).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4,
        outcome="COMPLETED", deadline=time.monotonic() + 10,
    )
    assert result.status is ValidationStatus.INVALID
    assert "-xerror" in runner.calls[1][0]


@pytest.mark.parametrize("document", [None, [], {"streams": {}}, {"streams": [1]}])
def test_validator_malformed_json_shapes_fail_closed(tmp_path, document):
    _video_file(tmp_path)

    class ShapeRunner(FakeCommandRunner):
        def __call__(self, command, **kwargs):
            if command[0] == "ffprobe" and command[:2] != ("ffprobe", "-version"):
                return subprocess.CompletedProcess(command, 0, json.dumps(document), "")
            return super().__call__(command, **kwargs)

    result = VideoValidator(command_runner=ShapeRunner()).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4,
        outcome="FAILED", deadline=time.monotonic() + 10,
    )
    assert result.status is ValidationStatus.INVALID
    assert "JSON" in result.detail


@pytest.mark.parametrize(("count", "outcome", "status"), [
    (39, "COMPLETED", ValidationStatus.INVALID),
    (39, "FAILED", ValidationStatus.VALID),
    (40, "COMPLETED", ValidationStatus.VALID),
    (41, "ABORTED", ValidationStatus.INVALID),
])
def test_configured_expected_count_is_independent_of_observed_frames(
    tmp_path, count, outcome, status
):
    _video_file(tmp_path)
    document = FakeCommandRunner().probe_document
    document["streams"][0]["nb_read_frames"] = str(count)
    result = VideoValidator(command_runner=FakeCommandRunner(document)).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=40,
        outcome=outcome, deadline=time.monotonic() + 10,
    )
    assert result.status is status


def test_finalize_passes_immutable_configured_count_not_observation_count(tmp_path):
    validator = FakeValidator()
    recorder = _recorder(tmp_path, validator=validator, expected_frame_count=40)
    recorder.partial_path.write_bytes(b"encoded")

    recorder.finalize(deadline=10.0, outcome="COMPLETED")

    assert validator.calls[0][2] == 40
    with pytest.raises(AttributeError):
        recorder.expected_frame_count = 1


def test_path_aba_during_probe_returns_no_stale_checksum_or_diagnostics(tmp_path):
    path = _video_file(tmp_path, b"original")

    class AbaRunner(FakeCommandRunner):
        def __call__(self, command, **kwargs):
            result = super().__call__(command, **kwargs)
            if command[0] == "ffprobe":
                retained = path.with_name("retained")
                path.rename(retained)
                path.write_bytes(b"replacement")
                path.unlink()
                retained.rename(path)
            return result

    result = VideoValidator(command_runner=AbaRunner()).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4,
        outcome="COMPLETED", deadline=time.monotonic() + 10,
    )
    assert result.status is ValidationStatus.INVALID
    assert result.size_bytes is None and result.sha256 is None
    assert result.diagnostics == ()


def test_unexpected_validator_exception_returns_failure_and_closes_retained_fds(tmp_path):
    class ExplodingValidator(FakeValidator):
        def validate(self, *args, **kwargs):
            raise RuntimeError("validator exploded")

    recorder = _recorder(tmp_path, validator=ExplodingValidator())
    recorder.partial_path.write_bytes(b"partial")

    result = recorder.finalize(deadline=10.0, outcome="FAILED")

    assert result.published is False and "validator exploded" in result.detail
    assert recorder._partial_fd is None and recorder._video_fd is None and recorder._root_fd is None
    assert recorder.partial_path.read_bytes() == b"partial"


def test_diagnostic_sink_exception_cannot_escape_finalization(tmp_path):
    process = FakeProcess((1,))
    recorder = _recorder(
        tmp_path,
        process_factory=FakeProcessFactory(process),
        diagnostic_sink=lambda _item: (_ for _ in ()).throw(RuntimeError("sink")),
    )
    recorder.partial_path.write_bytes(b"partial")

    result = recorder.finalize(deadline=10.0, outcome="FAILED")

    assert result.published is False
    assert any(item.event == "diagnostic_sink_failed" for item in recorder.diagnostics)


def test_unexpected_probe_exception_returns_immutable_invalid_without_fd_leak(tmp_path):
    _video_file(tmp_path)
    before = len(os.listdir("/proc/self/fd"))

    def exploding_runner(_command, **_kwargs):
        raise RuntimeError("runner exploded")

    result = VideoValidator(command_runner=exploding_runner).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4,
        outcome="COMPLETED", deadline=time.monotonic() + 10,
    )

    assert result.status is ValidationStatus.INVALID
    assert result.size_bytes is None and result.sha256 is None
    assert len(os.listdir("/proc/self/fd")) == before
    with pytest.raises(FrozenInstanceError):
        result.detail = "changed"


@pytest.mark.parametrize("boundary", ["wait", "signal"])
def test_unexpected_process_boundary_exception_returns_failure_and_closes_fds(
    tmp_path, boundary
):
    class ExplodingProcess(FakeProcess):
        def wait(self, timeout):
            if boundary == "wait":
                raise RuntimeError("wait exploded")
            return super().wait(timeout)

        def send_signal(self, signum):
            if boundary == "signal":
                raise RuntimeError("signal exploded")
            return super().send_signal(signum)

    outcomes = ("timeout",) if boundary == "signal" else (0,)
    clock = FakeClock()
    process = ExplodingProcess(outcomes, advance=clock.advance)
    recorder = _recorder(
        tmp_path,
        process_factory=FakeProcessFactory(process),
        monotonic=clock.monotonic,
    )
    recorder.partial_path.write_bytes(b"partial")

    result = recorder.finalize(deadline=9.0, outcome="FAILED")

    assert result.published is False and "exploded" in result.detail
    assert recorder._partial_fd is None and recorder._video_fd is None and recorder._root_fd is None


def test_validator_returns_no_stale_checksum_or_diagnostics_when_file_changes(tmp_path):
    path = _video_file(tmp_path)
    class MutatingRunner(FakeCommandRunner):
        def __call__(self, command, **kwargs):
            result = super().__call__(command, **kwargs)
            if command[0] == "ffprobe": path.write_bytes(b"changed during validation")
            return result
    result = VideoValidator(command_runner=MutatingRunner()).validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4, outcome="COMPLETED",
        deadline=time.monotonic() + 10)
    assert result.status is ValidationStatus.INVALID
    assert result.size_bytes is None and result.sha256 is None and result.diagnostics == ()
    assert "changed" in result.detail


def test_validator_rejects_symlink_and_hardlink_without_probing(tmp_path):
    outside = tmp_path.parent / "outside.mp4"; outside.write_bytes(b"outside")
    video = tmp_path / "video"; video.mkdir(); (video / "onboard.mp4").symlink_to(outside)
    runner = FakeCommandRunner(); validator = VideoValidator(command_runner=runner)
    symlink = validator.validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=1, outcome="FAILED",
        deadline=time.monotonic() + 10)
    (video / "onboard.mp4").unlink(); os.link(outside, video / "onboard.mp4")
    hardlink = validator.validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=1, outcome="FAILED",
        deadline=time.monotonic() + 10)
    assert symlink.status is ValidationStatus.INVALID and hardlink.status is ValidationStatus.INVALID
    assert runner.calls == []


def _require_ffmpeg():
    missing = [name for name in ("ffmpeg", "ffprobe") if shutil.which(name) is None]
    if not missing: return
    if os.environ.get("DRONE_SIM_REQUIRE_ROS_TESTS") == "1":
        pytest.fail(f"artifact image is missing required binaries: {missing}")
    pytest.skip("real FFmpeg fixture runs in the artifact test image")


def test_real_four_frame_ffmpeg_fixture_probes_and_fully_decodes(tmp_path):
    _require_ffmpeg()
    raw = tmp_path / "four.rgb"; raw.write_bytes(FRAME_BYTES * 4)
    video = tmp_path / "video/onboard.mp4"; video.parent.mkdir()
    subprocess.run([
        "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error", "-f", "rawvideo",
        "-pixel_format", "rgb24", "-video_size", "320x240", "-framerate", "20",
        "-i", str(raw), "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", "-frames:v", "4", "-f", "mp4", str(video),
    ], check=True)
    result = VideoValidator().validate(tmp_path, "video/onboard.mp4",
                                       expected_frame_count=4, outcome="COMPLETED",
                                       deadline=time.monotonic() + 15)
    assert result.status is ValidationStatus.VALID
    assert (result.codec_name, result.pix_fmt, result.avg_frame_rate) == ("h264", "yuv420p", "20/1")
    assert (result.width, result.height, result.frame_count) == (320, 240, 4)


def test_real_recorder_pipe_encodes_four_paired_frames_and_atomically_publishes(tmp_path):
    _require_ffmpeg()
    recorder = VideoStreamRecorder(
        tmp_path, run_id=RUN_ID, stream="onboard", expected_frame_count=4
    )
    recorder.start()
    for frame_id in range(4):
        timestamp_ns = frame_id * 50_000_000
        recorder.accept_image(_image(timestamp_ns))
        recorder.accept_metadata(_metadata(timestamp_ns, frame_id))

    result = recorder.finalize(deadline=__import__("time").monotonic() + 15, outcome="completed")

    assert result.published is True
    assert recorder.final_path.is_file() and recorder.final_path.stat().st_size > 0
    validated = VideoValidator().validate(
        tmp_path, "video/onboard.mp4", expected_frame_count=4, outcome="COMPLETED",
        deadline=time.monotonic() + 15)
    assert validated.status is ValidationStatus.VALID and validated.frame_count == 4


def test_test_image_ffmpeg_dependency_delta_matches_recorded_exact_versions():
    if os.environ.get("DRONE_SIM_REQUIRE_ROS_TESTS") != "1":
        pytest.skip("FFmpeg apt dependency lock is verified in the artifact test image")
    lock_path = os.environ.get("DRONE_SIM_FFMPEG_APT_PACKAGE_LOCK")
    assert lock_path is not None, "artifact image must expose the FFmpeg apt lock"
    recorded = dict(
        line.split("=", 1)
        for line in Path(lock_path).read_text(encoding="utf-8").splitlines()
        if line
    )
    installed = {
        package: subprocess.check_output(
            ["dpkg-query", "-W", "-f=${Version}", package], text=True
        )
        for package in recorded
    }
    assert installed == recorded


class FakeRecorder:
    def __init__(self, stream):
        self.stream, self.is_ready, self.images, self.metadata, self.started = stream, False, [], [], 0
        self.finalizations = []
    def start(self): self.started += 1; self.is_ready = True
    def accept_image(self, message): self.images.append(message)
    def accept_metadata(self, message): self.metadata.append(message)
    def finalize(self, deadline, *, outcome):
        self.finalizations.append((deadline, outcome))
        self.is_ready = False
        return VideoFinalization(True, 0, (), False, False, "rolled back")


class FakeNodeBackend:
    def __init__(self):
        self.subscriptions = []; self.counts = {topic: 1 for topic in VIDEO_TOPICS}
    def create_subscription(self, message_type, topic, callback, qos):
        item = SimpleNamespace(message_type=message_type, topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(item); return item
    def count_subscribers(self, topic): return self.counts[topic]


def test_video_node_owns_two_recorders_and_four_best_effort_depth_five_subscriptions(tmp_path):
    backend = FakeNodeBackend(); recorders = {s: FakeRecorder(s) for s in ("onboard", "observer")}
    node = VideoRecorderNode(tmp_path, RUN_ID, node_backend=backend,
        recorder_factory=lambda _run, _id, stream, _errors: recorders[stream],
        message_types=(object, object),
        qos_factory=lambda: SimpleNamespace(depth=5, reliability="best_effort"))
    assert tuple(node.recorders) == ("onboard", "observer") and len(backend.subscriptions) == 4
    assert all(s.qos.depth == 5 and s.qos.reliability == "best_effort" for s in backend.subscriptions)
    node.start(deadline=10.0)
    assert node.is_ready is True
    assert node.discovered_subscription_counts == {topic: 1 for topic in VIDEO_TOPICS}
    message = object()
    next(s for s in backend.subscriptions if s.topic.endswith("onboard/image_raw")).callback(message)
    assert recorders["onboard"].images == [message]


def test_video_node_rolls_back_started_recorder_when_second_start_fails(tmp_path):
    backend, events = FakeNodeBackend(), []
    recorders = {stream: FakeRecorder(stream) for stream in ("onboard", "observer")}

    def fail_observer_start():
        raise RuntimeError("observer failed")

    recorders["observer"].start = fail_observer_start
    node = VideoRecorderNode(
        tmp_path,
        RUN_ID,
        expected_frame_count=40,
        node_backend=backend,
        recorder_factory=lambda _run, _id, stream, _errors: recorders[stream],
        message_types=(object, object),
        qos_factory=lambda: SimpleNamespace(depth=5, reliability="best_effort"),
        error_sink=events.append,
    )

    with pytest.raises(RuntimeError, match="observer failed"):
        node.start(deadline=17.0)

    assert node.expected_frame_count == 40
    assert recorders["onboard"].finalizations == [(17.0, "FAILED")]
    assert events and events[0].event == "video_start_failed"
    assert events[0].fields["stream"] == "observer"


@pytest.mark.parametrize("kind", ["image", "metadata"])
def test_subscription_callback_contains_recorder_exception_and_keeps_executor_alive(
    tmp_path, kind
):
    backend, events = FakeNodeBackend(), []
    recorders = {stream: FakeRecorder(stream) for stream in ("onboard", "observer")}
    method = f"accept_{kind}"
    setattr(
        recorders["onboard"],
        method,
        lambda _message: (_ for _ in ()).throw(RuntimeError("recorder rejected")),
    )
    node = VideoRecorderNode(
        tmp_path,
        RUN_ID,
        node_backend=backend,
        recorder_factory=lambda _run, _id, stream, _errors: recorders[stream],
        message_types=(object, object),
        qos_factory=lambda: SimpleNamespace(depth=5, reliability="best_effort"),
        error_sink=events.append,
    )
    suffix = "image_raw" if kind == "image" else "frame_metadata"
    callback = next(
        item.callback for item in backend.subscriptions
        if item.topic == f"/camera/onboard/{suffix}"
    )

    callback(object())

    assert events and events[-1].event == "recorder_callback_failed"
    assert node.recorders["observer"].is_ready is False


def test_video_node_reports_structured_error_without_terminal_status(tmp_path):
    backend, events = FakeNodeBackend(), []
    node = VideoRecorderNode(tmp_path, RUN_ID, node_backend=backend,
        recorder_factory=lambda _run, _id, stream, _errors: FakeRecorder(stream),
        message_types=(object, object),
        qos_factory=lambda: SimpleNamespace(depth=5, reliability="best_effort"),
        error_sink=events.append,
        wall_clock=lambda: datetime(2026, 8, 23, tzinfo=UTC))
    node.report_error(VideoDiagnostic("onboard", "frame_rejected", "bad frame"))
    assert len(events) == 1
    assert json.loads(events[0].to_json_line()) == {
        "run_id": RUN_ID, "module": "artifacts", "severity": "ERROR",
        "event": "frame_rejected", "sim_timestamp": None,
        "wall_timestamp": "2026-08-23T00:00:00Z",
        "fields": {"stream": "onboard", "detail": "bad frame"},
    }
    assert not hasattr(node, "terminal_status")


def test_real_jazzy_video_node_is_spinable_and_owns_fixed_qos_subscriptions(tmp_path):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import ReliabilityPolicy
    except ImportError:
        if os.environ.get("DRONE_SIM_REQUIRE_ROS_TESTS") == "1":
            pytest.fail("artifact image is missing the Jazzy Python client")
        pytest.skip("real Jazzy node contract runs in the artifact test image")

    rclpy.init()
    node = None
    try:
        node = VideoRecorderNode(
            tmp_path,
            RUN_ID,
            recorder_factory=lambda _run, _id, stream, _errors: FakeRecorder(stream),
        )
        assert isinstance(node, Node)
        assert len(node.subscription_handles) == 4
        for topic, subscription in zip(VIDEO_TOPICS, node.subscription_handles):
            endpoints = node.get_subscriptions_info_by_topic(topic)
            assert len(endpoints) == 1
            assert subscription.qos_profile.depth == 5
            assert endpoints[0].qos_profile.reliability is ReliabilityPolicy.BEST_EFFORT
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
