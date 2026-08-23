"""Simulation-time RGB frame pairing, FFmpeg control, and MP4 validation."""

from collections.abc import Callable, Sequence
import ctypes
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import time
from typing import Any, Protocol

from ..validation import ValidationResult, ValidationStatus, validate_regular_file


FRAME_INTERVAL_NS = 50_000_000
WIDTH_PX = 320
HEIGHT_PX = 240
FPS = 20
ENCODING = "rgb8"
STEP_BYTES = WIDTH_PX * 3
PAYLOAD_BYTES = WIDTH_PX * HEIGHT_PX * 3
STREAMS = ("onboard", "observer")
_DIRECTORY_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_DIRECTORY", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_FILE_FLAGS = (
    os.O_RDONLY
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
)
_LOG_FLAGS = (
    os.O_WRONLY
    | os.O_APPEND
    | os.O_CREAT
    | getattr(os, "O_CLOEXEC", 0)
    | getattr(os, "O_NOFOLLOW", 0)
    | getattr(os, "O_NONBLOCK", 0)
)


class _Process(Protocol):
    stdin: Any
    returncode: int | None

    def poll(self) -> int | None: ...
    def wait(self, timeout: float) -> int: ...
    def send_signal(self, signum: int) -> None: ...


@dataclass(frozen=True)
class VideoDiagnostic:
    stream: str
    event: str
    detail: str
    sim_timestamp_ns: int | None = None


@dataclass(frozen=True)
class VideoFinalization:
    exited: bool
    returncode: int | None
    signals: tuple[str, ...]
    escalated: bool
    published: bool
    detail: str


@dataclass(frozen=True)
class VideoValidationResult(ValidationResult):
    diagnostics: tuple[VideoDiagnostic, ...] = ()
    codec_name: str | None = None
    pix_fmt: str | None = None
    avg_frame_rate: str | None = None
    width: int | None = None
    height: int | None = None
    frame_count: int | None = None


def _run_command(command: Sequence[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, **kwargs)


def _spawn(command: Sequence[str], **kwargs: Any) -> _Process:
    return subprocess.Popen(command, **kwargs)


def _timestamp_ns(stamp: Any) -> int:
    seconds = stamp.sec
    nanoseconds = stamp.nanosec
    if (
        not isinstance(seconds, int)
        or isinstance(seconds, bool)
        or not isinstance(nanoseconds, int)
        or isinstance(nanoseconds, bool)
    ):
        raise ValueError("simulation timestamp fields must be integers")
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        raise ValueError("simulation timestamp is outside the ROS Time range")
    return seconds * 1_000_000_000 + nanoseconds


def _same_entry(first: os.stat_result, second: os.stat_result) -> bool:
    return (
        first.st_dev == second.st_dev
        and first.st_ino == second.st_ino
        and first.st_mode == second.st_mode
    )


def _same_snapshot(first: os.stat_result, second: os.stat_result) -> bool:
    fields = ("st_dev", "st_ino", "st_mode", "st_nlink", "st_size", "st_mtime_ns", "st_ctime_ns")
    return all(getattr(first, field) == getattr(second, field) for field in fields)


def _unsafe_path(detail: str = "unsafe video path") -> RuntimeError:
    return RuntimeError(detail)


def _open_or_create_directory(parent_fd: int, name: str) -> tuple[int, os.stat_result]:
    try:
        os.mkdir(name, 0o755, dir_fd=parent_fd)
    except FileExistsError:
        pass
    except OSError as error:
        raise _unsafe_path() from error
    try:
        before = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        if not stat.S_ISDIR(before.st_mode) or stat.S_ISLNK(before.st_mode):
            raise _unsafe_path()
        descriptor = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
        opened = os.fstat(descriptor)
    except RuntimeError:
        raise
    except OSError as error:
        raise _unsafe_path() from error
    if not _same_entry(before, opened):
        os.close(descriptor)
        raise _unsafe_path()
    return descriptor, opened


def _rename_noreplace(parent_fd: int, source: str, destination: str) -> None:
    """Linux atomic rename with no replacement."""
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise OSError("renameat2 is required for collision-safe publication")
    renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    renameat2.restype = ctypes.c_int
    if renameat2(parent_fd, os.fsencode(source), parent_fd, os.fsencode(destination), 1) != 0:
        error_number = ctypes.get_errno()
        if error_number == getattr(os, "EEXIST", 17):
            raise FileExistsError(destination)
        raise OSError(error_number, os.strerror(error_number), destination)


class VideoValidator:
    """Read-only semantic validation of one stable, hardened MP4 snapshot."""

    def __init__(
        self,
        *,
        width_px: int = WIDTH_PX,
        height_px: int = HEIGHT_PX,
        fps: int = FPS,
        command_runner: Callable[..., subprocess.CompletedProcess[str]] = _run_command,
    ) -> None:
        self.width_px = width_px
        self.height_px = height_px
        self.fps = fps
        self._command_runner = command_runner

    @staticmethod
    def _result(
        filesystem: ValidationResult,
        status: ValidationStatus,
        detail: str,
        *,
        diagnostic: VideoDiagnostic | None = None,
        stream: dict[str, Any] | None = None,
        frame_count: int | None = None,
    ) -> VideoValidationResult:
        values = stream or {}
        return VideoValidationResult(
            status,
            filesystem.size_bytes,
            filesystem.sha256,
            detail,
            (diagnostic,) if diagnostic is not None else (),
            values.get("codec_name"),
            values.get("pix_fmt"),
            values.get("avg_frame_rate"),
            values.get("width"),
            values.get("height"),
            frame_count,
        )

    def _invalid(
        self, filesystem: ValidationResult, detail: str, stream_name: str, **facts: Any
    ) -> VideoValidationResult:
        return self._result(
            filesystem,
            ValidationStatus.INVALID,
            detail,
            diagnostic=VideoDiagnostic(stream_name, "video_invalid", detail),
            **facts,
        )

    def validate(
        self,
        run_directory: Path | str,
        relative_path: Path | str,
        *,
        expected_frame_count: int,
        outcome: str,
    ) -> VideoValidationResult:
        if outcome not in {"completed", "failed", "aborted"}:
            raise ValueError("outcome must be completed, failed, or aborted")
        if not isinstance(expected_frame_count, int) or isinstance(expected_frame_count, bool) or expected_frame_count < 0:
            raise ValueError("expected_frame_count must be a nonnegative integer")
        relative = Path(relative_path)
        stream_name = relative.name.split(".")[0]
        before = validate_regular_file(run_directory, relative)
        if before.status is not ValidationStatus.VALID:
            return self._result(before, before.status, before.detail)
        if before.size_bytes == 0:
            return self._invalid(before, "video is empty", stream_name)

        descriptors: list[int] = []
        file_fd: int | None = None
        semantic: VideoValidationResult
        initial_stat: os.stat_result | None = None
        try:
            parts = tuple(part for part in relative.parts if part != ".")
            if relative.is_absolute() or ".." in parts or not parts:
                return self._invalid(before, "path escapes run directory", stream_name)
            root_fd = os.open(run_directory, _DIRECTORY_FLAGS)
            descriptors.append(root_fd)
            parent_fd = root_fd
            for name in parts[:-1]:
                entry = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
                if not stat.S_ISDIR(entry.st_mode) or stat.S_ISLNK(entry.st_mode):
                    return self._invalid(before, "unsafe video path", stream_name)
                child_fd = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
                if not _same_entry(entry, os.fstat(child_fd)):
                    os.close(child_fd)
                    return self._invalid(before, "video changed during semantic validation", stream_name)
                descriptors.append(child_fd)
                parent_fd = child_fd
            entry = os.stat(parts[-1], dir_fd=parent_fd, follow_symlinks=False)
            file_fd = os.open(parts[-1], _FILE_FLAGS, dir_fd=parent_fd)
            initial_stat = os.fstat(file_fd)
            if (
                not stat.S_ISREG(initial_stat.st_mode)
                or initial_stat.st_nlink != 1
                or not _same_entry(entry, initial_stat)
            ):
                return self._invalid(before, "unsafe video path", stream_name)
            probe_path = f"/proc/self/fd/{file_fd}"
            probe = self._command_runner(
                (
                    "ffprobe", "-v", "error", "-print_format", "json", "-show_streams",
                    "-count_frames", probe_path,
                ),
                capture_output=True,
                text=True,
                check=False,
                pass_fds=(file_fd,),
            )
            if probe.returncode != 0:
                semantic = self._invalid(before, "ffprobe could not read video", stream_name)
            else:
                try:
                    document = json.loads(probe.stdout)
                    streams = document["streams"]
                except (json.JSONDecodeError, KeyError, TypeError):
                    semantic = self._invalid(before, "ffprobe returned invalid JSON", stream_name)
                else:
                    videos = [item for item in streams if item.get("codec_type") == "video"]
                    if len(videos) != 1:
                        semantic = self._invalid(before, "video must contain exactly one video stream", stream_name)
                    elif len(streams) != 1:
                        semantic = self._invalid(before, "video contains an unexpected audio or data stream", stream_name)
                    else:
                        video = videos[0]
                        semantic = self._validate_probe(before, stream_name, video, expected_frame_count, outcome)
                        if semantic.status is ValidationStatus.VALID:
                            decoded = self._command_runner(
                                (
                                    "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
                                    "-i", probe_path, "-map", "0:v:0", "-f", "null", "-",
                                ),
                                capture_output=True,
                                text=True,
                                check=False,
                                pass_fds=(file_fd,),
                            )
                            if decoded.returncode != 0:
                                semantic = self._invalid(
                                    before, "video failed full decode", stream_name,
                                    stream=video, frame_count=semantic.frame_count,
                                )
        except (FileNotFoundError, PermissionError, OSError):
            semantic = self._invalid(before, "video changed during semantic validation", stream_name)
        finally:
            changed = False
            if file_fd is not None and initial_stat is not None:
                try:
                    changed = not _same_snapshot(initial_stat, os.fstat(file_fd))
                except OSError:
                    changed = True
                os.close(file_fd)
            for descriptor in reversed(descriptors):
                os.close(descriptor)

        after = validate_regular_file(run_directory, relative)
        if (
            changed
            or after.status is not ValidationStatus.VALID
            or after.size_bytes != before.size_bytes
            or after.sha256 != before.sha256
        ):
            return VideoValidationResult(
                ValidationStatus.INVALID,
                None,
                None,
                "video changed during semantic validation",
            )
        return semantic

    def _validate_probe(
        self,
        filesystem: ValidationResult,
        stream_name: str,
        video: dict[str, Any],
        expected_frame_count: int,
        outcome: str,
    ) -> VideoValidationResult:
        if video.get("codec_name") != "h264":
            return self._invalid(filesystem, "video codec must be h264", stream_name, stream=video)
        if video.get("pix_fmt") != "yuv420p":
            return self._invalid(filesystem, "video pixel format must be yuv420p", stream_name, stream=video)
        if video.get("avg_frame_rate") != f"{self.fps}/1":
            return self._invalid(filesystem, "video average frame rate must be 20/1", stream_name, stream=video)
        if (video.get("width"), video.get("height")) != (self.width_px, self.height_px):
            return self._invalid(filesystem, "video dimensions are wrong", stream_name, stream=video)
        count_value = video.get("nb_read_frames")
        try:
            frame_count = int(count_value)
        except (TypeError, ValueError):
            return self._invalid(filesystem, "decoded frame count is not trustworthy", stream_name, stream=video)
        if frame_count <= 0:
            return self._invalid(filesystem, "video has zero decoded frames", stream_name, stream=video, frame_count=frame_count)
        if frame_count > expected_frame_count:
            return self._invalid(filesystem, "video frame count exceeds expected count", stream_name, stream=video, frame_count=frame_count)
        if outcome == "completed" and frame_count != expected_frame_count:
            return self._invalid(filesystem, "completed video frame count differs from expected count", stream_name, stream=video, frame_count=frame_count)
        detail = "valid H.264 yuv420p 20-FPS video with full decode"
        return self._result(
            filesystem,
            ValidationStatus.VALID,
            detail,
            diagnostic=VideoDiagnostic(stream_name, "video_valid", detail),
            stream=video,
            frame_count=frame_count,
        )


class VideoStreamRecorder:
    """Pair one ROS Image/FrameMetadata stream and feed raw frames to FFmpeg."""

    def __init__(
        self,
        run_directory: Path | str,
        *,
        run_id: str,
        stream: str,
        width_px: int = WIDTH_PX,
        height_px: int = HEIGHT_PX,
        fps: int = FPS,
        encoding: str = ENCODING,
        process_factory: Callable[..., _Process] = _spawn,
        command_runner: Callable[..., subprocess.CompletedProcess[str]] = _run_command,
        validator: VideoValidator | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        diagnostic_sink: Callable[[VideoDiagnostic], None] | None = None,
    ) -> None:
        if not run_id:
            raise ValueError("run_id must not be empty")
        if stream not in STREAMS:
            raise ValueError("stream must be onboard or observer")
        if (width_px, height_px, fps, encoding) != (WIDTH_PX, HEIGHT_PX, FPS, ENCODING):
            raise ValueError("recording configuration must equal frozen 320x240 rgb8 at 20 FPS")
        self.run_directory = Path(run_directory).resolve()
        self.run_id = run_id
        self.stream = stream
        self.width_px = width_px
        self.height_px = height_px
        self.fps = fps
        self.encoding = encoding
        self._process_factory = process_factory
        self._command_runner = command_runner
        self._validator = validator or VideoValidator(command_runner=command_runner)
        self._monotonic = monotonic
        self._diagnostic_sink = diagnostic_sink or (lambda _item: None)
        self._process: _Process | None = None
        self._log_stream: Any | None = None
        self._root_fd: int | None = None
        self._video_fd: int | None = None
        self._root_identity: os.stat_result | None = None
        self._video_identity: os.stat_result | None = None
        self._pending_image: Any | None = None
        self._pending_metadata: Any | None = None
        self._frame_count = 0
        self._last_timestamp_ns: int | None = None
        self._failed = False
        self._diagnostics: list[VideoDiagnostic] = []

    @property
    def partial_path(self) -> Path:
        return self.run_directory / f"video/{self.stream}.mp4.partial"

    @property
    def final_path(self) -> Path:
        return self.run_directory / f"video/{self.stream}.mp4"

    @property
    def log_path(self) -> Path:
        return self.run_directory / f"logs/docker/ffmpeg-{self.stream}.log.partial"

    @property
    def frame_count(self) -> int:
        return self._frame_count

    @property
    def last_timestamp_ns(self) -> int | None:
        return self._last_timestamp_ns

    @property
    def failed(self) -> bool:
        return self._failed

    @property
    def diagnostics(self) -> tuple[VideoDiagnostic, ...]:
        return tuple(self._diagnostics)

    @property
    def pending_counts(self) -> tuple[int, int]:
        return (int(self._pending_image is not None), int(self._pending_metadata is not None))

    @property
    def is_ready(self) -> bool:
        return self._process is not None and self._process.poll() is None and not self._failed

    def command(self) -> tuple[str, ...]:
        return (
            "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
            "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "320x240",
            "-framerate", "20", "-i", "pipe:0", "-an", "-c:v", "libx264",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-f", "mp4",
            str(self.partial_path),
        )

    def _preflight(self) -> None:
        encoders = self._command_runner(
            ("ffmpeg", "-encoders"), capture_output=True, text=True, check=False
        )
        probe = self._command_runner(
            ("ffprobe", "-version"), capture_output=True, text=True, check=False
        )
        if encoders.returncode != 0 or "libx264" not in encoders.stdout or probe.returncode != 0:
            raise RuntimeError("FFmpeg/ffprobe preflight failed")

    def _prepare_output(self) -> None:
        try:
            root_before = os.stat(self.run_directory, follow_symlinks=False)
            if not stat.S_ISDIR(root_before.st_mode):
                raise _unsafe_path()
            root_fd = os.open(self.run_directory, _DIRECTORY_FLAGS)
            root_opened = os.fstat(root_fd)
            if not _same_entry(root_before, root_opened):
                os.close(root_fd)
                raise _unsafe_path()
            video_fd, video_identity = _open_or_create_directory(root_fd, "video")
            for name in (f"{self.stream}.mp4", f"{self.stream}.mp4.partial"):
                try:
                    os.stat(name, dir_fd=video_fd, follow_symlinks=False)
                except FileNotFoundError:
                    continue
                os.close(video_fd); os.close(root_fd)
                raise FileExistsError(f"refusing existing video output: {name}")
        except (FileExistsError, RuntimeError):
            raise
        except OSError as error:
            raise _unsafe_path() from error
        self._root_fd, self._video_fd = root_fd, video_fd
        self._root_identity, self._video_identity = root_opened, video_identity

    def _paths_safe(self) -> bool:
        if None in (self._root_fd, self._video_fd, self._root_identity, self._video_identity):
            return False
        try:
            root_now = os.stat(self.run_directory, follow_symlinks=False)
            video_now = os.stat("video", dir_fd=self._root_fd, follow_symlinks=False)
            return (
                _same_entry(root_now, self._root_identity)
                and _same_entry(os.fstat(self._root_fd), self._root_identity)
                and _same_entry(video_now, self._video_identity)
                and _same_entry(os.fstat(self._video_fd), self._video_identity)
            )
        except OSError:
            return False

    def _open_log_for_append(self) -> Any:
        if self._root_fd is None:
            raise RuntimeError("unsafe recorder log path")
        held: list[int] = []
        log_fd: int | None = None
        try:
            parent_fd = self._root_fd
            for name in ("logs", "docker"):
                try:
                    descriptor, _identity = _open_or_create_directory(parent_fd, name)
                except RuntimeError as error:
                    raise RuntimeError("unsafe recorder log path") from error
                held.append(descriptor)
                parent_fd = descriptor
            name = f"ffmpeg-{self.stream}.log.partial"
            try:
                log_fd = os.open(name, _LOG_FLAGS, 0o644, dir_fd=parent_fd)
                opened = os.fstat(log_fd)
                current = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
            except OSError as error:
                raise RuntimeError("unsafe recorder log path") from error
            if (
                not stat.S_ISREG(opened.st_mode)
                or opened.st_nlink != 1
                or not _same_entry(opened, current)
            ):
                raise RuntimeError("unsafe recorder log path")
            stream = os.fdopen(log_fd, "ab", buffering=0)
            log_fd = None
            return stream
        finally:
            if log_fd is not None:
                os.close(log_fd)
            for descriptor in reversed(held):
                os.close(descriptor)

    def start(self) -> None:
        if self._process is not None:
            raise RuntimeError("video recorder has already been started")
        self._preflight()
        self._prepare_output()
        try:
            log_stream = self._open_log_for_append()
            self._process = self._process_factory(
                self.command(), stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                stderr=log_stream, shell=False,
            )
        except BaseException:
            if "log_stream" in locals():
                log_stream.close()
            self._close_resources()
            raise
        self._log_stream = log_stream
        if self._process.stdin is None:
            self._close_resources()
            raise RuntimeError("FFmpeg process did not expose stdin")

    def _record_failure(self, event: str, detail: str, sim_timestamp_ns: int | None = None) -> None:
        self._failed = True
        diagnostic = VideoDiagnostic(self.stream, event, detail, sim_timestamp_ns)
        self._diagnostics.append(diagnostic)
        self._diagnostic_sink(diagnostic)

    def _reject(self, detail: str, sim_timestamp_ns: int | None = None) -> None:
        self._record_failure("frame_rejected", detail, sim_timestamp_ns)
        raise ValueError(detail)

    def _require_active(self) -> None:
        if self._failed:
            raise RuntimeError("video recorder failed closed")
        if self._process is None or self._process.stdin is None:
            raise RuntimeError("video recorder has not been started")
        if self._process.poll() is not None:
            self._record_failure("ffmpeg_exited", "FFmpeg exited before finalization")
            raise RuntimeError("FFmpeg exited before finalization")

    def accept_image(self, image: Any) -> None:
        self._require_active()
        if self._pending_image is not None:
            self._reject("new unmatched image would replace pending unmatched image")
        try:
            timestamp_ns = _timestamp_ns(image.header.stamp)
        except (AttributeError, ValueError) as error:
            self._reject(f"invalid image stamp: {error}")
        if image.width % 2 or image.height % 2:
            self._reject("image dimensions must be even", timestamp_ns)
        if (image.width, image.height) != (self.width_px, self.height_px):
            self._reject("image dimensions changed or differ from 320x240", timestamp_ns)
        if image.encoding != self.encoding:
            self._reject("image encoding must be rgb8", timestamp_ns)
        if image.step != STEP_BYTES:
            self._reject("image step must be 960", timestamp_ns)
        if len(image.data) != PAYLOAD_BYTES:
            self._reject("image payload must contain exactly 230400 bytes", timestamp_ns)
        self._pending_image = image
        self._pair_if_ready()

    def accept_metadata(self, metadata: Any) -> None:
        self._require_active()
        if self._pending_metadata is not None:
            self._reject("new unmatched metadata would replace pending unmatched metadata")
        try:
            timestamp_ns = _timestamp_ns(metadata.sim_timestamp)
        except (AttributeError, ValueError) as error:
            self._reject(f"invalid metadata timestamp: {error}")
        if metadata.run_id != self.run_id:
            self._reject("metadata run_id does not match configured run_id", timestamp_ns)
        if metadata.stream != self.stream:
            self._reject("metadata stream does not match configured stream", timestamp_ns)
        expected_id = self._frame_count
        if not isinstance(metadata.frame_id, int) or isinstance(metadata.frame_id, bool) or metadata.frame_id != expected_id:
            self._reject(f"frame ID must be contiguous from zero; expected {expected_id}", timestamp_ns)
        if self._last_timestamp_ns is not None:
            delta = timestamp_ns - self._last_timestamp_ns
            if delta <= 0:
                self._reject("simulation timestamp must be strictly increasing", timestamp_ns)
            if delta != FRAME_INTERVAL_NS:
                self._reject("simulation timestamps must be exactly 50 ms apart", timestamp_ns)
        self._pending_metadata = metadata
        self._pair_if_ready()

    def _pair_if_ready(self) -> None:
        if self._pending_image is None or self._pending_metadata is None:
            return
        image_timestamp = _timestamp_ns(self._pending_image.header.stamp)
        metadata_timestamp = _timestamp_ns(self._pending_metadata.sim_timestamp)
        if image_timestamp != metadata_timestamp:
            self._reject("image header stamp must exactly match metadata stamp")
        payload = bytes(self._pending_image.data)
        try:
            self._process.stdin.write(payload)
        except (BrokenPipeError, OSError) as error:
            self._record_failure("ffmpeg_write_failed", f"FFmpeg pipe write failed: {error}", image_timestamp)
            raise RuntimeError("FFmpeg pipe write failed") from error
        self._pending_image = None
        self._pending_metadata = None
        self._frame_count += 1
        self._last_timestamp_ns = image_timestamp

    def finalize(self, deadline: float, *, outcome: str) -> VideoFinalization:
        process = self._process
        if process is None:
            raise RuntimeError("video recorder has not been started")
        if self._pending_image is not None or self._pending_metadata is not None:
            self._record_failure("unmatched_frame", "finalization found an unmatched image or metadata item")
        if process.stdin is not None and not process.stdin.closed:
            try:
                process.stdin.close()
            except (BrokenPipeError, OSError) as error:
                self._record_failure(
                    "ffmpeg_close_failed",
                    f"FFmpeg stdin close failed: {error}",
                )

        signals: list[str] = []
        if process.poll() is None:
            phases: tuple[int | None, ...] = (None, signal.SIGTERM, signal.SIGKILL)
            for index, signum in enumerate(phases):
                if process.poll() is not None:
                    break
                if signum is not None:
                    try:
                        process.send_signal(signum)
                    except ProcessLookupError:
                        if process.poll() is None:
                            raise
                    signals.append(signal.Signals(signum).name)
                remaining = max(0.0, deadline - self._monotonic())
                timeout = remaining / (len(phases) - index)
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue

        returncode = process.poll()
        exited = returncode is not None
        if not exited:
            detail = f"FFmpeg did not exit before deadline after {', '.join(signals)}"
            self._record_failure("ffmpeg_timeout", detail)
            self._close_resources()
            return VideoFinalization(False, None, tuple(signals), bool(signals), False, detail)
        if returncode != 0:
            detail = f"FFmpeg exited with return code {returncode}"
            self._record_failure("ffmpeg_failed", detail)
            self._close_resources()
            return VideoFinalization(True, returncode, tuple(signals), bool(signals), False, detail)
        if self._failed:
            detail = "video recorder failed before publication"
            self._close_resources()
            return VideoFinalization(True, 0, tuple(signals), bool(signals), False, detail)
        if not self._paths_safe():
            detail = "unsafe video path at finalization"
            self._record_failure("unsafe_video_path", detail)
            self._close_resources()
            return VideoFinalization(True, 0, tuple(signals), bool(signals), False, detail)

        relative = f"video/{self.stream}.mp4.partial"
        validation = self._validator.validate(
            self.run_directory, relative, expected_frame_count=self._frame_count, outcome=outcome
        )
        if validation.status is not ValidationStatus.VALID:
            self._record_failure("video_invalid", validation.detail)
            self._close_resources()
            return VideoFinalization(True, 0, tuple(signals), bool(signals), False, validation.detail)
        try:
            assert self._video_fd is not None
            _rename_noreplace(self._video_fd, f"{self.stream}.mp4.partial", f"{self.stream}.mp4")
        except (FileExistsError, OSError) as error:
            detail = f"video rename conflict or failure: {error}"
            self._record_failure("video_publish_failed", detail)
            return VideoFinalization(True, 0, tuple(signals), bool(signals), False, detail)
        finally:
            self._close_resources()
        return VideoFinalization(True, 0, tuple(signals), bool(signals), True, "video finalized")

    def _close_resources(self) -> None:
        if self._log_stream is not None:
            self._log_stream.close()
            self._log_stream = None
        for name in ("_video_fd", "_root_fd"):
            descriptor = getattr(self, name)
            if descriptor is not None:
                os.close(descriptor)
                setattr(self, name, None)


__all__ = [
    "ENCODING", "FPS", "FRAME_INTERVAL_NS", "HEIGHT_PX", "PAYLOAD_BYTES", "STEP_BYTES",
    "STREAMS", "VideoDiagnostic", "VideoFinalization", "VideoStreamRecorder",
    "VideoValidationResult", "VideoValidator", "WIDTH_PX",
]
