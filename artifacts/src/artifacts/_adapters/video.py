"""Simulation-time RGB frame pairing, FFmpeg control, and MP4 validation."""

from collections.abc import Callable, Sequence
import ctypes
from dataclasses import dataclass
import errno
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import threading
import time
from typing import Any, Protocol

from ..validation import ValidationResult, ValidationStatus


FRAME_INTERVAL_NS = 50_000_000
WIDTH_PX = 320
HEIGHT_PX = 240
FPS = 20
ENCODING = "rgb8"
STEP_BYTES = WIDTH_PX * 3
PAYLOAD_BYTES = WIDTH_PX * HEIGHT_PX * 3
STREAMS = ("onboard", "observer")
_RECOVERY_RESERVE_SECONDS = 0.5
_RECOVERY_LINK_MARGIN_SECONDS = 0.1
_RECOVERY_COPY_CHUNK_BYTES = 1024 * 1024
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
_OUTPUT_FLAGS = (
    os.O_RDWR
    | os.O_TMPFILE
    | getattr(os, "O_CLOEXEC", 0)
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
    kwargs.pop("timeout", None)
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
    descriptor: int | None = None
    try:
        before = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        if not stat.S_ISDIR(before.st_mode) or stat.S_ISLNK(before.st_mode):
            raise _unsafe_path()
        descriptor = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
        opened = os.fstat(descriptor)
        if not _same_entry(before, opened):
            raise _unsafe_path()
        return descriptor, opened
    except RuntimeError:
        if descriptor is not None:
            os.close(descriptor)
        raise
    except OSError as error:
        if descriptor is not None:
            try:
                os.close(descriptor)
            except OSError:
                pass
        raise _unsafe_path() from error


def _link_descriptor_noreplace(descriptor: int, parent_fd: int, destination: str) -> None:
    """Publish the exact retained inode without replacing a directory entry."""
    libc = ctypes.CDLL(None, use_errno=True)
    linkat = getattr(libc, "linkat", None)
    if linkat is None:
        raise OSError("linkat is required for descriptor-bound publication")
    linkat.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
    linkat.restype = ctypes.c_int
    source = os.fsencode(f"/proc/self/fd/{descriptor}")
    if linkat(-100, source, parent_fd, os.fsencode(destination), 0x400) != 0:
        error_number = ctypes.get_errno()
        if error_number == errno.EEXIST:
            raise FileExistsError(destination)
        raise OSError(error_number, os.strerror(error_number), destination)


def _watch_descriptors(descriptors: Sequence[int]) -> int:
    """Watch retained file/directory inodes for transient mutation or ABA moves."""
    libc = ctypes.CDLL(None, use_errno=True)
    init = getattr(libc, "inotify_init1", None)
    add = getattr(libc, "inotify_add_watch", None)
    if init is None or add is None:
        raise OSError("inotify is required for stable video validation")
    init.argtypes = [ctypes.c_int]
    init.restype = ctypes.c_int
    watch_fd = init(getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NONBLOCK", 0))
    if watch_fd < 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))
    add.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_uint32]
    add.restype = ctypes.c_int
    # IN_MODIFY | IN_CLOSE_WRITE | IN_DELETE_SELF | IN_MOVE_SELF
    for descriptor in descriptors:
        if add(watch_fd, os.fsencode(f"/proc/self/fd/{descriptor}"), 0x2 | 0x8 | 0x400 | 0x800) < 0:
            error_number = ctypes.get_errno()
            os.close(watch_fd)
            raise OSError(error_number, os.strerror(error_number))
    return watch_fd


def _watch_changed(watch_fd: int) -> bool:
    try:
        return bool(os.read(watch_fd, 64 * 1024))
    except BlockingIOError:
        return False
    except OSError:
        return True


class _SemanticComplete(Exception):
    """Internal control flow: semantic result is ready; stability checks remain."""


class VideoValidator:
    """Read-only semantic validation of one stable, hardened MP4 snapshot."""

    def __init__(
        self,
        *,
        width_px: int = WIDTH_PX,
        height_px: int = HEIGHT_PX,
        fps: int = FPS,
        command_runner: Callable[..., subprocess.CompletedProcess[str]] = _run_command,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.width_px = width_px
        self.height_px = height_px
        self.fps = fps
        self._command_runner = command_runner
        self._monotonic = monotonic

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
        deadline: float,
        descriptor: int | None = None,
    ) -> VideoValidationResult:
        canonical_outcome = outcome.upper()
        if canonical_outcome not in {"COMPLETED", "FAILED", "ABORTED"}:
            raise ValueError("outcome must be COMPLETED, FAILED, or ABORTED")
        if not isinstance(expected_frame_count, int) or isinstance(expected_frame_count, bool) or expected_frame_count < 0:
            raise ValueError("expected_frame_count must be a nonnegative integer")
        relative = Path(relative_path)
        stream_name = relative.name.split(".")[0]
        empty = ValidationResult(ValidationStatus.INVALID, None, None, "video is invalid")
        held: list[int] = []
        file_fd = descriptor
        owns_file = descriptor is None
        semantic: VideoValidationResult | None = None
        initial_stat: os.stat_result | None = None
        initial_digest: str | None = None
        parent_fd: int | None = None
        parent_snapshot: os.stat_result | None = None
        watch_fd: int | None = None
        filename = ""
        changed = False
        deadline_expired = False
        try:
            parts = tuple(part for part in relative.parts if part != ".")
            if relative.is_absolute() or ".." in parts or not parts:
                return self._invalid(empty, "path escapes run directory", stream_name)
            root_fd = os.open(run_directory, _DIRECTORY_FLAGS)
            held.append(root_fd)
            parent_fd = root_fd
            for name in parts[:-1]:
                entry = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
                if not stat.S_ISDIR(entry.st_mode) or stat.S_ISLNK(entry.st_mode):
                    return self._invalid(empty, "unsafe video path", stream_name)
                child_fd = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent_fd)
                held.append(child_fd)
                if not _same_entry(entry, os.fstat(child_fd)):
                    return self._invalid(empty, "video changed during semantic validation", stream_name)
                parent_fd = child_fd
            filename = parts[-1]
            if file_fd is None:
                entry = os.stat(filename, dir_fd=parent_fd, follow_symlinks=False)
                file_fd = os.open(filename, _FILE_FLAGS, dir_fd=parent_fd)
            initial_stat = os.fstat(file_fd)
            expected_links = 1 if owns_file else 0
            if not stat.S_ISREG(initial_stat.st_mode) or initial_stat.st_nlink != expected_links:
                return self._invalid(empty, "unsafe video path", stream_name)
            if owns_file and not _same_entry(entry, initial_stat):
                return self._invalid(empty, "unsafe video path", stream_name)
            parent_snapshot = os.fstat(parent_fd)
            watch_fd = _watch_descriptors((*held, file_fd))
            initial_digest = self._hash_descriptor(file_fd, deadline)
            filesystem = ValidationResult(
                ValidationStatus.VALID,
                initial_stat.st_size,
                initial_digest,
                "valid regular file",
            )
            if initial_stat.st_size == 0:
                semantic = self._invalid(filesystem, "video is empty", stream_name)
                raise _SemanticComplete
            probe_path = f"/proc/self/fd/{file_fd}"
            remaining = self._remaining(deadline)
            probe = self._command_runner(
                (
                    "ffprobe", "-v", "error", "-print_format", "json", "-show_streams",
                    "-count_frames", probe_path,
                ),
                capture_output=True,
                text=True,
                check=False,
                pass_fds=(file_fd,),
                timeout=remaining,
            )
            self._remaining(deadline)
            if probe.returncode != 0:
                semantic = self._invalid(filesystem, "ffprobe could not read video", stream_name)
            else:
                try:
                    document = json.loads(probe.stdout)
                    if not isinstance(document, dict):
                        raise TypeError
                    streams = document["streams"]
                    if not isinstance(streams, list) or not all(isinstance(item, dict) for item in streams):
                        raise TypeError
                except (json.JSONDecodeError, KeyError, TypeError):
                    semantic = self._invalid(filesystem, "ffprobe returned invalid JSON", stream_name)
                else:
                    videos = [item for item in streams if item.get("codec_type") == "video"]
                    if len(videos) != 1:
                        semantic = self._invalid(filesystem, "video must contain exactly one video stream", stream_name)
                    elif len(streams) != 1:
                        semantic = self._invalid(filesystem, "video contains an unexpected audio or data stream", stream_name)
                    else:
                        video = videos[0]
                        semantic = self._validate_probe(
                            filesystem, stream_name, video, expected_frame_count, canonical_outcome
                        )
                        if semantic.status is ValidationStatus.VALID:
                            remaining = self._remaining(deadline)
                            decoded = self._command_runner(
                                (
                                    "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
                                    "-xerror", "-err_detect", "explode", "-i", probe_path,
                                    "-map", "0:v:0", "-f", "null", "-",
                                ),
                                capture_output=True,
                                text=True,
                                check=False,
                                pass_fds=(file_fd,),
                                timeout=remaining,
                            )
                            self._remaining(deadline)
                            if decoded.returncode != 0 or decoded.stderr.strip():
                                semantic = self._invalid(
                                    filesystem, "video failed full decode", stream_name,
                                    stream=video, frame_count=semantic.frame_count,
                                )
        except _SemanticComplete:
            pass
        except subprocess.TimeoutExpired:
            semantic = self._invalid(empty, "video validation exceeded caller deadline", stream_name)
        except FileNotFoundError:
            if descriptor is None and initial_stat is None:
                semantic = VideoValidationResult(
                    ValidationStatus.MISSING, None, None, "path is missing"
                )
            else:
                semantic = self._invalid(
                    empty, "video changed during semantic validation", stream_name
                )
        except (PermissionError, OSError, ValueError, AttributeError):
            semantic = self._invalid(empty, "video changed during semantic validation", stream_name)
        except Exception:
            semantic = self._invalid(empty, "video validation failed unexpectedly", stream_name)
        finally:
            if file_fd is not None and initial_stat is not None:
                try:
                    self._remaining(deadline)
                    final_stat = os.fstat(file_fd)
                    changed = (
                        not _same_snapshot(initial_stat, final_stat)
                        or self._hash_descriptor(file_fd, deadline) != initial_digest
                        or parent_fd is None
                        or parent_snapshot is None
                        or not _same_snapshot(parent_snapshot, os.fstat(parent_fd))
                        or (watch_fd is not None and _watch_changed(watch_fd))
                        or (
                            owns_file
                            and not _same_entry(
                                os.stat(
                                    filename,
                                    dir_fd=parent_fd,
                                    follow_symlinks=False,
                                ),
                                final_stat,
                            )
                        )
                    )
                except subprocess.TimeoutExpired:
                    deadline_expired = True
                except OSError:
                    changed = True
            if watch_fd is not None:
                try:
                    os.close(watch_fd)
                except OSError:
                    pass
            if owns_file and file_fd is not None:
                try:
                    os.close(file_fd)
                except OSError:
                    pass
            for held_descriptor in reversed(held):
                try:
                    os.close(held_descriptor)
                except OSError:
                    pass

        if deadline_expired:
            return self._invalid(
                empty, "video validation exceeded caller deadline", stream_name
            )
        if changed:
            return VideoValidationResult(
                ValidationStatus.INVALID,
                None,
                None,
                "video changed during semantic validation",
            )
        return semantic or self._invalid(empty, "video validation failed unexpectedly", stream_name)

    def _remaining(self, deadline: float) -> float:
        remaining = deadline - self._monotonic()
        if remaining <= 0:
            raise subprocess.TimeoutExpired("video validation", max(0.0, remaining))
        return remaining

    def _hash_descriptor(self, descriptor: int, deadline: float) -> str:
        digest = hashlib.sha256()
        offset = 0
        while True:
            self._remaining(deadline)
            chunk = os.pread(descriptor, 1024 * 1024, offset)
            self._remaining(deadline)
            if not chunk:
                break
            digest.update(chunk)
            offset += len(chunk)
        return digest.hexdigest()

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
        if outcome == "COMPLETED" and frame_count != expected_frame_count:
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
        expected_frame_count: int = 40,
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
        if (
            not isinstance(expected_frame_count, int)
            or isinstance(expected_frame_count, bool)
            or expected_frame_count <= 0
        ):
            raise ValueError("expected_frame_count must be a positive integer")
        if (width_px, height_px, fps, encoding) != (WIDTH_PX, HEIGHT_PX, FPS, ENCODING):
            raise ValueError("recording configuration must equal frozen 320x240 rgb8 at 20 FPS")
        self.run_directory = Path(run_directory).resolve()
        self.run_id = run_id
        self.stream = stream
        self._expected_frame_count = expected_frame_count
        self.width_px = width_px
        self.height_px = height_px
        self.fps = fps
        self.encoding = encoding
        self._process_factory = process_factory
        self._command_runner = command_runner
        self._validator = validator or VideoValidator(
            command_runner=command_runner, monotonic=monotonic
        )
        self._monotonic = monotonic
        self._diagnostic_sink = diagnostic_sink or (lambda _item: None)
        self._process: _Process | None = None
        self._log_stream: Any | None = None
        self._root_fd: int | None = None
        self._video_fd: int | None = None
        self._output_fd: int | None = None
        self._readonly_output_fd: int | None = None
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
    def expected_frame_count(self) -> int:
        return self._expected_frame_count

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

    def command(self, output_descriptor: int | None = None) -> tuple[str, ...]:
        descriptor = self._output_fd if output_descriptor is None else output_descriptor
        if descriptor is None:
            raise RuntimeError("video output has not been reserved")
        return (
            "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error", "-y",
            "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "320x240",
            "-framerate", "20", "-i", "pipe:0", "-an", "-c:v", "libx264",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart", "-f", "mp4",
            f"/proc/self/fd/{descriptor}",
        )

    def _bounded_call(
        self, operation: Callable[[], Any], deadline: float, label: str
    ) -> Any:
        completed = threading.Event()
        result: list[Any] = []
        failures: list[BaseException] = []

        def invoke() -> None:
            try:
                result.append(operation())
            except BaseException as error:
                failures.append(error)
            finally:
                completed.set()

        threading.Thread(target=invoke, daemon=True).start()
        remaining = max(0.0, deadline - self._monotonic())
        if not completed.wait(remaining):
            raise subprocess.TimeoutExpired(label, remaining)
        if deadline - self._monotonic() <= 0:
            raise subprocess.TimeoutExpired(label, remaining)
        if failures:
            raise failures[0]
        return result[0]

    def _preflight(self, deadline: float) -> None:
        encoders = self._bounded_call(
            lambda: self._command_runner(
                ("ffmpeg", "-encoders"), capture_output=True, text=True,
                check=False, timeout=max(0.0, deadline - self._monotonic()),
            ),
            deadline,
            "ffmpeg -encoders",
        )
        probe = self._bounded_call(
            lambda: self._command_runner(
                ("ffprobe", "-version"), capture_output=True, text=True,
                check=False, timeout=max(0.0, deadline - self._monotonic()),
            ),
            deadline,
            "ffprobe -version",
        )
        encoder_names = {
            fields[1]
            for line in encoders.stdout.splitlines()
            if len(fields := line.split()) >= 2 and len(fields[0]) == 6
        }
        if encoders.returncode != 0 or "libx264" not in encoder_names or probe.returncode != 0:
            raise RuntimeError("FFmpeg/ffprobe preflight failed")

    def _prepare_output(self) -> None:
        root_fd: int | None = None
        video_fd: int | None = None
        output_fd: int | None = None
        committed = False
        try:
            root_before = os.stat(self.run_directory, follow_symlinks=False)
            if not stat.S_ISDIR(root_before.st_mode):
                raise _unsafe_path()
            root_fd = os.open(self.run_directory, _DIRECTORY_FLAGS)
            root_opened = os.fstat(root_fd)
            if not _same_entry(root_before, root_opened):
                raise _unsafe_path()
            video_fd, video_identity = _open_or_create_directory(root_fd, "video")
            for output_name in (
                f"{self.stream}.mp4",
                f"{self.stream}.mp4.partial",
            ):
                try:
                    os.stat(output_name, dir_fd=video_fd, follow_symlinks=False)
                except FileNotFoundError:
                    continue
                raise FileExistsError(
                    f"refusing existing video output: {output_name}"
                )
            try:
                output_fd = os.open(".", _OUTPUT_FLAGS, 0o600, dir_fd=video_fd)
            except OSError as error:
                if error.errno in {
                    errno.EINVAL,
                    errno.EISDIR,
                    errno.ENOTSUP,
                    errno.EOPNOTSUPP,
                }:
                    raise RuntimeError(
                        "video output filesystem does not support O_TMPFILE"
                    ) from error
                raise
            output_identity = os.fstat(output_fd)
            if (
                not stat.S_ISREG(output_identity.st_mode)
                or output_identity.st_nlink != 0
                or fcntl.fcntl(output_fd, fcntl.F_GETFL) & os.O_ACCMODE
                != os.O_RDWR
            ):
                raise _unsafe_path()
            self._root_fd, self._video_fd, self._output_fd = root_fd, video_fd, output_fd
            self._root_identity, self._video_identity = root_opened, video_identity
            committed = True
        except (FileExistsError, RuntimeError):
            raise
        except OSError as error:
            raise _unsafe_path() from error
        finally:
            if not committed:
                for descriptor in (output_fd, video_fd, root_fd):
                    if descriptor is not None:
                        try:
                            os.close(descriptor)
                        except OSError:
                            pass

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

    def start(self, *, deadline: float) -> None:
        if self._process is not None:
            raise RuntimeError("video recorder has already been started")
        self._preflight(deadline)
        self._prepare_output()
        try:
            log_stream = self._open_log_for_append()
            self._process, self._log_stream = self._spawn_with_deadline(
                deadline, log_stream
            )
            if self._process.stdin is None:
                raise RuntimeError("FFmpeg process did not expose stdin")
        except BaseException:
            process = self._process
            if process is not None:
                self._kill_started_process(process, deadline)
                self._process = None
            self._close_resources()
            raise

    def _spawn_with_deadline(self, deadline: float, log_stream: Any) -> tuple[_Process, Any]:
        assert self._output_fd is not None
        try:
            output_fd = os.dup(self._output_fd)
        except BaseException:
            try:
                log_stream.close()
            except Exception:
                pass
            raise
        handoff = threading.Condition()
        state: dict[str, Any] = {
            "status": "spawning",
            "process": None,
            "failure": None,
            "cleaned": False,
        }
        started_remaining = max(0.0, deadline - self._monotonic())
        cleanup_reserve = min(0.25, started_remaining / 2)
        claim_deadline = deadline - cleanup_reserve

        def spawn_process() -> None:
            process: _Process | None = None
            try:
                process = self._process_factory(
                    self.command(output_fd),
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=log_stream,
                    shell=False,
                    pass_fds=(output_fd,),
                    bufsize=0,
                    timeout=max(0.0, deadline - self._monotonic()),
                )
            except BaseException as error:
                try:
                    os.close(output_fd)
                except OSError:
                    pass
                try:
                    log_stream.close()
                except Exception:
                    pass
                with handoff:
                    state["failure"] = error
                    state["status"] = "failed"
                    state["cleaned"] = True
                    handoff.notify_all()
                return
            try:
                os.close(output_fd)
            except OSError:
                pass
            with handoff:
                if state["status"] == "cancelled":
                    claimed = False
                else:
                    state["process"] = process
                    state["status"] = "ready"
                    handoff.notify_all()
                    while state["status"] == "ready":
                        handoff.wait()
                    claimed = state["status"] == "claimed"
            if claimed:
                return
            try:
                self._terminate_worker_process(process)
            finally:
                try:
                    log_stream.close()
                except Exception:
                    pass
                with handoff:
                    state["cleaned"] = True
                    handoff.notify_all()

        threading.Thread(target=spawn_process, daemon=True).start()
        with handoff:
            while (
                state["status"] == "spawning"
                and self._monotonic() < claim_deadline
            ):
                handoff.wait(
                    max(0.0, claim_deadline - self._monotonic())
                )
            if (
                state["status"] == "ready"
                and self._monotonic() < claim_deadline
            ):
                state["status"] = "claimed"
                process = state["process"]
                handoff.notify_all()
                return process, log_stream
            if state["status"] == "failed":
                raise state["failure"]
            state["status"] = "cancelled"
            handoff.notify_all()
            while not state["cleaned"] and self._monotonic() < deadline:
                handoff.wait(max(0.0, deadline - self._monotonic()))
        raise subprocess.TimeoutExpired(
            "FFmpeg process creation", started_remaining
        )

    def _kill_started_process(self, process: _Process, deadline: float) -> None:
        try:
            process.send_signal(signal.SIGKILL)
        except Exception:
            pid = getattr(process, "pid", None)
            if isinstance(pid, int) and pid > 0:
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
        remaining = max(0.0, deadline - self._monotonic())
        if remaining > 0:
            try:
                process.wait(timeout=remaining)
            except Exception:
                pass

    @staticmethod
    def _terminate_worker_process(process: _Process) -> None:
        try:
            process.send_signal(signal.SIGKILL)
        except Exception:
            pid = getattr(process, "pid", None)
            if isinstance(pid, int):
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
        try:
            process.wait(timeout=1.0)
        except Exception:
            pid = getattr(process, "pid", None)
            if isinstance(pid, int) and pid > 0:
                try:
                    _waited_pid, status = os.waitpid(pid, 0)
                    process.returncode = os.waitstatus_to_exitcode(status)
                except (ChildProcessError, OSError):
                    pass

    def _record_failure(self, event: str, detail: str, sim_timestamp_ns: int | None = None) -> None:
        self._failed = True
        diagnostic = VideoDiagnostic(self.stream, event, detail, sim_timestamp_ns)
        self._diagnostics.append(diagnostic)
        try:
            self._diagnostic_sink(diagnostic)
        except Exception as error:
            self._diagnostics.append(
                VideoDiagnostic(
                    self.stream,
                    "diagnostic_sink_failed",
                    f"diagnostic sink failed: {type(error).__name__}",
                    sim_timestamp_ns,
                )
            )

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
        payload = memoryview(bytes(self._pending_image.data))
        written = 0
        while written < len(payload):
            try:
                count = self._process.stdin.write(payload[written:])
            except InterruptedError:
                continue
            except (BrokenPipeError, OSError) as error:
                self._record_failure("ffmpeg_write_failed", f"FFmpeg pipe write failed: {error}", image_timestamp)
                raise RuntimeError("FFmpeg pipe write failed") from error
            remaining = len(payload) - written
            if (
                not isinstance(count, int)
                or isinstance(count, bool)
                or count <= 0
                or count > remaining
            ):
                detail = "FFmpeg pipe write returned a malformed byte count"
                self._record_failure("ffmpeg_write_failed", detail, image_timestamp)
                raise RuntimeError(detail)
            written += count
        self._pending_image = None
        self._pending_metadata = None
        self._frame_count += 1
        self._last_timestamp_ns = image_timestamp

    def finalize(self, deadline: float, *, outcome: str) -> VideoFinalization:
        process = self._process
        if process is None:
            raise RuntimeError("video recorder has not been started")
        signals: list[str] = []
        path_watch_fd: int | None = None
        primary_deadline, recovery_copy_deadline = self._recovery_deadlines(
            deadline
        )
        try:
            canonical_outcome = outcome.upper()
            if canonical_outcome not in {"COMPLETED", "FAILED", "ABORTED"}:
                raise ValueError("outcome must be COMPLETED, FAILED, or ABORTED")
            if self._pending_image is not None or self._pending_metadata is not None:
                self._record_failure(
                    "unmatched_frame",
                    "finalization found an unmatched image or metadata item",
                )
            self._close_stdin_within(primary_deadline)
            self._reap_within(process, primary_deadline, signals)

            returncode = process.poll()
            exited = returncode is not None
            if not exited:
                detail = f"FFmpeg did not exit before deadline after {', '.join(signals)}"
                boundary_failure = next(
                    (
                        item.detail
                        for item in reversed(self._diagnostics)
                        if item.event == "ffmpeg_process_boundary_failed"
                    ),
                    None,
                )
                if boundary_failure is not None:
                    detail = f"{detail}; {boundary_failure}"
                self._record_failure("ffmpeg_timeout", detail)
                self._snapshot_recovery_output(
                    recovery_copy_deadline, deadline
                )
                self._start_async_process_cleanup(process)
                return VideoFinalization(False, None, tuple(signals), bool(signals), False, detail)
            self._seal_output_readonly(primary_deadline)
            if returncode != 0:
                detail = f"FFmpeg exited with return code {returncode}"
                self._record_failure("ffmpeg_failed", detail)
                self._preserve_recovery_output(deadline)
                return VideoFinalization(True, returncode, tuple(signals), bool(signals), False, detail)
            if self._failed:
                detail = "video recorder failed before publication"
                self._preserve_recovery_output(deadline)
                return VideoFinalization(True, 0, tuple(signals), bool(signals), False, detail)

            assert self._readonly_output_fd is not None
            assert self._root_fd is not None and self._video_fd is not None
            path_watch_fd = _watch_descriptors((self._root_fd, self._video_fd))
            validation = self._validator.validate(
                self.run_directory,
                f"video/{self.stream}.mp4",
                expected_frame_count=self.expected_frame_count,
                outcome=canonical_outcome,
                deadline=primary_deadline,
                descriptor=self._readonly_output_fd,
            )
            if _watch_changed(path_watch_fd) or not self._paths_safe():
                detail = "video parent path changed during validation"
                self._record_failure("unsafe_video_path", detail)
                self._preserve_recovery_output(deadline)
                return VideoFinalization(
                    True, 0, tuple(signals), bool(signals), False, detail
                )
            if validation.status is not ValidationStatus.VALID:
                self._record_failure("video_invalid", validation.detail)
                self._preserve_recovery_output(deadline)
                return VideoFinalization(
                    True, 0, tuple(signals), bool(signals), False, validation.detail
                )
            try:
                if self._monotonic() >= primary_deadline:
                    raise subprocess.TimeoutExpired("video publication", 0)
                self._publish_readonly_output(f"{self.stream}.mp4")
            except Exception as error:
                detail = (
                    "video publication failed: "
                    f"{type(error).__name__}: {error}"
                )
                self._record_failure("video_publish_failed", detail)
                self._preserve_recovery_output(deadline)
                return VideoFinalization(
                    True, 0, tuple(signals), bool(signals), False, detail
                )
            return VideoFinalization(
                True, 0, tuple(signals), bool(signals), True, "video finalized"
            )
        except Exception as error:
            detail = f"video finalization failed: {type(error).__name__}: {error}"
            self._record_failure("video_finalize_failed", detail)
            if not self._preserve_recovery_output(deadline):
                self._snapshot_recovery_output(
                    recovery_copy_deadline, deadline
                )
            try:
                returncode = process.poll()
            except Exception:
                returncode = None
            return VideoFinalization(
                returncode is not None,
                returncode,
                tuple(signals),
                bool(signals),
                False,
                detail,
            )
        finally:
            if path_watch_fd is not None:
                try:
                    os.close(path_watch_fd)
                except OSError:
                    pass
            self._close_resources()

    def _recovery_deadlines(self, deadline: float) -> tuple[float, float]:
        remaining = max(0.0, deadline - self._monotonic())
        recovery_reserve = min(
            _RECOVERY_RESERVE_SECONDS, remaining / 2
        )
        link_margin = min(
            _RECOVERY_LINK_MARGIN_SECONDS, recovery_reserve / 2
        )
        return deadline - recovery_reserve, deadline - link_margin

    def _start_async_process_cleanup(self, process: _Process) -> None:
        threading.Thread(
            target=self._terminate_worker_process,
            args=(process,),
            daemon=True,
        ).start()

    def _close_stdin_within(self, deadline: float) -> None:
        process = self._process
        if process is None or process.stdin is None or process.stdin.closed:
            return
        stream = process.stdin
        process.stdin = None
        completed = threading.Event()
        failures: list[BaseException] = []

        def close_stream() -> None:
            try:
                stream.close()
            except BaseException as error:
                failures.append(error)
            finally:
                completed.set()

        threading.Thread(target=close_stream, daemon=True).start()
        completed.wait(max(0.0, deadline - self._monotonic()))
        if completed.is_set():
            process.stdin = stream
        if not completed.is_set():
            self._record_failure(
                "ffmpeg_close_timeout", "FFmpeg stdin close exceeded caller deadline"
            )
        elif failures:
            self._record_failure(
                "ffmpeg_close_failed",
                f"FFmpeg stdin close failed: {failures[0]}",
            )

    def _reap_within(
        self, process: _Process, deadline: float, signals: list[str]
    ) -> None:
        try:
            if process.poll() is not None:
                return
        except Exception as error:
            self._record_failure(
                "ffmpeg_process_boundary_failed",
                f"FFmpeg poll failed: {type(error).__name__}: {error}",
            )
            self._fallback_terminate_and_reap(process, deadline, signals)
            return
        phases: tuple[int | None, ...] = (None, signal.SIGTERM, signal.SIGKILL)
        for index, signum in enumerate(phases):
            try:
                if process.poll() is not None:
                    break
            except Exception as error:
                self._record_failure(
                    "ffmpeg_process_boundary_failed",
                    f"FFmpeg poll failed: {type(error).__name__}: {error}",
                )
                self._fallback_terminate_and_reap(process, deadline, signals)
                return
            if signum is not None:
                try:
                    process.send_signal(signum)
                except ProcessLookupError:
                    pass
                except Exception as error:
                    self._record_failure(
                        "ffmpeg_process_boundary_failed",
                        f"FFmpeg signal failed: {type(error).__name__}: {error}",
                    )
                    self._fallback_terminate_and_reap(process, deadline, signals)
                    return
                signals.append(signal.Signals(signum).name)
            remaining = max(0.0, deadline - self._monotonic())
            timeout = remaining / (len(phases) - index)
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                continue
            except Exception as error:
                self._record_failure(
                    "ffmpeg_process_boundary_failed",
                    f"FFmpeg wait failed: {type(error).__name__}: {error}",
                )
                self._fallback_terminate_and_reap(process, deadline, signals)
                return

    def _fallback_terminate_and_reap(
        self, process: _Process, deadline: float, signals: list[str]
    ) -> None:
        pid = getattr(process, "pid", None)
        if not isinstance(pid, int) or pid <= 0:
            for signum in (signal.SIGTERM, signal.SIGKILL):
                try:
                    process.send_signal(signum)
                    name = signal.Signals(signum).name
                    if name not in signals:
                        signals.append(name)
                except Exception:
                    continue
            return
        for signum in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.kill(pid, signum)
                name = signal.Signals(signum).name
                if name not in signals:
                    signals.append(name)
            except ProcessLookupError:
                return
            except OSError:
                continue
            while self._monotonic() < deadline:
                try:
                    waited_pid, status = os.waitpid(pid, os.WNOHANG)
                except ChildProcessError:
                    return
                except OSError:
                    break
                if waited_pid == pid:
                    try:
                        process.returncode = os.waitstatus_to_exitcode(status)
                    except Exception:
                        pass
                    return
                threading.Event().wait(
                    min(0.01, max(0.0, deadline - self._monotonic()))
                )

    def _seal_output_readonly(self, deadline: float) -> None:
        if self._output_fd is None or self._video_fd is None:
            raise RuntimeError("video output is not available")
        if not self._paths_safe():
            raise RuntimeError("unsafe video path at finalization")
        writable_fd = self._output_fd
        readonly_fd: int | None = None
        try:
            readonly_fd = self._reopen_anonymous_readonly(
                writable_fd, deadline
            )
            os.close(writable_fd)
            self._output_fd = None
            self._readonly_output_fd = readonly_fd
            readonly_fd = None
        finally:
            if readonly_fd is not None:
                try:
                    os.close(readonly_fd)
                except OSError:
                    pass

    def _reopen_anonymous_readonly(
        self, writable_fd: int, deadline: float
    ) -> int:
        readonly_fd: int | None = None
        try:
            self._require_recovery_time(deadline, "video output fsync")
            os.fsync(writable_fd)
            self._require_recovery_time(deadline, "video output reopen")
            readonly_fd = os.open(
                f"/proc/self/fd/{writable_fd}",
                os.O_RDONLY | getattr(os, "O_CLOEXEC", 0),
            )
            writable_identity = os.fstat(writable_fd)
            readonly_identity = os.fstat(readonly_fd)
            if (
                not stat.S_ISREG(writable_identity.st_mode)
                or writable_identity.st_nlink != 0
                or not _same_entry(writable_identity, readonly_identity)
                or fcntl.fcntl(readonly_fd, fcntl.F_GETFL) & os.O_ACCMODE
                != os.O_RDONLY
            ):
                raise RuntimeError("anonymous video output identity is unsafe")
            self._require_recovery_time(deadline, "video output chmod")
            os.fchmod(writable_fd, 0o444)
            self._require_recovery_time(deadline, "video output fsync")
            os.fsync(writable_fd)
            self._require_recovery_time(deadline, "video output sealing")
            readonly_identity = os.fstat(readonly_fd)
            if readonly_identity.st_mode & 0o777 != 0o444:
                raise RuntimeError("video output permissions are not read-only")
            result = readonly_fd
            readonly_fd = None
            return result
        finally:
            if readonly_fd is not None:
                try:
                    os.close(readonly_fd)
                except OSError:
                    pass

    def _require_recovery_time(self, deadline: float, label: str) -> None:
        if self._monotonic() >= deadline:
            raise subprocess.TimeoutExpired(label, 0)

    def _publish_readonly_output(self, name: str) -> None:
        if self._readonly_output_fd is None or self._video_fd is None:
            raise RuntimeError("read-only video output is not available")
        self._publish_readonly_descriptor(self._readonly_output_fd, name)

    def _publish_readonly_descriptor(self, descriptor: int, name: str) -> None:
        if self._video_fd is None:
            raise RuntimeError("video directory is not available")
        retained = os.fstat(descriptor)
        if (
            not self._paths_safe()
            or not stat.S_ISREG(retained.st_mode)
            or retained.st_nlink != 0
            or retained.st_mode & 0o777 != 0o444
            or fcntl.fcntl(descriptor, fcntl.F_GETFL) & os.O_ACCMODE
            != os.O_RDONLY
        ):
            raise RuntimeError("read-only video output is unsafe")
        _link_descriptor_noreplace(descriptor, self._video_fd, name)

    def _preserve_recovery_output(self, deadline: float) -> bool:
        if self._readonly_output_fd is None:
            return False
        try:
            if deadline - self._monotonic() <= 0:
                return False
            self._publish_readonly_output(f"{self.stream}.mp4.partial")
            return True
        except Exception as error:
            self._record_failure(
                "video_recovery_failed",
                f"video recovery publication failed: {type(error).__name__}: {error}",
            )
            return False

    def _snapshot_recovery_output(
        self, copy_deadline: float, deadline: float
    ) -> bool:
        source_fd = self._output_fd
        if source_fd is None or self._video_fd is None:
            return False
        snapshot_fd: int | None = None
        readonly_fd: int | None = None
        try:
            self._require_recovery_time(deadline, "video recovery snapshot")
            snapshot_fd = os.open(
                ".", _OUTPUT_FLAGS, 0o600, dir_fd=self._video_fd
            )
            offset = 0
            copying = True
            try:
                while copying and self._monotonic() < copy_deadline:
                    self._require_recovery_time(
                        copy_deadline, "video recovery copy"
                    )
                    chunk = os.pread(
                        source_fd, _RECOVERY_COPY_CHUNK_BYTES, offset
                    )
                    if self._monotonic() >= copy_deadline or not chunk:
                        break
                    view = memoryview(chunk)
                    while view:
                        if self._monotonic() >= copy_deadline:
                            copying = False
                            break
                        count = os.write(snapshot_fd, view)
                        if (
                            not isinstance(count, int)
                            or isinstance(count, bool)
                            or count <= 0
                            or count > len(view)
                        ):
                            raise RuntimeError(
                                "video recovery write returned a malformed byte count"
                            )
                        offset += count
                        view = view[count:]
            except Exception as error:
                self._record_failure(
                    "video_recovery_copy_failed",
                    "video recovery copy stopped: "
                    f"{type(error).__name__}: {error}",
                )
            readonly_fd = self._reopen_anonymous_readonly(
                snapshot_fd, deadline
            )
            os.close(snapshot_fd)
            snapshot_fd = None
            self._require_recovery_time(
                deadline, "video recovery publication"
            )
            self._publish_readonly_descriptor(
                readonly_fd, f"{self.stream}.mp4.partial"
            )
            return True
        except Exception as error:
            self._record_failure(
                "video_recovery_failed",
                "video recovery snapshot failed: "
                f"{type(error).__name__}: {error}",
            )
            return False
        finally:
            for descriptor in (readonly_fd, snapshot_fd):
                if descriptor is not None:
                    try:
                        os.close(descriptor)
                    except OSError:
                        pass

    def _close_resources(self) -> None:
        if self._log_stream is not None:
            try:
                self._log_stream.close()
            except Exception:
                pass
            self._log_stream = None
        for name in (
            "_readonly_output_fd",
            "_output_fd",
            "_video_fd",
            "_root_fd",
        ):
            descriptor = getattr(self, name)
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
                setattr(self, name, None)


__all__ = [
    "ENCODING", "FPS", "FRAME_INTERVAL_NS", "HEIGHT_PX", "PAYLOAD_BYTES", "STEP_BYTES",
    "STREAMS", "VideoDiagnostic", "VideoFinalization", "VideoStreamRecorder",
    "VideoValidationResult", "VideoValidator", "WIDTH_PX",
]
