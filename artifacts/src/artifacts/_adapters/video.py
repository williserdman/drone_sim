"""Simulation-time RGB frame pairing, FFmpeg control, and MP4 validation."""

from collections.abc import Callable, Sequence
import ctypes
from dataclasses import dataclass
import errno
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
    | os.O_CREAT
    | os.O_EXCL
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
            entry = os.stat(filename, dir_fd=parent_fd, follow_symlinks=False)
            if file_fd is None:
                file_fd = os.open(filename, _FILE_FLAGS, dir_fd=parent_fd)
            initial_stat = os.fstat(file_fd)
            if (
                not stat.S_ISREG(initial_stat.st_mode)
                or initial_stat.st_nlink != 1
                or not _same_entry(entry, initial_stat)
            ):
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
                        or not _same_entry(
                            os.stat(filename, dir_fd=parent_fd, follow_symlinks=False),
                            final_stat,
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
        self._partial_fd: int | None = None
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
        descriptor = self._partial_fd if output_descriptor is None else output_descriptor
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
        partial_fd: int | None = None
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
            final_name = f"{self.stream}.mp4"
            try:
                os.stat(final_name, dir_fd=video_fd, follow_symlinks=False)
            except FileNotFoundError:
                pass
            else:
                raise FileExistsError(f"refusing existing video output: {final_name}")
            partial_name = f"{self.stream}.mp4.partial"
            partial_fd = os.open(partial_name, _OUTPUT_FLAGS, 0o644, dir_fd=video_fd)
            partial_identity = os.fstat(partial_fd)
            current = os.stat(partial_name, dir_fd=video_fd, follow_symlinks=False)
            if (
                not stat.S_ISREG(partial_identity.st_mode)
                or partial_identity.st_nlink != 1
                or not _same_entry(partial_identity, current)
            ):
                raise _unsafe_path()
            self._root_fd, self._video_fd, self._partial_fd = root_fd, video_fd, partial_fd
            self._root_identity, self._video_identity = root_opened, video_identity
            committed = True
        except (FileExistsError, RuntimeError):
            raise
        except OSError as error:
            raise _unsafe_path() from error
        finally:
            if not committed:
                if partial_fd is not None and video_fd is not None:
                    try:
                        retained = os.fstat(partial_fd)
                        current = os.stat(
                            f"{self.stream}.mp4.partial",
                            dir_fd=video_fd,
                            follow_symlinks=False,
                        )
                        if _same_entry(retained, current):
                            os.unlink(
                                f"{self.stream}.mp4.partial", dir_fd=video_fd
                            )
                    except OSError:
                        pass
                for descriptor in (partial_fd, video_fd, root_fd):
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
            self._discard_reserved_partial()
            self._close_resources()
            raise

    def _spawn_with_deadline(self, deadline: float, log_stream: Any) -> tuple[_Process, Any]:
        assert self._partial_fd is not None
        try:
            output_fd = os.dup(self._partial_fd)
        except BaseException:
            try:
                log_stream.close()
            except Exception:
                pass
            raise
        cancelled = threading.Event()
        completed = threading.Event()
        handoff_lock = threading.Lock()
        result: list[_Process] = []
        failures: list[BaseException] = []

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
                with handoff_lock:
                    cancel_process = cancelled.is_set()
                    if not cancel_process:
                        result.append(process)
                if cancel_process:
                    self._terminate_late_process(process)
            except BaseException as error:
                failures.append(error)
            finally:
                try:
                    os.close(output_fd)
                except OSError:
                    pass
                if cancelled.is_set():
                    try:
                        log_stream.close()
                    except Exception:
                        pass
                completed.set()

        threading.Thread(target=spawn_process, daemon=True).start()
        remaining = max(0.0, deadline - self._monotonic())
        if not completed.wait(remaining):
            with handoff_lock:
                cancelled.set()
                handed_off = result.pop() if result else None
            if handed_off is not None:
                self._kill_started_process(handed_off, deadline)
            raise subprocess.TimeoutExpired("FFmpeg process creation", remaining)
        if deadline - self._monotonic() <= 0:
            with handoff_lock:
                cancelled.set()
                handed_off = result.pop() if result else None
            if handed_off is not None:
                self._kill_started_process(handed_off, deadline)
            raise subprocess.TimeoutExpired("FFmpeg process creation", remaining)
        if failures:
            try:
                log_stream.close()
            except Exception:
                pass
            raise failures[0]
        return result[0], log_stream

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
    def _terminate_late_process(process: _Process) -> None:
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
            if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
                detail = "FFmpeg pipe write made no forward progress"
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
        guard_fd: int | None = None
        try:
            canonical_outcome = outcome.upper()
            if canonical_outcome not in {"COMPLETED", "FAILED", "ABORTED"}:
                raise ValueError("outcome must be COMPLETED, FAILED, or ABORTED")
            if self._pending_image is not None or self._pending_metadata is not None:
                self._record_failure(
                    "unmatched_frame",
                    "finalization found an unmatched image or metadata item",
                )
            self._close_stdin_within(deadline)
            self._reap_within(process, deadline, signals)

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
                return VideoFinalization(False, None, tuple(signals), bool(signals), False, detail)
            if returncode != 0:
                detail = f"FFmpeg exited with return code {returncode}"
                self._record_failure("ffmpeg_failed", detail)
                return VideoFinalization(True, returncode, tuple(signals), bool(signals), False, detail)
            if self._failed:
                detail = "video recorder failed before publication"
                return VideoFinalization(True, 0, tuple(signals), bool(signals), False, detail)
            if not self._paths_safe() or not self._partial_matches():
                detail = "unsafe video path at finalization"
                self._record_failure("unsafe_video_path", detail)
                return VideoFinalization(True, 0, tuple(signals), bool(signals), False, detail)

            assert self._partial_fd is not None
            assert self._root_fd is not None and self._video_fd is not None
            guard_fd = _watch_descriptors(
                (self._root_fd, self._video_fd, self._partial_fd)
            )
            publication_digest = self._hash_descriptor(self._partial_fd, deadline)
            validation = self._validator.validate(
                self.run_directory,
                f"video/{self.stream}.mp4.partial",
                expected_frame_count=self.expected_frame_count,
                outcome=canonical_outcome,
                deadline=deadline,
                descriptor=self._partial_fd,
            )
            if validation.status is not ValidationStatus.VALID:
                self._record_failure("video_invalid", validation.detail)
                return VideoFinalization(
                    True, 0, tuple(signals), bool(signals), False, validation.detail
                )
            if not self._publication_guard_stable(
                guard_fd, publication_digest, deadline
            ):
                detail = "video output changed after validation"
                self._record_failure("video_publish_failed", detail)
                return VideoFinalization(True, 0, tuple(signals), bool(signals), False, detail)
            self._publish_retained_inode(
                deadline=deadline,
                expected_digest=publication_digest,
                guard_fd=guard_fd,
            )
            return VideoFinalization(
                True, 0, tuple(signals), bool(signals), True, "video finalized"
            )
        except Exception as error:
            detail = f"video finalization failed: {type(error).__name__}: {error}"
            self._record_failure("video_finalize_failed", detail)
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
            if guard_fd is not None:
                try:
                    os.close(guard_fd)
                except OSError:
                    pass
            self._close_resources()

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

    def _partial_matches(self, expected_links: int = 1) -> bool:
        if self._partial_fd is None or self._video_fd is None:
            return False
        try:
            opened = os.fstat(self._partial_fd)
            current = os.stat(
                f"{self.stream}.mp4.partial",
                dir_fd=self._video_fd,
                follow_symlinks=False,
            )
            return (
                stat.S_ISREG(opened.st_mode)
                and opened.st_nlink == expected_links
                and _same_entry(opened, current)
            )
        except OSError:
            return False

    def _hash_descriptor(self, descriptor: int, deadline: float) -> str:
        digest = hashlib.sha256()
        offset = 0
        while True:
            if deadline - self._monotonic() <= 0:
                raise subprocess.TimeoutExpired("video hash", 0)
            chunk = os.pread(descriptor, 1024 * 1024, offset)
            if deadline - self._monotonic() <= 0:
                raise subprocess.TimeoutExpired("video hash", 0)
            if not chunk:
                return digest.hexdigest()
            digest.update(chunk)
            offset += len(chunk)

    def _publication_guard_stable(
        self,
        guard_fd: int,
        expected_digest: str,
        deadline: float,
        *,
        expected_links: int = 1,
    ) -> bool:
        if (
            not self._paths_safe()
            or not self._partial_matches(expected_links)
            or _watch_changed(guard_fd)
            or self._partial_fd is None
        ):
            return False
        digest = self._hash_descriptor(self._partial_fd, deadline)
        return (
            digest == expected_digest
            and self._paths_safe()
            and self._partial_matches(expected_links)
            and not _watch_changed(guard_fd)
        )

    def _publish_retained_inode(
        self, *, deadline: float, expected_digest: str, guard_fd: int
    ) -> None:
        assert self._partial_fd is not None and self._video_fd is not None
        partial_name = f"{self.stream}.mp4.partial"
        final_name = f"{self.stream}.mp4"
        linked = False
        try:
            _link_descriptor_noreplace(self._partial_fd, self._video_fd, final_name)
            linked = True
            final = os.stat(final_name, dir_fd=self._video_fd, follow_symlinks=False)
            if not _same_entry(final, os.fstat(self._partial_fd)):
                raise RuntimeError("published video does not match retained descriptor")
            current_partial = os.stat(
                partial_name, dir_fd=self._video_fd, follow_symlinks=False
            )
            retained = os.fstat(self._partial_fd)
            if (
                not _same_entry(current_partial, retained)
                or retained.st_nlink != 2
            ):
                raise RuntimeError("partial video changed during publication")
            if not self._publication_guard_stable(
                guard_fd, expected_digest, deadline, expected_links=2
            ):
                raise RuntimeError("video changed during publication")
            os.unlink(partial_name, dir_fd=self._video_fd)
        except Exception:
            if linked:
                self._unlink_final_if_retained(final_name)
            raise

    def _unlink_final_if_retained(self, final_name: str) -> None:
        assert self._partial_fd is not None and self._video_fd is not None
        try:
            current = os.stat(final_name, dir_fd=self._video_fd, follow_symlinks=False)
            if _same_entry(current, os.fstat(self._partial_fd)):
                os.unlink(final_name, dir_fd=self._video_fd)
        except OSError:
            pass

    def _discard_reserved_partial(self) -> None:
        if not self._partial_matches() or self._video_fd is None:
            return
        try:
            os.unlink(f"{self.stream}.mp4.partial", dir_fd=self._video_fd)
        except OSError:
            pass

    def _close_resources(self) -> None:
        if self._log_stream is not None:
            try:
                self._log_stream.close()
            except Exception:
                pass
            self._log_stream = None
        for name in ("_partial_fd", "_video_fd", "_root_fd"):
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
