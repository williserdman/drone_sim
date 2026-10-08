from __future__ import annotations

from pathlib import Path

from artifacts.media_spool import MediaSpool, MediaSpoolFailure, MediaSpoolPublication


RUN_ID = "11111111-1111-4111-8111-111111111111"


def _source(spool: MediaSpool, relative: str, contents: bytes) -> Path:
    path = spool.recording_directory / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(contents)
    path.chmod(0o444)
    return path


def test_publish_copies_closed_video_and_ffmpeg_log_atomically(tmp_path: Path) -> None:
    spool_root = tmp_path / "native-media"
    authoritative = tmp_path / "run"
    spool_root.mkdir()
    authoritative.mkdir()
    spool = MediaSpool.create(spool_root, authoritative, RUN_ID)
    _source(spool, "video/onboard.mp4", b"validated mp4")
    _source(
        spool,
        "logs/docker/ffmpeg-onboard.log.partial",
        b"ffmpeg diagnostics",
    )

    publication = spool.publish(deadline=10.0, monotonic=lambda: 0.0)

    assert publication.failures == ()
    assert publication.published == (
        "logs/docker/ffmpeg-onboard.log.partial",
        "video/onboard.mp4",
    )
    assert (authoritative / "video/onboard.mp4").read_bytes() == b"validated mp4"
    assert (
        authoritative / "logs/docker/ffmpeg-onboard.log.partial"
    ).read_bytes() == b"ffmpeg diagnostics"
    assert (authoritative / "video/onboard.mp4").stat().st_mode & 0o777 == 0o444
    assert not tuple(authoritative.rglob(".media-spool-*.tmp"))
    assert publication.closed_media_verified is False


def test_publication_verifies_both_closed_video_streams() -> None:
    publication = MediaSpoolPublication(
        (
            "logs/docker/ffmpeg-onboard.log.partial",
            "video/observer.mp4.partial",
            "video/onboard.mp4",
        ),
        (),
    )

    assert publication.closed_media_verified is True


def test_publication_verifies_absent_media_for_a_recorder_that_never_started() -> None:
    one_started = MediaSpoolPublication(
        ("video/onboard.mp4.partial",),
        (),
        ("observer",),
    )
    neither_started = MediaSpoolPublication((), (), ("onboard", "observer"))

    assert one_started.closed_media_verified is True
    assert neither_started.closed_media_verified is True


def test_publication_requires_media_for_every_started_recorder() -> None:
    publication = MediaSpoolPublication(
        ("video/onboard.mp4.partial",),
        (),
    )

    assert publication.closed_media_verified is False


def test_publication_does_not_verify_failed_or_ambiguous_media() -> None:
    failed = MediaSpoolPublication(
        ("video/onboard.mp4", "video/observer.mp4"),
        (MediaSpoolFailure("video/onboard.mp4", "copy failed"),),
    )
    ambiguous = MediaSpoolPublication(
        (
            "video/onboard.mp4",
            "video/onboard.mp4.partial",
            "video/observer.mp4",
        ),
        (),
    )

    assert failed.closed_media_verified is False
    assert ambiguous.closed_media_verified is False


def test_publish_preserves_partial_video_when_final_output_is_absent(
    tmp_path: Path,
) -> None:
    spool_root = tmp_path / "native-media"
    authoritative = tmp_path / "run"
    spool_root.mkdir()
    authoritative.mkdir()
    spool = MediaSpool.create(spool_root, authoritative, RUN_ID)
    _source(spool, "video/observer.mp4.partial", b"recoverable prefix")

    publication = spool.publish(deadline=10.0, monotonic=lambda: 0.0)

    assert publication.failures == ()
    assert publication.published == ("video/observer.mp4.partial",)
    assert (authoritative / "video/observer.mp4.partial").read_bytes() == (
        b"recoverable prefix"
    )
    assert not (authoritative / "video/observer.mp4").exists()


def test_publish_never_replaces_existing_authoritative_output(tmp_path: Path) -> None:
    spool_root = tmp_path / "native-media"
    authoritative = tmp_path / "run"
    spool_root.mkdir()
    (authoritative / "video").mkdir(parents=True)
    destination = authoritative / "video/onboard.mp4"
    destination.write_bytes(b"existing authority")
    spool = MediaSpool.create(spool_root, authoritative, RUN_ID)
    _source(spool, "video/onboard.mp4", b"new candidate")

    publication = spool.publish(deadline=10.0, monotonic=lambda: 0.0)

    assert publication.published == ()
    assert len(publication.failures) == 1
    assert publication.failures[0].relative_path == "video/onboard.mp4"
    assert "exists" in publication.failures[0].detail
    assert destination.read_bytes() == b"existing authority"
    assert not tuple(authoritative.rglob(".media-spool-*.tmp"))


def test_publish_timeout_leaves_no_public_or_temporary_output(tmp_path: Path) -> None:
    spool_root = tmp_path / "native-media"
    authoritative = tmp_path / "run"
    spool_root.mkdir()
    authoritative.mkdir()
    spool = MediaSpool.create(spool_root, authoritative, RUN_ID)
    _source(spool, "video/onboard.mp4", b"candidate")

    publication = spool.publish(deadline=0.0, monotonic=lambda: 0.0)

    assert publication.published == ()
    assert len(publication.failures) == 1
    assert "deadline" in publication.failures[0].detail
    assert not (authoritative / "video/onboard.mp4").exists()
    assert not tuple(authoritative.rglob(".media-spool-*.tmp"))


def test_create_refuses_reusing_a_run_scoped_spool_directory(tmp_path: Path) -> None:
    spool_root = tmp_path / "native-media"
    authoritative = tmp_path / "run"
    spool_root.mkdir()
    authoritative.mkdir()
    MediaSpool.create(spool_root, authoritative, RUN_ID)

    try:
        MediaSpool.create(spool_root, authoritative, RUN_ID)
    except FileExistsError as error:
        assert RUN_ID in str(error)
    else:
        raise AssertionError("run-scoped spool reuse must fail closed")


def test_publish_rejects_symlinked_source_without_copying_target(tmp_path: Path) -> None:
    spool_root = tmp_path / "native-media"
    authoritative = tmp_path / "run"
    spool_root.mkdir()
    authoritative.mkdir()
    spool = MediaSpool.create(spool_root, authoritative, RUN_ID)
    target = tmp_path / "outside.mp4"
    target.write_bytes(b"outside")
    source = spool.recording_directory / "video/onboard.mp4"
    source.parent.mkdir(parents=True)
    source.symlink_to(target)

    publication = spool.publish(deadline=10.0, monotonic=lambda: 0.0)

    assert publication.published == ()
    assert len(publication.failures) == 1
    assert publication.failures[0].relative_path == "video/onboard.mp4"
    assert not (authoritative / "video/onboard.mp4").exists()
