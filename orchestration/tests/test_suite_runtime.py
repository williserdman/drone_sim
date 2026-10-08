from pathlib import Path
import os
from types import SimpleNamespace

import pytest


def test_suite_lock_spans_different_output_roots(tmp_path):
    from orchestration._adapters.suite import SuiteRuntime
    first = SuiteRuntime(project_directory=tmp_path / 'checkout-a')
    second = SuiteRuntime(project_directory=tmp_path / 'checkout-b')
    with first.lock():
        with pytest.raises(RuntimeError, match='already running'):
            with second.lock():
                pytest.fail('second suite acquired workstation lock')
    with second.lock():
        pass


def test_dirty_source_fails_before_build(tmp_path):
    from orchestration._adapters.suite import SuiteRuntime
    from artifacts import SourceRevision
    class Controller:
        def source_revisions(self, deadline):
            return (SourceRevision('drone_sim','a'*40,True),)
    def no_build(*args, **kwargs):
        pytest.fail('dirty source must not build')
    runtime = SuiteRuntime(project_directory=tmp_path, controller=Controller(), runner=no_build)
    with pytest.raises(RuntimeError, match='clean committed'):
        runtime.build_and_freeze()


def test_frozen_images_compare_by_name_and_digest_not_listing_order(tmp_path):
    from artifacts import ImageDigest, SourceRevision
    from orchestration._adapters.suite import FrozenSuiteRuntime, SuiteRuntime

    sources = (SourceRevision('drone_sim', 'a' * 40, False),)
    images = (
        ImageDigest('image-a', 'b' * 64),
        ImageDigest('image-b', 'c' * 64),
    )

    class Controller:
        def source_revisions(self, deadline):
            return sources

    runtime = SuiteRuntime(project_directory=tmp_path, controller=Controller())
    frozen = FrozenSuiteRuntime(sources, images)
    runtime.compose = SimpleNamespace(image_digests=lambda _timeout: tuple(reversed(images)))

    runtime.assert_unchanged(frozen)

    for current in (
        (ImageDigest('image-a', 'd' * 64), images[1]),
        (images[0],),
    ):
        runtime.compose = SimpleNamespace(image_digests=lambda _timeout, rows=current: rows)
        with pytest.raises(RuntimeError, match='runtime image changed'):
            runtime.assert_unchanged(frozen)
