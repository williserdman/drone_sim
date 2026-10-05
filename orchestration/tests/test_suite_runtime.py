from pathlib import Path
import os

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
