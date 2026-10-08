from contextlib import nullcontext
from io import StringIO
import json
from pathlib import Path
from types import SimpleNamespace
from uuid import uuid4

import pytest
from artifacts import SourceRevision, ImageDigest
from artifacts.acceptance import BundleAcceptanceReport
from orchestration._adapters.suite import FrozenSuiteRuntime
from orchestration.suite import SuiteRunner

ROOT = Path(__file__).parents[2]


def make_suite(tmp_path, *, failed_case=None, lifecycle_failure=None, teardown_failure=None, changed_after=None):
    frozen = FrozenSuiteRuntime((SourceRevision('drone_sim','a'*40,False), SourceRevision('comp2026','a'*40,False)),
                                (ImageDigest('image','b'*64),))
    class Controller:
        def __init__(self):
            self.started_cases = []
            self.abort_calls = []
            self.auxiliaries = []
        def start(self, path, *, auxiliary_services=(), on_allocated=None):
            name = path.parent.name
            self.started_cases.append(name)
            self.auxiliaries.append(auxiliary_services)
            document = json.loads(path.read_text())
            run_id = str(uuid4())
            directory = Path(document['output_root']) / run_id
            (directory / '.status').mkdir(parents=True)
            on_allocated(run_id, directory)
            state = 'FAILED' if name == lifecycle_failure else 'COMPLETED'
            (directory / 'manifest.json').write_text(json.dumps({'terminal_status':state,
                'scoring': {'achieved_score':60 if name=='hover-roll' else 100,'maximum_available_score':100}}))
            (directory / '.status/operator-state.json').write_text(json.dumps({'run_id':run_id,'state':state,'reason':'finished','manifest_path':'manifest.json','primary_cause':None,'diagnostics':[{'kind':'teardown','reason':'down failed'}] if name==teardown_failure else []}))
            return SimpleNamespace(run_id=run_id,state=state,reason='failed' if state=='FAILED' else None)
        def abort(self, run_id, root):
            self.abort_calls.append((run_id,root))
    controller = Controller()
    class Runtime:
        build_calls = 0
        def lock(self): return nullcontext()
        def build_and_freeze(self):
            self.build_calls += 1
            return frozen
        def assert_unchanged(self, value):
            assert value is frozen
            if changed_after is not None and len(controller.started_cases)>=changed_after:
                raise RuntimeError('suite source changed after build')
        def inspect_run(self, case, directory, value):
            if case.name == failed_case:
                raise ValueError('video missing')
            return BundleAcceptanceReport(directory.name,60 if case.name=='hover-roll' else 100,100,'project','c'*64)
    runtime = Runtime()
    runner = SuiteRunner(project_directory=ROOT,controller=controller,runtime=runtime,event_stream=StringIO())
    return runner,controller,runtime


def test_suite_builds_once_and_runs_all_unique_bundles(tmp_path):
    runner,controller,runtime = make_suite(tmp_path)
    result = runner.run(ROOT/'config/ci-suite.json',output_root=tmp_path/'runs')
    report = json.loads(result.report_path.read_text())
    assert result.exit_code == 0 and result.state == 'PASSED'
    assert runtime.build_calls == 1 and len(controller.started_cases)==11
    assert len({row['run_id'] for row in report['cases']})==11
    assert controller.auxiliaries[3] == ('operator-wait-runtime',)
    assert all(row['physical_outcome']=='LANDED' for row in report['cases'])
    assert report['cases'][5]['score']['achieved_score']==60


@pytest.mark.parametrize('gate,count',[('calibration',1),('reload-validation',2)])
def test_failed_gate_blocks_remaining_cases(tmp_path,gate,count):
    runner,controller,_=make_suite(tmp_path,failed_case=gate)
    result=runner.run(ROOT/'config/ci-suite.json',output_root=tmp_path/'runs')
    assert result.exit_code==1 and len(controller.started_cases)==count
    report=json.loads(result.report_path.read_text())
    assert all(row['status']=='blocked' for row in report['cases'][count:])


def test_completed_run_with_invalid_artifacts_fails_but_continues(tmp_path):
    runner,controller,runtime=make_suite(tmp_path,failed_case='hover-roll')
    result=runner.run(ROOT/'config/ci-suite.json',output_root=tmp_path/'runs')
    assert result.exit_code==1 and len(controller.started_cases)==11
    assert controller.started_cases[-1]=='competition-realtime'
    case=json.loads(result.report_path.read_text())['cases'][5]
    assert case['lifecycle']=='COMPLETED' and case['physical_outcome'] is None
    assert case['artifact_acceptance']['accepted'] is False


@pytest.mark.parametrize('case_name,started_count,later_status', [
    ('calibration', 0, 'blocked'),
    ('reload-validation', 1, 'blocked'),
    ('competition-slow', 10, 'passed'),
])
def test_preparation_failure_respects_case_dependencies(
    tmp_path, case_name, started_count, later_status
):
    runner, controller, runtime = make_suite(tmp_path)
    original_build = runtime.build_and_freeze

    def block_case_directory():
        frozen = original_build()
        suite_directory = next((tmp_path / 'runs/suites').iterdir())
        configuration = suite_directory / 'configuration'
        configuration.mkdir()
        (configuration / case_name).write_text('cannot create a directory here')
        return frozen

    runtime.build_and_freeze = block_case_directory
    result = runner.run(ROOT / 'config/ci-suite.json', output_root=tmp_path / 'runs')
    report = json.loads(result.report_path.read_text())

    assert result.state == 'FAILED' and result.exit_code == 1
    assert len(controller.started_cases) == started_count
    index = next(i for i, row in enumerate(report['cases']) if row['name'] == case_name)
    failed = report['cases'][index]
    assert failed['status'] == 'failed' and failed['reason']
    assert failed['run_id'] is None and failed['bundle_path'] is None
    assert failed['lifecycle'] is None and failed['physical_outcome'] is None
    assert all(row['status'] == later_status for row in report['cases'][index + 1:])
    if later_status == 'passed':
        assert controller.started_cases[-1] == 'competition-realtime'


@pytest.mark.parametrize('failure',[{'teardown_failure':'configured-descent'},{'changed_after':3}])
def test_unconfirmed_runtime_blocks_remaining_launches(tmp_path,failure):
    runner,controller,_=make_suite(tmp_path,**failure)
    result=runner.run(ROOT/'config/ci-suite.json',output_root=tmp_path/'runs')
    assert result.exit_code==1 and len(controller.started_cases)==3


def test_setup_failure_preserves_partial_report(tmp_path):
    runner,controller,runtime=make_suite(tmp_path)
    def fail(): raise RuntimeError('build failed')
    runtime.build_and_freeze=fail
    result=runner.run(ROOT/'config/ci-suite.json',output_root=tmp_path/'runs')
    assert result.exit_code==2 and result.state=='SETUP_FAILED'
    assert result.report_path.is_file() and not controller.started_cases


def test_allocated_run_is_reported_before_long_running_start_returns(tmp_path):
    runner, controller, _ = make_suite(tmp_path)
    original = controller.start
    observed = []

    def inspect_allocation(path, *, auxiliary_services=(), on_allocated=None):
        def allocated(run_id, directory):
            on_allocated(run_id, directory)
            report_path = next((tmp_path / 'runs/suites').glob('*/report.json'))
            report = json.loads(report_path.read_text())
            row = report['cases'][len(observed)]
            assert row['run_id'] == run_id
            assert (report_path.parent / row['bundle_path']).resolve() == directory
            assert row['status'] == 'running'
            assert row['lifecycle'] == 'ALLOCATED'
            assert report['source_revisions'] and report['image_digests']
            observed.append(run_id)
        return original(path, auxiliary_services=auxiliary_services, on_allocated=allocated)

    controller.start = inspect_allocation
    result = runner.run(ROOT / 'config/ci-suite.json', output_root=tmp_path / 'runs')
    assert result.state == 'PASSED'
    assert len(observed) == 11


def test_startup_interruption_aborts_only_allocated_run_and_restores_handlers(tmp_path):
    import signal
    runner,controller,runtime=make_suite(tmp_path)
    previous=signal.getsignal(signal.SIGTERM)
    original=controller.start
    def interrupted(path,*,auxiliary_services=(),on_allocated=None):
        def allocated(run_id,directory):
            on_allocated(run_id,directory)
            signal.raise_signal(signal.SIGTERM)
        return original(path,auxiliary_services=auxiliary_services,on_allocated=allocated)
    controller.start=interrupted
    result=runner.run(ROOT/'config/ci-suite.json',output_root=tmp_path/'runs')
    assert result.state=='INTERRUPTED' and result.exit_code==130
    assert len(controller.started_cases)==1 and len(controller.abort_calls)==1
    report=json.loads(result.report_path.read_text())
    assert controller.abort_calls[0][0]==report['cases'][0]['run_id']
    assert all(row['status']=='unrun' for row in report['cases'][1:])
    assert signal.getsignal(signal.SIGTERM) is previous


def test_corrupt_case_manifest_does_not_skip_independent_cases(tmp_path):
    runner,controller,runtime=make_suite(tmp_path)
    original=controller.start
    def corrupt(path,**kwargs):
        result=original(path,**kwargs)
        if path.parent.name=='hover-roll':
            (tmp_path/'runs'/result.run_id/'manifest.json').write_text('{invalid')
        return result
    controller.start=corrupt
    result=runner.run(ROOT/'config/ci-suite.json',output_root=tmp_path/'runs')
    assert len(controller.started_cases)==11 and result.exit_code==1
    assert json.loads(result.report_path.read_text())['cases'][5]['status']=='failed'
