"""Chrome UI regression smoke: uv run --with playwright python tests/browser/check_console_views.py."""

import threading
from pathlib import Path
from uuid import UUID
from playwright.sync_api import expect, sync_playwright
from orchestration.console import ConsoleServer

class Session:
    project_directory = Path.cwd()
    run_ids = (
        '11111111-1111-4111-8111-111111111111',
        '22222222-2222-4222-8222-222222222222',
    )

    def __init__(self):
        self.connected = False
        self.failed = False
        self.state = 'IDLE'
        self.run_index = -1
        self.cleanup = None
        self.actions = []
        self.observations = []

    @property
    def run_id(self):
        return self.run_ids[self.run_index] if self.run_index >= 0 else None

    def snapshot(self):
        if self.failed:
            raise RuntimeError('owner unavailable')
        return {'state':self.state, 'run_id':self.run_id, 'cleanup':self.cleanup,
                'can_start':False,
                'qgc_admission_ready':False,
                'qgc_action_path':(f'/tmp/QGroundControl/MavlinkActions/drone-sim-{self.run_id}.json'
                                   if self.run_id else None),
                'preflight': {'ready': False, 'reasons': ['waiting for disarmed preflight']},
                'views': {'gazebo': {'available':self.connected, 'connected':bool(self.observations)}, 'camera': {'fresh':self.connected}, 'qgc':{}}}
    def checks(self): return {'checks':[], 'ok':True}
    def results(self): return {}
    def prepare(self, **request):
        self.run_index += 1
        self.state = 'PREPARING'
        self.cleanup = 'pending'
        self.actions.append(('prepare', request))
        return self.snapshot()
    def stop(self, **request):
        assert request == {'run_id': self.run_id}
        self.actions.append(('stop', request))
        self.state = 'ABORTED'
        self.cleanup = 'complete'
        return self.snapshot()
    def viewer_status(self, **observation):
        self.observations.append(observation)
        return {'observed':True}

def main():
    session=Session()
    server=ConsoleServer(('127.0.0.1',0), session)
    worker=threading.Thread(target=server.serve_forever,daemon=True)
    worker.start()
    try:
        with sync_playwright() as p:
            browser=p.chromium.launch(channel='chrome',headless=True)
            page=browser.new_page()
            visits=[]
            def desktop(route):
                visits.append(session.connected)
                if session.connected:
                    route.fulfill(status=200,content_type='text/html',body='''<p>Interactive desktop connected</p>
                    <script>parent.postMessage({type:'drone-sim-desktop',
                    run_id:new URLSearchParams(location.search).get('run'),connected:true},'*');</script>''')
                else:
                    route.abort('connectionrefused')
            page.route('http://127.0.0.1:6080/**',desktop)
            page.goto(f'http://127.0.0.1:{server.server_address[1]}')
            page.wait_for_timeout(500)
            assert page.locator('#start').count() == 0
            assert page.locator('#abort-flight').count() == 0
            assert page.locator('#prepare').count() == 1
            assert page.locator('#stop').count() == 1
            assert page.locator('#again').count() == 1
            guidance = page.locator('#qgc-guidance').inner_text()
            assert 'Application Settings' in guidance
            assert 'Fly View Actions' in guidance
            assert 'Start selected mission' in guidance
            assert not page.locator('#manual-note').is_visible()
            page.locator('#preset').select_option('operator-wait')
            assert page.locator('#manual-note').is_visible()
            assert 'GUIDED' in page.locator('#manual-note').inner_text()
            page.locator('#prepare').click()
            expect(page.locator('#session-state')).to_have_text('preparing')
            first_prepare = session.actions[0]
            assert first_prepare[0] == 'prepare'
            assert first_prepare[1]['preset_id'] == 'operator-wait'
            assert first_prepare[1]['preparation_wall_seconds'] == 1800
            UUID(first_prepare[1]['request_id'])
            assert session.run_id == '11111111-1111-4111-8111-111111111111'
            session.state='PREPARED'
            expect(page.locator('#session-state')).to_have_text('prepared', timeout=4000)
            assert '/tmp/QGroundControl/MavlinkActions/drone-sim-11111111-1111-4111-8111-111111111111.json' in page.locator('#qgc-action-path').inner_text()
            session.connected=True
            expect(page.locator('#gazebo-status')).to_have_text('Live', timeout=4000)
            assert any(visits), f'Gazebo did not reconnect when runtime became available: {visits}'
            assert session.observations == [{'run_id':'11111111-1111-4111-8111-111111111111', 'connected':True}]
            assert 'waiting for disarmed preflight' in page.locator('#flight-detail').inner_text()
            session.state='RUNNING'
            expect(page.locator('#session-state')).to_have_text('running', timeout=4000)
            assert 'onboard mission owns flight commands' in page.locator('#flight-detail').inner_text()
            assert 'waiting for disarmed preflight' not in page.locator('#flight-detail').inner_text()
            session.failed=True
            expect(page.locator('#session-state')).to_have_text('Console disconnected', timeout=4000)
            assert page.locator('#gazebo-status').inner_text() == 'Disconnected'
            assert 'STALE' in page.locator('#camera-stale').inner_text()
            assert page.locator('#camera-stale').is_visible()
            session.failed=False
            expect(page.locator('#session-state')).to_have_text('running', timeout=4000)
            page.locator('#stop').click()
            expect(page.locator('#session-state')).to_have_text('aborted')
            assert session.actions[1] == ('stop', {'run_id':'11111111-1111-4111-8111-111111111111'})
            assert page.locator('#again').is_visible()
            page.locator('#again').click()
            expect(page.locator('#session-state')).to_have_text('preparing')
            second_prepare = session.actions[2]
            assert second_prepare[0] == 'prepare'
            assert second_prepare[1]['preset_id'] == 'operator-wait'
            UUID(second_prepare[1]['request_id'])
            assert second_prepare[1]['request_id'] != first_prepare[1]['request_id']
            assert session.run_id == '22222222-2222-4222-8222-222222222222'
            assert [action for action, _request in session.actions] == ['prepare', 'stop', 'prepare']
            print('PASS: native-QGC guidance, Prepare/Stop/Run Again requests, desktop reconnect, and disconnected view indicators')
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
        worker.join()


if __name__ == "__main__":
    main()
