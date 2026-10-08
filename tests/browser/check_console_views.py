"""Chrome UI regression smoke: uv run --with playwright python tests/browser/check_console_views.py."""

import threading
from pathlib import Path
from playwright.sync_api import sync_playwright
from orchestration.console import ConsoleServer

class Session:
    project_directory = Path.cwd()
    connected = False
    failed = False
    observations = []
    def snapshot(self):
        if self.failed:
            raise RuntimeError('owner unavailable')
        return {'state':'PREPARED', 'run_id':'11111111-1111-4111-8111-111111111111', 'can_start':False,
                'views': {'gazebo': {'available':self.connected, 'connected':bool(self.observations)}, 'camera': {'fresh':self.connected}, 'qgc':{}}}
    def checks(self): return {'checks':[], 'ok':True}
    def results(self): return {}
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
            session.connected=True
            page.wait_for_timeout(2600)
            assert any(visits), f'Gazebo did not reconnect when runtime became available: {visits}'
            assert session.observations == [{'run_id':'11111111-1111-4111-8111-111111111111', 'connected':True}]
            session.failed=True
            page.wait_for_timeout(2600)
            assert page.locator('#gazebo-status').inner_text() == 'Disconnected'
            assert 'STALE' in page.locator('#camera-stale').inner_text()
            assert page.locator('#camera-stale').is_visible()
            assert page.locator('#start').is_disabled()
            print('PASS: desktop startup reconnect and disconnected view indicators')
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
        worker.join()


if __name__ == "__main__":
    main()
