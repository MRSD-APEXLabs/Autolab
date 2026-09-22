"""Headless Chrome over the DevTools protocol, just enough to drive the camera UI page in tests."""
import base64
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
import urllib.request

import websocket


class Chrome:
    """One headless Chrome tab. `js()` evaluates in the page; `click()` is a real mouse click, so disabled and
    pointer-events: none elements ignore it as they would for a user."""

    def __init__(self, executable, width=1280, height=900):
        self.profile = tempfile.mkdtemp(prefix='camera-ui-chrome-')
        self.proc = subprocess.Popen(
            [executable, '--headless=new', '--disable-gpu', '--no-first-run', '--no-default-browser-check',
             '--remote-debugging-port=0', f'--user-data-dir={self.profile}', f'--window-size={width},{height}',
             'about:blank'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.exceptions = []
        self._id = 0
        try:
            port_file = Path(self.profile) / 'DevToolsActivePort'   # written once Chrome listens
            deadline = time.monotonic() + 20
            while not (port_file.exists() and len(port_file.read_text().split()) >= 2):
                if time.monotonic() > deadline or self.proc.poll() is not None:
                    raise RuntimeError('headless Chrome did not start')
                time.sleep(0.05)
            port = int(port_file.read_text().split()[0])
            with urllib.request.urlopen(f'http://127.0.0.1:{port}/json', timeout=5) as response:
                page = next(t for t in json.load(response) if t['type'] == 'page')
            self.ws = websocket.create_connection(page['webSocketDebuggerUrl'], suppress_origin=True, timeout=15)
            self.call('Runtime.enable')
            self.call('Page.enable')
        except Exception:
            self.close()
            raise

    def call(self, method, **params):
        self._id += 1
        self.ws.send(json.dumps({'id': self._id, 'method': method, 'params': params}))
        while True:
            message = json.loads(self.ws.recv())
            if message.get('method') == 'Runtime.exceptionThrown':
                details = message['params']['exceptionDetails']
                self.exceptions.append(details.get('exception', {}).get('description') or details.get('text'))
            if message.get('id') == self._id:
                if 'error' in message:
                    raise RuntimeError(f'{method}: {message["error"]}')
                return message['result']

    def js(self, expression):
        result = self.call('Runtime.evaluate', expression=expression, returnByValue=True, awaitPromise=True)
        if 'exceptionDetails' in result:
            raise RuntimeError(f'{expression}: {result["exceptionDetails"]}')
        return result['result'].get('value')

    def wait(self, expression, timeout=5.0):
        """The first truthy value of `expression` within `timeout` s; fails with its last value otherwise."""
        deadline = time.monotonic() + timeout
        while True:
            value = self.js(expression)
            if value or time.monotonic() > deadline:
                assert value, f'still {value!r} after {timeout} s: {expression}'
                return value
            time.sleep(0.05)

    def open(self, url):
        self.call('Page.navigate', url=url)
        self.wait('document.readyState === "complete" && location.href.startsWith(%s)' % json.dumps(url))

    def click(self, selector):
        x, y = self.js('(r => [r.x + r.width / 2, r.y + r.height / 2])(document.querySelector(%s).getBoundingClientRect())'
                       % json.dumps(selector))
        self.call('Input.dispatchMouseEvent', type='mouseMoved', x=x, y=y)
        for kind in ('mousePressed', 'mouseReleased'):
            self.call('Input.dispatchMouseEvent', type=kind, x=x, y=y, button='left', clickCount=1)

    def screenshot(self, path):
        Path(path).write_bytes(base64.b64decode(self.call('Page.captureScreenshot', format='png')['data']))

    def close(self):
        ws = getattr(self, 'ws', None)
        if ws is not None:
            ws.close()
        self.proc.terminate()
        try:
            self.proc.wait(10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        shutil.rmtree(self.profile, ignore_errors=True)
