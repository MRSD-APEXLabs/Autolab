"""The page's own logic in headless Chrome, against the HTTP API with synthetic frames and a scripted hub (no ROS).

Skipped without google-chrome or the websocket-client package.
"""
from contextlib import contextmanager
import json
import shutil
import threading
import time

import pytest

pytest.importorskip('websocket')
from camera_ui.frames import FrameStore, Renderer  # noqa: E402
from camera_ui.web import CameraUIApp  # noqa: E402
from headless_chrome import Chrome  # noqa: E402
from ui_fakes import INTRINSICS, depth_ramp, hub_status, jpeg, rgb_image, serving  # noqa: E402

CHROME = shutil.which('google-chrome') or shutil.which('chromium') or shutil.which('chromium-browser')
pytestmark = pytest.mark.skipif(CHROME is None, reason='needs google-chrome or chromium')
LABELS = {'zedx': 'ZED X (base)', 'zedx_nano': 'ZED X Nano (wrist)'}


@pytest.fixture(scope='module')
def chrome():
    browser = Chrome(CHROME)
    yield browser
    browser.close()


class ScriptedHub:
    """A CameraUIApp whose hub status and streaming camera the test sets with `set()` while the page watches.

    Every 0.1 s it re-sends the status and, while the active camera streams, a frame of each stream.
    """

    def __init__(self, select=None, active='zedx_nano', state='streaming'):
        self.app = CameraUIApp(Renderer(FrameStore(('zedx', 'zedx_nano'), stale_after=1.0)), LABELS, select=select)
        self.set(active, state)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def set(self, active, state, error=None):
        self.hub = hub_status(active=active, state=state, error=error)

    def _run(self):
        image, depth, raw = rgb_image(), depth_ramp(400, 1600), jpeg(rgb_image())
        while not self._stop.is_set():
            hub = self.hub
            self.app.set_hub_status(json.dumps(hub))
            if hub['state'] == 'streaming':
                for stream, data in (('rect', image), ('depth', depth), ('info', INTRINSICS), ('raw', raw)):
                    self.app.store.update(hub['active'], stream, data, stamp_ns=time.time_ns())
            self._stop.wait(0.1)

    @contextmanager
    def page(self, chrome, path='/'):
        """The page open in `chrome` once it shows the status; yields its URL."""
        try:
            with serving(self.app) as port:
                url = f'http://127.0.0.1:{port}{path}'
                chrome.open(url)
                chrome.wait('ui.status !== null && ui.view !== null')
                yield url
        finally:
            self._stop.set()
            self._thread.join()


def test_snapshot_links_only_while_there_is_a_current_frame(chrome):
    hub = ScriptedHub(active='zedx', state='starting')   # no frames yet
    with hub.page(chrome):
        chrome.wait('$("hubText").textContent.includes("starting")')
        for snap in ('rgbSnap', 'depthSnap'):
            assert chrome.js(f'!$("{snap}").hasAttribute("href") && $("{snap}").classList.contains("off")')
        chrome.click('#depthSnap')
        time.sleep(0.5)
        assert chrome.js('location.pathname') == '/'   # the server would answer 503 JSON, replacing the page

        hub.set('zedx', 'streaming')
        chrome.wait('$("rgbSnap").getAttribute("href") === "/snapshot/zedx/rgb.png"')
        assert chrome.js('$("depthSnap").getAttribute("href")') == '/snapshot/zedx/depth.png'
        assert not chrome.js('$("depthSnap").classList.contains("off") || $("rgbSnap").classList.contains("off")')

        hub.set('zedx', 'starting')   # the frames stop and go stale after stale_after (1 s)
        chrome.wait('!$("rgbSnap").hasAttribute("href") && !$("depthSnap").hasAttribute("href")', timeout=4)
    assert chrome.exceptions == []


def test_a_manual_depth_range_is_checked_as_it_is_sent(chrome):
    hub = ScriptedHub()
    with hub.page(chrome) as url:
        chrome.wait('$("depthImg").naturalWidth > 0')
        chrome.click('#auto')   # manual range
        chrome.wait('$("depthImg").dataset.url.includes("&far=")')

        def enter(near, far):
            chrome.js(f'$("near").value = "{near}"; $("far").value = "{far}"; $("far").dispatchEvent(new Event("input")); 1')
            time.sleep(0.6)   # applied 350 ms after the last keystroke
        before = chrome.js('$("depthImg").dataset.url')
        enter('5.6', '5.9')   # both round to 6, and the server refuses near=6&far=6
        assert chrome.js('$("near").classList.contains("bad") && $("far").classList.contains("bad")')
        assert chrome.js('$("depthImg").dataset.url') == before and chrome.js('[prefs.near, prefs.far]') != [6, 6]
        enter('300', '1500')
        chrome.wait('$("depthImg").dataset.url.endsWith("?near=300&far=1500")')
        assert not chrome.js('$("near").classList.contains("bad")')

        # a range no page can send any more, stored by an older one: the defaults instead
        chrome.js('localStorage.setItem("camera_ui.depth", JSON.stringify({auto: false, near: 6, far: 6})); 1')
        chrome.open(url + '?again')
        chrome.wait('($("depthImg").dataset.url || "").endsWith("?near=200&far=2000")')
    assert chrome.exceptions == []


def test_a_failed_switch_notice_goes_once_the_hub_gets_there(chrome):
    def select(camera):
        hub.set(camera, 'error', 'failed to open')
        return 200, {'ok': False, 'message': 'failed to open'}
    hub = ScriptedHub(select)
    with hub.page(chrome):
        chrome.click('#seg button[data-cam="zedx"]')
        chrome.wait('!$("notice").hidden && '
                    '$("noticeText").textContent === "Switching to ZED X (base) failed: failed to open"')
        assert chrome.js('!$("noticeClose").hidden')
        hub.set('zedx', 'streaming')   # the hub's background retry worked
        chrome.wait('$("notice").hidden', timeout=3)

        chrome.click('#seg button[data-cam="zedx_nano"]')
        chrome.wait('$("noticeText").textContent === "Switching to ZED X Nano (wrist) failed: failed to open"')
        chrome.click('#noticeClose')   # dismissed; the hub's own error stays while it lasts
        chrome.wait('$("noticeText").textContent === "ZED X Nano (wrist): failed to open" && $("noticeClose").hidden')
    assert chrome.exceptions == []


def test_release_and_the_other_camera_stay_usable_while_a_switch_waits(chrome):
    answer, calls = threading.Event(), []

    def select(camera):
        calls.append(camera)
        if camera == 'zedx':   # a camera that never delivers; the release supersedes it at the hub
            hub.set('zedx', 'starting')
            answer.wait(10)
            return 200, {'ok': False, 'message': 'selection changed to None while waiting'}
        hub.set(None, 'idle')
        return 200, {'ok': True, 'message': 'no camera idle'}
    hub = ScriptedHub(select)
    try:
        with hub.page(chrome):
            chrome.click('#seg button[data-cam="zedx"]')
            chrome.wait('$("seg").querySelector("[data-cam=zedx]").disabled')
            assert chrome.js('!!$("seg").querySelector("[data-cam=zedx] .spin")')
            assert chrome.js('!$("release").disabled && !$("seg").querySelector("[data-cam=zedx_nano]").disabled')
            chrome.click('#release')
            chrome.wait('ui.pending === null && $("hubText").textContent.endsWith("no camera selected")')
            assert calls == ['zedx', None]
            answer.set()   # the superseded switch answers last, and is not reported
            time.sleep(1.0)
            assert chrome.js('$("notice").hidden')
            overlays = chrome.js('[...document.querySelectorAll(".overlay b")].map(b => b.textContent)')
            assert overlays == ['No camera selected'] * 2
    finally:
        answer.set()
    assert chrome.exceptions == []
