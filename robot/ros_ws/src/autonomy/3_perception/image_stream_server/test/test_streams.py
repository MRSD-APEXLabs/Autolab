"""parse_streams and StreamHub (no ROS)."""
import threading

import pytest

from image_stream_server.streams import StreamHub, parse_streams

KEY = ('annotated', 'zedx')
STREAMS = {KEY: '/inspect/annotated_image', ('raw', 'zedx'): '/zedx/left/image_rect_color'}


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def make_hub(encode=None, clock=None):
    calls = []

    def default_encode(message):
        calls.append(message)
        return b'jpeg:' + message.encode()
    return StreamHub(STREAMS, encode or default_encode, clock=clock or Clock()), calls


def test_parse_streams_valid():
    assert parse_streams(['annotated/zedx=/inspect/annotated_image',
                          ' raw/zedx_nano = /zedx_nano/left/image_rect_color ']) == {
        ('annotated', 'zedx'): '/inspect/annotated_image',
        ('raw', 'zedx_nano'): '/zedx_nano/left/image_rect_color'}


@pytest.mark.parametrize('entry', ['annotated/zedx', 'zedx=/topic', 'annotated/zedx=topic', '1bad/zedx=/t',
                                   'annotated/ze-dx=/t', 'annotated/zedx/extra=/t', ''])
def test_parse_streams_rejects_bad_entries(entry):
    with pytest.raises(ValueError, match='bad stream entry'):
        parse_streams([entry])


def test_parse_streams_rejects_duplicates_and_empty():
    with pytest.raises(ValueError, match='duplicate'):
        parse_streams(['raw/zedx=/a', 'raw/zedx=/b'])
    with pytest.raises(ValueError, match='empty'):
        parse_streams([])


def test_viewer_counting_drives_wanted():
    hub, _ = make_hub()
    assert hub.wanted() == set()
    hub.acquire(KEY)
    hub.acquire(KEY)
    assert hub.wanted() == {KEY}
    hub.release(KEY)
    assert hub.wanted() == {KEY}
    hub.release(KEY)
    assert hub.wanted() == set()
    hub.release(KEY)   # extra release never goes negative
    hub.acquire(KEY)
    assert hub.wanted() == {KEY}


def test_latest_jpeg_encodes_each_frame_once():
    hub, calls = make_hub()
    assert hub.latest_jpeg(KEY) == (None, None)
    hub.acquire(KEY)
    hub.update(KEY, 'a')
    seq, jpeg = hub.latest_jpeg(KEY)
    assert jpeg == b'jpeg:a'
    assert hub.latest_jpeg(KEY) == (seq, jpeg)
    assert calls == ['a']
    hub.update(KEY, 'b')
    seq2, jpeg2 = hub.latest_jpeg(KEY)
    assert seq2 > seq and jpeg2 == b'jpeg:b' and calls == ['a', 'b']


def test_concurrent_readers_share_one_encode():
    gate = threading.Event()
    calls = []

    def slow_encode(message):
        calls.append(message)
        gate.wait(2.0)
        return b'x'
    hub, _ = make_hub(encode=slow_encode)
    hub.acquire(KEY)
    hub.update(KEY, 'a')
    results = []
    threads = [threading.Thread(target=lambda: results.append(hub.latest_jpeg(KEY))) for _ in range(4)]
    for t in threads:
        t.start()
    gate.set()
    for t in threads:
        t.join(2.0)
    assert calls == ['a'] and len(results) == 4 and len(set(results)) == 1


def test_encode_error_is_reported_and_not_retried():
    calls = []

    def bad_encode(message):
        calls.append(message)
        raise ValueError('bad encoding')
    hub, _ = make_hub(encode=bad_encode)
    hub.acquire(KEY)
    hub.update(KEY, 'a')
    assert hub.latest_jpeg(KEY)[1] is None
    assert hub.latest_jpeg(KEY)[1] is None
    assert calls == ['a']
    assert 'bad encoding' in hub.health()['annotated/zedx']['error']


def test_last_release_drops_the_cached_frame():
    hub, _ = make_hub()
    hub.acquire(KEY)
    hub.update(KEY, 'old')
    assert hub.latest_jpeg(KEY)[1] == b'jpeg:old'
    hub.release(KEY)
    hub.acquire(KEY)   # a viewer minutes later must not see the old frame
    assert hub.latest_jpeg(KEY) == (None, None)


def test_health_reports_topic_subscription_viewers_and_age():
    clock = Clock()
    hub, _ = make_hub(clock=clock)
    health = hub.health()
    assert health['annotated/zedx'] == {'topic': '/inspect/annotated_image', 'subscribed': False, 'viewers': 0,
                                        'last_frame_age_s': None, 'error': None}
    hub.acquire(KEY)
    hub.set_subscribed({KEY})
    hub.update(KEY, 'a')
    clock.now += 1.5
    health = hub.health()['annotated/zedx']
    assert health['subscribed'] is True and health['viewers'] == 1
    assert health['last_frame_age_s'] == pytest.approx(1.5)
    assert hub.health()['raw/zedx']['subscribed'] is False


def test_frames_arriving_with_no_viewer_are_dropped():
    # The node unsubscribes up to one reconcile period after the last viewer leaves; a frame
    # arriving in that window must not be kept for a viewer who comes back much later.
    hub, _ = make_hub()
    hub.acquire(KEY)
    hub.release(KEY)
    hub.update(KEY, 'late')
    hub.acquire(KEY)
    assert hub.latest_jpeg(KEY) == (None, None)
