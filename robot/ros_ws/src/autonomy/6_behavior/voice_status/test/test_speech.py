import array
import ctypes
import threading
import time
import wave
from types import SimpleNamespace

import pytest

from voice_status.speech import AlsaStream, Announcer, PiperVoice, idle_noise


def fake_piper(rate=22050, channels=1, frames=100, returncode=0):
    calls = []

    def run(cmd, input, capture_output, timeout):
        calls.append((cmd, input))
        if returncode == 0:
            with wave.open(cmd[cmd.index('--output_file') + 1], 'wb') as wav:
                wav.setnchannels(channels)
                wav.setsampwidth(2)
                wav.setframerate(rate)
                wav.writeframes(b'\x01\x02' * channels * frames)
        return SimpleNamespace(returncode=returncode, stderr=b'model not found')
    return run, calls


def test_piper_synthesize_reads_wav():
    run, calls = fake_piper()
    rate, channels, pcm = PiperVoice('/v/piper', '/v/model.onnx', 1.2, run=run).synthesize('Moving home.')
    assert (rate, channels, len(pcm)) == (22050, 1, 200)
    assert calls[0][0][:5] == ['/v/piper', '--model', '/v/model.onnx', '--length_scale', '1.2']
    assert calls[0][1] == b'Moving home.'


def test_piper_failure_raises():
    run, _ = fake_piper(returncode=1)
    with pytest.raises(RuntimeError, match='model not found'):
        PiperVoice('p', 'm', run=run).synthesize('x')


class FakeAlsa:
    """libasound stand-in: records opens/closes and every byte written; writes take a little time."""

    def __init__(self, open_codes=(), script=()):
        self.open_codes, self.script = list(open_codes), list(script)
        self.events, self.written = [], b''
        self.channels = 1

    def snd_pcm_open(self, handle, device, stream, mode):
        code = self.open_codes.pop(0) if self.open_codes else 0
        self.events.append(('open', device, code))
        return code

    def __getattr__(self, name):
        # snd_pcm_hw_params_* calls: record the int arguments, accept everything
        if not name.startswith('snd_pcm_hw_params'):
            raise AttributeError(name)

        def call(*args):
            if name == 'snd_pcm_hw_params_set_channels':
                self.channels = args[2]
            self.events.append((name.replace('snd_pcm_hw_params', 'hw'),) + tuple(
                a for a in args[2:] if isinstance(a, int)))
            return 0
        return call

    def snd_pcm_writei(self, handle, address, frames):
        time.sleep(0.001)
        result = self.script.pop(0) if self.script else min(frames, 300)   # partial writes
        if result > 0:
            self.written += ctypes.string_at(address, result * 2 * self.channels)
        return result

    def snd_pcm_recover(self, handle, code, silent):
        self.events.append(('recover', code))
        return 0

    def snd_pcm_drain(self, handle):
        self.events.append(('drain',))

    def snd_pcm_close(self, handle):
        self.events.append(('close',))

    def snd_strerror(self, code):
        return b'Device or resource busy'


def names(alsa, prefix):
    return [e for e in alsa.events if e[0].startswith(prefix)]


def test_stream_stays_open_writing_silence_and_plays_phrases_whole():
    alsa = FakeAlsa(script=[-32])                        # one underrun right at the start
    stream = AlsaStream('plughw:CARD=HDA,DEV=3', print, lib_factory=lambda: alsa)
    time.sleep(0.05)
    phrase = bytes(range(1, 201)) * 10                   # 1000 frames, no zero bytes
    stream.play(22050, 1, phrase)
    stream.play(22050, 1, phrase)
    time.sleep(0.02)
    stream.close()
    assert alsa.written.count(phrase) == 2                # each phrase written whole, never split by idle fill
    assert idle_noise(1024, 1) in alsa.written           # idle fill between phrases
    assert names(alsa, 'open') == [('open', b'plughw:CARD=HDA,DEV=3', 0)]   # opened once, never per phrase
    assert ('recover', -32) in alsa.events
    assert ('hw_set_access', 3) in alsa.events and ('hw_set_format', 2) in alsa.events
    assert ('hw_set_channels', 1) in alsa.events
    assert alsa.events[-2:] == [('drain',), ('close',)]


def test_stream_reopens_for_another_format():
    alsa = FakeAlsa()
    stream = AlsaStream('hw:0,3', print, lib_factory=lambda: alsa)
    stream.play(16000, 2, b'\x01\x02\x03\x04' * 10)
    stream.close()
    assert len(names(alsa, 'open')) == 2
    assert ('hw_set_channels', 2) in alsa.events
    assert alsa.written.count(b'\x01\x02\x03\x04' * 10) == 1


def test_stream_unavailable_fails_phrases_warns_once_and_recovers():
    warnings = []
    alsa = FakeAlsa(open_codes=[-16, -16, -16])
    stream = AlsaStream('hw:0,3', warnings.append, retry_s=0.02, lib_factory=lambda: alsa)
    with pytest.raises(OSError):
        stream.play(22050, 1, b'\x01\x01' * 10)
    time.sleep(0.1)
    stream.play(22050, 1, b'\x02\x02' * 10)              # device back: plays
    stream.close()
    assert alsa.written.count(b'\x02\x02' * 10) == 1 and b'\x01\x01' * 10 not in alsa.written
    assert len(warnings) == 2 and 'busy' in warnings[0] and 'working again' in warnings[1]


def test_idle_noise_is_never_zero_and_inaudibly_small():
    samples = array.array('h', idle_noise(1024, 2))
    assert len(samples) == 2048
    assert 0 not in samples and max(abs(v) for v in samples) == 2


def test_announcer_plays_in_order_caches_and_survives_errors():
    played, warned, done = [], [], threading.Event()

    class Voice:
        calls = 0

        def synthesize(self, text):
            Voice.calls += 1
            if text == 'bad':
                raise RuntimeError('no voice')
            return 22050, 1, text.encode()

    class Player:
        def play(self, rate, channels, pcm):
            played.append(pcm.decode())
            if len(played) == 3:
                done.set()

    announcer = Announcer(Voice(), Player(), lambda m: None, warned.append)
    for text in ['Moving home.', 'bad', 'Arrived home.', 'Moving home.']:
        announcer.say(text)
    assert done.wait(5)
    announcer.close()
    assert played == ['Moving home.', 'Arrived home.', 'Moving home.']
    assert Voice.calls == 3
    assert warned and 'no voice' in warned[0]
