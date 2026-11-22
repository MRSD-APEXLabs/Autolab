"""Text to speech: Piper (offline neural TTS, the standalone aarch64 binary from fetch_voice.sh) renders a WAV,
which is played through ALSA via libasound and ctypes, so the image needs neither alsa-utils nor Python audio
packages.

The HDMI sink takes only 2-channel 32/44.1/48 kHz audio; the 'plughw' device converts Piper's 22.05 kHz mono.
The hardware parameters are set one by one, with an explicit period and buffer size: snd_pcm_set_params cannot
pick a period size for 22.05 kHz mono on this device ("Unable to get period size ... Invalid argument").
The HDMI sink mutes until it has locked onto an audio stream, which takes most of a second. Opening the device
per phrase clipped the first word ("Voice one" came out as "...ne"), so AlsaStream keeps the device open and
feeds it between phrases. What it feeds is not digital zero but noise of +-2 LSB (about -84 dBFS, inaudible):
with exact zeros the sink still muted and faded back in on each phrase, losing its first sound ("oice one").
"""
import array
import ctypes
import os
import queue
import random
import subprocess
import tempfile
import threading
import wave

SND_PCM_STREAM_PLAYBACK = 0
SND_PCM_FORMAT_S16_LE = 2
SND_PCM_ACCESS_RW_INTERLEAVED = 3
IDLE_LEVEL = 2   # peak of the idle noise in 16-bit steps


def idle_noise(frames, channels, level=IDLE_LEVEL, seed=0):
    """16-bit PCM noise that is never zero: keeps the sink from muting on digital silence, far below hearing."""
    rng = random.Random(seed)
    values = [v for v in range(-level, level + 1) if v != 0]
    return array.array('h', (rng.choice(values) for _ in range(frames * channels))).tobytes()


class PiperVoice:
    """`length_scale`: 1.0 is the voice's normal pace; larger is slower (1.2 = 20 % longer)."""

    def __init__(self, piper, model, length_scale=1.0, run=subprocess.run):
        self.piper, self.model, self.length_scale, self._run = piper, model, length_scale, run

    def synthesize(self, text):
        """(rate, channels, 16-bit PCM bytes) for `text`."""
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'say.wav')
            result = self._run([self.piper, '--model', self.model, '--length_scale', str(self.length_scale),
                                '--output_file', path], input=text.encode(), capture_output=True, timeout=30)
            if result.returncode != 0:
                raise RuntimeError(f'piper exited {result.returncode}: {result.stderr.decode(errors="replace")[-300:]}')
            with wave.open(path) as wav:
                if wav.getsampwidth() != 2:
                    raise RuntimeError(f'piper wrote {wav.getsampwidth() * 8}-bit audio, expected 16-bit')
                return wav.getframerate(), wav.getnchannels(), wav.readframes(wav.getnframes())


def load_libasound():
    lib = ctypes.CDLL('libasound.so.2')
    lib.snd_pcm_open.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
    p, uint, uframes = ctypes.c_void_p, ctypes.c_uint, ctypes.c_ulong
    lib.snd_pcm_hw_params_malloc.argtypes = [ctypes.POINTER(p)]
    lib.snd_pcm_hw_params_free.argtypes = [p]
    lib.snd_pcm_hw_params_any.argtypes = [p, p]
    lib.snd_pcm_hw_params_set_access.argtypes = [p, p, ctypes.c_int]
    lib.snd_pcm_hw_params_set_format.argtypes = [p, p, ctypes.c_int]
    lib.snd_pcm_hw_params_set_channels.argtypes = [p, p, uint]
    lib.snd_pcm_hw_params_set_rate_resample.argtypes = [p, p, uint]
    lib.snd_pcm_hw_params_set_rate_near.argtypes = [p, p, ctypes.POINTER(uint), ctypes.POINTER(ctypes.c_int)]
    lib.snd_pcm_hw_params_set_period_size_near.argtypes = [p, p, ctypes.POINTER(uframes),
                                                           ctypes.POINTER(ctypes.c_int)]
    lib.snd_pcm_hw_params_set_buffer_size_near.argtypes = [p, p, ctypes.POINTER(uframes)]
    lib.snd_pcm_hw_params.argtypes = [p, p]
    lib.snd_pcm_writei.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
    lib.snd_pcm_writei.restype = ctypes.c_long
    lib.snd_pcm_recover.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.snd_pcm_drain.argtypes = [ctypes.c_void_p]
    lib.snd_pcm_close.argtypes = [ctypes.c_void_p]
    lib.snd_strerror.argtypes = [ctypes.c_int]
    lib.snd_strerror.restype = ctypes.c_char_p
    return lib


class _Phrase:
    def __init__(self, rate, channels, pcm):
        self.rate, self.channels, self.pcm = rate, channels, pcm
        self.done = threading.Event()
        self.error = None


class AlsaStream:
    """Plays phrases on an ALSA device that stays open, writing inaudible noise whenever there is nothing to say,
    so the HDMI sink stays locked and unmuted and phrases start at once. Nothing else can use the device while this is open.

    The device is opened for (`rate`, `channels`) and reopened if a phrase comes in another format. If it cannot be
    opened (busy, unplugged), phrases fail at once and it is retried every `retry_s`.
    """

    def __init__(self, device, warn, rate=22050, channels=1, period_frames=1024, buffer_frames=4096, retry_s=5.0,
                 lib_factory=load_libasound):
        self.device, self._warn = device, warn
        self._format = (rate, channels)
        self.period_frames, self.buffer_frames, self._retry_s = period_frames, buffer_frames, retry_s
        self._lib_factory, self._lib, self._handle = lib_factory, None, None
        self._failing = False
        self._idle = {}
        self._phrases = queue.Queue()
        self._closed = threading.Event()
        self._thread = threading.Thread(target=self._run, name='voice-status-alsa', daemon=True)
        self._thread.start()

    def play(self, rate, channels, pcm):
        """Blocks until the phrase has been written to the device (it finishes playing within one buffer)."""
        phrase = _Phrase(rate, channels, pcm)
        self._phrases.put(phrase)
        phrase.done.wait()
        if phrase.error is not None:
            raise phrase.error

    def close(self, timeout=5.0):
        self._closed.set()
        self._thread.join(timeout)

    def _run(self):
        while not self._closed.is_set():
            try:
                phrase = self._phrases.get_nowait()
            except queue.Empty:
                phrase = None
            try:
                if phrase is not None and (phrase.rate, phrase.channels) != self._format:
                    self._close_device(drain=True)
                    self._format = (phrase.rate, phrase.channels)
                if self._handle is None:
                    self._open()
                if phrase is None:
                    channels = self._format[1]
                    if channels not in self._idle:
                        self._idle[channels] = idle_noise(self.period_frames, channels)
                    self._write(self._idle[channels])   # blocks ~one period
                else:
                    self._write(phrase.pcm)
                if self._failing:
                    self._failing = False
                    self._warn(f'ALSA {self.device} is working again')
            except Exception as exc:
                self._close_device(drain=False)
                if phrase is not None:
                    phrase.error = exc
                elif not self._failing:
                    self._warn(f'{exc}; retrying every {self._retry_s:g} s')
                self._failing = True
            finally:
                if phrase is not None:
                    phrase.done.set()
            if self._failing:
                self._closed.wait(self._retry_s)
                self._fail_queued()
        self._close_device(drain=True)
        self._fail_queued()

    def _fail_queued(self):
        """While the device is down, fail waiting phrases instead of letting them pile up and play late."""
        while self._handle is None:
            try:
                phrase = self._phrases.get_nowait()
            except queue.Empty:
                return
            phrase.error = OSError(f'ALSA {self.device} is not available')
            phrase.done.set()

    def _error(self, what, code):
        return OSError(f'ALSA {what} on {self.device}: {self._lib.snd_strerror(code).decode()}')

    def _open(self):
        if self._lib is None:
            self._lib = self._lib_factory()
        handle = ctypes.c_void_p()
        code = self._lib.snd_pcm_open(ctypes.byref(handle), self.device.encode(), SND_PCM_STREAM_PLAYBACK, 0)
        if code < 0:
            raise self._error('open', code)
        try:
            self._set_hw_params(handle, *self._format)
        except Exception:
            self._lib.snd_pcm_close(handle)
            raise
        self._handle = handle

    def _close_device(self, drain):
        if self._handle is not None:
            if drain:
                self._lib.snd_pcm_drain(self._handle)
            self._lib.snd_pcm_close(self._handle)
            self._handle = None

    def _set_hw_params(self, handle, rate, channels):
        lib = self._lib
        params = ctypes.c_void_p()
        code = lib.snd_pcm_hw_params_malloc(ctypes.byref(params))
        if code < 0:
            raise self._error('hw_params_malloc', code)
        try:
            actual_rate, direction = ctypes.c_uint(rate), ctypes.c_int(0)
            period, buffer = ctypes.c_ulong(self.period_frames), ctypes.c_ulong(self.buffer_frames)
            for what, call in (
                    ('any', lambda: lib.snd_pcm_hw_params_any(handle, params)),
                    ('access', lambda: lib.snd_pcm_hw_params_set_access(handle, params,
                                                                        SND_PCM_ACCESS_RW_INTERLEAVED)),
                    ('format', lambda: lib.snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE)),
                    ('channels', lambda: lib.snd_pcm_hw_params_set_channels(handle, params, channels)),
                    ('resample', lambda: lib.snd_pcm_hw_params_set_rate_resample(handle, params, 1)),
                    ('rate', lambda: lib.snd_pcm_hw_params_set_rate_near(handle, params, ctypes.byref(actual_rate),
                                                                         ctypes.byref(direction))),
                    ('period size', lambda: lib.snd_pcm_hw_params_set_period_size_near(
                        handle, params, ctypes.byref(period), ctypes.byref(direction))),
                    ('buffer size', lambda: lib.snd_pcm_hw_params_set_buffer_size_near(handle, params,
                                                                                      ctypes.byref(buffer))),
                    ('hw_params', lambda: lib.snd_pcm_hw_params(handle, params))):
                code = call()
                if code < 0:
                    raise self._error(f'set {what}', code)
            if actual_rate.value != rate:
                raise OSError(f'ALSA {self.device} cannot play {rate} Hz (offered {actual_rate.value} Hz)')
        finally:
            lib.snd_pcm_hw_params_free(params)

    def _write(self, data):
        lib, frame = self._lib, 2 * self._format[1]
        buffer = ctypes.create_string_buffer(data, len(data))
        base, offset = ctypes.addressof(buffer), 0
        while offset < len(data):
            written = lib.snd_pcm_writei(self._handle, base + offset, (len(data) - offset) // frame)
            if written < 0:
                code = lib.snd_pcm_recover(self._handle, int(written), 1)   # underrun or suspend
                if code < 0:
                    raise self._error('write', code)
                continue
            offset += written * frame


class Announcer:
    """Says queued phrases in order on a worker thread. Each phrase is rendered once and kept; a phrase that
    fails (missing voice, busy device) is logged and the next one still plays."""

    def __init__(self, voice, player, info, warn):
        self._voice, self._player, self._info, self._warn = voice, player, info, warn
        self._audio = {}
        self._queue = queue.Queue()
        self._thread = threading.Thread(target=self._run, name='voice-status', daemon=True)
        self._thread.start()

    def say(self, text):
        self._queue.put(text)

    def _run(self):
        while True:
            text = self._queue.get()
            if text is None:
                return
            try:
                if text not in self._audio:
                    self._audio[text] = self._voice.synthesize(text)
                self._player.play(*self._audio[text])
                self._info(f'said: "{text}"')
            except Exception as exc:   # keep speaking later phrases
                self._warn(f'could not say "{text}": {exc}')

    def close(self, timeout=5.0):
        self._queue.put(None)
        self._thread.join(timeout)
