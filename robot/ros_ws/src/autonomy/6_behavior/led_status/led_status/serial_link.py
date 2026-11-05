"""Serial link to the underglow Nano (led/robot_status firmware). No ROS dependency.

The firmware takes one status letter per line at 115200 baud (see led/robot_status/README.md).
"""

import glob
import os
import termios

CH340_VENDOR = '1a86'
CH340_PRODUCT = '7523'
DEFAULT_BY_ID = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
OFF = 'O'


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def find_port(configured, by_id_path, sysfs_root='/sys/class/tty', dev_root='/dev'):
    """Return the LED Nano's device path, or None.

    A configured path is used as-is. Otherwise prefer the udev by-id symlink, then scan sysfs
    for a CH340 (the by-id symlinks are often missing inside containers).
    """
    if configured:
        return configured if os.path.exists(configured) else None
    if os.path.exists(by_id_path):
        return by_id_path
    for tty in sorted(glob.glob(os.path.join(sysfs_root, 'tty*'))):
        device = os.path.join(tty, 'device')
        if not os.path.exists(device):
            continue
        # <usb device>/<interface>/<tty>: ids live two levels up from the tty node.
        usbdev = os.path.dirname(os.path.dirname(os.path.realpath(device)))
        if (_read(os.path.join(usbdev, 'idVendor')) == CH340_VENDOR
                and _read(os.path.join(usbdev, 'idProduct')) == CH340_PRODUCT):
            dev = os.path.join(dev_root, os.path.basename(tty))
            if os.path.exists(dev):
                return dev
    return None


class SerialPort:
    """Raw 115200 8N1 port, configured like led/robot_status/send_status.py."""

    def __init__(self, path):
        self._fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            settings = termios.tcgetattr(self._fd)
            settings[0] = settings[1] = settings[3] = 0
            settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            settings[4] = settings[5] = termios.B115200
            settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
            termios.tcsetattr(self._fd, termios.TCSANOW, settings)
        except termios.error as e:
            os.close(self._fd)
            raise OSError(str(e)) from e

    def write(self, data):
        # Discard the firmware's acknowledgements so the input buffer never fills.
        try:
            while os.read(self._fd, 4096):
                pass
        except BlockingIOError:
            pass
        os.write(self._fd, data)

    def close(self):
        os.close(self._fd)


class LedLink:
    """Keeps the port open, sends a letter on change and periodically, reconnects on errors.

    ``opener`` returns an object with ``write(bytes)`` and ``close()``, or raises OSError.
    Opening USB serial resets the Nano, so nothing is sent until ``settle_s`` after opening.
    """

    def __init__(self, opener, resend_s=3.0, retry_s=5.0, settle_s=2.0):
        self._opener = opener
        self._resend_s = resend_s
        self._retry_s = retry_s
        self._settle_s = settle_s
        self._port = None
        self._next_open_t = None
        self._ready_t = None
        self._sent = None
        self._sent_t = None
        self.last_error = None

    @property
    def connected(self):
        return self._port is not None

    def _drop(self, now, error):
        self.last_error = error
        if self._port is not None:
            try:
                self._port.close()
            except OSError:
                pass
        self._port = None
        self._next_open_t = now + self._retry_s

    def update(self, letter, now):
        if self._port is None:
            if self._next_open_t is not None and now < self._next_open_t:
                return
            try:
                self._port = self._opener()
            except OSError as e:
                self._drop(now, e)
                return
            self.last_error = None
            self._ready_t = now + self._settle_s
            self._sent = None
        if now < self._ready_t:
            return
        if letter == self._sent and now - self._sent_t < self._resend_s:
            return
        try:
            self._port.write((letter + '\n').encode('ascii'))
        except OSError as e:
            self._drop(now, e)
            return
        self._sent = letter
        self._sent_t = now

    def close(self):
        if self._port is None:
            return
        try:
            self._port.write((OFF + '\n').encode('ascii'))
        except OSError:
            pass
        try:
            self._port.close()
        except OSError:
            pass
        self._port = None
