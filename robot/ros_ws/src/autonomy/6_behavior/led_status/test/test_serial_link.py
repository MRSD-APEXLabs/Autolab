import os

from led_status.serial_link import LedLink, find_port

BY_ID = 'usb-1a86_USB_Serial-if00-port0'


# ---------------------------------------------------------------- find_port

def make_tty(tmp_path, name, vendor, product):
    """Fake /sys/class/tty/<name>/device -> <usbdev>/<iface>/<name>, like the kernel."""
    usbdev = tmp_path / 'devices' / f'usb-{name}'
    iface = usbdev / '1-1:1.0' / name
    iface.mkdir(parents=True)
    (usbdev / 'idVendor').write_text(vendor + '\n')
    (usbdev / 'idProduct').write_text(product + '\n')
    tty = tmp_path / 'class_tty' / name
    tty.mkdir(parents=True)
    os.symlink(iface, tty / 'device')
    (tmp_path / 'dev').mkdir(exist_ok=True)
    (tmp_path / 'dev' / name).write_text('')


def find(tmp_path, configured='', by_id=None):
    return find_port(configured, by_id or str(tmp_path / 'missing_by_id'),
                     sysfs_root=str(tmp_path / 'class_tty'), dev_root=str(tmp_path / 'dev'))


def test_configured_port_wins(tmp_path):
    port = tmp_path / 'ttyX'
    port.write_text('')
    assert find(tmp_path, configured=str(port)) == str(port)


def test_configured_port_missing_returns_none(tmp_path):
    make_tty(tmp_path, 'ttyUSB0', '1a86', '7523')
    assert find(tmp_path, configured=str(tmp_path / 'nope')) is None


def test_by_id_path_used_when_present(tmp_path):
    by_id = tmp_path / BY_ID
    by_id.write_text('')
    assert find(tmp_path, by_id=str(by_id)) == str(by_id)


def test_scan_finds_ch340_and_skips_other_adapters(tmp_path):
    make_tty(tmp_path, 'ttyUSB0', '0403', '6001')  # FTDI: the other Nano on Thor
    make_tty(tmp_path, 'ttyUSB1', '1a86', '7523')  # CH340: LED Nano
    assert find(tmp_path) == str(tmp_path / 'dev' / 'ttyUSB1')


def test_scan_finds_nothing(tmp_path):
    make_tty(tmp_path, 'ttyUSB0', '0403', '6001')
    assert find(tmp_path) is None


# ---------------------------------------------------------------- LedLink

class FakePort:
    def __init__(self):
        self.written = []
        self.closed = False
        self.fail = False

    def write(self, data):
        if self.fail:
            raise OSError('unplugged')
        self.written.append(data)

    def close(self):
        self.closed = True


class FakeOpener:
    def __init__(self):
        self.ports = []
        self.available = True

    def __call__(self):
        if not self.available:
            raise OSError('no device')
        self.ports.append(FakePort())
        return self.ports[-1]


def make_link(opener):
    return LedLink(opener, resend_s=3.0, retry_s=5.0, settle_s=2.0)


def test_waits_for_nano_reset_before_sending():
    opener = FakeOpener()
    link = make_link(opener)
    link.update('I', 0.0)
    link.update('I', 1.9)
    assert opener.ports[0].written == []
    link.update('I', 2.0)
    assert opener.ports[0].written == [b'I\n']


def test_sends_only_on_change():
    opener = FakeOpener()
    link = make_link(opener)
    for t, letter in [(0.0, 'I'), (2.0, 'I'), (2.1, 'I'), (2.2, 'M'), (2.3, 'M')]:
        link.update(letter, t)
    assert opener.ports[0].written == [b'I\n', b'M\n']


def test_resends_periodically():
    opener = FakeOpener()
    link = make_link(opener)
    link.update('M', 0.0)
    link.update('M', 2.0)
    link.update('M', 4.9)
    link.update('M', 5.0)
    assert opener.ports[0].written == [b'M\n', b'M\n']


def test_retries_open_after_delay_when_missing():
    opener = FakeOpener()
    opener.available = False
    link = make_link(opener)
    link.update('I', 0.0)
    opener.available = True
    link.update('I', 4.9)
    assert opener.ports == []
    link.update('I', 5.0)
    link.update('I', 7.0)
    assert opener.ports[0].written == [b'I\n']


def test_reconnects_after_write_error():
    opener = FakeOpener()
    link = make_link(opener)
    link.update('I', 0.0)
    link.update('I', 2.0)
    opener.ports[0].fail = True
    link.update('M', 2.5)
    assert opener.ports[0].closed
    link.update('M', 7.5)   # reopen
    link.update('M', 9.5)   # settled
    assert opener.ports[1].written == [b'M\n']


def test_close_turns_leds_off():
    opener = FakeOpener()
    link = make_link(opener)
    link.update('I', 0.0)
    link.update('I', 2.0)
    link.close()
    assert opener.ports[0].written[-1] == b'O\n'
    assert opener.ports[0].closed


def test_close_without_port_is_safe():
    opener = FakeOpener()
    opener.available = False
    link = make_link(opener)
    link.update('I', 0.0)
    link.close()
