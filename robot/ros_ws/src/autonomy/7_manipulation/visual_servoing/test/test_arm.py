"""XArm against a stand-in for the SDK's XArmAPI."""
import threading
import types

import pytest

from visual_servoing.arm import ArmError, XArm


class ReportingAPI:
    """XArmAPI stand-in: like the SDK, tcp_offset reads zeros until the report thread has parsed the first report."""

    def __init__(self, ip, is_radian=False, delay=0.1):
        self.connected = True
        self.tcp_offset = [0.0] * 6
        self._arm = types.SimpleNamespace(_first_report_over=False)
        if delay is not None:
            threading.Timer(delay, self._report).start()

    def _report(self):
        self.tcp_offset = [0.0, 0.0, 140.0, 0.0, 0.0, 0.0]
        self._arm._first_report_over = True

    def disconnect(self):
        self.connected = False


def test_connect_reads_the_tcp_offset_after_the_first_report():
    arm = XArm('192.0.2.1', api_factory=ReportingAPI)
    arm.connect()
    assert arm.flange_T_tcp[2, 3] == pytest.approx(0.140)


def test_connect_fails_without_a_report(monkeypatch):
    monkeypatch.setattr(XArm._wait_first_report, '__defaults__', (0.1,))
    arm = XArm('192.0.2.1', api_factory=lambda ip, is_radian: ReportingAPI(ip, delay=None))
    with pytest.raises(ArmError, match='no state report'):
        arm.connect()
    assert not arm.connected
