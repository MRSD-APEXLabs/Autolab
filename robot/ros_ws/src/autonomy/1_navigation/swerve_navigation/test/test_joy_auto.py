"""Controller layout, selection, and held-deadman handover regressions; no hardware."""
from swerve_navigation.joy_auto import Handover, NEUTRAL_AXES, SDLControllers, normalize


def test_neutral_and_trigger_polarity():
    axes, buttons = normalize([0] * 6, [0] * 15, 0.15)
    assert axes == NEUTRAL_AXES
    axes, _ = normalize([0, 0, 0, 0, 32767, 32767], [0] * 15, 0.15)
    assert axes[2] == axes[5] == -1.0
    assert buttons == [0] * 11


def test_stick_signs_deadzone_and_button_mapping():
    axes, _ = normalize([-32768, 1000, 32767, -32768, 0, 0], [0] * 15, 0.15)
    assert axes == [1.0, 0.0, 1.0, -1.0, 1.0, 1.0, 0.0, 0.0]
    for output, source in enumerate((0, 1, 2, 3, 9, 10, 4, 6, 5, 7, 8)):
        buttons = [0] * 15
        buttons[source] = 1
        _, mapped = normalize([0] * 6, buttons, 0.15)
        assert mapped[output] == 1
        assert sum(mapped) == 1


def test_handover_blocks_held_trigger_and_requires_release():
    gate = Handover()
    held = NEUTRAL_AXES.copy()
    held[2], held[4] = -1.0, 1.0
    released = [0] * 11
    assert gate.filter(1, held, released)[0] == NEUTRAL_AXES
    assert gate.filter(1, held, released)[0] == NEUTRAL_AXES
    gate.filter(1, NEUTRAL_AXES.copy(), released)
    assert gate.filter(1, held, released)[0] == held
    assert gate.filter(2, held, released)[0] == NEUTRAL_AXES
    assert gate.filter(2, held, released)[0] == NEUTRAL_AXES
    assert gate.filter(None, NEUTRAL_AXES.copy(), released)[0] == NEUTRAL_AXES
    assert gate.filter(3, held, released)[0] == NEUTRAL_AXES


def test_estop_during_gate_and_no_held_start():
    gate = Handover()
    buttons = [0] * 11
    gate.filter(1, NEUTRAL_AXES.copy(), buttons)
    buttons[1] = buttons[7] = 1
    _, safe = gate.filter(1, NEUTRAL_AXES.copy(), buttons)
    assert safe[1] == 1 and safe[7] == 0
    assert not gate.ready


class FakeSDL:
    """Mutable device inventory including failed opens and hotplug instance IDs."""
    def __init__(self):
        self.devices = [(1, b'Xbox 360 Controller'), (2, b'Xbox One S Controller'),
                        (3, b'3Dconnexion SpaceMouse Wireless BT')]
        self.unreadable = set()
        self.closed = []

    def SDL_PumpEvents(self): pass
    def SDL_FlushEvents(self, *args): pass
    def SDL_GameControllerUpdate(self): pass
    def SDL_NumJoysticks(self): return len(self.devices)
    def SDL_JoystickNameForIndex(self, i): return self.devices[i][1]
    def SDL_IsGameController(self, i): return self.devices[i][0] != 3
    def SDL_JoystickGetDeviceInstanceID(self, i): return self.devices[i][0]
    def SDL_GameControllerGetAttached(self, handle):
        return any(identity == handle for identity, _ in self.devices)
    def SDL_GameControllerOpen(self, i):
        identity = self.devices[i][0]
        return None if identity in self.unreadable else identity
    def SDL_GameControllerClose(self, handle): self.closed.append(handle)
    def SDL_GameControllerGetAxis(self, *args): return 0
    def SDL_GameControllerGetButton(self, *args): return 0


def test_preference_disconnect_fallback_reconnect_and_no_pad():
    driver = SDLControllers.__new__(SDLControllers)
    driver.sdl, driver.handles = FakeSDL(), {}
    def read():
        return driver.read('Xbox One S Controller', 'Xbox 360 Controller', 0.15)[0]
    assert read() == 2
    driver.sdl.devices.pop(1)
    assert read() == 1
    assert driver.sdl.closed == [2]
    driver.sdl.devices.append((4, b'Xbox One S Controller'))
    assert read() == 4
    driver.sdl.devices = [(3, b'3Dconnexion SpaceMouse Wireless BT')]
    assert read() is None


def test_inaccessible_bluetooth_falls_back():
    driver = SDLControllers.__new__(SDLControllers)
    driver.sdl, driver.handles = FakeSDL(), {}
    driver.sdl.unreadable.add(2)
    assert driver.read('Xbox One S Controller', 'Xbox 360 Controller', 0.15)[0] == 1


def test_handover_stops_teleop_and_preserves_latched_estop():
    from swerve_navigation.joy_teleop import TeleopConfig, TeleopLogic, ZERO
    logic, gate = TeleopLogic(TeleopConfig(field_centric=False)), Handover()
    buttons = [0] * 11
    held = NEUTRAL_AXES.copy()
    held[2], held[4] = -1.0, 1.0

    def feed(identity, axes, pressed, now):
        axes, pressed = gate.filter(identity, axes, pressed)
        logic.on_joy(axes, pressed, now)
        return logic.tick(now)

    feed(1, NEUTRAL_AXES.copy(), buttons, 0.0)
    feed(1, NEUTRAL_AXES.copy(), buttons, 0.05)
    assert feed(1, held, buttons, 0.1).command[0] > 0
    assert feed(2, held, buttons, 0.15).command == ZERO
    assert feed(2, held, buttons, 0.2).command is None
    buttons[1] = 1
    feed(2, held, buttons, 0.25)
    assert logic.e_stop
    feed(None, NEUTRAL_AXES.copy(), [0] * 11, 0.3)
    feed(3, NEUTRAL_AXES.copy(), [0] * 11, 0.35)
    feed(3, NEUTRAL_AXES.copy(), [0] * 11, 0.4)
    assert feed(3, held, [0] * 11, 0.45).command == ZERO
    assert logic.e_stop
