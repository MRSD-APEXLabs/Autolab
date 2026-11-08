"""Prefer the Bluetooth Pro 2, fall back to USB, publish the existing /joy layout.

SDL's GameController API normalizes differing raw button layouts (in particular Guide
and stick clicks). Only explicitly named devices are eligible; never select joystick 0.
This module can also be used without ROS for device inspection and unit tests.
"""
import ctypes as ct
import math


NEUTRAL_AXES = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]


def normalize(axes, buttons, deadzone):
    """SDL controller axes/buttons -> the raw Xbox layout expected by joy_teleop."""
    def stick(value):
        value = max(-1.0, min(1.0, value / 32767.0))
        if abs(value) <= deadzone:
            return 0.0
        return -math.copysign((abs(value) - deadzone) / (1.0 - deadzone), value)

    def trigger(value):
        value = max(0.0, min(1.0, value / 32767.0))
        value = max(0.0, (value - deadzone) / (1.0 - deadzone))
        return 1.0 - 2.0 * value

    # SDL axes LX, LY, RX, RY, LT, RT; buttons use SDL_GameControllerButton.
    out_axes = [stick(axes[0]), stick(axes[1]), trigger(axes[4]),
                stick(axes[2]), stick(axes[3]), trigger(axes[5]),
                float(buttons[13] - buttons[14]), float(buttons[11] - buttons[12])]
    out_buttons = [int(buttons[i]) for i in (0, 1, 2, 3, 9, 10, 4, 6, 5, 7, 8)]
    return out_axes, out_buttons


class Handover:
    """Suppress held controls until a neutral sample after every connection change.

    B remains available during the gate, but Start/A/X cannot generate actions from
    controls held on the previous device. A neutral frame separates controllers.
    """
    def __init__(self):
        self.identity = None
        self.ready = False

    def filter(self, identity, axes, buttons):
        if identity != self.identity:
            self.identity = identity
            self.ready = False
            safe_buttons = [0] * 11
            safe_buttons[1] = buttons[1]
            return NEUTRAL_AXES.copy(), safe_buttons
        if not self.ready:
            if (all(axes[i] == 0.0 for i in (0, 1, 3, 4))
                    and axes[2] == 1.0 and axes[5] == 1.0 and not any(buttons)):
                self.ready = True
            safe_buttons = [0] * 11
            safe_buttons[1] = buttons[1]  # e-stop must always work
            return NEUTRAL_AXES.copy(), safe_buttons
        return axes, buttons


class SDLControllers:
    def __init__(self):
        self.sdl = ct.CDLL('libSDL2-2.0.so.0')
        functions = {
            'SDL_Init': (ct.c_int, [ct.c_uint]),
            'SDL_SetHint': (ct.c_int, [ct.c_char_p, ct.c_char_p]),
            'SDL_NumJoysticks': (ct.c_int, []),
            'SDL_JoystickNameForIndex': (ct.c_char_p, [ct.c_int]),
            'SDL_JoystickGetDeviceInstanceID': (ct.c_int32, [ct.c_int]),
            'SDL_IsGameController': (ct.c_int, [ct.c_int]),
            'SDL_GameControllerOpen': (ct.c_void_p, [ct.c_int]),
            'SDL_GameControllerClose': (None, [ct.c_void_p]),
            'SDL_GameControllerGetAttached': (ct.c_int, [ct.c_void_p]),
            'SDL_GameControllerGetAxis': (ct.c_int16, [ct.c_void_p, ct.c_int]),
            'SDL_GameControllerGetButton': (ct.c_uint8, [ct.c_void_p, ct.c_int]),
            'SDL_PumpEvents': (None, []),
            'SDL_FlushEvents': (None, [ct.c_uint32, ct.c_uint32]),
            'SDL_GameControllerUpdate': (None, []),
            'SDL_GetError': (ct.c_char_p, []),
            'SDL_Quit': (None, []),
        }
        for name, (result, args) in functions.items():
            function = getattr(self.sdl, name)
            function.restype, function.argtypes = result, args
        self.handles = {}
        self.sdl.SDL_SetHint(b'SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', b'1')
        if self.sdl.SDL_Init(0x2000 | 0x200) != 0:  # GAMECONTROLLER | JOYSTICK
            raise RuntimeError(self.sdl.SDL_GetError().decode())

    def read(self, preferred, fallback, deadzone):
        sdl = self.sdl
        sdl.SDL_PumpEvents()
        sdl.SDL_FlushEvents(0, 0xffff)  # polling state, do not accumulate unused events
        for identity, handle in list(self.handles.items()):
            if not sdl.SDL_GameControllerGetAttached(handle):
                sdl.SDL_GameControllerClose(handle)
                del self.handles[identity]
        candidates = []
        for index in range(sdl.SDL_NumJoysticks()):
            name = (sdl.SDL_JoystickNameForIndex(index) or b'').decode()
            if name not in (preferred, fallback) or not sdl.SDL_IsGameController(index):
                continue
            identity = sdl.SDL_JoystickGetDeviceInstanceID(index)
            if identity not in self.handles:
                handle = sdl.SDL_GameControllerOpen(index)
                if not handle:
                    continue  # inaccessible preferred device must not prevent fallback
                self.handles[identity] = handle
            candidates.append((name != preferred, identity, name))
        if not candidates:
            return None, 'none', NEUTRAL_AXES.copy(), [0] * 11
        _, identity, name = min(candidates)
        handle = self.handles[identity]
        sdl.SDL_GameControllerUpdate()
        axes, buttons = normalize(
            [sdl.SDL_GameControllerGetAxis(handle, i) for i in range(6)],
            [sdl.SDL_GameControllerGetButton(handle, i) for i in range(15)], deadzone)
        return identity, name, axes, buttons

    def close(self):
        for handle in self.handles.values():
            self.sdl.SDL_GameControllerClose(handle)
        self.handles.clear()
        self.sdl.SDL_Quit()


def main(args=None):
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy

    rclpy.init(args=args)
    node = Node('joy_node')  # keep the existing config/node name
    preferred = node.declare_parameter('preferred_device_name', 'Xbox One S Controller').value
    fallback = node.declare_parameter('fallback_device_name', 'Xbox 360 Controller').value
    deadzone = float(node.declare_parameter('deadzone', 0.15).value)
    rate = float(node.declare_parameter('autorepeat_rate', 20.0).value)
    if not 0.0 <= deadzone < 1.0 or not 5.0 <= rate <= 200.0:
        raise ValueError('deadzone must be in [0, 1); autorepeat_rate in [5, 200] Hz')
    driver = SDLControllers()
    gate = Handover()
    pub = node.create_publisher(Joy, 'joy', 10)
    status = node.create_publisher(String, 'joy/selected_device', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    previous = object()

    def poll():
        nonlocal previous
        identity, name, axes, buttons = driver.read(preferred, fallback, deadzone)
        if identity != previous:
            previous = identity
            node.get_logger().info(
                f'Controller: {name}; release LT and centre sticks to enable teleop.')
            status.publish(String(data=name))
        axes, buttons = gate.filter(identity, axes, buttons)
        # Publish neutral on loss as well: immediately releases teleop; no stale held LT.
        msg = Joy()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.axes, msg.buttons = axes, buttons
        pub.publish(msg)

    node.create_timer(1.0 / rate, poll, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        driver.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
