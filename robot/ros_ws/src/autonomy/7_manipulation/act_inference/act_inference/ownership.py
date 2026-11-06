"""Explicit ROS-controller/SDK handoff, acknowledged by the patched hardware driver."""
import time


class DriverHandoff:
    def __init__(self, status, list_controllers, switch, acquire, release, ready,
                 controllers=('xarm6_traj_controller',), robot_ip=None):
        self.status = status
        self.list_controllers = list_controllers
        self.switch = switch
        self.acquire_driver = acquire
        self.release_driver = release
        self.ready = ready
        self.controllers = tuple(controllers)
        self.robot_ip = robot_ip
        self.held = False
        self.original_active = []

    def _status(self, check_abort):
        state = self.status(check_abort)
        if state.get('protocol') != 1 or not isinstance(state.get('sdk_owned'), bool):
            raise RuntimeError('Driver does not support the SDK ownership protocol; rebuild xarm_controller')
        if not isinstance(state.get('claimed'), int) or not isinstance(state.get('resync_pending'), bool):
            raise RuntimeError('Invalid driver ownership status')
        if self.robot_ip is not None and state.get('robot_ip') != self.robot_ip:
            raise RuntimeError('Driver/ACT robot_ip mismatch; refusing control of a different arm')
        return state

    def acquire(self, check_abort):
        check_abort()
        state = self._status(check_abort)  # Prove the driver is patched before changing controllers.
        if self.held:
            if not state['sdk_owned'] or state['claimed']:
                raise RuntimeError('Previously held SDK ownership was lost')
            return
        if state['sdk_owned']:
            raise RuntimeError('SDK control is already held by another run/node; recovery is required')
        controllers = {controller.name: controller.state for controller in self.list_controllers(check_abort)}
        missing = set(self.controllers) - controllers.keys()
        if missing:
            raise RuntimeError(f'Arm controllers not found: {sorted(missing)}')
        self.original_active = [name for name in self.controllers if controllers[name] == 'active']
        if self.original_active and not self.switch(self.original_active, [], check_abort):
            raise RuntimeError('Could not stop ROS arm controllers; no SDK motion enabled')
        if not self.acquire_driver(check_abort):
            raise RuntimeError('Driver refused SDK ownership; another ROS interface may still be claimed')
        self.held = True  # Retain this state even if cancellation happens immediately after acquisition.
        check_abort()
        state = self._status(check_abort)
        if not state['sdk_owned'] or state['claimed']:
            raise RuntimeError('Driver did not confirm exclusive SDK ownership')

    def finish(self, check_abort):
        """Call only after a successful run; failure/stop retains ownership and keeps ROS silent."""
        check_abort()
        if not self.held:
            raise RuntimeError('Cannot hand back arm without SDK ownership')
        self.ready(check_abort)  # No fault clearing; prepare mode 1/state 0 with no pose command.
        # The driver's independent report socket may still show the stopped state
        # from the completed move after our SDK connection confirms readiness.
        # Retry only that explicit rejection: no re-enabling or motion commands.
        deadline = time.monotonic() + 2.0
        while True:
            check_abort()
            response = self.release_driver(check_abort)
            if getattr(response, 'success', response):
                break
            reason = getattr(response, 'message', '')
            if (not reason.startswith('Arm must be fault-free in servo mode and ready before handback')
                    or time.monotonic() >= deadline):
                detail = f': {reason}' if reason else ''
                raise RuntimeError(f'Driver refused handback; SDK ownership retained{detail}')
            state = self._status(check_abort)
            if not state['sdk_owned'] or state['claimed']:
                raise RuntimeError('Exclusive SDK ownership lost while waiting for driver readiness')
            time.sleep(.05)
        self.held = False
        deadline = time.monotonic() + 3.0
        while True:
            state = self._status(check_abort)
            if state['sdk_owned'] or state['claimed']:
                raise RuntimeError('Unexpected command owner during joint resynchronization')
            if not state['resync_pending']:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError('Measured joint resynchronization timed out; ROS controllers remain stopped')
            time.sleep(.02)
        if self.original_active and not self.switch([], self.original_active, check_abort):
            raise RuntimeError('Could not restore ROS arm controllers after SDK handback')
        self.original_active = []
