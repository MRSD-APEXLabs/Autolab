#!/usr/bin/env python3
"""Start a trajectory controller, reconciling lifecycle state after lost replies."""

import argparse
import sys
import time


class ReplyTimeout(RuntimeError):
    """A service may have executed even though its reply was not received."""


def ensure_ready(backend, timeout, clock=time.monotonic, sleep=time.sleep):
    deadline = clock() + timeout
    last_error = 'controller has not been inspected'
    rejected = None
    while backend.running() and clock() < deadline:
        try:
            # Never infer the outcome of a mutation from its reply alone. In
            # particular, do not repeat load_controller after an ambiguous reply.
            state = backend.state(deadline)
            if rejected is not None and state == rejected[0]:
                raise RuntimeError(f'{rejected[1]} was rejected; controller state is {state!r}')
            rejected = None
            if state == 'active':
                if backend.action_ready(deadline):
                    return
                last_error = 'controller is active but its trajectory action server is unavailable'
            else:
                operation = {None: 'load', 'unconfigured': 'configure', 'inactive': 'activate'}.get(state)
                if operation is None:
                    raise RuntimeError(f'Cannot start controller in lifecycle state {state!r}')
                last_error = f'waiting for {operation} to complete from state {state!r}'
                if not backend.transition(operation, deadline):
                    # Reconcile even an explicit rejection: another request may
                    # have completed the transition before this one arrived.
                    rejected = (state, operation)
        except ReplyTimeout as error:
            last_error = str(error)
            backend.warn(f'{error}; inspecting controller state before any further transition')
        sleep(min(0.2, max(0.0, deadline - clock())))
    raise RuntimeError(f'Controller startup did not complete within {timeout:g}s: {last_error}')


class RosBackend:
    def __init__(self, node, controller, manager, service_timeout):
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from controller_manager_msgs.srv import ConfigureController, ListControllers
        from controller_manager_msgs.srv import LoadController, SwitchController
        from rclpy.action import ActionClient

        self.ros = rclpy
        self.node = node
        self.controller = controller
        self.service_timeout = service_timeout
        self.types = {
            'list_controllers': ListControllers,
            'load_controller': LoadController,
            'configure_controller': ConfigureController,
            'switch_controller': SwitchController,
        }
        self.clients = {
            name: node.create_client(service_type, f'{manager}/{name}')
            for name, service_type in self.types.items()
        }
        namespace = manager.rsplit('/', 1)[0]
        self.action = ActionClient(
            node, FollowJointTrajectory,
            f'{namespace}/{controller}/follow_joint_trajectory')

    def running(self):
        return self.ros.ok()

    def warn(self, message):
        self.node.get_logger().warning(message)

    def _call(self, name, request, deadline):
        client = self.clients[name]
        remaining = min(self.service_timeout, max(0.0, deadline - time.monotonic()))
        call_deadline = time.monotonic() + remaining
        if not client.wait_for_service(timeout_sec=remaining):
            raise ReplyTimeout(f'{client.srv_name} unavailable')
        if time.monotonic() >= call_deadline:
            raise ReplyTimeout(f'{client.srv_name} discovery timed out')
        future = client.call_async(request)
        self.ros.spin_until_future_complete(
            self.node, future, timeout_sec=max(0.0, call_deadline - time.monotonic()))
        if not future.done():
            client.remove_pending_request(future)
            raise ReplyTimeout(f'{client.srv_name} reply timed out')
        return future.result()

    def state(self, deadline):
        response = self._call(
            'list_controllers', self.types['list_controllers'].Request(), deadline)
        return next((c.state for c in response.controller if c.name == self.controller), None)

    def transition(self, operation, deadline):
        name = {'load': 'load_controller', 'configure': 'configure_controller',
                'activate': 'switch_controller'}[operation]
        request = self.types[name].Request()
        if operation == 'activate':
            request.activate_controllers = [self.controller]
            request.strictness = request.STRICT
            request.activate_asap = True
            # This server-side switch wait fits inside the client-side wait.
            seconds = min(3.0, self.service_timeout / 2,
                          max(0.0, deadline - time.monotonic()) / 2)
            request.timeout.sec = int(seconds)
            request.timeout.nanosec = int((seconds - int(seconds)) * 1e9)
        else:
            request.name = self.controller
        self.node.get_logger().info(f'{operation}: {self.controller}')
        return self._call(name, request, deadline).ok

    def action_ready(self, deadline):
        return self.action.wait_for_server(
            timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))

    def close(self):
        self.action.destroy()
        for client in self.clients.values():
            self.node.destroy_client(client)


def main(argv=None):
    import rclpy
    from rclpy.utilities import remove_ros_args

    argv = sys.argv if argv is None else argv
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('controller')
    parser.add_argument('--controller-manager', default='/controller_manager')
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--service-timeout', type=float, default=5.0)
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    if args.timeout <= 0 or args.service_timeout <= 0:
        parser.error('timeouts must be positive')
    manager = '/' + args.controller_manager.strip('/')
    rclpy.init(args=argv)
    node = rclpy.create_node('start_trajectory_controller')
    backend = None
    try:
        backend = RosBackend(node, args.controller, manager, args.service_timeout)
        ensure_ready(backend, args.timeout)
        node.get_logger().info(f'{args.controller} is active and its trajectory action server is ready')
        return 0
    except (RuntimeError, KeyboardInterrupt) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        if backend is not None:
            backend.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
