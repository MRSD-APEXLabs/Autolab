"""Fail closed when ros2_control can still write to the same xArm as the SDK."""


def verify_hardware_quiet(components):
    arms = [component for component in components
            if 'UFRobotSystemHardware' in component.name
            or 'UFRobotSystemHardware' in component.plugin_name]
    if not arms:
        raise RuntimeError('Cannot verify xArm command ownership: no UFRobotSystemHardware found')
    for arm in arms:
        # Humble calls System::write in BOTH active (3) and inactive (2).
        # Merely stopping a trajectory controller does not silence this custom driver.
        if arm.state.id not in (1, 4):  # unconfigured or finalized: write is not called
            raise RuntimeError(
                f'Unsafe concurrent xArm control: ROS hardware {arm.name} is {arm.state.label} '
                f'(state {arm.state.id}) and can send joint servo commands while ACT sends Cartesian targets. '
                'Execution blocked until an exclusive driver handoff is implemented; '
                'deactivating only the trajectory controller is insufficient.')
