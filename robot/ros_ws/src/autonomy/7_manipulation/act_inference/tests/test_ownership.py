from types import SimpleNamespace

import pytest

from act_inference.ownership import verify_hardware_quiet


def component(state):
    return SimpleNamespace(name='uf_robot_hardware/UFRobotSystemHardware',
                           plugin_name='uf_robot_hardware/UFRobotSystemHardware',
                           state=SimpleNamespace(id=state, label=str(state)))


@pytest.mark.parametrize('state', [0, 2, 3, 10, 15])
def test_blocks_unknown_transitional_active_and_inactive_hardware(state):
    with pytest.raises(RuntimeError, match='Unsafe concurrent'):
        verify_hardware_quiet([component(state)])


@pytest.mark.parametrize('state', [1, 4])
def test_accepts_only_states_that_do_not_call_hardware_write(state):
    verify_hardware_quiet([component(state)])


def test_no_matching_arm_is_not_proof_of_exclusive_control():
    with pytest.raises(RuntimeError, match='no UFRobotSystemHardware'):
        verify_hardware_quiet([])


def test_node_blocks_execution_before_camera_or_motion():
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    from types import MethodType
    calls = []
    future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(component=[component(3)]))
    client = SimpleNamespace(wait_for_service=lambda **kwargs: True, call_async=lambda request: future)
    node = SimpleNamespace(rollout=SimpleNamespace(execute=True), get_node_names=lambda: ['act_inference'],
                           _hardware_client=client, _prepare_camera=lambda check: calls.append('camera'))
    node._verify_motion_ownership = MethodType(ActInferenceNode._verify_motion_ownership, node)
    with pytest.raises(RuntimeError, match='Unsafe concurrent'):
        ActInferenceNode._prepare_run(node, lambda: None)
    assert not calls


def test_duplicate_nodes_are_blocked_before_hardware_query():
    pytest.importorskip('rclpy')
    from act_inference.node import ActInferenceNode
    node = SimpleNamespace(rollout=SimpleNamespace(execute=True),
                           get_node_names=lambda: ['act_inference', 'act_inference'])
    with pytest.raises(RuntimeError, match='Multiple act_inference'):
        ActInferenceNode._verify_motion_ownership(node, lambda: None)
