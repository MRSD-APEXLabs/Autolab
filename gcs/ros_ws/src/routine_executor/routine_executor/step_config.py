import json
import math
from behavior_tree_msgs.msg import LabMachineCommand, ManipulationCommand
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


def make_pose_stamped(x: float, y: float, yaw: float) -> PoseStamped:
    """Build a map-frame PoseStamped from x, y [m] and yaw [rad] (REP-103: CCW from +x)."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return pose


STEPS = {
    'pick_base': {
        'msg_type': 'manipulation',
        'publish_topic': 'behavior/manipulation_command',
        'required_params': [],
        'make_msg': lambda params: ManipulationCommand(
            type='pick_base',
            object_type='well_plate',
            target_machine='',
        ),
        'watch_topic': 'behavior/pick_base_object_status',
    },
    'pick_up': {
        'msg_type': 'manipulation',
        'publish_topic': 'behavior/manipulation_command',
        'required_params': [],
        'make_msg': lambda params: ManipulationCommand(
            type='pick_up',
            object_type='well_plate',
            target_machine='',
        ),
        'watch_topic': 'behavior/pick_up_object_status',
    },
    'place': {
        'msg_type': 'manipulation',
        'publish_topic': 'behavior/manipulation_command',
        'required_params': ['target_machine'],
        'make_msg': lambda params: ManipulationCommand(
            type='place',
            object_type='well_plate',
            target_machine=params['target_machine'],
        ),
        'watch_topic': 'behavior/place_object_status',
    },
    'pick_wellplate': {
        'msg_type': 'manipulation',
        'publish_topic': 'behavior/manipulation_command',
        'required_params': ['target_machine'],
        'make_msg': lambda params: ManipulationCommand(
            type='pick',
            object_type='well_plate',
            target_machine=params['target_machine'],
        ),
        'watch_topic': 'behavior/pick_object_status',
    },
    'shaker': {
        'msg_type': 'lab_machine',
        'publish_topic': 'behavior/lab_machine_command',
        'required_params': ['pwm'],
        'make_msg': lambda params: LabMachineCommand(
            device='shaker',
            action='protocol',
            parameters_json=json.dumps({'pwm': params['pwm']}),
        ),
        'watch_topic': 'behavior/execute_shaker_protocol_status',
    },
    'ot2': {
        'msg_type': 'lab_machine',
        'publish_topic': 'behavior/lab_machine_command',
        'required_params': ['parameters_json'],
        'make_msg': lambda params: LabMachineCommand(
            device='ot2',
            action='protocol',
            parameters_json=params['parameters_json'] if isinstance(params['parameters_json'], str) else json.dumps(params['parameters_json']),
        ),
        'watch_topic': 'behavior/execute_ot2_protocol_status',
    },
    'wait': {
        'msg_type': 'wait',
        'publish_topic': None,
        'required_params': ['time_s'],
        'make_msg': None,
        'watch_topic': None,
    },
    'go_to': {
        'msg_type': 'string',
        'publish_topic': 'behavior/go_to_location_command',
        'required_params': ['target_machine'],
        'make_msg': lambda params: String(data=params['target_machine']),
        'watch_topic': 'behavior/go_to_location_status',
    },
    'home': {
        'msg_type': 'string',
        'publish_topic': 'behavior/go_to_location_command',
        'required_params': [],
        'make_msg': lambda params: String(data='home'),
        'watch_topic': 'behavior/go_to_location_status',
    },
    'go_to_pose': {
        'msg_type': 'pose',
        'publish_topic': 'behavior/navigate_to_pose_command',
        'required_params': ['x', 'y', 'yaw'],
        'make_msg': lambda params: make_pose_stamped(params['x'], params['y'], params['yaw']),
        'watch_topic': 'behavior/navigate_to_pose_status',
    },
}

KNOWN_STEPS = list(STEPS.keys())
