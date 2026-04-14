import json
from behavior_tree_msgs.msg import LabMachineCommand, ManipulationCommand

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
            parameters_json=params['parameters_json'],
        ),
        'watch_topic': 'behavior/execute_ot2_protocol_status',
    },
}

KNOWN_STEPS = list(STEPS.keys())
