from behavior_tree_msgs.msg import LabMachineCommand, ManipulationCommand

# Full message payloads (object_type, target_machine, parameters_json, etc.)
# are intentionally minimal here — fill in per your application needs.

STEPS = {
    'pick_base': {
        'msg_type': 'manipulation',
        'publish_topic': 'behavior/manipulation_command',
        'make_msg': lambda: ManipulationCommand(
            type='pick_base',
            object_type='well_plate',
            target_machine='',
        ),
        'watch_topic': 'behavior/pick_base_object_status',
    },
    'place_ot2': {
            'msg_type': 'manipulation',
            'publish_topic': 'behavior/manipulation_command',
            'make_msg': lambda: ManipulationCommand(
                type='place',
                object_type='well_plate',
                target_machine='ot2',
            ),
            'watch_topic': 'behavior/place_object_status',
    },
    'place_shaker': {
        'msg_type': 'manipulation',
        'publish_topic': 'behavior/manipulation_command',
        'make_msg': lambda: ManipulationCommand(
            type='place',
            object_type='well_plate',
            target_machine='shaker',
        ),
        'watch_topic': 'behavior/place_object_status',
    },
    'pick_up': {
        'msg_type': 'manipulation',
        'publish_topic': 'behavior/manipulation_command',
        'make_msg': lambda: ManipulationCommand(
            type='pick_up',
            object_type='well_plate',
            target_machine='',
        ),
        'watch_topic': 'behavior/pick_up_object_status',
    },
    'ot2': {
        'msg_type': 'lab_machine',
        'publish_topic': 'behavior/lab_machine_command',
        'make_msg': lambda: LabMachineCommand(
            device='ot2',
            action='protocol',
            parameters_json='{}',
        ),
        'watch_topic': 'behavior/execute_ot2_protocol_status',
    },
    'shaker': {
        'msg_type': 'lab_machine',
        'publish_topic': 'behavior/lab_machine_command',
        'make_msg': lambda: LabMachineCommand(
            device='shaker',
            action='protocol',
            parameters_json='{"pwm": 125}',
        ),
        'watch_topic': 'behavior/execute_shaker_protocol_status',
    },
}

KNOWN_STEPS = list(STEPS.keys())
