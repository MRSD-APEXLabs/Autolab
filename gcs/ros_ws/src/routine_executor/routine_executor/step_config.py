from behavior_tree_msgs.msg import LabMachineCommand, ManipulationCommand

# Full message payloads (object_type, target_machine, parameters_json, etc.)
# are intentionally minimal here — fill in per your application needs.

STEPS = {
    'pick_base': {
        'msg_type': 'manipulation',
        'publish_topic': 'manipulation_command',
        'make_msg': lambda: ManipulationCommand(
            type='pick_base',
            object_type='well_plate',
            target_machine='',
        ),
        'watch_topic': 'pick_base_object_status',
    },
    'place': {
        'msg_type': 'manipulation',
        'publish_topic': 'manipulation_command',
        'make_msg': lambda: ManipulationCommand(
            type='place',
            object_type='well_plate',
            target_machine='ot2',
        ),
        'watch_topic': 'place_object_status',
    },
    'pick_up': {
        'msg_type': 'manipulation',
        'publish_topic': 'manipulation_command',
        'make_msg': lambda: ManipulationCommand(
            type='pick_up',
            object_type='well_plate',
            target_machine='',
        ),
        'watch_topic': 'pick_up_object_status',
    },
    'ot2': {
        'msg_type': 'lab_machine',
        'publish_topic': 'lab_machine_command',
        'make_msg': lambda: LabMachineCommand(
            device='ot2',
            action='protocol',
            parameters_json='{}',
        ),
        'watch_topic': 'execute_ot2_protocol_status',
    },
    'shaker': {
        'msg_type': 'lab_machine',
        'publish_topic': 'lab_machine_command',
        'make_msg': lambda: LabMachineCommand(
            device='shaker',
            action='protocol',
            parameters_json='{}',
        ),
        'watch_topic': 'execute_shaker_protocol_status',
    },
}

KNOWN_STEPS = list(STEPS.keys())
