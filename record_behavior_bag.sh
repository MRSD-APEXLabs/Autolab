#!/usr/bin/env bash
# record_behavior_bag.sh — Record behavior topics for routine executor debugging.
#
# Run this INSIDE the robot container while a routine is executing:
#   autolab connect robot
#   bash /path/to/record_behavior_bag.sh [ROBOT_NAME]
#
# ROBOT_NAME defaults to robot_1.
# Output bag is written to ~/behavior_debug_<timestamp>/
#
# Ctrl+C to stop recording.

ROBOT="${1:-robot_1}"
STAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT="$HOME/behavior_debug_${STAMP}"

echo "Recording behavior topics for robot '${ROBOT}' → ${OUTPUT}"
echo "Ctrl+C to stop."

ros2 bag record -o "${OUTPUT}" \
    "/${ROBOT}/behavior/manipulation_command" \
    "/${ROBOT}/behavior/lab_machine_command" \
    \
    "/${ROBOT}/behavior/pick_base_commanded_success" \
    "/${ROBOT}/behavior/pick_up_commanded_success" \
    "/${ROBOT}/behavior/place_commanded_success" \
    "/${ROBOT}/behavior/command_lab_machine_success" \
    "/${ROBOT}/behavior/ot2_liquid_handler_success" \
    "/${ROBOT}/behavior/custom_shaker_module_success" \
    \
    "/${ROBOT}/behavior/pick_base_object_active" \
    "/${ROBOT}/behavior/pick_up_object_active" \
    "/${ROBOT}/behavior/place_object_active" \
    "/${ROBOT}/behavior/execute_ot2_protocol_active" \
    "/${ROBOT}/behavior/execute_shaker_protocol_active" \
    \
    "/${ROBOT}/behavior/pick_base_object_status" \
    "/${ROBOT}/behavior/pick_up_object_status" \
    "/${ROBOT}/behavior/place_object_status" \
    "/${ROBOT}/behavior/execute_ot2_protocol_status" \
    "/${ROBOT}/behavior/execute_shaker_protocol_status" \
    \
    "/${ROBOT}/behavior/manipulation_phase" \
    "/${ROBOT}/behavior/active_actions" \
    \
    "/planning_command" \
    "/planning_state" \
    "/routine_executor/status"