#!/usr/bin/env bash
# test_full_routine.sh — End-to-end lab routine test
#
# Sequence:
#   1. Pick up well plate
#   2. Place well plate in OT2
#   3. Run OT2 protocol (tips → aspirate → dispense → return tips)
#   4. Pick up well plate
#   5. Place well plate in shaker
#   6. Activate shaker (PWM 100)

set -uo pipefail

# ── Topic names ───────────────────────────────────────────────────────────────

MANIP_TOPIC="/behavior/manipulation_command"
LAB_TOPIC="/behavior/lab_machine_command"
PICK_BASE_STATUS="/behavior/pick_base_object_status"
PLACE_STATUS="/behavior/place_object_status"

OT2_STATUS="/behavior/execute_ot2_protocol_status"
SHAKER_STATUS="/behavior/execute_shaker_protocol_status"

# ── Helpers ───────────────────────────────────────────────────────────────────

log() { echo "[$(date +%T)] $*"; }

# Wait for a BT action status topic to report SUCCESS (2).
# Exits with error on FAILURE (0) — but only after RUNNING (1) is seen first,
# to avoid acting on a stale FAILURE from a prior activation.
# Sleeps 1s first to let the BT activate the action after a command is sent.
wait_bt_success() {
    local topic="$1"
    local label="$2"
    local seen_running=0
    log "  Waiting for $label..."
    sleep 1
    while true; do
        output=$(ros2 topic echo --once --timeout 2 "$topic" 2>/dev/null || true)
        status=$(echo "$output" | grep "^status:" | awk '{print $2}')
        case "$status" in
            1) log "  $label: RUNNING"; seen_running=1 ;;
            2) log "  $label: SUCCESS"; return 0 ;;
            0) if [ "$seen_running" = "1" ]; then
                   log "ERROR: $label FAILED"; exit 1
               fi ;;
            # empty: topic went silent — if we saw RUNNING, the action completed successfully
            "") if [ "$seen_running" = "1" ]; then
                    log "  $label: SUCCESS (topic silent)"; return 0
                fi ;;
        esac
        sleep 0.5
    done
}

# ── Routine ───────────────────────────────────────────────────────────────────

log "=== Step 1: Pick up well plate ==="
ros2 topic pub --once "$MANIP_TOPIC" behavior_tree_msgs/msg/ManipulationCommand \
    "{'type': 'pick_base', 'object_type': 'well_plate', 'target_machine': ''}"
wait_bt_success "$PICK_BASE_STATUS" "Pick base"

log "=== Step 2: Place well plate in OT2 ==="
ros2 topic pub --once "$MANIP_TOPIC" behavior_tree_msgs/msg/ManipulationCommand \
    "{'type': 'place', 'object_type': 'well_plate', 'target_machine': 'ot2'}"
wait_bt_success "$PLACE_STATUS" "Place"

log "=== Step 3: Activate OT2 protocol ==="
ros2 topic pub --once "$LAB_TOPIC" behavior_tree_msgs/msg/LabMachineCommand \
    "{device: 'ot2', action: 'protocol', parameters_json: '{\"steps\": [{\"action\": \"pick_up_tips\", \"resource_name\": \"tip_rack\", \"well_indices\": [0]}, {\"action\": \"aspirate\", \"resource_name\": \"tube_rack\", \"well_indices\": [0], \"volumes\": [50]}, {\"action\": \"dispense\", \"resource_name\": \"empty_plate\", \"well_indices\": [0, 1, 2, 3, 4], \"volumes\": [10, 10, 10, 10, 10]}, {\"action\": \"return_tips\"}]}'}"
wait_bt_success "$OT2_STATUS" "OT2 protocol"

log "=== Step 6: Activate shaker (PWM 100) ==="
ros2 topic pub --once "$LAB_TOPIC" behavior_tree_msgs/msg/LabMachineCommand \
    "{device: 'shaker', action: 'protocol', parameters_json: '{\"pwm\": 100}'}"
wait_bt_success "$SHAKER_STATUS" "Shaker protocol"

sleep 10
log "=== Step 7: Stop shaker (PWM 0) ==="
ros2 topic pub --once "$LAB_TOPIC" behavior_tree_msgs/msg/LabMachineCommand \
    "{device: 'shaker', action: 'protocol', parameters_json: '{\"pwm\": 0}'}"
wait_bt_success "$SHAKER_STATUS" "Shaker protocol"

log "=== Routine complete ==="
