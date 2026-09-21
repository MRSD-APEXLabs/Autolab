#!/usr/bin/env bash
# Make sure the CANivore's SocketCAN network is up.  Idempotent, needs NO sudo.
#
#   scripts/canivore_up.sh           bring it up if it is not already, print what happened
#   scripts/canivore_up.sh --quiet   only complain on failure (used by the run scripts)
#
# WHY THIS EXISTS: the canivore_usb kernel driver creates the interface when the CANivore is
# plugged in, but nothing brings the CAN network UP - CTRE's `caniv -a -s` does that, and it has
# to be re-run after every re-plug, reset or reboot.  Until it runs, the CANivore's LED stays
# ORANGE and phoenix6 cannot reach the drivetrain.
#
# Two traps this script exists to avoid:
#   * The interface NAME is not stable.  The CANivore has been can4 and can0 on this machine, and
#     the Jetson's own on-chip mttcan controllers occupy can0..can4 with nothing attached, so a
#     hardcoded name silently checks the wrong device.  We match the driver canivore_usb instead.
#   * `caniv` and `canivore_setup` are usable WITHOUT sudo here (canivore_setup is setuid root and
#     caniv succeeds as a plain user), so the run scripts can fix this themselves.
#
# SAFETY: bringing a CAN interface up only opens the socket.  It sends no motor commands and does
# not enable anything: phoenix6 still has to feed the CTRE enable signal before a motor can move.
set -euo pipefail

QUIET=0
[[ "${1:-}" == --quiet ]] && QUIET=1
say() { ((QUIET)) || echo "$@"; }

# Every network interface whose kernel driver is canivore_usb (there is normally exactly one).
canivore_ifaces() {
  local d drv
  for d in /sys/class/net/*; do
    drv="$(basename "$(readlink -f "$d/device/driver" 2> /dev/null)" 2> /dev/null || true)"
    [[ "$drv" == canivore_usb ]] && basename "$d"
  done
  return 0
}

iface_is_up() { ip -br link show dev "$1" 2> /dev/null | grep -qw UP; }

mapfile -t ifaces < <(canivore_ifaces)

if ((${#ifaces[@]} > 0)) && iface_is_up "${ifaces[0]}"; then
  say "CANivore network already up on ${ifaces[0]}"
  exit 0
fi

if ! command -v caniv > /dev/null; then
  echo "canivore_up: caniv is not installed (package canivore-usb)" >&2
  exit 1
fi

if ((${#ifaces[@]} == 0)); then
  say "no canivore_usb interface yet; asking caniv to set the CANivore up"
else
  say "CANivore interface ${ifaces[0]} is DOWN; bringing the network up"
fi

# `caniv -a -s` = every discovered CANivore, set up its network.  No sudo needed.
if ! caniv -a -s > /dev/null 2>&1; then
  echo "canivore_up: 'caniv -a -s' failed" >&2
  exit 1
fi
sleep 1

mapfile -t ifaces < <(canivore_ifaces)
if ((${#ifaces[@]} == 0)); then
  echo "canivore_up: still no canivore_usb interface - is the CANivore plugged in? (check: caniv -l)" >&2
  exit 1
fi
if ! iface_is_up "${ifaces[0]}"; then
  echo "canivore_up: ${ifaces[0]} is still DOWN after 'caniv -a -s'" >&2
  echo "             try re-plugging the CANivore, then: caniv -l" >&2
  exit 1
fi

state="$(ip -details link show dev "${ifaces[0]}" 2> /dev/null | sed -n 's/.*can .*state \([A-Z-]*\).*/\1/p' | head -1)"
say "CANivore network up on ${ifaces[0]} (controller state ${state:-unknown})"
[[ "$state" == ERROR-ACTIVE || "$state" == "" ]] && exit 0
echo "canivore_up: warning - controller state $state (wiring / termination / robot power?)" >&2
exit 0
