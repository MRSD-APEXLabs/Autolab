#!/usr/bin/env bash
# Sourced by run.sh.  Points Fast DDS at the UDP-only profile in this directory unless the
# caller asked for stock behaviour with SWERVE_NAV_SHM=1.  See fastdds_no_shm.xml for why the
# shared-memory transport is off by default on this machine.
#
# Sets NAV_DDS_TRANSPORT to a one-word description for the startup banner.

_dds_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_dds_profile="$_dds_dir/fastdds_no_shm.xml"

if [[ "${SWERVE_NAV_SHM:-0}" == 1 ]]; then
  NAV_DDS_TRANSPORT="default (shared memory ON - SWERVE_NAV_SHM=1)"
elif [[ "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" != rmw_fastrtps_cpp ]]; then
  NAV_DDS_TRANSPORT="${RMW_IMPLEMENTATION} (the profile is Fast DDS only)"
  unset FASTRTPS_DEFAULT_PROFILES_FILE
elif [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -s "$FASTRTPS_DEFAULT_PROFILES_FILE" ]]; then
  # Someone already chose a real profile; do not fight them. Ignore missing/empty placeholders.
  NAV_DDS_TRANSPORT="profile from the environment: $FASTRTPS_DEFAULT_PROFILES_FILE"
elif [[ ! -f "$_dds_profile" ]]; then
  NAV_DDS_TRANSPORT="default (profile missing: $_dds_profile)"
else
  export FASTRTPS_DEFAULT_PROFILES_FILE="$_dds_profile"
  NAV_DDS_TRANSPORT="UDPv4 only, shared memory OFF"
fi
export NAV_DDS_TRANSPORT
unset _dds_dir _dds_profile
