#!/usr/bin/env bash
set -euo pipefail

# PX4 must be built separately. The launcher connects to the already running
# Gazebo world created by run_s500_gazebo_wsl.sh; it does not spawn a second one.
PX4_HOME="${PX4_AUTOPILOT_HOME:-}"

if [[ -z "$PX4_HOME" ]]; then
  for candidate in "$HOME/PX4-Autopilot" "/home/$(id -un)/PX4-Autopilot"; do
    if [[ -x "$candidate/build/px4_sitl_default/bin/px4" ]]; then
      PX4_HOME="$candidate"
      break
    fi
  done
fi

if [[ -z "$PX4_HOME" ]]; then
  PX4_HOME="$HOME/PX4-Autopilot"
fi
PX4_BIN="$PX4_HOME/build/px4_sitl_default/bin/px4"

if [[ ! -x "$PX4_BIN" ]]; then
  echo "PX4 SITL binary not found: $PX4_BIN" >&2
  echo "Set PX4_AUTOPILOT_HOME or build PX4 with: make px4_sitl" >&2
  exit 1
fi

export PX4_SIMULATOR=gz
export PX4_GZ_STANDALONE=1
export PX4_SYS_AUTOSTART=4001
export PX4_GZ_WORLD=s500_empty
export PX4_GZ_MODEL_NAME=s500_quad_x
# Allow QGroundControl on the Windows host to receive MAVLink from WSL2.
# PX4's POSIX rcS consumes PX4_PARAM_* before the airframe starts.
export PX4_PARAM_MAV_0_BROADCAST=1
unset PX4_SIM_MODEL

cd "$PX4_HOME"
exec "$PX4_BIN"
