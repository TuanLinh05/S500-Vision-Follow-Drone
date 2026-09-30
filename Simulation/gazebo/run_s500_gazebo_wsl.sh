#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
WORLD="$SIM_ROOT/gazebo/worlds/s500_empty.sdf"
MODEL_PATH="$SIM_ROOT/gazebo/models"
RENDER_ENGINE="${S500_GZ_RENDER_ENGINE:-ogre}"
PX4_HOME="${PX4_AUTOPILOT_HOME:-$HOME/PX4-Autopilot}"
PX4_GZ_PLUGIN_PATH="$PX4_HOME/build/px4_sitl_default/src/modules/simulation/gz_plugins"

if ! command -v gz >/dev/null 2>&1; then
  echo "Gazebo command 'gz' was not found. Install Gazebo Harmonic or source its environment." >&2
  exit 1
fi

export GZ_SIM_RESOURCE_PATH="$MODEL_PATH${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
if [[ ! -f "$PX4_GZ_PLUGIN_PATH/libOpticalFlowSystem.so" ]]; then
  echo "PX4 OpticalFlow plugin not found: $PX4_GZ_PLUGIN_PATH/libOpticalFlowSystem.so" >&2
  echo "Build PX4 SITL first or set PX4_AUTOPILOT_HOME." >&2
  exit 1
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PX4_GZ_PLUGIN_PATH${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
echo "Starting Gazebo with renderer: $RENDER_ENGINE" >&2
exec gz sim -r --render-engine "$RENDER_ENGINE" "$WORLD"
