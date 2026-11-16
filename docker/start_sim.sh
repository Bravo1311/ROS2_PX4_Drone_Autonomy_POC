#!/usr/bin/env bash
# Starts the Micro XRCE-DDS Agent in the background and PX4 SITL + Gazebo in the foreground.
# Standalone alternative to `ros2 launch px4_bringup minimal.launch.py`, which already starts
# PX4 SITL, the XRCE agent and RViz. Do NOT run both: they fight over UDP port 8888.
set -e

# World and vehicle come from px4_bringup/config/sim.env, the same file the launch files read.
# A variable already exported in your shell wins over the file.
# This path does not start the /clock bridge; run it in another terminal:
#   ros2 launch px4_bringup sim_clock.launch.py
SIM_ENV="${SIM_ENV_FILE:-$(dirname "$0")/../px4_bringup/config/sim.env}"
[ -f "$SIM_ENV" ] || { echo "sim.env not found: $SIM_ENV" >&2; exit 1; }
while IFS='=' read -r key value; do
    case "$key" in ''|\#*) continue ;; esac
    [ -z "${!key:-}" ] && export "$key=$value"
done < "$SIM_ENV"

WORLD="${PX4_GZ_WORLD:?PX4_GZ_WORLD missing in $SIM_ENV}"
VEHICLE="${PX4_VEHICLE:?PX4_VEHICLE missing in $SIM_ENV}"

MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
trap 'kill $AGENT_PID 2>/dev/null' EXIT

cd "$PX4_DIR"
PX4_GZ_WORLD="$WORLD" make px4_sitl "$VEHICLE"