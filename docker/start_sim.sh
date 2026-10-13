#!/usr/bin/env bash
# Starts the Micro XRCE-DDS Agent in the background and PX4 SITL + Gazebo in the foreground.
# Standalone alternative to `ros2 launch px4_bringup minimal.launch.py`, which already starts
# PX4 SITL, the XRCE agent and RViz. Do NOT run both: they fight over UDP port 8888.
set -e

WORLD="${PX4_GZ_WORLD:-walls}"
VEHICLE="${PX4_VEHICLE:-gz_x500_mono_cam_down}"

MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
trap 'kill $AGENT_PID 2>/dev/null' EXIT

cd "$PX4_DIR"
PX4_GZ_WORLD="$WORLD" make px4_sitl "$VEHICLE"