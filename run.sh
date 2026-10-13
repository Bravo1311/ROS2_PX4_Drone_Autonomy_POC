#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# Let the container open windows on your screen
xhost +local:docker >/dev/null

docker compose up --build -d

echo
echo "Container is up. Open a shell in it:"
echo "  docker exec -it px4-ros2-autonomy /entrypoint.sh bash"
echo
echo "Then start PX4 SITL + Gazebo + XRCE agent + RViz with one command:"
echo "  ros2 launch px4_bringup minimal.launch.py"
echo
echo "(docker/start_sim.sh starts PX4 + the agent without ROS; do not run it at the"
echo " same time as minimal.launch.py, both use UDP port 8888.)"