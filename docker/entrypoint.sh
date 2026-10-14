#!/usr/bin/env bash
# Source ROS and the workspace overlay, then run whatever command was given.
source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
exec "$@"