# Sourced from /root/.bashrc in the container (interactive shells only).
# Type `rhelp` to list the shortcuts.

# ---- ROS environment -------------------------------------------------------
source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

# ---- Workspace navigation / building ---------------------------------------
alias ws='cd /ws'
alias repo='cd /ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick'
alias sws='source /ws/install/setup.bash'

# cb            build everything          cb --cmake-args -DCMAKE_BUILD_TYPE=Release   (extra args pass through)
# cbp PKG...    build only these packages
# cbu PKG...    build these packages and everything they depend on
cb()  { ( cd /ws && colcon build --symlink-install "$@" ) && source /ws/install/setup.bash; }
cbp() { ( cd /ws && colcon build --symlink-install --packages-select "$@" ) && source /ws/install/setup.bash; }
cbu() { ( cd /ws && colcon build --symlink-install --packages-up-to "$@" ) && source /ws/install/setup.bash; }
# cclean PKG... delete build/install output of these packages (forces a clean rebuild)
cclean() { for p in "$@"; do rm -rf "/ws/build/$p" "/ws/install/$p"; done; }
alias ct='cd /ws && colcon test && colcon test-result --verbose'

# ---- ROS 2 introspection ---------------------------------------------------
alias tl='ros2 topic list'
alias te='ros2 topic echo'
alias th='ros2 topic hz'
alias ti='ros2 topic info'
alias nl='ros2 node list'
alias ni='ros2 node info'
alias pl='ros2 pkg list'
alias rl='ros2 launch'
alias rr='ros2 run'
alias fmu='ros2 topic list | grep fmu'

# ---- Project shortcuts -----------------------------------------------------
alias sim='ros2 launch px4_bringup minimal.launch.py'                 # PX4 SITL + XRCE agent + RViz
alias slam='ros2 launch px4_bringup slam.launch.py mode:=mapping'     # clock/sensor bridges + SLAM

rhelp() {
  cat <<'EOF'
Build:    cb [args]   cbp PKG..   cbu PKG..   cclean PKG..   ct (tests)   sws (re-source)
Navigate: ws (/ws)    repo (this repo)
ROS 2:    tl te th ti (topic list/echo/hz/info)   nl ni (node list/info)   pl rl rr (pkg list, launch, run)
Project:  sim (minimal.launch.py)   slam (slam.launch.py mode:=mapping)   fmu (list /fmu topics)
EOF
}
