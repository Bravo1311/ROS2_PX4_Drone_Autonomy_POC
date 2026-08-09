# ROS2-PX4-X500 Autonomy Proof of Concept

## Vision-Based Autonomous UAV Landing System  

[*(ROS2 + PX4 Offboard + Flow Matching-based Precision Landing)*](https://youtu.be/w6VsROLk_S0) 


[*(ROS2 + PX4 Offboard + Nav2-based autonomous GNC)*](https://www.youtube.com/watch?v=miEoYj2KqZ8) 


[*(ROS2 + PX4 Offboard + Reactive Obstacle Avoidance using a 2D Lidar)*](https://youtu.be/1jfcPgGP5Kg) 


[*(ROS2 + PX4 Offboard + ArUco Pose Estimation + Drone Reorientation + PD Control)*](https://youtu.be/zwz-KPHohZU) 


[*(ROS2 + PX4 Offboard + ArUco Pose Estimation + PD Control)*](https://youtu.be/I1ZB6Qup60M)


[*(ROS2 + PX4 Offboard + ArUco Pose Estimation + P Control)*](https://youtu.be/HkAIYsaiJwk) 


## Overview

This repository implements a full autonomous UAV stack for the PX4 **x500-mono-cam-down** quadrotor in simulation, built on ROS2 Jazzy + PX4 SITL + Gazebo Harmonic.
 
The system provides end-to-end autonomy from sensor fusion through mission planning:
 
- SLAM-based localisation (SLAM Toolbox) with a custom Gazebo ground-truth odometry bridge
- Full 2D autonomous navigation via Nav2 (adapted for holonomic drone dynamics)
- Real-time ArUco marker detection
- Diffusion Transformer-based Flow-matching precision landing policy based on a PD controller, takes marker pose from the ArUco detector
- Vision-guided precision landing using a PD velocity controller
- Reactive obstacle avoidance via a 2D LiDAR safety filter
- Command multiplexing between teleoperation, autonomous navigation, and landing modes
- Annotated semantic map generation (ArUco landmark positions embedded in SLAM map) - in progress
- Holonomic drone adaptation including ENU/NED frame conversion and velocity heading alignment

The result is a modular autonomy stack that bridges perception and control in a PX4-based UAV system. \
 - The system was evaluated from initial altitudes up to ~ 14m to evaluate convergence stability and oscillatory behavior. 
 - Stable lateral convergence and controlled descent were observed across varying initial altitudes and marker poses in simulation.

---

## System Architecture

```
─────────────────────────────────────────────────────┐
│                   VLM Layer (WIP)                   │
│         Natural language mission planning           │
│              Qwen 2.5 VL 3b (local)                │
└──────────────────────┬──────────────────────────────┘
                       │ Nav2 goals + actions
┌──────────────────────▼──────────────────────────────┐
│              Autonomous GNC Stack                   │
│                                                     │
│  Guidance          Navigation        Control        │
│  ─────────         ──────────        ───────        │
│  bt_navigator      SLAM Toolbox      DWB planner    │
│  planner_server    gz_odom_tf        vel_smoother   │
│  smoother_server   lidar_static_tf   safety_filter  │
│                    scan_frame_fix    offboard_mux    │
└──────────────────────┬──────────────────────────────┘
                       │ TrajectorySetpoint (NED)
┌──────────────────────▼──────────────────────────────┐
│                  PX4 SITL                           │
│         EKF2 · Attitude · Rate · Motor              │
└──────────────────────┬──────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────┐
│              Gazebo Harmonic                        │
│   x500 · 2D LiDAR · Downward camera · OAK-D        │
└─────────────────────────────────────────────────────┘
```

1. **Perception Layer**
   - ArUco marker detection
   - 6-DoF pose estimation in camera optical frame

2. **Mid-Level Control**
   - Vision-based PD velocity controller
   - Lateral error damping with derivative filtering
   - Descent gating based on horizontal convergence

3. **Low-Level Control**
   - PX4 internal cascaded position/velocity controllers (Offboard mode)

4. **Command Arbitration**
   - ROS2-based MUX node enabling seamless switching between:
     - Manual teleoperation
     - Autonomous landing mode

---

## Key Technical Contributions
 
### 1. Custom Gazebo Odometry Bridge (`gz_odom_tf`)
 
The system subscribes directly to Gazebo's `/world/walls/dynamic_pose/info` topic using `gz.transport13` Python bindings — filtering for the drone by model name. This gives ground-truth ENU pose with zero sensor noise, correct timestamps for SLAM TF lookups, and no dependency on PX4 EKF2 yaw estimation (which introduces 1–5° magnetic declination offset in simulation).
 
The node also publishes a `nav_msgs/Odometry` topic on `/odom` so Nav2's `bt_navigator` and `velocity_smoother` have the odometry data they require, in addition to the TF broadcast.

### 2. Holonomic Drone Adaptation of Nav2
 
Nav2 was designed for differential drive ground robots. Adapting it for a holonomic drone required:
 
- Enabling `StandardTrajectoryGenerator` in DWB to unlock y-axis velocity sampling
- Zeroing yawspeed in the Nav2 mux path — holonomic drones reach any goal without rotating
- ENU→NED frame conversion in `px4_offboard_mux`: DWB outputs ENU world frame, PX4 `TrajectorySetpoint.velocity` expects NED world frame. The conversion swaps x/y (`x_ned = y_enu, y_ned = x_enu`) and negates z
- A joystick convention correction (`vy_nav = -vy_b`) to account for the sign flip in the teleop axis mapping that the mux rotation was built around
- Velocity heading alignment: a yaw P-controller continuously rotates the drone to face its direction of motion, computed as `desired_yaw = atan2(wy, wx)` in NED

### 3. Safety Velocity Filter
 
A dedicated node intercepts all velocity commands and monitors `/scan_fixed` for obstacle proximity. If the LiDAR detects an obstacle within `stop_dist + stop_gain * speed` metres in the commanded heading, x/y velocity is zeroed. The filter times out scan data after 1 second and stops all lateral motion if the LiDAR feed goes stale. Z velocity (altitude) always passes through unconditionally.
 
---

## Sensor Suite
 
| Sensor | Purpose |
|--------|---------|
| 2D LiDAR (lidar_2d_v2) | SLAM mapping, Nav2 costmap obstacle layer, safety filter |
| Downward monocular camera | ArUco marker detection, precision landing |
| OAK-D (depth + RGB) | 3D obstacle avoidance (planned), VLM camera feed |
| IMU + barometer + magnetometer | PX4 EKF2 state estimation |
 
---
 
## Data Flow — ArUco Landing Pipeline
 
```
Gazebo downward camera
        ↓
aruco_detector.py
  - detects all markers in frame
  - publishes best marker as /marker_pose (camera_optical_frame)
  - publishes all detections as /aruco_detections_json
        ↓
autoland_twist_publisher.py
  - frame conversion: optical → body (body_fwd = -optical_y)
  - PID controller on lateral error (with derivative filtering)
  - descent gating: only descend when error < center_radius
  - yaw alignment above lock height, yaw lock below
  - publishes /autoland_velocity_cmd (body frame Twist)

  OR

flow_landing_policy/policy_node.py
  - Ingests marker_pose from aruco_detector, pose passed on as input to the policy
  - policy trained through imitation learning. More details can be foundin the repo: [Flow Matching Landing](https://github.com/Bravo1311/Flow_Matching_Landing)
          ↓
px4_offboard_mux.py (AUTO mode)
  - rotates body frame by NED yaw → NED world frame
  - sends to PX4 TrajectorySetpoint
```

---
 
## Data Flow — Nav2 Navigation Pipeline
 
```
RViz 2D Goal Pose
        ↓
bt_navigator
  - orchestrates planner + controller
  - monitors progress via /odom
        ↓
planner_server (NavFn A*)
  - plans global path on costmap (static + obstacle + inflation layers)
        ↓
controller_server (DWB, holonomic mode)
  - samples (vx, vy, theta) trajectories
  - scores with PathDist, GoalDist, BaseObstacle critics
  - outputs body-frame Twist to /cmd_vel_nav_raw
        ↓
velocity_smoother
  - limits acceleration, smooths commands
  - publishes /cmd_vel_nav
        ↓
nav2_cmd_relay
  - forces z = 0 (Nav2 is 2D, altitude held constant)
  - watchdog timeout on stale commands
  - publishes /nav_velocity_cmd
        ↓
px4_offboard_mux.py (NAV2 mode)
  - ENU → NED conversion (swap x/y, negate vy convention)
  - yaw P-controller: face direction of motion
  - sends to PX4 TrajectorySetpoint
```
 
---

## Launch Sequence
 
```bash
# Terminal 1 — Gazebo + PX4 SITL
ros2 launch px4_bringup minimal.launch.py
 
# Terminal 2 — Perception + SLAM
# Mapping:
ros2 launch px4_bringup slam.launch.py mode:=mapping
# Localization (after map is saved):
ros2 launch px4_bringup slam.launch.py mode:=localization
 
# Terminal 3 — Nav2 autonomous navigation
ros2 launch px4_bringup nav2.launch.py
 
# Terminal 4 — Offboard control + joystick
ros2 launch px4_offboard auto_joystick_teleop.launch.py

# Terminal 5 — Precision Landing Velocity Publisher
ros2 run px4_aruco_landing autoland_twist_publisher.py

# Terminal 6 — Precision Landing Velocity Publisher
ros2 run flow_landing_policy policy_landing.launch.py

```

### Saving a Map
 
```bash
# Save SLAM Toolbox posegraph (required for localization)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap  \
 "{name: {data: \"/{link_to_workspace}/maps2/walls_map\"}}"
```

### Enabling Nav2 Control
 
```bash
# Enable Nav2 channel on the mux (or use gamepad SELECT button)
ros2 topic pub --once /enable_nav_cmd std_msgs/msg/Bool "data: true"
```
 
---
 
## Gamepad Controls (EVOFOX)
 
| Button | Action |
|--------|--------|
| A | Arm drone |
| Y | Disarm / emergency stop |
| B | Toggle autonomous landing |
| SELECT | Toggle Nav2 autonomous navigation |
| X | Toggle auto-hover |
| Left stick UD | Throttle / altitude |
| Left stick LR | Yaw |
| Right stick UD | Forward / backward |
| Right stick LR | Strafe left / right |
 
---

## Package Structure
 
```
px4_bringup/
  launch/
    minimal.launch.py       — PX4 SITL + Gazebo + RViz
    perception.launch.py    — bridges + gz_odom_tf + scan + aruco
    slam.launch.py          — perception + SLAM Toolbox (mapping/localization)
    nav2.launch.py          — full Nav2 stack
  config/
    nav2_params.yaml        — Nav2 configuration (holonomic DWB)
 
px4_lidar/
  px4_lidar/
    gz_odom_tf.py           — Gazebo pose → odom TF + /odom topic
    scan_frame_fix.py       — rewrites LiDAR scan frame_id
    safety_vel_filter.py    — LiDAR-based velocity gating
    startup_spin.py         — AMCL convergence helper
 
px4_aruco_landing/
  px4_aruco_landing/
    aruco_detector.py       — ArUco detection, pose estimation, JSON output
    marker_kf.py            — Kalman filter for marker pose smoothing
    autoland_twist_publisher.py — PID landing controller
    aruco_map_annotator.py  — semantic map annotation with marker positions
 
px4_offboard/
  px4_offboard/
    px4_offboard_mux.py     — command arbitration + frame conversion
    teleop_joystick.py      — gamepad → velocity commands
    nav2_cmd_relay.py       — Nav2 cmd_vel relay with watchdog
```
 
---

## Ongoing Work
 
- Semantic map annotation pipeline for VLM visual grounding
- VLM mission planning layer (Qwen 2.5 VL 3b, local inference via Ollama)
- Multi-drone swarm coordination controlled by VLM high-level commands
- OAK-D depth camera integration for 3D obstacle avoidance and altitude control


## Key Extensions Beyond Original Repository

This project builds upon:

- Original Repository:  
  https://github.com/MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick  
- Author: Curious-Utkarsh  

### Design Decisions

- Gazebo odometry over PX4 EKF2 for simulation** — PX4 EKF2 introduces 1–5° yaw offset from simulated magnetic declination. Gazebo ground truth gives perfect frame alignment for SLAM and Nav2.
- PD control selected to prioritize fast lateral convergence and damping while avoiding integral windup during dynamic descent.
- Integral action intentionally omitted due to limited external disturbances in simulation and risk of overshoot during vertical convergence.
- Velocity control was chosen over position control to allow smoother interaction with PX4's internal cascaded controllers in Offboard mode.
- Derivative filtering was introduced to mitigate noise amplification arising from pose-estimation jitter at higher altitudes.

### Added Components

- ArUco-based pose estimation ROS2 node
- Vision-guided autonomous landing pipeline
- PD-based lateral stabilization controller
- Offboard command multiplexing logic
- Robust QoS configuration for real-time ROS2 communication

---

## Setup

Please refer to the original repository for base setup instructions.  
This repository assumes a working PX4 SITL + ROS2 integration.
