# ROS2-PX4-X500 Autonomoy Proof of Concept

## Vision-Based Autonomous UAV Landing System  
*(ROS2 + PX4 Offboard + ArUco Pose Estimation + PD Control)*
https://github.com/user-attachments/assets/5d5315e8-6e73-4576-a0da-d5c7f383487e

## Overview

This repository extends the original PX4 joystick teleoperation framework into a vision-guided autonomous landing system for the PX4 **x500-mono-cam-down** quadrotor in simulation.

The system integrates:

- Real-time ArUco-based pose estimation  
- Frame-consistent coordinate transformations (camera → body → world)  
- ROS2 Offboard velocity control  
- A PD-based closed-loop landing controller  
- Command multiplexing between manual teleoperation and autonomous control  

The result is a modular autonomy stack that bridges perception and control in a PX4-based UAV system.

---

## System Architecture

Perception → Control → PX4

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

## Key Extensions Beyond Original Repository

This project builds upon:

- Original Repository:  
  https://github.com/MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick  
- Author: Curious-Utkarsh  

### Added Components

- ArUco-based pose estimation ROS2 node
- Vision-guided autonomous landing pipeline
- PD-based lateral stabilization controller
- Offboard command multiplexing logic
- Robust QoS configuration for real-time ROS2 communication

---

## Ongoing Work

- Reinforcement Learning-based velocity controller
- Moving platform landing experiments
- Performance evaluation (convergence time, landing accuracy, robustness)

---

## Setup

Please refer to the original repository for base setup instructions.  
This repository assumes a working PX4 SITL + ROS2 integration.
