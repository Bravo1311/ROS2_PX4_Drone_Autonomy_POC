# ROS2-PX4-X500 Autonomoy Proof of Concept

This project builds on the work of:

- Original Repository: **[Link to the Repo](https://github.com/MechaMind-Labs/ROS2-PX4_Drone_Teleoperation_Using_Joystick?tab=readme-ov-file)**
- Author: **[Curious-Utkarsh](https://github.com/Curious-Utkarsh)**

## Overview

This repository build upon the original teleoperation framwork and extends it toward autonomous landing capabilities using vision-based perception and control.

## Modifications and Extensions:

- Integration of an ArUco marker detection ROS2 package for pose estimation
- Vision-based autonomous landing pipeline for the **x500-mono-cam-down** drone.
- Custom P-Controller package for closed-loop landing control.
- Reinforcement Learning based offboard-velocity controller (work in progress).
- Integrating teleoperation and autonomy in the offboard control logic (work in progress)

The original core implementation remains credited to the original author.

## Setup

For setup, please refer to the original repository.