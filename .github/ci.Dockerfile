# Build-only CI image: enough to `colcon build` every package in this workspace.
# No PX4 SITL / Gazebo here - this image only compiles/installs, never runs simulation.

FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
            build-essential \
            cmake \
            git \
            python3-colcon-common-extensions \
            python3-pip \
            python3-rosdep \
        && rm -rf /var/lib/apt/lists/* 

# torch, numpy, opencv-python have no rosdep key -> install them directly
# --break-system-packages: Ubuntu 24.04 blocks bare 'pip install (PEP 668) unless told otherwise'
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir --break-system-packages -r /tmp/requirements.txt

WORKDIR /workspace
