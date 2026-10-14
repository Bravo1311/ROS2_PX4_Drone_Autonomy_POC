ARG ROS_DISTRO=jazzy
ARG BASE_IMAGE=osrf/ros:${ROS_DISTRO}-desktop

# One compile job per core can OOM-kill the build (PX4 and XRCE compile heavily).
# Raise on a bigger machine: --build-arg BUILD_JOBS=16
ARG BUILD_JOBS=8

# Pinned refs: full 40-char commit SHAs (git fetch cannot resolve abbreviated ones).
# PX4_REF and PX4_MSGS_REF MUST match: PX4 v1.17.0-alpha1-607-g32c94bd3b1 (2026-02-13) and
# the px4_msgs commit from 2026-02-09 that you build against today.
ARG PX4_REF=32c94bd3b1bd16cc4897f61fb0135fb75bdb3c6e
ARG PX4_MSGS_REF=98517583c52b9c9b811606f75b28f31dfd7d67db
ARG PX4_ROS_COM_REF=86e9aeb20e55a4673fa8a9f1c29ea06a6c5ad1af
# v2.4.2 no longer builds: its superbuild clones Fast-DDS branch 2.12.x, which was deleted upstream.
# v3.0.1 is the version used on the host.
ARG XRCE_REF=v3.0.1


# ============================================================================
# Stage 1: PX4-Autopilot SITL (built once, cached)
# ============================================================================
FROM ${BASE_IMAGE} AS px4

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO
ARG BUILD_JOBS
ARG PX4_REF
ENV DEBIAN_FRONTEND=noninteractive

# ros-gz brings the vendored Gazebo Harmonic that PX4's gz plugin builds against
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        lsb-release \
        ros-${ROS_DISTRO}-ros-gz \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Shallow fetch of exactly PX4_REF (much faster than a full recursive clone)
RUN git init -q /opt/PX4-Autopilot && \
    cd /opt/PX4-Autopilot && \
    git remote add origin https://github.com/PX4/PX4-Autopilot.git && \
    git fetch --depth 1 origin ${PX4_REF} && \
    git checkout -q FETCH_HEAD && \
    git submodule update --init --recursive --depth 1

WORKDIR /opt/PX4-Autopilot
RUN bash Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx

# Custom Gazebo models/worlds (walls, aruco, x500_mono_cam_down, ...)
COPY px4_custom/ /tmp/px4_custom/
RUN cp -r /tmp/px4_custom/models/. Tools/simulation/gz/models/ && \
    cp -r /tmp/px4_custom/worlds/. Tools/simulation/gz/worlds/

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    MAKEFLAGS="-j${BUILD_JOBS}" make px4_sitl_default


# ============================================================================
# Stage 2: Micro XRCE-DDS Agent
# ============================================================================
FROM ${BASE_IMAGE} AS xrce
ARG BUILD_JOBS
ARG XRCE_REF
RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake \
        g++ \
        git \
    && rm -rf /var/lib/apt/lists/*

RUN git init -q /tmp/agent && \
    cd /tmp/agent && \
    git remote add origin https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    git fetch --depth 1 origin ${XRCE_REF} && \
    git checkout -q FETCH_HEAD && \
    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build --parallel "${BUILD_JOBS}" && \
    cmake --install build


# ============================================================================
# Stage 3: final image
# ============================================================================
FROM ${BASE_IMAGE}
ARG ROS_DISTRO
ARG PX4_MSGS_REF
ARG PX4_ROS_COM_REF
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# python3-gz-transport13 (used by px4_lidar/gz_odom_tf.py) is only published in the OSRF Gazebo
# apt repo, which the base image does not have. Same repo your host uses.
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $VERSION_CODENAME) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        libyaml-cpp-dev \
        nlohmann-json3-dev \
        python3-colcon-common-extensions \
        python3-gz-transport13 \
        python3-pip \
        python3-rosdep \
        python3-venv \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-eigen3-cmake-module \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-joy-teleop \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-ros-gz-image \
        ros-${ROS_DISTRO}-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Python deps from requirements.txt in a venv. It is copied alone first so this layer
# is cached until requirements.txt changes. --system-site-packages keeps apt's rclpy visible.
COPY requirements.txt /tmp/requirements.txt
RUN python3 -m venv --system-site-packages /opt/venv && \
    /opt/venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt
ENV PATH=/opt/venv/bin:$PATH
# Fallback if px4_msgs fails to generate with an empy error:
# RUN /opt/venv/bin/pip uninstall -y em empy && /opt/venv/bin/pip install empy==3.3.4

COPY --from=px4  /opt/PX4-Autopilot /opt/PX4-Autopilot
COPY --from=xrce /usr/local /usr/local
RUN ldconfig

# Workspace: external packages fetched by pinned ref, this repo copied in
WORKDIR /ws/src
RUN git init -q px4_msgs && cd px4_msgs && \
    git remote add origin https://github.com/PX4/px4_msgs.git && \
    git fetch --depth 1 origin ${PX4_MSGS_REF} && git checkout -q FETCH_HEAD
RUN git init -q px4_ros_com && cd px4_ros_com && \
    git remote add origin https://github.com/PX4/px4_ros_com.git && \
    git fetch --depth 1 origin ${PX4_ROS_COM_REF} && git checkout -q FETCH_HEAD
# optional: drone_agent lives in the separate LLM-controlled-drone repo
COPY . /ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick

WORKDIR /ws
# Nav2 etc. are not declared in package.xml, so failures here are tolerated
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y || true
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install

# Keeps the old /home/bravo1311/px4_ros2_ws/... paths in your commands working
RUN mkdir -p /home/bravo1311 && ln -s /ws /home/bravo1311/px4_ros2_ws

# PX4's `make px4_sitl <target>` calls /usr/bin/ninja at runtime; it was only installed in the
# px4 build stage. Kept as a separate late layer so the torch/colcon layers stay cached.
RUN apt-get update && apt-get install -y --no-install-recommends ninja-build \
    && rm -rf /var/lib/apt/lists/*

ENV PX4_DIR=/opt/PX4-Autopilot \
    NVIDIA_DRIVER_CAPABILITIES=all \
    NVIDIA_VISIBLE_DEVICES=all
# Interactive shells (docker exec ... bash, VS Code terminals) load ROS and the shortcuts from
# docker/ros_aliases.sh via root's ~/.bashrc, so no /entrypoint.sh prefix is needed for them.
COPY docker/ros_aliases.sh /etc/ros_aliases.sh
RUN echo 'source /etc/ros_aliases.sh' >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]