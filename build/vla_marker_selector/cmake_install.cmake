# Install script for directory: /home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/vla_marker_selector

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/install/vla_marker_selector")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/vla_marker_selector")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/vla_marker_selector")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector/environment" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector/environment" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_index/share/ament_index/resource_index/packages/vla_marker_selector")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector/cmake" TYPE FILE FILES
    "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_core/vla_marker_selectorConfig.cmake"
    "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/ament_cmake_core/vla_marker_selectorConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vla_marker_selector" TYPE FILE FILES "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/vla_marker_selector/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/build/vla_marker_selector/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
