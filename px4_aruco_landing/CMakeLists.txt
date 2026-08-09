cmake_minimum_required(VERSION 3.8)
project(px4_aruco_landing)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# ── C++ executable ────────────────────────────────────────
add_executable(aruco_map_annotator_cpp
  src/aruco_map_annotator.cpp
)

ament_target_dependencies(aruco_map_annotator_cpp
  rclcpp
  nav_msgs
  std_msgs
  std_srvs
  cv_bridge
)

target_link_libraries(aruco_map_annotator_cpp
  ${OpenCV_LIBS}
  yaml-cpp
  nlohmann_json::nlohmann_json
)

install(TARGETS aruco_map_annotator_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python nodes ──────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  px4_aruco_landing/aruco_detector.py
  px4_aruco_landing/autoland_twist_publisher.py
  px4_aruco_landing/marker_kf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()