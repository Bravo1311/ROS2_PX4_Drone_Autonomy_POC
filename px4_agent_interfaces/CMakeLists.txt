cmake_minimum_required(VERSION 3.8)
project(px4_agent_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlState.msg"
  "action/ExecutePlan.action"
)

ament_package()
