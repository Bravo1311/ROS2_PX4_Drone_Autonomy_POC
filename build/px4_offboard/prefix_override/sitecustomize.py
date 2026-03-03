import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/install/px4_offboard'
