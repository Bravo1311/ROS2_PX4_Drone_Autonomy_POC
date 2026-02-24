from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_image_bridge',
        output='screen',
        arguments=[
            "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image"
            "@sensor_msgs/msg/Image@gz.msgs.Image",

            "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info"
            "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
    )

    aruco = Node(
        package='px4_aruco_landing',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'image_topic': '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image',
            'camera_info_topic': '/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info',
            'marker_length_m': 0.5,
        }],
    )

    autoland = Node(
        package='px4_aruco_landing',
        executable='autoland_twist_publisher',
        name='autoland_twist_publisher',
        output='screen',
    )

    return LaunchDescription([
        bridge,
        aruco,
        # autoland,
    ])