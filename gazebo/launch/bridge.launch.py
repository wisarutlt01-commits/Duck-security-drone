"""
sim/launch/bridge.launch.py — LEGACY: ROS2 ↔ Gazebo Harmonic bridge

No longer needed. GazeboVisionSystem now subscribes to gz-transport
camera topics directly via python3-gz-transport13, with no ROS2 bridge.
Kept here for reference only.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros2_bridge",
        output="screen",
        parameters=[{
            "qos_overrides./tracker_drone/camera/image_raw.publisher.reliability": "best_effort",
        }],
        arguments=[
            # gz-transport → ROS2: camera image
            "/tracker_drone/camera/image_raw"
            "@sensor_msgs/msg/Image[gz.msgs.Image",
            # gz-transport → ROS2: camera info
            "/tracker_drone/camera/camera_info"
            "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # gz-transport → ROS2: simulation clock
            "/world/drone_tracking/clock"
            "@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        remappings=[
            ("/world/drone_tracking/clock", "/clock"),
        ],
    )
    return LaunchDescription([bridge_node])
