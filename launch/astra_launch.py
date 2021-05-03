from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="astra_camera",
            executable="astra_camera_node",
            name="astra_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"use_ir": True,
                 "device_id": "#0"}
            ]
        )
    ])