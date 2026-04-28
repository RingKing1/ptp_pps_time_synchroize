from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sync_export",
            executable="sync_export_node",
            name="sync_export_node",
            output="screen",
            parameters=["src/sync_export/config/sync_export.yaml"]
        )
    ])
