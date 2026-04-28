from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir',
            default_value='export_sync',
            description='Output directory for exported synchronized data'
        ),
        Node(
            package="sync_export",
            executable="sync_export_node",
            name="sync_export_node",
            output="screen",
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
                'queue_size': 20
            }]
        )
    ])
