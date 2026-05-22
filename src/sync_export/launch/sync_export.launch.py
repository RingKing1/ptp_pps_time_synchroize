from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir',
            default_value='export_sync',
            description='Sample 根目录（导出文件夹）'
        ),
        DeclareLaunchArgument(
            'cameras_config',
            description='cameras.yaml 路径（topic↔规范名映射真源）'
        ),
        DeclareLaunchArgument(
            'calib_dir',
            description='calib/camera/*.json 所在目录（由 convert_calibration.py 生成）'
        ),
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/rslidar_points',
            description='Lidar 点云 topic'
        ),
        DeclareLaunchArgument(
            'kinematic_state_topic',
            default_value='/localization/kinematicstate',
            description='定位状态 topic'
        ),
        DeclareLaunchArgument(
            'inspva_topic',
            default_value='/beidou/inspva',
            description='北斗 INS Inspva topic（提供 WGS84 经纬高）'
        ),
        DeclareLaunchArgument(
            'queue_size',
            default_value='20',
            description='message_filters 同步缓存深度'
        ),
        DeclareLaunchArgument(
            'max_sync_frames',
            default_value='0',
            description='最大同步导出帧数，0 表示无限制'
        ),
        Node(
            package="sync_export",
            executable="sync_export_node",
            name="sync_export_node",
            output="screen",
            parameters=[{
                'output_dir': LaunchConfiguration('output_dir'),
                'cameras_config': LaunchConfiguration('cameras_config'),
                'calib_dir': LaunchConfiguration('calib_dir'),
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'kinematic_state_topic': LaunchConfiguration('kinematic_state_topic'),
                'inspva_topic': LaunchConfiguration('inspva_topic'),
                'queue_size': LaunchConfiguration('queue_size'),
                'max_sync_frames': LaunchConfiguration('max_sync_frames'),
            }]
        )
    ])
