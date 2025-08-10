from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('differential_drive_v1'), 'config', 'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file_path]
        ),
        Node(
            package='differential_drive_v1',
            executable='merge_odom.py',
            name='merge_odom_node',
            output='screen',
            parameters=[{'frame_id': 'odom_raw', 'child_frame_id': 'base_link'}]
        )
    ])
