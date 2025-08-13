from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # กำหนด path แบบ absolute ไปที่ไฟล์ map.yaml จริง
    map_yaml_path = os.path.expanduser('~/swerve_drive_differential_v1/robot_ws/src/differential_drive_v1/map/wrg_map.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_path}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'base_frame_id': 'base_link',    # <-- เพิ่มตรงนี้
                'odom_frame_id': 'odom'          # <-- ปกติก็ odom อยู่แล้ว
            }]
        ),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
        ),
    ])
