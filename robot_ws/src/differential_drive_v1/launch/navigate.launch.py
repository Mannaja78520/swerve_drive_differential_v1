from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_dir = get_package_share_directory('differential_drive_v1')

    map_yaml_path = os.path.join(pkg_dir, 'map', 'wrg2024_map.yaml')
    param_file = os.path.join(pkg_dir, 'param', 'swerve.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': param_file,
        }.items(),
    )
    
    navigate_node = Node(
        package='differential_drive_v1',
        executable='navigate.py',
        name='navigate_node',
        output='screen',
    )

    # ---- nodes อื่น ๆ ของคุณ (tf, ekf, lidar) ใส่ตามเดิม ----

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_collision_monitor', default_value='False'),  # <<<<
        DeclareLaunchArgument('use_docking', default_value='False'),            # <<<<
        nav2,
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen',
        # ),
        # navigate_node,
    ])

