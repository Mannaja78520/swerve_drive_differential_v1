# Imports ที่จำเป็นทั้งหมด
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # กำหนด path ของไฟล์ config EKF
    diff_swerve_dir = get_package_share_directory('differential_drive_v1')
    lidar_dir = get_package_share_directory('sllidar_ros2')
    ekf_config_path = PathJoinSubstitution([FindPackageShare('differential_drive_v1'), 'config', 'ekf.yaml'])
    
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diff_swerve_dir, 'launch', 'navigate_launch.py')
        ),
    )
    
    esp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diff_swerve_dir, 'launch', 'esp_microros_launch.py')
        ),
    )
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'view_sllidar_c1_launch.py')
        ),
    )

    return LaunchDescription([
        
        # Node(
        #     package='differential_drive_v1',
        #     executable='imu_axis_corrector.py',
        #     name='imu_corrector',
        #     output='screen'
        # ),

        Node(
            package='differential_drive_v1',
            executable='merge_odom.py',
            name='odom',
            output='screen'
        ),

        # Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0.0', '0', '0.35', '3.141592653589793', '0', '0', 'base_link', 'laser']
        ),

        # Static TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_pub_laser',
            arguments=['0.0', '0', '0.0', '−1.58824962', '0', '0', 'base_link', 'imu_link']
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('/odometry/filtered', '/odom')]
        ),
        # esp_launch,
        lidar_launch,
        # nav_launch,
        # Node(
        #     package='differential_drive_v1',
        #     executable='robot_lidar.py',
        #     name='robot_lidar',
        #     output='screen'
        # ),
    ])