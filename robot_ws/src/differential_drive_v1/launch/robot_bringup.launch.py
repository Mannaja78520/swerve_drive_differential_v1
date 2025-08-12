# Imports ที่จำเป็นทั้งหมด
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # กำหนด path ของไฟล์ config EKF
    ekf_config_path = PathJoinSubstitution([FindPackageShare('differential_drive_v1'), 'config', 'ekf.yaml'])
    

    return LaunchDescription([

        Node(
            package='differential_drive_v1',
            executable='merge_odom.py',
            name='odom',
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['0.0', '0', '0.35','0', '0', '0','base_link','laser']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_pub_laser',
            arguments=['0.0', '0', '0.0','0.0', '0', '0','base_link','imu_link']
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[('/odometry/filtered', '/odom')]
        ),
        # Launch file สำหรับ RealSense
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(realsense_launch_path),
        #     launch_arguments={
        #         'enable_color': 'true',
        #         'enable_depth': 'true',
        #         'enable_gyro': 'true',
        #         'enable_accel': 'true',
        #         'rgb_camera.profile': '640x480x30',
        #         'depth_module.profile': '640x480x30',
        #         'unite_imu_method': '1',
        #         'gyro_fps': '200',
        #         'accel_fps': '250',
        #         'pointcloud.enable': 'true',
        #         'filters': 'pointcloud'
        #     }.items()
        # ),
        # Static transform สำหรับ camera ที่อยู่ด้านหน้า 0.2 เมตร แหงนขึ้น 15 องศา
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_tf_pub',
        #     arguments=['0.2', '0.0', '0.1', '0.0', '0.262', '0.0', 'base_link', 'camera_link']
        # ),
        Node(
            package='differential_drive_v1',
            executable='robot_lidar.py',
            name='robot_lidar',
            output='screen'
        ),
        # Node(
        #     package='differential_drive_v1',
        #     executable='manager',
        #     name='manager',
        #     output='screen'
        # ),
        # Node(
        #     package='differential_drive_v1',
        #     executable='battery_socket',
        #     name='battery_socket',
        #     output='screen'
        # )
    ])