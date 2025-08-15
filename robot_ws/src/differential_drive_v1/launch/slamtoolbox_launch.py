from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ประกาศตัวแปรสำหรับ launch
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # กำหนดค่าเริ่มต้น
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory('differential_drive_v1'),
                                   'config', 'slam_toolbox_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # สร้าง SLAM Toolbox node
    start_sync_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',  # ใช้ sync แทน async เพื่อความเสถียร
        name='slam_toolbox',
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug'],  # เพิ่ม debugging
        remappings=[
            # ถ้าจำเป็น ให้เพิ่ม remappings ที่นี่
            # ('scan', '/your_actual_scan_topic'),
        ]
    )
    
    # เพิ่ม node สำหรับบันทึกแผนที่ (อยากใช้เมื่อไหร่ค่อยปลดคอมเมนต์)
    # save_map_node = Node(
    #    package='slam_toolbox',
    #    executable='lifelong_slam_toolbox_node',
    #    name='map_saver',
    #    output='screen',
    #    parameters=[{
    #        'autostart': 'true',
    #        'autosave_interval': '300.0',  # บันทึกแผนที่ทุก 5 นาที
    #        'map_start_at_dock': 'true',
    #        'map_file_name': 'my_map'
    #    }]
    # )

    # ส่งคืน LaunchDescription ที่มี nodes และ launch arguments
    ld = LaunchDescription()
    
    # เพิ่ม launch args
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_params_file_cmd)
    
    # เพิ่ม environment variables
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    ld.add_action(SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'))
    
    # เพิ่ม node
    ld.add_action(start_sync_slam_toolbox_node)
    # ld.add_action(save_map_node)  # ถ้าต้องการใช้งาน map saver
    
    return ld