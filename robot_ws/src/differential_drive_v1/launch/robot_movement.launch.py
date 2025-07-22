import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    ld = LaunchDescription()
    
    # Path to config file for keyboard control node
    speed_config = os.path.join(
        get_package_share_directory('differential_drive_v1'),
        'config',
        'speed_control.yaml'
    )

    robot_movement_node = Node(
        package='differential_drive_v1',
        executable='robot_movement.py',
        name='robot_movement_Node',
        parameters=[speed_config],
        # output='screen'
    )
    
    ld.add_action(robot_movement_node)

    return ld
