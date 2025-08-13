import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_dir = get_package_share_directory('differential_drive_v1')

    speed_config = os.path.join(
        package_dir,
        'config',
        'speed_control.yaml'
    )

    # 🔁 Include robot_movement.launch.py
    movement_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'robot_movement.launch.py')
        )
    )

    # ✅ รัน keyboard control ผ่าน xterm (เพราะใช้ termios)
    command = f"ros2 run differential_drive_v1 keyboard_control.py --ros-args --params-file {speed_config}"
    keyboard_node = ExecuteProcess(
        cmd=['xterm', '-e', command],
        shell=True
    )

    return LaunchDescription([
        movement_launch,
        keyboard_node
    ])
