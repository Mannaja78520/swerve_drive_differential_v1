from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_microros_esp1 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp1"],
        name="micro_ros_agent_esp1"
    )
    node_microros_esp2 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp2"],
        name="micro_ros_agent_esp2"
    )
    node_microros_esp3 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/esp3"],
        name="micro_ros_agent_esp2"
    )
    
    ld.add_action(node_microros_esp1)
    ld.add_action(node_microros_esp2)
    ld.add_action(node_microros_esp3)

    return ld
