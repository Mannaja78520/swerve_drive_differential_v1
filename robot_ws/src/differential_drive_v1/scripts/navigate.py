#!/usr/bin/env python3
import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32MultiArray
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import quaternion_from_euler
import time
from src.utilize import To_Radians

class Navigate(Node):
    def __init__(self, navigator: BasicNavigator):
        super().__init__('navigate_node')
        self.navigator = navigator

        # Create publisher to indicate navigation goal completion
        self.nav_goal_finished = self.create_publisher(
            Bool,
            "goal/state",
            qos_profile=qos.qos_profile_system_default,
        )
        
        self.sub_ip = self.create_subscription(
            Float32MultiArray,
            "robot/target_nav_ip",
            self.sub_ip_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        
        self.sub_goal = self.create_subscription(
            Float32MultiArray,
            "robot/target_nav_goal",
            self.sub_goal_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        
        self.__previous_target_goal = Float32MultiArray()
        self.sent_timer = self.create_timer(0.1, self.timer_callback)
        
    def sub_ip_callback(self, initial_pose: Float32MultiArray):
        ip = self.get_initial_pose(
            initial_pose.data[0], initial_pose.data[1], initial_pose.data[2]
        )
        self.navigator.setInitialPose(ip)
        time.sleep(0.1)
        self.navigator.clearAllCostmaps()
        
    def get_initial_pose(self, x: float, y: float, yaw: float):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.position.z = 0.0
        (
            initial_pose.pose.orientation.x,
            initial_pose.pose.orientation.y,
            initial_pose.pose.orientation.z,
            initial_pose.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, To_Radians(yaw))
        return initial_pose

    def sub_goal_callback(self, goal: Float32MultiArray):
        if goal.data == self.__previous_target_goal:
            return
        targets = []
        for i in range(len(goal.data) // 3):
            target = self.get_point(goal.data[i * 3], goal.data[(i * 3) + 1], goal.data[(i * 3) + 2])
            targets.append(target)
        self.navigator.goThroughPoses(targets)
        self.__previous_target_goal = goal.data

    def get_point(self, x: float, y: float, yaw: float):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        (
            goal_pose.pose.orientation.x,
            goal_pose.pose.orientation.y,
            goal_pose.pose.orientation.z,
            goal_pose.pose.orientation.w
        ) = quaternion_from_euler(0.0, 0.0, To_Radians(yaw))
        return goal_pose
    
    def timer_callback(self):
        finished_msg = Bool(data=False)
        finished_msg.data = self.navigator.isTaskComplete()
        self.nav_goal_finished.publish(finished_msg)
        
        
class CancelNavigate(Node):
    def __init__(self, navigator: BasicNavigator):
        super().__init__('cancel_navigate_node')
        self.navigator = navigator

        self.sub_cancel = self.create_subscription(
            Bool,
            "robot/cancel_nav",
            self.sub_cancel_nav_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )

    def sub_cancel_nav_callback(self, cancel_msg: Bool):
        if cancel_msg.data:
            self.navigator.cancelTask()          
            # self.navigator.clearAllCostmaps()
            navigate_node.__previous_target_goal = Float32MultiArray()  # Reset the previous target goal
            # time.sleep(1)

def main():
    rclpy.init()

    # Initialize the navigator
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active() 

    global navigate_node
    navigate_node = Navigate(navigator)
    cancel_navigate_node = CancelNavigate(navigator)

    # Use a MultiThreadedExecutor to handle multiple nodes
    executor = MultiThreadedExecutor()
    executor.add_node(navigate_node)
    executor.add_node(cancel_navigate_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        navigate_node.destroy_node()
        cancel_navigate_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()