#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist
import tf_transformations
import math, time

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odom)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        self.pub.publish(msg)

rclpy.init()
node = FakeOdom()
rclpy.spin(node)
rclpy.shutdown()
