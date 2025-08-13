#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from builtin_interfaces.msg import Time

class FakeImuPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 20 Hz

        self.seq = 0
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        imu_msg = Imu()

        # timestamp
        now = self.get_clock().now().to_msg()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'imu_link'

        # orientation (quaternion) - here just zero rotation for example
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0
        imu_msg.orientation.w = 1.0

        # angular velocity - simulate slow rotation around z axis
        t = self.get_clock().now().seconds_nanoseconds()[0]
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.1 * math.sin(t)

        # linear acceleration - simulate static with gravity on z axis
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # gravity

        self.publisher_.publish(imu_msg)
        self.get_logger().info('Publishing fake IMU data')

def main(args=None):
    rclpy.init(args=args)
    node = FakeImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
