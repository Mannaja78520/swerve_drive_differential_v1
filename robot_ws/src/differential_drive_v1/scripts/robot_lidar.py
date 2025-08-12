#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class LidarClear(Node):
    def __init__(self):
        super().__init__('fobot_lidar')
        
        # กำหนด QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile)
        
        self.publisher = self.create_publisher(
            LaserScan,
            'filtered_scan',
            qos_profile)
        
        self.min_range = 0.1  # 25 centimeters in meters

    def listener_callback(self, msg):
        filtered_ranges = [float('inf') if range < self.min_range else range for range in msg.ranges]
        
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = self.min_range
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered_ranges
        filtered_msg.intensities = msg.intensities

        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    fobot_lidar = LidarClear()
    rclpy.spin(fobot_lidar)
    fobot_lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()