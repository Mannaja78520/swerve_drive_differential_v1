import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf2_ros
from math import sin, cos
from message_filters import Subscriber, ApproximateTimeSynchronizer

class MergeOdom(Node):
    def __init__(self):
        super().__init__('merge_odom')

        # Robot config
        self.module_positions = {
            0: (0.051, 0.0),         # Front
            1: (-0.0255, -0.0441),   # Rear Right
            2: (-0.0255, 0.0441),    # Rear Left
        }

        self.wheel_radius = 0.02  # meters
        self.ticks_per_rev = 2800.0
        self.wheel_circ = 2 * np.pi * self.wheel_radius

        self.last_ticks = [None, None, None]
        self.last_time = None

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.frame_id = 'odom_raw'
        self.child_frame_id = 'base_link'
        
        # Subscribers (one per module)
        self.sub0 = Subscriber(self, Float32MultiArray, '/differential_swerve_module_0')
        self.sub1 = Subscriber(self, Float32MultiArray, '/differential_swerve_module_1')
        self.sub2 = Subscriber(self, Float32MultiArray, '/differential_swerve_module_2')

        self.ts = ApproximateTimeSynchronizer(
            [self.sub0, self.sub1, self.sub2],
            queue_size=10,
            slop=0.05
        )
        self.ts.registerCallback(self.odom_callback)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Merge odom node with setzero support started.")

    def odom_callback(self, msg0, msg1, msg2):
        now = self.get_clock().now()

        ticks = []
        angles = []
        setzero_flags = []

        for msg in [msg0, msg1, msg2]:
            tick_left = msg.data[0]
            tick_right = msg.data[1]
            angle = msg.data[2]
            setzero = msg.data[3] > 0.5  # true if > 0.5

            avg_tick = (tick_left + tick_right) / 2.0

            ticks.append(avg_tick)
            angles.append(angle)
            setzero_flags.append(setzero)

        # If any module is in setzero mode, reset baseline and skip odometry update
        if any(setzero_flags):
            self.get_logger().info("Setzero flag received. Resetting baseline ticks.")
            for i in range(3):
                self.last_ticks[i] = ticks[i]
            self.last_time = now
            return

        if self.last_time is None or any(t is None for t in self.last_ticks):
            self.get_logger().info("Initializing baseline tick values...")
            self.last_time = now
            for i in range(3):
                self.last_ticks[i] = ticks[i]
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        A = []
        b = []

        for i in range(3):
            delta_tick = ticks[i] - self.last_ticks[i]
            self.last_ticks[i] = ticks[i]

            distance = (delta_tick / self.ticks_per_rev) * self.wheel_circ
            velocity = distance / dt

            angle = angles[i]
            vx = velocity * cos(angle)
            vy = velocity * sin(angle)

            x_i, y_i = self.module_positions[i]

            A.append([1, 0, -y_i])
            A.append([0, 1,  x_i])
            b.append(vx)
            b.append(vy)

        A = np.array(A)
        b = np.array(b)
        sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        vx, vy, omega = sol

        # Update robot pose
        dtheta = omega * dt
        self.theta += dtheta

        dx = vx * dt
        dy = vy * dt
        dx_world = cos(self.theta) * dx - sin(self.theta) * dy
        dy_world = sin(self.theta) * dx + cos(self.theta) * dy
        self.x += dx_world
        self.y += dy_world

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qz = sin(self.theta / 2)
        qw = cos(self.theta / 2)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = MergeOdom()
    rclpy.spin(node)
    rclpy.shutdown()
