#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import numpy as np
from math import sin, cos, radians
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
class MergeOdom(Node):
    def __init__(self):
        super().__init__('merge_odom')

        # Robot config
        self.module_positions = {
            0: (0.051,  0.0),    # Front          <- /debug/module1
            1: (-0.0255, 0.0441),# Rear Left      <- /debug/module2
            2: (-0.0255,-0.0441) # Rear Right     <- /debug/module3
        }

        self.wheel_radius   = 0.02        # m
        self.ticks_per_rev  = 2800.0
        self.wheel_circ     = 2*np.pi*self.wheel_radius

        self.last_ticks = [None, None, None]  # avg tick of each module
        self.last_time  = None

        # Robot state
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.frame_id = 'odom_raw'
        self.child_frame_id = 'base_link'

        # Subscribers (match MCU topics)
                
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.sub1 = Subscriber(self, Float32MultiArray, '/debug/module1', qos_profile=qos_profile)
        self.sub2 = Subscriber(self, Float32MultiArray, '/debug/module2', qos_profile=qos_profile)
        self.sub3 = Subscriber(self, Float32MultiArray, '/debug/module3', qos_profile=qos_profile)

        self.ts = ApproximateTimeSynchronizer([self.sub1, self.sub2, self.sub3],
                                              queue_size=10, slop=0.2,
                                              allow_headerless=True)
        self.ts.registerCallback(self.odom_callback)

        self.odom_pub = self.create_publisher(Odometry, '/odom/raw', 10)

    # --- Helpers to parse each module message ---
    def parse_module_msg(self, msg: Float32MultiArray, module_index: int):
        """
        Return: (avg_tick or None, angle_rad, setzero_flag)
        Supported formats:
          Standard: [tickL, tickR, angle_deg, setzero, target_deg]
          Special (module3 HARDWARE2): [angle_front_deg, target_front, angle_rr_deg, setzero, target_rr]
        """
        data = msg.data
        n = len(data)
        if n < 4:
            return (None, None, False)

        # Detect special format for module 3 (no ticks; angle at index 2)
        if module_index == 2 and n >= 4 and abs(data[0]) <= 400 and abs(data[2]) <= 400:
            # angle of rear-right is at index 2 (degrees). No ticks available.
            angle_deg = float(data[2])
            angle_rad = radians(angle_deg)
            setzero   = (data[3] > 0.5)
            return (None, angle_rad, setzero)

        # Default (standard) format: ticks at [0],[1], angle (deg) at [2], setzero at [3]
        tickL = float(data[0])
        tickR = float(data[1])
        angle_deg = float(data[2])
        setzero   = (data[3] > 0.5)
        avg_tick = (tickL + tickR) / 2.0
        angle_rad = radians(angle_deg)  # convert deg → rad for math
        return (avg_tick, angle_rad, setzero)

    def odom_callback(self, msg1, msg2, msg3):
        now = self.get_clock().now()

        # Parse 3 modules: 0=front(msg1),1=rear-left(msg2),2=rear-right(msg3)
        parsed = [
            self.parse_module_msg(msg1, 0),
            self.parse_module_msg(msg2, 1),
            self.parse_module_msg(msg3, 2),
        ]

        # If any module asks for setzero → reset baseline ticks and time
        if any(p[2] for p in parsed):
            self.get_logger().info("Setzero flag received. Resetting baselines.")
            for i, (avg_tick, _, _) in enumerate(parsed):
                self.last_ticks[i] = avg_tick
            self.last_time = now
            return

        # Initialize baselines on first run
        if self.last_time is None or any(t is None for t in self.last_ticks):
            self.get_logger().info("Initializing baseline tick values…")
            for i, (avg_tick, _, _) in enumerate(parsed):
                self.last_ticks[i] = avg_tick
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        A_rows = []
        b_vals = []

        # Build equations only from modules that have ticks (distance info)
        for i, (avg_tick, angle_rad, _) in enumerate(parsed):
            if angle_rad is None:
                continue  # skip invalid data

            # delta from last ticks (if ticks exist)
            if avg_tick is not None and self.last_ticks[i] is not None:
                delta_tick = avg_tick - self.last_ticks[i]
                self.last_ticks[i] = avg_tick

                distance = (delta_tick / self.ticks_per_rev) * self.wheel_circ
                v = distance / dt
            else:
                # No tick available for this module → cannot get its wheel speed
                # Skip contributing velocity constraints for this module
                continue

            # Project to robot frame
            vx_i = v * cos(angle_rad)
            vy_i = v * sin(angle_rad)

            x_i, y_i = self.module_positions[i]
            # Constraints:
            # vx_i = vx - omega * y_i
            # vy_i = vy + omega * x_i
            A_rows.append([1.0, 0.0, -y_i]); b_vals.append(vx_i)
            A_rows.append([0.0, 1.0,  x_i]); b_vals.append(vy_i)

        if len(A_rows) < 3:  # need at least 2 modules (4 rows) ideally; but 3 rows also solvable LS
            # Not enough information this cycle
            return

        A = np.array(A_rows)
        b = np.array(b_vals)
        sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        vx, vy, omega = sol

        # Integrate pose (world frame)
        dtheta = omega * dt
        self.theta += dtheta

        dx = vx * dt
        dy = vy * dt
        c = cos(self.theta); s = sin(self.theta)
        self.x += c*dx - s*dy
        self.y += s*dx + c*dy

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        # yaw-only quaternion
        odom.pose.pose.orientation.z = sin(self.theta/2.0)
        odom.pose.pose.orientation.w = cos(self.theta/2.0)

        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = MergeOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
