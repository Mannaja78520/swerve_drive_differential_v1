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

        # ---------------- Robot config ----------------
        # โมดูล: 0=หน้า (/debug/module1), 1=หลังซ้าย (/debug/module2), 2=หลังขวา (/debug/module3)
        self.module_positions = {
            0: (0.051,   0.0),     # Front
            1: (-0.0255, 0.0441),  # Rear-Left
            2: (-0.0255,-0.0441),  # Rear-Right
        }

        self.wheel_radius  = 0.02
        self.ticks_per_rev = 2800.0
        self.wheel_circ    = 2*np.pi*self.wheel_radius

        # -------- Invert/offset ให้สอดคล้องกับ MCU --------
        # หมายเหตุ:
        # - ถ้า MCU (Encoder*) ได้ invert ไว้แล้ว ให้คง +1
        # - ถ้ามุมจาก MCU เพิ่มแบบตามเข็ม (CW) ให้ angle_sign = -1 เพื่อกลับเป็น CCW
        # - angle_offset_deg จะถูก "บวก" หลังจาก apply sign แล้ว: angle = offset_deg + angle_sign * angle_deg_mcu
        self.tick_sign = {      # ผลต่อความเร็วแปลเป็นระยะ (เฉลี่ย L/R)
            0: +1.0,  # front
            1: +1.0,  # rear-left
            2: +1.0,  # rear-right
        }
        self.angle_sign = {     # +1 = เหมือน MCU, -1 = กลับทิศจาก MCU
            0: +1.0,
            1: +1.0,
            2: +1.0,
        }
        self.angle_offset_deg = {  # ออฟเซ็ตมุมต่อโมดูล (deg), เช่น front = 90.0 ถ้าศูนย์ล้อหันขึ้น
            0: 0.0,
            1: 0.0,
            2: 0.0,
        }
        self.drive_ticks_per_rev = {
            0: 4410.0,  # front
            1: 4410.0,  # rear-left
            2: 4410.0,  # rear-right
        }


        # ---------------------------------------------------

        self.last_ticks = [None, None, None]  # avg tick ของแต่ละโมดูล (หลัง apply sign)
        self.last_time  = None

        # Robot state
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.frame_id = 'odom_raw'
        self.child_frame_id = 'base_link'

        # Subscribers
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.sub1 = Subscriber(self, Float32MultiArray, '/debug/module1', qos_profile=qos_profile)
        self.sub2 = Subscriber(self, Float32MultiArray, '/debug/module2', qos_profile=qos_profile)
        self.sub3 = Subscriber(self, Float32MultiArray, '/debug/module3', qos_profile=qos_profile)

        self.ts = ApproximateTimeSynchronizer(
            [self.sub1, self.sub2, self.sub3],
            queue_size=10, slop=0.2, allow_headerless=True
        )
        self.ts.registerCallback(self.odom_callback)

        self.odom_pub = self.create_publisher(Odometry, '/odom/raw', 10)

    # --- Helpers to parse each module message ---
    def parse_module_msg(self, msg: Float32MultiArray, module_index: int):
        """
        Return: (avg_tick_signed or None, angle_rad_corrected, setzero_flag)

        รูปแบบข้อมูลรองรับ:
          - ปกติ: [tickL, tickR, angle_deg, setzero, target_deg]
          - กรณีพิเศษ (module3 / HARDWARE2): [angle_front_deg, target_front, angle_rr_deg, setzero, target_rr]
        """
        data = msg.data
        n = len(data)
        if n < 4:
            return (None, None, False)

        # กรณีพิเศษ: โมดูล 2 (rear-right) ไม่มีทิกส์ ใช้มุม index 2
        if module_index == 2 and n >= 4 and abs(data[0]) <= 400 and abs(data[2]) <= 400:
            angle_deg_mcu = float(data[2])
            # apply sign + offset ให้ตรงกับ MCU
            angle_deg = self.angle_offset_deg[module_index] + self.angle_sign[module_index] * angle_deg_mcu
            angle_rad = radians(angle_deg)
            setzero   = (data[3] > 0.5)
            return (None, angle_rad, setzero)

        # ปกติ: มีทิกส์ L/R + มุม (deg)
        tickL = float(data[0])
        tickR = float(data[1])
        angle_deg_mcu = float(data[2])
        setzero = (data[3] > 0.5)

        # average ticks (หลัง apply sign ต่อโมดูล)
        avg_tick = (tickL + tickR) / 2.0
        avg_tick *= self.tick_sign[module_index]

        # angle: apply sign + offset
        angle_deg = self.angle_offset_deg[module_index] + self.angle_sign[module_index] * angle_deg_mcu
        angle_rad = radians(angle_deg)
        return (avg_tick, angle_rad, setzero)

    def odom_callback(self, msg1, msg2, msg3):
        now = self.get_clock().now()

        parsed = [
            self.parse_module_msg(msg1, 0),
            self.parse_module_msg(msg2, 1),
            self.parse_module_msg(msg3, 2),
        ]

        # setzero → รีเซ็ต baseline
        if any(p[2] for p in parsed):
            self.get_logger().info("Setzero flag received. Resetting baselines.")
            for i, (avg_tick, _, _) in enumerate(parsed):
                self.last_ticks[i] = avg_tick
            self.last_time = now
            return

        # init baselines
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

        # build LS constraints (เฉพาะโมดูลที่มีทิกส์)
        for i, (avg_tick, angle_rad, _) in enumerate(parsed):
            if angle_rad is None:
                continue

            if avg_tick is not None and self.last_ticks[i] is not None:
                delta_tick = avg_tick - self.last_ticks[i]
                self.last_ticks[i] = avg_tick

                distance = (delta_tick / self.drive_ticks_per_rev[i]) * self.wheel_circ
                v = distance / dt
            else:
                continue

            vx_i = v * cos(angle_rad)
            vy_i = v * sin(angle_rad)

            x_i, y_i = self.module_positions[i]
            # vx_i = vx - omega * y_i
            # vy_i = vy + omega * x_i
            A_rows.append([1.0, 0.0, -y_i]); b_vals.append(vx_i)
            A_rows.append([0.0, 1.0,  x_i]); b_vals.append(vy_i)

        if len(A_rows) < 3:
            return

        A = np.array(A_rows)
        b = np.array(b_vals)
        sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        vx, vy, omega = sol

        # integrate pose
        dtheta = omega * dt
        self.theta += dtheta

        dx = vx * dt
        dy = vy * dt
        c = cos(self.theta); s = sin(self.theta)
        self.x += c*dx - s*dy
        self.y += s*dx + c*dy

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
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
