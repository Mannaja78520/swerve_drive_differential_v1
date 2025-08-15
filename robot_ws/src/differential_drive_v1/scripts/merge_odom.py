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

        self.wheel_radius  = 0.0205                     # m
        self.wheel_circ    = 2 * np.pi * self.wheel_radius

        # -------- Invert/offset ให้สอดคล้องกับ MCU --------
        # - angle = offset + sign * angle_mcu (องศา, CCW บวก)
        self.tick_sign = {        # ผลต่อความเร็วแปลเป็นระยะ (เฉลี่ย L/R)
            0: +1.0,  # front
            1: +1.0,  # rear-left
            2: +1.0,  # rear-right
        }
        self.angle_sign = {       # +1 = เหมือน MCU, -1 = กลับทิศจาก MCU
            0: -1.0,
            1: -1.0,
            2: -1.0,
        }
        self.angle_offset_deg = { # ออฟเซ็ตมุมต่อโมดูล (deg)
            0: -90.0,
            1: 30.0,
            2: 150.0,
        }

        # ticks ต่อ "รอบล้อ" (ถ้า encoder อยู่ที่เพลามอเตอร์ ให้คูณเกียร์รวมทางขับ)
        # ตัวอย่าง 2800 cnt/rev * 1.575 = 4410
        self.drive_ticks_per_rev = {
            0: 4410.0,  # front
            1: 4410.0,  # rear-left
            2: 4410.0,  # rear-right
        }
        # ---------------------------------------------------

        # baseline avg และ L/R ต่อโมดูล
        self.last_ticks    = [None, None, None]  # avg tick หลัง apply sign
        self.last_ticks_L  = [None, None, None]
        self.last_ticks_R  = [None, None, None]
        self.last_time     = None

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.frame_id = 'odom'        # ให้ตรงกับ EKF/TF-chain
        self.child_frame_id = 'base_link'

        # โหมดพ่น hold-odom (กัน Nav/EKF ล้มตอน setzero/สัญญาณขาด)
        self.publish_idle = True         # เปิด/ปิด
        self.stale_timeout = 0.3         # ถ้าเกินเวลานี้ไม่มี solution ใหม่ -> พ่น hold
        self.last_solution_time = None
        self.timer_hz = 50.0
        self.timer = self.create_timer(1.0 / self.timer_hz, self.on_timer)

        # ตัวกรองกำลังเลี้ยว / deadband / clamp
        self.v_deadband_mps   = 0.02     # m/s ต่ำกว่านี้ปัดเป็น 0 (เข้มขึ้นเล็กน้อย)
        self.steering_thresh  = 0.6      # |dL-dR| / (|dL|+|dR|) > thresh => ถือว่าเลี้ยว
        self.max_abs_omega    = 3.0      # rad/s จำกัดผล LS ไม่ให้หลุด

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
        Return: (avg_tick_signed or None, angle_rad_corrected, setzero_flag, tickL_signed, tickR_signed)

        รูปแบบข้อมูลรองรับ:
          - ปกติ: [tickL, tickR, angle_deg, setzero, target_deg]
          - กรณีพิเศษ (module3/HARDWARE2): [angle_front_deg, target_front, angle_rr_deg, setzero, target_rr]
        """
        data = msg.data
        n = len(data)
        if n < 4:
            return (None, None, False, None, None)

        # กรณีพิเศษ: โมดูล 2 (rear-right) ไม่มีทิกส์ ใช้มุม index 2
        if module_index == 2 and n >= 4 and abs(data[0]) <= 400 and abs(data[2]) <= 400:
            angle_deg_mcu = float(data[2])
            angle_deg = self.angle_offset_deg[module_index] + self.angle_sign[module_index] * angle_deg_mcu
            angle_rad = radians(angle_deg)
            setzero   = (data[3] > 0.5)
            return (None, angle_rad, setzero, None, None)

        # ปกติ: มีทิกส์ L/R + มุม (deg)
        tickL = float(data[0])
        tickR = float(data[1])
        angle_deg_mcu = float(data[2])
        setzero = (data[3] > 0.5)

        tickL_signed = tickL * self.tick_sign[module_index]
        tickR_signed = tickR * self.tick_sign[module_index]

        # average ticks (หลัง apply sign ต่อโมดูล)
        avg_tick = (tickL_signed + tickR_signed) / 2.0

        # angle: apply sign + offset
        angle_deg = self.angle_offset_deg[module_index] + self.angle_sign[module_index] * angle_deg_mcu
        angle_rad = radians(angle_deg)
        return (avg_tick, angle_rad, setzero, tickL_signed, tickR_signed)

    def odom_callback(self, msg1, msg2, msg3):
        now = self.get_clock().now()

        parsed = [
            self.parse_module_msg(msg1, 0),
            self.parse_module_msg(msg2, 1),
            self.parse_module_msg(msg3, 2),
        ]

        # setzero → รีเซ็ต baseline + พ่น hold-odom (ถ้าเปิดโหมด)
        if any(p[2] for p in parsed):
            self.get_logger().info("Setzero flag received. Resetting baselines.")
            for i, (avg_tick, _, _, tickL, tickR) in enumerate(parsed):
                self.last_ticks[i]   = avg_tick
                self.last_ticks_L[i] = tickL
                self.last_ticks_R[i] = tickR
            self.last_time = now
            if self.publish_idle:
                self.publish_hold(now)
                self.last_solution_time = now
            return

        # init baselines
        if self.last_time is None or any(t is None for t in self.last_ticks):
            self.get_logger().info("Initializing baseline tick values…")
            for i, (avg_tick, _, _, tickL, tickR) in enumerate(parsed):
                self.last_ticks[i]   = avg_tick
                self.last_ticks_L[i] = tickL
                self.last_ticks_R[i] = tickR
            self.last_time = now
            # ยังไม่ publish จนกว่าจะมี solution แรก
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        A_rows = []
        b_vals = []

        # build LS constraints (เฉพาะโมดูลที่มีทิกส์ และไม่กำลังเลี้ยว)
        for i, item in enumerate(parsed):
            avg_tick, angle_rad, _, tickL_signed, tickR_signed = item
            if angle_rad is None:
                continue

            # ต้องมี avg และ baseline avg
            if avg_tick is None or self.last_ticks[i] is None:
                # อัปเดต L/R baseline ถ้ามี
                if tickL_signed is not None:
                    self.last_ticks_L[i] = tickL_signed
                if tickR_signed is not None:
                    self.last_ticks_R[i] = tickR_signed
                continue

            # คำนวณตัวชี้วัดการเลี้ยว (ต้องมี L/R ครบ)
            if (tickL_signed is not None and tickR_signed is not None and
                self.last_ticks_L[i] is not None and self.last_ticks_R[i] is not None):
                dL = tickL_signed - self.last_ticks_L[i]
                dR = tickR_signed - self.last_ticks_R[i]
                steer_ind = abs(dL - dR) / (abs(dL) + abs(dR) + 1e-6)
            else:
                dL = dR = 0.0
                steer_ind = 0.0

            # อัปเดต L/R baseline
            if tickL_signed is not None: self.last_ticks_L[i] = tickL_signed
            if tickR_signed is not None: self.last_ticks_R[i] = tickR_signed

            # อัปเดต avg baseline และระยะ
            delta_tick = avg_tick - self.last_ticks[i]
            self.last_ticks[i] = avg_tick

            # ถ้ากำลังเลี้ยวแรง -> ตัดทิ้งรอบนี้ (ไม่ให้ contribute ระยะ)
            if steer_ind > self.steering_thresh:
                continue

            distance = (delta_tick / self.drive_ticks_per_rev[i]) * self.wheel_circ
            v = distance / dt

            # deadband ความเร็ว
            if abs(v) < self.v_deadband_mps:
                v = 0.0

            vx_i = v * cos(angle_rad)
            vy_i = v * sin(angle_rad)

            x_i, y_i = self.module_positions[i]
            # vx_i = vx - omega * y_i
            # vy_i = vy + omega * x_i
            A_rows.append([1.0, 0.0, -y_i]); b_vals.append(vx_i)
            A_rows.append([0.0, 1.0,  x_i]); b_vals.append(vy_i)

        if len(A_rows) < 3:
            # ไม่มีสมการพอ -> ให้ timer เป็นคนพ่น hold แทน
            return

        A = np.array(A_rows)
        b = np.array(b_vals)
        sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        vx, vy, omega = sol

        # จำกัด omega กันหลุด
        if omega > self.max_abs_omega:
            omega = self.max_abs_omega
        elif omega < -self.max_abs_omega:
            omega = -self.max_abs_omega

        # integrate pose
        dtheta = omega * dt
        self.theta += dtheta

        dx = vx * dt
        dy = vy * dt
        c = cos(self.theta); s = sin(self.theta)
        self.x += c * dx - s * dy
        self.y += s * dx + c * dy

        # publish odom (solution ใหม่)
        self.publish_odom(now, vx, vy, omega)
        self.last_solution_time = now

    # ---------- Hold-odom timer / helper ----------
    def on_timer(self):
        if not self.publish_idle:
            return
        now = self.get_clock().now()
        if self.last_solution_time is None:
            # ยังไม่เคยมี solution -> ยังไม่พ่น
            return
        dt = (now - self.last_solution_time).nanoseconds / 1e9
        if dt > self.stale_timeout:
            # ไม่มี solution ใหม่ -> พ่น hold เพื่อให้ timestamp สด
            self.publish_hold(now)

    def publish_odom(self, now, vx, vy, omega):
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = cos(self.theta / 2.0)

        # --- Pose covariance (กลางๆ ไม่ให้ EKFเชื่อเกินไป) ---
        for k in range(36): odom.pose.covariance[k] = 0.0
        odom.pose.covariance[0]  = 0.05 * 0.05                  # var(x)
        odom.pose.covariance[7]  = 0.05 * 0.05                  # var(y)
        odom.pose.covariance[35] = (5.0*np.pi/180.0)**2         # var(yaw) ≈ (5°)^2

        # twist
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = omega

        # --- Twist covariance (วางกว้างขึ้นที่ yaw กัน spin leak) ---
        for k in range(36): odom.twist.covariance[k] = 0.0
        odom.twist.covariance[0]  = 0.02 * 0.02                 # var(vx)
        odom.twist.covariance[7]  = 0.02 * 0.02                 # var(vy)
        odom.twist.covariance[35] = (0.2)**2                    # var(wz)

        self.odom_pub.publish(odom)

    def publish_hold(self, now):
        # ถือท่าไว้ + ความเร็วศูนย์ + covariance เดียวกับ publish_odom
        self.publish_odom(now, 0.0, 0.0, 0.0)


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
