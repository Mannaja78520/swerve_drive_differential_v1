#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data, QoSProfile

def rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    cp = math.cos(pitch* 0.5); sp = math.sin(pitch* 0.5)
    cy = math.cos(yaw  * 0.5); sy = math.sin(yaw  * 0.5)
    # ZYX
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return np.array([qw,qx,qy,qz], dtype=float)

def quat_to_mat(q):
    w,x,y,z = q
    # rotation matrix (world <- body)
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=float)
    return R

def quat_mul(q2, q1):
    # q = q2 * q1 (apply q1 then q2)
    w2,x2,y2,z2 = q2; w1,x1,y1,z1 = q1
    return np.array([
        w2*w1 - x2*x1 - y2*y1 - z2*z1,
        w2*x1 + x2*w1 + y2*z1 - z2*y1,
        w2*y1 - x2*z1 + y2*w1 + z2*x1,
        w2*z1 + x2*y1 - y2*x1 + z2*w1
    ], dtype=float)

def quat_normalize(q):
    n = np.linalg.norm(q)
    return q if n == 0.0 else (q / n)

class ImuAxisCorrector(Node):
    def __init__(self):
        super().__init__('imu_axis_corrector')

        # ---- params ----
        self.declare_parameter('input_topic',  '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_corrected')
        # ดีฟอลต์: หมุน 180° ที่แกน X เพื่อ “กลับสัญญาณ yaw”
        self.declare_parameter('r_offset_deg', 180.0)
        self.declare_parameter('p_offset_deg', 0.0)
        self.declare_parameter('y_offset_deg', 0.0)
        self.declare_parameter('copy_covariances', True)

        self.input_topic  = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.r_offset = math.radians(self.get_parameter('r_offset_deg').get_parameter_value().double_value)
        self.p_offset = math.radians(self.get_parameter('p_offset_deg').get_parameter_value().double_value)
        self.y_offset = math.radians(self.get_parameter('y_offset_deg').get_parameter_value().double_value)
        self.copy_cov  = self.get_parameter('copy_covariances').get_parameter_value().bool_value

        # สร้างควิเทอร์เนียนของ “การหมุนแก้เฟรมเซนเซอร์ -> เฟรมถูกต้อง (ENU)”
        self.q_corr = quat_normalize(rpy_to_quat(self.r_offset, self.p_offset, self.y_offset))
        self.R_corr = quat_to_mat(self.q_corr)

        # dynamic params
        self.add_on_set_parameters_callback(self.on_param_change)

        # sub/pub
        self.sub = self.create_subscription(
            Imu, self.input_topic, self.cb, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Imu, self.output_topic, 10)

        self.get_logger().info(
            f"IMU Axis Corrector: in={self.input_topic}, out={self.output_topic}, "
            f"RPY_offset(deg)=({math.degrees(self.r_offset):.1f},"
            f"{math.degrees(self.p_offset):.1f},{math.degrees(self.y_offset):.1f})"
        )

    def on_param_change(self, params):
        for p in params:
            if p.name == 'r_offset_deg':
                self.r_offset = math.radians(float(p.value))
            elif p.name == 'p_offset_deg':
                self.p_offset = math.radians(float(p.value))
            elif p.name == 'y_offset_deg':
                self.y_offset = math.radians(float(p.value))
        self.q_corr = quat_normalize(rpy_to_quat(self.r_offset, self.p_offset, self.y_offset))
        self.R_corr = quat_to_mat(self.q_corr)
        self.get_logger().info(
            f"Updated RPY_offset(deg)=({math.degrees(self.r_offset):.1f},"
            f"{math.degrees(self.p_offset):.1f},{math.degrees(self.y_offset):.1f})"
        )
        return SetParametersResult(successful=True)

    def cb(self, m: Imu):
        out = Imu()
        out.header = m.header  # frame_id เดิม (เช่น imu_link)

        # ---- orientation ----
        q_in = np.array([m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z], dtype=float)
        # หมายเหตุ: ต้อง pre-multiply ด้วย q_corr เพื่อ “หมุนเซนเซอร์ -> เฟรมถูกต้อง”
        q_out = quat_mul(self.q_corr, q_in)
        q_out = quat_normalize(q_out)
        out.orientation.w = float(q_out[0])
        out.orientation.x = float(q_out[1])
        out.orientation.y = float(q_out[2])
        out.orientation.z = float(q_out[3])

        # ---- angular velocity ----  (เวกเตอร์ต้องคูณด้วย R_corr)
        gx, gy, gz = m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z
        g = np.array([gx, gy, gz], dtype=float)
        g_corr = self.R_corr @ g
        out.angular_velocity.x = float(g_corr[0])
        out.angular_velocity.y = float(g_corr[1])
        out.angular_velocity.z = float(g_corr[2])

        # ---- linear acceleration ----
        ax, ay, az = m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z
        a = np.array([ax, ay, az], dtype=float)
        a_corr = self.R_corr @ a
        out.linear_acceleration.x = float(a_corr[0])
        out.linear_acceleration.y = float(a_corr[1])
        out.linear_acceleration.z = float(a_corr[2])

        # covariance
        if self.copy_cov:
            out.orientation_covariance = m.orientation_covariance
            out.angular_velocity_covariance = m.angular_velocity_covariance
            out.linear_acceleration_covariance = m.linear_acceleration_covariance
        else:
            # ตั้งแบบกลาง ๆ
            out.orientation_covariance[8] = (5.0*math.pi/180.0)**2  # yaw var
            out.angular_velocity_covariance[8] = (0.1)**2
            out.linear_acceleration_covariance[0] = 0.2**2
            out.linear_acceleration_covariance[4] = 0.2**2
            out.linear_acceleration_covariance[8] = 0.3**2

        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ImuAxisCorrector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
