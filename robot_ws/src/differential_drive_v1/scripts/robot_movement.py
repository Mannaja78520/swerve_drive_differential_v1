#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from rclpy import qos
from src.utilize import * 
from src.controller import *
import math
from typing import List, Tuple


class robot_movement(Node):

    def __init__(self):
        super().__init__("robot_movement")

        # Declare parameters with default values
        self.declare_parameter("wheel_radius", "mps")
        self.declare_parameter("max_pwm_Speed", 1023.0)
        self.declare_parameter("max_rpm", 6000)
        self.declare_parameter("max_linear_speed", 10.0)
        self.declare_parameter("wheel_diameter", 0.762)
        self.declare_parameter("gear_ratio", 18.0)
        self.declare_parameter("wheel_distance", 0.3)  # L
        self.declare_parameter("movement_mode", "default_value")

        # Load parameters
        self.movement_mode: str = self.get_parameter("movement_mode").get_parameter_value().string_value
        self.maxPWMSpeed: float = self.get_parameter("max_pwm_Speed").get_parameter_value().double_value
        self.maxRPM: int = self.get_parameter("max_rpm").get_parameter_value().integer_value
        self.max_linear_speed: float = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self.wheel_diameter: float = self.get_parameter("wheel_diameter").get_parameter_value().double_value
        self.gear_ratio: float = self.get_parameter("gear_ratio").get_parameter_value().double_value
        L: float = self.get_parameter("wheel_distance").get_parameter_value().double_value
        
        self.safe_maxRPM: float = self.maxRPM * 0.85
        
        self.wheel_commands: List[Tuple[float, float]] = [(0.0, 0.0)] * 3

        # Wheel location relative to robot center (in meters)
        # Equilateral triangle layout (L = distance from center to wheel)
        L = 0.3  # Change this based on your actual robot dimensions
        self.wheel_positions = [
            (L, 0),                          # Wheel 1: front
            (-L/2, math.sqrt(3)*L/2),        # Wheel 2: rear-left
            (-L/2, -math.sqrt(3)*L/2)        # Wheel 3: rear-right
        ]


 
        self.send_robot_move_speed = self.create_publisher(
            Twist, "/wheel/motor_speed", qos_profile=qos.qos_profile_system_default
        )
        
        self.send_robot_wheel_angle = self.create_publisher(
            Twist, "/wheel/angle", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_sub, qos_profile=qos.qos_profile_sensor_data
        )
        
        self.create_subscription(
            String, '/movement_mode', self.movement_mode_sub, qos_profile=qos.qos_profile_sensor_data
        )


        self.sent_data_timer = self.create_timer(0.03, self.sendData)


    def calculate (self, ticks_per_rev, motor_gear_ratio, module_track_width_mm, drive_gear_diameter_mm ):
        
        circumference_gear_motor = drive_gear_diameter_mm * math.pi
        circumference_gear_module = module_track_width_mm * math.pi
        gear_revs_for_360_turn = circumference_gear_module / circumference_gear_motor
        motor_revs_for_360_turn = gear_revs_for_360_turn * motor_gear_ratio
        self.TICKS_PER_360_DEG_ROTATION = motor_revs_for_360_turn * ticks_per_rev

        self.current_angle_deg = 0.0
        self.last_total_ticks_L = 0
        self.last_total_ticks_R = 0  
 




            
    
    def cmd_vel_sub(self, msg): 
        self.moveSpeed = msg.linear.x 
        self.slideSpeed = msg.linear.y
        self.turnSpeed = msg.angular.z 
        
        DEAD_ZONE = 0.01
        if abs(self.moveSpeed) < DEAD_ZONE:
            self.moveSpeed = 0.0
        if abs(self.slideSpeed) < DEAD_ZONE:
            self.slideSpeed = 0.0
        if abs(self.turnSpeed) < DEAD_ZONE:
            self.turnSpeed = 0.0
        
        # Vx, Vy, omega are the linear and angular velocities
        
        max_speed: float = self.max_linear_speed if self.movement_mode == "mps" else self.maxPWMSpeed
        # Normalize wheel speeds to ensure they fit within the max speed
        D = max(abs(self.moveSpeed) + abs(self.slideSpeed) + abs(self.turnSpeed), max_speed)
        # Scale wheel speeds by D if needed
        self.wheel_commands = [
            (Wa / D * max_speed, Wb / D * max_speed)
            for (Wa, Wb) in self.wheel_commands
        ]

    

        #use mps
        if self.movement_mode == "mps":
            print("[mps] mode")
            for i, (speed, angle) in enumerate(self.wheel_commands):
                print(f"Wheel {i+1}: speed = {speed:.2f} m/s, angle = {math.degrees(angle):.2f}°")
            
        #use pwm
        elif self.movement_mode == "manual":
            print("[manual] mode (PWM input, angle estimated)")
            for i, (pwm, angle) in enumerate(self.wheel_commands):
                print(f"  Wheel {i+1}: speed = {pwm:.0f} PWM, angle = {math.degrees(angle):.1f}°")


            
            
            
    def movement_mode_sub(self, msg):
        self.movement_mode = msg.data
        self.get_logger().info(f"Movement mode set to: {self.movement_mode}")
        
    
    def mps_to_rpm(self, speed_mps):
        return (speed_mps * 60) / (math.pi * self.wheel_diameter)

        
    def normalize_pwms(self, pwms: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        MAX_PWM = 1032 

        flat_pwms = [abs(Wa) for Wa, _ in pwms] + [abs(Wb) for _, Wb in pwms]
        max_pwm = max(flat_pwms) if flat_pwms else 1.0

        if max_pwm > MAX_PWM:
            scale = MAX_PWM / max_pwm
            return [(Wa * scale, Wb * scale) for Wa, Wb in pwms]
        return pwms



    def normalize_rpms(self, rpms: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
            flat_rpms = [abs(Wa) for Wa, _ in rpms] + [abs(Wb) for _, Wb in rpms]
            max_rpm = max(flat_rpms) if flat_rpms else 1.0

            if max_rpm > self.safe_maxRPM:
                scale = self.safe_maxRPM / max_rpm
                return [(Wa * scale, Wb * scale) for Wa, Wb in rpms]
            return rpms
    
    def sendData(self):
        send_robot_move_speed_msg = Twist()
        # robot_wheel_angle_msg = Twist()
       
        if self.movement_mode == "mps":
            rpm_values = [[self.mps_to_rpm(Wa) * self.gear_ratio , self.mps_to_rpm(Wb) * self.gear_ratio] for Wa, Wb in self.wheel_commands]
            rpm_values = self.normalize_rpms(rpm_values)
            # self.wheel_commands = self.normalize_pwms(self.wheel_commands)


            send_robot_move_speed_msg.linear.x = float(rpm_values[0][0])
            send_robot_move_speed_msg.linear.y = float(rpm_values[1][0])
            send_robot_move_speed_msg.linear.z = float(rpm_values[2][0])

            send_robot_move_speed_msg.angular.x = float(rpm_values[0][1])
            send_robot_move_speed_msg.angular.y = float(rpm_values[1][1])
            send_robot_move_speed_msg.angular.z = float(rpm_values[2][1])
            
        elif self.movement_mode == "manual":
            send_robot_move_speed_msg.linear.x = float(self.wheel_commands[0][0]) #Motor 1A
            send_robot_move_speed_msg.linear.y = float(self.wheel_commands[1][0]) #Motor 2A
            send_robot_move_speed_msg.linear.z = float(self.wheel_commands[2][0]) #Motor 3A

            send_robot_move_speed_msg.angular.x = float(self.wheel_commands[0][1])  #Motor 1B
            send_robot_move_speed_msg.angular.y = float(self.wheel_commands[1][1])  #Motor 2B
            send_robot_move_speed_msg.angular.z = float(self.wheel_commands[2][1])  #Motor 3B  

        # robot_wheel_angle_msg.linear.x = float(math.degrees(self.wheel_commands[0][1]))
        # robot_wheel_angle_msg.linear.y = float(math.degrees(self.wheel_commands[1][1]))
        # robot_wheel_angle_msg.linear.z = float(math.degrees(self.wheel_commands[2][1]))

        self.send_robot_move_speed.publish(send_robot_move_speed_msg)
        # self.send_robot_wheel_angle.publish(robot_wheel_angle_msg)




def main():
    rclpy.init()

    sub = robot_movement()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()