#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math
from geometry_msgs.msg import Twist
from roomba_interfaces.msg import SensorData
from std_msgs.msg import Float32

class SimplePIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, max_output=100):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        self.max_output = max_output
        
        # Internal state
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
    def calculate(self, target_velocity, current_velocity):
        """Calculate PID output"""
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            return 0.0
            
        dt = current_time - self.last_time
        if dt <= 0:
            return 0.0
            
        # Calculate error
        error = target_velocity - current_velocity
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term (accumulated error over time)
        self.integral += error * dt
        integral_term = self.ki * self.integral
        
        # Derivative term (rate of error change)
        derivative = (error - self.last_error) / dt
        derivative_term = self.kd * derivative
        
        # Calculate total output
        output = proportional + integral_term + derivative_term
        
        # Clamp output to valid PWM range
        output = max(-self.max_output, min(self.max_output, output))
        
        # Update for next iteration
        self.last_error = error
        self.last_time = current_time
        
        return output

class WheelVelocityCalculator:
    def __init__(self, wheel_radius=0.03):
        self.wheel_radius = wheel_radius
        self.last_encoder_reading = 0.0
        self.last_time = None
        self.current_velocity = 0.0
        
    def update_from_encoder(self, encoder_reading):
        """Calculate velocity from encoder reading"""
        current_time = time.time()
        
        if self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                # Calculate distance change
                distance_change = encoder_reading - self.last_encoder_reading
                
                # Convert to linear velocity (cm/s to m/s)
                linear_velocity = (distance_change / 100.0) / dt
                
                # Convert to angular velocity (rad/s)
                self.current_velocity = linear_velocity / self.wheel_radius
                
        # Update for next iteration
        self.last_encoder_reading = encoder_reading
        self.last_time = current_time
        
        return self.current_velocity

class PIDTestNode(Node):
    def __init__(self):
        super().__init__('pid_test_node')
        
        # PID controllers for each side (since you have 2 encoders)
        self.left_pid = SimplePIDController(kp=15.0, ki=0.5, kd=0.1)
        self.right_pid = SimplePIDController(kp=15.0, ki=0.5, kd=0.1)
        
        # Velocity calculators
        self.left_velocity_calc = WheelVelocityCalculator()
        self.right_velocity_calc = WheelVelocityCalculator()
        
        # Target velocities
        self.target_left_velocity = 0.0
        self.target_right_velocity = 0.0
        
        # Current velocities
        self.current_left_velocity = 0.0
        self.current_right_velocity = 0.0
        
        # Subscribe to your existing topics (READ-ONLY)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.sensor_sub = self.create_subscription(
            SensorData,
            '/wheel_states',
            self.sensor_callback,
            10)
        
        # Publisher for test results (won't interfere with your system)
        self.test_results_pub = self.create_publisher(
            Float32,
            '/pid_test_results',
            10)
        
        # Timer for regular logging
        self.timer = self.create_timer(1.0, self.log_status)
        
        # Robot parameters
        self.wheel_separation = 0.148  # Your measured value
        
        self.get_logger().info('PID Test Node started - monitoring your robot!')
        
    def cmd_vel_callback(self, msg):
        """Monitor cmd_vel commands (same as your motor_driver receives)"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Calculate target wheel velocities (differential drive math)
        self.target_left_velocity = linear_vel - (angular_vel * self.wheel_separation / 2)
        self.target_right_velocity = linear_vel + (angular_vel * self.wheel_separation / 2)
        
        self.get_logger().info(f'New cmd_vel: linear={linear_vel:.3f}, angular={angular_vel:.3f}')
        self.get_logger().info(f'Target wheel speeds: L={self.target_left_velocity:.3f}, R={self.target_right_velocity:.3f} rad/s')
        
    def sensor_callback(self, msg):
        """Monitor your encoder data and calculate what PID would do"""
        # Calculate current wheel velocities from encoder data
        # Note: You have front_left and back_right encoders
        # We'll use left_encoder for left side, right_encoder for right side
        self.current_left_velocity = self.left_velocity_calc.update_from_encoder(msg.left_encoder)
        self.current_right_velocity = self.right_velocity_calc.update_from_encoder(msg.right_encoder)
        
        # Calculate what PID would output (but don't actually control motors)
        left_pid_output = self.left_pid.calculate(self.target_left_velocity, self.current_left_velocity)
        right_pid_output = self.right_pid.calculate(self.target_right_velocity, self.current_right_velocity)
        
        # Log the results
        self.get_logger().debug(f'Encoder readings: L={msg.left_encoder:.1f}, R={msg.right_encoder:.1f} cm')
        self.get_logger().debug(f'Current velocities: L={self.current_left_velocity:.3f}, R={self.current_right_velocity:.3f} rad/s')
        self.get_logger().debug(f'PID outputs: L={left_pid_output:.1f}, R={right_pid_output:.1f} PWM')
        
        # Calculate errors for analysis
        left_error = self.target_left_velocity - self.current_left_velocity
        right_error = self.target_right_velocity - self.current_right_velocity
        
        if abs(left_error) > 0.1 or abs(right_error) > 0.1:  # Only log significant errors
            self.get_logger().info(f'Velocity errors: L={left_error:.3f}, R={right_error:.3f} rad/s')
            self.get_logger().info(f'PID suggests motor powers: L={left_pid_output:.1f}, R={right_pid_output:.1f}')
    
    def log_status(self):
        """Regular status logging"""
        self.get_logger().info('--- PID Test Status ---')
        self.get_logger().info(f'Targets: L={self.target_left_velocity:.3f}, R={self.target_right_velocity:.3f} rad/s')
        self.get_logger().info(f'Current: L={self.current_left_velocity:.3f}, R={self.current_right_velocity:.3f} rad/s')
        
        # Calculate how well we're tracking
        left_error = abs(self.target_left_velocity - self.current_left_velocity)
        right_error = abs(self.target_right_velocity - self.current_right_velocity)
        
        if self.target_left_velocity != 0 or self.target_right_velocity != 0:
            self.get_logger().info(f'Tracking errors: L={left_error:.3f}, R={right_error:.3f} rad/s')

def main(args=None):
    rclpy.init(args=args)
    pid_test_node = PIDTestNode()
    rclpy.spin(pid_test_node)
    pid_test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()