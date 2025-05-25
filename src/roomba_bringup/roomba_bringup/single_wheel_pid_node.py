#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math
import RPi.GPIO as GPIO
from roomba_interfaces.msg import SensorData
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class SimplePIDController:
    """
    Enhanced PID controller with better velocity-to-PWM mapping
    """
    def __init__(self, kp=1.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        
        # Internal state
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
        # For debugging
        self.last_proportional = 0.0
        self.last_integral_term = 0.0
        self.last_derivative = 0.0
        
    def calculate(self, target, current):
        """Calculate PID output (returns velocity correction)"""
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            return 0.0
            
        dt = current_time - self.last_time
        if dt <= 0:
            return 0.0
            
        # Calculate error
        error = target - current
        
        # Proportional term
        self.last_proportional = self.kp * error
        
        # Integral term with better windup prevention
        self.integral += error * dt
        # Limit integral accumulation
        max_integral = 2.0  # rad/s max integral contribution
        self.integral = max(-max_integral/self.ki if self.ki > 0 else 0, 
                           min(max_integral/self.ki if self.ki > 0 else 0, self.integral))
        self.last_integral_term = self.ki * self.integral
        
        # Derivative term with filtering
        derivative = (error - self.last_error) / dt
        # Simple low-pass filter for derivative
        alpha = 0.1
        if hasattr(self, 'filtered_derivative'):
            self.filtered_derivative = alpha * derivative + (1 - alpha) * self.filtered_derivative
        else:
            self.filtered_derivative = derivative
        self.last_derivative = self.kd * self.filtered_derivative
        
        # Calculate total velocity correction
        velocity_correction = self.last_proportional + self.last_integral_term + self.last_derivative
        
        # Update for next iteration
        self.last_error = error
        self.last_time = current_time
        
        return velocity_correction
    
    def reset(self):
        """Reset the PID controller state"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None
        if hasattr(self, 'filtered_derivative'):
            delattr(self, 'filtered_derivative')

class SingleWheelPIDNode(Node):
    """
    Single wheel PID controller for testing and tuning
    Tests the left wheel only using characterization data
    """
    
    def __init__(self):
        super().__init__('single_wheel_pid')
        
        # Initialize GPIO exactly like your motor_driver.py
        self.IN1, self.IN2, self.IN3, self.IN4 = 17, 18, 22, 23
        self.ENA, self.ENB = 25, 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)
        
        # Create PWM objects (we'll only use left motor)
        self.pwm_left = GPIO.PWM(self.ENB, 100)
        self.pwm_left.start(0)
        
        # Based on your characterization data:
        # - Dead zone: Below 40% PWM produces no movement
        # - Safe range: 40-90% PWM 
        # - Left motor: 1.89-6.96 rad/s for 40-90% PWM
        self.PWM_MIN = 40.0  # Minimum effective PWM
        self.PWM_MAX = 90.0  # Maximum safe PWM (avoiding 100% instability)
        self.VELOCITY_MAX = 6.96  # Maximum safe velocity (rad/s)
        
        # Create PID controller with gains based on characterization analysis
        self.pid = SimplePIDController(
            kp=12.0,     # From our analysis: 10-15 for moderate response
            ki=0.3,      # From our analysis: 0.1-0.5 to prevent windup
            kd=0.08      # From our analysis: 0.05-0.1 for smooth response
        )
        
        # Velocity tracking
        self.last_left_encoder = None
        self.last_time = None
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.desired_velocity = 0.0  # What PID wants to achieve
        
        # Velocity-to-PWM conversion based on characterization analysis
        # Our derived equation: PWM = 40 + (velocity × 7.14)
        self.velocity_to_pwm_slope = 7.14
        self.velocity_to_pwm_offset = 40.0  # Direct from our analysis
        
        # Subscribe to encoder data
        self.sensor_sub = self.create_subscription(
            SensorData,
            '/wheel_states',
            self.sensor_callback,
            10)
        
        # Subscribe to target velocity commands
        self.target_sub = self.create_subscription(
            Float32,
            '/target_velocity',
            self.target_callback,
            10)
        
        # Also allow cmd_vel for convenience (use linear.x as target velocity)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publishers for monitoring
        self.velocity_pub = self.create_publisher(Float32, '/current_velocity', 10)
        self.error_pub = self.create_publisher(Float32, '/velocity_error', 10)
        self.pwm_pub = self.create_publisher(Float32, '/pwm_output', 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz control
        
        # Status timer  
        self.status_timer = self.create_timer(1.0, self.print_status)   # 1Hz status
        
        # Safety timer - stop if no commands received
        self.last_command_time = time.time()
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        
        self.get_logger().info('🔧 Single Wheel PID Controller Started')
        self.get_logger().info('Testing LEFT motor only')
        self.get_logger().info('Send target velocities to /target_velocity (Float32)')
        self.get_logger().info('Or use /cmd_vel (Twist) - will use linear.x')
        self.get_logger().info(f'Safe velocity range: 0 to {self.VELOCITY_MAX:.1f} rad/s')
        self.get_logger().info(f'PWM range: {self.PWM_MIN:.0f}% to {self.PWM_MAX:.0f}%')
        self.get_logger().info(f'PID Gains: Kp={self.pid.kp}, Ki={self.pid.ki}, Kd={self.pid.kd}')
        self.get_logger().info(f'PWM Equation: PWM = {self.velocity_to_pwm_offset} + ({self.velocity_to_pwm_slope} × velocity)')
        
    def sensor_callback(self, msg):
        """Process encoder data and calculate current velocity"""
        current_time = time.time()
        
        if self.last_left_encoder is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                # Calculate distance change
                left_delta = msg.left_encoder - self.last_left_encoder
                
                # Convert to velocity (matching your characterization method)
                wheel_radius = 3.0  # cm
                self.current_velocity = (left_delta / wheel_radius) / dt  # rad/s
        
        self.last_left_encoder = msg.left_encoder
        self.last_time = current_time
        
    def target_callback(self, msg):
        """Set new target velocity"""
        target = msg.data
        
        # Clamp to safe range
        target = max(0.0, min(self.VELOCITY_MAX, target))
        
        # Reset PID if target changes significantly (prevents integral windup)
        if abs(target - self.target_velocity) > 0.5:
            self.pid.reset()
            self.get_logger().info(f'🔄 PID reset due to large target change')
        
        if abs(target - self.target_velocity) > 0.1:  # Only log significant changes
            self.get_logger().info(f'🎯 New target velocity: {target:.2f} rad/s')
        
        self.target_velocity = target
        self.last_command_time = time.time()
        
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages (use linear.x as target velocity)"""
        # Convert linear velocity to wheel angular velocity
        # For testing, we'll just use linear.x directly as target rad/s
        target = abs(msg.linear.x)  # Take absolute value, only test forward
        
        # Clamp to safe range
        target = max(0.0, min(self.VELOCITY_MAX, target))
        
        # Reset PID if target changes significantly
        if abs(target - self.target_velocity) > 0.5:
            self.pid.reset()
        
        if abs(target - self.target_velocity) > 0.1:
            self.get_logger().info(f'🎯 New target from cmd_vel: {target:.2f} rad/s')
        
        self.target_velocity = target
        self.last_command_time = time.time()
        
    def control_loop(self):
        """Enhanced PID control loop with proper velocity-to-PWM conversion"""
        if self.target_velocity == 0.0:
            # Stop the motor
            self.stop_motor()
            self.desired_velocity = 0.0
            return
        
        # Calculate PID velocity correction
        velocity_correction = self.pid.calculate(self.target_velocity, self.current_velocity)
        
        # Calculate desired velocity (target + correction)
        self.desired_velocity = self.target_velocity + velocity_correction
        
        # Clamp desired velocity to safe range
        self.desired_velocity = max(0.0, min(self.VELOCITY_MAX, self.desired_velocity))
        
        # Convert desired velocity to PWM using our characterization equation
        if self.desired_velocity < 0.1:
            pwm_output = 0.0
        else:
            # Use our derived equation: PWM = 40 + (velocity × 7.14)
            pwm_output = self.velocity_to_pwm_offset + (self.desired_velocity * self.velocity_to_pwm_slope)
            
            # Ensure we stay within safe operating range (40-90% from characterization)
            pwm_output = max(self.PWM_MIN, min(self.PWM_MAX, pwm_output))
        
        # Apply to motor
        self.set_left_motor_pwm(pwm_output)
        
        # Publish monitoring data
        vel_msg = Float32()
        vel_msg.data = self.current_velocity
        self.velocity_pub.publish(vel_msg)
        
        error_msg = Float32()
        error_msg.data = self.target_velocity - self.current_velocity
        self.error_pub.publish(error_msg)
        
        pwm_msg = Float32()
        pwm_msg.data = pwm_output
        self.pwm_pub.publish(pwm_msg)
        
    def set_left_motor_pwm(self, pwm_percent):
        """Set left motor PWM using your motor driver logic"""
        if pwm_percent <= 0:
            self.stop_motor()
            return
        
        # Left motor forward (from your motor_driver.py)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_left.ChangeDutyCycle(pwm_percent)
        
    def stop_motor(self):
        """Stop the left motor"""
        self.pwm_left.ChangeDutyCycle(0)
        GPIO.output([self.IN3, self.IN4], GPIO.LOW)
        
    def safety_check(self):
        """Stop motor if no commands received recently"""
        if time.time() - self.last_command_time > 2.0:  # 2 second timeout
            if self.target_velocity > 0:
                self.get_logger().warn('⚠️  No commands received - stopping for safety')
                self.target_velocity = 0.0
                self.desired_velocity = 0.0
                self.stop_motor()
                self.pid.reset()
                
    def print_status(self):
        """Print current status for monitoring"""
        if self.target_velocity > 0.1 or self.current_velocity > 0.1:
            error = self.target_velocity - self.current_velocity
            self.get_logger().info(
                f'📊 Target: {self.target_velocity:.2f} | '
                f'Current: {self.current_velocity:.2f} | '
                f'Desired: {self.desired_velocity:.2f} | '
                f'Error: {error:.2f} rad/s | '
                f'P: {self.pid.last_proportional:.2f} | '
                f'I: {self.pid.last_integral_term:.2f} | '
                f'D: {self.pid.last_derivative:.2f}'
            )
        
    def destroy_node(self):
        """Clean up GPIO"""
        self.stop_motor()
        self.pwm_left.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SingleWheelPIDNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nPID controller stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()