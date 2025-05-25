#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math
import RPi.GPIO as GPIO
from roomba_interfaces.msg import SensorData
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class SimpleVelocityController(Node):
    """
    Simple feedforward velocity controller using direct motor characterization mapping
    No complex PID - just direct velocity-to-PWM conversion with minor feedback correction
    """
    
    def __init__(self):
        super().__init__('simple_velocity_controller')
        
        # Initialize GPIO exactly like your motor_driver.py
        self.IN1, self.IN2, self.IN3, self.IN4 = 17, 18, 22, 23
        self.ENA, self.ENB = 25, 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)
        
        # Create PWM objects (we'll only use left motor for testing)
        self.pwm_left = GPIO.PWM(self.ENB, 100)
        self.pwm_left.start(0)
        
        # Motor characterization data (from your successful characterization)
        # These are the ACTUAL measured responses from your motor
        self.characterization_points = [
            (40, 1.89),   # (PWM%, velocity rad/s)
            (50, 3.39),
            (60, 4.13),
            (70, 5.29),
            (80, 6.02),
            (90, 6.96)
        ]
        
        # Safe operating limits
        self.PWM_MIN = 40.0
        self.PWM_MAX = 90.0
        self.VELOCITY_MAX = 6.96
        
        # Velocity tracking
        self.last_left_encoder = None
        self.last_time = None
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.last_pwm_output = 0.0
        
        # Target change tracking to handle stale velocity readings
        self.target_changed = False
        self.target_change_time = 0.0
        
        # Simple feedback correction
        self.feedback_gain = 2.0  # Small correction factor
        
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
        
        # Also allow cmd_vel for convenience
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publishers for monitoring
        self.velocity_pub = self.create_publisher(Float32, '/current_velocity', 10)
        self.error_pub = self.create_publisher(Float32, '/velocity_error', 10)
        self.pwm_pub = self.create_publisher(Float32, '/pwm_output', 10)
        
        # Control timer - slower and more stable
        self.control_timer = self.create_timer(0.2, self.control_loop)  # 5Hz control
        
        # Status timer  
        self.status_timer = self.create_timer(2.0, self.print_status)   # 0.5Hz status
        
        # Safety timer
        self.last_command_time = time.time()
        self.safety_timer = self.create_timer(0.5, self.safety_check)
        
        self.get_logger().info('🔧 Simple Velocity Controller Started')
        self.get_logger().info('Using direct feedforward control from motor characterization')
        self.get_logger().info('Testing LEFT motor only')
        self.get_logger().info('Send target velocities to /target_velocity (Float32)')
        self.get_logger().info(f'Safe velocity range: 0 to {self.VELOCITY_MAX:.1f} rad/s')
        self.get_logger().info(f'PWM range: {self.PWM_MIN:.0f}% to {self.PWM_MAX:.0f}%')
        
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
                
                # Clear the target_changed flag now that we have fresh velocity data
                if self.target_changed:
                    self.target_changed = False
                    self.get_logger().debug(f'Fresh velocity reading: {self.current_velocity:.2f} rad/s')
        
        self.last_left_encoder = msg.left_encoder
        self.last_time = current_time
        
    def target_callback(self, msg):
        """Set new target velocity"""
        target = msg.data
        
        # Clamp to safe range
        target = max(0.0, min(self.VELOCITY_MAX, target))
        
        # Check if this is a significant target change
        if abs(target - self.target_velocity) > 0.1:
            self.get_logger().info(f'🎯 New target velocity: {target:.2f} rad/s')
            
            # ALWAYS reset encoder tracking for ANY target change (including 0.0)
            # This prevents using stale velocity readings from previous tests
            self.target_changed = True
            self.target_change_time = time.time()
            
            # Reset encoder tracking to get fresh velocity reading
            self.last_left_encoder = None
            self.last_time = None
            self.current_velocity = 0.0  # Will be updated by next sensor reading
            
            # SPECIAL DEBUG: Log when target is 0.0
            if target == 0.0:
                self.get_logger().info('🛑 STOP COMMAND RECEIVED - Resetting velocity to 0.0')
        
        self.target_velocity = target
        self.last_command_time = time.time()
        
    def cmd_vel_callback(self, msg):
        """Handle cmd_vel messages"""
        target = abs(msg.linear.x)  # Take absolute value, only test forward
        target = max(0.0, min(self.VELOCITY_MAX, target))
        
        # Check if this is a significant target change
        if abs(target - self.target_velocity) > 0.1:
            self.get_logger().info(f'🎯 New target from cmd_vel: {target:.2f} rad/s')
            
            # ALWAYS reset encoder tracking for ANY target change (including 0.0)
            self.target_changed = True
            self.target_change_time = time.time()
            
            # Reset encoder tracking to get fresh velocity reading
            self.last_left_encoder = None
            self.last_time = None
            self.current_velocity = 0.0  # Will be updated by next sensor reading
        
        self.target_velocity = target
        self.last_command_time = time.time()
        
    def velocity_to_pwm_feedforward(self, target_velocity):
        """
        Convert target velocity to PWM using direct lookup from characterization data
        This is the primary control - uses your actual measured motor response
        """
        if target_velocity <= 0.1:
            return 0.0
        
        # Clamp to measured range
        if target_velocity <= 1.89:
            return self.PWM_MIN  # Below minimum characterized velocity
        if target_velocity >= 6.96:
            return self.PWM_MAX  # At maximum characterized velocity
        
        # Linear interpolation between characterized points
        for i in range(len(self.characterization_points) - 1):
            pwm1, vel1 = self.characterization_points[i]
            pwm2, vel2 = self.characterization_points[i + 1]
            
            if vel1 <= target_velocity <= vel2:
                # Linear interpolation
                ratio = (target_velocity - vel1) / (vel2 - vel1)
                pwm = pwm1 + ratio * (pwm2 - pwm1)
                return pwm
        
        # Fallback to linear equation if outside characterized points
        return 40.0 + (target_velocity * 7.14)
    
    def control_loop(self):
        """Simple feedforward control with minor feedback correction"""
        if self.target_velocity == 0.0:
            # Stop the motor
            self.stop_motor()
            self.last_pwm_output = 0.0
            return
        
        # Primary control: Feedforward based on characterization
        feedforward_pwm = self.velocity_to_pwm_feedforward(self.target_velocity)
        
        # Minor feedback correction (much smaller than PID)
        velocity_error = self.target_velocity - self.current_velocity
        feedback_correction = velocity_error * self.feedback_gain
        
        # Combine feedforward + small feedback
        total_pwm = feedforward_pwm + feedback_correction
        
        # Clamp to safe range
        total_pwm = max(0.0, min(self.PWM_MAX, total_pwm))
        
        # Apply to motor
        self.set_left_motor_pwm(total_pwm)
        self.last_pwm_output = total_pwm
        
        # Publish monitoring data
        vel_msg = Float32()
        vel_msg.data = self.current_velocity
        self.velocity_pub.publish(vel_msg)
        
        error_msg = Float32()
        error_msg.data = velocity_error
        self.error_pub.publish(error_msg)
        
        pwm_msg = Float32()
        pwm_msg.data = total_pwm
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
                self.stop_motor()
                
    def print_status(self):
        """Print current status for monitoring"""
        if self.target_velocity > 0.1 or self.current_velocity > 0.1:
            error = self.target_velocity - self.current_velocity
            error_percent = (abs(error) / self.target_velocity * 100) if self.target_velocity > 0 else 0
            
            # Calculate expected PWM from characterization
            expected_pwm = self.velocity_to_pwm_feedforward(self.target_velocity)
            
            self.get_logger().info(
                f'📊 Target: {self.target_velocity:.2f} rad/s | '
                f'Current: {self.current_velocity:.2f} rad/s | '
                f'Error: {error:.2f} rad/s ({error_percent:.1f}%) | '
                f'PWM: {self.last_pwm_output:.1f}% (expected: {expected_pwm:.1f}%)'
            )
        
    def destroy_node(self):
        """Clean up GPIO"""
        self.stop_motor()
        self.pwm_left.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleVelocityController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nSimple controller stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()