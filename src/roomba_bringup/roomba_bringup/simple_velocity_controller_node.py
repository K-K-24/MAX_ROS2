import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from roomba_interfaces.msg import SensorData

class SimpleVelocityController(Node):
    """
    Simple velocity controller based on motor characterization data.
    Uses linear mapping instead of PID since your motors show excellent linearity.
    
    Motor Characterization Results:
    - Left Motor: 40-90% PWM → 1.89-6.96 rad/s (~0.1 rad/s per 1% PWM)
    - Right Motor: 40-90% PWM → 1.69-7.02 rad/s (~0.11 rad/s per 1% PWM)
    - Dead Zone: <40% PWM produces no movement
    - Avoid 100% PWM due to instability
    """
    
    def __init__(self):
        super().__init__('simple_velocity_controller')
        
        # Robot Physical Parameters
        self.wheel_radius = 0.03  # meters (3cm)
        self.wheel_separation = 0.148  # meters (14.8cm between wheels)
        
        # Motor Characterization Data (from your findings)
        # Linear relationships in usable range (40-90% PWM)
        self.left_motor = {
            'min_pwm': 40,           # Dead zone threshold
            'max_pwm': 90,           # Avoid 100% due to instability
            'min_velocity': 1.89,    # rad/s at 40% PWM
            'max_velocity': 6.96,    # rad/s at 90% PWM
            'pwm_per_rad_s': 50/5.07 # (90-40)/(6.96-1.89) ≈ 9.86 PWM per rad/s
        }
        
        self.right_motor = {
            'min_pwm': 40,
            'max_pwm': 90,
            'min_velocity': 1.69,    # rad/s at 40% PWM  
            'max_velocity': 7.02,    # rad/s at 90% PWM
            'pwm_per_rad_s': 50/5.33 # (90-40)/(7.02-1.69) ≈ 9.38 PWM per rad/s
        }
        
        # Safety limits
        self.max_linear_velocity = 0.5   # m/s
        self.max_angular_velocity = 2.0  # rad/s
        self.max_wheel_velocity = 6.5    # rad/s (stay below max tested)
        
        # Initialize GPIO (same as your motor_driver.py)
        self.IN1, self.IN2, self.IN3, self.IN4 = 17, 18, 22, 23
        self.ENA, self.ENB = 25, 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)
        
        self.pwm_right = GPIO.PWM(self.ENA, 100)
        self.pwm_left = GPIO.PWM(self.ENB, 100)
        self.pwm_right.start(0)
        self.pwm_left.start(0)
        
        # Control state
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.last_cmd_time = time.time()
        self.cmd_timeout = 1.0  # Stop if no commands for 1 second
        
        # Safety features (keeping your existing obstacle avoidance)
        self.moving_forward = False
        self.moving_backward = False
        self.obstacle_distance = 100.0
        self.SAFE_DISTANCE = 25.0
        self.avoiding_obstacle = False
        
        # Subscribers (same as your original)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sensor_sub = self.create_subscription(
            SensorData, '/wheel_states', self.sensor_callback, 10)
            
        # Control loop - run at 4Hz (2x sensor rate for responsiveness)
        self.control_timer = self.create_timer(0.25, self.control_loop)
        
        # Safety check (keeping your existing logic)
        self.avoidance_timer = self.create_timer(0.5, self.check_obstacles)
        
        # For velocity feedback (optional enhancement)
        self.last_left_encoder = None
        self.last_right_encoder = None
        self.last_encoder_time = None
        self.actual_left_velocity = 0.0
        self.actual_right_velocity = 0.0
        
        self.get_logger().info('🚀 Simple Velocity Controller Started')
        self.get_logger().info(f'📏 Wheel separation: {self.wheel_separation}m')
        self.get_logger().info(f'⚙️  Using linear motor mapping (no PID needed)')
        self.get_logger().info(f'🔒 Safety limits: {self.max_linear_velocity}m/s linear, {self.max_angular_velocity}rad/s angular')
        self.get_logger().info(f'🕐 Control frequency: 4Hz (sensor rate: 2Hz)')
        
    def cmd_vel_callback(self, msg):
        """Process velocity commands with proper limits"""
        self.last_cmd_time = time.time()
        
        # Apply velocity limits
        self.current_linear_vel = max(-self.max_linear_velocity, 
                                    min(self.max_linear_velocity, msg.linear.x))
        self.current_angular_vel = max(-self.max_angular_velocity,
                                     min(self.max_angular_velocity, msg.angular.z))
        
        self.get_logger().debug(f'📨 Cmd received: lin={self.current_linear_vel:.3f}, ang={self.current_angular_vel:.3f}')
        
    def control_loop(self):
        """Main control loop - converts cmd_vel to motor PWM"""
        current_time = time.time()
        
        # Check for command timeout
        if current_time - self.last_cmd_time > self.cmd_timeout:
            self.current_linear_vel = 0.0
            self.current_angular_vel = 0.0
            self.get_logger().debug('⏰ Command timeout - stopping')
            
        # Safety check - stop if avoiding obstacle
        if self.avoiding_obstacle:
            self.stop_motors()
            return
            
        # Convert cmd_vel to individual wheel velocities (differential drive kinematics)
        left_wheel_vel = (self.current_linear_vel - 
                         self.current_angular_vel * self.wheel_separation / 2) / self.wheel_radius
        right_wheel_vel = (self.current_linear_vel + 
                          self.current_angular_vel * self.wheel_separation / 2) / self.wheel_radius
        
        # Apply wheel velocity limits
        left_wheel_vel = max(-self.max_wheel_velocity, min(self.max_wheel_velocity, left_wheel_vel))
        right_wheel_vel = max(-self.max_wheel_velocity, min(self.max_wheel_velocity, right_wheel_vel))
        
        self.get_logger().debug(f'🎯 Target wheel velocities: L={left_wheel_vel:.2f}, R={right_wheel_vel:.2f} rad/s')
        
        # Convert to PWM using linear mapping
        left_pwm = self.velocity_to_pwm(left_wheel_vel, 'left')
        right_pwm = self.velocity_to_pwm(right_wheel_vel, 'right')
        
        # Apply motor commands
        self.set_motor_speeds(left_pwm, right_pwm)
        
    def velocity_to_pwm(self, wheel_velocity_rad_s, motor_side):
        """
        Convert wheel velocity to PWM using linear mapping from characterization data.
        This replaces PID control with direct linear mapping.
        """
        if abs(wheel_velocity_rad_s) < 0.05:  # Small dead zone for near-zero velocities
            return 0
            
        # Select motor parameters
        motor = self.left_motor if motor_side == 'left' else self.right_motor
        
        # Get absolute velocity and direction
        abs_velocity = abs(wheel_velocity_rad_s)
        direction = 1 if wheel_velocity_rad_s >= 0 else -1
        
        # Check if velocity is achievable
        if abs_velocity < motor['min_velocity']:
            # For very low velocities, use minimum PWM (might not move, but that's expected)
            pwm_magnitude = motor['min_pwm']
        elif abs_velocity > motor['max_velocity']:
            # Clamp to maximum safe velocity
            pwm_magnitude = motor['max_pwm']
            self.get_logger().warn(f'⚠️  {motor_side} velocity {abs_velocity:.2f} clamped to {motor["max_velocity"]:.2f} rad/s')
        else:
            # Linear interpolation within characterized range
            velocity_range = motor['max_velocity'] - motor['min_velocity']
            pwm_range = motor['max_pwm'] - motor['min_pwm']
            
            # Linear mapping: PWM = min_pwm + (velocity - min_velocity) * (pwm_range / velocity_range)
            velocity_offset = abs_velocity - motor['min_velocity']
            pwm_magnitude = motor['min_pwm'] + (velocity_offset * pwm_range / velocity_range)
            
            # Ensure within bounds
            pwm_magnitude = max(motor['min_pwm'], min(motor['max_pwm'], pwm_magnitude))
        
        return pwm_magnitude * direction
        
    def set_motor_speeds(self, left_pwm, right_pwm):
        """Set motor speeds with proper direction control"""
        # Update movement state for obstacle avoidance
        if left_pwm > 0 and right_pwm > 0:
            self.moving_forward = True
            self.moving_backward = False
        elif left_pwm < 0 and right_pwm < 0:
            self.moving_forward = False
            self.moving_backward = True
        else:
            self.moving_forward = False
            self.moving_backward = False
            
        # Left motor control
        if left_pwm > 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
            self.pwm_left.ChangeDutyCycle(abs(left_pwm))
        elif left_pwm < 0:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
            self.pwm_left.ChangeDutyCycle(abs(left_pwm))
        else:
            self.pwm_left.ChangeDutyCycle(0)
            
        # Right motor control
        if right_pwm > 0:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
            self.pwm_right.ChangeDutyCycle(abs(right_pwm))
        elif right_pwm < 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
            self.pwm_right.ChangeDutyCycle(abs(right_pwm))
        else:
            self.pwm_right.ChangeDutyCycle(0)
            
        self.get_logger().debug(f'⚡ PWM output: L={left_pwm:.1f}%, R={right_pwm:.1f}%')
        
    def sensor_callback(self, msg):
        """Process sensor data - obstacle avoidance and velocity feedback"""
        self.obstacle_distance = msg.ultrasonic_distance
        
        # Calculate actual wheel velocities for monitoring (optional)
        current_time = time.time()
        if self.last_left_encoder is not None and self.last_encoder_time is not None:
            dt = current_time - self.last_encoder_time
            if dt > 0:
                left_delta = msg.left_encoder - self.last_left_encoder
                right_delta = msg.right_encoder - self.last_right_encoder
                
                # Convert to angular velocities
                self.actual_left_velocity = (left_delta / 100) / self.wheel_radius / dt  # cm to m
                self.actual_right_velocity = (right_delta / 100) / self.wheel_radius / dt
                
        self.last_left_encoder = msg.left_encoder
        self.last_right_encoder = msg.right_encoder
        self.last_encoder_time = current_time
        
    def check_obstacles(self):
        """Obstacle avoidance (keeping your existing logic)"""
        if self.avoiding_obstacle:
            return
        if self.moving_forward and self.obstacle_distance <= self.SAFE_DISTANCE:
            self.get_logger().info(f'🚧 Obstacle detected at {self.obstacle_distance:.2f} cm! Avoiding...')
            self.avoiding_obstacle = True
            
            # Execute avoidance sequence
            self.stop_motors()
            
            # Create sequential timers for avoidance maneuver
            self.create_timer(0.1, lambda: self.move_backward(50))
            self.create_timer(0.6, lambda: self.stop_motors())
            self.create_timer(0.8, lambda: self.turn_left(50))
            self.create_timer(1.3, lambda: self.stop_motors())
            self.create_timer(1.5, lambda: self.clear_avoidance_flag())
            
    def clear_avoidance_flag(self):
        """Reset avoidance flag"""
        self.avoiding_obstacle = False
        self.get_logger().info('✅ Obstacle avoidance completed')
        
    # Direct motor control methods (for obstacle avoidance)
    def move_backward(self, speed):
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_right.ChangeDutyCycle(speed)
        self.pwm_left.ChangeDutyCycle(speed)
        
    def turn_left(self, speed):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_right.ChangeDutyCycle(speed)
        self.pwm_left.ChangeDutyCycle(speed)
        
    def stop_motors(self):
        """Stop all motors immediately"""
        self.pwm_right.ChangeDutyCycle(0)
        self.pwm_left.ChangeDutyCycle(0)
        self.moving_forward = False
        self.moving_backward = False
        
    def destroy_node(self):
        """Clean shutdown"""
        self.stop_motors()
        self.pwm_right.stop()
        self.pwm_left.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleVelocityController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 Shutting down velocity controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()