import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from roomba_interfaces.msg import SensorData, WheelVelocities

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
        self.max_wheel_velocity = 6.9    # rad/s (stay below max tested)
        
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
        self.current_left_wheel_vel = 0.0
        self.current_right_wheel_vel = 0.0
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
        
        # UPDATED: Subscribe to direct wheel velocities (NO MORE CMD_VEL!)
        self.wheel_vel_sub = self.create_subscription(
            WheelVelocities, '/wheel_velocities', self.wheel_velocities_callback, 10)
        
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Safety check (keeping your existing logic)
        self.avoidance_timer = self.create_timer(0.1, self.check_obstacles)
        
        
        self.get_logger().info('🚀 Simple Velocity Controller Started')
        self.get_logger().info(f'📏 Wheel separation: {self.wheel_separation}m')
        self.get_logger().info(f'⚙️  Using linear motor mapping (no PID needed)')
       
     

    def wheel_velocities_callback(self, msg):

        self.last_cmd_time = time.time()
        
        # Apply wheel velocity limits for safety
        self.current_left_wheel_vel = max(-self.max_wheel_velocity, 
                                        min(self.max_wheel_velocity, msg.left_motor_velocity))
        self.current_right_wheel_vel = max(-self.max_wheel_velocity,
                                         min(self.max_wheel_velocity, msg.right_motor_velocity))
        
        self.get_logger().debug(f'📨 Direct wheel cmd: L={self.current_left_wheel_vel:.3f}, R={self.current_right_wheel_vel:.3f} rad/s')
        

        
    def control_loop(self):
        """Main control loop - converts cmd_vel to motor PWM"""
        current_time = time.time()
        
        # Check for command timeout
        # Check for command timeout
        if current_time - self.last_cmd_time > self.cmd_timeout:
            self.current_left_wheel_vel = 0.0
            self.current_right_wheel_vel = 0.0
            self.get_logger().debug('⏰ Command timeout - stopping')
            
            
        # Safety check - stop if avoiding obstacle
        if self.avoiding_obstacle:
            self.stop_motors()
            return
        
        self.get_logger().debug(f'🎯 Direct wheel velocities: L={self.current_left_wheel_vel:.2f}, R={self.current_right_wheel_vel:.2f} rad/s')
        
        # Convert to PWM using linear mapping (NO KINEMATICS CONVERSION!)
        left_pwm = self.velocity_to_pwm(self.current_left_wheel_vel, 'left')
        right_pwm = self.velocity_to_pwm(self.current_right_wheel_vel, 'right')
        
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
            # # Linear interpolation within characterized range
            # velocity_range = motor['max_velocity'] - motor['min_velocity']
            # pwm_range = motor['max_pwm'] - motor['min_pwm']
            
            # # Linear mapping: PWM = min_pwm + (velocity - min_velocity) * (pwm_range / velocity_range)
            # velocity_offset = abs_velocity - motor['min_velocity']
            # pwm_magnitude = motor['min_pwm'] + (velocity_offset * pwm_range / velocity_range)

            pwm_magnitude = 10*abs_velocity + 20
            
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
        
    def check_obstacles(self):
        """Obstacle avoidance (keeping your existing logic)"""
        if self.avoiding_obstacle:
            return
        if self.moving_forward and self.obstacle_distance <= self.SAFE_DISTANCE:
            self.get_logger().info(f'🚧 Obstacle detected at {self.obstacle_distance:.2f} cm! Avoiding...')
            self.avoiding_obstacle = True
            
            # Execute avoidance sequence
            self.stop_motors()
            

            self.create_timer(1.5, lambda: self.clear_avoidance_flag())
            
    def clear_avoidance_flag(self):
        """Reset avoidance flag"""
        self.avoiding_obstacle = False
        self.get_logger().info('✅ Obstacle avoidance completed')
        
    # Direct motor control methods (for obstacle avoidance)
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