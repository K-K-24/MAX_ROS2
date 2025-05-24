import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from roomba_interfaces.msg import SensorData

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        # Initialize GPIO
        self.IN1, self.IN2, self.IN3, self.IN4 = 17, 18, 22, 23
        self.ENA, self.ENB = 25, 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)
        
        self.pwm_right = GPIO.PWM(self.ENA, 100)
        self.pwm_left = GPIO.PWM(self.ENB, 100)
        self.pwm_right.start(0)
        self.pwm_left.start(0)
        
        
        # Subscribe to command velocity
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            

            
        # Subscribe to sensor data for obstacle avoidance
        self.sensor_sub = self.create_subscription(
            SensorData,
            '/wheel_states',
            self.sensor_callback,
            10)
            
        # State variables
        self.moving_forward = False
        self.moving_backward = False
        self.obstacle_distance = 100.0  # Set to a large value initially
        self.SAFE_DISTANCE = 25.0

        #Avoidance state tracking
        self.avoiding_obstacle = False
        
        # Set up a timer for the obstacle avoidance check
        self.avoidance_timer = self.create_timer(0.5, self.check_obstacles)
        
        self.get_logger().info('Motor driver node started')
        
    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Simple differential drive model
        left_speed = int(abs((linear_vel - angular_vel) * 100))
        right_speed = int(abs((linear_vel + angular_vel) * 100))
        
        # Clamp values
        left_speed = min(max(left_speed, 0), 100)
        right_speed = min(max(right_speed, 0), 100)
        
        if linear_vel > 0:  # Moving forward
            if angular_vel > 0:  # Turning left
                self.turn_left(right_speed)
            elif angular_vel < 0:  # Turning right
                self.turn_right(left_speed)
            else:  # Straight ahead
                self.move_forward(max(left_speed, right_speed))
        elif linear_vel < 0:  # Moving backward
            if angular_vel > 0:  # Turning left in reverse
                self.move_backward(left_speed)
            elif angular_vel < 0:  # Turning right in reverse
                self.move_backward(right_speed)
            else:  # Straight backward
                self.move_backward(max(left_speed, right_speed))
        elif angular_vel > 0:  # Turning in place left
            self.turn_left(max(left_speed, right_speed))
        elif angular_vel < 0:  # Turning in place right
            self.turn_right(max(left_speed, right_speed))
        else:  # Stop
            self.stop_motors()
            
    def sensor_callback(self, msg):
        self.obstacle_distance = msg.ultrasonic_distance
            
    def check_obstacles(self):
        if ( self.avoiding_obstacle):
            return
        if self.moving_forward and self.obstacle_distance <= self.SAFE_DISTANCE:
            self.get_logger().info(f'Obstacle detected at {self.obstacle_distance:.2f} cm! Avoiding...')
            # Set avoidance flag
            self.avoiding_obstacle = True
            
            # Execute avoidance sequence
            self.stop_motors()
            self.move_backward(90)
            
            # Create sequential timers
            self.create_timer(0.5, lambda: self.stop_motors())
            self.create_timer(0.7, lambda: self.turn_left(90))
            self.create_timer(1.2, lambda: self.stop_motors())
            
            # Clear avoidance flag after maneuver completes
            self.create_timer(1.5, lambda: self.clear_avoidance_flag())

    def clear_avoidance_flag(self):
        """Reset avoidance flag to allow normal obstacle detection"""
        self.avoiding_obstacle = False
        self.get_logger().info('Obstacle avoidance completed, resuming normal operation')
            

        
    def move_forward(self, speed=90):
        self.get_logger().debug(f'Moving forward at speed {speed}')
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_right.ChangeDutyCycle(speed)
        self.pwm_left.ChangeDutyCycle(speed)
        self.moving_forward = True
        self.moving_backward = False
        
    def move_backward(self, speed=50):
        self.get_logger().debug(f'Moving backward at speed {speed}')
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_right.ChangeDutyCycle(speed)
        self.pwm_left.ChangeDutyCycle(speed)
        self.moving_forward = False
        self.moving_backward = True
        
    def turn_left(self, speed=50):
        self.get_logger().debug(f'Turning left at speed {speed}')
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_right.ChangeDutyCycle(speed)
        self.pwm_left.ChangeDutyCycle(speed)
        self.moving_forward = False
        self.moving_backward = False
        
    def turn_right(self, speed=50):
        self.get_logger().debug(f'Turning right at speed {speed}')
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_right.ChangeDutyCycle(speed)
        self.pwm_left.ChangeDutyCycle(speed)
        self.moving_forward = False
        self.moving_backward = False
        
    def stop_motors(self):
        self.get_logger().debug('Stopping motors')
        self.pwm_right.ChangeDutyCycle(0)
        self.pwm_left.ChangeDutyCycle(0)
        self.moving_forward = False
        self.moving_backward = False
        
            
    def destroy_node(self):
        self.pwm_right.stop()
        self.pwm_left.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_driver = MotorDriverNode()
    rclpy.spin(motor_driver)
    motor_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()