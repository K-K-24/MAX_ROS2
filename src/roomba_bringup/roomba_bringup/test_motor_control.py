#!/usr/bin/env python3
# Save this as test_motor_control.py in your roomba_bringup/roomba_bringup directory

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import threading
import time

msg = """
ROOMBA MOTOR CONTROLLER TEST
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
s : stop
q/e : turn left/right
z/c : circle left/right

CTRL-C to quit
"""

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        
        # Create publisher for cmd_vel
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Velocity settings
        self.lin_vel_step = 0.1
        self.ang_vel_step = 0.1
        
        self.get_logger().info('Motor tester node started')
        self.get_logger().info(msg)
        
    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.angular.z = self.angular_vel
        self.vel_pub.publish(twist)
        self.get_logger().info(f'Published: linear={self.linear_vel:.2f}, angular={self.angular_vel:.2f}')

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = MotorTester()
    
    # Create a thread for processing ROS callbacks
    def spin_thread():
        rclpy.spin(node)
    
    thread = threading.Thread(target=spin_thread)
    thread.daemon = True
    thread.start()
    
    try:
        while True:
            key = getKey(settings)
            
            if key == '\x03': # CTRL-C
                break
                
            elif key == 'w':
                node.linear_vel += node.lin_vel_step
                node.publish_velocity()
                
            elif key == 'x':
                node.linear_vel -= node.lin_vel_step
                node.publish_velocity()
                
            elif key == 'a':
                node.angular_vel += node.ang_vel_step
                node.publish_velocity()
                
            elif key == 'd':
                node.angular_vel -= node.ang_vel_step
                node.publish_velocity()
                
            elif key == 's':
                node.linear_vel = 0.0
                node.angular_vel = 0.0
                node.publish_velocity()
                
            elif key == 'q':
                # Turn left in place
                node.linear_vel = 0.0
                node.angular_vel = 0.5
                node.publish_velocity()
                
            elif key == 'e':
                # Turn right in place
                node.linear_vel = 0.0
                node.angular_vel = -0.5
                node.publish_velocity()
                
            elif key == 'z':
                # Circle left
                node.linear_vel = 0.5
                node.angular_vel = 0.5
                node.publish_velocity()
                
            elif key == 'c':
                # Circle right
                node.linear_vel = 0.5
                node.angular_vel = -0.5
                node.publish_velocity()
                
            time.sleep(0.1)  # Small delay to avoid flooding
            
    except Exception as e:
        print(e)
        
    finally:
        # Stop the robot before exiting
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.vel_pub.publish(twist)
        
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()