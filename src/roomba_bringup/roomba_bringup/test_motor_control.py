#!/usr/bin/env python3
# Updated test_motor_control.py for ros2_control system

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import sys
import termios
import tty
import threading
import time

msg = """
🏎️ ROOMBA MOTOR CONTROLLER TEST (ACTUALLY UNLEASHED!) 🏎️
---------------------------------------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity
s : emergency stop
q/e : turn left/right in place  
z/c : circle left/right

🚀 SPECIAL COMMANDS:
l : LUDICROUS MODE toggle (FOR REAL THIS TIME!)
t : toggle ros2_control/direct mode
h : show this help

CTRL-C to quit

🎯 Normal mode: 30 cm/s linear, 2.5 rad/s angular
🚀 Ludicrous mode: 60 cm/s linear, 4.0 rad/s angular (FASTER than before!)
⚠️  Now we're talking! Your Lamborghini is properly unleashed! 🏁💨
"""

class MotorTester(Node):
    def __init__(self):
        super().__init__('motor_tester')
        
        # Create publisher for ros2_control (TwistStamped)
        self.vel_pub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        
        # Also keep the old publisher for direct control (optional)
        self.direct_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Velocity settings - PROPERLY UNLEASHED! 🚀
        self.lin_vel_step = 0.02  # Small steps for precise control
        self.ang_vel_step = 0.1
        
        # Speed limits based on your ACTUAL motor capabilities
        self.max_linear_vel = 0.21    # 30 cm/s (reasonable daily driving)
        self.max_angular_vel = 2.8   # 2.5 rad/s (good turning speed)
        
        # LUDICROUS mode limits - NOW WE'RE TALKING! 🏎️💨
        self.ludicrous_linear = 0.6    # 60 cm/s - FASTER than your old 50 cm/s!
        self.ludicrous_angular = 4.0   # 4.0 rad/s - SPINNING MADNESS!
        self.ludicrous_mode = False
        
        # Control mode
        self.use_ros2_control = True  # Toggle between ros2_control and direct control
        
        self.get_logger().info('🎮 Motor tester node started')
        self.get_logger().info('📡 Publishing to ros2_control differential drive controller')
        self.get_logger().info(msg)
        
    def publish_velocity(self):
        if self.use_ros2_control:
            # Create TwistStamped message for ros2_control
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = ''
            twist_stamped.twist.linear.x = self.linear_vel
            twist_stamped.twist.angular.z = self.angular_vel
            
            self.vel_pub.publish(twist_stamped)
            self.get_logger().info(f'🚀 ros2_control: linear={self.linear_vel:.2f}, angular={self.angular_vel:.2f}')
        else:
            # Direct control (bypass ros2_control)
            twist = Twist()
            twist.linear.x = self.linear_vel
            twist.angular.z = self.angular_vel
            
            self.direct_vel_pub.publish(twist)
            self.get_logger().info(f'🔄 Direct: linear={self.linear_vel:.2f}, angular={self.angular_vel:.2f}')
    
    def toggle_control_mode(self):
        """Toggle between ros2_control and direct control"""
        self.use_ros2_control = not self.use_ros2_control
        mode = "ros2_control" if self.use_ros2_control else "direct"
        self.get_logger().info(f'🔄 Switched to {mode} mode')
        
    def emergency_stop(self):
        """Send stop command to both controllers"""
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.publish_velocity()
        
        # Also send direct stop for safety
        twist = Twist()
        self.direct_vel_pub.publish(twist)

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
                max_speed = node.ludicrous_linear if node.ludicrous_mode else node.max_linear_vel
                node.linear_vel = min(node.linear_vel, max_speed)
                node.publish_velocity()
                
            elif key == 'x':
                node.linear_vel -= node.lin_vel_step
                max_speed = node.ludicrous_linear if node.ludicrous_mode else node.max_linear_vel
                node.linear_vel = max(node.linear_vel, -max_speed)
                node.publish_velocity()
                
            elif key == 'a':
                node.angular_vel += node.ang_vel_step
                max_angular = node.ludicrous_angular if node.ludicrous_mode else node.max_angular_vel
                node.angular_vel = min(node.angular_vel, max_angular)
                node.publish_velocity()
                
            elif key == 'd':
                node.angular_vel -= node.ang_vel_step
                max_angular = node.ludicrous_angular if node.ludicrous_mode else node.max_angular_vel
                node.angular_vel = max(node.angular_vel, -max_angular)
                node.publish_velocity()
                
            elif key == 's':
                node.emergency_stop()
                
            elif key == 'q':
                # Turn left in place
                node.linear_vel = 0.0
                node.angular_vel = 1.5 if not node.ludicrous_mode else 2.5
                node.publish_velocity()
                
            elif key == 'e':
                # Turn right in place
                node.linear_vel = 0.0
                node.angular_vel = -1.5 if not node.ludicrous_mode else -2.5
                node.publish_velocity()
                
            elif key == 'z':
                # Circle left
                node.linear_vel = 0.2 if not node.ludicrous_mode else 0.4
                node.angular_vel = 1.0 if not node.ludicrous_mode else 2.0
                node.publish_velocity()
                
            elif key == 'c':
                # Circle right
                node.linear_vel = 0.2 if not node.ludicrous_mode else 0.4
                node.angular_vel = -1.0 if not node.ludicrous_mode else -2.0
                node.publish_velocity()
                
            elif key == 'l':
                # LUDICROUS MODE TOGGLE! 🚀💨
                node.ludicrous_mode = not node.ludicrous_mode
                mode = "🚀💨🔥 LUDICROUS MODE ENGAGED! 🔥💨🚀" if node.ludicrous_mode else "Normal mode (safety on)"
                node.get_logger().info(f'🏎️ {mode}')
                if node.ludicrous_mode:
                    node.get_logger().info('⚠️ WARNING: 60 cm/s MAXIMUM SPEED UNLOCKED! 🏁💨')
                    node.get_logger().info('🌪️ SPINNING AT 4.0 rad/s ENABLED! Hold on tight! 🌪️')
                
            elif key == 't':
                # Toggle control mode
                node.toggle_control_mode()
                
            elif key == 'h':
                # Show help
                node.get_logger().info(msg)
                
            time.sleep(0.1)  # Small delay to avoid flooding
            
    except Exception as e:
        print(e)
        
    finally:
        # Emergency stop before exiting
        node.emergency_stop()
        
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        # Clean up
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()