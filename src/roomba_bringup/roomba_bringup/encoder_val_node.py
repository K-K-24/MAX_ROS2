#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from roomba_interfaces.msg import SensorData

class EncoderValidationNode(Node):
    def __init__(self):
        super().__init__('encoder_validation_node')
        
        # Subscribe to sensor data
        self.sensor_sub = self.create_subscription(
            SensorData,
            '/wheel_states',
            self.sensor_callback,
            10)
        
        # Data tracking
        self.last_left_encoder = None
        self.last_right_encoder = None
        self.last_time = None
        
        # For data analysis
        self.readings_count = 0
        self.left_deltas = []
        self.right_deltas = []
        self.time_deltas = []
        
        self.get_logger().info('🔍 ENCODER VALIDATION NODE STARTED')
        self.get_logger().info('This will analyze your encoder data quality')
        self.get_logger().info('Try manually moving your robot wheels and observe the output')
        
    def sensor_callback(self, msg):
        current_time = time.time()
        self.readings_count += 1
        
        # Log raw readings every 5th message to avoid spam
        if self.readings_count % 5 == 0:
            self.get_logger().info(f'📊 Raw encoder readings: Left={msg.left_encoder:.1f} cm, Right={msg.right_encoder:.1f} cm')
        
        # Calculate deltas if we have previous readings
        if self.last_left_encoder is not None:
            dt = current_time - self.last_time
            left_delta = msg.left_encoder - self.last_left_encoder
            right_delta = msg.right_encoder - self.last_right_encoder
            
            # Store for analysis
            self.left_deltas.append(left_delta)
            self.right_deltas.append(right_delta)
            self.time_deltas.append(dt)
            
            # Log changes (only if there's movement)
            if abs(left_delta) > 0.1 or abs(right_delta) > 0.1:
                self.get_logger().info('🔄 MOVEMENT DETECTED:')
                self.get_logger().info(f'   Time delta: {dt:.3f} seconds')
                self.get_logger().info(f'   Left change: {left_delta:.2f} cm')
                self.get_logger().info(f'   Right change: {right_delta:.2f} cm')
                
                # Calculate simple velocities
                if dt > 0:
                    left_vel_cm_s = left_delta / dt
                    right_vel_cm_s = right_delta / dt
                    self.get_logger().info(f'   Left velocity: {left_vel_cm_s:.2f} cm/s')
                    self.get_logger().info(f'   Right velocity: {right_vel_cm_s:.2f} cm/s')
                    
                    # Convert to angular velocity (wheel radius = 3cm)
                    wheel_radius = 3.0
                    left_vel_rad_s = left_vel_cm_s / wheel_radius
                    right_vel_rad_s = right_vel_cm_s / wheel_radius
                    self.get_logger().info(f'   Left angular: {left_vel_rad_s:.2f} rad/s')
                    self.get_logger().info(f'   Right angular: {right_vel_rad_s:.2f} rad/s')
                    
                    # Sanity checks
                    if abs(left_vel_rad_s) > 10:
                        self.get_logger().warn('⚠️  Left velocity seems too high!')
                    if abs(right_vel_rad_s) > 10:
                        self.get_logger().warn('⚠️  Right velocity seems too high!')
                        
                    if dt > 0.1:
                        self.get_logger().warn('⚠️  Large time gap between readings!')
                        
                self.get_logger().info('-' * 40)
        
        # Update for next iteration
        self.last_left_encoder = msg.left_encoder
        self.last_right_encoder = msg.right_encoder
        self.last_time = current_time
        
        # Periodic summary
        if self.readings_count % 100 == 0:
            self.print_summary()
    
    def print_summary(self):
        """Print a summary of data quality"""
        if len(self.time_deltas) > 10:
            avg_dt = sum(self.time_deltas[-50:]) / len(self.time_deltas[-50:])  # Last 50 readings
            avg_left_delta = sum([abs(d) for d in self.left_deltas[-50:]]) / len(self.left_deltas[-50:])
            avg_right_delta = sum([abs(d) for d in self.right_deltas[-50:]]) / len(self.right_deltas[-50:])
            
            self.get_logger().info('📈 DATA QUALITY SUMMARY (last 50 readings):')
            self.get_logger().info(f'   Average time between readings: {avg_dt:.3f} seconds')
            self.get_logger().info(f'   Average left movement: {avg_left_delta:.2f} cm')
            self.get_logger().info(f'   Average right movement: {avg_right_delta:.2f} cm')
            self.get_logger().info(f'   Update rate: {1/avg_dt:.1f} Hz')
            
            # Data quality checks
            if avg_dt > 0.06:
                self.get_logger().warn('⚠️  Update rate slower than expected (should be ~20Hz)')
            if avg_dt < 0.04:
                self.get_logger().warn('⚠️  Update rate faster than expected')

def main(args=None):
    rclpy.init(args=args)
    validator = EncoderValidationNode()
    rclpy.spin(validator)
    validator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()