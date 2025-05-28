#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import RPi.GPIO as GPIO
from roomba_interfaces.msg import SensorData
import json
import threading

class MotorCharacterizationNode(Node):
    """
    Simplified motor characterization with fixed timer management
    """
    
    def __init__(self):
        super().__init__('motor_characterization')
        
        # Initialize GPIO exactly like your motor_driver.py
        self.IN1, self.IN2, self.IN3, self.IN4 = 17, 18, 22, 23
        self.ENA, self.ENB = 25, 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)
        
        # Create PWM objects
        self.pwm_right = GPIO.PWM(self.ENA, 100)
        self.pwm_left = GPIO.PWM(self.ENB, 100)
        self.pwm_right.start(0)
        self.pwm_left.start(0)
        
        # Subscribe to encoder data
        self.sensor_sub = self.create_subscription(
            SensorData,
            '/wheel_states',
            self.sensor_callback,
            10)
        
        # Velocity calculation variables
        self.last_left_encoder = None
        self.last_right_encoder = None
        self.last_time = None
        self.current_left_velocity = 0.0
        self.current_right_velocity = 0.0
        
        # Test configuration
        self.test_pwm_values = [0, 30, 40, 50, 60, 70, 80, 90, 100]
        self.test_duration = 4.0
        self.settle_time = 1.5
        
        # Results storage
        self.characterization_results = {
            'left_motor': [],
            'right_motor': [],
            'test_config': {
                'pwm_values': self.test_pwm_values,
                'test_duration': self.test_duration,
                'settle_time': self.settle_time
            }
        }
        
        # Simple state management - no complex threading
        self.test_active = False
        self.current_test_motor = None
        self.current_pwm_index = 0
        self.test_start_time = None
        self.velocity_samples = []
        self.characterization_started = False
        
        # Use a single main timer for all control
        self.main_timer = self.create_timer(0.5, self.main_loop)
        self.next_action_time = time.time() + 3.0  # Start in 3 seconds
        self.current_state = 'WAITING_TO_START'
        
        self.get_logger().info('🔧 Motor Characterization Node Started')
        self.get_logger().info('Using simplified timer management')
        self.get_logger().info('⚠️  Make sure your robot has clear space to move!')
        self.get_logger().info('Starting characterization in 3 seconds...')
        
    def sensor_callback(self, msg):
        """Process encoder data and calculate velocities"""
        current_time = time.time()
        
        if self.last_left_encoder is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                # Calculate distance changes
                left_delta = msg.left_encoder - self.last_left_encoder
                right_delta = msg.right_encoder - self.last_right_encoder
                
                # Convert to velocities
                wheel_radius = 3.0  # cm
                self.current_left_velocity = (left_delta / wheel_radius) / dt
                self.current_right_velocity = (right_delta / wheel_radius) / dt
                
                # Collect samples during active test
                if self.test_active and self.test_start_time is not None:
                    time_since_start = current_time - self.test_start_time
                    
                    if time_since_start > self.settle_time:
                        if self.current_test_motor == 'left':
                            self.velocity_samples.append(abs(self.current_left_velocity))
                        elif self.current_test_motor == 'right':
                            self.velocity_samples.append(abs(self.current_right_velocity))
        
        self.last_left_encoder = msg.left_encoder
        self.last_right_encoder = msg.right_encoder
        self.last_time = current_time
        
    def main_loop(self):
        """Main state machine - runs every 0.5 seconds"""
        current_time = time.time()
        
        if current_time < self.next_action_time:
            return  # Not time for next action yet
            
        if self.current_state == 'WAITING_TO_START':
            self.start_characterization()
            
        elif self.current_state == 'RUNNING_TEST':
            # Check if current test should end
            if current_time >= self.next_action_time:
                self.finish_current_test()
                
        elif self.current_state == 'BETWEEN_TESTS':
            # Start next test
            self.run_next_test()
            
        elif self.current_state == 'COMPLETE':
            # Stop the main timer
            self.main_timer.cancel()
            
    def start_characterization(self):
        """Start the characterization process"""
        if self.characterization_started:
            return  # Prevent multiple starts
            
        self.get_logger().info('🚀 Starting Motor Characterization!')
        self.get_logger().info('Testing LEFT motor first...')
        
        self.characterization_started = True
        self.current_test_motor = 'left'
        self.current_pwm_index = 0
        self.current_state = 'BETWEEN_TESTS'
        self.next_action_time = time.time()  # Start immediately
        
    def run_next_test(self):
        """Start the next test in sequence"""
        # Check if we've finished all PWM values for current motor
        if self.current_pwm_index >= len(self.test_pwm_values):
            if self.current_test_motor == 'left':
                self.get_logger().info('✅ Left motor characterization complete!')
                self.get_logger().info('🔄 Testing RIGHT motor...')
                self.current_test_motor = 'right'
                self.current_pwm_index = 0
            else:
                # All tests complete
                self.finish_characterization()
                return
        
        # Start the next test
        if self.current_pwm_index < len(self.test_pwm_values):
            pwm_value = self.test_pwm_values[self.current_pwm_index]
            
            self.get_logger().info(f'🔬 Testing {self.current_test_motor} motor at PWM {pwm_value}%')
            
            # Reset for this test
            self.velocity_samples = []
            self.test_start_time = time.time()
            self.test_active = True
            
            # Start the motor
            if self.current_test_motor == 'left':
                self.move_left_wheel_forward(pwm_value)
            else:
                self.move_right_wheel_forward(pwm_value)
            
            # Set when this test should end
            self.next_action_time = time.time() + self.test_duration
            self.current_state = 'RUNNING_TEST'
            
    def finish_current_test(self):
        """Complete current test and store results"""
        self.test_active = False
        self.stop_motors()
        
        # Analyze results
        if len(self.velocity_samples) > 0:
            avg_velocity = sum(self.velocity_samples) / len(self.velocity_samples)
            max_velocity = max(self.velocity_samples)
            min_velocity = min(self.velocity_samples)
        else:
            avg_velocity = max_velocity = min_velocity = 0.0
        
        # Store results
        pwm_value = self.test_pwm_values[self.current_pwm_index]
        result = {
            'pwm': pwm_value,
            'avg_velocity': avg_velocity,
            'max_velocity': max_velocity,
            'min_velocity': min_velocity,
            'sample_count': len(self.velocity_samples)
        }
        
        if self.current_test_motor == 'left':
            self.characterization_results['left_motor'].append(result)
        else:
            self.characterization_results['right_motor'].append(result)
        
        self.get_logger().info(f'   📊 Average velocity: {avg_velocity:.2f} rad/s')
        self.get_logger().info(f'   📈 Velocity range: {min_velocity:.2f} to {max_velocity:.2f} rad/s')
        self.get_logger().info(f'   🔢 Samples: {len(self.velocity_samples)}')
        self.get_logger().info(f'   🔍 Current readings: L={self.current_left_velocity:.2f}, R={self.current_right_velocity:.2f} rad/s')
        
        # Move to next test
        self.current_pwm_index += 1
        self.next_action_time = time.time() + 2.0  # 2 second pause
        self.current_state = 'BETWEEN_TESTS'
        
    def finish_characterization(self):
        """Complete the entire characterization"""
        self.test_active = False
        self.stop_motors()
        self.current_state = 'COMPLETE'
        
        self.get_logger().info('🎉 Motor Characterization Complete!')
        
        # Save and summarize
        self.save_results()
        self.print_summary()
        
        self.get_logger().info('✅ Data saved to motor_characterization.json')
        
    # Motor control methods using exact logic from your motor_driver.py
    def move_left_wheel_forward(self, speed):
        """Move left wheel forward"""
        self.get_logger().debug(f'Left wheel forward at {speed}% PWM')
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_left.ChangeDutyCycle(speed)
        
    def move_right_wheel_forward(self, speed):
        """Move right wheel forward"""
        self.get_logger().debug(f'Right wheel forward at {speed}% PWM')
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        self.pwm_right.ChangeDutyCycle(speed)
        
    def stop_motors(self):
        """Stop all motors"""
        self.get_logger().debug('Stopping motors')
        self.pwm_right.ChangeDutyCycle(0)
        self.pwm_left.ChangeDutyCycle(0)
        
    def save_results(self):
        """Save results to JSON file"""
        try:
            with open('motor_characterization.json', 'w') as f:
                json.dump(self.characterization_results, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to save results: {e}')
            
    def print_summary(self):
        """Print characterization summary"""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('MOTOR CHARACTERIZATION SUMMARY')
        self.get_logger().info('='*50)
        
        for motor_name in ['left_motor', 'right_motor']:
            self.get_logger().info(f'\n{motor_name.upper()} RESULTS:')
            results = self.characterization_results[motor_name]
            
            if not results:
                self.get_logger().info('  No data collected!')
                continue
                
            # Find dead zone
            dead_zone = None
            for result in results:
                if result['avg_velocity'] > 0.1:
                    dead_zone = result['pwm']
                    break
            
            if dead_zone is not None:
                self.get_logger().info(f'  Dead Zone: PWM < {dead_zone}% produces no movement')
            else:
                self.get_logger().info('  ⚠️  No movement detected!')
            
            # Show all results
            if results:
                max_result = max(results, key=lambda x: x['avg_velocity'])
                self.get_logger().info(f'  Max Velocity: {max_result["avg_velocity"]:.2f} rad/s at {max_result["pwm"]}% PWM')
                
                self.get_logger().info('  PWM → Velocity Profile:')
                for r in results:
                    if r['avg_velocity'] > 0.05:
                        self.get_logger().info(f'    {r["pwm"]}% → {r["avg_velocity"]:.2f} rad/s')
        
        self.get_logger().info('='*50)
        
    def destroy_node(self):
        """Clean up"""
        self.stop_motors()
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorCharacterizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nCharacterization interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()