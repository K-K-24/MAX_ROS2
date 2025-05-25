#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import time
import csv
from datetime import datetime

class PIDTestTool(Node):
    """
    Comprehensive PID test tool with detailed analysis and logging
    """
    
    def __init__(self):
        super().__init__('pid_test_tool')
        
        # Publisher for target velocity
        self.target_pub = self.create_publisher(Float32, '/target_velocity', 10)
        
        # Subscribers for monitoring
        self.current_vel_sub = self.create_subscription(
            Float32, '/current_velocity', self.current_vel_callback, 10)
        self.error_sub = self.create_subscription(
            Float32, '/velocity_error', self.error_callback, 10)
        self.pwm_sub = self.create_subscription(
            Float32, '/pwm_output', self.pwm_callback, 10)
        
        self.current_velocity = 0.0
        self.velocity_error = 0.0
        self.pwm_output = 0.0
        self.target_velocity = 0.0
        
        # Data logging
        self.logging_active = False
        self.log_data = []
        self.test_start_time = None
        
        # Timer for status display (faster for better monitoring)
        self.create_timer(0.2, self.print_status)  # 5Hz display
        
        self.get_logger().info('🎮 Enhanced PID Test Tool Started')
        self.get_logger().info('Comprehensive testing and analysis capabilities')
        
    def current_vel_callback(self, msg):
        self.current_velocity = msg.data
        self.log_data_point()
        
    def error_callback(self, msg):
        self.velocity_error = msg.data
        
    def pwm_callback(self, msg):
        self.pwm_output = msg.data
        
    def log_data_point(self):
        """Log data for analysis"""
        if self.logging_active and self.test_start_time is not None:
            timestamp = time.time() - self.test_start_time
            self.log_data.append({
                'time': timestamp,
                'target': self.target_velocity,
                'current': self.current_velocity,
                'error': self.velocity_error,
                'pwm': self.pwm_output
            })
    
    def start_logging(self, test_name):
        """Start data logging for a test"""
        self.log_data = []
        self.test_start_time = time.time()
        self.logging_active = True
        print(f"📊 Started logging for: {test_name}")
        
    def stop_logging_and_analyze(self, test_name):
        """Stop logging and analyze the results"""
        self.logging_active = False
        
        if not self.log_data:
            print("⚠️  No data collected!")
            return
            
        # Save raw data to CSV
        filename = f"pid_test_{test_name}_{datetime.now().strftime('%H%M%S')}.csv"
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['time', 'target', 'current', 'error', 'pwm']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.log_data)
        
        # Analyze the data
        self.analyze_test_results(test_name)
        print(f"💾 Data saved to: {filename}")
        
    def analyze_test_results(self, test_name):
        """Analyze PID performance from logged data"""
        if len(self.log_data) < 10:
            return
            
        print(f"\n📈 ANALYSIS: {test_name}")
        print("=" * 50)
        
        # Get final 20% of data for steady-state analysis
        steady_start = int(len(self.log_data) * 0.8)
        steady_data = self.log_data[steady_start:]
        
        if steady_data:
            # Steady-state metrics
            avg_current = sum(d['current'] for d in steady_data) / len(steady_data)
            avg_error = sum(abs(d['error']) for d in steady_data) / len(steady_data)
            avg_pwm = sum(d['pwm'] for d in steady_data) / len(steady_data)
            target = steady_data[0]['target']
            
            # Performance metrics
            if target > 0:
                steady_error_percent = (avg_error / target) * 100
                print(f"🎯 Target Velocity: {target:.2f} rad/s")
                print(f"📊 Steady-State Velocity: {avg_current:.2f} rad/s")
                print(f"❌ Steady-State Error: {avg_error:.2f} rad/s ({steady_error_percent:.1f}%)")
                print(f"⚡ Average PWM: {avg_pwm:.1f}%")
                
                # Performance assessment
                if steady_error_percent < 5:
                    print("✅ EXCELLENT: Very low steady-state error")
                elif steady_error_percent < 10:
                    print("🟡 GOOD: Acceptable steady-state error")
                elif steady_error_percent < 20:
                    print("🟠 FAIR: High steady-state error - tune Ki")
                else:
                    print("🔴 POOR: Very high steady-state error - major tuning needed")
            
            # Settling time analysis
            settle_threshold = 0.1  # Within 0.1 rad/s of target
            settled_time = None
            for i, data in enumerate(self.log_data):
                if abs(data['error']) <= settle_threshold:
                    settled_time = data['time']
                    break
            
            if settled_time:
                print(f"⏱️  Settling Time: {settled_time:.1f} seconds")
                if settled_time < 2.0:
                    print("✅ FAST: Quick settling")
                elif settled_time < 4.0:
                    print("🟡 MODERATE: Acceptable settling time")
                else:
                    print("🔴 SLOW: Consider increasing Kp")
            else:
                print("🔴 NEVER SETTLED: Controller not reaching target")
                
            # Oscillation detection
            velocity_changes = 0
            for i in range(1, len(steady_data)):
                if (steady_data[i]['current'] - steady_data[i-1]['current']) * \
                   (steady_data[i-1]['current'] - steady_data[i-2]['current'] if i > 1 else 1) < 0:
                    velocity_changes += 1
            
            if velocity_changes > len(steady_data) * 0.3:
                print("🔴 OSCILLATING: Consider reducing Kp or increasing Kd")
            else:
                print("✅ STABLE: No significant oscillation")
        
        print("=" * 50)
        
    def send_target_velocity(self, velocity):
        """Send a target velocity command"""
        msg = Float32()
        msg.data = float(velocity)
        self.target_pub.publish(msg)
        self.target_velocity = velocity
        self.get_logger().info(f'🎯 Sent target velocity: {velocity} rad/s')
        
    def print_status(self):
        """Print current system status"""
        if self.current_velocity > 0.05 or abs(self.velocity_error) > 0.05:
            # Calculate PWM efficiency
            expected_pwm = 40 + (self.current_velocity * 7.14)  # From characterization
            pwm_diff = self.pwm_output - expected_pwm
            
            print(f'\r📊 Target: {self.target_velocity:.2f} | '
                  f'Current: {self.current_velocity:.2f} | '
                  f'Error: {self.velocity_error:.2f} | '
                  f'PWM: {self.pwm_output:.1f}% | '
                  f'PWM Δ: {pwm_diff:+.1f}%', end='', flush=True)

def main():
    rclpy.init()
    test_tool = PIDTestTool()
    
    print("\n🎮 Enhanced PID Test Tool")
    print("=" * 60)
    print("Commands:")
    print("  <number>     : Set target velocity (e.g., '2.5' for 2.5 rad/s)")
    print("  0            : Stop motor")
    print("  quick        : Quick response test (5 targets)")
    print("  comprehensive: Full PID analysis (recommended)")
    print("  step         : Step response analysis")
    print("  ramp         : Ramp response test")
    print("  characterize : Compare with motor characterization")
    print("  quit         : Exit")
    print("=" * 60)
    
    # Start ROS spinning in background
    import threading
    spin_thread = threading.Thread(target=lambda: rclpy.spin(test_tool), daemon=True)
    spin_thread.start()
    
    try:
        while True:
            command = input("\n\nEnter command: ").strip().lower()
            
            if command == 'quit' or command == 'q':
                break
            elif command == 'quick':
                run_quick_test(test_tool)
            elif command == 'comprehensive':
                run_comprehensive_test(test_tool)
            elif command == 'step':
                run_step_response_test(test_tool)
            elif command == 'ramp':
                run_ramp_test(test_tool)
            elif command == 'characterize':
                run_characterization_comparison(test_tool)
            elif command == '0':
                test_tool.send_target_velocity(0.0)
            else:
                try:
                    velocity = float(command)
                    if 0 <= velocity <= 7.0:
                        test_tool.send_target_velocity(velocity)
                    else:
                        print("⚠️  Please enter velocity between 0 and 7.0 rad/s")
                except ValueError:
                    print("⚠️  Invalid command. Type 'comprehensive' for full test or enter a number")
                    
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the motor before exiting
        test_tool.send_target_velocity(0.0)
        time.sleep(0.5)
        test_tool.destroy_node()
        rclpy.shutdown()

def run_quick_test(test_tool):
    """Quick 5-point test"""
    print("\n🚀 Running Quick Response Test...")
    
    test_points = [1.0, 3.0, 5.0, 2.0, 0.0]
    
    for i, vel in enumerate(test_points):
        print(f"\n📍 Test {i+1}/{len(test_points)}: Target = {vel} rad/s")
        test_tool.start_logging(f"quick_{i+1}_{vel}rads")
        test_tool.send_target_velocity(vel)
        
        # Wait longer for each test
        wait_time = 8.0 if vel > 0 else 3.0
        time.sleep(wait_time)
        
        test_tool.stop_logging_and_analyze(f"Quick Test {i+1}: {vel} rad/s")
    
    print("\n✅ Quick test sequence complete!")

def run_comprehensive_test(test_tool):
    """Comprehensive PID analysis"""
    print("\n🔬 Running Comprehensive PID Analysis...")
    print("This will take about 5 minutes but provide detailed insights")
    
    # Test different velocity ranges thoroughly
    test_scenarios = [
        (1.0, "Low Speed", 12.0),
        (2.5, "Medium-Low", 12.0), 
        (4.0, "Medium", 12.0),
        (6.0, "High Speed", 12.0),
        (3.0, "Return to Medium", 12.0),
        (0.0, "Full Stop", 5.0)
    ]
    
    for i, (vel, description, duration) in enumerate(test_scenarios):
        print(f"\n📍 Test {i+1}/{len(test_scenarios)}: {description} ({vel} rad/s)")
        test_tool.start_logging(f"comprehensive_{i+1}_{description.replace(' ', '_')}")
        test_tool.send_target_velocity(vel)
        
        # Show countdown
        for remaining in range(int(duration), 0, -1):
            print(f"   ⏱️  {remaining}s remaining...", end='\r')
            time.sleep(1.0)
        
        test_tool.stop_logging_and_analyze(f"{description}: {vel} rad/s")
    
    print("\n🎉 Comprehensive analysis complete!")
    print("📊 Check the generated CSV files for detailed data")

def run_step_response_test(test_tool):
    """Test step responses for PID tuning insights"""
    print("\n📈 Running Step Response Analysis...")
    
    steps = [(0.0, 2.0), (2.0, 4.0), (4.0, 1.0), (1.0, 0.0)]
    
    for i, (start_vel, end_vel) in enumerate(steps):
        print(f"\n📍 Step {i+1}: {start_vel} → {end_vel} rad/s")
        
        # Set initial velocity
        test_tool.send_target_velocity(start_vel)
        time.sleep(5.0 if start_vel > 0 else 2.0)
        
        # Execute step
        test_tool.start_logging(f"step_{start_vel}to{end_vel}")
        test_tool.send_target_velocity(end_vel)
        time.sleep(10.0)
        
        test_tool.stop_logging_and_analyze(f"Step: {start_vel}→{end_vel} rad/s")

def run_ramp_test(test_tool):
    """Test ramp responses"""
    print("\n📈 Running Ramp Response Test...")
    
    test_tool.start_logging("ramp_0_to_5")
    
    # Gradual ramp from 0 to 5 rad/s
    for vel in [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]:
        print(f"   🎯 Ramping to {vel} rad/s...")
        test_tool.send_target_velocity(vel)
        time.sleep(6.0)
    
    test_tool.stop_logging_and_analyze("Ramp 0→5 rad/s")

def run_characterization_comparison(test_tool):
    """Compare PID performance with motor characterization data"""
    print("\n🔍 Comparing with Motor Characterization...")
    
    # Test at the same PWM levels from characterization
    char_data = [
        (40, 1.89),  # Expected velocity from characterization
        (50, 3.39),
        (60, 4.13),
        (70, 5.29),
        (80, 6.02),
        (90, 6.96)
    ]
    
    for pwm, expected_vel in char_data:
        print(f"\n📍 Testing target {expected_vel:.2f} rad/s (from {pwm}% PWM characterization)")
        test_tool.start_logging(f"char_compare_{pwm}pct")
        test_tool.send_target_velocity(expected_vel)
        time.sleep(10.0)
        test_tool.stop_logging_and_analyze(f"Char Compare {pwm}%: {expected_vel:.2f} rad/s")

if __name__ == '__main__':
    main()