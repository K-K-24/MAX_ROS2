#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import threading

class SimpleTestTool(Node):
    """
    Dead simple test tool - no BS, just clear results
    """
    
    def __init__(self):
        super().__init__('simple_test_tool')
        
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
        
        self.get_logger().info('🔧 Simple Test Tool Ready')
        
    def current_vel_callback(self, msg):
        self.current_velocity = msg.data
        
    def error_callback(self, msg):
        self.velocity_error = msg.data
        
    def pwm_callback(self, msg):
        self.pwm_output = msg.data
        
    def send_target(self, velocity):
        """Send target velocity and return immediately"""
        msg = Float32()
        msg.data = float(velocity)
        self.target_pub.publish(msg)
        print(f"🎯 Target set to: {velocity} rad/s")
        
    def test_single_velocity(self, target_vel, test_duration=10):
        """Test a single velocity for specified duration"""
        print(f"\n{'='*60}")
        print(f"🧪 TESTING: {target_vel} rad/s for {test_duration} seconds")
        print(f"{'='*60}")
        
        # Send the target
        self.send_target(target_vel)
        
        # CRITICAL: Wait for controller to process the new target
        # This prevents showing stale values from before target change
        time.sleep(1.0)  # Give controller time to process and publish new values
        
        # Collect data for the test duration
        start_time = time.time()
        measurements = []
        
        print("Time(s) | Target | Current | Error  | PWM   | Status")
        print("-" * 50)
        
        while time.time() - start_time < test_duration:
            elapsed = time.time() - start_time
            error_pct = (abs(self.velocity_error) / target_vel * 100) if target_vel > 0 else 0
            
            # Determine status
            if target_vel == 0:
                status = "STOP" if self.current_velocity < 0.1 else "SLOWING"
            elif error_pct < 5:
                status = "EXCELLENT"
            elif error_pct < 10:
                status = "GOOD"
            elif error_pct < 20:
                status = "OK"
            else:
                status = "POOR"
            
            print(f"{elapsed:6.1f}  | {target_vel:6.2f} | {self.current_velocity:7.2f} | {self.velocity_error:6.2f} | {self.pwm_output:5.1f} | {status}")
            
            # Store measurement
            measurements.append({
                'time': elapsed,
                'target': target_vel,
                'current': self.current_velocity,
                'error': self.velocity_error,
                'pwm': self.pwm_output,
                'error_pct': error_pct
            })
            
            time.sleep(1.0)  # Update every second
        
        # Analyze results
        if len(measurements) > 1:  # Skip first measurement for settling
            stable_data = measurements[1:]
            avg_current = sum(m['current'] for m in stable_data) / len(stable_data)
            avg_error = sum(abs(m['error']) for m in stable_data) / len(stable_data)
            avg_pwm = sum(m['pwm'] for m in stable_data) / len(stable_data)
            avg_error_pct = (avg_error / target_vel * 100) if target_vel > 0 else 0
            
            print(f"\n📊 RESULTS SUMMARY:")
            print(f"   Target Velocity: {target_vel:.2f} rad/s")
            print(f"   Actual Velocity: {avg_current:.2f} rad/s")
            print(f"   Average Error: {avg_error:.2f} rad/s ({avg_error_pct:.1f}%)")
            print(f"   Average PWM: {avg_pwm:.1f}%")
            
            # Performance assessment
            if target_vel > 0:
                if avg_error_pct < 5:
                    print(f"   ✅ EXCELLENT performance!")
                elif avg_error_pct < 10:
                    print(f"   🟡 GOOD performance")
                elif avg_error_pct < 20:
                    print(f"   🟠 OK performance - could be better")
                else:
                    print(f"   🔴 POOR performance - needs work")
            else:
                # Special handling for stop command
                if avg_current < 0.1 and avg_pwm < 0.1:
                    print(f"   ✅ EXCELLENT - Motor properly stopped!")
                else:
                    print(f"   🔴 POOR - Motor not properly stopped")
        
        return measurements

def main():
    rclpy.init()
    test_tool = SimpleTestTool()
    
    # Start ROS in background
    spin_thread = threading.Thread(target=lambda: rclpy.spin(test_tool), daemon=True)
    spin_thread.start()
    
    print("\n🎮 SIMPLE VELOCITY CONTROLLER TEST")
    print("=" * 50)
    print("Commands:")
    print("  1. basic     - Test key velocities (2, 4, 6 rad/s)")
    print("  2. sequence  - Test velocity sequence with stops")
    print("  3. manual    - Enter individual velocities")
    print("  4. quick     - Quick 3-point test")
    print("  5. quit      - Exit")
    print("=" * 50)
    
    try:
        while True:
            choice = input("\nEnter choice (1-5): ").strip()
            
            if choice == '1' or choice.lower() == 'basic':
                run_basic_test(test_tool)
            elif choice == '2' or choice.lower() == 'sequence':
                run_sequence_test(test_tool)
            elif choice == '3' or choice.lower() == 'manual':
                run_manual_test(test_tool)
            elif choice == '4' or choice.lower() == 'quick':
                run_quick_test(test_tool)
            elif choice == '5' or choice.lower() == 'quit':
                break
            else:
                print("Invalid choice. Enter 1-5.")
                
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motor
        test_tool.send_target(0.0)
        time.sleep(1)
        test_tool.destroy_node()
        rclpy.shutdown()

def run_basic_test(test_tool):
    """Test the three key velocity points from characterization"""
    print("\n🧪 BASIC TEST - Testing key characterization points")
    
    test_velocities = [2.0, 4.0, 6.0, 0.0]
    
    for vel in test_velocities:
        duration = 4 if vel > 0 else 2  # Shorter durations
        test_tool.test_single_velocity(vel, duration)
        
        if vel > 0:  # Don't pause after stop
            input("\nPress Enter to continue to next test...")
    
    print("\n✅ Basic test complete!")

def run_sequence_test(test_tool):
    """Test a sequence of velocities with automatic progression"""
    print("\n🧪 SEQUENCE TEST - Automated velocity sequence")
    
    sequence = [
        (1.0, 4),  # (velocity, shorter duration)
        (3.0, 4),
        (5.0, 4),
        (2.0, 4),
        (0.0, 2)
    ]
    
    for vel, duration in sequence:
        test_tool.test_single_velocity(vel, duration)
        time.sleep(1)  # Brief pause between tests
    
    print("\n✅ Sequence test complete!")

def run_manual_test(test_tool):
    """Manual velocity testing"""
    print("\n🧪 MANUAL TEST - Enter velocities manually")
    print("Enter velocity (0-7 rad/s) or 'done' to finish")
    
    while True:
        try:
            cmd = input("\nVelocity: ").strip().lower()
            if cmd == 'done' or cmd == 'quit':
                break
            
            velocity = float(cmd)
            if 0 <= velocity <= 7:
                duration = int(input(f"Test duration (seconds, default 4): ") or "4")  # Default to 4s
                test_tool.test_single_velocity(velocity, duration)
            else:
                print("Velocity must be 0-7 rad/s")
                
        except ValueError:
            print("Invalid input. Enter a number or 'done'")
    
    # Always stop at end
    test_tool.send_target(0.0)
    print("\n✅ Manual test complete!")

def run_quick_test(test_tool):
    """Quick 3-point validation"""
    print("\n🧪 QUICK TEST - Fast 3-point validation")
    
    quick_points = [3.0, 6.0, 0.0]
    
    for vel in quick_points:
        duration = 3 if vel > 0 else 1  # Very short durations
        test_tool.test_single_velocity(vel, duration)
        time.sleep(0.5)
    
    print("\n✅ Quick test complete!")

if __name__ == '__main__':
    main()