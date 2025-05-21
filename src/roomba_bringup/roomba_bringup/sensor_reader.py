import rclpy
from rclpy.node import Node
import serial
import time
from roomba_interfaces.msg import SensorData

class SensorReaderNode(Node):
    def __init__(self):
        super().__init__('sensor_reader')
        self.publisher = self.create_publisher(SensorData, '/wheel_states', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Initialize serial connection
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser.flush()
        time.sleep(2)
        
        self.get_logger().info('Sensor reader node started')
        
    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                values = line.split(",")
                
                if len(values) >= 3:
                    left_distance = float(values[0])
                    right_distance = float(values[1])
                    ultrasonic_dist = float(values[2])
                    
                    msg = SensorData()
                    msg.left_encoder = left_distance
                    msg.right_encoder = right_distance
                    msg.ultrasonic_distance = ultrasonic_dist
                    
                    self.publisher.publish(msg)
                    
        except Exception as e:
            self.get_logger().error(f'Sensor read error: {e}')
            
    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    sensor_reader = SensorReaderNode()
    rclpy.spin(sensor_reader)
    sensor_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()