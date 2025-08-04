import rclpy
from rclpy.node import Node
import board
import busio
import adafruit_bno055
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.orientation_publisher = self.create_publisher(Float32, '/imu', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Initialize IMU
        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(i2c)
        
        self.get_logger().info('IMU node started')
        
    def timer_callback(self):
        try:
            yaw = self.imu.euler[0]
            if yaw is not None:

                self.get_logger().info(f'IMU yaw: {yaw} degrees')
                msg = Float32()
                msg.data = -math.radians(yaw)
                self.orientation_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'IMU read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()