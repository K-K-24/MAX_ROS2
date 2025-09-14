import rclpy
from rclpy.node import Node
import board
import busio
import adafruit_bno055
import math
from std_msgs.msg import Float32
from roomba_interfaces.msg import ImuData


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.orientation_publisher = self.create_publisher(ImuData, '/imu', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Initialize IMU
        i2c = busio.I2C(board.SCL, board.SDA)
        self.imu = adafruit_bno055.BNO055_I2C(i2c)
        
        self.get_logger().info('IMU node started')
        
    def timer_callback(self):
        try:
            yaw = self.imu.euler[0]
            w_z = self.imu.gyro[2]
            if yaw and w_z is not None:

       
                msg = ImuData()
                msg.yaw = -math.radians(yaw)
                msg.w_z = w_z
                self.get_logger().info(f'IMU yaw: {yaw} degrees')
                self.get_logger().info(f'Angular Vel(z-axis): {w_z} rad/s')
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