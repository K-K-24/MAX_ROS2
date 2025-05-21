import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float32
from roomba_interfaces.msg import SensorData, Odometry
from nav_msgs.msg import Odometry as NavOdom
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Publishers
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.nav_odom_publisher = self.create_publisher(NavOdom, '/nav_odom', 10)
        
        # Subscribers
        self.sensor_subscription = self.create_subscription(
            SensorData,
            '/wheel_states',
            self.sensor_callback,
            10)
        self.imu_subscription = self.create_subscription(
            Float32,
            '/imu',
            self.imu_callback,
            10)
            
        # Transform broadcaster for tf2
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left = 0.0
        self.last_right = 0.0
        self.first_reading = True
        self.moving_backward = False
        
        # For storing the trajectory
        self.trajectory = []
        
        self.get_logger().info('Odometry node started')
        
    def sensor_callback(self, msg):
        if self.first_reading:
            self.last_left = msg.left_encoder
            self.last_right = msg.right_encoder
            self.first_reading = False
            return
            
        # Compute incremental distances
        delta_left = msg.left_encoder - self.last_left
        delta_right = msg.right_encoder - self.last_right
        
        # Update last measurements
        self.last_left = msg.left_encoder
        self.last_right = msg.right_encoder
        
        # Average displacement
        d = (delta_left + delta_right) / 2.0
        
        if self.moving_backward:
            d = -d
            
        # Update position
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)
        
        # Add to trajectory
        self.trajectory.append((self.x, self.y, self.theta))
        
        # Publish odometry message
        self.publish_odometry()
        
    def imu_callback(self, msg):
        self.theta = msg.data
        self.publish_odometry()
        
    def publish_odometry(self):
        # Publish custom odometry message
        odom_msg = Odometry()
        odom_msg.x = self.x
        odom_msg.y = self.y
        odom_msg.theta = self.theta
        self.odom_publisher.publish(odom_msg)
        
        # Publish standard ROS nav_msgs/Odometry
        nav_odom = NavOdom()
        nav_odom.header.stamp = self.get_clock().now().to_msg()
        nav_odom.header.frame_id = "odom"
        nav_odom.child_frame_id = "base_link"
        
        # Set position
        nav_odom.pose.pose.position.x = self.x / 100.0  # convert to meters
        nav_odom.pose.pose.position.y = self.y / 100.0
        nav_odom.pose.pose.position.z = 0.0
        
        # Set orientation (from IMU)
        quat = euler_to_quaternion(0.0, 0.0, self.theta)
        nav_odom.pose.pose.orientation = quat
        
        self.nav_odom_publisher.publish(nav_odom)
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x / 100.0
        t.transform.translation.y = self.y / 100.0
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()