import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from roomba_interfaces.msg import RobotCoord
from visualization_msgs.msg import Marker

class ObjectDetectNode(Node):
    def __init__(self):
        super().__init__('object_detect_node')

        self.img_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.coordinate_publisher = self.create_publisher( RobotCoord, '/robot/coordinates',10)

        self.marker_publisher = self.create_publisher(Marker, '/object_marker', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("Received image message")
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        [height,width,_] = image.shape

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        mask = cv2.inRange(hsv, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        object_geo_center = 0
        pixel_object_width = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:  # Filter out small contours
                x, y, w, h = cv2.boundingRect(cnt)
                object_geo_center = x + w /2
                pixel_object_width = w
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                #Centroid
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                
                else:
                    cx, cy = 0, 0

                cv2.circle(image, (cx, cy), 5, (255, 0, 0), -1)

                #Angle Calculation

                offset = object_geo_center - width/2 # Assuming image width is 640 pixels ( negative means left, positive means right)

                field_of_view = 60  # Camera field of view in degrees

                theta = (offset / (width/2)) * (field_of_view/2)  # Convert pixel offset to angle
                theta_in_rad = math.radians(theta)  # Convert degrees to radians

                #Distance Calculation
                real_object_width = 5

                d = (real_object_width * width) / (pixel_object_width * 2 * math.tan(math.radians(field_of_view/2)))

                

                #robot coordinates
                robot_x = d * math.cos(theta_in_rad)
                robot_y = d * math.sin(theta_in_rad)

                coord_msg = RobotCoord()
                coord_msg.x = robot_x/100   #to (m)
                coord_msg.y = robot_y/100

                #Create marker
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "base_link"
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                #Position
                marker.pose.position.x = robot_x/100
                marker.pose.position.y = robot_y/100
                marker.pose.position.z = 0.05 #Slightly above the ground

                #Size
                marker.scale.x = 0.1 #10 cm Sphere
                marker.scale.y = 0.1
                marker.scale.z = 0.1

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker.color.a = 1.0 #Opacity

                self.get_logger().info(f"Robot Coordinates: x={robot_x}, y={robot_y}")
                self.coordinate_publisher.publish(coord_msg)
                self.marker_publisher.publish(marker)

          










def main(args = None):
    rclpy.init(args=args)
    object_detect_node = ObjectDetectNode()
    rclpy.spin(object_detect_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
