import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import random

class FakeCameraNode(Node):

    def __init__(self):
        super().__init__('fake_camera_node')

        self.img_publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge()

        self.colors = {
            'red': (0,0,255)
        }

        self.shapes = [
            'circle', 'triangle', 'rectangle'
        ]

        self.timer = self.create_timer(1, self.camera_callback)

        self.get_logger().info("Fake Camera Node has been started.")

    def camera_callback(self):
        img = np.ones((480, 640, 3), dtype=np.uint8)
        color = self.colors['red']
        shape = random.choice(self.shapes)

        if shape == 'circle':
            cv2.circle(img, (320, 240), 50, color, -1)
        elif shape == 'rectangle':
            cv2.rectangle(img, (270, 190), (390, 290), color, -1)
        else:
            pts = np.array([[320, 180], [270, 290], [370, 290]], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.drawContours(img, [pts], 0, color, -1)

        # Convert BGR to BGR8 (no need for RGBA2BGR since img is already BGR)
        ros2_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.img_publisher.publish(ros2_msg)

        self.get_logger().info(f"Published image with shape: {shape} and color: {color}")

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    fake_camera_node = FakeCameraNode()
    rclpy.spin(fake_camera_node)
    fake_camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


