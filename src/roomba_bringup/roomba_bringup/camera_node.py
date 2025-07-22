import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.img_publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge()


        #Camera initialization
        self.cam = cv2.VideoCapture(0)
        
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

                # Give camera time to initialize
        time.sleep(2)


        self.timer = self.create_timer(0.1, self.camera_callback)


    def camera_callback(self):

            ret, frame = self.cam.read()

            if ret: 
                frame = cv2.rotate(frame, cv2.ROTATE_180)  

                bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

                cv2.imwrite("captured_image.png", frame)

                ros2_msg = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")

                self.img_publisher.publish(ros2_msg)
     
              
                                    
            else:
                print("Frame not captured")

    def destroy_node(self):
         self.cam.release()
         super().destroy_node()

        
def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()

