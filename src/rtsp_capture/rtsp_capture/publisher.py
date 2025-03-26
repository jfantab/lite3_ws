import rclpy
from rclpy.node import Node 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

import os
from dotenv import load_dotenv

load_dotenv()

class CVPublisher(Node):

    def __init__(self):
        super().__init__('cv_pub')
        self.publisher_ = self.create_publisher(Image, 'image_rect', 10)
        self.publisher_
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        URL = os.getenv("RTSP_URL")
        self.get_logger().info("Publishing from " + URL)
        self.cap = cv2.VideoCapture(URL)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame)
            self.publisher_.publish(img_msg)
            self.get_logger().info("Image success")

        self.get_logger().info('Publishing image')

def main():
    rclpy.init()

    pub = CVPublisher()

    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()