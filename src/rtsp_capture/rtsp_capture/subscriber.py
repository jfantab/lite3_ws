import cv2
import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

class CVSubscriber(Node):

    def __init__(self):
        super().__init__('cv_sub')
        self.subscriber_ = self.create_subscription(Image, 'image_rect', self.listener_callback, 10)
        self.subscriber_
        self.bridge = CvBridge()
    
    def listener_callback(self, img_msg):
        self.get_logger().info("Successfully received image")
        img = self.bridge.imgmsg_to_cv2(img_msg)

        cv2.imshow('RTSP Stream: ', img)
        cv2.waitKey(1)

def main():
    rclpy.init()

    sub = CVSubscriber()

    rclpy.spin(sub)

    sub.destroy_node()

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()