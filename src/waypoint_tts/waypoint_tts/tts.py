import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import socket

def tts_file(file):
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
    ctrl_addr = ('192.168.1.120', 45678)

    server.bind(('192.168.1.65', 54321))

    filename = bytes(file, 'utf-8')
    server.sendto(filename, ctrl_addr)

    msg, addr = server.recvfrom(1024)
    result = msg.decode('utf-8')
    return result

class TTS(Node):

    def __init__(self):
        super().__init__("waypoint_tts")
        self.marker_sub = self.create_subscription(ArucoMarkers, 'aruco_markers', self.marker_callback, 10)
        self.yaml_path = os.path.join(
            get_package_share_directory("waypoint_tts"),
            "config",
            "tags.yaml"
        )
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.landmarks = [
            {
                "label": "msse",
                "reached": False
            },
            {
                "label": "mscs",
                "reached": False
            },
            {
                "label": "nursing",
                "reached": False
            },
            {
                "label": "psych",
                "reached": False
            }
        ]

        # try:
        #     with open(self.yaml_path, 'r') as file:
        #         self.landmarks = yaml.safe_load(file)
        #         self.get_logger().info(f"Loaded landmarks: {self.landmarks}")
        # except FileNotFoundError:
        #     self.get_logger().error(f"YAML file not found: {self.yaml_path}")
        # except yaml.YAMLError as exc:
        #     self.get_logger().error(f"Error parsing YAML file: {exc}")

    def marker_callback(self, markers):
        if not markers:
            self.get_logger().warn("No markers!")
            return
        
        if len(markers.marker_ids) == 0:
            return
        
        files = ["msse.wav", "mscs.wav", "nursing.wav", "psych.wav"]
        
        depth = markers.poses[-1].position.z
        marker_id = markers.marker_ids[-1]

        if marker_id == 0 and depth > 1.5:
            msg = Twist()
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            self.twist_pub.publish(msg)
            self.get_logger().info(f"Publishing Twist message: {msg}")

        if depth > 1.5:
            return

        self.get_logger().info("[INFO] Reached destination")
        
        if self.landmarks[marker_id]["reached"]:
            return

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.twist_pub.publish(msg)
        self.get_logger().info(f"Publishing Twist message: {msg}")

        tts_file(files[marker_id])

        self.landmarks[marker_id]["reached"] = True

def main():
    rclpy.init()
    
    tts = TTS()
    rclpy.spin(tts)

    tts.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()