import os
import socket
from enum import Enum
from threading import Lock

import rclpy 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion

from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose, PoseArray, Twist
from std_msgs.msg import Float64MultiArray


def tts_file(file):
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
    ctrl_addr = ('192.168.1.120', 45678)

    server.bind(('192.168.1.65', 54321))

    filename = bytes(file, 'utf-8')
    server.sendto(filename, ctrl_addr)

    msg, addr = server.recvfrom(1024)
    result = msg.decode('utf-8')
    return result

class ARUCO_STATE(Enum):
    IDLE = 1
    SEARCHING = 2
    DETECTED = 3
    ADJUSTING = 4
    GOAL = 5
    FAILED = 6

class TTS(Node):

    def __init__(self):
        super().__init__("waypoint_tts")
        
        ###
        self.linear_k = 0.05
        self.angular_k = 1.0

        # self.linear_max = 1.5
        # self.angular_max = 0.2

        ###
        self.latest_markers = None
        self.marker_mutex = Lock()

        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.marker_sub = self.create_subscription(ArucoMarkers, 'aruco_markers', self.marker_callback, 10, callback_group=self.sub_cb_group)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group=self.timer_cb_group)
        
        ###
        self.state = ARUCO_STATE.SEARCHING
        
        self.cur_id = 0
        self.goal_id = 0
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

    def setState(self, state):
        self.state = state

    def move(self, linear_error, angular_error):
        msg = Twist()
        msg.linear.x = float(self.linear_k * linear_error)
        msg.angular.z = -float(self.angular_k * angular_error)
        self.twist_pub.publish(msg)
        self.get_logger().info(f"Publishing Twist message: {msg}")

    def marker_callback(self, markers):
        with self.marker_mutex:
            self.latest_markers = markers
            self.get_logger().debug(f"New markers: {markers}")

    def timer_callback(self):
        # check mutex
        with self.marker_mutex:
            markers = self.latest_markers

        # check if markers were detected
        if not markers:
            return

        # orientation = markers.poses[-1].orientation
        # quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        # self.get_logger().info(f"quaternion: {quaternion}")
        # roll, pitch, yaw = euler_from_quaternion(quaternion)

        # navigate to current marker, don't want to be distracted
        marker_index = None
        for index in markers.marker_ids:
            if(markers.marker_ids[index] == self.cur_id):
                marker_index = index

        # marker_id = markers.marker_ids[-1]
        # if marker_id != cur_id:
        #     return

        match self.state:
            case ARUCO_STATE.IDLE:
                self.get_logger().info("[INFO] Idle state...")
            case ARUCO_STATE.SEARCHING:
                self.get_logger().info("[INFO] Searching state...")

                if markers and len(markers.marker_ids) > 0:
                    self.setState(ARUCO_STATE.DETECTED)
                    return
                
                # Slowly rotate the robot
                self.move(0.0, 0.3)
                
            case ARUCO_STATE.DETECTED:
                self.get_logger().info("[INFO] Detected state...")

                if not markers:
                    self.get_logger().warn("[WARN] No markers!")

                marker_id = markers.marker_ids[-1]
                depth = markers.poses[-1].position.z
                offset = markers.offsets[-1]

                # obtain yaw angle to turn robot
                yaw = markers.yaw_angles[-1]

                lin_v = 0.0
                ang_v = 0.0

                if depth > 0.5:
                    lin_v = depth 
                else: 
                    lin_v = 0.0

                if yaw > 0.01:
                    ang_v = yaw
                else: 
                    ang_v = 0.0

                self.move(lin_v, ang_v)

            case ARUCO_STATE.GOAL:
                self.get_logger().info("[INFO] Goal state...")

                if not markers:
                    self.get_logger().warn("[WARN] No markers!")

                marker_id = markers.marker_ids[-1]

                if(self.landmarks[marker_id]["reached"]):
                    self.setState(ARUCO_STATE.IDLE)

                self.get_logger().info("[INFO] Reached destination")

                # Stop the robot
                self.move(0.0, 0.0)

                audio_filename = self.landmarks[marker_id]["label"] + ".wav"
                tts_file(audio_filename)

                self.landmarks[marker_id]["reached"] = True

            case ARUCO_STATE.FAILED:
                self.get_logger().info("[INFO] Failure state...")

                # Stop the robot
                self.move(0.0, 0.0)

def main():
    rclpy.init()
    
    tts = TTS()
    rclpy.spin(tts)

    tts.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()