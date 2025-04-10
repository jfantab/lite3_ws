import os
import socket
import time
from enum import Enum
from threading import Lock

import rclpy 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion

from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose, PoseArray, Twist, PoseWithCovarianceStamped
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
        self.err_dist = 0
        self.err_theta = 0

        ###
        self.latest_markers = None
        self.marker_mutex = Lock()
        self.latest_odom = None 
        self.odom_mutex = Lock()

        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.odom_cb_group = MutuallyExclusiveCallbackGroup()

        self.marker_sub = self.create_subscription(ArucoMarkers, 'aruco_markers', self.marker_callback, 10, callback_group=self.sub_cb_group)
        self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, 'leg_odom', self.odom_callback, 10, callback_group=self.odom_cb_group)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group=self.timer_cb_group)
        
        ###
        self.state = ARUCO_STATE.SEARCHING
        
        self.cur_id = 0
        self.goal_id = 2
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

    def move(self, linear_velocity, angular_velocity, y_velocity=0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.linear.y = y_velocity
        msg.angular.z = angular_velocity
        self.twist_pub.publish(msg)
        self.get_logger().info(f"Publishing Twist message: {msg}")

    def marker_callback(self, markers):
        with self.marker_mutex:
            self.latest_markers = markers
            self.get_logger().debug(f"New markers: {markers}")

    def odom_callback(self, odom):
        with self.odom_mutex:
            self.latest_odom = odom 
            self.get_logger().debug(f"New odom: {odom}")

    def timer_callback(self):
        # check mutex
        with self.marker_mutex:
            markers = self.latest_markers

        with self.odom_mutex:
            cur_odom = self.latest_odom

        # check if markers were detected
        if not markers:
            return

        # TODO
        if not odom:
            pass

        cur_position = odom.pose.pose.position

        match self.state:
            case ARUCO_STATE.IDLE:
                self.get_logger().info("[INFO] Idle state...")

            case ARUCO_STATE.SEARCHING:
                self.get_logger().info("[INFO] Searching state...")

                if not markers:
                    # Slowly rotate the robot
                    self.move(0.0, 0.3)
                    return

                ### TODO: figure out PID for horizontal movement to
                ###       center the tag in camera's FOV
                # Kp_dist, Ki_dist, Kd_dist = 0.05, 0.1, 0.05

                ### TODO: figure out condition here
                # if markers and len(markers.marker_ids) > 0 and self.cur_id in markers.marker_ids:
                #     self.setState(ARUCO_STATE.DETECTED)
                #     return
                
                offset = markers.offsets[-1]

            case ARUCO_STATE.DETECTED:
                self.get_logger().info("[INFO] Detected state...")

                # params
                Kp_dist, Ki_dist, Kd_dist = 0.05, 0.1, 0.05
                Kp_theta, Ki_theta, Kd_theta = 0.15, 0.25, 0.1

                # obtain depth or linear distance
                depth = markers.poses[-1].position.z

                # obtain offset or yaw to turn robot
                yaw = markers.yaw_angles[-1]

                # initialize integral and derivative terms
                integral_dist = 0.0
                previous_err_dist = 0.0
                integral_theta = 0.0
                previous_err_theta = 0.0

                # errors
                self.err_dist = depth
                self.err_theta = yaw 

                l_v = 0.0
                a_v = 0.0

                # PID control for linear velocity
                if self.err_dist >= 0.5:
                    l_v = Kp_dist * abs(self.err_dist) + Ki_dist * integral_dist + Kd_dist * (self.err_dist - previous_err_dist)
                    previous_err_dist = self.err_dist
                else:
                    self.get_logger().info(f"Robot stopping as goal distance is within tolerance")
                    l_v = 0.0
                    self.move(0.0, 0.0)
                    self.setState(ARUCO_STATE.GOAL)
                    return

                # PID control for angular velocity
                if abs(self.err_theta) >= 0.05:
                    a_v = Kp_theta * self.err_theta + Ki_theta * integral_theta + Kd_theta * (self.err_theta - previous_err_theta)
                    # a_v = a_v * -1
                    previous_err_theta = self.err_theta
                else:
                    self.get_logger().info(f"Robot stopping as goal heading is within tolerance")
                    a_v = 0.0
                    
                self.move(l_v, -a_v)

            case ARUCO_STATE.GOAL:
                self.get_logger().info("[INFO] Goal state...")

                if self.cur_id >= self.goal_id:
                    self.setState(ARUCO_STATE.IDLE)
                    return

                if not markers:
                    self.get_logger().warn("[WARN] No markers!")

                if(self.landmarks[self.cur_id]["reached"]):
                    self.setState(ARUCO_STATE.IDLE)
                    return

                self.get_logger().info("[INFO] Reached destination")

                # audio_filename = self.landmarks[marker_id]["label"] + ".wav"
                # tts_file(audio_filename)

                self.landmarks[self.cur_id]["reached"] = True
                self.cur_id += 1

                time.sleep(1)

                self.setState(ARUCO_STATE.SEARCHING)
                return

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

'''
header:
  stamp:
    sec: 1980
    nanosec: 989259712
  frame_id: odom
pose:
  pose:
    position:
      x: 1.9869206752407795
      y: 5.4515822736145925
      z: 0.3261604505586008
    orientation:
      x: 0.0
      y: 0.0
      z: 0.024067529310749517
      w: 0.9997103350635504
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
---
'''
