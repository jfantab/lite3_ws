import os
import math
import socket
import time
import numpy as np
from collections import deque
from enum import Enum
from threading import Lock

import rclpy 
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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
        self.err_offset = 0

        # initialize integral and derivative terms
        self.integral_dist = 0.0
        self.previous_err_dist = 0.0
        self.integral_theta = 0.0
        self.previous_err_theta = 0.0

        ###
        self.latest_marker = None
        POSE_BUFFER_SIZE = 5
        self.markers_buffer = deque(maxlen=POSE_BUFFER_SIZE)
        self.latest_marker_time = None
        self.marker_mutex = Lock()
        # self.latest_odom = None 
        # self.odom_mutex = Lock()
        self.latest_pose = None 
        self.pose_mutex = Lock()

        self.sub_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        # self.odom_cb_group = MutuallyExclusiveCallbackGroup()

        self.marker_sub = self.create_subscription(ArucoMarkers, 'aruco_markers', self.marker_callback, 10, callback_group=self.sub_cb_group)
        # self.odom_sub = self.create_subscription(PoseWithCovarianceStamped, 'leg_odom', self.odom_callback, 1, callback_group=self.odom_cb_group)

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_pub_time = self.get_clock().now().nanoseconds
        self.max_rate = 20.0 
        self.MAX_STALE_TIME = 1.2
        
        self.timer_period = 1 / self.max_rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback, callback_group=self.timer_cb_group)
        
        ###
        self.state = ARUCO_STATE.SEARCHING
        
        self.cur_id = 0
        self.goal_id = 2
        self.landmarks = [
            {
                "label": "eng1",
                "reached": False
            },
            {
                "label": "eng2",
                "reached": False
            },
            {
                "label": "eng3",
                "reached": False
            },
            {
                "label": "eng4",
                "reached": False
            },
            {
                "label": "eng5",
                "reached": False
            }
        ]

    def setState(self, state):
        self.state = state

    def move(self, linear_velocity, angular_velocity, y_velocity=0.0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.linear.y = y_velocity
        msg.angular.z = angular_velocity
        self.twist_pub.publish(msg)
        self.get_logger().info(f"Publishing Twist message: {msg}")

    # def odom_callback(self, odom):
    #     with self.odom_mutex:
    #         self.latest_odom = odom 
    #         self.get_logger().debug(f"New odom: {odom}")

    def marker_callback(self, marker):
        with self.marker_mutex:
            # self.get_logger().info(f"New markers: {marker}")
            self.markers_buffer.append(marker)
            self.latest_marker_time = self.get_clock().now().nanoseconds
            # self.get_logger().info(f"New markers time: {self.latest_marker_time}")

    def timer_callback(self):
        with self.marker_mutex:
            buffer = self.markers_buffer

            # self.get_logger().info(f"Current deque/buffer: {buffer}")

            if not buffer:
                self.move(0.0, 0.0)
                self.setState(ARUCO_STATE.SEARCHING)
                return
            
            now = self.get_clock().now().nanoseconds
            self.get_logger().info(f"Timestamp subtractions: {[((now * 1e-9) - (p.header.stamp.sec + (p.header.stamp.nanosec * 1e-9))) for p in buffer]}")
            valid_markers = [
                p for p in buffer
                if ((now * 1e-9) - (p.header.stamp.sec + (p.header.stamp.nanosec * 1e-9))) < self.MAX_STALE_TIME
            ]

            self.get_logger().info(f"Valid markers: {valid_markers}")

            if len(valid_markers) == 0:
                self.move(0.0, 0.0)
                self.setState(ARUCO_STATE.SEARCHING)
                return
            
            markers = valid_markers[-1]

            self.get_logger().info(f"Most recent marker: {markers}")

        # check if markers were detected
        if not markers:
            return

        # with self.odom_mutex:
        #     cur_odom = self.latest_odom

        # if not cur_odom:
            # pass

        match self.state:
            case ARUCO_STATE.IDLE:
                self.get_logger().info("[INFO] Idle state...")

                self.move(0.0, 0.0)

            case ARUCO_STATE.SEARCHING:
                if self.cur_id > self.goal_id:
                    self.setState(ARUCO_STATE.IDLE)
                    return

                if markers and markers.marker_ids:
                    self.get_logger().info(f"[INFO] markers.marker_ids: {markers.marker_ids}")

                    if self.cur_id in markers.marker_ids:
                        self.get_logger().info(f"[INFO] Found marker with ID {self.cur_id}")
                        index = markers.marker_ids.index(self.cur_id)

                        self.get_logger().info(f"Aruco marker detected with ID {markers.marker_ids[index]}")
                        self.setState(ARUCO_STATE.DETECTED)
                        return

                # Keep looking for correct marker
                self.get_logger().info("[INFO] Searching state...")
                self.get_logger().info(f"[INFO] Searching for ID {self.cur_id}")
                self.move(0.0, 0.1)
                self.last_pub_time = now

            case ARUCO_STATE.ADJUSTING:
                self.get_logger().info(f"[INFO] Adjusting state")
                if markers and markers.marker_ids:
                    self.get_logger().info(f"[INFO] markers.marker_ids: {markers.marker_ids}")

                    if self.cur_id in markers.marker_ids:
                        self.get_logger().info(f"[INFO] Found marker with ID {self.cur_id}")
                        index = markers.marker_ids.index(self.cur_id)

                        offset = markers.offsets[index]
                        yaw = markers.yaw_angles[index]

                        self.get_logger().info(f"Yaw is {yaw}")

                        if abs(yaw) < (0.01 + (depth * 0.1)):
                            self.move(0.0, 0.0)
                            self.integral_theta = 0.0
                            self.previous_err_theta = 0.0
                            self.get_logger().info(f"Aruco marker detected with ID {markers.marker_ids[index]}")
                            self.setState(ARUCO_STATE.DETECTED)
                        else:
                            self.get_logger().info("[INFO] Getting into position")
                            self.err_theta = yaw 
                            av = self.angular_pid()
                            self.move(0.0, av)
                            self.last_pub_time = now
                else:
                    self.move(0.0, 0.0)
                    self.integral_theta = 0.0
                    self.previous_err_theta = 0.0
                    self.get_logger().info(f"Aruco marker with ID {markers.marker_ids[index]} lost")
                    self.setState(ARUCO_STATE.SEARCHING)

            case ARUCO_STATE.DETECTED:
                self.get_logger().info("[INFO] Detected state...")

                if not markers: 
                    self.move(0.0, 0.0)
                    self.setState(ARUCO_STATE.SEARCHING)
                    return
                
                if self.cur_id not in markers.marker_ids:
                    self.move(0.0, 0.0)
                    self.setState(ARUCO_STATE.SEARCHING)
                    return

                self.get_logger().info(f"[INFO] Moving towards detected marker...")

                # get correct marker
                index = markers.marker_ids.index(self.cur_id)
                self.get_logger().info(f"[INFO] Detected marker is ID {markers.marker_ids[index]}")

                # robot dog rpy
                # orientation = cur_odom.pose.pose.orientation
                # r, p, y = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

                # obtain depth using Euclidean norm
                marker_x = markers.poses[index].position.x
                marker_y = markers.poses[index].position.y
                marker_z = markers.poses[index].position.z
                depth = math.sqrt(marker_x**2 + marker_y**2 + marker_z**2)

                # obtain yaw to turn robot
                yaw = markers.yaw_angles[index]

                # offset
                offset = markers.offsets[index]

                # log info
                self.get_logger().info(f"Current robot yaw: {yaw}")
                self.get_logger().info(f"Current robot offset: {offset}")
                self.get_logger().info(f"Current depth: {depth}")

                # errors
                self.err_dist = depth
                self.err_theta = yaw

                lv, av = 0.0, 0.0

                if abs(self.err_theta) >= 0.01:
                    av = self.angular_pid()
                else:
                    av = 0.0

                if abs(self.err_dist) >= 0.7:
                    lv = self.linear_pid()
                else:
                    self.get_logger().info(f"Goal distance is within tolerance")
                    lv, av = 0.0, 0.0

                    self.integral_dist = 0.0
                    self.previous_err_dist = 0.0

                    self.integral_theta = 0.0
                    self.previous_err_theta = 0.0
                    
                    self.move(0.0, 0.0)
                    self.setState(ARUCO_STATE.GOAL)
                    return

                self.get_logger().info(f"lv: {lv} | av: {av}")

                self.move(lv, av)
                self.last_pub_time = now

            case ARUCO_STATE.GOAL:
                self.get_logger().info("[INFO] Goal state...")

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

    # PID control for linear velocity
    def linear_pid(self):
        lv = 0.0
        dt = 1.0 / self.max_rate

        Kp_dist, Ki_dist, Kd_dist = 0.01, 0.08, 0.1

        proportional = Kp_dist * self.err_dist

        self.integral_dist += Ki_dist * self.err_dist * dt 
        self.integral_dist = np.clip(self.integral_dist, -0.1, 0.1)

        derivative = Kd_dist * ((self.err_dist - self.previous_err_dist) / dt)
        derivative = np.clip(derivative, -0.15, 0.15)

        lv = proportional + self.integral_dist + derivative

        # linear_scale = np.clip(math.cos(self.err_theta), 0.1, 1.0)  # Range: 0.1 to 1.0
        # scaled_lv = lv * linear_scale

        max_angular_error = 0.4 # degrees (slow to 0 if error > 45°)

        # Calculate speed scaling factor (0 to 1)
        scaling_factor = max(0, 1 - (abs(self.err_theta) / max_angular_error))

        # Apply to linear speed
        lv = lv * scaling_factor

        lv = np.clip(lv, -0.5, 0.5) # m/s

        self.previous_err_dist = self.err_dist

        return lv

    # PID control for angular velocity
    def angular_pid(self):
        av = 0.0
        dt = 1.0 / self.max_rate
        
        Kp_theta, Ki_theta, Kd_theta = 0.05, 0.004, 0.3
        proportional = Kp_theta * self.err_theta

        self.integral_theta += Ki_theta * self.err_theta * dt 
        self.integral_theta = np.clip(self.integral_theta, -0.01, 0.01)

        derivative = Kd_theta * ((self.err_theta - self.previous_err_theta) / dt)
        derivative = np.clip(derivative, -0.2, 0.2)

        av = proportional + self.integral_theta + derivative
        av = np.clip(av, -0.4, 0.4)  # rad/s

        self.previous_err_theta = self.err_theta

        return av

def main():
    rclpy.init()
    
    tts = TTS()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(tts)
    
    try: 
        executor.spin()
    finally:
        tts.destroy_node()
        executor.shutdown()
    # rclpy.spin(tts)

    # tts.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
