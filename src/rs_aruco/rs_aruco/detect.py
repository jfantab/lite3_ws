# Standard library imports
import math
import os
import time
import socket
import time
from collections import deque
from enum import Enum
from threading import Lock
import pyrealsense2 as rs

# Third-party imports
import cv2
import numpy as np
import tf_transformations
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion

# ROS 2 imports
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

# ROS 2 message imports
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, Float64MultiArray, Header

# ROS 2 package utilities
from ament_index_python.packages import get_package_share_directory

class ARUCO_STATE(Enum):
    IDLE = 1
    SEARCHING = 2
    DETECTED = 3
    ADJUSTING = 4
    GOAL = 5
    FAILED = 6

class RS_Aruco_TTS(Node):

    def __init__(self):
        super().__init__("rs_aruco_tts")
        # Defining timer callback
        self.max_rate = 5
        self.image_timer_period = 1 / self.max_rate
        self.image_timer = self.create_timer(self.image_timer_period, self.image_timer_callback)

        # From /camera_info:
        self.width = 1280  # Image width
        self.height = 720   # Image height
        self.intrinsic_mat = np.array([[641.42, 0, 650.5],
                    [0, 640.79, 54.92],
                    [0, 0, 1]])
        self.distortion_mat = np.array([-0.0575, 0.0696, -0.00026, 0.000758, -0.0232])

        # Configure pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

        self.device_time_start = None
        self.host_time_start = None 
        
        self.sync_time()

        # Initialize ArUco detector along with its parameters
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_parameters.errorCorrectionRate = 0.6  # Higher values = stricter (default: 0.6)
        self.aruco_parameters.adaptiveThreshWinSizeMin = 5  # Larger values reduce noise sensitivity
        self.aruco_parameters.minMarkerPerimeterRate = 0.1  # Reject small blobs (default: 0.03)
        self.aruco_parameters.polygonalApproxAccuracyRate = 0.05  # Stricter contour precision
        self.aruco_parameters.minDistanceToBorder = 150 # Try not to detect markers on the edge of the image
        self.aruco_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_parameters.cornerRefinementWinSize = 5
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        self.bridge = CvBridge()

        # Initialize deques
        MAX_MARKER_POSES = 5
        self.markers_buffer = deque(maxlen=MAX_MARKER_POSES)

        MAX_FRAMES = 5
        self.frames_deque = deque(maxlen=MAX_FRAMES)

        # Mutexes
        self.markers_mutex = Lock()
        self.frames_mutex = Lock()

        self.MAX_STALE_TIME = 0.2

        # /cmd_vel publisher
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_pub_time = self.get_clock().now().nanoseconds

        # PID parameters
        self.err_dist = 0
        self.err_theta = 0
        self.err_offset = 0

        self.integral_dist = 0.0
        self.previous_err_dist = 0.0
        self.integral_theta = 0.0
        self.previous_err_theta = 0.0

        # State machine parameters
        self.state = ARUCO_STATE.SEARCHING

        self.TTS_ACTIVE = True
        self.cur_id = 2
        self.goal_id = 3
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

        # More ArUco parameters

        # Declare and read parameters
        self.declare_parameter(
            name="marker_size",
            value=0.0625,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_5X5_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.get_logger().info(f"Marker size: {self.marker_size}")

        dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.get_logger().info(f"Marker type: {dictionary_id_name}")

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
                raise AttributeError
        except AttributeError:
            self.get_logger().error(
                "bad aruco_dictionary_id: {}".format(dictionary_id_name)
            )
            options = "\n".join([s for s in dir(cv2.aruco) if s.startswith("DICT")])
            self.get_logger().error("valid options: {}".format(options))

        # ArUco image publisher
        self.aruco_img_pub = self.create_publisher(Image, "aruco_image", 10)

    def shutdown_hook(self):
        self.pipeline.stop()

    def sync_time(self):
        try: 
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                raise RuntimeError("No frames received during initialization")
            
            self.device_time_start = color_frame.get_frame_metadata(rs.frame_metadata_value.sensor_timestamp) * 1e3
            self.host_time_start = time.time_ns()
            
            self.get_logger().info(f"device_time_start: {self.device_time_start}")
            self.get_logger().info(f"host_time_start: {self.host_time_start}")

        except Exception as e: 
            self.get_logger().warn(f"Exception occurred during time sync: {e}")
            
    def image_timer_callback(self):
        try:
            frames = self.pipeline.poll_for_frames() # Drain buffer

            latest_frames = None
            while frames:
                latest_frames = frames 
                frames = self.pipeline.poll_for_frames()
            
            # Check if any frames remaining
            if not latest_frames:
                return

            # Retrieve latest color frame
            color_frame = latest_frames.get_color_frame()

            # Check if image received
            if not color_frame:
                return

            # Retrieve timestamp
            current_device_time = color_frame.get_frame_metadata(rs.frame_metadata_value.sensor_timestamp) * 1e3
            elapsed_device_time = current_device_time - self.device_time_start
            color_frame_timestamp = self.host_time_start + elapsed_device_time

            color_frame_sec = color_frame_timestamp // 1e9
            color_frame_nanosec = color_frame_timestamp % 1e9
  
            # Create header
            header = Header()
            header.stamp = Time(seconds=color_frame_sec, nanoseconds=color_frame_nanosec).to_msg()
            header.frame_id = "camera_frame"
            
            # Convert to OpenCV format
            raw_image = np.asanyarray(color_frame.get_data())

            # Convert to gray
            gray = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)

            # Pass grayscale image to ArUco processing code
            self.process_image(gray, header)

            # Add PID controller code
            self.navigate()

            self.get_logger().info("\n\n")

        except Exception as e:
            self.get_logger().warn(f"Error encountered: {e}")

    def process_image(self, image, header):
        self.get_logger().info(f"[INFO] Processing image for ArUco marker")
        cv_image = cv2.undistort(image, self.intrinsic_mat, self.distortion_mat)

        height, width = cv_image.shape
        
        center_x = height // 2
        center_y = width // 2

        markers = ArucoMarkers()
        # pose_array = PoseArray()

        corners, marker_ids, rejected = self.detector.detectMarkers(cv_image)

        # self.get_logger().info(f"Corners: {corners}")
        # self.get_logger().info(f"Marker IDs: {marker_ids}")

        if marker_ids is not None:
            # Define marker coordinate system
            marker_length = self.marker_size  # in meters (match your actual marker size)
            obj_points = np.array([
                [-marker_length/2,  marker_length/2, 0],
                [ marker_length/2,  marker_length/2, 0],
                [ marker_length/2, -marker_length/2, 0],
                [-marker_length/2, -marker_length/2, 0]
            ], dtype=np.float32)

            marker_centers = []
            yaws = []
            offsets = []

            # For each detected marker
            rvecs, tvecs = [], []
            for corner in corners:
                # Solve PnP using detected corners and predefined obj_points
                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    corner.reshape(4, 1, 2),  # Reshape to (4,1,2) for solver
                    self.intrinsic_mat,
                    self.distortion_mat
                )
                
                if success:
                    # Get center
                    marker_center = [np.sum(corner[0][:,0]) // 4, np.sum(corner[0][:,1]) // 4]
                    marker_centers.append(marker_center)

                    # Calculate the yaw
                    offset = center_x - marker_center[0] 
                    fx = self.intrinsic_mat[0][0]
                    angle = np.arctan2(offset, fx)
                    corrected_angle = (angle + np.pi) % (2 * np.pi) - np.pi
                    self.get_logger().info("===========================================")
                    self.get_logger().info(f"z distance: {tvec[2][0]}")
                    self.get_logger().info(f"offset: {offset} | fx: {fx} | angle: {angle}")
                    self.get_logger().info(f"corrected_angle: {corrected_angle}")
                    self.get_logger().info("============================================")

                    offsets.append(float(offset))
                    # yaws.append(float(corrected_angle))

                    # self.get_logger().info(f"{cv_image.shape} | {center_x}, {center_y}")
                    # self.get_logger().info(f"corner: {corner[0]}")
                    # self.get_logger().info(f"offset: {offset}")
                    # self.get_logger().info(f"fx: {fx}")
                    # self.get_logger().info(f"angle: {corrected_angle}")
                    # self.get_logger().info(f"aruco marker center: {marker_center}")

                    rvecs.append(rvec)
                    tvecs.append(tvec)
                else:
                    self.get_logger().warn("Pose estimation failed for one of the markers.")
                    continue

            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][1][0]
                pose.position.z = tvecs[i][2][0]
                
                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i]))[0]
                yaw = np.arctan2(rot_matrix[1,0], rot_matrix[0,0])
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                # pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.yaw_angles.append(float(yaw))
                markers.offsets.append(float(offsets[i]))
                markers.marker_ids.append(marker_id[0])

            ### For debugging in RViz
            # Draw detected markers and their IDs on the image
            cv2.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)

            # Draw axes
            for i in range(len(marker_ids)):
                cv2.drawFrameAxes(cv_image, self.intrinsic_mat, self.distortion_mat,
                                    rvecs[i], tvecs[i], self.marker_size)

        markers.header = header
        
        # Convert annotated image to ROS 2 topic
        annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
        annotated_image_msg.header = header 

        with self.markers_mutex:
            self.markers_buffer.append(markers)
        
        # Publish the annotated image
        self.aruco_img_pub.publish(annotated_image_msg)

    def setState(self, state):
        self.state = state

    def move(self, linear_velocity, angular_velocity, y_velocity=0.0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.linear.y = y_velocity
        msg.angular.z = angular_velocity
        self.twist_pub.publish(msg)
        self.get_logger().info(f"Publishing Twist message: {msg}")
        
    def navigate(self):
        with self.markers_mutex:
            now = self.get_clock().now().nanoseconds
            buffer = self.markers_buffer

            if not buffer or len(buffer) == 0:
                self.move(0.0, 0.0)
                self.setState(ARUCO_STATE.SEARCHING)
                return

            markers = buffer[-1]

            # Get the last 3 markers from the buffer
            last_3 = list(self.markers_buffer)[-3:]

            # Example: average the z distance of the first pose in each marker
            z_values = [m.poses[0].position.z for m in last_3 if len(m.poses) > 0]
            if z_values:
                avg_z = sum(z_values) / len(z_values)
            else:
                avg_z = None  # or handle as appropriate
            
            yaw_values = [m.yaw_angles[0] for m in last_3 if len(m.yaw_angles) > 0]
            if yaw_values:
                avg_yaw = sum(yaw_values) / len(yaw_values)
            else:
                avg_yaw = None

            # self.get_logger().info(f"Buffer: {buffer}")
            
            # self.get_logger().info(f"Current time: {now*1e-9}")
            # self.get_logger().info(f"Buffer timestamps: {[p.header.stamp for p in buffer]}")
            # self.get_logger().info(f"Timestamp subtractions: {[((now * 1e-9) - (p.header.stamp.sec + (p.header.stamp.nanosec*1e-9))) for p in buffer]}")
            # valid_markers = [
            #     p for p in buffer
            #     if ((now * 1e-9) - (p.header.stamp.sec + (p.header.stamp.nanosec*1e-9))) < self.MAX_STALE_TIME
            # ]

            # self.get_logger().info(f"Valid markers: {valid_markers}")

            # if len(valid_markers) == 0:
            #     self.move(0.0, 0.0)
            #     self.setState(ARUCO_STATE.SEARCHING)
            #     return
            
            # markers = valid_markers[-1]

            # self.get_logger().info(f"Most recent marker: {markers}")

        # check if markers were detected
        if not markers:
            return

        # self.get_logger().info(f"Markers: {markers}")
        
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

                        yaw = avg_yaw

                        self.get_logger().info(f"Yaw is {yaw}")

                        if abs(yaw) < 0.1:
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
                yaw = avg_yaw

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

                if abs(self.err_theta) >= 0.2:
                    av = self.angular_pid()
                else:
                    av = 0.0

                if abs(self.err_dist) >= 0.3:
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

                if self.TTS_ACTIVE:
                    audio_filename = self.landmarks[self.cur_id]["label"] + ".wav"
                    self.tts_file(audio_filename)

                self.landmarks[self.cur_id]["reached"] = True
                self.cur_id += 1

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

        Kp_dist, Ki_dist, Kd_dist = 0.01, 0.04, 0.1

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

        Kp_theta, Ki_theta, Kd_theta = 0.1, 0.08, 0.1
        proportional = Kp_theta * self.err_theta

        self.integral_theta += Ki_theta * self.err_theta * dt 
        self.integral_theta = np.clip(self.integral_theta, -0.01, 0.01)

        derivative = Kd_theta * ((self.err_theta - self.previous_err_theta) / dt)
        derivative = np.clip(derivative, -0.1, 0.1)

        av = proportional + self.integral_theta + derivative
        av = np.clip(av, -0.4, 0.4)  # rad/s

        self.previous_err_theta = self.err_theta

        return av
        
    def tts_file(self, file):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        server.settimeout(15.0)
        ctrl_addr = ('192.168.1.120', 45678)

        try:
            server.bind(('192.168.1.65', 54321))
            
            filename = bytes(file, 'utf-8')
            server.sendto(filename, ctrl_addr)

            msg, addr = server.recvfrom(1024)
            result = msg.decode('utf-8')
            return result
        except socket.timeout:
            self.get_logger().info("Socket timeout")
            server.close()
            return "Good"
        finally:
            server.close()

def main():
    rclpy.init()
    tts = RS_Aruco_TTS()

    try:
        rclpy.spin(tts)
    except KeyboardInterrupt:
        pass
    finally:
        tts.shutdown_hook()
        tts.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
