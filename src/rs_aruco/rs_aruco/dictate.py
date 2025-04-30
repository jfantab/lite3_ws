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

class Dictate_TTS(Node):

    def __init__(self):
        super().__init__("Dictate_TTS")
        # self.my_addr = ('172.20.10.6', 54321)
        self.my_addr = ('192.168.1.65', 54321)

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

        # Mutexes
        self.markers_mutex = Lock()

        # State machine parameters
        self.state = ARUCO_STATE.SEARCHING

        self.cur_id = None
        self.TTS_ACTIVE = True
        self.landmarks = [
            {
                "label": "boccardo",
                "reached": False
            },
            {
                "label": "clark",
                "reached": False
            },
            {
                "label": "dudley",
                "reached": False
            },
            {
                "label": "inter_sci",
                "reached": False
            },
            {
                "label": "library",
                "reached": False
            },
            {
                "label": "macquarrie",
                "reached": False
            },
            {
                "label": "student_union",
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

            ### Retrieve timestamp
            # current_device_time = color_frame.get_frame_metadata(rs.frame_metadata_value.sensor_timestamp) * 1e3
            # elapsed_device_time = current_device_time - self.device_time_start
            # color_frame_timestamp = self.host_time_start + elapsed_device_time

            # color_frame_sec = color_frame_timestamp // 1e9
            # color_frame_nanosec = color_frame_timestamp % 1e9
  
            ### Create header
            # header = Header()
            # header.stamp = Time(seconds=color_frame_sec, nanoseconds=color_frame_nanosec).to_msg()
            # header.frame_id = "camera_frame"
            
            # Convert to OpenCV format
            raw_image = np.asanyarray(color_frame.get_data())

            # Convert to gray
            gray = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)

            # Pass grayscale image to ArUco processing code
            self.process_image(gray)

            # Add PID controller code
            self.navigate()

            self.get_logger().info("\n\n")

        except Exception as e:
            self.get_logger().warn(f"Error encountered: {e}")

    def process_image(self, image):
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

        # markers.header = header
        
        ### Convert annotated image to ROS 2 topic
        # annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
        # annotated_image_msg.header = header 

        with self.markers_mutex:
            self.markers_buffer.append(markers)
        
        ### Publish the annotated image
        # self.aruco_img_pub.publish(annotated_image_msg)

    def navigate(self):
        self.get_logger().info("Starting navigation...")
        with self.markers_mutex:
            now = self.get_clock().now().nanoseconds
            buffer = self.markers_buffer

            self.get_logger().info(f"buffer: {buffer}")

            if not buffer or len(buffer) == 0:
                return

            markers = buffer[-1]
            self.cur_id = markers.marker_ids[-1]

        self.get_logger().info(f"markers: {markers} | markers.marker_ids[-1]: {markers.marker_ids[-1]}")
        self.get_logger().info(f"self.cur_id: {self.cur_id}")

        # check if markers were detected
        if not markers:
            return

        match self.state:
            case ARUCO_STATE.DETECTED:
                self.get_logger().info("[INFO] Detected state...")

                self.get_logger().info(f"[INFO] Moving towards detected marker...")

                # get correct marker
                index = markers.marker_ids.index(self.cur_id)
                self.get_logger().info(f"[INFO] Detected marker is ID {markers.marker_ids[index]}")

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

                if abs(depth) <= 0.4:
                    self.setState(ARUCO_STATE.GOAL)

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
        
    def tts_file(self, file):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        server.settimeout(45.0)
        ctrl_addr = ('192.168.1.120', 45678)

        try:
            server.bind(self.my_addr)
            
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
    tts = Dictate_TTS()

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
