"""
This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_poses (geometry_msgs.msg.PoseArray)
       Pose of all detected markers (suitable for rviz visualization)

    /aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
       Provides an array of all poses along with the corresponding
       marker ids.

Parameters:
    marker_size - size of the markers in meters (default .0625)
    aruco_dictionary_id - dictionary that was used to generate markers
                          (default DICT_5X5_250)
    image_topic - image topic to subscribe to (default /camera/image_raw)
    camera_info_topic - camera info topic to subscribe to
                         (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf_transformations
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64, Float64MultiArray
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruco_node")

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

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="depth_image_topic",
            value="/camera/camera/aligned_depth_to_color/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Aligned depth and RGB map.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
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

        image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image topic: {image_topic}")

        info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Image info topic: {info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )

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

        # Set up subscriptions
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )

        self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)
        self.aruco_img_pub = self.create_publisher(Image, "aruco_image", 10)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # self.aruco_dictionary = cv2.aruco.Dictionary(dictionary_id, int(self.marker_size))
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)
        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
        # cv_image = cv2.undistort(cv_image, self.intrinsic_mat, self.distortion)

        height, width = cv_image.shape

        center_x = height // 2
        center_y = width // 2

        markers = ArucoMarkers()
        pose_array = PoseArray()
        
        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        corners, marker_ids, rejected = self.detector.detectMarkers(cv_image)

        if marker_ids is not None:
            # Define marker coordinate system
            marker_length = 0.150  # in meters (match your actual marker size)
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
                    self.distortion
                )
                    
                if success:
                    # Get center
                    marker_center = [np.sum(corner[0][:,0]) // 4, np.sum(corner[0][:,1]) // 4]
                    marker_centers.append(marker_center)

                    # Calculate the yaw
                    offset = marker_center[0] - center_x
                    fx = self.intrinsic_mat[0][0]
                    angle = np.arctan2(offset, fx)
                    corrected_angle = (angle + np.pi) % (2 * np.pi) - np.pi
                    
                    offsets.append(float(offset))
                    yaws.append(float(corrected_angle))

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
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.yaw_angles.append(float(yaws[i]))
                markers.offsets.append(float(offsets[i]))
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

            # Draw detected markers and their IDs on the image
            cv2.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)

            # Draw axes
            for i in range(len(marker_ids)):
                cv2.drawFrameAxes(cv_image, self.intrinsic_mat, self.distortion,
                                rvecs[i], tvecs[i], self.marker_size)

            # Convert annotated OpenCV image back to ROS2 Image message
            annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="mono8")
            annotated_image_msg.header = img_msg.header

            # Publish the annotated image
            self.aruco_img_pub.publish(annotated_image_msg)

            # Publish pose and markers
            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)

def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
