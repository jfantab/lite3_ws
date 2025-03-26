from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():

    pub = Node(
        name="rtsp_pub",
        package="rtsp_capture",
        executable="rtsp_pub"
    )

    sub = Node(
        name="rtsp_sub",
        package="rtsp_capture",
        executable="rtsp_sub"
    )

    return LaunchDescription([pub, sub])