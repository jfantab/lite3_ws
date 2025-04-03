import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # declare_rviz_argument = DeclareLaunchArgument(
    #     'rviz_condition',
    #     default_value='true',
    #     description='Whether to launch RViz'
    # )

    # rviz_condition = LaunchConfiguration('rviz_condition')

    # rviz_config_path = os.path.join(
    #     get_package_share_directory('lite3_bringup'),
    #     'launch',
    #     'aruco.rviz'
    # )

    rs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch',
            'rs_launch.py'
        )),
        launch_arguments={
            # 'align_depth.enable': 'false', 
            # 'enable_sync': 'false',
            # 'enable_depth': 'false'
            'color_fps': "15"
        }.items()
    )

    aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_aruco'),
                'launch',
                'aruco_recognition.launch.py'
            )
        )
    )

    waypoint = Node(
        name="waypoint_tts",
        package="waypoint_tts",
        executable="tts"
    )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path],
    #     condition=IfCondition(rviz_condition)
    # )

    return LaunchDescription([
        # declare_rviz_argument,
        # rviz,
        rs,
        aruco, 
        waypoint
    ])
