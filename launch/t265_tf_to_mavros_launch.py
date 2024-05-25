from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    #  This node will launch frame conversion from vision pose (tf) to mavros pose
    """
    Two steps alignment:
    1) r,p,y: align current camera frame with default camera frame (x forward, y left, z up)
    2) gamma: align default camera frame's x axis with world y axis
    Frontfacing:
        Forward, USB port to the right (default): r = 0,          p = 0,          y = 0,  gamma = -1.5707963
        Forward, USB port to the left           : r = 3.1415926,  p = 0,          y = 0,  gamma = -1.5707963
    Downfacing: you need to tilt the vehicle's nose up a little (not flat) when launch the T265 realsense-ros node, otherwise the initial yaw will be randomized, read here: https://github.com/IntelRealSense/librealsense/issues/4080
    Tilt the vehicle to any other sides and the yaw might not be as stable.
        Downfacing, USB port to the right :       r = 0,          p = -1.5707963, y = 0,  gamma = -1.5707963
        Downfacing, USB port to the left  :       r = 3.1415926,  p = -1.5707963, y = 0,  gamma = -1.5707963
        Downfacing, USB port to the back  :       r = -1.5707963, p = -1.5707963, y = 0,  gamma = -1.5707963
        Downfacing, USB port to the front :       r = 1.5707963,  p = -1.5707963, y = 0,  gamma = -1.5707963
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_frame_id",
                default_value="/camera_odom_frame",
                description="Target frame ID",
            ),
            DeclareLaunchArgument(
                "source_frame_id",
                default_value="/camera_link",
                description="Source frame ID",
            ),
            DeclareLaunchArgument(
                "output_rate",
                default_value="30.0",
                description="Output rate",
            ),
            DeclareLaunchArgument(
                "roll_cam",
                default_value="0.0",
                description="Roll of the camera",
            ),
            DeclareLaunchArgument(
                "pitch_cam",
                default_value="0.0",
                description="Pitch of the camera",
            ),
            DeclareLaunchArgument(
                "yaw_cam",
                default_value="0.0",
                description="Yaw of the camera",
            ),
            DeclareLaunchArgument(
                "gamma_world",
                default_value="-1.5707963",
                description="Gamma of the world",
            ),
            Node(
                package="vision_to_mavros",
                executable="vision_to_mavros_node",
                name="t265_to_mavros",
                output="screen",
                parameters=[
                    {"target_frame_id": LaunchConfiguration("target_frame_id")},
                    {"source_frame_id": LaunchConfiguration("source_frame_id")},
                    {"output_rate": LaunchConfiguration("output_rate")},
                    {"roll_cam": LaunchConfiguration("roll_cam")},
                    {"pitch_cam": LaunchConfiguration("pitch_cam")},
                    {"yaw_cam": LaunchConfiguration("yaw_cam")},
                    {"gamma_world": LaunchConfiguration("gamma_world")},
                ],
                remappings=[("vision_pose", "/mavros/vision_pose/pose")],
                on_exit=Shutdown(),
            ),
        ]
    )
