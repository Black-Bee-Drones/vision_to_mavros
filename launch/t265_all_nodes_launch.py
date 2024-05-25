from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Configurações de parâmetros
    fcu_url = LaunchConfiguration("fcu_url", default="serial:///dev/ttyUSB0:57600")
    usb_port_id = LaunchConfiguration("usb_port_id", default="2-2")
    camera_name = LaunchConfiguration("camera_name", default="T265")
    device_type = LaunchConfiguration("device_type", default="t265")
    enable_fisheye1 = LaunchConfiguration("enable_fisheye1", default="false")
    enable_fisheye2 = LaunchConfiguration("enable_fisheye2", default="false")
    enable_gyro = LaunchConfiguration("enable_gyro", default="true")
    enable_accel = LaunchConfiguration("enable_accel", default="true")
    enable_pose = LaunchConfiguration("enable_pose", default="pose")

    # Diretório do pacote
    mavros_launch_file = os.path.join(
        FindPackageShare("mavros").find("mavros"), "launch", "apm.launch.py"
    )

    realsense_launch_file = os.path.join(
        FindPackageShare("realsense2_camera").find("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )

    vision_to_mavros_launch_file = os.path.join(
        FindPackageShare("vision_to_mavros").find("vision_to_mavros"),
        "launch",
        "t265_tf_to_mavros_launch.py",
    )

    return LaunchDescription(
        [
            # This node will launch MAVROS
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mavros_launch_file),
                launch_arguments={"fcu_url": fcu_url}.items(),
            ),
            # This node will launch the ROS driver for Realsense T265
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(realsense_launch_file),
                launch_arguments={
                    "usb_port_id": usb_port_id,
                    "camera_name": camera_name,
                    "device_type": device_type,
                    "enable_fisheye1": enable_fisheye1,
                    "enable_fisheye2": enable_fisheye2,
                    "enable_gyro": enable_gyro,
                    "enable_accel": enable_accel,
                    "enable_pose": enable_pose,
                }.items(),
            ),
            # This node will launch the node that bridges realsense-ros and MAVROS
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vision_to_mavros_launch_file)
            ),
        ]
    )
