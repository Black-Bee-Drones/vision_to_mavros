from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url", default="/dev/ttyTHS1:921600")
    gcs_url = LaunchConfiguration("gcs_url", default="")
    tgt_system = LaunchConfiguration("tgt_system", default="1")
    tgt_component = LaunchConfiguration("tgt_component", default="1")
    namespace = LaunchConfiguration("namespace", default="mavros")

    node_launch_file = os.path.join(
        FindPackageShare("mavros").find("mavros"),
        "launch",
        "node.launch"
    )

    vision_to_mavros_share_dir = get_package_share_directory('vision_to_mavros')
    indoor_pluginlists_file = os.path.join(vision_to_mavros_share_dir, 'config', 'indoor_pluginlists.yaml')
    indoor_config_file = os.path.join(vision_to_mavros_share_dir, 'config', 'indoor_config.yaml')

    # Relay Node - Bridge Visual SLAM pose to MAVROS
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='vslam_to_mavros_relay',
        output='screen',
        arguments=[
            '/visual_slam/tracking/vo_pose_covariance',  # Source topic from Visual SLAM
            '/mavros/vision_pose/pose_cov'  # Target topic for MAVROS
        ],
        parameters=[{
            'lazy': False,  # Always relay, don't wait for subscribers
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value=fcu_url),
        DeclareLaunchArgument('gcs_url', default_value=gcs_url),
        DeclareLaunchArgument('tgt_system', default_value=tgt_system),
        DeclareLaunchArgument('tgt_component', default_value=tgt_component),
        DeclareLaunchArgument('namespace', default_value=namespace),

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(node_launch_file),
            launch_arguments={
                "pluginlists_yaml": indoor_pluginlists_file,
                "config_yaml": indoor_config_file,
                "fcu_url": fcu_url,
                "gcs_url": gcs_url,
                "tgt_system": tgt_system,
                "tgt_component": tgt_component,
                "namespace": namespace,
            }.items()
        ),

        relay_node
    ])
