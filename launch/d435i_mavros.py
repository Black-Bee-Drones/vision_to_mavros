#!/usr/bin/env python3
"""
1. MAVROS for MAVLink communication
2. Relay node to bridge visual SLAM pose to MAVROS
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ============== LAUNCH ARGUMENTS ==============
    
    # MAVROS arguments
    fcu_url = LaunchConfiguration("fcu_url", default="/dev/ttyTHS1:921600")
    gcs_url = LaunchConfiguration("gcs_url", default="")
    tgt_system = LaunchConfiguration("tgt_system", default="1")
    tgt_component = LaunchConfiguration("tgt_component", default="1")

    
    # ============== LAUNCH FILE PATHS ==============
    
    # MAVROS launch file
    mavros_launch_file = os.path.join(
        FindPackageShare("mavros").find("mavros"), 
        "launch", 
        "apm.launch"
    )
    
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
    
    
    # ============== LAUNCH DESCRIPTION ==============
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('fcu_url', default_value=fcu_url,
                            description='FCU connection URL'),
        DeclareLaunchArgument('gcs_url', default_value=gcs_url,
                            description='GCS connection URL'),
        DeclareLaunchArgument('tgt_system', default_value=tgt_system,
                            description='Target system ID'),
        DeclareLaunchArgument('tgt_component', default_value=tgt_component,
                            description='Target component ID'),
        
        # Launch MAVROS
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(mavros_launch_file),
            launch_arguments={
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'tgt_system': tgt_system,
                'tgt_component': tgt_component,
            }.items(),
        ),
        
        # Launch relay node to bridge Visual SLAM pose to MAVROS
        relay_node,
    ])
