#!/usr/bin/env python3
"""
Launch file for D435i with Isaac ROS Visual SLAM and MAVROS integration.
This launch file starts:
1. MAVROS for MAVLink communication
2. RealSense D435i camera node
3. Isaac ROS Visual SLAM node
4. Relay node to bridge visual SLAM pose to MAVROS
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
    fcu_url = LaunchConfiguration("fcu_url", default="/dev/ttyUSB0:57600")
    gcs_url = LaunchConfiguration("gcs_url", default="")
    tgt_system = LaunchConfiguration("tgt_system", default="1")
    tgt_component = LaunchConfiguration("tgt_component", default="1")
    
    # RealSense D435i arguments
    camera_name = LaunchConfiguration("camera_name", default="camera")
    camera_namespace = LaunchConfiguration("camera_namespace", default="camera")
    usb_port_id = LaunchConfiguration("usb_port_id", default="")  # Auto-detect
    device_type = LaunchConfiguration("device_type", default="d435i")
    
    # Visual SLAM arguments
    enable_slam_visualization = LaunchConfiguration("enable_slam_visualization", default="true")
    enable_landmarks_view = LaunchConfiguration("enable_landmarks_view", default="true")
    enable_observations_view = LaunchConfiguration("enable_observations_view", default="true")
    
    # ============== LAUNCH FILE PATHS ==============
    
    # MAVROS launch file
    mavros_launch_file = os.path.join(
        FindPackageShare("mavros").find("mavros"), 
        "launch", 
        "apm.launch"
    )
    
    # ============== NODES CONFIGURATION ==============
    
    # 1. RealSense D435i Camera Node
    realsense_node = Node(
        name=camera_name,
        namespace=camera_namespace,
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            # Device configuration
            'device_type': device_type,
            'usb_port_id': usb_port_id,
            
            # Stream configuration for Visual SLAM
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': False,  # Not needed for SLAM
            'enable_depth': False,  # Not needed for SLAM
            'depth_module.emitter_enabled': 0,  # Turn off IR projector
            'depth_module.profile': '640x360x90',
            
            # IMU configuration
            'enable_gyro': True,
            'enable_accel': True,
            'gyro_fps': 200,
            'accel_fps': 200,
            'unite_imu_method': 2,  # Linear interpolation
            
            # Frame IDs
            'camera_name': camera_name,
            'base_frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            
            # Other settings
            'initial_reset': False,
            'reconnect_timeout': 6.0,
            'tf_publish_rate': 0.0,  # Disable TF publishing from camera
        }],
        output='screen'
    )
    
    # 2. Isaac ROS Visual SLAM Node (Composable)
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            # Image processing
            'enable_image_denoising': False,
            'rectified_images': True,
            
            # IMU fusion settings
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'image_jitter_threshold_ms': 22.00,
            
            # Frame configuration
            'base_frame': 'camera_link',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'imu_frame': 'camera_gyro_optical_frame',
            
            # Visualization settings
            'enable_slam_visualization': enable_slam_visualization,
            'enable_landmarks_view': enable_landmarks_view,
            'enable_observations_view': enable_observations_view,
            
            # Camera optical frames
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],
            
            # SLAM settings
            'enable_localization_n_mapping': True,
            'enable_loop_closure': True,
            'enable_mapping': True,
            'enable_reading_slam_internals': True,
        }],
        remappings=[
            # Image remappings
            ('visual_slam/image_0', '/camera/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),
            ('visual_slam/image_1', '/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),
            
            # IMU remapping
            ('visual_slam/imu', '/camera/imu'),
        ],
    )
    
    # 3. Visual SLAM Container
    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded container
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )
    
    # 4. Relay Node - Bridge Visual SLAM pose to MAVROS
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
        DeclareLaunchArgument('camera_name', default_value=camera_name,
                            description='Camera name'),
        DeclareLaunchArgument('camera_namespace', default_value=camera_namespace,
                            description='Camera namespace'),
        DeclareLaunchArgument('usb_port_id', default_value=usb_port_id,
                            description='USB port ID for camera'),
        DeclareLaunchArgument('device_type', default_value=device_type,
                            description='RealSense device type'),
        DeclareLaunchArgument('enable_slam_visualization', default_value=enable_slam_visualization,
                            description='Enable SLAM visualization'),
        DeclareLaunchArgument('enable_landmarks_view', default_value=enable_landmarks_view,
                            description='Enable landmarks visualization'),
        DeclareLaunchArgument('enable_observations_view', default_value=enable_observations_view,
                            description='Enable observations visualization'),
        
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
        
        # Launch RealSense D435i node
        realsense_node,
        
        # Launch Visual SLAM container with node
        visual_slam_container,
        
        # Launch relay node to bridge Visual SLAM pose to MAVROS
        relay_node,
    ])
