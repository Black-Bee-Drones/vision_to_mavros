# vision_to_mavros

A ROS2 adaptation of the [vision_to_mavros package](https://github.com/thien94/vision_to_mavros) for integrating vision-based pose estimation systems with MAVROS in ROS2.

* Easy integration between vision-based systems and ArduPilot/PX4 via MAVROS
* Support for Intel® RealSense™ D435i with Isaac ROS Visual SLAM (GPU-accelerated on Jetson)
* Support for Intel® RealSense™ T265 tracking camera with built-in VIO
* Simple configuration for various camera mounting orientations (T265)
* Direct pose relay for D435i (no transformation needed)
* Transforms TF2 pose data to the frame expected by flight controllers (T265)
* Full ROS2 implementation with launch files for quick setup
* Support for precision landing capabilities

## Overview

This package provides a bridge between vision-based localization systems and flight controllers using ROS2. It supports multiple vision systems:

1. **Intel RealSense D435i with Isaac ROS Visual SLAM** - GPU-accelerated visual-inertial odometry for high-performance indoor navigation on Jetson platforms
2. **Intel RealSense T265** - Dedicated tracking camera with built-in VIO processing

The package handles coordinate frame transformations and publishes pose data in a format that can be consumed by MAVROS and subsequently by flight controllers like ArduPilot or PX4, ensuring proper alignment according to ENU (East-North-Up) conventions.

### Authors

This ROS2 adaptation is based on the original [vision_to_mavros](https://github.com/thien94/vision_to_mavros) package created by Thien Nguyen for ROS1. 

## Usage Instructions

### With Intel® RealSense™ D435i and Isaac ROS Visual SLAM

The D435i camera paired with NVIDIA Isaac ROS Visual SLAM provides GPU-accelerated visual-inertial odometry, ideal for high-performance indoor navigation on Jetson platforms.

#### Prerequisites for D435i
```bash
# Install Isaac ROS packages (in Isaac ROS Docker container or on Jetson)
sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-visual-slam
sudo apt-get install -y ros-humble-isaac-ros-examples
sudo apt-get install -y ros-humble-isaac-ros-realsense
sudo apt-get install -y ros-humble-topic-tools
```

#### Launch D435i with Visual SLAM and MAVROS
```bash
# Launch all nodes (camera, visual SLAM, MAVROS, and relay)
ros2 launch vision_to_mavros d435i_all_nodes_launch.py

# With custom parameters
ros2 launch vision_to_mavros d435i_all_nodes_launch.py \
    fcu_url:=/dev/ttyACM0:57600 \
    enable_slam_visualization:=false
```

#### D435i Configuration
The D435i is automatically configured for optimal Visual SLAM performance:
- **Infrared stereo cameras**: 640x360@90fps for visual odometry
- **IMU**: 200Hz for both gyro and accelerometer
- **IR Projector**: Disabled to avoid interference
- **Direct pose relay**: Visual SLAM output is directly relayed to MAVROS (no transformation needed)

#### Key Topics for D435i
- `/visual_slam/tracking/vo_pose_covariance` - Visual SLAM pose with covariance
- `/mavros/vision_pose/pose_cov` - Relayed pose to MAVROS

### With Intel® RealSense™ T265 Tracking Camera

The package is particularly useful when integrating the T265 tracking camera with ArduPilot or PX4. The camera provides visual-inertial odometry data through TF2 transforms, which are then processed by vision_to_mavros to provide vision-based pose estimates to the flight controller.

A typical setup requires running three nodes:

1. The T265 camera node (from realsense-ros)
2. MAVROS node for communication with the flight controller
3. vision_to_mavros node to transform the pose data

### Using the Launch File

The package includes a launch file for easy setup with the T265 camera:

```bash
ros2 launch vision_to_mavros t265_tf_to_mavros_launch.py
```

For a complete setup with all required nodes:

```bash
ros2 launch vision_to_mavros t265_all_nodes_launch.py
```

### Camera Mounting Orientation

The package supports various camera mounting orientations with appropriate configuration parameters:

**Frontfacing:**
- Forward, USB port to the right (default): `roll_cam=0.0, pitch_cam=0.0, yaw_cam=0.0, gamma_world=-1.5707963`
- Forward, USB port to the left: `roll_cam=3.1415926, pitch_cam=0.0, yaw_cam=0.0, gamma_world=-1.5707963`

**Downfacing:**
- USB port to the right: `roll_cam=0.0, pitch_cam=-1.5707963, yaw_cam=0.0, gamma_world=-1.5707963`
- USB port to the left: `roll_cam=3.1415926, pitch_cam=-1.5707963, yaw_cam=0.0, gamma_world=-1.5707963`
- USB port to the back: `roll_cam=-1.5707963, pitch_cam=-1.5707963, yaw_cam=0.0, gamma_world=-1.5707963`
- USB port to the front: `roll_cam=1.5707963, pitch_cam=-1.5707963, yaw_cam=0.0, gamma_world=-1.5707963`

**Note for downfacing orientation**: You need to tilt the vehicle's nose up slightly (not completely flat) when launching the T265 realsense-ros node, otherwise the initial yaw may be randomized (see [this issue](https://github.com/IntelRealSense/librealsense/issues/4080)). Tilting the vehicle to any other side may affect yaw stability.

## Installation Instructions

### Installation Steps

1. Create a ROS2 workspace if you don't have one:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. Clone the repository into your workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Black-Bee-Drones/vision_to_mavros.git
```

3. Build the package:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select vision_to_mavros
source install/setup.bash
```

## Configuration Parameters

### D435i Launch Arguments

The `d435i_all_nodes_launch.py` provides these parameters:

**MAVROS Arguments:**
- `fcu_url` (default: `/dev/ttyUSB0:57600`) - Flight controller connection
- `gcs_url` (default: `""`) - Ground control station connection
- `tgt_system` (default: `1`) - Target system ID
- `tgt_component` (default: `1`) - Target component ID

**Camera Arguments:**
- `camera_name` (default: `camera`) - Camera name
- `camera_namespace` (default: `camera`) - Camera namespace
- `usb_port_id` (default: `""`) - USB port ID (auto-detect if empty)
- `device_type` (default: `d435i`) - RealSense device type

**Visual SLAM Arguments:**
- `enable_slam_visualization` (default: `true`) - Enable SLAM visualization
- `enable_landmarks_view` (default: `true`) - Show tracked landmarks
- `enable_observations_view` (default: `true`) - Show feature observations

### T265 Node Parameters

The vision_to_mavros node provides these configuration parameters:

- `target_frame_id`: The frame in which we find the transform (default: `/camera_odom_frame`)
- `source_frame_id`: The frame for which we find the transform (default: `/camera_link`)
- `output_rate`: The rate at which pose data is published (default: `30.0`)
- `roll_cam`, `pitch_cam`, `yaw_cam`: Rotation angles to align camera frame with body frame
- `gamma_world`: Rotation around Z axis between world frame and target world frame

### Precision Landing Parameters

- `enable_precland`: Flag to enable precision landing (default: `false`)
- `precland_target_frame_id`: Frame ID of the landing target (default: `/landing_target`)
- `precland_camera_frame_id`: Frame ID of the camera used for precision landing (default: `/camera_fisheye2_optical_frame`)

## Published Topics

- `/mavros/vision_pose/pose` (geometry_msgs/PoseStamped): The transformed pose for the flight controller
- `/body_frame/path` (nav_msgs/Path): Visualizes the trajectory of the body frame in rviz2

## Troubleshooting

### D435i Issues

**Camera Not Detected:**
```bash
# Check if camera is connected
rs-enumerate-devices

# Check USB port
ls /dev/video*
```

**Visual SLAM Not Starting:**
```bash
# Check if Isaac ROS packages are installed
ros2 pkg list | grep isaac_ros

# Verify camera topics
ros2 topic list | grep camera
```

**No Pose Output to MAVROS:**
```bash
# Check if relay is working
ros2 topic echo /visual_slam/tracking/vo_pose_covariance
ros2 topic echo /mavros/vision_pose/pose_cov
```

### T265 Issues

**Camera Not Publishing Data:**
```bash
# Check T265 topics
ros2 topic list | grep camera

# Check TF transforms
ros2 run tf2_ros tf2_echo camera_odom_frame camera_link
```

### Performance Tips

- **For D435i**: Ensure USB 3.0 connection, disable visualization for production, monitor with `jtop` on Jetson
- **For T265**: Tilt vehicle nose up slightly when starting (for downfacing orientation)
- **Both cameras**: Ensure good lighting and textured surfaces for optimal tracking

## Additional Resources

- [NVIDIA Isaac ROS Visual SLAM](https://nvidia-isaac-ros.github.io/concepts/visual_slam/index.html)
- [ArduPilot Vision Position Estimation with T265](https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html)
- [Non-ROS Documentation for T265 with ArduPilot](https://ardupilot.org/copter/docs/common-vio-tracking-camera.html)
- [LuckyBird Tutorials](https://discuss.ardupilot.org/t/integration-of-ardupilot-and-vio-tracking-camera-part-1-getting-started-with-the-intel-realsense-t265-on-rasberry-pi-3b/43162)

