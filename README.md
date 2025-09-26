# vision_to_mavros

A ROS2 adaptation of the [vision_to_mavros package](https://github.com/thien94/vision_to_mavros) for integrating vision-based pose estimation systems with MAVROS in ROS2.

* Easy integration between vision-based systems (like Intel® RealSense™ T265) and ArduPilot/PX4 via MAVROS
* Simple configuration for various camera mounting orientations
* Transforms TF2 pose data to the frame expected by flight controllers
* Full ROS2 implementation with launch files for quick setup
* Support for precision landing capabilities

## Overview

This package provides a bridge between vision-based localization systems and flight controllers using ROS2. It subscribes to TF2 transforms provided by vision systems (like the Intel RealSense T265 tracking camera), performs the necessary frame transformations, and publishes the pose data in a format that can be consumed by MAVROS and subsequently by flight controllers like ArduPilot or PX4.

The vision_to_mavros node handles the coordinate frame transformations to convert from the vision system's frame to the frame expected by the flight controller, ensuring proper alignment of the body frame with the world frame according to ENU (East-North-Up) conventions.

### Authors

This ROS2 adaptation is based on the original [vision_to_mavros](https://github.com/thien94/vision_to_mavros) package created by Thien Nguyen for ROS1. 

## Usage Instructions

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

The node provides several configuration parameters:

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

## Additional Resources

- [ArduPilot Vision Position Estimation with T265](https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html)
- [Non-ROS Documentation for T265 with ArduPilot](https://ardupilot.org/copter/docs/common-vio-tracking-camera.html)
- [LuckyBird Tutorials](https://discuss.ardupilot.org/t/integration-of-ardupilot-and-vio-tracking-camera-part-1-getting-started-with-the-intel-realsense-t265-on-rasberry-pi-3b/43162)

