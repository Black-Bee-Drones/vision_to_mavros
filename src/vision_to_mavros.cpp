#include <vision_to_mavros/vision_to_mavros.hpp>

VisionToMavros::VisionToMavros() : Node("vision_to_mavros_node") {
    // Create a tf2_ros Buffer and TransformListener
    buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    
    this->navigationParameters();
    this->precisionLandParameters();
}


void VisionToMavros::navigationParameters(void) {
    camera_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("vision_pose", 10);
    body_path_publisher = this->create_publisher<nav_msgs::msg::Path>("body_frame/path", 1);
    
    // The frame in which we find the transform into, the original "world" frame
    this->declare_parameter<std::string>("target_frame_id", "/camera_odom_frame");
    this->get_parameter("target_frame_id", target_frame_id);

    // The frame for which we find the tranform to target_frame_id, the original "camera" frame
    this->declare_parameter<std::string>("source_frame_id", "/camera_link");
    this->get_parameter("source_frame_id", source_frame_id);

        // The rate at which we wish to publish final pose data
    this->declare_parameter<double>("output_rate", 20.0);
    this->get_parameter("output_rate", output_rate);
    
    // The rotation around z axis between original world frame and target world frame, assuming the z axis needs not to be changed
    // In this case, target world frame has y forward, x to the right and z upwards (ENU as ROS dictates)
    this->declare_parameter<double>("gamma_world", -1.5707963);
    this->get_parameter("gamma_world", gamma_world);

    // The pitch angle around camera's own axis to align with body frame
    this->declare_parameter<double>("roll_cam", 0.0);
    this->get_parameter("roll_cam", roll_cam);
    
    // The roll angle around camera's own axis to align with body frame 
    this->declare_parameter<double>("pitch_cam", 0.0);
    this->get_parameter("pitch_cam", pitch_cam);
    
    // The yaw angle around camera's own axis to align with body frame 
    this->declare_parameter<double>("yaw_cam", 1.5707963);
    this->get_parameter("yaw_cam", yaw_cam);

    RCLCPP_INFO(this->get_logger(), "Get target_frame_id parameter: %s", target_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Get source_frame_id parameter: %s", source_frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Get output_rate parameter: %f", output_rate);
    RCLCPP_INFO(this->get_logger(), "Get gamma_world parameter: %f", gamma_world);
    RCLCPP_INFO(this->get_logger(), "Get roll_cam parameter: %f", roll_cam);
    RCLCPP_INFO(this->get_logger(), "Get pitch_cam parameter: %f", pitch_cam);
    RCLCPP_INFO(this->get_logger(), "Get yaw_cam parameter: %f", yaw_cam);
}

void VisionToMavros::precisionLandParameters(void) {
    this->declare_parameter<bool>("enable_precland", false);
    this->get_parameter("enable_precland", enable_precland);

    RCLCPP_INFO(this->get_logger(), "Precision landing: %s", enable_precland ? "enabled" : "disabled");

    if (enable_precland) {
        this->declare_parameter<std::string>("precland_target_frame_id", "/landing_target");
        this->get_parameter("precland_target_frame_id", precland_target_frame_id);
        this->declare_parameter<std::string>("precland_camera_frame_id", "/camera_fisheye2_optical_frame");
        this->get_parameter("precland_camera_frame_id", precland_camera_frame_id);

        RCLCPP_INFO(this->get_logger(), "Get precland_target_frame_id parameter: %s", precland_target_frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "Get precland_camera_frame_id parameter: %s", precland_camera_frame_id.c_str());

        precland_msg_publisher = this->create_publisher<mavros_msgs::msg::LandingTarget>("landing_raw", 10);
    }

}

void VisionToMavros::transformReady(const std::shared_future<geometry_msgs::msg::TransformStamped>& transform) {
    // Callback for when the transform is ready
    RCLCPP_INFO(this->get_logger(), "Transform result: %f %f %f", transform.get().transform.translation.x, transform.get().transform.translation.y, transform.get().transform.translation.z);
}

bool VisionToMavros::waitForFirstTransform(void) {
    // Wait for the first transform to be available
    bool received = false;
    std::string* error_msg = new std::string;
    auto start_time = this->now();
    while (rclcpp::ok())
    {
      if (buffer->canTransform(target_frame_id, source_frame_id, tf2::TimePointZero, error_msg))
      {
        received = true;
        break;
      }
      else if (this->now() - start_time > rclcpp::Duration::from_seconds(10)) // 10 seconds timeout
      {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for transform...");
        received = false;
        break;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Error mensage: %s", error_msg->c_str());
        RCLCPP_INFO(this->get_logger(), "Waiting for transform...");
        rclcpp::sleep_for(std::chrono::seconds(1));
      }
    }
    delete error_msg;
    return received;
}

void VisionToMavros::run(void) {
    RCLCPP_INFO(this->get_logger(), "Running Vision To Mavros");

    // Set a timeout duration
    rclcpp::Duration timeout(3, 0); 
    RCLCPP_INFO(this->get_logger(), "Waiting for transform between %s and %s", target_frame_id.c_str(), source_frame_id.c_str());

    if (!this->waitForFirstTransform()) return;

    this->last_tf_time = this->get_clock()->now();

    auto timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / this->output_rate)),
        std::bind(&VisionToMavros::publishVisionPositionEstimate, this)
    );
        
    rclcpp::spin(this->shared_from_this());
    
    // rclcpp::Rate loopRate(this->output_rate);

    // while (rclcpp::ok())
    // {
    //     this->publishVisionPositionEstimate();
        
    //     rclcpp::spin_some(this->shared_from_this());
    //     loopRate.sleep();
    // }
}

void VisionToMavros::publishVisionPositionEstimate() {
    // Publish vision_position_estimate message if transform is available
    try {
        transform_stamped = buffer->lookupTransform(target_frame_id, source_frame_id, tf2::TimePointZero);

        // Only publish pose messages when we have new transform data.
        if (last_tf_time < transform_stamped.header.stamp) {
            last_tf_time = transform_stamped.header.stamp;

            // 1) Rotation from original world frame to world frame with y forward.
            tf2::fromMsg(transform_stamped.transform.translation, position_orig);

            position_body.setX(cos(gamma_world) * position_orig.getX() + sin(gamma_world) * position_orig.getY());
            position_body.setY(-sin(gamma_world) * position_orig.getX() + cos(gamma_world) * position_orig.getY());
            position_body.setZ(position_orig.getZ());

            // 2) Rotation from camera to body frame.
            tf2::fromMsg(transform_stamped.transform.rotation, quat_cam);

            quat_cam_to_body_x.setRPY(roll_cam, 0, 0);
            quat_cam_to_body_y.setRPY(0, pitch_cam, 0);
            quat_cam_to_body_z.setRPY(0, 0, yaw_cam);

            // 3) Rotate body frame 90 degree (align body x with world y at launch)
            quat_rot_z.setRPY(0, 0, -gamma_world);

            quat_body = quat_rot_z * quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
            quat_body.normalize();

            // Create PoseStamped message to be sent
            msg_body_pose.header.stamp = transform_stamped.header.stamp;
            msg_body_pose.header.frame_id = transform_stamped.header.frame_id;
            msg_body_pose.pose.position.x = position_body.getX();
            msg_body_pose.pose.position.y = position_body.getY();
            msg_body_pose.pose.position.z = position_body.getZ();
            msg_body_pose.pose.orientation.x = quat_body.getX();
            msg_body_pose.pose.orientation.y = quat_body.getY();
            msg_body_pose.pose.orientation.z = quat_body.getZ();
            msg_body_pose.pose.orientation.w = quat_body.getW();

            // Publish pose of body frame in world frame
            camera_pose_publisher->publish(msg_body_pose);

            // Publish trajectory path for visualization
            body_path.header.stamp = msg_body_pose.header.stamp;
            body_path.header.frame_id = msg_body_pose.header.frame_id;
            body_path.poses.push_back(msg_body_pose);
            body_path_publisher->publish(body_path);
        }
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
    }
}

// Main function
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionToMavros>();
    node->run();
    rclcpp::shutdown();
    return 0;
}