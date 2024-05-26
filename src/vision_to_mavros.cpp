#include <vision_to_mavros/vision_to_mavros.hpp>

/**
 * @brief Constructor for the VisionToMavros class.
 * @details Initializes the node, creates a tf2_ros Buffer and TransformListener, and sets up the timer interface.
 * Also calls methods to initialize navigation and precision landing parameters.
 */
VisionToMavros::VisionToMavros() : Node("vision_to_mavros_node") {
    // Create a tf2_ros Buffer and TransformListener
    buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    // Create a timer interface and set it on the buffer
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    buffer->setCreateTimerInterface(timer_interface);
    
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

    this->navigationParameters();
    this->precisionLandParameters();
}


/**
 *   @brief Reads the parameters from the parameter server and sets the class variables accordingly. 
 *   @details Parameters are:
 *   - target_frame_id: The frame in which we find the transform into, the original "world" frame
 *   - source_frame_id: The frame for which we find the tranform to target_frame_id, the original "camera" frame
 *   - output_rate: The rate at which we wish to publish final pose data
 *   - gamma_world: The rotation around z axis between original world frame and target world frame, assuming the z axis needs not to be changed
 *   - roll_cam: The pitch angle around camera's own axis to align with body frame
 *   - pitch_cam: The roll angle around camera's own axis to align with body frame
 *   - yaw_cam: The yaw angle around camera's own axis to align with body frame
 */
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

/**
 * @brief Reads the precision landing parameters from the parameter server and sets the class variables accordingly.
 * @details Parameters are:
 * - enable_precland: Flag to enable or disable precision landing.
 * - precland_target_frame_id: The frame of the landing target.
 * - precland_camera_frame_id: The frame of the camera used for precision landing.
 */
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

/**
 * @brief Waits for the first transform to be available.
 * @param timeout The maximum time to wait for the transform in seconds. Default is 12.0 seconds.
 * @return true if the transform is received within the timeout period, false otherwise.
 * @details This method attempts to obtain the transform between the target and source frames
 * within the specified timeout period. It checks for the transform at a rate of 3 Hz and logs
 * any errors or warnings encountered during the process.
 */
bool VisionToMavros::waitForFirstTransform(double timeout=12.0) {
    // Wait for the first transform to be available
    bool received = false;
    std::string error_msg;
    auto start_time = this->now();

    RCLCPP_INFO(this->get_logger(), "Waiting for transform between %s and %s", target_frame_id.c_str(), source_frame_id.c_str());

    rclcpp::Rate rate(3.0);  // Check at a rate of 3 Hz
    while (rclcpp::ok() && (this->now() - start_time < rclcpp::Duration::from_seconds(timeout))) {
        if (buffer->canTransform(target_frame_id, source_frame_id, this->get_clock()->now(), rclcpp::Duration::from_seconds(3.0), &error_msg)) {
            received = true;
            break;
        } else {
            RCLCPP_WARN(this->get_logger(), "Error message: %s", error_msg.c_str());
            RCLCPP_INFO(this->get_logger(), "Waiting for transform...");
        }
        rate.sleep();
    }

    if (!received) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for transform after %.1f seconds.", timeout);
    }

    return received;
}

/**
 * @brief Starts the main processing loop of the VisionToMavros node.
 * @details This method initializes the timer to periodically call the publishVisionPositionEstimate method
 * at the specified output rate. It waits for the first transform to be available before starting the timer.
 * If the transform is not received within the timeout period, the method returns early. 
 * The method spins the node to keep it alive and processes callbacks.
 */
void VisionToMavros::run(void) {
    RCLCPP_INFO(this->get_logger(), "Running Vision To Mavros");
    
    double timeout = 12.0;
    
    if (!this->waitForFirstTransform(timeout)) return;

    // auto future = buffer->waitForTransform(target_frame_id, source_frame_id, tf2::TimePointZero, tf2::durationFromSec(timeout), std::bind(&VisionToMavros::transformReady, this, std::placeholders::_1));

    // if (future.wait_for(std::chrono::seconds(static_cast<int64_t>(timeout))) == std::future_status::timeout) {
    //     RCLCPP_ERROR(this->get_logger(), "Transformation not available after waiting for 3 seconds.");
    //     return;
    // }

    RCLCPP_INFO(this->get_logger(), "First transform is received");

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

/**
 * @brief Publishes the vision position estimate if the transform is available.
 * @details This method retrieves the latest available transform between the target and source frames.
 * If a new transform is received, it calculates the position and orientation in the body frame,
 * creates a PoseStamped message, and publishes it. It also updates the trajectory path for visualization.
 * Any exceptions encountered during the transform lookup are logged as warnings.
 */
void VisionToMavros::publishVisionPositionEstimate() {
    // Publish vision_position_estimate message if transform is available
    try {
        // For tf2, TimePointZero is a special value that means "latest available transform"
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