#ifndef VISION_TO_MAVROS_H
#define VISION_TO_MAVROS_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mavros_msgs/msg/landing_target.hpp>

#include <string>

class VisionToMavros : public rclcpp::Node {
    public:
        VisionToMavros();
        ~VisionToMavros() {}

        void run(void);

    private:
        void navigationParameters(void);
        void precisionLandParameters(void);
        void transformReady(const std::shared_future<geometry_msgs::msg::TransformStamped>&);
        bool waitForFirstTransform(void);
        void publishVisionPositionEstimate();

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr camera_pose_publisher; 
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr body_path_publisher;
        rclcpp::Publisher<mavros_msgs::msg::LandingTarget>::SharedPtr precland_msg_publisher;

        std::shared_ptr<tf2_ros::Buffer> buffer;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener;

        geometry_msgs::msg::TransformStamped transform_stamped;
        tf2::Vector3 position_orig, position_body;
        tf2::Quaternion quat_cam, quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z, quat_rot_z, quat_body;
        
        geometry_msgs::msg::PoseStamped msg_body_pose;
        nav_msgs::msg::Path body_path;

        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Time last_tf_time;


        std::string target_frame_id;
        std::string source_frame_id;
        std::string precland_target_frame_id;
        std::string precland_camera_frame_id;

        double output_rate;
        double gamma_world;
        double roll_cam;
        double pitch_cam;
        double yaw_cam;
        bool enable_precland;
};

#endif