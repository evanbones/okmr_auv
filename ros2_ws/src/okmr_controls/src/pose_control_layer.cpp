#include "okmr_controls/pose_control_layer.hpp"
#include <okmr_msgs/msg/control_mode.hpp>

namespace okmr_controls
{

PoseControlLayer::PoseControlLayer()
    : ControlLayerBase("pose_control_layer", okmr_msgs::msg::ControlMode::POSE)
{
    // Subscribe to pose target topic
    pose_target_sub_ = this->create_subscription<okmr_msgs::msg::RelativePose>(
        "/relative_pose_target", 10,
        std::bind(&PoseControlLayer::pose_target_callback, this, std::placeholders::_1));
    
    // Publisher for velocity target
    velocity_target_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/velocity_target", 10);
    
    // Initialize pose messages - current pose is always zero (relative pose origin)
    current_pose_.translation.x = current_pose_.translation.y = current_pose_.translation.z = 0.0;
    current_pose_.rotation.x = current_pose_.rotation.y = current_pose_.rotation.z = 0.0;
    
    pose_target_.translation.x = pose_target_.translation.y = pose_target_.translation.z = 0.0;
    pose_target_.rotation.x = pose_target_.rotation.y = pose_target_.rotation.z = 0.0;
}

void PoseControlLayer::pose_target_callback(const okmr_msgs::msg::RelativePose::SharedPtr msg)
{
    pose_target_ = *msg;
}

void PoseControlLayer::update()
{
    if( !is_enabled_ ) return;

    // Calculate pose errors - current pose is always zero
    geometry_msgs::msg::Vector3 linear_error;
    geometry_msgs::msg::Vector3 angular_error;
    
    linear_error.x = pose_target_.translation.x;
    linear_error.y = pose_target_.translation.y;
    linear_error.z = pose_target_.translation.z;
    
    angular_error.x = pose_target_.rotation.x;
    angular_error.y = pose_target_.rotation.y;
    angular_error.z = pose_target_.rotation.z;
    
    // Compute control commands
    auto command_output = compute_layer_command(linear_error, angular_error);
    
    // Create and publish velocity target message
    geometry_msgs::msg::TwistStamped velocity_target;
    velocity_target.header.stamp = this->now();
    velocity_target.header.frame_id = "base_link";
    velocity_target.twist.linear = command_output.first;
    velocity_target.twist.angular = command_output.second;
    
    velocity_target_pub_->publish(velocity_target);
}

}  // namespace okmr_controls

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<okmr_controls::PoseControlLayer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
