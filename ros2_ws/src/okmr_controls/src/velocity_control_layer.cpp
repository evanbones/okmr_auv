#include "okmr_controls/velocity_control_layer.hpp"

#include <okmr_msgs/msg/control_mode.hpp>

namespace okmr_controls {

VelocityControlLayer::VelocityControlLayer ()
    : ControlLayerBase ("velocity_control_layer", okmr_msgs::msg::ControlMode::VELOCITY) {
    // Subscribe to velocity topics
    velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped> (
        "/velocity", 10,
        std::bind (&VelocityControlLayer::velocity_callback, this, std::placeholders::_1));

    velocity_target_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped> (
        "/velocity_target", 10,
        std::bind (&VelocityControlLayer::velocity_target_callback, this, std::placeholders::_1));

    // Publisher for acceleration target
    accel_target_pub_ =
        this->create_publisher<geometry_msgs::msg::AccelStamped> ("/accel_target", 10);

    // Initialize velocity messages
    current_velocity_.linear.x = current_velocity_.linear.y = current_velocity_.linear.z = 0.0;
    current_velocity_.angular.x = current_velocity_.angular.y = current_velocity_.angular.z = 0.0;

    velocity_target_.linear.x = velocity_target_.linear.y = velocity_target_.linear.z = 0.0;
    velocity_target_.angular.x = velocity_target_.angular.y = velocity_target_.angular.z = 0.0;
}

void VelocityControlLayer::velocity_callback (
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    current_velocity_ = msg->twist;
}

void VelocityControlLayer::velocity_target_callback (
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    velocity_target_ = msg->twist;
}

void VelocityControlLayer::update () {
    if (!is_enabled_) return;

    // Calculate velocity errors
    geometry_msgs::msg::Vector3 linear_error;
    geometry_msgs::msg::Vector3 angular_error;

    linear_error.x = velocity_target_.linear.x - current_velocity_.linear.x;
    linear_error.y = velocity_target_.linear.y - current_velocity_.linear.y;
    linear_error.z = velocity_target_.linear.z - current_velocity_.linear.z;

    angular_error.x = velocity_target_.angular.x - current_velocity_.angular.x;
    angular_error.y = velocity_target_.angular.y - current_velocity_.angular.y;
    angular_error.z = velocity_target_.angular.z - current_velocity_.angular.z;

    // Compute control commands
    auto command_output = compute_layer_command (linear_error, angular_error);

    // Create and publish acceleration target message
    geometry_msgs::msg::AccelStamped accel_target;
    accel_target.header.stamp = this->now ();
    accel_target.header.frame_id = "base_link";
    accel_target.accel.linear = command_output.first;
    accel_target.accel.angular = command_output.second;

    accel_target_pub_->publish (accel_target);
}

}  // namespace okmr_controls

int main (int argc, char** argv) {
    rclcpp::init (argc, argv);
    auto node = std::make_shared<okmr_controls::VelocityControlLayer> ();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node (node);
    executor.spin ();
    rclcpp::shutdown ();
    return 0;
}
