#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <okmr_msgs/msg/motor_thrust.hpp>
#include <Eigen/Dense>
#include <okmr_msgs/srv/enable_thrust_allocation.hpp>

namespace okmr_controls
{

class ThrustAllocator : public rclcpp::Node
{
public:
    ThrustAllocator();

private:
    void wrench_target_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
    
    void enable_service_callback(
        const std::shared_ptr<okmr_msgs::srv::EnableThrustAllocation::Request> request,
        std::shared_ptr<okmr_msgs::srv::EnableThrustAllocation::Response> response);
    
    void setup_allocation_matrix();
    Eigen::MatrixXf pseudoInverse(const Eigen::MatrixXf& A, float tol = 1e-4);
    
    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Publisher<okmr_msgs::msg::MotorThrust>::SharedPtr motor_thrust_pub_;
    rclcpp::Service<okmr_msgs::srv::EnableThrustAllocation>::SharedPtr enable_service_;
    
    // Allocation matrix and its pseudoinverse
    Eigen::MatrixXf allocation_matrix_;  // 6x8 matrix
    Eigen::MatrixXf allocation_pseudoinverse_;  // 8x6 matrix
    
    // Motor configuration parameters
    std::vector<std::vector<double>> motor_positions_;  // 8 motors, 3D position vectors
    std::vector<std::vector<double>> motor_directions_; // 8 motors, 3D thrust direction vectors
    
    // Enable state
    bool is_enabled_;
    
    // Parameter callback
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter>& parameters);
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace okmr_controls