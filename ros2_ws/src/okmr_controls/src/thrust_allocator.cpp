#include "okmr_controls/thrust_allocator.hpp"
#include <Eigen/Dense>

namespace okmr_controls
{

ThrustAllocator::ThrustAllocator()
    : Node("thrust_allocator"), is_enabled_(false)
{
    // Subscribe to wrench target
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/wrench_target", 10,
        std::bind(&ThrustAllocator::wrench_target_callback, this, std::placeholders::_1));
    
    // Publisher for motor thrust
    motor_thrust_pub_ = this->create_publisher<okmr_msgs::msg::MotorThrust>("/motor_thrust", 10);
    
    // Enable service
    enable_service_ = this->create_service<okmr_msgs::srv::EnableThrustAllocation>(
        "/thrust_allocator/enable",
        std::bind(&ThrustAllocator::enable_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Declare motor configuration parameters
    // Each motor has position (x,y,z) and thrust direction (x,y,z)
    for (int i = 0; i < 8; ++i)
    {
        std::string motor_name = "motor_" + std::to_string(i);
        
        // Position vector from center of mass [x, y, z]
        this->declare_parameter(motor_name + "_position", std::vector<double>{0.0, 0.0, 0.0});
        
        // Thrust direction vector [x, y, z]
        this->declare_parameter(motor_name + "_direction", std::vector<double>{0.0, 0.0, 1.0});
    }
    
    // Get initial parameter values
    motor_positions_.resize(8);
    motor_directions_.resize(8);
    
    for (int i = 0; i < 8; ++i)
    {
        std::string motor_name = "motor_" + std::to_string(i);
        motor_positions_[i] = this->get_parameter(motor_name + "_position").as_double_array();
        motor_directions_[i] = this->get_parameter(motor_name + "_direction").as_double_array();
    }
    
    // Setup allocation matrix
    setup_allocation_matrix();
    
    // Setup parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ThrustAllocator::parameters_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Thrust allocator initialized. Use enable service to start.");
}

void ThrustAllocator::wrench_target_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    if (!is_enabled_) return;
    
    // Convert wrench to Eigen vector [Fx, Fy, Fz, Tx, Ty, Tz]
    Eigen::VectorXf wrench_vector(6);
    wrench_vector(0) = msg->wrench.force.x;
    wrench_vector(1) = msg->wrench.force.y;
    wrench_vector(2) = msg->wrench.force.z;
    wrench_vector(3) = msg->wrench.torque.x;
    wrench_vector(4) = msg->wrench.torque.y;
    wrench_vector(5) = msg->wrench.torque.z;
    
    // Calculate motor thrusts using pseudoinverse
    Eigen::VectorXf motor_thrusts = allocation_pseudoinverse_ * wrench_vector;
    
    // Create and publish motor thrust message
    okmr_msgs::msg::MotorThrust motor_thrust_msg;
    motor_thrust_msg.header.stamp = this->now();
    motor_thrust_msg.header.frame_id = "base_link";
    
    // Assign thrust values to motors array
    motor_thrust_msg.thrust.resize(8);
    for (int i = 0; i < 8; ++i) {
        motor_thrust_msg.thrust[i] = motor_thrusts(i);
    }
    
    motor_thrust_pub_->publish(motor_thrust_msg);
}

void ThrustAllocator::enable_service_callback(
    const std::shared_ptr<okmr_msgs::srv::EnableThrustAllocation::Request> request,
    std::shared_ptr<okmr_msgs::srv::EnableThrustAllocation::Response> response)
{
    is_enabled_ = request->enable;
    response->success = true;
    response->message = is_enabled_ ? "Thrust allocator enabled" : "Thrust allocator disabled";
    
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void ThrustAllocator::setup_allocation_matrix()
{
    // Initialize 6x8 allocation matrix
    allocation_matrix_ = Eigen::MatrixXf::Zero(6, 8);
    
    // Fill allocation matrix for each motor
    for (int motor = 0; motor < 8; ++motor)
    {
        // Position vector from center of mass
        Eigen::Vector3f pos(motor_positions_[motor][0], 
                           motor_positions_[motor][1], 
                           motor_positions_[motor][2]);
        
        // Thrust direction vector
        Eigen::Vector3f dir(motor_directions_[motor][0], 
                           motor_directions_[motor][1], 
                           motor_directions_[motor][2]);
        
        // Normalize thrust direction
        dir.normalize();
        
        // Force contribution (first 3 rows)
        allocation_matrix_(0, motor) = dir.x();
        allocation_matrix_(1, motor) = dir.y();
        allocation_matrix_(2, motor) = dir.z();
        
        // Torque contribution (last 3 rows) = position × thrust_direction
        Eigen::Vector3f torque = pos.cross(dir);
        allocation_matrix_(3, motor) = torque.x();
        allocation_matrix_(4, motor) = torque.y();
        allocation_matrix_(5, motor) = torque.z();
    }
    
    // Calculate pseudoinverse
    allocation_pseudoinverse_ = pseudoInverse(allocation_matrix_);
    
    RCLCPP_INFO(this->get_logger(), "Allocation matrix setup complete");
}

Eigen::MatrixXf ThrustAllocator::pseudoInverse(const Eigen::MatrixXf& A, float tol)
{
    // Compute thin SVD
    Eigen::BDCSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    const auto& singularValues = svd.singularValues();
    Eigen::VectorXf invS = singularValues;

    for (int i = 0; i < singularValues.size(); ++i)
        invS(i) = (singularValues(i) > tol) ? 1.0f / singularValues(i) : 0.0f;

    return svd.matrixV() * invS.asDiagonal() * svd.matrixU().transpose();
}

rcl_interfaces::msg::SetParametersResult ThrustAllocator::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    bool matrix_needs_update = false;
    
    for (const auto& param : parameters)
    {
        // Check if any motor configuration parameter changed
        for (int i = 0; i < 8; ++i)
        {
            std::string motor_name = "motor_" + std::to_string(i);
            if (param.get_name() == motor_name + "_position")
            {
                motor_positions_[i] = param.as_double_array();
                matrix_needs_update = true;
            }
            else if (param.get_name() == motor_name + "_direction")
            {
                motor_directions_[i] = param.as_double_array();
                matrix_needs_update = true;
            }
        }
    }
    
    // Recalculate allocation matrix if parameters changed
    if (matrix_needs_update)
    {
        setup_allocation_matrix();
        RCLCPP_INFO(this->get_logger(), "Allocation matrix updated due to parameter changes");
    }
    
    return result;
}

}  // namespace okmr_controls

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<okmr_controls::ThrustAllocator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}