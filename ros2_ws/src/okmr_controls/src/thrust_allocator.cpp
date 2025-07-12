#include "okmr_controls/thrust_allocator.hpp"
#include <Eigen/Dense>
#include "okmr_msgs/msg/control_mode.hpp"

namespace okmr_controls
{

ThrustAllocator::ThrustAllocator()
    : Node("thrust_allocator"), is_enabled_(false), current_control_mode_(okmr_msgs::msg::ControlMode::OFF)
{
    // Subscribe to wrench target
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/wrench_target", 10,
        std::bind(&ThrustAllocator::wrench_target_callback, this, std::placeholders::_1));
    
    // Publisher for motor thrust
    motor_thrust_pub_ = this->create_publisher<okmr_msgs::msg::MotorThrust>("/motor_thrust", 10);
    
    // Subscribe to control mode
    control_mode_sub_ = this->create_subscription<okmr_msgs::msg::ControlMode>(
        "/control_mode", 10,
        std::bind(&ThrustAllocator::control_mode_callback, this, std::placeholders::_1));
    
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
    for (int i = 0; i < 8; ++i) {
        motor_thrust_msg.thrust[i] = motor_thrusts(i);
    }
    
    motor_thrust_pub_->publish(motor_thrust_msg);
}

void ThrustAllocator::control_mode_callback(const okmr_msgs::msg::ControlMode::SharedPtr msg)
{
    int8_t new_mode = msg->control_mode;
    
    if (new_mode != current_control_mode_)
    {
        bool was_enabled = is_enabled_;
        current_control_mode_ = new_mode;
        
        // Enable thrust allocator for THRUST control mode only
        is_enabled_ = (current_control_mode_ <= okmr_msgs::msg::ControlMode::THRUST);
        
        if (was_enabled && !is_enabled_)
        {
            RCLCPP_INFO(this->get_logger(), "Thrust allocator disabled, sending zero thrust");
            
            // Create and publish "off" motor thrust message
            okmr_msgs::msg::MotorThrust motor_thrust_msg;
            motor_thrust_msg.header.stamp = this->now();
            motor_thrust_msg.header.frame_id = "base_link";
        
            // Assign thrust values to motors array
            for (int i = 0; i < 8; ++i) {
                motor_thrust_msg.thrust[i] = 0.0;
            }
        
            motor_thrust_pub_->publish(motor_thrust_msg);
        }
        else if (!was_enabled && is_enabled_)
        {
            RCLCPP_INFO(this->get_logger(), "Thrust allocator enabled");
        }
    }
    else
    {
        RCLCPP_DEBUG(this->get_logger(), "Control mode unchanged: %d", new_mode);
    }
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
    // Compute thin SVD using modern Eigen API
    Eigen::BDCSVD<Eigen::MatrixXf, Eigen::ComputeThinU | Eigen::ComputeThinV> svd(A);

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
