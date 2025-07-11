#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "okmr_msgs/msg/goal_velocity.hpp"
#include "okmr_msgs/msg/control_mode.hpp"
#include "okmr_msgs/srv/distance_from_goal.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class VelocityTargetServer : public rclcpp::Node
{
public:
    VelocityTargetServer() : Node("velocity_target_server")
    {
        // Declare parameters
        this->declare_parameter("update_frequency", 100.0);
        update_frequency_ = this->get_parameter("update_frequency").as_double();

        // Parameter callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&VelocityTargetServer::on_parameter_change, this, std::placeholders::_1));

        // Subscriptions
        goal_velocity_subscription_ = this->create_subscription<okmr_msgs::msg::GoalVelocity>(
            "/goal_velocity", 10,
            std::bind(&VelocityTargetServer::goal_velocity_callback, this, _1));

        actual_velocity_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/velocity", 10,
            std::bind(&VelocityTargetServer::actual_velocity_callback, this, _1));

        // Create separate callback groups
        control_mode_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // Subscribe to control mode with separate callback group
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = control_mode_callback_group_;
        
        control_mode_subscription_ = this->create_subscription<okmr_msgs::msg::ControlMode>(
            "/control_mode", 10,
            std::bind(&VelocityTargetServer::control_mode_callback, this, _1),
            sub_options);

        // Publisher for velocity target
        velocity_target_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/velocity_target", 10);

        // Service for distance from velocity goal
        distance_service_ = this->create_service<okmr_msgs::srv::DistanceFromGoal>(
            "distance_from_velocity_goal",
            std::bind(&VelocityTargetServer::distance_from_velocity_goal_callback, this, _1, _2));

        // Timer for regular updates
        auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / update_frequency_));
        
        timer_ = this->create_wall_timer(
            timer_period, std::bind(&VelocityTargetServer::update, this),
            timer_callback_group_);

        RCLCPP_INFO(this->get_logger(), "VelocityTargetServer initialized");
    }

private:
    void goal_velocity_callback(const okmr_msgs::msg::GoalVelocity::SharedPtr msg)
    {
        current_goal_velocity_ = *msg;
        has_active_goal_ = true;
        goal_start_time_ = this->now();
        last_update_time_ = this->now();
        
        // Reset integration counters
        integrated_distance_ = geometry_msgs::msg::Vector3();
        integrated_angle_ = geometry_msgs::msg::Vector3();
    }

    void actual_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        current_actual_velocity_ = msg->twist;
    }

    void control_mode_callback(const okmr_msgs::msg::ControlMode::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "VelocityTargetServer: Control mode callback received: %d", msg->control_mode);
        
        bool was_enabled = is_enabled_;
        is_enabled_ = (msg->control_mode == okmr_msgs::msg::ControlMode::VELOCITY);
        
        RCLCPP_INFO(this->get_logger(), "VelocityTargetServer: enabled: %s -> %s", 
                    was_enabled ? "true" : "false", is_enabled_ ? "true" : "false");
        
        if (was_enabled && !is_enabled_)
        {
            // Mode changed from velocity to something else - cancel timer and clear goal
            RCLCPP_INFO(this->get_logger(), "Velocity mode disabled, canceling timer");
            timer_->cancel();
            has_active_goal_ = false;
            current_goal_velocity_.twist = geometry_msgs::msg::Twist(); // Zero velocity
        }
        else if (!was_enabled && is_enabled_)
        {
            // Mode changed to velocity - start timer and set goal to zero velocity for safety
            RCLCPP_INFO(this->get_logger(), "Velocity mode enabled, starting timer");
            has_active_goal_ = false;
            current_goal_velocity_.twist = geometry_msgs::msg::Twist(); // Zero velocity
            
            auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / update_frequency_));
            
            timer_ = this->create_wall_timer(
                timer_period, std::bind(&VelocityTargetServer::update, this),
                timer_callback_group_);
        }
    }

    void distance_from_velocity_goal_callback(
        const std::shared_ptr<okmr_msgs::srv::DistanceFromGoal::Request> request,
        std::shared_ptr<okmr_msgs::srv::DistanceFromGoal::Response> response)
    {
        // Initialize response with zeros
        response->translation_differences = geometry_msgs::msg::Vector3();
        response->orientation_differences = geometry_msgs::msg::Vector3();
        
        // Only provide distance information if we have an active goal
        if (!has_active_goal_)
        {
            return;
        }
        
        // Calculate target distance from goal velocity * duration
        geometry_msgs::msg::Vector3 target_distance;
        target_distance.x = current_goal_velocity_.twist.linear.x * current_goal_velocity_.duration;
        target_distance.y = current_goal_velocity_.twist.linear.y * current_goal_velocity_.duration;
        target_distance.z = current_goal_velocity_.twist.linear.z * current_goal_velocity_.duration;
        
        // Calculate target angle from goal velocity * duration (degrees)
        geometry_msgs::msg::Vector3 target_angle;
        target_angle.x = current_goal_velocity_.twist.angular.x * current_goal_velocity_.duration;
        target_angle.y = current_goal_velocity_.twist.angular.y * current_goal_velocity_.duration;
        target_angle.z = current_goal_velocity_.twist.angular.z * current_goal_velocity_.duration;
        
        geometry_msgs::msg::Vector3 estimated_distance;
        geometry_msgs::msg::Vector3 estimated_angle;
        
        if (current_goal_velocity_.integrate)
        {
            // Use actual integrated values when integration is enabled
            estimated_distance = integrated_distance_;
            estimated_angle = integrated_angle_;
        }
        else
        {
            // Estimate distance based on requested speed * elapsed time
            auto current_time = this->now();
            auto elapsed_time = (current_time - goal_start_time_).seconds();
            
            estimated_distance.x = current_goal_velocity_.twist.linear.x * elapsed_time;
            estimated_distance.y = current_goal_velocity_.twist.linear.y * elapsed_time;
            estimated_distance.z = current_goal_velocity_.twist.linear.z * elapsed_time;
            
            estimated_angle.x = current_goal_velocity_.twist.angular.x * elapsed_time;
            estimated_angle.y = current_goal_velocity_.twist.angular.y * elapsed_time;
            estimated_angle.z = current_goal_velocity_.twist.angular.z * elapsed_time;
        }
        
        // Calculate remaining distance (target - estimated)
        response->translation_differences.x = target_distance.x - estimated_distance.x;
        response->translation_differences.y = target_distance.y - estimated_distance.y;
        response->translation_differences.z = target_distance.z - estimated_distance.z;
        
        // Calculate remaining angle (target - estimated)
        response->orientation_differences.x = target_angle.x - estimated_angle.x;
        response->orientation_differences.y = target_angle.y - estimated_angle.y;
        response->orientation_differences.z = target_angle.z - estimated_angle.z;
    }

    void update()
    {
        if (!is_enabled_)
        {
            // Don't publish anything when disabled
            return;
        }

        geometry_msgs::msg::TwistStamped velocity_target;
        velocity_target.header.stamp = this->now();
        velocity_target.header.frame_id = "base_link";
        
        if (!has_active_goal_)
        {
            // Publish zero velocity when no active goal
            velocity_target_pub_->publish(velocity_target);
            return;
        }

        auto current_time = this->now();
        auto dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        // Integrate actual velocity to track distance/angle moved
        if (dt > 0.0)
        {
            // Linear integration (meters)
            integrated_distance_.x += current_actual_velocity_.linear.x * dt;
            integrated_distance_.y += current_actual_velocity_.linear.y * dt;
            integrated_distance_.z += current_actual_velocity_.linear.z * dt;
            
            // Angular integration (degrees - system uses degrees throughout)
            integrated_angle_.x += current_actual_velocity_.angular.x * dt;
            integrated_angle_.y += current_actual_velocity_.angular.y * dt;
            integrated_angle_.z += current_actual_velocity_.angular.z * dt;
        }

        bool goal_reached = false;

        if (current_goal_velocity_.integrate)
        {
            // Check if we've reached the target distance/angle
            geometry_msgs::msg::Vector3 target_distance;
            geometry_msgs::msg::Vector3 target_angle;
            
            // Calculate target distance from goal velocity * duration
            target_distance.x = current_goal_velocity_.twist.linear.x * current_goal_velocity_.duration;
            target_distance.y = current_goal_velocity_.twist.linear.y * current_goal_velocity_.duration;
            target_distance.z = current_goal_velocity_.twist.linear.z * current_goal_velocity_.duration;
            
            // Calculate target angle from goal velocity * duration (degrees)
            target_angle.x = current_goal_velocity_.twist.angular.x * current_goal_velocity_.duration;
            target_angle.y = current_goal_velocity_.twist.angular.y * current_goal_velocity_.duration;
            target_angle.z = current_goal_velocity_.twist.angular.z * current_goal_velocity_.duration;
            
            // Check if any axis has reached its target
            if ((target_distance.x != 0.0 && std::abs(integrated_distance_.x) >= std::abs(target_distance.x)) ||
                (target_distance.y != 0.0 && std::abs(integrated_distance_.y) >= std::abs(target_distance.y)) ||
                (target_distance.z != 0.0 && std::abs(integrated_distance_.z) >= std::abs(target_distance.z)) ||
                (target_angle.x != 0.0 && std::abs(integrated_angle_.x) >= std::abs(target_angle.x)) ||
                (target_angle.y != 0.0 && std::abs(integrated_angle_.y) >= std::abs(target_angle.y)) ||
                (target_angle.z != 0.0 && std::abs(integrated_angle_.z) >= std::abs(target_angle.z)))
            {
                goal_reached = true;
            }
        }
        else
        {
            // Check if duration has been exceeded
            auto elapsed_time = (current_time - goal_start_time_).seconds();
            if (elapsed_time >= current_goal_velocity_.duration)
            {
                goal_reached = true;
            }
        }

        if (goal_reached)
        {
            // Goal reached, stop movement
            has_active_goal_ = false;
            velocity_target_pub_->publish(velocity_target); // Zero velocity
        }
        else
        {
            // Continue with goal velocity
            velocity_target.twist = current_goal_velocity_.twist;
            velocity_target_pub_->publish(velocity_target);
        }
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto& param : parameters)
        {
            if (param.get_name() == "update_frequency")
            {
                update_frequency_ = param.as_double();
                if (update_frequency_ <= 0.0)
                {
                    result.successful = false;
                    result.reason = "Update frequency must be positive";
                    return result;
                }
                
                // Update timer period
                auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(1.0 / update_frequency_));
                
                timer_->cancel();
                timer_ = this->create_wall_timer(
                    timer_period, std::bind(&VelocityTargetServer::update, this),
                    timer_callback_group_);
            }
        }
        
        return result;
    }

    // Member Variables
    double update_frequency_;
    bool is_enabled_ = false;
    bool has_active_goal_ = false;
    
    okmr_msgs::msg::GoalVelocity current_goal_velocity_;
    geometry_msgs::msg::Twist current_actual_velocity_;
    
    geometry_msgs::msg::Vector3 integrated_distance_;
    geometry_msgs::msg::Vector3 integrated_angle_;
    rclcpp::Time goal_start_time_;
    rclcpp::Time last_update_time_;
    
    rclcpp::Subscription<okmr_msgs::msg::GoalVelocity>::SharedPtr goal_velocity_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr actual_velocity_subscription_;
    rclcpp::Subscription<okmr_msgs::msg::ControlMode>::SharedPtr control_mode_subscription_;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_target_pub_;
    rclcpp::Service<okmr_msgs::srv::DistanceFromGoal>::SharedPtr distance_service_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::CallbackGroup::SharedPtr control_mode_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityTargetServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
