#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "okmr_msgs/msg/sensor_reading.hpp"
#include "okmr_msgs/msg/dvl.hpp"
#include "okmr_msgs/srv/get_pose_twist_accel.hpp"
#include "okmr_msgs/srv/set_dead_reckoning_enabled.hpp"
#include "okmr_msgs/srv/clear_pose.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include <cmath>

typedef unsigned int uint32;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class DeadReckoningNode : public rclcpp::Node{
    public:
        // Pose estimates using Vector3
        geometry_msgs::msg::Vector3 translation_estimate;
        geometry_msgs::msg::Vector3 rotation_estimate;
        
        // Current state messages
        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::Twist current_twist;
        geometry_msgs::msg::Accel current_accel;
        
        // Cached sensor messages
        sensor_msgs::msg::Imu current_imu_msg;
        okmr_msgs::msg::Dvl current_dvl_msg;
        
        // DVL-derived acceleration
        geometry_msgs::msg::Vector3 dvl_accel;
        
        // Filtered/smoothed values using Vector3
        geometry_msgs::msg::Vector3 smoothed_angular_vel;
        geometry_msgs::msg::Vector3 prev_angular_vel; // Fixed: this should track previous angular velocity, not DVL
        
        // State tracking
        bool gotFirstTime=false;
        bool is_dead_reckoning_enabled=false;
        rclcpp::Time last_time;
        rclcpp::Time last_dvl_time;
        
        // Parameters for filtering
        double update_frequency_ = 200.0;
        double complementary_filter_alpha_ = 0.995;
        double dvl_velocity_alpha_ = 0.995;
        double dvl_accel_alpha_ = 0.3;
        double angular_vel_filter_alpha_ = 0.7;
        double angular_accel_filter_alpha_ = 0.7;

	    DeadReckoningNode() : Node("dead_reckoning_node") {
            // Declare parameters with descriptors
            rcl_interfaces::msg::ParameterDescriptor desc;
            
            desc.description = "Update frequency for state estimation timer in Hz";
            desc.floating_point_range.resize(1);
            desc.floating_point_range[0].from_value = 1.0;
            desc.floating_point_range[0].to_value = 1000.0;
            this->declare_parameter("update_frequency", update_frequency_, desc);
            
            desc.description = "Complementary filter coefficient for IMU attitude estimation (0.0=accel only, 1.0=gyro only)";
            desc.floating_point_range[0].from_value = 0.0;
            desc.floating_point_range[0].to_value = 1.0;
            this->declare_parameter("complementary_filter_alpha", complementary_filter_alpha_, desc);
            
            desc.description = "Complementary filter coefficient for DVL velocity fusion (0.0=IMU only, 1.0=DVL only)";
            desc.floating_point_range[0].from_value = 0.0;
            desc.floating_point_range[0].to_value = 1.0;
            this->declare_parameter("dvl_velocity_alpha", dvl_velocity_alpha_, desc);
            
            desc.description = "Complementary filter coefficient for DVL acceleration fusion (0.0=IMU only, 1.0=DVL only)";
            desc.floating_point_range[0].from_value = 0.0;
            desc.floating_point_range[0].to_value = 1.0;
            this->declare_parameter("dvl_accel_alpha", dvl_accel_alpha_, desc);
            
            desc.description = "Leaky integrator coefficient for angular velocity smoothing (higher=more smoothing)";
            desc.floating_point_range[0].from_value = 0.0;
            desc.floating_point_range[0].to_value = 1.0;
            this->declare_parameter("angular_vel_filter_alpha", angular_vel_filter_alpha_, desc);
            
            desc.description = "Leaky integrator coefficient for angular acceleration smoothing (higher=more smoothing)";
            desc.floating_point_range[0].from_value = 0.0;
            desc.floating_point_range[0].to_value = 1.0;
            this->declare_parameter("angular_accel_filter_alpha", angular_accel_filter_alpha_, desc);
            
            // Get parameters
            update_frequency_ = this->get_parameter("update_frequency").as_double();
            complementary_filter_alpha_ = this->get_parameter("complementary_filter_alpha").as_double();
            dvl_velocity_alpha_ = this->get_parameter("dvl_velocity_alpha").as_double();
            dvl_accel_alpha_ = this->get_parameter("dvl_accel_alpha").as_double();
            angular_vel_filter_alpha_ = this->get_parameter("angular_vel_filter_alpha").as_double();
            angular_accel_filter_alpha_ = this->get_parameter("angular_accel_filter_alpha").as_double();
            
            // Parameter callback
            param_callback_handle_ = this->add_on_set_parameters_callback(
                std::bind(&DeadReckoningNode::on_parameter_change, this, std::placeholders::_1));
            
            rclcpp::QoS qos_profile(10);  // Create QoS profile with history depth 10
            qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

		    imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("/camera/camera/imu", qos_profile, std::bind(&DeadReckoningNode::imu_callback, this, _1));
		    
		    dvl_subscription = this->create_subscription<okmr_msgs::msg::Dvl>("/dvl", 10, std::bind(&DeadReckoningNode::dvl_callback, this, _1));

		    // Publishers
	    	pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
	    	twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/velocity", 10);
	    	accel_publisher = this->create_publisher<geometry_msgs::msg::Accel>("/acceleration", 10);
            
            // Services
            get_pose_twist_accel_service = this->create_service<okmr_msgs::srv::GetPoseTwistAccel>(
                "get_pose_twist_accel", 
                std::bind(&DeadReckoningNode::get_pose_twist_accel_callback, this, std::placeholders::_1, std::placeholders::_2));
                
            set_dead_reckoning_service = this->create_service<okmr_msgs::srv::SetDeadReckoningEnabled>(
                "/set_dead_reckoning_enabled",
                std::bind(&DeadReckoningNode::set_dead_reckoning_callback, this, std::placeholders::_1, std::placeholders::_2));
                
            clear_pose_service = this->create_service<okmr_msgs::srv::ClearPose>(
                "/clear_pose",
                std::bind(&DeadReckoningNode::clear_pose_callback, this, std::placeholders::_1, std::placeholders::_2));
            
            // Timer for regular updates
            auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / update_frequency_));
            
            timer_ = this->create_wall_timer(
                timer_period, std::bind(&DeadReckoningNode::update, this));
                
            RCLCPP_INFO(this->get_logger(), "DeadReckoningNode initialized with %f Hz update rate", update_frequency_);
	    }

	private:
	void imu_callback(const sensor_msgs::msg::Imu &msg) {
        // Cache IMU data - actual processing will happen in update() method
        current_imu_msg = msg;
        if(!gotFirstTime){
            last_time = this->now();
            gotFirstTime=true;
        }
    }

	void dvl_callback(const okmr_msgs::msg::Dvl::ConstSharedPtr msg) {
        auto current_dvl_time = this->now();
        
        // Calculate DVL acceleration using DVL timing
        if (gotFirstTime && (current_dvl_time - last_dvl_time).seconds() > 0.0) {
            double dvl_dt = (current_dvl_time - last_dvl_time).seconds();
            dvl_accel.x = (msg->velocity.x - current_dvl_msg.velocity.x) / dvl_dt;
            dvl_accel.y = (msg->velocity.y - current_dvl_msg.velocity.y) / dvl_dt;
            dvl_accel.z = (msg->velocity.z - current_dvl_msg.velocity.z) / dvl_dt;
        } else {
            dvl_accel.x = dvl_accel.y = dvl_accel.z = 0.0;
        }
        
        // Cache DVL data and update timing
        current_dvl_msg = *msg;
        last_dvl_time = current_dvl_time;
        
        // Apply complementary filter to linear velocity estimate
        current_twist.linear.x = dvl_velocity_alpha_ * msg->velocity.x + (1.0 - dvl_velocity_alpha_) * current_twist.linear.x;
        current_twist.linear.y = dvl_velocity_alpha_ * msg->velocity.y + (1.0 - dvl_velocity_alpha_) * current_twist.linear.y;
        current_twist.linear.z = dvl_velocity_alpha_ * msg->velocity.z + (1.0 - dvl_velocity_alpha_) * current_twist.linear.z;
    }

    void get_pose_twist_accel_callback(const std::shared_ptr<okmr_msgs::srv::GetPoseTwistAccel::Request> request,
                          std::shared_ptr<okmr_msgs::srv::GetPoseTwistAccel::Response> response) {
        (void)request; // Unused parameter
        
        response->pose = current_pose.pose;
        response->twist = current_twist;
        response->accel = current_accel;
        response->success = gotFirstTime; // Only return success if we've received at least one IMU measurement
    }

    void set_dead_reckoning_callback(const std::shared_ptr<okmr_msgs::srv::SetDeadReckoningEnabled::Request> request,
                                   std::shared_ptr<okmr_msgs::srv::SetDeadReckoningEnabled::Response> response) {
        is_dead_reckoning_enabled = request->enable;
        response->success = true;
        response->message = request->enable ? "Dead reckoning enabled" : "Dead reckoning disabled";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void clear_pose_callback(const std::shared_ptr<okmr_msgs::srv::ClearPose::Request> request,
                           std::shared_ptr<okmr_msgs::srv::ClearPose::Response> response) {
        (void)request; // Unused parameter
        
        // Reset pose estimates to zero (clear the integration)
        translation_estimate.x = 0.0;
        translation_estimate.y = 0.0;
        translation_estimate.z = 0.0;
        
        response->success = true;
        response->message = "Pose cleared - integration reset to origin";
        RCLCPP_INFO(this->get_logger(), "Pose cleared - integration reset to origin");
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        
        for (const auto& param : parameters) {
            if (param.get_name() == "update_frequency") {
                update_frequency_ = param.as_double();
                if (update_frequency_ <= 0.0) {
                    result.successful = false;
                    result.reason = "Update frequency must be positive";
                    return result;
                }
                
                // Update timer period
                auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(1.0 / update_frequency_));
                
                timer_->cancel();
                timer_ = this->create_wall_timer(
                    timer_period, std::bind(&DeadReckoningNode::update, this));
            }
            else if (param.get_name() == "complementary_filter_alpha") {
                complementary_filter_alpha_ = param.as_double();
            }
            else if (param.get_name() == "dvl_velocity_alpha") {
                dvl_velocity_alpha_ = param.as_double();
            }
            else if (param.get_name() == "dvl_accel_alpha") {
                dvl_accel_alpha_ = param.as_double();
            }
            else if (param.get_name() == "angular_vel_filter_alpha") {
                angular_vel_filter_alpha_ = param.as_double();
            }
            else if (param.get_name() == "angular_accel_filter_alpha") {
                angular_accel_filter_alpha_ = param.as_double();
            }
        }
        
        return result;
    }

    void update() {
        if (!gotFirstTime) {
            return;
        }

        auto current_time = this->now();
        double dt = (current_time - last_time).seconds();
        
        if (dt <= 0.0) {
            return; // Skip if no time has passed
        }
        
        last_time = current_time;

        // Step 1: Extract angular velocities from IMU (preserving original coordinate transforms)
        double angular_velocity_pitch = 0;
        double angular_velocity_yaw  = 0;
        double angular_velocity_roll = 0;

        // yaw and roll are negative to convert from D455 coordinate plane to standard
        if(abs(current_imu_msg.angular_velocity.x)>0.01)
            angular_velocity_pitch = -current_imu_msg.angular_velocity.x;
        if(abs(current_imu_msg.angular_velocity.y)>0.01)
            angular_velocity_yaw  = -current_imu_msg.angular_velocity.y;
        if(abs(current_imu_msg.angular_velocity.z)>0.01)
            angular_velocity_roll = current_imu_msg.angular_velocity.z;

        // Update current twist angular velocity (always published)
        current_twist.angular.x = angular_velocity_roll;
        current_twist.angular.y = angular_velocity_pitch;
        current_twist.angular.z = angular_velocity_yaw;

        // Step 2: Convert coordinate frame from camera IMU to ROS2 format (preserving original)
        double ax = current_imu_msg.linear_acceleration.z;
        double ay = -current_imu_msg.linear_acceleration.x;
        double az = -current_imu_msg.linear_acceleration.y;

        if (is_dead_reckoning_enabled) {
            // Step 3: Dead reckoning enabled - perform attitude estimation and integration
            
            // Complementary filter for attitude estimation (preserving original logic)
            double alpha = complementary_filter_alpha_;
            
            double accel_pitch = atan2(-ax, sqrt(ay * ay + az * az));
            double accel_roll  = atan2(ay, sqrt(ax * ax + az * az));
            
            // Don't use accel data when nearly vertical in pitch or roll (preserving original)
            if (std::abs(accel_pitch) > 80.0 || std::abs(accel_roll) > 80.0) {
                alpha = 1.0;
            }

            // Handle upside down cases (preserving original logic)
            if (az < 0) {  
                if (accel_pitch > 0) {
                    accel_pitch = M_PI - accel_pitch;
                } else {
                    accel_pitch = -M_PI - accel_pitch;
                }
            }
            
            // Complementary filter for attitude (preserving original)
            rotation_estimate.y = alpha * (rotation_estimate.y + angular_velocity_pitch * dt) + (1 - alpha) * accel_pitch;
            rotation_estimate.x = alpha * (rotation_estimate.x + angular_velocity_roll * dt) + (1 - alpha) * accel_roll;
            rotation_estimate.z += angular_velocity_yaw * dt;

            // Integrate velocity for pose estimation (preserving original method)
            tf2::Quaternion q;
            q.setRPY(rotation_estimate.x, rotation_estimate.y, rotation_estimate.z);
            tf2::Matrix3x3 tf_R(q);
            tf2::Vector3 rotated_point = tf_R * tf2::Vector3(current_twist.linear.x*dt, current_twist.linear.y*dt, current_twist.linear.z*dt);
            translation_estimate.x += rotated_point.x();
            translation_estimate.y += rotated_point.y();
            translation_estimate.z += rotated_point.z();

            // Update pose message
            current_pose.pose.orientation = tf2::toMsg(q);
            current_pose.pose.position.x = translation_estimate.x;
            current_pose.pose.position.y = translation_estimate.y;
            current_pose.pose.position.z = translation_estimate.z;
        }

        // Step 4: Calculate linear acceleration (gravity compensation)
        tf2::Quaternion q;
        q.setRPY(rotation_estimate.x, rotation_estimate.y, rotation_estimate.z);
        tf2::Matrix3x3 tf_R(q);
        tf2::Vector3 gravity_world(0, 0, 9.81);
        tf2::Vector3 gravity_body = tf_R.transpose() * gravity_world;

        // Calculate IMU true linear acceleration (subtract gravity)
        double imu_accel_x = ax - gravity_body.x();
        double imu_accel_y = ay - gravity_body.y();
        double imu_accel_z = az - gravity_body.z();

        // Combine DVL acceleration with IMU acceleration using complementary filter
        current_accel.linear.x = dvl_accel_alpha_ * dvl_accel.x + (1.0 - dvl_accel_alpha_) * imu_accel_x;
        current_accel.linear.y = dvl_accel_alpha_ * dvl_accel.y + (1.0 - dvl_accel_alpha_) * imu_accel_y;
        current_accel.linear.z = dvl_accel_alpha_ * dvl_accel.z + (1.0 - dvl_accel_alpha_) * imu_accel_z;

        // Update linear velocity estimate by integrating gravity-compensated IMU acceleration
        // This provides high-frequency velocity updates between DVL measurements
        current_twist.linear.x += imu_accel_x * dt;
        current_twist.linear.y += imu_accel_y * dt;
        current_twist.linear.z += imu_accel_z * dt;

        // Calculate angular acceleration (derivative of smoothed angular velocity)
        smoothed_angular_vel.x = angular_vel_filter_alpha_ * smoothed_angular_vel.x + (1.0 - angular_vel_filter_alpha_) * angular_velocity_roll;
        smoothed_angular_vel.y = angular_vel_filter_alpha_ * smoothed_angular_vel.y + (1.0 - angular_vel_filter_alpha_) * angular_velocity_pitch;
        smoothed_angular_vel.z = angular_vel_filter_alpha_ * smoothed_angular_vel.z + (1.0 - angular_vel_filter_alpha_) * angular_velocity_yaw;

        current_accel.angular.x = angular_accel_filter_alpha_ * current_accel.angular.x + (1.0 - angular_accel_filter_alpha_) * (smoothed_angular_vel.x - prev_angular_vel.x) / dt;
        current_accel.angular.y = angular_accel_filter_alpha_ * current_accel.angular.y + (1.0 - angular_accel_filter_alpha_) * (smoothed_angular_vel.y - prev_angular_vel.y) / dt;
        current_accel.angular.z = angular_accel_filter_alpha_ * current_accel.angular.z + (1.0 - angular_accel_filter_alpha_) * (smoothed_angular_vel.z - prev_angular_vel.z) / dt;

        prev_angular_vel = smoothed_angular_vel;

        // Step 5: Update message headers and publish
        current_pose.header.stamp = current_time.to_msg();
        current_pose.header.frame_id = "map";
        current_twist.header.stamp = current_time.to_msg();
        current_twist.header.frame_id = "base_link";
        current_accel.header.stamp = current_time.to_msg();
        current_accel.header.frame_id = "base_link";

        // Publish all state estimates
        pose_publisher->publish(current_pose);
        twist_publisher->publish(current_twist);
        accel_publisher->publish(current_accel);
    }

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
	rclcpp::Subscription<okmr_msgs::msg::Dvl>::SharedPtr dvl_subscription;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;
	rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_publisher;
    rclcpp::Service<okmr_msgs::srv::GetPoseTwistAccel>::SharedPtr get_pose_twist_accel_service;
    rclcpp::Service<okmr_msgs::srv::SetDeadReckoningEnabled>::SharedPtr set_dead_reckoning_service;
    rclcpp::Service<okmr_msgs::srv::ClearPose>::SharedPtr clear_pose_service;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeadReckoningNode>());
  rclcpp::shutdown();


  return 0;
}
