#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "okmr_msgs/msg/sensor_reading.hpp"
#include "okmr_msgs/srv/get_pose.hpp"
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
        double x=0,y=0,z=0,roll=0,pitch=0,yaw=0,surge=0,sway=0,heave=0;
        bool gotFirstTime=false;
        std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
        geometry_msgs::msg::PoseStamped current_pose;

	    DeadReckoningNode() : Node("dead_reckoning_node") {
            rclcpp::QoS qos_profile(10);  // Create QoS profile with history depth 10
            qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);

		    imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("/camera/camera/imu", qos_profile, std::bind(&DeadReckoningNode::imu_callback, this, _1));

		    roll_publisher = this->create_publisher<okmr_msgs::msg::SensorReading>("PID/roll/actual", 10);
		    pitch_publisher = this->create_publisher<okmr_msgs::msg::SensorReading>("PID/pitch/actual", 10);
	    	yaw_publisher = this->create_publisher<okmr_msgs::msg::SensorReading>("PID/yaw/actual", 10);
	    	pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
	    	//sway_publisher = this->create_publisher<okmr_msgs::msg::SensorReading>("/PID/sway/actual", 10);
            surge_subscriber.subscribe(this, "PID/surge/actual");
            sway_subscriber.subscribe(this, "PID/sway/actual");
            heave_subscriber.subscribe(this, "PID/heave/actual");
            sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), surge_subscriber,sway_subscriber,heave_subscriber);
            sync_->registerCallback(std::bind(&DeadReckoningNode::linear_velocity_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
            
            get_pose_service = this->create_service<okmr_msgs::srv::GetPose>(
                "get_pose", 
                std::bind(&DeadReckoningNode::get_pose_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
	    }

	private:
	void imu_callback(const sensor_msgs::msg::Imu &msg) {
        if(!gotFirstTime){
            last_time = std::chrono::high_resolution_clock::now();
            gotFirstTime=true;
            return;
        }

		std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - last_time;
        double dt = duration.count();
        last_time = std::chrono::high_resolution_clock::now();
		
        double angular_velocity_pitch = 0;
        double angular_velocity_yaw  = 0;
        double angular_velocity_roll = 0;

        //yaw and roll are negative to convert from D455 coordinate plane to standard
        if(abs(msg.angular_velocity.x)>0.01)
            angular_velocity_pitch = -msg.angular_velocity.x;
        if(abs(msg.angular_velocity.y)>0.01)
            angular_velocity_yaw  = -msg.angular_velocity.y;
        if(abs(msg.angular_velocity.z)>0.01)
            angular_velocity_roll = msg.angular_velocity.z;
        

        double alpha = 0.995;  //determines the ratio of accelerometer to gyroscope data in the estimated value 
        //TODO: make this a parameter
        //pitch and roll use a basic complemntary filter
        
        //the following filter is very jank and may look weird
        //it converts the coordinate frame of the camera imu into ros2 format
        double ax = msg.linear_acceleration.z;
        double ay = -msg.linear_acceleration.x;
        double az = -msg.linear_acceleration.y;

        double accel_pitch = atan2(-ax, sqrt(ay * ay + az * az));
        double accel_roll  = atan2(ay, sqrt(ax * ax + az * az));
        
        if (std::abs(accel_pitch) > 80.0 || std::abs(accel_roll) > 80.0) {
            alpha = 1.0; // dont use accel data when nearly vertical in pitch or roll
        }
        else{
            alpha = 0.995;
        }

        //this is some kind of logic to try to fix weird flipping issues when the imu is upsidedown, although it doesnt work all that well
        if (az < 0) {  
            if (accel_pitch > 0) {
                accel_pitch = M_PI - accel_pitch;
            } else {
                accel_pitch = -M_PI - accel_pitch;
            }
        }
        
        // Fix roll flipping when upside down
        //
        // The following filter works decently well when upright, 
        // it compensates for drift from integrating the angular velocities
        // by setting a small part of the pitch estimate to be from the accelerometer
        // This works by taking the angle between the different accelerometer axes
        // allowing us to estimate the angle the imu is facing just from the accelerometer data
        
        pitch = alpha * (pitch + angular_velocity_pitch * dt) + (1 - alpha) * accel_pitch;
        roll  = alpha * (roll + angular_velocity_roll * dt) + (1 - alpha) * accel_roll;
		yaw    += angular_velocity_yaw * dt;

		auto roll_msg  = okmr_msgs::msg::SensorReading();
		auto pitch_msg = okmr_msgs::msg::SensorReading();
		auto yaw_msg   = okmr_msgs::msg::SensorReading();
		auto pose_msg  = geometry_msgs::msg::PoseStamped();

        roll_msg.data=roll*180.0/3.141592653589793238463;
        yaw_msg.data=yaw*180.0/3.141592653589793238463;
        pitch_msg.data=pitch*180.0/3.141592653589793238463;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);//creating quaternion from roll pitch and yaw
        tf2::Matrix3x3 tf_R(q); //rotational matrix created using quaternion
        tf2::Vector3 rotated_point = tf_R * tf2::Vector3(surge*dt, sway*dt, heave*dt);
        x += rotated_point.x();
        y += rotated_point.y();
        z += rotated_point.z();

        pose_msg.pose.orientation=tf2::toMsg(q);
        pose_msg.pose.position.x=x;
        pose_msg.pose.position.y=y;
        pose_msg.pose.position.z=z;

        roll_msg.header.stamp=msg.header.stamp;
        pitch_msg.header.stamp=msg.header.stamp;
        yaw_msg.header.stamp=msg.header.stamp;
        pose_msg.header.stamp=msg.header.stamp;
        pose_msg.header.frame_id="map";

        current_pose = pose_msg;

		pitch_publisher -> publish(pitch_msg);
		roll_publisher  -> publish(roll_msg);
		yaw_publisher   -> publish(yaw_msg);
        pose_publisher  -> publish(pose_msg);
	}

	void linear_velocity_callback(const okmr_msgs::msg::SensorReading::ConstSharedPtr surge_msg, const okmr_msgs::msg::SensorReading::ConstSharedPtr sway_msg, const okmr_msgs::msg::SensorReading::ConstSharedPtr heave_msg) {
        surge=surge_msg->data;
        sway=sway_msg->data;
        heave=heave_msg->data;
    }

    void get_pose_callback(const std::shared_ptr<okmr_msgs::srv::GetPose::Request> request,
                          std::shared_ptr<okmr_msgs::srv::GetPose::Response> response) {
        (void)request; // Unused parameter
        response->pose = current_pose;
        response->success = gotFirstTime; // Only return success if we've received at least one IMU measurement
    }

    using SensorMsg = okmr_msgs::msg::SensorReading;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<SensorMsg, SensorMsg, SensorMsg>;

    // Synchronizer
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
	rclcpp::Publisher<SensorMsg>::SharedPtr roll_publisher, pitch_publisher, yaw_publisher, sway_publisher;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Service<okmr_msgs::srv::GetPose>::SharedPtr get_pose_service;
    message_filters::Subscriber<SensorMsg> surge_subscriber, sway_subscriber, heave_subscriber;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeadReckoningNode>());
  rclcpp::shutdown();


  return 0;
}
